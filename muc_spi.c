/*
 * Copyright (C) 2015 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>

#include "crc.h"
#include "muc_attach.h"
#include "mods_nw.h"
#include "muc_svc.h"

/* Size of payload of individual SPI packet (in bytes) */
#define MUC_SPI_PAYLOAD_SZ_MAX  (32)

#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = valid packet, 0 = dummy packet */
#define HDR_BIT_RSVD   (0x03 << 5)  /* Reserved */
#define HDR_BIT_PKTS   (0x1F << 0)  /* How many additional packets to expect */

#define RDY_TIMEOUT_JIFFIES     (1 * HZ) /* 1 sec */

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct muc_spi_data {
	struct spi_device *spi;
	struct mods_dl_device *dld;
	bool present;
	struct notifier_block attach_nb;   /* attach/detach notifications */
	struct mutex mutex;
	wait_queue_head_t rdy_wq;

	int gpio_wake_n;
	int gpio_rdy_n;

	__u8 *rx_buf;

	/*
	 * Buffer to hold incoming payload (which could be spread across
	 * multiple packets)
	 */
	__u8 rcvd_payload[MUC_MSG_SIZE_MAX];
	int rcvd_payload_idx;
};

#pragma pack(push, 1)
struct muc_spi_msg
{
	__u8    hdr_bits;
	__u8    data[MUC_SPI_PAYLOAD_SZ_MAX];
	__le16  crc16;
};
#pragma pack(pop)


static void parse_rx_dl(struct muc_spi_data *dd, uint8_t *rx_buf);

static inline struct muc_spi_data *dld_to_dd(struct mods_dl_device *dld)
{
	return (struct muc_spi_data *)dld->dl_priv;
}

static int muc_spi_transfer_locked(struct muc_spi_data *dd,
				   uint8_t *tx_buf, bool keep_wake)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = tx_buf,
			.rx_buf = dd->rx_buf,
			.len = sizeof(struct muc_spi_msg),
		},
	};
	int ret;

	/* Check if WAKE is not asserted */
	if (gpio_get_value(dd->gpio_wake_n)) {
		/* Assert WAKE */
		gpio_set_value(dd->gpio_wake_n, 0);

		/* Wait for ADC enable */
		udelay(300);
	}

	/* Wait for RDY to be asserted */
	ret = wait_event_timeout(dd->rdy_wq, !gpio_get_value(dd->gpio_rdy_n),
				 RDY_TIMEOUT_JIFFIES);

	if (!keep_wake) {
		/* Deassert WAKE */
		gpio_set_value(dd->gpio_wake_n, 1);
	}

	/*
	 * Check that RDY successfully was asserted after wake deassert to ensure
	 * wake line is deasserted if requested.
	 */
	if (ret <= 0) {
		dev_err(&dd->spi->dev, "Timeout waiting for rdy to assert\n");
		return ret;
	}

	ret = spi_sync_transfer(dd->spi, t, 1);

	if (!ret) {
		parse_rx_dl(dd, dd->rx_buf);
	}

	return ret;
}

static int muc_spi_transfer(struct muc_spi_data *dd, uint8_t *tx_buf,
			    bool keep_wake)
{
	int ret;

	mutex_lock(&dd->mutex);
	ret = muc_spi_transfer_locked(dd, tx_buf, keep_wake);
	mutex_unlock(&dd->mutex);

	return ret;
}

static void parse_rx_dl(struct muc_spi_data *dd, uint8_t *buf)
{
	struct muc_spi_msg *m = (struct muc_spi_msg *)buf;
	struct spi_device *spi = dd->spi;
	uint16_t calcrc = 0;

	if (!(m->hdr_bits & HDR_BIT_VALID)) {
		/* Received a dummy packet - nothing to do! */
		return;
	}

	if (dd->rcvd_payload_idx >= MUC_MSG_SIZE_MAX) {
		dev_err(&spi->dev, "%s: Too many packets received!\n", __func__);
		return;
	}

	calcrc = gen_crc16((uint8_t *)m, sizeof(struct muc_spi_msg) - 2);
	if (m->crc16 != calcrc) {
		dev_err(&spi->dev, "%s: CRC mismatch, received: 0x%x,"
			 "calculated: 0x%x\n", __func__, m->crc16, calcrc);
		return;
	}

	memcpy(&dd->rcvd_payload[dd->rcvd_payload_idx], m->data,
	       MUC_SPI_PAYLOAD_SZ_MAX);
	dd->rcvd_payload_idx += MUC_SPI_PAYLOAD_SZ_MAX;

	if (m->hdr_bits & HDR_BIT_PKTS) {
		/* Need additional packets */
		muc_spi_transfer_locked(dd, NULL, ((m->hdr_bits & HDR_BIT_PKTS) > 1));
		return;
	}

	mods_nw_switch(dd->dld, dd->rcvd_payload);
	memset(dd->rcvd_payload, 0, MUC_MSG_SIZE_MAX);
	dd->rcvd_payload_idx = 0;
}

static irqreturn_t muc_spi_isr(int irq, void *data)
{
	struct muc_spi_data *dd = data;

	/* Any interrupt while the MuC is not attached would be spurious */
	if (!dd->present)
		return IRQ_HANDLED;

	muc_spi_transfer(dd, NULL, false);
	return IRQ_HANDLED;
}

static irqreturn_t muc_spi_rdy_isr(int irq, void *data)
{
	struct muc_spi_data *dd = data;

	/* Wake up SPI transfer */
	wake_up(&dd->rdy_wq);

	return IRQ_HANDLED;
}

static int muc_attach(struct notifier_block *nb,
		      unsigned long now_present, void *not_used)
{
	struct muc_spi_data *dd = container_of(nb, struct muc_spi_data, attach_nb);
	struct spi_device *spi = dd->spi;
	int err;

	if (now_present != dd->present) {
		dev_info(&spi->dev, "%s: state = %lu\n", __func__, now_present);

		dd->present = now_present;

		if (now_present) {
			err = devm_request_threaded_irq(&spi->dev, spi->irq,
							NULL, muc_spi_isr,
							IRQF_TRIGGER_LOW |
							IRQF_ONESHOT,
							"muc_spi", dd);
			if (err) {
				dev_err(&spi->dev, "Unable to request irq.\n");
				goto set_missing;
			}

			err = devm_request_irq(&spi->dev,
					       gpio_to_irq(dd->gpio_rdy_n),
					       muc_spi_rdy_isr,
					       IRQF_TRIGGER_RISING |
					       IRQF_TRIGGER_FALLING,
					       "muc_spi_rdy", dd);
			if (err) {
				dev_err(&spi->dev, "Unable to request rdy.\n");
				goto free_irq;
			}

			err = mods_dl_dev_attached(dd->dld);
			if (err) {
				dev_err(&spi->dev, "Error attaching to SVC\n");
				goto free_rdy;
			}
		} else {
			devm_free_irq(&spi->dev, gpio_to_irq(dd->gpio_rdy_n), dd);
			devm_free_irq(&spi->dev, spi->irq, dd);
			mods_dl_dev_detached(dd->dld);
		}
	}
	return NOTIFY_OK;

free_rdy:
	devm_free_irq(&spi->dev, gpio_to_irq(dd->gpio_rdy_n), dd);
free_irq:
	devm_free_irq(&spi->dev, spi->irq, dd);
set_missing:
	dd->present = 0;

	return NOTIFY_OK;
}

/* send message from switch to muc */
static int muc_spi_message_send(struct mods_dl_device *dld,
				   uint8_t *buf, size_t len)
{
	struct muc_spi_msg *m;
	struct muc_spi_data *dd = dld_to_dd(dld);
	int remaining = len;
	uint8_t *dbuf = (uint8_t *)buf;
	int pl_size;
	int packets;

	if (!dd->present)
		return -ENODEV;

	/* Calculate how many packets are required to send whole payload */
	packets = (remaining + MUC_SPI_PAYLOAD_SZ_MAX - 1) / MUC_SPI_PAYLOAD_SZ_MAX;

	while ((remaining > 0) && (packets > 0)) {
		m = kzalloc(sizeof(struct muc_spi_msg), GFP_KERNEL);
		if (!m)
			return -ENOMEM;

		/* Determine the payload size of this packet */
		pl_size = MIN(remaining, MUC_SPI_PAYLOAD_SZ_MAX);

		/* Populate the SPI message */
		m->hdr_bits |= HDR_BIT_VALID;
		m->hdr_bits |= (--packets & HDR_BIT_PKTS);
		memcpy(m->data, dbuf, pl_size);
		m->crc16 = gen_crc16((uint8_t *)m, sizeof(struct muc_spi_msg) - 2);

		muc_spi_transfer(dd, (uint8_t *)m, (packets > 0));

		remaining -= pl_size;
		dbuf += pl_size;
		kfree(m);
	}

	return 0;
}

/*
 * The cookie value supplied is the value that message_send()
 * returned to its caller.  It identifies the buffer that should be
 * canceled.  This function must also handle (which is to say,
 * ignore) a null cookie value.
 */
static void muc_spi_message_cancel(void *cookie)
{
	/* Should never happen */
}

static struct mods_dl_driver muc_spi_dl_driver = {
	.dl_priv_size		= sizeof(struct muc_spi_data),
	.message_send		= muc_spi_message_send,
	.message_cancel		= muc_spi_message_cancel,
};

static int muc_spi_gpio_init(struct muc_spi_data *dd)
{
	struct device_node *np = dd->spi->dev.of_node;
	int ret;

	dd->gpio_wake_n = of_get_gpio(np, 0);
	dd->gpio_rdy_n = of_get_gpio(np, 1);

	ret = gpio_request_one(dd->gpio_wake_n, GPIOF_OUT_INIT_HIGH,
			       "muc_wake_n");
	if (ret)
		return ret;
	gpio_export(dd->gpio_wake_n, false);

	ret = gpio_request_one(dd->gpio_rdy_n, GPIOF_IN, "muc_rdy_n");
	if (ret)
		return ret;
	gpio_export(dd->gpio_rdy_n, false);

	return 0;
}

static int muc_spi_probe(struct spi_device *spi)
{
	struct muc_spi_data *dd;
	u8 intf_id;

	dev_info(&spi->dev, "%s: enter\n", __func__);

	if (spi->irq < 0) {
		dev_err(&spi->dev, "%s: IRQ not defined\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_u8(spi->dev.of_node, "mmi,intf-id", &intf_id)) {
		dev_err(&spi->dev, "Couldn't get mmi,intf-id\n");
		return -EINVAL;
	}

	dd = devm_kzalloc(&spi->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	dd->rx_buf = devm_kzalloc(&spi->dev, sizeof(struct muc_spi_msg),
					GFP_KERNEL);
	if (!dd->rx_buf)
		return -ENOMEM;

	dd->dld = mods_create_dl_device(&muc_spi_dl_driver, &spi->dev, intf_id);
	if (IS_ERR(dd->dld)) {
		dev_err(&spi->dev, "%s: Unable to create greybus host driver.\n",
		        __func__);
		return PTR_ERR(dd->dld);
	}

	dd->dld->dl_priv = (void *)dd;
	dd->spi = spi;
	dd->attach_nb.notifier_call = muc_attach;
	muc_spi_gpio_init(dd);
	mutex_init(&dd->mutex);
	init_waitqueue_head(&dd->rdy_wq);

	spi_set_drvdata(spi, dd);

	register_muc_attach_notifier(&dd->attach_nb);

	return 0;
}

static int muc_spi_remove(struct spi_device *spi)
{
	struct muc_spi_data *dd = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s: enter\n", __func__);

	if (dd->present)
		mods_dl_dev_detached(dd->dld);

	gpio_free(dd->gpio_wake_n);
	gpio_free(dd->gpio_rdy_n);

	unregister_muc_attach_notifier(&dd->attach_nb);
	mods_remove_dl_device(dd->dld);
	spi_set_drvdata(spi, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_muc_spi_match[] = {
	{ .compatible = "moto,muc_spi", },
	{},
};
#endif

static const struct spi_device_id muc_spi_id[] = {
	{ "muc_spi", 0 },
	{ }
};

static struct spi_driver muc_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "muc_spi",
		.of_match_table = of_match_ptr(of_muc_spi_match),
	},
	.id_table = muc_spi_id,
	.probe = muc_spi_probe,
	.remove  = muc_spi_remove,
};

int __init muc_spi_init(void)
{
	int err;

	err = spi_register_driver(&muc_spi_driver);
	if (err != 0)
		pr_err("muc_spi initialization failed\n");

	return err;
}

void __exit muc_spi_exit(void)
{
	spi_unregister_driver(&muc_spi_driver);
}
