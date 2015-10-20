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

#define pr_fmt(fmt) "SL-I2C: " fmt

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "endo.h"
#include "greybus.h"
#include "svc_msg.h"

#include "muc_attach.h"
#include "muc_svc.h"
#include "mods_nw.h"

/* Size of payload of individual I2C packet (in bytes) */
#define I2C_BUF_SIZE            (32)
#define I2C_MAX_PAYLOAD         (I2C_BUF_SIZE * 32)

#define HDR_BIT_VALID           (0x01 << 7)
#define HDR_BIT_RSVD            (0x03 << 5)
#define HDR_BIT_PKTS            (0x1F << 0)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct muc_i2c_data {
	struct i2c_client *client;
	struct mods_dl_device *dld;
	bool present;
	struct notifier_block attach_nb;   /* attach/detach notifications */
};

#pragma pack(push, 1)
struct muc_i2c_hdr {
	uint8_t checksum;
	uint8_t hdr_bits;
};

struct muc_i2c_msg {
	struct muc_i2c_hdr hdr;
	uint8_t data[I2C_BUF_SIZE];
};
#pragma pack(pop)

/* read a single i2c msg */
static int muc_i2c_msg_read(struct muc_i2c_data *dd, struct muc_i2c_msg *msg)
{
	struct i2c_msg i2c_msg;

	i2c_msg.addr = dd->client->addr;
	i2c_msg.flags = dd->client->flags | I2C_M_RD;
	i2c_msg.len = sizeof(*msg);
	i2c_msg.buf = (uint8_t *)msg;
	return i2c_transfer(dd->client->adapter, &i2c_msg, 1);
}

static int muc_i2c_message_read(struct muc_i2c_data *dd, uint8_t **msg)
{
	struct muc_i2c_msg i2c_msg;
	uint8_t pkt_cnt; /* total number of pkts */
	uint8_t i;
	uint8_t *buf;
	int ret;

	/* read the first packet and get the number of packts */
	ret = muc_i2c_msg_read(dd, &i2c_msg);
		if (ret < 0) {
			pr_err("i2c transfer failure\n");
			return ret;
		}

	pkt_cnt = i2c_msg.hdr.hdr_bits & HDR_BIT_PKTS;

	if (pkt_cnt < 1 || pkt_cnt > 32) {
		pr_err("ERR: no pkts\n");
		return -EIO;
	}

	buf = (uint8_t *)kzalloc(pkt_cnt * I2C_BUF_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	memcpy(&buf[0], i2c_msg.data, I2C_BUF_SIZE);

	for (i = 1; i < pkt_cnt; i++) {
	ret = muc_i2c_msg_read(dd, &i2c_msg);
		memcpy(&buf[i * I2C_BUF_SIZE], i2c_msg.data, I2C_BUF_SIZE);
	}

	/* TODO Calculate the checksum  */
	*msg = buf;
	return 0;
}

static irqreturn_t muc_i2c_isr(int irq, void *data)
{
	struct muc_i2c_data *dd = data;
	uint8_t *msg;

	/* Any interrupt while the MuC is not attached would be spurious */
	if (!dd->present)
		return IRQ_HANDLED;

	if (!muc_i2c_message_read(dd, &msg)) {
		mods_data_rcvd(dd->dld, msg);
		kfree(msg);
	}
	return IRQ_HANDLED;
}

static int muc_i2c_attach(struct notifier_block *nb,
		unsigned long now_present, void *not_used)
{
	struct muc_i2c_data *dd = container_of(nb, struct muc_i2c_data, attach_nb);
	struct i2c_client *client = dd->client;

	if (now_present != dd->present) {
		printk("%s: MuC attach state = %lu\n", __func__, now_present);

		dd->present = now_present;

		if (now_present) {
			if (devm_request_threaded_irq(&client->dev, client->irq,
						      NULL, muc_i2c_isr,
						      IRQF_TRIGGER_LOW |
						      IRQF_ONESHOT,
						      "muc_i2c", dd))
				printk(KERN_ERR "%s: Unable to request irq.\n", __func__);
		} else {
			devm_free_irq(&client->dev, client->irq, dd);
		}
	}
	return NOTIFY_OK;
}

static int muc_i2c_write(struct muc_i2c_data *dd, uint8_t *buf, int size)
{
	int err;
	struct i2c_msg msg;

	msg.addr = dd->client->addr;
	msg.flags = dd->client->flags;
	msg.len = size;
	msg.buf = buf;

	err = i2c_transfer(dd->client->adapter, &msg, 1);
	return err;
}

static int
muc_i2c_message_send(struct mods_dl_device *dld, uint8_t *msg, size_t len)
{
	struct muc_i2c_msg i2c_msg;
	uint8_t pkts_total;
	uint8_t pkts;
	size_t remaining;
	uint8_t *buf = (uint8_t *)msg;
	int i;
	struct i2c_client *client = container_of(dld->dev,
					struct i2c_client, dev);
	struct muc_i2c_data *dd = i2c_get_clientdata(client);

	memset(&i2c_msg, 0, sizeof(i2c_msg));

	pkts_total = (len / I2C_BUF_SIZE) + ((len % I2C_BUF_SIZE) > 0); /* round up better way?*/

	for (pkts = pkts_total, i = 0; pkts > 0; pkts--, i++) {
		remaining = len - (i * I2C_BUF_SIZE);
		buf += i * I2C_BUF_SIZE;

		i2c_msg.hdr.hdr_bits |= HDR_BIT_VALID;
		i2c_msg.hdr.hdr_bits |= pkts;

		memcpy(&i2c_msg.data[0], buf, MIN(remaining, I2C_BUF_SIZE));
		i2c_msg.hdr.checksum = 0xff; /* TODO */

		muc_i2c_write(dd, (uint8_t *)&i2c_msg, sizeof(i2c_msg));
	}

	return 0;
}

static struct mods_dl_driver muc_i2c_dl_driver = {
	.message_send		= muc_i2c_message_send,
};

static int muc_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct muc_i2c_data *dd;

	if (client->irq < 0) {
		pr_err("%s: IRQ not defined\n", __func__);
		return -EINVAL;
	}

	dd = devm_kzalloc(&client->dev, sizeof(*dd), GFP_KERNEL);
	dd->client = client;
	dd->attach_nb.notifier_call = muc_i2c_attach;

	dd->dld = mods_create_dl_device(&muc_i2c_dl_driver, &client->dev);
	if (IS_ERR(dd->dld)) {
		printk(KERN_ERR "%s: Unable to create greybus host driver.\n",
		       __func__);
		return PTR_ERR(dd->dld);
	}
	dd->dld->dl_priv = dd;
	i2c_set_clientdata(client, dd);

	register_muc_attach_notifier(&dd->attach_nb);

	return 0;
}

static int muc_i2c_remove(struct i2c_client *client)
{
	struct muc_i2c_data *dd = i2c_get_clientdata(client);

	unregister_muc_attach_notifier(&dd->attach_nb);
	mods_remove_dl_device(dd->dld);
	devm_kfree(&client->dev, dd);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_muc_i2c_match[] = {
	{ .compatible = "moto,muc_i2c", },
	{},
};
#endif

static const struct i2c_device_id muc_i2c_id[] = {
	{ "muc_i2c", 0 },
	{ }
};

static struct i2c_driver muc_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "muc_i2c",
		.of_match_table = of_match_ptr(of_muc_i2c_match),
	},
	.id_table = muc_i2c_id,
	.probe = muc_i2c_probe,
	.remove  = muc_i2c_remove,
};

int __init muc_i2c_init(void)
{
	int err;

	err = i2c_register_driver(THIS_MODULE, &muc_i2c_driver);
	if (err != 0)
		pr_err("muc_i2c initialization failed\n");

	return err;
}

void __exit muc_i2c_exit(void)
{
	i2c_del_driver(&muc_i2c_driver);
}
