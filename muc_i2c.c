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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slice_attach.h>

#include "endo.h"
#include "greybus.h"
#include "svc_msg.h"
#include "muc_svc.h"

#define MUC_MSG_SIZE_MAX        (1024)
#define MUC_PAYLOAD_SIZE_MAX    (MUC_MSG_SIZE_MAX - sizeof(struct muc_msg))

/* Size of payload of individual I2C packet (in bytes) */
#define I2C_BUF_SIZE            (32)

#define HDR_BIT_VALID           (0x01 << 7)
#define HDR_BIT_RSVD            (0x03 << 5)
#define HDR_BIT_PKTS            (0x1F << 0)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct muc_i2c_data {
	struct i2c_client *client;
	struct greybus_host_device *hd;
	bool present;
	struct notifier_block attach_nb;   /* attach/detach notifications */
};

#pragma pack(push, 1)
struct muc_msg_hdr {
    __le16  size;
    __u8    dest_cport;
    __u8    src_cport;
};

struct muc_msg {
    struct muc_msg_hdr hdr;
    __u8    gb_msg[0];
};

struct muc_i2c_hdr {
    uint8_t checksum;
    uint8_t hdr_bits;
};

struct muc_i2c_msg {
    struct muc_i2c_hdr hdr;
    uint8_t data[I2C_BUF_SIZE];
};
#pragma pack(pop)


static inline struct muc_i2c_data *hd_to_dd(struct greybus_host_device *hd)
{
	return (struct muc_i2c_data *)&hd->hd_priv;
}

/* read a single i2c msg */
static int muc_i2c_msg_read(struct muc_i2c_data *dd, struct muc_i2c_msg *msg)
{
	struct i2c_msg i2c_msg;
	int rv;

	i2c_msg.addr = dd->client->addr;
	i2c_msg.flags = dd->client->flags | I2C_M_RD;
	i2c_msg.len = sizeof(*msg);
	i2c_msg.buf = (uint8_t *)msg;

	rv = i2c_transfer(dd->client->adapter, &i2c_msg, 1);
	return rv;
}

static inline void muc_msg_free(struct muc_msg *msg)
{
	kfree(msg);
}

static int muc_msg_read(struct muc_i2c_data *dd, struct muc_msg **msg)
{
	struct muc_i2c_msg i2c_msg;
	uint8_t pkt_cnt; /* total number of pkts */
	uint8_t i;
	uint8_t *buf;
	int ret;

	/* read the first packet and get the number of packts */
	ret = muc_i2c_msg_read(dd, &i2c_msg);
		if (ret < 0) {
			pr_err("i2c tranfer failure\n");
			return ret;
		}

	pkt_cnt = i2c_msg.hdr.hdr_bits && HDR_BIT_PKTS;

	if (pkt_cnt < 1 || pkt_cnt > 32) {
		pr_err("ERR: no pkts\n");
		return -EIO;
	}

	buf = (uint8_t *)kzalloc(pkt_cnt * I2C_BUF_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	memcpy(&buf[0], i2c_msg.data, I2C_BUF_SIZE);

	for (i = 1; i < pkt_cnt; i++) {
		muc_i2c_msg_read(dd, &i2c_msg);
		memcpy(&buf[i * I2C_BUF_SIZE], i2c_msg.data, I2C_BUF_SIZE);
	}
	/* TODO Calculate the checksum  */
	*msg = (struct muc_msg *)buf;
	return 0;
}

static irqreturn_t muc_i2c_isr(int irq, void *data)
{
	struct greybus_host_device *hd = data;
	struct muc_i2c_data *dd = hd_to_dd(hd);
	struct muc_msg *msg;

	/* Any interrupt while the MuC is not attached would be spurious */
	if (!dd->present)
		return IRQ_HANDLED;

	muc_msg_read(dd, &msg);
	greybus_data_rcvd(hd, msg->hdr.dest_cport, msg->gb_msg, msg->hdr.size);
	muc_msg_free(msg);
	return IRQ_HANDLED;
}

static int muc_i2c_attach(struct notifier_block *nb,
		      unsigned long now_present, void *not_used)
{
	struct muc_i2c_data *dd = container_of(nb, struct muc_i2c_data, attach_nb);
	struct greybus_host_device *hd = dd->hd;
	struct i2c_client *client = dd->client;

	if (now_present != dd->present) {
		printk("%s: MuC attach state = %lu\n", __func__, now_present);

		dd->present = now_present;

		if (now_present) {
			if (devm_request_threaded_irq(&client->dev, client->irq,
						      NULL, muc_i2c_isr,
						      IRQF_TRIGGER_LOW |
						      IRQF_ONESHOT,
						      "muc_i2c", hd))
				printk(KERN_ERR "%s: Unable to request irq.\n", __func__);
            muc_svc_attach(hd);
		} else {
			devm_free_irq(&client->dev, client->irq, hd);
            muc_svc_detach(hd);
		}
	}
	return NOTIFY_OK;
}

static int muc_i2c_write(struct muc_i2c_data *dd, uint8_t *buf, int size)
{
	struct i2c_msg msg;

	msg.addr = dd->client->addr;
	msg.flags = dd->client->flags;
	msg.len = size;
	msg.buf = buf;

	return i2c_transfer(dd->client->adapter, &msg, 1);
}

static int muc_i2c_msg_send(struct muc_i2c_data *dd, struct muc_msg *msg)
{
	struct muc_i2c_msg i2c_msg;
	uint8_t pkts_total;
	uint8_t pkts;
	size_t remaining;
	size_t len = msg->hdr.size;
	uint8_t *buf = (uint8_t *)msg;
	int i;

	if (len > MUC_PAYLOAD_SIZE_MAX)
		return -E2BIG;

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

/*
 * Returns zero if the message was successfully queued, or a negative errno
 * otherwise.
 */
static int muc_msg_send(struct greybus_host_device *hd,
        u16 hd_cport_id,
		struct gb_message *message,
        gfp_t gfp_mask)
{
	struct muc_i2c_data *dd = hd_to_dd(hd);
	size_t buffer_size;
	struct gb_connection *connection;
	struct muc_msg *msg;

	connection = gb_connection_hd_find(hd, hd_cport_id);
	if (!connection) {
		pr_err("Invalid cport supplied to send\n");
		return -EINVAL;
	}

	buffer_size = sizeof(*message->header) + message->payload_size;

	msg = (struct muc_msg *)kzalloc(buffer_size + sizeof(struct muc_msg_hdr), GFP_KERNEL);
	if (!msg) {
		printk("%s: no memory\n", __func__);
		return -ENOMEM;
	}
	msg->hdr.dest_cport = connection->intf_cport_id;
	msg->hdr.src_cport = connection->hd_cport_id;
	msg->hdr.size = buffer_size + sizeof(struct muc_msg_hdr);
	memcpy(&msg->gb_msg[0], message->buffer, buffer_size);

	printk("%s: AP (CPort %d) -> Module (CPort %d)\n",
	       __func__, connection->hd_cport_id, connection->intf_cport_id);

	muc_i2c_msg_send(dd, msg);
	muc_msg_free(msg);

	return 0;
}

/*
 * The cookie value supplied is the value that message_send()
 * returned to its caller.  It identifies the buffer that should be
 * canceled.  This function must also handle (which is to say,
 * ignore) a null cookie value.
 */
static void muc_i2c_message_cancel(struct gb_message *message)
{
	printk("%s: enter\n", __func__);
}

static int muc_i2c_submit_svc(struct svc_msg *svc_msg,
			      struct greybus_host_device *hd)
{
	/* Currently don't have an SVC! */
	return 0;
}

static struct greybus_host_driver muc_i2c_host_driver = {
	.hd_priv_size		= sizeof(struct muc_i2c_data),
	.message_send		= muc_msg_send,
	.message_cancel		= muc_i2c_message_cancel,
	.submit_svc		    = muc_i2c_submit_svc,
};

static int muc_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct muc_i2c_data *dd;
	struct greybus_host_device *hd;
	u16 endo_id = 0x4755;
	u8 ap_intf_id = 0x01;
	int retval;

	pr_info("%s: enter\n", __func__);

	if (client->irq < 0) {
		pr_err("%s: IRQ not defined\n", __func__);
		return -EINVAL;
	}

	hd = greybus_create_hd(&muc_i2c_host_driver, &client->dev,
			       MUC_PAYLOAD_SIZE_MAX);
	if (IS_ERR(hd)) {
		printk(KERN_ERR "%s: Unable to create greybus host driver.\n",
		       __func__);
		return PTR_ERR(hd);
	}

	retval = greybus_endo_setup(hd, endo_id, ap_intf_id);
	if (retval)
		return retval;

	dd = hd_to_dd(hd);
	dd->hd = hd;
	dd->client = client;
	dd->attach_nb.notifier_call = muc_i2c_attach;

	i2c_set_clientdata(client, dd);

	register_slice_attach_notifier(&dd->attach_nb);

	return 0;
}

static int muc_i2c_remove(struct i2c_client *client)
{
	struct muc_i2c_data *dd = i2c_get_clientdata(client);

	pr_info("%s: enter\n", __func__);

	unregister_slice_attach_notifier(&dd->attach_nb);
	greybus_remove_hd(dd->hd);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_muc_i2c_match[] = {
	{ .compatible = "moto,slice_i2c", },
	{},
};
#endif

static const struct i2c_device_id muc_i2c_id[] = {
	{ "slice_i2c", 0 },
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

module_i2c_driver(muc_i2c_driver);

MODULE_AUTHOR("Motorola Mobility, LLC");
MODULE_DESCRIPTION("Mods uC (MuC) I2C bus driver");
MODULE_LICENSE("GPL");
