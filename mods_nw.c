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

#define pr_fmt(fmt) "SL-NWK: " fmt

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include "endo.h"
#include "greybus.h"
#include "svc_msg.h"
#include "muc_svc.h"
#include "muc_attach.h"
#include "mods_nw.h"

#define PAYLOAD_MAX_SIZE     (MUC_MSG_SIZE_MAX - sizeof(struct muc_msg))

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
#pragma pack(pop)

struct mods_nw_data {
	struct greybus_host_device *hd;
	struct notifier_block attach_nb;   /* attach/detach notifications */
	bool present;
};

static struct greybus_host_device *g_hd;

/* TODO: a real list with operations to select the device */
/* single entry table to start with */
static struct mods_host_device *g_routing;

/* TODO: use max_size to calc max payload */
struct mods_host_device *mods_create_hd(struct mods_host_driver *drv,
		struct device *dev)
{
	struct mods_host_device *mods_dev;

	mods_dev = (struct mods_host_device *)kzalloc(sizeof(*mods_dev),
			GFP_KERNEL);
	if (!mods_dev)
		return ERR_PTR(-ENOMEM);

	mods_dev->drv = drv;
	mods_dev->dev = dev;

	g_routing = mods_dev;

	return mods_dev;
}
EXPORT_SYMBOL_GPL(mods_create_hd);

void mods_remove_hd(struct mods_host_device *dev)
{
	g_routing = NULL;
	kfree(dev);
}
EXPORT_SYMBOL_GPL(mods_remove_hd);

void mods_data_rcvd(struct mods_host_device *hd, uint8_t *data)
{
	struct muc_msg *msg = (struct muc_msg *)data;

	greybus_data_rcvd(g_hd, msg->hdr.dest_cport, msg->gb_msg, msg->hdr.size);
}
EXPORT_SYMBOL_GPL(mods_data_rcvd);

static int mods_nw_attach(struct notifier_block *nb,
		      unsigned long now_present, void *not_used)
{
	struct mods_nw_data *dd = container_of(nb, struct mods_nw_data,
			attach_nb);
	struct greybus_host_device *hd = dd->hd;

	if (now_present != dd->present) {
		pr_debug("MuC attach state = %lu\n", now_present);

		dd->present = now_present;
		if (now_present)
			muc_svc_attach(hd);
		else
			muc_svc_detach(hd);
	}
	return NOTIFY_OK;
}

static void *mods_msg_send(struct greybus_host_device *hd,
		u16 hd_cport_id,
		struct gb_message *message,
		gfp_t gfp_mask)
{
	size_t buffer_size;
	struct gb_connection *connection;
	struct muc_msg *msg;

	if (message->payload_size > PAYLOAD_MAX_SIZE)
		return ERR_PTR(-E2BIG);

	connection = gb_connection_hd_find(hd, hd_cport_id);
	if (!connection) {
		pr_err("Invalid cport supplied to send\n");
		return ERR_PTR(-EINVAL);
	}

	buffer_size = sizeof(*message->header) + message->payload_size;

	msg = (struct muc_msg *)kzalloc(buffer_size +
			sizeof(struct muc_msg_hdr), GFP_KERNEL);
	if (!msg) {
		return ERR_PTR(-ENOMEM);
	}

	msg->hdr.dest_cport = connection->intf_cport_id;
	msg->hdr.src_cport = connection->hd_cport_id;
	msg->hdr.size = buffer_size + sizeof(struct muc_msg_hdr);
	memcpy(&msg->gb_msg[0], message->buffer, buffer_size);

	pr_info("AP (CPort %d) -> Module (CPort %d)\n",
			connection->hd_cport_id, connection->intf_cport_id);

	/* hand off to the dl layer */
	if (g_routing && g_routing->drv && g_routing->drv->message_send)
		g_routing->drv->message_send(
				(struct mods_host_device *)g_routing,
				(uint8_t *)msg,
				msg->hdr.size);
	kfree(msg);

	return NULL;
}

static void mods_msg_cancel(void *cookie)
{
	/* nothing currently */
}

static int mods_submit_svc(struct svc_msg *svc_msg,
			      struct greybus_host_device *hd)
{
	return 0;
}

static struct greybus_host_driver mods_nw_host_driver = {
	.hd_priv_size		= sizeof(struct mods_nw_data),
	.message_send		= mods_msg_send,
	.message_cancel		= mods_msg_cancel,
	.submit_svc		    = mods_submit_svc,
};

static int mods_nw_probe(struct platform_device *pdev)
{
	struct mods_nw_data *dd;
	u16 endo_id = 0x4755;
	u8 ap_intf_id = 0x01;
	int retval;

	/* setup host device */
	g_hd = greybus_create_hd(&mods_nw_host_driver, &pdev->dev,
			PAYLOAD_MAX_SIZE);
	if (IS_ERR(g_hd)) {
		dev_err(&pdev->dev, "Unable to create greybus host driver.\n");
		return PTR_ERR(g_hd);
	}

	/* setup endo */
	retval = greybus_endo_setup(g_hd, endo_id, ap_intf_id);
	if (retval) {
		greybus_remove_hd(g_hd);
		return retval;
	}

	/* register attach */
	dd = (struct mods_nw_data *)&g_hd->hd_priv;
	dd->hd = g_hd;
	platform_set_drvdata(pdev, dd);

	dd->attach_nb.notifier_call = mods_nw_attach;
	register_muc_attach_notifier(&dd->attach_nb);

	return 0;
}

static int mods_nw_remove(struct platform_device *pdev)
{
	struct mods_nw_data *dd = (struct mods_nw_data *)
			platform_get_drvdata(pdev);

	unregister_muc_attach_notifier(&dd->attach_nb);
	greybus_remove_hd(dd->hd);

	return 0;
}

/* TODO init device */
static struct platform_driver mods_nw_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mods_nw",
	},
	.probe = mods_nw_probe,
	.remove  = mods_nw_remove,
};


static struct platform_device mods_nw_device = {
	.name           = "mods_nw",
	.id             = 0,
	.num_resources  = 0,
};


int __init mods_nw_init(void)
{
	int rv;

	rv = platform_driver_register(&mods_nw_driver);
	if (rv < 0) {
		pr_err("mods_nw failed to register driver\n");
		return rv;
	}
	rv = platform_device_register(&mods_nw_device);
	if (rv < 0) {
		pr_err("mods_nw failed to register device\n");
		platform_driver_unregister(&mods_nw_driver);
		return rv;
	}

	return 0;
}

void __exit mods_nw_exit(void)
{
}
