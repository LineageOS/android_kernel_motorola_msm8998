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

#include "greybus.h"
#include "muc_svc.h"
#include "muc_attach.h"
#include "mods_nw.h"

#define PAYLOAD_MAX_SIZE     (MUC_MSG_SIZE_MAX - sizeof(struct muc_msg))

struct mods_nw_data {
	struct greybus_host_device *hd;
	struct notifier_block attach_nb;   /* attach/detach notifications */
	bool present;
	struct list_head nw_dev;
	atomic_t device_id;
};

static struct greybus_host_device *g_hd;

/* TODO: a real list with operations to select the device */
/* single entry table to start with */
static struct mods_dl_device *g_routing;

/* TODO: use max_size to calc max payload */
struct mods_dl_device *mods_create_dl_device(struct mods_dl_driver *drv,
		struct device *dev)
{
	struct mods_dl_device *mods_dev;

	if (!g_hd) {
		dev_err(dev, "NW HD not yet initialized\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	mods_dev = kzalloc(sizeof(*mods_dev), GFP_KERNEL);
	if (!mods_dev)
		return ERR_PTR(-ENOMEM);

	mods_dev->drv = drv;
	mods_dev->dev = dev;

	/* XXX Add to a list */
	g_routing = mods_dev;

	return mods_dev;
}
EXPORT_SYMBOL_GPL(mods_create_dl_device);

void mods_remove_dl_device(struct mods_dl_device *dev)
{
	/* XXX Free from list */
	g_routing = NULL;
	kfree(dev);
}
EXPORT_SYMBOL_GPL(mods_remove_dl_device);

void mods_data_rcvd(struct mods_dl_device *nd, uint8_t *data)
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

static int mods_msg_send(struct greybus_host_device *hd,
		u16 hd_cport_id,
		struct gb_message *message,
		gfp_t gfp_mask)
{
	size_t buffer_size;
	struct gb_connection *connection;
	struct muc_msg *msg;
	int rv = -EINVAL;

	if (message->payload_size > PAYLOAD_MAX_SIZE)
		return -E2BIG;

	/* XXX - This should lookup in the routing table for destination
	 * device and cport id. Which will also let us identify the DL
	 * driver to send towards.
	 */
	connection = gb_connection_hd_find(hd, hd_cport_id);
	if (!connection) {
		pr_err("Invalid cport supplied to send\n");
		return -EINVAL;
	}

	buffer_size = sizeof(*message->header) + message->payload_size;

	msg = (struct muc_msg *)kzalloc(buffer_size +
			sizeof(struct muc_msg_hdr), gfp_mask);
	if (!msg) {
		return -ENOMEM;
	}

	msg->hdr.dest_cport = connection->intf_cport_id;
	msg->hdr.src_cport = connection->hd_cport_id;
	msg->hdr.size = buffer_size + sizeof(struct muc_msg_hdr);
	memcpy(&msg->gb_msg[0], message->buffer, buffer_size);

	pr_info("AP (CPort %d) -> Module (CPort %d)\n",
			connection->hd_cport_id, connection->intf_cport_id);

	/* hand off to the dl layer */
	if (g_routing && g_routing->drv && g_routing->drv->message_send)
		rv = g_routing->drv->message_send(
				(struct mods_dl_device *)g_routing,
				(uint8_t *)msg,
				msg->hdr.size);

	/* Tell submitter that the message send (attempt) is
	 * complete and save the status.
	 */
	greybus_message_sent(hd, message, rv);

	kfree(msg);

	return rv;
}

static void mods_msg_cancel(struct gb_message *message)
{
	/* nothing currently */
}

static struct greybus_host_driver mods_nw_host_driver = {
	.hd_priv_size		= sizeof(struct mods_nw_data),
	.message_send		= mods_msg_send,
	.message_cancel		= mods_msg_cancel,
};

static int mods_nw_probe(struct platform_device *pdev)
{
	struct mods_nw_data *dd;

	/* setup host device */
	g_hd = greybus_create_hd(&mods_nw_host_driver, &pdev->dev,
			PAYLOAD_MAX_SIZE);
	if (IS_ERR(g_hd)) {
		dev_err(&pdev->dev, "Unable to create greybus host driver.\n");
		return PTR_ERR(g_hd);
	}

	/* register attach */
	dd = (struct mods_nw_data *)&g_hd->hd_priv;
	dd->hd = g_hd;
	platform_set_drvdata(pdev, dd);

	atomic_set(&dd->device_id, 0);
	INIT_LIST_HEAD(&dd->nw_dev);

	dd->attach_nb.notifier_call = mods_nw_attach;
	register_muc_attach_notifier(&dd->attach_nb);

	/* XXX Kick off the SVC initialization, it needs to exist */

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

static struct platform_device *mods_nw_device;

int __init mods_nw_init(void)
{
	int rv;

	rv = platform_driver_register(&mods_nw_driver);
	if (rv < 0) {
		pr_err("mods_nw failed to register driver\n");
		return rv;
	}

	mods_nw_device = platform_device_alloc("mods_nw", -1);
	if (!mods_nw_device) {
		rv = -ENOMEM;
		pr_err("mods_nw failed to alloc device\n");
		goto alloc_fail;
	}

	rv = platform_device_add(mods_nw_device);
	if (rv) {
		pr_err("mods_nw failed to add device: %d\n", rv);
		goto add_fail;
	}

	return 0;

add_fail:
	platform_device_put(mods_nw_device);
alloc_fail:
	platform_driver_unregister(&mods_nw_driver);

	return rv;
}

void __exit mods_nw_exit(void)
{
	platform_driver_unregister(&mods_nw_driver);
	platform_device_unregister(mods_nw_device);
}
