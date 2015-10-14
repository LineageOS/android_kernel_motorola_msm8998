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
#include <linux/module.h>
#include <linux/platform_device.h>

#include "greybus.h"

#include "mods_nw.h"
#include "muc_svc.h"

static struct gb_host_device *g_hd;

struct mods_ap_data {
	struct mods_dl_device *dld;
	struct gb_host_device *hd;
};

/* got a message from the nw switch forward it to greybus */
static int mods_ap_message_send(struct mods_dl_device *dld,
		uint8_t *buf, size_t len)
{
	struct muc_msg *msg = (struct muc_msg *)buf;

	greybus_data_rcvd(g_hd, le16_to_cpu(msg->hdr.cport),
			msg->gb_msg, (len - sizeof(msg->hdr)));
	return 0;
}

/* Get the corresponding connection's protocol */
static int mods_ap_get_protocol(uint16_t cport_id, uint8_t *protocol)
{
	struct gb_connection *conn;

	if (!g_hd)
		return -ENODEV;

	conn = gb_connection_hd_find(g_hd, cport_id);
	if (!conn) {
		pr_err("mods_ap: couldn't find protocol for: %d\n", cport_id);
		return -ENODEV;
	}

	*protocol = conn->protocol_id;

	return 0;
}

static struct mods_dl_driver mods_ap_dl_driver = {
	.dl_priv_size		= sizeof(struct mods_ap_data),
	.message_send		= mods_ap_message_send,
	.get_protocol		= mods_ap_get_protocol,
};

/* received a message from the AP to send to the switch */
static int mods_ap_msg_send(struct gb_host_device *hd,
		u16 hd_cport_id,
		struct gb_message *message,
		gfp_t gfp_mask)
{
	size_t buffer_size;
	size_t msg_size;
	struct muc_msg *msg;
	struct mods_ap_data *data;
	struct mods_dl_device *dl;
	int rv = -EINVAL;

	if (message->payload_size > PAYLOAD_MAX_SIZE)
		return -E2BIG;

	data = (struct mods_ap_data *)hd->hd_priv;
	dl = data->dld;

	buffer_size = sizeof(*message->header) + message->payload_size;

	msg_size = buffer_size + sizeof(struct muc_msg_hdr);
	msg = kzalloc(msg_size, gfp_mask);
	if (!msg)
		return -ENOMEM;

	msg->hdr.cport = cpu_to_le16(hd_cport_id);
	memcpy(&msg->gb_msg[0], message->buffer, buffer_size);

	/* hand off to the nw layer */
	rv = mods_nw_switch(dl, (uint8_t *)msg, msg_size);

	/* Tell submitter that the message send (attempt) is
	 * complete and save the status.
	 */
	greybus_message_sent(hd, message, rv);

	kfree(msg);

	return rv;
}

static void mods_ap_msg_cancel(struct gb_message *message)
{
	/* nothing currently */
}

static struct gb_hd_driver mods_ap_host_driver = {
	.hd_priv_size		= sizeof(struct mods_ap_data),
	.message_send		= mods_ap_msg_send,
	.message_cancel		= mods_ap_msg_cancel,
};

static int mods_ap_probe(struct platform_device *pdev)
{
	int err = 0;
	struct mods_ap_data *ap_data;

	/* setup host device */
	/* The use of ID_MAX is internally used as both a count and
	 * a max.  Here it is a count.  CONFIG_CPORT_ID_MAX will
	 * go away soon and this should be fixed then.
	 */
	g_hd = greybus_create_hd(&mods_ap_host_driver, &pdev->dev,
			PAYLOAD_MAX_SIZE, CONFIG_CPORT_ID_MAX);
	if (IS_ERR(g_hd)) {
		dev_err(&pdev->dev, "Unable to create greybus host driver.\n");
		return PTR_ERR(g_hd);
	}
	ap_data = (struct mods_ap_data *)&g_hd->hd_priv;
	ap_data->hd = g_hd;
	platform_set_drvdata(pdev, ap_data);

	/* create our data link device */
	ap_data->dld = mods_create_dl_device(&mods_ap_dl_driver,
			&pdev->dev, MODS_INTF_AP);
	if (IS_ERR(ap_data->dld)) {
		err = PTR_ERR(ap_data->dld);
		dev_err(&pdev->dev, "Unable to create greybus host driver.\n");
		if (err == -ENODEV) {
			/* the AP is a special case that creates a route with
			 * the SVC.  ENODEV means that the SVC isn't ready and
			 * we should try again later */
			err = -EPROBE_DEFER;
		}

		goto err;
	}

	err = mods_dl_dev_attached(ap_data->dld);
	if (err) {
		dev_err(&pdev->dev, "Unable to notify SVC of attach\n");
		goto remove_dl;
	}

	return 0;

remove_dl:
	mods_remove_dl_device(ap_data->dld);
err:
	greybus_remove_hd(g_hd);
	g_hd = NULL;
	return err;

}

static int mods_ap_remove(struct platform_device *pdev)
{
	struct mods_ap_data *ap_data = platform_get_drvdata(pdev);

	mods_remove_dl_device(ap_data->dld);
	greybus_remove_hd(ap_data->hd);

	return 0;
}

static struct platform_driver mods_ap_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mods_ap",
	},
	.probe = mods_ap_probe,
	.remove = mods_ap_remove,
};

static struct platform_device *mods_ap_device;

int __init mods_ap_init(void)
{
	int err;

	err = platform_driver_register(&mods_ap_driver);
	if (err) {
		pr_err("mods ap failed to register driver\n");
		return err;
	}

	mods_ap_device = platform_device_alloc("mods_ap", -1);
	if (!mods_ap_device) {
		err = -ENOMEM;
		pr_err("mods ap failed to alloc device\n");
		goto alloc_fail;
	}

	err = platform_device_add(mods_ap_device);
	if (err) {
		pr_err("mods ap failed to add device: %d\n", err);
		goto add_fail;
	}

	return 0;
add_fail:
	platform_device_put(mods_ap_device);
alloc_fail:
	platform_driver_unregister(&mods_ap_driver);
	return err;
}

void __exit mods_ap_exit(void)
{
	platform_device_unregister(mods_ap_device);
	platform_driver_unregister(&mods_ap_driver);
}
