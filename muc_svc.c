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
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>

#include "greybus.h"

#include "muc_svc.h"
#include "mods_nw.h"

struct muc_svc_data {
	struct mods_dl_device *dld;
};

#define HP_BASE_SIZE      (sizeof(struct svc_msg_header) + 2)
#define LU_PAYLOAD_SIZE   (sizeof(struct svc_function_unipro_management))
#define LU_MSG_SIZE       (sizeof(struct svc_msg_header) + LU_PAYLOAD_SIZE)

static void send_hot_plug(struct greybus_host_device *hd, int iid)
{
	pr_info("SVC -> AP hotplug event (plug) sent\n");
}

/* Mock SVC message to the AP */
static void send_hot_unplug(struct greybus_host_device *hd, int iid)
{
	printk("%s: SVC->AP hotplug event (unplug) sent\n", __func__);
}

static void send_link_up(struct greybus_host_device *hd, int iid, int did)
{
	pr_info("SVC -> AP Link Up (%d:%d) message sent\n", iid, did);
}


void muc_svc_attach(struct greybus_host_device *hd)
{
	send_hot_plug(hd, 1);
	send_link_up(hd, 1, 2);
}
EXPORT_SYMBOL(muc_svc_attach);

void muc_svc_detach(struct greybus_host_device *hd)
{
	send_hot_unplug(hd, 1);
}
EXPORT_SYMBOL(muc_svc_detach);

static int
muc_svc_msg_send(struct mods_dl_device *dld, uint8_t *buf, size_t len)
{
	return 0;
}

static void muc_svc_msg_cancel(void *cookie)
{
	/* Should never happen */
}

static struct mods_dl_driver muc_svc_dl_driver = {
	.dl_priv_size = sizeof(struct muc_svc_data),
	.message_send = muc_svc_msg_send,
	.message_cancel = muc_svc_msg_cancel,
};

static int muc_svc_probe(struct platform_device *pdev)
{
	struct muc_svc_data *dd;

	dd = devm_kzalloc(&pdev->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	dd->dld = mods_create_dl_device(&muc_svc_dl_driver, &pdev->dev);
	if (IS_ERR(dd->dld)) {
		dev_err(&pdev->dev, "Failed to create mods DL device.\n");
		return PTR_ERR(dd->dld);
	}

	platform_set_drvdata(pdev, dd);

	return 0;
}

static int muc_svc_remove(struct platform_device *pdev)
{
	struct muc_svc_data *dd = platform_get_drvdata(pdev);

	mods_remove_dl_device(dd->dld);

	return 0;
}

static struct platform_driver muc_svc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "muc_svc",
	},
	.probe = muc_svc_probe,
	.remove  = muc_svc_remove,
};

static struct platform_device *muc_svc_device;

int __init muc_svc_init(void)
{
	int ret;

	ret = platform_driver_register(&muc_svc_driver);
	if (ret < 0) {
		pr_err("muc_svc failed to register driver\n");
		return ret;
	}

	muc_svc_device = platform_device_alloc("muc_svc", -1);
	if (!muc_svc_device) {
		ret = -ENOMEM;
		pr_err("muc_svc failed to alloc device\n");
		goto alloc_fail;
	}

	ret = platform_device_add(muc_svc_device);
	if (ret) {
		pr_err("muc_svc failed to add device: %d\n", ret);
		goto add_fail;
	}

	return 0;

add_fail:
	platform_device_put(muc_svc_device);
alloc_fail:
	platform_driver_unregister(&muc_svc_driver);

	return ret;
}

void __exit muc_svc_exit(void)
{
	platform_driver_unregister(&muc_svc_driver);
	platform_device_unregister(muc_svc_device);
}
