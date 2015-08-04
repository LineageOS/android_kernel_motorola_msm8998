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

#include "endo.h"
#include "greybus.h"

#include "muc_svc.h"

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

int __init muc_svc_init(void)
{
	return 0;
}

void __exit muc_svc_exit(void)
{
	/* nada */
}
