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
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slice_attach.h>
#include <linux/spi/spi.h>

#include "endo.h"
#include "greybus.h"
#include "svc_msg.h"
#include "muc_svc.h"

#define HP_BASE_SIZE      (sizeof(struct svc_msg_header) + 2)
#define LU_PAYLOAD_SIZE   (sizeof(struct svc_function_unipro_management))
#define LU_MSG_SIZE       (sizeof(struct svc_msg_header) + LU_PAYLOAD_SIZE)

/* vendor only manifest */
static unsigned char manifest[] = {
  0x4c, 0x00, 0x00, 0x01, 0x08, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00,
  0x1c, 0x00, 0x02, 0x00, 0x16, 0x01, 0x4d, 0x6f, 0x74, 0x6f, 0x72, 0x6f,
  0x6c, 0x61, 0x20, 0x4d, 0x6f, 0x62, 0x69, 0x6c, 0x69, 0x74, 0x79, 0x2c,
  0x20, 0x4c, 0x4c, 0x43, 0x14, 0x00, 0x02, 0x00, 0x0e, 0x02, 0x45, 0x76,
  0x65, 0x72, 0x79, 0x64, 0x61, 0x79, 0x20, 0x53, 0x6c, 0x69, 0x63, 0x65,
  0x08, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0xff, 0x08, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x00
};
#define MANIFEST_SIZE 76

static void send_hot_plug(struct greybus_host_device *hd, int iid)
{
	struct svc_msg *msg;

	msg = kzalloc(HP_BASE_SIZE + MANIFEST_SIZE, GFP_KERNEL);
	if (!msg)
		return;

	msg->header.function_id = SVC_FUNCTION_HOTPLUG;
	msg->header.message_type = SVC_MSG_DATA;
	msg->header.payload_length = MANIFEST_SIZE + 2;
	msg->hotplug.hotplug_event = SVC_HOTPLUG_EVENT;
	msg->hotplug.interface_id = iid;
	memcpy(msg->hotplug.data, manifest, MANIFEST_SIZE);

	/* Send up hotplug message */
	greybus_svc_in(hd, (u8 *)msg, HP_BASE_SIZE + MANIFEST_SIZE);

	pr_info("SVC -> AP hotplug event (plug) sent\n");
	kfree(msg);
}

/* Mock SVC message to the AP */
static void send_hot_unplug(struct greybus_host_device *hd, int iid)
{
	struct svc_msg msg;

	msg.header.function_id = SVC_FUNCTION_HOTPLUG;
	msg.header.message_type = SVC_MSG_DATA;
	msg.header.payload_length = 2;

	msg.hotplug.hotplug_event = SVC_HOTUNPLUG_EVENT;
	msg.hotplug.interface_id = iid;

	/* Write out hotplug message */
	greybus_svc_in(hd, (u8 *)&msg, HP_BASE_SIZE);

	printk("%s: SVC->AP hotplug event (unplug) sent\n", __func__);
}

static void send_link_up(struct greybus_host_device *hd, int iid, int did)
{
	struct svc_msg msg;

	msg.header.function_id = SVC_FUNCTION_UNIPRO_NETWORK_MANAGEMENT;
	msg.header.message_type = SVC_MSG_DATA;
	msg.header.payload_length = LU_PAYLOAD_SIZE;
	msg.management.management_packet_type = SVC_MANAGEMENT_LINK_UP;
	msg.management.link_up.interface_id = iid;
	msg.management.link_up.device_id = did;

	/* Send up link up message */
	greybus_svc_in(hd, (u8 *)&msg,  LU_MSG_SIZE);
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


MODULE_AUTHOR("Motorola Mobility, LLC");
MODULE_DESCRIPTION("Mods uC (MuC) SVC shim driver");
MODULE_LICENSE("GPL");
