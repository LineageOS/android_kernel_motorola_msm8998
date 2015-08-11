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
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include "greybus.h"
#include "muc_svc.h"
#include "muc_attach.h"
#include "mods_nw.h"

	
/* TODO: a real list with operations to select the device */
/* single entry table to start with */
static LIST_HEAD(g_routing);
static DEFINE_SPINLOCK(g_routing_lock);

/* add the dl device to the table */
/* called by the svc while creating the dl device */
void mods_nw_add_dl_device(struct mods_dl_device *mods_dev)
{
	spin_lock_irq(&g_routing_lock);
	list_add(&mods_dev->list, &g_routing);
	spin_unlock_irq(&g_routing_lock);

}

void mods_nw_del_dl_device(struct mods_dl_device *mods_dev)
{
	spin_lock_irq(&g_routing_lock);
	list_del(&mods_dev->list);
	spin_unlock_irq(&g_routing_lock);
}

static inline struct mods_dl_device *find_by_intf(struct list_head *list,
	u8 intf_id)
{
	struct mods_dl_device *dld = NULL;
	struct list_head *ptr;

	spin_lock_irq(&g_routing_lock);
	list_for_each(ptr, list) {
		dld = list_entry(ptr, struct mods_dl_device, list);
		if (dld && dld->intf_id == intf_id) {
			pr_info("%s match found for intf_id = %d\n", __func__, intf_id);
			break;
		}
	}
	spin_unlock_irq(&g_routing_lock);
	return dld;
}

static inline void mods_msg_dump(const char *str, struct muc_msg *mm)
{
	struct gb_operation     *gb_op;
	gb_op = (struct gb_operation*)mm->gb_msg;

	pr_info("%s [%4u] type=%u size=%d (%u -> %u)\n", str,
		gb_op->id, gb_op->type,
		mm->hdr.size, mm->hdr.src_cport, mm->hdr.dest_cport);
}

int mods_nw_switch(struct mods_dl_device *from, uint8_t *msg)
{
	struct mods_dl_device *to = NULL;
	struct muc_msg *mm;
	size_t msg_size;
	int err = -ENODEV;

	if (!msg || !from) {
		pr_err("bad arguments\n");
		return -EINVAL;
	}

	mm = (struct muc_msg *)msg;
	msg_size = mm->hdr.size;
	
	/* FIXME - do something */
	switch (from->intf_id) {
	case MODS_INTF_AP:
		/* send to muc */
		to = find_by_intf(&g_routing, MODS_INTF_SVC);
		break;
	case MODS_INTF_MUC:
		/* send to ap */
		to = find_by_intf(&g_routing, MODS_INTF_AP);
		break;
	case MODS_INTF_SVC:
		to = find_by_intf(&g_routing, MODS_INTF_AP);
		break;
	default:
		pr_err("%s - Unsupported intf %d\n", __func__, from->intf_id);
	}
	mods_msg_dump(__func__, mm);
	msleep(10);

	if (to)
		err = to->drv->message_send(to, msg, msg_size);

	return err;
}
