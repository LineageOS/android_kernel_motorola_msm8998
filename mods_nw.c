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

/* TODO: use max_size to calc max payload */
struct mods_dl_device *mods_create_dl_device(struct mods_dl_driver *drv,
		struct device *dev, enum mods_dl_role role)
{
	struct mods_dl_device *mods_dev;

	pr_info("%s for %s [%d]\n", __func__, dev_name(dev), role);
	mods_dev = kzalloc(sizeof(*mods_dev), GFP_KERNEL);
	if (!mods_dev)
		return ERR_PTR(-ENOMEM);

	mods_dev->drv = drv;
	mods_dev->dev = dev;
	mods_dev->role = role;

	spin_lock_irq(&g_routing_lock);
	list_add(&mods_dev->list, &g_routing);
	spin_unlock_irq(&g_routing_lock);

	return mods_dev;
}
EXPORT_SYMBOL_GPL(mods_create_dl_device);

void mods_remove_dl_device(struct mods_dl_device *dev)
{
	spin_lock_irq(&g_routing_lock);
	list_del(&dev->list);
	spin_unlock_irq(&g_routing_lock);
	kfree(dev);
}
EXPORT_SYMBOL_GPL(mods_remove_dl_device);

static inline struct mods_dl_device *find_by_role(struct list_head *list,
	enum mods_dl_role role)
{
	struct mods_dl_device *dld = NULL;
	struct list_head *ptr;

	spin_lock_irq(&g_routing_lock);
	list_for_each(ptr, list) {
		dld = list_entry(ptr, struct mods_dl_device, list);
		if (dld && dld->role == role) {
			pr_info("%s match found for role = %d\n", __func__, role);
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
	switch (from->role) {
	case MODS_DL_ROLE_AP:
		/* send to muc */
		to = find_by_role(&g_routing, MODS_DL_ROLD_SVC);
		break;
	case MODS_DL_ROLE_MUC:
		/* send to ap */
		to = find_by_role(&g_routing, MODS_DL_ROLE_AP);
		break;
	case MODS_DL_ROLD_SVC:
		to = find_by_role(&g_routing, MODS_DL_ROLE_AP);
		break;
	default:
		pr_err("%s - Unsupported role %d\n", __func__, from->role);
	}
	mods_msg_dump(__func__, mm);
	msleep(10);

	if (to)
		err = to->drv->message_send(to, msg, msg_size);

	return err;
}
