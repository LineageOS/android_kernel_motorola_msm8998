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

#define CONFIG_MODS_DEV_MAX     (4) /* TODO: move to Kconfig */

struct dest_entry {
	struct mods_dl_device *dev; /* TODO: keep or lookup in route function */
	u8 intf;
	u8 cport;
};

struct cport_set {
	struct mods_dl_device	*dev;
	struct dest_entry	dest[CONFIG_CPORT_ID_MAX];
};

static struct cport_set routes[CONFIG_MODS_DEV_MAX];

/* TODO: reference counting  */
/* TODO: clear all routes */

/* add the dl device to the table */
/* called by the svc while creating the dl device */
void mods_nw_add_dl_device(struct mods_dl_device *mods_dev)
{
	BUG_ON(mods_dev == NULL);
	BUG_ON(mods_dev->intf_id >= CONFIG_MODS_DEV_MAX);

	routes[mods_dev->intf_id].dev = mods_dev;
}

void mods_nw_del_dl_device(struct mods_dl_device *mods_dev)
{
	BUG_ON(mods_dev == NULL);
	BUG_ON(mods_dev->intf_id >= CONFIG_MODS_DEV_MAX);

	memset(&routes[mods_dev->intf_id], 0, sizeof(struct cport_set));
}

int mods_nw_add_route(u8 from_intf, u8 from_cport, u8 to_intf, u8 to_cport)
{
	int err = 0;
	struct cport_set *from_cset;
	struct mods_dl_device *to_dev;

	BUG_ON(from_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(to_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(from_cport >= CONFIG_CPORT_ID_MAX);
	BUG_ON(to_cport >= CONFIG_CPORT_ID_MAX);

	from_cset = &routes[from_intf];
	to_dev = routes[to_intf].dev;
	if (from_cset->dev && to_dev) {
		from_cset->dest[from_cport].intf = to_intf;
		from_cset->dest[from_cport].cport = to_cport;
		from_cset->dest[from_cport].dev = routes[to_intf].dev;
	} else {
		pr_err("unable to add route %u:%u -> %u:%u\n",
			from_intf, from_cport, to_intf, to_cport);
		err = -ENODEV;
	}
	return err;
}

void mods_nw_del_route(u8 from_intf, u8 from_cport, u8 to_intf, u8 to_cport)
{
	struct cport_set *from_cset = &routes[from_intf];

	BUG_ON(from_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(to_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(from_cport >= CONFIG_CPORT_ID_MAX);
	BUG_ON(to_cport >= CONFIG_CPORT_ID_MAX);

	from_cset = &routes[from_intf];
	if (from_cset->dev) {
		from_cset->dest[from_cport].dev = NULL;
		from_cset->dest[from_cport].intf = 0;
		from_cset->dest[from_cport].cport = 0;
	}
}

static inline void mods_msg_dump(const char *str, struct muc_msg *mm)
{
	struct gb_operation     *gb_op;

	gb_op = (struct gb_operation *)mm->gb_msg;
	pr_info("%s [%4u] type=%u size=%d (%u -> %u)\n", str,
		gb_op->id, gb_op->type,
		mm->hdr.size, mm->hdr.src_cport, mm->hdr.dest_cport);
}

int mods_nw_switch(struct mods_dl_device *from, uint8_t *msg)
{
	struct muc_msg *mm;
	struct dest_entry dest;
	int err = -ENODEV;

	if (!msg || !from) {
		pr_err("bad arguments\n");
		return -EINVAL;
	}

	mm = (struct muc_msg *)msg;
	mods_msg_dump(__func__, mm);

	if (from->intf_id >= CONFIG_MODS_DEV_MAX) {
		dev_err(from->dev, "Attempt to send with invalid IID\n");
		err = -EINVAL;
		goto out;
	}
	if (mm->hdr.src_cport >= CONFIG_CPORT_ID_MAX) {
		dev_err(from->dev, "Attempt to send on invalid cport\n");
		err = -EINVAL;
		goto out;
	}
	if (routes[from->intf_id].dev) {
		dest = routes[from->intf_id].dest[mm->hdr.src_cport];
		if (dest.dev)
			err = dest.dev->drv->message_send(dest.dev, msg,
					mm->hdr.size);
	} else
		dev_err(from->dev, "No route for %u:%u\n",
				from->intf_id, mm->hdr.src_cport);

out:
	return err;
}
