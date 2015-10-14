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

#define pr_fmt(fmt) "MDNW: " fmt

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

#define CONFIG_MODS_DEV_MAX     (6) /* TODO: move to Kconfig */

struct dest_entry {
	struct mods_dl_device *dev;
	u16 cport;
	u8 protocol_id;
	u8 protocol_valid:1;
	u8 filter:1;
};

struct cport_set {
	struct mods_dl_device	*dev;
	struct dest_entry	dest[CONFIG_CPORT_ID_MAX];
};

static struct cport_set routes[CONFIG_MODS_DEV_MAX];

static LIST_HEAD(mods_nw_filters);

/* TODO: reference counting  */
/* TODO: clear all routes */

static inline bool _mods_nw_filter_present(uint8_t protocol)
{
	struct mods_nw_msg_filter *tmp, *existing;

	/* Check for a filter installed on this protocol */
	list_for_each_entry_safe(existing, tmp, &mods_nw_filters, entry)
		if (existing->protocol_id == protocol)
			return true;

	return false;
}

static inline int
_mods_nw_apply_filter(struct dest_entry *dest, struct mods_dl_device *to,
			uint8_t *payload, size_t size)
{
	struct mods_nw_msg_filter *tmp, *e;
	uint8_t protocol;
	struct muc_msg *mm;
	struct gb_operation_msg_hdr *hdr;

	/* Exit if no filter is present */
	if (!dest->filter)
		return -ENOENT;

	mm = (struct muc_msg *)payload;
	hdr = (struct gb_operation_msg_hdr *)mm->gb_msg;
	protocol = dest->protocol_id;

	/* Check for a filter installed on this protocol */
	list_for_each_entry_safe(e, tmp, &mods_nw_filters, entry)
		if (e->protocol_id == protocol && e->type == hdr->type)
			return e->filter_handler(to, payload, size);

	return -ENOENT;
}

struct mods_dl_device *mods_nw_get_dl_device(u8 intf_id)
{
	if (intf_id >= CONFIG_MODS_DEV_MAX)
		return NULL;

	return routes[intf_id].dev;
}

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

int mods_nw_add_route(u8 from_intf, u16 from_cport, u8 to_intf, u16 to_cport)
{
	int err = 0;
	struct cport_set *from_cset;
	struct mods_dl_device *to_dev;
	uint8_t protocol;
	bool proto_found = false;
	bool filter = false;

	BUG_ON(from_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(to_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(from_cport >= CONFIG_CPORT_ID_MAX);
	BUG_ON(to_cport >= CONFIG_CPORT_ID_MAX);

	from_cset = &routes[from_intf];
	to_dev = routes[to_intf].dev;
	if (from_cset->dev && from_cset->dev->drv->get_protocol) {
		if (!from_cset->dev->drv->get_protocol(from_cport, &protocol)) {
			filter = _mods_nw_filter_present(protocol);

			from_cset->dest[from_cport].protocol_valid = true;
			from_cset->dest[from_cport].protocol_id = protocol;
			from_cset->dest[from_cport].filter = filter;
			pr_debug("added %d:%d and %d:%d protocol %d %s\n",
				from_intf, from_cport, to_intf, to_cport,
				protocol, filter ? "(filter)" : "");
			proto_found = true;
		} else {
			pr_err("Failed to find protocol for cport %d\n",
				from_cport);
		}
	}

	if (from_cset->dev && to_dev) {
		from_cset->dest[from_cport].cport = to_cport;
		from_cset->dest[from_cport].dev = routes[to_intf].dev;
		if (proto_found) {
			routes[to_intf].dest[to_cport].protocol_id = protocol;
			routes[to_intf].dest[to_cport].protocol_valid = true;
			routes[to_intf].dest[to_cport].filter = filter;
		}
	} else {
		pr_err("unable to add route %u:%u -> %u:%u\n",
			from_intf, from_cport, to_intf, to_cport);
		err = -ENODEV;
	}
	return err;
}

void mods_nw_del_route(u8 from_intf, u16 from_cport, u8 to_intf, u16 to_cport)
{
	struct cport_set *from_cset = &routes[from_intf];

	BUG_ON(from_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(to_intf >= CONFIG_MODS_DEV_MAX);
	BUG_ON(from_cport >= CONFIG_CPORT_ID_MAX);
	BUG_ON(to_cport >= CONFIG_CPORT_ID_MAX);

	from_cset = &routes[from_intf];
	if (from_cset->dev)
		memset(&from_cset->dest[from_cport], 0,
			sizeof(struct dest_entry));
}

int mods_nw_switch(struct mods_dl_device *from, uint8_t *msg, size_t len)
{
	struct muc_msg *mm;
	struct dest_entry dest;
	int err = -ENODEV;

	if (!msg || !from) {
		pr_err("bad arguments\n");
		return -EINVAL;
	}

	mm = (struct muc_msg *)msg;

	if (from->intf_id >= CONFIG_MODS_DEV_MAX) {
		dev_err(from->dev, "Attempt to send with invalid IID\n");
		err = -EINVAL;
		goto out;
	}
	if (le16_to_cpu(mm->hdr.cport) >= CONFIG_CPORT_ID_MAX) {
		dev_err(from->dev, "Attempt to send on invalid cport\n");
		err = -EINVAL;
		goto out;
	}

	if (!routes[from->intf_id].dev) {
		dev_err(from->dev, "No device defined for %u:%u\n",
			from->intf_id, le16_to_cpu(mm->hdr.cport));
		goto out;
	}

	dest = routes[from->intf_id].dest[le16_to_cpu(mm->hdr.cport)];
	if (!dest.dev) {
		dev_err(from->dev, "No route for %u:%u\n",
				from->intf_id, le16_to_cpu(mm->hdr.cport));
		goto out;
	}

	mm->hdr.cport = cpu_to_le16(dest.cport);

	/* Try to apply any filter installed, or run standard message
	 * send if no filter was present. A filter can also choose
	 * to allow the message to continue to pass through with this
	 * error code.
	 */
	err = _mods_nw_apply_filter(&dest, dest.dev, msg, len);
	if (err == -ENOENT)
		err = dest.dev->drv->message_send(dest.dev, msg, len);

out:
	return err;
}

int mods_nw_register_filter(struct mods_nw_msg_filter *filter)
{
	struct mods_nw_msg_filter *tmp, *e;
	uint8_t type;
	uint8_t protocol;
	int intf;
	int cport;
	bool protocol_match = false;

	if (!filter)
		return -EINVAL;

	if (filter->initialized) {
		pr_warn("filter %d:%d already initialized\n", protocol, type);
		return 0;
	}

	type = filter->type;
	protocol = filter->protocol_id;

	/* Make sure the filter doesn't already exist */
	list_for_each_entry_safe(e, tmp, &mods_nw_filters, entry) {
		if (e->protocol_id != protocol)
			continue;
		protocol_match = true;
		if (e->type == type)
			return -EEXIST;
	}

	list_add_tail(&filter->entry, &mods_nw_filters);
	filter->initialized = 1;

	/* If there was an existing protocol, it will have been flagged */
	if (protocol_match)
		return 0;

	/* Mark existing connections with filter availabile */
	for (intf = 0; intf < ARRAY_SIZE(routes); intf++) {
		if (!routes[intf].dev)
			continue;

		for (cport = 0; cport < CONFIG_CPORT_ID_MAX; cport++) {
			if (!routes[intf].dest[cport].protocol_valid)
				continue;
			if (routes[intf].dest[cport].protocol_id == protocol)
				routes[intf].dest[cport].filter = true;
		}
	}

	return 0;
}

void mods_nw_unregister_filter(struct mods_nw_msg_filter *filter)
{
	int intf;
	int cport;
	uint8_t protocol;

	if (!filter || !filter->initialized)
		return;

	protocol = filter->protocol_id;
	list_del(&filter->entry);
	filter->initialized = 0;

	/* Exit if protocol still has filters */
	if (_mods_nw_filter_present(filter->protocol_id))
		return;

	/* This was last filter for the protocol, clear its availability */
	for (intf = 0; intf < ARRAY_SIZE(routes); intf++) {
		if (!routes[intf].dev)
			continue;

		for (cport = 0; cport < CONFIG_CPORT_ID_MAX; cport++) {
			if (!routes[intf].dest[cport].protocol_valid)
				continue;
			if (routes[intf].dest[cport].protocol_id == protocol)
				routes[intf].dest[cport].filter = false;
		}
	}
}
