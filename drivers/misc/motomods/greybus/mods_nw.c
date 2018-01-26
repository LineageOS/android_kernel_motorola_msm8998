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
#include <linux/radix-tree.h>

#include "greybus.h"
#include "muc_svc.h"
#include "mods_nw.h"
#include "mods_trace.h"

struct dest_entry {
	struct mods_dl_device *dev;
	u16 cport;
	u8 protocol_id;
	u8 protocol_valid:1;
	u8 filter:1;
};

struct cport_set {
	struct mods_dl_device *dev;
	struct radix_tree_root  tree;
};

static LIST_HEAD(mods_nw_filters);
static RADIX_TREE(nw_interfaces, GFP_KERNEL);
static DEFINE_MUTEX(list_lock);

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
	struct cport_set *route;

	route = radix_tree_lookup(&nw_interfaces, intf_id);
	if (!route)
		return NULL;

	return route->dev;
}

struct mods_dl_device *
mods_nw_find_dest_dl_device(struct mods_dl_device *from, u16 cport)
{
	struct dest_entry *dest;
	struct cport_set *route;

	if (!from)
		return ERR_PTR(-EINVAL);

	route = radix_tree_lookup(&nw_interfaces, from->intf_id);
	if (!route) {
		dev_err(from->dev, "DLD not found for interface: %d\n",
				from->intf_id);
		return ERR_PTR(-ENODEV);
	}

	dest = radix_tree_lookup(&route->tree, cport);
	if (!dest) {
		dev_err(from->dev, "No route for %u:%u\n",
				from->intf_id, cport);
		return ERR_PTR(-ENODEV);
	}

	return dest->dev;
}

/* add the dl device to the table */
/* called by the svc while creating the dl device */
int mods_nw_add_dl_device(struct mods_dl_device *mods_dev)
{
	struct cport_set *new;
	int ret = 0;

	if (!mods_dev)
		return -EINVAL;

	mutex_lock(&list_lock);
	if (radix_tree_lookup(&nw_interfaces, mods_dev->intf_id)) {
		ret = -EEXIST;
		goto unlock;
	}

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		ret = -ENOMEM;
		goto unlock;
	}

	new->dev = mods_dev;
	INIT_RADIX_TREE(&new->tree, GFP_KERNEL);
	radix_tree_insert(&nw_interfaces, mods_dev->intf_id, new);

unlock:
	mutex_unlock(&list_lock);

	return ret;
}

void mods_nw_del_dl_device(struct mods_dl_device *mods_dev)
{
	struct cport_set *set;

	if (!mods_dev)
		return;

	mutex_lock(&list_lock);
	set = radix_tree_lookup(&nw_interfaces, mods_dev->intf_id);
	if (!set)
		goto unlock;

	radix_tree_delete(&nw_interfaces, mods_dev->intf_id);
	kfree(set);

unlock:
	mutex_unlock(&list_lock);
}

int mods_nw_add_route(u8 from_intf, u16 from_cport, u8 to_intf, u16 to_cport)
{
	int err = 0;
	struct cport_set *from_cset;
	struct cport_set *to_cset;
	uint8_t protocol;
	bool filter = false;
	struct dest_entry *from_entry;
	struct dest_entry *to_entry;
	bool alloc_from = false;
	bool alloc_to = false;

	from_cset = radix_tree_lookup(&nw_interfaces, from_intf);
	to_cset = radix_tree_lookup(&nw_interfaces, to_intf);

	if (!from_cset || !to_cset) {
		pr_err("Unable to find cset %u:%u -> %u:%u\n",
			from_intf, from_cport, to_intf, to_cport);
		return -ENODEV;
	}

	/* Find existing entries which might have been created on the
	 * first route direction. Both entries need to exist for filter
	 * to be configured.
	 */
	from_entry = radix_tree_lookup(&from_cset->tree, from_cport);
	if (!from_entry) {
		from_entry = kzalloc(sizeof(*from_entry), GFP_KERNEL);
		if (!from_entry) {
			err = -ENOMEM;
			goto cleanup;
		}
		mutex_lock(&list_lock);
		radix_tree_insert(&from_cset->tree, from_cport, from_entry);
		mutex_unlock(&list_lock);
		alloc_from = true;
	}

	to_entry = radix_tree_lookup(&to_cset->tree, to_cport);
	if (!to_entry) {
		to_entry = kzalloc(sizeof(*to_entry), GFP_KERNEL);
		if (!to_entry) {
			err = -ENOMEM;
			goto cleanup;
		}
		mutex_lock(&list_lock);
		radix_tree_insert(&to_cset->tree, to_cport, to_entry);
		mutex_unlock(&list_lock);
		alloc_to = true;
	}

	from_entry->cport = to_cport;
	from_entry->dev = to_cset->dev;

	/* If there is no protocol handler, we are done */
	if (!from_cset->dev->drv->get_protocol)
		return 0;

	/* Try to get the protocol, any error should be fatal */
	err = from_cset->dev->drv->get_protocol(from_cport, &protocol);
	if (err) {
		pr_warn("Unable to get a protocol for %d:%d\n",
				from_intf, from_cport);
		goto cleanup;
	}

	/* Look for previously installed filters */
	filter = _mods_nw_filter_present(protocol);

	/* Save the protocol and filter status */
	from_entry->protocol_valid = true;
	from_entry->protocol_id = protocol;
	from_entry->filter = filter;

	to_entry->protocol_id = protocol;
	to_entry->protocol_valid = true;
	to_entry->filter = filter;

	return 0;

cleanup:
	mutex_lock(&list_lock);
	if (alloc_to) {
		radix_tree_delete(&to_cset->tree, to_cport);
		kfree(to_entry);
	}
	if (alloc_from) {
		radix_tree_delete(&from_cset->tree, from_cport);
		kfree(from_entry);
	}
	mutex_unlock(&list_lock);

	return err;
}

void mods_nw_del_route(u8 from_intf, u16 from_cport, u8 to_intf, u16 to_cport)
{
	struct cport_set *from_cset;
	struct dest_entry *entry;

	mutex_lock(&list_lock);
	from_cset = radix_tree_lookup(&nw_interfaces, from_intf);
	if (!from_cset)
		goto unlock;

	entry = radix_tree_lookup(&from_cset->tree, from_cport);
	if (entry) {
		radix_tree_delete(&from_cset->tree, from_cport);
		kfree(entry);
	}

unlock:
	mutex_unlock(&list_lock);
}

int mods_nw_switch(struct mods_dl_device *from, uint8_t *msg, size_t len)
{
	struct muc_msg *mm;
	struct dest_entry *dest;
	int err = -ENODEV;
	struct cport_set *route;

	if (!msg || !from) {
		pr_err("bad arguments\n");
		return -EINVAL;
	}

	mm = (struct muc_msg *)msg;

	route = radix_tree_lookup(&nw_interfaces, from->intf_id);
	if (!route) {
		dev_err(from->dev, "Attempt to send with invalid IID\n");
		err = -EINVAL;
		goto out;
	}

	dest = radix_tree_lookup(&route->tree, le16_to_cpu(mm->hdr.cport));
	if (!dest) {
		dev_err(from->dev, "No route for %u:%u\n",
				from->intf_id, le16_to_cpu(mm->hdr.cport));
		goto out;
	}

	trace_mods_switch((struct gb_operation_msg_hdr *)mm->gb_msg,
		from->intf_id, le16_to_cpu(mm->hdr.cport),
		dest->dev->intf_id, dest->cport);

	mm->hdr.cport = cpu_to_le16(dest->cport);

	/* Try to apply any filter installed, or run standard message
	 * send if no filter was present. A filter can also choose
	 * to allow the message to continue to pass through with this
	 * error code.
	 */
	err = _mods_nw_apply_filter(dest, dest->dev, msg, len);
	if (err == -ENOENT)
		err = dest->dev->drv->message_send(dest->dev, msg, len);

out:
	return err;
}
EXPORT_SYMBOL(mods_nw_switch);

static void _set_filter(uint8_t protocol, bool value)
{
	struct radix_tree_iter rt_iter;
	struct radix_tree_iter cp_iter;
	void **rt_slot;
	void **cp_slot;
	struct cport_set *route;
	struct dest_entry *dest;

	mutex_lock(&list_lock);
	radix_tree_for_each_slot(rt_slot, &nw_interfaces, &rt_iter, 0) {
		route = radix_tree_deref_slot(rt_slot);

		radix_tree_for_each_slot(cp_slot, &route->tree, &cp_iter, 0) {
			dest = radix_tree_deref_slot(cp_slot);
			if (!dest->protocol_valid)
				continue;
			if (dest->protocol_id == protocol)
				dest->filter = value;
		}
	}
	mutex_unlock(&list_lock);
}

int mods_nw_register_filter(struct mods_nw_msg_filter *filter)
{
	struct mods_nw_msg_filter *tmp, *e;
	uint8_t type;
	uint8_t protocol;
	bool protocol_match = false;

	if (!filter)
		return -EINVAL;

	type = filter->type;
	protocol = filter->protocol_id;

	if (filter->initialized) {
		pr_warn("filter %d:%d already initialized\n", protocol, type);
		return 0;
	}

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
	_set_filter(protocol, true);

	return 0;
}

void mods_nw_unregister_filter(struct mods_nw_msg_filter *filter)
{
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
	_set_filter(protocol, false);
}
