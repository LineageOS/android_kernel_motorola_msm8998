/*
 * Copyright (C) 2016 Motorola Mobility, Inc.
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

#ifndef __GREYBUS_CUST_KERNEL_VER_H
#define __GREYBUS_CUST_KERNEL_VER_H

/* Error Log Reporting */
#ifdef CONFIG_DROPBOX
#include <linux/dropbox.h>
static inline void
mods_queue_error_event_text(char *name, void *data, size_t size)
{
	dropbox_queue_event_text(name, data, size);
}

static inline void
mods_queue_error_event_empty(char *name)
{
	dropbox_queue_event_empty(name);
}
#else
static inline void
mods_queue_error_event_text(char *name, void *data, size_t size) {}

static inline void
mods_queue_error_event_empty(char *name) {}
#endif

#if defined(CONFIG_MODS_MODBUS_EXT) || defined(CONFIG_MODS_MODBUS_SWITCH)
#include <linux/mods/modbus_ext.h>

static inline void mods_ext_bus_vote(bool enable)
{
	struct modbus_ext_status modbus_status;

	modbus_status.proto = MODBUS_PROTO_MPHY;
	modbus_status.active = enable;
	modbus_ext_set_state(&modbus_status);
}
#else
static inline void mods_ext_bus_vote(bool enable) {}
#endif

#endif /* __GREYBUS_CUST_KERNEL_VER_H */
