/*
 * Copyright (C) 2015 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _MODS_NW_H__
#define _MODS_NW_H__

struct mods_host_device;

#define MUC_MSG_SIZE_MAX        (1024)

struct mods_host_driver {
	size_t hd_priv_size;

	int (*message_send)(struct mods_host_device *hd, uint8_t *payload,
			size_t size);
	void (*message_cancel)(void *cookie);
};

struct mods_host_device {
	struct device *dev;
	struct mods_host_driver *drv;
	void *hd_priv;
};

extern void mods_data_rcvd(struct mods_host_device *hd, uint8_t *data);
extern struct mods_host_device *mods_create_hd(struct mods_host_driver *drv,
		struct device *parent);
extern void mods_remove_hd(struct mods_host_device *dev);
#endif
