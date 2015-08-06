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

struct mods_dl_device;

#define MUC_MSG_SIZE_MAX        (1024)

struct mods_dl_driver {
	size_t dl_priv_size;

	int (*message_send)(struct mods_dl_device *nd, uint8_t *payload,
			size_t size);
	void (*message_cancel)(void *cookie);
};

struct mods_dl_device {
	struct device *dev;
	struct mods_dl_driver *drv;
	void *dl_priv;
};

extern void mods_data_rcvd(struct mods_dl_device *nd, uint8_t *data);
extern struct mods_dl_device *mods_create_dl_device(struct mods_dl_driver *drv,
		struct device *parent);
extern void mods_remove_dl_device(struct mods_dl_device *nd);
#endif
