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

#pragma pack(push, 1)
struct muc_msg_hdr {
	__le16  gb_msg_size;
	__u8    cport;
};

struct muc_msg {
	struct muc_msg_hdr hdr;
	__u8    gb_msg[0];
};
#pragma pack(pop)

#define MUC_MSG_SIZE_MAX        (1024)
#define PAYLOAD_MAX_SIZE        (MUC_MSG_SIZE_MAX - sizeof(struct muc_msg))

struct mods_dl_driver {
	size_t dl_priv_size;

	int (*message_send)(struct mods_dl_device *nd, uint8_t *payload,
			size_t size);
	void (*message_cancel)(void *cookie);
};

struct mods_dl_device {
	struct list_head	list;
	struct device		*dev;
	struct mods_dl_driver	*drv;
	u8			intf_id;
	u8			device_id;
	void			*dl_priv;
	struct kobject		intf_kobj;
	struct bin_attribute	manifest_attr;

	struct muc_svc_hotplug_work *hpw;
	char *manifest;
	__le16 manifest_size;

	__le64 uid_low;
	__le64 uid_high;
};

/* interfaces with the svc */
extern int mods_nw_add_route(u8 from_intf, u8 from_cport,
		u8 to_intf, u8 to_cport);
extern void mods_nw_del_route(u8 from_intf, u8 from_cport,
		u8 to_intf, u8 to_cport);
extern void mods_nw_add_dl_device(struct mods_dl_device *mods_dev);
extern void mods_nw_del_dl_device(struct mods_dl_device *mods_dev);
extern struct mods_dl_device *mods_nw_get_dl_device(u8 intf_id);

/* send message to switch to connect to destination */
extern int mods_nw_switch(struct mods_dl_device *from, uint8_t *msg);
#endif
