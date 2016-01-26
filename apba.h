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
 */

#ifndef __APBA_H__
#define __APBA_H__

#define APBA_MSG_SIZE_MAX  (0x800)

/* message from APBA Ctrl driver in kernel */
#pragma pack(push, 1)
struct apba_ctrl_msg_hdr {
	__le16 type;
	__le16 size; /* size of data followed by hdr */
};
#pragma pack(pop)

/* APBA CTRL message types */
enum {
	APBA_CTRL_INT_REASON,
	APBA_CTRL_PM_WAKE_ACK,
	APBA_CTRL_PM_SLEEP_IND,
	APBA_CTRL_PM_SLEEP_ACK,
	APBA_CTRL_LOG_IND,
	APBA_CTRL_LOG_REQUEST,
	APBA_CTRL_MODE_REQUEST,
};

int apba_uart_register(void *mods_uart);
void apba_handle_message(uint8_t *payload, size_t len);

int apba_enable(void);
void apba_disable(void);

void apba_wake_assert(bool assert);

/* Driver Initializations */
int apba_ctrl_init(void);
void apba_ctrl_exit(void);
#endif  /* __APBA_H__ */

