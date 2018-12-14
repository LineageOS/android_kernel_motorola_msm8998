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

#ifndef __MODS_UART_H__
#define __MODS_UART_H__

struct mhb_hdr;

#define MODS_UART_FLAG_NO_TXWAKE	(1 << 0)

int mods_uart_open(void *uart_data);
int mods_uart_close(void *uart_data);

int mods_uart_do_pm(void *uart_data, bool on);
void *mods_uart_get_pm_data(void *uart_data);

int mods_uart_send(void *uart_data, struct mhb_hdr *hdr,
	uint8_t *buf, size_t len, int flag);

int mods_uart_get_baud(void *uart_data);
/* Lock the UART while setting the baud. */
void mods_uart_lock_tx(void *uart_data, bool lock);
int mods_uart_set_baud(void *uart_data, uint32_t baud);

/* Driver Initializations */
int mods_uart_init(void);
void mods_uart_exit(void);

#endif  /* __MODS_UART_H__ */

