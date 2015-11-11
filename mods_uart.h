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

#define MODS_UART_FLAG_NO_TXWAKE	(1 << 0)

int mods_uart_apba_send(void *uart_data, uint8_t *buf, size_t len, int flag);
void mod_attach(void *uart_data, unsigned long now_present);

void mods_uart_lock_tx(void *uart_data, bool lock);
int mods_uart_do_pm(void *uart_data, bool on);
void *mods_uart_get_pm_data(void *uart_data);

/* Driver Initializations */
int mods_uart_init(void);
void mods_uart_exit(void);
#endif  /* __MODS_UART_H__ */

