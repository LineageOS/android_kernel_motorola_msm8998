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

int mods_uart_apba_send(void *uart_data, uint8_t *buf, size_t len);

/* Driver Initializations */
int mods_uart_init(void);
void mods_uart_exit(void);
#endif  /* __MODS_UART_H__ */

