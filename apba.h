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

int apba_uart_register(void *mods_uart);
void apba_handle_message(uint8_t *payload, size_t len);

int apba_enable(void);
void apba_disable(void);

/* Driver Initializations */
int apba_ctrl_init(void);
void apba_ctrl_exit(void);
#endif  /* __APBA_H__ */

