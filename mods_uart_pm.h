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

#ifndef __MODS_UART_PM_H__
#define __MODS_UART_PM_H__

#define UART_PM_FLAG_WAKE_ACK 1
#define UART_PM_FLAG_SLEEP_ACK 2
#define UART_PM_FLAG_SLEEP_IND 3

void mods_uart_pm_on(void *uart_data);
void mods_uart_pm_off(void *uart_data);
void mods_uart_pm_handle_wake_interrupt(void *uart_data);

void mods_uart_pm_update_idle_timer(void *uart_pm_data);
void mods_uart_pm_cancel_timer(void *uart_pm_data);

void mods_uart_pm_pre_tx(void *uart_pm_data, int flag);
void mods_uart_pm_post_tx(void *uart_pm_data, int flag);

void *mods_uart_pm_initialize(void *uart_data);
void mods_uart_pm_uninitialize(void *uart_pm_data);

void mods_uart_pm_handle_pm_wake_rsp(void *uart_data);

#endif  /* __MODS_UART__PM_H__ */
