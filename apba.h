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

struct mhb_hdr;

int apba_uart_register(void *mods_uart);
void apba_handle_message(struct mhb_hdr *hdr, uint8_t *payload, size_t len);

int apba_enable(void);
void apba_disable(void);

/* PM */
int apba_send_pm_wake_rsp(void);
int apba_send_pm_sleep_req(void);
void apba_wake_assert(bool assert);

/* Driver Initializations */
int apba_ctrl_init(void);
void apba_ctrl_exit(void);

#endif  /* __APBA_H__ */

