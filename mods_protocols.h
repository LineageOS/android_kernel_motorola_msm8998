/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 * Copyright(c) 2015 - 2016 Motorola LLC. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 for more details.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 * Copyright(c) 2015 - 2016 Motorola LLC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Google Inc. or Linaro Ltd. nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE INC. OR
 * LINARO LTD. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MODS_PROTOCOLS_H
#define __MODS_PROTOCOLS_H

/* Control Protocol */

/* Reserved CPORT on each interface for vendor control protocol */
#define VENDOR_CTRL_DEST_CPORT             (0xFFFF)

/* Version of the Greybus control protocol we support */
#define MB_CONTROL_VERSION_MAJOR              0x00
#define MB_CONTROL_VERSION_MINOR              0x03

/* Greybus control request types */
#define MB_CONTROL_TYPE_INVALID               0x00
#define MB_CONTROL_TYPE_PROTOCOL_VERSION      0x01
#define MB_CONTROL_TYPE_GET_IDS               0x02
#define MB_CONTROL_TYPE_REBOOT                0x03
#define MB_CONTROL_TYPE_PORT_CONNECTED        0x04
#define MB_CONTROL_TYPE_PORT_DISCONNECTED     0x05
#define MB_CONTROL_TYPE_SLAVE_POWER           0x06
#define MB_CONTROL_TYPE_GET_ROOT_VER          0x07
#define MB_CONTROL_TYPE_RTC_SYNC              0x08

/* Valid modes for the reboot request */
#define MB_CONTROL_REBOOT_MODE_RESET          0x01
#define MB_CONTROL_REBOOT_MODE_BOOTLOADER     0x02
#define MB_CONTROL_REBOOT_BLANK_FLASH         0x03

/* Valid masks for the slave mask */
#define MB_CONTROL_SLAVE_MASK_APBE            (1 << 0)

/* Valid modes for the slave power request */
#define MB_CONTROL_SLAVE_POWER_ON             0x01
#define MB_CONTROL_SLAVE_POWER_OFF            0x02
#define MB_CONTROL_SLAVE_POWER_FLASH_MODE     0x03

/* Reserved Values for core version, all others are versions */
#define MB_CONTROL_ROOT_VER_INVALID           0x00
#define MB_CONTROL_ROOT_VER_NOT_APPLICABLE    0xff

#define MB_CONTROL_SUPPORT_GET_ROOT_VER_MAJOR         0x00
#define MB_CONTROL_SUPPORT_GET_ROOT_VER_MINOR         0x02

#define MB_CONTROL_SUPPORT_RTC_SYNC_MAJOR             0x00
#define MB_CONTROL_SUPPORT_RTC_SYNC_MINOR             0x03

/* Version Support Macros */
#define MB_CONTROL_SUPPORTS(mods_dev, name) \
	((mods_dev->mb_ctrl_major > MB_CONTROL_SUPPORT_##name##_MAJOR) || \
	 (mods_dev->mb_ctrl_major == MB_CONTROL_SUPPORT_##name##_MAJOR && \
	  mods_dev->mb_ctrl_minor >= MB_CONTROL_SUPPORT_##name##_MINOR))

/* Control protocol reboot request */
struct mb_control_reboot_request {
	__u8      mode;
} __packed;
/* Control protocol reboot has no response */

/* Control protocol get_ids request has no payload */
struct mb_control_get_ids_response {
	__le32    unipro_mfg_id;
	__le32    unipro_prod_id;
	__le32    ara_vend_id;
	__le32    ara_prod_id;
	__le64    uid_low;
	__le64    uid_high;
	__le32    fw_version;
	__le32    slave_mask;
} __packed;

/* Control protocol [dis]connected request */
struct mb_control_connected_request {
	__le16 cport_id;
} __packed;

struct mb_control_disconnected_request {
	__le16 cport_id;
} __packed;
/* Control protocol [dis]connected response has no payload */

/* Control protocol slave power request */
struct mb_svc_slave_power_ctrl {
	__le32    slave_id;
	__u8      mode;
} __packed;
/* Control protocol slave power response */

/* Control protocol get core version response */
struct mb_control_root_ver_response {
	__u8      version;
} __packed;

/* Control protocol RTC sync request */
struct mb_control_rtc_sync_request {
	__le64    nsec;
} __packed;
/* Control protocol RTC sync has no response */

#endif /* __MODS_PROTOCOLS_H */
