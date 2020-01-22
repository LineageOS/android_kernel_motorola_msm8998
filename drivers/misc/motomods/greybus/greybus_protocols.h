/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
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

#ifndef __GREYBUS_PROTOCOLS_H
#define __GREYBUS_PROTOCOLS_H

/* Fixed IDs for control/svc protocols */

/* Device ID of SVC and AP */
#define GB_DEVICE_ID_SVC			0
#define GB_DEVICE_ID_AP				1
#define GB_DEVICE_ID_MODULES_START		2

/*
 * Bundle/cport for control/svc cport: The same bundle/cport is shared by both
 * CONTROL and SVC protocols for communication between AP and SVC.
 */
#define GB_SVC_BUNDLE_ID			0
#define GB_SVC_CPORT_ID				0
#define GB_CONTROL_BUNDLE_ID			0
#define GB_CONTROL_CPORT_ID			0


/*
 * All operation messages (both requests and responses) begin with
 * a header that encodes the size of the message (header included).
 * This header also contains a unique identifier, that associates a
 * response message with its operation.  The header contains an
 * operation type field, whose interpretation is dependent on what
 * type of protocol is used over the connection.  The high bit
 * (0x80) of the operation type field is used to indicate whether
 * the message is a request (clear) or a response (set).
 *
 * Response messages include an additional result byte, which
 * communicates the result of the corresponding request.  A zero
 * result value means the operation completed successfully.  Any
 * other value indicates an error; in this case, the payload of the
 * response message (if any) is ignored.  The result byte must be
 * zero in the header for a request message.
 *
 * The wire format for all numeric fields in the header is little
 * endian.  Any operation-specific data begins immediately after the
 * header.
 */
struct gb_operation_msg_hdr {
	__le16	size;		/* Size in bytes of header + payload */
	__le16	operation_id;	/* Operation unique id */
	__u8	type;		/* E.g GB_I2C_TYPE_* or GB_GPIO_TYPE_* */
	__u8	result;		/* Result of request (in responses only) */
	__u8	pad[2];		/* must be zero (ignore when read) */
} __packed;


/* Generic request numbers supported by all modules */
#define GB_REQUEST_TYPE_INVALID			0x00
#define GB_REQUEST_TYPE_PROTOCOL_VERSION	0x01

struct gb_protocol_version_request {
	__u8	major;
	__u8	minor;
} __packed;

struct gb_protocol_version_response {
	__u8	major;
	__u8	minor;
} __packed;

/* Control Protocol */

/* Version of the Greybus control protocol we support */
#define GB_CONTROL_VERSION_MAJOR		0x00
#define GB_CONTROL_VERSION_MINOR		0x01

/* Greybus control request types */
#define GB_CONTROL_TYPE_PROBE_AP		0x02
#define GB_CONTROL_TYPE_GET_MANIFEST_SIZE	0x03
#define GB_CONTROL_TYPE_GET_MANIFEST		0x04
#define GB_CONTROL_TYPE_CONNECTED		0x05
#define GB_CONTROL_TYPE_DISCONNECTED		0x06

/* Control protocol manifest get size request has no payload*/
struct gb_control_get_manifest_size_response {
	__le16			size;
} __packed;

/* Control protocol manifest get request has no payload */
struct gb_control_get_manifest_response {
	__u8			data[0];
} __packed;

/* Control protocol [dis]connected request */
struct gb_control_connected_request {
	__le16			cport_id;
} __packed;

struct gb_control_disconnected_request {
	__le16			cport_id;
} __packed;
/* Control protocol [dis]connected response has no payload */


/* Firmware Protocol */

/* Version of the Greybus firmware protocol we support */
#define GB_FIRMWARE_VERSION_MAJOR		0x00
#define GB_FIRMWARE_VERSION_MINOR		0x01

/* Greybus firmware request types */
#define GB_FIRMWARE_TYPE_FIRMWARE_SIZE		0x02
#define GB_FIRMWARE_TYPE_GET_FIRMWARE		0x03
#define GB_FIRMWARE_TYPE_READY_TO_BOOT		0x04
#define GB_FIRMWARE_TYPE_AP_READY		0x05	/* Request with no-payload */
#define GB_FIRMWARE_TYPE_GET_VID_PID		0x06	/* Request with no-payload */

/* Greybus firmware boot stages */
#define GB_FIRMWARE_BOOT_STAGE_ONE		0x01 /* Reserved for the boot ROM */
#define GB_FIRMWARE_BOOT_STAGE_TWO		0x02 /* Firmware package to be loaded by the boot ROM */
#define GB_FIRMWARE_BOOT_STAGE_THREE		0x03 /* Module personality package loaded by Stage 2 firmware */

/* Greybus firmware ready to boot status */
#define GB_FIRMWARE_BOOT_STATUS_INVALID		0x00 /* Firmware blob could not be validated */
#define GB_FIRMWARE_BOOT_STATUS_INSECURE	0x01 /* Firmware blob is valid but insecure */
#define GB_FIRMWARE_BOOT_STATUS_SECURE		0x02 /* Firmware blob is valid and secure */

/* Max firmware data fetch size in bytes */
#define GB_FIRMWARE_FETCH_MAX			0x7F0

/* Firmware protocol firmware size request/response */
struct gb_firmware_size_request {
	__u8			stage;
} __packed;

struct gb_firmware_size_response {
	__le32			size;
} __packed;

/* Firmware protocol get firmware request/response */
struct gb_firmware_get_firmware_request {
	__le32			offset;
	__le32			size;
} __packed;

struct gb_firmware_get_firmware_response {
	__u8			data[0];
} __packed;

/* Firmware protocol Ready to boot request */
struct gb_firmware_ready_to_boot_request {
	__u8			status;
} __packed;
/* Firmware protocol Ready to boot response has no payload */

/* Firmware protocol get VID/PID request has no payload */
struct gb_firmware_get_vid_pid_response {
	__le32			vendor_id;
	__le32			product_id;
} __packed;


/* BATTERY */

/* Version of the Greybus battery protocol we support */
#define	GB_BATTERY_VERSION_MAJOR		0x00
#define	GB_BATTERY_VERSION_MINOR		0x01

/* Greybus battery request types */
#define	GB_BATTERY_TYPE_TECHNOLOGY		0x02
#define	GB_BATTERY_TYPE_STATUS			0x03
#define	GB_BATTERY_TYPE_MAX_VOLTAGE		0x04
#define	GB_BATTERY_TYPE_PERCENT_CAPACITY	0x05
#define	GB_BATTERY_TYPE_TEMPERATURE		0x06
#define	GB_BATTERY_TYPE_VOLTAGE			0x07
#define	GB_BATTERY_TYPE_CURRENT			0x08
#define GB_BATTERY_TYPE_CAPACITY		0x09
#define GB_BATTERY_TYPE_SHUTDOWN_TEMP		0x0a	// TODO - POWER_SUPPLY_PROP_TEMP_ALERT_MAX

/* Should match up with battery types in linux/power_supply.h */
#define GB_BATTERY_TECH_UNKNOWN			0x0000
#define GB_BATTERY_TECH_NiMH			0x0001
#define GB_BATTERY_TECH_LION			0x0002
#define GB_BATTERY_TECH_LIPO			0x0003
#define GB_BATTERY_TECH_LiFe			0x0004
#define GB_BATTERY_TECH_NiCd			0x0005
#define GB_BATTERY_TECH_LiMn			0x0006

struct gb_battery_technology_response {
	__le32	technology;
} __packed;

/* Should match up with battery status in linux/power_supply.h */
#define GB_BATTERY_STATUS_UNKNOWN		0x0000
#define GB_BATTERY_STATUS_CHARGING		0x0001
#define GB_BATTERY_STATUS_DISCHARGING		0x0002
#define GB_BATTERY_STATUS_NOT_CHARGING		0x0003
#define GB_BATTERY_STATUS_FULL			0x0004

struct gb_battery_status_response {
	__le16	battery_status;
} __packed;

struct gb_battery_max_voltage_response {
	__le32	max_voltage;
} __packed;

struct gb_battery_capacity_response {
	__le32	capacity;
} __packed;

struct gb_battery_temperature_response {
	__le32	temperature;
} __packed;

struct gb_battery_voltage_response {
	__le32	voltage;
} __packed;

struct gb_battery_full_capacity_response {
	__le32	full_capacity;
} __packed;

struct gb_battery_current_now_response {
	__le32	current_now;
} __packed;

/* PTP */

/* Version of the Greybus ptp protocol we support */
#define GB_PTP_VERSION_MAJOR		0x00
#define GB_PTP_VERSION_MINOR		0x03

/* Greybus ptp operation types */
#define GB_PTP_TYPE_GET_FUNCTIONALITY		0x02
#define GB_PTP_TYPE_SET_CURRENT_FLOW		0x03
#define GB_PTP_TYPE_SET_MAX_INPUT_CURRENT	0x04
#define GB_PTP_TYPE_EXT_POWER_CHANGED		0x05
#define GB_PTP_TYPE_EXT_POWER_PRESENT		0x06
#define GB_PTP_TYPE_POWER_REQUIRED_CHANGED	0x07
#define GB_PTP_TYPE_POWER_REQUIRED		0x08
#define GB_PTP_TYPE_POWER_AVAILABLE_CHANGED	0x09 /* added in ver 00.02 */
#define GB_PTP_TYPE_POWER_AVAILABLE		0x0A /* added in ver 00.02 */
#define GB_PTP_TYPE_POWER_SOURCE		0x0B /* added in ver 00.02 */
#define GB_PTP_TYPE_GET_MAX_OUTPUT_CURRENT	0x0C /* added in ver 00.02 */
#define GB_PTP_TYPE_GET_CURRENT_FLOW		0x0D /* added in ver 00.03 */
#define GB_PTP_TYPE_SET_MAX_OUTPUT_VOLTAGE	0x0E /* added in ver 00.03 */
#define GB_PTP_TYPE_GET_OUTPUT_VOLTAGE		0x0F /* added in ver 00.03 */
#define GB_PTP_TYPE_GET_MAX_INPUT_VOLTAGE	0x10 /* added in ver 00.03 */
#define GB_PTP_TYPE_SET_INPUT_VOLTAGE		0x11 /* added in ver 00.03 */

/* Check for operation support */
#define GB_PTP_SUPPORTS(p, name) \
	((p->connection->module_major > GB_PTP_SUPPORT_##name##_MAJOR) || \
	(p->connection->module_major == GB_PTP_SUPPORT_##name##_MAJOR && \
	p->connection->module_minor >= GB_PTP_SUPPORT_##name##_MINOR))

/* Operations added in ver 00.02 */
#define GB_PTP_SUPPORT_POWER_AVAILABLE_CHANGED_MAJOR	0x00
#define GB_PTP_SUPPORT_POWER_AVAILABLE_CHANGED_MINOR	0x02
#define GB_PTP_SUPPORT_POWER_AVAILABLE_MAJOR		0x00
#define GB_PTP_SUPPORT_POWER_AVAILABLE_MINOR		0x02
#define GB_PTP_SUPPORT_POWER_SOURCE_MAJOR		0x00
#define GB_PTP_SUPPORT_POWER_SOURCE_MINOR		0x02
#define GB_PTP_SUPPORT_MAX_OUTPUT_CURRENT_MAJOR		0x00
#define GB_PTP_SUPPORT_MAX_OUTPUT_CURRENT_MINOR		0x02

/* Operations added in ver 00.03 */
#define GB_PTP_SUPPORT_GET_CURRENT_FLOW_MAJOR		0x00
#define GB_PTP_SUPPORT_GET_CURRENT_FLOW_MINOR		0x03
#define GB_PTP_SUPPORT_SET_MAX_OUTPUT_VOLTAGE_MAJOR	0x00
#define GB_PTP_SUPPORT_SET_MAX_OUTPUT_VOLTAGE_MINOR	0x03
#define GB_PTP_SUPPORT_GET_OUTPUT_VOLTAGE_MAJOR		0x00
#define GB_PTP_SUPPORT_GET_OUTPUT_VOLTAGE_MINOR		0x03
#define GB_PTP_SUPPORT_GET_MAX_INPUT_VOLTAGE_MAJOR	0x00
#define GB_PTP_SUPPORT_GET_MAX_INPUT_VOLTAGE_MINOR	0x03
#define GB_PTP_SUPPORT_SET_INPUT_VOLTAGE_MAJOR		0x00
#define GB_PTP_SUPPORT_SET_INPUT_VOLTAGE_MINOR		0x03

/* Mod internal source send power capabilities */
#define GB_PTP_INT_SND_NEVER		0x00
#define GB_PTP_INT_SND_SUPPLEMENTAL	0x01
#define GB_PTP_INT_SND_LOW_BATT_SAVER	0x02

/* Mod internal source receive power capabilities */
#define GB_PTP_INT_RCV_NEVER		0x00
#define GB_PTP_INT_RCV_FIRST		0x01
#define GB_PTP_INT_RCV_SECOND		0x02
#define GB_PTP_INT_RCV_PARALLEL		0x03

/* Mod external source capabilities */
#define GB_PTP_EXT_NONE			0x00
#define GB_PTP_EXT_SUPPORTED		0x01

/* Current Flow Request from Phone to Mod */
#define GB_PTP_CURRENT_OFF		0x00
#define GB_PTP_CURRENT_TO_MOD		0x01
#define GB_PTP_CURRENT_FROM_MOD		0x02

/* Mod External Power Presence */
#define GB_PTP_EXT_POWER_NOT_PRESENT		0x00
#define GP_PTP_EXT_POWER_PRESENT		0x01 /* removed in ver 00.02 */
#define GB_PTP_EXT_POWER_WIRELESS_PRESENT	0x02 /* added in ver 00.02 */
#define GB_PTP_EXT_POWER_WIRED_PRESENT		0x03 /* added in ver 00.02 */
#define GB_PTP_EXT_POWER_WIRED_WIRELESS_PRESENT	0x04 /* added in ver 00.02 */

/* Mod internal source power requirements */
#define GB_PTP_POWER_NOT_REQUIRED	0x00
#define GB_PTP_POWER_REQUIRED		0x01

/* Mod power availability for Phone */
#define GB_PTP_POWER_NOT_AVAILABLE	0x00
#define GB_PTP_POWER_AVAILABLE_EXT	0x01
#define GB_PTP_POWER_AVAILABLE_INT	0x02

/* Mod power source supplying current to Phone*/
#define GB_PTP_POWER_SOURCE_NONE		0x00
#define GB_PTP_POWER_SOURCE_BATTERY		0x01
#define GB_PTP_POWER_SOURCE_WIRED		0x02
#define GB_PTP_POWER_SOURCE_WIRELESS		0x03
#define GB_PTP_POWER_SOURCE_NONE_TURBO		0x04
#define GB_PTP_POWER_SOURCE_BATTERY_TURBO	0x05
#define GB_PTP_POWER_SOURCE_WIRED_TURBO		0x06
#define GB_PTP_POWER_SOURCE_WIRELESS_TURBO	0x07

/* Mod that does not support variable voltage charging */
#define GB_PTP_VARIABLE_VOLTAGE_NOT_SUPPORTED	5000000		/* uV */

/* Greybus messages */
struct gb_ptp_functionality_response {
	__u8 int_snd;
	__u8 int_rcv;
	__le32 unused;
	__u8 ext;
} __packed;

struct gb_ptp_max_output_current_response {
	__le32 curr;
} __packed;

struct gb_ptp_ext_power_present_response {
	__u8 present;
} __packed;

struct gb_ptp_power_required_response {
	__u8 required;
} __packed;

struct gb_ptp_power_available_response {
	__u8 available;
} __packed;

struct gb_ptp_power_source_response {
	__u8 source;
} __packed;

struct gb_ptp_current_flow_request {
	__u8 direction;
} __packed;

struct gb_ptp_current_flow_response {
	__u8 direction;
} __packed;

struct gb_ptp_max_input_current_request {
	__le32 curr;
} __packed;

struct gb_ptp_max_output_voltage_request {
	__le32 voltage;
} __packed;

struct gb_ptp_output_voltage_response {
	__le32 voltage;
} __packed;

struct gb_ptp_max_input_voltage_response {
	__le32 voltage;
} __packed;

struct gb_ptp_input_voltage_request {
	__le32 voltage;
} __packed;

/* HID */

/* Version of the Greybus hid protocol we support */
#define GB_HID_VERSION_MAJOR		0x00
#define GB_HID_VERSION_MINOR		0x01

/* Greybus HID operation types */
#define GB_HID_TYPE_GET_DESC		0x02
#define GB_HID_TYPE_GET_REPORT_DESC	0x03
#define GB_HID_TYPE_PWR_ON		0x04
#define GB_HID_TYPE_PWR_OFF		0x05
#define GB_HID_TYPE_GET_REPORT		0x06
#define GB_HID_TYPE_SET_REPORT		0x07
#define GB_HID_TYPE_IRQ_EVENT		0x08

/* Report type */
#define GB_HID_INPUT_REPORT		0
#define GB_HID_OUTPUT_REPORT		1
#define GB_HID_FEATURE_REPORT		2

/* Different request/response structures */
/* HID get descriptor response */
struct gb_hid_desc_response {
	__u8				bLength;
	__le16				wReportDescLength;
	__le16				bcdHID;
	__le16				wProductID;
	__le16				wVendorID;
	__u8				bCountryCode;
} __packed;

/* HID get report request/response */
struct gb_hid_get_report_request {
	__u8				report_type;
	__u8				report_id;
} __packed;

/* HID set report request */
struct gb_hid_set_report_request {
	__u8				report_type;
	__u8				report_id;
	__u8				report[0];
} __packed;

/* HID input report request, via interrupt pipe */
struct gb_hid_input_report_request {
	__u8				report[0];
} __packed;


/* I2C */

/* Version of the Greybus i2c protocol we support */
#define GB_I2C_VERSION_MAJOR		0x00
#define GB_I2C_VERSION_MINOR		0x01

/* Greybus i2c request types */
#define GB_I2C_TYPE_FUNCTIONALITY	0x02
#define GB_I2C_TYPE_TIMEOUT		0x03
#define GB_I2C_TYPE_RETRIES		0x04
#define GB_I2C_TYPE_TRANSFER		0x05

#define GB_I2C_RETRIES_DEFAULT		3
#define GB_I2C_TIMEOUT_DEFAULT		1000	/* milliseconds */

/* functionality request has no payload */
struct gb_i2c_functionality_response {
	__le32	functionality;
} __packed;

struct gb_i2c_timeout_request {
	__le16	msec;
} __packed;
/* timeout response has no payload */

struct gb_i2c_retries_request {
	__u8	retries;
} __packed;
/* retries response has no payload */

/*
 * Outgoing data immediately follows the op count and ops array.
 * The data for each write (master -> slave) op in the array is sent
 * in order, with no (e.g. pad) bytes separating them.
 *
 * Short reads cause the entire transfer request to fail So response
 * payload consists only of bytes read, and the number of bytes is
 * exactly what was specified in the corresponding op.  Like
 * outgoing data, the incoming data is in order and contiguous.
 */
struct gb_i2c_transfer_op {
	__le16	addr;
	__le16	flags;
	__le16	size;
} __packed;

struct gb_i2c_transfer_request {
	__le16				op_count;
	struct gb_i2c_transfer_op	ops[0];		/* op_count of these */
} __packed;
struct gb_i2c_transfer_response {
	__u8				data[0];	/* inbound data */
} __packed;


/* GPIO */

/* Version of the Greybus GPIO protocol we support */
#define GB_GPIO_VERSION_MAJOR		0x00
#define GB_GPIO_VERSION_MINOR		0x01

/* Greybus GPIO request types */
#define GB_GPIO_TYPE_LINE_COUNT		0x02
#define GB_GPIO_TYPE_ACTIVATE		0x03
#define GB_GPIO_TYPE_DEACTIVATE		0x04
#define GB_GPIO_TYPE_GET_DIRECTION	0x05
#define GB_GPIO_TYPE_DIRECTION_IN	0x06
#define GB_GPIO_TYPE_DIRECTION_OUT	0x07
#define GB_GPIO_TYPE_GET_VALUE		0x08
#define GB_GPIO_TYPE_SET_VALUE		0x09
#define GB_GPIO_TYPE_SET_DEBOUNCE	0x0a
#define GB_GPIO_TYPE_IRQ_TYPE		0x0b
#define GB_GPIO_TYPE_IRQ_MASK		0x0c
#define GB_GPIO_TYPE_IRQ_UNMASK		0x0d
#define GB_GPIO_TYPE_IRQ_EVENT		0x0e

#define GB_GPIO_IRQ_TYPE_NONE		0x00
#define GB_GPIO_IRQ_TYPE_EDGE_RISING	0x01
#define GB_GPIO_IRQ_TYPE_EDGE_FALLING	0x02
#define GB_GPIO_IRQ_TYPE_EDGE_BOTH	0x03
#define GB_GPIO_IRQ_TYPE_LEVEL_HIGH	0x04
#define GB_GPIO_IRQ_TYPE_LEVEL_LOW	0x08

/* line count request has no payload */
struct gb_gpio_line_count_response {
	__u8	count;
} __packed;

struct gb_gpio_activate_request {
	__u8	which;
} __packed;
/* activate response has no payload */

struct gb_gpio_deactivate_request {
	__u8	which;
} __packed;
/* deactivate response has no payload */

struct gb_gpio_get_direction_request {
	__u8	which;
} __packed;
struct gb_gpio_get_direction_response {
	__u8	direction;
} __packed;

struct gb_gpio_direction_in_request {
	__u8	which;
} __packed;
/* direction in response has no payload */

struct gb_gpio_direction_out_request {
	__u8	which;
	__u8	value;
} __packed;
/* direction out response has no payload */

struct gb_gpio_get_value_request {
	__u8	which;
} __packed;
struct gb_gpio_get_value_response {
	__u8	value;
} __packed;

struct gb_gpio_set_value_request {
	__u8	which;
	__u8	value;
} __packed;
/* set value response has no payload */

struct gb_gpio_set_debounce_request {
	__u8	which;
	__le16	usec;
} __packed;
/* debounce response has no payload */

struct gb_gpio_irq_type_request {
	__u8	which;
	__u8	type;
} __packed;
/* irq type response has no payload */

struct gb_gpio_irq_mask_request {
	__u8	which;
} __packed;
/* irq mask response has no payload */

struct gb_gpio_irq_unmask_request {
	__u8	which;
} __packed;
/* irq unmask response has no payload */

/* irq event requests originate on another module and are handled on the AP */
struct gb_gpio_irq_event_request {
	__u8	which;
} __packed;
/* irq event has no response */


/* PWM */

/* Version of the Greybus PWM protocol we support */
#define GB_PWM_VERSION_MAJOR		0x00
#define GB_PWM_VERSION_MINOR		0x01

/* Greybus PWM operation types */
#define GB_PWM_TYPE_PWM_COUNT		0x02
#define GB_PWM_TYPE_ACTIVATE		0x03
#define GB_PWM_TYPE_DEACTIVATE		0x04
#define GB_PWM_TYPE_CONFIG		0x05
#define GB_PWM_TYPE_POLARITY		0x06
#define GB_PWM_TYPE_ENABLE		0x07
#define GB_PWM_TYPE_DISABLE		0x08

/* pwm count request has no payload */
struct gb_pwm_count_response {
	__u8	count;
} __packed;

struct gb_pwm_activate_request {
	__u8	which;
} __packed;

struct gb_pwm_deactivate_request {
	__u8	which;
} __packed;

struct gb_pwm_config_request {
	__u8	which;
	__le32	duty;
	__le32	period;
} __packed;

struct gb_pwm_polarity_request {
	__u8	which;
	__u8	polarity;
} __packed;

struct gb_pwm_enable_request {
	__u8	which;
} __packed;

struct gb_pwm_disable_request {
	__u8	which;
} __packed;

/* I2S */
#define GB_I2S_MGMT_VERSION_MAJOR 0
#define GB_I2S_MGMT_VERSION_MINOR 3

#define GB_I2S_MGMT_VERSION_CFG_MASK_MAJOR 0
#define GB_I2S_MGMT_VERSION_CFG_MASK_MINOR 2

#define GB_I2S_MGMT_VERSION_START_MSG_MAJOR 0
#define GB_I2S_MGMT_VERSION_START_MSG_MINOR 3

#define GB_I2S_MGMT_TYPE_PROTOCOL_VERSION               0x01
#define GB_I2S_MGMT_TYPE_GET_SUPPORTED_CONFIGURATIONS	0x02
#define GB_I2S_MGMT_TYPE_SET_CONFIGURATION		0x03
#define GB_I2S_MGMT_TYPE_SET_SAMPLES_PER_MESSAGE	0x04
#define GB_I2S_MGMT_TYPE_GET_PROCESSING_DELAY		0x05
#define GB_I2S_MGMT_TYPE_SET_START_DELAY		0x06
#define GB_I2S_MGMT_TYPE_ACTIVATE_CPORT			0x07
#define GB_I2S_MGMT_TYPE_DEACTIVATE_CPORT		0x08
#define GB_I2S_MGMT_TYPE_REPORT_EVENT			0x09
#define GB_I2S_MGMT_TYPE_ACTIVATE_PORT		0x0a
#define GB_I2S_MGMT_TYPE_DEACTIVATE_PORT		0x0b
#define GB_I2S_MGMT_TYPE_START			0x0c
#define GB_I2S_MGMT_TYPE_STOP			0x0d

#define GB_I2S_MGMT_BYTE_ORDER_NA			BIT(0)
#define GB_I2S_MGMT_BYTE_ORDER_BE			BIT(1)
#define GB_I2S_MGMT_BYTE_ORDER_LE			BIT(2)

#define GB_I2S_MGMT_SPATIAL_LOCATION_FL			BIT(0)
#define GB_I2S_MGMT_SPATIAL_LOCATION_FR			BIT(1)
#define GB_I2S_MGMT_SPATIAL_LOCATION_FC			BIT(2)
#define GB_I2S_MGMT_SPATIAL_LOCATION_LFE		BIT(3)
#define GB_I2S_MGMT_SPATIAL_LOCATION_BL			BIT(4)
#define GB_I2S_MGMT_SPATIAL_LOCATION_BR			BIT(5)
#define GB_I2S_MGMT_SPATIAL_LOCATION_FLC		BIT(6)
#define GB_I2S_MGMT_SPATIAL_LOCATION_FRC		BIT(7)
#define GB_I2S_MGMT_SPATIAL_LOCATION_C			BIT(8) /* BC in USB */
#define GB_I2S_MGMT_SPATIAL_LOCATION_SL			BIT(9)
#define GB_I2S_MGMT_SPATIAL_LOCATION_SR			BIT(10)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TC			BIT(11)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TFL		BIT(12)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TFC		BIT(13)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TFR		BIT(14)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TBL		BIT(15)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TBC		BIT(16)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TBR		BIT(17)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TFLC		BIT(18)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TFRC		BIT(19)
#define GB_I2S_MGMT_SPATIAL_LOCATION_LLFE		BIT(20)
#define GB_I2S_MGMT_SPATIAL_LOCATION_RLFE		BIT(21)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TSL		BIT(22)
#define GB_I2S_MGMT_SPATIAL_LOCATION_TSR		BIT(23)
#define GB_I2S_MGMT_SPATIAL_LOCATION_BC			BIT(24)
#define GB_I2S_MGMT_SPATIAL_LOCATION_BLC		BIT(25)
#define GB_I2S_MGMT_SPATIAL_LOCATION_BRC		BIT(26)
#define GB_I2S_MGMT_SPATIAL_LOCATION_RD			BIT(31)

#define GB_I2S_MGMT_PCM_FMT_8                    BIT(0)
#define GB_I2S_MGMT_PCM_FMT_16                   BIT(1)
#define GB_I2S_MGMT_PCM_FMT_24                   BIT(2)
#define GB_I2S_MGMT_PCM_FMT_32                   BIT(3)
#define GB_I2S_MGMT_PCM_FMT_64                   BIT(4)

#define GB_I2S_MGMT_PCM_RATE_5512                BIT(0)
#define GB_I2S_MGMT_PCM_RATE_8000                BIT(1)
#define GB_I2S_MGMT_PCM_RATE_11025               BIT(2)
#define GB_I2S_MGMT_PCM_RATE_16000               BIT(3)
#define GB_I2S_MGMT_PCM_RATE_22050               BIT(4)
#define GB_I2S_MGMT_PCM_RATE_32000               BIT(5)
#define GB_I2S_MGMT_PCM_RATE_44100               BIT(6)
#define GB_I2S_MGMT_PCM_RATE_48000               BIT(7)
#define GB_I2S_MGMT_PCM_RATE_64000               BIT(8)
#define GB_I2S_MGMT_PCM_RATE_88200               BIT(9)
#define GB_I2S_MGMT_PCM_RATE_96000               BIT(10)
#define GB_I2S_MGMT_PCM_RATE_176400              BIT(11)
#define GB_I2S_MGMT_PCM_RATE_192000              BIT(12)

#define GB_I2S_MGMT_PROTOCOL_PCM			BIT(0)
#define GB_I2S_MGMT_PROTOCOL_I2S			BIT(1)
#define GB_I2S_MGMT_PROTOCOL_LR_STEREO			BIT(2)

#define GB_I2S_MGMT_ROLE_MASTER				BIT(0)
#define GB_I2S_MGMT_ROLE_SLAVE				BIT(1)

#define GB_I2S_MGMT_POLARITY_NORMAL			BIT(0)
#define GB_I2S_MGMT_POLARITY_REVERSED			BIT(1)

#define GB_I2S_MGMT_EDGE_RISING				BIT(0)
#define GB_I2S_MGMT_EDGE_FALLING			BIT(1)

#define GB_I2S_MGMT_EVENT_UNSPECIFIED			0x1
#define GB_I2S_MGMT_EVENT_HALT				0x2
#define GB_I2S_MGMT_EVENT_INTERNAL_ERROR		0x3
#define GB_I2S_MGMT_EVENT_PROTOCOL_ERROR		0x4
#define GB_I2S_MGMT_EVENT_FAILURE			0x5
#define GB_I2S_MGMT_EVENT_OUT_OF_SEQUENCE		0x6
#define GB_I2S_MGMT_EVENT_UNDERRUN			0x7
#define GB_I2S_MGMT_EVENT_OVERRUN			0x8
#define GB_I2S_MGMT_EVENT_CLOCKING			0x9
#define GB_I2S_MGMT_EVENT_DATA_LEN			0xa

#define GB_I2S_MGMT_PORT_TYPE_RECEIVER		0x1
#define GB_I2S_MGMT_PORT_TYPE_TRANSMITTER	0x2

struct gb_i2s_mgmt_configuration {
	__le32	sample_frequency;
	__u8	num_channels;
	__u8	bytes_per_channel;
	__u8	byte_order;
	__u8	pad;
	__le32	spatial_locations;
	__le32	ll_protocol;
	__u8	ll_mclk_role;
	__u8	ll_bclk_role;
	__u8	ll_wclk_role;
	__u8	ll_wclk_polarity;
	__u8	ll_wclk_change_edge;
	__u8	ll_wclk_tx_edge;
	__u8	ll_wclk_rx_edge;
	__u8	ll_data_offset;
} __packed;

struct gb_i2s_mgmt_config_masks {
	__le32	sample_frequency;
	__u8	num_channels;
	__le32	format;
	__u8	protocol;
	__u8	wclk_polarity;
	__u8	wclk_change_edge;
	__u8	wclk_tx_edge;
	__u8	wclk_rx_edge;
} __packed;

/* get supported configurations request has no payload */
struct gb_i2s_mgmt_get_supported_configurations_response {
	__u8					config_count;
	__u8					pad[3];
	struct gb_i2s_mgmt_configuration	config[0];
} __packed;

struct gb_i2s_mgmt_set_configuration_request {
	struct gb_i2s_mgmt_configuration	config;
} __packed;
/* set configuration response has no payload */

struct gb_i2s_mgmt_set_config_masks_request {
	struct gb_i2s_mgmt_config_masks	config;
} __packed;

struct gb_i2s_mgmt_get_config_masks_response {
	struct gb_i2s_mgmt_config_masks config;
};

struct gb_i2s_mgmt_set_samples_per_message_request {
	__le16	samples_per_message;
} __packed;
/* set samples per message response has no payload */

/* get processing request delay has no payload */
struct gb_i2s_mgmt_get_processing_delay_response {
	__le32	microseconds;
} __packed;

struct gb_i2s_mgmt_set_start_delay_request {
	__le32	microseconds;
} __packed;
/* set start delay response has no payload */

struct gb_i2s_mgmt_activate_cport_request {
	__le16	cport;
} __packed;
/* activate cport response has no payload */

struct gb_i2s_mgmt_deactivate_cport_request {
	__le16	cport;
} __packed;
/* deactivate cport response has no payload */

struct gb_i2s_mgmt_report_event_request {
	__u8	event;
} __packed;
/* report event response has no payload */

struct gb_i2s_mgmt_activate_port_request {
	__u8	port_type;
} __packed;
/* activate port response has no payload */

struct gb_i2s_mgmt_deactivate_port_request {
	__u8	port_type;
} __packed;
/* deactivate port response has no payload */

struct gb_i2s_mgmt_start_request {
    __u8    port_type;
} __packed;
/* start response has no payload */

struct gb_i2s_mgmt_stop_request {
    __u8    port_type;
} __packed;
/* stop response has no payload */

/* Mods Audio protocol*/

#define GB_MODS_AUDIO_VERSION_MAJOR 0
#define GB_MODS_AUDIO_VERSION_MINOR 3

#define GB_MODS_AUDIO_VERSION_SPKR_PRESET_MAJOR 0
#define GB_MODS_AUDIO_VERSION_SPKR_PRESET_MINOR 2

#define GB_MODS_AUDIO_VERSION_BF_PARAMS_MAJOR 0
#define GB_MODS_AUDIO_VERSION_BF_PARAMS_MINOR 3

/* Commands */
#define GB_AUDIO_GET_VOLUME_DB_RANGE		0x02
#define GB_AUDIO_GET_SUPPORTED_USE_CASES 		0x03
#define GB_AUDIO_SET_CAPTURE_USE_CASE		0x04
#define GB_AUDIO_SET_PLAYBACK_USE_CASE		0x05
#define GB_AUDIO_SET_VOLUME		0x06
#define GB_AUDIO_SET_SYSTEM_VOLUME		0x07
#define GB_AUDIO_GET_SUPPORTED_DEVICES		0x08
#define GB_AUDIO_DEVICES_REPORT_EVENT		0x09
#define GB_AUDIO_ENABLE_DEVICES		0x0a
#define GB_AUDIO_GET_SPEAKER_PRESET_EQ		0x0b
#define GB_AUDIO_GET_MIC_PARAMS		0x0c

/* Playback use cases bit mask*/

#define GB_AUDIO_PLAYBACK_NONE_USE_CASE		BIT(0)
/* Music */
#define GB_AUDIO_PLAYBACK_MUSIC_USE_CASE		BIT(1)
/* Voice call */
#define GB_AUDIO_PLAYBACK_VOICE_CALL_SPKR_USE_CASE	BIT(2)
/* Ringer  */
#define GB_AUDIO_PLAYBACK_RINGTONE_USE_CASE		BIT(3)
/* System sounds */
#define GB_AUDIO_PLAYBACK_SONIFICATION_USE_CASE		BIT(4)

/* Capture use cases bit mask*/

/* Default setting capture */
#define GB_AUDIO_CAPTURE_DEFAULT_USE_CASE	BIT(0)
/* Optimized for Voice  */
#define GB_AUDIO_CAPTURE_VOICE_USE_CASE		BIT(1)
/* Unprocessed pcm capture */
#define GB_AUDIO_CAPTURE_RAW_USE_CASE		BIT(2)
/* Optimized for Camcorder */
#define GB_AUDIO_CAPTURE_CAMCORDER_USE_CASE	BIT(3)
/* Ambisonic WXYZ 4-channel Capture */
#define GB_AUDIO_CAPTURE_AMBISONIC_USE_CASE	BIT(4)
/* Optimized for voice recognition */
#define GB_AUDIO_CAPTURE_VOICE_REC_USE_CASE	BIT(5)

/* audio output devices bit mask */
#define GB_AUDIO_DEVICE_OUT_LOUDSPEAKER BIT(0)
#define GB_AUDIO_DEVICE_OUT_HEADSET     BIT(1)
#define GB_AUDIO_DEVICE_OUT_LINE        BIT(2)
#define GB_AUDIO_DEVICE_OUT_HDMI        BIT(3)
#define GB_AUDIO_DEVICE_OUT_EARPIECE    BIT(4)

/* audio input devices bit mask */
#define GB_AUDIO_DEVICE_IN_MIC                   BIT(0)
#define GB_AUDIO_DEVICE_IN_HEADSET_MIC           BIT(1)
#define GB_AUDIO_DEVICE_IN_CAMCORDER_MIC         BIT(2)
#define GB_AUDIO_DEVICE_IN_EC_REF                BIT(3)
#define GB_AUDIO_DEVICE_IN_FM_TUNER              BIT(4)
#define GB_AUDIO_DEVICE_IN_MIC_EC                BIT(5)
#define GB_AUDIO_DEVICE_IN_MIC_ECNS              BIT(6)
#define GB_AUDIO_DEVICE_IN_MIC_NS                BIT(7)

/* speaker eq preset*/
#define GB_AUDIO_SPEAKER_PRESET_EQ_NONE                 0
 /* EQ and Bass Enhancement optimized to boost an approximate
  * frequency range of 400Hz - 800Hz.
  */
#define GB_AUDIO_SPEAKER_PRESET_EQ_SMALL                1
/* EQ and Bass Enhancement optimized to boost an approximate
 * frequency range of 150Hz - 300Hz.
 */
#define GB_AUDIO_SPEAKER_PRESET_EQ_MEDIUM               2
 /* EQ and Bass Enhancement optimized to boost a frequency range
  * below 150Hz.
  */
#define GB_AUDIO_SPEAKER_PRESET_EQ_LARGE                3

/* Tuning Parameters for microphone capture */
#define GB_AUDIO_MIC_PARAMS_SIZE                        1996

/* version request has no payload */
struct gb_audio_proto_version_response {
	__u8    major;
	__u8    minor;
} __packed;


struct gb_aud_usecases {
	__le32 playback_usecases;
	__le32 capture_usecases;
} __packed;

/* Audio bundle will indicate supported use cases for
 * capture and playback. Host will make routing decisions,
 * for capture based on the supported use cases.
*/

struct gb_audio_get_supported_usecases_response {
	struct gb_aud_usecases aud_use_cases;
} __packed;

struct gb_aud_vol_range {
	__le32 min;
	__le32 step;
} __packed;

struct gb_aud_devices {
	__le32 in_devices;
	__le32 out_devices;
} __packed;

/* get volume range min, max and volume step in DB */
struct gb_audio_get_volume_db_range_response {
	  struct gb_aud_vol_range vol_range;
} __packed;

/* get current volume step of the audio device */
struct gb_audio_get_volume_step_response {
	__le32                  vol_step;
} __packed;

/* Set current playback/capture use case.
 * can set unsupported use cases too, its up
 * to the mod to use the info or not to do
 * use case specific tuning.
 */
struct gb_audio_set_use_case_request {
	__le32                    use_case;
} __packed;

/* set volume */
struct gb_audio_set_volume_db_request {
	__le32                  vol_step;
} __packed;

/* set system volume peak db */
struct gb_audio_set_system_volume_db_request {
	__le32                  vol_db;
} __packed;

/* get available audio output devices */
struct gb_audio_get_devices_response {
	struct gb_aud_devices   devices;
} __packed;

/* enable audio devices */
struct gb_audio_enable_devices_request {
	struct gb_aud_devices   devices;
} __packed;

/* report available audio devices */
struct gb_audio_report_devices_request {
	struct gb_aud_devices   devices;
} __packed;

/* get eq preset for speaker */
struct gb_audio_get_speaker_preset_eq_response {
	__le32                  preset_eq;
};

/* get mic tuning parameters */
struct gb_audio_get_mic_params_response {
	u8  params[GB_AUDIO_MIC_PARAMS_SIZE];
};

/* SPI */

/* Version of the Greybus spi protocol we support */
#define GB_SPI_VERSION_MAJOR		0x00
#define GB_SPI_VERSION_MINOR		0x01

/* Should match up with modes in linux/spi/spi.h */
#define GB_SPI_MODE_CPHA		0x01		/* clock phase */
#define GB_SPI_MODE_CPOL		0x02		/* clock polarity */
#define GB_SPI_MODE_MODE_0		(0|0)		/* (original MicroWire) */
#define GB_SPI_MODE_MODE_1		(0|GB_SPI_MODE_CPHA)
#define GB_SPI_MODE_MODE_2		(GB_SPI_MODE_CPOL|0)
#define GB_SPI_MODE_MODE_3		(GB_SPI_MODE_CPOL|GB_SPI_MODE_CPHA)
#define GB_SPI_MODE_CS_HIGH		0x04		/* chipselect active high? */
#define GB_SPI_MODE_LSB_FIRST		0x08		/* per-word bits-on-wire */
#define GB_SPI_MODE_3WIRE		0x10		/* SI/SO signals shared */
#define GB_SPI_MODE_LOOP		0x20		/* loopback mode */
#define GB_SPI_MODE_NO_CS		0x40		/* 1 dev/bus, no chipselect */
#define GB_SPI_MODE_READY		0x80		/* slave pulls low to pause */

/* Should match up with flags in linux/spi/spi.h */
#define GB_SPI_FLAG_HALF_DUPLEX		BIT(0)		/* can't do full duplex */
#define GB_SPI_FLAG_NO_RX		BIT(1)		/* can't do buffer read */
#define GB_SPI_FLAG_NO_TX		BIT(2)		/* can't do buffer write */

/* Greybus spi operation types */
#define GB_SPI_TYPE_MASTER_CONFIG	0x02
#define GB_SPI_TYPE_DEVICE_CONFIG	0x03
#define GB_SPI_TYPE_TRANSFER		0x04

/* mode request has no payload */
struct gb_spi_master_config_response {
	__le32	bits_per_word_mask;
	__le32	min_speed_hz;
	__le32	max_speed_hz;
	__le16	mode;
	__le16	flags;
	__u8	num_chipselect;
} __packed;

struct gb_spi_device_config_request {
	__u8	chip_select;
} __packed;

struct gb_spi_device_config_response {
	__le16	mode;
	__u8	bits_per_word;
	__le32	max_speed_hz;
	__u8	name[32];
} __packed;

/**
 * struct gb_spi_transfer - a read/write buffer pair
 * @speed_hz: Select a speed other than the device default for this transfer. If
 *	0 the default (from @spi_device) is used.
 * @len: size of rx and tx buffers (in bytes)
 * @delay_usecs: microseconds to delay after this transfer before (optionally)
 * 	changing the chipselect status, then starting the next transfer or
 * 	completing this spi_message.
 * @cs_change: affects chipselect after this transfer completes
 * @bits_per_word: select a bits_per_word other than the device default for this
 *	transfer. If 0 the default (from @spi_device) is used.
 */
struct gb_spi_transfer {
	__le32		speed_hz;
	__le32		len;
	__le16		delay_usecs;
	__u8		cs_change;
	__u8		bits_per_word;
	__u8		rdwr;
#define GB_SPI_XFER_READ	0x01
#define GB_SPI_XFER_WRITE	0x02
} __packed;

struct gb_spi_transfer_request {
	__u8			chip_select;	/* of the spi device */
	__u8			mode;		/* of the spi device */
	__le16			count;
	struct gb_spi_transfer	transfers[0];	/* count of these */
} __packed;

struct gb_spi_transfer_response {
	__u8			data[0];	/* inbound data */
} __packed;

/* Version of the Greybus SVC protocol we support */
#define GB_SVC_VERSION_MAJOR		0x00
#define GB_SVC_VERSION_MINOR		0x01

/* Greybus SVC request types */
#define GB_SVC_TYPE_SVC_HELLO		0x02
#define GB_SVC_TYPE_INTF_DEVICE_ID	0x03
#define GB_SVC_TYPE_INTF_HOTPLUG	0x04
#define GB_SVC_TYPE_INTF_HOT_UNPLUG	0x05
#define GB_SVC_TYPE_INTF_RESET		0x06
#define GB_SVC_TYPE_CONN_CREATE		0x07
#define GB_SVC_TYPE_CONN_DESTROY	0x08
#define GB_SVC_TYPE_DME_PEER_GET	0x09
#define GB_SVC_TYPE_DME_PEER_SET	0x0a
#define GB_SVC_TYPE_ROUTE_CREATE	0x0b
#define GB_SVC_TYPE_ROUTE_DESTROY	0x0c

/*
 * SVC version request/response has the same payload as
 * gb_protocol_version_request/response.
 */

/* SVC protocol hello request */
struct gb_svc_hello_request {
	__le16			endo_id;
	__u8			interface_id;
} __packed;
/* hello response has no payload */

struct gb_svc_intf_device_id_request {
	__u8	intf_id;
	__u8	device_id;
} __packed;
/* device id response has no payload */

struct gb_svc_intf_hotplug_request {
	__u8	intf_id;
	struct {
		__le32	unipro_mfg_id;
		__le32	unipro_prod_id;
		__le32	ara_vend_id;
		__le32	ara_prod_id;
	} data;
} __packed;
/* hotplug response has no payload */

struct gb_svc_intf_hot_unplug_request {
	__u8	intf_id;
	__u8	attach_state;
} __packed;
/* hot unplug response has no payload */

struct gb_svc_intf_reset_request {
	__u8	intf_id;
} __packed;
/* interface reset response has no payload */

struct gb_svc_conn_create_request {
	__u8	intf1_id;
	__le16	cport1_id;
	__u8	intf2_id;
	__le16	cport2_id;
	__u8	tc;
	__u8	flags;
} __packed;
/* connection create response has no payload */

struct gb_svc_conn_destroy_request {
	__u8	intf1_id;
	__le16	cport1_id;
	__u8	intf2_id;
	__le16	cport2_id;
} __packed;
/* connection destroy response has no payload */

struct gb_svc_dme_peer_get_request {
	__u8	intf_id;
	__le16	attr;
	__le16	selector;
} __packed;

struct gb_svc_dme_peer_get_response {
	__le16	result_code;
	__le32	attr_value;
} __packed;

struct gb_svc_dme_peer_set_request {
	__u8	intf_id;
	__le16	attr;
	__le16	selector;
	__le32	value;
} __packed;

struct gb_svc_dme_peer_set_response {
	__le16	result_code;
} __packed;

/* Attributes for peer get/set operations */
#define DME_ATTR_SELECTOR_INDEX		0
#define DME_ATTR_T_TST_SRC_INCREMENT	0x4083

/* Return value from TST_SRC_INCREMENT */
#define DME_TSI_SPI_BOOT_STARTED		0x02
#define DME_TSI_TRUSTED_SPI_BOOT_FINISHED	0x03
#define DME_TSI_UNTRUSTED_SPI_BOOT_FINISHED	0x04
#define DME_TSI_UNIPRO_BOOT_STARTED		0x06
#define DME_TSI_FALLBACK_UNIPRO_BOOT_STARTED	0x09

struct gb_svc_route_create_request {
	__u8	intf1_id;
	__u8	dev1_id;
	__u8	intf2_id;
	__u8	dev2_id;
} __packed;
/* route create response has no payload */

struct gb_svc_route_destroy_request {
	__u8	intf1_id;
	__u8	intf2_id;
} __packed;
/* route destroy response has no payload */


/* RAW */

/* Version of the Greybus raw protocol we support */
#define	GB_RAW_VERSION_MAJOR			0x00
#define	GB_RAW_VERSION_MINOR			0x01

/* Greybus raw request types */
#define	GB_RAW_TYPE_SEND			0x02

struct gb_raw_send_request {
	__le32	len;
	__u8	data[0];
} __packed;


/* UART */

/* Version of the Greybus UART protocol we support */
#define GB_UART_VERSION_MAJOR		0x00
#define GB_UART_VERSION_MINOR		0x01

/* Greybus UART operation types */
#define GB_UART_TYPE_SEND_DATA			0x02
#define GB_UART_TYPE_RECEIVE_DATA		0x03	/* Unsolicited data */
#define GB_UART_TYPE_SET_LINE_CODING		0x04
#define GB_UART_TYPE_SET_CONTROL_LINE_STATE	0x05
#define GB_UART_TYPE_SEND_BREAK			0x06
#define GB_UART_TYPE_SERIAL_STATE		0x07	/* Unsolicited data */

/* Represents data from AP -> Module */
struct gb_uart_send_data_request {
	__le16	size;
	__u8	data[0];
} __packed;

/* recv-data-request flags */
#define GB_UART_RECV_FLAG_FRAMING		0x01	/* Framing error */
#define GB_UART_RECV_FLAG_PARITY		0x02	/* Parity error */
#define GB_UART_RECV_FLAG_OVERRUN		0x04	/* Overrun error */
#define GB_UART_RECV_FLAG_BREAK			0x08	/* Break */

/* Represents data from Module -> AP */
struct gb_uart_recv_data_request {
	__le16	size;
	__u8	flags;
	__u8	data[0];
} __packed;

struct gb_uart_set_line_coding_request {
	__le32	rate;
	__u8	format;
#define GB_SERIAL_1_STOP_BITS			0
#define GB_SERIAL_1_5_STOP_BITS			1
#define GB_SERIAL_2_STOP_BITS			2

	__u8	parity;
#define GB_SERIAL_NO_PARITY			0
#define GB_SERIAL_ODD_PARITY			1
#define GB_SERIAL_EVEN_PARITY			2
#define GB_SERIAL_MARK_PARITY			3
#define GB_SERIAL_SPACE_PARITY			4

	__u8	data_bits;
} __packed;

/* output control lines */
#define GB_UART_CTRL_DTR			0x01
#define GB_UART_CTRL_RTS			0x02

struct gb_uart_set_control_line_state_request {
	__u8	control;
} __packed;

struct gb_uart_set_break_request {
	__u8	state;
} __packed;

/* input control lines and line errors */
#define GB_UART_CTRL_DCD			0x01
#define GB_UART_CTRL_DSR			0x02
#define GB_UART_CTRL_RI				0x04

struct gb_uart_serial_state_request {
	__u8	control;
} __packed;

/* Loopback */

/* Version of the Greybus loopback protocol we support */
#define GB_LOOPBACK_VERSION_MAJOR		0x00
#define GB_LOOPBACK_VERSION_MINOR		0x01

/* Greybus loopback request types */
#define GB_LOOPBACK_TYPE_PING			0x02
#define GB_LOOPBACK_TYPE_TRANSFER		0x03
#define GB_LOOPBACK_TYPE_SINK			0x04

struct gb_loopback_transfer_request {
	__le32	len;
	__u8	data[0];
} __packed;

struct gb_loopback_transfer_response {
	__le32	len;
	__le32	reserved0;
	__le32	reserved1;
	__u8	data[0];
} __packed;

/* SDIO */
/* Version of the Greybus sdio protocol we support */
#define GB_SDIO_VERSION_MAJOR		0x00
#define GB_SDIO_VERSION_MINOR		0x01

/* Greybus SDIO operation types */
#define GB_SDIO_TYPE_GET_CAPABILITIES		0x02
#define GB_SDIO_TYPE_SET_IOS			0x03
#define GB_SDIO_TYPE_COMMAND			0x04
#define GB_SDIO_TYPE_TRANSFER			0x05
#define GB_SDIO_TYPE_EVENT			0x06

/* get caps response: request has no payload */
struct gb_sdio_get_caps_response {
	__le32	caps;
#define GB_SDIO_CAP_NONREMOVABLE	0x00000001
#define GB_SDIO_CAP_4_BIT_DATA		0x00000002
#define GB_SDIO_CAP_8_BIT_DATA		0x00000004
#define GB_SDIO_CAP_MMC_HS		0x00000008
#define GB_SDIO_CAP_SD_HS		0x00000010
#define GB_SDIO_CAP_ERASE		0x00000020
#define GB_SDIO_CAP_1_2V_DDR		0x00000040
#define GB_SDIO_CAP_1_8V_DDR		0x00000080
#define GB_SDIO_CAP_POWER_OFF_CARD	0x00000100
#define GB_SDIO_CAP_UHS_SDR12		0x00000200
#define GB_SDIO_CAP_UHS_SDR25		0x00000400
#define GB_SDIO_CAP_UHS_SDR50		0x00000800
#define GB_SDIO_CAP_UHS_SDR104		0x00001000
#define GB_SDIO_CAP_UHS_DDR50		0x00002000
#define GB_SDIO_CAP_DRIVER_TYPE_A	0x00004000
#define GB_SDIO_CAP_DRIVER_TYPE_C	0x00008000
#define GB_SDIO_CAP_DRIVER_TYPE_D	0x00010000
#define GB_SDIO_CAP_HS200_1_2V		0x00020000
#define GB_SDIO_CAP_HS200_1_8V		0x00040000
#define GB_SDIO_CAP_HS400_1_2V		0x00080000
#define GB_SDIO_CAP_HS400_1_8V		0x00100000

	/* see possible values below at vdd */
	__le32 ocr;
	__le16 max_blk_count;
	__le16 max_blk_size;
	__le32 f_min;
	__le32 f_max;
} __packed;

/* set ios request: response has no payload */
struct gb_sdio_set_ios_request {
	__le32	clock;
	__le32	vdd;
#define GB_SDIO_VDD_165_195	0x00000001
#define GB_SDIO_VDD_20_21	0x00000002
#define GB_SDIO_VDD_21_22	0x00000004
#define GB_SDIO_VDD_22_23	0x00000008
#define GB_SDIO_VDD_23_24	0x00000010
#define GB_SDIO_VDD_24_25	0x00000020
#define GB_SDIO_VDD_25_26	0x00000040
#define GB_SDIO_VDD_26_27	0x00000080
#define GB_SDIO_VDD_27_28	0x00000100
#define GB_SDIO_VDD_28_29	0x00000200
#define GB_SDIO_VDD_29_30	0x00000400
#define GB_SDIO_VDD_30_31	0x00000800
#define GB_SDIO_VDD_31_32	0x00001000
#define GB_SDIO_VDD_32_33	0x00002000
#define GB_SDIO_VDD_33_34	0x00004000
#define GB_SDIO_VDD_34_35	0x00008000
#define GB_SDIO_VDD_35_36	0x00010000

	__u8	bus_mode;
#define GB_SDIO_BUSMODE_OPENDRAIN	0x00
#define GB_SDIO_BUSMODE_PUSHPULL	0x01

	__u8	power_mode;
#define GB_SDIO_POWER_OFF	0x00
#define GB_SDIO_POWER_UP	0x01
#define GB_SDIO_POWER_ON	0x02
#define GB_SDIO_POWER_UNDEFINED	0x03

	__u8	bus_width;
#define GB_SDIO_BUS_WIDTH_1	0x00
#define GB_SDIO_BUS_WIDTH_4	0x02
#define GB_SDIO_BUS_WIDTH_8	0x03

	__u8	timing;
#define GB_SDIO_TIMING_LEGACY		0x00
#define GB_SDIO_TIMING_MMC_HS		0x01
#define GB_SDIO_TIMING_SD_HS		0x02
#define GB_SDIO_TIMING_UHS_SDR12	0x03
#define GB_SDIO_TIMING_UHS_SDR25	0x04
#define GB_SDIO_TIMING_UHS_SDR50	0x05
#define GB_SDIO_TIMING_UHS_SDR104	0x06
#define GB_SDIO_TIMING_UHS_DDR50	0x07
#define GB_SDIO_TIMING_MMC_DDR52	0x08
#define GB_SDIO_TIMING_MMC_HS200	0x09
#define GB_SDIO_TIMING_MMC_HS400	0x0A

	__u8	signal_voltage;
#define GB_SDIO_SIGNAL_VOLTAGE_330	0x00
#define GB_SDIO_SIGNAL_VOLTAGE_180	0x01
#define GB_SDIO_SIGNAL_VOLTAGE_120	0x02

	__u8	drv_type;
#define GB_SDIO_SET_DRIVER_TYPE_B	0x00
#define GB_SDIO_SET_DRIVER_TYPE_A	0x01
#define GB_SDIO_SET_DRIVER_TYPE_C	0x02
#define GB_SDIO_SET_DRIVER_TYPE_D	0x03
} __packed;

/* command request */
struct gb_sdio_command_request {
	__u8	cmd;
	__u8	cmd_flags;
#define GB_SDIO_RSP_NONE		0x00
#define GB_SDIO_RSP_PRESENT		0x01
#define GB_SDIO_RSP_136			0x02
#define GB_SDIO_RSP_CRC			0x04
#define GB_SDIO_RSP_BUSY		0x08
#define GB_SDIO_RSP_OPCODE		0x10

	__u8	cmd_type;
#define GB_SDIO_CMD_AC		0x00
#define GB_SDIO_CMD_ADTC	0x01
#define GB_SDIO_CMD_BC		0x02
#define GB_SDIO_CMD_BCR		0x03

	__le32	cmd_arg;
	__le16	data_blocks;
	__le16	data_blksz;
} __packed;

struct gb_sdio_command_response {
	__le32	resp[4];
} __packed;

/* transfer request */
struct gb_sdio_transfer_request {
	__u8	data_flags;
#define GB_SDIO_DATA_WRITE	0x01
#define GB_SDIO_DATA_READ	0x02
#define GB_SDIO_DATA_STREAM	0x04

	__le16	data_blocks;
	__le16	data_blksz;
	__u8	data[0];
} __packed;

struct gb_sdio_transfer_response {
	__le16	data_blocks;
	__le16	data_blksz;
	__u8	data[0];
} __packed;

/* event request: generated by module and is defined as unidirectional */
struct gb_sdio_event_request {
	__u8	event;
#define GB_SDIO_CARD_INSERTED	0x01
#define GB_SDIO_CARD_REMOVED	0x02
#define GB_SDIO_WP		0x04
} __packed;

/* Camera */

#define GB_CAMERA_VERSION_MAJOR			0x00
#define GB_CAMERA_VERSION_MINOR			0x01

/* Greybus Camera request types */
#define GB_CAMERA_TYPE_CAPABILITIES		0x02
#define GB_CAMERA_TYPE_CONFIGURE_STREAMS	0x03
#define GB_CAMERA_TYPE_CAPTURE			0x04
#define GB_CAMERA_TYPE_FLUSH			0x05
#define GB_CAMERA_TYPE_METADATA			0x06

#define GB_CAMERA_MAX_STREAMS			4
#define GB_CAMERA_MAX_SETTINGS_SIZE		8192

/* Greybus Camera Configure Streams request payload */
struct gb_camera_stream_config_request {
	__le16 width;
	__le16 height;
	__le16 format;
	__le16 padding;
} __packed;

struct gb_camera_configure_streams_request {
	__le16 num_streams;
	__le16 padding;
	struct gb_camera_stream_config_request config[0];
} __packed;

/* Greybus Camera Configure Streams response payload */
struct gb_camera_stream_config_response {
	__le16 width;
	__le16 height;
	__le16 format;
	__u8 virtual_channel;
	__u8 data_type[2];
	__u8 padding[3];
	__le32 max_size;
} __packed;

struct gb_camera_configure_streams_response {
	__le16 num_streams;
#define GB_CAMERA_CONFIGURE_STREAMS_ADJUSTED	0x01
	u8 flags;
	u8 padding;
	struct gb_camera_stream_config_response config[0];
} __packed;

/* Greybus Camera Capture request payload - response has no payload */
struct gb_camera_capture_request {
	__le32 request_id;
	u8 streams;
	u8 padding;
	__le16 num_frames;
	u8 settings[0];
} __packed;

/* Greybus Camera Flush response payload - request has no payload */
struct gb_camera_flush_response {
	__le32 request_id;
} __packed;

/* Greybus Camera Metadata request payload - operation has no response */
struct gb_camera_metadata_request {
	__le32 request_id;
	__le16 frame_number;
	__u8 stream;
	__u8 padding;
	u8 metadata[0];
} __packed;

/* Lights */

#define GB_LIGHTS_VERSION_MAJOR 0x00
#define GB_LIGHTS_VERSION_MINOR 0x01

/* Greybus Lights request types */
#define GB_LIGHTS_TYPE_GET_LIGHTS		0x02
#define GB_LIGHTS_TYPE_GET_LIGHT_CONFIG		0x03
#define GB_LIGHTS_TYPE_GET_CHANNEL_CONFIG	0x04
#define GB_LIGHTS_TYPE_GET_CHANNEL_FLASH_CONFIG	0x05
#define GB_LIGHTS_TYPE_SET_BRIGHTNESS		0x06
#define GB_LIGHTS_TYPE_SET_BLINK		0x07
#define GB_LIGHTS_TYPE_SET_COLOR		0x08
#define GB_LIGHTS_TYPE_SET_FADE			0x09
#define GB_LIGHTS_TYPE_EVENT			0x0A
#define GB_LIGHTS_TYPE_SET_FLASH_INTENSITY	0x0B
#define GB_LIGHTS_TYPE_SET_FLASH_STROBE		0x0C
#define GB_LIGHTS_TYPE_SET_FLASH_TIMEOUT	0x0D
#define GB_LIGHTS_TYPE_GET_FLASH_FAULT		0x0E

/* Greybus Light modes */

/*
 * if you add any specific mode below, update also the
 * GB_CHANNEL_MODE_DEFINED_RANGE value accordingly
 */
#define GB_CHANNEL_MODE_NONE		0x00000000
#define GB_CHANNEL_MODE_BATTERY		0x00000001
#define GB_CHANNEL_MODE_POWER		0x00000002
#define GB_CHANNEL_MODE_WIRELESS	0x00000004
#define GB_CHANNEL_MODE_BLUETOOTH	0x00000008
#define GB_CHANNEL_MODE_KEYBOARD	0x00000010
#define GB_CHANNEL_MODE_BUTTONS		0x00000020
#define GB_CHANNEL_MODE_NOTIFICATION	0x00000040
#define GB_CHANNEL_MODE_ATTENTION	0x00000080
#define GB_CHANNEL_MODE_FLASH		0x00000100
#define GB_CHANNEL_MODE_TORCH		0x00000200
#define GB_CHANNEL_MODE_INDICATOR	0x00000400

/* Lights Mode valid bit values */
#define GB_CHANNEL_MODE_DEFINED_RANGE	0x000004FF
#define GB_CHANNEL_MODE_VENDOR_RANGE	0x00F00000

/* Greybus Light Channels Flags */
#define GB_LIGHT_CHANNEL_MULTICOLOR	0x00000001
#define GB_LIGHT_CHANNEL_FADER		0x00000002
#define GB_LIGHT_CHANNEL_BLINK		0x00000004

/* get count of lights in module */
struct gb_lights_get_lights_response {
	__u8	lights_count;
} __packed;

/* light config request payload */
struct gb_lights_get_light_config_request {
	__u8	id;
} __packed;

/* light config response payload */
struct gb_lights_get_light_config_response {
	__u8	channel_count;
	__u8	name[32];
} __packed;

/* channel config request payload */
struct gb_lights_get_channel_config_request {
	__u8	light_id;
	__u8	channel_id;
} __packed;

/* channel flash config request payload */
struct gb_lights_get_channel_flash_config_request {
	__u8	light_id;
	__u8	channel_id;
} __packed;

/* channel config response payload */
struct gb_lights_get_channel_config_response {
	__u8	max_brightness;
	__le32	flags;
	__le32	color;
	__u8	color_name[32];
	__le32	mode;
	__u8	mode_name[32];
} __packed;

/* channel flash config response payload */
struct gb_lights_get_channel_flash_config_response {
	__le32	intensity_min_uA;
	__le32	intensity_max_uA;
	__le32	intensity_step_uA;
	__le32	timeout_min_us;
	__le32	timeout_max_us;
	__le32	timeout_step_us;
} __packed;

/* blink request payload: response have no payload */
struct gb_lights_blink_request {
	__u8	light_id;
	__u8	channel_id;
	__le16	time_on_ms;
	__le16	time_off_ms;
} __packed;

/* set brightness request payload: response have no payload */
struct gb_lights_set_brightness_request {
	__u8	light_id;
	__u8	channel_id;
	__u8	brightness;
} __packed;

/* set color request payload: response have no payload */
struct gb_lights_set_color_request {
	__u8	light_id;
	__u8	channel_id;
	__le32	color;
} __packed;

/* set fade request payload: response have no payload */
struct gb_lights_set_fade_request {
	__u8	light_id;
	__u8	channel_id;
	__u8	fade_in;
	__u8	fade_out;
} __packed;

/* event request: generated by module */
struct gb_lights_event_request {
	__u8	light_id;
	__u8	event;
#define GB_LIGHTS_LIGHT_CONFIG		0x01
} __packed;

/* set flash intensity request payload: response have no payload */
struct gb_lights_set_flash_intensity_request {
	__u8	light_id;
	__u8	channel_id;
	__le32	intensity_uA;
} __packed;

/* set flash strobe state request payload: response have no payload */
struct gb_lights_set_flash_strobe_request {
	__u8	light_id;
	__u8	channel_id;
	__u8	state;
} __packed;

/* set flash timeout request payload: response have no payload */
struct gb_lights_set_flash_timeout_request {
	__u8	light_id;
	__u8	channel_id;
	__le32	timeout_us;
} __packed;

/* get flash fault request payload */
struct gb_lights_get_flash_fault_request {
	__u8	light_id;
	__u8	channel_id;
} __packed;

/* get flash fault response payload */
struct gb_lights_get_flash_fault_response {
	__le32	fault;
#define GB_LIGHTS_FLASH_FAULT_OVER_VOLTAGE		0x00000000
#define GB_LIGHTS_FLASH_FAULT_TIMEOUT			0x00000001
#define GB_LIGHTS_FLASH_FAULT_OVER_TEMPERATURE		0x00000002
#define GB_LIGHTS_FLASH_FAULT_SHORT_CIRCUIT		0x00000004
#define GB_LIGHTS_FLASH_FAULT_OVER_CURRENT		0x00000008
#define GB_LIGHTS_FLASH_FAULT_INDICATOR			0x00000010
#define GB_LIGHTS_FLASH_FAULT_UNDER_VOLTAGE		0x00000020
#define GB_LIGHTS_FLASH_FAULT_INPUT_VOLTAGE		0x00000040
#define GB_LIGHTS_FLASH_FAULT_LED_OVER_TEMPERATURE	0x00000080
} __packed;

/* Vendor */

#define GB_VENDOR_MOTO_VERSION_MAJOR		0x00
#define GB_VENDOR_MOTO_VERSION_MINOR		0x03

/* Greybus Motorola vendor specific request types */
#define GB_VENDOR_MOTO_TYPE_GET_DMESG		0x02
#define GB_VENDOR_MOTO_TYPE_GET_LAST_DMESG	0x03
#define GB_VENDOR_MOTO_TYPE_GET_PWR_UP_REASON	0x04
#define GB_VENDOR_MOTO_TYPE_GET_DMESG_SIZE	0x05
#define GB_VENDOR_MOTO_TYPE_GET_UPTIME		0x06

#define GB_VENDOR_MOTO_DEFAULT_DMESG_SIZE   1000
#define GB_VENDOR_MOTO_VER_DMESG_SIZE       2
#define GB_VENDOR_MOTO_VER_UPTIME           3

/* power up reason request has no payload */
struct gb_vendor_moto_pwr_up_reason_response {
	__le32 reason;
} __packed;

/* get dmesg size request has no payload */
struct gb_vendor_moto_get_dmesg_size_resp {
	__le16 size;
} __packed;

struct gb_vendor_moto_get_uptime_response {
	__le32 secs;
} __packed;

/* DISPLAY */

/* Version of the Greybus display protocol we support */
#define	GB_DISPLAY_VERSION_MAJOR		0x00
#define	GB_DISPLAY_VERSION_MINOR		0x02

/* Greybus Display operation types */
#define	GB_DISPLAY_HOST_READY			0x02
#define	GB_DISPLAY_GET_CONFIG_SIZE		0x03
#define	GB_DISPLAY_GET_CONFIG			0x04
#define	GB_DISPLAY_SET_CONFIG			0x05
#define	GB_DISPLAY_GET_STATE			0x06
#define	GB_DISPLAY_SET_STATE			0x07
#define	GB_DISPLAY_NOTIFICATION			0x08

#define GB_DISPLAY_STATE_OFF			0x00
#define GB_DISPLAY_STATE_ON			0x01

#define GB_DISPLAY_NOTIFY_INVALID		0x00
#define GB_DISPLAY_NOTIFY_FAILURE		0x01
#define GB_DISPLAY_NOTIFY_AVAILABLE		0x02
#define GB_DISPLAY_NOTIFY_UNAVAILABLE		0x03
#define GB_DISPLAY_NOTIFY_CONNECT		0x04
#define GB_DISPLAY_NOTIFY_DISCONNECT		0x05
#define GB_DISPLAY_NOTIFY_NUM_EVENTS		0x06

/* get display config size request has no payload */
struct gb_display_get_display_config_size_response {
	__le32	size;
} __packed;

/* get display config request has no payload */
struct gb_display_get_display_config_response {
	__u8	display_type;
	__u8	config_type;
	__u8	reserved[2];
	__u8	data[0];
} __packed;

struct gb_display_set_display_config_request {
	__u8	index;
} __packed;
/* set display config has no response */

/* get display config request has no payload */
struct gb_display_get_display_state_response {
	__u8	state;
} __packed;

struct gb_display_set_display_state_request {
	__u8	state;
} __packed;
/* set display state has no response */


/* USB-EXT */

#define GB_USB_EXT_VERSION_MAJOR     0x00
#define GB_USB_EXT_VERSION_MINOR     0x02

#define GB_USB_EXT_TYPE_READY        0x02
#define GB_USB_EXT_TYPE_ATTACH_STATE 0x03

#define GB_USB_EXT_PROTOCOL_2_0      0x00
#define GB_USB_EXT_PROTOCOL_3_1      0x01
#define GB_USB_EXT_PROTOCOL_DUAL     0x02

#define GB_USB_EXT_PATH_ENTERPRISE   0x00
#define GB_USB_EXT_PATH_BRIDGE       0x01

#define GB_USB_EXT_REMOTE_DEVICE     0x00
#define GB_USB_EXT_REMOTE_HOST       0x01

struct gb_usb_ext_attach_request {
	__u8 active;        /* attach or detach */
	__u8 protocol;      /* 2.0 or 3.1 */
	__u8 path;          /* tsb bridge or shared dp/usb */
	__u8 remote_type;   /* host or device */
} __packed;

/* no data for gb_usb_ext_attach_response */

/* SENSORS EXT */

/* Version of the Greybus protocol we support */
#define	GB_SENSORS_EXT_VERSION_MAJOR		0x00
#define	GB_SENSORS_EXT_VERSION_MINOR		0x02

/* Greybus Motorola vendor specific request types */
#define GB_SENSORS_EXT_TYPE_SENSOR_COUNT	0x02
#define GB_SENSORS_EXT_TYPE_SENSOR_INFO		0x03
#define GB_SENSORS_EXT_TYPE_START_REPORTING	0x04
#define GB_SENSORS_EXT_TYPE_FLUSH		0x05
#define GB_SENSORS_EXT_TYPE_STOP_REPORTING	0x06
#define GB_SENSORS_EXT_TYPE_EVENT		0x07

/* get count of sensors in module */
struct gb_sensors_get_sensor_count_response {
	__u8	sensors_count;
} __packed;

struct gb_sensors_ext_sensor_info_request {
	__u8	sensor_id;
} __packed;
/* sensor info response structure is gb_sensor */

/* this request has no response payload */
struct gb_sensors_ext_start_reporting_request {
	__u8	sensor_id;
	__u8	reserved[3];
	__le64	sampling_period;
	__le64	max_report_latency;
} __packed;

/* this request has no response payload */
struct gb_sensors_ext_flush_request {
	__u8	sensor_id;
} __packed;

struct gb_sensors_ext_report_data {
	__u8	sensor_id;
	__u8	flags;
	__le16	readings;
	__le64	reference_time;
	__u8	reading[];
} __packed;

struct gb_sensors_ext_report_hdr {
	__u8	reporting_sensors_count;
	__u8	reserved;
	struct gb_sensors_ext_report_data sensor[];
} __packed;


/* this request has no response payload */
struct gb_sensors_ext_stop_reporting_request {
	__u8	sensor_id;
} __packed;

/* CAMERA EXT */

/* Version of the Greybus camera protocol we support */
#define GB_CAMERA_EXT_VERSION_MAJOR 0x01
#define GB_CAMERA_EXT_VERSION_MINOR 0x01

/* Used to indicate enum index is not found */
#define GB_CAMERA_EXT_INVALID_INDEX cpu_to_le32(0xFFFFFFFF)

/* Greybus camera request types */
#define GB_CAMERA_EXT_TYPE_INVALID		0x00
#define GB_CAMERA_EXT_TYPE_PROTOCOL_VERSION	0x01

#define GB_CAMERA_EXT_TYPE_POWER_ON		0x02
#define GB_CAMERA_EXT_TYPE_POWER_OFF		0x03

#define GB_CAMERA_EXT_TYPE_INPUT_ENUM		0x04
#define GB_CAMERA_EXT_TYPE_INPUT_GET		0x05
#define GB_CAMERA_EXT_TYPE_INPUT_SET		0x06

#define GB_CAMERA_EXT_TYPE_FMT_ENUM		0x07
#define GB_CAMERA_EXT_TYPE_FMT_GET		0x08
#define GB_CAMERA_EXT_TYPE_FMT_SET		0x09

#define GB_CAMERA_EXT_TYPE_FMSIZE_ENUM		0x0A
#define GB_CAMERA_EXT_TYPE_FRMIVAL_ENUM		0x0B

#define GB_CAMERA_EXT_TYPE_STREAM_ON		0x0C
#define GB_CAMERA_EXT_TYPE_STREAM_OFF		0x0D

#define GB_CAMERA_EXT_TYPE_STREAM_PARM_SET	0x0E
#define GB_CAMERA_EXT_TYPE_STREAM_PARM_GET	0x0F

#define GB_CAMERA_EXT_TYPE_CTRL_GET_CFG		0x10

#define GB_CAMERA_EXT_TYPE_CTRL_GET		0x11
#define GB_CAMERA_EXT_TYPE_CTRL_SET		0x12
#define GB_CAMERA_EXT_TYPE_CTRL_TRY		0x13

#define GB_CAMERA_EXT_ASYNC_MESSAGE		0x14

struct camera_ext_predefined_ctrl_mod_req {
	/* Phone access MOD control by index (0, 1, ...).
	 */
	__le32 idx;
	/* required response size
	 * Expected responding config data size (including header) for
	 * GB_CAMERA_EXT_TYPE_CTRL_GET_CFG.
	 * Expected control value size for GB_CAMERA_EXT_TYPE_CTRL_GET.
	 * Size of control value to set/try for GB_CAMERA_EXT_TYPE_CTRL_SET/TRY.
	 */
	__le32 data_size;
	/* control value to set for GB_CAMERA_EXT_TYPE_CTRL_SET/TRY
	 */
	uint8_t data[0];
};

/* max control value size */
#define CAMERA_EXT_CTRL_MAX_VAL_SIZE (16 * 1024)

/* next available control info (to calc expected gb response size) */
struct camera_ext_ctrl_size_info {
	/* id, -1 if no more */
	__le32 id;
	/* if control has menu/dims from MOD, indicating the array
	 * item number
	 */
	__le32 array_size;
	/* control's value size.  */
	__le32 val_size;
} __packed;

/* ctrl config from MOD, playload is decided by
 * CAMERA_EXT_CTRL_FLAG_NEED_XXX. MOD side must provide all fields
 * each field is tagged by its NEED_XXX flag.
 */
struct camera_ext_predefined_ctrl_mod_cfg {
	/* control id at position idx (camera_ext_predefined_ctrl_mod_req) */
	__le32 id;
	struct camera_ext_ctrl_size_info next;
	/* FLAG0 DATA0 FLAG1 DATA1 ... */
	uint8_t data[0];
} __packed;

/* event send from MOD to AP */
struct camera_ext_event_hdr {
	__le32 type;
	uint8_t data[0];
} __packed;

struct camera_ext_event_error {
	__le32 error_code;
} __packed;

/* use 1024 as metadata length*/
#define CAMERA_EXT_EVENT_METADATA_DESC_LEN 1024

struct camera_ext_event_metadata {
	char desc[CAMERA_EXT_EVENT_METADATA_DESC_LEN];
} __packed;

/* open mode hint value sent along with power up request */
#define CAMERA_EXT_BOOTMODE_NORMAL	0
#define CAMERA_EXT_BOOTMODE_PREVIEW	1
#define CAMERA_EXT_BOOTMODE_DFU		2
#define CAMERA_EXT_BOOTMODE_MAX		3

#endif /* __GREYBUS_PROTOCOLS_H */

