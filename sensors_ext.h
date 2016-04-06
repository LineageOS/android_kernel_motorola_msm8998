/*
 * Copyright (C) 2016 Motorola Mobility LLC
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

#ifndef _SENSORS_EXT_H
#define _SENSORS_EXT_H

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
	__u8	reporting_sensors_num;
	__u8	reserved;
	struct gb_sensors_ext_report_data sensor[];
} __packed;


/* this request has no response payload */
struct gb_sensors_ext_stop_reporting_request {
	__u8	sensor_id;
} __packed;

int gb_sensors_ext_start_reporting(uint8_t sensor_id,
	uint64_t sampling_period, uint64_t max_report_latency);
int gb_sensors_ext_stop_reporting(uint8_t sensor_id);
int gb_sensors_ext_flush(uint8_t sensor_id);
struct gb_sensor *get_sensor_from_id(uint8_t sensor_id);
void gb_sensors_ext_get(void);
void gb_sensors_ext_put(void);
#endif /* _SENSORS_EXT_H */
