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

#include "greybus_protocols.h"

int gb_sensors_ext_start_reporting(uint8_t sensor_id,
	uint64_t sampling_period, uint64_t max_report_latency);
int gb_sensors_ext_stop_reporting(uint8_t sensor_id);
int gb_sensors_ext_flush(uint8_t sensor_id);
struct gb_sensor *get_sensor_from_id(uint8_t sensor_id);
void gb_sensors_ext_get(void);
void gb_sensors_ext_put(void);
#endif /* _SENSORS_EXT_H */
