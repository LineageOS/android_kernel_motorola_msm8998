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

#ifndef _SENSORS_EXT_IIO_H
#define _SENSORS_EXT_IIO_H

#include <linux/iio/iio.h>
#include "sensors_ext.h"

/* All values are endian-converted (le_to_cpu) by the greybus code. */
struct gb_sensor {
	uint32_t version;       /* RO - gb_sensors_intdata_show */
	uint32_t type;          /* RO - gb_sensors_intdata_show */
	uint32_t max_range;     /* RO - gb_sensors_intdata_show */
	uint32_t resolution;    /* RO - gb_sensors_intdata_show */

	/* IIO has a predefined 'power' directory, so this attribute must have a
	 * different name when exposed in SysFS. */
	uint32_t power;         /* RO - gb_sensors_intdata_show (uA) */

	int32_t min_delay;      /* RO - gb_sensors_intdata_show (us) */
	uint32_t max_delay;     /* RO - gb_sensors_intdata_show (us) */
	uint32_t fifo_rec;      /* RO - gb_sensors_intdata_show */
	uint32_t fifo_mec;      /* RO - gb_sensors_intdata_show */
	uint32_t flags;         /* RO - gb_sensors_intdata_show */

	int32_t scale_int;      /* RO - gb_sensors_read_raw */
	uint32_t scale_nano;    /* RO - gb_sensors_read_raw */
	int32_t offset_int;     /* RO - gb_sensors_read_raw */
	uint32_t offset_nano;   /* RO - gb_sensors_read_raw */

	uint8_t channels;
	uint8_t pad[3];

	uint16_t name_len;          /* Must be followed by the char[] field */
	char name[128];             /* RO - gb_sensors_strdata_show */
	uint16_t vendor_len;        /* Must be followed by the char[] field */
	char vendor[128];           /* RO - gb_sensors_strdata_show */
	uint16_t string_type_len;   /* Must be followed by the char[] field */
	char string_type[256];      /* RO - gb_sensors_strdata_show */

	uint64_t END_OF_GB_STRUCT;
	/* End of the Greybus structure. Below are a few kernel additions. */

	/** Sensor sampling period, in nano-seconds. */
	uint64_t sampling_period;   /* RW - gb_sensors_read/write_raw */
	/** Sensor maximum latency, in nano-seconds. */
	uint64_t max_latency;       /* RW - gb_sensors_intdata_show/store */

	uint64_t last_time;
	/* Contains data for all channels from the last reading. */
	int32_t *last_reading;
	/* Data written to the IIO buffer. Contains only enabled channels. */
	int32_t *out_data;
	int event_en;
	int event_val;

	struct iio_dev *iio;
	struct iio_trigger *trig;
	struct list_head   node;
	uint8_t sensor_id;
} __packed;


int gb_sensors_mod_attached(uint16_t count, struct list_head *sensors_list);
int gb_sensors_mod_detached(void);
int gb_sensors_rcv_data(struct gb_sensors_ext_report_hdr *report, size_t size);

/* Prototypes */
struct gb_sensor *iio_dev_to_sensor(struct iio_dev *);

/* Greybus sensor types.
 *
 * When updating this list, gb_sensor_type_map[] must also be updated. */
enum gb_sensors_type {
	UNDEFINED                     = 0x00000000,
	ACCELEROMETER                 = 0x00000001,
	GEOMAGNETIC_FIELD             = 0x00000002,
	ORIENTATION                   = 0x00000003,
	GYROSCOPE                     = 0x00000004,
	LIGHT                         = 0x00000005,
	PRESSURE                      = 0x00000006,
	TEMPERATURE                   = 0x00000007,
	PROXIMITY                     = 0x00000008,
	GRAVITY                       = 0x00000009,
	LINEAR_ACCELERATION           = 0x0000000a,
	ROTATION_VECTOR               = 0x0000000b,
	RELATIVE_HUMIDITY             = 0x0000000c,
	AMBIENT_TEMPERATURE           = 0x0000000d,
	MAGNETIC_FIELD_UNCALIBRATED   = 0x0000000e,
	GAME_ROTATION_VECTOR          = 0x0000000f,
	GYROSCOPE_UNCALIBRATED        = 0x00000010,
	SIGNIFICANT_MOTION            = 0x00000011,
	STEP_DETECTOR                 = 0x00000012,
	STEP_COUNTER                  = 0x00000013,
	GEOMAGNETIC_ROTATION_VECTOR   = 0x00000014,
	HEART_RATE                    = 0x00000015,
	TILT_DETECTOR                 = 0x00000016,
	WAKE_GESTURE                  = 0x00000017,
	GLANCE_GESTURE                = 0x00000018,
	PICK_UP_GESTURE               = 0x00000019,
	WRIST_TILT_GESTURE            = 0x0000001a,

	LAST_TYPE
};

enum gb_reporting_flags {
	FLUSHING	= 0x01,
	FLUSH_COMPLETE	= 0x02
};

#endif /* _SENSORS_EXT_IIO_H */
