/*
 * Copyright (C) 2016 Motorola Mobility.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "%s: " fmt, __func__
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/platform_device.h>
#include <linux/random.h>

#include "kernel_ver.h"
#include "sensors_ext_iio.h"
#include "sensors_ext.h"

/* The name of the DebugFS directory */
#define DEBUGFS_NAME "greybus-sensors-ext"

/* Encoding flags for sensor attributes */
#define ATTR_08     (1LL << 32)
#define ATTR_16     (1LL << 33)
#define ATTR_32     (1LL << 34)
#define ATTR_64     (1LL << 35)
#define ATTR_SIGNED (1LL << 36)

static const uint64_t e9 = 1000uLL * 1000uLL * 1000uLL;

/* Mapping of Greybus sensor types to IIO types */
static const int const gb_sensor_type_map[] = {
	[UNDEFINED]                     = IIO_PROPRIETARY,   /* No match */
	[ACCELEROMETER]                 = IIO_ACCEL,
	[GEOMAGNETIC_FIELD]             = IIO_MAGN,
	[ORIENTATION]                   = IIO_MAGN,
	[GYROSCOPE]                     = IIO_ANGL_VEL,
	[LIGHT]                         = IIO_LIGHT,
	[PRESSURE]                      = IIO_PRESSURE,
	[TEMPERATURE]                   = IIO_TEMP,
	[PROXIMITY]                     = IIO_PROXIMITY,
	[GRAVITY]                       = IIO_ACCEL,
	[LINEAR_ACCELERATION]           = IIO_ACCEL,
	[ROTATION_VECTOR]               = IIO_ROT,
	[RELATIVE_HUMIDITY]             = IIO_HUMIDITYRELATIVE,
	[AMBIENT_TEMPERATURE]           = IIO_TEMP,
	[MAGNETIC_FIELD_UNCALIBRATED]   = IIO_MAGN,
	[GAME_ROTATION_VECTOR]          = IIO_ROT,
	[GYROSCOPE_UNCALIBRATED]        = IIO_ANGL_VEL,
	[SIGNIFICANT_MOTION]            = IIO_PROPRIETARY,   /* No match */
	[STEP_DETECTOR]                 = IIO_PROPRIETARY,   /* No match */
	[STEP_COUNTER]                  = IIO_PROPRIETARY,   /* No match */
	[GEOMAGNETIC_ROTATION_VECTOR]   = IIO_ROT,
	[HEART_RATE]                    = IIO_PROPRIETARY,   /* No match */
	[TILT_DETECTOR]                 = IIO_INCLI,
	[WAKE_GESTURE]                  = IIO_PROPRIETARY,   /* No match */
	[GLANCE_GESTURE]                = IIO_PROPRIETARY,   /* No match */
	[PICK_UP_GESTURE]               = IIO_PROPRIETARY,   /* No match */
	[WRIST_TILT_GESTURE]            = IIO_PROPRIETARY,   /* No match */

	/* Add new types before this line */
	[LAST_TYPE]                     = IIO_PROPRIETARY
};

struct iio_gb_sensors_ext {
	struct dentry		*dbgfs_root;
	uint8_t			sensors_cnt;
	struct list_head	*sensors_list;
	struct mutex		ilock;
};

static const struct iio_event_spec iio_sensor_event = {
	.type = IIO_EV_TYPE_BUFFER_EMPTY,
	.dir = IIO_EV_DIR_FALLING,
	.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
};

/**
 * gb_sensors_read_event_config() - is event enabled?
 * @indio_dev: the device instance data
 * @chan: channel for the event whose state is being queried
 * @type: type of the event whose state is being queried
 * @dir: direction of the vent whose state is being queried
 */
int gb_sensors_read_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	pr_debug("type=%d dir=%d\n", type, dir);
	return sensor->event_en;
}

/**
 * gb_sensors_write_event_config() - set whether event is enabled
 * @indio_dev: the device instance data
 * @chan: channel for the event whose state is being set
 * @type: type of the event whose state is being set
 * @dir: direction of the vent whose state is being set
 * @state: whether to enable or disable the device.
 */
int gb_sensors_write_event_config(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir,
					int state)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	pr_debug("type=%d dir=%d state=%d\n", type, dir, state);

	switch (type) {
	case IIO_EV_TYPE_BUFFER_EMPTY:
		if (dir == IIO_EV_DIR_FALLING)
			sensor->event_en = state;
		else
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * gb_sensors_read_event_value() - get value associated with event
 * @indio_dev: device instance specific data
 * @chan: channel for the event whose value is being read
 * @type: type of the event whose value is being read
 * @dir: direction of the vent whose value is being read
 * @info: info type of the event whose value is being read
 * @val: value for the event code.
 */
int gb_sensors_read_event_value(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
					  enum iio_event_info info,
				      int *val, int *val2)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	pr_debug("type=%d dir=%d\n", type, dir);
	*val = sensor->event_val;

	return IIO_VAL_INT;
}

/**
 * gb_sensors_write_event_value() - set value associate with event
 * @indio_dev: device instance specific data
 * @chan: channel for the event whose value is being set
 * @type: type of the event whose value is being set
 * @dir: direction of the vent whose value is being set
 * @info: info type of the event whose value is being set
 * @val: the value to be set.
 */
int gb_sensors_write_event_value(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
					   enum iio_event_info info,
				       int val, int val2)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	pr_debug("type=%d dir=%d\n", type, dir);
	sensor->event_val = val;

	return 0;
}

static struct iio_gb_sensors_ext gb_drv;

int gb_sensors_buffer_postenable(struct iio_dev *indio_dev)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	pr_debug("start_reporting() sensorID=%d sensorName=%s scan_bytes=%d\n",
			sensor->sensor_id, sensor->name, indio_dev->scan_bytes);
	sensor->out_data = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);

	pr_debug("sensor=%p iio=%p out_data=%p id=%d\n", sensor, indio_dev,
			sensor->out_data, sensor->sensor_id);

	if (!sensor->out_data) {
		pr_err("out_data alloc failed.\n");
		return -ENOMEM;
	}

	gb_sensors_ext_start_reporting(sensor->sensor_id,
				sensor->sampling_period, sensor->max_latency);

	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * We can mess with hardware state above, before we attach the trigger.
	 */
	return iio_triggered_buffer_postenable(indio_dev);

}

int gb_sensors_buffer_predisable(struct iio_dev *indio_dev)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	pr_debug("stop_reporting() sensorID=%d sensorName=%s\n",
			sensor->sensor_id, sensor->name);

	mutex_lock(&gb_drv.ilock);

	gb_sensors_ext_stop_reporting(sensor->sensor_id);

	kfree(sensor->out_data);
	sensor->out_data = NULL;

	mutex_unlock(&gb_drv.ilock);

	/* iio_triggered_buffer_predisable:
	 * Generic function that simple detaches the pollfunc from the trigger.
	 * Before calling this, we can put hardware state back after the trigger
	 * is detached but before userspace knows we have disabled the buffer.
	 */
	return iio_triggered_buffer_predisable(indio_dev);
}

int gb_sensors_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	mutex_lock(&gb_drv.ilock);

	if (sensor->out_data) {
		/* This means the postenable() failed */
		gb_sensors_ext_stop_reporting(sensor->sensor_id);
		kfree(sensor->out_data);
		sensor->out_data = NULL;
	}

	mutex_unlock(&gb_drv.ilock);
	return 0;
}

static const struct iio_buffer_setup_ops gb_sensors_buffer_setup_ops = {
	.postenable = &gb_sensors_buffer_postenable,
	.predisable = &gb_sensors_buffer_predisable,
	.postdisable = &gb_sensors_buffer_postdisable,
};

/** Extract the gb_sensor struct associated with the given iio_dev */
struct gb_sensor *iio_dev_to_sensor(struct iio_dev *indio_dev)
{
	struct gb_sensor *sensor = *(struct gb_sensor **)iio_priv(indio_dev);
	return sensor;
}

/** Callback function to query the expected format/precision of an attribute
 * read by the gb_sensors_write_raw() function. If not set by the driver,
 * write_raw returns IIO_VAL_INT_PLUS_MICRO.
 */
int gb_sensors_write_raw_get_fmt(struct iio_dev *indio_dev,
		 struct iio_chan_spec const *chan,
		 long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}
}

/**
 * gb_sensors_read_raw() - data read function.
 *
 * This is called when the user tries to read one of the standard device
 * attributes.
 *
 * @indio_dev:	the struct iio_dev associated with this device instance
 * @chan:	the channel whose data is to be read
 * @val:	first element of returned value (typically INT)
 * @val2:	second element of returned value (typically MICRO or NANO)
 * @mask:	what we actually want to read as per the info_mask_*
 *		in iio_chan_spec. One of the iio_chan_info_enum values.
 */
int gb_sensors_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);
	uint64_t rem;
	int ret = -EINVAL;

	if (sensor == NULL)
		return ret;

	mutex_lock(&gb_drv.ilock);

	pr_debug("mask=%ld name=%s\n", mask, sensor->name);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (sensor->last_reading) {
			*val = sensor->last_reading[chan->scan_index];
			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = div64_u64(e9, sensor->sampling_period);
		rem = e9 - (sensor->sampling_period * *val);
		*val2 = div64_u64(rem * e9, sensor->sampling_period);
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = sensor->scale_int;
		*val2 = sensor->scale_nano;
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	case IIO_CHAN_INFO_OFFSET:
		*val = sensor->offset_int;
		*val2 = sensor->offset_nano;
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	default:
		pr_debug("Unsupported. mask=%ld", mask);
		break;
	}

	mutex_unlock(&gb_drv.ilock);
	return ret;
}

/**
 * gb_sensors_write_raw() - data write function.
 *
 * This is called when the user tries to configure one of the standard device
 * attributes.
 *
 * @indio_dev:	the struct iio_dev associated with this device instance
 * @chan:	the channel whose data is to be written
 * @val:	first element of value to set (typically INT)
 * @val2:	second element of value to set (typically MICRO)
 * @mask:	what we actually want to write as per the info_mask_*
 *		in iio_chan_spec.
 *
 * Note that all raw writes are assumed IIO_VAL_INT and info mask elements
 * are assumed to be IIO_INT_PLUS_MICRO unless the callback write_raw_get_fmt
 * in struct iio_info is provided by the driver.
 */
int gb_sensors_write_raw(struct iio_dev *indio_dev,
		 struct iio_chan_spec const *chan,
		 int val, int val2, long mask)
{
	int ret = -EINVAL;
	uint64_t freq;
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	pr_debug("val=%d val2=%d mask=%ld name=%s\n", val, val2, mask,
			sensor->name);
	mutex_lock(&gb_drv.ilock);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/* write_raw_get_fmt sets this input to IIO_INT_PLUS_NANO */
		freq = val * e9 + val2;
		if (freq > 0) {
			sensor->sampling_period = div64_u64(e9 * e9, freq);
			if (sensor->sampling_period == 0)
				sensor->sampling_period = 1;
			pr_debug("sampling_period=%lld 0x%08x%08x\n",
					sensor->sampling_period,
					(uint32_t)(sensor->sampling_period>>32),
					(uint32_t)sensor->sampling_period);
			ret = 0;
		} else {
			pr_err("Invalid frequency (%d, %d)", val, val2);
		}
		break;
	default:
		pr_debug("Unsupported. mask=%ld\n", mask);
		break;
	}

	mutex_unlock(&gb_drv.ilock);
	return ret;
}

/** Flush sensor data to support batching. */
static ssize_t gb_sensors_flush(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int flush;
	int err;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	err = kstrtouint(buf, 0, &flush);

	/* We don't care what the user sends us */
	pr_debug("flush=%d sensor=%d\n", flush, sensor->sensor_id);

	err = gb_sensors_ext_flush(sensor->sensor_id);
	if (err < 0) {
		pr_err("Flush result %d", err);
		return err;
	} else {
		return count;
	}
}

/** Display the last sensor reading in a user-friendly way. */
static ssize_t gb_sensors_iiodata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t size = 0;
	ssize_t res;
	int i;

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);

	if (!sensor->last_reading)
		return -EINVAL;

	mutex_lock(&gb_drv.ilock);

#define CHECK_SIZE() do { if (res < 0 || res == PAGE_SIZE - size) \
	goto buffer_full; else size += res; } while (0)

	res = snprintf(buf + size, PAGE_SIZE - size,
			"Time: %llu\n", sensor->last_time);
	CHECK_SIZE();

	/* indio_dev->scan_bytes is only updated when the buffer is enabled,
	   based on the scan_elements that are enabled at the time. */
	res = snprintf(buf + size, PAGE_SIZE - size,
			"ScanBytes: %d\n", indio_dev->scan_bytes);
	CHECK_SIZE();

	for (i = 0; i < sensor->channels; ++i) {
		res = snprintf(buf + size, PAGE_SIZE - size,
				"Chan%02d: 0x%08x  %d\n", i,
				sensor->last_reading[i],
				sensor->last_reading[i]);
		CHECK_SIZE();
	}

buffer_full:
	mutex_unlock(&gb_drv.ilock);
	return (size < 0 || size == PAGE_SIZE) ? 0 : size;
}

static inline uint64_t gb_sensors_rtc_to_mono(uint64_t rtc)
{
	struct timespec rtc_ts;
	struct timespec mon_ts;
	uint64_t m_0;
	uint64_t r_0;
	uint64_t r_t;

	ktime_get_ts(&mon_ts);
	ktime_get_real_ts(&rtc_ts);
	m_0 = timespec_to_ns(&mon_ts);
	r_0 = timespec_to_ns(&rtc_ts);
	r_t = rtc;

	/*                              */
	/* mono  = rtc   - rtc  + mono  */
	/*     t      t       0       0 */
	return r_t - r_0 + m_0;
}

/** This function processes data from a single sensor. */
static size_t gb_sensors_rcv_data_sensor(
		struct gb_sensors_ext_report_data *report, size_t size)
{
	struct gb_sensor *sensor;
	struct iio_dev *indio_dev;
	uint64_t timestamp = 0;
	uint16_t r, chan, data_offset;
	int dest, src, data_sz;
	int32_t *out_data;

	/* The sensor-specific header size. */
	const size_t sensor_hdr_sz =
		offsetof(struct gb_sensors_ext_report_data, reading);

	struct sensor_reading {
		uint16_t time_delta;
		int32_t value[];
	} __packed;
	struct sensor_reading *s_reading = NULL;

	/* Make sure we have at least the header fields to start off with. */
	if (size < sensor_hdr_sz) {
		pr_err("Expected at least %zu bytes. Got only %zu.\n",
				sensor_hdr_sz, size);
		return -EINVAL;
	}

	report->readings       = le16_to_cpu(report->readings);
	report->reference_time =
		gb_sensors_rtc_to_mono(le64_to_cpu(report->reference_time));

	pr_debug("SensorID=%d, readings=%d flags=%d\n",
			report->sensor_id, report->readings, report->flags);

	sensor = get_sensor_from_id(report->sensor_id);
	if (sensor == NULL) {
		pr_err("Invalid sensor ID (%d)", report->sensor_id);
		return -EINVAL;
	}

	size -= sensor_hdr_sz;
	data_sz = sensor->channels * sizeof(int32_t);

	if (size < (data_sz + 2) * report->readings) {
		pr_err("Insufficient data (%zu bytes for %d readings)\n",
				size, report->readings);
		return -EINVAL;
	}

	mutex_lock(&gb_drv.ilock);
	indio_dev = sensor->iio;
	out_data = sensor->out_data;

	if (!out_data) {
		mutex_unlock(&gb_drv.ilock);
		/* This could happen if the mod reports the wrong sensor_id and
		then we try to publish data for a sensor that is not enabled. */
		pr_err("Sensor out_data is NULL\n");
		return -EIO;
	}

	data_offset = 0;

	for (r = 0; r < report->readings; ++r) {
		s_reading = (struct sensor_reading *)
			&(report->reading[data_offset]);

		timestamp = report->reference_time +
			le16_to_cpu(s_reading->time_delta);

		for (chan = 0; chan < sensor->channels; ++chan) {
			s_reading->value[chan] =
				le32_to_cpu(s_reading->value[chan]);
		}

		if (bitmap_full(indio_dev->active_scan_mask,
				 indio_dev->masklength)) {
			memcpy(out_data, s_reading->value, data_sz);
		} else {
			for (dest = 0, src = 0;
			     dest < bitmap_weight(indio_dev->active_scan_mask,
						  indio_dev->masklength);
			     dest++, src++) {

				src = find_next_bit(indio_dev->active_scan_mask,
					indio_dev->masklength, src);
				out_data[dest] = s_reading->value[src];
			}
		}

		iio_push_to_buffers_with_timestamp(sensor->iio,
				out_data, timestamp);
		data_offset += 2 + data_sz;
	}

	if (s_reading && sensor->last_reading) {
		sensor->last_time = timestamp;
		memcpy(sensor->last_reading, s_reading->value, data_sz);
	}

	sensor->event_val = report->flags;
	if ((report->flags & FLUSH_COMPLETE) && sensor->event_en) {
		pr_debug("Flush complete (%d)", report->flags);
		/* Only emit event for chan 0 */
		iio_push_event(indio_dev,
				IIO_EVENT_CODE(IIO_PROPRIETARY, 0, 0,
						IIO_EV_DIR_FALLING,
						IIO_EV_TYPE_BUFFER_EMPTY,
						0, 0, 0),
				timestamp);
	}

	mutex_unlock(&gb_drv.ilock);
	return offsetof(struct gb_sensors_ext_report_data, reading) +
		data_offset;
}

/** This function is called by the Greybus code when it receives sensor data
 * from a mod. All the endian conversions must be done here. */
int gb_sensors_rcv_data(struct gb_sensors_ext_report_hdr *event_report,
		size_t size)
{
	char *sensor_data_ptr;
	uint16_t s;
	size_t sensor_data_offset, bytes_consumed;
	struct gb_sensors_ext_report_data *sensor_data;
	const size_t hdr_size =
		offsetof(struct gb_sensors_ext_report_hdr, sensor);

	if (size < hdr_size) {
		pr_err("Not enough data. Expecting >= %zu bytes\n", hdr_size);
		return -EINVAL;
	}

	sensor_data_ptr = (char *)&(event_report->sensor[0]);
	sensor_data_offset = 0;
	size -= hdr_size;

	pr_debug("Sensors: %d, sz=%zu\n",
			event_report->reporting_sensors_count, size);

	for (s = 0; s < event_report->reporting_sensors_count; ++s) {
		sensor_data = (struct gb_sensors_ext_report_data *)
			(sensor_data_ptr + sensor_data_offset);
		bytes_consumed = gb_sensors_rcv_data_sensor(sensor_data, size);
		if (bytes_consumed < 0)
			return bytes_consumed;
		sensor_data_offset += bytes_consumed;
		size -= bytes_consumed;
	}

	if (size)
		pr_warn("Extra (unparsed) data present");

	return 0;
}

/** Show string typed sensor attributes.
 *
 * @attr:	The iio_dev_attr.address field must contain the offset in struct
 *              gb_sensor to the length field. The string (char array) must
 *              immediately follow the length field in the gb_sensor struct.
 */
static ssize_t gb_sensors_strdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t length;

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);
	long offset = (long)this_attr->address;

	mutex_lock(&gb_drv.ilock);

	length = *(uint16_t *)((char *)sensor + offset);
	if (length > 0) {
		/* Leave room for a \n terminator */
		if (length + 1 > PAGE_SIZE)
			length = PAGE_SIZE - 1;
		memcpy(buf, (char *)sensor + offset + sizeof(length), length);
		buf[length++] = '\n';
	}

	mutex_unlock(&gb_drv.ilock);
	return length;
}

/** Convert an integer value from the gb_sensor struct to a string for display
 * to the user.
 *
 * The iio_dev_attr.address field must contain the offset in struct gb_sensor to
 * the field to be converted in the 4 LSB bytes. To this offset, one of the
 * ATTR_08, ATTR_16, ATTR_32, or ATTR_64 field width flags must be added, along
 * with the optional ATTR_SIGNED flag if the integer value is signed.
 */
static ssize_t gb_sensors_intdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t size = 0;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);
	long offset = (long)this_attr->address & 0xffFFffFF;
	uint64_t flags = this_attr->address & (0xffFFffFFll << 32);

	uint8_t     val_u8;
	int8_t      val_s8;
	uint16_t    val_u16;
	int16_t     val_s16;
	uint32_t    val_u32;
	int32_t     val_s32;
	uint64_t    val_u64;
	int64_t     val_s64;

	mutex_lock(&gb_drv.ilock);

	if (flags & ATTR_08) {
		if (flags & ATTR_SIGNED) {
			val_s8 = *(int8_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%d\n", val_s8);
		} else {
			val_u8 = *(uint8_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%u\n", val_u8);
		}
	} else if (flags & ATTR_16) {
		if (flags & ATTR_SIGNED) {
			val_s16 = *(int16_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%d\n", val_s16);
		} else {
			val_u16 = *(uint16_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%u\n", val_u16);
		}
	} else if (flags & ATTR_32) {
		if (flags & ATTR_SIGNED) {
			val_s32 = *(int32_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%d\n", val_s32);
		} else {
			val_u32 = *(uint32_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%u\n", val_u32);
		}
	} else if (flags & ATTR_64) {
		if (flags & ATTR_SIGNED) {
			val_s64 = *(int64_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%lld\n", val_s64);
		} else {
			val_u64 = *(uint64_t *)((char *)sensor + offset);
			size = snprintf(buf, PAGE_SIZE, "%llu\n", val_u64);
		}
	} else {
		pr_err("Error converting (flags=0x%x)\n",
				(uint32_t)(flags >> 32));
	}

	mutex_unlock(&gb_drv.ilock);
	return size;
}

/** Convert a user-provided string to an int and store it in the gb_sensor
 * struct represented by the dev argument.
 *
 * The iio_dev_attr.address field must contain in its 4 LSB bytes the offset in
 * struct gb_sensor to the field that will hold the converted value. To this
 * offset, one of the ATTR_08, ATTR_16, ATTR_32, or ATTR_64 field width flags
 * must be added, along with the optional ATTR_SIGNED flag if the destination
 * integer value is signed.
 */
static ssize_t gb_sensors_intdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct gb_sensor *sensor = iio_dev_to_sensor(indio_dev);
	long offset = (long)this_attr->address & 0xffFFffFF;
	uint64_t flags = this_attr->address & (0xffFFffFFll << 32);

/* Stores a value of the given type at a specific offset in the sensor struct */
#define TYPED_VAL(type) (*(type *)((char *)sensor + offset))

	uint8_t     val_u8;
	int8_t      val_s8;
	uint16_t    val_u16;
	int16_t     val_s16;
	uint32_t    val_u32;
	int32_t     val_s32;
	uint64_t    val_u64;
	int64_t     val_s64;

	mutex_lock(&gb_drv.ilock);

	if (flags & ATTR_08) {
		if (flags & ATTR_SIGNED) {
			if (kstrtos8(buf, 10, &val_s8))
				goto store_error;
			TYPED_VAL(int8_t) = val_s8;
		} else {
			if (kstrtou8(buf, 10, &val_u8))
				goto store_error;
			TYPED_VAL(uint8_t) = val_u8;
		}
	} else if (flags & ATTR_16) {
		if (flags & ATTR_SIGNED) {
			if (kstrtos16(buf, 10, &val_s16))
				goto store_error;
			TYPED_VAL(int16_t) = val_s16;
		} else {
			if (kstrtou16(buf, 10, &val_u16))
				goto store_error;
			TYPED_VAL(uint16_t) = val_u16;
		}
	} else if (flags & ATTR_32) {
		if (flags & ATTR_SIGNED) {
			if (kstrtoint(buf, 10, &val_s32))
				goto store_error;
			TYPED_VAL(int32_t) = val_s32;
		} else {
			if (kstrtouint(buf, 10, &val_u32))
				goto store_error;
			TYPED_VAL(uint32_t) = val_u32;
		}
	} else if (flags & ATTR_64) {
		if (flags & ATTR_SIGNED) {
			if (kstrtoll(buf, 10, &val_s64))
				goto store_error;
			TYPED_VAL(int64_t) = val_s64;
		} else {
			if (kstrtoull(buf, 10, &val_u64))
				goto store_error;
			TYPED_VAL(uint64_t) = val_u64;
		}
	} else {
		pr_err("Error converting (flags=0x%x %s)\n",
				(uint32_t)(flags >> 32), buf);
	}

store_error:
	mutex_unlock(&gb_drv.ilock);
	return size;
}

/*                     name,    mode,    show,                   store, addr */
static IIO_DEVICE_ATTR(iiodata, S_IRUGO, gb_sensors_iiodata_show, NULL, 0);
static IIO_DEVICE_ATTR(flush,   S_IWUSR, NULL, gb_sensors_flush, 0);

static IIO_DEVICE_ATTR(greybus_name, S_IRUGO, gb_sensors_strdata_show, NULL,
	offsetof(struct gb_sensor, name_len));
static IIO_DEVICE_ATTR(vendor, S_IRUGO, gb_sensors_strdata_show, NULL,
	offsetof(struct gb_sensor, vendor_len));
static IIO_DEVICE_ATTR(string_type, S_IRUGO, gb_sensors_strdata_show, NULL,
	offsetof(struct gb_sensor, string_type_len));

static IIO_DEVICE_ATTR(greybus_version, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, version) + ATTR_32);
static IIO_DEVICE_ATTR(greybus_type, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, type) + ATTR_32);
static IIO_DEVICE_ATTR(max_range, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, max_range) + ATTR_32);
static IIO_DEVICE_ATTR(resolution, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, resolution) + ATTR_32);
static IIO_DEVICE_ATTR(power_uA, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, power) + ATTR_32);

static IIO_DEVICE_ATTR(min_delay_us, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, min_delay) + ATTR_32+ATTR_SIGNED);
static IIO_DEVICE_ATTR(max_delay_us, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, max_delay) + ATTR_32);
static IIO_DEVICE_ATTR(fifo_rec, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, fifo_rec) + ATTR_32);
static IIO_DEVICE_ATTR(fifo_mec, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, fifo_mec) + ATTR_32);
static IIO_DEVICE_ATTR(flags, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, flags) + ATTR_32);

static IIO_DEVICE_ATTR(greybus_name_len, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, name_len) + ATTR_16);
static IIO_DEVICE_ATTR(vendor_len, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, vendor_len) + ATTR_16);
static IIO_DEVICE_ATTR(string_type_len, S_IRUGO, gb_sensors_intdata_show, NULL,
	(uint64_t)offsetof(struct gb_sensor, string_type_len) + ATTR_16);

static IIO_DEVICE_ATTR(max_latency_ns, S_IRUGO | S_IWUSR,
	gb_sensors_intdata_show, gb_sensors_intdata_store,
	(uint64_t)offsetof(struct gb_sensor, max_latency) + ATTR_64);

static struct attribute *gb_sensors_iio_attributes[] = {
	&iio_dev_attr_iiodata           .dev_attr.attr,
	&iio_dev_attr_flush             .dev_attr.attr,

	&iio_dev_attr_greybus_name      .dev_attr.attr,
	&iio_dev_attr_vendor            .dev_attr.attr,
	&iio_dev_attr_string_type       .dev_attr.attr,

	&iio_dev_attr_greybus_version   .dev_attr.attr,
	&iio_dev_attr_greybus_type      .dev_attr.attr,
	&iio_dev_attr_max_range         .dev_attr.attr,
	&iio_dev_attr_resolution        .dev_attr.attr,
	&iio_dev_attr_power_uA          .dev_attr.attr,

	&iio_dev_attr_min_delay_us      .dev_attr.attr,
	&iio_dev_attr_max_delay_us      .dev_attr.attr,
	&iio_dev_attr_fifo_rec          .dev_attr.attr,
	&iio_dev_attr_fifo_mec          .dev_attr.attr,
	&iio_dev_attr_flags             .dev_attr.attr,

	&iio_dev_attr_greybus_name_len  .dev_attr.attr,
	&iio_dev_attr_vendor_len        .dev_attr.attr,
	&iio_dev_attr_string_type_len   .dev_attr.attr,

	&iio_dev_attr_max_latency_ns    .dev_attr.attr,

	NULL
};

/* This is the same for each individual sensor */
static const struct attribute_group gb_sensors_attr_group = {
	.attrs = gb_sensors_iio_attributes,
};

static irqreturn_t gb_sensors_trigger_thread(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio = pf->indio_dev;

	pr_debug("irq=%d\n", irq);

	iio_trigger_notify_done(indio->trig);
	return IRQ_HANDLED;
}

static const struct iio_trigger_ops sensors_ext_iio_trigger_ops = {
	.owner = THIS_MODULE,
};

#ifdef IIO_CORE_REGISTERS_BUFFER
static inline int
gb_sensors_buffer_register(struct iio_dev *indio_dev,
				const struct iio_chan_spec *channels,
				int num_channels)
{
	return 0;
}

static inline void gb_sensors_buffer_unregister(struct iio_dev *indio_dev)
{}
#else
static inline int
gb_sensors_buffer_register(struct iio_dev *indio_dev,
				const struct iio_chan_spec *channels,
				int num_channels)
{
	return iio_buffer_register(indio_dev, channels, num_channels);
}

static inline void gb_sensors_buffer_unregister(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
}
#endif

#ifdef IIO_KFIFO_ALLOC_NO_PARAMS
#define TO_KFIFO_ALLOC_PARAM(dev)
#else
#define TO_KFIFO_ALLOC_PARAM(dev) (dev)
#endif

/** Creates an IIO device for the given sensor.
 *
 * If successful, SysFS will contain a new directory corresponding to this
 * sensor. */
static int gb_sensors_add_sensor(struct gb_sensor *sensor)
{
	int retval, c;
	struct iio_info *info;
	struct iio_chan_spec *chan;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct gb_sensor **iio_sensor;

	pr_debug("Sensor name = %s.\n",
			sensor->name_len ? sensor->name : "Unknown");
	mutex_lock(&gb_drv.ilock);

	/* Initialize to a sane sampling period (1Hz) */
	sensor->sampling_period = 1LL * 1000 * 1000 * 1000;

	indio_dev = iio_device_alloc(sizeof(struct gb_sensor *));
	if (indio_dev == NULL) {
		pr_err(" Failed to allocate IIO device.\n");
		mutex_unlock(&gb_drv.ilock);
		return -ENOMEM;
	}

	sensor->last_reading = kcalloc(sensor->channels, sizeof(int32_t),
			GFP_KERNEL);
	if (!sensor->last_reading) {
		retval = -ENOMEM;
		goto error_last_reading;
	}

	iio_sensor = iio_priv(indio_dev);
	*iio_sensor = sensor;
	sensor->iio = indio_dev;
	sensor->event_en = 1;

	indio_dev->name = sensor->name;
	indio_dev->modes  = INDIO_DIRECT_MODE;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	/* Define the info structure */
	info = kzalloc(sizeof(struct iio_info), GFP_KERNEL);
	if (info == NULL) {
		retval = -ENOMEM;
		goto error_info;
	}
	pr_debug("Allocated IIO Info\n");

	info->driver_module = THIS_MODULE;
	info->attrs = &gb_sensors_attr_group;
	info->read_raw = &gb_sensors_read_raw;
	info->write_raw = &gb_sensors_write_raw;
	info->write_raw_get_fmt = &gb_sensors_write_raw_get_fmt;

	/* For event handling */
	info->read_event_config = &gb_sensors_read_event_config;
	info->write_event_config = &gb_sensors_write_event_config;
	info->read_event_value = &gb_sensors_read_event_value;
	info->write_event_value = &gb_sensors_write_event_value;

	indio_dev->info = info;

	/* Define the channel(s) for this sensor: timestamp + data */
	indio_dev->num_channels = sensor->channels + 1;
	chan = kcalloc(indio_dev->num_channels, sizeof(struct iio_chan_spec),
			GFP_KERNEL);
	if (chan == NULL) {
		retval = -ENOMEM;
		goto error_chan;
	}
	pr_debug("Allocated %d IIO Channels\n", indio_dev->num_channels);

	for (c = 0; c < sensor->channels; c++) {
		pr_debug("Configuring channel %d\n", c);
		chan[c].type = gb_sensor_type_map[
			min_t(uint32_t, LAST_TYPE, sensor->type)];
		chan[c].scan_index = c;
		chan[c].channel = c;

		/* Decide how do we want to handle multiple data channels per
		 * channel type. */
		switch (sensor->channels) {
		/* libiio doesn't handle non-modified & non-indexed channels.
		 * Enable this case once libiio is fixed.
		case 1: // Single value is not indexed or modified
			break;
		*/
		case 3: /* Assume X, Y, Z */
			chan[c].modified = 1;
			chan[c].channel2 = IIO_MOD_X + c;
			break;
		default: /* Indexed */
			chan[c].indexed = 1;
			break;
		}
		pr_debug("idx=%d type=%d mod=%d chan2=%d map=%d",
				chan[c].scan_index, chan[c].type,
				chan[c].modified, chan[c].channel2,
			min_t(uint32_t, LAST_TYPE, sensor->type));

		chan[c].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		chan[c].info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET);
		chan[c].scan_type.sign = 's';
		chan[c].scan_type.realbits = 4*8;
		chan[c].scan_type.storagebits = 4*8;
		chan[c].scan_type.shift = 0;
		chan[c].scan_type.endianness = IIO_CPU;

		/* We generate events for flush complete. */
		chan[c].event_spec = &iio_sensor_event;
		chan[c].num_event_specs = 1;
	}

	/* The first channel is the timestamp. Beware,
	 * iio_push_to_buffers_with_timestamp() always adds the timestamp at the
	 * end, regardless of what the scan_index here is set to. It also aligns
	 * it on 8-byte boundary, so there might be padding between the last
	 * scan element and the timestamp. */
	c = sensor->channels;
	chan[c].type            = IIO_TIMESTAMP;
	chan[c].channel         = -1;
	chan[c].scan_index      = c;
	chan[c].scan_type.sign          = 's';
	chan[c].scan_type.realbits      = 64;
	chan[c].scan_type.storagebits   = 64;
	chan[c].scan_type.shift         = 0;
	chan[c].scan_type.endianness    = IIO_CPU;

	chan[c].event_spec = &iio_sensor_event;
	chan[c].num_event_specs = 1;

	indio_dev->channels = chan;

	/* Configure the buffer. TODO: Switch to a ring buffer, so we discard
	 * the oldest entries if we reach the limit. */
	buffer = iio_kfifo_allocate(TO_KFIFO_ALLOC_PARAM(indio_dev));
	if (buffer == NULL) {
		pr_err("Failed to allocate IIO kfifo\n");
		retval = -ENOMEM;
		goto error_kfifo;
	}

	iio_device_attach_buffer(indio_dev, buffer);

	/* Used in conjunction with iio_push_to_buffers_with_timestamp() */
	buffer->scan_timestamp = true;

	/* Tell the core what device type specific functions should be run on
	 * either side of the buffer capture enable / disable. */
	indio_dev->setup_ops = &gb_sensors_buffer_setup_ops;

	/*
	 * Configure a polling function.
	 * When a trigger event with this polling function connected
	 * occurs, this function is run. Typically this grabs data
	 * from the device.
	 *
	 * NULL for the bottom half. This is normally implemented only if we
	 * either want to ping a capture now pin (no sleeping) or grab
	 * a timestamp as close as possible to a data ready trigger firing.
	 *
	 * IRQF_ONESHOT ensures irqs are masked such that only one instance
	 * of the handler can run at a time.
	 *
	 * "gb_sensors_consumer%d" formatting string for the irq 'name'
	 * as seen under /proc/interrupts. Remaining parameters as per printk.
	 */
	indio_dev->pollfunc = iio_alloc_pollfunc(NULL,
			&gb_sensors_trigger_thread,
			IRQF_ONESHOT,
			indio_dev,
			"gb_sensors_consumer%d",
			indio_dev->id);

	if (indio_dev->pollfunc == NULL) {
		retval = -ENOMEM;
		goto error_buffer;
	}


	retval = gb_sensors_buffer_register(indio_dev, indio_dev->channels,
			indio_dev->num_channels);
	if (retval < 0) {
		pr_err("Failed to register IIO buffer (%d).\n", retval);
		goto error_pollfunc;
	}

	sensor->trig = iio_trigger_alloc("gb_sensors_consumer%d",
			indio_dev->id);
	if (!sensor->trig) {
		retval = -ENOMEM;
		goto error_trigger_alloc;
	}

	sensor->trig->ops = &sensors_ext_iio_trigger_ops;
	iio_trigger_set_drvdata(sensor->trig, indio_dev);
	indio_dev->trig = iio_trigger_get(sensor->trig);
	retval = iio_trigger_register(sensor->trig);
	if (retval)
		goto error_free_trig;

	retval = iio_device_register(indio_dev);
	if (retval < 0) {
		pr_err("Failed to register IIO device (%d).\n", retval);
		goto error_register;
	}

	mutex_unlock(&gb_drv.ilock);
	return retval;

error_register:
	iio_trigger_unregister(sensor->trig);
error_free_trig:
	iio_trigger_free(sensor->trig);
error_trigger_alloc:
	gb_sensors_buffer_unregister(indio_dev);
error_pollfunc:
	iio_dealloc_pollfunc(indio_dev->pollfunc);
error_buffer:
	iio_kfifo_free(buffer);
error_kfifo:
	kzfree(chan);
error_chan:
	kzfree(info);
error_info:
	kfree(sensor->last_reading);
	sensor->last_reading = NULL;
error_last_reading:
	iio_device_free(indio_dev);

	mutex_unlock(&gb_drv.ilock);
	return retval;
}

static void gb_sensors_rm_sensor(struct gb_sensor *sensor)
{
	struct iio_dev *indio_dev = sensor->iio;
	struct iio_chan_spec const *chan = indio_dev->channels;
	struct iio_info const *info = indio_dev->info;

	pr_debug("Removing %s\n", sensor->name_len ? sensor->name : "Unknown");
	iio_device_unregister(indio_dev);
	gb_sensors_buffer_unregister(indio_dev);
	iio_trigger_unregister(sensor->trig);
	iio_trigger_free(sensor->trig);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
	iio_kfifo_free(indio_dev->buffer);
	iio_device_free(indio_dev);

	kzfree(chan);
	kzfree(info);

	kfree(sensor->last_reading);
	sensor->last_reading = NULL;

	kfree(sensor->out_data);
	sensor->out_data = NULL;
}

#if defined(CONFIG_DEBUG_FS)
/** Tests the dentry to see if it's valid. */
static int gb_sensors_valid_dentry(struct dentry *entry)
{
	return entry && entry != ERR_PTR(-ENODEV);
}

static void gb_sensors_create_dbgfs(void)
{
	pr_debug("IIO Greybus Sensors creating DebugFS\n");

	gb_drv.dbgfs_root = debugfs_create_dir(DEBUGFS_NAME, NULL);
	if (!gb_sensors_valid_dentry(gb_drv.dbgfs_root)) {
		pr_debug("Failed creating debugfs\n");
		return;
	}

	debugfs_create_u16("sensors_cnt", S_IRUGO, gb_drv.dbgfs_root,
			(uint16_t *)&gb_drv.sensors_cnt);
}

static void gb_sensors_destroy_dbgfs(void)
{
	if (gb_sensors_valid_dentry(gb_drv.dbgfs_root))
		debugfs_remove_recursive(gb_drv.dbgfs_root);
}
#else
static void gb_sensors_create_dgbfs(void) { }
static void gb_sensors_destroy_dgbfs(void) { }
#endif /* CONFIG_DEBUG_FS */

/** This function is called by the Greybus code when a mod is attached. */
int gb_sensors_mod_attached(uint16_t count, struct list_head *sensors_list)
{

	int retval = 0;
	struct gb_sensor *sensor;

	pr_info("IIO Greybus Sensors creating\n");
	mutex_init(&gb_drv.ilock);

	gb_drv.sensors_cnt = count;
	gb_drv.sensors_list = sensors_list;

	list_for_each_entry(sensor, sensors_list, node) {
		retval = gb_sensors_add_sensor(sensor);
		if (retval < 0) {
			pr_err("Failed adding sensor %s (%d)\n",
				sensor->name_len ? sensor->name : "Unknown",
				sensor->sensor_id);
			goto error_add_sensor;
		}
		gb_sensors_ext_get();
	}

	gb_sensors_create_dbgfs();

	return retval;

error_add_sensor:
	list_for_each_entry_continue_reverse(sensor, sensors_list, node) {
		gb_sensors_rm_sensor(sensor);
		gb_sensors_ext_put();
	}

	return retval;
}

/** This function is called by the Greybus code when a mod is detached. */
int gb_sensors_mod_detached(void)
{
	struct gb_sensor *sensor_info;
	struct list_head *iter;

	pr_info(" IIO Greybus Sensors releasing\n");

	gb_sensors_destroy_dbgfs();

	list_for_each(iter, gb_drv.sensors_list) {
		sensor_info = list_entry(iter, struct gb_sensor, node);
		gb_sensors_rm_sensor(sensor_info);
		gb_sensors_ext_put();
	}

	mutex_destroy(&gb_drv.ilock);

	return 0;
}

