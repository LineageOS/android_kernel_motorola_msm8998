/*
 * Greybus sensors extension protocol driver.
 *
 * Copyright 2015-2016 Motorola Mobility, LLC.
 * Copyright 2014 Google Inc.
 * Copyright 2014 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/device.h>
#include <linux/idr.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "greybus.h"
#include "sensors_ext.h"
#include "sensors_ext_iio.h"


struct gb_sensors_ext {
	struct list_head	sensors_list;
	struct gb_connection	*connection;
	u8 			sensors_cnt;
	struct mutex		mlock;
	struct kref		refcount;
};

static struct gb_sensors_ext *sensors_ext;
static DEFINE_SPINLOCK(se_lock);

static void gb_sensors_ext_kref_release(struct kref *kref)
{
	struct gb_sensors_ext *gb_sense;

	gb_sense = container_of(kref, struct gb_sensors_ext, refcount);
	mutex_destroy(&gb_sense->mlock);
	kfree(gb_sense);
	sensors_ext = NULL;
}

void gb_sensors_ext_get(void)
{
	unsigned long flags;

	if (!sensors_ext)
		return;

	gb_connection_get(sensors_ext->connection);
	spin_lock_irqsave(&se_lock, flags);
	kref_get(&sensors_ext->refcount);
	spin_unlock_irqrestore(&se_lock, flags);
}

void gb_sensors_ext_put(void)
{
	unsigned long flags;
	struct gb_connection *conn;

	if (!sensors_ext)
		return;

	conn = sensors_ext->connection;
	spin_lock_irqsave(&se_lock, flags);
	kref_put(&sensors_ext->refcount, gb_sensors_ext_kref_release);
	spin_unlock_irqrestore(&se_lock, flags);
	gb_connection_put(conn);
}

struct gb_sensor *get_sensor_from_id(uint8_t sensor_id)
{
	struct gb_sensor *sensor_info;
	struct list_head *iter;

	list_for_each(iter, &sensors_ext->sensors_list) {
		sensor_info = list_entry(iter, struct gb_sensor, node);
		if (sensor_info->sensor_id == sensor_id)
			return sensor_info;
	}

	return NULL;
}

static int gb_sensors_ext_event_receive(u8 type, struct gb_operation *op)
{
	struct gb_connection *connection = op->connection;
	struct gb_message *request;
	int ret;

	if (!connection->private) {
		dev_err(&connection->bundle->dev,
			"sensor ext initialization incomplete\n");
		return -EAGAIN;
	}

	mutex_lock(&sensors_ext->mlock);

	if (type != GB_SENSORS_EXT_TYPE_EVENT) {
		dev_err(&connection->bundle->dev,
			"Unsupported unsolicited event: %u\n", type);
		ret = -EINVAL;
		goto error;
	}

	request = op->request;

	dev_dbg(&connection->bundle->dev,
		"payload size received %zu\n", request->payload_size);

	ret = gb_sensors_rcv_data(request->payload, request->payload_size);

error:
	mutex_unlock(&sensors_ext->mlock);
	return ret;

}

static int gb_sensors_ext_get_count(struct gb_sensors_ext *sensors_ext)
{
	struct gb_sensors_get_sensor_count_response resp;
	int ret;

	/* get sensor count */
	ret = gb_operation_sync(sensors_ext->connection,
				GB_SENSORS_EXT_TYPE_SENSOR_COUNT,
				NULL, 0, &resp, sizeof(resp));
	if (ret)
		return ret;

	if (!resp.sensors_count)
		return -EINVAL;

	sensors_ext->sensors_cnt = resp.sensors_count;

	return ret;
}

int gb_sensors_ext_start_reporting(uint8_t sensor_id, uint64_t sampling_period,
					uint64_t max_report_latency)
{
	struct gb_sensors_ext_start_reporting_request req;
	int ret;

	req.sensor_id = sensor_id;
	req.sampling_period = cpu_to_le64(sampling_period);
	req.max_report_latency = cpu_to_le64(max_report_latency);

	ret = gb_operation_sync(sensors_ext->connection,
				GB_SENSORS_EXT_TYPE_START_REPORTING,
				&req, sizeof(req), NULL, 0);

	return ret;
}

int gb_sensors_ext_stop_reporting(uint8_t sensor_id)
{
	struct gb_sensors_ext_stop_reporting_request req;
	int ret;

	req.sensor_id = sensor_id;

	ret = gb_operation_sync(sensors_ext->connection,
				GB_SENSORS_EXT_TYPE_STOP_REPORTING,
				&req, sizeof(req), NULL, 0);

	return ret;
}

int gb_sensors_ext_flush(uint8_t sensor_id)
{
	struct gb_sensors_ext_flush_request req;
	int ret;

	req.sensor_id = sensor_id;

	ret = gb_operation_sync(sensors_ext->connection,
				GB_SENSORS_EXT_TYPE_FLUSH,
				&req, sizeof(req), NULL, 0);
	if (ret)
		return ret;

	return ret;
}

static int gb_sensors_ext_config(struct gb_sensors_ext *sensors_ext, u8 id)
{
	struct gb_sensor *sensor;
	struct gb_sensors_ext_sensor_info_request req;
	struct gb_sensor conf;
	int ret;

	req.sensor_id = id;

	ret = gb_operation_sync(sensors_ext->connection,
				GB_SENSORS_EXT_TYPE_SENSOR_INFO,
				&req, sizeof(req), &conf,
				offsetof(struct gb_sensor, END_OF_GB_STRUCT));
	if (ret)
		return ret;

	sensor = get_sensor_from_id(id);

	sensor->version = le32_to_cpu(conf.version);
	sensor->type = le32_to_cpu(conf.type);
	sensor->max_range = le32_to_cpu(conf.max_range);
	sensor->resolution = le32_to_cpu(conf.resolution);
	sensor->power = le32_to_cpu(conf.power);
	sensor->min_delay = le32_to_cpu(conf.min_delay);
	sensor->max_delay = le32_to_cpu(conf.max_delay);
	sensor->fifo_rec = le32_to_cpu(conf.fifo_rec);
	sensor->fifo_mec = le32_to_cpu(conf.fifo_mec);
	sensor->flags = le32_to_cpu(conf.flags);
	sensor->scale_int = le32_to_cpu(conf.scale_int);
	sensor->scale_nano = le32_to_cpu(conf.scale_nano);
	sensor->offset_int = le32_to_cpu(conf.offset_int);
	sensor->offset_nano = le32_to_cpu(conf.offset_nano);
	sensor->channels = conf.channels;

	sensor->name_len = le16_to_cpu(conf.name_len);
	if (sensor->name_len) {
		if (sensor->name_len > sizeof(sensor->name))
			sensor->name_len = sizeof(sensor->name);
		memcpy(&sensor->name[0], &conf.name[0], sensor->name_len);
	}

	sensor->vendor_len = le16_to_cpu(conf.vendor_len);
	if (sensor->vendor_len) {
		if (sensor->vendor_len > sizeof(sensor->vendor))
			sensor->vendor_len = sizeof(sensor->vendor);
		memcpy(&sensor->vendor[0], &conf.vendor[0], sensor->vendor_len);
	}

	sensor->string_type_len = le16_to_cpu(conf.string_type_len);
	if (sensor->string_type_len) {
		if (sensor->string_type_len > sizeof(sensor->string_type))
			sensor->string_type_len = sizeof(sensor->string_type);
		memcpy(&sensor->string_type[0], &conf.string_type[0],
			sensor->string_type_len);
	}

    return 0;
}

static int gb_sensors_ext_setup(struct gb_sensors_ext *sensors_ext)
{
	struct gb_connection *connection = sensors_ext->connection;
	int ret, i;
	struct list_head *iter, *next;
	struct gb_sensor *sensor_info;

	/* get the sensors count */
	ret = gb_sensors_ext_get_count(sensors_ext);
	if (ret)
		goto error_count;

	INIT_LIST_HEAD(&sensors_ext->sensors_list);

	for (i = 0; i < sensors_ext->sensors_cnt; i++) {
		sensor_info = kzalloc(sizeof(struct gb_sensor), GFP_KERNEL);
		if (!sensor_info) {
			ret = -ENOMEM;
			goto error_out;
		}
		list_add(&sensor_info->node, &sensors_ext->sensors_list);
		sensor_info->sensor_id = i;
		ret = gb_sensors_ext_config(sensors_ext, i);
		if (ret) {
			dev_err(&connection->bundle->dev,
				"Failed to config sensor device %d (err=%d)\n",
				i, ret);
			goto error_out;
		}
	}

	ret = gb_sensors_mod_attached(sensors_ext->sensors_cnt,
					&sensors_ext->sensors_list);
	if(ret)
		goto error_out;

	return 0;

error_out:
	list_for_each_safe(iter, next, &sensors_ext->sensors_list) {
		sensor_info = list_entry(iter, struct gb_sensor, node);
		kfree(sensor_info);
	}
error_count:
	return ret;
}

static int gb_sensors_ext_connection_init(struct gb_connection *connection)
{
	int ret;

	/* TODO: allow multiple instance connection */
	if (sensors_ext)
		return -EEXIST;

	sensors_ext = kzalloc(sizeof(*sensors_ext), GFP_KERNEL);
	if (!sensors_ext)
		return -ENOMEM;

	kref_init(&sensors_ext->refcount);
	sensors_ext->connection = connection;
	gb_connection_get(sensors_ext->connection);

	mutex_init(&sensors_ext->mlock);

	/* set up the sensors */
	ret = gb_sensors_ext_setup(sensors_ext);
	if (ret)
		goto error_setup;

	connection->private = sensors_ext;

	dev_info(&connection->bundle->dev, "module_minor=%d, count = %d\n",
		 connection->module_minor, sensors_ext->sensors_cnt);

	return 0;

error_setup:
	gb_sensors_ext_put();
	return ret;
}

static void gb_sensors_ext_connection_exit(struct gb_connection *connection)
{
	struct gb_sensors_ext *gb = connection->private;
	struct list_head *iter, *next;
	struct gb_sensor *sensor_info;

	gb_sensors_mod_detached();

	list_for_each_safe(iter, next, &gb->sensors_list) {
		sensor_info = list_entry(iter, struct gb_sensor, node);
		kfree(sensor_info);
	}

	gb_sensors_ext_put();
}

static struct gb_protocol sensors_ext_protocol = {
	.name			= "sensors-ext",
	.id			= GREYBUS_PROTOCOL_SENSORS_EXT,
	.major			= GB_SENSORS_EXT_VERSION_MAJOR,
	.minor			= GB_SENSORS_EXT_VERSION_MINOR,
	.connection_init	= gb_sensors_ext_connection_init,
	.connection_exit	= gb_sensors_ext_connection_exit,
	.request_recv		= gb_sensors_ext_event_receive,
};

static __init int protocol_init(void)
{
	return gb_protocol_register(&sensors_ext_protocol);
}
module_init(protocol_init);

static __exit void protocol_exit(void)
{
	gb_protocol_deregister(&sensors_ext_protocol);
}
module_exit(protocol_exit);

MODULE_LICENSE("GPL v2");
