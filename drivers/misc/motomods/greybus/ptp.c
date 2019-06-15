/*
 * Greybus power transfer protocol (ptp) driver.
 *
 * Copyright 2015 Motorola Mobility, LLC.
 * Copyright 2014 Google Inc.
 * Copyright 2014 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/idr.h>
#include <linux/power_supply.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "greybus.h"

struct gb_ptp {
	struct gb_connection	*connection;
	struct mutex		conn_lock;
#ifdef DRIVER_OWNS_PSY_STRUCT
	struct power_supply psy;
#define to_gb_ptp(x) container_of(x, struct gb_ptp, psy)
#define power_supply_ptr(x) (&x->psy)
#else
	struct power_supply *psy;
	struct power_supply_desc desc;
#define to_gb_ptp(x) power_supply_get_drvdata(x)
#define power_supply_ptr(x) (x->psy)
#endif
};

static enum power_supply_property gb_ptp_props[] = {
	POWER_SUPPLY_PROP_PTP_INTERNAL_SEND,
	POWER_SUPPLY_PROP_PTP_INTERNAL_RECEIVE,
	POWER_SUPPLY_PROP_PTP_EXTERNAL,
	POWER_SUPPLY_PROP_PTP_CURRENT_FLOW,
	POWER_SUPPLY_PROP_PTP_MAX_INPUT_CURRENT,
	POWER_SUPPLY_PROP_PTP_MAX_OUTPUT_CURRENT,
	POWER_SUPPLY_PROP_PTP_EXTERNAL_PRESENT,
	POWER_SUPPLY_PROP_PTP_POWER_REQUIRED,
	POWER_SUPPLY_PROP_PTP_POWER_AVAILABLE,
	POWER_SUPPLY_PROP_PTP_POWER_SOURCE,
	POWER_SUPPLY_PROP_PTP_MAX_OUTPUT_VOLTAGE,
	POWER_SUPPLY_PROP_PTP_OUTPUT_VOLTAGE,
	POWER_SUPPLY_PROP_PTP_MAX_INPUT_VOLTAGE,
	POWER_SUPPLY_PROP_PTP_INPUT_VOLTAGE,
};

/* Internal structure */
struct gb_ptp_functionality {
	int int_snd;
	int int_rcv;
	int ext;
};

static int to_internal_send_property(__u8 int_snd)
{
	int prop;

	switch (int_snd) {
	case GB_PTP_INT_SND_NEVER:
		prop = POWER_SUPPLY_PTP_INT_SND_NEVER;
		break;
	case GB_PTP_INT_SND_SUPPLEMENTAL:
		prop = POWER_SUPPLY_PTP_INT_SND_SUPPLEMENTAL;
		break;
	case GB_PTP_INT_SND_LOW_BATT_SAVER:
		prop = POWER_SUPPLY_PTP_INT_SND_LOW_BATT_SAVER;
		break;
	default:
		prop = POWER_SUPPLY_PTP_INT_SND_UNKNOWN;
		break;
	}

	return prop;
}

static int to_internal_receive_property(__u8 int_rcv)
{
	int prop;

	switch (int_rcv) {
	case GB_PTP_INT_RCV_NEVER:
		prop = POWER_SUPPLY_PTP_INT_RCV_NEVER;
		break;
	case GB_PTP_INT_RCV_FIRST:
		prop = POWER_SUPPLY_PTP_INT_RCV_FIRST;
		break;
	case GB_PTP_INT_RCV_SECOND:
		prop = POWER_SUPPLY_PTP_INT_RCV_SECOND;
		break;
	case GB_PTP_INT_RCV_PARALLEL:
		prop = POWER_SUPPLY_PTP_INT_RCV_PARALLEL;
		break;
	default:
		prop = POWER_SUPPLY_PTP_INT_RCV_UNKNOWN;
		break;
	}

	return prop;
}

static int to_external_property(__u8 ext)
{
	int prop;

	switch (ext) {
	case GB_PTP_EXT_NONE:
		prop = POWER_SUPPLY_PTP_EXT_NOT_SUPPORTED;
		break;
	case GB_PTP_EXT_SUPPORTED:
		prop = POWER_SUPPLY_PTP_EXT_SUPPORTED;
		break;
	default:
		prop = POWER_SUPPLY_PTP_EXT_SUPPORT_UNKNOWN;
		break;
	}

	return prop;
}

static int to_external_present_property(__u8 present)
{
	int prop;

	switch (present) {
	case GB_PTP_EXT_POWER_NOT_PRESENT:
		prop = POWER_SUPPLY_PTP_EXT_NOT_PRESENT;
		break;
	case GB_PTP_EXT_POWER_WIRED_PRESENT:
		prop = POWER_SUPPLY_PTP_EXT_WIRED_PRESENT;
		break;
	case GP_PTP_EXT_POWER_PRESENT: /* default to wireless when unknown */
	case GB_PTP_EXT_POWER_WIRELESS_PRESENT:
		prop = POWER_SUPPLY_PTP_EXT_WIRELESS_PRESENT;
		break;
	case GB_PTP_EXT_POWER_WIRED_WIRELESS_PRESENT:
		prop = POWER_SUPPLY_PTP_EXT_WIRED_WIRELESS_PRESENT;
		break;
	default:
		prop = POWER_SUPPLY_PTP_EXT_PRESENCE_UNKNOWN;
		break;
	}

	return prop;
}

static int to_power_required_property(__u8 required)
{
	int prop;

	switch (required) {
	case GB_PTP_POWER_NOT_REQUIRED:
		prop = POWER_SUPPLY_PTP_POWER_NOT_REQUIRED;
		break;
	case GB_PTP_POWER_REQUIRED:
		prop = POWER_SUPPLY_PTP_POWER_REQUIRED;
		break;
	default:
		prop = POWER_SUPPLY_PTP_POWER_REQUIREMENTS_UNKNOWN;
		break;
	}

	return prop;
}

static int to_power_available_property(__u8 available)
{
	int prop;

	switch (available) {
	case GB_PTP_POWER_NOT_AVAILABLE:
		prop = POWER_SUPPLY_PTP_POWER_NOT_AVAILABLE;
		break;
	case GB_PTP_POWER_AVAILABLE_EXT:
		prop = POWER_SUPPLY_PTP_POWER_AVAILABLE_EXTERNAL;
		break;
	case GB_PTP_POWER_AVAILABLE_INT:
		prop = POWER_SUPPLY_PTP_POWER_AVAILABLE_INTERNAL;
		break;
	default:
		prop = POWER_SUPPLY_PTP_POWER_AVAILABILITY_UNKNOWN;
		break;
	}

	return prop;
}

static int to_power_source_property(__u8 source)
{
	int prop;

	switch (source) {
	case GB_PTP_POWER_SOURCE_NONE:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_NONE;
		break;
	case GB_PTP_POWER_SOURCE_BATTERY:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_BATTERY;
		break;
	case GB_PTP_POWER_SOURCE_WIRED:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_WIRED;
		break;
	case GB_PTP_POWER_SOURCE_WIRELESS:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_WIRELESS;
		break;
	case GB_PTP_POWER_SOURCE_NONE_TURBO:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_NONE_TURBO;
		break;
	case GB_PTP_POWER_SOURCE_BATTERY_TURBO:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_BATTERY_TURBO;
		break;
	case GB_PTP_POWER_SOURCE_WIRED_TURBO:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_WIRED_TURBO;
		break;
	case GB_PTP_POWER_SOURCE_WIRELESS_TURBO:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_WIRELESS_TURBO;
		break;
	default:
		prop = POWER_SUPPLY_PTP_POWER_SOURCE_UNKNOWN;
		break;
	}

	return prop;
}

static int gb_ptp_get_functionality(struct gb_ptp *ptp,
				    struct gb_ptp_functionality *func)
{
	struct gb_ptp_functionality_response response;
	int retval = gb_operation_sync(ptp->connection,
				       GB_PTP_TYPE_GET_FUNCTIONALITY, NULL, 0,
				       &response, sizeof(response));
	if (retval)
		return retval;

	func->int_snd = to_internal_send_property(response.int_snd);
	func->int_rcv = to_internal_receive_property(response.int_rcv);
	func->ext = to_external_property(response.ext);

	return 0;
}

static int gb_ptp_get_max_output_current(struct gb_ptp *ptp, int *curr)
{
	struct gb_ptp_max_output_current_response response;
	int retval;

	if (!GB_PTP_SUPPORTS(ptp, MAX_OUTPUT_CURRENT))
		return -ENODEV;

	retval = gb_operation_sync(ptp->connection,
				   GB_PTP_TYPE_GET_MAX_OUTPUT_CURRENT, NULL, 0,
				   &response, sizeof(response));
	if (retval)
		return retval;

	*curr = le32_to_cpu(response.curr);
	return 0;
}

static int gb_ptp_ext_power_present(struct gb_ptp *ptp, int *present)
{
	struct gb_ptp_ext_power_present_response response;
	int retval = gb_operation_sync(ptp->connection,
				       GB_PTP_TYPE_EXT_POWER_PRESENT, NULL, 0,
				       &response, sizeof(response));

	if (retval)
		return retval;

	*present = to_external_present_property(response.present);
	return 0;
}

static int gb_ptp_power_required(struct gb_ptp *ptp, int *required)
{
	struct gb_ptp_power_required_response response;
	int retval = gb_operation_sync(ptp->connection,
				       GB_PTP_TYPE_POWER_REQUIRED, NULL, 0,
				       &response, sizeof(response));

	if (retval)
		return retval;

	*required = to_power_required_property(response.required);
	return 0;
}

static int gb_ptp_power_available(struct gb_ptp *ptp, int *available)
{
	struct gb_ptp_power_available_response response;
	int retval;

	if (!GB_PTP_SUPPORTS(ptp, POWER_AVAILABLE)) {
		*available = POWER_SUPPLY_PTP_POWER_AVAILABILITY_UNKNOWN;
		return 0;
	}

	retval = gb_operation_sync(ptp->connection,
				   GB_PTP_TYPE_POWER_AVAILABLE, NULL, 0,
				   &response, sizeof(response));
	if (retval)
		return retval;

	*available = to_power_available_property(response.available);
	return 0;
}

static int gb_ptp_power_source(struct gb_ptp *ptp, int *source)
{
	struct gb_ptp_power_source_response response;
	int retval;

	if (!GB_PTP_SUPPORTS(ptp, POWER_SOURCE)) {
		*source = POWER_SUPPLY_PTP_POWER_SOURCE_UNKNOWN;
		return 0;
	}

	retval = gb_operation_sync(ptp->connection,
				   GB_PTP_TYPE_POWER_SOURCE, NULL, 0,
				   &response, sizeof(response));
	if (retval)
		return retval;

	*source = to_power_source_property(response.source);
	return 0;
}

static int gb_ptp_set_current_flow(struct gb_ptp *ptp, int direction)
{
	struct gb_ptp_current_flow_request request;

	switch (direction) {
	case POWER_SUPPLY_PTP_CURRENT_OFF:
		request.direction = GB_PTP_CURRENT_OFF;
		break;
	case POWER_SUPPLY_PTP_CURRENT_FROM_PHONE:
		request.direction = GB_PTP_CURRENT_TO_MOD;
		break;
	case POWER_SUPPLY_PTP_CURRENT_TO_PHONE:
		request.direction = GB_PTP_CURRENT_FROM_MOD;
		break;
	default:
		return -EINVAL;
	}

	return gb_operation_sync(ptp->connection, GB_PTP_TYPE_SET_CURRENT_FLOW,
				 &request, sizeof(request), NULL, 0);
}

static int gb_ptp_get_current_flow(struct gb_ptp *ptp, int *direction)
{
	struct gb_ptp_current_flow_response response;
	int retval;

	if (!GB_PTP_SUPPORTS(ptp, GET_CURRENT_FLOW))
		return -ENODEV;

	retval = gb_operation_sync(ptp->connection,
				   GB_PTP_TYPE_GET_CURRENT_FLOW, NULL, 0,
				   &response, sizeof(response));
	if (retval)
		return retval;

	switch (response.direction) {
	case GB_PTP_CURRENT_OFF:
		*direction = POWER_SUPPLY_PTP_CURRENT_OFF;
		break;
	case GB_PTP_CURRENT_TO_MOD:
		*direction = POWER_SUPPLY_PTP_CURRENT_FROM_PHONE;
		break;
	case GB_PTP_CURRENT_FROM_MOD:
		*direction = POWER_SUPPLY_PTP_CURRENT_TO_PHONE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int gb_ptp_set_maximum_input_current(struct gb_ptp *ptp, int curr)
{
	struct gb_ptp_max_input_current_request request;

	request.curr = cpu_to_le32((u32)curr);
	return gb_operation_sync(ptp->connection,
				 GB_PTP_TYPE_SET_MAX_INPUT_CURRENT, &request,
				 sizeof(request), NULL, 0);
}

static int gb_ptp_set_maximum_output_voltage(struct gb_ptp *ptp, int voltage)
{
	struct gb_ptp_max_output_voltage_request request;

	if (!GB_PTP_SUPPORTS(ptp, SET_MAX_OUTPUT_VOLTAGE))
		return 0;

	request.voltage = cpu_to_le32((u32)voltage);
	return gb_operation_sync(ptp->connection,
				 GB_PTP_TYPE_SET_MAX_OUTPUT_VOLTAGE, &request,
				 sizeof(request), NULL, 0);
}

static int gb_ptp_get_output_voltage(struct gb_ptp *ptp, int *voltage)
{
	struct gb_ptp_output_voltage_response response;
	int retval;

	if (!GB_PTP_SUPPORTS(ptp, GET_OUTPUT_VOLTAGE)) {
		*voltage = GB_PTP_VARIABLE_VOLTAGE_NOT_SUPPORTED;
		return 0;
	}

	retval = gb_operation_sync(ptp->connection,
				   GB_PTP_TYPE_GET_OUTPUT_VOLTAGE, NULL, 0,
				   &response, sizeof(response));
	if (retval)
		return retval;

	*voltage = le32_to_cpu(response.voltage);
	return 0;
}

static int gb_ptp_get_max_input_voltage(struct gb_ptp *ptp, int *voltage)
{
	struct gb_ptp_max_input_voltage_response response;
	int retval;

	if (!GB_PTP_SUPPORTS(ptp, GET_MAX_INPUT_VOLTAGE)) {
		*voltage = GB_PTP_VARIABLE_VOLTAGE_NOT_SUPPORTED;
		return 0;
	}

	retval = gb_operation_sync(ptp->connection,
				   GB_PTP_TYPE_GET_MAX_INPUT_VOLTAGE, NULL, 0,
				   &response, sizeof(response));
	if (retval)
		return retval;

	*voltage = le32_to_cpu(response.voltage);
	return 0;
}

static int gb_ptp_set_input_voltage(struct gb_ptp *ptp, int voltage)
{
	struct gb_ptp_input_voltage_request request;

	if (!GB_PTP_SUPPORTS(ptp, SET_INPUT_VOLTAGE))
		return voltage == GB_PTP_VARIABLE_VOLTAGE_NOT_SUPPORTED ?
			0 : -EINVAL;

	request.voltage = cpu_to_le32((u32)voltage);
	return gb_operation_sync(ptp->connection,
				 GB_PTP_TYPE_SET_INPUT_VOLTAGE, &request,
				 sizeof(request), NULL, 0);
}

static int gb_ptp_receive(u8 type, struct gb_operation *op)
{
	struct gb_connection *connection = op->connection;
	struct gb_ptp *ptp = connection->private;

	if (!ptp) {
		pr_warn("%s: connection not initialized, type = %d\n",
			__func__, type);
		return -EAGAIN;
	}

	switch (type) {
	case GB_PTP_TYPE_POWER_AVAILABLE_CHANGED:
		if (!GB_PTP_SUPPORTS(ptp, POWER_AVAILABLE_CHANGED))
			return -EINVAL;
	case GB_PTP_TYPE_EXT_POWER_CHANGED:
	case GB_PTP_TYPE_POWER_REQUIRED_CHANGED:
		if (!power_supply_ptr(ptp)) {
			pr_warn("%s: psy not initialized\n",
				__func__);
			return -EAGAIN;
		}
		power_supply_changed(power_supply_ptr(ptp));
		return 0;
	default:
		return -EINVAL;
	}
}

static int gb_ptp_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct gb_ptp *ptp = to_gb_ptp(psy);
	struct gb_ptp_functionality func;
	int retval;

	mutex_lock(&ptp->conn_lock);
	if (!ptp->connection) {
		mutex_unlock(&ptp->conn_lock);
		pr_warn("%s: supply already free'd: %s\n",
			__func__, power_supply_name(psy));
		return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PTP_INTERNAL_SEND:
		retval = gb_ptp_get_functionality(ptp, &func);
		if (!retval)
			val->intval = func.int_snd;
		break;
	case POWER_SUPPLY_PROP_PTP_INTERNAL_RECEIVE:
		retval = gb_ptp_get_functionality(ptp, &func);
		if (!retval)
			val->intval = func.int_rcv;
		break;
	case POWER_SUPPLY_PROP_PTP_EXTERNAL:
		retval = gb_ptp_get_functionality(ptp, &func);
		if (!retval)
			val->intval = func.ext;
		break;
	case POWER_SUPPLY_PROP_PTP_CURRENT_FLOW:
		retval = gb_ptp_get_current_flow(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_MAX_INPUT_CURRENT:
		retval = -ENODEV; /* to make power_supply_uevent() happy */
		break;
	case POWER_SUPPLY_PROP_PTP_MAX_OUTPUT_CURRENT:
		retval = gb_ptp_get_max_output_current(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_EXTERNAL_PRESENT:
		retval = gb_ptp_ext_power_present(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_POWER_REQUIRED:
		retval = gb_ptp_power_required(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_POWER_AVAILABLE:
		retval = gb_ptp_power_available(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_POWER_SOURCE:
		retval = gb_ptp_power_source(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_MAX_OUTPUT_VOLTAGE:
		retval = -ENODEV; /* to make power_supply_uevent() happy */
		break;
	case POWER_SUPPLY_PROP_PTP_OUTPUT_VOLTAGE:
		retval = gb_ptp_get_output_voltage(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_MAX_INPUT_VOLTAGE:
		retval = gb_ptp_get_max_input_voltage(ptp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_INPUT_VOLTAGE:
		retval = -ENODEV; /* to make power_supply_uevent() happy */
		break;
	default:
		retval = -EINVAL;
	}

	mutex_unlock(&ptp->conn_lock);

	return retval;
}

static int gb_ptp_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct gb_ptp *ptp = to_gb_ptp(psy);
	int retval;

	mutex_lock(&ptp->conn_lock);
	if (!ptp->connection) {
		mutex_unlock(&ptp->conn_lock);
		pr_warn("%s: supply already free'd: %s\n",
			__func__, power_supply_name(psy));
		return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PTP_CURRENT_FLOW:
		retval = gb_ptp_set_current_flow(ptp, val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_MAX_INPUT_CURRENT:
		retval = gb_ptp_set_maximum_input_current(ptp, val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_MAX_OUTPUT_VOLTAGE:
		retval = gb_ptp_set_maximum_output_voltage(ptp, val->intval);
		break;
	case POWER_SUPPLY_PROP_PTP_INPUT_VOLTAGE:
		retval = gb_ptp_set_input_voltage(ptp, val->intval);
		break;
	default:
		retval = -EINVAL;
	}

	mutex_unlock(&ptp->conn_lock);

	return retval;
}

static int gb_ptp_property_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_PTP_CURRENT_FLOW ||
	       psp == POWER_SUPPLY_PROP_PTP_MAX_INPUT_CURRENT ||
	       psp == POWER_SUPPLY_PROP_PTP_MAX_OUTPUT_VOLTAGE ||
	       psp == POWER_SUPPLY_PROP_PTP_INPUT_VOLTAGE;
}

#ifdef DRIVER_OWNS_PSY_STRUCT
static void gb_ptp_psy_release(struct device *dev)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct gb_ptp *ptp;

	if (!psy)
		return;

	ptp = to_gb_ptp(psy);
	kfree(ptp);
	kfree(dev);
}

static int
init_and_register(struct gb_connection *connection, struct gb_ptp *ptp)
{
	int retval;

	/* Create a power supply */
	ptp->psy.name		= "gb_ptp";
	ptp->psy.type		= POWER_SUPPLY_TYPE_PTP;
	ptp->psy.properties	= gb_ptp_props;
	ptp->psy.num_properties	= ARRAY_SIZE(gb_ptp_props);
	ptp->psy.get_property	= gb_ptp_get_property;
	ptp->psy.set_property	= gb_ptp_set_property;
	ptp->psy.property_is_writeable = gb_ptp_property_is_writeable;

	retval = power_supply_register(&connection->bundle->intf->dev,
				       &ptp->psy);
	if (retval)
		goto error;

	ptp->psy.dev->release = gb_ptp_psy_release;

error:
	return retval;
}
#else
static int
init_and_register(struct gb_connection *connection, struct gb_ptp *ptp)
{
	struct power_supply_config cfg = {};

	cfg.drv_data = ptp;
	cfg.free_drv_data = true;

	/* Create a power supply */
	ptp->desc.name		= "gb_ptp";
	ptp->desc.type		= POWER_SUPPLY_TYPE_PTP;
	ptp->desc.properties	= gb_ptp_props;
	ptp->desc.num_properties	= ARRAY_SIZE(gb_ptp_props);
	ptp->desc.get_property	= gb_ptp_get_property;
	ptp->desc.set_property	= gb_ptp_set_property;
	ptp->desc.property_is_writeable = gb_ptp_property_is_writeable;

	ptp->psy = power_supply_register(&connection->bundle->dev,
					&ptp->desc, &cfg);
	if (IS_ERR(ptp->psy))
		return PTR_ERR(ptp->psy);

	return 0;
}
#endif




static int gb_ptp_connection_init(struct gb_connection *connection)
{
	struct gb_ptp *ptp;
	int retval;

	ptp = kzalloc(sizeof(*ptp), GFP_KERNEL);
	if (!ptp)
		return -ENOMEM;

	ptp->connection = connection;
	mutex_init(&ptp->conn_lock);

	connection->private = ptp;

	retval = init_and_register(connection, ptp);
	if (retval)
		goto error;

	return 0;

error:
	kfree(ptp);
	return retval;
}

static void gb_ptp_connection_exit(struct gb_connection *connection)
{
	struct gb_ptp *ptp = connection->private;

	mutex_lock(&ptp->conn_lock);
	ptp->connection = NULL;
	mutex_unlock(&ptp->conn_lock);
#ifdef DRIVER_OWNS_PSY_STRUCT
	power_supply_unregister(&ptp->psy);
#else
	power_supply_unregister(ptp->psy);
#endif
}

static struct gb_protocol ptp_protocol = {
	.name			= "ptp",
	.id			= GREYBUS_PROTOCOL_PTP,
	.major			= GB_PTP_VERSION_MAJOR,
	.minor			= GB_PTP_VERSION_MINOR,
	.connection_init	= gb_ptp_connection_init,
	.connection_exit	= gb_ptp_connection_exit,
	.request_recv		= gb_ptp_receive,
};

gb_protocol_driver(&ptp_protocol);

MODULE_LICENSE("GPL v2");
