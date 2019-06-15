/*
 * Battery driver for a Greybus module.
 *
 * Copyright 2014 Google Inc.
 * Copyright 2014 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include "greybus.h"

struct gb_battery {
	/*
	 * The power supply api changed in 4.1, so handle both the old
	 * and new apis in the same driver for now, until this is merged
	 * upstream, when all of these version checks can be removed.
	 */
#ifdef DRIVER_OWNS_PSY_STRUCT
	struct power_supply bat;
#define to_gb_battery(x) container_of(x, struct gb_battery, bat)
#else
	struct power_supply *bat;
	struct power_supply_desc desc;
#define to_gb_battery(x) power_supply_get_drvdata(x)
#endif
	struct mutex conn_lock;
	// FIXME
	// we will want to keep the battery stats in here as we will be getting
	// updates from the SVC "on the fly" so we don't have to always go ask
	// the battery for some information.  Hopefully...
	struct gb_connection *connection;

};

static int get_tech(struct gb_battery *gb, int *val)
{
	struct gb_battery_technology_response tech_response;
	u32 technology;
	int retval;

	retval = gb_operation_sync(gb->connection, GB_BATTERY_TYPE_TECHNOLOGY,
				   NULL, 0,
				   &tech_response, sizeof(tech_response));
	if (retval)
		return retval;

	/*
	 * Map greybus values to power_supply values.  Hopefully these are
	 * "identical" which should allow gcc to optimize the code away to
	 * nothing.
	 */
	technology = le32_to_cpu(tech_response.technology);
	switch (technology) {
	case GB_BATTERY_TECH_NiMH:
		technology = POWER_SUPPLY_TECHNOLOGY_NiMH;
		break;
	case GB_BATTERY_TECH_LION:
		technology = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case GB_BATTERY_TECH_LIPO:
		technology = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case GB_BATTERY_TECH_LiFe:
		technology = POWER_SUPPLY_TECHNOLOGY_LiFe;
		break;
	case GB_BATTERY_TECH_NiCd:
		technology = POWER_SUPPLY_TECHNOLOGY_NiCd;
		break;
	case GB_BATTERY_TECH_LiMn:
		technology = POWER_SUPPLY_TECHNOLOGY_LiMn;
		break;
	case GB_BATTERY_TECH_UNKNOWN:
	default:
		technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;
	}
	*val = (int)technology;
	return 0;
}

static int get_status(struct gb_battery *gb, int *val)
{
	struct gb_battery_status_response status_response;
	u16 battery_status;
	int retval;

	retval = gb_operation_sync(gb->connection, GB_BATTERY_TYPE_STATUS,
				   NULL, 0,
				   &status_response, sizeof(status_response));
	if (retval)
		return retval;

	/*
	 * Map greybus values to power_supply values.  Hopefully these are
	 * "identical" which should allow gcc to optimize the code away to
	 * nothing.
	 */
	battery_status = le16_to_cpu(status_response.battery_status);
	switch (battery_status) {
	case GB_BATTERY_STATUS_CHARGING:
		battery_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case GB_BATTERY_STATUS_DISCHARGING:
		battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case GB_BATTERY_STATUS_NOT_CHARGING:
		battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case GB_BATTERY_STATUS_FULL:
		battery_status = POWER_SUPPLY_STATUS_FULL;
		break;
	case GB_BATTERY_STATUS_UNKNOWN:
	default:
		battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}
	*val = (int)battery_status;
	return 0;
}

static int get_max_voltage(struct gb_battery *gb, int *val)
{
	struct gb_battery_max_voltage_response volt_response;
	int retval;

	retval = gb_operation_sync(gb->connection, GB_BATTERY_TYPE_MAX_VOLTAGE,
				   NULL, 0,
				   &volt_response, sizeof(volt_response));
	if (retval)
		return retval;

	*val = (int)le32_to_cpu(volt_response.max_voltage);
	return 0;
}

static int get_percent_capacity(struct gb_battery *gb, int *val)
{
	struct gb_battery_capacity_response capacity_response;
	int retval;

	retval = gb_operation_sync(gb->connection,
				   GB_BATTERY_TYPE_PERCENT_CAPACITY,
				   NULL, 0, &capacity_response,
				   sizeof(capacity_response));
	if (retval)
		return retval;

	*val = (int)le32_to_cpu(capacity_response.capacity);
	return 0;
}

static int get_temp(struct gb_battery *gb, int *val)
{
	struct gb_battery_temperature_response temp_response;
	int retval;

	retval = gb_operation_sync(gb->connection, GB_BATTERY_TYPE_TEMPERATURE,
				   NULL, 0,
				   &temp_response, sizeof(temp_response));
	if (retval)
		return retval;

	*val = (int)le32_to_cpu(temp_response.temperature);
	return 0;
}

static int get_voltage(struct gb_battery *gb, int *val)
{
	struct gb_battery_voltage_response voltage_response;
	int retval;

	retval = gb_operation_sync(gb->connection, GB_BATTERY_TYPE_VOLTAGE,
				   NULL, 0,
				   &voltage_response, sizeof(voltage_response));
	if (retval)
		return retval;

	*val = (int)le32_to_cpu(voltage_response.voltage);
	return 0;
}

static int get_capacity(struct gb_battery *gb, int *val)
{
	struct gb_battery_full_capacity_response full_capacity_response;
	int retval;

	retval = gb_operation_sync(gb->connection,
				   GB_BATTERY_TYPE_CAPACITY,
				   NULL, 0, &full_capacity_response,
				   sizeof(full_capacity_response));
	if (retval)
		return retval;

	*val = (int)le32_to_cpu(full_capacity_response.full_capacity);
	return 0;
}

static int get_current_now(struct gb_battery *gb, int *val)
{
	struct gb_battery_current_now_response current_now_response;
	int retval;

	retval = gb_operation_sync(gb->connection,
				   GB_BATTERY_TYPE_CURRENT,
				   NULL, 0, &current_now_response,
				   sizeof(current_now_response));
	if (retval)
		return retval;

	*val = (int)le32_to_cpu(current_now_response.current_now);
	return 0;
}

static int get_property(struct power_supply *b,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int retval;

	struct gb_battery *gb = to_gb_battery(b);

	mutex_lock(&gb->conn_lock);
	if (!gb->connection) {
		mutex_unlock(&gb->conn_lock);
		pr_err("%s: supply already free'd: %s\n",
			__func__, power_supply_name(b));
		return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		retval = get_tech(gb, &val->intval);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		retval = get_status(gb, &val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		retval = get_max_voltage(gb, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		retval = get_percent_capacity(gb, &val->intval);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		retval = get_temp(gb, &val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		retval = get_voltage(gb, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		retval = get_capacity(gb, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		retval = get_current_now(gb, &val->intval);
		break;

	default:
		retval = -EINVAL;
	}
	mutex_unlock(&gb->conn_lock);

	return retval;
}

// FIXME - verify this list, odds are some can be removed and others added.
static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

#ifdef DRIVER_OWNS_PSY_STRUCT
static void gb_batt_psy_release(struct device *dev)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct gb_battery *gb;

	if (!psy)
		return;

	gb = to_gb_battery(psy);
	kfree(gb);
	kfree(dev);
}

static int init_and_register(struct gb_connection *connection,
			     struct gb_battery *gb)
{
	int ret;

	// FIXME - get a better (i.e. unique) name
	// FIXME - anything else needs to be set?
	gb->bat.name		= "gb_battery";
	gb->bat.type		= POWER_SUPPLY_TYPE_BATTERY;
	gb->bat.properties	= battery_props;
	gb->bat.num_properties	= ARRAY_SIZE(battery_props);
	gb->bat.get_property	= get_property;

	ret = power_supply_register(&connection->bundle->dev, &gb->bat);

	if (!ret)
		gb->bat.dev->release = gb_batt_psy_release;

	return ret;
}
#else
static int init_and_register(struct gb_connection *connection,
			     struct gb_battery *gb)
{
	struct power_supply_config cfg = {};

	cfg.drv_data = gb;
	cfg.free_drv_data = true;

	// FIXME - get a better (i.e. unique) name
	// FIXME - anything else needs to be set?
	gb->desc.name		= "gb_battery";
	gb->desc.type		= POWER_SUPPLY_TYPE_BATTERY;
	gb->desc.properties	= battery_props;
	gb->desc.num_properties	= ARRAY_SIZE(battery_props);
	gb->desc.get_property	= get_property;

	gb->bat = power_supply_register(&connection->bundle->dev,
					&gb->desc, &cfg);
	if (IS_ERR(gb->bat))
		return PTR_ERR(gb->bat);

	return 0;
}
#endif

static int gb_battery_connection_init(struct gb_connection *connection)
{
	struct gb_battery *gb;
	int retval;

	gb = kzalloc(sizeof(*gb), GFP_KERNEL);
	if (!gb)
		return -ENOMEM;

	gb->connection = connection;
	connection->private = gb;

	mutex_init(&gb->conn_lock);

	retval = init_and_register(connection, gb);
	if (retval)
		kfree(gb);

	return retval;
}

static void gb_battery_connection_exit(struct gb_connection *connection)
{
	struct gb_battery *gb = connection->private;

	mutex_lock(&gb->conn_lock);
	gb->connection = NULL;
	mutex_unlock(&gb->conn_lock);
#ifdef DRIVER_OWNS_PSY_STRUCT
	power_supply_unregister(&gb->bat);
#else
	power_supply_unregister(gb->bat);
#endif
}

static struct gb_protocol battery_protocol = {
	.name			= "battery",
	.id			= GREYBUS_PROTOCOL_BATTERY,
	.major			= GB_BATTERY_VERSION_MAJOR,
	.minor			= GB_BATTERY_VERSION_MINOR,
	.connection_init	= gb_battery_connection_init,
	.connection_exit	= gb_battery_connection_exit,
	.request_recv		= NULL,	/* no incoming requests */
};

gb_protocol_driver(&battery_protocol);

MODULE_LICENSE("GPL v2");
