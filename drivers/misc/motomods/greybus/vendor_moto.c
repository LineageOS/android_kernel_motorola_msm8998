/*
 * Motorola specific driver for a Greybus module.
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
#include <linux/module.h>
#include <linux/slab.h>
#include "greybus.h"

struct gb_vendor_moto {
	struct gb_connection *connection;
	struct device *dev;
	int minor;  /* vendor minor number */
	uint16_t dmesg_size;
};

static ssize_t do_get_dmesg(struct device *dev, struct device_attribute *attr,
			    char *buf, int type)
{
	struct gb_vendor_moto *gb = dev_get_drvdata(dev);
	char *dmesg;
	int ret;

	dmesg = kmalloc(gb->dmesg_size, GFP_KERNEL);
	if (!dmesg)
		return -ENOMEM;

	ret = gb_operation_sync(gb->connection, type,
				NULL, 0, dmesg, gb->dmesg_size);
	if (ret)
		goto err;

	ret = scnprintf(buf, gb->dmesg_size, "%s\n", dmesg);

err:
	kfree(dmesg);

	return ret;
}

static int do_get_uptime(struct gb_vendor_moto *gb, unsigned int *uptime)
{
	struct gb_vendor_moto_get_uptime_response rsp;
	int ret;

	if (gb->connection->module_minor < GB_VENDOR_MOTO_VER_UPTIME)
		return -EOPNOTSUPP;

	ret = gb_operation_sync(gb->connection,
				GB_VENDOR_MOTO_TYPE_GET_UPTIME,
				NULL, 0, &rsp, sizeof(rsp));
	if (ret)
		return ret;

	*uptime = le32_to_cpu(rsp.secs);

	return 0;
}

static int do_get_pwrup(struct gb_vendor_moto *gb, unsigned int *pwrup)
{
	struct gb_vendor_moto_pwr_up_reason_response rsp;
	int ret;

	ret = gb_operation_sync(gb->connection,
				GB_VENDOR_MOTO_TYPE_GET_PWR_UP_REASON,
				NULL, 0, &rsp, sizeof(rsp));
	if (ret)
		return ret;

	*pwrup = le32_to_cpu(rsp.reason);

	return 0;
}

static ssize_t dmesg_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return do_get_dmesg(dev, attr, buf, GB_VENDOR_MOTO_TYPE_GET_DMESG);
}
static DEVICE_ATTR_RO(dmesg);

static ssize_t last_dmesg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return do_get_dmesg(dev, attr, buf, GB_VENDOR_MOTO_TYPE_GET_LAST_DMESG);
}
static DEVICE_ATTR_RO(last_dmesg);

static ssize_t pwr_up_reason_show(struct device *dev,
			          struct device_attribute *attr, char *buf)
{
	struct gb_vendor_moto *gb = dev_get_drvdata(dev);
	unsigned int pwrup;
	int ret;

	ret = do_get_pwrup(gb, &pwrup);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "0x%08X\n", pwrup);
}
static DEVICE_ATTR_RO(pwr_up_reason);

static ssize_t uptime_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct gb_vendor_moto *gb = dev_get_drvdata(dev);
	unsigned int uptime;
	int ret;

	ret = do_get_uptime(gb, &uptime);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d sec\n", uptime);
}
static DEVICE_ATTR_RO(uptime);

static struct attribute *vendor_attrs[] = {
	&dev_attr_dmesg.attr,
	&dev_attr_last_dmesg.attr,
	&dev_attr_pwr_up_reason.attr,
	&dev_attr_uptime.attr,
	NULL,
};
ATTRIBUTE_GROUPS(vendor);

static struct class vendor_class = {
	.name		= "vendor",
	.owner		= THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,11,0)
	.dev_groups	= vendor_groups,
#endif
};

static DEFINE_IDA(minors);

static inline void gb_vendor_moto_print_status(struct gb_vendor_moto *gb)
{
	unsigned int pwr;
	unsigned int uptime;
	int ret;

	ret = do_get_pwrup(gb, &pwr);
	if (ret) {
		dev_err(gb->dev, "Failed to read powerup reason: %d\n", ret);
		return;
	}

	ret = do_get_uptime(gb, &uptime);

	/* We're done as uptime is not supported. Print the info we have */
	if (ret == -EOPNOTSUPP) {
		dev_info(gb->dev, "power up reason: %08X\n", pwr);
		return;
	}

	if (ret) {
		dev_err(gb->dev, "Failed to read uptime: %d\n", ret);
		return;
	}

	dev_info(gb->dev, "power up reason: %08X uptime: %us\n", pwr, uptime);
}

static int gb_vendor_moto_connection_init(struct gb_connection *connection)
{
	struct gb_vendor_moto *gb;
	struct device *dev;
	int retval;
	struct gb_vendor_moto_get_dmesg_size_resp *rsp;

	gb = kzalloc(sizeof(*gb), GFP_KERNEL);
	if (!gb)
		return -ENOMEM;

	gb->connection = connection;
	connection->private = gb;

	gb->dmesg_size = GB_VENDOR_MOTO_DEFAULT_DMESG_SIZE;
	if (connection->module_minor >= GB_VENDOR_MOTO_VER_DMESG_SIZE) {
		rsp = kmalloc(sizeof(struct gb_vendor_moto_get_dmesg_size_resp),
			      GFP_KERNEL);
		if (!rsp) {
			retval = -ENOMEM;
			goto error;
		}

		retval = gb_operation_sync(gb->connection,
					   GB_VENDOR_MOTO_TYPE_GET_DMESG_SIZE,
					   NULL, 0, rsp, sizeof(*rsp));
		if (retval) {
			kfree(rsp);
			goto error;
		}

		gb->dmesg_size = le16_to_cpu(rsp->size);

		kfree(rsp);
	}

	dev_info(&connection->bundle->dev, "module_minor=%d, dmesg_size=%d\n",
		 connection->module_minor, gb->dmesg_size);

	/* Create a device in sysfs */
	gb->minor = ida_simple_get(&minors, 0, 0, GFP_KERNEL);
	if (gb->minor < 0) {
		retval = gb->minor;
		goto error;
	}
	dev = device_create(&vendor_class, &connection->bundle->dev,
				MKDEV(0, 0), gb, "mod%d", gb->minor);
	if (IS_ERR(dev)) {
		retval = -EINVAL;
		goto err_ida_remove;
	}
	gb->dev = dev;

	gb_vendor_moto_print_status(gb);

	return 0;

err_ida_remove:
	ida_simple_remove(&minors, gb->minor);
error:
	kfree(gb);
	return retval;
}

static void gb_vendor_moto_connection_exit(struct gb_connection *connection)
{
	struct gb_vendor_moto *gb = connection->private;

	ida_simple_remove(&minors, gb->minor);
	device_unregister(gb->dev);
	kfree(gb);
}

static struct gb_protocol vendor_moto_protocol = {
	.name			= "vendor-moto",
	.id			= GREYBUS_PROTOCOL_VENDOR,
	.major			= GB_VENDOR_MOTO_VERSION_MAJOR,
	.minor			= GB_VENDOR_MOTO_VERSION_MINOR,
	.connection_init	= gb_vendor_moto_connection_init,
	.connection_exit	= gb_vendor_moto_connection_exit,
	.request_recv		= NULL,	/* no incoming requests */
};

static __init int protocol_init(void)
{
	int retval;

	retval = class_register(&vendor_class);
	if (retval)
		return retval;

	return gb_protocol_register(&vendor_moto_protocol);
}
module_init(protocol_init);

static __exit void protocol_exit(void)
{
	gb_protocol_deregister(&vendor_moto_protocol);
	class_unregister(&vendor_class);
	ida_destroy(&minors);
}
module_exit(protocol_exit);

MODULE_LICENSE("GPL v2");
