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

/* Version of the Greybus protocol we support */
#define	GB_VENDOR_MOTO_VERSION_MAJOR		0x00
#define	GB_VENDOR_MOTO_VERSION_MINOR		0x01  // FIXME: Set to 0x02 when supported

/* Greybus Motorola vendor specific request types */
#define	GB_VENDOR_MOTO_TYPE_GET_DMESG		0x02
#define	GB_VENDOR_MOTO_TYPE_GET_LAST_DMESG	0x03
#define	GB_VENDOR_MOTO_TYPE_GET_PWR_UP_REASON	0x04
#define GB_VENDOR_MOTO_TYPE_GET_DMESG_SIZE	0x05

#define GB_VENDOR_MOTO_DEFAULT_DMESG_SIZE   1000
#define GB_VENDOR_MOTO_VER_DMESG_SIZE       2

/* power up reason request has no payload */
struct gb_vendor_moto_pwr_up_reason_response {
	__le32	reason;
} __packed;

/* get dmesg size request has no payload */
struct gb_vendor_moto_get_dmesg_size_resp {
	__le16  size;
} __packed;

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
	struct gb_vendor_moto_pwr_up_reason_response rsp;
	int ret;

	ret = gb_operation_sync(gb->connection,
				GB_VENDOR_MOTO_TYPE_GET_PWR_UP_REASON,
				NULL, 0, &rsp, sizeof(rsp));
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "0x%08X\n", le32_to_cpu(rsp.reason));
}
static DEVICE_ATTR_RO(pwr_up_reason);

static struct attribute *vendor_attrs[] = {
	&dev_attr_dmesg.attr,
	&dev_attr_last_dmesg.attr,
	&dev_attr_pwr_up_reason.attr,
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

static int gb_vendor_moto_connection_init(struct gb_connection *connection)
{
	struct gb_vendor_moto *gb;
	struct device *dev;
	int retval;
	struct gb_vendor_moto_get_dmesg_size_resp *rsp = NULL;

	gb = kzalloc(sizeof(*gb), GFP_KERNEL);
	if (!gb)
		return -ENOMEM;

	gb->connection = connection;
	connection->private = gb;

	gb->dmesg_size = GB_VENDOR_MOTO_DEFAULT_DMESG_SIZE;
	if (connection->module_minor >= GB_VENDOR_MOTO_VER_DMESG_SIZE) {
		rsp = kmalloc(sizeof(struct gb_vendor_moto_get_dmesg_size_resp),
			      GFP_KERNEL);
		if (!rsp)
			goto error;

		retval = gb_operation_sync(gb->connection,
					   GB_VENDOR_MOTO_TYPE_GET_DMESG_SIZE,
					   NULL, 0, rsp, sizeof(*rsp));
		if (retval)
			goto err_rsp_free;

		gb->dmesg_size = le16_to_cpu(rsp->size);
	}

	dev_info(&connection->bundle->dev, "module_minor=%d, dmesg_size=%d\n",
		 connection->module_minor, gb->dmesg_size);

	/* Create a device in sysfs */
	gb->minor = ida_simple_get(&minors, 0, 0, GFP_KERNEL);
	if (gb->minor < 0) {
		retval = gb->minor;
		goto err_rsp_free;
	}
	dev = device_create(&vendor_class, &connection->bundle->dev,
				MKDEV(0, 0), gb, "mod%d", gb->minor);
	if (IS_ERR(dev)) {
		retval = -EINVAL;
		goto err_ida_remove;
	}
	gb->dev = dev;

	return 0;

err_ida_remove:
	ida_simple_remove(&minors, gb->minor);
err_rsp_free:
	kfree(rsp);
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
