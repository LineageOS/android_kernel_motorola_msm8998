/*
 * Motorola specific driver for a Greybus module.
 *
 * Copyright 2015 Motorola Mobility, LLC.
 * Copyright 2014 Google Inc.
 * Copyright 2014 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "greybus.h"

struct gb_vendor_moto {
	struct gb_connection *connection;
	u8 version_major;
	u8 version_minor;
};

/* Version of the Greybus protocol we support */
#define	GB_VENDOR_MOTO_VERSION_MAJOR		0x00
#define	GB_VENDOR_MOTO_VERSION_MINOR		0x01

/* Greybus Motorola vendor specific request types */
#define	GB_VENDOR_MOTO_TYPE_PROTOCOL_VERSION	0x01
#define	GB_VENDOR_MOTO_TYPE_CHARGE_BASE		0x02

struct gb_vendor_moto_charge_base_request {
	__u8	enable;
};

/* Define get_version() routine */
define_get_version(gb_vendor_moto, VENDOR_MOTO);

static int charge_base(struct gb_vendor_moto *gb, u8 enable)
{
	struct gb_vendor_moto_charge_base_request request;

	request.enable = enable;
	return gb_operation_sync(gb->connection, GB_VENDOR_MOTO_TYPE_CHARGE_BASE,
				 &request, sizeof(request), NULL, 0);
}

static int gb_vendor_moto_connection_init(struct gb_connection *connection)
{
	struct gb_vendor_moto *gb;
	int retval;

	gb = kzalloc(sizeof(*gb), GFP_KERNEL);
	if (!gb)
		return -ENOMEM;

	gb->connection = connection;
	connection->private = gb;

	/* Check the version */
	retval = get_version(gb);
	if (retval) {
		kfree(gb);
		return retval;
	}

	/* Enable charging */
	retval = charge_base(gb, 1);
	if (retval) {
		kfree(gb);
		return retval;
	}

	return 0;
}

static void gb_vendor_moto_connection_exit(struct gb_connection *connection)
{
	struct gb_vendor_moto *gb = connection->private;

	kfree(gb);
}

static struct gb_protocol vendor_moto_protocol = {
	.name			= "vendor-moto",
	.id			= GREYBUS_PROTOCOL_VENDOR,
	.major			= 0,
	.minor			= 1,
	.connection_init	= gb_vendor_moto_connection_init,
	.connection_exit	= gb_vendor_moto_connection_exit,
	.request_recv		= NULL,	/* no incoming requests */
};

gb_protocol_driver(&vendor_moto_protocol);

MODULE_LICENSE("GPL v2");
