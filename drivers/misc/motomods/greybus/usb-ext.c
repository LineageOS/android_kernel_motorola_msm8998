/* Copyright (C) 2016 Motorola Mobility, Inc.
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
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/mods/usb_ext_bridge.h>

#include "greybus.h"

struct gb_usb_ext_data {
	uint8_t protocol;
	uint8_t path;
	uint8_t remote_type;
};

static void gb_usb_ext_ap_ready_cb(struct gb_operation *operation)
{
	/* noop */
}

static int gb_usb_ext_send_ap_ready(struct gb_connection *connection)
{
	struct gb_operation *operation;
	int ret;

	operation = gb_operation_create(connection, GB_USB_EXT_TYPE_READY,
			0, 0, GFP_KERNEL);

	if (!operation) {
		return -ENOMEM;
	}

	ret = gb_operation_request_send(operation,
			gb_usb_ext_ap_ready_cb, GFP_KERNEL);
	if (ret) {
		dev_err(&connection->bundle->dev,
			"synchronous operation failed: %d\n", ret);
	}

	gb_operation_put(operation);

	return 0;
}

static int gb_usb_ext_handle_attach(struct gb_operation *operation)
{
	struct gb_usb_ext_attach_request *request;
	struct usb_ext_status status;

	if (operation->request->payload_size < sizeof(*request)) {
		pr_err("transfer request too small (%zu < %zu)\n",
			operation->request->payload_size,
			sizeof(*request));
		return -EINVAL;
	}

	request = operation->request->payload;

	status.active = request->active;
	switch (request->protocol) {
	case GB_USB_EXT_PROTOCOL_2_0:
		status.proto = USB_EXT_PROTO_2_0;
		break;
	case GB_USB_EXT_PROTOCOL_3_1:
		status.proto = USB_EXT_PROTO_3_1;
		break;
	case GB_USB_EXT_PROTOCOL_DUAL:
		status.proto = USB_EXT_PROTO_DUAL;
		break;
	default:
		status.proto = USB_EXT_PROTO_3_1;
		break;
	}

	status.path = (request->path == GB_USB_EXT_PATH_ENTERPRISE) ?
			USB_EXT_PATH_ENTERPRISE : USB_EXT_PATH_BRIDGE;
	status.type = (request->remote_type == GB_USB_EXT_REMOTE_DEVICE) ?
			USB_EXT_REMOTE_DEVICE : USB_EXT_REMOTE_HOST;

	usb_ext_set_state(&status);

	return 0;
}

static int gb_usb_ext_request_recv(u8 type, struct gb_operation *operation)
{
	switch (type) {
	case GB_USB_EXT_TYPE_ATTACH_STATE:
		return gb_usb_ext_handle_attach(operation);
	break;
	default:
		pr_err("Invalid message type 0x%02x\n", type);
	break;
	};

	return -EINVAL;
}

static int gb_usb_ext_connection_init(struct gb_connection *connection)
{
	return gb_usb_ext_send_ap_ready(connection);
}

static void gb_usb_ext_connection_exit(struct gb_connection *connection)
{
	struct usb_ext_status status;

	status.active = false;
	status.proto = USB_EXT_PROTO_UNKNOWN;
	status.path = USB_EXT_PROTO_UNKNOWN;
	status.type = USB_EXT_REMOTE_UNKNOWN;

	usb_ext_set_state(&status);
}

static struct gb_protocol usb_ext_protocol = {
	.name			= "usb-ext",
	.id			= GREYBUS_PROTOCOL_USB_EXT,
	.major			= GB_USB_EXT_VERSION_MAJOR,
	.minor			= GB_USB_EXT_VERSION_MINOR,
	.connection_init	= gb_usb_ext_connection_init,
	.connection_exit	= gb_usb_ext_connection_exit,
	.request_recv		= gb_usb_ext_request_recv,
};

static __init int usb_ext_protocol_init(void)
{
	return gb_protocol_register(&usb_ext_protocol);
}
module_init(usb_ext_protocol_init);

static __exit void usb_ext_protocol_exit(void)
{
	return gb_protocol_deregister(&usb_ext_protocol);
}
module_exit(usb_ext_protocol_exit);

MODULE_LICENSE("GPL v2");

