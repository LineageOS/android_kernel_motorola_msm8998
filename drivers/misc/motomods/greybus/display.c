/*
 * Greybus Display protocol driver.
 *
 * Copyright 2015 Motorola LLC
 *
 * Released under the GPLv2 only.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/idr.h>

#include <linux/mod_display_comm.h>

#include "greybus.h"

struct gb_display_device {
	struct gb_connection	*connection;
	struct device		*dev;
	int			minor;		/* display minor number */
};

/* Helper Functions */

char* display_event_values[] = {
	"DISPLAY_EXT=unknown",
	"DISPLAY_EXT=failure",
	"DISPLAY_EXT=available",
	"DISPLAY_EXT=unavailable",
	"DISPLAY_EXT=connect",
	"DISPLAY_EXT=disconnect",
};

void report_display_event(struct device *dev,
	enum mod_display_notification event)
{
	char *envp[2];

	envp[0] = display_event_values[event < GB_DISPLAY_NOTIFY_NUM_EVENTS ?
		event : GB_DISPLAY_NOTIFY_INVALID];
	envp[1] = NULL;

	dev_dbg(dev, "%s: %s\n", __func__, envp[0]);

	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
}

int handle_notification(struct device *dev, enum mod_display_notification event)
{
	int ret = -EINVAL;

	switch(event) {
	case GB_DISPLAY_NOTIFY_FAILURE:
		dev_err(dev, "GB_DISPLAY_NOTIFY_FAILURE\n");
		ret = mod_display_notification(MOD_NOTIFY_FAILURE);
		break;
	case GB_DISPLAY_NOTIFY_AVAILABLE:
		dev_dbg(dev, "GB_DISPLAY_NOTIFY_AVAILABLE\n");
		ret = mod_display_notification(MOD_NOTIFY_AVAILABLE);
		break;
	case GB_DISPLAY_NOTIFY_UNAVAILABLE:
		dev_dbg(dev, "GB_DISPLAY_NOTIFY_UNAVAILABLE\n");
		ret = mod_display_notification(MOD_NOTIFY_UNAVAILABLE);
		break;
	case GB_DISPLAY_NOTIFY_CONNECT:
		dev_dbg(dev, "GB_DISPLAY_NOTIFY_CONNECT\n");
		ret = mod_display_notification(MOD_NOTIFY_CONNECT);
		break;
	case GB_DISPLAY_NOTIFY_DISCONNECT:
		dev_dbg(dev, "GB_DISPLAY_NOTIFY_DISCONNECT\n");
		ret = mod_display_notification(MOD_NOTIFY_DISCONNECT);
		break;
	default:
		dev_err(dev, "unsupported event: %u\n", event);
	}

	if (!ret)
		report_display_event(dev, event);

	return ret;
}

/* Protocol Handlers */

/* host ready request has no payload */
/* host ready request has no response */
static int host_ready(void *data)
{
	struct gb_display_device *disp = (struct gb_display_device *)data;
	int ret;

	ret = gb_operation_sync(disp->connection, GB_DISPLAY_HOST_READY,
				NULL, 0, NULL, 0);

	return ret;
}

#define MAX_DISPLAY_CONFIG_SIZE 1024

static int get_display_config(void *data, struct mod_display_panel_config **display_config)
{
	struct gb_display_device *disp = (struct gb_display_device *)data;
	struct gb_display_get_display_config_size_response size_response;
	struct gb_display_get_display_config_response *config_response;
	struct mod_display_panel_config *config;
	u32 config_size;
	u32 config_response_size;
	int ret;

	ret = gb_operation_sync(disp->connection, GB_DISPLAY_GET_CONFIG_SIZE,
				 NULL, 0, &size_response, sizeof(size_response));
	if (ret)
		goto exit;

	config_size = le32_to_cpu(size_response.size);
	if (config_size > MAX_DISPLAY_CONFIG_SIZE) {
		dev_err(disp->dev, "Config size too large: %d\n", config_size);
		ret = -EINVAL;
		goto exit;
	}

	config_response_size = config_size + sizeof(*config_response);

	config_response = kmalloc(config_response_size, GFP_KERNEL);
	if (!config_response) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = gb_operation_sync(disp->connection, GB_DISPLAY_GET_CONFIG,
				 NULL, 0, config_response, config_response_size);

	if (ret)
		goto free_config_response;

	config = kzalloc(config_size + sizeof(*config), GFP_KERNEL);
	if (!config) {
		ret = -ENOMEM;
		goto free_config_response;
	}

	config->display_type = config_response->display_type;
	config->config_type = config_response->config_type;
	config->config_size = config_size;
	memcpy(config->config_buf, config_response->data, config_size);

	*display_config = config;

free_config_response:
	kfree(config_response);
exit:
	return ret;
}

static int set_display_config(void *data, u8 index)
{
	struct gb_display_device *disp = (struct gb_display_device *)data;
	struct gb_display_set_display_config_request request;
	int ret;

	request.index = index;

	ret = gb_operation_sync(disp->connection, GB_DISPLAY_SET_CONFIG,
				&request, sizeof(request), NULL, 0);

	return ret;
}

static int get_display_state(void *data, u8 *state)
{
	struct gb_display_device *disp = (struct gb_display_device *)data;
	struct gb_display_get_display_state_response response;
	int ret;

	ret = gb_operation_sync(disp->connection, GB_DISPLAY_GET_STATE,
				NULL, 0, &response, sizeof(response));
	if (ret)
		goto exit;

	*state = response.state;

exit:
	return ret;
}

#define SET_STATE_TIMEOUT (10 * 1000) /* 10 seconds */

static int set_display_state(void *data, u8 state)
{
	struct gb_display_device *disp = (struct gb_display_device *)data;
	struct gb_display_set_display_state_request request;
	int ret;

	request.state = state;

	ret = gb_operation_sync_timeout(disp->connection, GB_DISPLAY_SET_STATE,
				&request, sizeof(request), NULL, 0, SET_STATE_TIMEOUT);

	return ret;
}

/* Sysfs Entries */

static ssize_t config_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct gb_display_device *disp = dev_get_drvdata(dev);
	struct mod_display_panel_config *display_config;
	char dump_buf[64];
	char *ptr;
	int cnt;
	ssize_t len, tot;
	int ret;

	ret = get_display_config((void *)disp, &display_config);
	if (ret) {
		dev_err(dev, "Failed to get config: %d\n", ret);
		tot = ret;
		goto exit;
	}

	tot = scnprintf(buf, PAGE_SIZE, "0x%02x\n0x%02x\n0x%08x\n",
		display_config->display_type,
		display_config->config_type,
		display_config->config_size);

	ptr = display_config->config_buf;

	for (cnt = display_config->config_size; cnt > 0; cnt -= 16) {
		hex_dump_to_buffer(ptr, min(cnt, 16), 16, 1, dump_buf,
			sizeof(dump_buf), false);
		len = scnprintf(buf + tot, PAGE_SIZE - tot, "%s\n", dump_buf);
		ptr += 16;
		tot += len;
		if (tot >= PAGE_SIZE)
			break;
	}

	kfree(display_config);

exit:
	return tot;
}
static DEVICE_ATTR_RO(config);

static ssize_t notification_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u8 event = MOD_NOTIFY_INVALID;

	if (!strncmp(buf, "connect", 7)) {
		dev_dbg(dev, "%s: Forcing connect\n", __func__);
		event = MOD_NOTIFY_CONNECT;
	} else if (!strncmp(buf, "disconnect", 10)) {
		dev_dbg(dev, "%s: Forcing disconnect\n", __func__);
		event = MOD_NOTIFY_DISCONNECT;
	} else if (!strncmp(buf, "available", 9)) {
		dev_dbg(dev, "%s: Forcing available\n", __func__);
		event = MOD_NOTIFY_AVAILABLE;
	} else if (!strncmp(buf, "unavailable", 11)) {
		dev_dbg(dev, "%s: Forcing unavailable\n", __func__);
		event = MOD_NOTIFY_UNAVAILABLE;
	} else if (!strncmp(buf, "failure", 7)) {
		dev_dbg(dev, "%s: Forcing failure\n", __func__);
		event = MOD_NOTIFY_FAILURE;
	} else
		dev_err(dev, "%s: Unknown notification: %s\n", __func__, buf);

	if (event != MOD_NOTIFY_INVALID) {
		if (!mod_display_notification(event))
			report_display_event(dev, event);
	}

	return count;
}
static DEVICE_ATTR_WO(notification);

static ssize_t state_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct gb_display_device *disp = dev_get_drvdata(dev);
	u8 display_state;
	ssize_t len = 0;
	int ret = 0;

	ret = get_display_state(disp, &display_state);
	if (ret) {
		dev_err(dev, "%s: failed to get display state! (ret: %d)\n",
			__func__, ret);
		len = ret;
	} else if (display_state != GB_DISPLAY_STATE_ON &&
		   display_state != GB_DISPLAY_STATE_OFF) {
		dev_err(dev, "%s: invalid display state! (display_state: %d)\n",
			__func__, display_state);
		len = -EINVAL;
	} else
		len = scnprintf(buf, PAGE_SIZE, "%s\n", display_state ? "on" : "off");

	return len;
}

static ssize_t state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gb_display_device *disp = dev_get_drvdata(dev);

	if (!strncmp(buf, "off", 3)) {
		dev_dbg(dev, "%s: Setting display state OFF\n", __func__);
		set_display_state(disp, 0);
	} else if (!strncmp(buf, "on", 2)) {
		dev_dbg(dev, "%s: Setting display state ON\n", __func__);
		set_display_state(disp, 1);
	} else {
		dev_err(dev, "%s: Invalid state: %s\n", __func__, buf);
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(state);

static struct attribute *display_attrs[] = {
	&dev_attr_config.attr,
	&dev_attr_notification.attr,
	&dev_attr_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(display);

static struct class display_class = {
	.name		= "display",
	.owner		= THIS_MODULE,
	.dev_groups	= display_groups,
};

static DEFINE_IDA(minors);

struct gb_display_notification_request {
	__u8	event;
} __packed;

static int gb_display_event_recv(u8 type, struct gb_operation *op)
{
	struct gb_connection *connection = op->connection;
	struct gb_display_notification_request *request;
	struct gb_display_device *disp = connection->private;
	int ret = -EINVAL;

	if (!disp) {
		dev_err(&connection->bundle->dev,
			"%s: display device not yet initialized\n", __func__);
		return -EAGAIN;
	}

	if (op->request->payload_size != sizeof(*request)) {
		dev_err(&connection->bundle->dev,
			"%s: illegal size of gb_display_notification_request (%zu != %zu)\n",
			__func__, op->request->payload_size, sizeof(*request));
		goto exit;
	}

	/* By convention, the AP initiates the version operation */
	switch (type) {
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		dev_err(&connection->bundle->dev,
			"module-initiated version operation\n");
		break;
	case GB_DISPLAY_NOTIFICATION:
		dev_dbg(&connection->bundle->dev, "GB_DISPLAY_NOTIFICATION\n");
		request = op->request->payload;
		ret = handle_notification(disp->dev, request->event);
		break;
	default:
		dev_err(&connection->bundle->dev,
			"unsupported request: %hhu\n", type);
	}

exit:
	return ret;
}

struct mod_display_comm_ops mod_display_comm_ops = {
	.host_ready = host_ready,
	.get_display_config = get_display_config,
	.set_display_config = set_display_config,
	.get_display_state = get_display_state,
	.set_display_state = set_display_state,
};

static struct mod_display_comm_data mod_display_comm = {
	.ops = &mod_display_comm_ops,
};

static int gb_display_connection_init(struct gb_connection *connection)
{
	struct gb_display_device *disp;
	struct device *dev;
	int retval;

	if (mod_display_comm_ops.data) {
		pr_err("%s: Only one display connection is supported at a time",
			__func__);
		return -EBUSY;
	}

	disp = kzalloc(sizeof(*disp), GFP_KERNEL);
	if (!disp)
		return -ENOMEM;

	disp->connection = connection;

	disp->minor = ida_simple_get(&minors, 0, 0, GFP_KERNEL);
	if (disp->minor < 0) {
		retval = disp->minor;
		goto error;
	}
	dev = device_create(&display_class, &connection->bundle->dev, MKDEV(0, 0), disp,
			    "display%d", disp->minor);
	if (IS_ERR(dev)) {
		retval = PTR_ERR(dev);
		goto err_ida_remove;
	}

	connection->private = disp;
	disp->dev = dev;

	mod_display_comm_ops.data = disp;
	mod_display_register_comm(&mod_display_comm);

	return 0;

err_ida_remove:
	ida_simple_remove(&minors, disp->minor);
error:
	kfree(disp);
	return retval;
}

static void gb_display_connection_exit(struct gb_connection *connection)
{
	struct gb_display_device *disp = connection->private;

	mod_display_unregister_comm(&mod_display_comm);
	mod_display_comm_ops.data = NULL;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,11,0)
	sysfs_remove_group(&disp->dev->kobj, display_groups[0]);
#endif
	device_unregister(disp->dev);
	ida_simple_remove(&minors, disp->minor);
	kfree(disp);
}

static struct gb_protocol display_protocol = {
	.name			= "display",
	.id			= GREYBUS_PROTOCOL_MODS_DISPLAY,
	.major			= GB_DISPLAY_VERSION_MAJOR,
	.minor			= GB_DISPLAY_VERSION_MINOR,
	.connection_init	= gb_display_connection_init,
	.connection_exit	= gb_display_connection_exit,
	.request_recv		= gb_display_event_recv,
};

static __init int protocol_init(void)
{
	int retval;

	retval = class_register(&display_class);
	if (retval)
		return retval;

	return gb_protocol_register(&display_protocol);
}
module_init(protocol_init);

static __exit void protocol_exit(void)
{
	gb_protocol_deregister(&display_protocol);
	class_unregister(&display_class);
	ida_destroy(&minors);
}
module_exit(protocol_exit);

MODULE_LICENSE("GPL v2");
