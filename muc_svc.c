/*
 * Copyright (C) 2015 Motorola Mobility, Inc.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include "greybus.h"

#include "mods_protocols.h"
#include "muc.h"
#include "muc_attach.h"
#include "muc_svc.h"
#include "mods_nw.h"

struct muc_svc_data {
	struct mods_dl_device *dld;
	atomic_t msg_num;
	struct list_head operations;
	struct platform_device *pdev;
	struct workqueue_struct *wq;
	struct kset *intf_kset;

	struct list_head ext_intf;
	struct list_head slave_drv;

	bool authenticate;
	u16 endo_mask;

	struct notifier_block attach_nb;

	u8 mod_root_ver;
};
struct muc_svc_data *svc_dd;

static DEFINE_MUTEX(slave_lock);
static DEFINE_MUTEX(svc_list_lock);

/* Define the SVCs reserved area of CPORTS to create the vendor
 * connections to each interface
 */
#define SVC_VENDOR_CTRL_CPORT_BASE (0x8000)
#define SVC_VENDOR_CTRL_CPORT(intfid) (SVC_VENDOR_CTRL_CPORT_BASE + intfid)

struct muc_svc_hotplug_work {
	struct work_struct work;
	struct mods_dl_device *dld;
	struct gb_svc_intf_hotplug_request hotplug;
};

#define MUC_SVC_RESPONSE_TYPE 0

#define SVC_MSG_DEFAULT_TIMEOUT 500
#define SVC_AP_HOTPLUG_UNPLUG_TIMEOUT 5000

#define kobj_to_device(k) \
	container_of(k, struct mods_dl_device, intf_kobj)

static int muc_svc_send_reboot(struct mods_dl_device *mods_dev, uint8_t mode);

static int
muc_svc_attach(struct notifier_block *nb, unsigned long state, void *unused)
{
	if (!state)
		svc_dd->mod_root_ver = MB_CONTROL_ROOT_VER_INVALID;

	return 0;
}


static ssize_t manifest_read(struct file *fp, struct kobject *kobj,
				struct bin_attribute *attr, char *buf,
				loff_t pos, size_t size)
{
	struct mods_dl_device *mods_dev = kobj_to_device(kobj);
	ssize_t count = 0;

	if (!mods_dev->manifest || !mods_dev->manifest_size)
		return -EINVAL;

	for ( ; size > 0 && pos < mods_dev->manifest_size; count++, size--)
		*buf++ = mods_dev->manifest[pos++];

	return count;
}

static ssize_t fw_version_show(struct mods_dl_device *dev, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%08X", dev->fw_version);
}

static ssize_t serial_show(struct mods_dl_device *dev, char *buf)
{
	/* Zero's for the serial number indicate unsupported */
	if (!dev->uid_low && !dev->uid_high)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%016llX%016llX",
		dev->uid_high, dev->uid_low);
}

static ssize_t vid_show(struct mods_dl_device *dev, char *buf)
{
	if (!dev->hpw)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%04X",
		dev->hpw->hotplug.data.ara_vend_id);
}

static ssize_t pid_show(struct mods_dl_device *dev, char *buf)
{
	if (!dev->hpw)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%04X",
		dev->hpw->hotplug.data.ara_prod_id);
}

static ssize_t unipro_mid_show(struct mods_dl_device *dev, char *buf)
{
	if (!dev->hpw)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%04X",
		dev->hpw->hotplug.data.unipro_mfg_id);
}

static ssize_t unipro_pid_show(struct mods_dl_device *dev, char *buf)
{
	if (!dev->hpw)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%04X",
		dev->hpw->hotplug.data.unipro_prod_id);
}

static ssize_t
hotplug_store(struct mods_dl_device *dev, const char *buf, size_t count)
{
	unsigned long val;

	/* If authentication is disabled, this is a no-op */
	if (!svc_dd->authenticate)
		return count;

	/* Nothing to do, there is no hotplug, or we already sent */
	if (!dev->hpw || dev->hotplug_sent)
		return -EINVAL;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val == 1)
		queue_work(svc_dd->wq, &dev->hpw->work);

	/* XXX What to do if told to 'deny' */

	return count;
}

static ssize_t
blank_store(struct mods_dl_device *mods_dev, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0 || val != 1)
		return -EINVAL;

	if (muc_svc_send_reboot(mods_dev, MB_CONTROL_REBOOT_BLANK_FLASH)) {
		dev_err(&svc_dd->pdev->dev,
				"INTF: %d, failed to send blankflash\n",
				mods_dev->intf_id);
		return -ENODEV;
	}

	return count;
}

static ssize_t
uevent_store(struct mods_dl_device *dev, const char *buf, size_t count)
{
	if (strncmp(buf, "add", 3))
		return -EINVAL;

	kobject_uevent(&dev->intf_kobj, KOBJ_ADD);

	return count;
}

struct muc_svc_attribute {
	struct attribute attr;
	ssize_t (*show)(struct mods_dl_device *dev, char *buf);
	ssize_t (*store)(struct mods_dl_device *dev, const char *buf,
			size_t count);
};

#define MUC_SVC_ATTR(_name, _mode, _show, _store) \
struct muc_svc_attribute muc_svc_attr_##_name = {\
	.attr = { .name = __stringify(_name), .mode = _mode }, \
	.show = _show, \
	.store = _store, \
}

static MUC_SVC_ATTR(hotplug, 0200, NULL, hotplug_store);
static MUC_SVC_ATTR(vid, 0444, vid_show, NULL);
static MUC_SVC_ATTR(pid, 0444, pid_show, NULL);
static MUC_SVC_ATTR(unipro_mid, 0444, unipro_mid_show, NULL);
static MUC_SVC_ATTR(unipro_pid, 0444, unipro_pid_show, NULL);
static MUC_SVC_ATTR(serial, 0444, serial_show, NULL);
static MUC_SVC_ATTR(uevent, 0200, NULL, uevent_store);
static MUC_SVC_ATTR(fw_version, 0444, fw_version_show, NULL);
static MUC_SVC_ATTR(blank, 0200, NULL, blank_store);

#define to_muc_svc_attr(a) \
	container_of(a, struct muc_svc_attribute, attr)

static ssize_t
muc_svc_sysfs_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct mods_dl_device *dev = kobj_to_device(kobj);
	struct muc_svc_attribute *pattr = to_muc_svc_attr(attr);

	if (!pattr->show)
		return -EIO;

	return pattr->show(dev, buf);
}

static ssize_t
muc_svc_sysfs_store(struct kobject *kobj, struct attribute *attr,
			const char *buf, size_t count)
{
	struct mods_dl_device *dev = kobj_to_device(kobj);
	struct muc_svc_attribute *pattr = to_muc_svc_attr(attr);

	if (!pattr->store)
		return -EIO;

	return pattr->store(dev, buf, count);
}

static const struct sysfs_ops muc_svc_sysfs_ops = {
	.show = muc_svc_sysfs_show,
	.store = muc_svc_sysfs_store,
};

static struct attribute *muc_svc_default_attrs[] = {
	&muc_svc_attr_hotplug.attr,
	&muc_svc_attr_vid.attr,
	&muc_svc_attr_pid.attr,
	&muc_svc_attr_unipro_mid.attr,
	&muc_svc_attr_unipro_pid.attr,
	&muc_svc_attr_serial.attr,
	&muc_svc_attr_uevent.attr,
	&muc_svc_attr_fw_version.attr,
	&muc_svc_attr_blank.attr,
	NULL,
};

static struct kobj_type ktype_muc_svc = {
	.sysfs_ops = &muc_svc_sysfs_ops,
	.default_attrs = muc_svc_default_attrs,
};

static int muc_svc_create_dl_dev_sysfs(struct mods_dl_device *mods_dev)
{
	int err;

	mods_dev->intf_kobj.kset = svc_dd->intf_kset;
	err = kobject_init_and_add(&mods_dev->intf_kobj, &ktype_muc_svc,
					NULL, "%d", mods_dev->intf_id);
	if (err)
		goto put_kobj;

	sysfs_bin_attr_init(&mods_dev->manifest_attr);
	mods_dev->manifest_attr.attr.name = "manifest";
	mods_dev->manifest_attr.attr.mode = S_IRUGO;
	mods_dev->manifest_attr.read = manifest_read;
	mods_dev->manifest_attr.size = 0;

	err = sysfs_create_bin_file(&mods_dev->intf_kobj,
					&mods_dev->manifest_attr);
	if (err)
		goto put_kobj;

	kobject_uevent(&mods_dev->intf_kobj, KOBJ_ADD);

	return 0;

put_kobj:
	kobject_put(&mods_dev->intf_kobj);

	return err;
}

static void muc_svc_destroy_dl_dev_sysfs(struct mods_dl_device *mods_dev)
{
	if (!mods_dev->intf_kobj.state_initialized)
		return;

	sysfs_remove_bin_file(&mods_dev->intf_kobj,
				&mods_dev->manifest_attr);
	kobject_put(&mods_dev->intf_kobj);
	memset(&mods_dev->intf_kobj, 0, sizeof(mods_dev->intf_kobj));
}

/*
 * Map an enum gb_operation_status value (which is represented in a
 * message as a single byte) to an appropriate Linux negative errno.
 */
static int gb_operation_status_map(u8 status)
{
	switch (status) {
	case GB_OP_SUCCESS:
		return 0;
	case GB_OP_INTERRUPTED:
		return -EINTR;
	case GB_OP_TIMEOUT:
		return -ETIMEDOUT;
	case GB_OP_NO_MEMORY:
		return -ENOMEM;
	case GB_OP_PROTOCOL_BAD:
		return -EPROTONOSUPPORT;
	case GB_OP_OVERFLOW:
		return -EMSGSIZE;
	case GB_OP_INVALID:
		return -EINVAL;
	case GB_OP_RETRY:
		return -EAGAIN;
	case GB_OP_NONEXISTENT:
		return -ENODEV;
	case GB_OP_MALFUNCTION:
		return -EILSEQ;
	case GB_OP_UNKNOWN_ERROR:
	default:
		return -EIO;
	}
}

/*
 * Map a Linux errno value (from operation->errno) into the value
 * that should represent it in a response message status sent
 * over the wire.  Returns an enum gb_operation_status value (which
 * is represented in a message as a single byte).
 */
static u8 gb_operation_errno_map(int errno)
{
	switch (errno) {
	case 0:
		return GB_OP_SUCCESS;
	case -EINTR:
		return GB_OP_INTERRUPTED;
	case -ETIMEDOUT:
		return GB_OP_TIMEOUT;
	case -ENOMEM:
		return GB_OP_NO_MEMORY;
	case -EPROTONOSUPPORT:
		return GB_OP_PROTOCOL_BAD;
	case -EMSGSIZE:
		return GB_OP_OVERFLOW;	/* Could be underflow too */
	case -EINVAL:
		return GB_OP_INVALID;
	case -EAGAIN:
		return GB_OP_RETRY;
	case -EILSEQ:
		return GB_OP_MALFUNCTION;
	case -ENODEV:
		return GB_OP_NONEXISTENT;
	case -EIO:
	default:
		return GB_OP_UNKNOWN_ERROR;
	}
}

static struct gb_message *svc_gb_msg_alloc(u8 type, size_t payload_size)
{
	struct gb_message *msg;
	struct gb_operation_msg_hdr *hdr;
	size_t message_size = payload_size + sizeof(*hdr);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return NULL;

	msg->buffer = kzalloc(message_size, GFP_KERNEL);
	if (!msg) {
		kfree(msg);
		return NULL;
	}

	hdr = msg->buffer;
	hdr->size = cpu_to_le16(message_size);
	hdr->operation_id = 0;
	hdr->type = type;
	hdr->result = 0;

	msg->header = hdr;
	msg->payload = payload_size ? hdr + 1 : NULL;
	msg->payload_size = payload_size;

	return msg;
}

static void svc_gb_msg_free(struct gb_message *msg)
{
	if (!msg)
		return;
	kfree(msg->buffer);
	kfree(msg);
}

struct svc_op {
	struct list_head entry;
	struct completion completion;
	struct gb_message *request;
	struct gb_message *response;
	u16 msg_id;
};

static inline struct muc_svc_data *dld_get_dd(struct mods_dl_device *dld)
{
	return (struct muc_svc_data *)dld->dl_priv;
}

static inline size_t get_gb_msg_size(struct gb_message *msg)
{
	return sizeof(*msg->header) + msg->payload_size;
}

static struct svc_op *svc_find_op(struct muc_svc_data *dd, uint16_t id)
{
	struct svc_op *e, *tmp;

	list_for_each_entry_safe(e, tmp, &dd->operations, entry)
		if (e->msg_id == id)
			return e;

	return NULL;
}

/* Route a gb_message to the mods_nw layer, adding the necessary
 * envelope that it understands.
 */
static int
svc_route_msg(struct mods_dl_device *dld, uint16_t cport,
		struct gb_message *msg)
{
	struct muc_msg *m;
	size_t muc_payload = get_gb_msg_size(msg);
	size_t msg_size = muc_payload + sizeof(m->hdr);
	int ret;

	m = kmalloc(msg_size, GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	memcpy(m->gb_msg, msg->buffer, muc_payload);
	m->hdr.cport = cpu_to_le16(cport);

	ret = mods_nw_switch(dld, (uint8_t *)m, msg_size);

	kfree(m);

	return ret;
}

static inline size_t get_gb_payload_size(size_t message_size)
{
	return message_size - sizeof(struct gb_operation_msg_hdr);
}

static int svc_set_intf_id(struct mods_dl_device *dld, struct gb_message *req)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct gb_svc_intf_device_id_request *id = req->payload;
	struct mods_dl_device *mods_dev = mods_nw_get_dl_device(id->intf_id);

	if (!mods_dev) {
		dev_err(&dd->pdev->dev, "No device found for interface %d\n",
			id->intf_id);
		return -ENODEV;
	}

	mods_dev->device_id = id->intf_id;

	return 0;
}

static int
svc_gb_conn_create(struct mods_dl_device *dld, struct gb_message *req,
		   uint16_t cport)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct gb_svc_conn_create_request *conn = req->payload;
	int ret;

	dev_info(&dd->pdev->dev, "Create Connection: %hu:%hu to %hu:%hu\n",
			conn->intf1_id, conn->cport1_id,
			conn->intf2_id, conn->cport2_id);

	/* Create the two bi-directional connection routes */
	ret = mods_nw_add_route(conn->intf1_id, conn->cport1_id,
				conn->intf2_id, conn->cport2_id);
	if (ret) {
		dev_err(&dd->pdev->dev,
			"Failed to create route: %d:%d->%d.%d\n",
			conn->intf1_id, conn->cport1_id,
			conn->intf2_id, conn->cport2_id);
		return ret;
	}

	ret = mods_nw_add_route(conn->intf2_id, conn->cport2_id,
				conn->intf1_id, conn->cport1_id);
	if (ret) {
		dev_err(&dd->pdev->dev,
			"Failed to create route: %d:%d->%d.%d\n",
			conn->intf2_id, conn->cport2_id,
			conn->intf1_id, conn->cport1_id);
		goto del_route;
	}

	return 0;

del_route:
	mods_nw_del_route(conn->intf1_id, conn->cport1_id,
			conn->intf2_id, conn->cport2_id);

	return ret;
}

static int
svc_gb_conn_destroy(struct mods_dl_device *dld, struct gb_message *req,
		    uint16_t cport)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct gb_svc_conn_destroy_request *conn = req->payload;

	dev_info(&dd->pdev->dev, "Destroy Connection: %hu:%hu to %hu:%hu\n",
			conn->intf1_id, conn->cport1_id,
			conn->intf2_id, conn->cport2_id);

	/* Destroy the bi-directional routes */
	mods_nw_del_route(conn->intf1_id, conn->cport1_id,
				conn->intf2_id, conn->cport2_id);
	mods_nw_del_route(conn->intf2_id, conn->cport2_id,
				conn->intf1_id, conn->cport1_id);

	return 0;
}

static int
svc_gb_send_response(struct mods_dl_device *dld, uint16_t cport,
			struct gb_message *req_msg, size_t payload_size,
			void *payload, uint8_t status)
{
	struct gb_message *resp_msg;
	int ret;
	u8 type = req_msg->header->type | GB_MESSAGE_TYPE_RESPONSE;

	resp_msg = svc_gb_msg_alloc(type, payload_size);
	if (!resp_msg)
		return -ENOMEM;

	resp_msg->header->operation_id = req_msg->header->operation_id;
	resp_msg->header->result = status;
	if (payload_size)
		memcpy(resp_msg->payload, payload, payload_size);

	ret = svc_route_msg(dld, cport, resp_msg);

	svc_gb_msg_free(resp_msg);

	return ret;
}

struct svc_gb_dme_entry {
	u16 attr;
	u16 selector;
	u32 default_value;
	int (*dme_get)(struct mods_dl_device *dld, u8 intf_id,
			u16 attr, u16 selector, u32 *value);
	int (*dme_set)(struct mods_dl_device *dld, u8 intf_id,
			u16 attr, u16 selector, u32 value);
};

static struct svc_gb_dme_entry dme_entries[] = {
	{
		.attr = DME_ATTR_T_TST_SRC_INCREMENT,
		.selector = DME_ATTR_SELECTOR_INDEX,
		.default_value = 0xB007ED,
	},
};

static struct svc_gb_dme_entry *svc_gb_get_dme_entry(u16 attr, u16 selector)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dme_entries); i++) {
		if (dme_entries[i].attr != attr)
			continue;
		if (dme_entries[i].selector != selector)
			continue;
		return &dme_entries[i];
	}

	return NULL;
}

static int
svc_gb_dme_get(struct mods_dl_device *dld, struct gb_message *req_msg,
		uint16_t cport)
{
	struct gb_svc_dme_peer_get_request *req;
	struct gb_svc_dme_peer_get_response resp;
	struct svc_gb_dme_entry *entry;
	int ret;
	u16 attr;
	u16 selector;

	req = (struct gb_svc_dme_peer_get_request *)req_msg->payload;
	attr = le16_to_cpu(req->attr);
	selector = le16_to_cpu(req->selector);

	entry = svc_gb_get_dme_entry(attr, selector);
	if (!entry) {
		resp.result_code = GB_OP_NONEXISTENT;
		goto send_response;
	}

	/* If there is a specific handler, use it, otherwise lets just
	 * use the default value we always want to return.
	 */
	if (entry->dme_get) {
		ret = entry->dme_get(dld, req->intf_id, attr,
					selector, &resp.attr_value);
		resp.result_code = gb_operation_errno_map(ret);
	} else {
		resp.result_code = GB_OP_SUCCESS;
		resp.attr_value = entry->default_value;
	}

send_response:
	ret = svc_gb_send_response(dld, cport, req_msg, sizeof(resp),
					&resp, GB_OP_SUCCESS);
	if (ret)
		dev_err(&svc_dd->pdev->dev,
			"Failed response to DME_GET request for intf_id: %d\n",
			req->intf_id);

	return 0;
}

static int
svc_gb_dme_set(struct mods_dl_device *dld, struct gb_message *req_msg,
		uint16_t cport)
{
	struct gb_svc_dme_peer_set_request *req;
	struct gb_svc_dme_peer_set_response resp;
	struct svc_gb_dme_entry *entry;
	int ret;
	u16 attr;
	u16 selector;

	req = (struct gb_svc_dme_peer_set_request *)req_msg->payload;
	attr = le16_to_cpu(req->attr);
	selector = le16_to_cpu(req->selector);

	entry = svc_gb_get_dme_entry(attr, selector);
	if (!entry) {
		resp.result_code = GB_OP_NONEXISTENT;
		goto send_response;
	}

	/* If there is a specific handler, use it, otherwise done
	 */
	if (entry->dme_set) {
		ret = entry->dme_set(dld, req->intf_id, attr, selector,
					le32_to_cpu(req->value));
		resp.result_code = gb_operation_errno_map(ret);
	} else {
		resp.result_code = GB_OP_SUCCESS;
	}

send_response:
	ret = svc_gb_send_response(dld, cport, req_msg, sizeof(resp),
					&resp, GB_OP_SUCCESS);
	if (ret)
		dev_err(&svc_dd->pdev->dev,
			"Failed response to DME_SET request for intf_id: %d\n",
			req->intf_id);

	return 0;
}

static int
muc_svc_handle_ap_request(struct mods_dl_device *dld, uint8_t *data,
			  size_t msg_size, uint16_t cport)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	size_t payload_size = get_gb_payload_size(msg_size);
	struct gb_message *req;
	int ret = 0;
	struct gb_operation_msg_hdr hdr;

	memcpy(&hdr, data, sizeof(hdr));

	req = svc_gb_msg_alloc(hdr.type, payload_size);
	if (!req)
		return -ENOMEM;
	memcpy(req->header, data, msg_size);

	switch (hdr.type) {
	case GB_SVC_TYPE_INTF_DEVICE_ID:
		ret = svc_set_intf_id(dld, req);
		break;
	case GB_SVC_TYPE_INTF_RESET:
		/* XXX Handle interface reset request */
		break;
	case GB_SVC_TYPE_CONN_CREATE:
		ret = svc_gb_conn_create(dld, req, cport);
		break;
	case GB_SVC_TYPE_CONN_DESTROY:
		ret = svc_gb_conn_destroy(dld, req, cport);
		break;
	case GB_SVC_TYPE_ROUTE_CREATE:
	case GB_SVC_TYPE_ROUTE_DESTROY:
		/* Just send an ACK, we do not have use for device id */
		break;
	case GB_SVC_TYPE_DME_PEER_GET:
		ret = svc_gb_dme_get(dld, req, cport);
		goto free_request;
	case GB_SVC_TYPE_DME_PEER_SET:
		ret = svc_gb_dme_set(dld, req, cport);
		goto free_request;
	default:
		dev_err(&dd->pdev->dev, "Unsupported AP Request type: %d\n",
					hdr.type);
		ret = -EINVAL;
		goto free_request;
	}

	/* If hdr operation id is non-zero, it expects a response */
	if (hdr.operation_id) {
		ret = svc_gb_send_response(dld, cport, req, 0, NULL,
					gb_operation_errno_map(ret));
		if (ret) {
			dev_err(&dd->pdev->dev,
				"Failed to send AP response for type: %d\n",
				hdr.type);
			goto free_request;
		}
	}

free_request:
	/* Done with the request and op */
	svc_gb_msg_free(req);

	return ret;
}

/* Handle the incoming greybus message and complete the waiting thread, or
 * process the new incoming request.
 */
static int
svc_gb_msg_recv(struct mods_dl_device *dld, uint8_t *data,
		size_t msg_size, uint16_t cport)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct svc_op *op;
	size_t payload_size = get_gb_payload_size(msg_size);
	struct gb_operation_msg_hdr hdr;

	if (msg_size < sizeof(hdr)) {
		dev_err(&dd->pdev->dev, "msg size too small: %zu\n", msg_size);
		return -EINVAL;
	}

	memcpy(&hdr, data, sizeof(hdr));

	/* If this is a response, notify the the waiter */
	if (hdr.type & GB_MESSAGE_TYPE_RESPONSE) {
		op = svc_find_op(dd, le16_to_cpu(hdr.operation_id));
		if (!op) {
			dev_err(&dd->pdev->dev, "OpID: %d unknown\n",
				le16_to_cpu(hdr.operation_id));
			return -EINVAL;
		}

		op->response = svc_gb_msg_alloc(MUC_SVC_RESPONSE_TYPE, payload_size);
		if (!op->response)
			return -ENOMEM;

		memcpy(op->response->header, data, msg_size);
		complete(&op->completion);

		return 0;
	}

	/* If not a response, process the new request */
	return muc_svc_handle_ap_request(dld, data, msg_size, cport);
}

/* Send a message out the specified CPORT and wait for a response */
static struct gb_message *
_svc_gb_msg_send_sync(struct mods_dl_device *dld, uint8_t *data, uint8_t type,
		size_t payload_size, uint16_t cport, bool response,
		uint16_t timeout)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct svc_op *op;
	struct gb_message *msg;
	int ret;
	uint16_t cycle;

	op = kzalloc(sizeof(*op), GFP_KERNEL);
	if (!op)
		return ERR_PTR(-ENOMEM);

	msg = svc_gb_msg_alloc(type, payload_size);
	if (!msg) {
		ret = -ENOMEM;
		goto gb_msg_alloc;
	}

	if (payload_size)
		memcpy(msg->payload, data, payload_size);
	op->request = msg;

	/* Only set the operation id when we want a response */
	if (response) {
		cycle = (u16)atomic_inc_return(&dd->msg_num);
		op->msg_id = cycle % U16_MAX + 1;
		init_completion(&op->completion);

		msg->header->operation_id = cpu_to_le16(op->msg_id);
		list_add_tail(&op->entry, &dd->operations);
	}

	/* Send to NW Routing Layer */
	ret = svc_route_msg(dld, cport, msg);
	if (ret) {
		dev_err(&dd->pdev->dev, "failed sending svc msg: %d\n", ret);
		goto remove_op;
	}

	/* If not waiting for response, we're done */
	if (!response) {
		svc_gb_msg_free(op->request);
		kfree(op);

		return NULL;
	}

	ret = wait_for_completion_interruptible_timeout(&op->completion,
					msecs_to_jiffies(timeout));
	if (ret <= 0) {
		dev_err(&dd->pdev->dev, "svc msg response timeout\n");
		if (!ret)
			ret = -ETIMEDOUT;
		goto remove_op;
	}

	/* Remove and free the request */
	list_del(&op->entry);
	svc_gb_msg_free(op->request);
	op->request = NULL;

	msg = op->response;
	kfree(op);

	if (msg->header->result) {
		int err = gb_operation_status_map(msg->header->result);

		svc_gb_msg_free(msg);
		return ERR_PTR(err);
	}

	return msg;

remove_op:
	if (response)
		list_del(&op->entry);
	svc_gb_msg_free(op->request);
gb_msg_alloc:
	kfree(op);

	return ERR_PTR(ret);
}

static inline struct gb_message *
svc_gb_msg_send_sync(struct mods_dl_device *dld, uint8_t *data, uint8_t type,
		size_t size, uint16_t cport)
{
	return _svc_gb_msg_send_sync(dld, data, type, size, cport,
					true, SVC_MSG_DEFAULT_TIMEOUT);
}

static inline struct gb_message *
svc_gb_msg_send_sync_timeout(struct mods_dl_device *dld, uint8_t *data,
		uint8_t type, size_t size, uint16_t cport, uint16_t timeout)
{
	return _svc_gb_msg_send_sync(dld, data, type, size, cport,
					true, timeout);
}


static inline int
svc_gb_msg_send_no_resp(struct mods_dl_device *dld, uint8_t *data,
		uint8_t type, size_t size, uint16_t cport)
{
	struct gb_message *msg;

	msg = _svc_gb_msg_send_sync(dld, data, type, size, cport,
					false, SVC_MSG_DEFAULT_TIMEOUT);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	return 0;
}


static int muc_svc_version_check(struct mods_dl_device *dld)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct gb_protocol_version_response *ver;
	struct gb_message *msg;

	ver = kmalloc(sizeof(*ver), GFP_KERNEL);
	if (!ver)
		return -ENOMEM;

	ver->major = GB_SVC_VERSION_MAJOR;
	ver->minor = GB_SVC_VERSION_MINOR;

	msg = svc_gb_msg_send_sync(dld, (uint8_t *)ver,
				GB_REQUEST_TYPE_PROTOCOL_VERSION,
				sizeof(*ver), GB_SVC_CPORT_ID);
	if (IS_ERR(msg)) {
		dev_err(&dd->pdev->dev, "Failed to get VERSION from AP\n");
		kfree(ver);
		return PTR_ERR(msg);
	}

	kfree(ver);
	ver = msg->payload;

	/* XXX We could check versions... */
	dev_info(&dd->pdev->dev, "VERSION: %hhu.%hhu\n",
		ver->major, ver->minor);

	svc_gb_msg_free(msg);

	return 0;
}

static int
muc_svc_hello_req(struct mods_dl_device *dld, uint8_t ap_intf_id)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct gb_message *msg;
	struct gb_svc_hello_request *hello;

	hello = kmalloc(sizeof(*hello), GFP_KERNEL);
	if (!hello)
		return -ENOMEM;

	/* Send the endo id and the AP's interface ID */
	hello->endo_id = cpu_to_le16(dd->endo_mask);
	hello->interface_id = ap_intf_id;

	msg = svc_gb_msg_send_sync(dld, (uint8_t *)hello,
				GB_SVC_TYPE_SVC_HELLO,
				sizeof(*hello), GB_SVC_CPORT_ID);
	if (IS_ERR(msg)) {
		dev_err(&dd->pdev->dev, "Failed to send HELLO to AP\n");
		kfree(hello);
		return PTR_ERR(msg);
	}

	svc_gb_msg_free(msg);
	kfree(hello);

	return 0;
}

static int
muc_svc_probe_ap(struct mods_dl_device *dld, uint8_t ap_intf_id)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	int ret;

	ret = muc_svc_version_check(dld);
	if (ret) {
		dev_err(&dd->pdev->dev, "SVC version check failed\n");
		return ret;
	}

	ret = muc_svc_hello_req(dld, ap_intf_id);
	if (ret) {
		dev_err(&dd->pdev->dev, "SVC HELLO failed\n");
		return ret;
	}

	return 0;
}

/* Loop through existing devices to see if there was already a
 * slave mask indicated before the current driver registration.
 *
 * The slave_lock must be held to ensure 'atomic' sequence.
 */
static void muc_svc_check_slave_present(struct mods_slave_ctrl_driver *drv)
{
	struct mods_dl_device *mods_dev;

	if (!drv->slave_present)
		return;

	mutex_lock(&svc_list_lock);
	list_for_each_entry(mods_dev, &svc_dd->ext_intf, list)
		if (mods_dev->slave_mask)
			drv->slave_present(mods_dev->intf_id,
						mods_dev->slave_mask);
	mutex_unlock(&svc_list_lock);
}

/* Notify all Slave Control Drivers of the new slave mask */
static void muc_svc_broadcast_slave_present(struct mods_dl_device *master)
{
	struct mods_slave_ctrl_driver *drv;

	mutex_lock(&slave_lock);
	list_for_each_entry(drv, &svc_dd->slave_drv, list)
		if (drv->slave_present)
			drv->slave_present(master->intf_id,
						master->slave_mask);
	mutex_unlock(&slave_lock);
}

static int
muc_svc_get_hotplug_data(struct mods_dl_device *dld,
			struct gb_svc_intf_hotplug_request *hotplug,
			struct mods_dl_device *mods_dev)
{
	struct mb_control_get_ids_response *ids;
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct gb_message *msg;
	int ret;

	/* GET_IDs has no payload */
	msg = svc_gb_msg_send_sync(dld, NULL, MB_CONTROL_TYPE_GET_IDS,
				0, SVC_VENDOR_CTRL_CPORT(mods_dev->intf_id));
	if (IS_ERR(msg)) {
		dev_err(&dd->pdev->dev, "[%d] Failed to get GET_IDS\n",
			mods_dev->intf_id);
		return PTR_ERR(msg);
	}

	ids = kzalloc(sizeof(*ids), GFP_KERNEL);
	if (!ids) {
		ret = -ENOMEM;
		goto free_gb_msg;
	}

	memcpy(ids, msg->payload, min(msg->payload_size, sizeof(*ids)));

	hotplug->data.unipro_mfg_id = le32_to_cpu(ids->unipro_mfg_id);
	hotplug->data.unipro_prod_id = le32_to_cpu(ids->unipro_prod_id);
	hotplug->data.ara_vend_id = le32_to_cpu(ids->ara_vend_id);
	hotplug->data.ara_prod_id = le32_to_cpu(ids->ara_prod_id);

	/* Save interface device specific data */
	mods_dev->uid_low = le64_to_cpu(ids->uid_low);
	mods_dev->uid_high = le64_to_cpu(ids->uid_high);
	mods_dev->fw_version = le32_to_cpu(ids->fw_version);

	dev_info(&dd->pdev->dev, "[%d] UNIPRO_IDS: %x:%x ARA_IDS: %x:%x\n",
		mods_dev->intf_id, hotplug->data.unipro_mfg_id,
		hotplug->data.unipro_prod_id, hotplug->data.ara_vend_id,
		hotplug->data.ara_prod_id);
	dev_info(&dd->pdev->dev, "[%d] MOD SERIAL: %016llX%016llX\n",
		mods_dev->intf_id, mods_dev->uid_high, mods_dev->uid_low);
	dev_info(&dd->pdev->dev, "[%d] MOD FW_VER: 0x%08X\n",
		mods_dev->intf_id, mods_dev->fw_version);

	mods_dev->slave_mask = le32_to_cpu(ids->slave_mask);
	muc_svc_broadcast_slave_present(mods_dev);

	svc_gb_msg_free(msg);
	kfree(ids);

	return 0;

free_gb_msg:
	svc_gb_msg_free(msg);

	return ret;
}

static int muc_svc_get_root_version(struct mods_dl_device *dld,
					struct mods_dl_device *mods_dev)
{
	struct mb_control_root_ver_response *ver;
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct gb_message *msg;
	int ret = 0;

	/* GET_ROOT_VER has no payload */
	msg = svc_gb_msg_send_sync(dld, NULL, MB_CONTROL_TYPE_GET_ROOT_VER,
				0, SVC_VENDOR_CTRL_CPORT(mods_dev->intf_id));
	if (IS_ERR(msg)) {
		dev_warn(&dd->pdev->dev, "[%d] Failed to get GET_ROOT_VER\n",
			mods_dev->intf_id);
		return PTR_ERR(msg);
	}

	ver = msg->payload;

	/* This interface does not report core version */
	if (ver->version == MB_CONTROL_ROOT_VER_NOT_APPLICABLE)
		goto free_msg;

	if (dd->mod_root_ver == MB_CONTROL_ROOT_VER_INVALID) {
		dd->mod_root_ver = ver->version;
		dev_info(&dd->pdev->dev, "[%d] ROOT_VER: %d\n",
				mods_dev->intf_id, ver->version);
	} else if (dd->mod_root_ver != ver->version) {
		dev_err(&dd->pdev->dev,
			"[%d] Got ROOT_VER: %d but already had: %d\n",
			mods_dev->intf_id, ver->version, dd->mod_root_ver);
		ret = -EINVAL;
	}

free_msg:
	svc_gb_msg_free(msg);

	return ret;
}

static int muc_svc_create_control_route(u8 intf_id, u16 src, u16 dest)
{
	int ret;

	/* Create a route to interface's Control Port, we assign the CPort
	 * on SVC same as interface since its unique.
	 */
	ret = mods_nw_add_route(MODS_INTF_SVC, src, intf_id, dest);
	if (ret)
		return ret;

	ret = mods_nw_add_route(intf_id, dest, MODS_INTF_SVC, src);
	if (ret)
		goto clean_route1;

	return 0;

clean_route1:
	mods_nw_del_route(MODS_INTF_SVC, src, intf_id, dest);

	return ret;
}

static void muc_svc_destroy_control_route(u8 intf_id, u16 src, u16 dest)
{
	mods_nw_del_route(MODS_INTF_SVC, src, intf_id, dest);
	mods_nw_del_route(intf_id, dest, MODS_INTF_SVC, src);
}

static void muc_svc_attach_work(struct work_struct *work)
{
	struct muc_svc_hotplug_work *hpw;
	struct gb_message *msg;

	hpw = container_of(work, struct muc_svc_hotplug_work, work);

	if (hpw->dld->hotplug_sent)
		return;

	msg = svc_gb_msg_send_sync_timeout(svc_dd->dld,
					(uint8_t *)&hpw->hotplug,
					GB_SVC_TYPE_INTF_HOTPLUG,
					sizeof(hpw->hotplug), GB_SVC_CPORT_ID,
					SVC_AP_HOTPLUG_UNPLUG_TIMEOUT);
	if (IS_ERR(msg)) {
		dev_err(&svc_dd->pdev->dev, "[%d] Failed to send HOTPLUG\n",
			hpw->hotplug.intf_id);
		return;
	}

	hpw->dld->hotplug_sent = true;
	dev_info(&svc_dd->pdev->dev, "[%d] Successfully sent HOTPLUG\n",
			hpw->hotplug.intf_id);

	svc_gb_msg_free(msg);
}

static int muc_svc_control_version(struct mods_dl_device *mods_dev, u8 type,
					uint16_t cport, u8 *major, u8 *minor)
{
	struct gb_protocol_version_response *ver;
	struct gb_message *msg;

	ver = kmalloc(sizeof(*ver), GFP_KERNEL);
	if (!ver)
		return -ENOMEM;

	ver->major = GB_CONTROL_VERSION_MAJOR;
	ver->minor = GB_CONTROL_VERSION_MINOR;

	msg = svc_gb_msg_send_sync(svc_dd->dld, (uint8_t *)ver,
				type, sizeof(*ver), cport);
	if (IS_ERR(msg)) {
		kfree(ver);
		return PTR_ERR(msg);
	}

	kfree(ver);
	ver = msg->payload;

	dev_dbg(&svc_dd->pdev->dev, "[%d] CONTROL VERSION: %hhu.%hhu\n",
		cport, ver->major, ver->minor);

	*major = ver->major;
	*minor = ver->minor;

	svc_gb_msg_free(msg);

	return 0;
}

static int
muc_svc_get_manifest(struct mods_dl_device *mods_dev, uint16_t out_cport)
{
	struct gb_control_get_manifest_size_response *size_resp;
	struct device *dev = &svc_dd->pdev->dev;
	struct gb_message *msg;
	u8 type = GB_REQUEST_TYPE_PROTOCOL_VERSION;
	int err;

	err = muc_svc_control_version(mods_dev, type, out_cport,
					&mods_dev->gb_ctrl_major,
					&mods_dev->gb_ctrl_minor);
	if (err) {
		dev_err(dev, "[%d] Failed VERSION on CONTROL\n",
			mods_dev->intf_id);
		return PTR_ERR(msg);
	}

	/* GET_SIZE has no payload */
	msg = svc_gb_msg_send_sync(svc_dd->dld, NULL,
					GB_CONTROL_TYPE_GET_MANIFEST_SIZE,
					0, out_cport);
	if (IS_ERR(msg)) {
		dev_err(dev, "[%d] Failed to get MANIFEST_SIZE\n",
			mods_dev->intf_id);
		return PTR_ERR(msg);
	}

	size_resp = msg->payload;
	mods_dev->manifest_size = le16_to_cpu(size_resp->size);

	svc_gb_msg_free(msg);

	mods_dev->manifest = kmalloc(mods_dev->manifest_size, GFP_KERNEL);
	if (!mods_dev->manifest) {
		err = -ENOMEM;
		goto clear_size;
	}

	/* GET_MANIFEST has no payload */
	msg = svc_gb_msg_send_sync(svc_dd->dld, NULL,
					GB_CONTROL_TYPE_GET_MANIFEST,
					0, out_cport);
	if (IS_ERR(msg)) {
		dev_err(dev, "[%d] Failed to get MANIFEST\n",
			mods_dev->intf_id);
		err = PTR_ERR(msg);
		goto free_manifest;
	}

	memcpy(mods_dev->manifest, msg->payload, mods_dev->manifest_size);

	svc_gb_msg_free(msg);

	/* Update with the latest size and notify userspace */
	mods_dev->manifest_attr.size = mods_dev->manifest_size;

	err = muc_svc_create_dl_dev_sysfs(mods_dev);
	if (err)
		goto free_manifest;

	return 0;

free_manifest:
	kfree(mods_dev->manifest);
	mods_dev->manifest = NULL;
clear_size:
	mods_dev->manifest_size = 0;

	return err;
}

static struct muc_svc_hotplug_work *
muc_svc_create_hotplug_work(struct mods_dl_device *mods_dev)
{
	struct muc_svc_hotplug_work *hpw;
	int ret;

	hpw = kzalloc(sizeof(*hpw), GFP_KERNEL);
	if (!hpw)
		return ERR_PTR(-ENOMEM);

	hpw->dld = mods_dev;
	INIT_WORK(&hpw->work, muc_svc_attach_work);

	/* Create route SVC:INTFID<-->INTFID:0 to hook into the reserved
	 * control protocol to obtain the manifest.
	 */
	ret = muc_svc_create_control_route(mods_dev->intf_id,
				mods_dev->intf_id, GB_CONTROL_CPORT_ID);
	if (ret) {
		dev_err(&svc_dd->pdev->dev,
			"[%d] Failed setup GB CONTROL route\n",
			mods_dev->intf_id);
		goto free_hpw;
	}

	/* Get/Negotiate MB Control Protocol Version */
	ret = muc_svc_control_version(mods_dev,
				MB_CONTROL_TYPE_PROTOCOL_VERSION,
				SVC_VENDOR_CTRL_CPORT(mods_dev->intf_id),
				&mods_dev->mb_ctrl_major,
				&mods_dev->mb_ctrl_minor);
	if (ret)
		goto free_route;

	/* Get the hotplug IDs */
	ret = muc_svc_get_hotplug_data(svc_dd->dld, &hpw->hotplug, mods_dev);
	if (ret)
		goto free_route;

	hpw->hotplug.intf_id = mods_dev->intf_id;

	/* Get the hardware's core version if protocol reported support */
	if (MB_CONTROL_SUPPORTS(mods_dev, GET_ROOT_VER)) {
		ret = muc_svc_get_root_version(svc_dd->dld, mods_dev);
		if (ret)
			goto free_route;
	}

	ret = muc_svc_get_manifest(mods_dev, mods_dev->intf_id);
	if (ret)
		goto free_route;

	muc_svc_destroy_control_route(mods_dev->intf_id,
				mods_dev->intf_id, GB_CONTROL_CPORT_ID);

	return hpw;

free_route:
	muc_svc_destroy_control_route(mods_dev->intf_id,
				mods_dev->intf_id, GB_CONTROL_CPORT_ID);
free_hpw:
	kfree(hpw);

	return ERR_PTR(ret);
}

static int muc_svc_generate_hotplug(struct mods_dl_device *mods_dev)
{
	struct muc_svc_hotplug_work *hpw;

	hpw = muc_svc_create_hotplug_work(mods_dev);
	if (IS_ERR(hpw))
		return PTR_ERR(hpw);

	mods_dev->hpw = hpw;

	if (svc_dd->authenticate == false)
		queue_work(svc_dd->wq, &hpw->work);

	return 0;
}

static int muc_svc_generate_unplug(struct mods_dl_device *mods_dev)
{
	struct gb_message *msg;
	struct gb_svc_intf_hot_unplug_request unplug;

	if (!mods_dev->hotplug_sent)
		return 0;

	mods_dev->hotplug_sent = 0;
	unplug.intf_id = mods_dev->intf_id;

	msg = svc_gb_msg_send_sync_timeout(svc_dd->dld, (uint8_t *)&unplug,
					GB_SVC_TYPE_INTF_HOT_UNPLUG,
					sizeof(unplug), GB_SVC_CPORT_ID,
					SVC_AP_HOTPLUG_UNPLUG_TIMEOUT);
	if (IS_ERR(msg)) {
		dev_err(&svc_dd->pdev->dev, "[%d] Failed to send UNPLUG\n",
			mods_dev->intf_id);
		return PTR_ERR(msg);
	}

	svc_gb_msg_free(msg);

	dev_info(&svc_dd->pdev->dev, "[%d] Successfully sent UNPLUG\n",
			mods_dev->intf_id);

	return 0;
}

void mods_dl_dev_detached(struct mods_dl_device *mods_dev)
{
	/* AP is special case */
	if (mods_dev->intf_id == MODS_INTF_AP) {
		mods_nw_del_route(MODS_INTF_SVC, 0, MODS_INTF_AP, 0);
		mods_nw_del_route(MODS_INTF_AP,  0, MODS_INTF_SVC, 0);

		return;
	}

	muc_svc_destroy_dl_dev_sysfs(mods_dev);

	mutex_lock(&svc_list_lock);
	list_del(&mods_dev->list);
	mutex_unlock(&svc_list_lock);

	muc_svc_generate_unplug(mods_dev);

	/* Destroy custom vendor control route */
	muc_svc_destroy_control_route(mods_dev->intf_id,
			SVC_VENDOR_CTRL_CPORT(mods_dev->intf_id),
			VENDOR_CTRL_DEST_CPORT);

	kfree(mods_dev->manifest);
}
EXPORT_SYMBOL_GPL(mods_dl_dev_detached);

/* Notifies that the DL device is in attached state and the
 * hotplug event can be kicked off
 */
int mods_dl_dev_attached(struct mods_dl_device *mods_dev)
{
	int err;

	if (mods_dev->intf_id == MODS_INTF_AP) {
		/* Special case for AP, we'll setup the routes right away */
		err = mods_nw_add_route(MODS_INTF_SVC, 0, MODS_INTF_AP, 0);
		if (err)
			return err;

		err = mods_nw_add_route(MODS_INTF_AP, 0, MODS_INTF_SVC, 0);
		if (err)
			goto free_svc_to_ap;

		err = muc_svc_probe_ap(svc_dd->dld, MODS_INTF_AP);
		if (err)
			goto free_ap_to_svc;

		return 0;
	}

	/* Create route for vendor control protocol on reserved CPORT */
	err = muc_svc_create_control_route(mods_dev->intf_id,
				SVC_VENDOR_CTRL_CPORT(mods_dev->intf_id),
				VENDOR_CTRL_DEST_CPORT);
	if (err) {
		dev_err(&svc_dd->pdev->dev,
			"[%d] VENDOR CONTROL setup failed\n",
			mods_dev->intf_id);
		return err;
	}

	err = muc_svc_generate_hotplug(mods_dev);
	if (err) {
		muc_svc_destroy_control_route(mods_dev->intf_id,
				SVC_VENDOR_CTRL_CPORT(mods_dev->intf_id),
				VENDOR_CTRL_DEST_CPORT);
		goto done;
	}

	mutex_lock(&svc_list_lock);
	list_add_tail(&mods_dev->list, &svc_dd->ext_intf);
	mutex_unlock(&svc_list_lock);
done:
	return err;

free_ap_to_svc:
	mods_nw_del_route(MODS_INTF_AP, 0, MODS_INTF_SVC, 0);
free_svc_to_ap:
	mods_nw_del_route(MODS_INTF_SVC, 0, MODS_INTF_AP, 0);

	return err;
}
EXPORT_SYMBOL_GPL(mods_dl_dev_attached);

struct mods_dl_device *_mods_create_dl_device(struct mods_dl_driver *drv,
		struct device *dev, u8 intf_id)
{
	struct mods_dl_device *mods_dev;

	pr_info("%s for %s [%d]\n", __func__, dev_name(dev), intf_id);
	mods_dev = kzalloc(sizeof(*mods_dev), GFP_KERNEL);
	if (!mods_dev)
		return ERR_PTR(-ENOMEM);

	mods_dev->drv = drv;
	mods_dev->dev = dev;
	mods_dev->intf_id = intf_id;

	mods_nw_add_dl_device(mods_dev);

	return mods_dev;
}

struct mods_dl_device *mods_create_dl_device(struct mods_dl_driver *drv,
		struct device *dev, u8 intf_id)
{
	/* If the SVC hasn't been fully initialized, return error */
	if (!svc_dd)
		return ERR_PTR(-ENODEV);

	return _mods_create_dl_device(drv, dev, intf_id);
}
EXPORT_SYMBOL_GPL(mods_create_dl_device);

void mods_remove_dl_device(struct mods_dl_device *dev)
{
	mods_nw_del_dl_device(dev);
	kfree(dev->hpw);
	kfree(dev);
}
EXPORT_SYMBOL_GPL(mods_remove_dl_device);

int mods_slave_ctrl_power(uint16_t master_id, uint8_t mode, uint32_t slave_id)
{
	struct gb_message *msg;
	struct mb_svc_slave_power_ctrl power;
	struct mods_dl_device *mods_dev = mods_nw_get_dl_device(master_id);

	if (!mods_dev)
		return -ENODEV;

	power.mode = mode;
	power.slave_id = cpu_to_le32(slave_id);

	msg = svc_gb_msg_send_sync(svc_dd->dld, (uint8_t *)&power,
				MB_CONTROL_TYPE_SLAVE_POWER,
				sizeof(power),
				SVC_VENDOR_CTRL_CPORT(master_id));
	if (IS_ERR(msg)) {
		dev_err(&svc_dd->pdev->dev,
			"[%d] Failed send SLAVE_POWER for %d\n",
			mods_dev->intf_id, slave_id);
		return PTR_ERR(msg);
	}

	svc_gb_msg_free(msg);

	return 0;
}
EXPORT_SYMBOL_GPL(mods_slave_ctrl_power);

int mods_register_slave_ctrl_driver(struct mods_slave_ctrl_driver *drv)
{
	if (!svc_dd)
		return -ENODEV;

	mutex_lock(&slave_lock);
	list_add_tail(&drv->list, &svc_dd->slave_drv);
	muc_svc_check_slave_present(drv);
	mutex_unlock(&slave_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(mods_register_slave_ctrl_driver);

void mods_unregister_slave_ctrl_driver(struct mods_slave_ctrl_driver *drv)
{
	if (!svc_dd)
		return;

	mutex_lock(&slave_lock);
	list_del(&drv->list);
	mutex_unlock(&slave_lock);
}
EXPORT_SYMBOL_GPL(mods_unregister_slave_ctrl_driver);

static inline uint16_t
svc_get_unsigned_manifest_size(struct mods_dl_device *mods_dev)
{
	struct greybus_manifest_header *hdr;

	if (!mods_dev->manifest || !mods_dev->manifest_size)
		return 0;

	hdr = (struct greybus_manifest_header *)mods_dev->manifest;

	return le16_to_cpu(hdr->size);
}

static int
svc_filter_ap_control_ver(struct mods_dl_device *orig_dev,
			uint8_t *payload, size_t size)
{
	struct muc_msg *mm = (struct muc_msg *)payload;
	struct gb_message msg;
	struct device *dev = &svc_dd->pdev->dev;
	struct gb_protocol_version_response resp;
	int ret;

	msg.header = (struct gb_operation_msg_hdr *)mm->gb_msg;

	resp.major = orig_dev->gb_ctrl_major;
	resp.minor = orig_dev->gb_ctrl_minor;

	ret = svc_gb_send_response(orig_dev, le16_to_cpu(mm->hdr.cport), &msg,
					sizeof(resp), &resp, GB_OP_SUCCESS);
	if (ret)
		dev_err(dev, "[%d] Failed to route CONTROL version\n",
			orig_dev->intf_id);

	return ret;
}

static int
svc_filter_ap_manifest_size(struct mods_dl_device *orig_dev,
			uint8_t *payload, size_t size)
{
	struct muc_msg *mm = (struct muc_msg *)payload;
	struct gb_message msg;
	struct device *dev = &svc_dd->pdev->dev;
	struct gb_control_get_manifest_size_response resp;
	int ret;

	msg.header = (struct gb_operation_msg_hdr *)mm->gb_msg;

	/* Get the unsigned manifest size */
	resp.size = svc_get_unsigned_manifest_size(orig_dev);
	if (!resp.size)
		return -EINVAL;

	ret = svc_gb_send_response(orig_dev, le16_to_cpu(mm->hdr.cport), &msg,
					sizeof(resp), &resp, GB_OP_SUCCESS);
	if (ret)
		dev_err(dev, "[%d] Failed to route manifest size\n",
			orig_dev->intf_id);

	return ret;
}

static int
svc_filter_ap_manifest(struct mods_dl_device *orig_dev,
			uint8_t *payload, size_t size)
{
	struct muc_msg *mm = (struct muc_msg *)payload;
	struct gb_message msg;
	struct device *dev = &svc_dd->pdev->dev;
	uint16_t mnf_size;
	int ret;

	msg.header = (struct gb_operation_msg_hdr *)mm->gb_msg;

	/* Only allocate the actual manifest size, not signed */
	mnf_size = svc_get_unsigned_manifest_size(orig_dev);
	if (!mnf_size)
		return -EINVAL;

	/* We skip intermediate copy to 'get_manifest_response' */
	ret = svc_gb_send_response(orig_dev, le16_to_cpu(mm->hdr.cport), &msg,
					mnf_size, orig_dev->manifest,
					GB_OP_SUCCESS);
	if (ret)
		dev_err(dev, "[%d] Failed to route manifest\n",
			orig_dev->intf_id);

	return ret;
}

static int
svc_filter_ready_to_boot(struct mods_dl_device *orig_dev,
			uint8_t *payload, size_t size)
{
	struct device *dev = &svc_dd->pdev->dev;

	dev_info(dev, "[%d] Firmware flashing complete; resetting\n",
		orig_dev->intf_id);

	muc_simulate_reset();

	return 0;
}

static int
svc_filter_ap_connected(struct mods_dl_device *orig_dev,
			uint8_t *payload, size_t size)
{
	struct mb_control_connected_request conn;
	struct gb_control_connected_request *req;
	struct gb_message *msg;
	struct muc_msg *mm = (struct muc_msg *)payload;
	struct gb_operation_msg_hdr *hdr;

	/* Pull out the cport ID from the connected request */
	hdr = (struct gb_operation_msg_hdr *)mm->gb_msg;
	req = (struct gb_control_connected_request *)(hdr + 1);
	conn.cport_id = req->cport_id;

	msg = svc_gb_msg_send_sync(svc_dd->dld, (uint8_t *)&conn,
				MB_CONTROL_TYPE_PORT_CONNECTED,
				sizeof(conn),
				SVC_VENDOR_CTRL_CPORT(orig_dev->intf_id));

	if (IS_ERR(msg)) {
		dev_err(&svc_dd->pdev->dev, "[%d] Failed send CONNECTED\n",
			orig_dev->intf_id);
		return -ENOENT;
	}

	svc_gb_msg_free(msg);

	/* Return -ENOENT so the message still routed to the interface */
	return -ENOENT;
}

static int
svc_filter_ap_disconnected(struct mods_dl_device *orig_dev,
			uint8_t *payload, size_t size)
{
	struct mb_control_disconnected_request conn;
	struct gb_control_disconnected_request *req;
	struct gb_message *msg;
	struct muc_msg *mm = (struct muc_msg *)payload;
	struct gb_operation_msg_hdr *hdr;

	/* Pull out the cport ID from the disconnected request */
	hdr = (struct gb_operation_msg_hdr *)mm->gb_msg;
	req = (struct gb_control_disconnected_request *)(hdr + 1);
	conn.cport_id = req->cport_id;

	msg = svc_gb_msg_send_sync(svc_dd->dld, (uint8_t *)&conn,
				MB_CONTROL_TYPE_PORT_DISCONNECTED,
				sizeof(conn),
				SVC_VENDOR_CTRL_CPORT(orig_dev->intf_id));

	if (IS_ERR(msg)) {
		dev_err(&svc_dd->pdev->dev, "[%d] Failed send DISCONNECTED\n",
			orig_dev->intf_id);
		return -ENOENT;
	}

	svc_gb_msg_free(msg);

	/* Return -ENOENT so the message still routed to the interface */
	return -ENOENT;
}

struct mods_nw_msg_filter svc_ap_filters[] = {
	{
		.protocol_id = GREYBUS_PROTOCOL_CONTROL,
		.type = GB_REQUEST_TYPE_PROTOCOL_VERSION,
		.filter_handler = svc_filter_ap_control_ver,
	},
	{
		.protocol_id = GREYBUS_PROTOCOL_CONTROL,
		.type = GB_CONTROL_TYPE_GET_MANIFEST_SIZE,
		.filter_handler = svc_filter_ap_manifest_size,
	},
	{
		.protocol_id = GREYBUS_PROTOCOL_CONTROL,
		.type = GB_CONTROL_TYPE_GET_MANIFEST,
		.filter_handler = svc_filter_ap_manifest,
	},
	{
		.protocol_id = GREYBUS_PROTOCOL_CONTROL,
		.type = GB_CONTROL_TYPE_CONNECTED,
		.filter_handler = svc_filter_ap_connected,
	},
	{
		.protocol_id = GREYBUS_PROTOCOL_CONTROL,
		.type = GB_CONTROL_TYPE_DISCONNECTED,
		.filter_handler = svc_filter_ap_disconnected,
	},
	{
		.protocol_id = GREYBUS_PROTOCOL_FIRMWARE,
		.type = GB_FIRMWARE_TYPE_READY_TO_BOOT,
		.filter_handler = svc_filter_ready_to_boot,
	},
};

static int muc_svc_install_ap_filters(struct muc_svc_data *dd)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(svc_ap_filters); i++) {
		ret = mods_nw_register_filter(&svc_ap_filters[i]);
		if (ret)
			goto free_filters;
	}

	return 0;

free_filters:
	for (--i; i >= 0; i--)
		mods_nw_unregister_filter(&svc_ap_filters[i]);

	return ret;
}

static void muc_svc_remove_ap_filters(struct muc_svc_data *dd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(svc_ap_filters); i++)
		mods_nw_unregister_filter(&svc_ap_filters[i]);
}

/* Handle the muc_msg and strip out its envelope to pass along the
 * actual gb_message we're interested in.
 */
static int
muc_svc_msg_send(struct mods_dl_device *dld, uint8_t *buf, size_t len)
{
	struct muc_msg *m = (struct muc_msg *)buf;

	return svc_gb_msg_recv(dld, m->gb_msg, (len - sizeof(m->hdr)),
				le16_to_cpu(m->hdr.cport));
}

static struct mods_dl_driver muc_svc_dl_driver = {
	.message_send = muc_svc_msg_send,
};

static int muc_svc_of_parse(struct muc_svc_data *dd, struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;
	int val;

	if (!np)
		return -EINVAL;

	dd->authenticate = of_property_read_bool(np, "mmi,use-authentication");

	ret = of_property_read_u32(np, "mmi,endo-mask", &val);
	if (ret) {
		dev_err(dev, "Failed to retrieve endo-mask: %d\n", ret);
		return ret;
	}
	dd->endo_mask = (u16)val;

	return 0;
}

static int muc_svc_send_reboot(struct mods_dl_device *mods_dev, uint8_t mode)
{
	int ret;
	struct mb_control_reboot_request req;

	req.mode = mode;

	ret = svc_gb_msg_send_no_resp(svc_dd->dld, (uint8_t *)&req,
				MB_CONTROL_TYPE_REBOOT, sizeof(req),
				SVC_VENDOR_CTRL_CPORT(mods_dev->intf_id));

	return ret;
}

static int muc_svc_enter_fw_flash(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct muc_svc_data *dd = platform_get_drvdata(pdev);
	struct mods_dl_device *mods_dev;
	uint8_t mode = MB_CONTROL_REBOOT_MODE_BOOTLOADER;

	/* Need to generate a hot unplug for each interface and then
	 * issue the reboot command */
	mutex_lock(&svc_list_lock);
	list_for_each_entry(mods_dev, &dd->ext_intf, list) {
		muc_svc_generate_unplug(mods_dev);

		/* XXX What to do if a single interface fails? Clear the
		 * other ones?
		 */
		if (muc_svc_send_reboot(mods_dev, mode))
			dev_warn(dev, "INTF: %d, failed to enter flashmode\n",
				mods_dev->intf_id);
	}
	mutex_unlock(&svc_list_lock);

	/* Reset the muc, to trigger the tear-down and re-init */
	muc_simulate_reset();

	return 0;
}

static ssize_t flashmode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val != 1) {
		dev_err(dev, "Invalid FLASH Mode\n");
		return -EINVAL;
	}

	dev_info(dev, "Entering FLASH mode\n");
	if (muc_svc_enter_fw_flash(dev))
		return -EINVAL;

	return count;
}
static DEVICE_ATTR_WO(flashmode);

static struct attribute *muc_svc_base_attrs[] = {
	&dev_attr_flashmode.attr,
	NULL,
};
ATTRIBUTE_GROUPS(muc_svc_base);

static int muc_svc_base_sysfs_init(struct muc_svc_data *dd)
{
	struct platform_device *pdev = dd->pdev;
	int ret;

	/* Create an 'interfaces' directory in sysfs */
	dd->intf_kset = kset_create_and_add("mods_interfaces", NULL,
						&pdev->dev.kobj);
	if (!dd->intf_kset) {
		dev_err(&pdev->dev, "Failed to create 'interfaces' sysfs\n");
		return -ENOMEM;
	}

	ret = sysfs_create_groups(&pdev->dev.kobj, muc_svc_base_groups);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create base sysfs attr\n");
		goto free_kset;
	}

	return 0;

free_kset:
	kset_unregister(dd->intf_kset);
	dd->intf_kset = NULL;

	return ret;
}

static void muc_svc_base_sysfs_exit(struct muc_svc_data *dd)
{
	sysfs_remove_groups(&dd->pdev->dev.kobj, muc_svc_base_groups);
	kset_unregister(dd->intf_kset);
}

static int muc_svc_probe(struct platform_device *pdev)
{
	struct muc_svc_data *dd;
	int ret;

	dd = devm_kzalloc(&pdev->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	ret = muc_svc_of_parse(dd, &pdev->dev);
	if (ret)
		return ret;

	dd->dld = _mods_create_dl_device(&muc_svc_dl_driver, &pdev->dev,
			MODS_INTF_SVC);
	if (IS_ERR(dd->dld)) {
		dev_err(&pdev->dev, "Failed to create mods DL device.\n");
		return PTR_ERR(dd->dld);
	}

	dd->wq = alloc_workqueue("muc_svc_attach", WQ_UNBOUND, 1);
	if (!dd->wq) {
		dev_err(&pdev->dev, "Failed to create attach workqueue.\n");
		ret = -ENOMEM;
		goto free_dl_dev;
	}

	dd->dld->dl_priv = dd;

	dd->pdev = pdev;
	atomic_set(&dd->msg_num, 1);
	INIT_LIST_HEAD(&dd->operations);
	INIT_LIST_HEAD(&dd->ext_intf);
	INIT_LIST_HEAD(&dd->slave_drv);

	/* Create the core sysfs structure */
	ret = muc_svc_base_sysfs_init(dd);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create base sysfs\n");
		goto free_wq;
	}

	ret = muc_svc_install_ap_filters(svc_dd);
	if (ret) {
		dev_err(&pdev->dev, "Failed to install nw filters\n");
		goto free_kset;
	}

	platform_set_drvdata(pdev, dd);

	svc_dd = dd;

	dd->attach_nb.notifier_call = muc_svc_attach;
	register_muc_attach_notifier(&dd->attach_nb);

	/* XXX Let's re-notify user space the device is added... since the
	 * OF framework will create our platform device during parsing,
	 * userspace won't be able to know new sysfs entries have been
	 * created....
	 */
	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);

	return 0;

free_kset:
	kset_unregister(dd->intf_kset);
free_wq:
	destroy_workqueue(dd->wq);
free_dl_dev:
	mods_remove_dl_device(dd->dld);

	return ret;
}

static int muc_svc_remove(struct platform_device *pdev)
{
	struct muc_svc_data *dd = platform_get_drvdata(pdev);

	unregister_muc_attach_notifier(&dd->attach_nb);
	muc_svc_remove_ap_filters(dd);
	muc_svc_base_sysfs_exit(dd);
	destroy_workqueue(dd->wq);
	mods_remove_dl_device(dd->dld);

	return 0;
}

static struct of_device_id muc_svc_match_tbl[] = {
	{ .compatible = "mmi,muc_svc" },
	{ },
};
MODULE_DEVICE_TABLE(of, muc_svc_match_tbl);

static struct platform_driver muc_svc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "muc_svc",
		.of_match_table = muc_svc_match_tbl,
	},
	.probe = muc_svc_probe,
	.remove  = muc_svc_remove,
};

int __init muc_svc_init(void)
{
	int ret;

	ret = platform_driver_register(&muc_svc_driver);
	if (ret < 0) {
		pr_err("muc_svc failed to register driver\n");
		return ret;
	}

	return 0;
}

void __exit muc_svc_exit(void)
{
	platform_driver_unregister(&muc_svc_driver);
}
