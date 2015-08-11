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
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>

#include "greybus.h"

#include "muc_svc.h"
#include "mods_nw.h"

struct muc_svc_data {
	struct mods_dl_device *dld;
	atomic_t msg_num;
	struct list_head operations;
	struct platform_device *pdev;
};

/* XXX Move these into device tree? */
#define MUC_SVC_AP_INTF_ID 1
#define MUC_SVC_ENDO_ID 0x4755

#define MUC_SVC_RESPONSE_TYPE 0

#define SVC_MSG_TIMEOUT 500

void muc_svc_attach(struct greybus_host_device *hd)
{
}
EXPORT_SYMBOL(muc_svc_attach);

void muc_svc_detach(struct greybus_host_device *hd)
{
}
EXPORT_SYMBOL(muc_svc_detach);

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
svc_route_msg(struct mods_dl_device *dld, uint8_t src_cport,
		uint8_t dest_cport, struct gb_message *msg)
{
	struct muc_msg *m;
	size_t muc_payload = get_gb_msg_size(msg);
	size_t msg_size = muc_payload + sizeof(m->hdr);

	m = kmalloc(msg_size, GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	memcpy(m->gb_msg, msg->buffer, muc_payload);
	m->hdr.size = msg->header->size;
	m->hdr.dest_cport = dest_cport;
	m->hdr.src_cport = src_cport;

	mods_data_rcvd(dld, (uint8_t *)m);
	kfree(m);

	return 0;
}

/* Handle the incoming greybus message and complete the waiting thread */
/* XXX This only accounts for responses.... need to determine if its response
 * or a brand new message to the SVC (even possible? AP set route, maybe).
 */
static int
svc_gb_msg_recv(struct mods_dl_device *dld, uint8_t *data,
		size_t msg_size, uint8_t cport)
{
	struct muc_svc_data *dd = dld_get_dd(dld);
	struct svc_op *op;
	size_t payload_size;
	struct gb_operation_msg_hdr hdr;

	payload_size = msg_size - sizeof(struct gb_operation_msg_hdr);
	if (msg_size < sizeof(hdr)) {
		dev_err(&dd->pdev->dev, "msg size too small: %zu\n", msg_size);
		return -EINVAL;
	}

	memcpy(&hdr, data, sizeof(hdr));

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
	} else {
		/* This is a new incoming request! */
	}

	return 0;
}

/* Send a message out the specified CPORT and wait for a response */
static struct gb_message *
svc_gb_msg_send_sync(struct mods_dl_device *dld, uint8_t *data, uint8_t type,
		size_t payload_size, uint8_t src_cport, uint8_t dest_cport)
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

	cycle = (u16)atomic_inc_return(&dd->msg_num);
	op->msg_id = cycle % U16_MAX + 1;
	init_completion(&op->completion);

	msg->header->operation_id = cpu_to_le16(op->msg_id);
	if (payload_size)
		memcpy(msg->payload, data, payload_size);
	op->request = msg;

	list_add_tail(&op->entry, &dd->operations);

	/* Send to NW Routing Layer */
	ret = svc_route_msg(dld, src_cport, dest_cport, msg);
	if (ret) {
		dev_err(&dd->pdev->dev, "failed sending svc msg: %d\n", ret);
		goto remove_op;
	}

	ret = wait_for_completion_interruptible_timeout(&op->completion,
					msecs_to_jiffies(SVC_MSG_TIMEOUT));
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

	/* XXX Check result here? */
	if (msg->header->result) {
		svc_gb_msg_free(op->response);
		return ERR_PTR(-EINVAL);
	}

	return msg;

remove_op:
	list_del(&op->entry);
	svc_gb_msg_free(op->request);
gb_msg_alloc:
	kfree(op);

	return ERR_PTR(ret);
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

	msg = svc_gb_msg_send_sync(dld, (uint8_t *)ver, GB_SVC_TYPE_PROTOCOL_VERSION,
				sizeof(*ver), 0, 0);
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
	hello->endo_id = cpu_to_le16(MUC_SVC_ENDO_ID);
	hello->interface_id = ap_intf_id;

	msg = svc_gb_msg_send_sync(dld, (uint8_t *)hello, GB_SVC_TYPE_SVC_HELLO,
				sizeof(*hello), 0, 0);
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

/* Handle the muc_msg and strip out its envelope to pass along the
 * actual gb_message we're interested in.
 */
static int
muc_svc_msg_send(struct mods_dl_device *dld, uint8_t *buf, size_t len)
{
	struct muc_msg *m = (struct muc_msg *)buf;

	return svc_gb_msg_recv(dld, m->gb_msg, m->hdr.size, m->hdr.dest_cport);
}

static void muc_svc_msg_cancel(void *cookie)
{
	/* Should never happen */
}

static struct mods_dl_driver muc_svc_dl_driver = {
	.dl_priv_size = sizeof(struct muc_svc_data),
	.message_send = muc_svc_msg_send,
	.message_cancel = muc_svc_msg_cancel,
};

static int muc_svc_probe(struct platform_device *pdev)
{
	struct muc_svc_data *dd;
	int ret;

	dd = devm_kzalloc(&pdev->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	dd->dld = mods_create_dl_device(&muc_svc_dl_driver, &pdev->dev);
	if (IS_ERR(dd->dld)) {
		dev_err(&pdev->dev, "Failed to create mods DL device.\n");
		return PTR_ERR(dd->dld);
	}
	dd->dld->dl_priv = dd;

	dd->pdev = pdev;
	atomic_set(&dd->msg_num, 1);
	INIT_LIST_HEAD(&dd->operations);

	platform_set_drvdata(pdev, dd);

	/* We are a single entity, AP is always attached */
	ret = muc_svc_probe_ap(dd->dld, MUC_SVC_AP_INTF_ID);
	if (ret) {
		dev_err(&pdev->dev, "Failed to probe the AP: %d\n", ret);
		goto ap_probe_fail;
	}
	return 0;

ap_probe_fail:
	mods_remove_dl_device(dd->dld);

	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int muc_svc_remove(struct platform_device *pdev)
{
	struct muc_svc_data *dd = platform_get_drvdata(pdev);

	mods_remove_dl_device(dd->dld);

	return 0;
}

static struct platform_driver muc_svc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "muc_svc",
	},
	.probe = muc_svc_probe,
	.remove  = muc_svc_remove,
};

static struct platform_device *muc_svc_device;

int __init muc_svc_init(void)
{
	int ret;

	ret = platform_driver_register(&muc_svc_driver);
	if (ret < 0) {
		pr_err("muc_svc failed to register driver\n");
		return ret;
	}

	muc_svc_device = platform_device_alloc("muc_svc", -1);
	if (!muc_svc_device) {
		ret = -ENOMEM;
		pr_err("muc_svc failed to alloc device\n");
		goto alloc_fail;
	}

	ret = platform_device_add(muc_svc_device);
	if (ret) {
		pr_err("muc_svc failed to add device: %d\n", ret);
		goto add_fail;
	}

	return 0;

add_fail:
	platform_device_put(muc_svc_device);
alloc_fail:
	platform_driver_unregister(&muc_svc_driver);

	return ret;
}

void __exit muc_svc_exit(void)
{
	platform_driver_unregister(&muc_svc_driver);
	platform_device_unregister(muc_svc_device);
}
