/*
 * Copyright (C) 2016 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/usb.h>

#include "muc.h"
#include "muc_svc.h"

#define SIM_GBUF_MSG_SIZE_MAX	2048

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x18d1, 0x1eaf) },
	{ },
};

#define NUM_BULK_IN_URB	(4 * 1)

/* Number of CPort OUT urbs in flight at any point in time.
 * Adjust if we get messages saying we are out of urbs in the system log.
 */
#define NUM_BULK_OUT_URB	(8 * 1)

struct sim_urb {
	struct urb      *urb;
	bool            busy;
	unsigned char   *buffer;
	size_t          size;
};

static bool is_rx_tx_enabled;

struct muc_sim_data {
	struct usb_device *usb_dev;
	struct usb_interface *usb_intf;
	struct mods_dl_device *dld;

	struct notifier_block reset_nb;

	bool present;	/* MuC is physically present */
	bool attached;	/* MuC attach is reported to SVC */

	struct work_struct attach_work;	/* Worker to send attach to SVC */

	/* the urb to read data with */
	struct sim_urb bulk_in_urb[NUM_BULK_IN_URB];
	/* the address of the bulk in endpoint */
	__u8 bulk_in_endpointAddr;

	struct sim_urb bulk_out_urb[NUM_BULK_OUT_URB];
	/* the address of the bulk out endpoint */
	__u8 bulk_out_endpointAddr;
	spinlock_t bulk_out_urb_lock;
};

struct cport_data {
	struct muc_sim_data	*sim_data;
	unsigned char	buff[SIM_GBUF_MSG_SIZE_MAX];
	size_t	len;	/* number of bytes in the buffer */
	struct work_struct	worker;
};

static inline void dump(void *data, size_t size)
{
	uint8_t *buf = data;
	int i;

	pr_debug("muc_sim: DUMP ->");
	for (i = 0; i < size; i++)
		pr_debug(" %02hhx", buf[i]);
	pr_debug("\n");
}

static struct sim_urb *next_free_urb(struct muc_sim_data *dd, gfp_t gfp_mask)
{
	struct sim_urb *urb = NULL;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&dd->bulk_out_urb_lock, flags);
	for (i = 0; i < NUM_BULK_OUT_URB; ++i) {
		if (dd->bulk_out_urb[i].busy == false) {
			dd->bulk_out_urb[i].busy = true;
			urb = &dd->bulk_out_urb[i];
			break;
		}
	}
	spin_unlock_irqrestore(&dd->bulk_out_urb_lock, flags);
	return urb;
}

static void free_urb(struct urb *urb)
{
	unsigned long flags;
	int i;
	struct muc_sim_data *dd = urb->context;

	spin_lock_irqsave(&dd->bulk_out_urb_lock, flags);
	for (i = 0; i < NUM_BULK_OUT_URB; ++i) {
		if (urb == dd->bulk_out_urb[i].urb) {
			dd->bulk_out_urb[i].busy = false;
			memset(dd->bulk_out_urb[i].buffer,
					0, dd->bulk_out_urb[i].size);
			break;
		}
	}
	spin_unlock_irqrestore(&dd->bulk_out_urb_lock, flags);
}

static void bulk_out_callback(struct urb *urb)
{
	pr_debug("bulk_out_callback\n");
	free_urb(urb);
}

static int __muc_sim_message_send(struct muc_sim_data *dd,
				  uint8_t *buf, size_t len)
{
	int ret = 0;
	struct sim_urb *urb;

	pr_debug("__muc_sim_message_send\n");

	if (len > SIM_GBUF_MSG_SIZE_MAX) {
		pr_err("%s: urb->buffer too large %zu\n",
			__func__, len);
		return -EINVAL;
	}

	/* Find a free urb */
	urb = next_free_urb(dd, GFP_KERNEL);
	if (!urb)
		return -ENOMEM;

	memcpy(urb->buffer, buf, len);

	usb_fill_bulk_urb(urb->urb, dd->usb_dev,
			  usb_sndbulkpipe(dd->usb_dev,
					  dd->bulk_out_endpointAddr),
			  urb->buffer, len,
			  bulk_out_callback, dd);
	urb->urb->transfer_flags |= URB_ZERO_PACKET;
	ret = usb_submit_urb(urb->urb, GFP_KERNEL);
	if (ret) {
		free_urb(urb->urb);

		return ret;
	}

	return 0;
}

static int muc_sim_message_send(struct mods_dl_device *dld,
				uint8_t *buf, size_t len)
{
	struct muc_sim_data *dd = (struct muc_sim_data *)dld->dl_priv;

	if (is_rx_tx_enabled == false) {
		pr_debug("rx tx disable\n");

		return -EIO;
	}

	dump(buf, len);
	return __muc_sim_message_send(dd, buf, len);
}


static struct mods_dl_driver muc_sim_dl_driver = {
	.message_send		= muc_sim_message_send,
};

static void cport_in_worker(struct work_struct *work)
{
	struct cport_data *cd = container_of(work, struct cport_data, worker);

	pr_debug("cport_in_worker\n");
	if (is_rx_tx_enabled)
		mods_nw_switch(cd->sim_data->dld, &cd->buff[0], cd->len);
	else
		pr_debug("rx tx disable\n");

	kfree(cd);
}

static void bulk_in_callback(struct urb *urb)
{
	struct muc_sim_data *dd = (struct muc_sim_data *) urb->context;
	struct cport_data *cd;
	int retval;

	pr_debug("bulk_in_callback\n");

	if (is_rx_tx_enabled == false) {
		pr_debug("rx tx disable\n");
		goto exit;
	}

	cd = kmalloc(sizeof(struct cport_data), GFP_ATOMIC);
	if (!cd)
		goto exit;

	cd->sim_data = dd;
	cd->len = urb->actual_length;
	memcpy(&cd->buff[0], urb->transfer_buffer, urb->actual_length);
	INIT_WORK(&cd->worker, cport_in_worker);
	schedule_work(&cd->worker);

exit:
	/* put our urb back in the request pool */
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(&dd->usb_dev->dev,
				"failed to resubmit in-urb: %d\n", retval);
}

static int bulk_in_enable(struct muc_sim_data *dd)
{
	struct urb *urb;
	int ret;
	int i;

	for (i = 0; i < NUM_BULK_IN_URB; ++i) {
		urb = dd->bulk_in_urb[i].urb;

		ret = usb_submit_urb(urb, GFP_KERNEL);
		if (ret) {
			pr_debug("failed to submit in-urb: %d\n", ret);
			goto err_kill_urbs;
		}
	}

	return 0;

err_kill_urbs:
	for (--i; i >= 0; --i) {
		urb = dd->bulk_in_urb[i].urb;
		usb_kill_urb(urb);
	}

	return ret;
}

static void bulk_in_disable(struct muc_sim_data *dd)
{
	struct urb *urb;
	int i;

	is_rx_tx_enabled = false;

	for (i = 0; i < NUM_BULK_IN_URB; ++i) {
		urb = dd->bulk_in_urb[i].urb;
		usb_kill_urb(urb);
	}
}

static void muc_sim_destroy(struct muc_sim_data *dd)
{
	struct usb_device *udev;
	int i;

	bulk_in_disable(dd);

	flush_work(&dd->attach_work);
	if (dd->attached) {
		mods_dl_dev_detached(dd->dld);
		dd->attached = false;
	}

	if (dd->dld)
		mods_remove_dl_device(dd->dld);

	for (i = 0; i < NUM_BULK_OUT_URB; ++i) {
		struct urb *urb = dd->bulk_out_urb[i].urb;

		if (!urb)
			break;
		usb_kill_urb(urb);
		usb_free_urb(urb);
		dd->bulk_out_urb[i].urb = NULL;
		dd->bulk_out_urb[i].busy = false;	/* just to be anal */
		kfree(dd->bulk_out_urb[i].buffer);
	}

	for (i = 0; i < NUM_BULK_IN_URB; i++) {
		struct urb *urb = dd->bulk_in_urb[i].urb;

		if (!urb)
			break;
		usb_free_urb(urb);
		kfree(dd->bulk_in_urb[i].buffer);
		dd->bulk_in_urb[i].buffer = NULL;
	}

	udev = dd->usb_dev;
	usb_put_dev(udev);
}

static void attach_worker(struct work_struct *work)
{
	struct muc_sim_data *dd = container_of(work, struct muc_sim_data,
					       attach_work);
	int ret;

	ret = mods_dl_dev_attached(dd->dld);
	if (ret)
		dev_err(&dd->usb_dev->dev,
				"Error (%d) attaching to SVC\n", ret);
	else
		dd->attached = true;
}

static int
muc_reset_nb(struct notifier_block *nb, unsigned long val, void *not_used)
{
	struct muc_sim_data *dd =
			container_of(nb, struct muc_sim_data, reset_nb);
	struct usb_device *udev = dd->usb_dev;

	dev_info(&udev->dev, "%s: we got a reset\n", __func__);

	flush_work(&dd->attach_work);
	if (dd->attached) {
		mods_dl_dev_detached(dd->dld);
		dd->attached = false;
	}
	schedule_work(&dd->attach_work);

	return NOTIFY_OK;
}

static int bulk_in_allocate_urbs(struct muc_sim_data *dd)
{
	struct urb *urb;
	unsigned char *buffer;
	int j;

	for (j = 0; j < NUM_BULK_IN_URB; j++) {
		dd->bulk_in_urb[j].urb = usb_alloc_urb(0, GFP_KERNEL);
		urb = dd->bulk_in_urb[j].urb;
		if (!urb)
			return -ENOMEM;

		dd->bulk_in_urb[j].size = SIM_GBUF_MSG_SIZE_MAX;
		dd->bulk_in_urb[j].buffer =
				kmalloc(dd->bulk_in_urb[j].size, GFP_KERNEL);
		buffer = dd->bulk_in_urb[j].buffer;
		if (!buffer)
			return -ENOMEM;

		usb_fill_bulk_urb(urb, dd->usb_dev,
				usb_rcvbulkpipe(dd->usb_dev, dd->bulk_in_endpointAddr),
				buffer, dd->bulk_in_urb[j].size,
				bulk_in_callback, dd);
	}
	return 0;
}

static int bulk_out_allocate_urbs(struct muc_sim_data *dd)
{
	struct urb *urb;
	unsigned char *buffer;
	int j;

	for (j = 0; j < NUM_BULK_OUT_URB; j++) {
		dd->bulk_out_urb[j].size = SIM_GBUF_MSG_SIZE_MAX;
		dd->bulk_out_urb[j].urb = usb_alloc_urb(0, GFP_KERNEL);
		urb = dd->bulk_out_urb[j].urb;
		if (!urb)
			return -ENOMEM;

		dd->bulk_out_urb[j].buffer =
				kmalloc(dd->bulk_out_urb[j].size, GFP_KERNEL);
		buffer = dd->bulk_out_urb[j].buffer;
		if (!buffer)
			return -ENOMEM;

		dd->bulk_out_urb[j].busy = false;
	}
	return 0;
}

static int muc_sim_probe(struct usb_interface *interface,
		    const struct usb_device_id *id)
{
	struct muc_sim_data *dd;
	struct usb_device *udev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i, retval;

	dev_info(&interface->dev, "muc_sim_probe\n");

	dd = devm_kzalloc(&interface->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	usb_set_intfdata(interface, dd);
	udev = usb_get_dev(interface_to_usbdev(interface));
	dd->usb_dev = udev;
	dd->usb_intf = interface;
	dd->reset_nb.notifier_call = muc_reset_nb;

	dd->dld = mods_create_dl_device(&muc_sim_dl_driver,
				&interface->dev, MODS_INTF_SIM);

	if (IS_ERR(dd->dld)) {
		dev_err(&interface->dev, "%s: Unable to create data link device\n",
				__func__);
		usb_put_dev(udev);
		return PTR_ERR(dd->dld);
	}

	dd->dld->dl_priv = (void *)dd;
	spin_lock_init(&dd->bulk_out_urb_lock);
	INIT_WORK(&dd->attach_work, attach_worker);

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dd->bulk_in_endpointAddr &&
			usb_endpoint_is_bulk_in(endpoint)) {
			/* we found a bulk in endpoint */
			dd->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev_info(&interface->dev,
					"bulk_in_endpointAddr: %d\n",
					dd->bulk_in_endpointAddr);

			retval = bulk_in_allocate_urbs(dd);
			if (retval) {
				dev_err(&interface->dev,
					"Could not allocate bulk_out_buffer\n");
				goto error;
			}

			bulk_in_enable(dd);
		}

		if (!dd->bulk_out_endpointAddr &&
			usb_endpoint_is_bulk_out(endpoint)) {
			/* we found a bulk out endpoint */
			dd->bulk_out_endpointAddr = endpoint->bEndpointAddress;
			dev_info(&interface->dev,
					"bulk_out_endpointAddr: %d\n",
					dd->bulk_out_endpointAddr);

			retval = bulk_out_allocate_urbs(dd);
			if (retval) {
				dev_err(&interface->dev,
					"Could not allocate bulk_out_buffer\n");
				goto error;
			}
		}
	}

	if (!(dd->bulk_in_endpointAddr && dd->bulk_out_endpointAddr)) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	is_rx_tx_enabled = true;

	schedule_work(&dd->attach_work);

	register_muc_reset_notifier(&dd->reset_nb);

	return 0;

error:
	muc_sim_destroy(dd);
	return -ENOMEM;
}

static void muc_sim_disconnect(struct usb_interface *interface)
{
	struct muc_sim_data *dd;

	pr_debug("muc_sim_disconnect\n");
	dd = usb_get_intfdata(interface);

	unregister_muc_reset_notifier(&dd->reset_nb);
	muc_sim_destroy(dd);
	usb_set_intfdata(interface, NULL);
}

static struct usb_driver muc_sim_driver = {
	.name =		"muc_sim",
	.probe =	muc_sim_probe,
	.disconnect =	muc_sim_disconnect,
	.id_table =	id_table,
};

module_usb_driver(muc_sim_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Motorola");
