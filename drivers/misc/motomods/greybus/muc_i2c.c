/*
 * Copyright (C) 2015-2016 Motorola Mobility, LLC.
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

#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "greybus.h"

#include "mods_nw.h"
#include "muc.h"
#include "muc_svc.h"

/* Protocol version supported by this driver */
#define PROTO_VER               (1)

#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

/*
 * Default size of a I2C packet (in bytes). This can be negotiated to be
 * larger, all the way up to the maximum size of a datagram.
 */
#define DEFAULT_PKT_SZ      PKT_SIZE(32)

/*
 * The maximum number of packets allowed for one datagram. For example, if
 * the packet size is the default size (32 bytes), the maximum supported
 * datagram size would be 2048 bytes (32 bytes x 64 packets). To transmit
 * larger datagrams, a larger packet size would need to be negotiated.
 *
 * This limit exists because there are only 6 bits in the I2C packet header
 * to indicate how many packets are remaining to complete the datagram.
 */
#define MAX_PKTS_PER_DG     (64)

/* I2C packet header bit definitions */
#define HDR_BIT_ACK    (0x01 << 10) /* 1 = MuC has ACK'd last packet sent */
#define HDR_BIT_DUMMY  (0x01 << 9)  /* 1 = dummy packet */
#define HDR_BIT_PKT1   (0x01 << 8)  /* 1 = first packet of message */
#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = packet has valid payload */
#define HDR_BIT_TYPE   (0x01 << 6)  /* I2C message type */
#define HDR_BIT_PKTS   (0x3F << 0)  /* How many additional packets to expect */

/* Possible values for HDR_BIT_TYPE */
#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

/* I2C packet CRC size (in bytes) */
#define CRC_SIZE       (2)

/* Size of the header in bytes */
#define HDR_SIZE       sizeof(struct i2c_msg_hdr)

/* Macro to determine the payload size from the packet size */
#define PL_SIZE(pkt_size)  (pkt_size - HDR_SIZE - CRC_SIZE)

/* Macro to determine the packet size from the payload size */
#define PKT_SIZE(pl_size)  (pl_size + HDR_SIZE + CRC_SIZE)

/* Macro to determine the location of the CRC in the packet */
#define CRC_NDX(pkt_size)  (pkt_size - CRC_SIZE)

#define RDY_TIMEOUT_JIFFIES     (HZ /  4) /* 250 milliseconds */
#define ACK_TIMEOUT_JIFFIES     (HZ / 10) /* 100 milliseconds */

/* The number of times to try sending a datagram to the MuC */
#define NUM_TRIES      (3)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/*
 * Wait until the specified condition is true, a timeout occurs, or MuC is
 * detached.
 */
#define WAIT_WHILE(cond, timeout, d)                                \
	{                                                           \
		unsigned long until = jiffies + timeout;            \
		while ((cond) && time_before_eq(jiffies, until) &&  \
			d->present)                                 \
			;                                           \
	}

/*
 * Possible data link layer messages. Responses IDs should be the request ID
 * with the MSB set.
 */
enum dl_msg_id {
	DL_MSG_ID_BUS_CFG_REQ         = 0x00,
	DL_MSG_ID_BUS_CFG_RESP        = 0x80,
};

struct muc_i2c_data {
	struct i2c_client *client;
	struct mods_dl_device *dld;
	bool present;                      /* MuC is physically present */
	bool attached;                     /* MuC attach is reported to SVC */
	struct notifier_block attach_nb;   /* attach/detach notifications */
	struct mutex mutex;                /* Used to serialize transfers */
	struct work_struct attach_work;    /* Worker to send attach to SVC */

	size_t pkt_size;                   /* Size of hdr + pl + CRC in bytes */

	__u8 *tx_pkt;                      /* Buffer for transmit packets */
	__u8 *tx_datagram;                 /* Buffer for transmit datagram */
	uint32_t tx_datagram_ndx;          /* Index into transmit datagram */
	uint8_t tx_pkts_remaining;         /* Packets needed to complete msg */
	size_t tx_datagram_len;            /* bytes */
	bool tx_ack_pending;

	__u8 *rx_pkt;                      /* Buffer for receive packets */
	__u8 *rx_datagram;                 /* Buffer for receive datagram */
	uint32_t rx_datagram_ndx;          /* Index into receive datagram */
	uint8_t rx_pkts_remaining;         /* Packets needed to complete msg */
	size_t rx_datagram_len;            /* bytes */
	bool rx_ack_pending;               /* received pkt needs ack */
	bool rx_first_pkt_rcvd;            /* first pkt of datagram received */
};

struct i2c_msg_hdr {
	__le16 bitmask;                    /* See HDR_BIT_* defines */
} __packed;

struct muc_i2c_msg {
	struct i2c_msg_hdr hdr;
	uint8_t data[0];
};

struct i2c_dl_msg_bus_config_req {
	__le16 max_pl_size;                /* Max payload size base supports */
	__u8   features;                   /* See DL_BIT_* defines for values */
	__u8   version;                    /* msg format version supported */
} __packed;

struct i2c_dl_msg_bus_config_resp {
	__le32 max_speed;                  /* Max bus speed mod supports */
	__le16 pl_size;                    /* Payload size mod selected */
	__u8   features;                   /* See DL_BIT_* defines for values */
	__u8   version;                    /* I2C msg format version of mod */
} __packed;

struct i2c_dl_msg {
	__u8 id;                           /* enum dl_msg_id */
	union {
		struct i2c_dl_msg_bus_config_req     bus_req;
		struct i2c_dl_msg_bus_config_resp    bus_resp;
	};
} __packed;

size_t muc_i2c_get_pkt_sz(size_t pl_size)
{
	return PKT_SIZE(pl_size);
}

static inline struct muc_i2c_data *dld_to_dd(struct mods_dl_device *dld)
{
	return (struct muc_i2c_data *)dld->dl_priv;
}

static inline bool check_rx_pkt_crc(struct muc_i2c_data *dd)
{
	uint16_t crc;
	uint16_t *snt_crc = (uint16_t *)&dd->rx_pkt[CRC_NDX(dd->pkt_size)];

	crc = crc16(0, dd->rx_pkt, CRC_NDX(dd->pkt_size));
	crc = cpu_to_le16(crc);

	return crc == *snt_crc;
}

static inline void set_tx_pkt_crc(struct muc_i2c_data *dd)
{
	uint16_t *crc = (uint16_t *)&dd->tx_pkt[CRC_NDX(dd->pkt_size)];

	*crc = crc16(0, dd->tx_pkt, CRC_NDX(dd->pkt_size));
	*crc = cpu_to_le16(*crc);
}

static int set_packet_size(struct muc_i2c_data *dd, size_t pkt_size)
{
	struct device *dev = &dd->client->dev;
	size_t pl_size = PL_SIZE(pkt_size);

	/*
	 * Verify new packet size is valid. The new size must be no smaller
	 * than the default packet size and payload must be a power of two.
	 */
	if (!is_power_of_2(pl_size) || (pkt_size < DEFAULT_PKT_SZ))
		return -EINVAL;

	/* Save the new packet size */
	dd->pkt_size = pkt_size;

	dev_info(dev, "Packet size is %zu bytes\n", pkt_size);

	return 0;
}

static int dl_recv(struct mods_dl_device *dld)
{
	struct muc_i2c_data *dd = dld_to_dd(dld);
	struct device *dev = &dd->client->dev;
	struct i2c_dl_msg *msg = (struct i2c_dl_msg *)dd->rx_datagram;
	size_t pl_size;
	int ret;

	/* Only BUS_CFG_RESP is supported */
	if (msg->id != DL_MSG_ID_BUS_CFG_RESP) {
		dev_err(dev, "Unknown ID (%d)!\n", msg->id);
		return -EINVAL;
	}

	if (dd->attached) {
		dev_warn(&dd->client->dev, "Trying to reconfigure bus!\n");
		return -EINVAL;
	}

	/* Ignore max_bus_speed for I2C */
	pl_size = le16_to_cpu(msg->bus_resp.pl_size);

	/* Ignore payload size if zero and continue to use default size */
	if (pl_size > 0) {
		/*
		 * Workaround for the size field in the message being only
		 * 16 bits. Need to add one to get even number.
		 */
		if (pl_size == U16_MAX)
			pl_size++;

		ret = set_packet_size(dd, PKT_SIZE(pl_size));
		if (ret) {
			dev_err(dev, "Error (%d) setting new packet size\n",
					ret);
			return ret;
		}
	}

	/* Schedule work to send attach to SVC */
	schedule_work(&dd->attach_work);

	return 0;
}

static void attach_worker(struct work_struct *work)
{
	struct muc_i2c_data *dd = container_of(work, struct muc_i2c_data,
					       attach_work);
	int ret;

	/* Verify mod is still present before reporting attach */
	if (!dd->present)
		return;

	ret = mods_dl_dev_attached(dd->dld);
	if (ret)
		dev_err(&dd->client->dev, "Error (%d) attaching to SVC\n", ret);
	else
		dd->attached = true;
}

static int dispatch_rx_dg(struct muc_i2c_data *dd,
		uint16_t bitmask, size_t size)
{
	if (unlikely((bitmask & HDR_BIT_TYPE) == MSG_TYPE_DL))
		return dl_recv(dd->dld);
	else
		return mods_nw_switch(dd->dld, dd->rx_datagram, size);
}

/* read a single i2c msg */
static int muc_i2c_read(struct muc_i2c_data *dd)
{
	struct i2c_msg msg[1];
	int ret;

	msg[0].addr = dd->client->addr;
	msg[0].flags = dd->client->flags | I2C_M_RD;
	msg[0].len = dd->pkt_size;
	msg[0].buf = dd->rx_pkt;

	memset(dd->rx_pkt, 0, dd->pkt_size);

	/* Wait for RDY to be asserted */
	WAIT_WHILE((ret = muc_gpio_get_ready_n()), RDY_TIMEOUT_JIFFIES, dd);
	if (ret) {
		dev_err(&dd->client->dev, "timeout waiting for ready\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = i2c_transfer(dd->client->adapter, msg, 1);

	if (!check_rx_pkt_crc(dd)) {
		dev_err(&dd->client->dev, "CRC mismatch\n");
		ret = -EIO;
	}
out:
	return ret;
}

static int muc_i2c_write(struct muc_i2c_data *dd)
{
	struct i2c_msg msg[1];
	int ret;

	msg[0].addr = dd->client->addr;
	msg[0].flags = dd->client->flags;
	msg[0].len = dd->pkt_size;
	msg[0].buf = dd->tx_pkt;

	set_tx_pkt_crc(dd);

	/* Wait for RDY to be asserted */
	WAIT_WHILE((ret = muc_gpio_get_ready_n()), RDY_TIMEOUT_JIFFIES, dd);
	if (ret) {
		dev_err(&dd->client->dev, "timeout waiting for ready\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = i2c_transfer(dd->client->adapter, msg, 1);

out:
	return ret;
}

int muc_i2c_transfer(struct muc_i2c_data *dd, __u16 msg_type)
{
	int ret = 0;
	int cnt;                /* byte count or err */
	bool do_write = false;  /* should we write */
	struct muc_i2c_msg *tx_msg = (struct muc_i2c_msg *)dd->tx_pkt;
	struct muc_i2c_msg *rx_msg = (struct muc_i2c_msg *)dd->rx_pkt;
	int num_tries_remaining = NUM_TRIES;
	int num_ack_tries_remaining = NUM_TRIES;
	bool rx_valid = false;

	tx_msg->hdr.bitmask = msg_type;

	if (dd->tx_pkts_remaining) {
		size_t remaining = dd->tx_datagram_len - dd->tx_datagram_ndx;
		uint8_t pkts_total = (dd->tx_datagram_len / dd->pkt_size) +
			((dd->tx_datagram_len % dd->pkt_size) > 0);
		tx_msg->hdr.bitmask |= HDR_BIT_VALID;
		tx_msg->hdr.bitmask |= dd->tx_pkts_remaining - 1;
		if (pkts_total == dd->tx_pkts_remaining)
			tx_msg->hdr.bitmask |= HDR_BIT_PKT1;
		memcpy(&tx_msg->data[0], &dd->tx_datagram[dd->tx_datagram_ndx],
		       MIN(remaining, dd->pkt_size));

		do_write = true;
	}

	if (dd->rx_ack_pending) {
		tx_msg->hdr.bitmask |= HDR_BIT_ACK;
		if (dd->tx_pkts_remaining == 0)
			tx_msg->hdr.bitmask |= HDR_BIT_DUMMY;
		do_write = true;
	}

retry_write:
	if (do_write) {
		if (muc_gpio_get_wake_n())
			muc_gpio_set_wake_n(0);     /* Assert WAKE */

		cnt = muc_i2c_write(dd);
		if (cnt < 0) {
			dev_err(&dd->client->dev, "I2C write error %d\n", cnt);
			muc_gpio_set_wake_n(1);     /* Deassert WAKE */
			if (--num_tries_remaining > 0)
				goto retry_write;
			return cnt;
		}
		num_tries_remaining = NUM_TRIES;

		/* STATE: we sent a real message, so expect an ack */
		if (tx_msg->hdr.bitmask & HDR_BIT_VALID) {
			dd->tx_ack_pending = true;
		}

		if (dd->rx_ack_pending) {
			/* STATE: RX ACK was pending and has now been sent */
			dd->rx_ack_pending = false;
			dd->rx_datagram_ndx += PL_SIZE(dd->pkt_size);

			if (dd->rx_pkts_remaining == 0) {
				__u16 rx_bm = le16_to_cpu(rx_msg->hdr.bitmask);

				/* STATE: We ACKED the last message of a
				   datagram, datagram is now complete
				   and can be sent up */
				dispatch_rx_dg(dd, rx_bm, dd->rx_datagram_ndx);
				memset(dd->rx_datagram, 0, dd->rx_datagram_len);
				ret = 0;
				dd->rx_first_pkt_rcvd = false;
				dd->rx_datagram_ndx = 0;
				dd->rx_datagram_len = 0;
			} else
				dd->rx_pkts_remaining--;
		}
	}


retry_read:
	/* read if we expect an ACK or the Mod has signaled with "int"
	 * that it has something to send */
	if (dd->tx_ack_pending || !muc_gpio_get_int_n()) {
		rx_valid = false;

		if (muc_gpio_get_wake_n())
			muc_gpio_set_wake_n(0);         /* Assert WAKE */

		cnt = muc_i2c_read(dd);
		if (cnt < 0) {
			dev_err(&dd->client->dev, "I2C read error %d\n", cnt);
			muc_gpio_set_wake_n(1);     /* Deassert WAKE */
			if (--num_tries_remaining > 0) {
				goto retry_read;
			}
			return cnt;
		}

		if ((dd->tx_ack_pending) &&
				(le16_to_cpu(rx_msg->hdr.bitmask) &
				HDR_BIT_ACK)) {
			/* STATE: Expected an ACK
					- Got an ACK pkt send is complete */
			dd->tx_ack_pending = false;
			dd->tx_datagram_ndx += PL_SIZE(dd->pkt_size);
			dd->tx_pkts_remaining--;

			if (dd->tx_pkts_remaining == 0) {
				/* Datagram send is complete */
				muc_gpio_set_wake_n(1);   /* Deassert WAKE */
				dd->tx_datagram_ndx = 0;
			}
		} else if (dd->tx_ack_pending) {
			/* STATE: Expected an ACK but not received
				- PKT send error */
			dev_err(&dd->client->dev, "Missing ACK\n");
			muc_gpio_set_wake_n(1);     /* Deassert WAKE */
			if (--num_ack_tries_remaining > 0) {
				goto retry_write;
			}
			return -ETIMEDOUT;
		}

		if (rx_msg->hdr.bitmask & HDR_BIT_VALID) {
			/* STATE: Received a Valid Message
					- Need To ACK it on next write */
			rx_valid = true;
			dd->rx_ack_pending = true;
			if ((dd->rx_first_pkt_rcvd) &&
			    (rx_msg->hdr.bitmask & HDR_BIT_PKT1))
				rx_valid = false;
		}

		if (rx_valid) {
			if (rx_msg->hdr.bitmask & HDR_BIT_PKT1) {
				dd->rx_first_pkt_rcvd = true;
				dd->rx_pkts_remaining =
					rx_msg->hdr.bitmask & HDR_BIT_PKTS;
			}

			/* copy to the rx_datagram */
			memcpy(&dd->rx_datagram[dd->rx_datagram_ndx],
				&rx_msg->data[0], PL_SIZE(dd->pkt_size));
		}
	}

	return ret;
}

static int __muc_i2c_message_send(struct muc_i2c_data *dd, __u16 msg_type,
		uint8_t *data, size_t len)
{
	int ret = 0;

	mutex_lock(&dd->mutex);
	pm_stay_awake(&dd->client->dev);

	/* setup structure values for tx datagrams */
	dd->tx_datagram_len = len;
	memcpy(dd->tx_datagram, data, len);
	dd->tx_pkts_remaining = (len / dd->pkt_size) +
			((len % dd->pkt_size) > 0);
	dd->tx_ack_pending = false;
	dd->tx_datagram_ndx = 0;

	while (dd->present && (dd->rx_ack_pending ||
	       (dd->tx_pkts_remaining) || (dd->rx_pkts_remaining))) {
		ret = muc_i2c_transfer(dd, msg_type);
		if (ret < 0) {
			dev_err(&dd->client->dev, "i2c failed me\n");
			break;
		}
	}

	pm_relax(&dd->client->dev);
	mutex_unlock(&dd->mutex);

	return ret;
}

/**
 * @brief Takes in a network message
 */
static int muc_i2c_message_send(struct mods_dl_device *dld,
				uint8_t *buf, size_t len)
{
	struct muc_i2c_data *dd = dld_to_dd(dld);

	return __muc_i2c_message_send(dd, MSG_TYPE_NW, buf, len);
}

static irqreturn_t muc_i2c_isr(int irq, void *data)
{
	struct muc_i2c_data *dd = data;

	mutex_lock(&dd->mutex);
	pm_stay_awake(&dd->client->dev);

	while (dd->present && (!muc_gpio_get_int_n() || (dd->rx_ack_pending))) {
		int ret = muc_i2c_transfer(dd, 0);

		if (ret < 0) {
			dev_err(&dd->client->dev, "i2c failed me\n");
			break;
		}
	}

	pm_relax(&dd->client->dev);
	mutex_unlock(&dd->mutex);

	return IRQ_HANDLED;
}

#define I2C_NEGOTIATE_RETRIES 3
static int _muc_i2c_negotiate(struct muc_i2c_data *dd)
{
	int err = 0;
	struct i2c_dl_msg msg;
	int retries = 0;

	memset(&msg, 0, sizeof(msg));
	msg.id = DL_MSG_ID_BUS_CFG_REQ;
	msg.bus_req.max_pl_size = U16_MAX;
	msg.bus_req.version = PROTO_VER;

	do {
		err = __muc_i2c_message_send(dd, MSG_TYPE_DL,
				(uint8_t *)&msg, sizeof(msg));
	} while (err && retries++ < I2C_NEGOTIATE_RETRIES);

	if (retries)
		dev_dbg(&dd->client->dev, "negotiate retried: %d\n", retries);

	return err;
}

static int muc_attach(struct notifier_block *nb,
		      unsigned long now_present, void *not_used)
{
	struct muc_i2c_data *dd = container_of(nb,
			struct muc_i2c_data, attach_nb);
	struct i2c_client *client = dd->client;
	int err;

	if (now_present != dd->present) {
		pr_info("%s: MuC attach state = %lu\n", __func__, now_present);

		dd->present = now_present;

		if (now_present) {
			err = devm_request_threaded_irq(&client->dev,
					client->irq, NULL, muc_i2c_isr,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"muc_i2c", dd);
			if (err) {
				dev_err(&client->dev,
						"Unable to request irq (%d).\n",
						client->irq);
				goto set_missing;
			}

			enable_irq_wake(client->irq);

			/* First step after attach is to negotiate bus config */
			err = _muc_i2c_negotiate(dd);
			if (err) {
				dev_err(&client->dev,
						"Error requesting bus cfg\n");
				goto free_irq;
			}
		} else {
			disable_irq_wake(client->irq);
			devm_free_irq(&client->dev, client->irq, dd);

			flush_work(&dd->attach_work);
			if (dd->attached) {
				mods_dl_dev_detached(dd->dld);
				dd->attached = false;
			}

			/* Reset bus settings to default values */
			set_packet_size(dd, DEFAULT_PKT_SZ);
			dd->rx_datagram_ndx = 0;
			dd->rx_datagram_len = 0;
			dd->rx_pkts_remaining = 0;
			dd->rx_ack_pending = false;
			dd->rx_first_pkt_rcvd = false;

			dd->tx_datagram_ndx = 0;
			dd->tx_pkts_remaining = 0;
			dd->tx_ack_pending = false;
		}
	}
	return NOTIFY_OK;

free_irq:
	disable_irq_wake(client->irq);
	devm_free_irq(&client->dev, client->irq, dd);
set_missing:
	dd->present = 0;

	return NOTIFY_OK;
}

static struct mods_dl_driver muc_i2c_dl_driver = {
	.message_send		= muc_i2c_message_send,
};

static int allocate_buffers(struct muc_i2c_data *dd)
{
	struct muc_buffers *b = muc_get_buffers();

	if (!b)
		return -ENOMEM;

	dd->rx_pkt = b->rx_pkt;
	dd->tx_pkt = b->tx_pkt;
	dd->rx_datagram = b->rx_datagram;
	dd->tx_datagram = b->tx_datagram;

	return 0;
}

static int muc_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct muc_i2c_data *dd;
	u8 intf_id;
	int ret;

	if (client->irq < 0) {
		dev_err(&client->dev, "%s: IRQ not defined\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_u8(client->dev.of_node, "mmi,intf-id", &intf_id)) {
		dev_err(&client->dev, "Couldn't get mmi,intf-id\n");
		return -EINVAL;
	}

	dd = devm_kzalloc(&client->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	dd->dld = mods_create_dl_device(&muc_i2c_dl_driver,
			&client->dev, intf_id);
	if (IS_ERR(dd->dld)) {
		dev_err(&client->dev,
				"Unable to create greybus host driver.\n");
		return PTR_ERR(dd->dld);
	}

	dd->dld->dl_priv = (void *)dd;
	dd->client = client;
	dd->attach_nb.notifier_call = muc_attach;
	INIT_WORK(&dd->attach_work, attach_worker);

	ret = allocate_buffers(dd);
	if (ret)
		goto remove_dl_device;

	ret = set_packet_size(dd, DEFAULT_PKT_SZ);
	if (ret)
		goto remove_dl_device;

	mutex_init(&dd->mutex);

	i2c_set_clientdata(client, dd);

	device_set_wakeup_capable(&client->dev, true);
	if (ret)
		dev_warn(&client->dev, "Failed to wakeup_enable: %d\n", ret);

	register_muc_attach_notifier(&dd->attach_nb);

	return 0;

remove_dl_device:
	mods_remove_dl_device(dd->dld);

	return ret;
}

static int muc_i2c_remove(struct i2c_client *client)
{
	struct muc_i2c_data *dd = i2c_get_clientdata(client);

	device_wakeup_disable(&client->dev);

	unregister_muc_attach_notifier(&dd->attach_nb);
	if (dd->present) {
		disable_irq_wake(client->irq);
		devm_free_irq(&client->dev, client->irq, dd);
	}

	flush_work(&dd->attach_work);
	if (dd->attached) {
		mods_dl_dev_detached(dd->dld);
		dd->attached = false;
	}

	mods_remove_dl_device(dd->dld);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_muc_i2c_match[] = {
	{ .compatible = "moto,muc_i2c", },
	{},
};
#endif

static const struct i2c_device_id muc_i2c_id[] = {
	{ "muc_i2c", 0 },
	{ }
};

static struct i2c_driver muc_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "muc_i2c",
		.of_match_table = of_match_ptr(of_muc_i2c_match),
	},
	.id_table = muc_i2c_id,
	.probe = muc_i2c_probe,
	.remove  = muc_i2c_remove,
};

int __init muc_i2c_init(void)
{
	int err;

	err = i2c_register_driver(THIS_MODULE, &muc_i2c_driver);
	if (err != 0)
		pr_err("muc_i2c initialization failed\n");

	return err;
}

void muc_i2c_exit(void)
{
	i2c_del_driver(&muc_i2c_driver);
}
