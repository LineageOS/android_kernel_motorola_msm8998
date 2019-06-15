/*
 * Copyright (C) 2015-2016 Motorola Mobility, Inc.
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

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "crc.h"
#include "mods_nw.h"
#include "muc.h"
#include "muc_svc.h"

/* Protocol version supported by this driver */
#define PROTO_VER           (2)

/* Protocol version change log */
#define PROTO_VER_PKT1      (1)     /* Version that added PKT1 bit */
#define PROTO_VER_ACK       (1)     /* Minimum version for ACK support */
#define PROTO_VER_DUMMY     (2)     /* Version that added DUMMY bit */

/*
 * Protocol version support macro for checking if the requested feature (f)
 * is supported by the attached MuC.
 */
#define MUC_SUPPORTS(d, f)  (d->proto_ver >= PROTO_VER_##f)

/*
 * Default size of a SPI packet (in bytes). This can be negotiated to be
 * larger, all the way up to the maximum size of a datagram.
 */
#define DEFAULT_PKT_SZ      PKT_SIZE(32)

/*
 * The maximum number of packets allowed for one datagram. For example, if
 * the packet size is the default size (32 bytes), the maximum supported
 * datagram size would be 2048 bytes (32 bytes x 64 packets). To transmit
 * larger datagrams, a larger packet size would need to be negotiated.
 *
 * This limit exists because there are only 6 bits in the SPI packet header
 * to indicate how many packets are remaining to complete the datagram.
 */
#define MAX_PKTS_PER_DG     (64)

/* SPI packet header bit definitions */
#define HDR_BIT_DUMMY  (0x01 << 9)  /* 1 = dummy packet */
#define HDR_BIT_PKT1   (0x01 << 8)  /* 1 = first packet of message */
#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = packet has valid payload */
#define HDR_BIT_TYPE   (0x01 << 6)  /* SPI message type */
#define HDR_BIT_PKTS   (0x3F << 0)  /* How many additional packets to expect */

/* Possible values for HDR_BIT_TYPE */
#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

/* Possible values for bus config features */
#define DL_BIT_ACK     (1 << 0)     /* Flag to indicate ACKing is supported */

/* SPI packet CRC size (in bytes) */
#define CRC_SIZE       (2)

/* Size of the header in bytes */
#define HDR_SIZE       sizeof(struct spi_msg_hdr)

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
			d->present);                                \
	}

typedef int (*handler_t)(struct mods_dl_device *from, uint8_t *msg, size_t len);

enum ack {
	ACK_NEEDED,
	ACK_NOT_NEEDED,
	ACK_ERROR,
};

/*
 * Possible data link layer messages. Responses IDs should be the request ID
 * with the MSB set.
 */
enum dl_msg_id {
	DL_MSG_ID_BUS_CFG_REQ         = 0x00,
	DL_MSG_ID_BUS_CFG_RESP        = 0x80,
};

struct muc_spi_data {
	struct spi_device *spi;
	struct mods_dl_device *dld;
	bool present;                      /* MuC is physically present */
	bool attached;                     /* MuC attach is reported to SVC */
	struct notifier_block attach_nb;   /* attach/detach notifications */
	struct mutex mutex;                /* Used to serialize SPI transfers */
	struct work_struct attach_work;    /* Worker to send attach to SVC */
	__u32 default_speed_hz;            /* Default SPI clock rate to use */
	__u8 proto_ver;                    /* Protocol version supported by MuC */
	bool ack_supported;                /* MuC supports ACK'ing on success */

	size_t pkt_size;                   /* Size of hdr + pl + CRC in bytes */
	__u8 *tx_pkt;                      /* Buffer for transmit packets */
	__u8 *rx_pkt;                      /* Buffer for received packets */

	__u8 *rx_datagram;                 /* Buffer used to assemble datagram */
	uint32_t rx_datagram_ndx;          /* Index into datagram buffer for new data */
	uint8_t pkts_remaining;            /* Packets needed to complete msg */

	/* Statistics below */
	struct dentry *stats_dentry;       /* Debugfs entry */
	uint32_t no_ack_sent;              /* Number of times no ACK was sent */
	uint32_t no_ack_rcvd;              /* Number of times no ACK was received */
	uint32_t no_ack_abort;             /* Number of times transfer was aborted */

	/* Quirks below */
	bool wake_delay;                   /* Delay after wake assert is req'd */
};

struct spi_msg_hdr {
	__le16 bitmask;                    /* See HDR_BIT_* defines for values */
} __packed;

struct spi_dl_msg_bus_config_req {
	__le16 max_pl_size;                /* Max payload size base supports */
	__u8   features;                   /* See DL_BIT_* defines for values */
	__u8   version;                    /* SPI msg format version supported */
} __packed;

struct spi_dl_msg_bus_config_resp {
	__le32 max_speed;                  /* Max bus speed mod supports */
	__le16 pl_size;                    /* Payload size mod selected */
	__u8   features;                   /* See DL_BIT_* defines for values */
	__u8   version;                    /* SPI msg format version of mod */
} __packed;

struct spi_dl_msg {
	__u8 id;                           /* enum dl_msg_id */
	union {
		struct spi_dl_msg_bus_config_req     bus_req;
		struct spi_dl_msg_bus_config_resp    bus_resp;
	};
} __packed;

size_t muc_spi_get_pkt_sz(size_t pl_size)
{
	return PKT_SIZE(pl_size);
}

static enum ack parse_rx_pkt(struct muc_spi_data *dd);
static int __muc_spi_message_send(struct muc_spi_data *dd, __u8 msg_type,
				  uint8_t *buf, size_t len);

static inline struct muc_spi_data *dld_to_dd(struct mods_dl_device *dld)
{
	return (struct muc_spi_data *)dld->dl_priv;
}

static inline bool is_tx_pkt_valid(struct muc_spi_data *dd)
{
	struct spi_msg_hdr *hdr = (struct spi_msg_hdr *)dd->tx_pkt;

	return !!(le16_to_cpu(hdr->bitmask) & HDR_BIT_VALID);
}

static inline void set_tx_pkt_hdr(struct muc_spi_data *dd, uint16_t bitmask)
{
	struct spi_msg_hdr *hdr = (struct spi_msg_hdr *)dd->tx_pkt;
	hdr->bitmask = cpu_to_le16(bitmask);
}

static inline void set_tx_pkt_crc(struct muc_spi_data *dd)
{
	uint16_t *crc = (uint16_t *)&dd->tx_pkt[CRC_NDX(dd->pkt_size)];

	*crc = crc16_calc(0, dd->tx_pkt, CRC_NDX(dd->pkt_size));
	*crc = cpu_to_le16(*crc);
}

static void set_bus_speed(struct muc_spi_data *dd, __u32 max_speed_hz)
{
	struct spi_device *spi = dd->spi;
	__u32 master_max_speed_hz = spi->master->max_speed_hz;

	/*
	 * Verify the requested speed is not higher than the maximum speed
	 * supported by the master controller. Setting the SPI device to a
	 * higher speed than the master controller will result in an error.
	 *
	 * If the master controller does not set its max_speed_hz field, then
	 * we have to hope for the best.
	 */
	if ((master_max_speed_hz > 0) && (max_speed_hz > master_max_speed_hz)) {
		dev_warn(&spi->dev, "Requested bus speed (%d) is higher than what "
			"master supports (%d)\n", max_speed_hz, master_max_speed_hz);
		max_speed_hz = master_max_speed_hz;
	}

	spi->max_speed_hz = max_speed_hz;
	dev_info(&spi->dev, "Max bus speed set to %u HZ\n", max_speed_hz);
}

static int set_packet_size(struct muc_spi_data *dd, size_t pkt_size)
{
	struct device *dev = &dd->spi->dev;
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

static int dl_recv(struct mods_dl_device *dld, uint8_t *msg, size_t len)
{
	struct muc_spi_data *dd = dld_to_dd(dld);
	struct device *dev = &dd->spi->dev;
	struct spi_dl_msg resp;
	uint32_t max_speed;
	size_t pl_size;
	int ret;

	/*
	 * To support MuCs running firmware with fewer values in the
	 * response, copy the message into a zero'd local struct.
	 */
	memset(&resp, 0, sizeof(resp));
	memcpy(&resp, msg, min(len, sizeof(resp)));

	/* Only BUS_CFG_RESP is supported */
	if (resp.id != DL_MSG_ID_BUS_CFG_RESP) {
		dev_err(dev, "Unknown ID (%d)!\n", resp.id);
		return -EINVAL;
	}

	if (dd->attached) {
		dev_warn(&dd->spi->dev, "Trying to reconfigure bus!\n");
		return -EINVAL;
	}

	max_speed = le32_to_cpu(resp.bus_resp.max_speed);
	pl_size = le16_to_cpu(resp.bus_resp.pl_size);

	/* Ignore max_bus_speed if zero and continue to use default speed */
	if (max_speed > 0)
		set_bus_speed(dd, max_speed);

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
			dev_err(dev, "Error (%d) setting new packet size\n", ret);
			return ret;
		}
	}

	if (!(resp.bus_resp.features & DL_BIT_ACK))
		dd->ack_supported = false;

	dd->proto_ver = resp.bus_resp.version;

	/* MuCs that support ACKing must use PROTO_VER_ACK or later */
	if (dd->ack_supported && !MUC_SUPPORTS(dd, ACK)) {
		dev_warn(dev, "ACK requires newer protocol version\n");
		dd->proto_ver = PROTO_VER_ACK;
	}

	dev_info(dev, "proto_ver=%d, ack_supported=%d\n",
		 dd->proto_ver, dd->ack_supported);

	/* Schedule work to send attach to SVC */
	schedule_work(&dd->attach_work);

	return 0;
}

static void attach_worker(struct work_struct *work)
{
	struct muc_spi_data *dd = container_of(work, struct muc_spi_data,
					       attach_work);
	int ret;

	/* Verify mod is still present before reporting attach */
	if (!dd->present)
		return;

	ret = mods_dl_dev_attached(dd->dld);
	if (ret)
		dev_err(&dd->spi->dev, "Error (%d) attaching to SVC\n", ret);
	else
		dd->attached = true;
}

static int muc_spi_transfer(struct muc_spi_data *dd, bool keep_wake)
{
	struct spi_device *spi = dd->spi;
	struct spi_transfer t[] = {
		{
			.tx_buf = dd->tx_pkt,
			.rx_buf = dd->rx_pkt,
			.len = dd->pkt_size,
		},
	};
	int ret;
	enum ack ack_req;
	int ack;
	int intn;
	int num_tries_remaining = NUM_TRIES;

retry:

	/* Set pinmux back to SPI configuration */
	if (dd->ack_supported) {
		muc_gpio_set_ack(0);
		(void)muc_gpio_ack_cfg(false);
	}

	/* Check if WAKE is not asserted */
	if (muc_gpio_get_wake_n()) {
		/* Assert WAKE */
		muc_gpio_set_wake_n(0);

		/* Wait for ADC enable (if neccessary) */
		if (dd->wake_delay)
			udelay(300);
	}

	/* Wait for RDY to be asserted */
	WAIT_WHILE((ret = muc_gpio_get_ready_n()), RDY_TIMEOUT_JIFFIES, dd);

	/* Deassert WAKE if no longer requested OR on timeout OR removal */
	if (!keep_wake || dd->ack_supported || ret != 0 || !dd->present)
		muc_gpio_set_wake_n(1);

	/*
	 * Check that RDY successfully was asserted after wake deassert to
	 * ensure wake line is deasserted.
	 */
	if (unlikely(!dd->present))
		return -ENODEV;
	if (unlikely(ret != 0)) {
		dev_err(&spi->dev, "Timeout waiting for rdy to assert\n");
		muc_gpio_toggle_spi_mux();
		if (--num_tries_remaining > 0)
			goto retry;
		return -ETIMEDOUT;
	}

	ret = spi_sync_transfer(spi, t, 1);

	if (ret) {
		if (--num_tries_remaining > 0) {
			dev_err(&spi->dev, "Retry: transfer failed\n");
			goto retry;
		} else {
			dev_err(&spi->dev, "Abort: transfer failed\n");
			return ret;
		}
	}

	ack_req = parse_rx_pkt(dd);

	if (!dd->ack_supported)
		return 0;

	/* Set pinmux for ACK'ing */
	(void)muc_gpio_ack_cfg(true);

	if (ack_req == ACK_NEEDED)
		muc_gpio_set_ack(1);
	else if (ack_req == ACK_ERROR)
		dd->no_ack_sent++;

	if (is_tx_pkt_valid(dd)) {
		WAIT_WHILE(!(ack = muc_gpio_get_ack()) &&
			   (intn = muc_gpio_get_int_n()),
			   ACK_TIMEOUT_JIFFIES, dd);
		if (!ack && intn) {
			if (ack_req == ACK_ERROR) {
				/*
				 * Since both TX and RX failed, need to spin
				 * longer to ensure MuC does not see false ACK.
				 */
				WAIT_WHILE((intn = muc_gpio_get_int_n()),
					   ACK_TIMEOUT_JIFFIES, dd);
			}

			dd->no_ack_rcvd++;
			if (--num_tries_remaining > 0) {
				dev_err(&spi->dev, "Retry: No ACK received\n");
				goto retry;
			} else {
				dev_err(&spi->dev, "Abort: No ACK received\n");
				dd->no_ack_abort++;
				return -ETIMEDOUT;
			}
		}
	}

	if (ack_req == ACK_ERROR) {
		/* Must block long enough to ensure MuC does not see false
		 * ACK.
		 */
		WAIT_WHILE((intn = muc_gpio_get_int_n()),
			   (2 * ACK_TIMEOUT_JIFFIES), dd);

		/*
		 * If we get here, there either was no TX or the TX was
		 * successful and ACK'd. Either way, return from this
		 * function to unblock any potential new messages.
		 */
	}

	return ret;
}

static enum ack parse_rx_pkt(struct muc_spi_data *dd)
{
	struct spi_msg_hdr *hdr = (struct spi_msg_hdr *)dd->rx_pkt;
	uint16_t bitmask = le16_to_cpu(hdr->bitmask);
	struct spi_device *spi = dd->spi;
	uint16_t *rcvcrc_p;
	uint16_t calcrc;
	size_t pl_size = PL_SIZE(dd->pkt_size);
	handler_t handler = mods_nw_switch;

	rcvcrc_p = (uint16_t *)&dd->rx_pkt[CRC_NDX(dd->pkt_size)];
	calcrc = crc16_calc(0, dd->rx_pkt, CRC_NDX(dd->pkt_size));
	if (le16_to_cpu(*rcvcrc_p) != calcrc) {
		dev_err(&spi->dev, "CRC mismatch, received: 0x%x, "
			"calculated: 0x%x\n", le16_to_cpu(*rcvcrc_p), calcrc);

		/*
		 * If ACK'ing is supported, keep received data to allow for
		 * a successful retry.
		 */
		if (!dd->ack_supported) {
			dd->rx_datagram_ndx = 0;
			dd->pkts_remaining = 0;
		}
		return ACK_ERROR;
	}

	if (MUC_SUPPORTS(dd, DUMMY)) {
		switch (bitmask & (HDR_BIT_VALID | HDR_BIT_DUMMY)) {
			case HDR_BIT_VALID:
				/* Process message below */
				break;

			case HDR_BIT_DUMMY:
				/* Received a dummy packet - nothing to do! */
				return ACK_NOT_NEEDED;

			default:
				dev_err(&spi->dev, "garbage packet\n");
				return ACK_ERROR;
		}
	} else if (!(bitmask & HDR_BIT_VALID)) {
		/* Received a dummy packet - nothing to do! */
		return ACK_NOT_NEEDED;
	}

	if (unlikely((bitmask & HDR_BIT_TYPE) == MSG_TYPE_DL))
		handler = dl_recv;

	/* Check if un-packetizing is not required */
	if (MAX_DATAGRAM_SZ == pl_size) {
		if (!MUC_SUPPORTS(dd, PKT1) || (bitmask & HDR_BIT_PKT1))
			handler(dd->dld, &dd->rx_pkt[HDR_SIZE],
				pl_size);
		else
			dev_err(&spi->dev, "1st pkt bit not set\n");

		return ACK_NEEDED;
	}

	if (!MUC_SUPPORTS(dd, PKT1))
		goto skip_pkt1;

	if (bitmask & HDR_BIT_PKT1) {
		/* Check if data exists from earlier packets */
		if (dd->rx_datagram_ndx) {
			dev_warn(&spi->dev,
				"1st pkt recv'd before prev msg complete: "
				"bitmask=0x%04x\n", bitmask);
			dd->rx_datagram_ndx = 0;
		}

		dd->pkts_remaining = bitmask & HDR_BIT_PKTS;
	} else {
		/* Check for data from earlier packets */
		if (!dd->rx_datagram_ndx) {
			dev_warn(&spi->dev, "Ignore non-first packet: "
				"bitmask=0x%04x\n", bitmask);
			return ACK_NEEDED;
		}

		if ((bitmask & HDR_BIT_PKTS) != --dd->pkts_remaining) {
			dev_err(&spi->dev,
				"Packets remaining out of sync: "
				"bitmask=0x%04x\n", bitmask);

			/* Drop the entire message */
			dd->rx_datagram_ndx = 0;
			dd->pkts_remaining = 0;
			return ACK_NEEDED;
		}
	}

skip_pkt1:
	if (unlikely(dd->rx_datagram_ndx >= MAX_DATAGRAM_SZ)) {
		dev_err(&spi->dev, "Too many packets received!\n");
		dd->rx_datagram_ndx = 0;
		return ACK_NEEDED;
	}

	memcpy(&dd->rx_datagram[dd->rx_datagram_ndx],
	       &dd->rx_pkt[HDR_SIZE], pl_size);
	dd->rx_datagram_ndx += pl_size;

	if (bitmask & HDR_BIT_PKTS) {
		/* Need additional packets before calling handler */
		return ACK_NEEDED;
	}

	handler(dd->dld, dd->rx_datagram, dd->rx_datagram_ndx);
	dd->rx_datagram_ndx = 0;

	return ACK_NEEDED;
}

static irqreturn_t muc_spi_isr(int irq, void *data)
{
	struct muc_spi_data *dd = data;
	int ret = 0;

	/* Any interrupt while the MuC is not present would be spurious */
	if (!dd->present)
		return IRQ_HANDLED;

	mutex_lock(&dd->mutex);
	pm_stay_awake(&dd->spi->dev);

	/* Populate the SPI dummy message */
	set_tx_pkt_hdr(dd, HDR_BIT_DUMMY);
	set_tx_pkt_crc(dd);

	while (!muc_gpio_get_int_n() && dd->present) {
		ret = muc_spi_transfer(dd, (dd->pkts_remaining > 1));
		if (ret) {
			dev_err(&dd->spi->dev, "isr spi transfer failed\n");
			break;
		}
	}

	pm_relax(&dd->spi->dev);
	mutex_unlock(&dd->mutex);

	return IRQ_HANDLED;
}

#define SPI_NEGOTIATE_RETRIES 3
static int _muc_spi_negotiate(struct muc_spi_data *dd)
{
	struct spi_dl_msg msg;
	int err;
	int retries = 0;

	memset(&msg, 0, sizeof(msg));
	msg.id = DL_MSG_ID_BUS_CFG_REQ;
	msg.bus_req.max_pl_size = U16_MAX;
	msg.bus_req.version = PROTO_VER;

	if (dd->ack_supported)
		msg.bus_req.features |= DL_BIT_ACK;

	do {
		err = __muc_spi_message_send(dd, MSG_TYPE_DL, (uint8_t *)&msg,
						sizeof(msg));
	} while (err && retries++ < SPI_NEGOTIATE_RETRIES);

	if (retries)
		dev_dbg(&dd->spi->dev, "negotiate retried: %d\n", retries);

	return err;
}

static int muc_attach(struct notifier_block *nb,
		      unsigned long now_present, void *not_used)
{
	struct muc_spi_data *dd = container_of(nb, struct muc_spi_data, attach_nb);
	struct spi_device *spi = dd->spi;
	int err;

	if (now_present != dd->present) {
		dev_info(&spi->dev, "%s: state = %lu\n", __func__, now_present);

		dd->present = now_present;

		if (now_present) {
			err = devm_request_threaded_irq(&spi->dev, spi->irq,
							NULL, muc_spi_isr,
							IRQF_TRIGGER_LOW |
							IRQF_ONESHOT,
							"muc_spi", dd);
			if (err) {
				dev_err(&spi->dev, "Unable to request irq.\n");
				goto set_missing;
			}

			enable_irq_wake(spi->irq);

			/* First step after attach is to negotiate bus config */
			err = _muc_spi_negotiate(dd);
			if (err) {
				dev_err(&spi->dev, "Error requesting bus cfg\n");
				goto free_irq;
			}
		} else {
			disable_irq_wake(spi->irq);
			devm_free_irq(&spi->dev, spi->irq, dd);

			flush_work(&dd->attach_work);
			if (dd->attached) {
				mods_dl_dev_detached(dd->dld);
				dd->attached = false;
			}

			if (dd->ack_supported)
				dev_info(&spi->dev, "No ACK sent: %u | "
					"No ACK rcvd: %u | No ACK abort: %u\n",
					dd->no_ack_sent, dd->no_ack_rcvd,
					dd->no_ack_abort);

			/* Reset bus settings to default values */
			set_bus_speed(dd, dd->default_speed_hz);
			set_packet_size(dd, DEFAULT_PKT_SZ);
			dd->rx_datagram_ndx = 0;
			dd->pkts_remaining = 0;
			dd->proto_ver = 0;
			dd->ack_supported = muc_gpio_ack_is_supported();
			dd->no_ack_sent = 0;
			dd->no_ack_rcvd = 0;
			dd->no_ack_abort = 0;
		}
	}
	return NOTIFY_OK;

free_irq:
	disable_irq_wake(spi->irq);
	devm_free_irq(&spi->dev, spi->irq, dd);
set_missing:
	dd->present = 0;

	return NOTIFY_OK;
}

static int __muc_spi_message_send(struct muc_spi_data *dd, __u8 msg_type,
				  uint8_t *buf, size_t len)
{
	int remaining = len;
	size_t pl_size = PL_SIZE(dd->pkt_size);
	int packets;
	int ret = 0;

	if (!dd->present)
		return -ENODEV;

	/* Calculate how many packets are required to send whole datagram */
	packets = (remaining + pl_size - 1) / pl_size;

	if ((len > MAX_DATAGRAM_SZ) || (packets > MAX_PKTS_PER_DG))
		return -E2BIG;

	mutex_lock(&dd->mutex);
	pm_stay_awake(&dd->spi->dev);

	while ((remaining > 0) && (packets > 0)) {
		int this_pl;
		uint16_t bitmask;

		/* Determine the payload size of this packet */
		this_pl = MIN(remaining, pl_size);

		/* Setup bitmask for packet header */
		bitmask  = HDR_BIT_VALID;
		bitmask |= (msg_type & HDR_BIT_TYPE);
		bitmask |= (--packets & HDR_BIT_PKTS);
		if (remaining == len)
			bitmask |= HDR_BIT_PKT1;

		/* Populate the SPI message */
		set_tx_pkt_hdr(dd, bitmask);
		memcpy((dd->tx_pkt + HDR_SIZE), buf, this_pl);
		set_tx_pkt_crc(dd);

		ret = muc_spi_transfer(dd, (packets > 0));
		if (ret)
			break;

		remaining -= this_pl;
		buf += this_pl;
	}

	pm_relax(&dd->spi->dev);
	mutex_unlock(&dd->mutex);

	return ret;
}

/* send message from switch to muc */
static int muc_spi_message_send(struct mods_dl_device *dld,
				uint8_t *buf, size_t len)
{
	struct muc_spi_data *dd = dld_to_dd(dld);

	return __muc_spi_message_send(dd, MSG_TYPE_NW, buf, len);
}

static struct mods_dl_driver muc_spi_dl_driver = {
	.message_send		= muc_spi_message_send,
};

#define STATS_BUF_SZ 100
static ssize_t muc_spi_stats_read(struct file *f, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct muc_spi_data *dd = f->f_inode->i_private;
	char tmp[STATS_BUF_SZ];
	int size;

	size = snprintf(tmp, STATS_BUF_SZ, "No ACK sent:  %u\nNo ACK rcvd:  %u"
		"\nNo ACK abort: %u\n", dd->no_ack_sent, dd->no_ack_rcvd,
		dd->no_ack_abort);
	return simple_read_from_buffer(buf, count, ppos, tmp, size);
}

static const struct file_operations muc_spi_stats_fops = {
	.read	= muc_spi_stats_read,
};

static int allocate_buffers(struct muc_spi_data *dd)
{
	struct muc_buffers *b = muc_get_buffers();

	if (!b)
		return -ENOMEM;

	dd->tx_pkt = b->tx_pkt;
	dd->rx_pkt = b->rx_pkt;
	dd->rx_datagram = b->rx_datagram;

	return 0;
}
static void muc_spi_quirks_init(struct muc_spi_data *dd)
{
	struct device_node *np = dd->spi->dev.of_node;

	dd->wake_delay = of_property_read_bool(np, "mmi,delay-after-wake");
	if (dd->wake_delay)
		dev_info(&dd->spi->dev, "Delay after wake is set\n");
}

static int muc_spi_probe(struct spi_device *spi)
{
	struct muc_spi_data *dd;
	u8 intf_id;
	int ret;

	dev_dbg(&spi->dev, "default_speed_hz=%d\n", spi->max_speed_hz);

	if (spi->irq < 0) {
		dev_err(&spi->dev, "%s: IRQ not defined\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_u8(spi->dev.of_node, "mmi,intf-id", &intf_id)) {
		dev_err(&spi->dev, "Couldn't get mmi,intf-id\n");
		return -EINVAL;
	}

	dd = devm_kzalloc(&spi->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	dd->dld = mods_create_dl_device(&muc_spi_dl_driver, &spi->dev, intf_id);
	if (IS_ERR(dd->dld)) {
		dev_err(&spi->dev, "%s: Unable to create greybus host driver.\n",
		        __func__);
		return PTR_ERR(dd->dld);
	}

	dd->dld->dl_priv = (void *)dd;
	dd->spi = spi;
	dd->default_speed_hz = spi->max_speed_hz;
	dd->attach_nb.notifier_call = muc_attach;
	dd->ack_supported = muc_gpio_ack_is_supported();
	INIT_WORK(&dd->attach_work, attach_worker);

	ret = allocate_buffers(dd);
	if (ret)
		goto remove_dl_device;

	ret = set_packet_size(dd, DEFAULT_PKT_SZ);
	if (ret)
		goto remove_dl_device;

	muc_spi_quirks_init(dd);
	mutex_init(&dd->mutex);

	spi_set_drvdata(spi, dd);

	device_set_wakeup_capable(&spi->dev, true);
	ret = device_wakeup_enable(&spi->dev);
	if (ret)
		dev_warn(&spi->dev, "Failed to wakeup_enable: %d\n", ret);

	dd->stats_dentry = debugfs_create_file("muc_spi_stats", S_IRUGO,
				mods_debugfs_get(), dd, &muc_spi_stats_fops);

	register_muc_attach_notifier(&dd->attach_nb);

	return 0;

remove_dl_device:
	mods_remove_dl_device(dd->dld);

	return ret;
}

static int muc_spi_remove(struct spi_device *spi)
{
	struct muc_spi_data *dd = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s: enter\n", __func__);

	device_wakeup_disable(&spi->dev);

	unregister_muc_attach_notifier(&dd->attach_nb);
	if (dd->present) {
		disable_irq_wake(spi->irq);
		devm_free_irq(&spi->dev, spi->irq, dd);
	}

	flush_work(&dd->attach_work);
	if (dd->attached) {
		mods_dl_dev_detached(dd->dld);
		dd->attached = false;
	}

	/*
	 * The SPI bus speed must be set back to default so the correct value
	 * is passed to the probe on the next module insertion.
	 */
	set_bus_speed(dd, dd->default_speed_hz);

	mods_remove_dl_device(dd->dld);
	debugfs_remove(dd->stats_dentry);
	spi_set_drvdata(spi, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_muc_spi_match[] = {
	{ .compatible = "moto,muc_spi", },
	{},
};
#endif

static const struct spi_device_id muc_spi_id[] = {
	{ "muc_spi", 0 },
	{ }
};

static struct spi_driver muc_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "muc_spi",
		.of_match_table = of_match_ptr(of_muc_spi_match),
	},
	.id_table = muc_spi_id,
	.probe = muc_spi_probe,
	.remove  = muc_spi_remove,
};

int __init muc_spi_init(void)
{
	int err;

	err = spi_register_driver(&muc_spi_driver);
	if (err != 0)
		pr_err("muc_spi initialization failed\n");

	return err;
}

void muc_spi_exit(void)
{
	spi_unregister_driver(&muc_spi_driver);
}
