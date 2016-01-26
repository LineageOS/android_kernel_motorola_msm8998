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
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>

#include "apba.h"
#include "crc.h"
#include "mods_nw.h"
#include "mods_uart.h"
#include "mods_uart_pm.h"
#include "muc_svc.h"

#define DRIVERNAME	"mods_uart"
#define N_MODS_UART	25

#pragma pack(push, 1)
struct uart_msg_hdr {
	__le16 nw_msg_size;
	__le16 msg_type;
};
#pragma pack(pop)

#define MODS_UART_MAX_SIZE (APBA_MSG_SIZE_MAX + sizeof(struct uart_msg_hdr))

struct mods_uart_err_stats {
	uint32_t tx_failure;
	uint32_t rx_crc;
	uint32_t rx_timeout;
	uint32_t rx_abort;
};

struct mods_uart_data {
	struct platform_device *pdev;
	struct tty_struct *tty;
	struct mods_dl_device *dld;
	bool present;
	char rx_data[MODS_UART_MAX_SIZE];
	size_t rx_len;
	unsigned long last_rx;
	struct mods_uart_err_stats stats;
	struct mutex tx_mutex;
	void *mods_uart_pm_data;
};

enum {
	MODS_UART_DL_GB,
	MODS_UART_DL_APBA
};

#define MODS_UART_SEGMENT_TIMEOUT 500 /* msec */

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Found in tty_io.c */
extern struct mutex tty_mutex;

static ssize_t ldisc_rel_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mods_uart_data *mud = dev_get_drvdata(dev);

	dev_info(dev, "%s: Releasing mods_uart ldisc\n", __func__);

	/*
	 * Set the line discipline to the default ldisc. This will allow this
	 * driver module to be removed from the system.
	 */
	if (tty_set_ldisc(mud->tty, N_TTY))
		dev_err(dev, "%s: Failed to set ldisc\n", __func__);

	return count;
}
static DEVICE_ATTR_WO(ldisc_rel);

static ssize_t uart_stats_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mods_uart_data *mud = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE,
			 "tx err:%d, rx crc:%d, rx timeout:%d, rx abort:%d\n",
			 mud->stats.tx_failure, mud->stats.rx_crc,
			 mud->stats.rx_timeout, mud->stats.rx_abort);
}

static DEVICE_ATTR_RO(uart_stats);

static struct attribute *uart_attrs[] = {
	&dev_attr_ldisc_rel.attr,
	&dev_attr_uart_stats.attr,
	NULL,
};

ATTRIBUTE_GROUPS(uart);

static int mods_uart_send_internal(struct mods_uart_data *mud,
				   __le16 type, uint8_t *buf, size_t len,
				   int flag)
{
	struct device *dev = &mud->pdev->dev;
	int ret;
	u8 *pkt;
	size_t pkt_size;
	struct uart_msg_hdr *hdr;
	__le16 crc16;

	if (len > APBA_MSG_SIZE_MAX) {
		mud->stats.tx_failure++;
		return -E2BIG;
	}

	pkt_size = sizeof(struct uart_msg_hdr) + len;
	pkt = kmalloc(pkt_size, GFP_KERNEL);
	if (!pkt) {
		mud->stats.tx_failure++;
		return -ENOMEM;
	}

	/* Populate the packet */
	hdr = (struct uart_msg_hdr *) pkt;
	hdr->nw_msg_size = cpu_to_le16(len);
	hdr->msg_type = cpu_to_le16(type);
	memcpy(pkt + sizeof(*hdr), buf, len);

	crc16 = crc16_calc(0, pkt, pkt_size);

	mutex_lock(&mud->tx_mutex);

	/*
	 * This call may block if APBA is in sleep.
	 * Try to fail through even if wake up was unsuccessful.
	 */
	mods_uart_pm_pre_tx(mud->mods_uart_pm_data, flag);

	/* First send the message */
	ret = mud->tty->ops->write(mud->tty, pkt, pkt_size);
	if (ret != pkt_size) {
		dev_err(dev, "%s: Failed to send message\n", __func__);
		goto send_err;
	}

	/* Then send the CRC */
	ret = mud->tty->ops->write(mud->tty, (uint8_t *)&crc16, sizeof(crc16));
	if (ret != sizeof(crc16)) {
		dev_err(dev, "%s: Failed to send CRC\n", __func__);
		goto send_err;
	}

	mods_uart_pm_post_tx(mud->mods_uart_pm_data, flag);
	mutex_unlock(&mud->tx_mutex);
	kfree(pkt);
	return 0;

send_err:
	mud->stats.tx_failure++;
	mutex_unlock(&mud->tx_mutex);
	kfree(pkt);
	return -EIO;
}

static int mods_uart_message_send(struct mods_dl_device *dld,
				  uint8_t *buf, size_t len)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)dld->dl_priv;

	return mods_uart_send_internal(mud, MODS_UART_DL_GB, buf, len, 0);
}

int mods_uart_apba_send(void *uart_data, uint8_t *buf, size_t len, int flag)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)uart_data;

	return mods_uart_send_internal(mud, MODS_UART_DL_APBA, buf, len, flag);
}

static struct mods_dl_driver mods_uart_dl_driver = {
	.message_send		= mods_uart_message_send,
};

static int config_tty(struct mods_uart_data *mud)
{
	struct device *dev = &mud->pdev->dev;
	struct device_node *np = dev->of_node;
	struct ktermios kt;
	speed_t speed;
	int ret;

	ret = of_property_read_u32(np, "mmi,tty_speed", &speed);
	if (ret) {
		dev_err(dev, "%s: TTY speed not populated\n", __func__);
		return -EINVAL;
	}
	dev_info(dev, "%s: speed=%d\n", __func__, speed);

	/* Use one stop bit (CSTOPB not set) and no parity (PARENB not set) */
	kt.c_cflag = 0;
	kt.c_cflag |= CLOCAL;  /* Ignore modem control lines */
	kt.c_cflag |= CREAD;   /* Enable receiver */
	kt.c_cflag |= CS8;     /* Character size */
	kt.c_cflag |= CRTSCTS; /* Enable RTS/CTS (hardware) flow control */
	kt.c_iflag = 0;
	kt.c_iflag |= IGNBRK;  /* Ignore BREAK condition on input */
	kt.c_iflag |= IGNPAR;  /* Ignore framing errors and parity errors. */
	kt.c_oflag = 0;
	kt.c_lflag = 0;

	tty_termios_encode_baud_rate(&kt, speed, speed);

	return tty_set_termios(mud->tty, &kt);
}

/**
 * Find the tty driver for a given tty name.
 *
 * @name: name string to match
 * @line: pointer to resulting tty line nr
 *
 * Originally copied from tty_find_polling_driver, with polling check removed.
 */
static struct tty_driver *find_tty_driver(char *name, int *line)
{
	struct tty_driver *p, *res = NULL;
	int tty_line = 0;
	int len;
	char *str, *stp;

	for (str = name; *str; str++)
		if ((*str >= '0' && *str <= '9') || *str == ',')
			break;
	if (!*str)
		return NULL;

	len = str - name;
	tty_line = simple_strtoul(str, &str, 10);

	mutex_lock(&tty_mutex);
	/* Search through the tty devices to look for a match */
	list_for_each_entry(p, &tty_drivers, tty_drivers) {
		if (strncmp(name, p->name, len) != 0)
			continue;
		stp = str;
		if (*stp == ',')
			stp++;
		if (*stp == '\0')
			stp = NULL;

		if (tty_line >= 0 && tty_line < p->num) {
			res = tty_driver_kref_get(p);
			*line = tty_line;
			break;
		}
	}
	mutex_unlock(&tty_mutex);

	return res;
}

void mod_attach(void *uart_data, unsigned long now_present)
{
	struct mods_uart_data *mud;
	int err;

	mud = (struct mods_uart_data *)uart_data;
	if (now_present == mud->present)
		return;

	mud->present = now_present;

	if (now_present) {
		err = mods_dl_dev_attached(mud->dld);
		if (err) {
			dev_err(&mud->pdev->dev, "Error attaching to SVC\n");
			mud->present = 0;
		}
	} else
		mods_dl_dev_detached(mud->dld);
}

void mods_uart_lock_tx(void *uart_data, bool lock)
{
	struct mods_uart_data *mud;

	mud = (struct mods_uart_data *)uart_data;

	if (lock)
		mutex_lock(&mud->tx_mutex);
	else
		mutex_unlock(&mud->tx_mutex);
}

int mods_uart_do_pm(void *uart_data, bool on)
{
	struct mods_uart_data *mud;

	mud = (struct mods_uart_data *)uart_data;
	return mud->tty->ops->ioctl(mud->tty, on ? TIOCPMGET : TIOCPMPUT, 0);
}

void *mods_uart_get_pm_data(void *uart_data)
{
	struct mods_uart_data *mud;

	mud = (struct mods_uart_data *)uart_data;

	return mud->mods_uart_pm_data;
}

static int mods_uart_probe(struct platform_device *pdev)
{
	struct mods_uart_data *mud;
	struct tty_driver *driver;
	int tty_line = 0;
	int ret;
	struct device_node *np = pdev->dev.of_node;
	const char *tty_name = NULL;
	u8 intf_id;

	mud = devm_kzalloc(&pdev->dev, sizeof(*mud), GFP_KERNEL);
	if (!mud)
		return -ENOMEM;

	ret = of_property_read_u8(np, "mmi,intf-id", &intf_id);
	if (ret) {
		dev_err(&pdev->dev, "%s: Couldn't read intf-id\n", __func__);
		return ret;
	}

	/* apba_ctrl must be probed and initialized */
	if (apba_uart_register(mud))
		return -EPROBE_DEFER;

	mud->dld = mods_create_dl_device(&mods_uart_dl_driver, &pdev->dev,
					intf_id);
	if (IS_ERR(mud->dld)) {
		dev_err(&pdev->dev, "%s: Unable to create data link device\n",
			__func__);
		ret = PTR_ERR(mud->dld);
		goto unreg_apba;
	}

	mud->dld->dl_priv = (void *)mud;
	mud->pdev = pdev;

	/* Retrieve the name of the tty from the device tree */
	ret = of_property_read_string(np, "mmi,tty", &tty_name);
	if (ret) {
		dev_err(&pdev->dev, "%s: TTY name not populated\n", __func__);
		goto free_dld;
	}
	dev_info(&pdev->dev, "%s: Using %s\n", __func__, tty_name);

	/* Find the driver for the specified tty */
	driver = find_tty_driver((char *)tty_name, &tty_line);
	if (!driver || !driver->ttys) {
		dev_err(&pdev->dev, "%s: Did not find tty driver\n", __func__);
		ret = -ENODEV;
		goto free_dld;
	}

	/* Use existing tty if present */
	mud->tty = driver->ttys[tty_line];
	if (!mud->tty)
		mud->tty = tty_init_dev(driver, tty_line);

	tty_driver_kref_put(driver);

	if (IS_ERR(mud->tty)) {
		ret = PTR_ERR(mud->tty);
		goto free_dld;
	}

	/* For now, have the tty always open */
	ret = mud->tty->ops->open(mud->tty, NULL);
	if (ret) {
		dev_err(&pdev->dev, "%s: Failed to open tty\n", __func__);
		goto release_tty;
	}

	ret = config_tty(mud);
	if (ret) {
		dev_err(&pdev->dev, "%s: Failed to config tty\n", __func__);
		goto close_tty_locked;
	}

	/*
	 * Before the line discipline can be set, the tty must be unlocked.
	 * If this is not done, the kernel will deadlock.
	 */
	tty_unlock(mud->tty);

	/*
	 * Set the line discipline to the local ldisc. This will allow this
	 * driver to know when new data is received without having to poll.
	 */
	ret = tty_set_ldisc(mud->tty, N_MODS_UART);
	if (ret) {
		dev_err(&pdev->dev, "%s: Failed to set ldisc\n", __func__);
		goto close_tty_unlocked;
	}

	ret = sysfs_create_groups(&pdev->dev.kobj, uart_groups);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
		goto set_ldisc_tty;
	}

	mud->mods_uart_pm_data = mods_uart_pm_initialize(mud);
	if (!mud->mods_uart_pm_data) {
		dev_err(&pdev->dev, "Failed to initialize uart pm\n");
		goto remove_sysfs;
	}

	mutex_init(&mud->tx_mutex);

	mud->tty->disc_data = mud;
	platform_set_drvdata(pdev, mud);

	return 0;

remove_sysfs:
	sysfs_remove_groups(&pdev->dev.kobj, uart_groups);

set_ldisc_tty:
	tty_set_ldisc(mud->tty, N_TTY);

close_tty_unlocked:
	/* TTY must be locked to close the connection */
	tty_lock(mud->tty);

close_tty_locked:
	mud->tty->ops->close(mud->tty, NULL);
	tty_unlock(mud->tty);

release_tty:
	/* Release the TTY, which requires the mutex to be held */
	mutex_lock(&tty_mutex);
	release_tty(mud->tty, mud->tty->index);
	mutex_unlock(&tty_mutex);

free_dld:
	mods_remove_dl_device(mud->dld);

unreg_apba:
	apba_uart_register(NULL);

	return ret;
}

static int mods_uart_remove(struct platform_device *pdev)
{
	struct mods_uart_data *mud = platform_get_drvdata(pdev);
	struct tty_struct *tty = mud->tty;

	if (mud->present)
		mods_dl_dev_detached(mud->dld);

	mods_uart_pm_uninitialize(mud->mods_uart_pm_data);

	sysfs_remove_groups(&pdev->dev.kobj, uart_groups);

	apba_uart_register(NULL);

	mods_remove_dl_device(mud->dld);

	/* TTY must be locked to close the connection */
	tty_lock(tty);
	tty->ops->close(tty, NULL);
	tty_unlock(tty);

	/* Release the TTY, which requires the mutex to be held */
	mutex_lock(&tty_mutex);
	release_tty(tty, tty->index);
	mutex_unlock(&tty_mutex);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id mods_uart_match_table[] = {
	{ .compatible = "mmi,mod-uart"},
	{}
};

static struct platform_driver mods_uart_driver = {
	.probe	= mods_uart_probe,
	.remove = mods_uart_remove,
	.driver = {
		.name = DRIVERNAME,
		.of_match_table = mods_uart_match_table,
	},
};

static int mods_uart_consume_segment(struct mods_uart_data *mud)
{
	struct device *dev = &mud->pdev->dev;
	uint16_t calc_crc;
	uint16_t *rcvd_crc;
	struct uart_msg_hdr *hdr;
	uint8_t *payload;
	size_t content_size;
	size_t segment_size;

	if (mud->rx_len < sizeof(*hdr))
		return 0;

	hdr = (struct uart_msg_hdr *) mud->rx_data;
	content_size = sizeof(*hdr) + le16_to_cpu(hdr->nw_msg_size);
	segment_size = content_size + sizeof(calc_crc);

	if (mud->rx_len < segment_size)
		return 0;

	rcvd_crc = (uint16_t *)&mud->rx_data[content_size];
	calc_crc = crc16_calc(0, (uint8_t *) mud->rx_data, content_size);
	if (le16_to_cpu(*rcvd_crc) != calc_crc) {
		mud->stats.rx_crc++;
		dev_err(dev, "%s: CRC mismatch, received: 0x%x, "
			"calculated: 0x%x\n", __func__,
			le16_to_cpu(*rcvd_crc), calc_crc);

		/*
		 * Not sure what is best to return in this case. Since the
		 * entire buf was technically received and parsed, return the
		 * total count.
		 */
	} else {
		payload = ((uint8_t *)mud->rx_data) + sizeof(*hdr);
		switch (le16_to_cpu(hdr->msg_type)) {
		case MODS_UART_DL_GB:
			mods_nw_switch(mud->dld, payload,
				       le16_to_cpu(hdr->nw_msg_size));
			break;
		case MODS_UART_DL_APBA:
			apba_handle_message(payload,
					    le16_to_cpu(hdr->nw_msg_size));
			break;
		default:
			dev_err(dev,
				"%s: Unknown DL message type (%d) received\n",
				__func__, le16_to_cpu(hdr->msg_type));
		}
	}
	mud->rx_len -= segment_size;

	if (mud->rx_len) {
		memmove(mud->rx_data, mud->rx_data + segment_size,
			mud->rx_len);
		/* more data to consume */
		return 1;
	}
	return 0;
}

static int n_mods_uart_receive_buf2(struct tty_struct *tty,
				    const unsigned char *cp,
				    char *fp, int count)
{
	struct mods_uart_data *mud = tty->disc_data;
	struct device *dev = &mud->pdev->dev;
	int to_be_consumed = count;

	/*
	 * Try to clean up garbage/incomplete chars received
	 * for some reason.
	 */
	if (mud->rx_len &&
	    (jiffies_to_msecs(jiffies - mud->last_rx) >
	     MODS_UART_SEGMENT_TIMEOUT)) {
		mud->stats.rx_timeout++;
		dev_err(dev, "%s: RX Buffer cleaned up\n", __func__);
		mud->rx_len = 0;
	}
	mud->last_rx = jiffies;

	while (to_be_consumed > 0) {
		int copy_size;

		if (mud->rx_len == MODS_UART_MAX_SIZE) {
			/*
			 * buffer is already left full, not consumed.
			 * Something is wrong. Need to reset the buffer
			 * to proceed
			 */
			/*
			 * TODO: Send a break signa to APBA to reset
			 *       UART status.
			 */
			mud->stats.rx_abort++;
			dev_err(dev, "%s: RX buffer overflow\n", __func__);
			mud->rx_len = 0;
		}

		copy_size = MIN(to_be_consumed,
				MODS_UART_MAX_SIZE - mud->rx_len);

		memcpy(&mud->rx_data[mud->rx_len], cp, copy_size);
		mud->rx_len += copy_size;

		do {} while (mods_uart_consume_segment(mud));

		to_be_consumed -= copy_size;
	}
	mods_uart_pm_update_idle_timer(mud->mods_uart_pm_data);

	return count;
}

static struct tty_ldisc_ops mods_uart_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= DRIVERNAME,
	.receive_buf2	= n_mods_uart_receive_buf2,
};

int __init mods_uart_init(void)
{
	int ret;

	ret = tty_register_ldisc(N_MODS_UART, &mods_uart_ldisc);
	if (ret < 0) {
		pr_err("%s: ldisc registration failed: %d\n", __func__, ret);
		return ret;
	}

	return platform_driver_register(&mods_uart_driver);
}

void __exit mods_uart_exit(void)
{
	int ret;

	platform_driver_unregister(&mods_uart_driver);

	ret = tty_unregister_ldisc(N_MODS_UART);
	if (ret < 0)
		pr_err("%s: ldisc unregistration failed: %d\n", __func__, ret);
}
