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

#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>

#include "apba.h"
#include "mods_nw.h"
#include "mods_uart.h"
#include "mods_uart_pm.h"
#include "muc_svc.h"
#include "mhb_protocol.h"

#define DRIVERNAME	"mods_uart"
#define N_MODS_UART	25

struct mods_uart_err_stats {
	uint32_t tx_failure;
	uint32_t rx_crc;
	uint32_t rx_timeout;
	uint32_t rx_abort;
	uint32_t rx_len;
};

struct mods_uart_data {
	struct platform_device *pdev;
	struct tty_struct *tty;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_default;
	struct pinctrl_state *pinctrl_state_active;
	char rx_data[MHB_MAX_MSG_SIZE];
	size_t rx_len;
	unsigned long last_rx;
	struct mods_uart_err_stats stats;
	struct mutex tx_mutex;
	void *mods_uart_pm_data;
	const char *tty_name;
	uint8_t intf_id;
	speed_t default_baud;
};

enum {
	MODS_UART_DL_GB,
	MODS_UART_DL_APBA
};

#define MODS_UART_SEGMENT_TIMEOUT 500 /* msec */

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Found in tty_io.c */
extern struct mutex tty_mutex;

static ssize_t uart_stats_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mods_uart_data *mud = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE,
			 "tx err:%d, rx crc:%d, rx timeout:%d, rx abort:%d, "
			 "rx len:%d\n",
			 mud->stats.tx_failure, mud->stats.rx_crc,
			 mud->stats.rx_timeout, mud->stats.rx_abort,
			 mud->stats.rx_len);
}

static DEVICE_ATTR_RO(uart_stats);

static struct attribute *uart_attrs[] = {
	&dev_attr_uart_stats.attr,
	NULL,
};

ATTRIBUTE_GROUPS(uart);

static int mods_uart_send_internal(struct mods_uart_data *mud,
				   struct mhb_hdr *hdr, uint8_t *buf,
				   size_t len, int flag)
{
	struct device *dev = &mud->pdev->dev;
	int ret;
	u8 *pkt;
	size_t pkt_size;
	__le16 calc_crc;

	if (len > MHB_MAX_MSG_SIZE) {
		mud->stats.tx_failure++;
		return -E2BIG;
	}

	pkt_size = sizeof(struct mhb_hdr) + len;
	pkt = kmalloc(pkt_size, GFP_KERNEL);
	if (!pkt) {
		mud->stats.tx_failure++;
		return -ENOMEM;
	}

	/* Populate the packet */
	hdr->length = cpu_to_le16(len + sizeof(*hdr) + sizeof(calc_crc));
	memcpy(pkt, hdr, sizeof(*hdr));
	memcpy(pkt + sizeof(*hdr), buf, len);

	calc_crc = crc16(0, pkt, pkt_size);

	mutex_lock(&mud->tx_mutex);

	if (!mud->tty) {
		mutex_unlock(&mud->tx_mutex);
		kfree(pkt);
		dev_err(dev, "%s: no tty\n", __func__);
		return -ENODEV;
	}

	/*
	 * This call may block if APBA is in sleep.
	 * Try to fail through even if wake up was unsuccessful.
	 */
	mods_uart_pm_pre_tx(mud->mods_uart_pm_data, flag);

	/* First send the message */
	print_hex_dump_debug("RAW TX: ", DUMP_PREFIX_OFFSET, 16, 1,
		pkt, pkt_size, true);
	ret = mud->tty->ops->write(mud->tty, pkt, pkt_size);
	if (ret != pkt_size) {
		dev_err(dev, "%s: Failed to send message\n", __func__);
		goto send_err;
	}

	/* Then send the CRC */
	print_hex_dump_debug("RAW TX (crc): ", DUMP_PREFIX_OFFSET, 16, 1,
		(uint8_t *)&calc_crc, sizeof(calc_crc), true);
	ret = mud->tty->ops->write(mud->tty, (uint8_t *)&calc_crc, sizeof(calc_crc));
	if (ret != sizeof(calc_crc)) {
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

int mods_uart_send(void *uart_data, struct mhb_hdr *hdr, uint8_t *buf,
	size_t len, int flag)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)uart_data;

	pr_debug("MHB TX: addr=%x, type=%x, result=%x\n",
	        hdr->addr, hdr->type, hdr->result);
	print_hex_dump_debug("MHB TX: ", DUMP_PREFIX_OFFSET, 16, 1,
		buf, len, true);
	return mods_uart_send_internal(mud, hdr, buf, len, flag);
}

int mods_uart_get_baud(void *uart_data)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)uart_data;
	speed_t speed;

	if (!mud || !mud->tty) {
		pr_err("%s: no tty\n", __func__);
		return -ENODEV;
	}

	down_read(&mud->tty->termios_rwsem);
	speed = mud->tty->termios.c_ispeed;
	up_read(&mud->tty->termios_rwsem);

	return (int)speed;
}

static int config_tty(struct mods_uart_data *mud, speed_t speed,
		      struct tty_struct *tty)
{
	struct ktermios kt;

	if (!speed)
		speed = mud->default_baud;
	dev_info(&mud->pdev->dev, "%s: speed=%d\n", __func__, speed);

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

	/* Save off the previous c_line so we don't overwrite it */
	kt.c_line = tty->termios.c_line;

	tty_termios_encode_baud_rate(&kt, speed, speed);

	return tty_set_termios(tty, &kt);
}

int mods_uart_set_baud(void *uart_data, uint32_t baud)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)uart_data;
	int ret = -ENODEV;

	if (mud && mud->tty)
		ret = config_tty(mud, (speed_t)baud, mud->tty);

	return ret;
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

int mods_uart_open(void *uart_data)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)uart_data;
	struct tty_driver *driver;
	int tty_line = 0;
	int ret;
	struct tty_struct *tty_tmp;

	mutex_lock(&mud->tx_mutex);
	if (mud->tty) {
		mutex_unlock(&mud->tx_mutex);
		dev_warn(&mud->pdev->dev, "%s: already open\n", __func__);
		return -EEXIST;
	}

	dev_dbg(&mud->pdev->dev, "%s: opening uart\n", __func__);

	ret = pinctrl_select_state(mud->pinctrl, mud->pinctrl_state_active);
	if (ret) {
		dev_err(&mud->pdev->dev, "%s: Pinctrl set failed %d\n", __func__, ret);
		goto open_fail;
	}

	/* Find the driver for the specified tty */
	driver = find_tty_driver((char *)mud->tty_name, &tty_line);
	if (!driver || !driver->ttys) {
		dev_err(&mud->pdev->dev, "%s: Did not find tty driver\n", __func__);
		ret = -ENODEV;
		goto open_fail;
	}

	/* Use existing tty if present */
	tty_tmp = driver->ttys[tty_line];
	if (!tty_tmp)
		tty_tmp = tty_init_dev(driver, tty_line);

	tty_driver_kref_put(driver);

	if (IS_ERR(tty_tmp)) {
		ret = PTR_ERR(tty_tmp);
		goto open_fail;
	}

	ret = tty_tmp->ops->open(tty_tmp, NULL);
	if (ret) {
		dev_err(&mud->pdev->dev, "%s: Failed to open tty\n", __func__);
		goto release_tty;
	}

	ret = config_tty(mud, 0 /* default speed */, tty_tmp);
	if (ret) {
		dev_err(&mud->pdev->dev, "%s: Failed to config tty\n", __func__);
		goto close_tty_locked;
	}

	/*
	 * Before the line discipline can be set, the tty must be unlocked.
	 * If this is not done, the kernel will deadlock.
	 */
	tty_unlock(tty_tmp);

	/*
	 * Set the line discipline to the local ldisc. This will allow this
	 * driver to know when new data is received without having to poll.
	 */
	ret = tty_set_ldisc(tty_tmp, N_MODS_UART);
	if (ret) {
		dev_err(&mud->pdev->dev, "%s: Failed to set ldisc\n", __func__);
		goto close_tty_unlocked;
	}
	tty_tmp->disc_data = mud;

	mud->mods_uart_pm_data = mods_uart_pm_initialize(mud);
	if (!mud->mods_uart_pm_data) {
		dev_err(&mud->pdev->dev, "Failed to initialize uart pm\n");
		goto set_ldisc_tty;
	}

	/* Reset sysfs stat entries */
	memset((void *)&mud->stats, 0, sizeof(struct mods_uart_err_stats));

	mud->tty = tty_tmp;

	mods_uart_pm_on(mud);

	mutex_unlock(&mud->tx_mutex);

	return 0;

set_ldisc_tty:
	tty_set_ldisc(tty_tmp, N_TTY);

close_tty_unlocked:
	/* TTY must be locked to close the connection */
	tty_lock(tty_tmp);

close_tty_locked:
	tty_tmp->ops->close(tty_tmp, NULL);
	tty_unlock(tty_tmp);

release_tty:
	/* Release the TTY, which requires the mutex to be held */
	mutex_lock(&tty_mutex);
	release_tty(tty_tmp, tty_tmp->index);
	mutex_unlock(&tty_mutex);

open_fail:
	mutex_unlock(&mud->tx_mutex);

	return ret;
}

int mods_uart_close(void *uart_data)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)uart_data;
	int ret;

	dev_dbg(&mud->pdev->dev, "%s: closing uart\n", __func__);
	mods_uart_pm_off(mud);

	if (mud->mods_uart_pm_data)
		mods_uart_pm_cancel_timer(mud->mods_uart_pm_data);

	mutex_lock(&mud->tx_mutex);
	if (!mud->tty) {
		mutex_unlock(&mud->tx_mutex);
		dev_warn(&mud->pdev->dev, "%s: already closed\n", __func__);
		return -ENODEV;
	}

	dev_dbg(&mud->pdev->dev, "%s: really closing\n", __func__);

	if (tty_set_ldisc(mud->tty, N_TTY))
		dev_err(&mud->pdev->dev, "%s: Failed to set ldisc\n", __func__);

	/* TTY must be locked to close the connection */
	tty_lock(mud->tty);
	mud->tty->ops->close(mud->tty, NULL);
	tty_unlock(mud->tty);

	/* Release the TTY, which requires the mutex to be held */
	mutex_lock(&tty_mutex);
	release_tty(mud->tty, mud->tty->index);
	mutex_unlock(&tty_mutex);
	mud->tty = NULL;

	if (mud->mods_uart_pm_data) {
		mods_uart_pm_uninitialize(mud->mods_uart_pm_data);
		mud->mods_uart_pm_data = NULL;
	}

	ret = pinctrl_select_state(mud->pinctrl, mud->pinctrl_state_default);
	if (ret)
		dev_err(&mud->pdev->dev, "%s: Pinctrl set failed %d\n", __func__, ret);

	mutex_unlock(&mud->tx_mutex);

	return ret;
}

static int mods_uart_probe(struct platform_device *pdev)
{
	struct mods_uart_data *mud;
	speed_t speed;
	int ret;

	struct device_node *np = pdev->dev.of_node;

	mud = devm_kzalloc(&pdev->dev, sizeof(*mud), GFP_KERNEL);
	if (!mud)
		return -ENOMEM;

	ret = of_property_read_u8(np, "mmi,intf-id", &mud->intf_id);
	if (ret) {
		dev_err(&pdev->dev, "%s: Couldn't read intf-id\n", __func__);
		return ret;
	}

	ret = of_property_read_u32(np, "mmi,tty_speed", &speed);
	if (ret) {
		dev_err(&pdev->dev, "%s: TTY speed not populated\n", __func__);
		return ret;
	}
	mud->default_baud = speed;

	mud->pdev = pdev;
	platform_set_drvdata(pdev, mud);

	/* Retrieve the name of the tty from the device tree */
	ret = of_property_read_string(np, "mmi,tty", &mud->tty_name);
	if (ret) {
		dev_err(&pdev->dev, "%s: TTY name not populated\n", __func__);
		return ret;
	}
	dev_info(&pdev->dev, "%s: Using %s\n", __func__, mud->tty_name);

	mud->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(mud->pinctrl)) {
		dev_err(&pdev->dev, "Pinctrl not defined\n");
		return PTR_ERR(mud->pinctrl);
	}

	mud->pinctrl_state_default = pinctrl_lookup_state(mud->pinctrl,
							  PINCTRL_STATE_DEFAULT);
	if (IS_ERR(mud->pinctrl_state_default)) {
		dev_err(&pdev->dev, "Pinctrl lookup failed for default\n");
		return PTR_ERR(mud->pinctrl_state_default);
	}

	mud->pinctrl_state_active = pinctrl_lookup_state(mud->pinctrl,
							 "active");
	if (IS_ERR(mud->pinctrl_state_active)) {
		dev_err(&pdev->dev, "Pinctrl lookup failed for active\n");
		return PTR_ERR(mud->pinctrl_state_active);
	}

	mutex_init(&mud->tx_mutex);

	ret = sysfs_create_groups(&pdev->dev.kobj, uart_groups);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
		return ret;
	}
	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	/* apba_ctrl must be probed and initialized */
	if (apba_uart_register(mud)) {
		ret = -EPROBE_DEFER;
		goto remove_sysfs;
	}

	return 0;

remove_sysfs:
	sysfs_remove_groups(&pdev->dev.kobj, uart_groups);

	return ret;
}

static int mods_uart_remove(struct platform_device *pdev)
{
	struct mods_uart_data *mud = platform_get_drvdata(pdev);

	apba_uart_register(NULL);
	sysfs_remove_groups(&pdev->dev.kobj, uart_groups);
	kobject_uevent(&pdev->dev.kobj, KOBJ_REMOVE);
	mods_uart_close(mud);
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
	struct mhb_hdr *hdr;
	uint8_t *payload;
	size_t content_size;
	size_t segment_size;

	if (mud->rx_len < sizeof(*hdr))
		return 0;

	hdr = (struct mhb_hdr *) mud->rx_data;
	segment_size = le16_to_cpu(hdr->length);
	if ((segment_size < sizeof(struct mhb_hdr) + sizeof(calc_crc)) ||
	    (segment_size > MHB_MAX_MSG_SIZE)) {
		mud->stats.rx_len++;
		dev_err(dev, "%s: invalid len %zd\n", __func__, segment_size);

		mud->rx_len = 0;
		return 0;
	}

	content_size = segment_size - sizeof(calc_crc);

	if (mud->rx_len < segment_size)
		return 0;

	rcvd_crc = (uint16_t *)&mud->rx_data[content_size];
	calc_crc = crc16(0, (uint8_t *) mud->rx_data, content_size);
	if (le16_to_cpu(*rcvd_crc) != calc_crc) {
		mud->stats.rx_crc++;
		print_hex_dump_debug("RX (CRC error): ", DUMP_PREFIX_OFFSET,
			16, 1, mud->rx_data, content_size, true);
		dev_err(dev, "%s: CRC mismatch, received: 0x%x, "
			"calculated: 0x%x\n", __func__,
			le16_to_cpu(*rcvd_crc), calc_crc);

		/*
		 * Not sure what is best to return in this case. Since the
		 * entire buf was technically received and parsed, return the
		 * total count.
		 */
		mud->rx_len = 0;
		return 0;
	} else {
		payload = ((uint8_t *)mud->rx_data) + sizeof(*hdr);
		pr_debug("MHB RX: addr=%x, type=%x, result=%x, len=%zd\n",
			hdr->addr, hdr->type, hdr->result, content_size);

		print_hex_dump_debug("MHB RX: ", DUMP_PREFIX_OFFSET, 16, 1,
			payload, content_size - sizeof(*hdr), true);
		apba_handle_message(hdr, payload, content_size - sizeof(*hdr));
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

	print_hex_dump_debug("RAW RX: ", DUMP_PREFIX_OFFSET, 16, 1,
		cp, count, true);

	/*
	 * Try to clean up garbage/incomplete chars received
	 * for some reason.
	 */
	if (mud->rx_len &&
	    (jiffies_to_msecs(jiffies - mud->last_rx) >
	     MODS_UART_SEGMENT_TIMEOUT)) {
		mud->stats.rx_timeout++;
		print_hex_dump_debug("RX (timeout): ", DUMP_PREFIX_OFFSET,
			16, 1, mud->rx_data, mud->rx_len, true);
		dev_err(dev, "%s: RX Buffer cleaned up\n", __func__);
		mud->rx_len = 0;
	}
	mud->last_rx = jiffies;

	while (to_be_consumed > 0) {
		int copy_size;

		if (mud->rx_len == MHB_MAX_MSG_SIZE) {
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
			print_hex_dump_debug("RX (overflow): ",
				DUMP_PREFIX_OFFSET, 16, 1,
				mud->rx_data, mud->rx_len, true);
			dev_err(dev, "%s: RX buffer overflow\n", __func__);
			mud->rx_len = 0;
		}

		copy_size = MIN(to_be_consumed,
				MHB_MAX_MSG_SIZE - mud->rx_len);

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
	if (ret) {
		pr_err("%s: ldisc registration failed: %d\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&mods_uart_driver);
	if (ret)
		tty_unregister_ldisc(N_MODS_UART);

	return ret;
}

void mods_uart_exit(void)
{
	int ret;

	platform_driver_unregister(&mods_uart_driver);

	ret = tty_unregister_ldisc(N_MODS_UART);
	if (ret < 0)
		pr_err("%s: ldisc unregistration failed: %d\n", __func__, ret);
}
