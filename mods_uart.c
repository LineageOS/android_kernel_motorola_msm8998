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
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>

#include "crc.h"
#include "muc_attach.h"
#include "mods_nw.h"
#include "muc_svc.h"

#define DRIVERNAME	"mods_uart"
#define N_MODS_UART	25

#pragma pack(push, 1)
struct uart_msg_hdr {
	__le16  nw_msg_size;
};
#pragma pack(pop)

#define MODS_UART_MAX_SIZE (MUC_MSG_SIZE_MAX + sizeof(struct uart_msg_hdr))

struct mods_uart_data {
	struct platform_device *pdev;
	struct tty_struct *tty;
	struct mods_dl_device *dld;
	struct notifier_block attach_nb;
	bool present;
	char rx_data[MODS_UART_MAX_SIZE];
	size_t rx_len;
};

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

static int mods_uart_message_send(struct mods_dl_device *dld,
				  uint8_t *buf, size_t len)
{
	struct mods_uart_data *mud = (struct mods_uart_data *)dld->dl_priv;
	struct device *dev = &mud->pdev->dev;
	int ret;
	u8 *pkt;
	size_t pkt_size;
	struct uart_msg_hdr *hdr;
	__le16 crc16;

	pkt_size = sizeof(struct uart_msg_hdr) + len;
	pkt = kmalloc(pkt_size, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;

	/* Populate the packet */
	hdr = (struct uart_msg_hdr *) pkt;
	hdr->nw_msg_size = cpu_to_le16(len);
	memcpy(pkt + sizeof(*hdr), buf, len);

	crc16 = gen_crc16(pkt, pkt_size);

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

	kfree(pkt);
	return 0;

send_err:
	kfree(pkt);
	return -EIO;
}

/*
 * The cookie value supplied is the value that message_send()
 * returned to its caller.  It identifies the buffer that should be
 * canceled.  This function must also handle (which is to say,
 * ignore) a null cookie value.
 */
static void mods_uart_message_cancel(void *cookie)
{
	/* Should never happen */
}

static struct mods_dl_driver mods_uart_dl_driver = {
	.dl_priv_size		= sizeof(struct mods_uart_data),
	.message_send		= mods_uart_message_send,
	.message_cancel		= mods_uart_message_cancel,
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

static int
mod_attach(struct notifier_block *nb, unsigned long now_present, void *unused)
{
	struct mods_uart_data *mud;
	int err;

	mud = container_of(nb, struct mods_uart_data, attach_nb);
	if (now_present == mud->present)
		return NOTIFY_OK;

	mud->present = now_present;

	if (now_present) {
		err = mods_dl_dev_attached(mud->dld);
		if (err) {
			dev_err(&mud->pdev->dev, "Error attaching to SVC\n");
			mud->present = 0;
		}
	} else
		mods_dl_dev_detached(mud->dld);

	return NOTIFY_OK;
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

	mud->dld = mods_create_dl_device(&mods_uart_dl_driver, &pdev->dev,
					intf_id);
	if (IS_ERR(mud->dld)) {
		dev_err(&pdev->dev, "%s: Unable to create data link device\n",
			__func__);
		return PTR_ERR(mud->dld);
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

	mud->tty->disc_data = mud;
	platform_set_drvdata(pdev, mud);

	mud->attach_nb.notifier_call = mod_attach;
	register_muc_attach_notifier(&mud->attach_nb);

	ret = device_create_file(&pdev->dev, &dev_attr_ldisc_rel);
	if (ret)
		dev_warn(&pdev->dev, "%s: Failed to create sysfs\n", __func__);

	return 0;

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

	return ret;
}

static int mods_uart_remove(struct platform_device *pdev)
{
	struct mods_uart_data *mud = platform_get_drvdata(pdev);
	struct tty_struct *tty = mud->tty;

	if (mud->present)
		mods_dl_dev_detached(mud->dld);

	device_remove_file(&pdev->dev, &dev_attr_ldisc_rel);

	unregister_muc_attach_notifier(&mud->attach_nb);
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

static int n_mods_uart_receive_buf2(struct tty_struct *tty,
				    const unsigned char *cp,
				    char *fp, int count)
{
	struct mods_uart_data *mud = tty->disc_data;
	struct device *dev = &mud->pdev->dev;
	uint16_t calc_crc;
	uint16_t *rcvd_crc;
	struct uart_msg_hdr *hdr;
	uint8_t *payload;
	size_t content_size;

	if (mud->rx_len + count > MODS_UART_MAX_SIZE) {
		dev_err(dev, "%s: RX buffer overflow\n", __func__);
		/* Should not happen as long as data in uart is read out
		   constantly. */
		mud->rx_len = 0;
		return count;
	}

	memcpy(&mud->rx_data[mud->rx_len], cp, count);
	mud->rx_len += count;

	if (mud->rx_len < sizeof(*hdr))
		return count;

	hdr = (struct uart_msg_hdr *) mud->rx_data;
	content_size = sizeof(*hdr) + le16_to_cpu(hdr->nw_msg_size);

	if (mud->rx_len < content_size + sizeof(calc_crc))
		return count;

	rcvd_crc = (uint16_t *)&mud->rx_data[content_size];
	calc_crc = gen_crc16((uint8_t *) mud->rx_data, content_size);
	if (le16_to_cpu(*rcvd_crc) != calc_crc) {
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
		mods_nw_switch(mud->dld, payload,
			       le16_to_cpu(hdr->nw_msg_size));
	}
	mud->rx_len = 0;
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
