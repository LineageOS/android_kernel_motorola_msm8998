/*
 * Copyright (C) 2015 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/workqueue.h>

#include "apba.h"
#include "kernel_ver.h"
#include "mods_nw.h"
#include "mods_uart.h"

#define MAX_PARTITION_NAME (sizeof(((struct mtd_info *)NULL)->name))

#define FFFF_EXT (".ffff")
#define BIN_EXT  (".bin")

#define APBA_FIRMWARE_PARTITION ("apba")
#define APBA_FIRMWARE_NAME ("apba.ffff")

#define APBA_NUM_GPIOS (8)
#define APBA_MAX_SEQ   (APBA_NUM_GPIOS*3*2)

struct apba_seq {
	u32 val[APBA_MAX_SEQ];
	size_t len;
};

struct apba_ctrl {
	struct device *dev;
	struct clk *mclk;
	int gpio_cnt;
	int gpios[APBA_NUM_GPIOS];
	const char *gpio_labels[APBA_NUM_GPIOS];
	int int_index;
	int irq;
	struct apba_seq enable_seq;
	struct apba_seq disable_seq;
	struct apba_seq wake_seq;
	void *mods_uart;
	int desired_on;
	struct mutex log_mutex;
	struct completion comp;
} *g_ctrl;

/* message from APBA Ctrl driver in kernel */
#pragma pack(push, 1)
struct apba_ctrl_msg_hdr {
	__le16 type;
	__le16 size; /* size of data followed by hdr */
};

struct apba_ctrl_int_reason_resp {
	struct apba_ctrl_msg_hdr hdr;
	__le16 reason;
};
#pragma pack(pop)

enum {
	APBA_CTRL_GET_INT_REASON,
	APBA_CTRL_LOG_IND,
	APBA_CTRL_LOG_REQUEST,
};

enum {
	APBA_INT_REASON_NONE,
	APBA_INT_APBE_ON,
	APBA_INT_APBE_RESET,
	APBA_INT_APBE_CONNECTED,
	APBA_INT_APBE_DISCONNECTED,
};

#define APBA_LOG_SIZE	SZ_16K
static DEFINE_KFIFO(apba_log_fifo, char, APBA_LOG_SIZE);

/* used as temporary buffer to pop out content from FIFO */
static char fifo_overflow[MUC_MSG_SIZE_MAX];

#define APBA_LOG_REQ_TIMEOUT	1000 /* ms */

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct apbe_attach_work_struct {
	struct work_struct work;
	int present;
};

static inline struct apba_ctrl *apba_sysfs_to_ctrl(struct device *dev)
{
	return g_ctrl;
}

static int apba_mtd_erase(struct mtd_info *mtd_info,
	 unsigned int start, unsigned int len)
{
	int err;
	struct erase_info ei = {0};

	ei.addr = start;
	ei.len = len;
	ei.mtd = mtd_info;
	err = mtd_info->_erase(mtd_info, &ei);
	return err;
}

static struct mtd_info * apba_init_mtd_module(const char *partition_name)
{
	struct mtd_info *mtd_info;
	int num;

	for (num = 0; num < 16; num++) {
		mtd_info = get_mtd_device(NULL, num);
		if (IS_ERR(mtd_info)) {
			pr_debug("%s: No device for num %d\n", __func__, num);
			continue;
		}

		if (mtd_info->type == MTD_ABSENT) {
			put_mtd_device(mtd_info);
			continue;
		}

		if (strcmp(mtd_info->name, partition_name)) {
			put_mtd_device(mtd_info);
			continue;
		}

		pr_debug("%s: MTD name: %s\n", __func__, mtd_info->name);
		pr_debug("%s: MTD type: %d\n", __func__, mtd_info->type);
		pr_debug("%s: MTD total size : %ld bytes\n", __func__,
			 (long)mtd_info->size);
		pr_debug("%s: MTD erase size : %ld bytes\n", __func__,
			 (long)mtd_info->erasesize);

		return mtd_info;
	}

	return NULL;
}

static int apba_parse_seq(struct device *dev, const char *name,
	struct apba_seq *seq)
{
	int ret;
	int cnt = 0;
	struct property *pp = of_find_property(dev->of_node, name, &cnt);

	cnt /= sizeof(u32);
	if (!pp || cnt == 0 || cnt > seq->len || cnt % 3) {
		pr_err("%s: error reading property %s, cnt = %d\n",
			__func__, name, cnt);
		ret = -EINVAL;
	} else {
		ret = of_property_read_u32_array(dev->of_node, name,
			seq->val, cnt);
		if (ret) {
			pr_err("%s: unable to read %s, ret = %d\n",
				__func__, name, ret);
		} else {
			seq->len = cnt;
		}
	}

	return ret;
}

static void apba_seq(struct apba_ctrl *ctrl, struct apba_seq *seq)
{
	size_t i;

	for (i = 0; i < seq->len; i += 3) {
		u32 index = seq->val[i];
		int value = (int)seq->val[i+1];
		unsigned long delay = (unsigned long)seq->val[i+2];

		/* Set a gpio (if valid). */
		if (index < ARRAY_SIZE(ctrl->gpios)) {
			int gpio = ctrl->gpios[index];

			if (gpio_is_valid(gpio)) {
				pr_debug("%s: set gpio=%d, value=%u\n",
					__func__, gpio, value);
				gpio_set_value(gpio, value);
			}
		}

		/* Delay (if valid). */
		if (delay) {
			usleep_range(delay * 1000, delay * 1000);
			pr_debug("%s: delay=%lu\n",
				__func__, delay);
		}
	}
}

static void apba_on(struct apba_ctrl *ctrl, bool on)
{
	pr_info("%s: %s\n", __func__, on ? "on" : "off");

	if (on)
		apba_seq(ctrl, &ctrl->enable_seq);
	else
		apba_seq(ctrl, &ctrl->disable_seq);
}

static int apba_erase_partition(struct apba_ctrl *ctrl, const char *partition)
{
	struct mtd_info *mtd_info;
	int err;

	if (!ctrl)
		return -EINVAL;

	/* Disable the APBA so that it does not access the flash. */
	apba_on(ctrl, false);

	mtd_info = apba_init_mtd_module(partition);
	if (!mtd_info) {
		pr_err("%s: mtd init module failed for %s, err=%d\n",
			__func__, partition, err);
		goto no_mtd;
	}

	/* Erase the flash */
	err = apba_mtd_erase(mtd_info, 0, mtd_info->size);
	if (err < 0) {
		pr_err("%s: mtd erase failed for %s, err=%d\n",
			__func__, partition, err);
		goto cleanup;
	}

	pr_debug("%s: %s complete\n", __func__, partition);

cleanup:
	put_mtd_device(mtd_info);

no_mtd:
	if (ctrl->desired_on)
		apba_on(ctrl, true);

	return err;
}

static ssize_t erase_partition_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct apba_ctrl *ctrl = platform_get_drvdata(pdev);
	char partition[MAX_PARTITION_NAME];
	int err;

	if (count && buf[count - 1] == '\n')
		count--;

	if (!count || (count >= sizeof(partition)))
		return -EINVAL;

	memcpy(partition, buf, count);
	partition[count] = 0;
	pr_debug("%s: partition=%s\n", __func__, partition);

	err = apba_erase_partition(ctrl, partition);
	if (err < 0)
		pr_err("%s: flashing erase err=%d\n", __func__, err);

	return err ? err : count;
}

static DEVICE_ATTR_WO(erase_partition);

static int apba_flash_partition(struct apba_ctrl *ctrl,
	const char *partition, const struct firmware *fw)
{
	struct mtd_info *mtd_info;
	int err;
	size_t retlen = 0;

	if (!fw || !ctrl)
		return -EINVAL;

	/* Disable the APBA so that it does not access the flash. */
	apba_on(ctrl, false);

	mtd_info = apba_init_mtd_module(partition);
	if (!mtd_info) {
		pr_err("%s: mtd init module failed for %s, err=%d\n",
			__func__, partition, err);
		goto no_mtd;
	}

	/* Erase the flash */
	err = apba_mtd_erase(mtd_info, 0, mtd_info->size);
	if (err < 0) {
		pr_err("%s: mtd flash failed for %s, err=%d\n",
			__func__, partition, err);
		goto cleanup;
	}

	err = mtd_info->_write(mtd_info, 0, fw->size, &retlen, fw->data);

	pr_debug("%s: %s complete\n", __func__, partition);

cleanup:
	put_mtd_device(mtd_info);

no_mtd:
	if (ctrl->desired_on)
		apba_on(ctrl, true);

	return err;
}

static ssize_t flash_partition_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct apba_ctrl *ctrl = platform_get_drvdata(pdev);
	char partition[MAX_PARTITION_NAME];
	/* Null-termination accounted for in *_EXT macros. */
	char fw_name[MAX_PARTITION_NAME + max(sizeof(FFFF_EXT), sizeof(BIN_EXT))];
	const struct firmware *fw = NULL;
	int err;

	if (count && buf[count - 1] == '\n')
		count--;

	if (!count || (count >= sizeof(partition)))
		return -EINVAL;

	/* Try .ffff extension first. */
        memcpy(fw_name, buf, count);
	memcpy(fw_name + count, FFFF_EXT, sizeof(FFFF_EXT));

	err = request_firmware(&fw, fw_name, ctrl->dev);
	if (err < 0) {
		pr_debug("%s: request firmware failed for %s, err=%d\n",
			__func__, partition, err);

		/* Fallback and try .bin extension. */
		memcpy(fw_name + count, BIN_EXT, sizeof(BIN_EXT));
		err = request_firmware(&fw, fw_name, ctrl->dev);
	}

	if (err < 0) {
		pr_err("%s: request firmware failed for %s, err=%d\n",
			__func__, partition, err);
		return err;
	}

	if (!fw || !fw->size) {
		pr_err("%s: firmware invalid for %s\n",
			__func__, partition);
		return -EINVAL;
	}

	memcpy(partition, buf, count);
	partition[count] = 0;
	pr_debug("%s: partition=%s, fw=%s, size=%zu\n",
		__func__, partition, fw_name, fw->size);

	err = apba_flash_partition(ctrl, partition, fw);
	if (err < 0)
		pr_err("%s: flashing failed for %s, err=%d\n",
			__func__, partition, err);

	release_firmware(fw);
	return err ? err : count;
}

static DEVICE_ATTR_WO(flash_partition);

static ssize_t apba_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", g_ctrl->desired_on);
}

static ssize_t apba_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;
	else if (val != 0 && val != 1)
		return -EINVAL;

	if (val)
		apba_enable();
	else
		apba_disable();

	return count;
}

static DEVICE_ATTR_RW(apba_enable);

static ssize_t apba_log_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct apba_ctrl_msg_hdr msg;
	int count;

	if (!g_ctrl)
		return 0;

	msg.type = cpu_to_le16(APBA_CTRL_LOG_REQUEST);
	msg.size = 0;

	if (mods_uart_apba_send(g_ctrl->mods_uart,
				(uint8_t *)&msg, sizeof(msg)) != 0) {
		pr_err("%s: failed to send LOG REQUEST\n", __func__);
		return 0;
	}

	if (!wait_for_completion_timeout(
		    &g_ctrl->comp,
		    msecs_to_jiffies(APBA_LOG_REQ_TIMEOUT))) {
		return 0;
	}

	mutex_lock(&g_ctrl->log_mutex);
	count = kfifo_out(&apba_log_fifo, buf, PAGE_SIZE - 1);
	mutex_unlock(&g_ctrl->log_mutex);

	return count;
}

static DEVICE_ATTR_RO(apba_log);

static struct attribute *apba_attrs[] = {
	&dev_attr_erase_partition.attr,
	&dev_attr_flash_partition.attr,
	&dev_attr_apba_enable.attr,
	&dev_attr_apba_log.attr,
	NULL,
};

ATTRIBUTE_GROUPS(apba);

static void apba_firmware_callback(const struct firmware *fw,
					 void *context)
{
	struct apba_ctrl *ctrl = (struct apba_ctrl *)context;
	int err;

	if (!ctrl) {
		pr_err("%s: invalid ctrl\n", __func__);
		return;
	}

	if (!fw) {
		pr_err("%s: no firmware available\n", __func__);
		if (ctrl->desired_on)
			apba_on(ctrl, true);
		return;
	}

	pr_debug("%s: size=%zu data=%p\n", __func__, fw->size, fw->data);

	/* TODO: extract the version from the binary, check the version,
		  apply the firmware. */

	err = apba_flash_partition(ctrl, APBA_FIRMWARE_PARTITION, fw);
	if (err < 0)
		pr_err("%s: flashing failed err=%d\n", __func__, err);

	/* TODO: notify system, in case of error */

	release_firmware(fw);
}

static irqreturn_t apba_isr(int irq, void *data)
{
	struct apba_ctrl *ctrl = data;
	struct apba_ctrl_msg_hdr msg;
	int value = gpio_get_value(ctrl->gpios[ctrl->int_index]);

	pr_debug("%s: ctrl=%p, value=%d\n", __func__, ctrl, value);

	if (!ctrl->mods_uart)
		return IRQ_HANDLED;

	msg.type = cpu_to_le16(APBA_CTRL_GET_INT_REASON);
	msg.size = 0;

	if (mods_uart_apba_send(ctrl->mods_uart,
				(uint8_t *)&msg, sizeof(msg)) != 0)
		pr_err("%s: failed to send INT_REASON requeist\n", __func__);

	return IRQ_HANDLED;
}

static int apba_int_setup(struct apba_ctrl *ctrl,
	struct device *dev)
{
	int ret;
	int gpio;
	unsigned int flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	ret = of_property_read_u32(dev->of_node,
		"mmi,int-index", &ctrl->int_index);
	if (ret) {
		dev_err(dev, "failed to read int index.\n");
		return ret;
	}

	if (ctrl->int_index < 0 || ctrl->int_index >= ctrl->gpio_cnt) {
		dev_err(dev, "int index out of range: %d\n", ctrl->int_index);
		return -EINVAL;
	}

	gpio = ctrl->gpios[ctrl->int_index];
	ctrl->irq = gpio_to_irq(gpio);
	dev_dbg(dev, "irq: gpio=%d irq=%d\n", gpio, ctrl->irq);

	ret = devm_request_threaded_irq(dev, ctrl->irq, NULL /* handler */,
		apba_isr, flags, "apba_ctrl", ctrl);
	if (ret) {
		dev_err(dev, "irq request failed: %d\n", ret);
		return ret;
	}

	enable_irq_wake(ctrl->irq);

	return ret;
}

static void apba_gpio_free(struct apba_ctrl *ctrl, struct device *dev)
{
	int i;

	for (i = 0; i < ctrl->gpio_cnt; i++) {
		sysfs_remove_link(&dev->kobj, ctrl->gpio_labels[i]);
		gpio_unexport(ctrl->gpios[i]);
	}
}

static int apba_gpio_setup(struct apba_ctrl *ctrl, struct device *dev)
{
	int i;
	int gpio_cnt = of_gpio_count(dev->of_node);
	const char *label_prop = "mmi,gpio-labels";
	int label_cnt = of_property_count_strings(dev->of_node, label_prop);
	int ret;

	if (gpio_cnt <= 0) {
		dev_err(dev, "No GPIOs were defined\n");
		return -EINVAL;
	}

	if (gpio_cnt > ARRAY_SIZE(ctrl->gpios)) {
		dev_err(dev, "%s: gpio count is greater than %zu.\n",
			__func__, ARRAY_SIZE(ctrl->gpios));
		return -EINVAL;
	}

	if (label_cnt != gpio_cnt) {
		dev_err(dev, "%s: label count does not match gpio count.\n",
			__func__);
		return -EINVAL;
	}

	for (i = 0; i < gpio_cnt; i++) {
		enum of_gpio_flags flags = 0;
		int gpio;
		const char *label = NULL;

		gpio = of_get_gpio_flags(dev->of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(dev, "of_get_gpio failed: %d\n", gpio);
			ret = -EINVAL;
			goto gpio_cleanup;
		}

		ret = of_property_read_string_index(dev->of_node,
					label_prop, i, &label);
		if (ret) {
			dev_err(dev, "reading label failed: %d\n", ret);
			goto gpio_cleanup;
		}

		ret = devm_gpio_request_one(dev, gpio, flags, label);
		if (ret)
			goto gpio_cleanup;

		ret = gpio_export(gpio, true);
		if (ret)
			goto gpio_cleanup;

		ret = gpio_export_link(dev, label, gpio);
		if (ret) {
			gpio_unexport(gpio);
			goto gpio_cleanup;
		}

		dev_dbg(dev, "%s: gpio=%d, flags=0x%x, label=%s\n",
			__func__, gpio, flags, label);

		ctrl->gpios[i] = gpio;
		ctrl->gpio_labels[i] = label;
		ctrl->gpio_cnt++;
	}

	return 0;

gpio_cleanup:
	apba_gpio_free(ctrl, dev);

	return ret;
}

int apba_uart_register(void *mods_uart)
{
	if (!g_ctrl)
		return -ENODEV;

	g_ctrl->mods_uart = mods_uart;
	return 0;
}

static void apba_apbe_attach_work_func(struct work_struct *work)
{
	struct apbe_attach_work_struct *apbe;

	apbe = container_of(work, struct apbe_attach_work_struct, work);

	if (g_ctrl && g_ctrl->mods_uart)
		mod_attach(g_ctrl->mods_uart, apbe->present);

	kfree(apbe);
}

static void apba_notify_abpe_attach(int present)
{
	struct apbe_attach_work_struct *apbe;

	if (!g_ctrl)
		return;

	apbe = kzalloc(sizeof(struct apbe_attach_work_struct), GFP_KERNEL);

	if (!apbe)
		return;

	apbe->present = present;
	INIT_WORK(&apbe->work, apba_apbe_attach_work_func);
	schedule_work(&apbe->work);
}

static void apba_action_on_int_reason(uint16_t reason)
{
	pr_info("%s: %d\n", __func__, reason);

	switch (reason) {
	case APBA_INT_APBE_ON:
		/* TODO: will be filled in */
		break;
	case APBA_INT_APBE_RESET:
		/* TODO: will be filled in */
		break;
	case APBA_INT_APBE_CONNECTED:
		apba_notify_abpe_attach(1);
		break;
	case APBA_INT_APBE_DISCONNECTED:
		apba_notify_abpe_attach(0);
		break;
	default:
		pr_debug("%s: Unknown int reason (%d) received.\n",
			 __func__, reason);
		break;
	}
}

void apba_handle_message(uint8_t *payload, size_t len)
{
	int of;

	struct apba_ctrl_msg_hdr *msg;

	if (len < sizeof(struct apba_ctrl_msg_hdr)) {
		pr_err("%s: Invalid message received.\n", __func__);
		return;
	}

	msg = (struct apba_ctrl_msg_hdr *)payload;

	switch (le16_to_cpu(msg->type)) {
	case APBA_CTRL_GET_INT_REASON:
		if (len >= sizeof(struct apba_ctrl_int_reason_resp)) {
			struct apba_ctrl_int_reason_resp *resp;

			resp = (struct apba_ctrl_int_reason_resp *)payload;
			apba_action_on_int_reason(le16_to_cpu(resp->reason));
		}
		break;
	case APBA_CTRL_LOG_IND:
		mutex_lock(&g_ctrl->log_mutex);
		of = kfifo_len(&apba_log_fifo) + msg->size - APBA_LOG_SIZE;
		if (of > 0) {
			/* pop out from older content if buffer is full */
			of = kfifo_out(&apba_log_fifo, fifo_overflow,
				       MIN(of, MUC_MSG_SIZE_MAX));
		}
		kfifo_in(&apba_log_fifo,
			 payload + sizeof(*msg), msg->size);
		mutex_unlock(&g_ctrl->log_mutex);
		break;
	case APBA_CTRL_LOG_REQUEST:
		complete(&g_ctrl->comp);
		break;
	default:
		pr_err("%s: Unknown message received.\n", __func__);
		break;
	}
}

int apba_enable(void)
{
	int ret;

	if (!g_ctrl)
		return -ENODEV;

	ret = clk_prepare_enable(g_ctrl->mclk);
	if (ret) {
		dev_err(g_ctrl->dev, "%s: failed to prepare clock.\n",
			__func__);
		return ret;
	}

	g_ctrl->desired_on = 1;

	ret = request_firmware_nowait(THIS_MODULE, true, APBA_FIRMWARE_NAME,
				      g_ctrl->dev, GFP_KERNEL, g_ctrl,
				      apba_firmware_callback);
	if (ret) {
		dev_err(g_ctrl->dev, "failed to request firmware.\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	clk_disable_unprepare(g_ctrl->mclk);
	g_ctrl->desired_on = 0;

	return ret;
}

void apba_disable(void)
{
	if (!g_ctrl || !g_ctrl->desired_on)
		return;

	if (g_ctrl->mods_uart)
		mod_attach(g_ctrl->mods_uart, 0);

	clk_disable_unprepare(g_ctrl->mclk);
	apba_on(g_ctrl, false);
	g_ctrl->desired_on = 0;
}

static int apba_ctrl_probe(struct platform_device *pdev)
{
	struct apba_ctrl *ctrl;
	int ret;

	if (!pdev->dev.of_node) {
		/* Platform data not currently supported */
		dev_err(&pdev->dev, "%s: of devtree not found\n", __func__);
		return -EINVAL;
	}

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl),
		GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = &pdev->dev;

	ctrl->mclk = devm_clk_get(&pdev->dev, "apba_mclk");
	if (IS_ERR(ctrl->mclk)) {
		dev_err(&pdev->dev, "%s: failed to get clock.\n", __func__);
		return PTR_ERR(ctrl->mclk);
	}

	ret = apba_gpio_setup(ctrl, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to read gpios.\n");
		return ret;
	}

	ret = apba_int_setup(ctrl, &pdev->dev);
	if (ret)
		goto free_gpios;

	ctrl->enable_seq.len = ARRAY_SIZE(ctrl->enable_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,enable-seq",
		&ctrl->enable_seq);
	if (ret)
		goto disable_irq;

	ctrl->disable_seq.len = ARRAY_SIZE(ctrl->disable_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,disable-seq",
		&ctrl->disable_seq);
	if (ret)
		goto disable_irq;

	ctrl->wake_seq.len = ARRAY_SIZE(ctrl->wake_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,wake-seq",
		&ctrl->wake_seq);
	if (ret)
		goto disable_irq;

	mutex_init(&ctrl->log_mutex);
	init_completion(&ctrl->comp);

	ret = sysfs_create_groups(&pdev->dev.kobj, apba_groups);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs attr\n");
		goto disable_irq;
	}

	/* start with APBA turned OFF */
	apba_on(ctrl, false);

	g_ctrl = ctrl;

	platform_set_drvdata(pdev, ctrl);

	return 0;

disable_irq:
	disable_irq_wake(ctrl->irq);
free_gpios:
	apba_gpio_free(ctrl, &pdev->dev);

	return ret;
}

static int apba_ctrl_remove(struct platform_device *pdev)
{
	struct apba_ctrl *ctrl = platform_get_drvdata(pdev);

	sysfs_remove_groups(&pdev->dev.kobj, apba_groups);

	disable_irq_wake(ctrl->irq);
	apba_disable();
	apba_gpio_free(ctrl, &pdev->dev);

	return 0;
}

static const struct of_device_id apba_ctrl_match[] = {
	{.compatible = "mmi,apba-ctrl",},
	{},
};

static const struct platform_device_id apba_ctrl_id_table[] = {
	{"apba_ctrl", 0},
	{},
};

static struct platform_driver apba_ctrl_driver = {
	.driver = {
		.name = "apba_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(apba_ctrl_match),
	},
	.probe = apba_ctrl_probe,
	.remove = apba_ctrl_remove,
	.id_table = apba_ctrl_id_table,
};

int __init apba_ctrl_init(void)
{
	return platform_driver_register(&apba_ctrl_driver);
}

void __exit apba_ctrl_exit(void)
{
	platform_driver_unregister(&apba_ctrl_driver);
}
