/*
 * Copyright (C) 2015 Motorola Mobility LLC
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include "muc.h"

static BLOCKING_NOTIFIER_HEAD(muc_attach_chain_head);

static int muc_attach_notifier_call_chain(unsigned long val)
{
	int ret;

	pr_debug("%s val = %lu\n", __func__, val);
	ret  = blocking_notifier_call_chain(&muc_attach_chain_head,
			val, NULL);
	return notifier_to_errno(ret);
}

unsigned long muc_attach_get_state(void)
{
	if (!muc_misc_data)
		return 0;

	return (unsigned long)muc_misc_data->muc_detected;
}
EXPORT_SYMBOL(muc_attach_get_state);

int register_muc_attach_notifier(struct notifier_block *nb)
{
	int rv;

	pr_debug("%s <- %pS\n", __func__, __builtin_return_address(0));

	rv = blocking_notifier_chain_register(&muc_attach_chain_head, nb);

	if (!rv)
		(void)muc_attach_notifier_call_chain(
				muc_attach_get_state());

	return rv;
}
EXPORT_SYMBOL(register_muc_attach_notifier);

int unregister_muc_attach_notifier(struct notifier_block *nb)
{
	pr_debug("%s <- %pS\n", __func__, __builtin_return_address(0));

	return blocking_notifier_chain_unregister(&muc_attach_chain_head, nb);
}
EXPORT_SYMBOL(unregister_muc_attach_notifier);

static void muc_seq(struct muc_data *cdata, u32 seq[],
	size_t seq_len)
{
	size_t i;
	for (i = 0; i < seq_len; i += 3) {
		u32 index = seq[i];
		int value = (int)seq[i+1];
		unsigned long delay = (unsigned long)seq[i+2];

		/* Set a gpio (if valid). */
		if (index < ARRAY_SIZE(cdata->gpios)) {
			int gpio = cdata->gpios[index];

			if (gpio_is_valid(gpio)) {
				pr_debug("%s:%d: set gpio=%d, value=%u\n",
					__func__, __LINE__, gpio, value);
				gpio_set_value(gpio, value);
			}
		}

		/* Delay (if valid). */
		if (delay) {
			usleep_range(delay * 1000, delay * 1000);
			pr_debug("%s:%d: delay=%lu\n",
				__func__, __LINE__, delay);
		}
	}
}

static void muc_handle_detection(struct muc_data *cdata)
{
	int err;
	/* Detection gpio is active-low. */
	bool tmp_detected = gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]) == 0;

	if (tmp_detected == cdata->muc_detected) {
		pr_debug("%s:%d: detection has not changed: %d\n",
			__func__, __LINE__, tmp_detected);
		return;
	}

	/* Power on first when inserted */
	if (tmp_detected)
		muc_seq(cdata, cdata->en_seq, cdata->en_seq_len);

	err = muc_attach_notifier_call_chain((unsigned int)tmp_detected);
	if (err)
		pr_err("notification chain failed %d\n", err);

	pr_debug("%s:%d detected=%d\n", __func__, __LINE__, tmp_detected);
	cdata->muc_detected = tmp_detected;

	/* Power off on removal */
	if (!tmp_detected)
		muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);

	return;
}

static irqreturn_t muc_isr(int irq, void *data)
{
	struct muc_data *cdata = data;

	if (cdata->det_hysteresis) {
		usleep_range(cdata->det_hysteresis * 1000,
					cdata->det_hysteresis * 1000);
		pr_debug("%s:%d: det_hysteresis=%u\n",
			__func__, __LINE__, cdata->det_hysteresis);
	}

	muc_handle_detection(cdata);

	return IRQ_HANDLED;
}

int muc_intr_setup(struct muc_data *cdata, struct device *dev)
{
	int ret;
	int gpio;
	unsigned int flags;

	flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	gpio = cdata->gpios[MUC_GPIO_DET_N];
	cdata->irq = gpio_to_irq(gpio);
	dev_dbg(dev, "%s:%d irq: gpio=%d irq=%d\n",
		__func__, __LINE__, gpio, cdata->irq);

	ret = request_threaded_irq(cdata->irq, NULL /* handler */, muc_isr,
				   flags, "muc_ctrl", cdata);
	if (ret) {
		dev_err(dev, "%s:%d irq request failed: %d\n",
			__func__, __LINE__, ret);
		return ret;
	}

	enable_irq_wake(cdata->irq);

	return ret;
}

void muc_intr_destroy(struct muc_data *cdata, struct device *dev)
{
	disable_irq_wake(cdata->irq);
	free_irq(cdata->irq, cdata);
}

static int muc_gpio_setup(struct muc_data *cdata, struct device *dev)
{
	int i;
	int gpio_cnt = of_gpio_count(dev->of_node);
	const char *label_prop = "mmi,muc-ctrl-gpio-labels";
	int label_cnt = of_property_count_strings(dev->of_node, label_prop);

	if (gpio_cnt != ARRAY_SIZE(cdata->gpios)) {
		dev_err(dev, "gpio count is %d expected %zu.\n",
			gpio_cnt, ARRAY_SIZE(cdata->gpios));
		return -EINVAL;
	}

	for (i = 0; i < gpio_cnt; i++) {
		enum of_gpio_flags flags = 0;
		int gpio;
		int ret;
		const char *label = NULL;

		gpio = of_get_gpio_flags(dev->of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(dev, "of_get_gpio failed: %d\n", gpio);
			return -EINVAL;
		}

		if (i < label_cnt)
			of_property_read_string_index(dev->of_node, label_prop,
				i, &label);

		ret = devm_gpio_request_one(dev, gpio, flags, label);
		if (ret) {
			dev_err(dev, "Failed to get gpio %d\n", gpio);
			return ret;
		}
		gpio_export(gpio, true);

		dev_dbg(dev, "%s:%d gpio=%d, flags=0x%x, label=%s\n",
			__func__, __LINE__, gpio, flags, label);

		cdata->gpios[i] = gpio;
	}

	return 0;
}

static int muc_parse_seq(struct muc_data *cdata,
	struct device *dev, const char *name, u32 *seq, size_t *len)
{
	int ret;
	int cnt = 0;
	struct property *pp = of_find_property(dev->of_node, name, &cnt);

	cnt /= sizeof(u32);
	if (!pp || cnt == 0 || cnt > *len || cnt % 3) {
		dev_err(dev, "%s:%d, error reading property %s, cnt = %d\n",
			__func__, __LINE__, name, cnt);
		return -EINVAL;
	}


	ret = of_property_read_u32_array(dev->of_node, name, seq, cnt);
	if (ret)
		dev_err(dev, "%s:%d, unable to read %s, ret = %d\n",
			__func__, __LINE__, name, ret);
	else
		*len = cnt;

	return ret;
}

int muc_gpio_init(struct device *dev, struct muc_data *cdata)
{
	int ret;

	/* Mandatory configuration */
	ret = muc_gpio_setup(cdata, dev);
	if (ret) {
		dev_err(dev, "%s:%d: failed to read gpios.\n",
			__func__, __LINE__);
		return ret;
	}

	/* Optional configuration */
	cdata->en_seq_len = ARRAY_SIZE(cdata->en_seq);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-en-seq",
		cdata->en_seq, &cdata->en_seq_len);
	if (ret) {
		dev_warn(dev, "%s:%d failed to read enable sequence.\n",
			__func__, __LINE__);
		cdata->en_seq_len = 0;
	}

	cdata->dis_seq_len = ARRAY_SIZE(cdata->dis_seq);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-dis-seq",
		cdata->dis_seq, &cdata->dis_seq_len);
	if (ret) {
		dev_warn(dev, "%s:%d failed to read disable sequence.\n",
			__func__, __LINE__);
		cdata->dis_seq_len = 0;
	}

	ret = of_property_read_u32(dev->of_node,
		"mmi,muc-ctrl-det-hysteresis", &cdata->det_hysteresis);
	if (ret) {
		dev_warn(dev, "%s:%d failed to read det hysteresis.\n",
			__func__, __LINE__);
	}

	/* Handle initial detection state. */
	muc_handle_detection(cdata);

	return 0;
}

void muc_gpio_exit(struct device *dev, struct muc_data *cdata)
{
	/* Disable the module on unload */
	muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);
}
