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
#include <linux/slab.h>

#include "muc.h"

static BLOCKING_NOTIFIER_HEAD(muc_attach_chain_head);
static void muc_reset_do_work(struct work_struct *work);

static int muc_attach_notifier_call_chain(unsigned long val)
{
	int ret;

	pr_debug("%s val = %lu\n", __func__, val);
	ret  = blocking_notifier_call_chain(&muc_attach_chain_head,
			val, NULL);
	return notifier_to_errno(ret);
}

int register_muc_attach_notifier(struct notifier_block *nb)
{
	int rv;
	unsigned long state = 0;

	pr_debug("%s <- %pS\n", __func__, __builtin_return_address(0));

	if (muc_misc_data)
		state = (unsigned long)muc_misc_data->muc_detected;

	rv = blocking_notifier_chain_register(&muc_attach_chain_head, nb);

	if (!rv)
		(void)muc_attach_notifier_call_chain(state);

	return rv;
}
EXPORT_SYMBOL(register_muc_attach_notifier);

int unregister_muc_attach_notifier(struct notifier_block *nb)
{
	pr_debug("%s <- %pS\n", __func__, __builtin_return_address(0));

	return blocking_notifier_chain_unregister(&muc_attach_chain_head, nb);
}
EXPORT_SYMBOL(unregister_muc_attach_notifier);

static void muc_seq(struct muc_data *cdata, u32 seq[], size_t seq_len)
{
	size_t i;

	for (i = 0; i < seq_len; i += 3) {
		u32 index = seq[i];
		int value = (int)seq[i+1];
		unsigned long delay = (unsigned long)seq[i+2];

		/* Set a gpio (if valid). */
		if (index < ARRAY_SIZE(cdata->gpios)) {
			int gpio = cdata->gpios[index];

			pr_debug("%s:%d: set gpio=%d, value=%u\n",
				__func__, __LINE__, gpio, value);
			gpio_set_value(gpio, value);
		}

		/* Delay (if valid). */
		if (delay) {
			msleep(delay);
			pr_debug("%s:%d: delay=%lu\n",
				__func__, __LINE__, delay);
		}
	}
}

static void muc_handle_detection(struct muc_data *cdata)
{
	bool detected = gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]) == 0;
	int err;

	pr_debug("%s: detected: %d previous state: %d\n",
			__func__, detected, cdata->muc_detected);

	/* If this is a force removal, send out removal first if we were
	 * previously detected
	 */
	if (cdata->force_removal && cdata->muc_detected) {
		pr_debug("%s: sending force removal\n", __func__);
		err = muc_attach_notifier_call_chain(0);
		if (err)
			pr_warn("%s: force notify fail: %d\n", __func__, err);

		/* If we were doing a force removal, and we are still detected
		 * re-queue it for debouncing so it settles.
		 */
		if (detected) {
			cdata->force_removal = false;
			cdata->muc_detected = false;
			queue_delayed_work(cdata->attach_wq,
						&cdata->attach_work,
						cdata->det_hysteresis);
			return;
		}
	}
	cdata->force_removal = false;

	if (detected == cdata->muc_detected) {
		pr_debug("%s: detection in same state, skipping\n", __func__);
		return;
	}

	cdata->muc_detected = detected;

	/* Send enable sequence when detected and in disabled. */
	if (detected && cdata->bplus_state == MUC_BPLUS_DISABLED) {
		cdata->bplus_state = MUC_BPLUS_ENABLING;
		muc_seq(cdata, cdata->en_seq, cdata->en_seq_len);
		cdata->bplus_state = MUC_BPLUS_ENABLED;

		/* Re-read state after BPLUS settle time */
		detected = gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]) == 0;
	}

	err = muc_attach_notifier_call_chain(detected);
	if (err)
		pr_warn("%s: notification failed: %d for detected = %s\n",
			__func__, err, detected ? "true" : "false");

	if (!detected && cdata->bplus_state == MUC_BPLUS_ENABLED) {
		muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);
		cdata->bplus_state = MUC_BPLUS_DISABLED;
	}
}

static void attach_work(struct work_struct *work)
{
	struct delayed_work *workitem;
	struct muc_data *cdata;

	workitem = container_of(work, struct delayed_work, work);
	cdata = container_of(workitem, struct muc_data, attach_work);

	muc_handle_detection(cdata);
}

static irqreturn_t muc_isr(int irq, void *data)
{
	struct muc_data *cdata = data;
	bool det = gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]) == 0;
	bool res;

	/* Ignore CC pin during BPLUS enable sequence due to propagation
	 * of the reset/power-on sequence of the MUC.
	 */
	if (cdata->bplus_state == MUC_BPLUS_ENABLING)
		return IRQ_HANDLED;

	pr_debug("%s: detected: %d previous state: %d\n",
			__func__, det, cdata->muc_detected);

	/* Always cancel existing work */
	res = cancel_delayed_work_sync(&cdata->attach_work);
	if (res)
		pr_debug("%s: Cancelled existing work\n", __func__);

	cdata->force_removal = !det ? true : false;

	queue_delayed_work(cdata->attach_wq, &cdata->attach_work,
			cdata->force_removal ? 0 : cdata->det_hysteresis);

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

	ret = devm_request_threaded_irq(dev, cdata->irq, NULL, muc_isr,
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
}

static int muc_gpio_setup(struct muc_data *cdata, struct device *dev)
{
	int i;
	int gpio_cnt = of_gpio_count(dev->of_node);
	const char *label_prop = "mmi,muc-ctrl-gpio-labels";
	int label_cnt = of_property_count_strings(dev->of_node, label_prop);
	int ret;

	if (gpio_cnt != ARRAY_SIZE(cdata->gpios)) {
		dev_err(dev, "gpio count is %d expected %zu.\n",
			gpio_cnt, ARRAY_SIZE(cdata->gpios));
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
			goto free_gpios;
		}

		if (i < label_cnt)
			of_property_read_string_index(dev->of_node, label_prop,
				i, &label);

		ret = devm_gpio_request_one(dev, gpio, flags, label);
		if (ret) {
			dev_err(dev, "Failed to get gpio %d\n", gpio);
			goto free_gpios;
		}
		gpio_export(gpio, true);

		dev_dbg(dev, "%s:%d gpio=%d, flags=0x%x, label=%s\n",
			__func__, __LINE__, gpio, flags, label);

		cdata->gpios[i] = gpio;
	}

	return 0;

free_gpios:
	for (--i; i >= 0; --i)
		gpio_unexport(cdata->gpios[i]);

	return ret;
}

static void muc_gpio_cleanup(struct muc_data *cdata, struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cdata->gpios); i++)
		gpio_unexport(cdata->gpios[i]);
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

#define MSEC_TO_JIFFIES(msec) ((msec) * HZ / 1000)
int muc_gpio_init(struct device *dev, struct muc_data *cdata)
{
	int ret;

	/* WQ for 'fake' reset sequence where we can't detect the
	 * actual reset on the detection line.
	 */
	INIT_WORK(&cdata->reset_work, muc_reset_do_work);
	cdata->wq = alloc_workqueue("muc_reset", WQ_UNBOUND, 1);
	if (!cdata->wq) {
		dev_err(dev, "Failed to create reset workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&cdata->attach_work, attach_work);
	cdata->attach_wq = alloc_workqueue("muc_attach", WQ_UNBOUND, 1);
	if (!cdata->attach_wq) {
		dev_err(dev, "Failed to create attach workqueue\n");
		goto freewq;
	}

	/* Mandatory configuration */
	ret = muc_gpio_setup(cdata, dev);
	if (ret) {
		dev_err(dev, "%s:%d: failed to read gpios.\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	cdata->en_seq_len = ARRAY_SIZE(cdata->en_seq);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-en-seq",
		cdata->en_seq, &cdata->en_seq_len);
	if (ret) {
		dev_err(dev, "%s:%d failed to read enable sequence.\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	cdata->dis_seq_len = ARRAY_SIZE(cdata->dis_seq);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-dis-seq",
		cdata->dis_seq, &cdata->dis_seq_len);
	if (ret) {
		dev_err(dev, "%s:%d failed to read disable sequence.\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	/* Force Flash Sequences (mod core dependent) */
	cdata->ff_seq_v1_len = ARRAY_SIZE(cdata->ff_seq_v1);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-ff-seq-v1",
		cdata->ff_seq_v1, &cdata->ff_seq_v1_len);
	if (ret) {
		dev_err(dev, "%s:%d no ff sequence (v1)\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	cdata->ff_seq_v2_len = ARRAY_SIZE(cdata->ff_seq_v2);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-ff-seq-v2",
		cdata->ff_seq_v2, &cdata->ff_seq_v2_len);
	if (ret) {
		dev_err(dev, "%s:%d no ff sequence (v2)\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	ret = of_property_read_u32(dev->of_node,
		"mmi,muc-ctrl-det-hysteresis", &cdata->det_hysteresis);
	if (ret) {
		dev_warn(dev, "%s:%d failed to read det hysteresis.\n",
			__func__, __LINE__);
	}
	cdata->det_hysteresis = MSEC_TO_JIFFIES(cdata->det_hysteresis);

	/* Handle initial detection state. */
	queue_delayed_work(cdata->attach_wq, &cdata->attach_work,
				cdata->det_hysteresis);

	return 0;
free_attach_wq:
	destroy_workqueue(cdata->attach_wq);
freewq:
	destroy_workqueue(cdata->wq);

	return ret;
}

void muc_gpio_exit(struct device *dev, struct muc_data *cdata)
{
	muc_gpio_cleanup(cdata, dev);
	/* Disable the module on unload */
	muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);
	cancel_delayed_work_sync(&cdata->attach_work);
	destroy_workqueue(cdata->attach_wq);
	cancel_work_sync(&cdata->reset_work);
	destroy_workqueue(cdata->wq);
}

/* The simulated reset is performed in cases where we've asked
 * the mod to reset itself and we do not have capability to detect
 * that reset on the CC pin. We send out a detach, wait for 4s,
 * then send out an attach. This is a wait-and-pray for older
 * hardware.
 */
static void muc_reset_do_work(struct work_struct *work)
{
	disable_irq_wake(muc_misc_data->irq);
	disable_irq(muc_misc_data->irq);

	/* Cancel any pending work and reset the detection states */
	cancel_delayed_work_sync(&muc_misc_data->attach_work);
	muc_misc_data->force_removal = false;
	muc_misc_data->muc_detected = false;

	muc_attach_notifier_call_chain(0);

	msleep(4000);

	queue_delayed_work(muc_misc_data->attach_wq,
				&muc_misc_data->attach_work, 0);

	enable_irq(muc_misc_data->irq);
	enable_irq_wake(muc_misc_data->irq);
}

void muc_simulate_reset(void)
{
	queue_work(muc_misc_data->wq, &muc_misc_data->reset_work);
}

static void __muc_ff_reset(u8 root_ver, bool reset)
{
	struct muc_data *cd = muc_misc_data;
	unsigned int flags;
	int ret;

	/* Take control of BPLUS, ignoring interrupts until done */
	cd->bplus_state = MUC_BPLUS_ENABLING;

	/* In order to provide reset / force flash, need to control the CC pin,
	 * which means free/disable the IRQ, and set as an output low.
	 */
	disable_irq_wake(cd->irq);
	disable_irq(cd->irq);
	devm_free_irq(cd->dev, cd->irq, cd);
	gpio_direction_output(cd->gpios[MUC_GPIO_DET_N], 0);

	/* Perform force flash sequence */
	if (root_ver <= MUC_ROOT_V1)
		muc_seq(cd, cd->ff_seq_v1, cd->ff_seq_v1_len);
	else
		muc_seq(cd, cd->ff_seq_v2, cd->ff_seq_v2_len);


	gpio_direction_input(cd->gpios[MUC_GPIO_DET_N]);

	/* Re-setup the IRQ */
	flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	ret = devm_request_threaded_irq(cd->dev, cd->irq, NULL, muc_isr,
				   flags, "muc_ctrl", cd);
	if (ret) {
		pr_err("%s: Failed re-request IRQ!\n", __func__);
		BUG();
	}
	enable_irq_wake(cd->irq);

	/* Reset from FF simply does the disable sequence */
	if (reset) {
		muc_seq(cd, cd->dis_seq, cd->dis_seq_len);
		cd->bplus_state = MUC_BPLUS_DISABLED;
	} else
		cd->bplus_state = MUC_BPLUS_ENABLED;

	cd->force_removal = true;
	queue_delayed_work(cd->attach_wq, &cd->attach_work, 0);
}

void muc_force_flash(u8 root_ver)
{
	__muc_ff_reset(root_ver, false);
}

void muc_hard_reset(u8 root_ver)
{
	__muc_ff_reset(root_ver, true);
}

