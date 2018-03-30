/*
 * Copyright (C) 2015-2016 Motorola Mobility LLC
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
#include <linux/ratelimit.h>
#include <linux/slab.h>

#include "muc.h"

static BLOCKING_NOTIFIER_HEAD(muc_attach_chain_head);
static BLOCKING_NOTIFIER_HEAD(muc_reset_chain_head);

static void do_muc_ff_reset(struct work_struct *work);
static int muc_pinctrl_select_state_con(struct muc_data *cdata);

static int muc_attach_notifier_call_chain(unsigned long val)
{
	int ret;
	struct muc_data *cd = muc_misc_data;

	/* Log the new state and interrupts since last change in state */
	if (cd) {
		pr_info("muc_attach_%s: val = %lu intr = %d b+fault = %d\n",
				cd->det_testmode ? "TESTMODE" : "state",
				val, cd->intr_count, cd->bplus_fault_cnt);
		cd->bplus_fault_cnt = 0;
		cd->intr_count = 0;
	}

	ret = blocking_notifier_call_chain(&muc_attach_chain_head,
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

static int muc_reset_notifier_call_chain(void)
{
	int ret;

	ret = blocking_notifier_call_chain(&muc_reset_chain_head, 0, NULL);

	return notifier_to_errno(ret);
}

int register_muc_reset_notifier(struct notifier_block *nb)
{
	pr_debug("%s <- %pS\n", __func__, __builtin_return_address(0));

	return blocking_notifier_chain_register(&muc_reset_chain_head, nb);
}
EXPORT_SYMBOL(register_muc_reset_notifier);

int unregister_muc_reset_notifier(struct notifier_block *nb)
{
	pr_debug("%s <- %pS\n", __func__, __builtin_return_address(0));

	return blocking_notifier_chain_unregister(&muc_reset_chain_head, nb);
}
EXPORT_SYMBOL(unregister_muc_reset_notifier);

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

static bool muc_short_detected(struct muc_data *cdata)
{
	bool shortdet = false;

	/* De-assert wake_n */
	gpio_set_value(cdata->gpios[MUC_GPIO_WAKE_N], 1);

	pr_debug("%s: wake: %d int: %d ready: %d\n", __func__,
		gpio_get_value(cdata->gpios[MUC_GPIO_WAKE_N]),
		gpio_get_value(cdata->gpios[MUC_GPIO_INT_N]),
		gpio_get_value(cdata->gpios[MUC_GPIO_READY_N]));

	/* Read INT_N, if not asserted no short */
	if (gpio_get_value(cdata->gpios[MUC_GPIO_INT_N]))
		return false;

	/* Assert wake_n, sleep for 1ms */
	gpio_set_value(cdata->gpios[MUC_GPIO_WAKE_N], 0);
	usleep_range(1000, 1010);

	pr_debug("%s: after assert: wake: %d int: %d ready: %d\n", __func__,
		gpio_get_value(cdata->gpios[MUC_GPIO_WAKE_N]),
		gpio_get_value(cdata->gpios[MUC_GPIO_INT_N]),
		gpio_get_value(cdata->gpios[MUC_GPIO_READY_N]));

	/* If READY or INT are de-asserted, there is no short */
	if (gpio_get_value(cdata->gpios[MUC_GPIO_READY_N]))
		goto clear_wake;
	if (gpio_get_value(cdata->gpios[MUC_GPIO_INT_N]))
		goto clear_wake;

	shortdet = true;
	pr_debug("%s: READY and INT asserted, short detected!\n", __func__);

clear_wake:
	gpio_set_value(cdata->gpios[MUC_GPIO_WAKE_N], 1);

	return shortdet;
}

static void muc_send_uevent(const char *error)
{
	struct kobj_uevent_env *env;

	env = kzalloc(sizeof(*env), GFP_KERNEL);
	if (!env)
		return;

	add_uevent_var(env, error);
	kobject_uevent_env(&muc_misc_data->dev->kobj, KOBJ_CHANGE, env->envp);
	kfree(env);
}


#define MUC_SHORT_MAX_RETRIES 8
static void muc_handle_short(struct muc_data *cdata)
{
	int err;

	muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);
	cdata->bplus_state = MUC_BPLUS_DISABLED;
	cdata->pinctrl_disconnect = true;

	/* Force a removal if we were previously detected */
	if (cdata->muc_detected) {
		err = muc_attach_notifier_call_chain(0);
		if (err)
			pr_warn("%s: force notify fail: %d\n", __func__, err);
	}

	if (cdata->short_count == 0) {
		pr_err("%s: Short detected, disabled BPLUS\n", __func__);
		muc_send_uevent("MOD_ERROR=SHORT_DETECTED");
	}

	cdata->muc_detected = false;

	/* Queue another detection if we haven't exceeded max short retries */
	if (cdata->short_count++ < MUC_SHORT_MAX_RETRIES)
		queue_delayed_work(cdata->attach_wq,
					&cdata->isr_work.work,
					cdata->det_hysteresis);
	else {
		pr_err("%s: Too many sequential shorts detected\n", __func__);
		muc_send_uevent("MOD_ERROR=SHORT_RECOVERY_FAIL");
	}
}

static void muc_handle_detection(bool force_removal)
{
	struct muc_data *cdata = muc_misc_data;
	bool detected = gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]) == 0;
	int err;

	/* If detected, check for short whenever BPLUS is disabled */
	if (detected && cdata->bplus_state == MUC_BPLUS_DISABLED)
		if (muc_short_detected(cdata)) {
			muc_handle_short(cdata);
			return;
		}
	cdata->short_count = 0;

	pr_debug("%s: detected: %d previous state: %d\n",
			__func__, detected, cdata->muc_detected);

	/* If this is a force removal, we were previously detected, and we
	 * still are detected, we need to send out the removal and then
	 * debounce the newly detected state.
	 */
	if (force_removal && cdata->muc_detected && detected) {
		pr_debug("%s: sending force removal\n", __func__);
		err = muc_attach_notifier_call_chain(0);
		if (err)
			pr_warn("%s: force notify fail: %d\n", __func__, err);

		cdata->muc_detected = false;

		/* Disable BPLUS on force removal to guarantee the attached mod
		 * sees the BPLUS removal.
		 */
		muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);
		cdata->bplus_state = MUC_BPLUS_DISABLED;

		/* Perform a normal/isr detection */
		queue_delayed_work(cdata->attach_wq,
					&cdata->isr_work.work,
					cdata->det_hysteresis);
		return;
	}

	if (detected == cdata->muc_detected) {
		pr_debug("%s: detection in same state, skipping\n", __func__);

		/* If we're detected, or don't need to mux the spi pins on
		 * removal we're all done.
		 */
		if (detected || !cdata->pinctrl_disconnect)
			return;

		cdata->pinctrl_disconnect = false;
		pinctrl_select_state(cdata->pinctrl, cdata->pins_discon);
		pr_debug("%s: pinctrl: disconnected\n", __func__);

		return;
	}

	/* Send enable sequence when detected and in disabled. */
	if (detected && cdata->bplus_state == MUC_BPLUS_DISABLED) {
		cdata->bplus_state = MUC_BPLUS_TRANSITIONING;
		muc_seq(cdata, cdata->en_seq, cdata->en_seq_len);
		cdata->bplus_state = MUC_BPLUS_ENABLED;

		/* Select SPI/I2C based on CLK signal */
		if (!cdata->i2c_transport_err &&
				gpio_get_value(cdata->gpios[MUC_GPIO_CLK])) {
			pr_info("%s: I2C selected\n", __func__);
			muc_register_i2c();
		}
#ifdef CONFIG_MODS_2ND_GEN
		else if (gpio_get_value(cdata->gpios[MUC_GPIO_SPI_MISO]) &&
				!gpio_get_value(cdata->gpios[MUC_GPIO_CLK])) {
			gpio_direction_input(cdata->gpios[MUC_GPIO_SPI_MOSI]);
			if (gpio_get_value(cdata->gpios[MUC_GPIO_SPI_MOSI])) {
				pr_info("%s: 2nd mods bus selection, miso high,i2c selected\n", __func__);
				pinctrl_select_state(cdata->pinctrl, cdata->pins_i2c_con);
				muc_seq(cdata, cdata->select_i2c_seq, cdata->select_i2c_seq_len);
				muc_register_i2c();
			} else {
				pr_info("%s: 2nd mods bus selection, SPI selected\n", __func__);
				gpio_direction_output(cdata->gpios[MUC_GPIO_SPI_MOSI], 0);
				cdata->i2c_transport_err = false;
				muc_register_spi();
			}
		}
#endif
		else {
			pr_info("%s: SPI selected\n", __func__);
			cdata->i2c_transport_err = false;
			muc_register_spi();
		}

		if (muc_pinctrl_select_state_con(cdata))
			pr_warn("%s: select active pinctrl failed\n",
				__func__);

		/* Re-read state after BPLUS settle time */
		detected = gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]) == 0;
	}

	cdata->muc_detected = detected;

	err = muc_attach_notifier_call_chain(detected);
	if (err)
		pr_warn("%s: notification failed: %d for detected = %s\n",
			__func__, err, detected ? "true" : "false");

	if (!detected && cdata->bplus_state == MUC_BPLUS_ENABLED) {
		if (pinctrl_select_state(cdata->pinctrl, cdata->pins_discon))
			pr_warn("%s: select disconnected pinctrl failed\n",
				__func__);

		muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);
		cdata->bplus_state = MUC_BPLUS_DISABLED;
	}
}

static void attach_work(struct work_struct *work)
{
	struct delayed_work *workitem;
	struct muc_attach_work *muc_work;
	bool force;

	workitem = container_of(work, struct delayed_work, work);
	muc_work = container_of(workitem, struct muc_attach_work, work);

	force = muc_work->force_removal;
	muc_work->force_removal = false;

	pr_debug("%s: force: %s\n", __func__, force ? "yes" : "no");

	muc_handle_detection(force);
}

static DEFINE_RATELIMIT_STATE(bpf_rate_state, HZ, 1);
static irqreturn_t muc_bplus_fault(int irq, void *data)
{
	struct muc_data *cdata = data;
	int level;

	level = gpio_get_value(cdata->gpios[MUC_GPIO_BPLUS_FAULT_N]);

	/* Accounting and logging when asserted only */
	if (!level) {
		cdata->bplus_fault_cnt++;

		if (__ratelimit(&bpf_rate_state))
			muc_send_uevent("MOD_ERROR=BPLUS_FAULT_DETECTED");
	}

	return IRQ_HANDLED;
}

static irqreturn_t muc_isr(int irq, void *data)
{
	struct muc_data *cdata = data;
	bool det = gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]) == 0;
	bool res;

	/* Ignore CC pin during BPLUS enable sequence due to propagation
	 * of the reset/power-on sequence of the MUC.
	 */
	if (cdata->bplus_state == MUC_BPLUS_TRANSITIONING)
		return IRQ_HANDLED;

	cdata->intr_count++;

	pr_debug("%s: detected: %d previous state: %d\n",
			__func__, det, cdata->muc_detected);

	/* Always cancel existing isr work */
	res = cancel_delayed_work_sync(&cdata->isr_work.work);
	if (res)
		pr_debug("%s: Cancelled existing work\n", __func__);

	cdata->isr_work.force_removal = !det ? true : false;

	queue_delayed_work(cdata->attach_wq, &cdata->isr_work.work,
		cdata->isr_work.force_removal ?
		cdata->rm_hysteresis : cdata->det_hysteresis);

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

	gpio = cdata->gpios[MUC_GPIO_BPLUS_FAULT_N];
	if (gpio_is_valid(gpio)) {
		cdata->bplus_fault_irq = gpio_to_irq(gpio);
		ret = devm_request_threaded_irq(dev, cdata->bplus_fault_irq,
					NULL, muc_bplus_fault,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"muc_bplus_fault_n", cdata);
		if (ret) {
			dev_err(dev, "%s:%d bplus fault irq failed: %d\n",
				__func__, __LINE__, ret);
			cdata->gpios[MUC_GPIO_BPLUS_FAULT_N] = -ENODEV;
			return ret;
		}
		enable_irq_wake(cdata->bplus_fault_irq);
	}

	enable_irq_wake(cdata->irq);

	/* Handle initial detection state. */
	queue_delayed_work(cdata->attach_wq, &cdata->isr_work.work,
				cdata->det_hysteresis);

	return ret;
}

void muc_intr_destroy(struct muc_data *cdata, struct device *dev)
{

	if (!cdata->det_testmode) {
		disable_irq_wake(cdata->irq);
		disable_irq(cdata->irq);
	}

	if (gpio_is_valid(cdata->gpios[MUC_GPIO_BPLUS_FAULT_N])) {
		disable_irq_wake(cdata->bplus_fault_irq);
		disable_irq(cdata->bplus_fault_irq);
	}
}

int muc_gpio_ack_cfg(bool en)
{
	int ret;

	/* Only allow the configuration to change if the muc is detected */
	if (!muc_gpio_ack_is_supported() || !muc_misc_data->muc_detected ||
	    !muc_misc_data->spi_transport_done)
		return -ENODEV;

	if (en)
		ret = pinctrl_select_state(muc_misc_data->pinctrl,
					   muc_misc_data->pins_spi_ack);
	else
		ret = pinctrl_select_state(muc_misc_data->pinctrl,
					   muc_misc_data->pins_spi_con);

	if (ret)
		pr_warn("%s: select SPI pinctrl failed (en = %d)\n",
			__func__, en);

	return ret;
}

static int muc_pinctrl_select_state_con(struct muc_data *cdata)
{
	if (cdata->spi_transport_done)
		return pinctrl_select_state(cdata->pinctrl,
					    cdata->pins_spi_con);
	else if (cdata->i2c_transport_done)
		return pinctrl_select_state(cdata->pinctrl,
					    cdata->pins_i2c_con);

	dev_err(cdata->dev, "No transport done to select pinctrl\n");

	return -ENODEV;
}

static int muc_pinctrl_setup(struct muc_data *cdata, struct device *dev)
{
	int ret;

	cdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(cdata->pinctrl)) {
		dev_err(dev, "Failed to get pin ctrl\n");
		return PTR_ERR(cdata->pinctrl);
	}

	cdata->pins_discon = pinctrl_lookup_state(cdata->pinctrl,
				"disconnected");
	if (IS_ERR(cdata->pins_discon)) {
		dev_err(dev, "Failed to lookup 'disconnected' pinctrl\n");
		return PTR_ERR(cdata->pins_discon);
	}

	cdata->pins_spi_con = pinctrl_lookup_state(cdata->pinctrl,
				"spi_active");
	if (IS_ERR(cdata->pins_spi_con)) {
		dev_err(dev, "Failed to lookup 'spi_active' pinctrl\n");
		return PTR_ERR(cdata->pins_spi_con);
	}

	cdata->pins_spi_ack = pinctrl_lookup_state(cdata->pinctrl,
				"spi_ack");
	if (IS_ERR(cdata->pins_spi_ack)) {
		dev_info(dev, "Failed to lookup 'spi_ack' pinctrl\n");
		cdata->pins_spi_ack = NULL;
	}

	cdata->pins_i2c_con = pinctrl_lookup_state(cdata->pinctrl,
				"i2c_active");
	if (IS_ERR(cdata->pins_i2c_con)) {
		dev_err(dev, "Failed to lookup 'i2c_active' pinctrl\n");
		return PTR_ERR(cdata->pins_i2c_con);
	}

	/* Default to connected initially until detection is complete */
	ret = pinctrl_select_state(cdata->pinctrl, cdata->pins_spi_con);
	if (ret) {
		dev_err(dev, "Failed to select pinctrl initial state\n");
		return ret;
	}

	return 0;
}

static void muc_pinctrl_cleanup(struct muc_data *cd, struct device *dev)
{
	int rc;

	if (!gpio_get_value(cd->gpios[MUC_GPIO_DET_N]))
		dev_warn(dev, "%s MUC_GPIO_DET_N is LOW!\n", __func__);
	rc = pinctrl_select_state(cd->pinctrl, cd->pins_discon);
	if (rc)
		dev_err(dev, "%s select_state error %d\n", __func__, rc);
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
			if (muc_gpio_optional(i)) {
				cdata->gpios[i] = -ENODEV;
				continue;
			}
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
		if (gpio_is_valid(cdata->gpios[i]))
			gpio_unexport(cdata->gpios[i]);
}

static int muc_parse_seq(struct muc_data *cdata,
	struct device *dev, const char *name, u32 *seq, size_t *len)
{
	int ret;
	int i;
	int cnt = 0;
	struct property *pp = of_find_property(dev->of_node, name, &cnt);

	cnt /= sizeof(u32);
	if (!pp || cnt == 0 || cnt > *len || cnt % 3) {
		dev_err(dev, "%s:%d, error reading property %s, cnt = %d\n",
			__func__, __LINE__, name, cnt);
		return -EINVAL;
	}


	ret = of_property_read_u32_array(dev->of_node, name, seq, cnt);
	if (ret) {
		dev_err(dev, "%s:%d, unable to read %s, ret = %d\n",
			__func__, __LINE__, name, ret);
		return ret;
	}

	for (i = 0; i < cnt; i += 3) {
		int index = seq[i];

		if (index >= ARRAY_SIZE(cdata->gpios)) {
			dev_err(dev, "%s:%d, invalid gpio index: %d\n",
				__func__, __LINE__, index);
			ret = -ENODEV;
			break;
		}

		if (cdata->gpios[index] < 0) {
			dev_err(dev, "%s:%d, gpio not supported: %d\n",
				__func__, __LINE__, index);
			ret = -ENODEV;
			break;
		}
	}

	if (!ret)
		*len = cnt;

	return ret;
}

#define MSEC_TO_JIFFIES(msec) ((msec) * HZ / 1000)
int muc_gpio_init(struct device *dev, struct muc_data *cdata)
{
	int ret;

	INIT_DELAYED_WORK(&cdata->isr_work.work, attach_work);
	cdata->attach_wq = alloc_workqueue("muc_attach", WQ_UNBOUND, 1);
	if (!cdata->attach_wq) {
		dev_err(dev, "Failed to create attach workqueue\n");
		return -ENOMEM;
	}

	/* Worker lock and work for force flash / reset */
	mutex_init(&cdata->work_lock);
	INIT_DELAYED_WORK(&cdata->ff_work.work, do_muc_ff_reset);

	/* Pin Configuration */
	ret = muc_pinctrl_setup(cdata, dev);
	if (ret)
		goto free_attach_wq;

	/* Mandatory configuration */
	ret = muc_gpio_setup(cdata, dev);
	if (ret) {
		dev_err(dev, "%s:%d: failed to read gpios.\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	/* Set pinctrl to disconnected if no mod attached */
	if (gpio_get_value(cdata->gpios[MUC_GPIO_DET_N]))
		if (pinctrl_select_state(cdata->pinctrl, cdata->pins_discon))
			dev_warn(dev, "fail setting pinctrl disconnected\n");

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

#ifdef CONFIG_MODS_2ND_GEN
	cdata->select_spi_seq_len = ARRAY_SIZE(cdata->select_spi_seq);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-select-spi-seq",
		cdata->select_spi_seq, &cdata->select_spi_seq_len);
	if (ret) {
		dev_err(dev, "%s:%d failed to read muc-ctrl-select-spi-seq sequence.\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	cdata->select_i2c_seq_len = ARRAY_SIZE(cdata->select_i2c_seq);
	ret = muc_parse_seq(cdata, dev, "mmi,muc-ctrl-select-i2c-seq",
		cdata->select_i2c_seq, &cdata->select_i2c_seq_len);
	if (ret) {
		dev_err(dev, "%s:%d failed to read muc-ctrl-select-i2c-seq sequence.\n",
			__func__, __LINE__);
		goto free_attach_wq;
	}

	ret = gpio_direction_input(cdata->gpios[MUC_GPIO_SPI_MOSI]);
	if (ret)
		pr_info("%s set mosi(%d) input ret %d\n",
			__func__, cdata->gpios[MUC_GPIO_SPI_MOSI], ret);
#endif

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

	ret = of_property_read_u32(dev->of_node,
		"mmi,muc-ctrl-rm-hysteresis", &cdata->rm_hysteresis);
	if (ret)
		dev_warn(dev, "%s:%d failed to read rm hysteresis.\n",
			__func__, __LINE__);
	cdata->rm_hysteresis = MSEC_TO_JIFFIES(cdata->rm_hysteresis);

	cdata->need_det_output = of_property_read_bool(dev->of_node,
		"mmi,muc-det-pin-reconfig");

	return 0;
free_attach_wq:
	destroy_workqueue(cdata->attach_wq);

	return ret;
}

void muc_gpio_exit(struct device *dev, struct muc_data *cdata)
{
	muc_gpio_cleanup(cdata, dev);
	/* Disable the module on unload */
	muc_seq(cdata, cdata->dis_seq, cdata->dis_seq_len);
	cancel_delayed_work_sync(&cdata->isr_work.work);
	destroy_workqueue(cdata->attach_wq);
	muc_pinctrl_cleanup(cdata, dev);
}

/* The simulated reset is performed in cases where we've asked
 * the mod to reset itself and we do not have capability to detect
 * that reset on the CC pin. We send out a detach, wait for 4s,
 * then send out an attach. This is a wait-and-pray for older
 * hardware.
 */
static void do_muc_reset(struct work_struct *work)
{
	struct delayed_work *dwork;

	pr_debug("%s: start simulated reset\n", __func__);

	dwork = container_of(work, struct delayed_work, work);

	muc_misc_data->muc_detected = false;
	muc_attach_notifier_call_chain(0);

	msleep(4000);

	/* Cancel any pending interrupts that may have been queued
	 * up. We are starting from a known detached state in this
	 * workaround.
	 */
	cancel_delayed_work_sync(&muc_misc_data->isr_work.work);

	muc_handle_detection(false);

	kfree(dwork);

	pr_debug("%s: end simulated reset\n", __func__);
}

void muc_simulate_reset(void)
{
	struct delayed_work *dw;

	dw = kzalloc(sizeof(*dw), GFP_KERNEL);
	if (!dw)
		return;

	INIT_DELAYED_WORK(dw, do_muc_reset);
	queue_delayed_work(muc_misc_data->attach_wq, dw, 0);
}

#define DET_TIMEOUT_JIFFIES (HZ / 5) /* 200ms */
static void do_muc_ff_reset(struct work_struct *work)
{
	struct muc_data *cd = muc_misc_data;
	unsigned int flags;
	struct delayed_work *dwork;
	struct muc_reset_work *rw;
	int ret;
	unsigned long det_timeout;

	dwork = container_of(work, struct delayed_work, work);
	rw = container_of(dwork, struct muc_reset_work, work);

	pr_info("%s: root: %d reset: %s\n", __func__, rw->root_ver,
				rw->do_reset ? "yes" : "no");

	/* Take control of BPLUS, ignoring interrupts until done */
	cd->bplus_state = MUC_BPLUS_TRANSITIONING;

	/* In order to provide reset / force flash, need to control the CC pin,
	 * which means free/disable the IRQ, and set as an output low.
	 */
	if (!cd->det_testmode) {
		disable_irq_wake(cd->irq);
		disable_irq(cd->irq);
		devm_free_irq(cd->dev, cd->irq, cd);
	}

	/* Send force removal before resetting the muc */
	ret = muc_attach_notifier_call_chain(0);
	if (ret)
		pr_warn("%s: force notify fail: %d\n", __func__, ret);
	cd->muc_detected = false;

	/* During reset, drive CS_N asserted to ensure the mod
	 * cannot drive back ready or int preventing us from
	 * asserting force flash pin.
	 */
	pinctrl_select_state(cd->pinctrl, cd->pins_discon);

	if (cd->need_det_output)
		gpio_direction_output(cd->gpios[MUC_GPIO_DET_N], 0);

	/* Perform force flash sequence */
	if (rw->root_ver <= MUC_ROOT_V1)
		muc_seq(cd, cd->ff_seq_v1, cd->ff_seq_v1_len);
	else
		muc_seq(cd, cd->ff_seq_v2, cd->ff_seq_v2_len);


	if (cd->need_det_output)
		gpio_direction_input(cd->gpios[MUC_GPIO_DET_N]);

	/* Re-setup the IRQ */
	if (!cd->det_testmode) {
		flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT;
		ret = devm_request_threaded_irq(cd->dev, cd->irq, NULL,
					muc_isr, flags, "muc_ctrl", cd);
		if (ret) {
			pr_err("%s: Failed re-request IRQ!\n", __func__);
			BUG();
		}
		enable_irq_wake(cd->irq);
	}
	/* Reset from FF simply does the disable sequence */
	if (rw->do_reset) {
		muc_seq(cd, cd->dis_seq, cd->dis_seq_len);
		cd->bplus_state = MUC_BPLUS_DISABLED;
	} else
		cd->bplus_state = MUC_BPLUS_ENABLED;

	muc_reset_notifier_call_chain();

	/* Lets wait for the device to be re-detected */
	det_timeout = jiffies + DET_TIMEOUT_JIFFIES;
	while (gpio_get_value(cd->gpios[MUC_GPIO_DET_N]) &&
	       time_before_eq(jiffies, det_timeout))
		;

	/* If the gpio is still de-asserted, the device is gone */
	if (gpio_get_value(cd->gpios[MUC_GPIO_DET_N])) {
		pinctrl_select_state(cd->pinctrl, cd->pins_discon);
		muc_seq(cd, cd->dis_seq, cd->dis_seq_len);
		cd->bplus_state = MUC_BPLUS_DISABLED;
	} else {
		if (cd->bplus_state == MUC_BPLUS_ENABLED)
			muc_pinctrl_select_state_con(cd);
		muc_handle_detection(false);
	}
}

static void __muc_ff_queue_reset(u8 root_ver, bool reset)
{
	struct muc_data *cd = muc_misc_data;

	mutex_lock(&cd->work_lock);
	if (work_busy(&cd->ff_work.work.work) && cd->ff_work.do_reset == reset)
		goto busy;

	cd->ff_work.root_ver = root_ver;
	cd->ff_work.do_reset = reset;

	queue_delayed_work(cd->attach_wq, &cd->ff_work.work, 0);
	mutex_unlock(&cd->work_lock);

	return;

busy:
	mutex_unlock(&cd->work_lock);
	pr_warn("%s: %s already in progress; skipping\n",
		__func__, reset ? "reset" : "flashmode");
}

void muc_force_flash(u8 root_ver)
{
	__muc_ff_queue_reset(root_ver, false);
}

void muc_hard_reset(u8 root_ver)
{
	__muc_ff_queue_reset(root_ver, true);
}

static void do_muc_poweroff(struct work_struct *work)
{
	struct muc_data *cd = muc_misc_data;
	struct delayed_work *dwork;

	pr_info("%s: requested poweroff\n", __func__);

	dwork = container_of(work, struct delayed_work, work);

	muc_attach_notifier_call_chain(0);

	cd->bplus_state = MUC_BPLUS_TRANSITIONING;
	pinctrl_select_state(cd->pinctrl, cd->pins_discon);
	muc_seq(cd, cd->dis_seq, cd->dis_seq_len);
	cd->bplus_state = MUC_BPLUS_DISABLED;

	cd->muc_detected = false;

	kfree(dwork);

	/* If we've been removed, we're done */
	if (gpio_get_value(cd->gpios[MUC_GPIO_DET_N])) {
		pr_debug("%s: mod no longer present\n", __func__);
		return;
	}

	cd->pinctrl_disconnect = true;
	if (muc_pinctrl_select_state_con(cd))
		pr_warn("%s: select pinctrl failed\n", __func__);
}

void muc_poweroff(void)
{
	struct delayed_work *dw;

	dw = kzalloc(sizeof(*dw), GFP_KERNEL);
	if (!dw)
		return;

	INIT_DELAYED_WORK(dw, do_muc_poweroff);
	queue_delayed_work(muc_misc_data->attach_wq, dw, 0);
}

static void do_muc_soft_reset(struct work_struct *work)
{
	struct muc_data *cd = muc_misc_data;
	struct delayed_work *dwork;

	pr_info("%s: requested soft reset\n", __func__);

	dwork = container_of(work, struct delayed_work, work);

	muc_attach_notifier_call_chain(0);
	cd->muc_detected = false;

	pinctrl_select_state(cd->pinctrl, cd->pins_discon);
	muc_seq(cd, cd->dis_seq, cd->dis_seq_len);
	cd->bplus_state = MUC_BPLUS_DISABLED;

	muc_reset_notifier_call_chain();

	muc_handle_detection(false);

	kfree(dwork);
}

void muc_soft_reset(void)
{
	struct delayed_work *dw;

	dw = kzalloc(sizeof(*dw), GFP_KERNEL);
	if (!dw)
		return;

	INIT_DELAYED_WORK(dw, do_muc_soft_reset);
	queue_delayed_work(muc_misc_data->attach_wq, dw, 0);
}

void muc_force_detect(u32 val)
{
	struct muc_data *cd = muc_misc_data;

	if (!cd->det_testmode) {
		cd->det_testmode = true;
		disable_irq_wake(cd->irq);
		disable_irq(cd->irq);
		devm_free_irq(cd->dev, cd->irq, cd);
	}

	gpio_direction_output(cd->gpios[MUC_GPIO_DET_N], !val);

	muc_isr(cd->irq, cd);
}
