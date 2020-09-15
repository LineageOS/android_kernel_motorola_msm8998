/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

/*
*
*  ets_fps.c
*  Date: 2016/03/16
*  Version: 0.9.0.1
*  Revise Date:  2017/12/05
*  Copyright (C) 2007-2018 Egis Technology Inc.
*
*/


#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/list.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <soc/qcom/scm.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#else
#include <linux/pm_wakeup.h>
#endif

#include "ets_fps.h"
#include "ets_navi_input.h"

#define EDGE_TRIGGER_FALLING    0x0
#define	EDGE_TRIGGER_RAISING    0x1
#define	LEVEL_TRIGGER_LOW       0x2
#define	LEVEL_TRIGGER_HIGH      0x3
#define EGIS_NAVI_INPUT 1  /* 1:open ; 0:close */
#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock ets_wake_lock;
#else
static struct wakeup_source ets_wake_lock;
#endif
/*
 * FPS interrupt table
 */

struct interrupt_desc fps_ints = {0, 0, "BUT0", 0};

static unsigned int bufsiz = 4096;

static int gpio_irq;
static int request_irq_done;
/* int t_mode = 255; */

#define EDGE_TRIGGER_FALLING    0x0
#define EDGE_TRIGGER_RISING    0x1
#define LEVEL_TRIGGER_LOW       0x2
#define LEVEL_TRIGGER_HIGH      0x3


struct ioctl_cmd {
int int_mode;
int detect_period;
int detect_threshold;
};
/* To be compatible with fpc driver */
#ifndef CONFIG_SENSORS_FPC_1020
static struct FPS_data {
	unsigned int enabled;
	unsigned int state;
	struct blocking_notifier_head nhead;
} *fpsData;

struct FPS_data *FPS_init(void)
{
	struct FPS_data *mdata;
	if (!fpsData) {
		mdata = kzalloc(
				sizeof(struct FPS_data), GFP_KERNEL);
		if (mdata) {
			BLOCKING_INIT_NOTIFIER_HEAD(&mdata->nhead);
			pr_debug("%s: FPS notifier data structure init-ed\n", __func__);
		}
		fpsData = mdata;
	}
	return fpsData;
}
int FPS_register_notifier(struct notifier_block *nb,
	unsigned long stype, bool report)
{
	int error;
	struct FPS_data *mdata = fpsData;

	mdata = FPS_init();
	if (!mdata)
		return -ENODEV;
	mdata->enabled = (unsigned int)stype;
	pr_debug("%s: FPS sensor %lu notifier enabled\n", __func__, stype);

	error = blocking_notifier_chain_register(&mdata->nhead, nb);
	if (!error && report) {
		int state = mdata->state;
		/* send current FPS state on register request */
		blocking_notifier_call_chain(&mdata->nhead,
				stype, (void *)&state);
		pr_debug("%s: FPS reported state %d\n", __func__, state);
	}
	return error;
}
EXPORT_SYMBOL_GPL(FPS_register_notifier);

int FPS_unregister_notifier(struct notifier_block *nb,
		unsigned long stype)
{
	int error;
	struct FPS_data *mdata = fpsData;

	if (!mdata)
		return -ENODEV;

	error = blocking_notifier_chain_unregister(&mdata->nhead, nb);
	pr_debug("%s: FPS sensor %lu notifier unregister\n", __func__, stype);

	if (!mdata->nhead.head) {
		mdata->enabled = 0;
		pr_debug("%s: FPS sensor %lu no clients\n", __func__, stype);
	}

	return error;
}
EXPORT_SYMBOL_GPL(FPS_unregister_notifier);

void FPS_notify(unsigned long stype, int state)
{
	struct FPS_data *mdata = fpsData;

	pr_debug("%s: Enter", __func__);

	if (!mdata) {
		pr_err("%s: FPS notifier not initialized yet\n", __func__);
		return;
	}

	pr_debug("%s: FPS current state %d -> (0x%x)\n", __func__,
		mdata->state, state);

	if (mdata->enabled && mdata->state != state) {
		mdata->state = state;
		blocking_notifier_call_chain(&mdata->nhead,
						stype, (void *)&state);
		pr_debug("%s: FPS notification sent\n", __func__);
	} else if (!mdata->enabled) {
		pr_err("%s: !mdata->enabled", __func__);
	} else {
		pr_err("%s: mdata->state==state", __func__);
	}
}
#endif

DECLARE_BITMAP(minors, N_SPI_MINORS);
LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* ------------------------------ Interrupt -----------------------------*/
/*
 * Interrupt description
 */

#define FP_INT_DETECTION_PERIOD  10
#define FP_DETECTION_THRESHOLD	10
/* struct interrupt_desc fps_ints; */
static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);
/*
 *	FUNCTION NAME.
 *		interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

void interrupt_timer_routine(unsigned long _data)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;

	DEBUG_PRINT("FPS interrupt count = %d", bdata->int_count);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		DEBUG_PRINT("FPS triggered !!!!!!!\n");
	} else {
		DEBUG_PRINT("FPS not triggered !!!!!!!\n");
	}

	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}

#ifdef CONFIG_DISPLAY_SPEED_UP
extern void ext_dsi_display_early_power_on(void);
static bool is_auth_ready = false;
#endif

static irqreturn_t fp_eint_func(int irq, void *dev_id)
{

	fps_ints.finger_on = 1;
	wake_up_interruptible(&interrupt_waitq);
	DEBUG_PRINT("etspi: %s int trigger\n", __func__);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&ets_wake_lock, msecs_to_jiffies(1500));
#else
	__pm_wakeup_event(&ets_wake_lock, 1500);
#endif
#ifdef CONFIG_DISPLAY_SPEED_UP
	if (is_auth_ready) {
		pr_info("etspi: call ext_dsi_display_early_power_on()");
		ext_dsi_display_early_power_on();
	} else
		pr_info("etspi: not in authentication mode");
#endif
	return IRQ_HANDLED;
}

static irqreturn_t fp_eint_func_ll(int irq, void *dev_id)
{
	pr_debug("etspi: fp_eint_func_ll\n");
	fps_ints.finger_on = 1;
	/* fps_ints.int_count = 0; */
	disable_irq_nosync(gpio_irq);
	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	wake_up_interruptible(&interrupt_waitq);
	/* printk_ratelimited(KERN_WARNING "-----------   zq fp fp_eint_func  ,fps_ints.int_count=%d",fps_ints.int_count);*/
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&ets_wake_lock, msecs_to_jiffies(1500));
#else
	__pm_wakeup_event(&ets_wake_lock, 1500);
#endif
	return IRQ_RETVAL(IRQ_HANDLED);
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		int_mode - determine trigger mode
 *			EDGE_TRIGGER_FALLING    0x0
 *			EDGE_TRIGGER_RAISING    0x1
 *			LEVEL_TRIGGER_LOW        0x2
 *			LEVEL_TRIGGER_HIGH       0x3
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Init(struct etspi_data *etspi, int int_mode, int detect_period, int detect_threshold)
{

	int err = 0;
	int status = 0;

	fps_ints.detect_period = detect_period;
	fps_ints.detect_threshold = detect_threshold;
	fps_ints.int_count = 0;
	fps_ints.finger_on = 0;


	if (request_irq_done == 0)	{
		gpio_irq = gpio_to_irq(etspi->irqPin);
		if (gpio_irq < 0) {
			DEBUG_PRINT("etspi: %s gpio_to_irq failed\n", __func__);
			status = gpio_irq;
			goto done;
		}

		DEBUG_PRINT("etspi:Interrupt_Init flag current: %d disable:\
			%d enable: %d\n",
		fps_ints.drdy_irq_flag, DRDY_IRQ_DISABLE, DRDY_IRQ_ENABLE);
		/* t_mode = int_mode; */
		if (int_mode == EDGE_TRIGGER_RISING) {
			DEBUG_PRINT("etspi:%s EDGE_TRIGGER_RISING\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func, IRQ_TYPE_EDGE_RISING,\
				"fp_detect-eint", etspi);
			if (err) {
				pr_err("etspi:request_irq failed==========%s,%d\n", __func__, __LINE__);
			}
		} else if (int_mode == EDGE_TRIGGER_FALLING) {
			DEBUG_PRINT("etspi:%s EDGE_TRIGGER_FALLING\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func, IRQ_TYPE_EDGE_FALLING,\
				"fp_detect-eint", etspi);
			if (err) {
				pr_err("etspi:request_irq failed==========%s,%d\n",\
					__func__, __LINE__);
			}
		} else if (int_mode == LEVEL_TRIGGER_LOW) {
			DEBUG_PRINT("etspi:%s LEVEL_TRIGGER_LOW\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func_ll, IRQ_TYPE_LEVEL_LOW,\
				"fp_detect-eint", etspi);
			if (err) {
				pr_err("etspi:request_irq failed==========%s,%d\n", __func__, __LINE__);
			}
		} else if (int_mode == LEVEL_TRIGGER_HIGH) {
			DEBUG_PRINT("etspi:%s LEVEL_TRIGGER_HIGH\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func_ll, IRQ_TYPE_LEVEL_HIGH,\
				"fp_detect-eint", etspi);
			if (err) {
				pr_err("etspi:request_irq failed==========%s,%d\n", __func__, __LINE__);
			}
		}
		DEBUG_PRINT("etspi:Interrupt_Init:gpio_to_irq return: %d\n", gpio_irq);
		DEBUG_PRINT("etspi:Interrupt_Init:request_irq return: %d\n", err);
		/* disable_irq_nosync(gpio_irq); */
		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		request_irq_done = 1;
	}


	if (fps_ints.drdy_irq_flag == DRDY_IRQ_DISABLE) {
		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		enable_irq(gpio_irq);
		DEBUG_PRINT("etspi: Interrupt_Init: %s irq/done:%d %d mode:%d\
			period:%d \threshold:%d \n", __func__, gpio_irq, request_irq_done,\
			int_mode, detect_period, detect_threshold);
	}
done:
	return 0;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Free(struct etspi_data *etspi)
{
	DEBUG_PRINT("etspi: %s\n", __func__);
	fps_ints.finger_on = 0;

	if (fps_ints.drdy_irq_flag == DRDY_IRQ_ENABLE) {
		DEBUG_PRINT("etspi: %s (DISABLE IRQ)\n", __func__);
		disable_irq_nosync(gpio_irq);
		/* disable_irq(gpio_irq); */
		del_timer_sync(&fps_ints.timer);
		fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	}
	return 0;
}




/*
 *	FUNCTION NAME.
 *		fps_interrupt_re d
 *
 *	FUNCTIONAL DESCRIPTION.
 *		FPS interrupt read status
 *
 *	ENTRY PARAMETERS.
 *		wait poll table structure
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

unsigned int fps_interrupt_poll(
struct file *file,
struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	if (fps_ints.finger_on) {
		mask |= POLLIN | POLLRDNORM;
	} else {
		poll_wait(file, &interrupt_waitq, wait);
		if (fps_ints.finger_on) {
			mask |= POLLIN | POLLRDNORM;
		}
	}
	return mask;
}

void fps_interrupt_abort(void)
{
	DEBUG_PRINT("etspi:%s\n", __func__);
	fps_ints.finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*-------------------------------------------------------------------------*/

static void etspi_reset(struct etspi_data *etspi)
{
	DEBUG_PRINT("etspi:%s\n", __func__);
	gpio_set_value(etspi->rstPin, 0);
	msleep(30);
	gpio_set_value(etspi->rstPin, 1);
	msleep(20);
}

static ssize_t etspi_read(struct file *filp,
	char __user *buf,
	size_t count,
	loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t etspi_write(struct file *filp,
	const char __user *buf,
	size_t count,
	loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}
static ssize_t etspi_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state = (*buf == '1') ? 1 : 0;
	FPS_notify(0xbeef, state);
	DEBUG_PRINT("%s  state = %d\n", __func__, state);
	return 1;
}
static DEVICE_ATTR(etspi_enable, S_IWUSR | S_IWGRP, NULL, etspi_enable_set);
static struct attribute *attributes[] = {
	&dev_attr_etspi_enable.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};
static long etspi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	int retval = 0;
	struct etspi_data *etspi;
	struct ioctl_cmd data;

	memset(&data, 0, sizeof(data));

	pr_debug("etspi: %s, cmd=%d \n", __func__, cmd);

	etspi = filp->private_data;

	switch (cmd) {
	case INT_TRIGGER_INIT:
		DEBUG_PRINT("etspi:fp_ioctl >>> fp Trigger function init\n");
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			retval = -EFAULT;
			pr_err("etspi:INT_TRIGGER_INIT failed,%s,%d\n", __func__, __LINE__);
			goto done;
		}
		retval = Interrupt_Init(etspi, data.int_mode, data.detect_period, data.detect_threshold);
		DEBUG_PRINT("etspi:fp_ioctl trigger init = %x\n", retval);
	break;

	case FP_SENSOR_RESET:
		DEBUG_PRINT("etspi:fp_ioctl ioc->opcode == FP_SENSOR_RESET --");
		etspi_reset(etspi);
		goto done;
	case INT_TRIGGER_CLOSE:
		pr_info("etspi: skip interrupt close request from HAL.");
		goto done;
	case INT_TRIGGER_ABORT:
		DEBUG_PRINT("etspi:fp_ioctl <<< fp Trigger function abort\n");
		fps_interrupt_abort();
		goto done;
#ifdef CONFIG_DISPLAY_SPEED_UP
	case SET_AUTH_STATUS:
		pr_info("etspi: - set auth status: %lu", arg);
		if (arg) is_auth_ready = true;
		else is_auth_ready = false;
		goto done;
#endif
	default:
	retval = -ENOTTY;
	break;
	}
done:
	return retval;
}

#ifdef CONFIG_COMPAT
static long etspi_compat_ioctl(struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return etspi_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define etspi_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int etspi_open(struct inode *inode, struct file *filp)
{
	struct etspi_data *etspi;
	int			status = -ENXIO;

	DEBUG_PRINT("%s\n", __func__);

	mutex_lock(&device_list_lock);

	list_for_each_entry(etspi, &device_list, device_entry)	{
		if (etspi->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (etspi->buffer == NULL) {
			etspi->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (etspi->buffer == NULL) {
				/* dev_dbg(&etspi->spi->dev, "open/ENOMEM\n"); */
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			etspi->users++;
			filp->private_data = etspi;
			nonseekable_open(inode, filp);
		}
	} else {
		pr_info("%s nothing for minor %d\n"
			, __func__, iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	return status;
}

static int etspi_release(struct inode *inode, struct file *filp)
{
	struct etspi_data *etspi;

	DEBUG_PRINT("%s\n", __func__);

	mutex_lock(&device_list_lock);
	etspi = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	etspi->users--;
	if (etspi->users == 0) {
		int	dofree;

		kfree(etspi->buffer);
		etspi->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&etspi->spi_lock);
		dofree = (etspi->spi == NULL);
		spin_unlock_irq(&etspi->spi_lock);

		if (dofree)
			kfree(etspi);
	}
	mutex_unlock(&device_list_lock);
	return 0;

}



int etspi_platformInit(struct etspi_data *etspi, bool init)
{
	int status = 0;
	DEBUG_PRINT("%s\n", __func__);

	if (!etspi) {
		pr_err("%s etspi_data is empty.\n", __func__);
		return -EBUSY;
	}

	if (init) {
		if (gpio_is_valid(etspi->vdd_18v_Pin)) {
			/* initial 18V power pin */
			status = gpio_request(etspi->vdd_18v_Pin, "18v-gpio");
			if (status < 0) {
				pr_err("%s gpio_requset vdd_18v_Pin failed\n",
					__func__);
				goto etspi_platformInit_18v_failed;
			}
			gpio_direction_output(etspi->vdd_18v_Pin, 1);
			if (status < 0) {
				pr_err("%s gpio_direction_output vdd_18v_Pin failed\n",
						__func__);
				status = -EBUSY;
				goto etspi_platformInit_18v_set_failed;
			}

			gpio_set_value(etspi->vdd_18v_Pin, 1);
			pr_info("etspi:  vdd_18v_Pin set to high\n");
			mdelay(1);
		}
		if (gpio_is_valid(etspi->vcc_33v_Pin)) {
			/* initial 33V power pin */
			status = gpio_request(etspi->vcc_33v_Pin, "33v-gpio");
			if (status < 0) {
				pr_err("%s gpio_requset vcc_33v_Pin failed\n",
					__func__);
				goto etspi_platformInit_33v_failed;
			}
			gpio_direction_output(etspi->vcc_33v_Pin, 1);
			if (status < 0) {
				pr_err("%s gpio_direction_output vcc_33v_Pin failed\n",
						__func__);
				status = -EBUSY;
				goto etspi_platformInit_33v_set_failed;
			}
			gpio_set_value(etspi->vcc_33v_Pin, 1);
			pr_info("etspi:  vcc_33v_Pin set to high\n");
			mdelay(2);
		}
		/* Initial Reset Pin*/
		status = gpio_request(etspi->rstPin, "reset-gpio");
		if (status < 0) {
			pr_err("%s gpio_requset etspi_Reset failed\n",
				__func__);
			goto etspi_platformInit_rst_failed;
		}
		gpio_direction_output(etspi->rstPin, 1);
		if (status < 0) {
			pr_err("%s gpio_direction_output Reset failed\n",
					__func__);
			status = -EBUSY;
			goto etspi_platformInit_rst_set_failed;
		}
		/* gpio_set_value(etspi->rstPin, 1); */
		pr_info("etspi:  reset to high\n");

		/* Initial IRQ Pin*/
		status = gpio_request(etspi->irqPin, "irq-gpio");
		if (status < 0) {
			pr_err("%s gpio_request etspi_irq failed\n",
				__func__);
			goto etspi_platformInit_irq_failed;
		}
		status = gpio_direction_input(etspi->irqPin);
		if (status < 0) {
			pr_err("%s gpio_direction_input IRQ failed\n",
				__func__);
			goto etspi_platformInit_set_irq_failed;
		}

		DEBUG_PRINT("ets320: %s successful status=%d\n", __func__, status);
		return 0;
	}

etspi_platformInit_set_irq_failed:
	gpio_free(etspi->irqPin);
etspi_platformInit_irq_failed:
etspi_platformInit_rst_set_failed:
	gpio_free(etspi->rstPin);
etspi_platformInit_rst_failed:
etspi_platformInit_33v_set_failed:
	if (gpio_is_valid(etspi->vcc_33v_Pin))
		gpio_free(etspi->vcc_33v_Pin);
etspi_platformInit_33v_failed:
etspi_platformInit_18v_set_failed:
	if (gpio_is_valid(etspi->vdd_18v_Pin))
		gpio_free(etspi->vdd_18v_Pin);
etspi_platformInit_18v_failed:
	return status;
}

static int etspi_parse_dt(struct device *dev,
	struct etspi_data *data)
{
	struct device_node *np = dev->of_node;
	int errorno = 0;
	int gpio;
	gpio = of_get_named_gpio(np, "egistec,gpio_rst", 0);
	if (gpio < 0) {
		errorno = gpio;
		goto dt_exit;
	} else {
		data->rstPin = gpio;
		DEBUG_PRINT("%s: sleepPin=%d\n", __func__, data->rstPin);
	}
	gpio = of_get_named_gpio(np, "egistec,gpio_irq", 0);
	if (gpio < 0) {
		errorno = gpio;
		goto dt_exit;
	} else {
		data->irqPin = gpio;
		DEBUG_PRINT("%s: drdyPin=%d\n", __func__, data->irqPin);
	}

	gpio = of_get_named_gpio(np, "egistec,gpio_ldo3p3_en", 0);
	if (gpio < 0) {
		data->vcc_33v_Pin = ARCH_NR_GPIOS;
		pr_warn("%s: 3.3v power pin is not used\n", __func__);
	} else {
		data->vcc_33v_Pin = gpio;
		pr_info("%s: 3.3v power pin=%d\n", __func__, data->vcc_33v_Pin);
	}
	gpio = of_get_named_gpio(np, "egistec,gpio_ldo1p8_en", 0);
	if (gpio < 0) {
		data->vdd_18v_Pin = ARCH_NR_GPIOS;
		pr_warn("%s: 1.8v power pin is not used\n", __func__);
	} else {
		data->vdd_18v_Pin = gpio;
		pr_info("%s: 18v power pin=%d\n", __func__, data->vdd_18v_Pin);
	}
	DEBUG_PRINT("%s is successful\n", __func__);
	return errorno;
dt_exit:
	pr_err("%s is failed\n", __func__);
	return errorno;
}

static const struct file_operations etspi_fops = {
	.owner = THIS_MODULE,
	.write = etspi_write,
	.read = etspi_read,
	.unlocked_ioctl = etspi_ioctl,
	.compat_ioctl = etspi_compat_ioctl,
	.open = etspi_open,
	.release = etspi_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll
};

/*-------------------------------------------------------------------------*/
static int etspi_probe(struct platform_device *pdev);
static int etspi_remove(struct platform_device *pdev);

static struct of_device_id etspi_match_table[] = {
	{ .compatible = "egistec,et516",},
	{},
};
MODULE_DEVICE_TABLE(of, etspi_match_table);

/* fp_id is used only when dual sensor need to be support  */
#if 0
static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp_id,fp_id",},
	{},
};
MODULE_DEVICE_TABLE(of, fp_id_match_table);
#endif



static struct platform_driver etspi_driver = {
	.driver = {
		.name		= "et320",
		.owner		= THIS_MODULE,
		.of_match_table = etspi_match_table,
	},
    .probe =    etspi_probe,
    .remove =   etspi_remove,
};
/* remark for dual sensors */
/* module_platform_driver(etspi_driver); */


/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "egis");
}
static DEVICE_ATTR_RO(vendor);

static struct attribute *class_attributes[] = {
	&dev_attr_vendor.attr,
	NULL
};

static const struct attribute_group class_attribute_group = {
	.attrs = class_attributes,
};


#define MAX_INSTANCE	5
#define MAJOR_BASE	32
static int etspi_create_sysfs(struct etspi_data *etspi, bool create) {
	struct device *dev = &etspi->spi->dev;
	static struct device *class_dev;
	static struct class *fingerprint_class;
	static dev_t dev_no;
	int rc = 0;

	if (create) {
		rc = alloc_chrdev_region(&dev_no, MAJOR_BASE, MAX_INSTANCE, "egis");
		if (rc < 0) {
			dev_err(dev, "%s alloc fingerprint class device MAJOR failed.\n", __func__);
			goto ALLOC_REGION;
		}
		if (!fingerprint_class) {
			fingerprint_class = class_create(THIS_MODULE, "fingerprint");
			if (IS_ERR(fingerprint_class)) {
				dev_err(dev, "%s create fingerprint class failed.\n", __func__);
				rc = PTR_ERR(fingerprint_class);
				fingerprint_class = NULL;
				goto CLASS_CREATE_ERR;
			}
		}
		class_dev = device_create(fingerprint_class, NULL, MAJOR(dev_no),
			etspi, etspi_driver.driver.name);
		if (IS_ERR(class_dev)) {
			dev_err(dev, "%s create fingerprint class device failed.\n", __func__);
			rc = PTR_ERR(class_dev);
			class_dev = NULL;
			goto DEVICE_CREATE_ERR;
		}
		rc = sysfs_create_group(&class_dev->kobj, &class_attribute_group);
		if (rc) {
			dev_err(dev, "could not create sysfs\n");
			goto CREATE_SYSFS_ERR;
		}
		return 0;
	}

	sysfs_remove_group(&class_dev->kobj, &class_attribute_group);
CREATE_SYSFS_ERR:
	device_destroy(fingerprint_class, MAJOR(dev_no));
	class_dev = NULL;
DEVICE_CREATE_ERR:
	class_destroy(fingerprint_class);
	fingerprint_class = NULL;
CLASS_CREATE_ERR:
	unregister_chrdev_region(dev_no, 1);
ALLOC_REGION:
	return rc;
}
static int etspi_create_device(struct etspi_data *etspi, bool create) {
	static struct class *etspi_class;
	static struct device *fdev;
	static unsigned long minor;
	struct platform_device *pdev = etspi->spi;
	int rc = 0;

	if (create) {
		rc = register_chrdev(ET320_MAJOR,
			etspi_driver.driver.name, &etspi_fops);
		if (rc < 0) {
			dev_err(&pdev->dev, "%s register_chrdev error.\n", __func__);
			return rc;
		}
		etspi_class = class_create(THIS_MODULE, etspi_driver.driver.name);
		if (IS_ERR(etspi_class)) {
			rc = PTR_ERR(etspi_class);
			dev_err(&pdev->dev, "%s class_create error.\n", __func__);
			goto CREATE_DEVICE_CLASS_CREATE_FAILED;
		}
		/*
		 * If we can allocate a minor number, hook up this device.
		 * Reusing minors is fine so long as udev or mdev is working.
		 */
		minor = find_first_zero_bit(minors, N_SPI_MINORS);
		if (minor >= N_SPI_MINORS) {
			dev_err(&pdev->dev, "no minor number available!\n");
			rc = -ENODEV;
			goto CREATE_DEVICE_CLASS_NO_MINOR;
		}
		etspi->devt = MKDEV(ET320_MAJOR, minor);
		fdev = device_create(etspi_class, &pdev->dev, etspi->devt, etspi, "esfp0");
		rc = IS_ERR(fdev) ? PTR_ERR(fdev) : 0;
		if (rc < 0) {
			dev_err(&pdev->dev, "create esfp0 failed\n");
			goto CREATE_DEVICE_CLASS_CREATE_DEVICE_FAILED;
		}
		set_bit(minor, minors);
		mutex_lock(&device_list_lock);
		list_add(&etspi->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
#if EGIS_NAVI_INPUT
		/*
		 * William Add.
		 */
		sysfs_egis_init(etspi);
		uinput_egis_init(etspi);
#endif
		return 0;
	}


#if EGIS_NAVI_INPUT
	uinput_egis_destroy(etspi);
	sysfs_egis_destroy(etspi);
#endif
	mutex_lock(&device_list_lock);
	list_del(&etspi->device_entry);
	mutex_unlock(&device_list_lock);
	clear_bit(minor, minors);
	device_destroy(etspi_class, etspi->devt);
CREATE_DEVICE_CLASS_CREATE_DEVICE_FAILED:
CREATE_DEVICE_CLASS_NO_MINOR:
	class_destroy(etspi_class);
	etspi_class = NULL;
CREATE_DEVICE_CLASS_CREATE_FAILED:
	unregister_chrdev(ET320_MAJOR, etspi_driver.driver.name);
	return rc;
}

static int etspi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct etspi_data *etspi = dev_get_drvdata(dev);

	DEBUG_PRINT("%s(#%d)\n", __func__, __LINE__);
	etspi_create_sysfs(etspi, false);
	sysfs_remove_group(&dev->kobj, &attribute_group);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&ets_wake_lock);
#else
	wakeup_source_trash(&ets_wake_lock);
#endif
	del_timer_sync(&fps_ints.timer);
	etspi_create_device(etspi, false);
	//free_irq(gpio_irq, NULL);
	etspi_platformInit(etspi, false);
	request_irq_done = 0;
	/* t_mode = 255; */
	return 0;
}

static int etspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct etspi_data *etspi;
	int status = 0;
	/* int retval; */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	DEBUG_PRINT("%s initial\n", __func__);

	etspi = devm_kzalloc(dev, sizeof(*etspi), GFP_KERNEL);
	dev_set_drvdata(dev, etspi);
	etspi->spi = pdev;

	/* device tree call */
	if (pdev->dev.of_node) {
		status = etspi_parse_dt(&pdev->dev, etspi);
		if (status) {
			pr_err("%s - Failed to parse DT\n", __func__);
			goto etspi_probe_parse_dt_failed;
		}
	}

	/* platform init */
	status = etspi_platformInit(etspi, true);
	if (status != 0) {
		pr_err("%s platforminit failed\n", __func__);
		goto etspi_probe_platformInit_failed;
	}

	/* Initialize the driver data */
	mutex_init(&device_list_lock);
	spin_lock_init(&etspi->spi_lock);
	mutex_init(&etspi->buf_lock);
	INIT_LIST_HEAD(&etspi->device_entry);

	status = etspi_create_device(etspi, true);
	if (status < 0) {
		pr_err("%s create device failed\n", __func__);
		goto etspi_probe_create_device_failed;
	}

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;

#ifdef ETSPI_NORMAL_MODE
/*
	spi->bits_per_word = 8;
	spi->max_speed_hz = SLOW_BAUD_RATE;
	spi->mode = SPI_MODE_0;
	spi->chip_select = 0;
	status = spi_setup(spi);
	if (status != 0) {
		pr_err("%s spi_setup() is failed. status : %d\n",
			__func__, status);
		return status;
	}
*/
#endif
	etspi_reset(etspi);

	/* the timer is for ET310 */
	setup_timer(&fps_ints.timer, interrupt_timer_routine, (unsigned long)&fps_ints);
	add_timer(&fps_ints.timer);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&ets_wake_lock, WAKE_LOCK_SUSPEND, "ets_wake_lock");
#else
	wakeup_source_init(&ets_wake_lock, "ets_wake_lock");
#endif
	DEBUG_PRINT("  add_timer ---- \n");
	DEBUG_PRINT("%s : initialize success %d\n",
		__func__, status);

	status = sysfs_create_group(&dev->kobj, &attribute_group);
	if (status) {
		pr_err("%s could not create sysfs\n", __func__);
		goto etspi_create_group_failed;
	}

	status = etspi_create_sysfs(etspi, true);
	if (status) {
		pr_err("%s could not create sysfs\n", __func__);
		goto etspi_sysfs_failed;
	}

	return status;

etspi_sysfs_failed:
	sysfs_remove_group(&dev->kobj, &attribute_group);
etspi_create_group_failed:
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&ets_wake_lock);
#else
	wakeup_source_trash(&ets_wake_lock);
#endif
	del_timer_sync(&fps_ints.timer);
	etspi_create_device(etspi, false);
etspi_probe_create_device_failed:
	etspi_platformInit(etspi, false);
etspi_probe_platformInit_failed:
etspi_probe_parse_dt_failed:
	pr_err("%s is failed\n", __func__);
	return status;
}

static int __init ets_init(void)
{
	int status = 0;
	DEBUG_PRINT("%s  enter\n", __func__);
#if 0
	/* fp_id is used only when dual sensor need to be support  */
	struct device_node *fp_id_np = NULL;
	int fp_id_gpio = 0, fp_id_gpio_value;

	fp_id_np = of_find_matching_node(fp_id_np, fp_id_match_table);
	if (fp_id_np)
	    fp_id_gpio = of_get_named_gpio(fp_id_np, "fp,gpio-id", 0);

	if (fp_id_gpio < 0) {
		/* errorno = gpio; */
		DEBUG_PRINT("%s: device tree error \n", __func__);
		return status;
	} else {
		/* data->rstPin = gpio; */
		DEBUG_PRINT("%s: fp_id Pin=%d\n", __func__, fp_id_gpio);
	}
	gpio_direction_input(fp_id_gpio);
	fp_id_gpio_value = gpio_get_value(fp_id_gpio);
	if (fp_id_gpio_value == 0)
	    DEBUG_PRINT("%s:  Load et320-int driver \n", __func__);
	else {
		DEBUG_PRINT("%s:  Load fpc driver \n", __func__);
		return status;
    }
#endif
	status = platform_driver_register(&etspi_driver);
	DEBUG_PRINT("%s  done\n", __func__);

	return status;
}

static void __exit ets_exit(void)
{
	platform_driver_unregister(&etspi_driver);
}

module_init(ets_init);
module_exit(ets_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("SPI Interface for ET320");
MODULE_LICENSE("GPL");
