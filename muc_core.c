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

#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>

#include "muc.h"

#define MUC_NAME "muc"

struct muc_data *muc_misc_data;

static int muc_hw_init(struct muc_data *ps_muc)
{
	int err = 0;

	dev_dbg(ps_muc->dev, "%s\n", __func__);
	ps_muc->hw_initialized = 1;

	return err;
}

static void muc_device_power_off(struct muc_data *ps_muc)
{
	dev_dbg(ps_muc->dev, "%s\n", __func__);
	ps_muc->hw_initialized = 0;
}

static int muc_device_power_on(struct muc_data *ps_muc)
{
	int err = 0;

	dev_dbg(ps_muc->dev, "%s\n", __func__);
	if (!ps_muc->hw_initialized) {
		err = muc_hw_init(ps_muc);
		if (err < 0) {
			muc_device_power_off(ps_muc);
			return err;
		}
	}

	return err;
}

int muc_enable(struct muc_data *ps_muc)
{
	int err = 0;

	dev_dbg(ps_muc->dev, "%s\n", __func__);
	if (!atomic_cmpxchg(&ps_muc->enabled, 0, 1)) {
		err = muc_device_power_on(ps_muc);
		if (err < 0) {
			atomic_set(&ps_muc->enabled, 0);
			dev_err(ps_muc->dev,
				"muc_enable returned with %d\n", err);
			return err;
		}
	}

	return err;
}

static int muc_probe(struct platform_device *pdev)
{
	struct muc_data *ps_muc;
	struct device *dev = &pdev->dev;
	int err = -1;

	dev_dbg(dev, "probe begun\n");

	ps_muc = devm_kzalloc(dev, sizeof(*ps_muc), GFP_KERNEL);
	if (!ps_muc)
		return -ENOMEM;

	muc_misc_data = ps_muc;

	mutex_init(&ps_muc->lock);
	mutex_lock(&ps_muc->lock);

	err = muc_device_power_on(ps_muc);
	if (err < 0) {
		dev_err(dev, "power on failed: %d\n", err);
		goto err1;
	}

	atomic_set(&ps_muc->enabled, 1);

	err = muc_gpio_init(dev, ps_muc);
	if (err) {
		dev_err(dev, "gpio init failed\n");
		goto err_gpio_init;
	}

	err = muc_intr_setup(ps_muc, dev);
	if (err) {
		dev_err(dev, "%s:%d: failed to setup interrupt.\n",
			__func__, __LINE__);
		goto err_intr_init;
	}

	mutex_unlock(&ps_muc->lock);

	dev_info(dev, "probed finished");

	return 0;

err_intr_init:
	/* XXX gpio free */
err_gpio_init:
	muc_device_power_off(ps_muc);
err1:
	mutex_unlock(&ps_muc->lock);
	mutex_destroy(&ps_muc->lock);

	return err;
}

static int muc_remove(struct platform_device *pdev)
{
	/* XXX Free up */
	return 0;
}

static struct of_device_id muc_match_tbl[] = {
	{ .compatible = "mmi,muc" },
	{ },
};
MODULE_DEVICE_TABLE(of, muc_match_tbl);


static struct platform_driver muc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = MUC_NAME,
		.of_match_table = of_match_ptr(muc_match_tbl),
	},
	.probe = muc_probe,
	.remove = muc_remove,
};

static int __init muc_init(void)
{
	platform_driver_register(&muc_driver);
	return 0;
}

static void __exit muc_exit(void)
{
	platform_driver_unregister(&muc_driver);
}

module_init(muc_init);
module_exit(muc_exit);

MODULE_DESCRIPTION("MuC Driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
