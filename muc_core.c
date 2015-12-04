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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "muc.h"

#define MUC_NAME "muc"

struct muc_data *muc_misc_data;

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

	ps_muc->dev = dev;
	dev_info(dev, "probed finished");

	return 0;

err_intr_init:
	muc_gpio_exit(dev, ps_muc);
err_gpio_init:
	muc_misc_data = NULL;

	return err;
}

static int muc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct muc_data *ps_muc = muc_misc_data;

	muc_intr_destroy(ps_muc, dev);
	muc_gpio_exit(dev, ps_muc);

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

int __init muc_core_init(void)
{
	platform_driver_register(&muc_driver);
	return 0;
}

void __exit muc_core_exit(void)
{
	platform_driver_unregister(&muc_driver);
}
