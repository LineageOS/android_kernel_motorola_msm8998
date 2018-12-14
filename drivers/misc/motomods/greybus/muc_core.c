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
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "muc.h"

#define MUC_NAME "muc"

struct muc_data *muc_misc_data;

void muc_enable_det(void)
{
	if (!muc_misc_data)
		return;

	/* one-time enable detection interrupts after apba is ready */
	if (muc_misc_data->det_irq_enabled)
		return;

	if (muc_intr_setup(muc_misc_data, muc_misc_data->dev)) {
		pr_err("%s: failed to setup interrupt.\n",
			__func__);
		return;
	}

#ifdef CONFIG_MODS_2ND_GEN
	pr_info("%s: reset gpio state as disconnected one\n", __func__);
	if (pinctrl_select_state(muc_misc_data->pinctrl, muc_misc_data->pins_discon))
		pr_warn("%s: select disconnected pinctrl failed\n",
			__func__);
#endif

	muc_misc_data->det_irq_enabled = true;
}

bool muc_core_probed(void)
{
	return !!muc_misc_data;
}

static int muc_probe(struct platform_device *pdev)
{
	struct muc_data *ps_muc;
	struct device *dev = &pdev->dev;
	int err;

	dev_dbg(dev, "probe begun\n");

	ps_muc = devm_kzalloc(dev, sizeof(*ps_muc), GFP_KERNEL);
	if (!ps_muc)
		return -ENOMEM;

	err = muc_gpio_init(dev, ps_muc);
	if (err) {
		dev_err(dev, "gpio init failed\n");
		goto err_gpio_init;
	}

	ps_muc->spi_shared_with_flash = of_property_read_bool(dev->of_node,
					"mmi,spi-shared-with-flash");

	ps_muc->dev = dev;
	dev_info(dev, "probed finished");

	muc_misc_data = ps_muc;

	return 0;

err_gpio_init:
	muc_misc_data = NULL;

	dev_err(dev, "Failed to probe: %d\n", err);

	return err;
}

void muc_register_spi_flash(void)
{
	if (!muc_misc_data)
		return;

	if (muc_misc_data->spi_shared_with_flash) {
		pinctrl_select_state(muc_misc_data->pinctrl,
				     muc_misc_data->pins_spi_con);
		muc_register_spi();
	}
}

void muc_deregister_spi_flash(void)
{
	if (!muc_misc_data ||
	    gpio_get_value(muc_misc_data->gpios[MUC_GPIO_DET_N]))
		return;

	if (muc_misc_data->spi_shared_with_flash)
		pinctrl_select_state(muc_misc_data->pinctrl,
				     muc_misc_data->pins_discon);
}

void muc_register_spi(void)
{
	struct device *dev = muc_misc_data->dev;
	struct device_node *np;
	struct device_node *spi_np;
	struct platform_device *pdev;

	if (muc_misc_data->spi_transport_done)
		return;

	muc_misc_data->i2c_transport_done = false;

	of_platform_depopulate(dev);

	np = of_find_node_by_name(dev->of_node, "transports");
	if (!np) {
		dev_warn(dev, "Transport node not present\n");
		muc_misc_data->spi_transport_done = true;
		return;
	}

	spi_np = of_find_compatible_node(np, NULL, "moto,mod-spi-transfer");
	if (!spi_np) {
		dev_warn(dev, "SPI transport device not present\n");
		goto put_np;
	}

	pdev = of_platform_device_create(spi_np, NULL, dev);
	if (!pdev) {
		dev_err(dev, "Failed to populate SPI transport devices\n");
		goto put_spi_np;
	}

	of_node_set_flag(dev->of_node, OF_POPULATED_BUS);
	muc_misc_data->spi_transport_done = true;

put_spi_np:
	of_node_put(spi_np);
put_np:
	of_node_put(np);
}

void muc_register_i2c(void)
{
	struct device *dev = muc_misc_data->dev;
	struct device_node *np;
	struct device_node *i2c_np;
	struct platform_device *pdev;

	if (muc_misc_data->i2c_transport_done)
		return;

	muc_misc_data->spi_transport_done = false;
	of_platform_depopulate(dev);

	np = of_find_node_by_name(dev->of_node, "transports");
	if (!np) {
		dev_warn(dev, "Transport node not present\n");
		muc_misc_data->i2c_transport_done = true;
		return;
	}

	i2c_np = of_find_compatible_node(np, NULL, "moto,mod-i2c-transfer");
	if (!i2c_np) {
		dev_warn(dev, "I2C transport device not present\n");
		goto put_np;
	}

	pdev = of_platform_device_create(i2c_np, NULL, dev);
	if (!pdev) {
		dev_err(dev, "Failed to populate I2C transport devices\n");
		goto put_i2c_np;
	}

	of_node_set_flag(dev->of_node, OF_POPULATED_BUS);
	muc_misc_data->i2c_transport_done = true;

put_i2c_np:
	of_node_put(i2c_np);
put_np:
	of_node_put(np);
};

static int muc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct muc_data *ps_muc = muc_misc_data;

	if (ps_muc->det_irq_enabled)
		muc_intr_destroy(ps_muc, dev);
	muc_gpio_exit(dev, ps_muc);

	of_platform_depopulate(dev);
	of_node_clear_flag(dev->of_node, OF_POPULATED_BUS);

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
	return platform_driver_register(&muc_driver);
}

void muc_core_exit(void)
{
	platform_driver_unregister(&muc_driver);
}
