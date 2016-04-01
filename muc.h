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

#ifndef __MUC_H__
#define __MUC_H__

#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/workqueue.h>

enum {
	MUC_GPIO_DET_N    = 0,
	MUC_GPIO_BPLUS_EN = 1,
	MUC_GPIO_INT_N = 2,
	MUC_GPIO_WAKE_N = 3,
	MUC_GPIO_READY_N = 4,
	MUC_GPIO_FORCE_FLASH = 5,
	MUC_GPIO_SPI_MOSI = 6,
	MUC_GPIO_SPI_MISO = 7,
	MUC_MAX_GPIOS
};

/* The force flash pin is optional depending on specific
 * hardware version.
 */
static inline bool muc_gpio_optional(int index)
{
	return index == MUC_GPIO_FORCE_FLASH;
}

#define MUC_ROOT_VER_UNKNOWN     0x00 /* For not implemented */
#define MUC_ROOT_V1              0x01
#define MUC_ROOT_V2              0x02
#define MUC_ROOT_VER_NA          0xff /* If the interface isn't master */

#define MUC_MAX_SEQ (MUC_MAX_GPIOS*8)

/* BPLUS State Transitions */
enum bplus_state {
	MUC_BPLUS_DISABLED = 0,
	MUC_BPLUS_ENABLING,
	MUC_BPLUS_ENABLED,
	MUC_BPLUS_SHORTED,
};

struct muc_attach_work {
	struct delayed_work work;
	bool force_removal;
};

struct muc_data {
	struct device *dev;
	u8 muc_detected;

	/* Attach workqueue / delayed work */
	struct workqueue_struct *attach_wq;
	struct muc_attach_work isr_work; /* Dedicated work for ISR */
	uint8_t bplus_state;

	/* Configuration */
	int gpios[MUC_MAX_GPIOS];
	int irq;
	u32 det_hysteresis;
	u32 rm_hysteresis;
	u32 en_seq[MUC_MAX_SEQ];
	size_t en_seq_len;
	u32 dis_seq[MUC_MAX_SEQ];
	size_t dis_seq_len;

	/* Force Flash Sequences */
	u32 ff_seq_v1[MUC_MAX_SEQ];
	size_t ff_seq_v1_len;
	u32 ff_seq_v2[MUC_MAX_SEQ];
	size_t ff_seq_v2_len;

	/* Pin Control */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_discon;
	struct pinctrl_state *pins_spi_con;
	struct pinctrl_state *pins_spi_ack;
	bool pinctrl_disconnect;

	bool need_det_output;
	bool spi_transport_done;
	bool i2c_transport_done;
	bool spi_shared_with_flash;
	bool det_irq_enabled;

	/* Mod short detection */
	int short_count;

	/* Platform Device Pointers */
	struct platform_device *spi_pdev;
	struct platform_device *i2c_pdev;

	u32 intr_count;
};

/* Global functions */
void muc_hard_reset(u8 root_ver);
void muc_force_flash(u8 root_ver);
void muc_poweroff(void);
int muc_gpio_init(struct device *dev, struct muc_data *cdata);
void muc_gpio_exit(struct device *dev, struct muc_data *cdata);
int muc_intr_setup(struct muc_data *cdata, struct device *dev);
void muc_intr_destroy(struct muc_data *cdata, struct device *dev);
void muc_simulate_reset(void);
void muc_soft_reset(void);
bool muc_core_probed(void);
void muc_enable_det(void);
void muc_register_spi(void);
void muc_register_spi_flash(void);
void muc_deregister_spi_flash(void);
void muc_register_i2c(void);
/* Global variables */
extern struct muc_data *muc_misc_data;

/* Driver Initializations */
int muc_spi_init(void);
void muc_spi_exit(void);

int muc_core_init(void);
void muc_core_exit(void);

int muc_svc_init(void);
void muc_svc_exit(void);

int mods_ap_init(void);
void mods_ap_exit(void);

struct dentry *mods_debugfs_get(void);

/* Indicates whether the muc's core can force flash via hardware */
static inline bool muc_can_force_flash(u8 root_ver)
{
	return root_ver >= MUC_ROOT_V2 && root_ver != MUC_ROOT_VER_NA;
}

/* Indicates whether the muc's core can propagate a reset back to the AP */
static inline bool muc_can_detect_reset(u8 root_ver)
{
	return root_ver >= MUC_ROOT_V2 && root_ver != MUC_ROOT_VER_NA;
}

/* The caller must ensure software control to enter flash modes are
 * performed prior to calling this when the hardware does not support
 * hardware force-flashing.
 */
static inline void muc_reset(u8 root_ver, bool force_flash)
{
	/* If the device can be reset via hardware, do that */
	if (!force_flash)
		muc_hard_reset(root_ver);
	else if (force_flash && muc_can_force_flash(root_ver))
		muc_force_flash(root_ver);

	/* If the device can't detect a reset, simulate one */
	if (!muc_can_detect_reset(root_ver))
		muc_simulate_reset();
}

/* Shared GPIO APIs */
static inline int muc_gpio_get_wake_n(void)
{
	return gpio_get_value(muc_misc_data->gpios[MUC_GPIO_WAKE_N]);
}

static inline void muc_gpio_set_wake_n(int value)
{
	gpio_set_value(muc_misc_data->gpios[MUC_GPIO_WAKE_N], value);
}

static inline int muc_gpio_get_ready_n(void)
{
	return gpio_get_value(muc_misc_data->gpios[MUC_GPIO_READY_N]);
}

static inline int muc_gpio_get_int_n(void)
{
	return gpio_get_value(muc_misc_data->gpios[MUC_GPIO_INT_N]);
}

/* ACKing APIs */
int muc_gpio_ack_cfg(bool en);

static inline bool muc_gpio_ack_is_supported(void)
{
	return (muc_misc_data->pins_spi_ack != NULL);
}

static inline int muc_gpio_get_ack(void)
{
	return gpio_get_value(muc_misc_data->gpios[MUC_GPIO_SPI_MISO]);
}

static inline void muc_gpio_set_ack(int value)
{
	gpio_set_value(muc_misc_data->gpios[MUC_GPIO_SPI_MOSI], value);
}
#endif  /* __MUC_H__ */
