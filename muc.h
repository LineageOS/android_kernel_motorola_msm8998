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

#ifndef __MUC_H__
#define __MUC_H__

enum {
	MUC_GPIO_DET_N    = 0,
	MUC_GPIO_BPLUS_EN = 1,
	MUC_MAX_GPIOS
};

#define MUC_MAX_SEQ (MUC_MAX_GPIOS*8)

struct muc_data {
	struct device *dev;
	u8 muc_detected;

	/* Configuration */
	int gpios[MUC_MAX_GPIOS];
	int irq;
	u32 det_hysteresis;
	u32 en_seq[MUC_MAX_SEQ];
	size_t en_seq_len;
	u32 dis_seq[MUC_MAX_SEQ];
	size_t dis_seq_len;
};

/* Global functions */
int muc_gpio_init(struct device *dev, struct muc_data *cdata);
void muc_gpio_exit(struct device *dev, struct muc_data *cdata);
int muc_intr_setup(struct muc_data *cdata, struct device *dev);
void muc_intr_destroy(struct muc_data *cdata, struct device *dev);

/* Global variables */
extern struct muc_data *muc_misc_data;

/* Driver Initializations */
int muc_spi_init(void);
void muc_spi_exit(void);

int muc_core_init(void);
void muc_core_exit(void);

int mods_uart_init(void);
void mods_uart_exit(void);

int muc_svc_init(void);
void muc_svc_exit(void);

int mods_ap_init(void);
void mods_ap_exit(void);
#endif  /* __MUC_H__ */

