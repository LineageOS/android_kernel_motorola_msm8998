/*  Himax Android Driver Sample Code for HX83102 chipset

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic_core.h"
#include <linux/slab.h>

#define hx83102ab_fw_addr_sorting_mode_en		0x100007FC
#define hx83102ab_fw_addr_selftest_addr_en		0x100007F8
#define hx83102ab_data_adc_cfg_1			0x10007B00

#define hx83102d_fw_addr_raw_out_sel			0x800204f4
#define hx83102d_zf_data_adc_cfg_1 0x10007B00
#define hx83102d_zf_data_adc_cfg_2 0x10006A00
#define hx83102d_zf_data_adc_cfg_3 0x10007500
#define hx83102d_zf_data_bor_prevent_info 0x10007268
#define hx83102d_zf_data_notch_info 0x10007300
#define hx83102d_zf_func_info_en 0x10007FD0
#define hx83102d_zf_po_sub_func 0x10005A00
#define hx83102d_zf_data_sram_start_addr 0x20000000


#ifdef HX_ESD_RECOVERY
	extern u8 HX_ESD_RESET_ACTIVATE;
#endif

