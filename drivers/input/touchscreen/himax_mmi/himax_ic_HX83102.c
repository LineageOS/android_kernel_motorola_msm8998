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

#include "himax_ic_HX83102.h"
extern struct himax_ts_data *private_ts;
extern struct himax_core_fp g_core_fp;
extern struct fw_operation *pfw_op;
#if defined(HX_ZERO_FLASH)
extern struct zf_operation *pzf_op;
extern struct ic_operation *pic_op;
extern int G_POWERONOF;
#endif


extern unsigned char IC_CHECKSUM;
extern bool DSRAM_Flag;

extern unsigned long	FW_VER_MAJ_FLASH_ADDR;
extern unsigned long	FW_VER_MIN_FLASH_ADDR;
extern unsigned long	CFG_VER_MAJ_FLASH_ADDR;
extern unsigned long	CFG_VER_MIN_FLASH_ADDR;
extern unsigned long	CID_VER_MAJ_FLASH_ADDR;
extern unsigned long	CID_VER_MIN_FLASH_ADDR;

extern unsigned long	FW_VER_MAJ_FLASH_LENG;
extern unsigned long	FW_VER_MIN_FLASH_LENG;
extern unsigned long	CFG_VER_MAJ_FLASH_LENG;
extern unsigned long	CFG_VER_MIN_FLASH_LENG;
extern unsigned long	CID_VER_MAJ_FLASH_LENG;
extern unsigned long	CID_VER_MIN_FLASH_LENG;

#ifdef HX_AUTO_UPDATE_FW
	extern int g_i_FW_VER;
	extern int g_i_CFG_VER;
	extern int g_i_CID_MAJ;
	extern int g_i_CID_MIN;
	extern unsigned char *i_CTPM_FW;
#endif

#ifdef HX_TP_PROC_2T2R
	extern bool Is_2T2R;
#endif

#ifdef HX_USB_DETECT_GLOBAL
	extern void himax_cable_detect_func(bool force_renew);
#endif

#ifdef HX_RST_PIN_FUNC
	extern void himax_rst_gpio_set(int pinnum, uint8_t value);
#endif

static void hx83102_chip_init(void)
{
	private_ts->chip_cell_type = CHIP_IS_IN_CELL;
	I("%s:IC cell type = %d\n", __func__, private_ts->chip_cell_type);
	IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	FW_VER_MAJ_FLASH_ADDR	= 49157;  /*0x00C005*/
	FW_VER_MAJ_FLASH_LENG	= 1;
	FW_VER_MIN_FLASH_ADDR	= 49158;  /*0x00C006*/
	FW_VER_MIN_FLASH_LENG	= 1;
	CFG_VER_MAJ_FLASH_ADDR	= 49408;  /*0x00C100*/
	CFG_VER_MAJ_FLASH_LENG	= 1;
	CFG_VER_MIN_FLASH_ADDR	= 49409;  /*0x00C101*/
	CFG_VER_MIN_FLASH_LENG	= 1;
	CID_VER_MAJ_FLASH_ADDR	= 49154;  /*0x00C002*/
	CID_VER_MAJ_FLASH_LENG	= 1;
	CID_VER_MIN_FLASH_ADDR	= 49155;  /*0x00C003*/
	CID_VER_MIN_FLASH_LENG	= 1;
	/*PANEL_VERSION_ADDR	= 49156;*/  /*0x00C004*/
	/*PANEL_VERSION_LENG	= 1;*/

}

void hx83102_burst_enable(uint8_t auto_add_4_byte)
{
	uint8_t tmp_data[4];

	tmp_data[0] = 0x31;
	if (himax_bus_write(0x13, tmp_data, 1, HIMAX_I2C_RETRY_TIMES) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}

	tmp_data[0] = (0x10 | auto_add_4_byte);
	if (himax_bus_write(0x0D, tmp_data, 1, HIMAX_I2C_RETRY_TIMES) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
}

int hx83102_flash_write_burst(uint8_t *reg_byte, uint8_t *write_data)
{
	uint8_t data_byte[8];
	int i = 0, j = 0;

	for (i = 0; i < 4; i++) {
		data_byte[i] = reg_byte[i];
	}
	for (j = 4; j < 8; j++) {
		data_byte[j] = write_data[j-4];
	}

	if (himax_bus_write(0x00, data_byte, 8, HIMAX_I2C_RETRY_TIMES) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}
	return 0;
}

static int hx83102_register_read(uint8_t *read_addr, int read_length, uint8_t *read_data)
{
	uint8_t tmp_data[4];
	int i = 0;
	int address = 0;

	if (read_length > 256) {
		E("%s: read len over 256!\n", __func__);
		return LENGTH_FAIL;
	}
	if (read_length > 4)
		hx83102_burst_enable(1);
	else
		hx83102_burst_enable(0);

	address = (read_addr[3] << 24) + (read_addr[2] << 16) + (read_addr[1] << 8) + read_addr[0];
	i = address;
	tmp_data[0] = (uint8_t)i;
	tmp_data[1] = (uint8_t)(i >> 8);
	tmp_data[2] = (uint8_t)(i >> 16);
	tmp_data[3] = (uint8_t)(i >> 24);
	if (himax_bus_write(0x00, tmp_data, 4, HIMAX_I2C_RETRY_TIMES) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}
	tmp_data[0] = 0x00;
	if (himax_bus_write(0x0C, tmp_data, 1, HIMAX_I2C_RETRY_TIMES) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	if (himax_bus_read(0x08, read_data, read_length, HIMAX_I2C_RETRY_TIMES) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}
	if (read_length > 4)
		hx83102_burst_enable(0);

	return 0;
}

static bool hx83102_sense_off(void)
{
	uint8_t cnt = 0;
	uint8_t tmp_addr[FOUR_BYTE_DATA_SZ];
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];

	do {
		/*===========================================
		 I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = 0x27;
		if (himax_bus_write(0x31, tmp_data, 1, HIMAX_I2C_RETRY_TIMES) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/*===========================================
		 I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = 0x95;
		if (himax_bus_write(0x32, tmp_data, 1, HIMAX_I2C_RETRY_TIMES) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/* ======================
		 Check enter_save_mode
		 ======================*/
		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0xA8;
		hx83102_register_read(tmp_addr, FOUR_BYTE_ADDR_SZ, tmp_data);
		I("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/*=====================================
			 Reset TCON
			=====================================*/
			tmp_addr[3] = 0x80; tmp_addr[2] = 0x02; tmp_addr[1] = 0x00; tmp_addr[0] = 0x20;
			tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00; tmp_data[0] = 0x00;
			hx83102_flash_write_burst(tmp_addr, tmp_data);
			msleep(1);
			tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00; tmp_data[0] = 0x01;
			hx83102_flash_write_burst(tmp_addr, tmp_data);
			/*=====================================
			 Reset ADC
			=====================================*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x94;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102_flash_write_burst(tmp_addr, tmp_data);
			msleep(1);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			hx83102_flash_write_burst(tmp_addr, tmp_data);

			return true;
		} else {
			msleep(10);
#ifdef HX_RST_PIN_FUNC
			himax_rst_gpio_set(private_ts->rst_gpio, 0);
			msleep(20);
			himax_rst_gpio_set(private_ts->rst_gpio, 1);
			msleep(50);
#endif
		}
	} while (cnt++ < 15);

	return false;
}

static void hx83102ab_set_SMWP_enable(uint8_t SMWP_enable, bool suspended)
{
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];
	uint8_t back_data[FOUR_BYTE_DATA_SZ];
	uint8_t retry_cnt = 0;
	g_core_fp.fp_sense_off();

	do {
		if (SMWP_enable) {
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, tmp_data, 4);
			g_core_fp.fp_flash_write_burst(pfw_op->addr_smwp_enable, tmp_data);
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, back_data, 4);
		} else {
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
			g_core_fp.fp_flash_write_burst(pfw_op->addr_smwp_enable, tmp_data);
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, back_data, 4);
		}

		g_core_fp.fp_register_read(pfw_op->addr_smwp_enable, FOUR_BYTE_DATA_SZ, tmp_data, false);
		/*I("%s: tmp_data[0]=%d, SMWP_enable=%d, retry_cnt=%d \n", __func__, tmp_data[0],SMWP_enable,retry_cnt);*/
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]  || tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);

	g_core_fp.fp_sense_on(0x00);
}

static void hx83102ab_set_HSEN_enable(uint8_t HSEN_enable, bool suspended)
{
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];

	if (HSEN_enable) {
		himax_in_parse_assign_cmd(fw_func_handshaking_pwd, tmp_data, 4);
		g_core_fp.fp_flash_write_burst(pfw_op->addr_hsen_enable, tmp_data);
	} else {
		himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
		g_core_fp.fp_flash_write_burst(pfw_op->addr_hsen_enable, tmp_data);
	}

	/*I("%s: tmp_data[0]=%d, HSEN_enable=%d, retry_cnt=%d \n", __func__, tmp_data[0],HSEN_enable,retry_cnt);*/
}

static void hx83102ab_usb_detect_set(uint8_t *cable_config)
{
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];

	if (cable_config[1] == 0x01) {
		himax_in_parse_assign_cmd(fw_func_handshaking_pwd, tmp_data, 4);
		g_core_fp.fp_flash_write_burst(pfw_op->addr_usb_detect, tmp_data);
		I("%s: USB detect status IN!\n", __func__);
	} else {
		himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
		g_core_fp.fp_flash_write_burst(pfw_op->addr_usb_detect, tmp_data);
		I("%s: USB detect status OUT!\n", __func__);
	}
}

static uint8_t hx83102ab_read_DD_status(uint8_t *cmd_set, uint8_t *tmp_data)
{
	int cnt = 0;
	uint8_t req_size = cmd_set[0];
	cmd_set[3] = pfw_op->data_dd_request[0];
	g_core_fp.fp_register_write(pfw_op->addr_dd_handshak_addr, FOUR_BYTE_DATA_SZ, cmd_set, 0);
	I("%s: cmd_set[0] = 0x%02X,cmd_set[1] = 0x%02X,cmd_set[2] = 0x%02X,cmd_set[3] = 0x%02X\n",
	  __func__, cmd_set[0], cmd_set[1], cmd_set[2], cmd_set[3]);

	/* Doing hand shaking 0xAA -> 0xBB */
	for (cnt = 0; cnt < 100; cnt++) {
		g_core_fp.fp_register_read(pfw_op->addr_dd_handshak_addr, FOUR_BYTE_DATA_SZ, tmp_data, false);
		msleep(10);

		if (tmp_data[3] == pfw_op->data_dd_ack[0]) {
			I("%s Data ready goto moving data\n", __func__);
			break;
		} else {
			if (cnt >= 99) {
				I("%s Data not ready in FW \n", __func__);
				return FW_NOT_READY;
			}
		}
	}

	g_core_fp.fp_sense_off();
	g_core_fp.fp_register_read(pfw_op->addr_dd_data_addr, req_size, tmp_data, false);
	g_core_fp.fp_sense_on(0x01);
	return NO_ERR;
}

static int hx83102ab_read_FW_status(uint8_t *state_addr, uint8_t *tmp_addr)
{
	uint8_t i;
	uint8_t req_size = 0;
	uint8_t status_addr[FOUR_BYTE_DATA_SZ]; /*0x10007F44*/
	uint8_t cmd_addr[FOUR_BYTE_DATA_SZ]; /*0x900000F8*/

	if (state_addr[0] == 0x01) {
		state_addr[1] = 0x04;

		for (i = 0; i < FOUR_BYTE_DATA_SZ; i++) {
			/* 0x10007F44 */
			state_addr[i + 2] = pfw_op->addr_fw_dbg_msg_addr[i];
			status_addr[i] = pfw_op->addr_fw_dbg_msg_addr[i];
		}

		req_size = 0x04;
		g_core_fp.fp_sense_off();
		g_core_fp.fp_register_read(status_addr, req_size, tmp_addr, false);
		g_core_fp.fp_sense_on(0x01);
	} else if (state_addr[0] == 0x02) {
		state_addr[1] = 0x30;

		for (i = 0; i < FOUR_BYTE_DATA_SZ; i++) {
			/* 0x900000F8 */
			state_addr[i + 2] = pfw_op->addr_fw_state_addr[i];
			cmd_addr[i] = pfw_op->addr_fw_state_addr[i];
		}

		req_size = 0x30;
		g_core_fp.fp_register_read(cmd_addr, req_size, tmp_addr, false);
	}

	return NO_ERR;
}

static void hx83102ab_power_on_init(void)
{
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];
	I("%s:\n", __func__);
	himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
	g_core_fp.fp_flash_write_burst(pfw_op->addr_raw_out_sel, tmp_data);
	g_core_fp.fp_flash_write_burst(pfw_op->addr_sorting_mode_en, tmp_data);
	g_core_fp.fp_touch_information();
	g_core_fp.fp_sense_on(0x00);
}

static int hx83102ab_fts_ctpm_fw_upgrade_with_sys_fs_64k(unsigned char *fw, int len, bool change_iref)
{
	int burnFW_success = 0;
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];

	I("[%s][%d]:enter\n", __func__, __LINE__);

	if (len != FW_SIZE_64k) {
		E("%s: The file size is not 64K bytes\n", __func__);
		return false;
	}

#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, false);
#else
	g_core_fp.fp_system_reset();
#endif
	g_core_fp.fp_sense_off();
	g_core_fp.fp_chip_erase();
	g_core_fp.fp_flash_programming(fw, FW_SIZE_64k);

	if (g_core_fp.fp_check_CRC(pfw_op->addr_program_reload_from, FW_SIZE_64k) == 0) {
		burnFW_success = 1;
	}

	I("[%s][%d]\n", __func__, __LINE__);

	himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
	g_core_fp.fp_flash_write_burst(pfw_op->addr_raw_out_sel, tmp_data);
	g_core_fp.fp_flash_write_burst(pfw_op->addr_sorting_mode_en, tmp_data);
	I("[%s][%d]\n", __func__, __LINE__);
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, false);
#else
	/*System reset*/
	g_core_fp.fp_system_reset();
#endif

	I("[%s][%d]\n", __func__, __LINE__);
	return burnFW_success;
}

#ifdef HX_ESD_RECOVERY
static void hx83102ab_esd_ic_reset(void)
{
	HX_ESD_RESET_ACTIVATE = 1;
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_pin_reset();
#endif
	I("%s:\n", __func__);
}
#endif

#if defined(HX_ZERO_FLASH)
static void himax_hx83102d_reload_to_active(void)
{
	uint8_t addr[FOUR_BYTE_DATA_SZ] = {0};
	uint8_t data[FOUR_BYTE_DATA_SZ] = {0};
	uint8_t retry_cnt = 0;

	addr[3] = 0x90;
	addr[2] = 0x00;
	addr[1] = 0x00;
	addr[0] = 0x48;

	do {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x00;
		data[0] = 0xEC;
		g_core_fp.fp_flash_write_burst(addr, data);

		g_core_fp.fp_register_read(addr, FOUR_BYTE_DATA_SZ, data, 0);
		I("%s: data[1]=%d, data[0]=%d, retry_cnt=%d \n", __func__, data[1], data[0], retry_cnt);
		retry_cnt++;
	} while ((data[1] != 0x01 || data[0] != 0xEC) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static void himax_hx83102d_resume_ic_action(void)
{
#ifndef HX_RESUME_HW_RESET
	himax_hx83102d_reload_to_active();
#endif
}

static void himax_hx83102d_suspend_ic_action(void)
{
#ifndef HX_RESUME_HW_RESET
	himax_hx83102d_reload_to_active();
#endif
}

static void himax_hx83102d_sense_on(uint8_t FlashMode)
{
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];
	int retry = 0;
	I("Enter %s \n", __func__);
	g_core_fp.fp_interface_on();
	g_core_fp.fp_register_write(pfw_op->addr_ctrl_fw_isr,
		sizeof(pfw_op->data_clear), pfw_op->data_clear, false);
	msleep(20);

	if (!FlashMode) {
#ifdef HX_RST_PIN_FUNC
		g_core_fp.fp_ic_reset(false, false);
#else
		g_core_fp.fp_system_reset();
#endif
		himax_hx83102d_reload_to_active();
	} else {
		himax_hx83102d_reload_to_active();
		do {
			g_core_fp.fp_register_write(pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_active), pfw_op->data_safe_mode_release_pw_active, false);

			g_core_fp.fp_register_read(pfw_op->addr_flag_reset_event, FOUR_BYTE_DATA_SZ, tmp_data, 0);
			I("%s:Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01 || tmp_data[0] != 0x00) && retry++ < 5);

		if (retry >= 5) {
			E("%s: Fail:\n", __func__);
#ifdef HX_RST_PIN_FUNC
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
			himax_hx83102d_reload_to_active();
		} else {
			I("%s:OK and Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
			/* reset code*/
			tmp_data[0] = 0x00;

			if (himax_bus_write(pic_op->adr_i2c_psw_lb[0], tmp_data, 1, HIMAX_I2C_RETRY_TIMES) < 0) {
				E("%s: i2c access fail!\n", __func__);
			}

			if (himax_bus_write(pic_op->adr_i2c_psw_ub[0], tmp_data, 1, HIMAX_I2C_RETRY_TIMES) < 0) {
				E("%s: i2c access fail!\n", __func__);
			}

			g_core_fp.fp_register_write(pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_reset), pfw_op->data_safe_mode_release_pw_reset, false);
		}
	}
}

static void hx83102d_firmware_update_0f(const struct firmware *fw_entry)
{
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ] = {0};
	int retry = 0;
	int crc = -1;

	I("%s, Entering \n", __func__);

	g_core_fp.fp_register_write(pzf_op->addr_system_reset,  4,  pzf_op->data_system_reset,  false);

	g_core_fp.fp_sense_off();

	/* first 40K - isram */
	do {
		g_core_fp.fp_write_sram_0f (fw_entry, pzf_op->data_sram_start_addr, 0, HX_40K_SZ);
		crc = g_core_fp.fp_check_CRC (pzf_op->data_sram_start_addr,  HX_40K_SZ);
		if (crc == 0) {
			I("%s, HW CRC OK in %d time \n", __func__, retry);
			break;
		} else {
			E("%s, HW CRC FAIL in %d time !\n", __func__, retry);
		}
		retry++;
	} while (crc != 0 && retry < 3);

	if (crc != 0) {
		E("Last time CRC Fail!\n");
		return;
	}

	/* clean
	if (G_POWERONOF == 1) {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_sram_clean, HX_32K_SZ, 0);
	} */

	/*last 16k*/
	/*config info*/
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, pzf_op->data_cfg_info, 0xC000, 132);
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_cfg_info, 132, 2);
	}
	/*FW config*/
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, pzf_op->data_fw_cfg_1, 0xC0FE, 484);
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_fw_cfg_1, 484, 1);
	}
	/*ADC config*/
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, pzf_op->data_adc_cfg_1, 0xD000, 768);
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_adc_cfg_1, 768, 2);
	}

	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, pzf_op->data_adc_cfg_2, 0xD300, 1536);
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_adc_cfg_2, 1536, 2);
	}

	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, pzf_op->data_adc_cfg_3, 0xE000, 1536);
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_adc_cfg_3, 1536, 2);
	}

	/*border prevent info*/
	himax_in_parse_assign_cmd(hx83102d_zf_data_bor_prevent_info, tmp_data, 4);
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, tmp_data, 0xC9E0, 32);
	} else {
		g_core_fp.fp_clean_sram_0f(tmp_data, 32, 2);
	}
	/*notch info*/
	himax_in_parse_assign_cmd(hx83102d_zf_data_notch_info, tmp_data, 4);
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, tmp_data, 0xCA00, 128);
	} else {
		g_core_fp.fp_clean_sram_0f(tmp_data, 128, 2);
	}
	/*enable func info*/
	himax_in_parse_assign_cmd(hx83102d_zf_func_info_en, tmp_data, 4);
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, tmp_data, 0xCB00, 12);
	} else {
		g_core_fp.fp_clean_sram_0f(tmp_data, 12, 2);
	}
	/*power on sub func*/
	himax_in_parse_assign_cmd(hx83102d_zf_po_sub_func, tmp_data, 4);
	if (G_POWERONOF == 1) {
		retry = 0;
		do {
			g_core_fp.fp_write_sram_0f(fw_entry, tmp_data, 0xA000, HX4K);
			crc = g_core_fp.fp_check_CRC (tmp_data, HX4K);
			if (crc == 0) {
				I("%s, power on sub func CRC OK in %d time \n", __func__, retry);
				break;
			} else {
				E("%s, HW CRC FAIL in %d time !\n", __func__, retry);
				I("%s, data[3] = %2X,data[2] = %2X,data[1] = %2X,data[0] = %2X !\n", __func__,
				tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
			}
			retry++;
		} while (crc != 0 && retry < 3);
	} else {
		g_core_fp.fp_clean_sram_0f(tmp_data, 4096, 2);
	}

	I("%s, END \n", __func__);
}

static void hx83102ab_firmware_update_0f(const struct firmware *fw_entry)
{
	uint8_t tmp_addr[4];

	I("%s, Entering \n", __func__);

	g_core_fp.fp_register_write(pzf_op->addr_system_reset,  4,  pzf_op->data_system_reset,  false);

	g_core_fp.fp_sense_off();

	/* first 32K */
	/*bin file start*/
	/*isram*/
	g_core_fp.fp_write_sram_0f (fw_entry, pzf_op->data_sram_start_addr, 0, HX_32K_SZ);

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x73;
	tmp_addr[0] = 0x00;
	g_core_fp.fp_write_sram_0f (fw_entry, tmp_addr, 0x8000, (0x200*4));

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x68;
	tmp_addr[0] = 0x00;
	g_core_fp.fp_write_sram_0f (fw_entry, tmp_addr, 0x8800, (0x200*4));


	/*last 16k*/
	/*config info*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72;
	tmp_addr[0] = 0x04; /*0x10007206 <= size 60,from 0x0000C180*/
	g_core_fp.fp_write_sram_0f (fw_entry, tmp_addr, 0xC178, (84));




	/*FW config*/
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, pzf_op->data_fw_cfg_1, 0xBFFE, 388);
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_fw_cfg_1, 388, 1);
	}
	/*ADC config*/
	if (G_POWERONOF == 1) {
		g_core_fp.fp_write_sram_0f(fw_entry, pzf_op->data_adc_cfg_1, 0xe000, (128*4));
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_adc_cfg_1, (128*4), 2);
	}

	I("%s, END \n", __func__);
}
#if defined(HX_0F_DEBUG)
static void hx83102ab_firmware_read_0f(const struct firmware *fw_entry, int type)
{
	uint8_t tmp_addr[4];

	I("%s, Entering \n", __func__);
	if (type == 0) { /* first 32K */
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_sram_start_addr, 0, HX_32K_SZ);
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x68;
		tmp_addr[0] = 0x00;
		g_core_fp.fp_read_sram_0f(fw_entry, tmp_addr, 0x8000, (0x200*4));

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x73;
		tmp_addr[0] = 0x00;
		g_core_fp.fp_read_sram_0f(fw_entry, tmp_addr, 0x8800, (0x200*4));

	} else { /*last 16k*/
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x72;
		tmp_addr[0] = 0x04; /*0x10007206 <= size 80,from 0x0000C180*/
		g_core_fp.fp_read_sram_0f(fw_entry, tmp_addr, 0xC178, (84));

		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_fw_cfg_1, 0xBFFE, 388);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_1, 0xE000, (128*4));

	}
	I("%s, END \n", __func__);
}
static void hx83102d_firmware_read_0f(const struct firmware *fw_entry, int type)
{
	uint8_t tmp_data[4];

	I("%s, Entering \n", __func__);

	switch (type) {
	case 0:
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_sram_start_addr, 0, HX_40K_SZ);
		break;
	case 1:
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_cfg_info, 0xC000, 132);
		break;
	case 2:
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_fw_cfg_1, 0xC0FE, 484);
		break;
	case 3:
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_1, 0xD000, 768);
		break;
	case 4:
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_2, 0xD300, 1536);
		break;
	case 5:
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_3, 0xE000, 1536);
		break;
	case 6:
		himax_in_parse_assign_cmd(hx83102d_zf_data_bor_prevent_info, tmp_data, 4);
		g_core_fp.fp_read_sram_0f(fw_entry, tmp_data, 0xC9E0, 32);
		break;
	case 7:
		himax_in_parse_assign_cmd(hx83102d_zf_data_notch_info, tmp_data, 4);
		g_core_fp.fp_read_sram_0f(fw_entry, tmp_data, 0xCA00, 128);
		break;
	case 8:
		himax_in_parse_assign_cmd(hx83102d_zf_func_info_en, tmp_data, 4);
		g_core_fp.fp_read_sram_0f(fw_entry, tmp_data, 0xCB00, 12);
		break;
	case 9:
		himax_in_parse_assign_cmd(hx83102d_zf_po_sub_func, tmp_data, 4);
		g_core_fp.fp_read_sram_0f(fw_entry, tmp_data, 0xA000, HX4K);
		break;
	default:
		break;
	}
	I("%s, END \n", __func__);
}
#endif
#endif

static void himax_hx83102ab_reg_re_init(void)
{
	I("%s:Entering!\n", __func__);
	himax_in_parse_assign_cmd(hx83102ab_fw_addr_sorting_mode_en, pfw_op->addr_sorting_mode_en, sizeof(pfw_op->addr_sorting_mode_en));
	himax_in_parse_assign_cmd(hx83102ab_fw_addr_selftest_addr_en, pfw_op->addr_selftest_addr_en, sizeof(pfw_op->addr_selftest_addr_en));
#ifdef HX_ZERO_FLASH
	himax_in_parse_assign_cmd(hx83102ab_data_adc_cfg_1, pzf_op->data_adc_cfg_1, sizeof(pzf_op->data_adc_cfg_1));
#endif

}

static void himax_hx83102ab_func_re_init(void)
{
	I("%s:Entering!\n", __func__);
	g_core_fp.fp_chip_init = hx83102_chip_init;
	g_core_fp.fp_set_SMWP_enable = hx83102ab_set_SMWP_enable;
	g_core_fp.fp_set_HSEN_enable = hx83102ab_set_HSEN_enable;
	g_core_fp.fp_usb_detect_set = hx83102ab_usb_detect_set;
	g_core_fp.fp_read_DD_status = hx83102ab_read_DD_status;
	g_core_fp.fp_read_FW_status = hx83102ab_read_FW_status;
	g_core_fp.fp_power_on_init = hx83102ab_power_on_init;
	g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_64k = hx83102ab_fts_ctpm_fw_upgrade_with_sys_fs_64k;
#ifdef HX_ESD_RECOVERY
	g_core_fp.fp_esd_ic_reset = hx83102ab_esd_ic_reset;
#endif
#if defined(HX_ZERO_FLASH)
	g_core_fp.fp_firmware_update_0f = hx83102ab_firmware_update_0f;
#if defined(HX_0F_DEBUG)
	g_core_fp.fp_firmware_read_0f = hx83102ab_firmware_read_0f;
#endif
#endif

}

static void himax_hx83102d_reg_re_init(void)
{
	I("%s:Entering!\n", __func__);
	himax_in_parse_assign_cmd(hx83102d_fw_addr_raw_out_sel, pfw_op->addr_raw_out_sel, sizeof(pfw_op->addr_raw_out_sel));
#if defined(HX_ZERO_FLASH)
	himax_in_parse_assign_cmd(hx83102d_zf_data_sram_start_addr, pzf_op->data_sram_start_addr, sizeof(pzf_op->data_sram_start_addr));
	himax_in_parse_assign_cmd(hx83102d_zf_data_adc_cfg_1, pzf_op->data_adc_cfg_1, sizeof(pzf_op->data_adc_cfg_1));
	himax_in_parse_assign_cmd(hx83102d_zf_data_adc_cfg_2, pzf_op->data_adc_cfg_2, sizeof(pzf_op->data_adc_cfg_2));
	himax_in_parse_assign_cmd(hx83102d_zf_data_adc_cfg_3, pzf_op->data_adc_cfg_3, sizeof(pzf_op->data_adc_cfg_3));
#endif
}

static void himax_hx83102d_func_re_init(void)
{
	I("%s:Entering!\n", __func__);
	g_core_fp.fp_chip_init = hx83102_chip_init;
#if defined(HX_ZERO_FLASH)
	g_core_fp.fp_firmware_update_0f = hx83102d_firmware_update_0f;
	g_core_fp.fp_resume_ic_action = himax_hx83102d_resume_ic_action;
	g_core_fp.fp_suspend_ic_action = himax_hx83102d_suspend_ic_action;
	g_core_fp.fp_sense_on = himax_hx83102d_sense_on;
#if defined(HX_0F_DEBUG)
	g_core_fp.fp_firmware_read_0f = hx83102d_firmware_read_0f;
#endif
#endif

}

bool hx83102_chip_detect(void)
{
	uint8_t tmp_data[FOUR_BYTE_DATA_SZ];
	uint8_t tmp_addr[FOUR_BYTE_DATA_SZ];
	bool ret_data = false;
	int i = 0;

	I("[%s][%d]:enter\n", __func__, __LINE__);

	msleep(50);
	hx83102_sense_off();

	for (i = 0; i < 5; i++) {
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xD0;
		hx83102_register_read(tmp_addr, FOUR_BYTE_DATA_SZ, tmp_data);
		I("%s:Read driver IC ID = %X,%X,%X\n", __func__, tmp_data[3], tmp_data[2], tmp_data[1]); /*83,10,2X*/

		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x10) && ((tmp_data[1] == 0x2a) || (tmp_data[1] == 0x2b) || (tmp_data[1] == 0x2d))) {
			if (tmp_data[1] == 0x2a) {
				strlcpy(private_ts->chip_name, HX_83102A_SERIES_PWON, 30);
				I("%s:detect IC HX83102A successfully\n", __func__);
			} else if (tmp_data[1] == 0x2b) {
				strlcpy(private_ts->chip_name, HX_83102B_SERIES_PWON, 30);
				I("%s:detect IC HX83102B successfully\n", __func__);
			} else {
				strlcpy(private_ts->chip_name, HX_83102D_SERIES_PWON, 30);
				I("%s:detect IC HX83102D successfully\n", __func__);
			}
			himax_mcu_in_cmd_struct_init();
			himax_mcu_in_cmd_init();
			if ((tmp_data[1] == 0x2a) || (tmp_data[1] == 0x2b)) {
				himax_hx83102ab_reg_re_init();
				himax_hx83102ab_func_re_init();
			} else {
				himax_hx83102d_reg_re_init();
				himax_hx83102d_func_re_init();
			}
			ret_data = true;
			break;
		} else {
			ret_data = false;
			E("%s:Read driver ID register Fail:\n", __func__);
			E("Could NOT find Himax Chipset \n");
			E("Please check 1.VCCD,VCCA,VSP,VSN \n");
			E("2. LCM_RST,TP_RST \n");
			E("3. Power On Sequence \n");
		}
	}

	return ret_data;
}

#if 0
#ifdef CONFIG_CHIP_DTCFG
static int himax_hx83102_probe(struct platform_device *pdev)
{
	I("%s:Enter\n", __func__);

	g_core_fp.fp_chip_detect = hx83102_chip_detect;

	return 0;
}

static int himax_hx83102_remove(struct platform_device *pdev)
{
	g_core_fp.fp_chip_detect = NULL;
	g_core_fp.fp_chip_init = NULL;
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id himax_hx83102_mttable[] = {
	{ .compatible = "himax,hx83102"},
	{ },
};
#else
#define himax_hx83102_mttabl NULL
#endif

static struct platform_driver himax_hx83102_driver = {
	.probe	= himax_hx83102_probe,
	.remove	= himax_hx83102_remove,
	.driver	= {
		.name           = "HIMAX_HX83102",
		.owner          = THIS_MODULE,
		.of_match_table = himax_hx83102_mttable,
	},
};

static int __init himax_hx83102_init(void)
{
	I("%s\n", __func__);
	platform_driver_register(&himax_hx83102_driver);
	return 0;
}

static void __exit himax_hx83102_exit(void)
{
	platform_driver_unregister(&himax_hx83102_driver);
}

#else
static int himax_hx83102_probe(void)
{
	I("%s:Enter\n", __func__);

	g_core_fp.fp_chip_detect = hx83102_chip_detect;

	return 0;
}

static int himax_hx83102_remove(void)
{
	g_core_fp.fp_chip_detect = NULL;
	g_core_fp.fp_chip_init = NULL;
	return 0;
}

static int __init himax_hx83102_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx83102_probe();
	return 0;
}

static void __exit himax_hx83102_exit(void)
{
	himax_hx83102_remove();
}
#endif

module_init(himax_hx83102_init);
module_exit(himax_hx83102_exit);

MODULE_DESCRIPTION("HIMAX HX83102 touch driver");
MODULE_LICENSE("GPL");
#endif
