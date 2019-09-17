/*
 * Copyright (c) 2018 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/usb/usbpd.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>

#define mmi_pl_err(chip, fmt, ...)		\
	pr_err("%s: %s: " fmt, chip->name,	\
		__func__, ##__VA_ARGS__)	\

#define mmi_pl_dbg(chip, reason, fmt, ...)			\
	do {							\
		if (*chip->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3
#define TYPEC_HIGH_CURRENT_UA		3000000
#define TYPEC_MIDDLE_CURRENT_UA		2000000
#define SWITCH_CHARGER_PPS_VOLT		5000000
#define FLASH_CHARGER_PPS_MIN_VOLT	8000000
#define FLASH_CHARGER_PPS_VOLT		9000000
#define FLASH_CHARGER_PPS_MAX_VOLT	10000000

#define	BAT_OVP_FAULT_SHIFT			8
#define	BAT_OCP_FAULT_SHIFT			9
#define	BUS_OVP_FAULT_SHIFT			10
#define	BUS_OCP_FAULT_SHIFT			11
#define	BAT_THERM_FAULT_SHIFT			12
#define	BUS_THERM_FAULT_SHIFT			13
#define	DIE_THERM_FAULT_SHIFT			14

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BAT_THERM_ALARM_MASK		(1 << BAT_THERM_ALARM_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)

#define VBAT_REG_STATUS_SHIFT			0
#define IBAT_REG_STATUS_SHIFT			1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)

#define FLASHC_SHOW_MAX_SIZE 32
#define PPS_RET_HISTORY_SIZE	8

enum print_reason {
	PR_INTERRUPT    = BIT(0),
	PR_MISC         = BIT(2),
	PR_MOTO         = BIT(7),
};

static int __debug_mask = PR_INTERRUPT;
module_param_named(
	debug_mask, __debug_mask, int, S_IRUSR | S_IWUSR
);

typedef enum  {
	PM_STATE_DISCONNECT,
	PM_STATE_ENTRY,
	PM_STATE_SW_ENTRY,
	PM_STATE_SW_LOOP,
	PM_STATE_FLASHC_ENTRY,
	PM_STATE_FLASHC_TUNNING_CURR,
	PM_STATE_FLASHC_TUNNING_VOLT,
	PM_STATE_FLASHC_CC_LOOP,
	PM_STATE_FLASHC_CV_LOOP,
	PM_STATE_FLASHC_QUIT_1,
	PM_STATE_FLASHC_QUIT_2,
	PM_STATE_STOP_CHARGE,
} pm_sm_state_t;

struct mmi_pl_temp_zone {
	int		temp_c;
	int		super_uv;
	int		fcc_super_ua;
	int		high_uv;
	int		fcc_high_ua;
	int		norm_uv;
	int		fcc_norm_ua;
};

struct mmi_thermal_power_zone {
	int		power_voltage;
	int		power_current;
};

#define MAX_NUM_STEPS 10
enum mmi_pl_temp_zones {
	ZONE_FIRST = 0,
	/* states 0-9 are reserved for zones */
	ZONE_LAST = MAX_NUM_STEPS + ZONE_FIRST - 1,
	ZONE_HOT,
	ZONE_COLD,
	ZONE_NONE = 0xFF,
};

enum mmi_chrg_step {
	STEP_SUPER,
	STEP_HIGH,
	STEP_NORMAL,
	STEP_NUM,
};

enum mmi_pd_pps_result {
	NO_ERROR,
	BLANCE_POWER,
	RESET_POWER,
};

struct sys_config {
	u32 flashc_volt_hysteresis;
	u32 flashc_curr_hysteresis;
	u32 flashc_volt_up_steps;
	u32 flashc_curr_up_steps;
	u32 flashc_curr_down_steps;
	u32 flashc_volt_down_steps;
	u32 flashc_min_vbat_start;
	u32	pmic_curr_lp_lmt;
	u32	batt_ovp_lmt;
};

struct flashc_info {
	bool charge_enabled;
	bool batt_pres;
	bool vbus_pres;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;
	bool bat_ucp_alarm;

	bool bat_therm_alarm;
	bool bus_therm_alarm;
	bool die_therm_alarm;
	bool bat_therm_fault;
	bool bus_therm_fault;
	bool die_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	bool vbat_reg;
	bool ibat_reg;
	int  vbat_volt;
	int  vbus_volt;
	int  ibat_curr;
	int  ibus_curr;
	int  bat_temp;
	int  bus_temp;
	int  die_temp;
};

struct pmic_info {
	bool charge_enabled;
	bool batt_pres;
	bool vbus_pres;
	int  vbat_volt;
	int  vbus_volt;
	int  ibat_curr;
	int  ibus_curr;
	int  bat_temp;
};

struct mmi_pl_chg_manager {
	struct device		*dev;
	const char		*name;
	const char		*flashc_name;
	int			*debug_mask;

	struct sys_config	sys_configs;
	struct flashc_info	flashc_handle;
	struct pmic_info	pmic_handle;

	struct power_supply	*flashc_psy;
	struct power_supply	*batt_psy;
	struct power_supply	*usb_psy;
	struct power_supply *mmi_pl_pm_psy;

	struct notifier_block	psy_nb;
	struct usbpd		*pd_handle;
	struct usbpd_pdo_info mmi_pdo_info[PD_MAX_PDO_NUM];
	int			mmi_pps_pdo_idx;

	int			pmic_ichg_val;
	bool			pmic_ichg_limited;
	int			pmic_step_volt;
	int			pmic_step_curr;

	int			flashc_cc_max_curr_pre;
	int			flashc_cv_max_volt_pre;
	int			flashc_cv_taper_curr_pre;
	int			request_volt;
	int			request_current;
	int			request_volt_pre;
	int			request_curr_pre;
	int			target_volt;
	int			target_curr;
	int			pps_current_max;
	int			pps_voltage_max;
	bool			pps_increase_volt;
	bool			pps_power_balance;
	int			pps_result;
	int			pps_result_history[PPS_RET_HISTORY_SIZE];
	int			pps_result_history_idx;
	int			flashc_taper_delta_volt;
	int			flashc_taper_cnt;
	int			flashc_cc_tunning_cnt;
	bool			vbus_present;
	bool			force_pmic_chg;
	bool			enter_sw_loop;
	bool			pd_pps_support;
	bool			pd_pps_mitigation;
	bool			flashc_alarm;
	bool			flashc_fault;
	int			flashc_recovery_count;
	pm_sm_state_t	sm_state;
	u64			fg_esr_pulse_timeout;
	struct delayed_work	mmi_pl_sm_work;
	struct work_struct	psy_changed_work;
	struct completion	sm_completion;

	int			num_temp_zones;
	struct mmi_pl_temp_zone	*temp_zones;
	enum mmi_pl_temp_zones	pres_temp_zone;

	int		num_thermal_zones;
	struct mmi_thermal_power_zone *thermal_mitigation_zones;
	int		thermal_mitigation_level;
	enum mmi_chrg_step	pres_chrg_step;
};

static int __param_flashc_cc_max_ua;
module_param_named(
	param_flashc_cc_max_ua,
	__param_flashc_cc_max_ua, int, S_IRUSR | S_IWUSR);

static void mmi_update_pmic_status(struct mmi_pl_chg_manager *chip)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return;
	}

	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy)
			return;
	}

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (!rc)
		chip->pmic_handle.vbat_volt = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (!rc)
		chip->pmic_handle.ibat_curr = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_TEMP, &prop);
	if (!rc)
		chip->pmic_handle.bat_temp = prop.intval;

	rc = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (!rc)
		chip->pmic_handle.vbus_volt = prop.intval;

	rc = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &prop);
	if (!rc)
		chip->pmic_handle.ibus_curr = prop.intval;

	rc = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_PRESENT, &prop);
	if (!rc)
		chip->pmic_handle.vbus_pres = !!prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &prop);
	if (!rc)
		chip->pmic_handle.charge_enabled= !!prop.intval;

	mmi_pl_dbg(chip, PR_MOTO, "main chargr IC : ----- status update ----\n");
	mmi_pl_dbg(chip, PR_MOTO, "vbat_volt %d \n",
		chip->pmic_handle.vbat_volt);
	mmi_pl_dbg(chip, PR_MOTO, "ibat_curr %d \n",
		chip->pmic_handle.ibat_curr);
	mmi_pl_dbg(chip, PR_MOTO, "vbus_volt %d \n",
		chip->pmic_handle.vbus_volt);
	mmi_pl_dbg(chip, PR_MOTO, "ibus_curr %d \n",
		chip->pmic_handle.ibus_curr);
	mmi_pl_dbg(chip, PR_MOTO, "bat_temp %d \n",
		chip->pmic_handle.bat_temp);
	mmi_pl_dbg(chip, PR_MOTO, "vbus_pres %d \n",
		chip->pmic_handle.vbus_pres);
	mmi_pl_dbg(chip, PR_MOTO, "charge_enabled %d \n",
		chip->pmic_handle.charge_enabled);


}

static void mmi_update_flashc_status(struct mmi_pl_chg_manager *chip)
{

	int rc;
	union power_supply_propval prop = {0,};

	chip->flashc_psy = power_supply_get_by_name(chip->flashc_name);
	if (!chip->flashc_psy)
		return;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (!rc)
		chip->flashc_handle.vbat_volt = prop.intval*1000;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (!rc)
		chip->flashc_handle.ibat_curr = prop.intval*1000;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED, &prop);
	if (!rc)
		chip->flashc_handle.vbus_volt = prop.intval*1000;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &prop);
	if (!rc)
		chip->flashc_handle.ibus_curr = prop.intval*1000;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_PRESENT, &prop);
	if (!rc)
		chip->flashc_handle.vbus_pres = !!prop.intval;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (!rc)
		chip->flashc_handle.charge_enabled = !!prop.intval;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_CHARGE_NOW_ERROR, &prop);
	if (!rc) {
		chip->flashc_handle.bat_ovp_alarm =
				!!(prop.intval & BAT_OVP_ALARM_MASK);
		chip->flashc_handle.bat_ocp_alarm =
				!!(prop.intval & BAT_OCP_ALARM_MASK);
		chip->flashc_handle.bus_ovp_alarm =
				!!(prop.intval & BUS_OVP_ALARM_MASK);
		chip->flashc_handle.bus_ocp_alarm =
				!!(prop.intval & BUS_OCP_ALARM_MASK);
		chip->flashc_handle.bat_ucp_alarm =
				!!(prop.intval & BAT_UCP_ALARM_MASK);
		chip->flashc_handle.bat_therm_alarm =
				!!(prop.intval & BAT_THERM_ALARM_MASK);
		chip->flashc_handle.bus_therm_alarm =
				!!(prop.intval & BUS_THERM_ALARM_MASK);
		chip->flashc_handle.die_therm_alarm =
				!!(prop.intval & DIE_THERM_ALARM_MASK);
		chip->flashc_handle.bat_ovp_fault =
				!!(prop.intval & BAT_OVP_FAULT_MASK);
		chip->flashc_handle.bat_ocp_fault =
				!!(prop.intval & BAT_OCP_FAULT_MASK);
		chip->flashc_handle.bus_ovp_fault =
				!!(prop.intval & BUS_OVP_FAULT_MASK);
		chip->flashc_handle.bus_ocp_fault =
				!!(prop.intval & BUS_OCP_FAULT_MASK);
		chip->flashc_handle.bat_therm_fault =
				!!(prop.intval & BAT_THERM_FAULT_MASK);
		chip->flashc_handle.bus_therm_fault =
				!!(prop.intval & BUS_THERM_FAULT_MASK);
		chip->flashc_handle.die_therm_fault =
				!!(prop.intval & DIE_THERM_FAULT_MASK);
	}
	power_supply_put(chip->flashc_psy);

	mmi_pl_dbg(chip, PR_MOTO, "flash charge IC : ---- status update ---\n");
	mmi_pl_dbg(chip, PR_MOTO, "vbat_volt %d \n",
		chip->flashc_handle.vbat_volt);
	mmi_pl_dbg(chip, PR_MOTO, "ibat_curr %d \n",
		chip->flashc_handle.ibat_curr);
	mmi_pl_dbg(chip, PR_MOTO, "vbus_volt %d \n",
		chip->flashc_handle.vbus_volt);
	mmi_pl_dbg(chip, PR_MOTO, "ibus_curr %d \n",
		chip->flashc_handle.ibus_curr);
	mmi_pl_dbg(chip, PR_MOTO, "bus_temp %d \n",
		chip->flashc_handle.bus_temp);
	mmi_pl_dbg(chip, PR_MOTO, "bat_temp %d \n",
		chip->flashc_handle.bat_temp);
	mmi_pl_dbg(chip, PR_MOTO, "die_temp %d \n",
		chip->flashc_handle.die_temp);
	mmi_pl_dbg(chip, PR_MOTO, "batt_pres %d \n",
		chip->flashc_handle.batt_pres);
	mmi_pl_dbg(chip, PR_MOTO, "vbus_pres %d \n",
		chip->flashc_handle.vbus_pres);
	mmi_pl_dbg(chip, PR_MOTO, "charge_enabled %d \n",
		chip->flashc_handle.charge_enabled);
	mmi_pl_dbg(chip, PR_MOTO, "bat_ovp_alarm %d \n",
		chip->flashc_handle.bat_ovp_alarm);
	mmi_pl_dbg(chip, PR_MOTO, "bat_ocp_alarm %d \n",
		chip->flashc_handle.bat_ocp_alarm);
	mmi_pl_dbg(chip, PR_MOTO, "bus_ovp_alarm %d \n",
		chip->flashc_handle.bus_ovp_alarm);
	mmi_pl_dbg(chip, PR_MOTO, "bus_ocp_alarm %d \n",
		chip->flashc_handle.bus_ocp_alarm);
	mmi_pl_dbg(chip, PR_MOTO, "bat_ucp_alarm %d \n",
		chip->flashc_handle.bat_ucp_alarm);
	mmi_pl_dbg(chip, PR_MOTO, "bat_therm_alarm %d \n",
		chip->flashc_handle.bat_therm_alarm);
	mmi_pl_dbg(chip, PR_MOTO, "bus_therm_alarm %d \n",
		chip->flashc_handle.bus_therm_alarm);
	mmi_pl_dbg(chip, PR_MOTO, "die_therm_alarm %d \n",
		chip->flashc_handle.die_therm_alarm);

	mmi_pl_dbg(chip, PR_MOTO, "bat_ovp_fault %d \n",
		chip->flashc_handle.bat_ovp_fault);
	mmi_pl_dbg(chip, PR_MOTO, "bat_ocp_fault %d \n",
		chip->flashc_handle.bat_ocp_fault);
	mmi_pl_dbg(chip, PR_MOTO, "bus_ovp_fault %d \n",
		chip->flashc_handle.bus_ovp_fault);
	mmi_pl_dbg(chip, PR_MOTO, "bus_ocp_fault %d \n",
		chip->flashc_handle.bus_ocp_fault);
	mmi_pl_dbg(chip, PR_MOTO, "bat_therm_fault %d \n",
		chip->flashc_handle.bat_therm_fault);
	mmi_pl_dbg(chip, PR_MOTO, "bus_therm_fault %d \n",
		chip->flashc_handle.bus_therm_fault);
	mmi_pl_dbg(chip, PR_MOTO, "die_therm_fault %d \n",
		chip->flashc_handle.die_therm_fault);
	mmi_pl_dbg(chip, PR_MOTO, "flash charge : ---- over ---\n");

}

static int mmi_pl_pm_flashc_enable(struct mmi_pl_chg_manager *chip, bool enable)
{
	int rc;
	union power_supply_propval prop = {0,};

	chip->flashc_psy = power_supply_get_by_name(chip->flashc_name);
	if (!chip->flashc_psy)
		return -ENODEV;

	prop.intval = enable;
	rc = power_supply_set_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc < 0) {
		mmi_pl_err(chip, "Couldn't disable flashc charging, rc=%d\n", rc);
		power_supply_put(chip->flashc_psy);
		return rc;
	}
	power_supply_put(chip->flashc_psy);

	return rc;
}

static int mmi_pl_pm_check_flashc_enable(struct mmi_pl_chg_manager *chip)
{
	int rc;
	union power_supply_propval prop = {0,};

	chip->flashc_psy = power_supply_get_by_name(chip->flashc_name);
	if (!chip->flashc_psy) {
		chip->flashc_handle.charge_enabled = false;
		return -ENODEV;
	}

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (!rc) {
		chip->flashc_handle.charge_enabled = !!prop.intval;
	} else
		chip->flashc_handle.charge_enabled = false;

	power_supply_put(chip->flashc_psy);

	return rc;
}

static int mmi_pl_pm_pmic_enable(struct mmi_pl_chg_manager *chip, bool enable)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return -ENODEV;
	}

	prop.intval = enable;
	rc = power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &prop);
	if (rc < 0) {
		mmi_pl_err(chip, "Couldn't disable pmic charging, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int mmi_pl_pm_check_pmic_enable(struct mmi_pl_chg_manager *chip)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy) {
			chip->pmic_handle.charge_enabled = false;
			return -ENODEV;
		}
	}

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &prop);
	if (!rc) {
		chip->pmic_handle.charge_enabled = !!prop.intval;
	} else
		chip->pmic_handle.charge_enabled = false;

	return rc;
}

static int mmi_pl_pm_pmic_ichg_lmt(struct mmi_pl_chg_manager *chip, unsigned uA)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return -ENODEV;
	}

	prop.intval = uA;
	rc = power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0) {
		mmi_pl_err(chip, "Couldn't limit usb current, rc=%d\n", rc);
		return rc;
	}

	chip->pmic_ichg_limited = true;

	return rc;
}

static int mmi_pl_pm_get_usb_icl(struct mmi_pl_chg_manager *chip)
{
	int rc;
	union power_supply_propval prop = {0,};

	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_HW_CURRENT_MAX, &prop);
	if (rc < 0) {
		mmi_pl_err(chip, "Couldn't limit usb current, rc=%d\n", rc);
		return rc;
	}

	chip->pmic_ichg_val = prop.intval;
	return rc;
}

#define MIN_TEMP_C -20
#define MAX_TEMP_C 60
#define MIN_MAX_TEMP_C 47
#define HYSTERISIS_DEGC 2
static bool mmi_find_temp_zone(struct mmi_pl_chg_manager *chip, int temp_c)
{
	int prev_zone, num_zones;
	struct mmi_pl_temp_zone *zones;
	int hotter_t;
	int colder_t;
	int i;
	int max_temp;

	if (!chip) {
		mmi_pl_err(chip, "called before chip valid!\n");
		return false;
	}

	zones = chip->temp_zones;
	num_zones = chip->num_temp_zones;
	prev_zone = chip->pres_temp_zone;

	max_temp = zones[num_zones - 1].temp_c;

	if (prev_zone == ZONE_NONE) {
		for (i = num_zones - 1; i >= 0; i--) {
			if (temp_c >= zones[i].temp_c) {
				if (i == num_zones - 1)
					chip->pres_temp_zone = ZONE_HOT;
				else
					chip->pres_temp_zone = i + 1;
				return true;
			}
		}
		chip->pres_temp_zone = ZONE_COLD;
		return true;
	}

	if (prev_zone == ZONE_COLD) {
		if (temp_c >= MIN_TEMP_C + HYSTERISIS_DEGC)
			chip->pres_temp_zone = ZONE_FIRST;
	} else if (prev_zone == ZONE_HOT) {
		if (temp_c <=  max_temp - HYSTERISIS_DEGC)
			chip->pres_temp_zone = num_zones - 1;
	} else {
		if (prev_zone == ZONE_FIRST) {
			hotter_t = zones[prev_zone].temp_c;
			colder_t = MIN_TEMP_C;
		} else if (prev_zone == num_zones - 1) {
			hotter_t = zones[prev_zone].temp_c + HYSTERISIS_DEGC;
			colder_t = zones[prev_zone - 1].temp_c;
		} else {
			hotter_t = zones[prev_zone].temp_c + HYSTERISIS_DEGC;
			colder_t = zones[prev_zone - 1].temp_c;
		}

		if (temp_c < MIN_TEMP_C)
			chip->pres_temp_zone = ZONE_COLD;
		else if (temp_c >= max_temp)
			chip->pres_temp_zone = ZONE_HOT;
		else if (temp_c >= hotter_t)
			chip->pres_temp_zone++;
		else if (temp_c < colder_t)
			chip->pres_temp_zone--;
	}

	if (prev_zone != chip->pres_temp_zone) {
		mmi_pl_dbg(chip, PR_MOTO,
			   "Entered Temp Zone %d!\n",
			   chip->pres_temp_zone);
		return true;
	}

	return false;
}

const unsigned char *pm_state_str[] = {
	"PM_STATE_DISCONNECT",
	"PM_STATE_ENTRY",
	"PM_STATE_SW_ENTRY",
	"PM_STATE_SW_LOOP",
	"PM_STATE_FLASHC_ENTRY",
	"PM_STATE_FLASHC_TUNNING_CURR",
	"PM_STATE_FLASHC_TUNNING_VOLT",
	"PM_STATE_FLASHC_CC_LOOP",
	"PM_STATE_FLASHC_CV_LOOP",
	"PM_STATE_FLASHC_QUIT_1",
	"PM_STATE_FLASHC_QUIT_2",
	"PM_STATE_STOP_CHARGE",
};

static int mmi_calculate_delta_volt(int pps_voltage, int pps_current, int delta_curr)
{
	u64 power;
	int delta_volt;
	u64 pps_volt;
	u64 pps_curr;

	pps_volt = pps_voltage;
	pps_curr = pps_current;
	power = pps_volt * pps_curr;

	delta_volt = (int)(power / (pps_curr - delta_curr) - pps_volt);
	delta_volt -= delta_volt % 20000;

	if (delta_volt > 0)
		return delta_volt;
	else
		return 0;
}

static void mmi_pl_pm_move_state(struct mmi_pl_chg_manager *chip, pm_sm_state_t state)
{
	mmi_pl_dbg(chip, PR_INTERRUPT, "pm_state change:%s -> %s\n",
		pm_state_str[chip->sm_state], pm_state_str[state]);
	chip->sm_state = state;
}

static void mmi_set_pps_result_history(struct mmi_pl_chg_manager *chip, int pps_result)
{
	if (chip->pps_result_history_idx >= PPS_RET_HISTORY_SIZE -1)
		chip->pps_result_history_idx = 0;

	if (pps_result < 0)
		chip->pps_result_history[chip->pps_result_history_idx] = 1;
	else
		chip->pps_result_history[chip->pps_result_history_idx] = 0;

	chip->pps_result_history_idx++;
}

static bool mmi_get_pps_result_history(struct mmi_pl_chg_manager *chip)
{
	int i = 0;
	int result = 0;
	for (i = 0; i < PPS_RET_HISTORY_SIZE; i++)
		result += chip->pps_result_history[i];

	if (result >= PPS_RET_HISTORY_SIZE)
		return RESET_POWER;
	else if (result >= PPS_RET_HISTORY_SIZE / 2)
		return BLANCE_POWER;
	else
		return NO_ERROR;
}

static void kick_sm(struct mmi_pl_chg_manager *chip, int ms)
{
	if (!delayed_work_pending(&chip->mmi_pl_sm_work)) {

		mmi_pl_dbg(chip, PR_INTERRUPT,
					"lunch flash charge state machine\n");
		schedule_delayed_work(&chip->mmi_pl_sm_work,
				msecs_to_jiffies(ms));
	} else
		mmi_pl_dbg(chip, PR_INTERRUPT,
					"flash charge state machine already existed\n");
}

static void cancel_sm(struct mmi_pl_chg_manager *chip)
{
	mmi_pl_dbg(chip, PR_INTERRUPT, "flush and cancel flash charge sm\n");
	flush_delayed_work(&chip->mmi_pl_sm_work);
	cancel_delayed_work(&chip->mmi_pl_sm_work);
}

static void clear_chg_manager(struct mmi_pl_chg_manager *chip)
{
	mmi_pl_dbg(chip, PR_INTERRUPT, "clear pl chg manager!\n");

	if (chip->flashc_handle.charge_enabled) {
		mmi_pl_err(chip, "disable flashc charge\n");
		mmi_pl_pm_flashc_enable(chip, false);
		mmi_pl_pm_check_flashc_enable(chip);
	}

	if (chip->pmic_ichg_limited) {
		mmi_pl_dbg(chip, PR_MOTO, "recovery ichg lmt\n");
		mmi_pl_pm_pmic_ichg_lmt(chip, chip->pmic_ichg_val);
		chip->pmic_ichg_limited = false;
	}

	mmi_update_pmic_status(chip);
	mmi_update_flashc_status(chip);
	chip->sm_state  = PM_STATE_DISCONNECT;
	chip->request_volt = 0;
	chip->request_current = 0;
	chip->target_curr = 0;
	chip->target_volt = 0;
	chip->pps_current_max = 0;
	chip->pps_voltage_max = 0;
	chip->force_pmic_chg = false;
	chip->enter_sw_loop = false;
	memset(chip->mmi_pdo_info, 0,
			sizeof(struct usbpd_pdo_info) * PD_MAX_PDO_NUM);
}

#define FG_ESR_PULSE_MAX_TIMEOUT 60000
#define FG_ESR_DECREMENT_UA		300000
#define FG_ESR_DECREMENT_UV		300000
#define FG_ESR_MIN_CURR_UA		1500000
#define FLASHC_CV_DECREMENT_UV	100000
#define HEARTBEAT_lOOP_WAIT_MS 5000
#define HEARTBEAT_PPS_TUNNING_MS 50
#define HEARTBEAT_SHORT_DELAY_MS 500
#define HEARTBEAT_NEXT_STATE_MS 100
#define HEARTBEAT_CANNEL -1
#define FLASHC_TAPPER_COUNT 3
#define PPS_SELECT_PDO_RETRY_COUNT 3
#define FLASHC_ALARM_RECOVERY_COUNT 5
#define FLASHC_FAULT_RECOVERY_COUNT 10

static void mmi_pl_sm_work_func(struct work_struct *work)
{
	struct mmi_pl_chg_manager *chip = container_of(work,
				struct mmi_pl_chg_manager, mmi_pl_sm_work.work);
	int rc = 0, i = 0;
	int heartbeat_dely_ms = 0;
	int request_volt = 0;
	int request_curr = 0;
	int ibat_curr = 0;
	int vbat_volt = 0;
	int bat_temp = 0;
	int flashc_cc_max_curr = 0;
	int flashc_cv_max_volt = 0;
	int flashc_cv_taper_curr = 0;
	int pluse_curr = 0;
	int pluse_volt = 0;
	int chrg_step_max = 0;
	enum mmi_chrg_step	chrg_step;
	struct mmi_pl_temp_zone *zone;
	bool zone_change = false;
	bool flashc_fault_now = false;

	mmi_pl_dbg(chip, PR_MOTO, "schedule state matchine work, state : %s\n",
					pm_state_str[chip->sm_state]);
	if (!chip->usb_psy)
		chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		mmi_pl_err(chip, "USB psy not found\n");
		return;
	}

	mmi_update_pmic_status(chip);
	mmi_update_flashc_status(chip);

	if (chip->flashc_handle.bat_ocp_alarm
		|| chip->flashc_handle.bat_ovp_alarm
		|| chip->flashc_handle.bat_therm_alarm
		|| chip->flashc_handle.bus_ocp_alarm
		|| chip->flashc_handle.bus_ovp_alarm
		|| chip->flashc_handle.bus_therm_alarm
		|| chip->flashc_handle.die_therm_alarm) {
		chip->request_volt -= chip->sys_configs.flashc_volt_up_steps;
		chip->flashc_alarm = true;
		chip->flashc_recovery_count = FLASHC_ALARM_RECOVERY_COUNT;
		heartbeat_dely_ms = HEARTBEAT_lOOP_WAIT_MS;
		goto schedule;
	} else if (chip->flashc_alarm) {
			chip->flashc_recovery_count--;

			if (chip->flashc_recovery_count <= 0) {
				chip->flashc_alarm = false;
				chip->flashc_recovery_count = 0;
				mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);
				heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
			}
	}

	if (chip->flashc_handle.bat_ocp_fault
		|| chip->flashc_handle.bat_ovp_fault
		|| chip->flashc_handle.bat_therm_fault
		|| chip->flashc_handle.bus_ocp_fault
		|| chip->flashc_handle.bus_ovp_fault
		|| chip->flashc_handle.bus_therm_fault
		|| chip->flashc_handle.die_therm_fault) {
		chip->flashc_recovery_count = FLASHC_FAULT_RECOVERY_COUNT;
			mmi_pl_pm_move_state(chip, PM_STATE_STOP_CHARGE);
			chip->flashc_fault = true;
			flashc_fault_now = true;
	}

	ibat_curr = chip->pmic_handle.ibat_curr;
	if (ibat_curr < 0)
		ibat_curr *= -1;

	vbat_volt = chip->pmic_handle.vbat_volt;
	bat_temp = chip->pmic_handle.bat_temp / 10;

	zone_change = mmi_find_temp_zone(chip, bat_temp);
	zone = &chip->temp_zones[chip->pres_temp_zone];

	mmi_pl_dbg(chip, PR_MOTO, "zone info: %d, temp %d, "
					"super_uv %d, fcc super ua %d, "
					"high_uv %d, fcc_high_uv %d, "
					"norm_uv %d, fcc_norm_ua %d\n",
					chip->pres_temp_zone,
					zone->temp_c, zone->super_uv,
					zone->fcc_super_ua,
					zone->high_uv, zone->fcc_high_ua,
					zone->norm_uv, zone->fcc_norm_ua);

	if (chip->pres_temp_zone == ZONE_HOT
		|| chip->pres_temp_zone == ZONE_COLD
		|| !chip->pmic_handle.charge_enabled) {
		mmi_pl_pm_move_state(chip, PM_STATE_STOP_CHARGE);
	}

	if (!chip->pmic_handle.vbus_pres) {
		chip->sm_state = PM_STATE_DISCONNECT;
	} else if (chip->sm_state == PM_STATE_DISCONNECT
			|| zone_change
			|| chip->sm_state == PM_STATE_ENTRY) {
		mmi_pl_pm_get_usb_icl(chip);
		mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);

		if (vbat_volt > zone->high_uv && zone->norm_uv > 0) {
			chip->pres_chrg_step = STEP_NORMAL;
			chip->flashc_cc_max_curr_pre = zone->fcc_norm_ua;
			chip->flashc_cv_taper_curr_pre = zone->fcc_norm_ua;
			chip->flashc_cv_max_volt_pre = zone->norm_uv;
		} else if (vbat_volt > zone->super_uv && zone->high_uv > 0) {
			chip->pres_chrg_step = STEP_HIGH;
			chip->flashc_cc_max_curr_pre = zone->fcc_high_ua;
			chip->flashc_cv_max_volt_pre = zone->high_uv;
			if (zone->fcc_norm_ua > 0)
				chip->flashc_cv_taper_curr_pre =
					zone->fcc_norm_ua;
			else
				chip->flashc_cv_taper_curr_pre =
					zone->fcc_high_ua;

		} else {
			chip->pres_chrg_step = STEP_SUPER;
			chip->flashc_cc_max_curr_pre = zone->fcc_super_ua;
			chip->flashc_cv_max_volt_pre = zone->super_uv;
			if (zone->fcc_high_ua > 0)
				chip->flashc_cv_taper_curr_pre = zone->fcc_high_ua;
			else
				chip->flashc_cv_taper_curr_pre = zone->fcc_super_ua;
		}

		chip->flashc_cc_max_curr_pre =
		 	min(chip->flashc_cc_max_curr_pre, chip->pps_current_max * 2);
	}

	if (vbat_volt > zone->high_uv && zone->norm_uv > 0) {
		chrg_step = STEP_NORMAL;
		flashc_cc_max_curr = zone->fcc_norm_ua;
		flashc_cv_max_volt = zone->norm_uv;
		flashc_cv_taper_curr = zone->fcc_norm_ua;
	} else if (vbat_volt > zone->super_uv && zone->high_uv > 0) {
		chrg_step = STEP_HIGH;
		flashc_cc_max_curr = zone->fcc_high_ua;
		flashc_cv_max_volt = zone->high_uv;
		if (zone->fcc_norm_ua > 0)
			flashc_cv_taper_curr = zone->fcc_norm_ua;
		else
			flashc_cv_taper_curr = zone->fcc_high_ua;
	} else {
		chrg_step = STEP_SUPER;
		flashc_cc_max_curr = zone->fcc_super_ua;
		flashc_cv_max_volt = zone->super_uv;
		if (zone->fcc_high_ua > 0)
			flashc_cv_taper_curr = zone->fcc_high_ua;
		else
			flashc_cv_taper_curr = zone->fcc_super_ua;
	}

	chrg_step_max = max(chrg_step,chip->pres_chrg_step);
	if (flashc_cc_max_curr >
		chip->pps_current_max * 2 - chip->sys_configs.pmic_curr_lp_lmt)
		flashc_cc_max_curr =
		chip->pps_current_max * 2 - chip->sys_configs.pmic_curr_lp_lmt;

	if (__param_flashc_cc_max_ua > 0)
		chip->flashc_cc_max_curr_pre = __param_flashc_cc_max_ua;

	mmi_pl_dbg(chip, PR_MOTO,
				" chrg_step %d, chip->pre_chrg_step %d, "
				"chrg_step_max %d, "
				"chip->flashc_cc_max_curr_pre %d, "
				"chip->flashc_cv_max_volt_pre %d, "
				"chip->flashc_cv_taper_curr_pre %d\n",
				chrg_step, chip->pres_chrg_step,
				chrg_step_max,
				chip->flashc_cc_max_curr_pre,
				chip->flashc_cv_max_volt_pre,
				chip->flashc_cv_taper_curr_pre);

	switch (chip->sm_state) {
	case PM_STATE_DISCONNECT:
		mmi_pl_dbg(chip, PR_MOTO,
			" PM_STATE_DiSCONNECT,flashc charge_enable %d, "
			"pmic charge_enable %d, "
			" pmic_ichg_limited %d \n",
			chip->flashc_handle.charge_enabled,
			chip->pmic_handle.charge_enabled,
			chip->pmic_ichg_limited);

		if (chip->flashc_handle.charge_enabled) {
			mmi_pl_pm_flashc_enable(chip, false);
			mmi_pl_pm_check_flashc_enable(chip);
		}

		if (chip->pmic_ichg_limited) {
			rc = mmi_pl_pm_pmic_ichg_lmt(chip, chip->pmic_ichg_val);
			mmi_pl_dbg(chip, PR_MOTO, "recovery ichg lmt, %d\n",
							chip->pmic_ichg_val);
			if (rc < 0)
				mmi_pl_err(chip,
						"Couldn't recovery usb current, rc=%d\n", rc);

			chip->pmic_ichg_limited = false;
		}
		heartbeat_dely_ms = HEARTBEAT_CANNEL;
		break;
	case PM_STATE_ENTRY:
		if (!chip->pd_pps_support
			|| chip->pmic_handle.vbat_volt
				< chip->sys_configs.flashc_min_vbat_start
			|| chip->flashc_cc_max_curr_pre
				<= chip->pmic_step_curr) {
			mmi_pl_dbg(chip, PR_MOTO, "start switch charge due to : "
						"vbat %d, flashc_min_vbat %d, pps_support %d, "
						"flashc_cc_max_curr %d\n",
						chip->pmic_handle.vbat_volt,
						chip->sys_configs.flashc_min_vbat_start,
						chip->pd_pps_support,
						chip->flashc_cc_max_curr_pre);
			mmi_pl_pm_move_state(chip, PM_STATE_SW_ENTRY);
		} else if (chip->flashc_cc_max_curr_pre >
					chip->pmic_step_curr
				&& chip->pres_chrg_step < STEP_NORMAL
				&& !chip->force_pmic_chg) {
			mmi_pl_dbg(chip, PR_MOTO,
						"battery volt %d , flashc max curr %d,"
						"start flash charging\n",
						vbat_volt, chip->flashc_cc_max_curr_pre);
			mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_ENTRY);
			chip->flashc_taper_cnt = 0;
		} else {
			mmi_pl_dbg(chip, PR_MOTO, "zone change  %d ,battery volt %d, "
						"flashc max curr %d, max volt %d,"
						"start pmic charging\n",
						zone_change, vbat_volt,
						chip->flashc_cc_max_curr_pre,
						chip->flashc_cv_max_volt_pre);
			mmi_pl_pm_move_state(chip, PM_STATE_SW_ENTRY);
		}

		heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		break;
	case PM_STATE_SW_ENTRY:
		if (chip->flashc_handle.charge_enabled) {
			mmi_pl_pm_flashc_enable(chip, false);
			mmi_pl_pm_check_flashc_enable(chip);
		}

		if (chip->pmic_ichg_limited) {
			rc = mmi_pl_pm_pmic_ichg_lmt(chip, chip->pmic_ichg_val);
			if (rc < 0)
				mmi_pl_err(chip, "Couldn't recovery usb current, rc=%d\n", rc);
			chip->pmic_ichg_limited = false;
		}

		mmi_pl_err(chip, "recovery and rerun usb AICL, rc=%d\n", rc);
		mmi_pl_pm_pmic_enable(chip, false);
		msleep(100);
		mmi_pl_pm_pmic_enable(chip, true);
		mmi_pl_pm_check_pmic_enable(chip);

		for (i = 0; i < PD_MAX_PDO_NUM; i++) {

			if (chip->mmi_pdo_info[i].type ==
					PD_SRC_PDO_TYPE_FIXED
				&& chip->mmi_pdo_info[i].uv_max >= SWITCH_CHARGER_PPS_VOLT
				&& chip->mmi_pdo_info[i].ua >= TYPEC_HIGH_CURRENT_UA) {
					mmi_pl_dbg(chip, PR_MOTO, "select 5V/3A pps, pdo %d\n", i);
					chip->mmi_pps_pdo_idx =
						chip->mmi_pdo_info[i].pdo_pos;
					break;
				}
		}

		chip->pps_current_max = TYPEC_HIGH_CURRENT_UA;
		chip->pps_voltage_max = SWITCH_CHARGER_PPS_VOLT;
		chip->request_current = chip->pps_current_max;
		chip->request_volt = chip->pps_voltage_max;

		mmi_pl_dbg(chip, PR_MOTO, "select pdo %d, volt %d, curr %d\n",
						chip->mmi_pps_pdo_idx, SWITCH_CHARGER_PPS_VOLT,
						TYPEC_HIGH_CURRENT_UA);
		mmi_pl_pm_move_state(chip, PM_STATE_SW_LOOP);
		heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		break;
	case PM_STATE_SW_LOOP:
		if (chip->pd_pps_support
			&& (vbat_volt
				> chip->sys_configs.flashc_min_vbat_start)
			&& chip->flashc_cc_max_curr_pre >
					chip->pmic_step_curr
			&& chrg_step_max < STEP_NORMAL
			&& !chip->force_pmic_chg
			&& !chip->enter_sw_loop) {

			mmi_pl_dbg(chip, PR_MOTO,
						"battery volt %d, start flash charge\n",
						chip->pmic_handle.vbat_volt);
			mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_ENTRY);
			heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		} else {
			mmi_pl_dbg(chip, PR_MOTO,
					"keep in sw charge, vbat %d ,ibat %d\n",
					chip->pmic_handle.vbat_volt,
					chip->pmic_handle.ibat_curr);
			heartbeat_dely_ms = HEARTBEAT_lOOP_WAIT_MS;
		}
		break;
	case PM_STATE_FLASHC_ENTRY:
		if (!chip->pmic_ichg_limited) {
			mmi_pl_dbg(chip, PR_MOTO,
						"flashc charge, limit pmic output curr %d\n",
						chip->sys_configs.pmic_curr_lp_lmt);
			rc = mmi_pl_pm_pmic_ichg_lmt(chip,
						chip->sys_configs.pmic_curr_lp_lmt);
			if (rc < 0)
				mmi_pl_err(chip,
						"Couldn't limit pmic ichg current, rc=%d\n", rc);
			chip->pmic_ichg_limited = true;
		}

		usbpd_get_pdo_info(chip->pd_handle, chip->mmi_pdo_info);
		mmi_pl_dbg(chip, PR_MOTO, "check all effective pdo info\n");
		for (i = 0; i < PD_MAX_PDO_NUM; i++) {

			if (chip->mmi_pdo_info[i].type ==
					PD_SRC_PDO_TYPE_AUGMENTED
				&& chip->mmi_pdo_info[i].uv_max
					>= FLASH_CHARGER_PPS_MIN_VOLT
				&& chip->mmi_pdo_info[i].ua
					>= TYPEC_MIDDLE_CURRENT_UA) {
				chip->mmi_pps_pdo_idx =
					chip->mmi_pdo_info[i].pdo_pos;
				mmi_pl_dbg(chip, PR_MOTO,
					"pd charger support pps, pdo %d,"
					"volt %d, curr %d \n",
					chip->mmi_pps_pdo_idx,
					chip->mmi_pdo_info[i].uv_max,
					chip->mmi_pdo_info[i].ua);
				chip->pd_pps_support = true;

				if (chip->mmi_pdo_info[i].uv_max
						> FLASH_CHARGER_PPS_MAX_VOLT) {
					chip->pps_voltage_max
						= FLASH_CHARGER_PPS_MAX_VOLT;
				} else
					chip->pps_voltage_max
						= chip->mmi_pdo_info[i].uv_max;

				if (chip->mmi_pdo_info[i].ua
						> TYPEC_HIGH_CURRENT_UA) {
					chip->pps_current_max
						= TYPEC_HIGH_CURRENT_UA;
				} else
					chip->pps_current_max
						= chip->mmi_pdo_info[i].ua;
				break;
			}
		}

		request_volt = (2 * vbat_volt) % 20000;
		request_volt = 2 * vbat_volt - request_volt
						+ chip->sys_configs.flashc_volt_hysteresis;
		request_volt = min(request_volt,FLASH_CHARGER_PPS_VOLT);

		if (chip->pps_current_max > TYPEC_MIDDLE_CURRENT_UA)
			request_curr = TYPEC_MIDDLE_CURRENT_UA;
		else
			request_curr = chip->pps_current_max;

		chip->request_current = request_curr;
		chip->request_volt = request_volt;

		mmi_pl_dbg(chip, PR_MOTO, "flashc charge ready 1 step,"
						"init pps output volt %d (2*vbat + %d uv),"
						"output curr %d, pdo %d\n",
						request_volt,
						chip->sys_configs.flashc_volt_hysteresis,
						request_curr,
						chip->mmi_pps_pdo_idx);
		if (!chip->flashc_handle.charge_enabled) {
			mmi_pl_pm_flashc_enable(chip, true);
			mmi_pl_pm_check_flashc_enable(chip);
		}

		mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_TUNNING_CURR);
		heartbeat_dely_ms = HEARTBEAT_SHORT_DELAY_MS;
		break;
	case PM_STATE_FLASHC_TUNNING_CURR:
		mmi_pl_dbg(chip, PR_MOTO, "flashc charge ready 2 step,"
								"increase pps current\n");
		if ((chip->request_current +
			chip->sys_configs.flashc_curr_up_steps)
			<= chip->pps_current_max
			&& ((chip->pmic_handle.vbat_volt <
				chip->flashc_cv_max_volt_pre
			&& ibat_curr < chip->flashc_cc_max_curr_pre)
			|| chrg_step < STEP_HIGH)) {
			chip->request_current +=
				chip->sys_configs.flashc_curr_up_steps;
			mmi_pl_dbg(chip, PR_MOTO, "increase pps current %d\n",
							chip->request_current);
			heartbeat_dely_ms = HEARTBEAT_PPS_TUNNING_MS;
		} else {

			chip->pps_increase_volt = true;
			mmi_pl_pm_move_state(chip,
					PM_STATE_FLASHC_TUNNING_VOLT);
			heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;

			if (chrg_step !=chip->pres_chrg_step) {
				if (vbat_volt > zone->high_uv && zone->norm_uv > 0) {
					chip->pres_chrg_step = STEP_NORMAL;
					chip->flashc_cc_max_curr_pre = zone->fcc_norm_ua;
					chip->flashc_cv_taper_curr_pre = zone->fcc_norm_ua;
					chip->flashc_cv_max_volt_pre = zone->norm_uv;
				} else if (vbat_volt > zone->super_uv && zone->high_uv > 0) {
					chip->pres_chrg_step = STEP_HIGH;
					chip->flashc_cc_max_curr_pre = zone->fcc_high_ua;
					chip->flashc_cv_max_volt_pre = zone->high_uv;
					if (zone->fcc_norm_ua > 0)
						chip->flashc_cv_taper_curr_pre =
							zone->fcc_norm_ua;
					else
						chip->flashc_cv_taper_curr_pre =
							zone->fcc_high_ua;

				} else {
					chip->pres_chrg_step = STEP_SUPER;
					chip->flashc_cc_max_curr_pre = zone->fcc_super_ua;
					chip->flashc_cv_max_volt_pre = zone->super_uv;
					if (zone->fcc_high_ua > 0)
						chip->flashc_cv_taper_curr_pre = zone->fcc_high_ua;
					else
						chip->flashc_cv_taper_curr_pre = zone->fcc_super_ua;
				}

				chip->flashc_cc_max_curr_pre =
			 	min(chip->flashc_cc_max_curr_pre, chip->pps_current_max * 2);
				mmi_pl_dbg(chip, PR_INTERRUPT,
						"update flashc parameters for float voltage\n");
			}
		}

		if (!chip->flashc_handle.charge_enabled) {
			mmi_pl_dbg(chip, PR_MOTO,
					"flashc was disable,jump to SW change\n");
			mmi_pl_pm_move_state(chip, PM_STATE_SW_ENTRY);
		}
		break;
	case PM_STATE_FLASHC_TUNNING_VOLT:
		mmi_pl_dbg(chip, PR_MOTO, "flashc charge ready 3 step,"
					"increase pps voltage, effective flashc cc max curr %d,"
					"__param_flashc_cc_max_ua %d,"
					" allocated from pps max curr %d\n",
					chip->flashc_cc_max_curr_pre,
					__param_flashc_cc_max_ua,
				chip->pps_current_max * 2 - chip->sys_configs.pmic_curr_lp_lmt);

		if (chip->pps_result < 0) {
			mmi_pl_err(chip, "last select pdo failed\n");
			if (mmi_get_pps_result_history(chip) != NO_ERROR) {
				mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_CC_LOOP);
				mmi_pl_err(chip, "direct access to cc loop,"
								"because of too many pdo failed\n");
			}

			chip->request_volt = chip->request_volt_pre;
			goto schedule;
		}

			if (chrg_step !=chip->pres_chrg_step) {
				if (vbat_volt > zone->high_uv && zone->norm_uv > 0) {
					chip->pres_chrg_step = STEP_NORMAL;
					chip->flashc_cc_max_curr_pre = zone->fcc_norm_ua;
					chip->flashc_cv_taper_curr_pre = zone->fcc_norm_ua;
					chip->flashc_cv_max_volt_pre = zone->norm_uv;
				} else if (vbat_volt > zone->super_uv && zone->high_uv > 0) {
					chip->pres_chrg_step = STEP_HIGH;
					chip->flashc_cc_max_curr_pre = zone->fcc_high_ua;
					chip->flashc_cv_max_volt_pre = zone->high_uv;
					if (zone->fcc_norm_ua > 0)
						chip->flashc_cv_taper_curr_pre =
							zone->fcc_norm_ua;
					else
						chip->flashc_cv_taper_curr_pre =
							zone->fcc_high_ua;

				} else {
					chip->pres_chrg_step = STEP_SUPER;
					chip->flashc_cc_max_curr_pre = zone->fcc_super_ua;
					chip->flashc_cv_max_volt_pre = zone->super_uv;
					if (zone->fcc_high_ua > 0)
						chip->flashc_cv_taper_curr_pre = zone->fcc_high_ua;
					else
						chip->flashc_cv_taper_curr_pre = zone->fcc_super_ua;
				}

				mmi_pl_dbg(chip, PR_INTERRUPT,
						"update flashc parameters for float voltage\n");
			}

		if (chip->pps_increase_volt
			&& ibat_curr <
			((chip->pres_chrg_step == STEP_SUPER) ?
			chip->flashc_cc_max_curr_pre +
			chip->sys_configs.flashc_curr_hysteresis :
			chip->flashc_cc_max_curr_pre)
			&& chip->request_volt <
				chip->pps_voltage_max
			&& chip->pmic_handle.vbat_volt <
				chip->flashc_cv_max_volt_pre) {

			if ((chip->request_current +
				chip->sys_configs.flashc_curr_up_steps)
				< chip->pps_current_max) {
				chip->request_current +=
				chip->sys_configs.flashc_curr_up_steps;
				mmi_pl_dbg(chip, PR_MOTO,
					"increase pps current %d\n", chip->request_current);
			} else {
				chip->request_volt +=
					chip->sys_configs.flashc_volt_up_steps;
				mmi_pl_dbg(chip, PR_MOTO,
						"increase pps voltage %d\n", chip->request_volt);
			}
			heartbeat_dely_ms = HEARTBEAT_PPS_TUNNING_MS;
		} else if (!chip->pps_increase_volt
				&& ibat_curr > chip->flashc_cc_max_curr_pre
				&& chip->request_volt >
					SWITCH_CHARGER_PPS_VOLT) {
			chip->request_volt -=
					chip->sys_configs.flashc_volt_up_steps;
			mmi_pl_dbg(chip, PR_MOTO,
					"decrease pps voltage %d\n", chip->request_volt);
			heartbeat_dely_ms = HEARTBEAT_PPS_TUNNING_MS;
		} else {
			mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_CC_LOOP);
			heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		}

		if (!chip->flashc_handle.charge_enabled) {
			mmi_pl_dbg(chip, PR_MOTO,
					"flashc was disable,access to SW change\n");
			mmi_pl_pm_move_state(chip, PM_STATE_SW_ENTRY);
		}
		break;
	case PM_STATE_FLASHC_CC_LOOP:
		mmi_pl_dbg(chip, PR_MOTO, "flashc charge CC loop\n");
		heartbeat_dely_ms = HEARTBEAT_lOOP_WAIT_MS;

		if (chip->pps_result < 0) {
			mmi_pl_err(chip, "last select pdo failed\n");
			chip->pps_result = mmi_get_pps_result_history(chip);
			switch (chip->pps_result) {
			case BLANCE_POWER:
				chip->pps_power_balance = true;
				mmi_pl_err(chip, "enable pps power blance\n");
				break;
			case RESET_POWER:
				mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);
				mmi_pl_err(chip, "hard reset charge policy to recovery power,"
								"too many pdo failed\n");
				break;
			default:
				break;
			}

			chip->request_volt = chip->request_volt_pre;
			if (ibat_curr < chip->flashc_cc_max_curr_pre
				&& chip->request_volt < chip->pps_voltage_max)
				chip->flashc_cc_tunning_cnt =
							PPS_SELECT_PDO_RETRY_COUNT;
			goto schedule;
		}

		 if (ibat_curr < chip->flashc_cc_max_curr_pre
				&& chip->request_volt < chip->pps_voltage_max
				&& chip->flashc_cc_tunning_cnt >=
					PPS_SELECT_PDO_RETRY_COUNT
				&& chip->thermal_mitigation_level == 0) {
			if (chip->pps_power_balance) {
				chip->request_current -=
						chip->sys_configs.flashc_curr_down_steps;
				chip->request_volt +=
						mmi_calculate_delta_volt(chip->request_volt_pre,
							chip->request_curr_pre,
							chip->sys_configs.flashc_curr_down_steps);
				mmi_pl_dbg(chip, PR_MOTO, "request_curr decrease to %dmA, "
								"request_volt increase to %d\n",
								chip->request_current,
								chip->request_volt);
			} else {
				if ((chip->request_current +
					chip->sys_configs.flashc_curr_up_steps)
					< chip->pps_current_max) {
					chip->request_current +=
					chip->sys_configs.flashc_curr_up_steps;
					mmi_pl_dbg(chip, PR_MOTO,
						"increase pps current %d\n", chip->request_current);
				} else {
					chip->request_volt += chip->sys_configs.flashc_volt_up_steps;
					mmi_pl_dbg(chip, PR_MOTO, "request_volt increase %dmv\n",
							chip->sys_configs.flashc_volt_up_steps);
					}
			}

			chip->flashc_cc_tunning_cnt = 0;
			mmi_pl_dbg(chip, PR_MOTO, "need increase pps voltage,"
						"or decrease pps current"
						"to keep CC charger power,"
						"request volt %d, "
						"request curr %d, "
						"thermal level %d \n",
						chip->request_volt,
						chip->request_current,chip->thermal_mitigation_level);
		} else if (ibat_curr <
					chip->flashc_cc_max_curr_pre
				&& chip->request_volt < chip->pps_voltage_max) {
					chip->flashc_cc_tunning_cnt++;
					mmi_pl_dbg(chip, PR_MOTO,
						"flashc charge CC tunning cnt++ : %d \n",
						chip->flashc_cc_tunning_cnt);
		} else
			chip->flashc_cc_tunning_cnt = 0;

		if (chip->pmic_handle.vbat_volt
					> chip->flashc_cv_max_volt_pre) {
			if (chip->flashc_taper_cnt > FLASHC_TAPPER_COUNT) {
				mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_CV_LOOP);
				chip->flashc_taper_cnt = 0;
				chip->flashc_taper_delta_volt =
						FLASHC_CV_DECREMENT_UV;
				heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
			} else {
				chip->flashc_taper_cnt++;
				mmi_pl_dbg(chip, PR_MOTO,
							"flashc CC charge taper_cnt %d, "
							"flashc_cv_max_volt - 20mv %d\n",
							chip->flashc_taper_cnt,
							chip->flashc_cv_max_volt_pre - 20000);
			}
		} else
			chip->flashc_taper_cnt = 0;

		if (!chip->flashc_handle.charge_enabled) {
			mmi_pl_dbg(chip, PR_MOTO,
					"flashc was disable,access to SW change\n");
			mmi_pl_pm_move_state(chip, PM_STATE_SW_ENTRY);
		}
		break;
	case PM_STATE_FLASHC_CV_LOOP:
		heartbeat_dely_ms = HEARTBEAT_lOOP_WAIT_MS;
		mmi_pl_dbg(chip, PR_MOTO, "flashc charge CV loop,"
						"chrg_step %d , pre_chrg_step  %d \n",
						chrg_step, chip->pres_chrg_step);

		if (chip->pps_result < 0) {
			mmi_pl_err(chip, "last select pdo failed\n");
			chip->pps_result = mmi_get_pps_result_history(chip);
			switch (chip->pps_result) {
			case BLANCE_POWER:
				chip->request_current -=
						chip->sys_configs.flashc_curr_down_steps;
				mmi_pl_err(chip, "decrease pps curr %d uA for balnce power\n",
							chip->sys_configs.flashc_curr_down_steps);				
				break;
			case RESET_POWER:
				mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);
				mmi_pl_err(chip, "hard reset charge policy to recovery power,"
								"too many pdo failed\n");
				break;
			default:
				break;
			}

			chip->request_volt = chip->request_volt_pre;
			goto schedule;
		}

		if (vbat_volt > chip->flashc_cv_max_volt_pre + 20000) {
			if (chip->flashc_taper_delta_volt > 20000)
				chip->request_volt -= chip->flashc_taper_delta_volt;
			else chip->request_volt -= 20000;
			mmi_pl_dbg(chip, PR_MOTO, "For keeping flashc constant voltage step,"
						"decrease pps voltage %d ,delta volta %d \n",
						chip->request_volt, chip->flashc_taper_delta_volt);
		} else if (chip->pmic_handle.vbat_volt
				< chip->flashc_cv_max_volt_pre - 20000) {
			chip->flashc_taper_delta_volt -=  20000;
			chip->request_volt += 20000;
			mmi_pl_dbg(chip, PR_MOTO, "For keeping flashc constant voltage step,"
						"increase pps voltage %d , delta volta %d \n",
						chip->request_volt, chip->flashc_taper_delta_volt);
		} else {
			heartbeat_dely_ms = HEARTBEAT_lOOP_WAIT_MS;
			mmi_pl_dbg(chip, PR_MOTO, "Flashc CV loop work well,"
					"go on keeping pps voltage %d ,pps current %d\n",
					chip->request_volt, chip->request_current);
		}

		if (ibat_curr <= chip->flashc_cv_taper_curr_pre ||
			ibat_curr < chip->pmic_step_curr) {
			if (chip->flashc_taper_cnt >= FLASHC_TAPPER_COUNT) {
				if (ibat_curr < chip->pmic_step_curr) {
					mmi_pl_dbg(chip, PR_MOTO,
								"Ready to quit flashc CV charge, "
								"chrg step %d ,"
								"battery curr %d, "
								"flashc max curr %d, "
								"flashc max cv %d, "
								"flashc cv taper curr %d, "
								"pmic step volt %d, "
								"pimc step curr %d\n",
								chrg_step, ibat_curr,
								chip->flashc_cc_max_curr_pre,
								chip->flashc_cv_max_volt_pre,
								chip->flashc_cv_taper_curr_pre,
								chip->pmic_step_volt,
								chip->pmic_step_curr);
					mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_QUIT_1);
				} else if (chrg_step != chip->pres_chrg_step
					&& flashc_cc_max_curr < chip->flashc_cc_max_curr_pre) {
					chip->pres_chrg_step = chrg_step;
					chip->flashc_cc_max_curr_pre = flashc_cc_max_curr;
					chip->flashc_cv_max_volt_pre = flashc_cv_max_volt;
					chip->flashc_cv_taper_curr_pre = flashc_cv_taper_curr;

					mmi_pl_dbg(chip, PR_MOTO,
							"Start jump to the next charge step %d \n",
							chip->pres_chrg_step);

					mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_CC_LOOP);
					heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
				}
				chip->flashc_taper_cnt = 0;
			} else {
				chip->flashc_taper_cnt++;
				mmi_pl_dbg(chip, PR_MOTO,
					"Flashc CV taper cnt ++,%d \n", chip->flashc_taper_cnt);
			}
		} else
			chip->flashc_taper_cnt = 0;

		if (!chip->flashc_handle.charge_enabled) {
			mmi_pl_dbg(chip, PR_MOTO,
					"flashc was disable,jump to SW change\n");
			mmi_pl_pm_move_state(chip, PM_STATE_SW_ENTRY);
		}
		break;
	case PM_STATE_FLASHC_QUIT_1:
		heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		if (chip->flashc_handle.charge_enabled) {
			mmi_pl_pm_flashc_enable(chip, false);
			mmi_pl_pm_check_flashc_enable(chip);
		}

		if (chip->pmic_ichg_limited) {
			rc = mmi_pl_pm_pmic_ichg_lmt(chip, chip->pmic_ichg_val);
			if (rc < 0)
				mmi_pl_err(chip, "Couldn't recovery ichg, rc=%d\n", rc);
			chip->pmic_ichg_limited = false;
		}

		mmi_pl_pm_move_state(chip, PM_STATE_FLASHC_QUIT_2);
		break;
	case PM_STATE_FLASHC_QUIT_2:
		heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		if (chip->request_volt > SWITCH_CHARGER_PPS_VOLT) {
			chip->request_volt -=
					chip->sys_configs.flashc_volt_down_steps;
			chip->request_current = TYPEC_HIGH_CURRENT_UA;
			mmi_pl_dbg(chip, PR_MOTO,
					"decrease pps voltage %d\n", chip->request_volt);
			heartbeat_dely_ms = HEARTBEAT_PPS_TUNNING_MS;
		} else {
			mmi_pl_pm_pmic_enable(chip, false);
			msleep(100);
			mmi_pl_pm_pmic_enable(chip, true);
			mmi_pl_pm_check_pmic_enable(chip);
			mmi_pl_pm_move_state(chip, PM_STATE_SW_LOOP);
			chip->enter_sw_loop = true;
			mmi_pl_dbg(chip, PR_MOTO,
					"enter pmic charge loop\n");
		}

		break;
	case PM_STATE_STOP_CHARGE:
		heartbeat_dely_ms = HEARTBEAT_lOOP_WAIT_MS;
		if (chip->flashc_handle.charge_enabled) {
			mmi_pl_pm_flashc_enable(chip, false);
			mmi_pl_pm_check_flashc_enable(chip);
		}

		if (chip->pmic_ichg_limited) {
			rc = mmi_pl_pm_pmic_ichg_lmt(chip, chip->pmic_ichg_val);
			if (rc < 0)
				mmi_pl_err(chip, "Couldn't recovery ichg, rc=%d\n", rc);
			chip->pmic_ichg_limited = false;
		}

		 if (chip->flashc_recovery_count > 0
			&& chip->flashc_fault
			&& !flashc_fault_now){
				chip->flashc_recovery_count-- ;
		} else if (chip->flashc_recovery_count <= 0
			&& chip->flashc_fault
			&& !flashc_fault_now) {
			mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);
			heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
			chip->flashc_fault = false;
		} else if (chip->pres_temp_zone != ZONE_HOT
				&& chip->pres_temp_zone != ZONE_COLD
				&& !flashc_fault_now
				&& chip->pmic_handle.charge_enabled) {
			mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);
			heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		}

		chip->request_current = TYPEC_HIGH_CURRENT_UA;
		chip->request_volt = SWITCH_CHARGER_PPS_VOLT;
		break;
	}

schedule:

	if(vbat_volt > chip->sys_configs.batt_ovp_lmt) {
		chip->request_volt -= chip->sys_configs.flashc_volt_down_steps;
		mmi_pl_dbg(chip, PR_MOTO, "trigger batt OVP, reduce volt %d, "
				"request volt %d\n",
				chip->sys_configs.flashc_volt_down_steps,
				chip->request_volt);
	}
	chip->target_volt = min(chip->request_volt, chip->pps_voltage_max);
	chip->target_curr = min(chip->request_current, chip->pps_current_max);

	if (chip->thermal_mitigation_level > 0
		&& chip->thermal_mitigation_level < chip->num_thermal_zones - 1
		&& (chip->sm_state == PM_STATE_FLASHC_CC_LOOP
		|| chip->sm_state == PM_STATE_FLASHC_CV_LOOP
		|| chip->sm_state == PM_STATE_SW_LOOP)) {
		chip->target_volt = min(chip->target_volt,
		chip->thermal_mitigation_zones[chip->thermal_mitigation_level].power_voltage);
		chip->target_curr = min(chip->target_curr,
		chip->thermal_mitigation_zones[chip->thermal_mitigation_level].power_current);

		mmi_pl_dbg(chip, PR_MOTO, "thermal level %d, "
					"thermal_voltage %d, thermal_current %d\n",
					chip->thermal_mitigation_level,
		chip->thermal_mitigation_zones[chip->thermal_mitigation_level].power_voltage,
		chip->thermal_mitigation_zones[chip->thermal_mitigation_level].power_current);

		if (chip->force_pmic_chg) {
			chip->force_pmic_chg = false;
			mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);
			heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
			mmi_pl_dbg(chip, PR_MOTO, "thermal level is %d ,"
					"cannel force pmic chg, recovery parallel chg !\n",
					chip->thermal_mitigation_level);
		}
	} else if (chip->thermal_mitigation_level == chip->num_thermal_zones -1
	&& !chip->force_pmic_chg) {
		chip->force_pmic_chg = true;
		mmi_pl_pm_move_state(chip, PM_STATE_SW_ENTRY);
		heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		mmi_pl_dbg(chip, PR_MOTO, "thermal level is %d ,"
				"force to only pmic chg !\n",
				chip->thermal_mitigation_level);
	} else if (chip->thermal_mitigation_level == 0
	&& chip->force_pmic_chg) {
		chip->force_pmic_chg = false;
		mmi_pl_pm_move_state(chip, PM_STATE_ENTRY);
		heartbeat_dely_ms = HEARTBEAT_NEXT_STATE_MS;
		mmi_pl_dbg(chip, PR_MOTO, "thermal level is %d ,"
				"cannel force pmic chg, recovery parallel chg !\n",
				chip->thermal_mitigation_level);
	}

	mmi_pl_dbg(chip, PR_INTERRUPT, "sm work step %s,select power: "
				"target voltage %dmV, target current %dmA,"
				"request voltage %dmV, request current %dmA,"
				"thermal level %d\n",
				pm_state_str[chip->sm_state],chip->target_volt,
				chip->target_curr,
				chip->request_volt,chip->request_current,
				chip->thermal_mitigation_level);

	chip->pps_result = usbpd_select_pdo(chip->pd_handle,
				chip->mmi_pps_pdo_idx,
				chip->target_volt, chip->target_curr);
	mmi_set_pps_result_history(chip, chip->pps_result);

	if (chip->thermal_mitigation_level == 0
		&& !chip->pps_result) {
		chip->request_volt_pre = chip->request_volt;
		chip->request_curr_pre = chip->request_current;
	}

	if (chip->flashc_handle.charge_enabled
		&& chip->pres_chrg_step < STEP_NORMAL
		&& (chip->sm_state == PM_STATE_FLASHC_CC_LOOP
		|| chip->sm_state == PM_STATE_FLASHC_CV_LOOP)
		&& chip->flashc_handle.ibus_curr > FG_ESR_MIN_CURR_UA) {
		if (chip->fg_esr_pulse_timeout >=
			FG_ESR_PULSE_MAX_TIMEOUT) {
			pluse_curr = chip->flashc_handle.ibus_curr % 50000;
			pluse_curr = chip->flashc_handle.ibus_curr - pluse_curr;


			pluse_volt = chip->flashc_handle.vbus_volt % 20000;
			pluse_volt = chip->flashc_handle.vbus_volt - pluse_volt;

			if (chip->sm_state == PM_STATE_FLASHC_CV_LOOP) {

				usbpd_select_pdo(chip->pd_handle,
						chip->mmi_pps_pdo_idx,
						pluse_volt - FG_ESR_DECREMENT_UV,
						pluse_curr - FG_ESR_DECREMENT_UA);
				mmi_pl_dbg(chip, PR_MOTO, "kick a ESR pulse, "
						"pps vbus volt %d "
						"pps ibus curr %d\n",
						pluse_volt - FG_ESR_DECREMENT_UV,
						pluse_curr - FG_ESR_DECREMENT_UA);

			} else {
				usbpd_select_pdo(chip->pd_handle,
						chip->mmi_pps_pdo_idx,
						chip->target_volt,
						pluse_curr - FG_ESR_DECREMENT_UA);
				mmi_pl_dbg(chip, PR_MOTO, "kick a ESR pulse, "
						"pps ibus curr %d\n",
						pluse_curr - FG_ESR_DECREMENT_UA);
				}

			msleep(5000);
			usbpd_select_pdo(chip->pd_handle,
					chip->mmi_pps_pdo_idx,
					chip->target_volt,
					chip->target_curr);
			mmi_pl_dbg(chip, PR_MOTO, "recovery pps curr %d\n",
							chip->target_curr);
			chip->fg_esr_pulse_timeout = 0;
		}
		chip->fg_esr_pulse_timeout +=
			heartbeat_dely_ms;

		if(chip->sm_state == PM_STATE_ENTRY) {
			usbpd_select_pdo(chip->pd_handle,
					chip->mmi_pps_pdo_idx,
					SWITCH_CHARGER_PPS_VOLT,
					TYPEC_HIGH_CURRENT_UA);
			msleep(1000);
			mmi_pl_dbg(chip, PR_MOTO,
				"reset 5V/3A,clear up float voltage\n");
		}

	}

	if (heartbeat_dely_ms >= 0) {
		if (chip->pps_result < 0)
			heartbeat_dely_ms = 0;
		schedule_delayed_work(&chip->mmi_pl_sm_work,
				msecs_to_jiffies(heartbeat_dely_ms));
	} else {
		clear_chg_manager(chip);
		chip->pd_pps_support =  false;
	}

	return;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct mmi_pl_chg_manager *chip = container_of(nb, struct mmi_pl_chg_manager, psy_nb);

	if (!chip->usb_psy) {
		mmi_pl_dbg(chip, PR_MOTO, "usb psy is NULL, direct return\n");
		return NOTIFY_OK;
	}

	if (ptr == chip->usb_psy && evt == PSY_EVENT_PROP_CHANGED)
		schedule_work(&chip->psy_changed_work);

	return NOTIFY_OK;
}

static void psy_changed_work_func(struct work_struct *work)
{
	struct mmi_pl_chg_manager *chip = container_of(work,
				struct mmi_pl_chg_manager, psy_changed_work);
	union power_supply_propval val;
	bool pd_active;
	int ret, i;

	mmi_pl_dbg(chip, PR_MOTO, "kick psy changed work\n");

	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy) {
			mmi_pl_err(chip,
				"Could not get USB power_supply, deferring probe\n");
			return;
		}
	}

	ret = power_supply_get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		mmi_pl_err(chip, "Unable to read USB PRESENT: %d\n", ret);
		return;
	}
	chip->vbus_present = val.intval;

	ret = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_PD_ACTIVE, &val);
	if (ret) {
		mmi_pl_err(chip, "Unable to read PD ACTIVE: %d\n", ret);
		return;
	}
	pd_active = val.intval;

	if (!chip->pd_handle) {
		chip->pd_handle = devm_usbpd_get_by_phandle(chip->dev,
						    "qcom,usbpd-phandle");
		if (IS_ERR_OR_NULL(chip->pd_handle)) {
			mmi_pl_err(chip, "Error getting the pd phandle %ld\n",
				PTR_ERR(chip->pd_handle));
			chip->pd_handle = NULL;
			return;
		}
	}

	if (pd_active && chip->vbus_present) {
		usbpd_get_pdo_info(chip->pd_handle, chip->mmi_pdo_info);
		mmi_pl_dbg(chip, PR_MOTO, "check all effective pdo info\n");
		for (i = 0; i < PD_MAX_PDO_NUM; i++) {
			if ((chip->mmi_pdo_info[i].type ==
					PD_SRC_PDO_TYPE_AUGMENTED)
				&& chip->mmi_pdo_info[i].uv_max >= FLASH_CHARGER_PPS_MIN_VOLT
				&& chip->mmi_pdo_info[i].ua >= TYPEC_MIDDLE_CURRENT_UA) {
					chip->mmi_pps_pdo_idx = chip->mmi_pdo_info[i].pdo_pos;
					mmi_pl_dbg(chip, PR_INTERRUPT,
							"pd charger support pps, pdo %d, "
							"volt %d, curr %d \n",
							chip->mmi_pps_pdo_idx,
							chip->mmi_pdo_info[i].uv_max,
							chip->mmi_pdo_info[i].ua);
					chip->pd_pps_support = true;

					if (chip->mmi_pdo_info[i].uv_max >
							FLASH_CHARGER_PPS_MAX_VOLT) {
						chip->pps_voltage_max = FLASH_CHARGER_PPS_MAX_VOLT;
					} else
						chip->pps_voltage_max =
						chip->mmi_pdo_info[i].uv_max;

					if (chip->mmi_pdo_info[i].ua >
							TYPEC_HIGH_CURRENT_UA) {
						chip->pps_current_max =
							TYPEC_HIGH_CURRENT_UA;
					} else
						chip->pps_current_max =
						chip->mmi_pdo_info[i].ua;
				break;
			}
		}
	}

	mmi_pl_dbg(chip, PR_INTERRUPT, "vbus present %d, pd pps support %d, "
					"pps max voltage %d, pps max curr %d\n",
					chip->vbus_present,
					chip->pd_pps_support,
					chip->pps_voltage_max,
					chip->pps_current_max);

	mmi_pl_pm_check_flashc_enable(chip);
	mmi_pl_pm_check_pmic_enable(chip);
	mmi_pl_dbg(chip, PR_INTERRUPT, "flashc enable %d, pmic enable %d\n",
					chip->flashc_handle.charge_enabled,
					chip->pmic_handle.charge_enabled);

	if (chip->vbus_present && chip->pd_pps_support) {
		kick_sm(chip, 100);
	} else {
		cancel_sm(chip);
		clear_chg_manager(chip);
		chip->pd_pps_support =  false;
	}
	return;
}

#define DEFAULT_FLASHC_BATT_VOLT_LMT		4200000
#define DEFAULT_FLASHC_BATT_CURR_LP_LMT		5500000
#define DEFAULT_BATT_OVP_LMT		4475000
#define DEFAULT_PMIC_CURR_LP_LMT		500000
#define DEFAULT_PMIC_STEP_VOLT		4400000
#define DEFAULT_PMIC_STEP_CURR		2000000
#define DEFAULT_FLASHC_TAPER_CURR_LMT		2000000
#define DEFAULT_FLASHC_VOLT_HYSTERESIS		100000
#define DEFAULT_FLASHC_CURR_HYSTERESIS		300000
#define DEFAULT_FLASHC_VOLT_UP_STEPS			100000
#define DEFAULT_FLASHC_CURR_UP_STEPS			100000
#define DEFAULT_FLASHC_VOLT_DOWN_STEPS		100000
#define DEFAULT_FLASHC_CURR_DOWN_STEPS		250000
#define DEFAULT_FLASHC_MIN_VBAT_START		3600000

static int mmi_pl_chg_manager_parse_dt(struct mmi_pl_chg_manager *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc;
	int byte_len, i;

	rc = of_property_read_u32(node,
				"mmi,batt-ovp-lmt",
				&chip->sys_configs.batt_ovp_lmt);
	if (rc < 0)
		chip->sys_configs.batt_ovp_lmt =
				DEFAULT_BATT_OVP_LMT;

	rc = of_property_read_u32(node,
				"mmi,pmic-curr-lp-lmt",
				&chip->sys_configs.pmic_curr_lp_lmt);
	if (rc < 0)
		chip->sys_configs.pmic_curr_lp_lmt =
				DEFAULT_PMIC_CURR_LP_LMT;

	rc = of_property_read_u32(node,
				"mmi,pmic-step-volt",
				&chip->pmic_step_volt);
	if (rc < 0)
		chip->pmic_step_volt =
				DEFAULT_PMIC_STEP_VOLT;

	rc = of_property_read_u32(node,
				"mmi,pmic-step-curr",
				&chip->pmic_step_curr);
	if (rc < 0)
		chip->pmic_step_curr =
				DEFAULT_PMIC_STEP_CURR;

	rc = of_property_read_u32(node,
				"mmi,flashc-volt-hysteresis",
				&chip->sys_configs.flashc_volt_hysteresis);
	if (rc < 0)
		chip->sys_configs.flashc_volt_hysteresis =
				DEFAULT_FLASHC_VOLT_HYSTERESIS;

	rc = of_property_read_u32(node,
				"mmi,flashc-curr-hysteresis",
				&chip->sys_configs.flashc_curr_hysteresis);
	if (rc < 0)
		chip->sys_configs.flashc_curr_hysteresis =
				DEFAULT_FLASHC_CURR_HYSTERESIS;

	rc = of_property_read_u32(node,
				"mmi,flashc-volt-up-steps",
				&chip->sys_configs.flashc_volt_up_steps);
	if (rc < 0)
		chip->sys_configs.flashc_volt_up_steps =
				DEFAULT_FLASHC_VOLT_UP_STEPS;

	rc = of_property_read_u32(node,
				"mmi,flashc-curr-up-steps",
				&chip->sys_configs.flashc_curr_up_steps);
	if (rc < 0)
		chip->sys_configs.flashc_curr_up_steps =
				DEFAULT_FLASHC_CURR_UP_STEPS;

	rc = of_property_read_u32(node,
				"mmi,flashc-volt-down-steps",
				&chip->sys_configs.flashc_volt_down_steps);
	if (rc < 0)
		chip->sys_configs.flashc_volt_down_steps =
				DEFAULT_FLASHC_VOLT_DOWN_STEPS;

	rc = of_property_read_u32(node,
				"mmi,flashc-curr-down-steps",
				&chip->sys_configs.flashc_curr_down_steps);
	if (rc < 0)
		chip->sys_configs.flashc_curr_down_steps =
				DEFAULT_FLASHC_CURR_DOWN_STEPS;

	rc = of_property_read_u32(node,
				"mmi,flashc-min-vbat-start",
				&chip->sys_configs.flashc_min_vbat_start);
	if (rc < 0)
		chip->sys_configs.flashc_min_vbat_start =
				DEFAULT_FLASHC_MIN_VBAT_START;

	byte_len = of_property_count_strings(node, "flashc-name");
	if (byte_len < 0) {
		mmi_pl_err(chip, "Cannot parse flashc-name: %d\n",
			byte_len);
		return byte_len;
	}

	for (i = 0; i < byte_len; i++) {
		rc = of_property_read_string_index(node, "flashc-name",
			i, &chip->flashc_name);
	}

	if (of_find_property(node, "mmi,mmi-pl-temp-zones", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 7) {
			mmi_pl_err(chip,
				   "DT error wrong mmi temp zones 7\n");
			return -ENODEV;
		}

		chip->temp_zones = (struct mmi_pl_temp_zone *)
			devm_kzalloc(chip->dev, byte_len, GFP_KERNEL);

		if (chip->temp_zones == NULL)
			return -ENOMEM;

		chip->num_temp_zones =
			byte_len / sizeof(struct mmi_pl_temp_zone);

		rc = of_property_read_u32_array(node,
				"mmi,mmi-pl-temp-zones",
				(u32 *)chip->temp_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_pl_err(chip,
				   "Couldn't read mmi temp zones rc = %d\n",
				   rc);
			return rc;
		}
		mmi_pl_dbg(chip, PR_MOTO, "mmi temp zones: Num: %d\n",
			   chip->num_temp_zones);
		for (i = 0; i < chip->num_temp_zones; i++) {
			   chip->temp_zones[i].super_uv *= 1000;
			   chip->temp_zones[i].fcc_super_ua *= 1000;
			   chip->temp_zones[i].high_uv *= 1000;
			   chip->temp_zones[i].fcc_high_ua *= 1000;
			   chip->temp_zones[i].norm_uv *= 1000;
			   chip->temp_zones[i].fcc_norm_ua  *= 1000;

			mmi_pl_dbg(chip, PR_MOTO,
				   "mmi temp zones: Zone %d, Temp %d C, " \
				   "super_mv %d, fcc super ma %d, " \
					"high_mv %d, fcc_high_mv %d, " \
					"norm_mv %d, fcc_norm_ma %d\n", i,
				   chip->temp_zones[i].temp_c,
				   chip->temp_zones[i].super_uv,
				   chip->temp_zones[i].fcc_super_ua,
				   chip->temp_zones[i].high_uv,
				   chip->temp_zones[i].fcc_high_ua,
				   chip->temp_zones[i].norm_uv,
				   chip->temp_zones[i].fcc_norm_ua);
		}
		chip->pres_temp_zone = ZONE_NONE;
	}

	if (of_find_property(node, "mmi,mmi-thermal-zones", &byte_len)) {
		if ((byte_len / sizeof(u32)) % 2) {
			mmi_pl_err(chip,
				   "DT error wrong mmi thermal zones 2\n");
			return -ENODEV;
		}

		chip->thermal_mitigation_zones = (struct mmi_thermal_power_zone *)
			devm_kzalloc(chip->dev, byte_len, GFP_KERNEL);

		if (chip->thermal_mitigation_zones == NULL)
			return -ENOMEM;

		chip->num_thermal_zones =
			byte_len / sizeof(struct mmi_thermal_power_zone);

		rc = of_property_read_u32_array(node,
				"mmi,mmi-thermal-zones",
				(u32 *)chip->thermal_mitigation_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_pl_err(chip,
				   "Couldn't read mmi thermal zones rc = %d\n",
				   rc);
			return rc;
		}
		mmi_pl_dbg(chip, PR_MOTO, "mmi thermal zones: Num: %d\n",
			   chip->num_thermal_zones);
		for (i = 0; i < chip->num_thermal_zones; i++) {

			mmi_pl_dbg(chip, PR_MOTO,
				   "mmi thermal mitigation zones: Zone %d, " \
				   "power_current %d, " \
					"power_voltage %d\n", i,
				   chip->thermal_mitigation_zones[i].power_current,
				   chip->thermal_mitigation_zones[i].power_voltage);
		}
		chip->thermal_mitigation_level = 0;
	}

	return rc;
}

static ssize_t flashc_vbat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;
	union power_supply_propval prop = {0,};
	struct mmi_pl_chg_manager *chip = dev_get_drvdata(dev);

	chip->flashc_psy = power_supply_get_by_name(chip->flashc_name);
	if (!chip->flashc_psy)
		return rc;

	rc = power_supply_get_property(chip->flashc_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (rc) {
		power_supply_put(chip->flashc_psy);
		return rc;
	}

	power_supply_put(chip->flashc_psy);

	return scnprintf(buf, FLASHC_SHOW_MAX_SIZE, "%d\n", prop.intval*1000);
}
static DEVICE_ATTR(flashc_vbat, S_IRUGO, flashc_vbat_show, NULL);

static ssize_t factory_flashc_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int state;
	struct mmi_pl_chg_manager *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	mmi_pl_pm_check_flashc_enable(chip);

	state = (chip->flashc_handle.charge_enabled) ? 1 : 0;
	return scnprintf(buf, FLASHC_SHOW_MAX_SIZE, "%d\n", state);
}

static ssize_t factory_flashc_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long enable;
	struct mmi_pl_chg_manager *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &enable);
	if (r) {
		mmi_pl_err(chip, "Invalid enable value = %ld\n", enable);
		return -EINVAL;
	}

	mmi_pl_pm_flashc_enable(chip, enable);
	mmi_pl_pm_check_flashc_enable(chip);

	return r ? r : count;
}

static DEVICE_ATTR(factory_flashc_enable,
			0664, factory_flashc_enable_show, factory_flashc_enable_store);

static ssize_t factory_pmic_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int state;
	struct mmi_pl_chg_manager *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	mmi_pl_pm_check_pmic_enable(chip);

	state = (chip->pmic_handle.charge_enabled) ? 1 : 0;
	return scnprintf(buf, FLASHC_SHOW_MAX_SIZE, "%d\n", state);
}

static ssize_t factory_pmic_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long enable;
	struct mmi_pl_chg_manager *chip = dev_get_drvdata(dev);

	r = kstrtoul(buf, 0, &enable);
	if (r) {
		mmi_pl_err(chip, "Invalid enable value = %ld\n", enable);
		return -EINVAL;
	}

	mmi_pl_pm_pmic_enable(chip, enable);
	mmi_pl_pm_check_pmic_enable(chip);

	return r ? r : count;
}

static DEVICE_ATTR(factory_pmic_enable,
			0664, factory_pmic_enable_show, factory_pmic_enable_store);

static void create_sysfs_entries(struct mmi_pl_chg_manager *chip)
{
	int rc;
	rc = device_create_file(chip->dev, &dev_attr_flashc_vbat);
	if (rc)
		mmi_pl_err(chip, "Couldn't create flashc_vbat\n");

	rc = device_create_file(chip->dev, &dev_attr_factory_flashc_enable);
	if (rc)
		mmi_pl_err(chip, "Couldn't create factory_flashc_enable\n");

	rc = device_create_file(chip->dev, &dev_attr_factory_pmic_enable);
	if (rc)
		mmi_pl_err(chip, "Couldn't create factory_pmic_enable\n");
}

static void remove_sysfs_entries(struct mmi_pl_chg_manager *chip)
{
	device_remove_file(chip->dev, &dev_attr_flashc_vbat);
	device_remove_file(chip->dev, &dev_attr_factory_flashc_enable);
	device_remove_file(chip->dev, &dev_attr_factory_pmic_enable);
}

static enum power_supply_property mmi_pl_pm_props[] = {
	POWER_SUPPLY_PROP_PD_CURRENT_MAX,
	POWER_SUPPLY_PROP_PD_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PARALLEL_DISABLE,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_NUM_SYSTEM_TEMP_LEVELS,
};

static int mmi_pl_pm_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct mmi_pl_chg_manager *chip  = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
		val->intval = chip->request_current;
		break;
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MAX:
		val->intval = chip->request_volt;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = chip->target_curr;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED:
		val->intval = chip->target_volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = chip->pmic_handle.ibat_curr;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->pmic_handle.vbat_volt;
		break;
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
		val->intval = !chip->flashc_handle.charge_enabled;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->thermal_mitigation_level;
		break;
	case POWER_SUPPLY_PROP_NUM_SYSTEM_TEMP_LEVELS:
		val->intval = chip->num_thermal_zones;
		break;
	default:
		return -EINVAL;

	}
	return 0;
}

static int mmi_pl_pm_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct mmi_pl_chg_manager *chip  = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		if (val->intval < 0)
			return -EINVAL;

		if (chip->num_thermal_zones<= 0)
			return -EINVAL;

		if (val->intval >= chip->num_thermal_zones)
			chip->thermal_mitigation_level =
				chip->num_thermal_zones - 1;
		else
			chip->thermal_mitigation_level = val->intval;

		break;
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
		if (val->intval > TYPEC_HIGH_CURRENT_UA)
			return -EINVAL;
		chip->request_current = val->intval;
		break;
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MAX:
		if (val->intval > FLASH_CHARGER_PPS_MAX_VOLT)
			return -EINVAL;
		chip->request_volt = val->intval;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int mmi_pl_pm_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static const struct power_supply_desc mmi_pl_pm_psy_desc = {
	.name = "mmi_pl_chg_manager",
	.type = POWER_SUPPLY_TYPE_PARALLEL,
	.properties = mmi_pl_pm_props,
	.num_properties = ARRAY_SIZE(mmi_pl_pm_props),
	.get_property = mmi_pl_pm_get_property,
	.set_property = mmi_pl_pm_set_property,
	.property_is_writeable = mmi_pl_pm_is_writeable,
};

static int mmi_pl_pm_psy_register(struct mmi_pl_chg_manager *chip)
{
	struct power_supply_config mmi_pl_pm_cfg = {};

	mmi_pl_pm_cfg.drv_data = chip;
	mmi_pl_pm_cfg.of_node = chip->dev->of_node;
	chip->mmi_pl_pm_psy = power_supply_register(chip->dev,
						&mmi_pl_pm_psy_desc,
						&mmi_pl_pm_cfg);
	if (IS_ERR(chip->mmi_pl_pm_psy)) {
		pr_err("Couldn't register mmi_pl_pm_psy power supply\n");
		return PTR_ERR(chip->mmi_pl_pm_psy);
	}

	pr_info("power supply register mmi_pl_pm_psy successfully\n");
	return 0;
}

static int mmi_pl_chg_manager_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mmi_pl_chg_manager *chip;

	if (!pdev)
		return -ENODEV;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct mmi_pl_chg_manager),
								GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for mmi_pl_chg_manager\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->name = "mmi_pl_chg_manager";
	chip->debug_mask = &__debug_mask;
	ret = mmi_pl_chg_manager_parse_dt(chip);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"parse dt failed\n");
		goto cleanup;
	}

	chip->pd_handle =
			devm_usbpd_get_by_phandle(chip->dev, "qcom,usbpd-phandle");
	if (IS_ERR_OR_NULL(chip->pd_handle)) {
		dev_err(&pdev->dev, "Error getting the pd phandle %ld\n",
							PTR_ERR(chip->pd_handle));
		chip->pd_handle = NULL;
	}

	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy)
			mmi_pl_err(chip, "Could not get USB power_supply\n");
	}

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			mmi_pl_err(chip, "Could not get battery power_supply\n");
	}

	INIT_DELAYED_WORK(&chip->mmi_pl_sm_work, mmi_pl_sm_work_func);
	INIT_WORK(&chip->psy_changed_work, psy_changed_work_func);

	ret = mmi_pl_pm_psy_register(chip);
	if (ret)
		goto cleanup;

	chip->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_nb);
	if (ret)
		goto cleanup;

	init_completion(&chip->sm_completion);
	platform_set_drvdata(pdev, chip);

	chip->pd_handle =
				devm_usbpd_get_by_phandle(chip->dev,
						"qcom,usbpd-phandle");
	if (IS_ERR_OR_NULL(chip->pd_handle)) {
		dev_err(&pdev->dev,
				"Error getting the pd phandle %ld\n",
				PTR_ERR(chip->pd_handle));
		chip->pd_handle = NULL;
	}

	create_sysfs_entries(chip);
	mmi_pl_err(chip, "mmi pl chg manager initialized successfully, ret %d\n", ret);
	return 0;
cleanup:

	devm_kfree(&pdev->dev, chip);
	return ret;
}

static int mmi_pl_chg_manager_remove(struct platform_device *pdev)
{
	struct mmi_pl_chg_manager *chip =  platform_get_drvdata(pdev);

	remove_sysfs_entries(chip);
	cancel_delayed_work_sync(&chip->mmi_pl_sm_work);

	power_supply_unreg_notifier(&chip->psy_nb);
	power_supply_unregister(chip->mmi_pl_pm_psy );

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);
	return 0;
}

static const struct of_device_id mmi_pl_chg_manager_match_table[] = {
	{.compatible = "mmi,pl-chg-manager"},
	{},
};
MODULE_DEVICE_TABLE(of, mmi_pl_chg_manager_match_table);

static struct platform_driver mmi_pl_chg_manager_driver = {
	.probe = mmi_pl_chg_manager_probe,
	.remove = mmi_pl_chg_manager_remove,
	.driver = {
		.name = "mmi_pl_chg_manager",
		.owner = THIS_MODULE,
		.of_match_table = mmi_pl_chg_manager_match_table,
	},
};

static int __init mmi_pl_chg_manager_init(void)
{
	int ret;
	ret = platform_driver_register(&mmi_pl_chg_manager_driver);
	if (ret) {
		pr_err("mmi_pl_chg_manager failed to register driver\n");
		return ret;
	}
	return 0;
}

static void __exit mmi_pl_chg_manager_exit(void)
{
	platform_driver_unregister(&mmi_pl_chg_manager_driver);
}

module_init(mmi_pl_chg_manager_init);
module_exit(mmi_pl_chg_manager_exit);

MODULE_ALIAS("platform:mmi_pl_chg_manager");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("Motorola Mobility parallel charge");
MODULE_LICENSE("GPL");
