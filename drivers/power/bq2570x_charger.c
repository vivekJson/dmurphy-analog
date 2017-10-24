/*
 * BQ2570x battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"bq2570x: %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>

#include "bq2570x_reg.h"
#include "bq2570x.h"

#if 1
#undef pr_debug
#define pr_debug pr_err
#undef pr_info
#define pr_info pr_err
#undef dev_dbg
#define dev_dbg dev_err
#else
#undef pr_info
#define pr_info pr_debug
#endif

enum {
	USER = BIT(0),
	BATT = BIT(1),		/* Batt FULL */
};

enum {
	BQ25703 = 0x00,
};

enum {
	CHARGE_STATE_NOT_CHARGING,
	CHARGE_STATE_PRECHARGE,
	CHARGE_STATE_FASTCHARGE,
	CHARGE_STATE_OTG,
};

struct bq2570x_otg_regulator {
	struct regulator_desc rdesc;
	struct regulator_dev *rdev;
};

struct bq2570x {
	struct device *dev;
	struct i2c_client *client;

	int manufacture_id;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex profile_change_lock;
	struct mutex charging_disable_lock;
	struct mutex usb_online_lock;
	struct mutex adc_lock;

	bool batt_present;
	bool usb_present;
	bool usb_online;

	bool vbus_present;

	bool batt_full;

	bool charge_enabled;	/* Register bit status */
	bool otg_enabled;

	bool vindpm_triggered;
	bool iindpm_triggered;

	bool in_therm_regulation;
	bool in_vsys_regulation;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int psys_volt;
	int cmpin_volt;
	int ibus_current;
	int charge_current;
	int discharge_current;

	int batt_temp;

	int usb_psy_ma;

	int icl_ma;
	int ivl_mv;

	int chg_ma;
	int chg_mv;

	int charge_state;
	int charging_disabled_status;

	int fault_status;
	uint8_t prochot_status;

	int skip_writes;
	int skip_reads;

	struct bq2570x_platform_data *platform_data;

	struct delayed_work monitor_work;

	struct dentry *debug_root;

	struct bq2570x_otg_regulator otg_vreg;
	int otg_gpio;

	enum power_supply_type supply_type;
	int typec_mode;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply *typec_psy;
	struct power_supply batt_psy;
};

static void set_usb_present(struct bq2570x *bq);
static void set_usb_online(struct bq2570x *bq);

static int bq2570x_update_charging_profile(struct bq2570x *bq);

/************************************************************************/

static int __bq2570x_read_byte(struct bq2570x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __bq2570x_write_byte(struct bq2570x *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int __bq2570x_read_word(struct bq2570x *bq, u8 reg, u16 *data)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}

static int __bq2570x_write_word(struct bq2570x *bq, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%04X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq2570x_read_byte(struct bq2570x *bq, u8 *data, u8 reg)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2570x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#if 0
static int bq2570x_read_word(struct bq2570x *bq, u16 *data, u8 reg)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2570x_read_word(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}
#endif

static int bq2570x_update_bits_byte(struct bq2570x *bq, u8 reg,
				    u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2570x_read_byte(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2570x_write_byte(bq, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2570x_update_bits_word(struct bq2570x *bq, u8 reg,
				    u16 mask, u16 data)
{
	int ret;
	u16 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2570x_read_word(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2570x_write_word(bq, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

/*********************************************************************/

static int bq2570x_enable_charger(struct bq2570x *bq)
{
	int ret;
	u8 val = CHRG_ENABLE << CHRG_INHIBIT_SHIFT;

	ret =
	    bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG, CHRG_INHIBIT_MASK,
				     val);

	return ret;
}

static int bq2570x_disable_charger(struct bq2570x *bq)
{
	int ret;
	u8 val = CHRG_INHIBIT << CHRG_INHIBIT_SHIFT;

	ret =
	    bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG, CHRG_INHIBIT_MASK,
				     val);
	return ret;
}

static int bq2570x_set_chargecurrent(struct bq2570x *bq, uint16_t curr)
{
	u16 ichg;

	if (curr < CHARGECURRENT_BASE)
		curr = CHARGECURRENT_BASE;

	ichg = (curr - CHARGECURRENT_BASE) / CHARGECURRENT_LSB;
	return bq2570x_update_bits_word(bq, CHARGECURRENT_REG,
					CHARGECURRENT_MASK,
					ichg << CHARGECURRENT_SHIFT);
}

static int bq2570x_set_chargevolt(struct bq2570x *bq, uint16_t volt)
{
	u16 val;

	if (volt < CHARGEVOLT_BASE)
		volt = CHARGEVOLT_BASE;

	val = (volt - CHARGEVOLT_BASE) / CHARGEVOLT_LSB;
	return bq2570x_update_bits_word(bq, CHARGEVOLT_REG, CHARGEVOLT_MASK,
					val << CHARGEVOLT_SHIFT);
}

static int bq2570x_set_input_volt_limit(struct bq2570x *bq, int volt)
{
	u16 val;

	if (volt < INPUTVOLTLIM_BASE)
		volt = INPUTVOLTLIM_BASE;

	val = (volt - INPUTVOLTLIM_BASE) / INPUTVOLTLIM_LSB;

	return bq2570x_update_bits_word(bq, INPUTVOLTLIM_REG, INPUTVOLTLIM_MASK,
					val << INPUTVOLTLIM_SHIFT);
}

static int bq2570x_set_input_current_limit(struct bq2570x *bq, int curr)
{
	u8 val;

	if (curr < INPUTCURRENTLIM_BASE)
		curr =  INPUTCURRENTLIM_BASE;

	val = (curr - INPUTCURRENTLIM_BASE) / INPUTCURRENTLIM_LSB;
	return bq2570x_update_bits_byte(bq, INPUTCURRENTLIM_REG,
					INPUTCURRENTLIM_MASK,
					val << INPUTCURRENTLIM_SHIFT);
}

int bq2570x_set_watchdog_timer(struct bq2570x *bq, u8 timeout)
{
	u8 temp;

	temp = (u8) timeout << WDTMR_ADJ_SHIFT;
	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_1_REG, WDTMR_ADJ_MASK,
					temp);
}
EXPORT_SYMBOL_GPL(bq2570x_set_watchdog_timer);

int bq2570x_disable_watchdog_timer(struct bq2570x *bq)
{
	u8 val = WDTMR_ADJ_DISABLE << WDTMR_ADJ_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_1_REG, WDTMR_ADJ_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_disable_watchdog_timer);

int bq2570x_reset_chip(struct bq2570x *bq)
{
	int ret;
	u8 val = RESET_REG << RESET_REG_SHIFT;

	ret =
	    bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG, RESET_REG_MASK,
				     val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2570x_reset_chip);

int bq2570x_enter_hiz_mode(struct bq2570x *bq)
{
	u8 val = HIZ_MODE_ENABLE << EN_HIZ_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG, EN_HIZ_MASK,
					val);

}
EXPORT_SYMBOL_GPL(bq2570x_enter_hiz_mode);

int bq2570x_exit_hiz_mode(struct bq2570x *bq)
{
	u8 val = HIZ_MODE_DISABLE << EN_HIZ_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG, EN_HIZ_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_exit_hiz_mode);

int bq2570x_get_hiz_mode(struct bq2570x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2570x_read_byte(bq, &val, CHARGEOPTION3_1_REG);
	if (ret)
		return ret;
	*state = (val & EN_HIZ_MASK) >> EN_HIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2570x_get_hiz_mode);

int bq2570x_reset_vindpm(struct bq2570x *bq)
{
	u8 val;
	int ret;
	int retry = 0;

	val = RESET_VINDPM << RESET_VINDPM_SHIFT;

	ret = bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG,
				RESET_VINDPM_MASK, val);
	while (retry++ < 10) {
		msleep(5);
		ret = bq2570x_read_byte(bq, &val, CHARGEOPTION3_1_REG);
		if (!ret && !(val & RESET_VINDPM_MASK))
			break;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(bq2570x_reset_vindpm);

int bq2570x_set_otg_current(struct bq2570x *bq, int curr)
{
	u8 val;

	val = (curr - OTGCURRENT_BASE) / OTGCURRENT_LSB;

	return bq2570x_update_bits_byte(bq, OTGCURRENT_REG, OTGCURRENT_MASK,
					val << OTGCURRENT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2570x_set_otg_current);

int bq2570x_set_otg_voltage(struct bq2570x *bq, int volt)
{
	u16 val;

	val = (volt - OTGVOLT_BASE) / OTGVOLT_LSB;
	return bq2570x_update_bits_word(bq, OTGVOLT_REG, OTGVOLT_MASK,
					val << OTGVOLT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2570x_set_otg_voltage);

int bq2570x_otg_enable(struct bq2570x *bq)
{
	const u8 val = OTG_ENABLE << EN_OTG_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG, EN_OTG_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_otg_enable);

int bq2570x_otg_disable(struct bq2570x *bq)
{
	const u8 val = OTG_DISABLE << EN_OTG_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG, EN_OTG_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_otg_disable);

int bq2570x_ico_enable(struct bq2570x *bq)
{
	const u8 val = ICO_MODE_ENABLE << EN_ICO_MODE_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG,
					EN_ICO_MODE_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2570x_ico_enable);

int bq2570x_ico_disable(struct bq2570x *bq)
{
	const u8 val = ICO_MODE_DISABLE << EN_ICO_MODE_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION3_1_REG,
					EN_ICO_MODE_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2570x_ico_disable);

int bq2570x_idpm_enable(struct bq2570x *bq)
{
	const u8 val = IDPM_ENABLE << EN_IDPM_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG, EN_IDPM_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_idpm_enable);

int bq2570x_idpm_disable(struct bq2570x *bq)
{
	const u8 val = IDPM_DISABLE << EN_IDPM_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG, EN_IDPM_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_idpm_disable);

int bq2570x_ldo_mode_enable(struct bq2570x *bq)
{
	const u8 val = LDO_MODE_ENABLE << EN_LDO_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG, EN_LDO_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_ldo_mode_enable);

int bq2570x_ldo_mode_disable(struct bq2570x *bq)
{
	const u8 val = LDO_MODE_DISABLE << EN_LDO_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG, EN_LDO_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_ldo_mode_disable);

int bq2570x_set_iadpt_gain(struct bq2570x *bq, int gain)
{
	u8 val;

	if (gain == 20)
		val = IADPT_GAIN_20X;
	else if (gain == 40)
		val = IADPT_GAIN_40X;
	else
		return -EINVAL;

	val <<= IADPT_GAIN_SHIFT;
	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG,
					IADPT_GAIN_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2570x_set_iadpt_gain);

int bq2570x_set_ibat_gain(struct bq2570x *bq, int gain)
{
	u8 val;

	if (gain == 8)
		val = IBAT_GAIN_8X;
	else if (gain == 16)
		val = IBAT_GAIN_16X;
	else
		return -EINVAL;

	val <<= IBAT_GAIN_SHIFT;
	return bq2570x_update_bits_byte(bq, CHARGEOPTION0_0_REG, IBAT_GAIN_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_set_ibat_gain);

int bq2570x_ibat_enable(struct bq2570x *bq)
{
	const u8 val = IBAT_ENABLE << EN_IBAT_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_1_REG, EN_IBAT_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_ibat_enable);

int bq2570x_ibat_disable(struct bq2570x *bq)
{
	const u8 val = IBAT_DISABLE << EN_IBAT_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_1_REG, EN_IBAT_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_ibat_disable);

int bq2570x_psys_enable(struct bq2570x *bq)
{
	const u8 val = PSYS_ENABLE << EN_PSYS_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_1_REG, EN_PSYS_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_psys_enable);

int bq2570x_psys_disable(struct bq2570x *bq)
{
	const u8 val = PSYS_DISABLE << EN_PSYS_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_1_REG, EN_PSYS_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_psys_disable);

int bq2570x_set_rsns_rac(struct bq2570x *bq, uint8_t resistor)
{
	u8 val;

	if (resistor == 10)
		val = RSNS_RAC_10MOHM;
	else if (resistor == 20)
		val = RSNS_RAC_20MOHM;
	else
		return -EINVAL;
	val <<= RSNS_RAC_SHIFT;
	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_1_REG, RSNS_RAC_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_set_rsns_rac);

int bq2570x_set_rsns_rsr(struct bq2570x *bq, uint8_t resistor)
{
	u8 val;

	if (resistor == 10)
		val = RSNS_RSR_10MOHM;
	else if (resistor == 20)
		val = RSNS_RSR_20MOHM;
	else
		return -EINVAL;
	val <<= RSNS_RSR_SHIFT;
	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_1_REG, RSNS_RSR_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_set_rsns_rsr);

int bq2570x_auto_wakeup_enable(struct bq2570x *bq)
{
	const u8 val = AUTO_WAKEUP_ENABLE << AUTO_WAKEUP_EN_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_0_REG,
					AUTO_WAKEUP_EN_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2570x_auto_wakeup_enable);

int bq2570x_auto_wakeup_disable(struct bq2570x *bq)
{
	const u8 val = AUTO_WAKEUP_DISABLE << AUTO_WAKEUP_EN_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION1_0_REG,
					AUTO_WAKEUP_EN_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2570x_auto_wakeup_disable);

int bq2570x_acoc_enable(struct bq2570x *bq)
{
	const u8 val = ACOC_ENABLE << EN_ACOC_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION2_0_REG, EN_ACOC_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_acoc_enable);

int bq2570x_acoc_disable(struct bq2570x *bq)
{
	const u8 val = ACOC_DISABLE << EN_ACOC_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION2_0_REG, EN_ACOC_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_acoc_disable);

int bq2570x_set_acoc_vth(struct bq2570x *bq, int vth)
{
	u8 val;

	if (vth == 125)
		val = ACOC_VTH_125PCT;
	else if (vth == 210)
		val = ACOC_VTH_210PCT;
	else
		return -EINVAL;
	return bq2570x_update_bits_byte(bq, CHARGEOPTION2_0_REG, ACOC_VTH_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_set_acoc_vth);

int bq2570x_batoc_enable(struct bq2570x *bq)
{
	const u8 val = BATOC_ENABLE << EN_BATOC_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION2_0_REG, EN_BATOC_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_batoc_enable);

int bq2570x_batoc_disable(struct bq2570x *bq)
{
	const u8 val = BATOC_DISABLE << EN_BATOC_SHIFT;

	return bq2570x_update_bits_byte(bq, CHARGEOPTION2_0_REG, EN_BATOC_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_batoc_disable);

int bq2570x_set_batoc_vth(struct bq2570x *bq, int vth)
{
	u8 val;

	if (vth == 125)
		val = BATOC_VTH_125PCT;
	else if (vth == 200)
		val = BATOC_VTH_200PCT;
	else
		return -EINVAL;
	return bq2570x_update_bits_byte(bq, CHARGEOPTION2_0_REG, BATOC_VTH_MASK,
					val);
}
EXPORT_SYMBOL_GPL(bq2570x_set_batoc_vth);

int bq2570x_adc_start(struct bq2570x *bq, bool oneshot)
{
	u8 val, mask;
	int ret;

	mutex_lock(&bq->adc_lock);
	if (oneshot) {
		val = ADC_START << ADC_START_SHIFT;
		mask = ADC_START_MASK;
	} else {
		val = ADC_CONV_CONTINUOUS << ADC_CONV_SHIFT;
		mask = ADC_CONV_MASK;
	}

	ret = bq2570x_update_bits_byte(bq, ADCOPTION_1_REG, mask, val);

	mutex_unlock(&bq->adc_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2570x_adc_start);

int bq2570x_adc_stop(struct bq2570x *bq)
{
	u8 val, mask;
	int ret;

	mutex_lock(&bq->adc_lock);
	val = ADC_CONV_ONESHOT << ADC_CONV_SHIFT;
	mask = ADC_CONV_MASK;

	ret = bq2570x_update_bits_byte(bq, ADCOPTION_1_REG, mask, val);

	mutex_unlock(&bq->adc_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2570x_adc_stop);

int bq2570x_set_adc_fullscale(struct bq2570x *bq, int scale)
{
	u8 val = 0;

	if (scale == 204)
		val = ADC_FULLSCALE_2P04V;
	else if (scale == 306)
		val = ADC_FULLSCALE_3P06V;

	val <<= ADC_FULLSCALE_SHIFT;

	return bq2570x_update_bits_byte(bq, ADCOPTION_1_REG,
					ADC_FULLSCALE_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2570x_set_adc_fullscale);

int bq2570x_adc_channel_enable(struct bq2570x *bq, uint8_t mask)
{

	return bq2570x_update_bits_byte(bq, ADCOPTION_0_REG, 0xFF, mask);
}
EXPORT_SYMBOL_GPL(bq2570x_adc_channel_enable);

static int bq2570x_read_vbus_volt(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCVBUS_REG);
	if (!ret) {
		raw >>= ADCVBUS_SHIFT;
		raw &= ADCVBUS_MASK;
		bq->vbus_volt = (uint16_t) raw * ADCVBUS_LSB + ADCVBUS_BASE;
	}

	return ret;
}

#if 0
static int bq2570x_read_psys_volt(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCPSYS_REG);
	if (!ret) {
		raw >>= ADCPSYS_SHIFT;
		raw &= ADCPSYS_MASK;
		bq->psys_volt = (uint16_t) raw * ADCPSYS_LSB + ADCPSYS_BASE;
	}

	return ret;
}
#endif

static int bq2570x_read_charge_current(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCIBAT_CHG_REG);
	if (!ret) {
		raw >>= ADCIBAT_CHG_SHIFT;
		raw &= ADCIBAT_CHG_MASK;
		bq->charge_current = (uint16_t) raw * ADCIBAT_CHG_LSB
		    + ADCIBAT_CHG_BASE;
	}

	return ret;
}

static int bq2570x_read_discharge_current(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCIBAT_DSG_REG);
	if (!ret) {
		raw >>= ADCIBAT_DSG_SHIFT;
		raw &= ADCIBAT_DSG_MASK;
		bq->discharge_current = (uint16_t) raw * ADCIBAT_DSG_LSB
		    + ADCIBAT_DSG_BASE;
	}

	return ret;
}

static int bq2570x_read_vsys_volt(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCVSYS_REG);
	if (!ret) {
		raw >>= ADCVSYS_SHIFT;
		raw &= ADCVSYS_MASK;
		bq->vsys_volt = (uint16_t) raw * ADCVSYS_LSB + ADCVSYS_BASE;
	}

	return ret;
}

static int bq2570x_read_vbat_volt(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCVBAT_REG);
	if (!ret) {
		raw >>= ADCVBAT_SHIFT;
		raw &= ADCVBAT_MASK;
		bq->vbat_volt = (uint16_t) raw * ADCVBAT_LSB + ADCVBAT_BASE;
	}

	return ret;
}

static int bq2570x_read_ibus_current(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCIBUS_REG);
	if (!ret) {
		raw >>= ADCIBUS_SHIFT;
		raw &= ADCIBUS_MASK;
		bq->ibus_current = (uint16_t) raw * ADCIBUS_LSB + ADCIBUS_BASE;
	}

	return ret;
}

#if 0
static int bq2570x_read_cmpin_volt(struct bq2570x *bq)
{
	int ret;
	u8 raw;

	ret = bq2570x_read_byte(bq, &raw, ADCCMPIN_REG);
	if (!ret) {
		raw >>= ADCCMPIN_SHIFT;
		raw &= ADCCMPIN_MASK;
		bq->cmpin_volt = (uint16_t) raw * ADCCMPIN_LSB + ADCCMPIN_BASE;
	}

	return ret;
}
#endif

static int bq2570x_get_vbus_present(struct bq2570x *bq)
{
	int ret;
	u8 stat;

	ret = bq2570x_read_byte(bq, &stat, CHARGERSTATUS_1_REG);
	if (!ret)
		bq->vbus_present = stat & CHARGERSTATUS_AC_PRESENT;

	return ret;
}

static int bq2570x_get_charge_state(struct bq2570x *bq)
{
	int ret;
	u8 mode;

	ret = bq2570x_read_byte(bq, &mode, CHARGERSTATUS_1_REG);
	if (!ret) {
		if (mode & CHARGERSTATUS_IN_FCHRG)
			bq->charge_state = CHARGE_STATE_FASTCHARGE;
		else if (mode & CHARGERSTATUS_IN_PCHRG)
			bq->charge_state = CHARGE_STATE_PRECHARGE;
		else if (mode & CHARGERSTATUS_IN_OTG)
			bq->charge_state = CHARGE_STATE_OTG;
		else
			bq->charge_state = CHARGE_STATE_NOT_CHARGING;
	}
	return ret;
}

static int bq2570x_get_fault_status(struct bq2570x *bq)
{
	int ret;
	u8 stat;

	ret = bq2570x_read_byte(bq, &stat, CHARGERSTATUS_0_REG);
	if (!ret)
		bq->fault_status = stat;
	return ret;
}

#if 0
static int bq2570x_get_prochot_status(struct bq2570x *bq)
{
	int ret;
	u8 stat;

	ret = bq2570x_read_byte(bq, &stat, PROCHOTSTATUS_REG);
	if (!ret)
		bq->prochot_status = stat;
	return ret;

}
#endif

static int bq2570x_charging_disable(struct bq2570x *bq, int reason, int disable)
{

	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
		reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2570x_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2570x_enable_charger(bq);

	if (ret) {
		pr_err
		    ("Couldn't disable/enable charging for reason=%d ret=%d\n",
		     ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	set_usb_online(bq);

	return ret;
}
#if 0
static struct power_supply *get_bms_psy(struct bq2570x *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;

	bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy) {
		pr_debug("bms power supply not found\n");
		return NULL;
	}

	return bq->bms_psy;
}
#endif

static int purify_voltage_now(int in_mv)
{
	int ret;

	if (in_mv == 5000)
		ret = in_mv - 600;
	else
		ret = in_mv - 1280;

	if (ret < 4400)
		ret = 4400;

	return ret;
}

static int bq2570x_get_batt_property(struct bq2570x *bq,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret;

	if (bq->bms_psy == NULL) {
		pr_err("%s: bms power supply is NULL\n", __func__);
		return -ENODEV;
	}

	ret = bq->bms_psy->get_property(bq->bms_psy, psp, val);

	return ret;
}

static int bq2570x_get_prop_charge_type(struct bq2570x *bq)
{

	switch (bq->charge_state) {
	case CHARGE_STATE_FASTCHARGE:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHARGE_STATE_PRECHARGE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHARGE_STATE_NOT_CHARGING:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

#if 0
static int bq2570x_get_prop_batt_present(struct bq2570x *bq)
{
	union power_supply_propval batt_prop = { 0, };
	int ret;

	ret = bq2570x_get_batt_property(bq,
					POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret)
		bq->batt_present = batt_prop.intval;

	return ret;

}

static int bq2570x_get_prop_batt_full(struct bq2570x *bq)
{
	union power_supply_propval batt_prop = { 0, };
	int ret;

	ret = bq2570x_get_batt_property(bq,
					POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret)
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);

	return ret;
}
#endif

static int bq2570x_get_prop_charge_status(struct bq2570x *bq)
{
	union power_supply_propval batt_prop = { 0, };
	int ret;

	ret = bq2570x_get_batt_property(bq,
					POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL)
		return POWER_SUPPLY_STATUS_FULL;

	if (bq->charge_state == CHARGE_STATE_FASTCHARGE ||
	    bq->charge_state == CHARGE_STATE_PRECHARGE)
		return POWER_SUPPLY_STATUS_CHARGING;
	else if (bq->charge_state == CHARGE_STATE_NOT_CHARGING)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (!bq->vbus_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else
		return POWER_SUPPLY_STATUS_UNKNOWN;

}

static enum power_supply_property bq2570x_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
/*	POWER_SUPPLY_PROP_TIME_TO_EMPTY,*/
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
	POWER_SUPPLY_PROP_TYPEC_MODE,
	POWER_SUPPLY_PROP_TYPE,

};

static int bq2570x_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{

	struct bq2570x *bq = container_of(psy, struct bq2570x, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2570x_get_prop_charge_type(bq);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 4800000; /* 4800 mAh */
		break;

	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = !bq->charging_disabled_status;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2570x_get_prop_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		return bq2570x_get_batt_property(bq, psp, val);
	default:
		return -EINVAL;

	}

	return 0;
}

static int bq2570x_charger_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	struct bq2570x *bq = container_of(psy,
					  struct bq2570x, batt_psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2570x_charging_disable(bq, USER, !val->intval);

		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		pr_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
			val->intval ? "enable" : "disable");
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		bq2570x_charging_disable(bq, BATT, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		bq->chg_mv = val->intval / 1000;
		ret = bq2570x_set_chargevolt(bq, bq->chg_mv);
		if (ret)
			pr_err("Failed to set chargevolt, ret=%d\n", ret);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		bq->chg_ma = val->intval / 1000;
		ret = bq2570x_set_chargecurrent(bq, bq->chg_ma);
		if (ret)
			pr_err("Failed to set chargecurrent, ret=%d\n", ret);

		set_usb_online(bq);
		break;
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		bq->supply_type = val->intval;
		/* notify USB of supply type */
		ret =
		    power_supply_set_supply_type(bq->usb_psy, bq->supply_type);
		pr_err("set power supply type :%d %s\n", bq->supply_type,
		       !ret ? "successfully" : "failed");
		set_usb_present(bq);
		set_usb_online(bq);
		break;

	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		bq->icl_ma = val->intval / 1000; /* uA to mA */
		bq2570x_update_charging_profile(bq);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		bq->ivl_mv = purify_voltage_now(val->intval / 1000 /* uV to mV */);
		bq2570x_update_charging_profile(bq);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2570x_charger_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
	case POWER_SUPPLY_PROP_TYPEC_MODE:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq2570x_update_charging_profile(struct bq2570x *bq)
{
	int ret;

	mutex_lock(&bq->profile_change_lock);

	ret = bq2570x_set_chargevolt(bq, bq->chg_mv);
	if (ret)
		pr_err("Failed to set chargevolt, ret=%d\n", ret);

	ret = bq2570x_set_chargecurrent(bq, bq->chg_ma);
	if (ret)
		pr_err("Failed to set chargecurrent, ret=%d\n", ret);

	ret = bq2570x_set_input_current_limit(bq, bq->icl_ma);
	if (ret < 0)
		pr_err("couldn't set input current limit, ret=%d\n", ret);

	bq2570x_reset_vindpm(bq);
	ret = bq2570x_set_input_volt_limit(bq, bq->ivl_mv);
	if (ret < 0)
		pr_err("couldn't set input voltage limit, ret=%d\n", ret);

	mutex_unlock(&bq->profile_change_lock);

	return 0;
}

static void bq2570x_external_power_changed(struct power_supply *psy)
{
#if 0
	struct bq2570x *bq = container_of(psy, struct bq2570x, batt_psy);

	union power_supply_propval prop = { 0, };
	int ret, current_limit = 0;

	ret = bq->usb_psy->get_property(bq->usb_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		pr_err("could not read USB current_max property, ret=%d\n",
		       ret);
	else
		current_limit = prop.intval / 1000;

	pr_info("current_limit = %d\n", current_limit);

	if (bq->usb_psy_ma != current_limit) {
		bq->usb_psy_ma = current_limit;
		bq2570x_update_charging_profile(bq);
	}

	ret = bq->usb_psy->get_property(bq->usb_psy,
					POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		pr_info("usb online status =%d\n", prop.intval);

	ret = 0;
	if (bq->usb_present
	/* && bq->charge_enabled *//*!bq->charging_disabled_status */
	    /*&& bq->usb_psy_ma != 0 */) {
		if (prop.intval == 0) {
			pr_err("set usb online\n");
			ret = power_supply_set_online(bq->usb_psy, true);
		}
	} else {
		if (prop.intval == 1) {
			pr_err("set usb offline\n");
			ret = power_supply_set_online(bq->usb_psy, false);
		}
	}

	if (ret < 0)
		pr_info("could not set usb online state, ret=%d\n", ret);
#endif

}

static void determine_initial_typec_state(struct bq2570x *bq)
{
	int ret;
	union power_supply_propval val;

	/* if system bootup with adapter plugin, TYPEC PD event
	 * will be missed, so here to get them explicitly.
	 */
	if (!bq->typec_psy->get_property)
		return;

	ret = bq->typec_psy->get_property(bq->typec_psy,
					POWER_SUPPLY_PROP_TYPE, &val);
	if (!ret)
		bq->supply_type = val.intval;

	ret = bq->typec_psy->get_property(bq->typec_psy,
					  POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
					  &val);
	if (!ret)
		bq->icl_ma = val.intval / 1000; /* uA to mA */

	ret = bq->typec_psy->get_property(bq->typec_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&val);
	if (!ret)
		bq->ivl_mv = purify_voltage_now(val.intval / 1000 /* uV to mV */);

	pr_info("Initial type-c input current limit = %d\n ma", bq->icl_ma);
	pr_info("Initial type-c input voltage limit = %d\n mv", bq->ivl_mv);
}

static int bq2570x_psy_register(struct bq2570x *bq)
{
	int ret;

	bq->batt_psy.name = "battery";
	bq->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy.properties = bq2570x_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq2570x_charger_props);
	bq->batt_psy.get_property = bq2570x_charger_get_property;
	bq->batt_psy.set_property = bq2570x_charger_set_property;
	bq->batt_psy.external_power_changed = bq2570x_external_power_changed;
	bq->batt_psy.property_is_writeable = bq2570x_charger_is_writeable;

	ret = power_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}

	return 0;
}

static void bq2570x_psy_unregister(struct bq2570x *bq)
{
	power_supply_unregister(&bq->batt_psy);
}

static int bq2570x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2570x *bq = rdev_get_drvdata(rdev);

	ret = bq2570x_otg_enable(bq);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("bq2570x OTG mode Enabled!\n");
	}

	return ret;
}

static int bq2570x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2570x *bq = rdev_get_drvdata(rdev);

	ret = bq2570x_otg_disable(bq);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("bq2570x OTG mode Disabled\n");
	}

	return ret;
}

static int bq2570x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq2570x *bq = rdev_get_drvdata(rdev);

	bq2570x_get_charge_state(bq);

	return (bq->charge_state == CHARGE_STATE_OTG) ? 1 : 0;

}

struct regulator_ops bq2570x_otg_reg_ops = {
	.enable = bq2570x_otg_regulator_enable,
	.disable = bq2570x_otg_regulator_disable,
	.is_enabled = bq2570x_otg_regulator_is_enable,
};

static int bq2570x_regulator_init(struct bq2570x *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = { };

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2570x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
		    |= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev =
		    regulator_register(&bq->otg_vreg.rdesc, &cfg);
		if (IS_ERR(bq->otg_vreg.rdev)) {
			ret = PTR_ERR(bq->otg_vreg.rdev);
			bq->otg_vreg.rdev = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(bq->dev,
					"OTG reg failed, rc=%d\n", ret);
		}
	}

	return ret;
}

static struct bq2570x_platform_data *bq2570x_parse_dt(struct device *dev,
						      struct bq2570x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct bq2570x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq2570x_platform_data),
			     GFP_KERNEL);
	if (!pdata) {
		pr_err("Out of memory\n");
		return NULL;
	}

	ret =
	    of_property_read_u32(np, "ti,bq2570x,input-volt-limit",
				 &pdata->ivl_mv);
	if (ret)
		pr_err("Failed to read node of ti,bq2570x,input-volt-limit\n");

	ret =
	    of_property_read_u32(np, "ti,bq2570x,input-curr-limit",
				 &pdata->icl_ma);
	if (ret)
		pr_err("Failed to read node of ti,bq2570x,input-curr-limit\n");

	ret =
	    of_property_read_u32(np, "ti,bq2570x,charge-volt", &pdata->chg_mv);
	if (ret)
		pr_err("Failed to read node of ti,bq2570x,charge-volt\n");

	ret =
	    of_property_read_u32(np, "ti,bq2570x,charge-current",
				 &pdata->chg_ma);
	if (ret)
		pr_err("Failed to read node of ti,bq2570x,charge-current\n");

	ret =
	    of_property_read_u32(np, "ti,bq2570x,boost-voltage",
				&pdata->boostv);
	if (ret)
		pr_err("Failed to read node of ti,bq2570x,boost-voltage\n");

	ret =
	    of_property_read_u32(np, "ti,bq2570x,boost-current",
				 &pdata->boosti);
	if (ret)
		pr_err("Failed to read node of ti,bq2570x,boost-current\n");

	bq->otg_gpio = of_get_named_gpio(np, "ti,bq2570x,otg-gpio", 0);

	return pdata;
}

static int bq2570x_init_device(struct bq2570x *bq)
{
	int ret;

	bq2570x_set_watchdog_timer(bq, WDTMR_ADJ_175S);

	bq->ivl_mv = bq->platform_data->ivl_mv;
	bq->icl_ma = bq->platform_data->icl_ma;
	bq->chg_mv = bq->platform_data->chg_mv;
	bq->chg_ma = bq->platform_data->chg_ma;

	/* Determine type C state */
	determine_initial_typec_state(bq);

	/* set initial charging profile */
	bq2570x_update_charging_profile(bq);

	bq2570x_set_rsns_rac(bq, 10);
	bq2570x_set_rsns_rsr(bq, 10);

	ret = bq2570x_set_otg_voltage(bq, bq->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = bq2570x_set_otg_current(bq, bq->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current, ret = %d\n", ret);

	bq2570x_set_acoc_vth(bq, 125);
	bq2570x_acoc_enable(bq);
	bq2570x_set_batoc_vth(bq, 125);
	bq2570x_batoc_enable(bq);

	ret = bq2570x_enable_charger(bq);
	if (ret) {
		pr_err("Failed to enable charger, ret = %d\n", ret);
	} else {
		bq->charge_enabled = true;
		pr_info("Charger Enabled Successfully!\n");
	}

	bq2570x_adc_channel_enable(bq, 0xFF);

	ret = bq2570x_adc_start(bq, false);
	pr_info("ADC start %s\n", !ret ? "successfully" : "failed");

	return 0;
}

static int bq2570x_detect_device(struct bq2570x *bq)
{
	int ret;
	u8 data;

	ret = bq2570x_read_byte(bq, &data, MANUFACTUREID_REG);
	if (ret == 0)
		bq->manufacture_id = data;

	return ret;
}

static const unsigned char *charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq2570x_dump_status(struct bq2570x *bq)
{
	pr_info("battery %s, temperature: %d\n",
		bq->batt_present ? "present" : "not present", bq->batt_temp);

	if (bq->vindpm_triggered)
		pr_info("VINDPM triggered\n");
	if (bq->iindpm_triggered)
		pr_info("IINDPM triggered\n");

	pr_info("vbus:%d, ibus:%d, vbat:%d,ichg:%d, idsg:%d\n",
	     bq->vbus_volt, bq->ibus_current, bq->vbat_volt, bq->charge_current,
	     bq->discharge_current);

	pr_info("%s\n", charge_stat_str[bq->charge_state]);
}

static void bq2570x_update_status(struct bq2570x *bq)
{
	u8 status;
	int ret;
	int last_charge_state;
	int last_fault_status;

	last_charge_state = bq->charge_state;
	bq2570x_get_charge_state(bq);

	ret = bq2570x_read_byte(bq, &status, CHARGERSTATUS_1_REG);
	if (ret) {
		pr_err("failed to read reg0a\n");
		return;
	}

	mutex_lock(&bq->data_lock);
	bq->vindpm_triggered = !!(status & CHARGERSTATUS_IN_VINDPM);
	bq->iindpm_triggered = !!(status & CHARGERSTATUS_IN_IINDPM);
	mutex_unlock(&bq->data_lock);

	last_fault_status = bq->fault_status;
	bq2570x_get_fault_status(bq);

	if (bq->charge_state != last_charge_state ||
	    bq->fault_status != last_fault_status)
		power_supply_changed(&bq->batt_psy);

	bq2570x_read_vbus_volt(bq);
	bq2570x_read_charge_current(bq);
	bq2570x_read_discharge_current(bq);
	bq2570x_read_vbat_volt(bq);
	bq2570x_read_vsys_volt(bq);
	bq2570x_read_ibus_current(bq);
}

static void set_usb_present(struct bq2570x *bq)
{

	bq2570x_get_vbus_present(bq);

	if (!bq->vbus_present && bq->usb_present) {
		bq->usb_present = false;
		power_supply_set_present(bq->usb_psy, bq->usb_present);

		pr_info("usb removed, set usb present = %d\n", bq->usb_present);
	} else if (bq->vbus_present && !bq->usb_present) {
		bq->usb_present = true;
		power_supply_set_present(bq->usb_psy, bq->usb_present);

		pr_info("usb plugged in, set usb present = %d\n",
			bq->usb_present);
	}
}

static void set_usb_online(struct bq2570x *bq)
{
	int ret;

	mutex_lock(&bq->usb_online_lock);

	if (!bq->usb_online && bq->usb_present
	    && bq->charge_enabled && bq->chg_ma) {
		ret = power_supply_set_online(bq->usb_psy, true);
		if (!ret) {
			bq->usb_online = true;
			pr_info("set usb online %d successfully\n",
				bq->usb_online);
		} else {
			pr_err("set usb online 1 failed\n");
		}
	} else if (bq->usb_online && (!bq->usb_present || !bq->charge_enabled
				      || !bq->chg_ma)) {
		ret = power_supply_set_online(bq->usb_psy, false);
		if (!ret) {
			bq->usb_online = false;
			pr_info("set usb online %d successfully\n",
				bq->usb_online);
		} else {
			pr_err("set usb online 0 failed\n");
		}
	}

	mutex_unlock(&bq->usb_online_lock);
}

static void bq2570x_monitor_workfunc(struct work_struct *work)
{
	struct bq2570x *bq =
	    container_of(work, struct bq2570x, monitor_work.work);

	bq2570x_update_status(bq);
	bq2570x_dump_status(bq);

	schedule_delayed_work(&bq->monitor_work, 5 * HZ);
}

static int show_registers(struct seq_file *m, void *data)
{
	struct bq2570x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x3B; addr++) {
		ret = bq2570x_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2570x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = {
	.owner = THIS_MODULE,
	.open = reg_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_debugfs_entry(struct bq2570x *bq)
{
	bq->debug_root = debugfs_create_dir("bq2570x", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
				    bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
				   bq->debug_root,
				   &(bq->charging_disabled_status));

		debugfs_create_x32("fault_status", S_IFREG | S_IRUGO,
				   bq->debug_root, &(bq->fault_status));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
				   bq->debug_root, &(bq->charge_state));

		debugfs_create_x32("skip_reads",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->skip_writes));
	}
}

static int bq2570x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq2570x *bq;
	struct power_supply *usb_psy = NULL;
	struct power_supply *bms_psy = NULL;
	struct power_supply *typec_psy = NULL;

	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	typec_psy = power_supply_get_by_name("typec");
	if (!typec_psy) {
		dev_dbg(&client->dev, "typec supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_dbg(&client->dev, "bms supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2570x), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;
	bq->typec_psy = typec_psy;

	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->usb_online_lock);
	mutex_init(&bq->adc_lock);

	create_debugfs_entry(bq);
	ret = bq2570x_detect_device(bq);
	if (ret) {
		pr_err("No bq2570x device found!\n");
		return -ENODEV;
	}

	if (client->dev.of_node)
		bq->platform_data = bq2570x_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}

	ret = bq2570x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	ret = bq2570x_psy_register(bq);
	if (ret)
		return ret;

	ret = bq2570x_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq2570x regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->monitor_work, bq2570x_monitor_workfunc);

	if (gpio_is_valid(bq->otg_gpio)) {
		ret = devm_gpio_request_one(bq->dev, bq->otg_gpio,
					    GPIOF_OUT_INIT_HIGH,
					    "bq2570x otg gpio");
		if (ret)
			pr_err("Unable to request otg gpio ret=%d\n", ret);
	}

	bq2570x_update_charging_profile(bq);
	set_usb_present(bq);
	set_usb_online(bq);
	schedule_delayed_work(&bq->monitor_work, 0);

	pr_info("bq2570x probe successfully, ManufactureID:%d\n!",
		bq->manufacture_id);

	return 0;

}

static int bq2570x_charger_remove(struct i2c_client *client)
{
	struct bq2570x *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);

	regulator_unregister(bq->otg_vreg.rdev);

	bq2570x_psy_unregister(bq);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->usb_online_lock);
	mutex_destroy(&bq->adc_lock);

	debugfs_remove_recursive(bq->debug_root);

	return 0;
}

static void bq2570x_charger_shutdown(struct i2c_client *client)
{
	pr_info("shutdown\n");
}

static struct of_device_id bq2570x_charger_match_table[] = {
	{.compatible = "ti,bq25703",},
	{},
};

MODULE_DEVICE_TABLE(of, bq2570x_charger_match_table);

static const struct i2c_device_id bq2570x_charger_id[] = {
	{"bq25703-charger", BQ25703},
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2570x_charger_id);

static struct i2c_driver bq2570x_charger_driver = {
	.driver = {
		   .name = "bq2570x-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = bq2570x_charger_match_table,
		   },
	.id_table = bq2570x_charger_id,

	.probe = bq2570x_charger_probe,
	.remove = bq2570x_charger_remove,
	.shutdown = bq2570x_charger_shutdown,

};

module_i2c_driver(bq2570x_charger_driver);

MODULE_DESCRIPTION("TI BQ2570x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
