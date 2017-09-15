/*
 * TPS6598X usb phy driver for type-c and PD
 *
 * Author:      Dan Murphy <dmurphy@ti.com>
 * Copyright (C) 2017 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/class-dual-role.h>
#include <linux/usb/usb_typec.h>
#include <linux/usb/typec.h>
#include <linux/debugfs.h>

#include "tps6598x.h"

#define TYPEC_STD_MA			900
#define TYPEC_MED_MA			1500
#define TYPEC_HIGH_MA			3000

#define TPS6598X_BYTE_CNT_W		2

#define TPS6598X_REG_STABLE_ATTEMPTS 5
#define TPS6598X_REG_STABLE_WAIT_MS  250

static const struct tps6598x_reg_data tps6598x_reg[] = {
	{ TPS6598X_VID, 4, 0 },
	{ TPS6598X_DID, 4, 0 },
	{ TPS6598X_PROTO_VER, 4, 0 },
	{ TPS6598X_MODE, 4, 0 },
	{ TPS6598X_TYPE, 4, 0 },
	{ TPS6598X_UID, 16, 0 },
	{ TPS6598X_VERSION, 4, 0 },
	{ TPS6598X_INT_EVENT_1, 8, 0 },
	{ TPS6598X_INT_EVENT_2, 8, 0 },
	{ TPS6598X_INT_MASK_1, 8, 1 },
	{ TPS6598X_INT_MASK_2, 8, 1 },
	{ TPS6598X_INT_CLEAR_1, 8, 1 },
	{ TPS6598X_INT_CLEAR_2, 8, 1 },
	{ TPS6598X_STATUS, 4, 0 },
	{ TPS6598X_PWR_STATUS, 2, 0 },
	{ TPS6598X_SYS_CFG, 17, 1 },
	{ TPS6598X_CTRL_CFG, 5, 1 },
	{ TPS6598X_TX_SNK_CAP, 57, 1 },
	{ TPS6598X_PDO_CONTRACT, 6, 0 },
	{ TPS6598X_RDO_CONTRACT, 4, 0 },
	{ TPS6598X_PD_STATUS, 4, 0 },
};

struct tps6598x_priv {
	struct device *dev;
	struct i2c_client *client;
	struct dentry *debugfs_root;
	struct dual_role_phy_instance *tps6598x_instance;
	int gpio_int;
	int gpio_reset;
	struct work_struct tps6598x_work;
	struct power_supply type_c_psy;
	struct power_supply *batt_psy;
	struct power_supply *usb_psy;
	enum typec_attached_state attached_state;
	enum typec_port_mode port_mode;
	int irqz_int;
	int typec_state;
	int current_ma;
	int current_volt;
	int bc_charger_type;
	struct mutex i2c_mutex;
	bool force_bc_enable;
	int usb_mode;
	bool far_end_usb_host;
	bool notify_usb_data_role_change;
};

static struct tps6598x_priv *tps6598x_data;

enum dual_role_property tps6598x_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_PR,
};

static enum power_supply_property tps6598x_typec_properties[] = {
	POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_TYPEC_MODE,
};

static int tps6598x_i2c_write(struct tps6598x_priv *tps6598x_data, int reg,
				const void *data)
{
	unsigned i, reg_count, writeable, bytes_to_write = 0, buf_size = 0;
	u8 *buf;
	int ret;

	reg_count = sizeof(tps6598x_reg) / sizeof(tps6598x_reg[0]);
	for (i = 0; i < reg_count; i++) {
		if (tps6598x_reg[i].reg_num == reg) {
			writeable = tps6598x_reg[i].writeable;
			bytes_to_write = tps6598x_reg[i].num_of_bytes;
			break;
		}

		if (i == reg_count) {
			dev_warn(&tps6598x_data->client->dev,
				"Reg 0x%X not found\n", reg);
			return -ENODEV;
		}
	}

	buf_size = bytes_to_write + TPS6598X_BYTE_CNT_W;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg;
	memcpy(&buf[1], data, bytes_to_write + 1);

	mutex_lock(&tps6598x_data->i2c_mutex);
	ret = i2c_master_send(tps6598x_data->client, buf, buf_size);
	mutex_unlock(&tps6598x_data->i2c_mutex);
	if (ret == buf_size) {
		ret = 0;
#ifdef DEBUG_I2C_DUMP
		pr_info("%s: Write register = 0x%02x:\n", __func__, reg);
		print_hex_dump(KERN_INFO, "tps6598x_i2c_write: ", DUMP_PREFIX_NONE, 16, 1, buf, buf_size, false);
#endif
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&tps6598x_data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}

	kfree(buf);
	return ret;

}

static int tps6598x_i2c_read(struct tps6598x_priv *tps6598x_data, int reg,
				void *data)
{
	unsigned i, reg_count, bytes_to_read = 0;
	u8 buf[1];
	int ret;

	reg_count = sizeof(tps6598x_reg) / sizeof(tps6598x_reg[0]);
	for (i = 0; i < reg_count; i++) {
		if (tps6598x_reg[i].reg_num == reg) {
			/* Add one to the number of bytes as the first byte
			 * indicates the number of bytes acutally returned
			 */
			bytes_to_read = tps6598x_reg[i].num_of_bytes + 1;
			break;
		}

		if (i == reg_count) {
			dev_warn(&tps6598x_data->client->dev,
				"Reg 0x%X not found\n", reg);
			return -ENODEV;
		}
	}

	buf[0] = reg;

	mutex_lock(&tps6598x_data->i2c_mutex);
	ret = i2c_master_send(tps6598x_data->client, buf, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;

		dev_err(&tps6598x_data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
		mutex_unlock(&tps6598x_data->i2c_mutex);
		return ret;
	}

	ret = i2c_master_recv(tps6598x_data->client, data, bytes_to_read);
	mutex_unlock(&tps6598x_data->i2c_mutex);
	if (ret < 0)
		dev_warn(&tps6598x_data->client->dev, "i2c read data cmd failed\n");
	else {
#ifdef DEBUG_I2C_DUMP
		pr_info("%s: Read register = 0x%02x:\n", __func__, reg);
		print_hex_dump(KERN_INFO, "tps6598x_i2c_read: ", DUMP_PREFIX_NONE, 16, 1, data, bytes_to_read, false);
#endif
	}

	return ret;
}

/* Power supply functions */
static int set_property_on_battery(enum power_supply_property prop)
{
	int rc = 0;
	union power_supply_propval ret = {0, };

	if (!tps6598x_data->batt_psy) {
		tps6598x_data->batt_psy = power_supply_get_by_name("battery");
		if (!tps6598x_data->batt_psy) {
			pr_err("no batt psy found\n");
			return -ENODEV;
		}
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		ret.intval = tps6598x_data->current_ma;
		rc = tps6598x_data->batt_psy->set_property(tps6598x_data->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_CAPABILITY, &ret);
		if (rc)
			pr_err("failed to set current max rc=%d\n", rc);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret.intval = tps6598x_data->current_volt;
		rc = tps6598x_data->batt_psy->set_property(tps6598x_data->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		if (rc)
			pr_err("failed to set voltage now rc=%d\n", rc);
		break;

	case POWER_SUPPLY_PROP_TYPE:
		/*
		 * Notify the typec mode to charger. This is useful in the DFP
		 * case where there is no notification of OTG insertion to the
		 * charger driver.
		 */
		ret.intval = tps6598x_data->bc_charger_type;
		rc = tps6598x_data->batt_psy->set_property(tps6598x_data->batt_psy,
				POWER_SUPPLY_PROP_TYPE, &ret);
		if (rc)
			pr_err("failed to set typec mode rc=%d\n", rc);
		break;

	case POWER_SUPPLY_PROP_TYPEC_MODE:
		/*
		 * Notify the typec mode to charger. This is useful in the DFP
		 * case where there is no notification of OTG insertion to the
		 * charger driver.
		 */
		ret.intval = tps6598x_data->typec_state;
		rc = tps6598x_data->batt_psy->set_property(tps6598x_data->batt_psy,
				POWER_SUPPLY_PROP_TYPEC_MODE, &ret);
		if (rc)
			pr_err("failed to set typec mode rc=%d\n", rc);
		break;
	default:
		pr_err("invalid request\n");
		rc = -EINVAL;
	}

	return rc;
}

static int set_property_on_usb(enum power_supply_property prop)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!tps6598x_data->notify_usb_data_role_change)
		return 0;

	switch(prop) {
		case POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE:
			ret.intval = tps6598x_data->usb_mode;
			rc = tps6598x_data->usb_psy->set_property(tps6598x_data->usb_psy,
				POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE, &ret);
			if (rc)
				pr_err("failed to set usb mode, rc = %d\n", rc);
			break;

		default:
			rc = -EINVAL;
	}
	return rc;
}

static int tps6598x_typec_get_property(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *val)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = tps6598x_data->bc_charger_type;
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		val->intval = tps6598x_data->typec_state;
		break;
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		val->intval = tps6598x_data->current_ma;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = tps6598x_data->current_volt;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tps6598x_set_port_mode(enum typec_port_mode port_mode)
{
	int ret = 0;
	u8 obuf[18];
	u8 mask_val = 0;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -EIO;
	}

	switch (port_mode) {
	case TYPEC_UFP_MODE:
		pr_info("%s: UFP Mode\n", __func__);
		mask_val = TPS6598X_UFP_ONLY_MODE;
		break;
	case TYPEC_DFP_MODE:
		pr_info("%s: DFP Mode\n", __func__);
		mask_val = TPS6598X_DFP_ONLY_MODE;
		break;
	case TYPEC_DRP_MODE:
		mask_val = TPS6598X_DUAL_ROLE_MODE;
		pr_info("%s: DRP Mode\n", __func__);
		break;
	default:
		pr_info("%s: Default\n", __func__);
		break;
	}

	tps6598x_data->port_mode = port_mode;

	obuf[1] &= TPS6598X_PORTINFO_MASK;
	obuf[1] |= mask_val;

	return tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(obuf[0]));
}

/* Type C framework functions */
static enum typec_port_mode tps6598x_port_mode_get(void)
{
	enum typec_port_mode port_mode = TYPEC_MODE_ACCORDING_TO_PROT;
	int ret;
	u8 obuf[5];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, &obuf);
	if (ret < 0) {
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);
		return port_mode;
	}

	if ((obuf[1] & TPS6598X_ATTACHED_STATUS) == 0x00)
		return TYPEC_NOT_ATTACHED;

	mask_val = obuf[1] & TPS6598X_DATA_ROLE;
	switch (mask_val) {
	case TPS6598X_REG_STATUS_AS_DFP:
		port_mode = TYPEC_ATTACHED_AS_DFP;
		break;
	case TPS6598X_REG_STATUS_AS_UFP:
		port_mode = TYPEC_ATTACHED_AS_UFP;
		break;
	default:
		port_mode = TYPEC_NOT_ATTACHED;
	}
	pr_info("%s: port mode is %d\n", __func__, port_mode);

	return port_mode;
}

static enum typec_attached_state tps6598x_attached_state_detect(void)
{
	u8 obuf[5];
	int ret;
	u8 mask_val;

	tps6598x_data->attached_state = TYPEC_NOT_ATTACHED;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, &obuf);
	if (ret < 0) {
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);
		return tps6598x_data->attached_state;
	}

	printk("%s: 0x%X 0x%X 0x%X 0x%X\n", __func__,
		obuf[1], obuf[2], obuf[3], obuf[4]);

	if ((obuf[1] & TPS6598X_ATTACHED_STATUS) == 0x00)
		return TYPEC_NOT_ATTACHED;

	mask_val = obuf[1] & TPS6598X_DATA_ROLE;
	switch (mask_val) {
	case TPS6598X_REG_STATUS_AS_DFP:
		tps6598x_data->attached_state = TYPEC_ATTACHED_AS_DFP;
		break;
	case TPS6598X_REG_STATUS_AS_UFP:
		tps6598x_data->attached_state = TYPEC_ATTACHED_AS_UFP;
		break;
	default:
		tps6598x_data->attached_state = TYPEC_NOT_ATTACHED;
	}

	/* Determine if the far-end of cable has a USB host */
	if ((tps6598x_data->attached_state  != TYPEC_ATTACHED_AS_UFP) ||
			((obuf[3] & TPS6598X_REG_STATUS_USB_HOST_PRS_MASK) ==
				TPS6598X_REG_STATUS_USB_HOST_PD_NO_USB))
		tps6598x_data->far_end_usb_host = false;
	else
		tps6598x_data->far_end_usb_host = true;

	pr_debug("%s: attached state is %d\n", __func__,
			tps6598x_data->attached_state);

	return tps6598x_data->attached_state;
}

static int tps6598x_get_pd_voltage(void)
{
	u8 obuf[7];
	int ret;
	u16 volt;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PDO_CONTRACT, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -EIO;
	}

	volt = ((((obuf[3] & TPS6598X_PDO_VOLT_UPPER_MASK) << TPS6598X_PDO_VOLT_UPPER_SHIFT) |
		(obuf[2] & TPS6598X_PDO_VOLT_LOWER_MASK)) >> TPS6598X_PDO_VOLT_LOWER_SHIFT);

	printk("%s: PDO Contracted voltage is %d mV\n",
		__func__, volt * TPS6598X_PD_VOLT_ADJ);

	return volt * TPS6598X_PD_VOLT_ADJ;
}

static int tps6598x_get_pd_current(void)
{
	u8 obuf[7];
	int ret;
	u16 pdo_curr;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PDO_CONTRACT, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -EIO;
	}

	pdo_curr = (((obuf[2] << TPS6598X_PDO_CURR_UPPER_SHIFT) | obuf[1]) &
		TPS6598X_PDO_CURR_MASK);

	printk("%s: PDO Contracted current is %d mA\n",
		__func__, pdo_curr * TPS6598X_PD_CURR_ADJ);

	return pdo_curr * TPS6598X_PD_CURR_ADJ;
}

static enum typec_current_mode tps6598x_current_mode_detect(void)
{
	enum typec_current_mode current_mode = TYPEC_CURRENT_MODE_DEFAULT;
	int ret;
	int current_ma, current_volt;
	int charger_type;
	u8 obuf[3];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PWR_STATUS, &obuf);
	if (ret < 0) {
		pr_err("%s: read POWER_STATUS error\n", __func__);
		return current_mode;
	}

	if (tps6598x_data->attached_state == TYPEC_NOT_ATTACHED) {
		tps6598x_data->current_ma = TYPEC_CURRENT_MODE_DEFAULT;
		tps6598x_data->current_volt = 0;
		tps6598x_data->bc_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		set_property_on_battery(POWER_SUPPLY_PROP_CURRENT_CAPABILITY);
		set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
		set_property_on_battery(POWER_SUPPLY_PROP_VOLTAGE_NOW);

		return 0;
	}

	mask_val = obuf[1] & TPS6598X_DETECT_CURR_MASK;
	switch (mask_val) {
	case TPS6598X_REG_CUR_MODE_DETECT_DEFAULT:
		current_mode = TYPEC_CURRENT_MODE_DEFAULT;
		current_ma = TYPEC_STD_MA;
		current_volt = 5000;
		break;
	case TPS6598X_REG_CUR_MODE_DETECT_MID:
		current_mode = TYPEC_CURRENT_MODE_MID;
		current_ma = TYPEC_MED_MA;
		current_volt = 5000;
		break;
	case TPS6598X_REG_CUR_MODE_DETECT_HIGH:
		current_mode = TYPEC_CURRENT_MODE_HIGH;
		current_ma = TYPEC_HIGH_MA;
		current_volt = 5000;
		break;
	case TPS6598X_REG_CUR_MODE_DETECT_PD:
		current_mode = TYPEC_CURRENT_MODE_PD;
		current_ma = tps6598x_get_pd_current();
		current_volt = tps6598x_get_pd_voltage();
		break;
	default:
		current_mode = TYPEC_CURRENT_MODE_UNSPPORTED;
		current_ma = 0;
		current_volt = 0;
	}

	if (tps6598x_data->current_ma != current_ma) {
		tps6598x_data->current_ma = current_ma;
		set_property_on_battery(POWER_SUPPLY_PROP_CURRENT_CAPABILITY);
	}

	if (tps6598x_data->current_volt != current_volt) {
		tps6598x_data->current_volt = current_volt;
		set_property_on_battery(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	}


	mask_val = obuf[1] & TPS6598X_BCSTATUS_MASK;
	switch (mask_val) {
	case TPS6598X_BCSTATUS_SDP:
		charger_type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case TPS6598X_BCSTATUS_DCP:
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case TPS6598X_BCSTATUS_CDP:
		charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	default:
		charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	if (tps6598x_data->bc_charger_type != charger_type) {
		tps6598x_data->bc_charger_type = charger_type;
		set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
	}

	pr_info("%s: current mode is %d\n", __func__, current_mode);
	pr_info("%s: bcstatus is %d\n", __func__, charger_type);
	pr_info("%s: current ma is %d\n", __func__, current_ma);
	pr_info("%s: voltage is %d\n", __func__, current_volt);

	return current_mode;
}

static enum typec_current_mode tps6598x_current_advertise_get(void)
{
	enum typec_current_mode current_mode = TYPEC_CURRENT_MODE_DEFAULT;
	int ret;
	u8 obuf[18];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return current_mode;
	}

	mask_val = obuf[1] & TPS6598X_REG_CUR_MODE_ADVERTISE_MASK;
	switch (mask_val) {
	case TPS6598X_REG_CUR_MODE_ADVERTISE_DEFAULT:
		current_mode = TYPEC_CURRENT_MODE_DEFAULT;
		break;
	case TPS6598X_REG_CUR_MODE_ADVERTISE_MID:
		current_mode = TYPEC_CURRENT_MODE_MID;
		break;
	case TPS6598X_REG_CUR_MODE_ADVERTISE_HIGH:
		current_mode = TYPEC_CURRENT_MODE_HIGH;
		break;
	default:
		current_mode = TYPEC_CURRENT_MODE_UNSPPORTED;
	}

	pr_info("%s: current advertise is %d\n", __func__, current_mode);

	return current_mode;
}

static int tps6598x_set_bcenabled(int enable)
{
	int ret;
	u8 obuf[18];
	u8 bcenable;
	u8 bcmask;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_MASK_1, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -ENODEV;
	}

	if (enable)
		bcmask = obuf[4] | BIT(0);
	else
		bcmask = obuf[4] & ~BIT(0);

	obuf[4] = bcmask;

	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_MASK_1, &(obuf[0]));
	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_MASK_2, &(obuf[0]));

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_MASK_1, &obuf);
	if (ret < 0) {
		pr_err("%s: read TPS6598X_INT_MASK_1 error\n", __func__);
		return -ENODEV;
	}

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -ENODEV;
	}

	if (enable)
		bcenable = obuf[5] | BIT(0);
	else
		bcenable = obuf[5] & ~BIT(0);

	obuf[5] = bcenable;

	tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(obuf[0]));

	return ret;
}

static int tps6598x_current_advertise_set(enum typec_current_mode current_mode)
{
	int ret;
	u8 obuf[18];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return current_mode;
	}

	switch (current_mode) {
	case TYPEC_CURRENT_MODE_MID:
		mask_val = TPS6598X_REG_CUR_MODE_ADVERTISE_MID;
		break;
	case TYPEC_CURRENT_MODE_HIGH:
		mask_val = TPS6598X_REG_CUR_MODE_ADVERTISE_HIGH;
		break;
	default:
		mask_val = TPS6598X_REG_CUR_MODE_ADVERTISE_DEFAULT;
	}

	if (mask_val == (obuf[1] & TPS6598X_REG_CUR_MODE_ADVERTISE_MASK)) {
		pr_info("%s: current advertise is %d already\n", __func__,
			current_mode);
		return 0;
	}

	obuf[1] &= ~TPS6598X_REG_CUR_MODE_ADVERTISE_MASK;
	obuf[1] |= mask_val;

	tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(obuf[0]));

	pr_info("%s: current advertise set to %d\n", __func__, current_mode);
	return 0;
}

static int tps6598x_port_mode_set(enum typec_port_mode port_mode)
{
	int ret = 0;

	ret = tps6598x_set_port_mode(port_mode);
	if (ret)
		pr_info("%s: port mode failed to set to %d\n",
			__func__, port_mode);
	else
		pr_info("%s: port mode set to %d\n", __func__, port_mode);

	return ret;
}

static ssize_t tps6598x_dump_regs(char *buf)
{
	u8 obuf[7];
	int i;
	int ret;
	u16 pdo_volt, pdo_curr;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, &obuf);
	if (ret < 0)
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);

	printk("%s: STATUS 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
		obuf[1], obuf[2], obuf[3], obuf[4]);

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PWR_STATUS, &obuf);
	if (ret < 0)
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);

	printk("%s: PWR STATUS 0x%02X 0x%02X\n", __func__,
		obuf[1], obuf[2]);

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PDO_CONTRACT, &obuf);
	if (ret < 0)
		pr_err("%s: read SYS_CFG error\n", __func__);

	pdo_curr = (((obuf[2] << TPS6598X_PDO_CURR_UPPER_SHIFT) |
			obuf[1]) & TPS6598X_PDO_CURR_MASK);
	printk("%s: PDO Contracted current is %d mA\n", __func__,
		pdo_curr * TPS6598X_PD_CURR_ADJ);

	pdo_volt = ((((obuf[3] & TPS6598X_PDO_VOLT_UPPER_MASK) << TPS6598X_PDO_VOLT_UPPER_SHIFT) |
		(obuf[2] & TPS6598X_PDO_VOLT_LOWER_MASK)) >> TPS6598X_PDO_VOLT_LOWER_SHIFT);
	printk("%s: PDO Contracted voltage is %d mV\n", __func__,
		pdo_volt * TPS6598X_PD_VOLT_ADJ);

	for (i = 0; i <= obuf[0]; i++)
		printk("%s: PDO Contract byte %i is 0x%02X\n", __func__, i, obuf[i]);

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_RDO_CONTRACT, &obuf);
	if (ret < 0)
		pr_err("%s: read SYS_CFG error\n", __func__);

	for (i = 0; i <= obuf[0]; i++)
		printk("%s: RDO Contract byte %i is 0x%02X\n", __func__, i, obuf[i]);

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PD_STATUS, &obuf);
	if (ret < 0)
		pr_err("%s: read PD STATUS error\n", __func__);

	printk("%s: PD STATUS 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
		obuf[1], obuf[2], obuf[3], obuf[4]);

	return scnprintf(buf, PAGE_SIZE,
			 "0x%02X,0x%02X,0x%02X\n", obuf[0], obuf[1], obuf[2]);
}

/* Dual Role Class Functions */
static int tps6598x_get_property(struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop,
				unsigned int *val)
{
	enum typec_attached_state attached_state;

	attached_state = tps6598x_attached_state_detect();
	tps6598x_current_mode_detect();

	if (attached_state == TYPEC_ATTACHED_AS_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == TYPEC_ATTACHED_AS_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

static int tps6598x_set_power_role(const unsigned int *val)
{
	int ret = 0;
	u8 obuf[6];
	u8 sink_cap[58];
	u8 sys_cfg[18];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_CTRL_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read CTRL_CFG error\n", __func__);
		return -EIO;
	}

	if (*val == DUAL_ROLE_PROP_PR_SRC)
		mask_val = (TPS6598X_INIT_SRC_MODE | TPS6598X_PROCESS_SRC_MODE);
	else if (*val == DUAL_ROLE_PROP_PR_SNK)
		mask_val = (TPS6598X_INIT_SNK_MODE | TPS6598X_PROCESS_SNK_MODE);
	else
		return -EINVAL;

	obuf[1] &= TPS6598X_PR_SWAP_MASK;
	obuf[1] |= mask_val;

	if (tps6598x_data->port_mode == TYPEC_DFP_MODE) {
		ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &sys_cfg);
		if (ret < 0) {
			pr_err("%s: read SYS_CFG error\n", __func__);
			return -EIO;
		}
		sys_cfg[1] &= TPS6598X_PORTINFO_MASK;
		sys_cfg[1] |= TPS6598X_UFP_DR_PR_SWAP;
		sys_cfg[4] &= TPS6598X_PP_HV_MASK;
		sys_cfg[4] |= TPS6598X_PP_HV_INPUT;

		ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_TX_SNK_CAP, &sink_cap);
		if (ret < 0) {
			pr_err("%s: read SYS_CFG error\n", __func__);
			return -EIO;
		}
		/* The TRM indicates what these numbers are but not how they
		 * are derived so they are magical numbers
		 */
		sink_cap[1] = 0x01;
		sink_cap[2] = 0x2c;
		sink_cap[3] = 0x91;
		sink_cap[4] = 0x1;

		tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(sys_cfg[0]));
		tps6598x_i2c_write(tps6598x_data, TPS6598X_TX_SNK_CAP, &(sink_cap[0]));
	}

	tps6598x_i2c_write(tps6598x_data, TPS6598X_CTRL_CFG, &(obuf[0]));

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_CTRL_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read CTRL_CFG error\n", __func__);
		return -EIO;
	}

	return ret;
}

static int tps6598x_set_property(struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop,
				const unsigned int *val)
{
	enum typec_port_mode mode_switch;
	int ret = 0;

	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		if (*val == DUAL_ROLE_PROP_MODE_DFP)
			mode_switch = TYPEC_DFP_MODE;
		else if (*val == DUAL_ROLE_PROP_MODE_UFP)
			mode_switch = TYPEC_UFP_MODE;
		else if (*val == DUAL_ROLE_PROP_MODE_DRP)
			mode_switch = TYPEC_DRP_MODE;
		else
			return -EINVAL;

		ret = tps6598x_set_port_mode(mode_switch);
		break;
	case DUAL_ROLE_PROP_PR:
		ret = tps6598x_set_power_role(val);
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
	case DUAL_ROLE_PROP_SUPPORTED_MODES:
	case DUAL_ROLE_PROP_DR:
		pr_info("%s: prop: %d, not supported case so far\n",
			__func__, prop);
		break;
	default:
		pr_info("%s: the input(prop: %d) is not supported\n",
			__func__, prop);
		break;
	}

	return ret;
}

static int tps6598x_property_is_writeable(struct dual_role_phy_instance *dual_role,
			enum dual_role_property prop)
{
	switch (prop) {
	case DUAL_ROLE_PROP_PR:
	case DUAL_ROLE_PROP_MODE:
		return 1;
	default:
		return 0;
	}

	return 0;
}

static int tps6598x_wait_for_stable_read_regs(u8 *reg_status, u8 *reg_pwr_status)
{
	int ret;
	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, reg_status);
	if (ret < 0) {
		pr_err("%s: read TPS6598X_STATUS error, ret = %d\n", __func__, ret);
		return ret;
	}

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PWR_STATUS, reg_pwr_status);
	if (ret < 0) {
		pr_err("%s: read TPS6598X_PWR_STATUS error, ret = %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int tps6598x_wait_for_stable(void)
{
	int remaining = TPS6598X_REG_STABLE_ATTEMPTS;
	int ret;
	u8 prev_reg_status[5];
	u8 prev_reg_pwr_status[3];
	u8 reg_status[5];
	u8 reg_pwr_status[3];

	ret = tps6598x_wait_for_stable_read_regs(prev_reg_status, prev_reg_pwr_status);
	if (ret < 0) {
		pr_err("%s: Failed to read status registers, ret = %d\n", __func__, ret);
		return ret;
	}

	/* If there is no cable attached, don't need to wait for registers to stabilze since we don't care */
	if ((prev_reg_status[1] & TPS6598X_ATTACHED_STATUS) == 0x00)
		return 0;

	while (remaining > 0) {
		remaining--;
		msleep(TPS6598X_REG_STABLE_WAIT_MS);

		ret = tps6598x_wait_for_stable_read_regs(reg_status, reg_pwr_status);
		if (ret < 0) {
			pr_err("%s: Failed to read status registers, ret = %d\n", __func__, ret);
			return ret;
		}

		pr_info("%s: TPS6598X_STATUS = 0x%X 0x%X 0x%X 0x%X\n", __func__,
			reg_status[1], reg_status[2], reg_status[3], reg_status[4]);
		pr_info("%s: TPS6598X_PWR_STATUS = 0x%X 0x%X\n", __func__,
			reg_pwr_status[1], reg_pwr_status[2]);

		if ((memcmp(prev_reg_status, reg_status, sizeof(reg_status)) != 0) ||
			(memcmp(prev_reg_pwr_status, reg_pwr_status, sizeof(reg_pwr_status)) != 0)) {
			memcpy(prev_reg_status, reg_status, sizeof(reg_status));
			memcpy(prev_reg_pwr_status, reg_pwr_status, sizeof(reg_pwr_status));

			pr_info("%s: registers not stable\n", __func__);
		} else {
			pr_info("%s: registers stable\n", __func__);
			return 0;
		}
	}
	return -EINVAL;
}

static void tps6598x_int_work(struct work_struct *work)
{
	enum typec_attached_state attached_state;
	u8 obuf[TPS6598X_MAX_READ_BYTES];
	int ret = 0;

	/* As cable detection occurs, many intermidate states are reported via the status
	 * registers.  Instead of reporting each intermidate state to the entire system,
	 * wait here to attempt to wait for the status registers to become stable / cable
	 * detection to complete.  If the status registers do not become stable, just continue
	 * on with the latest register state.
	 */
	tps6598x_wait_for_stable();

	attached_state = tps6598x_attached_state_detect();
	tps6598x_current_mode_detect();

	if (TYPEC_ATTACHED_AS_DFP == attached_state) {
		typec_sink_detected_handler(TYPEC_SINK_DETECTED);
		tps6598x_data->typec_state = POWER_SUPPLY_TYPE_DFP;
		tps6598x_data->type_c_psy.type = POWER_SUPPLY_TYPE_DFP;
		tps6598x_data->usb_mode = POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE_HOST;
		ret = set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
		if (ret)
			pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", ret);
		set_property_on_usb(POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE);
	} else if (TYPEC_ATTACHED_AS_UFP == attached_state) {
		typec_sink_detected_handler(TYPEC_SINK_REMOVED);
		/* device in UFP state */
		tps6598x_data->typec_state = POWER_SUPPLY_TYPE_UFP;
		tps6598x_data->type_c_psy.type = POWER_SUPPLY_TYPE_UFP;

		/* If we are in UFP, and the far-end has a USB host, go to device mode */
		tps6598x_data->usb_mode =
			tps6598x_data->far_end_usb_host ?
				POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE_DEVICE : POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE_HOST;
		ret = set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
		if (ret)
			pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", ret);
		set_property_on_usb(POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE);
	} else if (TYPEC_NOT_ATTACHED == attached_state) {
		typec_sink_detected_handler(TYPEC_SINK_REMOVED);
		tps6598x_data->typec_state = POWER_SUPPLY_TYPE_UNKNOWN;
		tps6598x_data->type_c_psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
		tps6598x_data->usb_mode = POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE_HOST;
		ret = set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
		if (ret)
			pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", ret);
		set_property_on_usb(POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE);
	}

	tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_EVENT_1, &obuf);
	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_CLEAR_1, &obuf);

	tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_EVENT_2, &obuf);
	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_CLEAR_2, &obuf);

	tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, &obuf);

	if (tps6598x_data->tps6598x_instance)
		dual_role_instance_changed(tps6598x_data->tps6598x_instance);
}

static irqreturn_t tps6598x_irq(int irq, void *dev)
{
	struct tps6598x_priv *tps6598x_data = dev;

	schedule_work(&tps6598x_data->tps6598x_work);

	return IRQ_HANDLED;
}

static const struct dual_role_phy_desc tps6598x_desc = {
	.name = "otg_default",
	.properties = tps6598x_properties,
	.num_properties = ARRAY_SIZE(tps6598x_properties),
	.get_property = tps6598x_get_property,
	.set_property = tps6598x_set_property,
	.property_is_writeable = tps6598x_property_is_writeable,
	.supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP
};

struct typec_device_ops tps6598x_ops = {
	.current_detect = tps6598x_current_mode_detect,
	.attached_state_detect = tps6598x_attached_state_detect,
	.current_advertise_get = tps6598x_current_advertise_get,
	.current_advertise_set = tps6598x_current_advertise_set,
	.port_mode_get = tps6598x_port_mode_get,
	.port_mode_set = tps6598x_port_mode_set,
	.dump_regs = tps6598x_dump_regs
};

static int tps6598x_debugfs_registers_show(struct seq_file *m, void *data)
{
	const int bytes_per_row = 16;
	int i;
	int ret;
	u8 buf[TPS6598X_MAX_READ_BYTES];
	int buf_index;
	unsigned char hexdump[TPS6598X_MAX_READ_BYTES * 4];
	int bytes_to_print;

	for (i = 0; i < ARRAY_SIZE(tps6598x_reg); i++) {
		bytes_to_print = tps6598x_reg[i].num_of_bytes;
		ret = tps6598x_i2c_read(tps6598x_data, tps6598x_reg[i].reg_num, buf);
		if (ret < 0)
			seq_printf(m, "Failed to read register 0x%02x, ret = %d\n", tps6598x_reg[i].reg_num, ret);
		else {
			/* Do hex dump of data, group data by 1 byte. +1 to the buffer index since
			 * the first byte is the length read
			 */
			buf_index = 1;
			hex_dump_to_buffer(buf + buf_index, bytes_to_print, bytes_per_row,
				1, hexdump, sizeof(hexdump), false);
			seq_printf(m, "Reg 0x%02x = %s\n", tps6598x_reg[i].reg_num, hexdump);

			/* Print any remaining data */
			bytes_to_print -= bytes_per_row;
			buf_index += bytes_per_row;
			while (bytes_to_print > 0 &&
				(buf_index + bytes_to_print < sizeof(buf))) {
				hex_dump_to_buffer(buf + buf_index, bytes_to_print, bytes_per_row,
					1, hexdump, sizeof(hexdump), false);
				seq_printf(m, "           %s\n", hexdump);
				bytes_to_print -= bytes_per_row;
				buf_index += bytes_per_row;
			}
		}
	}

	return 0;
}

static int tps6598x_debugfs_registers_open(struct inode *inode, struct file *file)
{
	return single_open(file, tps6598x_debugfs_registers_show, tps6598x_data);
}

static const struct file_operations tps6598x_debugfs_register_ops = {
	.owner = THIS_MODULE,
	.open = tps6598x_debugfs_registers_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void tps6598x_debugfs_create_entry(struct i2c_client *client)
{
	tps6598x_data->debugfs_root = debugfs_create_dir("tps6598x", NULL);
	if (!tps6598x_data->debugfs_root)
		dev_err(&client->dev, "Failed to create debugfs directory\n");
	else {
		debugfs_create_file("registers", S_IFREG | S_IRUGO,
			tps6598x_data->debugfs_root, tps6598x_data, &tps6598x_debugfs_register_ops);
	}
}

static int tps6598x_usb_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct power_supply *usb_psy = NULL;
	union power_supply_propval propval = {0, };
	struct device *dev = &client->dev;
	int ret = 0;
	int irq = 0;
	bool notify_usb_data_role_change;

	notify_usb_data_role_change = of_property_read_bool(dev->of_node, "notify-usb-data-role-change");
	if (notify_usb_data_role_change) {
		usb_psy = power_supply_get_by_name("usb");
		if (!usb_psy) {
			pr_err("%s: No USB power supply, deferring\n", __func__);
			return -EPROBE_DEFER;
		}

		/* To cover cases where we need to do a data role switch on boot, make sure USB is ready */
		ret = usb_psy->get_property(usb_psy, POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE, &propval);
		if (ret || propval.intval == POWER_SUPPLY_PROP_USB_OTG_DATA_ROLE_NONE) {
			pr_err("%s: USB not ready\n", __func__);
			return -EPROBE_DEFER;
		}
	}

	tps6598x_data = devm_kzalloc(dev, sizeof(*tps6598x_data), GFP_KERNEL);
	if (!tps6598x_data)
		return -ENOMEM;

	mutex_init(&tps6598x_data->i2c_mutex);
	tps6598x_data->client = client;
	tps6598x_data->usb_psy = usb_psy;
	tps6598x_data->notify_usb_data_role_change = notify_usb_data_role_change;
	i2c_set_clientdata(client, tps6598x_data);

	tps6598x_data->force_bc_enable = of_property_read_bool(dev->of_node, "force_bc_enable");
	tps6598x_data->gpio_reset = of_get_named_gpio(dev->of_node, "reset", 0);
	if (!gpio_is_valid(tps6598x_data->gpio_reset))
		dev_warn(&client->dev, "failed to get Reset GPIO\n");
	else {
		ret = devm_gpio_request(&client->dev, tps6598x_data->gpio_reset, "tps6598x_reset");
		if (ret < 0)
			dev_warn(&client->dev, "failed to request Reset GPIO, ret = %d\n", ret);
	}

	tps6598x_data->gpio_int = of_get_named_gpio(dev->of_node, "i2c-irqz", 0);
	if (!gpio_is_valid(tps6598x_data->gpio_int)) {
		dev_err(&client->dev, "failed to get IRQz GPIO\n");
		ret = -EINVAL;
		goto err_interrupt;
	}

	ret = devm_gpio_request(&client->dev, tps6598x_data->gpio_int, "tps6598x_irqz");
	if (ret < 0) {
		dev_warn(&client->dev, "failed to request IRQz GPIO, ret = %d\n", ret);
		goto err_interrupt;
	}

	tps6598x_data->irqz_int  = gpio_to_irq(tps6598x_data->gpio_int);
	if (tps6598x_data->irqz_int < 0) {
		dev_err(&client->dev, "failed to translate ID_OUT GPIO to IRQ\n");
		goto err_irq;
	}

	tps6598x_data->type_c_psy.name = "typec";
	tps6598x_data->type_c_psy.get_property = tps6598x_typec_get_property;
	tps6598x_data->type_c_psy.properties = tps6598x_typec_properties;
	tps6598x_data->type_c_psy.num_properties = ARRAY_SIZE(tps6598x_typec_properties);

	ret = power_supply_register(tps6598x_data->dev,
				&tps6598x_data->type_c_psy);
	if (ret < 0) {
		pr_err("Unable to register type_c_psy ret=%d\n", ret);
		goto err_irq;
	}

	tps6598x_data->port_mode = TYPEC_MODE_ACCORDING_TO_PROT;

	INIT_WORK(&tps6598x_data->tps6598x_work, tps6598x_int_work);
	ret = request_irq(tps6598x_data->irqz_int, tps6598x_irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "tps6598x_usb",
			tps6598x_data);
	if (ret) {
		pr_err("%s: request_irq error, ret=%d\n", __func__, ret);
		irq = -1;
		goto err_req_irq;
	}

	enable_irq_wake(tps6598x_data->irqz_int);

	schedule_work(&tps6598x_data->tps6598x_work);

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		tps6598x_data->tps6598x_instance = devm_dual_role_instance_register(dev,
			&tps6598x_desc);
	}

	ret = add_typec_device(&tps6598x_data->client->dev, &tps6598x_ops);

	if (tps6598x_data->force_bc_enable)
		tps6598x_set_bcenabled(1);

	tps6598x_debugfs_create_entry(client);
	return ret;

err_req_irq:
	free_irq(tps6598x_data->irqz_int, tps6598x_data);
err_irq:
err_interrupt:
	return ret;

}

static int tps6598x_usb_remove(struct i2c_client *client)
{
	if (tps6598x_data->debugfs_root)
		debugfs_remove_recursive(tps6598x_data->debugfs_root);

	free_irq(tps6598x_data->irqz_int, tps6598x_data);

	if (tps6598x_data->tps6598x_instance)
		devm_dual_role_instance_unregister(tps6598x_data->dev,
			tps6598x_data->tps6598x_instance);

	return 0;
}

static const struct i2c_device_id tps6598x_i2c_id[] = {
	{ "tps65986", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps6598x_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id tps6598x_id_table[] = {
	{ .compatible = "ti,tps6598x" },
	{}
};
MODULE_DEVICE_TABLE(of, tps6598x_id_table);
#endif

static struct i2c_driver tps6598x_usb_driver = {
	.probe		= tps6598x_usb_probe,
	.remove		= tps6598x_usb_remove,
	.driver		= {
		.name	= "tps6598x_usb",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tps6598x_id_table),
	},
	.id_table = tps6598x_i2c_id,
};

module_i2c_driver(tps6598x_usb_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_DESCRIPTION("tps6598x USB Type C and PD controller driver");
MODULE_LICENSE("GPL");
