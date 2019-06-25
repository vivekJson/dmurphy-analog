// SPDX-License-Identifier: GPL-2.0
// BQ25150 Battery Charger Driver
// Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define BQ25150_MANUFACTURER "Texas Instruments"

#define BQ25150_STAT0		0x00
#define BQ25150_STAT1		0x01
#define BQ25150_STAT2		0x02
#define BQ25150_FLAG0		0x03
#define BQ25150_FLAG1		0x04
#define BQ25150_FLAG2		0x05
#define BQ25150_FLAG3		0x06
#define BQ25150_MASK0		0x07
#define BQ25150_MASK1		0x08
#define BQ25150_MASK2		0x09
#define BQ25150_MASK3		0x0a
#define BQ25150_VBAT_CTRL	0x12
#define BQ25150_ICHG_CTRL	0x13
#define BQ25150_PCHRGCTRL	0x14
#define BQ25150_TERMCTRL	0x15
#define BQ25150_BUVLO		0x16
#define BQ25150_CHARGERCTRL0	0x17
#define BQ25150_CHARGERCTRL1	0x18
#define BQ25150_ILIMCTRL	0x19
#define BQ25150_LDOCTRL		0x1d
#define BQ25150_MRCTRL		0x30
#define BQ25150_ICCTRL0		0x35
#define BQ25150_ICCTRL1		0x36
#define BQ25150_ICCTRL2		0x37
#define BQ25150_ADCCTRL0	0x40
#define BQ25150_ADCCTRL1	0x41
#define BQ25150_ADC_VBAT_M	0x42
#define BQ25150_ADC_VBAT_L	0x43
#define BQ25150_ADC_TS_M	0x44
#define BQ25150_ADC_TS_L	0x45
#define BQ25150_ADC_ICHG_M	0x46
#define BQ25150_ADC_ICHG_L	0x47
#define BQ25150_ADC_ADCIN_M	0x48
#define BQ25150_ADC_ADCIN_L	0x49
#define BQ25150_ADC_VIN_M	0x4a
#define BQ25150_ADC_VIN_L	0x4b
#define BQ25150_ADC_PMID_M	0x4c
#define BQ25150_ADC_PMID_L	0x4d
#define BQ25150_ADC_IIN_M	0x4e
#define BQ25150_ADC_IIN_L	0x4f
#define BQ25150_ADC_COMP1_M	0x52
#define BQ25150_ADC_COMP1_L	0X53
#define BQ25150_ADC_COMP2_M	0X54
#define BQ25150_ADC_COMP2_L	0x55
#define BQ25150_ADC_COMP3_M	0x56
#define BQ25150_ADC_COMP3_L	0x57
#define BQ25150_ADC_READ_EN	0x58
#define BQ25150_TS_FASTCHGCTRL	0x61
#define BQ25150_TS_COLD		0x62
#define BQ25150_TS_COOL		0x63
#define BQ25150_TS_WARM		0x64
#define BQ25150_TS_HOT		0x65
#define BQ25150_DEVICE_ID	0x6f

#define BQ25150_DIVISOR		65536
#define BQ25150_VBAT_BASE_VOLT	3600
#define BQ25150_VBAT_REG_MAX	4600
#define BQ25150_VBAT_REG_MIN	3600

#define BQ25150_ILIM_150MA	0x2
#define BQ25150_ILIM_MASK	0x7
#define BQ25150_HEALTH_MASK	0xf
#define BQ25150_OVERVOLT_MASK	0x80

#define BQ25150_VIN_GOOD	BIT(0)
#define BQ25150_CHRG_DONE	BIT(5)
#define BQ25150_CV_CHRG_MODE	BIT(6)

static const int bq25150_ilim_lvl_values[] = {
	50, 100, 150, 200, 300, 400, 500, 600
};

/* initial field values, converted to register values */
struct bq25150_init_data {
	u8 ichg;	/* charge current		*/
	u8 vreg;	/* regulation voltage		*/
	u8 iterm;	/* termination current		*/
	u8 iprechg;	/* precharge current		*/
	u8 sysvmin;	/* minimum system voltage limit */
	u8 ilim;	/* ILIM current contol		*/
};

struct bq25150_device {
	struct power_supply *mains;
	struct power_supply *battery;
	struct i2c_client *client;
	struct regmap *regmap;
	struct device *dev;
	struct mutex lock;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *lp_gpio;
	struct gpio_desc *pg_gpio;

	char model_name[I2C_NAME_SIZE];
	int mains_online;

	uint32_t voltage_min_design;
	uint32_t voltage_max_design;
	uint32_t charge_full_design;

	struct bq25150_init_data init_data;
};

static struct reg_default bq25150_reg_defs[] = {
	{BQ25150_STAT0, 0xff},
	{BQ25150_STAT1, 0x0},
	{BQ25150_STAT2, 0x0},
	{BQ25150_FLAG0, 0x0},
	{BQ25150_FLAG1, 0x0},
	{BQ25150_FLAG2, 0x0},
	{BQ25150_FLAG3, 0x0},
	{BQ25150_MASK0, 0x0},
	{BQ25150_MASK1, 0x0},
	{BQ25150_MASK2, 0x0},
	{BQ25150_MASK3, 0x0},
};

static bool bq25150_is_ps_online(struct bq25150_device *bq25150)
{
	return bq25150->mains_online;
}

static int bq25150_wake_up(struct bq25150_device *bq25150)
{
	int ret;
	int val;

	/* Read the STAT register if we can read it then the device is out
	 * of ship mode.  If the register cannot be read then attempt to wake
	 * it up and enable the ADC.
	 */
	ret = regmap_read(bq25150->regmap, BQ25150_STAT0, &val);
	if (!ret)
		return ret;

	/* Need to toggle LP and MR here */
	if (bq25150->lp_gpio)
		gpiod_direction_output(bq25150->lp_gpio, 1);

	if (bq25150->reset_gpio) {
		gpiod_direction_output(bq25150->lp_gpio, 0);
		mdelay(2000);
		gpiod_direction_output(bq25150->lp_gpio, 1);
	}

	ret = regmap_write(bq25150->regmap, BQ25150_ADC_READ_EN, BIT(3));
	/*regmap_write(bq25150->regmap, BQ25150_ADCCTRL0, vbat_reg_code);*/
	return ret;
}

static int bq25150_update_ps_status(struct bq25150_device *bq25150)
{
	bool dc = false;
	unsigned int val;
	int ret;

	if (bq25150->pg_gpio)
		val = gpiod_get_value(bq25150->pg_gpio);
	else {
		ret = regmap_read(bq25150->regmap, BQ25150_STAT0, &val);
		if (ret < 0)
			return ret;
	}

	dc = val & BQ25150_VIN_GOOD;

	ret = bq25150->mains_online != dc;

	bq25150->mains_online = dc;

	return ret;
}

static int get_const_charge_current(struct bq25150_device *bq25150)
{
	int ret;
	int intval;
	int iin_msb;
	int iin_lsb;
	u16 ichg_measurement;
	int ilim_val, ichg_multiplier;

	if (!bq25150_is_ps_online(bq25150))
		return -ENODATA;

	ret = regmap_read(bq25150->regmap, BQ25150_ADC_IIN_M, &iin_msb);
	if (ret < 0)
		return ret;

	ret = regmap_read(bq25150->regmap, BQ25150_ADC_IIN_L, &iin_lsb);
	if (ret < 0)
		return ret;

	ichg_measurement = (iin_msb << 8) | iin_lsb;
	ret = regmap_read(bq25150->regmap, BQ25150_ILIMCTRL, &ilim_val);
	if (ret < 0)
		return ret;

	if (ilim_val >= BQ25150_ILIM_150MA)
		ichg_multiplier = 350;
	else
		ichg_multiplier = 750;

	intval = (ichg_measurement * 100 / BQ25150_DIVISOR) * ichg_multiplier;
	return intval / 100;
}

static int get_const_charge_voltage(struct bq25150_device *bq25150)
{
	int ret;
	int intval;
	int vin_msb;
	int vin_lsb;
	u16 vbat_measurement;

	if (!bq25150_is_ps_online(bq25150))
		bq25150_wake_up(bq25150);

	ret = regmap_read(bq25150->regmap, BQ25150_ADC_VBAT_M, &vin_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq25150->regmap, BQ25150_ADC_VBAT_L, &vin_lsb);
	if (ret)
		return ret;

	vbat_measurement = (vin_msb << 8) | vin_lsb;
	intval = ((vbat_measurement * 10000) / BQ25150_DIVISOR) * 6;
	return intval / 10;
}

static int bq25150_charging_status(struct bq25150_device *bq25150,
				   union power_supply_propval *val)
{
	unsigned int status;
	int ret;

	if (!bq25150_is_ps_online(bq25150))
		return 0;

	ret = regmap_read(bq25150->regmap, BQ25150_STAT0, &status);
	if (ret) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return ret;
	}

	if (status & BQ25150_CV_CHRG_MODE || status & BQ25150_VIN_GOOD)
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
	else if (status & BQ25150_CHRG_DONE)
		val->intval = POWER_SUPPLY_STATUS_FULL;
	else if (status & BQ25150_CHRG_DONE)
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

	return ret;
}

static int bq25150_get_batt_reg(struct bq25150_device *bq25150)
{
	int vbat_reg_code;
	int ret;

	ret = regmap_read(bq25150->regmap, BQ25150_VBAT_CTRL, &vbat_reg_code);
	if (ret)
		return ret;

	return BQ25150_VBAT_BASE_VOLT + vbat_reg_code * 10;
}

static int bq25150_set_batt_reg(struct bq25150_device *bq25150, int val)
{
	int vbat_reg_code;

	if (val > BQ25150_VBAT_REG_MAX || val < BQ25150_VBAT_REG_MIN)
		return -EINVAL;

	vbat_reg_code = (val - BQ25150_VBAT_BASE_VOLT) / 10;

	return regmap_write(bq25150->regmap, BQ25150_VBAT_CTRL, vbat_reg_code);
}

static int bq25150_get_ilim_lvl(struct bq25150_device *bq25150)
{
	int ret;
	int val;

	ret = regmap_read(bq25150->regmap, BQ25150_ILIMCTRL, &val);
	if (ret)
		return ret;

	return bq25150_ilim_lvl_values[val & BQ25150_ILIM_MASK];
}

static int bq25150_set_ilim_lvl(struct bq25150_device *bq25150, int val)
{
	int i;

	for (i = 0; i <= ARRAY_SIZE(bq25150_ilim_lvl_values); i++) {
		if (val == bq25150_ilim_lvl_values[i])
			break;

		if (val > bq25150_ilim_lvl_values[i - 1] &&
		    val < bq25150_ilim_lvl_values[i]) {
			if (val - bq25150_ilim_lvl_values[i - 1] <
			    bq25150_ilim_lvl_values[i] - val) {
				i = i - 1;
				break;
			}
		}
	}

	return regmap_write(bq25150->regmap, BQ25150_ILIMCTRL, i);
}

static int bq25150_power_supply_property_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return true;
	default:
		return false;
	}
}

static int bq25150_charger_get_health(struct bq25150_device *bq25150,
				      union power_supply_propval *val)
{
	int health;
	int ret;
	int v;

	if (!bq25150_is_ps_online(bq25150))
		bq25150_wake_up(bq25150);

	ret = regmap_read(bq25150->regmap, BQ25150_FLAG1, &v);
	if (ret)
		return -EIO;

	if (v & BQ25150_HEALTH_MASK) {
		switch (v & BQ25150_HEALTH_MASK) {
		case 0x4: /* Cool */
		case 0x8: /* Cold */
		case 0xc: /* both */
			health = POWER_SUPPLY_HEALTH_COLD;
			break;
		case 0x1: /* Hot */
		case 0x2: /* warm */
		case 0x3: /* Both */
			health = POWER_SUPPLY_HEALTH_OVERHEAT;
			break;
		default:
			health = POWER_SUPPLY_HEALTH_UNKNOWN;
		}
	} else if (v & BQ25150_OVERVOLT_MASK) {
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else {
		health = POWER_SUPPLY_HEALTH_GOOD;
	}

	ret = regmap_read(bq25150->regmap, BQ25150_FLAG3, &v);
	if (v & BIT(5))
		health = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;

	val->intval = health;

	return 0;
}

static int bq25150_mains_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq25150_device *bq25150 = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25150_set_ilim_lvl(bq25150, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25150_set_batt_reg(bq25150, val->intval);
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int bq25150_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct bq25150_device *bq25150 = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq25150->mains_online;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_const_charge_current(bq25150);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25150_get_ilim_lvl(bq25150);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25150_get_batt_reg(bq25150);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq25150->model_name;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25150_MANUFACTURER;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = BQ25150_VBAT_REG_MAX;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = BQ25150_VBAT_REG_MIN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25150_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct bq25150_device *bq25150 = power_supply_get_drvdata(psy);
	int ret;

	ret = bq25150_update_ps_status(bq25150);
	if (ret < 0)
		return ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!bq25150_is_ps_online(bq25150)) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}

		ret = bq25150_charging_status(bq25150, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = get_const_charge_voltage(bq25150);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bq25150->voltage_min_design;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = bq25150->voltage_max_design;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq25150->charge_full_design;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq25150_charger_get_health(bq25150, val);
		if (ret)
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property bq25150_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static enum power_supply_property bq25150_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static struct power_supply_desc bq25150_mains_desc = {
	.name			= "bq25150-mains",
	.type			= POWER_SUPPLY_TYPE_MAINS,
	.get_property		= bq25150_mains_get_property,
	.set_property		= bq25150_mains_set_property,
	.properties		= bq25150_charger_properties,
	.num_properties		= ARRAY_SIZE(bq25150_charger_properties),
	.property_is_writeable = bq25150_power_supply_property_is_writeable,

};

static struct power_supply_desc bq25150_battery_desc = {
	.name			= "bq25150-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= bq25150_battery_get_property,
	.properties		= bq25150_battery_properties,
	.num_properties		= ARRAY_SIZE(bq25150_battery_properties),
};


static int bq25150_power_supply_register(struct bq25150_device *bq25150)
{
	int ret = -EINVAL;
	struct power_supply_config psy_cfg = { .drv_data = bq25150, };

	bq25150->mains = devm_power_supply_register(bq25150->dev,
						    &bq25150_mains_desc,
						    &psy_cfg);
	if (IS_ERR(bq25150->mains))
		return ret;

	bq25150->battery = devm_power_supply_register(bq25150->dev,
						      &bq25150_battery_desc,
						      &psy_cfg);
	if (IS_ERR(bq25150->battery)) {
		power_supply_unregister(bq25150->mains);
		return ret;
	}

	return 0;
}

static int bq25150_hw_init(struct bq25150_device *bq25150)
{
	int ret = 0;

	if (bq25150->init_data.ichg)
		ret = bq25150_set_ilim_lvl(bq25150, bq25150->init_data.ichg);

	if (bq25150->init_data.vreg)
		ret = bq25150_set_batt_reg(bq25150, bq25150->init_data.vreg);

	return ret;
}

static int bq25150_read_properties(struct bq25150_device *bq25150)
{
	int ret;

	ret = device_property_read_u8(bq25150->dev, "ti,charge-current",
				      &bq25150->init_data.ichg);
	if (ret)
		goto fail;

	ret = device_property_read_u8(bq25150->dev,
				      "ti,battery-regulation-voltage",
				      &bq25150->init_data.vreg);
	if (ret)
		goto fail;

	bq25150->pg_gpio = devm_gpiod_get_optional(bq25150->dev,
						   "pg", GPIOD_IN);
	if (IS_ERR(bq25150->pg_gpio))
		dev_info(bq25150->dev, "PG GPIO not defined");

	bq25150->reset_gpio = devm_gpiod_get_optional(bq25150->dev,
						   "reset", GPIOD_OUT_LOW);
	if (IS_ERR(bq25150->reset_gpio))
		dev_info(bq25150->dev, "reset GPIO not defined");

	bq25150->lp_gpio = devm_gpiod_get_optional(bq25150->dev, "low-power",
						   GPIOD_OUT_LOW);
	if (IS_ERR(bq25150->lp_gpio))
		dev_info(bq25150->dev, "LP GPIO not defined");

fail:
	return ret;
}

static const struct regmap_config bq25150_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25150_DEVICE_ID,
	.reg_defaults     = bq25150_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25150_reg_defs),
	.cache_type	  = REGCACHE_NONE,
};

static int bq25150_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bq25150_device *bq;
	int ret;

	bq = devm_kzalloc(dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;

	mutex_init(&bq->lock);

	bq->regmap = devm_regmap_init_i2c(client, &bq25150_regmap_config);
	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(bq->regmap);
	}

	strncpy(bq->model_name, id->name, I2C_NAME_SIZE);

	i2c_set_clientdata(client, bq);

	ret = bq25150_read_properties(bq);
	if (ret < 0) {
		dev_err(dev, "Failed to register power supply\n");
		return ret;
	}

	ret = bq25150_hw_init(bq);
	if (ret < 0) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
	}

	return bq25150_power_supply_register(bq);
}

static const struct i2c_device_id bq25150_i2c_ids[] = {
	{ "bq25150", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25150_i2c_ids);

static const struct of_device_id bq25150_of_match[] = {
	{ .compatible = "ti,bq25150", },
	{ },
};
MODULE_DEVICE_TABLE(of, bq25150_of_match);

static struct i2c_driver bq25150_driver = {
	.driver = {
		.name = "bq25150-charger",
		.of_match_table = of_match_ptr(bq25150_of_match),
	},
	.probe = bq25150_probe,
	.id_table = bq25150_i2c_ids,
};
module_i2c_driver(bq25150_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("BQ25150 charger driver");
MODULE_LICENSE("GPL v2");
