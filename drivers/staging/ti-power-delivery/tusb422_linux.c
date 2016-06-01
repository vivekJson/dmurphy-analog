/*
 * Texas Instruments TUSB422 Power Delivery
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Copyright: (C) 2016 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "tusb422_common.h"

#define PDO1   0
#define PDO2   1
#define PDO3   2

struct tusb422_pwr_delivery {
	struct device *dev;
	struct power_supply *ps;
	struct regmap *regmap;
	struct i2c_client *client;
	struct src_pdo_t *source_pwr;
	struct snk_pdo_t *sink_pwr;
	struct gpio_desc *alert_gpio;
	struct gpio_desc *vbus_src_gpio;
	struct gpio_desc *vbus_snk_gpio;
	struct gpio_desc *vconn_gpio;
	struct gpio_desc *vbus_hv_gpio;
	struct gpio_desc *vbus_5v_gpio;
        struct hrtimer timer;
	void (*call_back) (unsigned int);
	tcpc_config_t *configuration;
	usb_pd_port_config_t *port_config;
	int alert_irq;
	int id;
};

static struct tusb422_pwr_delivery *tusb422_pd;

/* Remove this line to disable debug */
#define TUSB422_DEBUG

#ifdef TUSB422_DEBUG
/* The registers can be accessed via
 * cat /sys/class/i2c-adapter/i2c-2/2-0020/registers
 * And written through echo for example
 * echo "CFG_STAND_OUT 0x00" > /sys/class/i2c-adapter/i2c-2/2-0020/registers
 */
struct tusb422_reg {
	const char *name;
	uint8_t reg;
} tusb422_regs[] = {
	{ "VEN_ID_0", TUSB422_VENDOR_ID_0 },
	{ "VEN_ID_1", TUSB422_VENDOR_ID_1 },
	{ "PROD_ID_0", TUSB422_PRODUCT_ID_0 },
	{ "PROD_ID_1", TUSB422_PRODUCT_ID_1 },
	{ "DEV_ID_0", TUSB422_DEV_ID_0 },
	{ "DEV_ID_1", TUSB422_DEV_ID_1 },
	{ "TYPEC_REV_0",  TUSB422_USBTYPEC_REV_0 },
	{ "TYPEC_REV_1", TUSB422_USBTYPEC_REV_1 },
	{ "REV_VER_0", TUSB422_USBPD_REV_VER_0 },
	{ "REV_VER_1", TUSB422_USBPD_REV_VER_1 },
	{ "PD_INT_0", TUSB422_PD_INTERFACE_REV_0 },
	{ "PD_INT_1", TUSB422_PD_INTERFACE_REV_1 },
	{ "CFG_STAND_OUT", TUSB422_CFG_STAND_OUT },
	{ "ALERT_0", TUSB422_ALERT_0 },
	{ "ALERT_1", TUSB422_ALERT_1 },
	{ "ALERT_MASK_0", TUSB422_ALERT_MASK_0 },
	{ "ALERT_MASK_1", TUSB422_ALERT_MASK_1 },
	{ "PWR_MASK_0", TUSB422_POWER_STATUS_MASK },
	{ "FAULT_MASK_0", TUSB422_FAULT_STATUS_MASK },
	{ "TCPC_CTRL", TUSB422_TCPC_CONTROL },
	{ "ROLE_CTRL", TUSB422_ROLE_CONTROL },
	{ "FAULT_CTRL", TUSB422_FAULT_CONTROL },
	{ "PWR_CTRL", TUSB422_PWR_CONTROL },
	{ "CC_STAT", TUSB422_CC_STATUS },
	{ "PWR_STAT", TUSB422_PWR_STATUS },
	{ "FAULT_STAT", TUSB422_FAULT_STATUS },
	{ "INT_STAT", TUSB422_INT_STATUS },
	{ "INT_STAT_MSK", TUSB422_INT_STATUS_MSK },
};

static ssize_t tusb422_registers_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	unsigned i, n, reg_count;
	unsigned int read_buf;
	struct tusb422_pwr_delivery *data = dev_get_drvdata(dev);

	reg_count = sizeof(tusb422_regs) / sizeof(tusb422_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		regmap_read(data->regmap, tusb422_regs[i].reg, &read_buf);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       tusb422_regs[i].name,
			       read_buf);
	}
	return n;
}

static ssize_t tusb422_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];
	struct tusb422_pwr_delivery *data = dev_get_drvdata(dev);

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(tusb422_regs) / sizeof(tusb422_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, tusb422_regs[i].name)) {
			error = regmap_write(data->regmap, tusb422_regs[i].reg, value);
			if (error) {
				pr_err("%s:Failed to write %s\n",
					__func__, name);
				return -1;
			}
			return count;
		}
	}
	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		tusb422_registers_show, tusb422_registers_store);

static struct attribute *tusb422_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group tusb422_attr_group = {
	.attrs = tusb422_attrs,
};
#endif

static const struct reg_default tusb422_reg_defs[] = {
	{ TUSB422_ALERT_0, 0x0},
	{ TUSB422_ALERT_1, 0x0},
	{ TUSB422_ALERT_MASK_0, 0xff },
	{ TUSB422_ALERT_MASK_1, 0xf },
	{ TUSB422_POWER_STATUS_MASK, 0xff },
	{ TUSB422_FAULT_STATUS_MASK, 0x7f },
	{ TUSB422_CFG_STAND_OUT, 0x60 },
	{ TUSB422_TCPC_CONTROL, 0x0 },
	{ TUSB422_ROLE_CONTROL, 0x1f },
	{ TUSB422_FAULT_CONTROL, 0x6 },
	{ TUSB422_PWR_CONTROL, 0x60 },
	{ TUSB422_CC_STATUS, 0 },
	{ TUSB422_PWR_STATUS, 0 },
	{ TUSB422_FAULT_STATUS,0 },
};

int tusb422_modify_reg(int reg, int clr_mask, int set_mask)
{
	int mask;

	if (clr_mask)
		mask = ~clr_mask;
	else
		mask = set_mask;

	printk("%s: reg 0x%X, clr 0x%X, set 0x%X\n", __func__, reg, clr_mask, set_mask);
	regmap_update_bits(tusb422_pd->regmap, reg, 0xff, mask);
	return 0;
};

int tusb422_write(int reg, int value, int num_of_regs)
{
	printk("%s:reg 0x%X, val 0x%X, num of regs %i\n", __func__, reg, value, num_of_regs);
	regmap_raw_write_async(tusb422_pd->regmap, reg, &value, num_of_regs);

	return 0;
};

int tusb422_read(int reg, int *value, int num_of_regs)
{
	printk("%s: reg 0x%X, num of regs %i\n", __func__, reg, num_of_regs);
	regmap_raw_read(tusb422_pd->regmap, reg, value, num_of_regs);
	printk("%s: value 0x%X\n", __func__, (uint16_t) *value);
	return 0;
};

void tusb422_set_timer_func(void (*function)(unsigned int))
{
	tusb422_pd->call_back=function;
};

int tusb422_start_timer(unsigned int timeout_ms)
{
	if (hrtimer_active(&tusb422_pd->timer))
		return -1;

	hrtimer_start(&tusb422_pd->timer, ms_to_ktime(timeout_ms), HRTIMER_MODE_REL);

	return 0;
};

int tusb422_stop_timer(void)
{
	hrtimer_cancel(&tusb422_pd->timer);
	return 0;
};

int tusb422_set_vbus(int vbus_sel)
{

	printk("%s: vBus %i\n", __func__, vbus_sel);
	if (vbus_sel == VBUS_SRC_5V) {
		/* Disable high voltage. */
		gpiod_direction_output(tusb422_pd->vbus_hv_gpio, 0);
		/* Enable 5V. */
		gpiod_direction_output(tusb422_pd->vbus_5v_gpio, 1);
		/* Enable SRC switch. */
		gpiod_direction_output(tusb422_pd->vbus_src_gpio, 0);
	} else if (vbus_sel == VBUS_SRC_HI_VOLT) {
		/* Disable 5v */
		gpiod_direction_output(tusb422_pd->vbus_5v_gpio, 0);
		/* Enable high voltage. */
		gpiod_direction_output(tusb422_pd->vbus_hv_gpio, 1);
		/* Enable SRC switch. */
		gpiod_direction_output(tusb422_pd->vbus_src_gpio, 0);
	} else if (vbus_sel == VBUS_SNK) {
		/* Enable SNK switch. */
		gpiod_direction_output(tusb422_pd->vbus_snk_gpio, 0);
	}

	return 0;
};

int tusb422_clr_vbus(int vbus_sel)
{
	printk("%s: vBus %i\n", __func__, vbus_sel);
	if (vbus_sel == VBUS_SRC_5V) {
		/* Disable SRC switch. */
		gpiod_direction_output(tusb422_pd->vbus_src_gpio, 0);
		/* Disable 5V. */
		gpiod_direction_output(tusb422_pd->vbus_5v_gpio, 0);
	} else if (vbus_sel == VBUS_SRC_HI_VOLT) {
		/* Disable high voltage. */
		gpiod_direction_output(tusb422_pd->vbus_hv_gpio, 0);
	} else if (vbus_sel == VBUS_SNK) {
		/* Disable SNK switch. */
		gpiod_direction_output(tusb422_pd->vbus_snk_gpio, 1);
	}

	return 0;
};

static irqreturn_t tusb422_event_handler(int irq, void *private)
{
	tcpm_alert_event(0);
	tcpm_connection_task();
	usb_pd_task();

	return IRQ_HANDLED;
};

static int tusb422_of_get_gpios(struct tusb422_pwr_delivery *tusb422_pd)
{
	int ret;

	tusb422_pd->alert_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,alert", GPIOD_IN);
	if (IS_ERR(tusb422_pd->alert_gpio)) {
		ret = PTR_ERR(tusb422_pd->alert_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->alert_gpio = NULL;
	} else {
		gpiod_direction_input(tusb422_pd->alert_gpio);
		tusb422_pd->alert_irq = gpiod_to_irq(tusb422_pd->alert_gpio);
		printk("%s: value %i,\n", __func__, tusb422_pd->alert_irq);
	}

	tusb422_pd->vbus_snk_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-snk",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_snk_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_snk_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_snk_gpio = NULL;
	}

	tusb422_pd->vbus_src_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-src",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_src_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_src_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_src_gpio = NULL;
	}

	tusb422_pd->vconn_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vconn-en",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vconn_gpio)) {
		ret = PTR_ERR(tusb422_pd->vconn_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vconn_gpio = NULL;
	}

	tusb422_pd->vbus_hv_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-hv",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_hv_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_hv_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_hv_gpio = NULL;
	}

	tusb422_pd->vbus_5v_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-5v",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_5v_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_5v_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_5v_gpio = NULL;
	}

	return 0;
};

static int tusb422_of_init(struct tusb422_pwr_delivery *tusb422_pd)
{
	struct device_node *of_node = tusb422_pd->dev->of_node;
	unsigned int supply_type;
	unsigned int min_volt;
	unsigned int max_volt;
	int ret;

	struct device *dev = tusb422_pd->dev;

	tusb422_pd->port_config = devm_kzalloc(dev,
			sizeof(*tusb422_pd->port_config), GFP_KERNEL);

	if (!tusb422_pd->port_config)
		return -ENOMEM;

	tusb422_pd->port_config->usb_comm_capable = true;
	tusb422_pd->port_config->usb_suspend_supported = false;
	tusb422_pd->port_config->externally_powered = false; 
	tusb422_pd->port_config->dual_role_data = false;
	tusb422_pd->port_config->unchunked_msg_support = false; /* Always false for TUSB422 */

	tusb422_pd->port_config->num_src_pdos = 2;
	/* vSafe5V Source PDO */
	tusb422_pd->port_config->src_caps[PDO1].SupplyType = SUPPLY_TYPE_FIXED;
	tusb422_pd->port_config->src_caps[PDO1].PeakI = PEAK_CURRENT_0;
	tusb422_pd->port_config->src_caps[PDO1].MinV = PDO_VOLT(5000);
	tusb422_pd->port_config->src_caps[PDO1].MaxV = 0;   /* N/A */  
	tusb422_pd->port_config->src_caps[PDO1].MaxI = PDO_CURR(3000);
	tusb422_pd->port_config->src_caps[PDO1].MaxPower = 0; /* N/A */
	/* Fixed, Battery, and Variable Supply PDOs each in ascending voltage order */
	tusb422_pd->port_config->src_caps[PDO2].SupplyType = SUPPLY_TYPE_FIXED;
	tusb422_pd->port_config->src_caps[PDO2].PeakI = PEAK_CURRENT_0;
	tusb422_pd->port_config->src_caps[PDO2].MinV = PDO_VOLT(15000);
	tusb422_pd->port_config->src_caps[PDO2].MaxV = 0;   /* N/A */
	tusb422_pd->port_config->src_caps[PDO2].MaxI = PDO_CURR(3000);    
	tusb422_pd->port_config->src_caps[PDO2].MaxPower = 0; /* N/A */

	tusb422_pd->port_config->src_settling_time_ms = 50;
		       
	tusb422_pd->port_config->num_snk_pdos = 2;
	/* vSafe5V Sink PDO */
	tusb422_pd->port_config->snk_caps[PDO1].SupplyType = SUPPLY_TYPE_FIXED;
	tusb422_pd->port_config->snk_caps[PDO1].PeakI = PEAK_CURRENT_0;      
	tusb422_pd->port_config->snk_caps[PDO1].MinV = PDO_VOLT(5000);
	tusb422_pd->port_config->snk_caps[PDO1].MaxV = 0; /* N/A */
	tusb422_pd->port_config->snk_caps[PDO1].MaxOperatingCurrent = PDO_CURR(3000);
	tusb422_pd->port_config->snk_caps[PDO1].MinOperatingCurrent = PDO_CURR(900);
	tusb422_pd->port_config->snk_caps[PDO1].OperationalCurrent = PDO_CURR(900);
	tusb422_pd->port_config->snk_caps[PDO1].MaxOperatingPower = 0; /* N/A */
	tusb422_pd->port_config->snk_caps[PDO1].MinOperatingPower = 0; /* N/A */
	tusb422_pd->port_config->snk_caps[PDO1].OperationalPower = 0;  /* N/A */
	/* Sink PDO #2 */
	tusb422_pd->port_config->snk_caps[PDO2].SupplyType = SUPPLY_TYPE_FIXED;
	tusb422_pd->port_config->snk_caps[PDO2].PeakI = PEAK_CURRENT_0;      
	tusb422_pd->port_config->snk_caps[PDO2].MinV = PDO_VOLT(14800);
	tusb422_pd->port_config->snk_caps[PDO2].MaxV = 0; /* N/A */
	tusb422_pd->port_config->snk_caps[PDO2].MaxOperatingCurrent = PDO_CURR(3000);
	tusb422_pd->port_config->snk_caps[PDO2].MinOperatingCurrent = PDO_CURR(1000);
	tusb422_pd->port_config->snk_caps[PDO2].OperationalCurrent = PDO_CURR(1000);
	tusb422_pd->port_config->snk_caps[PDO2].MaxOperatingPower = 0; /* N/A */
	tusb422_pd->port_config->snk_caps[PDO2].MinOperatingPower = 0; /* N/A */
	tusb422_pd->port_config->snk_caps[PDO2].OperationalPower = 0;  /* N/A */

	tusb422_pd->port_config->higher_capability = false;
	tusb422_pd->port_config->giveback_flag = false;    
	tusb422_pd->port_config->no_usb_suspend = true;
	tusb422_pd->port_config->fast_role_support = FAST_ROLE_NOT_SUPPORTED;
	tusb422_pd->port_config->priority = PRIORITY_VOLTAGE;

	ret = of_property_read_u32(of_node, "ti,supply-type",
				   &supply_type);
	if (ret) {
		printk("%s: ret is %i\n", __func__, ret);
		return ret;
	}
printk("%s: Supply type is %i\n", __func__, supply_type);
	ret = of_property_read_u32(of_node, "ti,min-voltage",
				   &min_volt);
	if (ret)
		return ret;
printk("%s: Min Volt is %i\n", __func__, min_volt);
	ret = of_property_read_u32(of_node, "ti,max-voltage",
				   &max_volt);
	if (ret)
		return ret;
printk("%s: Max Volt is %i\n", __func__, max_volt);
	switch (supply_type) {
	case SUPPLY_TYPE_BATTERY:
/*MaxPower
MaxOperatingPower
MinOperatingPower
OperationalPower*/

		break;
	case SUPPLY_TYPE_FIXED:
/*PeakI
MaxI
MaxOperatingCurrent
MinOperatingCurrent
OperationalCurrent*/
		break;
	case SUPPLY_TYPE_VARIABLE:
/*PeakI
MaxI
MaxOperatingCurrent
MinOperatingCurrent
OperationalCurrent*/
		break;
	default:
		return -ENODEV;
		break;
	};

	usb_pd_init(tusb422_pd->port_config);

	return 0;
}
static enum hrtimer_restart tusb422_timer_tasklet(struct hrtimer *hrtimer)
{
	struct tusb422_pwr_delivery *tusb422_pwr = container_of(hrtimer, struct tusb422_pwr_delivery, timer);

	printk("%s: call timer call back\n", __func__);

	printk("%s: call back 0x%pF\n", __func__, tusb422_pwr->call_back);
	tusb422_pwr->call_back(0);

	return HRTIMER_NORESTART;
}
	
static const struct regmap_config tusb422_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.reg_defaults = tusb422_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(tusb422_reg_defs),
	.cache_type = REGCACHE_NONE,
};

static int tusb422_set_config(struct tusb422_pwr_delivery *tusb422_pd)
{

	struct device *dev = tusb422_pd->dev;

	tusb422_pd->configuration = devm_kzalloc(dev,
			sizeof(*tusb422_pd->configuration), GFP_KERNEL);
	if (!tusb422_pd->configuration)
		return -ENOMEM;

	tusb422_pd->configuration->role = ROLE_DRP;
	tusb422_pd->configuration->flags = 0;
	tusb422_pd->configuration->rp_val = RP_HIGH_CURRENT;
	tusb422_pd->configuration->slave_addr = 0x20;
	tusb422_pd->configuration->intf = SMBUS_MASTER0;

	tcpm_init(tusb422_pd->configuration);

	return 0;
};

static int tusb422_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int ret;

	tusb422_pd = devm_kzalloc(dev, sizeof(*tusb422_pd), GFP_KERNEL);
	if (!tusb422_pd)
		return -ENOMEM;

	tusb422_pd->client = client;
	i2c_set_clientdata(client, tusb422_pd);

	tusb422_pd->dev = &client->dev;

	tusb422_of_get_gpios(tusb422_pd);

	tusb422_pd->regmap = devm_regmap_init_i2c(client, &tusb422_regmap_config);
	if (IS_ERR(tusb422_pd->regmap)) {
		ret = PTR_ERR(tusb422_pd->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	if (tusb422_pd->alert_irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, tusb422_pd->alert_irq,
			NULL,
			tusb422_event_handler,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			"tusb422_event", tusb422_pd);
		if (ret) {
			dev_err(&client->dev, "unable to request IRQ\n");
			return -EIO;
		}
	}

	hrtimer_init(&tusb422_pd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tusb422_pd->timer.function = tusb422_timer_tasklet;

	ret = tusb422_of_init(tusb422_pd);
	ret = tusb422_set_config(tusb422_pd);

	tcpm_alert_event(0);
	tcpm_connection_task();
	usb_pd_task();

#ifdef TUSB422_DEBUG
	ret = sysfs_create_group(&client->dev.kobj, &tusb422_attr_group);
	if (ret < 0)
		dev_err(&client->dev, "Failed to create sysfs: %d\n", ret);
#endif
	return ret;
};

static const struct i2c_device_id tusb422_id[] = {
	{ TUSB422_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tusb422_id);

#ifdef CONFIG_OF
static const struct of_device_id tusb422_pd_ids[] = {
	{ .compatible = "ti,tusb422-usb-pd" },
	{ }
};
MODULE_DEVICE_TABLE(of, tusb422_pd_ids);
#endif

static struct i2c_driver tusb422_i2c_driver = {
	.driver = {
		.name = TUSB422_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tusb422_pd_ids),
	},
	.probe = tusb422_i2c_probe,
	.id_table = tusb422_id,
};
module_i2c_driver(tusb422_i2c_driver);

MODULE_LICENSE("GPL v2");
