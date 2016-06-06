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
#include <linux/delay.h>
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
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "tusb422_common.h"

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
	struct work_struct work;
	void (*call_back) (unsigned int);
	tcpc_config_t *configuration;
	usb_pd_port_config_t *port_config;
	int alert_irq;
	int id;
	int alert_status;
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

int tusb422_write(int reg, int value, int num_of_regs)
{
	return regmap_raw_write_async(tusb422_pd->regmap, reg, &value, num_of_regs);
};

int tusb422_write_block(int reg, int *data, int num_of_regs)
{
	return regmap_raw_write_async(tusb422_pd->regmap, reg, data, num_of_regs);
};

int tusb422_read(int reg, int *value, int num_of_regs)
{
	return regmap_raw_read(tusb422_pd->regmap, reg, value, num_of_regs);
};

int tusb422_modify_reg(int reg, int clr_mask, int set_mask)
{
	return regmap_update_bits(tusb422_pd->regmap,
			reg, (clr_mask | set_mask), set_mask);
};

void tusb422_set_timer_func(void (*function)(unsigned int))
{
	tusb422_pd->call_back=function;
};

void tusb422_clr_timer_func(void)
{
	tusb422_pd->call_back = NULL;
};

int tusb422_start_timer(unsigned int timeout_ms)
{
    hrtimer_try_to_cancel(&tusb422_pd->timer);

	if (hrtimer_active(&tusb422_pd->timer)) {
		printk("##### Timer active\n");
		return -1;
	}

	hrtimer_start(&tusb422_pd->timer, ms_to_ktime(timeout_ms), HRTIMER_MODE_REL);

	return 0;
};

int tusb422_stop_timer(void)
{
	hrtimer_cancel(&tusb422_pd->timer);
	return 0;
};

/* BQ - use for short sleeps < 20ms. */
void tusb422_msleep(int msecs)
{
	udelay(msecs * 1000);
};

int tusb422_set_vbus(int vbus_sel)
{
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
	if (vbus_sel == VBUS_SRC_5V) {
		/* Disable SRC switch. */
		gpiod_direction_output(tusb422_pd->vbus_src_gpio, 1);
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
	tusb422_pd->alert_status = 1;

	tcpm_alert_event(0);
	tcpm_connection_task();
	usb_pd_task();

	tusb422_pd->alert_status = 0;

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
	struct device_node *pp;
	unsigned int supply_type;
	unsigned int min_volt, current_flow, peak_current, pdo;
	unsigned int max_volt, max_current, max_power, fast_role_support;
	unsigned int op_current, min_current, op_power, priority;
	unsigned int max_op_power, min_op_power;
	int ret;
	int num_of_sink = 0, num_of_src = 0;

	struct device *dev = tusb422_pd->dev;

	tusb422_pd->port_config = devm_kzalloc(dev,
			sizeof(*tusb422_pd->port_config), GFP_KERNEL);

    printk("\n#### sizeof(*tusb422_pd->port_config) = 0x%x\n\n", sizeof(*tusb422_pd->port_config));
	if (!tusb422_pd->port_config)
		return -ENOMEM;

	if (of_property_read_bool(of_node, "ti,usb_comm_capable"))
		tusb422_pd->port_config->usb_comm_capable = true;

	if (of_property_read_bool(of_node, "ti,usb_suspend_supported"))
		tusb422_pd->port_config->usb_suspend_supported = true;

	if (of_property_read_bool(of_node, "ti,externally_powered"))
		tusb422_pd->port_config->externally_powered = true;

	if (of_property_read_bool(of_node, "ti,dual_role_data"))
		tusb422_pd->port_config->dual_role_data = true;

	if (of_property_read_bool(of_node, "ti,unchunked_msg_support"))
		tusb422_pd->port_config->unchunked_msg_support = true;

	if (of_property_read_bool(of_node, "ti,higher_capability"))
		tusb422_pd->port_config->higher_capability = true;

	if (of_property_read_bool(of_node, "ti,giveback_flag"))
		tusb422_pd->port_config->giveback_flag = true;

	if (of_property_read_bool(of_node, "ti,no_usb_suspend"))
		tusb422_pd->port_config->no_usb_suspend = true;

	ret = of_property_read_u16(of_node, "ti,src_settling_time_ms",
			&tusb422_pd->port_config->src_settling_time_ms);
	if (ret)
		printk("%s: Missing src_settling_time_ms\n", __func__);

    if (tusb422_pd->port_config->src_settling_time_ms < 50)
    {
        // Mandate at least 50ms settling time.
        tusb422_pd->port_config->src_settling_time_ms = 50;
    }

	ret = of_property_read_u32(of_node, "ti,fast_role_support",
			&fast_role_support);
	if (ret)
		printk("%s: Missing fast_role_support\n", __func__);
	else
		tusb422_pd->port_config->fast_role_support = (fast_role_t) fast_role_support;

	ret = of_property_read_u32(of_node, "ti,priority", &priority);
	if (ret)
		printk("%s: Missing priority\n", __func__);
	else
		tusb422_pd->port_config->priority = (pdo_priority_t) priority;


	for_each_child_of_node(of_node, pp) {
		ret = of_property_read_u32(pp, "ti,current-flow",
					   &current_flow);
		if (ret)
			return ret;

		ret = of_property_read_u32(pp, "ti,supply-type",
					   &supply_type);
		if (ret)
			return ret;

		ret = of_property_read_u32(pp, "ti,min-voltage",
					   &min_volt);
		if (ret)
			return ret;
		ret = of_property_read_u32(pp, "ti,max-voltage",
					   &max_volt);
		if (ret)
			return ret;

		ret = of_property_read_u32(pp, "ti,pdo_number",
					   &pdo);
		if (ret)
			return ret;

		ret = of_property_read_u32(pp, "ti,peak_current",
					   &peak_current);
		if (ret)
			printk("%s: Missing peak current\n", __func__);

		switch (supply_type) {
		case SUPPLY_TYPE_BATTERY:
			if (current_flow == 0) {
				num_of_src++;
				ret = of_property_read_u32(pp, "ti,max_power",
							   &max_power);
				if (ret)
					return ret;

				tusb422_pd->port_config->src_caps[pdo].SupplyType = supply_type;
				tusb422_pd->port_config->src_caps[pdo].PeakI = peak_current;
				tusb422_pd->port_config->src_caps[pdo].MinV = PDO_VOLT(min_volt);
				tusb422_pd->port_config->src_caps[pdo].MaxV = PDO_VOLT(max_volt);
				tusb422_pd->port_config->src_caps[pdo].MaxPower = max_power;
			} else if (current_flow == 1) {
				num_of_sink++;
				ret = of_property_read_u32(pp, "ti,operational-pwr",
							   &op_power);
				if (ret)
					printk("%s: Missing op_power\n", __func__);

				ret = of_property_read_u32(pp, "ti,max_operational-pwr",
							   &max_op_power);
				if (ret)
					printk("%s: Missing max_op_power\n", __func__);

				ret = of_property_read_u32(pp, "ti,min_operational-pwr",
							   &min_op_power);
				if (ret)
					printk("%s: Missing min_op_power\n", __func__);

				tusb422_pd->port_config->snk_caps[pdo].SupplyType = supply_type;
				tusb422_pd->port_config->snk_caps[pdo].PeakI = peak_current;
				tusb422_pd->port_config->snk_caps[pdo].MinV = PDO_VOLT(min_volt);
				tusb422_pd->port_config->snk_caps[pdo].MaxV = PDO_VOLT(max_volt);
				tusb422_pd->port_config->snk_caps[pdo].MaxOperatingPower = max_op_power;
				tusb422_pd->port_config->snk_caps[pdo].MinOperatingPower = min_op_power;
				tusb422_pd->port_config->snk_caps[pdo].OperationalPower = op_power;
			} else {
				printk("%s: Undefined current flow\n", __func__);
			}

			break;
		case SUPPLY_TYPE_FIXED:
			if (current_flow == 0) {
				num_of_src++;
				ret = of_property_read_u32(pp, "ti,max_current",
							   &max_current);
				if (ret)
					printk("%s: Missing max current\n", __func__);

				tusb422_pd->port_config->src_caps[pdo].SupplyType = supply_type;
				tusb422_pd->port_config->src_caps[pdo].PeakI = peak_current;
				tusb422_pd->port_config->src_caps[pdo].MinV = PDO_VOLT(min_volt);
				tusb422_pd->port_config->src_caps[pdo].MaxV = PDO_VOLT(max_volt);
				tusb422_pd->port_config->src_caps[pdo].MaxI = PDO_CURR(max_current);

			} else if (current_flow == 1) {
				num_of_sink++;

				ret = of_property_read_u32(pp, "ti,max-operating-curr",
							   &max_current);
				if (ret)
					printk("%s: Missing max op current\n", __func__);

				ret = of_property_read_u32(pp, "ti,min-operating-curr",
							   &min_current);
				if (ret)
					printk("%s: Missing min op current\n", __func__);

				ret = of_property_read_u32(pp, "ti,operational-curr",
							   &op_current);
				if (ret)
					printk("%s: Missing op_curr\n", __func__);

				ret = of_property_read_u32(pp, "ti,operational-pwr",
							   &op_power);
				if (ret)
					printk("%s: Missing op_power\n", __func__);

				tusb422_pd->port_config->snk_caps[pdo].SupplyType = supply_type;
				tusb422_pd->port_config->snk_caps[pdo].PeakI = peak_current;
				tusb422_pd->port_config->snk_caps[pdo].MinV = PDO_VOLT(min_volt);
				tusb422_pd->port_config->snk_caps[pdo].MaxV = PDO_VOLT(max_volt);
				tusb422_pd->port_config->snk_caps[pdo].MaxOperatingCurrent = PDO_CURR(max_current);
				tusb422_pd->port_config->snk_caps[pdo].MinOperatingCurrent = PDO_CURR(min_current);
				tusb422_pd->port_config->snk_caps[pdo].OperationalCurrent = PDO_CURR(op_current);
				tusb422_pd->port_config->snk_caps[pdo].MaxOperatingPower = 0; /* N/A */
				tusb422_pd->port_config->snk_caps[pdo].MinOperatingPower = 0; /* N/A */
				tusb422_pd->port_config->snk_caps[pdo].OperationalPower = op_power;  /* N/A */
			} else {
				printk("%s: Undefined current flow\n", __func__);
			}

			break;
		case SUPPLY_TYPE_VARIABLE:

#if 0
struct src_pdo_t {
	enum supply_type_t SupplyType; /*! Supply type (fixed, variable, battery)  */
	enum peak_current_t PeakI;      /*! Peak current (fixed and var only)       */
	uint16_t        MinV;       /*! Minimum voltage                         */
	uint16_t        MaxV;       /*! Maximum voltage                         */
	uint16_t        MaxI;       /*! Maximum Current (fixed and var only)    */
	uint16_t        MaxPower;   /*! Maximum Power (battery only)            */
};

struct snk_pdo_t {
	enum supply_type_t   SupplyType; /*! Supply type (fixed, variable, battery)  */
	enum peak_current_t  PeakI;      /*! Peak current                            */
	uint16_t        MinV;       /*! Minimum voltage                         */
	uint16_t        MaxV;       /*! Maximum voltage    (variable, battery)  */
	uint16_t        MaxOperatingCurrent; /*! Maximum Current  (fixed, variable)    */
	uint16_t        MinOperatingCurrent; /*! Mininum Current  (fixed, variable)    */
	uint16_t        OperationalCurrent;  /*! Current  (fixed, variable)    */
	uint16_t        MaxOperatingPower;   /*! Maximum Power    (battery only)       */
	uint16_t        MinOperatingPower;   /*! Minimum Power    (battery only)       */
	uint16_t        OperationalPower;    /*! Power    (battery only)       */
};
#endif
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
	};

	tusb422_pd->port_config->num_snk_pdos = num_of_sink;
	tusb422_pd->port_config->num_src_pdos = num_of_src;

	usb_pd_init(tusb422_pd->port_config);

	return 0;
}

static enum hrtimer_restart tusb422_timer_tasklet(struct hrtimer *hrtimer)
{
	struct tusb422_pwr_delivery *tusb422_pwr = container_of(hrtimer, struct tusb422_pwr_delivery, timer);

	schedule_work(&tusb422_pwr->work);

	return HRTIMER_NORESTART;
}

static void tusb422_work(struct work_struct *work)
{
	struct tusb422_pwr_delivery *tusb422_pwr =
		container_of(work, struct tusb422_pwr_delivery, work);
	int i;

	if (tusb422_pwr->alert_status) {
		for(i = 0; i <= 2000; i++) {
			udelay(500);
			if (tusb422_pwr->alert_status == 0)
				break;
		}
	}

	disable_irq(tusb422_pwr->alert_irq);

	if (tusb422_pwr->call_back)
		tusb422_pwr->call_back(0);
	else
		printk("%s: call back NULL\n", __func__);

	tcpm_connection_task();
	usb_pd_task();
	enable_irq(tusb422_pwr->alert_irq);

	return;
};

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
	struct device_node *of_node = tusb422_pd->dev->of_node;
	unsigned int role, flags, rp_val;
	int ret;

	tusb422_pd->configuration = devm_kzalloc(dev,
			sizeof(*tusb422_pd->configuration), GFP_KERNEL);

	if (!tusb422_pd->configuration)
		return -ENOMEM;


	ret = of_property_read_u32(of_node, "ti,role", &role);
	if (ret)
		return ret;

	ret = of_property_read_u32(of_node, "ti,rp_val", &rp_val);
	if (ret)
		return ret;

	ret = of_property_read_u32(of_node, "ti,flags", &flags);
	if (ret)
		printk("%s: Missing ti,flags setting to 0\n", __func__);

	tusb422_pd->configuration->role = (tc_role_t) role;
	tusb422_pd->configuration->flags = (uint16_t) flags;
	tusb422_pd->configuration->rp_val = (tcpc_role_rp_val_t) rp_val;
	tusb422_pd->configuration->slave_addr = tusb422_pd->client->addr;
	tusb422_pd->configuration->intf = tusb422_pd->client->adapter->nr;

	ret = tcpm_init(tusb422_pd->configuration);

	return ret;
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
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"tusb422_event", tusb422_pd);
		if (ret) {
			dev_err(&client->dev, "unable to request IRQ\n");
			return -EIO;
		}
	}

	hrtimer_init(&tusb422_pd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	INIT_WORK(&tusb422_pd->work, tusb422_work);

	tusb422_pd->timer.function = tusb422_timer_tasklet;

	ret = tusb422_of_init(tusb422_pd);
	ret = tusb422_set_config(tusb422_pd);
	if (ret) {
		dev_err(&client->dev, "%s: No TUSB422 found\n", __func__);
		ret = -ENODEV;
		goto no_dev;
	}

	tcpm_alert_event(0);
	schedule_work(&tusb422_pd->work);

#ifdef TUSB422_DEBUG
	ret = sysfs_create_group(&client->dev.kobj, &tusb422_attr_group);
	if (ret < 0)
		dev_err(&client->dev, "Failed to create sysfs: %d\n", ret);
#endif
	return ret;

no_dev:
	cancel_work_sync(&tusb422_pd->work);
	return ret;
};

static int tusb422_remove(struct i2c_client *client)
{
	cancel_work_sync(&tusb422_pd->work);
#ifdef TUSB422_DEBUG
	sysfs_remove_group(&client->dev.kobj, &tusb422_attr_group);
#endif
	return 0;
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
	.remove = tusb422_remove,
	.id_table = tusb422_id,
};
module_i2c_driver(tusb422_i2c_driver);

MODULE_LICENSE("GPL v2");
