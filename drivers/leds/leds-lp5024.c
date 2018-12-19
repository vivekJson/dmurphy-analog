// SPDX-License-Identifier: GPL-2.0
/* TI LP50XX LED chip family driver
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <uapi/linux/uleds.h>

#define LP5024_DEV_CFG0		0x00
#define LP5024_DEV_CFG1		0x01
#define LP5024_LED_CFG0		0x02
#define LP5024_BNK_BRT		0x03
#define LP5024_BNKA_CLR		0x04
#define LP5024_BNKB_CLR		0x05
#define LP5024_BNKC_CLR		0x06
#define LP5024_LED0_BRT		0x07
#define LP5024_LED1_BRT		0x08
#define LP5024_LED2_BRT		0x09
#define LP5024_LED3_BRT		0x0a
#define LP5024_LED4_BRT		0x0b
#define LP5024_LED5_BRT		0x0c
#define LP5024_LED6_BRT		0x0d
#define LP5024_LED7_BRT		0x0e

#define LP5024_OUT0_CLR		0x0f
#define LP5024_OUT1_CLR		0x10
#define LP5024_OUT2_CLR		0x11
#define LP5024_OUT3_CLR		0x12
#define LP5024_OUT4_CLR		0x13
#define LP5024_OUT5_CLR		0x14
#define LP5024_OUT6_CLR		0x15
#define LP5024_OUT7_CLR		0x16
#define LP5024_OUT8_CLR		0x17
#define LP5024_OUT9_CLR		0x18
#define LP5024_OUT10_CLR	0x19
#define LP5024_OUT11_CLR	0x1a
#define LP5024_OUT12_CLR	0x1b
#define LP5024_OUT13_CLR	0x1c
#define LP5024_OUT14_CLR	0x1d
#define LP5024_OUT15_CLR	0x1e
#define LP5024_OUT16_CLR	0x1f
#define LP5024_OUT17_CLR	0x20
#define LP5024_OUT18_CLR	0x21
#define LP5024_OUT19_CLR	0x22
#define LP5024_OUT20_CLR	0x23
#define LP5024_OUT21_CLR	0x24
#define LP5024_OUT22_CLR	0x25
#define LP5024_OUT23_CLR	0x26

#define LP5024_RESET		0x27
#define LP5024_SW_RESET		0xff

#define LP5024_CHIP_EN		BIT(6)

#define LP5018_MAX_LED_STRINGS	6
#define LP5024_MAX_LED_STRINGS	8

enum lp5024_model {
	LP5018,
	LP5024,
};

struct lp5024_led {
	u32 led_strings[LP5024_MAX_LED_STRINGS];
	char label[LED_MAX_NAME_SIZE];
	struct led_classdev led_dev;
	struct lp5024 *priv;
	int led_number;
	u8 ctrl_bank_enabled;
};

/**
 * struct lp5024 -
 * @enable_gpio: Hardware enable gpio
 * @regulator: LED supply regulator pointer
 * @client: Pointer to the I2C client
 * @regmap: Devices register map
 * @dev: Pointer to the devices device struct
 * @lock: Lock for reading/writing the device
 * @model_id: ID of the device
 * @leds: Array of LED strings
 */
struct lp5024 {
	struct gpio_desc *enable_gpio;
	struct regulator *regulator;
	struct i2c_client *client;
	struct regmap *regmap;
	struct device *dev;
	struct mutex lock;
	int model_id;
	int max_leds;
	int num_of_leds;

	/* This needs to be at the end of the struct */
	struct lp5024_led leds[];
};

static const struct reg_default lp5024_reg_defs[] = {
	{LP5024_DEV_CFG0, 0x0},
	{LP5024_DEV_CFG1, 0x3c},
	{LP5024_BNK_BRT, 0xff},
	{LP5024_BNKA_CLR, 0x0f},
	{LP5024_BNKB_CLR, 0x0f},
	{LP5024_BNKC_CLR, 0x0f},
	{LP5024_LED0_BRT, 0x0f},
	{LP5024_LED1_BRT, 0xff},
	{LP5024_LED2_BRT, 0xff},
	{LP5024_LED3_BRT, 0xff},
	{LP5024_LED4_BRT, 0xff},
	{LP5024_LED5_BRT, 0xff},
	{LP5024_LED6_BRT, 0xff},
	{LP5024_LED7_BRT, 0xff},
	{LP5024_OUT0_CLR, 0x0f},
	{LP5024_OUT1_CLR, 0x00},
	{LP5024_OUT2_CLR, 0x00},
	{LP5024_OUT3_CLR, 0x00},
	{LP5024_OUT4_CLR, 0x00},
	{LP5024_OUT5_CLR, 0x00},
	{LP5024_OUT6_CLR, 0x00},
	{LP5024_OUT7_CLR, 0x00},
	{LP5024_OUT8_CLR, 0x00},
	{LP5024_OUT9_CLR, 0x00},
	{LP5024_OUT10_CLR, 0x00},
	{LP5024_OUT11_CLR, 0x00},
	{LP5024_OUT12_CLR, 0x00},
	{LP5024_OUT13_CLR, 0x00},
	{LP5024_OUT14_CLR, 0x00},
	{LP5024_OUT15_CLR, 0x00},
	{LP5024_OUT16_CLR, 0x00},
	{LP5024_OUT17_CLR, 0x00},
	{LP5024_OUT18_CLR, 0x00},
	{LP5024_OUT19_CLR, 0x00},
	{LP5024_OUT20_CLR, 0x00},
	{LP5024_OUT21_CLR, 0x00},
	{LP5024_OUT22_CLR, 0x00},
	{LP5024_OUT23_CLR, 0x00},
	{LP5024_RESET, 0x00}
};

static const struct regmap_config lp5024_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = LP5024_RESET,
	.reg_defaults = lp5024_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(lp5024_reg_defs),
	.cache_type = REGCACHE_RBTREE,
};

static int lp5024_set_color_mix(struct lp5024_led *led, u8 color_reg,
				u8 color_val)
{
	return regmap_write(led->priv->regmap, color_reg, color_val);
}


static ssize_t ctrl_bank_a_mix_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	u8 mix_value;
	int ret;

	ret = kstrtou8(buf, 0, &mix_value);
	if (ret)
		return ret;

	lp5024_set_color_mix(led, LP5024_BNKA_CLR, mix_value);

	return size;
}
static ssize_t ctrl_bank_b_mix_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	u8 mix_value;
	int ret;

	ret = kstrtou8(buf, 0, &mix_value);
	if (ret)
		return ret;

	lp5024_set_color_mix(led, LP5024_BNKB_CLR, mix_value);

	return size;
}
static ssize_t ctrl_bank_c_mix_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	u8 mix_value;
	int ret;

	ret = kstrtou8(buf, 0, &mix_value);
	if (ret)
		return ret;

	lp5024_set_color_mix(led, LP5024_BNKC_CLR, mix_value);

	return size;
}

static DEVICE_ATTR_WO(ctrl_bank_a_mix);
static DEVICE_ATTR_WO(ctrl_bank_b_mix);
static DEVICE_ATTR_WO(ctrl_bank_c_mix);

static struct attribute *lp5024_ctrl_bank_attrs[] = {
	&dev_attr_ctrl_bank_a_mix.attr,
	&dev_attr_ctrl_bank_b_mix.attr,
	&dev_attr_ctrl_bank_c_mix.attr,
	NULL
};
ATTRIBUTE_GROUPS(lp5024_ctrl_bank);

static ssize_t led3_mix_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	u8 mix_value;
	u8 reg_value;
	int ret;

	ret = kstrtou8(buf, 0, &mix_value);
	if (ret)
		return ret;

	reg_value = (led->led_number * 3) + LP5024_OUT2_CLR;

	lp5024_set_color_mix(led, reg_value, mix_value);

	return size;
}

static ssize_t led2_mix_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	u8 mix_value;
	u8 reg_value;
	int ret;

	ret = kstrtou8(buf, 0, &mix_value);
	if (ret)
		return ret;

	reg_value = (led->led_number * 3) + LP5024_OUT1_CLR;

	lp5024_set_color_mix(led, reg_value, mix_value);

	return size;
}

static ssize_t led1_mix_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	u8 mix_value;
	u8 reg_value;
	int ret;

	ret = kstrtou8(buf, 0, &mix_value);
	if (ret)
		return ret;

	reg_value = (led->led_number * 3) + LP5024_OUT0_CLR;

	lp5024_set_color_mix(led, reg_value, mix_value);

	return size;
}

static DEVICE_ATTR_WO(led1_mix);
static DEVICE_ATTR_WO(led2_mix);
static DEVICE_ATTR_WO(led3_mix);

static struct attribute *lp5024_led_independent_attrs[] = {
	&dev_attr_led1_mix.attr,
	&dev_attr_led2_mix.attr,
	&dev_attr_led3_mix.attr,
	NULL
};
ATTRIBUTE_GROUPS(lp5024_led_independent);

static int lp5024_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness brt_val)
{
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	int ret = 0;
	u8 reg_val;

	mutex_lock(&led->priv->lock);

	if (led->ctrl_bank_enabled)
		reg_val = LP5024_BNK_BRT;
	else
		reg_val = led->led_number + LP5024_LED0_BRT;

	ret = regmap_write(led->priv->regmap, reg_val, brt_val);

	mutex_unlock(&led->priv->lock);

	return ret;
}

static enum led_brightness lp5024_brightness_get(struct led_classdev *led_cdev)
{
	struct lp5024_led *led = container_of(led_cdev, struct lp5024_led,
					      led_dev);
	unsigned int brt_val;
	u8 reg_val;
	int ret;

	mutex_lock(&led->priv->lock);

	if (led->ctrl_bank_enabled)
		reg_val = LP5024_BNK_BRT;
	else
		reg_val = led->led_number + LP5024_LED0_BRT;

	ret = regmap_read(led->priv->regmap, reg_val, &brt_val);

	mutex_unlock(&led->priv->lock);

	return brt_val;
}

static int lp5024_set_led_values(struct lp5024 *priv)
{
	struct lp5024_led *led;
	int i, j;
	u8 led_ctrl_enable = 0;

	for (i = 0; i <= priv->num_of_leds; i++) {
		led = &priv->leds[i];
		if (led->ctrl_bank_enabled) {
			for (j = 0; j <= LP5024_MAX_LED_STRINGS - 1; j++)
				led_ctrl_enable |= (1 << led->led_strings[j]);
		}
	}

	regmap_write(priv->regmap, LP5024_LED_CFG0, led_ctrl_enable);

	return 0;
}

static int lp5024_init(struct lp5024 *priv)
{
	int ret;

	if (priv->enable_gpio) {
		gpiod_direction_output(priv->enable_gpio, 1);
	} else {
		ret = regmap_write(priv->regmap, LP5024_RESET, LP5024_SW_RESET);
		if (ret) {
			dev_err(&priv->client->dev,
				"Cannot reset the device\n");
			goto out;
		}
	}

	ret = lp5024_set_led_values(priv);
	if (ret)
		dev_err(&priv->client->dev, "Setting the CRTL bank failed\n");

	ret = regmap_write(priv->regmap, LP5024_DEV_CFG0, LP5024_CHIP_EN);
	if (ret) {
		dev_err(&priv->client->dev, "Cannot write ctrl enable\n");
		goto out;
	}
out:
	return ret;
}

static int lp5024_probe_dt(struct lp5024 *priv)
{
	struct fwnode_handle *child = NULL;
	struct lp5024_led *led;
	const char *name;
	int led_number;
	size_t i = 0;
	int ret;

	priv->enable_gpio = devm_gpiod_get_optional(&priv->client->dev,
						   "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->enable_gpio)) {
		ret = PTR_ERR(priv->enable_gpio);
		dev_err(&priv->client->dev, "Failed to get enable gpio: %d\n",
			ret);
		return ret;
	}

	priv->regulator = devm_regulator_get(&priv->client->dev, "vled");
	if (IS_ERR(priv->regulator))
		priv->regulator = NULL;

	if (priv->model_id == LP5018)
		priv->max_leds = LP5018_MAX_LED_STRINGS;
	else
		priv->max_leds = LP5024_MAX_LED_STRINGS;

	device_for_each_child_node(&priv->client->dev, child) {
		led = &priv->leds[i];

		if (fwnode_property_present(child, "ti,control-bank"))
			led->ctrl_bank_enabled = 1;
		else
			led->ctrl_bank_enabled = 0;

		if (led->ctrl_bank_enabled) {
			ret = fwnode_property_read_u32_array(child,
							     "led-sources",
							     NULL, 0);
			ret = fwnode_property_read_u32_array(child,
							     "led-sources",
							     led->led_strings,
							     ret);

			led->led_number = led->led_strings[0];

		} else {
			ret = fwnode_property_read_u32(child, "led-sources",
					       &led_number);

			led->led_number = led_number;
		}
		if (ret) {
			dev_err(&priv->client->dev,
				"led-sources property missing\n");
			fwnode_handle_put(child);
			goto child_out;
		}

		if (led_number > priv->max_leds) {
			dev_err(&priv->client->dev,
				"led-sources property is invalid\n");
			ret = -EINVAL;
			fwnode_handle_put(child);
			goto child_out;
		}

		ret = fwnode_property_read_string(child, "label", &name);
		if (ret)
			snprintf(led->label, sizeof(led->label),
				"%s::", priv->client->name);
		else
			snprintf(led->label, sizeof(led->label),
				 "%s:%s", priv->client->name, name);

		fwnode_property_read_string(child, "linux,default-trigger",
				    &led->led_dev.default_trigger);

		led->priv = priv;
		led->led_dev.name = led->label;
		led->led_dev.brightness_set_blocking = lp5024_brightness_set;
		led->led_dev.brightness_get = lp5024_brightness_get;

		if (led->ctrl_bank_enabled)
			led->led_dev.groups = lp5024_ctrl_bank_groups;
		else
			led->led_dev.groups = lp5024_led_independent_groups;

		ret = devm_led_classdev_register(&priv->client->dev,
						 &led->led_dev);
		if (ret) {
			dev_err(&priv->client->dev, "led register err: %d\n",
				ret);
			fwnode_handle_put(child);
			goto child_out;
		}
		i++;
	}
	priv->num_of_leds = i;

child_out:
	return ret;
}

static int lp5024_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lp5024 *led;
	int count;
	int ret;

	count = device_get_child_node_count(&client->dev);
	if (!count) {
		dev_err(&client->dev, "LEDs are not defined in device tree!");
		return -ENODEV;
	}

	led = devm_kzalloc(&client->dev, struct_size(led, leds, count),
			   GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	mutex_init(&led->lock);
	led->client = client;
	led->dev = &client->dev;
	led->model_id = id->driver_data;
	i2c_set_clientdata(client, led);

	led->regmap = devm_regmap_init_i2c(client, &lp5024_regmap_config);
	if (IS_ERR(led->regmap)) {
		ret = PTR_ERR(led->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = lp5024_probe_dt(led);
	if (ret)
		return ret;

	ret = lp5024_init(led);
	if (ret)
		return ret;

	return 0;
}

static int lp5024_remove(struct i2c_client *client)
{
	struct lp5024 *led = i2c_get_clientdata(client);
	int ret;

	ret = regmap_update_bits(led->regmap, LP5024_DEV_CFG0,
				 LP5024_CHIP_EN, 0);
	if (ret) {
		dev_err(&led->client->dev, "Failed to disable regulator\n");
		return ret;
	}

	if (led->enable_gpio)
		gpiod_direction_output(led->enable_gpio, 0);

	if (led->regulator) {
		ret = regulator_disable(led->regulator);
		if (ret)
			dev_err(&led->client->dev,
				"Failed to disable regulator\n");
	}

	mutex_destroy(&led->lock);

	return 0;
}

static const struct i2c_device_id lp5024_id[] = {
	{ "lp5018", LP5018 },
	{ "lp5024", LP5024 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp5024_id);

static const struct of_device_id of_lp5024_leds_match[] = {
	{ .compatible = "ti,lp5018", },
	{ .compatible = "ti,lp5024", },
	{},
};
MODULE_DEVICE_TABLE(of, of_lp5024_leds_match);

static struct i2c_driver lp5024_driver = {
	.driver = {
		.name	= "lp5024",
		.of_match_table = of_lp5024_leds_match,
	},
	.probe		= lp5024_probe,
	.remove		= lp5024_remove,
	.id_table	= lp5024_id,
};
module_i2c_driver(lp5024_driver);

MODULE_DESCRIPTION("Texas Instruments LP5024 LED driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_LICENSE("GPL v2");
