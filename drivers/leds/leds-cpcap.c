/*
 * Copyright (c) 2017 Sebastian Reichel <sre@kernel.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/leds.h>
#include <linux/led-class-rgb.h>
#include <linux/mfd/motorola-cpcap.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define CPCAP_LED_NO_CURRENT 0x0001

struct cpcap_led_info {
	u16 reg;
	u16 mask;
	u16 limit;
	u16 init_mask;
	u16 init_val;
	u16 rgb_combo;
	u16 red_reg;
	u16 green_reg;
	u16 blue_reg;
};

static const struct cpcap_led_info cpcap_led_red = {
	.reg	= CPCAP_REG_REDC,
	.mask	= 0x03FF,
	.limit	= 31,
};

static const struct cpcap_led_info cpcap_led_green = {
	.reg	= CPCAP_REG_GREENC,
	.mask	= 0x03FF,
	.limit	= 31,
};

static const struct cpcap_led_info cpcap_led_blue = {
	.reg	= CPCAP_REG_BLUEC,
	.mask	= 0x03FF,
	.limit	= 31,
};

static const struct cpcap_led_info cpcap_led_rgb = {
	.red_reg = CPCAP_REG_REDC,
	.green_reg = CPCAP_REG_GREENC,
	.blue_reg = CPCAP_REG_BLUEC,
	.rgb_combo = 0x1,
	.mask	= 0x03FF,
	.limit	= 31,
};

/* aux display light */
static const struct cpcap_led_info cpcap_led_adl = {
	.reg		= CPCAP_REG_ADLC,
	.mask		= 0x000F,
	.limit		= 1,
	.init_mask	= 0x7FFF,
	.init_val	= 0x5FF0,
};

/* camera privacy led */
static const struct cpcap_led_info cpcap_led_cp = {
	.reg		= CPCAP_REG_CLEDC,
	.mask		= 0x0007,
	.limit		= 1,
	.init_mask	= 0x03FF,
	.init_val	= 0x0008,
};

struct cpcap_led {
	struct led_classdev led;
	struct led_classdev_rgb rgb_cdev;
	const struct cpcap_led_info *info;
	struct device *dev;
	struct regmap *regmap;
	struct mutex update_lock;
	struct regulator *vdd;
	bool powered;

	u32 current_limit;
};

static u16 cpcap_led_val(u8 current_limit, u8 duty_cycle)
{
	current_limit &= 0x1f; /* 5 bit */
	duty_cycle &= 0x0f; /* 4 bit */

	return current_limit << 4 | duty_cycle;
}

static int cpcap_led_set_power(struct cpcap_led *led, bool status)
{
	int err;

	if (status == led->powered)
		return 0;

	if (status)
		err = regulator_enable(led->vdd);
	else
		err = regulator_disable(led->vdd);

	if (err) {
		dev_err(led->dev, "regulator failure: %d", err);
		return err;
	}

	led->powered = status;

	return 0;
}

static int cpcap_led_set(struct led_classdev *ledc, enum led_brightness value)
{
	struct cpcap_led *led = container_of(ledc, struct cpcap_led, led);
	int brightness;
	int err;

	mutex_lock(&led->update_lock);

	if (led->info->rgb_combo) {
		err = 0;
		goto exit;
	}

	if (value > LED_OFF) {
		err = cpcap_led_set_power(led, true);
		if (err)
			goto exit;
	}

	if (value == LED_OFF) {
		/* Avoid HW issue by turning off current before duty cycle */
		err = regmap_update_bits(led->regmap,
			led->info->reg, led->info->mask, CPCAP_LED_NO_CURRENT);
		if (err) {
			dev_err(led->dev, "regmap failed: %d", err);
			goto exit;
		}

		brightness = cpcap_led_val(value, LED_OFF);
	} else {
		brightness = cpcap_led_val(value, LED_ON);
	}

	err = regmap_update_bits(led->regmap, led->info->reg, led->info->mask,
		brightness);
	if (err) {
		dev_err(led->dev, "regmap failed: %d", err);
		goto exit;
	}

	if (value == LED_OFF) {
		err = cpcap_led_set_power(led, false);
		if (err)
			goto exit;
	}

exit:
	mutex_unlock(&led->update_lock);
	return err;
}

static struct cpcap_led *rgbled_cdev_to_led(struct led_classdev_rgb *rgbled_cdev)
{
	return container_of(rgbled_cdev, struct cpcap_led, rgb_cdev);
}

static int cpcap_led_set_red(struct led_classdev_rgb *rgbled_cdev,
			    enum led_brightness value)
{
	struct cpcap_led *led = rgbled_cdev_to_led(rgbled_cdev);
	int brightness;
	int err;

	mutex_lock(&led->update_lock);

	if (value > LED_OFF) {
		err = cpcap_led_set_power(led, true);
		if (err)
			goto exit;
	}

	if (value == LED_OFF) {
		/* Avoid HW issue by turning off current before duty cycle */
		err = regmap_update_bits(led->regmap,
			led->info->red_reg, led->info->mask, CPCAP_LED_NO_CURRENT);
		if (err) {
			dev_err(led->dev, "regmap failed: %d", err);
			goto exit;
		}

		brightness = cpcap_led_val(value, LED_OFF);
	} else {
		brightness = cpcap_led_val(value, LED_ON);
	}

	err = regmap_update_bits(led->regmap, led->info->red_reg, led->info->mask,
		brightness);
	if (err) {
		dev_err(led->dev, "regmap failed: %d", err);
		goto exit;
	}

	if (value == LED_OFF)
		err = cpcap_led_set_power(led, false);

exit:
	mutex_unlock(&led->update_lock);
	return err;
}

static int cpcap_led_set_green(struct led_classdev_rgb *rgbled_cdev,
			    enum led_brightness value)
{
	struct cpcap_led *led = rgbled_cdev_to_led(rgbled_cdev);
	int brightness;
	int err;

	mutex_lock(&led->update_lock);

	if (value > LED_OFF) {
		err = cpcap_led_set_power(led, true);
		if (err)
			goto exit;
	}

	if (value == LED_OFF) {
		/* Avoid HW issue by turning off current before duty cycle */
		err = regmap_update_bits(led->regmap,
			led->info->green_reg, led->info->mask, CPCAP_LED_NO_CURRENT);
		if (err) {
			dev_err(led->dev, "regmap failed: %d", err);
			goto exit;
		}

		brightness = cpcap_led_val(value, LED_OFF);
	} else {
		brightness = cpcap_led_val(value, LED_ON);
	}

	err = regmap_update_bits(led->regmap, led->info->green_reg, led->info->mask,
		brightness);
	if (err) {
		dev_err(led->dev, "regmap failed: %d", err);
		goto exit;
	}

	if (value == LED_OFF)
		err = cpcap_led_set_power(led, false);
exit:
	mutex_unlock(&led->update_lock);
	return err;
}

static int cpcap_led_set_blue(struct led_classdev_rgb *rgbled_cdev,
			    enum led_brightness value)
{
	struct cpcap_led *led = rgbled_cdev_to_led(rgbled_cdev);
	int brightness;
	int err;

	mutex_lock(&led->update_lock);

	if (value > LED_OFF) {
		err = cpcap_led_set_power(led, true);
		if (err)
			goto exit;
	}

	if (value == LED_OFF) {
		/* Avoid HW issue by turning off current before duty cycle */
		err = regmap_update_bits(led->regmap,
			led->info->blue_reg, led->info->mask, CPCAP_LED_NO_CURRENT);
		if (err) {
			dev_err(led->dev, "regmap failed: %d", err);
			goto exit;
		}

		brightness = cpcap_led_val(value, LED_OFF);
	} else {
		brightness = cpcap_led_val(value, LED_ON);
	}

	err = regmap_update_bits(led->regmap, led->info->blue_reg, led->info->mask,
		brightness);
	if (err) {
		dev_err(led->dev, "regmap failed: %d", err);
		goto exit;
	}

	if (value == LED_OFF)
		err = cpcap_led_set_power(led, false);

exit:
	mutex_unlock(&led->update_lock);
	return err;
}
static int cpcap_led_set_color(struct led_classdev_rgb *rgbled_cdev)
{
	struct led_rgb_colors *colors = &rgbled_cdev->rgb_colors;
	struct cpcap_led *led = rgbled_cdev_to_led(rgbled_cdev);
	int red_brightness, green_brightness, blue_brightness;
	int err;

	mutex_lock(&led->update_lock);

	if (colors->red > LED_OFF || colors->green > LED_OFF ||
	    colors->blue > LED_OFF) {
		err = cpcap_led_set_power(led, true);
		if (err)
			goto exit;
	}

	if (colors->red == LED_OFF && colors->green == LED_OFF &&
	    colors->blue == LED_OFF) {
		/* Avoid HW issue by turning off current before duty cycle */
		err = regmap_update_bits(led->regmap,
			led->info->red_reg, led->info->mask, CPCAP_LED_NO_CURRENT);
		if (err) {
			dev_err(led->dev, "regmap failed: %d", err);
			goto exit;
		}

		err = regmap_update_bits(led->regmap,
			led->info->green_reg, led->info->mask, CPCAP_LED_NO_CURRENT);
		if (err) {
			dev_err(led->dev, "regmap failed: %d", err);
			goto exit;
		}

		err = regmap_update_bits(led->regmap,
			led->info->blue_reg, led->info->mask, CPCAP_LED_NO_CURRENT);
		if (err) {
			dev_err(led->dev, "regmap failed: %d", err);
			goto exit;
		}

		red_brightness = cpcap_led_val(colors->red, LED_OFF);
		green_brightness = cpcap_led_val(colors->green, LED_OFF);
		blue_brightness = cpcap_led_val(colors->blue, LED_OFF);
	} else {
		red_brightness = cpcap_led_val(colors->red, LED_ON);
		green_brightness = cpcap_led_val(colors->green, LED_ON);
		blue_brightness = cpcap_led_val(colors->blue, LED_ON);
	}

	err = regmap_update_bits(led->regmap, led->info->red_reg, led->info->mask,
		red_brightness);
	if (err) {
		dev_err(led->dev, "regmap failed: %d", err);
		goto exit;
	}

	err = regmap_update_bits(led->regmap, led->info->green_reg, led->info->mask,
		green_brightness);
	if (err) {
		dev_err(led->dev, "regmap failed: %d", err);
		goto exit;
	}

	err = regmap_update_bits(led->regmap, led->info->blue_reg, led->info->mask,
		blue_brightness);
	if (err) {
		dev_err(led->dev, "regmap failed: %d", err);
		goto exit;
	}

	if (colors->red == LED_OFF && colors->green == LED_OFF &&
	    colors->blue == LED_OFF)
		err = cpcap_led_set_power(led, false);

exit:
	mutex_unlock(&led->update_lock);
	return err;
}

static struct led_rgb_ops cpcap_led_rgb_ops = {
	.set_color =  cpcap_led_set_color,
	.set_red_brightness = cpcap_led_set_red,
	.set_green_brightness = cpcap_led_set_green,
	.set_blue_brightness = cpcap_led_set_blue,
};

static const struct of_device_id cpcap_led_of_match[] = {
	{ .compatible = "motorola,cpcap-led-red", .data = &cpcap_led_red },
	{ .compatible = "motorola,cpcap-led-green", .data = &cpcap_led_green },
	{ .compatible = "motorola,cpcap-led-blue",  .data = &cpcap_led_blue },
	{ .compatible = "motorola,cpcap-led-rgb",  .data = &cpcap_led_rgb },
	{ .compatible = "motorola,cpcap-led-adl", .data = &cpcap_led_adl },
	{ .compatible = "motorola,cpcap-led-cp", .data = &cpcap_led_cp },
	{},
};
MODULE_DEVICE_TABLE(of, cpcap_led_of_match);

static int cpcap_led_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct led_classdev *led_cdev;
	struct cpcap_led *led;
	int err;

	match = of_match_device(of_match_ptr(cpcap_led_of_match), &pdev->dev);
	if (!match || !match->data)
		return -EINVAL;

	led = devm_kzalloc(&pdev->dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;
	platform_set_drvdata(pdev, led);
	led->info = match->data;
	led->dev = &pdev->dev;

	if (!led->info->rgb_combo && led->info->reg == 0x0000) {
		dev_err(led->dev, "Unsupported LED");
		return -ENODEV;
	}

	led->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!led->regmap)
		return -ENODEV;

	led->vdd = devm_regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(led->vdd)) {
		err = PTR_ERR(led->vdd);
		dev_err(led->dev, "Couldn't get regulator: %d", err);
		return err;
	}

	err = device_property_read_string(&pdev->dev, "label", &led->led.name);
	if (err) {
		dev_err(led->dev, "Couldn't read LED label: %d", err);
		return err;
	}

	if (led->info->init_mask) {
		if (led->info->rgb_combo) {
			err = regmap_update_bits(led->regmap, led->info->red_reg,
				led->info->init_mask, led->info->init_val);
			if (err) {
				dev_err(led->dev, "regmap failed: %d", err);
				return err;
			}
			err = regmap_update_bits(led->regmap, led->info->green_reg,
				led->info->init_mask, led->info->init_val);
			if (err) {
				dev_err(led->dev, "regmap failed: %d", err);
				return err;
			}
			err = regmap_update_bits(led->regmap, led->info->blue_reg,
				led->info->init_mask, led->info->init_val);
			if (err) {
				dev_err(led->dev, "regmap failed: %d", err);
				return err;
			}
		} else {
			err = regmap_update_bits(led->regmap, led->info->reg,
				led->info->init_mask, led->info->init_val);
			if (err) {
				dev_err(led->dev, "regmap failed: %d", err);
				return err;
			}
		}
	}

	mutex_init(&led->update_lock);

	if (!led->info->rgb_combo) {
		led->led.max_brightness = led->info->limit;
		led->led.brightness_set_blocking = cpcap_led_set;
		err = devm_led_classdev_register(&pdev->dev, &led->led);
	} else {
		led->rgb_cdev.ops = &cpcap_led_rgb_ops;
		led_cdev = &led->rgb_cdev.led_cdev;
		led_cdev->name = led->led.name;
		led_cdev->brightness_set_blocking = cpcap_led_set;
		err = led_classdev_rgb_register(&pdev->dev, &led->rgb_cdev);
	}

	if (err) {
		dev_err(led->dev, "Couldn't register LED: %d", err);
		return err;
	}

	return 0;
}

static struct platform_driver cpcap_led_driver = {
	.probe = cpcap_led_probe,
	.driver = {
		.name = "cpcap-led",
		.of_match_table = cpcap_led_of_match,
	},
};
module_platform_driver(cpcap_led_driver);

MODULE_DESCRIPTION("CPCAP LED driver");
MODULE_AUTHOR("Sebastian Reichel <sre@kernel.org>");
MODULE_LICENSE("GPL");
