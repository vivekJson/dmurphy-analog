// SPDX-License-Identifier: GPL-2.0
// LED RGB class interface
// Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/led-class-rgb.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <uapi/linux/uleds.h>

struct rgb_fw_test_led {
	struct led_classdev_rgb rgbled_cdev;
	struct i2c_client *client;

	u32 white_setting[3];

	u32 led_strings[3];

	char led_name[LED_MAX_NAME_SIZE];
	u8 available_rgbs;
};

static struct rgb_fw_test_led *rgbled_cdev_to_led(struct led_classdev_rgb *rgbled_cdev)
{
	return container_of(rgbled_cdev, struct rgb_fw_test_led, rgbled_cdev);
}

static int rgb_fw_set_red(struct led_classdev_rgb *rgbled_cdev,
			    enum led_brightness brightness)
{
	printk("%s: Brightness is 0x%X\n", __func__, brightness);
	return 0;
}

static int rgb_fw_set_green(struct led_classdev_rgb *rgbled_cdev,
			    enum led_brightness brightness)
{

	printk("%s: Brightness is 0x%X\n", __func__, brightness);
	return 0;
}

static int rgb_fw_set_blue(struct led_classdev_rgb *rgbled_cdev,
			    enum led_brightness brightness)
{

	printk("%s: Brightness is 0x%X\n", __func__, brightness);
	return 0;
}

static int rgb_fw_set_color(struct led_classdev_rgb *rgbled_cdev)
{
//	struct rgb_fw_test_led *led = rgbled_cdev_to_led(rgbled_cdev);
	struct led_rgb_colors *colors = &rgbled_cdev->rgb_colors;

	printk("%s: Would set red to %d\n", __func__, colors->red);
	printk("%s: Would set green to %d\n", __func__, colors->green);
	printk("%s: Would set blue to %d\n", __func__, colors->blue);

	return 0;
}

static struct led_rgb_ops rgb_test_ops = {
	.set_color = rgb_fw_set_color,
/* Need to do get functions */
//	.get_color = rgb_fw_get_color,
};
static int rgb_fw_test_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{

	printk("%s: Brightness is 0x%X\n", __func__, brightness);
	return 0;
}

static int rgb_fw_test_register_leds(struct rgb_fw_test_led *led)
{
	struct led_classdev *led_cdev;

	led->rgbled_cdev.ops = &rgb_test_ops;

	if (led->led_strings[0] != 0xff)
		led->rgbled_cdev.ops->set_red_brightness = rgb_fw_set_red;
	if (led->led_strings[1] != 0xff)
		led->rgbled_cdev.ops->set_green_brightness = rgb_fw_set_green;
	if (led->led_strings[2] != 0xff)
		led->rgbled_cdev.ops->set_blue_brightness = rgb_fw_set_blue;

	led_cdev = &led->rgbled_cdev.led_cdev;
	led_cdev->brightness_set_blocking = rgb_fw_test_brightness_set;
	led_cdev->name = led->led_name;

	return led_classdev_rgb_register(&led->client->dev, &led->rgbled_cdev);
}

static int rgb_fw_test_probe_node(struct rgb_fw_test_led *led)
{

	struct fwnode_handle *child = NULL;
	const char *name;
	int ret;

/* Need to set this up for multiple nodes this is not working since we reuse
the same led node.  Need to fix that to continue */
	device_for_each_child_node(&led->client->dev, child) {
		ret = fwnode_property_read_string(child, "label", &name);

		snprintf(led->led_name, sizeof(led->led_name),
			 "%s:%s", led->client->name, name);

		ret = fwnode_property_read_u32_array(child, "rgb-sources",
						     NULL, 0);
		ret = fwnode_property_read_u32_array(child, "rgb-sources",
						     led->led_strings, ret);

		ret = rgb_fw_test_register_leds(led);
	}
	return 0;
}

static int rgb_fw_test_probe(struct i2c_client *client)
{
	struct rgb_fw_test_led *led;
	int ret;

	led = devm_kzalloc(&client->dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->client = client;
	i2c_set_clientdata(client, led);

	ret = rgb_fw_test_probe_node(led);
	if (ret)
		return -ENODEV;

	return ret;
}

static int rgb_fw_test_remove(struct i2c_client *client)
{
	struct rgb_fw_test_led *led = i2c_get_clientdata(client);

	led_classdev_rgb_unregister(&led->rgbled_cdev);

	return 0;
}

static const struct i2c_device_id rgb_fw_test_id[] = {
	{ "RGBNONE", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3601x_id);

static const struct of_device_id of_rgb_fw_test_leds_match[] = {
	{ .compatible = "rgb,framework_test", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_rgb_fw_test_leds_match);

static struct i2c_driver rgb_fw_test_i2c_driver = {
	.driver = {
		.name = "rgb_framework_test",
		.of_match_table = of_rgb_fw_test_leds_match,
	},
	.probe_new = rgb_fw_test_probe,
	.remove = rgb_fw_test_remove,
	.id_table = rgb_fw_test_id,
};
module_i2c_driver(rgb_fw_test_i2c_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("RGB LED class interface");
MODULE_LICENSE("GPL v2");
