/*
 * Backlight driver for Analog Devices KTD253
 *
 * Copyright (C) 2011 Marvell Internation Ltd.
 *
 * Xiaofan Tian <tianxf@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/delay.h>

#include <mach/gpio.h>

#define KTD253_MAX_BRIGHTNESS 32

struct ktd253_data {
	int current_brightness;
	int ctrl_pin;
};

static int ktd253_bl_set(struct backlight_device *bl, int brightness)
{
	struct ktd253_data *data = bl_get_data(bl);
	int i, gpio = data->ctrl_pin;

	/* The brightness is configured by generating pulses
	 * through control pin. The relationship is shown below:
	 *   pulses	led current ratio
	 * 	1	32/32
	 * 	2	31/32
	 * 	3	30/32
	 * 	......
	 * 	31	2/32
	 * 	32	1/32
	 */

	if (brightness == data->current_brightness)
		return 0;
	else if (brightness == 0) {
		/* To shutdown backlight, pull down ctrl pin */
		i = 0;
		gpio_set_value(gpio, 0);
	} else if (data->current_brightness == 0)
		i = 33 - brightness;
	else
		i = (data->current_brightness - brightness + 32) % 32;

	pr_debug("[ktd253_backlight] set brightness to %d\n", brightness);

	for (; i > 0; i--) {
		gpio_set_value(gpio, 0);
		udelay(1);
		gpio_set_value(gpio, 1);
		udelay(1);
	}

	data->current_brightness = brightness;

	return 0;
}

static int ktd253_bl_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	return ktd253_bl_set(bl, brightness);
}

static int ktd253_bl_get_brightness(struct backlight_device *bl)
{
	struct ktd253_data *data = bl_get_data(bl);

	return data->current_brightness;
}

static const struct backlight_ops ktd253_bl_ops = {
	.update_status = ktd253_bl_update_status,
	.get_brightness = ktd253_bl_get_brightness,
};

static int __devinit ktd253_probe(struct platform_device *pdev)
{
	unsigned int *pdata = pdev->dev.platform_data;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct ktd253_data *data;
	int ret;

	if (!pdata) {
		pr_err("[ktd253_probe] no platform data?\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct ktd253_data), GFP_KERNEL);
	if (data == NULL) {
		pr_err("[ktd253_probe] failed to allocate ktd253_data!\n");
		return -ENOMEM;
	}

	data->ctrl_pin = *pdata;
	data->current_brightness = 0;

	pr_debug("[ktd253_probe] control pin %d\n", data->ctrl_pin);

	gpio_request(data->ctrl_pin, "ktd253_ctrl");

	/* Reset state machine to the init state */
	gpio_direction_output(data->ctrl_pin, 0);
	msleep(3);

	memset(&props, 0, sizeof(props));

	props.max_brightness = KTD253_MAX_BRIGHTNESS;
	props.brightness = KTD253_MAX_BRIGHTNESS;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register("backlight-0", &pdev->dev, data,
				       &ktd253_bl_ops, &props);
	if (IS_ERR(bl)) {
		pr_err("[ktd253_probe] failed to register backlight device\n");
		ret = PTR_ERR(bl);
		goto out;
	}

	platform_set_drvdata(pdev, bl);

	backlight_update_status(bl);

	pr_debug("[ktd253_probe] backlight register successfully\n");

	return 0;

out:
	gpio_free(data->ctrl_pin);
	kfree(data);

	return ret;
}

static int __devexit ktd253_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct ktd253_data *data = bl_get_data(bl);

	bl->props.brightness = 0;
	backlight_update_status(bl);

	backlight_device_unregister(bl);

	gpio_free(data->ctrl_pin);

	kfree(data);

	return 0;
}

static struct platform_driver ktd253_driver = {
	.driver = {
		.name = "ktd253_bl",
		.owner = THIS_MODULE,
	},
	.probe = ktd253_probe,
	.remove = __devexit_p(ktd253_remove),
};

static int __init ktd253_init(void)
{
	return platform_driver_register(&ktd253_driver);
}

module_init(ktd253_init);

static void __exit ktd253_exit(void)
{
	platform_driver_unregister(&ktd253_driver);
}

module_exit(ktd253_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xiaofan Tian <tianxf@marvell.com>");
MODULE_DESCRIPTION("ktd253 backlight driver");
