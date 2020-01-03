// SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-or-later
/*
 * Dell Wyse 3020 a.k.a. "Ariel" Embedded Controller Driver
 *
 * Copyright (C) 2020 Lubomir Rintel
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/reboot.h>

enum {
	DATA_IN		= 0x00,
	RAM_OUT		= 0x80,
	RAM_IN		= 0x81,
};

enum {
	ORANGE_LED	= 0x02,
	GREEN_LED	= 0x03,
	USB1_POWER	= 0x12,
	USB2_POWER	= 0x13,
	USB3_POWER	= 0x14,
	USB4_POWER	= 0x15,
	ALWAYS_ON	= 0x16,
	WAKE_ON_LAN	= 0x20,
	WAKE_ON_USB	= 0x21,
	MODEL_ID	= 0x30,
	VERSION_MAJ	= 0x31,
	VERSION_MIN	= 0x32,
};

enum {
	LED_OFF		= 0x00,
	LED_STILL	= 0x01,
	LED_FADE	= 0x02,
	LED_BLINK	= 0x03,
};

struct ariel_ec {
	struct i2c_client *client;
        struct gpio_desc *off0_gpio;
        struct gpio_desc *off1_gpio;
};

struct ariel_ec *global_ariel_ec;

static void ram_write(struct i2c_client *client, u8 addr, u8 value)
{
	i2c_smbus_write_word_data(client, RAM_OUT, (value << 8) | addr);
}

static int ram_read(struct i2c_client *client, u8 addr)
{
	i2c_smbus_write_word_data(client, RAM_IN, addr);
	return i2c_smbus_read_word_data(client, DATA_IN) >> 8;
}

static void ariel_ec_off(struct ariel_ec *priv, int poweroff)
{
	ram_write (priv->client, ORANGE_LED, LED_STILL);
	ram_write (priv->client, GREEN_LED, LED_OFF);

        gpiod_direction_output(priv->off1_gpio, poweroff);

        while (1) {
                mdelay(50);
                gpiod_direction_output(priv->off0_gpio, 0);
                mdelay(50);
                gpiod_direction_output(priv->off0_gpio, 1);
        }
}

static int ariel_ec_reboot(struct notifier_block *this,
                           unsigned long mode, void *cmd)
{
        ariel_ec_off(global_ariel_ec, 0);
        return NOTIFY_DONE;
}

static void ariel_ec_power_off(void)
{
        ariel_ec_off(global_ariel_ec, 1);
}

static struct notifier_block ariel_ec_reboot_nb = {
        .notifier_call = ariel_ec_reboot,
        .priority = 128,
};

static int ariel_ec_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ariel_ec *priv;
	u8 model_id;

	if (global_ariel_ec)
		return -EEXIST;

	model_id = ram_read(client, MODEL_ID);
	if (model_id != 'J') {
		dev_err(dev, "unknown model: %02x\n", model_id);
		return -ENODEV;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	global_ariel_ec = priv;
	dev_set_drvdata(dev, priv);
	priv->client = client;

	priv->off0_gpio = devm_gpiod_get_index(dev, "off", 0, GPIOD_IN);
	if (IS_ERR(priv->off0_gpio)) {
		dev_info(dev, "failed to request OFF0 GPIO\n");
		priv->off0_gpio = NULL;
	};
	priv->off1_gpio = devm_gpiod_get_index(dev, "off", 1, GPIOD_IN);
	if (IS_ERR(priv->off1_gpio)) {
		dev_info(dev, "failed to request OFF1 GPIO\n");
		priv->off1_gpio = NULL;
	};

	devm_register_reboot_notifier(dev, &ariel_ec_reboot_nb);

	if (pm_power_off == NULL)
		pm_power_off = ariel_ec_power_off;

	ram_write (client, ORANGE_LED, LED_OFF);
	ram_write (client, GREEN_LED, LED_STILL);

	dev_info(dev, "Dell Wyse 3020 Embedded Controller\n");
	return 0;
}

static int ariel_ec_remove(struct i2c_client *client)
{
//	struct device *dev = &client->dev;
//	struct ariel_ec *priv = dev_get_drvdata(dev);

	global_ariel_ec = NULL;
        if (pm_power_off == ariel_ec_power_off)
		pm_power_off = NULL;

	return 0;
}

static const struct of_device_id ariel_ec_dt_ids[] = {
	{ .compatible = "dell,wyse-ariel-ec", },
	{ }
};
MODULE_DEVICE_TABLE(of, ariel_ec_dt_ids);

static const struct i2c_device_id ariel_ec_ids[] = {
	{ "wyse-ariel-ec", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ariel_ec_ids);

static struct i2c_driver ariel_ec_driver = {
	.probe = ariel_ec_probe,
	.remove = ariel_ec_remove,
	.driver = {
		.name = "wyse-ariel-ec",
		.of_match_table = of_match_ptr(ariel_ec_dt_ids),
	},
	.id_table = ariel_ec_ids,
};

module_i2c_driver(ariel_ec_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("Dell Wyse 3020 Embedded Controller Driver");
MODULE_LICENSE("Dual BSD/GPL");
