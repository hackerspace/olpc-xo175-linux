// SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-or-later
/*
 * Dell Wyse 3020 a.k.a. "Ariel" Embedded Controller Driver
 *
 * Copyright (C) 2020 Lubomir Rintel
 */

#include <linux/module.h>
#include <linux/i2c.h>

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

struct ariel_ec_priv {
	struct i2c_client *client;
};

static void ram_write(struct i2c_client *client, u8 addr, u8 value)
{
	i2c_smbus_write_word_data(client, RAM_OUT, (value << 8) | addr);
}

static int ram_read(struct i2c_client *client, u8 addr)
{
	i2c_smbus_write_word_data(client, RAM_IN, addr);
	return i2c_smbus_read_word_data(client, DATA_IN) >> 8;
}

static int ariel_ec_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ariel_ec_priv *priv;
	u8 model_id;

	model_id = ram_read(client, MODEL_ID);
	if (model_id != '0') {
		dev_err(dev, "unknown model: %02x\n", model_id);
		return -ENODEV;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	priv->client = client;

	ram_write (client, ORANGE_LED, LED_OFF);
	ram_write (client, GREEN_LED, LED_STILL);

	dev_info(dev, "Dell Wyse 3020 Embedded Controller\n");
	return 0;
}

static int ariel_ec_remove(struct i2c_client *client)
{
//	struct device *dev = &client->dev;
//	struct ariel_ec_priv *priv = dev_get_drvdata(dev);

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
