// SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-or-later
/*
 * ENE KB3930 Embedded Controller Driver
 *
 * Copyright (C) 2020 Lubomir Rintel
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>

enum {
	EC_DATA_IN	= 0x00,
	EC_RAM_OUT	= 0x80,
	EC_RAM_IN	= 0x81,
};

enum {
	EC_MODEL_ID	= 0x30,
	EC_VERSION_MAJ	= 0x31,
	EC_VERSION_MIN	= 0x32,
};

struct kb3930 {
	struct i2c_client *client;
	struct regmap *ec_ram;
	struct gpio_descs *off_gpios;
};

struct kb3930 *global_kb3930;

static void kb3930_off(struct kb3930 *priv, int poweroff)
{
	gpiod_direction_output(priv->off_gpios->desc[1], poweroff);

	while (1) {
		mdelay(50);
		gpiod_direction_output(priv->off_gpios->desc[0], 0);
		mdelay(50);
		gpiod_direction_output(priv->off_gpios->desc[0], 1);
	}
}

static int kb3930_restart(struct notifier_block *this,
			  unsigned long mode, void *cmd)
{
	kb3930_off(global_kb3930, 0);
	return NOTIFY_DONE;
}

static void kb3930_power_off(void)
{
	kb3930_off(global_kb3930, 1);
}

static struct notifier_block kb3930_restart_nb = {
	.notifier_call = kb3930_restart,
	.priority = 128,
};

static const struct mfd_cell ariel_ec_cells[] = {
	{ .name = "dell-wyse-ariel-led", },
	{ .name = "dell-wyse-ariel-power", },
};

static int kb3930_ec_ram_reg_write(void *context, unsigned int reg,
				   unsigned int val)
{
	struct kb3930 *priv = context;

	return i2c_smbus_write_word_data(priv->client, EC_RAM_OUT,
					 (val << 8) | reg);
}

static int kb3930_ec_ram_reg_read(void *context, unsigned int reg,
				  unsigned int *val)
{
	struct kb3930 *priv = context;
	int ret;

	ret = i2c_smbus_write_word_data(priv->client, EC_RAM_IN, reg);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_word_data(priv->client, EC_DATA_IN);
	if (ret < 0)
		return ret;

	*val = ret >> 8;
	return 0;
}

static const struct regmap_config kb3930_ec_ram_regmap_config = {
	.name = "ec_ram",
	.reg_bits = 8,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0xff,
	.reg_write = kb3930_ec_ram_reg_write,
	.reg_read = kb3930_ec_ram_reg_read,
	.fast_io = false,
};

static int kb3930_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct kb3930 *priv;
	unsigned int model_id;
	int ret;

	if (global_kb3930)
		return -EEXIST;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	global_kb3930 = priv;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->ec_ram = devm_regmap_init(dev, NULL, priv,
					&kb3930_ec_ram_regmap_config);
	if (IS_ERR(priv->ec_ram))
		return PTR_ERR(priv->ec_ram);

	ret = regmap_read(priv->ec_ram, EC_MODEL_ID, &model_id);
	if (ret < 0)
		return ret;

	if (model_id == 'J') {
		ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE,
					   ariel_ec_cells,
					   ARRAY_SIZE(ariel_ec_cells),
					   NULL, 0, NULL);
		if (ret < 0)
			return ret;
	} else {
		dev_err(dev, "unknown board model: %02x\n", model_id);
		return -ENODEV;
	}

	if (of_property_read_bool (np, "system-power-controller")) {
		priv->off_gpios = devm_gpiod_get_array_optional(dev, "off",
								GPIOD_IN);
	}
	if (IS_ERR(priv->off_gpios))
		return PTR_ERR(priv->off_gpios);
	if (priv->off_gpios->ndescs < 2) {
		dev_err(dev, "invalid off-gpios property\n");
		return -EINVAL;
	}
	if (priv->off_gpios) {
		register_restart_handler(&kb3930_restart_nb);
		if (pm_power_off == NULL)
			pm_power_off = kb3930_power_off;
	}

	dev_info(dev, "ENE KB3930 Embedded Controller\n");
	return 0;
}

static int kb3930_remove(struct i2c_client *client)
{
	struct kb3930 *priv = i2c_get_clientdata(client);

	if (priv->off_gpios) {
		if (pm_power_off == kb3930_power_off)
			pm_power_off = NULL;
		unregister_restart_handler(&kb3930_restart_nb);
	}
	global_kb3930 = NULL;

	return 0;
}

static const struct i2c_device_id kb3930_ids[] = {
	{ "kb3930", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, kb3930_ids);

static const struct of_device_id kb3930_dt_ids[] = {
	{ .compatible = "ene,kb3930" },
	{ }
};
MODULE_DEVICE_TABLE(of, kb3930_dt_ids);

static struct i2c_driver kb3930_driver = {
	.probe = kb3930_probe,
	.remove = kb3930_remove,
	.driver = {
		.name = "ene-kb3930",
		.of_match_table = of_match_ptr(kb3930_dt_ids),
	},
	.id_table = kb3930_ids,
};

module_i2c_driver(kb3930_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("ENE KB3930 Embedded Controller Driver");
MODULE_LICENSE("Dual BSD/GPL");
