/*
 * Maxim MAX77601 I2C driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77601.h>

#define RTC_I2C_ADDR	0x48
static struct max77601_chip *max77601_chip_p;
static int max77601_i2c_write(struct i2c_client *i2c, unsigned char addr,
			       void *src, unsigned int bytes)
{
	unsigned char buf[bytes + 1];
	int ret;
	buf[0] = addr;
	memcpy(&buf[1], src, bytes);
	ret = i2c_master_send(i2c, buf, bytes + 1);
	if (ret < 0)
		return ret;
	return 0;
}

static int max77601_i2c_read(struct i2c_client *i2c, unsigned char addr,
				unsigned char *dest, unsigned int bytes)
{
	int ret;
	if (bytes > 1) {
		ret =
		    i2c_smbus_read_i2c_block_data(i2c, addr, bytes, dest);
	} else {
		ret = i2c_smbus_read_byte_data(i2c, addr);
		if (ret < 0)
			return ret;
		*dest = (unsigned char) ret;
	}
	return 0;
}

int max77601_write(struct max77601_chip *chip, u8 addr, u8 * values,
		     unsigned int len)
{
	int ret;
	if (chip == NULL)
		return -EINVAL;
	mutex_lock(&chip->io_lock);
	ret = max77601_i2c_write(chip->i2c, addr, values, len);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77601_write);

int max77601_read(struct max77601_chip *chip, u8 addr, u8 * values,
		    unsigned int len)
{
	int ret;
	if (chip == NULL)
		return -EINVAL;
	mutex_lock(&chip->io_lock);
	ret = max77601_i2c_read(chip->i2c, addr, values, len);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77601_read);

int max77601_set_bits(struct max77601_chip *chip, u8 addr, u8 mask,
			u8 value)
{
	u8 tmp;
	int ret;
	if (chip == NULL)
		return -EINVAL;
	mutex_lock(&chip->io_lock);
	ret = max77601_i2c_read(chip->i2c, addr, &tmp, 1);
	if (ret == 0) {
		value = (tmp & ~mask) | (value & mask);
		ret = max77601_i2c_write(chip->i2c, addr, &value, 1);
	}
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77601_set_bits);

int max77601_pmic_reg_write(int reg, unsigned char data)
{
	u8 addr = (unsigned char) reg;
	return max77601_write(max77601_chip_p, addr, &data, 1);
}
EXPORT_SYMBOL(max77601_pmic_reg_write);

int max77601_pmic_reg_read(int reg)
{
	u8 addr = (unsigned char) reg;
	u8 value = 0;
	int ret;
	ret = max77601_read(max77601_chip_p, addr, &value, 1);
	if (ret < 0)
		return ret;
	else
		return value;
}
EXPORT_SYMBOL(max77601_pmic_reg_read);

int max77601_pmic_set_bits(int reg, unsigned char mask,
			     unsigned char data)
{
	u8 addr = (unsigned char) reg;
	return max77601_set_bits(max77601_chip_p, addr, mask, data);
}
EXPORT_SYMBOL(max77601_pmic_set_bits);

#ifdef CONFIG_PM
static int max77601_suspend(struct device *dev)
{
	struct i2c_client *client;
	struct max77601_chip *chip;
	client = to_i2c_client(dev);
	chip = i2c_get_clientdata(client);
	return 0;
}

static int max77601_resume(struct device *dev)
{
	struct i2c_client *client;
	struct max77601_chip *chip;
	client = to_i2c_client(dev);
	chip = i2c_get_clientdata(client);
	return 0;
}

#else
#define max77601_suspend      NULL
#define max77601_resume       NULL
#endif

static struct dev_pm_ops max77601_pm = {
	.suspend = max77601_suspend,
	.resume = max77601_resume,
};

static const struct i2c_device_id max77601_id[] = {
	    {"max77601", 0},
		{},
};
MODULE_DEVICE_TABLE(i2c, max77601_id);

static int max77601_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct max77601_platform_data *pdata = client->dev.platform_data;
	struct max77601_chip *chip;
	if (!pdata) {
		pr_err("%s: Invalid platform_data.\n", __func__);
		return -ENODEV;
	}
	chip = kzalloc(sizeof(struct max77601_chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}
	max77601_chip_p = chip;
	chip->i2c = client;
	chip->dev = &client->dev;
	chip->irq_base = pdata->irq_base;
	i2c_set_clientdata(client, chip);
	dev_set_drvdata(chip->dev, chip);
	mutex_init(&chip->io_lock);
	chip->rtc = i2c_new_dummy(chip->i2c->adapter, RTC_I2C_ADDR);
	i2c_set_clientdata(chip->rtc, chip);
	if (pdata->setup)
		pdata->setup(chip);
	max77601_device_init(chip, pdata);
	return 0;
}

static int __devexit max77601_remove(struct i2c_client *client)
{
	struct max77601_chip *chip = i2c_get_clientdata(client);
	max77601_device_exit(chip);
	if (chip) {
		i2c_unregister_device(chip->rtc);
		mutex_destroy(&chip->io_lock);
		chip->i2c = NULL;
		kfree(chip);
	}
	return 0;
}

static struct i2c_driver max77601_driver = {
	.driver = {
		.name = "max77601",
		.owner = THIS_MODULE,
		.pm = &max77601_pm,
	},
	.probe = max77601_probe,
	.remove = __devexit_p(max77601_remove),
	.id_table = max77601_id,
};

static int __init max77601_i2c_init(void)
{
	int rc;
	rc = i2c_add_driver(&max77601_driver);
	if (rc != 0)
		pr_err("Failed to add max77601 i2c driver: rc = %d\n",
			rc);
	else
		pr_info("%s: i2c add driver: rc = %d\n",
			 __func__, rc);
	return rc;
}
subsys_initcall(max77601_i2c_init);

static void __exit max77601_i2c_exit(void)
{
	i2c_del_driver(&max77601_driver);
}
module_exit(max77601_i2c_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("I2C Driver for Maxim 77601");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:max77601-i2c");
