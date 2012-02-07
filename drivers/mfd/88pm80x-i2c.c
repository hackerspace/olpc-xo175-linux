/*
 * I2C driver for Marvell 88PM80x
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mfd/88pm80x.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

/*
#define CACHE_88PM80X_I2C
	not implemented yet
*/

#if defined(CACHE_88PM80X_I2C)

#include "88pm80x-cache.h"

#else

#define pmic_cache_stat_print(RST)
#define pmic_cache_init(ID)
#define pmic_cache_hit_before_read(ID, REG, COUNT, PDATA)		(-1)
#define pmic_cache_hit_before_write(ID, REG, COUNT, PDATA)		(-1)
#define pmic_cache_save_after_readwrite(ID, I2C_RET, REG, COUNT, PDATA)
#define pmic_cache_invalidate(ID, REG)

#endif /*CACHE_88PM80X_I2C */

static struct pm80x_chip *g_pm80x_chip;

/**
************  LOCAL UTILITIES  ****************************
**/


static int read_device(struct i2c_client *i2c, int reg,
		       int bytes, void *dest)
{
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX + 3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX + 2];
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[2] = {{i2c->addr, 0, 1, msgbuf0},
				 {i2c->addr, I2C_M_RD, 0, msgbuf1},
				};
	int num = 1, ret = 0;

	if (dest == NULL)
		return -EINVAL;
	msgbuf0[0] = (unsigned char)reg;	/* command */
	msg[1].len = bytes;

	/* if data needs to read back, num should be 2 */
	if (bytes > 0)
		num = 2;
	ret = adap->algo->master_xfer(adap, msg, num);
	memcpy(dest, msgbuf1, bytes);
	if (ret < 0)
		return ret;
	return 0;
}

static int write_device(struct i2c_client *i2c, int reg,
			int bytes, void *src)
{
	unsigned char buf[bytes + 1];
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg;
	int ret;

	buf[0] = (unsigned char)reg;
	memcpy(&buf[1], src, bytes);
	msg.addr = i2c->addr;
	msg.flags = 0;
	msg.len = bytes + 1;
	msg.buf = buf;

	ret = adap->algo->master_xfer(adap, &msg, 1);
	if (ret < 0)
		return ret;
	return 0;
}


int pm805_debug_reg_read(struct i2c_client *i2c, int reg)
{
	struct pm80x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char zero = 0;
	unsigned char data = 0;
	int ret;

	mutex_lock(&chip->io_lock);
	i2c_lock_adapter(i2c->adapter);
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	ret = read_device(i2c, reg, 1, &data);
	if (ret >= 0)
		ret = (int)data;
	read_device(i2c, 0xFC, 0, &zero);
	i2c_unlock_adapter(i2c->adapter);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm805_debug_reg_read);

int pm805_debug_reg_write(struct i2c_client *i2c, int reg,
			  unsigned char data)
{
	struct pm80x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char zero;
	int ret;

	mutex_lock(&chip->io_lock);
	i2c_lock_adapter(i2c->adapter);
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	ret = write_device(i2c, reg, 1, &data);
	read_device(i2c, 0xFC, 0, &zero);
	i2c_unlock_adapter(i2c->adapter);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm805_debug_reg_write);

static inline int pm80x_read_device(struct i2c_client *i2c,
				    int reg, int bytes, void *dest)
{
	int ret;

	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return ret;
}

static inline int pm80x_write_device(struct i2c_client *i2c,
				     int reg, int bytes, void *src)
{
	unsigned char buf[bytes + 1];
	int ret;

	buf[0] = (unsigned char)reg;
	memcpy(&buf[1], src, bytes);

	ret = i2c_master_send(i2c, buf, bytes + 1);
	if (ret < 0)
		return ret;
	return 0;
}

/**
************  GLOBAL APIs  ****************************
**/
int pm80x_reg_read(struct i2c_client *i2c, int reg)
{
	unsigned char data = 0;
	int ret;
	ret = pm80x_bulk_read(i2c, reg, 1, &data);
	if (ret < 0)
		return ret;
	else
		return (int)data;
}
EXPORT_SYMBOL(pm80x_reg_read);

int pm80x_reg_write(struct i2c_client *i2c, int reg, unsigned char data)
{
	return pm80x_bulk_write(i2c, reg, 1, &data);
}
EXPORT_SYMBOL(pm80x_reg_write);

static inline struct i2c_client *get_i2c_client(int reg)
{
	switch (reg >> 8) {
	case PM80X_BASE_PAGE:
		return g_pm80x_chip->base_page;
	case PM80X_POWER_PAGE:
		return g_pm80x_chip->power_page;
	case PM80X_GPADC_PAGE:
		return g_pm80x_chip->gpadc_page;
	case PM80X_TEST_PAGE:
		return g_pm80x_chip->test_page;
	default:
		return NULL;
	}
}

int pm80x_codec_reg_read(int reg)
{
	unsigned char data = 0;
	int ret;
	struct i2c_client *i2c = get_i2c_client(reg);
	BUG_ON(!i2c);
	reg &= 0xff;
	ret = pm80x_bulk_read(i2c, reg, 1, &data);
	if (ret < 0)
		return ret;
	else
		return (int)data;
}
EXPORT_SYMBOL(pm80x_codec_reg_read);

int pm80x_codec_reg_write(int reg, unsigned char data)
{
	struct i2c_client *i2c = get_i2c_client(reg);
	BUG_ON(!i2c);
	reg &= 0xff;
	return pm80x_bulk_write(i2c, reg, 1, &data);
}
EXPORT_SYMBOL(pm80x_codec_reg_write);

int pm80x_codec_reg_set_bits(int reg, unsigned char mask, unsigned char data)
{
	int ret;
	struct i2c_client *i2c = get_i2c_client(reg);
	BUG_ON(!i2c);
	reg &= 0xff;
	/*we have mutex protect in pm80x_set_bits() */
	ret = pm80x_set_bits(i2c, reg, mask, data);
	return ret;
}
EXPORT_SYMBOL(pm80x_codec_reg_set_bits);

int pm80x_bulk_read(struct i2c_client *i2c, int reg,
		    int count, unsigned char *buf)
{
	struct pm80x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pmic_cache_hit_before_read(chip->id, reg, count, buf);
	if (ret < 0) {
		ret = pm80x_read_device(i2c, reg, count, buf);
		pmic_cache_save_after_readwrite(chip->id, ret, reg, count, buf);
	}
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm80x_bulk_read);

int pm80x_bulk_write(struct i2c_client *i2c, int reg,
		     int count, unsigned char *buf)
{
	struct pm80x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pmic_cache_hit_before_write(chip->id, reg, count, buf);
	if (ret < 0) {
		ret = pm80x_write_device(i2c, reg, count, buf);
		pmic_cache_save_after_readwrite(chip->id, ret, reg, count, buf);
	}
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm80x_bulk_write);

int pm80x_set_bits(struct i2c_client *i2c, int reg,
		   unsigned char mask, unsigned char data)
{
	struct pm80x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char valget, valset;
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pmic_cache_hit_before_read(chip->id, reg, 1, &valget);
	if (ret < 0) {
		ret = pm80x_read_device(i2c, reg, 1, &valget);
		pmic_cache_save_after_readwrite(chip->id, ret, reg, 1, &valget);
	}
	if (ret >= 0) {
		valset = valget;
		valset &= ~mask;
		valset |= data;

		ret = pm80x_write_device(i2c, reg, 1, &valset);
		pmic_cache_save_after_readwrite(chip->id, ret, reg, 1, &valset);

	}
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm80x_set_bits);

static const struct i2c_device_id pm80x_id_table[] = {
	{"88PM80x", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, pm80x_id_table);

static int verify_addr(struct i2c_client *i2c)
{
	unsigned short addr_800[] = { 0x30, 0x34 };
	unsigned short addr_805[] = { 0x38, 0x39 };
	int size, i;

	if (i2c == NULL)
		return 0;
	size = ARRAY_SIZE(addr_800);
	for (i = 0; i < size; i++) {
		if (i2c->addr == *(addr_800 + i))
			return CHIP_PM800;
	}
	size = ARRAY_SIZE(addr_805);
	for (i = 0; i < size; i++) {
		if (i2c->addr == *(addr_805 + i))
			return CHIP_PM805;
	}
	return 0;
}

static int pm800_pages_init(struct pm80x_chip *chip,
		struct pm80x_platform_data *pdata, struct i2c_client *client)
{
	/* PM800 block base 0x30 */
	if (pdata->base_page_addr) {
		chip->base_page_addr = pdata->base_page_addr;
		chip->base_page = i2c_new_dummy(chip->client->adapter,
						chip->base_page_addr);
		i2c_set_clientdata(chip->base_page, chip);
	} else
		dev_info(&client->dev,
			 "PM800 block base 0x30: No base_page_addr\n");

	/* PM800 block power 0x31 */
	if (pdata->power_page_addr &&
		(pdata->power_page_addr != client->addr)) {
		chip->power_page_addr = pdata->power_page_addr;
		chip->power_page = i2c_new_dummy(chip->client->adapter,
						 chip->power_page_addr);
		i2c_set_clientdata(chip->power_page, chip);
	} else
		dev_info(&client->dev,
			 "PM800 block power 0x31: No power_page_addr\n");

	/* PM800 block GPADC 0x32 */
	if (pdata->gpadc_page_addr &&
		(pdata->gpadc_page_addr != client->addr)) {
		chip->gpadc_page_addr = pdata->gpadc_page_addr;
		chip->gpadc_page = i2c_new_dummy(chip->client->adapter,
						 chip->gpadc_page_addr);
		i2c_set_clientdata(chip->gpadc_page, chip);
	} else
		dev_info(&client->dev,
			 "PM800 block GPADC 0x32: No gpadc_page_addr\n");

	/* PM800 block test page 0x37 */
	if (pdata->test_page_addr &&
		(pdata->test_page_addr != client->addr)) {
		chip->test_page_addr = pdata->test_page_addr;
		chip->test_page = i2c_new_dummy(chip->client->adapter,
						chip->test_page_addr);
		i2c_set_clientdata(chip->test_page, chip);
	} else
		dev_info(&client->dev,
			 "PM800 block test page 0x37: No test_page_addr\n");
	return 0;
}

static int __devinit pm80x_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct pm80x_platform_data *pdata = client->dev.platform_data;
	struct pm80x_chip *chip;

	if (!pdata) {
		dev_info(&client->dev, "No platform data in %s!\n", __func__);
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm80x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->id = verify_addr(client);
	chip->client = client;
	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
	dev_set_drvdata(chip->dev, chip);

	chip->irq_base = pdata->irq_base;
	chip->irq_companion = pdata->irq_companion;

	/*
	 * Both client and companion client shares same platform driver.
	 * Driver distinguishes them by pdata->companion_addr.
	 * pdata->companion_addr is only assigned if companion chip exists.
	 * At the same time, the companion_addr shouldn't equal to client
	 * address.
	 */
	/* Companion chip */
	if (pdata->companion_addr && (pdata->companion_addr != client->addr)) {
		chip->companion_addr = pdata->companion_addr;
		chip->companion = i2c_new_dummy(chip->client->adapter,
						chip->companion_addr);
		i2c_set_clientdata(chip->companion, chip);
		dev_info(&client->dev,
			 "companion_addr=0x%x\n", chip->companion_addr);
	} else
		dev_info(&client->dev, "No companion_addr\n");

	if (chip->id == CHIP_PM800)
		pm800_pages_init(chip, pdata, client);

	pm80x_device_init(chip, pdata);

	pmic_cache_init(chip->id);

	if (chip->id == CHIP_PM800)
		g_pm80x_chip = chip;

	return 0;
}

static int __devexit pm80x_remove(struct i2c_client *client)
{
	struct pm80x_chip *chip = i2c_get_clientdata(client);

	pm80x_device_exit(chip);

	if (chip->companion)
		i2c_unregister_device(chip->companion);
	if (chip->base_page)
		i2c_unregister_device(chip->base_page);
	if (chip->power_page)
		i2c_unregister_device(chip->power_page);
	if (chip->gpadc_page)
		i2c_unregister_device(chip->gpadc_page);
	if (chip->test_page)
		i2c_unregister_device(chip->test_page);

	kfree(chip);
	return 0;
}

static struct i2c_driver pm80x_driver = {
	.driver = {
		   .name = "88PM80x",
		   .owner = THIS_MODULE,
		   },
	.probe = pm80x_probe,
	.remove = __devexit_p(pm80x_remove),
	.id_table = pm80x_id_table,
};

static int __init pm80x_i2c_init(void)
{
	int ret;
	ret = i2c_add_driver(&pm80x_driver);
	if (ret != 0)
		pr_err("Failed to register 88PM80x I2C driver: %d\n", ret);
	return ret;
}

subsys_initcall(pm80x_i2c_init);

static void __exit pm80x_i2c_exit(void)
{
	i2c_del_driver(&pm80x_driver);
}

module_exit(pm80x_i2c_exit);

MODULE_DESCRIPTION("I2C Driver for Marvell 88PM80x");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
