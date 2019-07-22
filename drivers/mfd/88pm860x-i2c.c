/*
 * I2C driver for Marvell 88PM860x
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mfd/88pm860x.h>
#include <linux/slab.h>

#define CACHE_88PM860X_I2C
#if defined(CACHE_88PM860X_I2C)
#include "88pm860x-cache.h"
#else
#define pmic_cache_stat_print(RST)
#define pmic_cache_init(ID)
#define pmic_cache_hit_before_read(ID, REG, COUNT, PDATA)		(-1)
#define pmic_cache_hit_before_write(ID, REG, COUNT, PDATA)		(-1)
#define pmic_cache_save_after_readwrite(ID, I2C_RET, REG, COUNT, PDATA)
#define pmic_cache_invalidate(ID, REG)
#endif/*CACHE_88PM860X_I2C*/


static struct i2c_client *pm8607_i2c_client;

static inline int pm860x_read_device(struct i2c_client *i2c,
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

static inline int pm860x_write_device(struct i2c_client *i2c,
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

int pm860x_reg_read(struct i2c_client *i2c, int reg)
{
	unsigned char data;
	int ret;

	ret = pm860x_bulk_read(i2c, reg, 1, &data);
	if (ret < 0)
		return ret;
	else
		return (int)data;
}
EXPORT_SYMBOL(pm860x_reg_read);

int pm860x_reg_write(struct i2c_client *i2c, int reg,
		     unsigned char data)
{
	return pm860x_bulk_write(i2c, reg, 1, &data);
}
EXPORT_SYMBOL(pm860x_reg_write);

int pm860x_codec_reg_read(int reg)
{
	unsigned char data = 0;
	int ret;

	ret = pm860x_bulk_read(pm8607_i2c_client, reg, 1, &data);
	if (ret < 0)
		return ret;
	else
		return (int)data;
}
EXPORT_SYMBOL(pm860x_codec_reg_read);

int pm860x_codec_reg_write(int reg, unsigned char data)
{
	return pm860x_bulk_write(pm8607_i2c_client, reg, 1, &data);
}
EXPORT_SYMBOL(pm860x_codec_reg_write);

int pm860x_codec_reg_set_bits(int reg, unsigned char mask,
			unsigned char data)
{
	int ret;
	/*we have mutex protection in pm860x_set_bits()*/
	ret = pm860x_set_bits(pm8607_i2c_client, reg, mask, data);
	return ret;
}
EXPORT_SYMBOL(pm860x_codec_reg_set_bits);

int pm860x_bulk_read(struct i2c_client *i2c, int reg,
		     int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pmic_cache_hit_before_read(chip->id, reg, count, buf);
	if (ret < 0) {
		ret = pm860x_read_device(i2c, reg, count, buf);
		pmic_cache_save_after_readwrite(chip->id, ret, reg, count, buf);
	}
	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(pm860x_bulk_read);

int pm860x_bulk_write(struct i2c_client *i2c, int reg,
		      int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pmic_cache_hit_before_write(chip->id, reg, count, buf);
	if (ret < 0) {
		ret = pm860x_write_device(i2c, reg, count, buf);
		pmic_cache_save_after_readwrite(chip->id, ret, reg, count, buf);
	}
	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(pm860x_bulk_write);

int pm860x_set_bits(struct i2c_client *i2c, int reg,
		    unsigned char mask, unsigned char data)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char valget, valset;
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pmic_cache_hit_before_read(chip->id, reg, 1, &valget);
	if (ret < 0) {
		ret = pm860x_read_device(i2c, reg, 1, &valget);
		pmic_cache_save_after_readwrite(chip->id, ret, reg, 1, &valget);
	}
	if (ret >= 0) {
		valset = valget;
		valset &= ~mask;
		valset |= data;

		/* hw issue fix from pm860x C0
		 * unexpected BAT interrupt when chip-sleep
		 */
		if (1) { /*(valget != valset)*/
			ret = pm860x_write_device(i2c, reg, 1, &valset);
			pmic_cache_save_after_readwrite(chip->id, ret, reg, 1, &valset);
		}
	}
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_set_bits);

/* read_device() and write_device() utilities are cloned from
**  "i2-core" i2c_transfer()
** but they do NOT take adapt/bus MUTEX!
** Mutex is taken once for some read_device/write_device calls
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

/************   PAGE MODE  *********************************/
static void pm860x_page_in(struct i2c_client *i2c)
{
	unsigned char zero = 0;
	i2c_lock_adapter(i2c->adapter);
	if (i2c->adapter->hardware_lock)
		i2c->adapter->hardware_lock();
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	read_device(i2c, 0xFF, 0, &zero);
}

static void pm860x_page_out(struct i2c_client *i2c)
{
	unsigned char zero = 0;
	read_device(i2c, 0xFE, 0, &zero);
	read_device(i2c, 0xFC, 0, &zero);
	if (i2c->adapter->hardware_unlock)
		i2c->adapter->hardware_unlock();
	i2c_unlock_adapter(i2c->adapter);
}

int pm860x_page_reg_read(struct i2c_client *i2c, int reg)
{
	unsigned char data;
	int ret;
	ret = pm860x_page_bulk_read(i2c, reg, 1, &data);
	if (ret >= 0)
		ret = (int)data;
	return ret;
}
EXPORT_SYMBOL(pm860x_page_reg_read);

int pm860x_page_reg_write(struct i2c_client *i2c, int reg,
			  unsigned char data)
{
	return pm860x_page_bulk_write(i2c, reg, 1, &data);
}
EXPORT_SYMBOL(pm860x_page_reg_write);

int pm860x_page_bulk_read(struct i2c_client *i2c, int reg,
			  int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	pm860x_page_in(i2c);
	ret = pmic_cache_hit_before_read(PM8607_PAGE_ID, reg, count, buf);
	if (ret < 0) {
		ret = read_device(i2c, reg, count, buf);
		pmic_cache_save_after_readwrite(PM8607_PAGE_ID, ret, reg, count, buf);
	}
	pm860x_page_out(i2c);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_page_bulk_read);

int pm860x_page_bulk_write(struct i2c_client *i2c, int reg,
			   int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	pm860x_page_in(i2c);
	ret = pmic_cache_hit_before_write(PM8607_PAGE_ID, reg, count, buf);
	if (ret < 0) {
		ret = write_device(i2c, reg, count, buf);
		pmic_cache_save_after_readwrite(PM8607_PAGE_ID, ret, reg, count, buf);
	}
	pm860x_page_out(i2c);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_page_bulk_write);

/* Not used and to be deleted
int pm860x_page_set_bits(struct i2c_client *i2c, int reg,
			 unsigned char mask, unsigned char data)
EXPORT_SYMBOL(pm860x_page_set_bits);
*/

static const struct i2c_device_id pm860x_id_table[] = {
	{ "88PM860x", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, pm860x_id_table);

static int verify_addr(struct i2c_client *i2c)
{
	unsigned short addr_8607[] = {0x34, 0x30};
	unsigned short addr_8606[] = {0x10, 0x11};
	int size, i;

	if (i2c == NULL)
		return 0;
	size = ARRAY_SIZE(addr_8606);
	for (i = 0; i < size; i++) {
		if (i2c->addr == *(addr_8606 + i))
			return CHIP_PM8606;
	}
	size = ARRAY_SIZE(addr_8607);
	for (i = 0; i < size; i++) {
		if (i2c->addr == *(addr_8607 + i))
			return CHIP_PM8607;
	}
	return 0;
}

static int __devinit pm860x_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct pm860x_platform_data *pdata = client->dev.platform_data;
	struct pm860x_chip *chip;

	if (!pdata) {
		pr_info("No platform data in %s!\n", __func__);
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm860x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->id = verify_addr(client);
	chip->client = client;
	pm8607_i2c_client = client;
	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
	dev_set_drvdata(chip->dev, chip);

	/*
	 * Both client and companion client shares same platform driver.
	 * Driver distinguishes them by pdata->companion_addr.
	 * pdata->companion_addr is only assigned if companion chip exists.
	 * At the same time, the companion_addr shouldn't equal to client
	 * address.
	 */
	if (pdata->companion_addr && (pdata->companion_addr != client->addr)) {
		chip->companion_addr = pdata->companion_addr;
		chip->companion = i2c_new_dummy(chip->client->adapter,
						chip->companion_addr);
		i2c_set_clientdata(chip->companion, chip);
	}

	if (pdata->fixup)
		pdata->fixup(chip, pdata);

	pm860x_device_init(chip, pdata);

	pmic_cache_init(chip->id);
	return 0;
}

static int __devexit pm860x_remove(struct i2c_client *client)
{
	struct pm860x_chip *chip = i2c_get_clientdata(client);

	pm860x_device_exit(chip);
	i2c_unregister_device(chip->companion);
	kfree(chip);
	return 0;
}

static struct i2c_driver pm860x_driver = {
	.driver	= {
		.name	= "88PM860x",
		.owner	= THIS_MODULE,
	},
	.probe		= pm860x_probe,
	.remove		= __devexit_p(pm860x_remove),
	.id_table	= pm860x_id_table,
};

static int __init pm860x_i2c_init(void)
{
	int ret;
	ret = i2c_add_driver(&pm860x_driver);
	if (ret != 0)
		pr_err("Failed to register 88PM860x I2C driver: %d\n", ret);
	return ret;
}
subsys_initcall(pm860x_i2c_init);

static void __exit pm860x_i2c_exit(void)
{
	i2c_del_driver(&pm860x_driver);
}
module_exit(pm860x_i2c_exit);

MODULE_DESCRIPTION("I2C Driver for Marvell 88PM860x");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
