/*
 * LED/flash driver for Analog Devices ADP1650 chip
 *
 * Copyright (C) 2012 Marvell Internation Ltd.
 *
 * Bin Zhou <zhoub@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>

#include <linux/i2c/adp1650.h>

#define DESIGN_INFO		0x00
#define VREF_TIME		0x02
#define CURRENT_SET		0x03
#define OUTPUT_MODE		0x04
#define FAULT_INFO		0x05
#define INPUT_CONTROL		0x06
#define ADD_MODE1		0x07
#define ADD_MODE2		0x08
#define BATTERY_LOW_MODE	0x09

#define OUTPUT_EN		(1<<3)
#define LED_FLASH_MODE		(3<<0)

struct i2c_client *g_adp1650_i2c_client;

static int adp1650_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		dev_err(&client->dev, "adp1650: failed to write reg-0x%02x\n", reg);

	return ret;
}

static int adp1650_read(struct i2c_client *client, int reg, uint8_t * val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "adp1650: failed to read reg-0x%02x\n", reg);
		return ret;
	}
	*val = (uint8_t)ret;
	return 0;
}

static int adp1650_set_torch_mode(struct i2c_client *client, bool flag)
{
	int ret;
	uint8_t value = 0;
	struct adp1650 *adp1650 = i2c_get_clientdata(client);
	struct adp1650_platform_data *pdata = adp1650->pdata;

	/* 1 for torch on, 0 for torch off */
	if (flag) {
		if (pdata->strobe_enable) {
			printk(KERN_INFO "adp1650: not support torch if strobe enabled\n");
			return 0;
		}

		/* read to clear fault register */
		adp1650_read(client, FAULT_INFO, &value);

		adp1650_read(client, VREF_TIME, &value);
		value &= 0xCF;
		value |= 0x10;
		ret = adp1650_write(client, VREF_TIME, value);

		adp1650_read(client, OUTPUT_MODE, &value);
		value |= OUTPUT_EN;
		value &= 0xFC;
		ret = adp1650_write(client, OUTPUT_MODE, value);
	} else {
		adp1650_read(client, OUTPUT_MODE, &value);
		value &= ~OUTPUT_EN;
		ret = adp1650_write(client, OUTPUT_MODE, value);
	}

	if (pdata->torch_enable)
		pdata->torch_enable(flag);

	pdata->torch_is_on = flag;

	return ret;
}

static int adp1650_set_flash_mode(struct i2c_client *client)
{
	int ret;
	uint8_t value = 0;

	/* read to clear fault register */
	adp1650_read(client, FAULT_INFO, &value);

	adp1650_read(client, OUTPUT_MODE, &value);
	value |= OUTPUT_EN;
	value |= LED_FLASH_MODE;
	ret = adp1650_write(client, OUTPUT_MODE, value);

	return ret;
}

int adp1650_set_strobe_mode(bool flag)
{
	struct adp1650 *adp1650 = i2c_get_clientdata(g_adp1650_i2c_client);
	struct adp1650_platform_data *pdata = adp1650->pdata;

	/* strobe mode has high priority. If torch on, turn off it. */
	if (pdata->torch_is_on) {
		adp1650_set_torch_mode(adp1650->client, 0);
	}

	if (flag)
		adp1650_set_flash_mode(adp1650->client);

	pdata->strobe_enable = flag;

	printk(KERN_INFO "adp1650: set strobe mode, flag=%d\n", flag);

	return 0;
}
EXPORT_SYMBOL(adp1650_set_strobe_mode);

#ifdef	CONFIG_PROC_FS
#define	ADP1650_PROC_FILE	"driver/adp1650"
static struct proc_dir_entry *adp1650_proc_file;

static ssize_t adp1650_proc_read(struct file *filp,
				 char *buffer, size_t count, loff_t *offset)
{
	return 0;
}

static ssize_t adp1650_proc_write(struct file *filp,
				  const char *buff, size_t len, loff_t *off)
{
	struct adp1650 *adp1650 = i2c_get_clientdata(g_adp1650_i2c_client);
	char messages[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	switch(messages[0]) {
	case MODE_TORCH_OFF:
		printk(KERN_INFO "adp1650: set torch off\n");
		adp1650_set_torch_mode(adp1650->client, 0);
		break;
	case MODE_TORCH_ON:
		printk(KERN_INFO "adp1650: set torch on\n");
		adp1650_set_torch_mode(adp1650->client, 1);
		break;
	case MODE_SET_FLASH_MODE:
		printk(KERN_INFO "adp1650: set flash mode\n");
		adp1650_set_flash_mode(adp1650->client);
		break;
	default:
		printk(KERN_ERR "adp1650: unknown request\n");
		break;
	}

	return len;
}

static struct file_operations adp1650_proc_ops = {
	.read = adp1650_proc_read,
	.write = adp1650_proc_write,
};

static int adp1650_create_proc_file(void)
{
	if (adp1650_proc_file != NULL)
		return 0;

	adp1650_proc_file = create_proc_entry(ADP1650_PROC_FILE, 0644, NULL);
	if (adp1650_proc_file) {
		adp1650_proc_file->proc_fops = &adp1650_proc_ops;
	} else {
		printk(KERN_ERR "adp1650: failed to create proc file\n");
		return -EFAULT;
	}

	return 0;
}

static void adp1650_remove_proc_file(void)
{
	if (adp1650_proc_file == NULL)
		return;

	remove_proc_entry(ADP1650_PROC_FILE, NULL);
	adp1650_proc_file = NULL;
}
#endif /*CONFIG_PROC_FS */

static int adp1650_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;

	ret = adp1650_read(client, DESIGN_INFO, &v);

	if (ret < 0)
		return ret;
	if (v != 0x22)
		return -ENODEV;

	printk(KERN_INFO "adp1650: chip detected\n");
	return 0;
}

static int __devinit adp1650_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int ret;
	struct adp1650_platform_data *pdata = client->dev.platform_data;
	struct adp1650 *adp1650;

	g_adp1650_i2c_client = client;

	if (!pdata) {
		dev_err(&client->dev, "adp1650: missing platform data\n");
		return -EINVAL;
	}

	adp1650 = kzalloc(sizeof(struct adp1650), GFP_KERNEL);
	if (!adp1650) {
		dev_err(&client->dev, "adp1650: failed to allocate adp1650 struct\n");
		return -ENOMEM;
	}

	adp1650->client = client;
	adp1650->pdata = pdata;

	i2c_set_clientdata(client, adp1650);

	ret = adp1650_detect(client);
	if (ret < 0)
		goto out;

#ifdef	CONFIG_PROC_FS
	ret = adp1650_create_proc_file();
	if (ret < 0)
		goto out;
#endif

	return 0;

out:
	kfree(adp1650);
	return ret;
}

static int adp1650_remove(struct i2c_client *client)
{
	struct adp1650 *adp1650 = i2c_get_clientdata(client);

	g_adp1650_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);

#ifdef	CONFIG_PROC_FS
	adp1650_remove_proc_file();
#endif

	kfree(adp1650);
	return 0;
}

static const struct i2c_device_id adp1650_id[] = {
	{"adp1650", 0},
};

MODULE_DEVICE_TABLE(i2c, adp1650_id);

static struct i2c_driver adp1650_driver = {
	.driver = {
		   .name = "adp1650",
		   },
	.probe = adp1650_probe,
	.remove = adp1650_remove,
	.id_table = adp1650_id,
};

static int __init adp1650_mod_init(void)
{
	return i2c_add_driver(&adp1650_driver);
}

module_init(adp1650_mod_init);

static void __exit adp1650_mode_exit(void)
{
	i2c_del_driver(&adp1650_driver);
}

module_exit(adp1650_mode_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bin Zhou <zhoub@marvell.com>");
MODULE_DESCRIPTION("adp1650 LED/flash driver");

