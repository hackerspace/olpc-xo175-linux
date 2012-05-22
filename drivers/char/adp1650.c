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

static int adp1650_output_off(struct i2c_client *client)
{
	int ret;
	uint8_t value = 0;
	struct adp1650 *adp1650 = i2c_get_clientdata(client);
	struct adp1650_platform_data *pdata = adp1650->pdata;

	adp1650_read(client, REG_OUTPUT_MODE, &value);
	/* disable output and enable standby mode */
	value &= ~OUTPUT_MODE_MASK;
	ret = adp1650_write(client, REG_OUTPUT_MODE, value);

	if (pdata->torch_enable)
		pdata->torch_enable(0);

	pdata->torch_is_on = 0;
	pdata->strobe_enable = 0;

	return ret;
}

static int adp1650_enable_torch_mode(struct i2c_client *client)
{
	int ret;
	uint8_t value = 0;
	struct adp1650 *adp1650 = i2c_get_clientdata(client);
	struct adp1650_platform_data *pdata = adp1650->pdata;

	if(pdata->strobe_enable) {
		printk(KERN_ERR "adp1650: flash mode on, not support torch\n");
		return -1;
	}

	/* read to clear fault register */
	adp1650_read(client, REG_FAULT_INFO, &value);

	adp1650_read(client, REG_VERF_TIMER, &value);
	value &= ~GPIO1_CONFIG_MASK;
	value |= GPIO1_CONFIG_TORCH;
	ret = adp1650_write(client, REG_VERF_TIMER, value);

	adp1650_read(client, REG_OUTPUT_MODE, &value);
	value &= ~OUTPUT_MODE_MASK;
	value |= OUTPUT_ENABLE;
	ret = adp1650_write(client, REG_OUTPUT_MODE, value);

	if (pdata->torch_enable)
		pdata->torch_enable(1);

	pdata->torch_is_on = 1;

	return ret;
}

static int adp1650_enable_flash_mode(struct i2c_client *client)
{
	int ret;
	uint8_t value = 0;
	struct adp1650 *adp1650 = i2c_get_clientdata(client);
	struct adp1650_platform_data *pdata = adp1650->pdata;

	if(pdata->torch_is_on) {
		if (pdata->torch_enable)
			pdata->torch_enable(0);
		pdata->torch_is_on = 0;
	}

	/* read to clear fault register */
	adp1650_read(client, REG_FAULT_INFO, &value);

	adp1650_read(client, REG_AD_MOD, &value);
	value |= DYNAMIC_OVP_ON;
	ret = adp1650_write(client, REG_AD_MOD, value);

	adp1650_read(client, REG_OUTPUT_MODE, &value);
	value &= ~OUTPUT_MODE_MASK;
	value |= OUTPUT_MODE_FLASH;
	value |= OUTPUT_ENABLE;

	ret = adp1650_write(client, REG_OUTPUT_MODE, value);

	pdata->strobe_enable = 1;

	return ret;
}

static int adp1650_set_output_mode(struct i2c_client *client, char mode)
{
	int ret;

	switch(mode) {
	case MODE_STANDBY:
		printk(KERN_INFO "adp1650: output off\n");
		ret = adp1650_output_off(client);
		break;
	case MODE_TORCH:
		printk(KERN_INFO "adp1650: enable torch mode\n");
		ret = adp1650_enable_torch_mode(client);
		break;
	case MODE_FLASH:
		printk(KERN_INFO "adp1650: enable flash mode\n");
		ret = adp1650_enable_flash_mode(client);
		break;
	default:
		printk(KERN_ERR "adp1650: unknown request\n");
		break;
	}

	return ret;
}

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
	char messages[2];

	if (len > 2)
		len = 2;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	adp1650_set_output_mode(adp1650->client, messages[0]);

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

	ret = adp1650_read(client, REG_DESIGN_INFO, &v);

	if (ret < 0)
		return ret;
	if (v != DI_DEFAULT_VAL)
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

