/*
 * IDT1338 rtc class driver
 *
 * Copyright 2011 Saadia Husain Baloch <saadia at laptop.org>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/slab.h>

#define RTC_SECONDS	0
#define RTC_MINUTES	1
#define RTC_HOURS	2
#define RTC_DAY_OF_WEEK	3
#define RTC_DATE	4
#define RTC_MONTH	5
#define RTC_YEAR	6
#define BYTE_NUM	7

static const struct i2c_device_id idt1338_id[] = {
	{ "rtc_idt1338", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, idt1338_id);

struct idt1338 {
	struct rtc_device *rtc;
};

static int idt1338_get_time(struct i2c_client *client, struct rtc_time *tm)
{
	unsigned int year, month, date, hour, minute, second, week;
	u8 buf[BYTE_NUM] = {0};
	int err = 0;

	err = i2c_master_send(client, buf, 1); /* Send 0 */
	if (err < 0)
		return (err);
	err = i2c_master_recv(client, buf, BYTE_NUM);	  
	
	second 	= buf[RTC_SECONDS] & 0x7f;
	minute 	= buf[RTC_MINUTES];
	hour 	= buf[RTC_HOURS] & 0xbf;
	week 	= buf[RTC_DAY_OF_WEEK] & 0x07;
	date 	= buf[RTC_DATE] & 0x3f;
	month 	= buf[RTC_MONTH];
	year 	= buf[RTC_YEAR];

	tm->tm_sec 	= bcd2bin(second);
	tm->tm_min 	= bcd2bin(minute);
	tm->tm_hour 	= bcd2bin(hour);
	tm->tm_wday 	= bcd2bin(week);
	tm->tm_mday 	= bcd2bin(date);
	tm->tm_mon 	= bcd2bin(month) - 1;  /* read in 1-12 range */
	tm->tm_year 	= 100 + bcd2bin(year); /* read delta from 2000 */

	err = rtc_valid_tm(tm);
	if (err < 0) 
		rtc_time_to_tm(0, tm);

	return err;
}

static int idt1338_read_time(struct device *dev, struct rtc_time *tm)
{
	return idt1338_get_time(to_i2c_client(dev), tm);
}

static int idt1338_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 buf[BYTE_NUM] = {0};
	int err = 0;

	err = i2c_master_send(client, buf, 1); /* Send 0 */
	if (err < 0)
		return (err);

	buf[RTC_SECONDS] 	= bin2bcd(tm->tm_sec) & 0x7f;
	buf[RTC_MINUTES]	= bin2bcd(tm->tm_min) & 0x7f;
	/* Retain 24 hour mode by keeping bit 6 of HOURS register low */
	buf[RTC_HOURS] 		= bin2bcd(tm->tm_hour) & 0x3f;
	buf[RTC_DAY_OF_WEEK] 	= bin2bcd(tm->tm_wday) & 0x07;
	buf[RTC_DATE] 		= bin2bcd(tm->tm_mday) & 0x3f;
	buf[RTC_MONTH] 		= bin2bcd(tm->tm_mon + 1) & 0x1f;
	buf[RTC_YEAR] 		= bin2bcd((tm->tm_year > 100)?
					  tm->tm_year-100:tm->tm_year);

	err = i2c_master_send(client, (char *)buf, BYTE_NUM);
	return err;
}

static const struct rtc_class_ops idt1338_ops = {
	.read_time	= idt1338_read_time,
	.set_time	= idt1338_set_time,
};

static struct i2c_driver idt1338_driver;

static int __devinit idt1338_probe(struct i2c_client *client,
				   const struct i2c_device_id *idp)
{
	struct idt1338 *idt1338;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	idt1338 = kzalloc(sizeof(struct idt1338), GFP_KERNEL);
	if (!idt1338) 
		return -ENOMEM;

	i2c_set_clientdata(client, idt1338);

	idt1338->rtc = rtc_device_register(idt1338_driver.driver.name,
					   &client->dev, &idt1338_ops, 
					   THIS_MODULE);
	if (IS_ERR(idt1338->rtc)) {
		err = PTR_ERR(idt1338->rtc);
		kfree (idt1338);
		return err;
	}
	return 0;
}

static int idt1338_remove(struct i2c_client *client)
{
	struct idt1338 *idt1338 = i2c_get_clientdata(client);
	if (idt1338->rtc)
		rtc_device_unregister(idt1338->rtc);

	kfree(idt1338);
	return 0;
}

static struct i2c_driver idt1338_driver = {
	.driver	= {
		.name	= "rtc_idt1338",
	},
	.probe = idt1338_probe,
	.remove	= __devexit_p(idt1338_remove),
	.id_table = idt1338_id,
};

static int __init idt1338_init(void)
{
	int err;

	err = i2c_add_driver(&idt1338_driver);
	if (err)
		printk(KERN_ERR "IDT1338 RTC init err=%d\n", err);
	return err;
}

static void __exit idt1338_exit(void)
{
	i2c_del_driver(&idt1338_driver);
}

MODULE_AUTHOR("Saadia Baloch <saadia at laptop.org");
MODULE_DESCRIPTION("IDT 1338 RTC driver");
MODULE_LICENSE("GPL");

module_init(idt1338_init);
module_exit(idt1338_exit);
