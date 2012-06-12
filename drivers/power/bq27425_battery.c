/*
 * BQ27425 battery driver
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mfd/88pm80x.h>
#include <asm/unaligned.h>
#include <plat/pm.h>

#define BQ27425_REG_CTRL		0x00
/* CTRL set bit */
#define DEVICE_TYPE            (0x0001)
#define FW_VERSION             (0x0002)
#define HW_VERSION             (0x0003)
#define PREV_MACWRIT           (0x0007)
#define BAT_INSERT             (0x000c)
#define BAT_REMOVE             (0x000d)
#define SET_HIBERNATE          (0x0011)
#define CLEAR_HIBERNATE        (0x0012)
#define SET_CFGUPDATE          (0x0013)
#define FACTORY_RESTORE        (0x0015)
#define SEALED                 (0x0020)
#define RESET                  (0x0041)
#define SOFT_RESET             (0x0042)

#define BQ27425_REG_TEMP		0x02
#define BQ27425_REG_VOLT		0x04
#define BQ27425_REG_FLAGS		0x06
/*flags set bit*/
#define FLAGS_DSG               (1<<0)
#define FLAGS_BAT_DET           (1<<3)
#define FLAGS_CHG               (1<<8)
#define FLAGS_FC                (1<<9)
#define FLAGS_UT                (1<<14)
#define FLAGS_OT                (1<<15)

#define BQ27425_REG_NAC			0x08
#define BQ27425_REG_FAC			0x0a
#define BQ27425_REG_RM			0x0c
#define BQ27425_REG_FCC			0x0e
#define BQ27425_REG_AI			0x10
#define BQ27425_REG_SI			0x12 /* Nominal available capaciy */
#define BQ27425_REG_MLI			0x14 /* Last measured discharge */
#define BQ27425_REG_AP			0x18 /* Cycle count total */
#define BQ27425_REG_SOC			0x1c /* Available enery */
#define BQ27425_REG_ITEMP		0x1e /* RemainingCapacity */
#define BQ27425_REG_SOH			0x20

#define KELVIN_VAL              (2732)

#define NORMAL_POLL_TIME		(60)
#define POWER_CHANGE_POLL_TIME	(2)
#define DETECT_CHARGING_TIME	(10)

struct bq27425_device_info;
struct bq27425_access_methods {
	int (*read)(struct bq27425_device_info *di, u8 reg, bool single);
};

struct bq27425_reg_cache {
	short           cntl;
	short           temp;   /*temperature;*/
	short           volt;   /*voltage;*/
	short           nac;    /*NominalAvailableCapacity */
	short           fac;    /*FullAvailableCapacit*/
	short           rm;     /*RemainingCapacity*/
	short           fcc;    /*FullChargeCapacity*/
	short           si;     /*StandbyCurrent*/
	short           ai;     /*averagecurrent*/
	short           mli;    /*MaxLoadCurrent*/
	short           ap;     /*AveragePower*/
	short           soc;    /*StateOfCharge*/
	short           itemp;  /*IntTemperature*/
	short           soh;    /*StateofHealth*/
	unsigned short  flags;
	short           health;
	short           status;
	short			present;
};

struct bq27425_device_info {
	struct device *dev;
	struct i2c_client *client;

	int id;
	int irq;

	struct bq27425_reg_cache cache;
	int charge_design_full;

	struct delayed_work work;
	struct delayed_work test_work;
	struct timer_list timer;

	struct power_supply bat;

	struct bq27425_access_methods bus;

	struct mutex lock;
};

static enum power_supply_property bq27425_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static struct bq27425_device_info *g_di;
static char *usb_supplied[] = {
	"usb",
};

static char *charger_supplied[] = {
	"ac",
};

static unsigned int poll_interval = NORMAL_POLL_TIME;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
		"0 disables polling");

static int bq27425_battery_status(struct bq27425_device_info *di);


static int bq27425_read_i2c(struct bq27425_device_info *di, u8 reg, u16 *val)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret = 0;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;


	*val = get_unaligned_le16(data);

	return ret;
}

static int bq27425_write_i2c(struct bq27425_device_info *di, u8 reg, void *src)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	unsigned char data[3];
	int ret;

	data[0] = (unsigned char)reg;
	memcpy(&data[1], src, 2);


	ret = i2c_master_send(client, data, 3);
	if (ret < 0)
		return ret;

	return ret;
}

static int bq27425_get_cntl(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_CTRL, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

#if 0
static int bq27425_set_cntl(struct bq27425_device_info *di, u16 data)
{
	int ret = 0;

	ret = bq27425_write_i2c(di, BQ27425_REG_CTRL, &data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}
#endif

static int bq27425_get_temp(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;
	u16 val;

	ret = bq27425_read_i2c(di, BQ27425_REG_TEMP, &val);
	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");
	*data = val - KELVIN_VAL;

	return ret;
}

#if 0
static int bq27425_set_temp(struct bq27425_device_info *di, u16 data)
{
	int ret = 0;

	ret = bq27425_write_i2c(di, BQ27425_REG_TEMP, &data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}
#endif

static int bq27425_get_volt(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_VOLT, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_flags(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_FLAGS, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_nac(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_NAC, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_fac(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_FAC, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_rm(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_RM, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_fcc(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_FCC, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_ai(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_AI, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_si(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_SI, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_mli(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_MLI, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_ap(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_AP, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_soc(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_SOC, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_itemp(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_ITEMP, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static int bq27425_get_soh(struct bq27425_device_info *di, u16 *data)
{
	int ret = 0;

	ret = bq27425_read_i2c(di, BQ27425_REG_SOH, data);

	if (ret < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return ret;
}

static void bq27425_get_status(struct bq27425_device_info *di)
{
	int ret = 0;
	u16 data;

	ret = bq27425_get_cntl(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get cntl failed!\n");
	di->cache.cntl = data;

	ret = bq27425_get_temp(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get temp failed!\n");
	di->cache.temp = data;	/*temperature;*/

	ret = bq27425_get_volt(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get volt failed!\n");
	di->cache.volt = data;	/*voltage;*/

	ret = bq27425_get_nac(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get nac failed!\n");
	di->cache.nac = data;

	ret = bq27425_get_fac(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get fac failed!\n");
	di->cache.fac = data;

	ret = bq27425_get_rm(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get rm failed!\n");
	di->cache.rm = data;		/*RemainingCapacity*/

	ret = bq27425_get_fcc(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get fcc failed!\n");
	di->cache.fcc = data;

	ret = bq27425_get_si(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get si failed!\n");
	di->cache.si = data;

	ret = bq27425_get_ai(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get ai failed!\n");
	di->cache.ai = data;

	ret = bq27425_get_mli(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get mli failed!\n");
	di->cache.mli = data;

	ret = bq27425_get_ap(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get ap failed!\n");
	di->cache.ap = data;

	ret = bq27425_get_soc(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get soc failed!\n");
	di->cache.soc = data;

	ret = bq27425_get_itemp(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get di failed!\n");
	di->cache.itemp = data;

	ret = bq27425_get_soh(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get soh failed!\n");
	di->cache.soh = data;

	ret = bq27425_get_flags(di, &data);
	if (ret < 0)
		printk(KERN_ERR "get flags failed!\n");
	di->cache.flags = data;
	if ((FLAGS_UT & di->cache.flags) ||
			(FLAGS_OT & di->cache.flags))
		di->cache.health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		di->cache.health = POWER_SUPPLY_HEALTH_GOOD;
	di->cache.status = bq27425_battery_status(di);
	di->cache.present = (di->cache.flags & FLAGS_BAT_DET) ? 1 : 0;
}

static void bq27425_update(struct bq27425_device_info *di)
{
	static struct bq27425_reg_cache cache = {0, };

	bq27425_get_status(di);

	if (cache.soc != di->cache.soc ||
			cache.status != di->cache.status) {
		cache = di->cache;
		power_supply_changed(&di->bat);
	}
}

int is_charger_usb_online(void)
{
	struct power_supply *psy;
	union power_supply_propval data;
	int online = 0;
	int ret = 0;

	psy = power_supply_get_by_name(usb_supplied[0]);
	BUG_ON(!psy);

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &data);
	BUG_ON(ret);
	online = data.intval;

	if (!online) {
		psy = power_supply_get_by_name(charger_supplied[0]);
		BUG_ON(!psy);

		ret = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &data);
		BUG_ON(ret);
		online = data.intval;
	}

	return online;
}

static void bq27425_battery_poll(struct work_struct *work)
{
	int online;
	struct bq27425_device_info *di =
		container_of(work, struct bq27425_device_info, work.work);

	bq27425_update(di);

	printk(KERN_INFO "%s capacity: %d, voltag: %d, current: %d,"
			 "status: %d, present: %d\n", __func__,
			 di->cache.soc, di->cache.volt,
			 di->cache.ai, di->cache.status, di->cache.present);

	online = is_charger_usb_online();

	if ((di->cache.status == POWER_SUPPLY_STATUS_DISCHARGING) &&
			online)
		poll_interval = DETECT_CHARGING_TIME;
	else
		poll_interval = NORMAL_POLL_TIME;

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}


static int bq27425_battery_status(struct bq27425_device_info *di)
{
	int status;

	if (di->cache.flags & FLAGS_DSG)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (di->cache.flags & FLAGS_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (di->cache.flags & FLAGS_CHG)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return status;
}

#define to_bq27425_device_info(x) container_of((x), \
		struct bq27425_device_info, bat);

static int bq27425_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct bq27425_device_info *di = to_bq27425_device_info(psy);

	mutex_lock(&di->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->cache.status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = di->cache.health;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->cache.volt * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->cache.ai;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->cache.present)
			val->intval = di->cache.soc;
		else {
			/*report fake capacity*/
			val->intval = 80;
			printk(KERN_WARNING "No battery fake capacity!!!\n");
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->cache.temp;
		break;
	default:
		ret = -ENODEV;
		break;
	}
	mutex_unlock(&di->lock);

	return ret;
}

static void bq27425_external_power_changed(struct power_supply *psy)
{
	struct bq27425_device_info *di = to_bq27425_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	set_timer_slack(&di->work.timer, POWER_CHANGE_POLL_TIME * HZ / 4);
	schedule_delayed_work(&di->work, POWER_CHANGE_POLL_TIME * HZ);
}

static int bq27425_powersupply_init(struct bq27425_device_info *di)
{
	int ret;
	unsigned short ctrl_cmd, ver_id;
	int gpadc1_meas1;

	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27425_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27425_battery_props);
	di->bat.get_property = bq27425_battery_get_property;
	di->bat.external_power_changed = bq27425_external_power_changed;

	ret = pm80x_codec_reg_read(
			(PM80X_BASE_PAGE << 8) | PM800_POWER_DOWN_LOG1);
	printk(KERN_INFO "PM800_POWER_DOWN_LOG1: 0x%x\n", ret);
	ret = pm80x_codec_reg_read(
			(PM80X_BASE_PAGE << 8) | PM800_POWER_DOWN_LOG2);
	printk(KERN_INFO "PM800_POWER_DOWN_LOG2: 0x%x\n", ret);

	/*clean power-down log register */
	ret = pm80x_codec_reg_write(
			(PM80X_BASE_PAGE << 8) | PM800_POWER_DOWN_LOG1, 0xff);
	ret = pm80x_codec_reg_write(
			(PM80X_BASE_PAGE << 8) | PM800_POWER_DOWN_LOG2, 0xff);

	ret = pm80x_codec_reg_read((PM80X_GPADC_PAGE << 8) | PM800_INT_ENA_3);
	ret |= PM800_GPADC_ODD_6UA;
	ret = pm80x_codec_reg_write(
			((PM80X_GPADC_PAGE << 8) | PM800_INT_ENA_3), ret);

	ret = pm80x_codec_reg_read((PM80X_GPADC_PAGE << 8) | PM800_STATUS_2);
	ret |= PM800_GPADC1_MEAS_EN;
	ret = pm80x_codec_reg_write(
			((PM80X_GPADC_PAGE << 8) | PM800_STATUS_2), ret);

	ret = pm80x_codec_reg_read(
			(PM80X_GPADC_PAGE << 8) | PM800_INT_STATUS4);
	ret |= PM800_GPADC_BD_GP1_EN;
	ret = pm80x_codec_reg_write(
			((PM80X_GPADC_PAGE << 8) | PM800_INT_STATUS4), ret);

	msleep(20);

	gpadc1_meas1 = pm80x_codec_reg_read((PM80X_GPADC_PAGE << 8) |
			PM800_GPADC1_MEAS1);
	printk(KERN_INFO "%s PM800_GPADC1_MEAS1: 0x%x\n", __func__,
			gpadc1_meas1);

	ret = pm80x_codec_reg_read((PM80X_BASE_PAGE << 8) | PM800_STATUS_1);
	printk(KERN_INFO "PM800_STATUS_1: 0x%x\n", ret);

	if (!(gpadc1_meas1 & 0xf)) {
		printk(KERN_INFO "Battery is detected!\n");
		ctrl_cmd = BAT_INSERT;
		bq27425_write_i2c(di, BQ27425_REG_CTRL, &ctrl_cmd);
		di->cache.present = 1;
	} else {
		ctrl_cmd = BAT_REMOVE;
		bq27425_write_i2c(di, BQ27425_REG_CTRL, &ctrl_cmd);
		di->cache.present = 0;
	}

	ctrl_cmd = FW_VERSION;
	bq27425_write_i2c(di, BQ27425_REG_CTRL, &ctrl_cmd);
	ret = bq27425_read_i2c(di, BQ27425_REG_CTRL, &ver_id);
	printk(KERN_NOTICE "Firmware version: 0x%x\n", ver_id);


	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	bq27425_update(di);

	INIT_DELAYED_WORK(&di->work, bq27425_battery_poll);
	return 0;
}

static void bq27425_powersupply_unregister(struct bq27425_device_info *di)
{
	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
}

static DEFINE_MUTEX(battery_mutex);



static void bq27425_test_work(struct work_struct *work)
{
	struct bq27425_device_info *di =
		container_of(work, struct bq27425_device_info, test_work.work);

	printk(KERN_INFO "%s soc: %d\n", __func__, di->cache.soc);
	if (100 == di->cache.soc)
		di->cache.soc = 10;
	else
		di->cache.soc += 5;
	if (100 < di->cache.soc)
		di->cache.soc = 100;
	power_supply_changed(&di->bat);

	set_timer_slack(&di->test_work.timer, HZ*1);
	schedule_delayed_work(&di->test_work, HZ*4);
}

static ssize_t capacity_show_attrs(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{

	cancel_delayed_work_sync(&g_di->work);

	INIT_DELAYED_WORK(&g_di->test_work, bq27425_test_work);
	set_timer_slack(&g_di->test_work.timer, HZ*1);
	schedule_delayed_work(&g_di->test_work, HZ*4);

	return 0;
}


static ssize_t capacity_change_attrs(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	cancel_delayed_work_sync(&g_di->test_work);

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&g_di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&g_di->work, poll_interval * HZ);
	}
	return count;
}


static DEVICE_ATTR(test_capacity, S_IRUSR|S_IWUSR, \
		capacity_show_attrs, capacity_change_attrs);

static struct attribute *battery_attributes[] = {
	&dev_attr_test_capacity.attr,
	NULL,
};


static struct attribute_group battery_attr_group = {
	.attrs = battery_attributes,
};

static irqreturn_t bq27425_irq_handler(int irq, void *data)
{
	schedule_delayed_work(&g_di->work, 0);
	return IRQ_HANDLED;
}

static int bq27425_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct bq27425_device_info *di;
	int retval = 0;
	int ret;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di->client = client;
	di->dev = &client->dev;
	di->bat.name = "battery";
	di->irq = client->irq;

	mutex_init(&di->lock);

	if (bq27425_powersupply_init(di))
		goto batt_failed_1;

	i2c_set_clientdata(client, di);

	ret = request_irq(di->irq, bq27425_irq_handler,
			IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			"bq27425", di);
	if (ret < 0)
		goto batt_irq_faild;

	ret = sysfs_create_group(&di->dev->kobj, &battery_attr_group);
	g_di = di;

	return 0;
batt_irq_faild:
	free_irq(di->irq, di);
batt_failed_1:
	kfree(di);

	return retval;
}

static int bq27425_battery_remove(struct i2c_client *client)
{
	struct bq27425_device_info *di = i2c_get_clientdata(client);

	bq27425_powersupply_unregister(di);

	kfree(di);

	return 0;
}

static const struct i2c_device_id bq27425_id[] = {
	{ "bq27425", 0},	/* bq27200 is same as bq27000, but with i2c */
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27425_id);

static struct i2c_driver bq27425_battery_driver = {
	.driver = {
		.name = "bq27425",
	},
	.probe = bq27425_battery_probe,
	.remove = bq27425_battery_remove,
	.id_table = bq27425_id,
};

static int __init bq27425_battery_i2c_init(void)
{
	int ret;
	ret = i2c_add_driver(&bq27425_battery_driver);

	if (ret)
		printk(KERN_ERR "Unable to register BQ27425 i2c driver\n");

	return ret;
}

module_init(bq27425_battery_i2c_init);

static inline void bq27425_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27425_battery_driver);
}
module_exit(bq27425_battery_i2c_exit);

/*
 * Module stuff
 */

MODULE_AUTHOR("");
MODULE_DESCRIPTION("BQ27425 battery monitor driver");
MODULE_LICENSE("GPL");
