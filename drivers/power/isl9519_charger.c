/*
 * ISL9519 system voltage regulator and battery charger driver.
 *
 * Copyright (c) 2011 Marvell Technology Ltd.
 * Yunfan Zhang <yfzhang@marvell.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/mfd/max8925.h>
#include <linux/power/isl9519.h>

/* ISL9519 register address */
#define CHG_CUR_REG			0x14
#define MAX_SYS_VOL_REG		0x15
#define CONTROL_REG			0x3D
#define MIN_SYS_VOL_REG		0x3E
#define INPUT_CUR_REG		0x3F
#define MANUFACTURE_ID_REG	0xFE
#define DEVICE_ID_REG		0xFF

/* Control register bit map */
#define CONTROL_TRICKLE		BIT(7)
#define CONTROL_AC_OK		BIT(6)
#define CONTROL_SEL_VFBIT	BIT(5)
#define CONTROL_LOW_POWR	BIT(4)
#define CONTROL_VAR_FREQ	BIT(3)
#define CONTROL_ISO_ADAPTER	BIT(2)
#define CONTROL_80KHZ		BIT(1)
#define CONTROL_SGATE_ON	BIT(0)

/* Register mask */
#define MAX_SYS_VOL_MASK	0x3FF0
#define MIN_SYS_VOL_MASK	0x3F00
#define CHG_CUR_MASK		0x1F80
#define INPUT_CUR_MASK		0x1F80
/* Default charge setting */
#define DEFAULT_MAX_SYS_VOL	0x1070	/* 4208mV */
#define DEFAULT_MIN_SYS_VOL	0x0D00	/* 3328mV */
#define DEFAULT_INPUT_CUR	0x0800	/* 2048mA */
#define DEFAULT_CHG_CUR		0x0800	/* 2048mA */

/* Max update interval: second.
 * Max value 175s defined in Spec, set 150s here. */
#define DEFAULT_MAX_UPDATE_INTERVAL	150

struct isl9519_device_info {
	struct device *dev;
	struct i2c_client *client;
	struct power_supply ac;
	struct power_supply usb;
	struct notifier_block chg_notif;
	struct wake_lock chg_wake_lock;
	struct delayed_work chg_update_work;
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *chg_proc;
#endif
	int ac_chg_online;
	int usb_chg_online;
	int is_charging;
	/* Charger operation settings */
	u16 max_sys_vol;
	u16 min_sys_vol;
	u16 max_chg_cur;
	u16 max_input_cur;
	/* Charging current now */
	u16 chg_cur_now;
	/* Wake up settings */
	int stay_awake_en;
	int update_interval;
};

/* Return negative errno, else zero on success */
static int isl9519_read_reg(struct i2c_client *client, u8 reg, u16 *data)
{
	int ret = 0;

	if (!client || !data)
		return -EINVAL;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	}
	*data = ret;
	return 0;
}

/* Return negative errno, else zero on success */
static int isl9519_write_reg(struct i2c_client *client, u8 reg, u16 data)
{
	int ret = 0;

	if (!client)
		return -EINVAL;

	ret = i2c_smbus_write_word_data(client, reg, data);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			data, reg, ret);
		return ret;
	}
	return 0;
}

/* Get device infomation helper functions */
static int isl9519_get_max_sys_vol(struct isl9519_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = isl9519_read_reg(di->client, MAX_SYS_VOL_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int isl9519_get_min_sys_vol(struct isl9519_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = isl9519_read_reg(di->client, MIN_SYS_VOL_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int isl9519_get_input_cur(struct isl9519_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = isl9519_read_reg(di->client, INPUT_CUR_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int isl9519_get_chg_cur(struct isl9519_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = isl9519_read_reg(di->client, CHG_CUR_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int isl9519_get_control(struct isl9519_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = isl9519_read_reg(di->client, CONTROL_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int isl9519_get_device_id(struct isl9519_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = isl9519_read_reg(di->client, DEVICE_ID_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int isl9519_get_manuf_id(struct isl9519_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = isl9519_read_reg(di->client, MANUFACTURE_ID_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

/* Configure device helper functions */
static int isl9519_set_max_sys_vol(struct isl9519_device_info *di, u16 val)
{
	return isl9519_write_reg(di->client, MAX_SYS_VOL_REG, val);
}

static int isl9519_set_min_sys_vol(struct isl9519_device_info *di, u16 val)
{
	return isl9519_write_reg(di->client, MIN_SYS_VOL_REG, val);
}

static int isl9519_set_chg_cur(struct isl9519_device_info *di, u16 val)
{
	return isl9519_write_reg(di->client, CHG_CUR_REG, val);
}

static int isl9519_set_input_cur(struct isl9519_device_info *di, u16 val)
{
	return isl9519_write_reg(di->client, INPUT_CUR_REG, val);
}

#ifdef NOT_USED
static int isl9519_set_control(struct isl9519_device_info *di, u16 val)
{
	return isl9519_write_reg(di->client, CONTROL_REG, val);
}
#endif

/* NOTE: ISL9519 includes a timer to insure the SMBus master
 * is active and to prevent over charging the battery.
 * If the adapter is present and if ISL9519Q does not receive
 * a write to the MaxSystemVoltage or ChargeCurrent register
 * within 175s, ISL9519 will terminate charging. */
static void isl9519_enable_charge(struct isl9519_device_info *di)
{
	/* Write max system voltage or/and charging current
	 * to re-enable charging */
	isl9519_set_max_sys_vol(di, di->max_sys_vol);
	isl9519_set_chg_cur(di, di->chg_cur_now);
	/* Read back to verify */
	if (di->max_sys_vol != isl9519_get_max_sys_vol(di) ||
		di->chg_cur_now != isl9519_get_chg_cur(di))
		dev_err(di->dev, "Set Register Failed\n");
}

static void isl9519_disable_charge(struct isl9519_device_info *di)
{
	isl9519_set_chg_cur(di, 0);
}

static void chg_update_work_func(struct work_struct *work)
{
	struct isl9519_device_info *di =
		container_of(work, struct isl9519_device_info,
			chg_update_work.work);
	if (!di->is_charging)
		return;
	isl9519_enable_charge(di);
	schedule_delayed_work(&di->chg_update_work, di->update_interval);
}

static int isl9519_start_charging(struct isl9519_device_info *di)
{
	if (di->stay_awake_en)
		wake_lock(&di->chg_wake_lock);
	di->is_charging = 1;
	isl9519_enable_charge(di);
	schedule_delayed_work(&di->chg_update_work, di->update_interval);
	return 0;
}

static int isl9519_stop_charging(struct isl9519_device_info *di)
{
	di->is_charging = 0;
	isl9519_disable_charge(di);
	cancel_delayed_work_sync(&di->chg_update_work);
	if (di->stay_awake_en)
		wake_lock_timeout(&di->chg_wake_lock, 2 * HZ);
	return 0;
}

static int isl9519_chg_notifier_callback(struct notifier_block *nb,
				unsigned long event, void *chg_data)
{
	struct isl9519_device_info *di =
		container_of(nb, struct isl9519_device_info, chg_notif);
	struct chg_data *data = chg_data;
	if (!data)
		return 0;
	/* Get charger type */
	switch (data->charger_type) {
	case POWER_SUPPLY_TYPE_MAINS:
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_ACA:
		di->ac_chg_online = 1;
		di->usb_chg_online = 0;
		di->chg_cur_now = di->max_chg_cur;
		break;
	case POWER_SUPPLY_TYPE_USB:
		di->usb_chg_online = 1;
		di->ac_chg_online = 0;
		/* NOTE: hardcode for USB here */
		di->chg_cur_now = 512;
		break;
	case POWER_SUPPLY_TYPE_BATTERY:
	default:
		di->ac_chg_online = 0;
		di->usb_chg_online = 0;
		di->chg_cur_now = 0;
		break;
	}
	/* Handle start/stop charging event */
	switch (event) {
	case CHG_EVENT_START:
		isl9519_start_charging(di);
		if (di->ac_chg_online)
			power_supply_changed(&di->ac);
		else if (di->usb_chg_online)
			power_supply_changed(&di->usb);
		break;
	case CHG_EVENT_STOP:
		di->ac_chg_online = 0;
		di->usb_chg_online = 0;
		isl9519_stop_charging(di);
		power_supply_changed(&di->ac);
		power_supply_changed(&di->usb);
		break;
	default:
		return 0;
	}
	return 0;
}

static int isl9519_charger_setup(struct isl9519_device_info *di,
				struct isl9519_charger_pdata *pdata)
{
	int ret = 0;

	ret = isl9519_get_device_id(di);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "Device ID: 0x%04x\n", ret);
	ret = isl9519_get_manuf_id(di);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "Manufacturer ID: 0x%04x\n", ret);

	/* Max system voltage */
	if (pdata->max_sys_vol & MAX_SYS_VOL_MASK)
		di->max_sys_vol = pdata->max_sys_vol & MAX_SYS_VOL_MASK;
	else
		di->max_sys_vol = DEFAULT_MAX_SYS_VOL;
	/* Min system voltage */
	if (pdata->min_sys_vol & MIN_SYS_VOL_MASK)
		di->min_sys_vol = pdata->min_sys_vol & MIN_SYS_VOL_MASK;
	else
		di->min_sys_vol = DEFAULT_MIN_SYS_VOL;
	/* Charging current */
	if (pdata->chg_cur & CHG_CUR_MASK)
		di->max_chg_cur = pdata->chg_cur & CHG_CUR_MASK;
	else
		di->max_chg_cur = DEFAULT_CHG_CUR;
	/* Input current limitation */
	if (pdata->input_cur & INPUT_CUR_MASK)
		di->max_input_cur = pdata->input_cur & INPUT_CUR_MASK;
	else
		di->max_input_cur = DEFAULT_INPUT_CUR;
	/* Update interval, Unit:second */
	if (pdata->update_interval > 0 &&
	    pdata->update_interval <= DEFAULT_MAX_UPDATE_INTERVAL)
		di->update_interval = pdata->update_interval * HZ;
	else
		di->update_interval = DEFAULT_MAX_UPDATE_INTERVAL * HZ;
	/* Stay awake setting */
	di->stay_awake_en = pdata->stay_awake_en;

	/* Setup max system voltage */
	ret = isl9519_get_max_sys_vol(di);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "Max sys vol: %dmV\n", ret);
	if ((ret & MAX_SYS_VOL_MASK) != di->max_sys_vol) {
		ret = isl9519_set_max_sys_vol(di, di->max_sys_vol);
		if (ret < 0)
			return ret;
	}
	/* Setup min system voltage */
	ret = isl9519_get_min_sys_vol(di);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "Min sys vol: %dmV\n", ret);
	if ((ret & MIN_SYS_VOL_MASK) != di->min_sys_vol) {
		ret = isl9519_set_min_sys_vol(di, di->min_sys_vol);
		if (ret < 0)
			return ret;
	}
	/* Setup input current limit */
	ret = isl9519_get_input_cur(di);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "Input current limit: %dmA\n", ret);
	if ((ret & INPUT_CUR_MASK) != di->max_input_cur) {
		ret = isl9519_set_input_cur(di, di->max_input_cur);
		if (ret < 0)
			return ret;
	}

	di->usb_chg_online = 0;
	di->ac_chg_online = 0;

	return 0;
}

static enum power_supply_property isl9519_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int isl9519_ac_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct isl9519_device_info *di =
		container_of(psy, struct isl9519_device_info, ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->ac_chg_online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property isl9519_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int isl9519_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct isl9519_device_info *di =
		container_of(psy, struct isl9519_device_info, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->usb_chg_online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char *isl9519_supplied_to[] = {
	"max17042-battery",
	"bq27410-battery",
};

static int isl9519_powersupply_init(struct isl9519_device_info *di,
				struct isl9519_charger_pdata *pdata)
{
	int ret = 0;

	if (pdata->supplied_to) {
		di->ac.supplied_to = pdata->supplied_to;
		di->ac.num_supplicants = pdata->num_supplicants;
		di->usb.supplied_to = pdata->supplied_to;
		di->usb.num_supplicants = pdata->num_supplicants;
	} else {
		di->ac.supplied_to = isl9519_supplied_to;
		di->ac.num_supplicants = ARRAY_SIZE(isl9519_supplied_to);
		di->usb.supplied_to = isl9519_supplied_to;
		di->usb.num_supplicants = ARRAY_SIZE(isl9519_supplied_to);
	}
	/* register ac charger props */
	di->ac.name = "isl9519-chg-ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = isl9519_ac_props;
	di->ac.num_properties = ARRAY_SIZE(isl9519_ac_props);
	di->ac.get_property = isl9519_ac_get_property;

	ret = power_supply_register(di->dev, &di->ac);
	if (ret)
		goto err_reg_ac;
	/* register usb charger props */
	di->usb.name = "isl9519-chg-usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = isl9519_usb_props;
	di->usb.num_properties = ARRAY_SIZE(isl9519_usb_props);
	di->usb.get_property = isl9519_usb_get_property;

	ret = power_supply_register(di->dev, &di->usb);
	if (ret)
		goto err_reg_usb;

	return ret;

err_reg_usb:
	power_supply_unregister(&di->ac);
err_reg_ac:
	return ret;
}

#ifdef CONFIG_PROC_FS
static int isl9519_write_proc(struct file *file, const char *buffer,
				unsigned long count, void *data)
{
	struct isl9519_device_info *di = data;
	unsigned long index = 0;
	u8 kbuf[8];

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
	case 0:
		wake_unlock(&di->chg_wake_lock);
		di->stay_awake_en = 0;
		break;
	case 1:
		wake_lock(&di->chg_wake_lock);
		di->stay_awake_en = 1;
		break;
	default:
		break;
	}
	return count;
}
#endif

static int isl9519_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct isl9519_device_info *di;
	struct isl9519_charger_pdata *pdata;
	int ret = 0;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "missing platform data\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	di->client = client;
	di->dev = &client->dev;
	i2c_set_clientdata(client, di);

	ret = isl9519_charger_setup(di, pdata);
	if (ret) {
		dev_err(&client->dev, "failed to setup charger\n");
		goto err_chg_setup;
	}

	/* Register power supply device for ac and usb */
	ret = isl9519_powersupply_init(di, pdata);
	if (ret) {
		dev_err(&client->dev, "failed to register ac/usb\n");
		goto err_power_supply_reg;
	}
	wake_lock_init(&di->chg_wake_lock, WAKE_LOCK_SUSPEND,
					"isl9519-charger");
	INIT_DELAYED_WORK(&di->chg_update_work, chg_update_work_func);
	/* Register charger event notifier */
	di->chg_notif.notifier_call = isl9519_chg_notifier_callback;
	max8925_chg_register_client(&di->chg_notif);
#ifdef CONFIG_PROC_FS
	di->chg_proc = create_proc_entry("isl9519_wakelock", 0666, NULL);
	if (di->chg_proc) {
		di->chg_proc->write_proc = isl9519_write_proc;
		di->chg_proc->data = di;
	}
#endif
	dev_info(&client->dev, "charger is enabled\n");
	return 0;

err_power_supply_reg:
err_chg_setup:
	kfree(di);
	return ret;
}

static int isl9519_charger_remove(struct i2c_client *client)
{
	struct isl9519_device_info *di = i2c_get_clientdata(client);

	isl9519_stop_charging(di);
	if (di->chg_proc)
		remove_proc_entry("isl9519_wakelock", NULL);
	max8925_chg_unregister_client(&di->chg_notif);
	cancel_delayed_work(&di->chg_update_work);
	wake_lock_destroy(&di->chg_wake_lock);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->ac);
	kfree(di);
	return 0;
}

/* Charger attach/detach action will wakeup system by PMIC */
#ifdef CONFIG_PM
static int isl9519_charger_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl9519_device_info *di = i2c_get_clientdata(client);

	if (di->is_charging) {
		cancel_delayed_work(&di->chg_update_work);
		isl9519_enable_charge(di);
	}

	return 0;
}

static int isl9519_charger_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl9519_device_info *di = i2c_get_clientdata(client);
	int ret = 0;

	ret = isl9519_get_control(di);
	if (ret < 0)
		return ret;
	if (ret & CONTROL_AC_OK) {
		/* Restore update work */
		isl9519_start_charging(di);
	} else
		isl9519_stop_charging(di);

	return 0;
}

static const struct dev_pm_ops isl9519_pm_ops  = {
	.suspend = isl9519_charger_suspend,
	.resume = isl9519_charger_resume,
};
#endif

static const struct i2c_device_id isl9519_id[] = {
	{"isl9519", -1},
	{},
};

static struct i2c_driver isl9519_charger_driver = {
	.driver = {
		.name = "isl9519-charger",
#ifdef CONFIG_PM
		.pm = &isl9519_pm_ops,
#endif
	},
	.probe = isl9519_charger_probe,
	.remove = isl9519_charger_remove,
	.id_table = isl9519_id,
};

static int __init isl9519_charger_init(void)
{
	int ret;
	ret = i2c_add_driver(&isl9519_charger_driver);
	if (ret)
		pr_err("Unable to register ISL9519 charger driver");
	return ret;
}

module_init(isl9519_charger_init);

static void __exit isl9519_charger_exit(void)
{
	i2c_del_driver(&isl9519_charger_driver);
}

module_exit(isl9519_charger_exit);

MODULE_AUTHOR("Yunfan Zhang <yfzhang@marvell.com>");
MODULE_DESCRIPTION("ISL9519Q battery charger driver");
MODULE_LICENSE("GPL");
