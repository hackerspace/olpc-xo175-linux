/*
 * FAN4010 High-Side Current Sensor Battery Driver.
 *
 * Copyright (c) 2012 Marvell Technology Ltd.
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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/power/fan4010_battery.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>


struct fan4010_battery_params {
	int status;
	int present;
	int volt;	/* µV */
	int cap;	/* percents: 0~100% */
	int chg_full;	/* charge: µAh */
	int chg_now;
	int eng_full;	/* energy: µWh */
	int eng_now;
	int health;
	int tech;
};

struct fan4010_device_info {
	struct device *dev;
	struct power_supply bat;
	struct fan4010_battery_params bat_params;
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *fan4010_dump;
#endif
	struct delayed_work bat_monitor_work;
	int (*get_bat_vol)(void);
	int (*get_sys_cur_vol)(void);
	int (*get_bat_state)(void);
	unsigned int bat_design_cap;
	unsigned int r_bat;
	unsigned int r_sns;
	unsigned int r_vout;
	unsigned int interval;
};

struct capacity_percent {
	int voltage;
	int percent;
};

#define uAh_to_uWh(val) (val * 37 / 10) /* Nominal voltage: 3.7v */

/* Default parameters: 1500mAh, 3.7V, 5.6Wh */
#define BAT_DESIGN_CAP		1500	/* mAh */
#define BAT_UPDATE_INTERVAL	30		/* seconds */
#define BAT_R_SNS			20		/* milli-ohms */
#define BAT_R_VOUT			1300	/* ohms */
#define BAT_IMPEDANCE		140		/* milli-ohms */

static struct capacity_percent cap_table[] = {
	{3200, 0}, {3320, 10}, {3420, 20}, {3500, 30}, {3550, 40},
	{3600, 50}, {3650, 60}, {3750, 70}, {3850, 80}, {3950, 90},
	{4100, 100},
};

/* Get capacity by voltage */
static int get_cap_by_vol(int vol)
{
	int i, size;
	int ret = -EINVAL;

	size = ARRAY_SIZE(cap_table);
	if (vol >= cap_table[size - 1].voltage) {
		ret = 100;
		return ret;
	} else if (vol <= cap_table[0].voltage) {
		ret = 0;
		return ret;
	}
	for (i = size - 2; i >= 0; i--) {
		if (vol < cap_table[i].voltage)
			continue;
		ret = (vol - cap_table[i].voltage) * 10
		    / (cap_table[i + 1].voltage - cap_table[i].voltage);
		ret += cap_table[i].percent;
		break;
	}
	return ret;
}

#define SMOOTH_CNT	10
/* Get battery voltage: mV */
static int fan4010_get_bat_vol(struct fan4010_device_info *di)
{
	int vol = 0, cnt = SMOOTH_CNT;
	while (cnt-- > 0)
		vol += di->get_bat_vol();
	return vol / SMOOTH_CNT;
}

/* Get system current: mA */
static int fan4010_get_sys_cur(struct fan4010_device_info *di)
{
	int vol = 0, cnt = SMOOTH_CNT;
	while (cnt-- > 0)
		vol += di->get_sys_cur_vol();
	vol /= SMOOTH_CNT;
	return (vol * 100 * 1000) / (di->r_sns * di->r_vout);
}

/* Get battery state: present or not */
static int fan4010_is_bat_present(struct fan4010_device_info *di)
{
	/* If can't get bat state, assume battery always online */
	if (!di->get_bat_state)
		return 1;
	return di->get_bat_state();
}

/* Get battery capacity: in percent */
static int fan4010_get_bat_cap(struct fan4010_device_info *di)
{
	int vol, vol_offset, sys_cur, cap;

	vol = fan4010_get_bat_vol(di);
	sys_cur = fan4010_get_sys_cur(di);
	/* Battery voltage compensation */
	if (power_supply_am_i_supplied(&di->bat)) {
		/* FIXME: add offset when charging */
		vol_offset = 0;
	} else
		vol_offset = -1 * (sys_cur * di->r_bat / 1000);
	vol -= vol_offset;
	cap = get_cap_by_vol(vol);
	return cap;
}

/* Update battery status */
static void fan4010_bat_update_status(struct fan4010_device_info *di)
{
	/* NOTE: hardcode battery type[Lion] and health[Good] */
	di->bat_params.health = POWER_SUPPLY_HEALTH_GOOD;
	di->bat_params.tech = POWER_SUPPLY_TECHNOLOGY_LION;
	/* Battery presence state */
	di->bat_params.present = fan4010_is_bat_present(di);
	/* Voltage */
	di->bat_params.volt = fan4010_get_bat_vol(di);
	/* Capacity: % */
	di->bat_params.cap = fan4010_get_bat_cap(di);
	/* Charge: µAh */
	di->bat_params.chg_full = di->bat_design_cap * 1000;
	di->bat_params.chg_now = di->bat_design_cap * di->bat_params.cap * 10;
	/* Energy: µWh */
	di->bat_params.eng_full = uAh_to_uWh(di->bat_params.chg_full);
	di->bat_params.eng_now = uAh_to_uWh(di->bat_params.chg_now);
	/* Charging status */
	if (di->bat_params.present) {
		if (di->bat_params.cap >= 100)
			di->bat_params.status = POWER_SUPPLY_STATUS_FULL;
		else {
			/* Charging: charger online */
			if (power_supply_am_i_supplied(&di->bat))
				di->bat_params.status =
						POWER_SUPPLY_STATUS_CHARGING;
			else
				di->bat_params.status =
						POWER_SUPPLY_STATUS_DISCHARGING;
		}
	} else
		di->bat_params.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static void bat_monitor_work_func(struct work_struct *work)
{
	struct fan4010_device_info *di =
			container_of(work, struct fan4010_device_info,
						bat_monitor_work.work);
	fan4010_bat_update_status(di);
	power_supply_changed(&di->bat);
	/* Reschedule for the next time */
	schedule_delayed_work(&di->bat_monitor_work, HZ * di->interval);
}

static enum power_supply_property fan4010_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int fan4010_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct fan4010_device_info *di =
		container_of(psy, struct fan4010_device_info, bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->bat_params.present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->bat_params.status;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->bat_params.cap;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = di->bat_params.chg_full;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->bat_params.chg_now;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = di->bat_params.eng_full;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = di->bat_params.eng_now;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = di->bat_params.health;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = di->bat_params.tech;
		break;
	/* NOTE: voltage keeps changing, read real-time value here */
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fan4010_get_bat_vol(di);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void fan4010_bat_external_power_changed(struct power_supply *psy)
{
	struct fan4010_device_info *di =
			container_of(psy, struct fan4010_device_info, bat);
	cancel_delayed_work_sync(&di->bat_monitor_work);
	schedule_delayed_work(&di->bat_monitor_work, HZ / 2);
}

static int fan4010_powersupply_init(struct fan4010_device_info *di)
{
	int ret = 0;

	/* Register battery props */
	di->bat.name = "fan4010-battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = fan4010_bat_props;
	di->bat.num_properties = ARRAY_SIZE(fan4010_bat_props);
	di->bat.get_property = fan4010_bat_get_property;
	di->bat.external_power_changed = fan4010_bat_external_power_changed;

	ret = power_supply_register(di->dev, &di->bat);
	return ret;
}

static int fan4010_device_init(struct fan4010_device_info *di,
				struct fan4010_battery_pdata *pdata)
{
	int ret = 0;
	/* Battery design capacity */
	if (!pdata->bat_design_cap)
		di->bat_design_cap = BAT_DESIGN_CAP;
	else
		di->bat_design_cap = pdata->bat_design_cap;
	/* Update interval */
	if (!pdata->interval)
		di->interval = BAT_UPDATE_INTERVAL;
	else
		di->interval = pdata->interval;
	/* Battery internal impedance */
	if (!pdata->r_bat)
		di->r_bat = BAT_IMPEDANCE;
	else
		di->r_bat = pdata->r_bat;
	/* System current sensor resistor */
	if (!pdata->r_sns)
		di->r_sns = BAT_R_SNS;
	else
		di->r_sns = pdata->r_sns;
	/* System current VOUT resistor */
	if (!pdata->r_vout)
		di->r_vout = BAT_R_VOUT;
	else
		di->r_vout = pdata->r_vout;
	/* Platform related callbacks */
	di->get_bat_vol = pdata->get_bat_vol;
	di->get_sys_cur_vol = pdata->get_sys_cur_vol;
	di->get_bat_state = pdata->get_bat_state;

	return ret;
}

#ifdef CONFIG_PROC_FS
static int fan4010_proc_read(char *buf, char **buffer_location,
				off_t offset, int buffer_length,
				int *zero, void *data)
{
	struct fan4010_device_info *di = data;
	int cnt = 0, ret = 0;
	if (offset > 0)
		return 0;
	ret = fan4010_get_bat_vol(di);
	cnt += sprintf(buf + cnt, "Battery voltage: %dmV\n", ret);
	ret = fan4010_get_sys_cur(di);
	cnt += sprintf(buf + cnt, "System current:  %dmA\n", ret);
	return cnt;
}
#endif	/* CONFIG_PROC_FS */

static __devinit int fan4010_battery_probe(struct platform_device *pdev)
{
	struct fan4010_device_info *di;
	struct fan4010_battery_pdata *pdata;
	int ret = 0;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -EINVAL;
	}
	if (!pdata->get_bat_vol) {
		dev_err(&pdev->dev, "missing get_bat_vol callback\n");
		return -EINVAL;
	}
	if (!pdata->get_sys_cur_vol) {
		dev_err(&pdev->dev, "missing get_sys_cur_vol callback\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	di->dev = &pdev->dev;
	platform_set_drvdata(pdev, di);

	ret = fan4010_device_init(di, pdata);
	if (ret) {
		dev_err(&pdev->dev, "failed to init device\n");
		goto err_dev_init;
	}
	/* Register power supply device for ac and usb */
	ret = fan4010_powersupply_init(di);
	if (ret) {
		dev_err(&pdev->dev, "failed to register battery power supply\n");
		goto err_power_supply_reg;
	}
	/* Update battery status */
	fan4010_bat_update_status(di);
	/* Init delayed work queue */
	INIT_DELAYED_WORK(&di->bat_monitor_work, bat_monitor_work_func);
	schedule_delayed_work(&di->bat_monitor_work, HZ / 2);
#ifdef CONFIG_PROC_FS
	di->fan4010_dump = create_proc_entry("fan4010_battery", 0666, NULL);
	if (di->fan4010_dump) {
		di->fan4010_dump->read_proc = fan4010_proc_read;
		di->fan4010_dump->data = di;
	} else
		dev_err(&pdev->dev, "failed to create proc entry\n");
#endif
	dev_info(&pdev->dev, "battery driver is enabled\n");
	return 0;

err_power_supply_reg:
err_dev_init:
	kfree(di);
	return ret;
}

static int fan4010_battery_remove(struct platform_device *pdev)
{
	struct fan4010_device_info *di = platform_get_drvdata(pdev);
	if (di->fan4010_dump)
		remove_proc_entry("fan4010_battery", NULL);
	cancel_delayed_work(&di->bat_monitor_work);
	power_supply_unregister(&di->bat);
	kfree(di);
	return 0;
}

#ifdef CONFIG_PM
static int fan4010_battery_suspend(struct device *dev)
{
	struct fan4010_device_info *di = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&di->bat_monitor_work);
	return 0;
}

static int fan4010_battery_resume(struct device *dev)
{
	struct fan4010_device_info *di = dev_get_drvdata(dev);

	schedule_delayed_work(&di->bat_monitor_work, HZ / 100);
	return 0;
}

static const struct dev_pm_ops fan4010_pm_ops  = {
	.suspend = fan4010_battery_suspend,
	.resume = fan4010_battery_resume,
};
#endif

static struct platform_driver fan4010_battery_driver = {
	.driver = {
		.name = "fan4010-battery",
#ifdef CONFIG_PM
		.pm = &fan4010_pm_ops,
#endif
	},
	.probe = fan4010_battery_probe,
	.remove = __devexit_p(fan4010_battery_remove),
};

static int __init fan4010_battery_init(void)
{
	return platform_driver_register(&fan4010_battery_driver);
}

module_init(fan4010_battery_init);

static void __exit fan4010_battery_exit(void)
{
	platform_driver_unregister(&fan4010_battery_driver);
}

module_exit(fan4010_battery_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FAN4010 Battery Driver");
MODULE_AUTHOR("Yunfan Zhang <yfzhang@marvell.com>");
MODULE_ALIAS("fan4010-battery");
