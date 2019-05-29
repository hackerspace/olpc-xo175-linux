/*
 * wiskbc_power Battery Driver
 * Battery with A/D convert chip wiskbc_power(I2C)
 *
 * Copyright (c) 2011 Marvell Technology Ltd.
 * Yunfan Zhang <yfzhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
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
#include <linux/idr.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/reboot.h>

#define BAT_POLLING_INTERVAL	msecs_to_jiffies(1000)	/* Unit: ms */

enum{
	POWER_SUPPLY_STATUS_OFFLINE = 0, 
	POWER_SUPPLY_STATUS_ONLINE,
};

static int IsShutdown;

static struct wiskbc_power_info{

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend wiskbc_power_early_suspend;
#endif

	struct delayed_work monitor_work;
	int interval;

	int ac_online;
	struct power_supply ac;

	int battery_present;
	int battery_online;
	int battery_health;
	int battery_current;
	int battery_voltage;
	int battery_status;
	int battery_temp;
	int battery_charging_now;
	int battery_capacity;
	int battery_energy_now;
	int battery_energy_full;
	struct power_supply battery;

	struct mutex work_lock;
}*di;

extern s32 wiskbc_reg_read(u8);
extern s32 wiskbc_reg_write(u8, u8);
extern int wiskbc_firmware_updating(void);

static enum power_supply_property ac_power_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static int ac_get_property(struct power_supply *psy,
                                 enum power_supply_property psp,
                                 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp){

		case POWER_SUPPLY_PROP_ONLINE:
	
			val->intval = di->ac_online;
		        break;
		default:
	        	ret = -EINVAL;
        	break;
	}

	return ret;
}

static enum power_supply_property battery_power_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,

	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_NOW,
};

static int battery_get_property(struct power_supply *psy,
                                      enum power_supply_property psp,
                                      union power_supply_propval *val)
{
	int ret = 0;

	switch (psp){
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = di->battery_present;
			break;

		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = di->battery_online;
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = di->battery_health;
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = di->battery_current;
			break;
	
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:

			val->intval = di->battery_voltage;
			break;
	
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = di->battery_status;
			break;

		case POWER_SUPPLY_PROP_TEMP:
			val->intval = di->battery_temp;
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			val->intval =100;
			break;

		case POWER_SUPPLY_PROP_CHARGE_NOW:
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = di->battery_capacity;
			break;

		case POWER_SUPPLY_PROP_ENERGY_NOW:
			val->intval = di->battery_energy_now;
			break;

		case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
			val->intval = di->battery_energy_full;
			break;

		default:
        		ret = -EINVAL;
			break;
	}

	return ret;
}

static void wiskbc_power_monitor(struct work_struct *work)
{
	s32 val1 = 0, val2 = 0;
	
	if( IsShutdown == 0x333)
	{	
		printk(KERN_ERR " =====> %d %s %s\n", __LINE__, __func__, __FILE__);
		return;
	}
	mutex_lock(&di->work_lock);
	if(wiskbc_firmware_updating())
		goto err;

	val1 = wiskbc_reg_read(0x0);
	if(val1 < 0)						//indicates battery detected and kbc<->battery smbus init ok
		goto err;

	if(wiskbc_reg_read(0x20) & (0x1 << 1))				//<gpio status> a high indicates ac attached
		di->ac_online = 1;
	else
		di->ac_online = 0;

	val1 = wiskbc_reg_read(0x80);
	if(val1 & (0x1 << 3))						//indicates battery detected and kbc<->battery smbus init ok
		di->battery_online = 1;
	else
		di->battery_online = 0;

	if(wiskbc_reg_read(0x20) & (0x1 << 0)) 				//<gpio status> a high indicates battery attached
		di->battery_present = 1;
	else 
		di->battery_present = 0;

	val1 = wiskbc_reg_read(0x80);
	if(val1 & (0x1 << 4))
		di->battery_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	else
		di->battery_health = POWER_SUPPLY_HEALTH_GOOD;
		
	val1 = wiskbc_reg_read(0x85);
	val2 = wiskbc_reg_read(0x84);	
	di->battery_current = ((val1 << 8) | val2) * 1000;

	
	val1 = wiskbc_reg_read(0x83);
	val2 = wiskbc_reg_read(0x82);	
	di->battery_voltage = ((val1 << 8) | val2) * 1000;

	val1 = wiskbc_reg_read(0x80);
	if(val1 & (0x1 << 1))
		di->battery_status = POWER_SUPPLY_STATUS_CHARGING;
	else if(val1 & (0x1 << 2))
		di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if(val1 & (0x1 << 5))
		di->battery_status = POWER_SUPPLY_STATUS_FULL;
	else if(val1 & (0x1 << 7))
		di->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else{
		di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;	
	}
	
	val1 = wiskbc_reg_read(0x87);
	val2 = wiskbc_reg_read(0x86);
	di->battery_temp = ((val1 << 8) | val2) - 2732;

	val1 = wiskbc_reg_read(0x89);
	val2 = wiskbc_reg_read(0x88);
	di->battery_energy_now = ((val1 << 8) | val2) * 1000;

	val1 = wiskbc_reg_read(0x8b);
	val2 = wiskbc_reg_read(0x8a);
	di->battery_energy_full = ((val1 << 8) | val2) * 1000;

	di->battery_capacity = wiskbc_reg_read(0x81);

	power_supply_changed(&(di->battery));

	mutex_unlock(&di->work_lock);

	goto end;

err:
	mutex_unlock(&di->work_lock);
	printk(KERN_ERR " =====> %d %s %s\n", __LINE__, __func__, __FILE__);
	di->ac_online = 1;
	di->battery_online = 1;
	di->battery_present = 1;
	di->battery_health = POWER_SUPPLY_HEALTH_GOOD;
	di->battery_current = 0;
	di->battery_voltage = 8349000;
	di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
	di->battery_temp = 242;
	di->battery_capacity = 80;

end:	
	/* reschedule for the next time */
	schedule_delayed_work(&di->monitor_work, di->interval);

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void wiskbc_power_early_suspend_func(struct early_suspend *h)
{
	//cancel_delayed_work_sync(&di->monitor_work);

	/* Extend polling interval when early suspend */
//	di->interval = 10 * BAT_POLLING_INTERVAL;
//	schedule_delayed_work(&di->monitor_work, di->interval);
}

static void wiskbc_power_late_resume_func(struct early_suspend *h)
{
	//cancel_delayed_work_sync(&di->monitor_work);

	/* Normalize polling interval */
	//di->interval = BAT_POLLING_INTERVAL;
	//schedule_delayed_work(&di->monitor_work, di->interval);
}
#endif

static int wiskbc_power_probe(struct platform_device *pdev)
{
	int ret = 0;

	/* Allocate device info data */
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		printk(KERN_ERR "%d %s\n", __LINE__, __func__);
		return -ENOMEM;
	}

	di->ac.properties = ac_power_props;
	di->ac.num_properties = ARRAY_SIZE(ac_power_props);
	di->ac.get_property = ac_get_property;
	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	ret = power_supply_register(&pdev->dev, &di->ac);
	if(ret < 0)
		goto err; 

	di->battery.properties = battery_power_props;
	di->battery.num_properties = ARRAY_SIZE(battery_power_props);
	di->battery.get_property = battery_get_property;
	di->battery.name = "battery";
	di->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	ret = power_supply_register(&pdev->dev, &di->battery);
	if(ret < 0)
		goto err; 
		
	di->interval = BAT_POLLING_INTERVAL;
	INIT_DELAYED_WORK(&di->monitor_work, wiskbc_power_monitor);
	schedule_delayed_work(&di->monitor_work, di->interval);

#ifdef CONFIG_HAS_EARLYSUSPEND
	di->wiskbc_power_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	di->wiskbc_power_early_suspend.suspend = wiskbc_power_early_suspend_func;
	di->wiskbc_power_early_suspend.resume = wiskbc_power_late_resume_func;
	register_early_suspend(&di->wiskbc_power_early_suspend);
#endif

	if(ret)
		goto err;

	mutex_init(&di->work_lock);
	IsShutdown = 0x10;

	return 0;

err:
	kfree(di);
	return ret;
}

static int __devexit wiskbc_power_remove(struct platform_device *pdev)
{
	struct wiskbc_power_info *di = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&di->monitor_work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&di->wiskbc_power_early_suspend);
#endif

	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->battery);

	kfree(di);

	return 0;
}
static void wiskbc_power_shutdown(struct platform_device *pdev)
{
	IsShutdown = 0x333;
	printk(KERN_ERR " =====> %d %s %s\n", __LINE__, __func__, __FILE__);
	cancel_delayed_work_sync(&di->monitor_work);

}

#ifdef CONFIG_PM
static int wiskbc_power_suspend(struct platform_device *pdev, pm_message_t state)
{
	//if (device_may_wakeup(&pdev->dev)) {
	//}
	cancel_delayed_work_sync(&di->monitor_work);

	return 0;
}

static int wiskbc_power_resume(struct platform_device *pdev)
{
	//if (device_may_wakeup(&pdev->dev)) {
	//}

	schedule_delayed_work(&di->monitor_work, di->interval);
	return 0;
}
#else
#define wiskbc_power_suspend NULL
#define wiskbc_power_resume NULL
#endif

static struct platform_driver wiskbc_power_driver =
{
	.probe          = wiskbc_power_probe,
	.remove         = wiskbc_power_remove,
	.suspend        = wiskbc_power_suspend,
	.resume         = wiskbc_power_resume,
	.shutdown       = wiskbc_power_shutdown,
	.driver = {
	        .name = "wis-kbc-power"
   	}
};
static int wiskbc_battery_reboot_notifier_call(struct notifier_block *this,
                                       unsigned long code, void *_cmd)
{
//       mutex_lock(&mmp3_ddr_lock);
//       pr_err("%s: disabling devfreq\n", __func__);
//       mmp3_ddr_devfreq_disable = 1;
//       mutex_unlock(&mmp3_ddr_lock);
	IsShutdown = 0x333;
	printk(KERN_ERR " =====> %d %s %s\n", __LINE__, __func__, __FILE__);

       return NOTIFY_DONE;
}

static struct notifier_block wiskbc_battery_reboot_notifier = {
       .notifier_call = wiskbc_battery_reboot_notifier_call,
};

static int __init wiskbc_power_init(void)
{
	register_reboot_notifier(&wiskbc_battery_reboot_notifier);
	return platform_driver_register(&wiskbc_power_driver);
}

static void __exit wiskbc_power_exit(void)
{
	unregister_reboot_notifier(&wiskbc_battery_reboot_notifier);
	platform_driver_unregister(&wiskbc_power_driver);
}

module_init(wiskbc_power_init);
module_exit(wiskbc_power_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Wistron KBC Battery Driver");
