/*
 * FAN540x Switching Charger with USB-OTG Boost Regualtor driver.
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
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/power/fan540x_charger.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <plat/usb.h>

/* Register Address */
#define FAN540x_CNTL0		0x00
#define FAN540x_CNTL1		0x01
#define FAN540x_OREG		0x02
#define FAN540x_IC_INFO		0x03
#define FAN540x_IBAT		0x04
#define FAN540x_SP_CHG		0x05	/* Only for fan5403-05 */
#define FAN540x_SAFETY		0x06	/* Only for fan5403-05 */
#define FAN540x_MONITOR		0x10

/* Register Bit Map */
/* CONTROL0: 0x00 */
#define CNTL0_TMR_RST		(1 << 7)
#define CNTL0_OTG			(1 << 7) /* Same bit as TMR_RST, RO */
#define CNTL0_EN_STAT		(1 << 6)
#define CNTL0_STAT_MASK		(0x3 << 4)
#define CNTL0_STAT_SHIFT	4
#define CNTL0_BOOST			(1 << 3)
#define CNTL0_FAULT_MASK	(0x7 << 0)
#define CNTL0_FAULT_SHIFT	0
/* CONTROL1: 0x01 */
#define CNTL1_INLIM_MASK	(0x3 << 6)
#define CNTL1_INLIM_SHIFT	6
#define CNTL1_LOWV_MASK		(0x3 << 4)
#define CNTL1_LOWV_SHIFT	4
#define CNTL1_TE			(1 << 3)
#define CNTL1_CE_N			(1 << 2)
#define CNTL1_HZ_MODE		(1 << 1)
#define CNTL1_OPA_MODE		(1 << 0)
/* OREG: 0x02 */
#define OREG_VOREG_MASK		(0x3F << 2)
#define OREG_VOREG_SHIFT	2
#define OREG_OTG_PL			(1 << 1)
#define OREG_OTG_EN			(1 << 0)
/* IC_INFO: 0x03 */
#define INFO_VENDOR_MASK	(0x7 << 5)
#define INFO_VENDOR_SHIFT	5
#define INFO_PN_MASK		(0x3 << 3)
#define INFO_PN_SHIFT		3
#define INFO_REV_MASK		(0x7 << 0)
#define INFO_REV_SHIFT		0
/* IBAT: 0x04 */
#define IBAT_RESET			(1 << 7)
#define IBAT_IOCHG_MASK		(0x7 << 4)
#define IBAT_IOCHG_SHIFT	4
#define IBAT_ITERM_MASK		(0x7 << 0)
#define IBAT_ITERM_SHIFT	0
/* SP_CHG: 0x05 */
#define SPCHG_DIS_VERG		(1 << 6)
#define SPCHG_IO_LEVEL		(1 << 5)
#define SPCHG_SP			(1 << 4)
#define SPCHG_EN_LEVEL		(1 << 3)
#define SPCHG_VSP_MASK		(0x7 << 0)
#define SPCHG_VSP_SHIFT		0
/* SAFETY: 0x06 */
#define SFT_ISAFE_MASK		(0x7 << 4)
#define SFT_ISAFE_SHIFT		4
#define SFT_VSAFE_MASK		(0xF << 0)
#define SFT_VSAFE_SHIFT		0
/* MONITOR: 0x10 */
#define MONT_ITERM_CMP		(1 << 7)
#define MONT_VBAT_CMP		(1 << 6)
#define MONT_LINCHG			(1 << 5)
#define MONT_T_120			(1 << 4)
#define MONT_ICHG			(1 << 3)
#define MONT_IBUS			(1 << 2)
#define MONT_VBUS_VALID		(1 << 1)
#define MONT_CV				(1 << 0)

/* Default max monitor interval: should < 32s */
#define DEFAULT_MAX_MONITOR_INTERVAL	30
/* VOREG: Default cell's fully charged float voltage */
#define DEFAULT_VOREG	4100	/* 4.1V: (4100 - 3500)/20=0x1E */

/* Input Current Limit */
enum {
	CUR_INLIM_100MA = 0,
	CUR_INLIM_500MA,
	CUR_INLIM_800MA,
	CUR_INLIM_NO,	/* No limit */
};
/* Charge current limit: Rsns = 68m Ohm */
enum {
	CUR_CHG_550MA = 0,
	CUR_CHG_650MA,
	CUR_CHG_750MA,
	CUR_CHG_850MA,
	CUR_CHG_950MA,
	CUR_CHG_1050MA,
	CUR_CHG_1150MA,
	CUR_CHG_1250MA,
};

enum {
	DEV_ID_FAN5400 = 0,
	DEV_ID_FAN5401,
	DEV_ID_FAN5402,
	DEV_ID_FAN5403,
	DEV_ID_FAN5404,
	DEV_ID_FAN5405,
};

struct fan540x_device_info {
	struct device *dev;
	struct i2c_client *client;
	struct power_supply ac;
	struct power_supply usb;
	struct notifier_block chg_notif;
	struct delayed_work chg_monitor_work;
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *fan540x_dump;
#endif
	int ac_chg_online;
	int usb_chg_online;
	int is_charging;

	int voreg;
	int input_cur;
	int chg_cur;

	int monitor_interval;
	unsigned supply_type; /* power supply type */
	int gpio_dis;
	bool gpio_dis_active_low;
};

/* Global device info */
static struct fan540x_device_info *g_fan540x_di;

/* Return negative errno, else success */
static int fan540x_read_reg(struct i2c_client *i2c, u8 reg)
{
	int ret = 0;
	if (!i2c)
		return -EINVAL;
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0)
		dev_err(&i2c->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
	return ret;
}

/* Return negative errno, else zero on success */
static int fan540x_write_reg(struct i2c_client *i2c, u8 reg, u8 data)
{
	int ret = 0;
	if (!i2c)
		return -EINVAL;
	ret = i2c_smbus_write_byte_data(i2c, reg, data);
	if (ret < 0) {
		dev_err(&i2c->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			data, reg, ret);
		return ret;
	}
	return 0;
}

/* Return negative errno, else zero on success */
static int fan540x_set_bits(struct i2c_client *i2c, u8 reg,
				u8 mask, u8 data)
{
	int ret = 0;
	ret = fan540x_read_reg(i2c, reg);
	if (ret < 0)
		return ret;
	/* NOTE: Avoid to reset charge paramters by IBAT_RESET bit */
	if (reg == FAN540x_IBAT)
		ret &= ~IBAT_RESET;
	ret &= ~mask;
	ret |= data;
	ret = fan540x_write_reg(i2c, reg, ret);
	return ret;
}

/* Enable OTG Vbus output */
int fan540x_set_vbus(int on)
{
	struct fan540x_device_info *di = g_fan540x_di;
	struct i2c_client *i2c;
	int ret;
	if (!di)
		return -EINVAL;
	i2c = di->client;
	/* Read CNTL0 to clear Fault */
	fan540x_read_reg(i2c, FAN540x_CNTL0);
	/* Clear HZ MODE */
	ret = fan540x_set_bits(i2c, FAN540x_CNTL1, CNTL1_HZ_MODE, 0);
	if (ret)
		return ret;
	/* Set boost mode if on, else charge mode  */
	return fan540x_set_bits(i2c, FAN540x_CNTL1, CNTL1_OPA_MODE,
				on ? CNTL1_OPA_MODE : 0);
}
EXPORT_SYMBOL(fan540x_set_vbus);

static int fan540x_set_voreg(struct fan540x_device_info *di, int voreg)
{
	return fan540x_set_bits(di->client, FAN540x_OREG,
				OREG_VOREG_MASK, voreg << OREG_VOREG_SHIFT);
}

static int fan540x_set_input_cur(struct fan540x_device_info *di, int cur)
{
	return fan540x_set_bits(di->client, FAN540x_CNTL1,
				CNTL1_INLIM_MASK, cur << CNTL1_INLIM_SHIFT);
}

static int fan540x_set_chg_cur(struct fan540x_device_info *di, int cur)
{
	return fan540x_set_bits(di->client, FAN540x_IBAT,
				IBAT_IOCHG_MASK, cur << IBAT_IOCHG_SHIFT);
}

static int fan540x_get_voreg(struct fan540x_device_info *di)
{
	int ret;
	ret = fan540x_read_reg(di->client, FAN540x_OREG);
	if (ret < 0)
		return ret;
	return (ret & OREG_VOREG_MASK) >> OREG_VOREG_SHIFT;
}

static int fan540x_get_input_cur(struct fan540x_device_info *di)
{
	int ret;
	ret = fan540x_read_reg(di->client, FAN540x_CNTL1);
	if (ret < 0)
		return ret;
	return (ret & CNTL1_INLIM_MASK) >> CNTL1_INLIM_SHIFT;
}


static int fan540x_get_chg_cur(struct fan540x_device_info *di)
{
	int ret;
	ret = fan540x_read_reg(di->client, FAN540x_IBAT);
	if (ret < 0)
		return ret;
	return (ret & IBAT_IOCHG_MASK) >> IBAT_IOCHG_SHIFT;
}

static int fan540x_reset_reg(struct fan540x_device_info *di)
{
	int ret = 0;
	ret = fan540x_read_reg(di->client, FAN540x_IBAT);
	if (ret < 0)
		return ret;
	ret |= IBAT_RESET;
	ret = fan540x_write_reg(di->client, FAN540x_IBAT, ret);
	return ret;
}

#ifdef CONFIG_PROC_FS
static int fan540x_proc_read(char *buf, char **buffer_location,
				off_t offset, int buffer_length,
				int *zero, void *data)
{
	struct fan540x_device_info *di = data;
	int cnt = 0, ret = 0;
	if (offset > 0)
		return 0;
	/* Regsiter maps */
	ret = fan540x_read_reg(di->client, FAN540x_CNTL0);
	cnt += sprintf(buf + cnt, "[0x00]CNTL0:   0x%x\n", ret);
	ret = fan540x_read_reg(di->client, FAN540x_CNTL1);
	cnt += sprintf(buf + cnt, "[0x01]CONTL1:  0x%x\n", ret);
	ret = fan540x_read_reg(di->client, FAN540x_OREG);
	cnt += sprintf(buf + cnt, "[0x02]OREG:    0x%x\n", ret);
	ret = fan540x_read_reg(di->client, FAN540x_IC_INFO);
	cnt += sprintf(buf + cnt, "[0x03]ICINFO:  0x%x\n", ret);
	ret = fan540x_read_reg(di->client, FAN540x_IBAT);
	cnt += sprintf(buf + cnt, "[0x04]IBAT:    0x%x\n", ret);
	ret = fan540x_read_reg(di->client, FAN540x_SP_CHG);
	cnt += sprintf(buf + cnt, "[0x05]SP_CHG:  0x%x\n", ret);
	ret = fan540x_read_reg(di->client, FAN540x_SAFETY);
	cnt += sprintf(buf + cnt, "[0x06]SAFETY:  0x%x\n", ret);
	ret = fan540x_read_reg(di->client, FAN540x_MONITOR);
	cnt += sprintf(buf + cnt, "[0x10]MONITOR: 0x%x\n", ret);
	/* Charge parameters */
	ret = fan540x_get_voreg(di);
	cnt += sprintf(buf + cnt, "VOREG = 0x%x\n", ret);
	ret = fan540x_get_input_cur(di);
	cnt += sprintf(buf + cnt, "INLIM = 0x%x\n", ret);
	ret = fan540x_get_chg_cur(di);
	cnt += sprintf(buf + cnt, "IOCHG = 0x%x\n", ret);

	return cnt;
}
#endif	/* CONFIG_PROC_FS */

static int fan540x_charger_enable(struct fan540x_device_info *di)
{
	struct i2c_client *i2c = di->client;
	/* Read CNTL0 to clear Fault */
	fan540x_read_reg(i2c, FAN540x_CNTL0);
	/* Clear Boost Mode */
	fan540x_set_bits(i2c, FAN540x_CNTL1, CNTL1_OPA_MODE, 0);
	/* Clear HZ Mode */
	fan540x_set_bits(i2c, FAN540x_CNTL1, CNTL1_HZ_MODE, 0);
	/* Set VOREG */
	fan540x_set_voreg(di, di->voreg);
	/* Clear IO_LEVEL, charge current controlled by IOCHARGE */
	fan540x_set_bits(i2c, FAN540x_SP_CHG, SPCHG_IO_LEVEL, 0);
	/* Set input current limit */
	fan540x_set_input_cur(di, di->input_cur);
	/* Set charge current limit */
	fan540x_set_chg_cur(di, di->chg_cur);
	/* Enable Charging */
	if (gpio_is_valid(di->gpio_dis)) {
		/* Deassert disable pin */
		gpio_direction_output(di->gpio_dis,
			di->gpio_dis_active_low ? 1 : 0);
	}
	/* Assert CE_N: enable charger */
	fan540x_set_bits(i2c, FAN540x_CNTL1, CNTL1_CE_N, 0);
	pr_info("FAN540x charger: Enabled!\n");
	return 0;
}

static int fan540x_charger_disable(struct fan540x_device_info *di)
{
	struct i2c_client *i2c = di->client;
	/* Disable Charging */
	if (gpio_is_valid(di->gpio_dis)) {
		/* Assert disable pin */
		gpio_direction_output(di->gpio_dis,
			di->gpio_dis_active_low ? 0 : 1);
	}
	/* Deassert CE_N: disable charger */
	fan540x_set_bits(i2c, FAN540x_CNTL1, CNTL1_CE_N, CNTL1_CE_N);
	pr_info("FAN540x charger: Disabled!\n");
	return 0;
}

static int fan540x_chg_notifier_callback(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct fan540x_device_info *di =
		container_of(nb, struct fan540x_device_info, chg_notif);
	/* Parse vbus event passed by usb driver */
	switch (event) {
	case DEFAULT_CHARGER:
	case VBUS_CHARGER:
		/* Standard Downstream Port */
		di->supply_type = POWER_SUPPLY_TYPE_USB;
		di->input_cur = CUR_INLIM_NO;
		di->chg_cur = CUR_CHG_1250MA;
		/* Charger states */
		di->usb_chg_online = 1;
		di->ac_chg_online = 0;
		break;
	case AC_CHARGER_STANDARD:
		/* Dedicated Charging Port */
		di->supply_type = POWER_SUPPLY_TYPE_USB_DCP;
		di->input_cur = CUR_INLIM_NO;
		di->chg_cur = CUR_CHG_1250MA;
		/* Charger states */
		di->usb_chg_online = 0;
		di->ac_chg_online = 1;
		break;
	case AC_CHARGER_OTHER:
		/* Adapter Charger */
		di->supply_type = POWER_SUPPLY_TYPE_MAINS;
		di->input_cur = CUR_INLIM_NO;
		di->chg_cur = CUR_CHG_1250MA;
		/* Charger states */
		di->usb_chg_online = 0;
		di->ac_chg_online = 1;
		break;
	case NULL_CHARGER:
	default:
		/* No charger */
		di->supply_type = POWER_SUPPLY_TYPE_BATTERY;
		/* Charger states */
		di->usb_chg_online = 0;
		di->ac_chg_online = 0;
		break;
	}

	if (di->supply_type == POWER_SUPPLY_TYPE_BATTERY) {
		/* Stop charging */
		fan540x_charger_disable(di);
	} else {
		/* Start charing */
		fan540x_charger_enable(di);
	}

	return NOTIFY_OK;
}

static void chg_monitor_work_func(struct work_struct *work)
{
	struct fan540x_device_info *di =
		container_of(work, struct fan540x_device_info,
			chg_monitor_work.work);
	/* Reset 32s timer */
	fan540x_set_bits(di->client, FAN540x_CNTL0,
			CNTL0_TMR_RST, CNTL0_TMR_RST);
	schedule_delayed_work(&di->chg_monitor_work, HZ * di->monitor_interval);
}

/* MUST and ONLY before any other register is written */
static int fan540x_safety_reg_init(struct fan540x_device_info *di)
{
	int ret;
	u8 data = 0x70;	/* ISAFE:1.25A, VSAFE:4.2V */

	ret = fan540x_read_reg(di->client, FAN540x_SAFETY);
	if (ret)
		return ret;
	/* If it is, return */
	if (ret == data)
		return 0;
	/* Set VSAFE and ISAFE */
	ret = fan540x_write_reg(di->client, FAN540x_SAFETY, data);
	if (ret)
		return ret;
	/* Verify */
	ret = fan540x_read_reg(di->client, FAN540x_SAFETY);
	if (ret != data) {
		pr_err("%s: failed to init SAFETY register\n", __func__);
		return -1;
	}
	return 0;
}

static int fan540x_charger_setup(struct fan540x_device_info *di,
				struct fan540x_charger_pdata *pdata)
{
	struct i2c_client *i2c = di->client;
	int ret;
	/* Get chip information */
	ret = fan540x_read_reg(i2c, FAN540x_IC_INFO);
	if (ret < 0)
		return ret;
	dev_info(di->dev,
		"Vendor Code: 0x%x;Part Number: %d;Revision: 1.%d\n",
		(ret & INFO_VENDOR_MASK) >> INFO_VENDOR_SHIFT,
		(ret & INFO_PN_MASK) >> INFO_PN_SHIFT,
		(ret & INFO_REV_MASK) >> INFO_REV_SHIFT);
	/* Update ISAFE and VSAFE */
	fan540x_safety_reg_init(di);
	/* Reset charge paramters firstly */
	fan540x_reset_reg(di);
	/* Set Vlowv as 3.4V */
	fan540x_set_bits(di->client, FAN540x_CNTL1,
			CNTL1_LOWV_MASK, 0x0 << CNTL1_LOWV_SHIFT);
	/* Init monitor interval: seconds */
	if ((pdata->monitor_interval <= 0) || (pdata->monitor_interval > 30)) {
		dev_err(di->dev, "monitor interval is out of range,"
			"using default value: %ds\n",
			DEFAULT_MAX_MONITOR_INTERVAL);
		di->monitor_interval = DEFAULT_MAX_MONITOR_INTERVAL;
	} else
		di->monitor_interval = pdata->monitor_interval;
	/* Init VOREG */
	if (pdata->voreg >= 3500)
		di->voreg = (pdata->voreg - 3500) / 20;
	else
		di->voreg = (DEFAULT_VOREG - 3500) / 20;
	/* Init disable gpio */
	di->gpio_dis = pdata->gpio_dis;
	di->gpio_dis_active_low = pdata->gpio_dis_active_low;
	if (gpio_is_valid(di->gpio_dis)) {
		if (di->gpio_dis == 0)
			dev_warn(di->dev, "Warning: Using GPIO0 for disable pin!\n");
		ret = gpio_request(di->gpio_dis, "Charge Disable");
		if (ret) {
			dev_err(di->dev,
				"failed to request GPIO%d\n", di->gpio_dis);
			return ret;
		}
		/* Disbale by default */
		gpio_direction_output(di->gpio_dis,
			di->gpio_dis_active_low ? 0 : 1);
	}

	return 0;
}

static enum power_supply_property fan540x_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int fan540x_ac_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct fan540x_device_info *di =
		container_of(psy, struct fan540x_device_info, ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->ac_chg_online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property fan540x_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int fan540x_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct fan540x_device_info *di =
		container_of(psy, struct fan540x_device_info, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->usb_chg_online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char *fan540x_supplied_to[] = {
	"fan4010-battery",
};

static int fan540x_powersupply_init(struct fan540x_device_info *di,
				struct fan540x_charger_pdata *pdata)
{
	int ret = 0;

	if (pdata->supplied_to) {
		di->ac.supplied_to = pdata->supplied_to;
		di->ac.num_supplicants = pdata->num_supplicants;
		di->usb.supplied_to = pdata->supplied_to;
		di->usb.num_supplicants = pdata->num_supplicants;
	} else {
		di->ac.supplied_to = fan540x_supplied_to;
		di->ac.num_supplicants = ARRAY_SIZE(fan540x_supplied_to);
		di->usb.supplied_to = fan540x_supplied_to;
		di->usb.num_supplicants = ARRAY_SIZE(fan540x_supplied_to);
	}
	/* register ac charger props */
	di->ac.name = "fan540x-chg-ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = fan540x_ac_props;
	di->ac.num_properties = ARRAY_SIZE(fan540x_ac_props);
	di->ac.get_property = fan540x_ac_get_property;

	ret = power_supply_register(di->dev, &di->ac);
	if (ret)
		goto err_reg_ac;
	/* register usb charger props */
	di->usb.name = "fan540x-chg-usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = fan540x_usb_props;
	di->usb.num_properties = ARRAY_SIZE(fan540x_usb_props);
	di->usb.get_property = fan540x_usb_get_property;

	ret = power_supply_register(di->dev, &di->usb);
	if (ret)
		goto err_reg_usb;

	return ret;

err_reg_usb:
	power_supply_unregister(&di->ac);
err_reg_ac:
	return ret;
}

static int fan540x_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct fan540x_device_info *di;
	struct fan540x_charger_pdata *pdata;
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

	ret = fan540x_charger_setup(di, pdata);
	if (ret) {
		dev_err(&client->dev, "failed to setup charger\n");
		goto err_chg_setup;
	}
	/* Register power supply device for ac and usb */
	ret = fan540x_powersupply_init(di, pdata);
	if (ret) {
		dev_err(&client->dev, "failed to register ac/usb\n");
		goto err_power_supply_reg;
	}
	/* Init global device info */
	g_fan540x_di = di;
	/* Init delayed work queue */
	INIT_DELAYED_WORK(&di->chg_monitor_work, chg_monitor_work_func);
	/* Register charger event notifier */
	di->chg_notif.notifier_call = fan540x_chg_notifier_callback;
#ifdef CONFIG_USB_PXA_U2O
	mv_udc_register_client(&di->chg_notif);
#endif
	schedule_delayed_work(&di->chg_monitor_work, HZ / 100);
#ifdef CONFIG_PROC_FS
	di->fan540x_dump = create_proc_entry("fan540x_charger", 0666, NULL);
	if (di->fan540x_dump) {
		di->fan540x_dump->read_proc = fan540x_proc_read;
		di->fan540x_dump->data = di;
	} else
		dev_err(&client->dev, "failed to create proc entry\n");
#endif
	dev_info(&client->dev, "charger is enabled\n");
	return 0;

err_power_supply_reg:
	if (gpio_is_valid(di->gpio_dis))
		gpio_free(di->gpio_dis);
err_chg_setup:
	kfree(di);
	return ret;
}

static int fan540x_charger_remove(struct i2c_client *client)
{
	struct fan540x_device_info *di = i2c_get_clientdata(client);
	if (di->fan540x_dump)
		remove_proc_entry("fan540x_charger", NULL);
	if (gpio_is_valid(di->gpio_dis))
		gpio_free(di->gpio_dis);
	cancel_delayed_work(&di->chg_monitor_work);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->ac);
	kfree(di);
	return 0;
}

#ifdef CONFIG_PM
static int fan540x_charger_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fan540x_device_info *di = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&di->chg_monitor_work);
	return 0;
}

static int fan540x_charger_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fan540x_device_info *di = i2c_get_clientdata(client);

	schedule_delayed_work(&di->chg_monitor_work, HZ / 100);
	return 0;
}

static const struct dev_pm_ops fan540x_pm_ops  = {
	.suspend = fan540x_charger_suspend,
	.resume = fan540x_charger_resume,
};
#endif

static void fan540x_charger_shutdown(struct i2c_client *client)
{
	struct fan540x_device_info *di = i2c_get_clientdata(client);
	/* Disable Charger */
	fan540x_charger_disable(di);
}

static const struct i2c_device_id fan540x_id[] = {
	{"fan5400", DEV_ID_FAN5400},
	{"fan5401", DEV_ID_FAN5401},
	{"fan5402", DEV_ID_FAN5402},
	{"fan5403", DEV_ID_FAN5403},
	{"fan5404", DEV_ID_FAN5404},
	{"fan5405", DEV_ID_FAN5405},
	{},
};
MODULE_DEVICE_TABLE(i2c, fan540x_id);

static struct i2c_driver fan540x_charger_driver = {
	.driver = {
		.name = "fan540x-charger",
#ifdef CONFIG_PM
		.pm = &fan540x_pm_ops,
#endif
	},
	.probe = fan540x_charger_probe,
	.remove = __devexit_p(fan540x_charger_remove),
	.shutdown = fan540x_charger_shutdown,
	.id_table = fan540x_id,
};

static int __init fan540x_charger_init(void)
{
	int ret;
	ret = i2c_add_driver(&fan540x_charger_driver);
	if (ret)
		pr_err("Unable to register fan540x charger driver!\n");
	return ret;
}

module_init(fan540x_charger_init);

static void __exit fan540x_charger_exit(void)
{
	i2c_del_driver(&fan540x_charger_driver);
}

module_exit(fan540x_charger_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FAN540X Charger Driver");
MODULE_AUTHOR("Yunfan Zhang <yfzhang@marvell.com>");
MODULE_ALIAS("fan540x-charger");
