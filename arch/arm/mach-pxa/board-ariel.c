/*
 *  Copyright (C) 2012 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/pxa95x-i2c.h>
#include <linux/mfd/88pm80x.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/regulator/machine.h>
#include <linux/sd8x_rfkill.h>
#include <linux/mmc/sdhci.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/proc_fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/i2c/ft5x06_touch.h>
#include <linux/power_supply.h>
#include <linux/leds.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/pxa95xfb.h>
#include <mach/pxa95x_dvfm.h>
#include <linux/switch.h>
#include <mach/camera.h>
#include <media/soc_camera.h>
#include <mach/uio_hdmi.h>
#include <mach/audio.h>
#include <mach/usb-regs.h>
#include <plat/pxa27x_keypad.h>
#include <linux/keyreset.h>
#include <plat/devfreq.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/cmmb.h>

#include <plat/usb.h>
#include <mach/ariel.h>

#include "devices.h"
#include "generic.h"

#include "panel_settings.h"
#include <mach/display.h>
#ifdef CONFIG_RTC_DRV_PXA
#include <linux/rtc-pxa.h>
#endif

#ifdef CONFIG_LEDS_PCA963X
#include <linux/leds-pca963x.h>
#endif

#ifdef CONFIG_LEDS_LM3530
#include <linux/led-lm3530.h>
#endif

#ifdef CONFIG_CHARGER_ISL9226
#include <linux/power/isl9226.h>
#endif

#define BOARD_NR_IRQS	(IRQ_BOARD_START + 40)

static void init_rfreset_gpio(void)
{
}

#define PM8XXX_REGULATOR_MAX PM800_ID_RG_MAX

static struct regulator_consumer_supply regulator_supply[PM8XXX_REGULATOR_MAX];
static struct regulator_init_data regulator_data[PM8XXX_REGULATOR_MAX];

static int PM800_ID_regulator_index[] = {
	PM800_ID_BUCK1,
	PM800_ID_BUCK2,
	PM800_ID_BUCK3,
	PM800_ID_BUCK4,
	PM800_ID_BUCK5,
	PM800_ID_LDO1,
	PM800_ID_LDO2,
	PM800_ID_LDO3,
	PM800_ID_LDO4,
	PM800_ID_LDO5,
	PM800_ID_LDO6,
	PM800_ID_LDO7,
	PM800_ID_LDO8,
	PM800_ID_LDO9,
	PM800_ID_LDO10,
	PM800_ID_LDO11,
	PM800_ID_LDO12,
	PM800_ID_LDO13,
	PM800_ID_LDO14,
	PM800_ID_LDO15,
	PM800_ID_LDO16,
	PM800_ID_LDO17,
	PM800_ID_LDO18,
	PM800_ID_LDO19,
};


#define REG_SUPPLY_INIT(_id, _name, _dev_name)		\
{							\
	int _i = _id;				\
	regulator_supply[_i].supply		= _name;		\
	regulator_supply[_i].dev_name	= _dev_name;	\
}

/* notes: apply_uV which means proper voltage value (latest set value or min)
* would be applied first time when enabled. So it would be set 1 if min voltage
* == max voltage*/
#define REG_INIT(_id, _chip, _name, _min, _max, _always, _boot)	\
{									\
	int _i = _id;				\
	regulator_data[_i].constraints.name	=	__stringify(_name);	\
	regulator_data[_i].constraints.min_uV	= _min;	\
	regulator_data[_i].constraints.max_uV	= _max;	\
	regulator_data[_i].constraints.always_on	= _always;	\
	regulator_data[_i].constraints.boot_on	= _boot;	\
	regulator_data[_i].constraints.valid_ops_mask	=	\
		REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS;	\
	regulator_data[_i].num_consumer_supplies	= 1;	\
	regulator_data[_i].consumer_supplies	=	\
		&regulator_supply[_chip##_##_name];	\
	regulator_data[_i].driver_data	=	\
		&_chip##_regulator_index[_chip##_##_name];	\
	regulator_data[_i].constraints.apply_uV = (_min == _max);	\
}

static struct pm80x_rtc_pdata pm80x_rtc = {
	.vrtc           = 1,
	.rtc_wakeup     = 0,
#ifdef CONFIG_RTC_DRV_PXA
	.sync		= pxa_rtc_sync_time,
#endif
};

static int pm800_plat_config(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	if (!chip || !pdata ||
		chip->id != CHIP_PM800 ||
		!chip->base_page) {
		pr_err("%s:chip or pdata is not availiable!\n", __func__);
		return -EINVAL;
	}
	/* Initializain actions to enable 88pm805 */
	/* Clear WDT */
	/*clear this bit will cause system restart in C0, C0 has
	* different register definition with B0
	*/
	if (get_pmic_id() == PM800_CHIP_B0)
		pm80x_reg_write(chip->base_page, 0x0E, 0x00);
	/* Enable 32Khz-out-1 and resetoutn */
	pm80x_reg_write(chip->base_page, 0xE1, 0xB0);
	/* Enable 32Khz-out-3  low jitter */
	pm80x_reg_write(chip->base_page, 0x21, 0x20);
	/* Enable 32Khz-out-3 */
	pm80x_reg_write(chip->base_page, 0xE2, 0x22);
	/* Set XO CAP to 22pF to avoid speaker noise */
	if (get_pmic_id() >= PM800_CHIP_C0)
		pm80x_reg_write(chip->base_page, 0xE8, 0x70);

	return 0;
}

static void mic_set_power(unsigned int on)
{
	struct regulator	*v_ldo = regulator_get(NULL, "mic_bias");
	if (IS_ERR(v_ldo)) {
		v_ldo = NULL;
		pr_err("Get regulator error\n");
		return;
	}
	if (on)
		regulator_enable(v_ldo);
	else
		regulator_disable(v_ldo);

	regulator_put(v_ldo);
	v_ldo = NULL;
}

static struct pm80x_headset_pdata pm80x_headset = {
	.gpio	= 4,			/* GPIO 4 */
	.gpio_ctl = 0x32,		/* PM800 GPIO 4 */
	.gpio_enable_irq = (1 << 4),
	.gpio_set_mask = 0xff,
	.gpio_set_val = 0x08,
	.gpio_val_bit = (1 << 0),
	.mic_set_power = mic_set_power,
};

static struct pm80x_dvc_pdata pm80x_dvc = {
};

static struct pm80x_platform_data pm800_info = {
	.headset = &pm80x_headset,
	.regulator	= regulator_data,
	.rtc  = &pm80x_rtc,
	.dvc = &pm80x_dvc,
	.companion_addr		= 0x38,
	.base_page_addr		= 0x30,
	.power_page_addr	= 0x31,
	.gpadc_page_addr	= 0x32,
	.test_page_addr		= 0x37,
	.irq_mode		= 0,
	.irq_base		= IRQ_BOARD_START,
	/*PM805 has it's own interrupt line (GPIO96)!!!*/
	.irq_companion		= gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO96)),

	.i2c_port		= PI2C_PORT,
	.pm800_plat_config = pm800_plat_config,
};

extern struct pxa95x_freq_mach_info freq_mach_info;

static void headsetflag_init_pm800(void)
{
	pm800_info.headset_flag = 1;
}

static void regulator_init_pm800(void)
{
	int i = 0;

	REG_SUPPLY_INIT(PM800_ID_LDO16, "v_cam", NULL);
	REG_SUPPLY_INIT(PM800_ID_LDO8, "v_lcd", NULL);
	REG_SUPPLY_INIT(PM800_ID_LDO6, "v_ihdmi", NULL);

	REG_INIT(i++, PM800_ID, LDO16, 1800000, 3300000, 0, 0);
	REG_INIT(i++, PM800_ID, LDO8, 1800000, 3300000, 1, 1);
	REG_INIT(i++, PM800_ID, LDO6, 1200000, 3300000, 0, 0);
	switch (get_board_id()) {
	case OBM_TK_ARIEL_P10:
	case OBM_TK_ARIEL_P11:
		/* Turn on LDO12 and 17 before wifi regulator support added */
		/*OBM haven't support DKB 2.1 version yet,  so currently we
		* distinguish it by Procida version
		*/
		if (get_pmic_id() == PM800_CHIP_C0) {
			REG_SUPPLY_INIT(PM800_ID_LDO18, "Vdd_IO", NULL);
			REG_SUPPLY_INIT(PM800_ID_LDO2, "mic_bias", NULL);
			REG_SUPPLY_INIT(PM800_ID_LDO17, "v_cam_af_vcc", NULL);
			REG_SUPPLY_INIT(PM800_ID_LDO9, "v_wifi_3v3", NULL);
			REG_SUPPLY_INIT(PM800_ID_LDO10, "v_vibrator", NULL);
			REG_SUPPLY_INIT(PM800_ID_LDO12, "vmmc_io", NULL);
			REG_SUPPLY_INIT(PM800_ID_LDO13, "vmmc", "sdhci-pxa.1");
			REG_SUPPLY_INIT(PM800_ID_LDO19, "v_gps", NULL);

			REG_INIT(i++, PM800_ID, LDO18, 1800000, 3300000, 0, 0);
			REG_INIT(i++, PM800_ID, LDO2, 1200000, 3300000, 0, 0);
			REG_INIT(i++, PM800_ID, LDO17, 1200000, 3300000, 0, 0);
			REG_INIT(i++, PM800_ID, LDO9, 3300000, 3300000, 0, 0);
			REG_INIT(i++, PM800_ID, LDO10, 2800000, 2800000, 0, 0);
			REG_INIT(i++, PM800_ID, LDO12, 1800000, 2800000, 0, 0);
			REG_INIT(i++, PM800_ID, LDO13, 2800000, 2800000, 1, 1);
			REG_INIT(i++, PM800_ID, LDO19, 1800000, 1800000, 1, 1);
		} else {
			BUG();
		}
		break;
	default:
		BUG();
		break;
	}
	pr_info("%s: select saarC NEVO austica ldo map\n", __func__);

	pm800_info.num_regulators = i;
}

#ifdef CONFIG_CHARGER_ISL9226
static struct isl9226_charger_pdata isl9226_data = {
	.default_input_current = 95,		/*charge current if USB enumeration fail: mA*/
	.usb_input_current = 475,	/*USB charger current: mA*/
	.ac_input_current = 950,	/*AC charger current: mA*/
	.eoc_current = 100,		/*end of charge current: mA*/
	.prechg_current = 130,	/*pre-charge current: mA */
	.prechg_voltage = 3000,	/*pre-charge voltage: mV*/
};


struct platform_device isl9226_device = {
	.name = "isl9226-charger",
	.id	= -1,
};
#endif
static struct i2c_board_info i2c1_80x_info[] = {
	{
		I2C_BOARD_INFO("88PM80x", 0x34),
		.platform_data	= &pm800_info,
		.irq		= IRQ_PMIC_INT,
	},
#ifdef CONFIG_CHARGER_ISL9226
	{
		I2C_BOARD_INFO("isl9226", 0x59),
		.platform_data = &isl9226_data,
	},
#endif
};

extern int set_vmeta_freqs_table(struct devfreq *devfreq);
static struct devfreq_platform_data devfreq_vmeta_pdata = {
	.clk_name = "VMETA_CLK",
	.setup_freq_table = set_vmeta_freqs_table,
};

static struct clk *clk_tout_s0;

/* specific 8787 power on/off setting for SAARC */
static void wifi_set_power(unsigned int on)
{
	unsigned long wlan_pd_mfp = 0;
	int gpio_power_down = mfp_to_gpio(MFP_PIN_GPIO70);
	/* 8787_LDO_EN(WIB_EN) is useless on ariel */
	/* the 1v8 is supplied by vbuk2 on ariel */
	struct regulator *v_ldo_3v3 = NULL;

	v_ldo_3v3 = regulator_get(NULL, "v_wifi_3v3");
	if (IS_ERR(v_ldo_3v3)) {
		v_ldo_3v3 = NULL;
		return;
	}

	wlan_pd_mfp = mfp_read(gpio_power_down);

	if (on) {
		regulator_enable(v_ldo_3v3);

		/* set wlan_pd pin to output high in low power
			mode to ensure 8787 is not power off in low power mode*/
		wlan_pd_mfp |= 0x100;
		mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* enable 32KHz TOUT */
		clk_enable(clk_tout_s0);
	} else {
		/*set wlan_pd pin to output low in low power
			mode to save power in low power mode */
		wlan_pd_mfp &= ~0x100;
		mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* disable 32KHz TOUT */
		clk_disable(clk_tout_s0);
		regulator_disable(v_ldo_3v3);
	}
	regulator_put(v_ldo_3v3);
}

#if defined(CONFIG_MMC_SDHCI_PXAV2_TAVOR) || defined(CONFIG_MMC_SDHCI_PXAV3)
static struct wake_lock wifi_delayed_work_wake_lock;

static void pxa95x_handle_cdint(struct sdhci_host *host)
{
	if (mmc_card_sdio(host->mmc->card) && host->mmc->card->disabled) { /* host sleep feature case */
		/* disable CARD INT 1st to avoid serving CARD INT repeatedly */
		host->mmc->ops->enable_sdio_irq(host->mmc, 0);

		/* get wakelock to trigger devices resume sequence,
		necessary for the current PM implementation - partial suspend.
		And when the Full-suspend is implemented in future, such wakelock will be delected */
		wake_lock_timeout(&wifi_delayed_work_wake_lock, 3*HZ);
		printk(KERN_INFO"[sd/sdio]-->sdhci-irq: get_sdio_wakelock, 3 seconds.\n");
	} else { /* normal case */
		mmc_signal_sdio_irq(host->mmc);
		pr_debug("[sd/sdio]-->sdhci-irq: mmc_signal_sdio_irq.\n");
	}
}

struct sdhci_pxa_platdata mci0_platform_data = {
	.flags	= PXA_FLAG_CARD_PERMANENT |
				PXA_FLAG_SD_8_BIT_CAPABLE_SLOT |
				PXA_FLAG_ACITVE_IN_SUSPEND |
				PXA_FLAG_ENABLE_CLOCK_GATING |
				PXA_FLAG_KEEP_POWER_IN_SUSPEND,
};
EXPORT_SYMBOL(mci0_platform_data);

#define MAX_SD_PINS 6
/*
* This function is used to check whether SD slot has short circuit issue
* 1. Set all of the MFPR as GPIO input
* 2. If there are external pull-ups, don't set pull-up.
* 3. If the pin is not HIGH, it means short circuit happening.
* 4. Recover the MFPR as SD functionality.
*/
static int pxa_check_sd_short_circuit(struct sdhci_host *host, const int mfp_start,
	const int mfp_num, const int pull_up)
{
	unsigned long mfp_bak[MAX_SD_PINS];
	int i;
	unsigned long val;
	int err;
	int ret = 0;
	int regulator_enabled;
	int retry = 0;

	if (!host->vmmc) {
		printk(KERN_INFO"%s short circuit check not necessary\n",
			mmc_hostname(host->mmc));
		return 0;
	}

	if (mfp_num > MAX_SD_PINS) {
		printk(KERN_ERR "%s mfp_num(%d) > MAX_SD_PINS(%d)\n",
			__func__, mfp_num, MAX_SD_PINS);
		return 1;
	}

	regulator_enabled = regulator_is_enabled(host->vmmc);
	if (regulator_enabled)
		regulator_disable(host->vmmc);

	for (i = 0; i < mfp_num; i++) {
		mfp_bak[i] = mfp_read(mfp_start+i);
		val = mfp_bak[i];
		val &= ~(MFPR_PULL_MASK | MFPR_AF_MASK);
		if (pull_up)
			val |= MFPR_PULL_FLOAT;
		else
			val |= MFPR_PULL_HIGH;
		mfp_write(mfp_start + i, val);
		err = gpio_request(MFP_PIN(mfp_start+i), NULL);
		if (err) {
			printk(KERN_ERR"%s gpio_request[%d] fail\n",
					__func__, MFP_PIN(mfp_start+i));
			ret = 1;
			goto out;
		}
		gpio_direction_input(MFP_PIN(mfp_start+i));
	}
	udelay(200);

	regulator_enable(host->vmmc);
	for (retry = 0; retry < 3; retry++) {
		/*retry 3 times, in case the voltage cannot reach in 300us*/
		udelay(300);/*TODO: check whether LDO can reach high voltage*/
		for (i = 0; i < mfp_num; i++) {
			val = gpio_get_value(MFP_PIN(mfp_start+i));
			val &= 1 << (MFP_PIN(mfp_start+i) % 32);
			if (!val && (retry == 2)) {
				ret = 1;
				printk(KERN_ERR "%s GPIO[%d] short circuit "
						"detected\n", __func__,
						MFP_PIN(mfp_start+i));
			}
		}
		if (!ret) /* There are no short circuit detected*/
			break;
	}

	regulator_disable(host->vmmc);
out:
	for (i = 0; i < mfp_num; i++) {
		gpio_free(MFP_PIN(mfp_start + i));
		mfp_write(mfp_start + i, mfp_bak[i]);
	}

	return ret;
}

/*
 * According to SD spec, CLK, CMD, and DAT3-0 should be LOW
 * before VDD becomes high.
 */
static int pxa_safe_sd_on(struct sdhci_host *host, const int mfp_start,
	const int mfp_num)
{
	unsigned long mfp_bak[MAX_SD_PINS];
	int i;
	unsigned long val;
	int err;
	int ret = 0;
	int regulator_enabled;

	if (!host->vmmc)
		return 0;

	if (mfp_num > MAX_SD_PINS) {
		printk(KERN_ERR "%s mfp_num(%d) > MAX_SD_PINS(%d)\n",
				__func__, mfp_num, MAX_SD_PINS);
		return 1;
	}

	regulator_enabled = regulator_is_enabled(host->vmmc);
	if (regulator_enabled)
		regulator_disable(host->vmmc);

	for (i = 0; i < mfp_num; i++) {
		mfp_bak[i] = mfp_read(mfp_start+i);
		val = mfp_bak[i];
		val &= ~MFPR_AF_MASK;
		mfp_write(mfp_start + i, val);
		err = gpio_request(MFP_PIN(mfp_start+i), NULL);
		if (err) {
			printk(KERN_ERR "%s gpio_request[%d] fail\n",
					__func__, MFP_PIN(mfp_start + i));
			ret = 1;
			goto out;
		}
		gpio_direction_output(MFP_PIN(mfp_start + i), 0);
		printk(KERN_ERR "gpio[%d] set as LOW before power on\n",
				MFP_PIN(mfp_start + i));
	}

	udelay(200);
	regulator_enable(host->vmmc);
	udelay(300);

out:
	for (i = 0; i < mfp_num; i++) {
		gpio_free(MFP_PIN(mfp_start + i));
		mfp_write(mfp_start + i, mfp_bak[i]);
	}

	return ret;
}

/*
* This function is support for low voltage card.
* Host should switch the voltage from 3.3v to 1.8v
* if both the host and the card support SD Rev. 3.0 in indentification stage.
*/
static void pxa_mci1_signal_1v8(struct sdhci_host *host, int set)
{
	int vol,retval;
	struct regulator *v_ldo_vmmc_x;
	vol = set ? 1800000 : 2800000;

	v_ldo_vmmc_x = regulator_get(NULL, "vmmc_io");
	if (IS_ERR(v_ldo_vmmc_x)) {
		printk(KERN_ERR "Failed to get VMMC_IO\n");
		return;
	}
	retval = regulator_set_voltage(v_ldo_vmmc_x, vol, vol);
	regulator_enable(v_ldo_vmmc_x);
	msleep(10);
	regulator_put(v_ldo_vmmc_x);
	return;
}

struct sdhci_pxa_platdata mci1_platform_data = {
	.flags = PXA_FLAG_ENABLE_CLOCK_GATING |
			PXA_FLAG_ACITVE_IN_SUSPEND,
	.ext_cd_gpio = 0,
	.ext_cd_gpio_invert = 1,
	.mfp_start = 116,
	.mfp_num = 6,
	.pull_up = 0,
	.check_short_circuit = pxa_check_sd_short_circuit,
	.safe_regulator_on = pxa_safe_sd_on,
	.signal_1v8 = pxa_mci1_signal_1v8,
};
EXPORT_SYMBOL(mci1_platform_data);

struct sdhci_pxa_platdata mci2_platform_data = {
	.flags  = PXA_FLAG_CARD_PERMANENT |
			PXA_FLAG_SDIO_CTRLCLKSRC_GATE,
	.pm_caps = MMC_PM_KEEP_POWER |
				MMC_PM_IRQ_ALWAYS_ON,
	.handle_cdint = pxa95x_handle_cdint,
};
EXPORT_SYMBOL(mci2_platform_data);

static void __init init_mmc(void)
{
	int gpio_rst, gpio_pd;

	gpio_pd = mfp_to_gpio(MFP_PIN_GPIO70);
	gpio_rst = mfp_to_gpio(MFP_PIN_GPIO97);

	if (cpu_is_pxa978())
		mci1_platform_data.quirks = SDHCI_QUIRK_INVERTED_WRITE_PROTECT;

	/*add emmc only, need to add sdcard and sdio later*/
	pxa95x_set_mci_info(0, &mci0_platform_data);
	pxa95x_set_mci_info(1, &mci1_platform_data);
#ifdef CONFIG_SD8XXX_RFKILL
	add_sd8x_rfkill_device(gpio_pd, gpio_rst,
			&mci2_platform_data.pmmc, wifi_set_power);

	clk_tout_s0 = clk_get(NULL, "CLK_TOUT_S0");
	if (IS_ERR(clk_tout_s0))
		pr_err("init_mmc: unable to get tout_s0 clock");

	pxa95x_set_mci_info(2, &mci2_platform_data);
#endif
}
#endif

static void vibrator_set_power(int on)
{
	struct regulator *v_ldo = regulator_get(NULL, "v_vibrator");

	if (IS_ERR(v_ldo)) {
		v_ldo = NULL;
		pr_err("Get regulator error\n");
		return;
	}

	if (on)
		regulator_enable(v_ldo);
	else
		regulator_disable(v_ldo);

	regulator_put(v_ldo);
	v_ldo = NULL;
}

static struct pm80x_vibrator_pdata vibrator_pdata = {
	.min_timeout = 100,
	.vibrator_power = vibrator_set_power,
};

static struct gpio_led ariel_gpio_leds[] = {
	{
		.name = "button-backlight",
		.gpio = mfp_to_gpio(MFP_PIN_GPIO62),
		.active_low = 1,
	},
};

static struct gpio_led_platform_data ariel_gpio_led_data = {
	.leds	 = ariel_gpio_leds,
	.num_leds = ARRAY_SIZE(ariel_gpio_leds),
};

static struct platform_device ariel_leds_gpio = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &ariel_gpio_led_data,
	},
};

#if defined(CONFIG_TOUCHSCREEN_FT5X06)
static int ft5x06_touch_io_power_onoff(int on)
{
	/* touch power is LDO8, it's shared by LCD and Sensors, it is on */
	return 0;
}

static void ft5x06_touch_reset(void)
{
	unsigned int touch_reset;

	touch_reset = mfp_to_gpio(MFP_PIN_GPIO127);

	if (gpio_request(touch_reset, "ft5x06_reset")) {
		pr_err("Failed to request GPIO for ft5x06_reset pin!\n");
		goto out;
	}

	gpio_direction_output(touch_reset, 1);
	msleep(5);
	gpio_direction_output(touch_reset, 0);
	msleep(5);
	gpio_direction_output(touch_reset, 1);
	msleep(300);
	printk(KERN_INFO "ft5x06_touch reset done.\n");
	gpio_free(touch_reset);
out:
	return;
}

#define VIRT_KEYS(x...)  __stringify(x)
static ssize_t virtual_keys_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_MENU)
		":60:850:120:100" ":"
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_HOMEPAGE)
		":180:850:120:100" ":"
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_BACK)
		":300:850:120:100" ":"
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_SEARCH)
		":420:850:120:100\n");
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5x06-ts",
		.mode = S_IRUGO,
	},
	.show = &virtual_keys_show,
};

static struct attribute *props_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group props_attr_group = {
	.attrs = props_attrs,
};

static int ft5x06_set_virtual_key(struct input_dev *input_dev)
{
	struct kobject *props_kobj;
	int ret = 0;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_HOMEPAGE, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_SEARCH, input_dev->keybit);

	props_kobj = kobject_create_and_add("board_properties", NULL);
	if (props_kobj)
		ret = sysfs_create_group(props_kobj, &props_attr_group);
	if (!props_kobj || ret)
		printk(KERN_ERR "failed to create board_properties\n");

	return 0;
}

static struct ft5x06_touch_platform_data ft5x06_touch_data = {
	.power = ft5x06_touch_io_power_onoff,
	.reset = ft5x06_touch_reset,
	.abs_x_max = 480,
	.abs_y_max = 800,
	.set_virtual_key = ft5x06_set_virtual_key,
};
#endif

#if defined(CONFIG_LEDS_LM3530)
static struct lm3530_platform_data lm3530_pdata = {
	.mode = LM3530_BL_MODE_MANUAL,
	.max_current = LM3530_FS_CURR_29mA,
	.brt_ramp_law = true,
	.brt_ramp_fall = LM3530_RAMP_TIME_1ms,
	.brt_ramp_rise = LM3530_RAMP_TIME_1ms,
	.brt_val = 0,
};

static void __init lm3530_enable(void)
{
	int gpio;
	gpio = mfp_to_gpio(MFP_PIN_GPIO72);
	if (gpio_request(gpio, "backlight_enable"))
		pr_err("%s: Request gpio #%d failed\n", __func__, gpio);
	else
		printk(KERN_ERR "set backlight enable\n");
	gpio_direction_output(gpio, 1);
	usleep_range(500, 1500);
	gpio_free(gpio);
}
#endif

static struct i2c_board_info i2c2_info[] = {
#ifdef CONFIG_BATTERY_BQ27425
	{
		I2C_BOARD_INFO("bq27425", 0x55),
		.irq           = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO110)),
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_FT5X06)
	{
		I2C_BOARD_INFO("ft5x06_touch", 0x3D),
		.irq           = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO130)),
		.platform_data = &ft5x06_touch_data,
	},
#endif
#if defined(CONFIG_LEDS_LM3530)
	{
		I2C_BOARD_INFO("lm3530-led", 0x38),
		.platform_data = &lm3530_pdata,
	},
#endif
};

#if defined(CONFIG_LEDS_PCA963X)
static struct pca963x_platform_data pca963x_pdata = {
	.num_leds = 4,
	.leds = {
		[0] = {
			.name = "blue",
			.state = PCA963X_LED_PWM_GRPPWM,
		},
		[1] = {
			.name = "green",
			.state = PCA963X_LED_PWM_GRPPWM,
		},
		[2] = {
			.name = "red",
			.state = PCA963X_LED_PWM_GRPPWM,
		},
	},
};
#endif

static struct i2c_board_info i2c3_info[] = {
#if defined(CONFIG_SENSORS_BMA250)
	{
		I2C_BOARD_INFO("bma250", 0x18),
		.platform_data = NULL,
	},
#endif
#if defined(CONFIG_SENSORS_BMM050)
	{
		I2C_BOARD_INFO("bmm050", 0x10),
		.platform_data = NULL,
	},
#endif
#if defined(CONFIG_SENSORS_BMM050_MPU3050)
	{
		I2C_BOARD_INFO("itg3200", 0x68),
		.platform_data = NULL,
	},
#endif
#if defined(CONFIG_SENSORS_APDS990X_MRVL)
	{
		I2C_BOARD_INFO("apds990x", 0x39), /* 0x72 */
		.irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO87)),
	},
#endif
#if defined(CONFIG_LEDS_PCA963X)
	{
		I2C_BOARD_INFO("pca9632tk", 0x62),
		.platform_data = &pca963x_pdata,
	},
#endif
};

static void register_i2c_board_info(void)
{

	switch (get_board_id()) {
	case OBM_TK_ARIEL_P10:
	case OBM_TK_ARIEL_P11:
		pm800_info.vibrator = &vibrator_pdata;
		i2c_register_board_info(0, ARRAY_AND_SIZE(i2c1_80x_info));
		i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info));
		break;
	default:
		pr_err("%s: Unknown board type!\n", __func__);
		BUG();
	}

	i2c_register_board_info(2, ARRAY_AND_SIZE(i2c3_info));
}

/* workaround for reset i2c bus by GPIO20 -SCL, GPIO21 -SDA,
GPIO125 -SCL, GPIO126 -SDA */
static void i2c_pxa_bus_reset(int i2c_adap_id)
{
	int scl_pin=0,sda_pin=0;
	int i2c_scl_mfp,i2c_sda_mfp;
	int ccnt;
	if (i2c_adap_id == 1) {
		scl_pin = mfp_to_gpio(MFP_PIN_GPIO125);
		sda_pin = mfp_to_gpio(MFP_PIN_GPIO126);
	}
	else if (i2c_adap_id == 2) {
		scl_pin = mfp_to_gpio(MFP_PIN_GPIO20);
		sda_pin = mfp_to_gpio(MFP_PIN_GPIO21);
	}
	else if (i2c_adap_id == 0) {
		scl_pin = mfp_to_gpio(MFP_PIN_GPIO68);
		sda_pin = mfp_to_gpio(MFP_PIN_GPIO69);
	}
	i2c_scl_mfp = mfp_read(scl_pin);
	i2c_sda_mfp = mfp_read(sda_pin);
	mfp_write(scl_pin, i2c_scl_mfp & 0xfff8);
	mfp_write(sda_pin, i2c_sda_mfp & 0xfff8);

	if (gpio_request(scl_pin, "SCL")) {
		pr_err("Failed to request GPIO for SCL pin!\n");
		goto out;
	}
	if (gpio_request(sda_pin, "SDA")) {
		pr_err("Failed to request GPIO for SDA pin!\n");
		goto out_sda;
	}
	pr_info("\t<<<i2c bus gpio reseting,i2c bus id is %d>>>\n",i2c_adap_id);

	gpio_direction_input(sda_pin);
	for (ccnt = 20; ccnt; ccnt--) {
		gpio_direction_output(scl_pin, ccnt & 0x01);
		udelay(4);
	}
	gpio_direction_output(scl_pin, 0);
	udelay(4);
	gpio_direction_output(sda_pin, 0);
	udelay(4);
	/* stop signal */
	gpio_direction_output(scl_pin, 1);
	udelay(4);
	gpio_direction_output(sda_pin, 1);
	udelay(4);

	mfp_write(scl_pin, i2c_scl_mfp);
	mfp_write(sda_pin, i2c_sda_mfp);
	gpio_free(sda_pin);
out_sda:
	gpio_free(scl_pin);
out:
	return;
}

static struct i2c_pxa_platform_data i2c1_pdata = {
	.use_pio        = 0,
	/*
	 * charger ic do not support high mode on Ariel
	 */
	.flags		= PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
	.master_code	= (0x08 | 0x06), /*8 -highest, 0xF -lowest arbitration*/
	.i2c_bus_reset		= i2c_pxa_bus_reset,
};

static struct i2c_pxa_platform_data i2c2_pdata = {
	.use_pio	= 0,
	/*
	 * FT5x06 does not support FAST mode on Ariel
	 */
	.flags		= PXA_I2C_STANDARD_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
	.i2c_bus_reset		= i2c_pxa_bus_reset,
};

static struct i2c_pxa_platform_data i2c3_pdata = {
	.use_pio = 0,
	.flags = PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
	.i2c_bus_reset		= i2c_pxa_bus_reset,
};

static struct platform_device *devices[] __initdata = {
	&pxa95x_device_i2c1,
	&pxa95x_device_i2c2,
	&pxa95x_device_i2c3,
};

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)
static char *pxa9xx_usb_clock_name[] = {
	[0] = "AXICLK",
	[1] = "IMUCLK",
	[2] = "U2OCLK",
};
static struct mv_usb_addon_irq pm80x_id = {
	.irq	= IRQ_BOARD_START + PM800_IRQ_GPADC2,
	.poll	= pm80x_read_id_val,
	.init	= pm80x_init_id,
};

static struct mv_usb_addon_irq pm80x_vbus = {
	.irq	= IRQ_BOARD_START + PM800_IRQ_CHG,
	.poll	= pm80x_read_vbus_val,
};

static struct mv_usb_platform_data pxa978_usb_pdata = {
	.clknum		= 3,
	.clkname	= pxa9xx_usb_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_OTG,
	.otg_force_a_bus_req = 1,
	.phy_init	= pxa978_usb_phy_init,
	.phy_deinit	= pxa978_usb_phy_deinit,
	.set_vbus	= NULL,
	.otg_force_a_bus_req = 1,
};
#endif

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int matrix_key_maps[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_BACK),
	KEY(0, 1, KEY_END),
	KEY(0, 2, KEY_CAMERA),
	KEY(1, 0, KEY_OK),
	KEY(1, 1, KEY_HOME),
	KEY(1, 2, KEY_CAMERA),
	KEY(2, 0, KEY_MENU),
	KEY(2, 1, KEY_SEND),
	KEY(2, 2, KEY_RIGHT),/*not exist*/
	KEY(3, 0, KEY_VOLUMEUP),
	KEY(3, 1, KEY_VOLUMEDOWN),
	KEY(3, 2, KEY_DOWN),/*not exists*/
};

static struct pxa27x_keypad_platform_data keypad_info = {
	.matrix_key_rows = 4,
	.matrix_key_cols = 3,
	.matrix_key_map = matrix_key_maps,
	.matrix_key_map_size = ARRAY_SIZE(matrix_key_maps),
	.debounce_interval = 30,
	.active_low = 1,
};

int reset_keys_reset_fn(void)
{
	return 0;
}

int reset_keys_dump_fn(int pressed)
{
	static unsigned long trigger_time;
	if (pressed) {
		trigger_time = get_seconds();
		printk(KERN_EMERG "Panic key is pressed, please hold 5 ~ 10 "
				"seconds to dump system state, hold 10+ seconds to "
				"trigger a kernel panic!\n");
	} else {
		trigger_time = get_seconds() - trigger_time;
		if (trigger_time >= 10) {
			if (trigger_time < 25)
				panic("Panic key triggers panic!");
			else if (trigger_time < 40)
				BUG();
			else
				*(int *)0 = 0;
		} else if (trigger_time >= 5) {
			show_state();
			dump_stack();
		}
	}
	return 0;
}

static int reset_keys_up[] = {
	0
};

static struct keyreset_platform_data reset_keys_pdata = {
	.dump_fn	= reset_keys_dump_fn,
	.reset_fn	= reset_keys_reset_fn,
	.keys_up        = reset_keys_up,
	.keys_down      = {
		KEY_VOLUMEUP,
		KEY_VOLUMEDOWN,
		0
	}
};

static struct platform_device reset_keys_device = {
	.name           = KEYRESET_NAME,
	.dev            = {
		.platform_data = &reset_keys_pdata,
	},
};
#endif /* CONFIG_KEYBOARD_PXA27x || CONFIG_KEYBOARD_PXA27x_MODULE */

#include "board-ariel-camera.c"

static inline int pxa978_add_spi(int id, struct pxa2xx_spi_master *pdata)
{
	struct platform_device *pd;

	pd = platform_device_alloc("pxa2xx-spi", id);
	if (pd == NULL) {
		pr_err("pxa2xx-spi: failed to allocate device (id=%d)\n", id);
		return -ENOMEM;
	}

	platform_device_add_data(pd, pdata, sizeof(*pdata));

	return platform_device_add(pd);
}

#if (defined CONFIG_CMMB)

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 1,
	.enable_dma = 1,
};

static int cmmb_power_reset(void)
{
	int cmmb_rst;

	cmmb_rst = mfp_to_gpio(MFP_PIN_GPIO17);

	if (gpio_request(cmmb_rst, "cmmb rst")) {
		pr_warning("failed to request GPIO for CMMB RST\n");
		return -EIO;
	}

	/* reset cmmb, keep low for about 1ms */
	gpio_direction_output(cmmb_rst, 0);
	msleep(100);

	/* get cmmb go out of reset state */
	gpio_direction_output(cmmb_rst, 1);
	gpio_free(cmmb_rst);

	return 0;
}

static int cmmb_regulator(bool en);
static int cmmb_power_on(void)
{
	int cmmb_en;

	cmmb_regulator(1);

	cmmb_en = mfp_to_gpio(MFP_PIN_GPIO16);
	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("[ERROR] failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}
	gpio_direction_output(cmmb_en, 0);
	msleep(100);

	gpio_direction_output(cmmb_en, 1);
	gpio_free(cmmb_en);

	msleep(100);

	cmmb_power_reset();

	return 0;
}

static int cmmb_power_off(void)
{
	int cmmb_en;

	cmmb_en = mfp_to_gpio(MFP_PIN_GPIO16);

	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}

	gpio_direction_output(cmmb_en, 0);
	gpio_free(cmmb_en);
	msleep(100);

	cmmb_regulator(0);

	return 0;
}
/*.
 ** Add two functions: cmmb_cs_assert and cmmb_cs_deassert.
 ** Provide the capbility that
 ** cmmb driver can handle the SPI_CS by itself.
 **/
static int cmmb_cs_assert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO88_SSP2_FRM);
	gpio_direction_output(cs, 0);
	return 0;
}

static int cmmb_cs_deassert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO88_SSP2_FRM);
	gpio_direction_output(cs, 1);
	return 0;
}

static int cmmb_regulator(bool en)
{

	struct regulator *v_ldo1v2, *v_ldo1v8;
	int regulator_enabled_1v2, regulator_enabled_1v8;

	v_ldo1v2 = regulator_get(NULL, "Vdd_CMMB12");	/* should be updated! */

	if (IS_ERR(v_ldo1v2)) {
		printk(KERN_ERR "cmmb: fail to get ldo1v2 handle!\n");
		return -EINVAL;
	}

	v_ldo1v8 = regulator_get(NULL, "Vdd_IO");	/* should be updated! */

	if (IS_ERR(v_ldo1v8)) {
		printk(KERN_ERR "cmmb: fail to get ldo1v8 handle!\n");
		regulator_put(v_ldo1v2);
		return -EINVAL;
	}

	if (en) {
		regulator_set_voltage(v_ldo1v2, 1200000, 1200000);
		regulator_set_voltage(v_ldo1v8, 1800000, 1800000);
		regulator_enable(v_ldo1v2);
		regulator_enable(v_ldo1v8);
		printk(KERN_INFO "cmmb: turn on ldo1v2 and ldo1v8 to high.\n");
	} else {
		regulator_enabled_1v2 = regulator_is_enabled(v_ldo1v2);
		regulator_enabled_1v8 = regulator_is_enabled(v_ldo1v8);

		if ((regulator_enabled_1v2) && (regulator_enabled_1v8)) {
			printk(KERN_INFO "cmmb: turn off LDO\n");
			regulator_disable(v_ldo1v2);
			regulator_disable(v_ldo1v8);
		}
	}

	regulator_put(v_ldo1v2);
	regulator_put(v_ldo1v8);
	return 0;
}

static struct cmmb_platform_data cmmb_info = {
	.power_on = cmmb_power_on,
	.power_off = cmmb_power_off,
	.power_reset = cmmb_power_reset,
	.cs_assert = cmmb_cs_assert,
	.cs_deassert = cmmb_cs_deassert,

	.cmmb_regulator = cmmb_regulator,

	.gpio_power = mfp_to_gpio(MFP_PIN_GPIO16),
	.gpio_reset = mfp_to_gpio(MFP_PIN_GPIO17),
	.gpio_cs = mfp_to_gpio(GPIO88_SSP2_FRM),
	.gpio_defined = 1,
};

static void cmmb_if101_cs(u32 cmd)
{
/* Because in CMMB read/write,the max data size is more than 8kB
 * 8k = max data length per dma transfer for pxaxxx
 * But till now,The spi_read/write driver doesn't support muti DMA cycles
 *
 * Here the spi_read/write will not affect the SPI_CS,but provides
 * cs_assert and cs_deassert in the struct cmmb_platform_data
 *
 * And cmmb driver can/should control SPI_CS by itself
 */
}

static struct pxa2xx_spi_chip cmmb_if101_chip = {
	.rx_threshold   = 1,
	.tx_threshold   = 1,
	.cs_control     = cmmb_if101_cs,
};

/* bus_num must match id in pxa2xx_set_spi_info() call */
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "cmmb_if",
		.platform_data	= &cmmb_info,
		.controller_data	= &cmmb_if101_chip,
		.irq			= gpio_to_irq(mfp_to_gpio(GPIO93_GPIO)),
		.max_speed_hz	= 8000000,
		.bus_num		= 2,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
};

static void __init nevo_init_spi(void)
{
	int err;
	int cmmb_int, cmmb_cs;

	cmmb_cs = mfp_to_gpio(GPIO88_SSP2_FRM);
	err = gpio_request(cmmb_cs, "cmmb cs");
	if (err) {
		pr_warning(KERN_ERR "failed to request GPIO for CMMB CS\n");
		return;
	}
	gpio_direction_output(cmmb_cs, 1);

	cmmb_int = mfp_to_gpio(GPIO93_GPIO);

	err = gpio_request(cmmb_int, "cmmb irq");
	if (err) {
		pr_warning(KERN_ERR "failed to request GPIO for CMMB IRQ\n");
		return;
	}
	gpio_direction_input(cmmb_int);

	pxa978_add_spi(2, &pxa_ssp_master_info);
	if (spi_register_board_info(spi_board_info,
			ARRAY_SIZE(spi_board_info))) {
		pr_warning("[ERROR] failed to register spi device.\n");
		printk(KERN_ERR "failed to register spi device.\n");
		return;
	}
}

#endif /*defined CONFIG_CMMB*/

#if defined(CONFIG_FB_PXA95x)

#if defined(CONFIG_MV_IHDMI)
#ifndef CONFIG_UIO_HDMI
static int mv_ihdmi_format = 4;
#endif
static int hdtx_en;
static int hdtx_power(int en)
{
	struct regulator *v_ldo;
	int pin1 = mfp_to_gpio(MFP_PIN_GPIO73);
	if (hdtx_en == en) {
		printk("hdmi: already turn %s\n", en?"on":"off");
		return 0;
	}
	v_ldo = regulator_get(NULL, "v_ihdmi");
	if (IS_ERR(v_ldo)) {
		printk(KERN_ERR "hdmi: fail to get ldo handle!\n");
		return -EINVAL;
	}

	if (en) {
		regulator_set_voltage(v_ldo, 3300000, 3300000);
		regulator_enable(v_ldo);
		printk(KERN_INFO "hdmi: turn on hdmi-term to 3.3v.\n");
	} else {
		printk(KERN_INFO "hdmi: turn off hdmi-term\n");
		regulator_disable(v_ldo);
	}

	if (gpio_request(pin1, "hdmi-cp")) {
		printk(KERN_ERR "hdmi: hdmi-cp failed!\n");
		gpio_free(pin1);
		regulator_put(v_ldo);
		return -EINVAL;
	}

	gpio_direction_output(pin1, en);
	msleep(10);

	gpio_free(pin1);
	printk(KERN_INFO "hdmi: turn %s charge pump 5V\n",
		en ? "ON" : "OFF");

	regulator_put(v_ldo);
	hdtx_en = en;
	return 0;
}

#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data hdtx_uio_data = {
	.sspa_reg_base = 0x41900000,
	.itlc_reg_base = 0x4410c000,
	.gpio = MFP_PIN_GPIO112,
	.hdmi_v5p_power = hdtx_power,
	.edid_bus_num = 3,	/* edid on i2c-3*/
	.hpd_val = 0x10000,	/* when plug in, its value is 1<<16 */
};

static void __init init_hdmi(void)
{
	mfp_cfg_t hpd_pin = GPIO112_HDMI_HPD;
	mfp_config(&hpd_pin, 1);
	pxa_register_device(&pxa978_device_uio_ihdmi, &hdtx_uio_data);
}
#endif

static struct pxa95xfb_mach_info ihdmi_base_info __initdata = {
	.id                     = "HDMI-Base",
	.modes                  = video_modes_ihdmi,
	.num_modes              = ARRAY_SIZE(video_modes_ihdmi),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_TV_HDMI,
	.window                 = 4,
	.mixer_id               = 2,
	.zorder                 = 0,
	.converter              = LCD_M2HDMI,
	.output                 = OUTPUT_HDMI,
	.active                 = 1,
#ifndef CONFIG_UIO_HDMI
	.panel_power            = hdtx_power,
#endif
	.invert_pixclock        = 1,
	.init_mode              = 15,
};

static struct pxa95xfb_mach_info ihdmi_ovly_info __initdata = {
	.id                     = "HDMI-Ovly",
	.modes 	                = video_modes_ihdmi,
	.num_modes              = ARRAY_SIZE(video_modes_ihdmi),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_TV_HDMI,
	.window			= 4,
	.mixer_id		= 2,
	.zorder			= 0,
	.converter		= LCD_M2HDMI,
	.output			= OUTPUT_HDMI,
	.active			= 1,
#ifndef CONFIG_UIO_HDMI
	.panel_power			= hdtx_power,
#endif
	.invert_pixclock	= 1,
	.init_mode 		= 15,
};
#endif

static void __init init_lcd(void)
{
#ifdef CONFIG_LCD_BYD_9009AD
	pxa95x_add_lcd_byd();
#endif
#if defined(CONFIG_MV_IHDMI)
#ifdef CONFIG_UIO_HDMI
	init_hdmi();
#else
	pxa_register_device(&pxa978_device_ihdmi, &mv_ihdmi_format);
#endif
	set_pxa95x_fb_ovly_info(&ihdmi_base_info, 1);
	set_pxa95x_fb_ovly_info(&ihdmi_ovly_info, 2);
#endif
}
#endif

static mfp_cfg_t pxa95x_hsl_mfp_cfg[] = {
	/* HSL for PXA978 */
	MFP116_HSL_CLK,
	MFP117_HSL_DATA0,
	MFP118_HSL_DATA1,
	MFP119_HSL_DATA2,
	MFP120_HSL_DATA3,
	MFP121_HSL_DATA4
};

static void abu_mfp_init(bool abu)
{
	int i;
	unsigned long mfpr, af;

	af = abu ? 2 : 1;

	for (i = 63; i < 68; i++) {
		mfpr = mfp_read(i);
		mfpr &= ~MFPR_AF_MASK;
		mfpr |= MFPR_AF_SEL(af);
		mfp_write(i, mfpr);
	}
}

static void ssp3_mfp_init(bool bssp)
{
	int i;
	unsigned long mfpr, af;

	af = bssp ? 1 : 2;

	for (i = 58; i < 62; i++) {
		mfpr = mfp_read(i);
		mfpr &= ~MFPR_AF_MASK;
		mfpr |= MFPR_AF_SEL(af);
		mfp_write(i, mfpr);
	}
}

#ifdef CONFIG_PM
static int init_idle_wakeup(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.rtc = 1;
	src->bits.ost = 1;
#ifdef CONFIG_PXA9XX_ACIPC
	src->bits.msl = 1;
#endif
	src->bits.ext0 = 1;
	src->bits.uart1 = 1;
	src->bits.mkey = 1;
	src->bits.eth = 1;
	src->bits.tsi = 1;
	src->bits.cmwdt = 1;
	src->bits.mmc1_cd = 1;
	src->bits.mmc3_dat1 = 1;

	if (cpu_is_pxa978_Dx())
		src->bits.display = 1;

	return 0;
}

static int init_suspend_wakeup(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.mmc3_dat1 = 1;	/* MMC3 Data1 */
	src->bits.rtc = 1;		/* RTC */
	src->bits.msl = 1;		/* ACS-IPC */
	src->bits.tsi = 1;		/* On-key */

	return 0;
}

u32 *sd_normal_int_err_status0;
u32 *sd_normal_int_err_status1;
u32 *sd_normal_int_err_status2;
#include <mach/regs-intc.h>
static int is_wkr_nevo_2243(void)
{
	return cpu_is_pxa978();
}

static int query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
{
	int is_mmc = 0;
	memset(src, 0, sizeof(pm_wakeup_src_t));
	if (reg & PXA95x_PM_WE_RTC)
		src->bits.rtc = 1;
	if (reg & PXA95x_PM_WE_OST)
		src->bits.ost = 1;
	if (reg & PXA95x_PM_WE_MSL0)
		src->bits.msl = 1;
	if (reg & PXA95x_PM_WE_EXTERNAL0)
		src->bits.ext0 = 1;
	if (reg & PXA95x_PM_WE_KP)
		src->bits.mkey = 1;
	if (reg & PXA95x_PM_WE_MMC3) {
		src->bits.mmc3_dat1 = 1;
		if (is_wkr_nevo_2243())
			is_mmc = 1;
	}
	if (reg & PXA95x_PM_WE_GENERIC(3))
		src->bits.tsi = 1;
	if (reg & PXA95x_PM_WE_GENERIC(9))
		src->bits.uart1 = 1;
	if (reg & PXA95x_PM_WE_GENERIC(2))
		src->bits.uart2 = 1;
	if (reg & PXA95x_PM_WE_GENERIC(12))
		src->bits.cmwdt = 1;
	if (reg & PXA95x_PM_WE_GENERIC(13)) {
		if (pxa95x_query_gwsr(97))
			src->bits.eth = 1;
		/*if (pxa95x_query_gwsr(53))
			src->bits.uart1 = 1;
		if (pxa95x_query_gwsr(123))
			src->bits.mmc1_cd = 1;*/
	}
	if (is_wkr_nevo_2243()) {
		/* Clear unexpected irq. */
		/* MMC0 interrupt pending. */
		if (ICIP3 & (1 << (72 - 64)))
			writel(0xffffffff, sd_normal_int_err_status0);

		/* MMC1 interrupt pending. */
		if (ICIP3 & (1 << (73 - 64)))
			writel(0xffffffff, sd_normal_int_err_status1);

		/* MMC2 interrupt pending. */
		if (ICIP3 & (1 << (74 - 64)))
			writel(0xfffffeff, sd_normal_int_err_status2);

		if (!is_mmc)
			writel(0x100, sd_normal_int_err_status2);
	}

	return 0;
}

static int ext_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.ext0)
			ret |= PXA95x_PM_WE_EXTERNAL0;
		if (src.bits.ext1)
			ret |= PXA95x_PM_WE_EXTERNAL1;
	}
	return ret;
}

static int key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	unsigned int i = 0;
	unsigned int key_matrix = 4;
	/*GPIO0, GPIO2, GPIO4, GPIO6,*/
	if (enable) {
		if (src.bits.mkey) {
			for (i = 0; i < key_matrix; i++)
				lpm_mfpr_edge_config(MFP_PIN_GPIO0 + i*2, MFP_LPM_EDGE_BOTH);
			ret |= PXA95x_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			for (i = 0; i < key_matrix; i++)
				lpm_mfpr_edge_config(MFP_PIN_GPIO0 + i*2, MFP_LPM_EDGE_NONE);
		}
	}
	return ret;
}

static int mmc1_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	/*if (enable) {
		if (src.bits.mmc1_cd) {
			lpm_mfpr_edge_config(MFP_PIN_GPIO123, MFP_LPM_EDGE_BOTH);
			ret |= PXA95x_PM_WE_GENERIC(13);
		}
	} else
		lpm_mfpr_edge_config(MFP_PIN_GPIO123, MFP_LPM_EDGE_NONE);*/
	return ret;
}

static int mmc3_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mmc3_dat1) {
			/*GPIO80_MMC3_DAT1*/
			lpm_mfpr_edge_config(MFP_PIN_GPIO80, MFP_LPM_EDGE_BOTH);
			ret |= PXA95x_PM_WE_MMC3;
		}
	} else
		lpm_mfpr_edge_config(MFP_PIN_GPIO80, MFP_LPM_EDGE_NONE);

	return ret;
}

static int uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;

	if (enable) {
		if (src.bits.uart1) {
			lpm_mfpr_edge_config(MFP_PIN_GPIO131, MFP_LPM_EDGE_FALL);
			ret |= PXA95x_PM_WE_GENERIC(9);
		}
		if (src.bits.uart2) {
			lpm_mfpr_edge_config(MFP_PIN_GPIO94, MFP_LPM_EDGE_FALL);
			/* note: on pxa930, uart2 use this bit */
			ret |= PXA95x_PM_WE_GENERIC(2);
		}
	} else {
		if (src.bits.uart1)
			lpm_mfpr_edge_config(MFP_PIN_GPIO131, MFP_LPM_EDGE_NONE);
		if (src.bits.uart2)
			lpm_mfpr_edge_config(MFP_PIN_GPIO94, MFP_LPM_EDGE_NONE);
	}
	return ret;
}

static int tsi_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.tsi) {
			lpm_mfpr_edge_config(MFP_PIN_PMIC_INT, MFP_LPM_EDGE_FALL);
			ret |= PXA95x_PM_WE_GENERIC(3);
		}
	} else {
		if (src.bits.tsi)
			lpm_mfpr_edge_config(MFP_PIN_PMIC_INT, MFP_LPM_EDGE_NONE);
	}
	return ret;
}

static int comm_wdt_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.cmwdt)
			ret |= PXA95x_PM_WE_GENERIC(12);
	}
	return ret;
}

static int display_wakeup(pm_wakeup_src_t src, int enable)
{
	return 0;
}

static struct pxa95x_peripheral_wakeup_ops wakeup_ops = {
	.init_idle_wakeup   = init_idle_wakeup,
	.init_suspend_wakeup   = init_suspend_wakeup,
	.query  = query_wakeup,
	.ext    = ext_wakeup,
	.key    = key_wakeup,
	.mmc1    = mmc1_wakeup,
	.mmc3    = mmc3_wakeup,
	.uart   = uart_wakeup,
	.tsi    = tsi_wakeup,
	.cmwdt  = comm_wdt_wakeup,
	.display  = display_wakeup,
};
#endif

static void pull_up_dcdc(void)
{
	int dcdc_pin;
	int err;

	dcdc_pin = mfp_to_gpio(MFP_PIN_RF_MFP14);
	err = gpio_request(dcdc_pin, "DCDC_Pin");
	if (err) {
		gpio_free(dcdc_pin);
		printk(KERN_ERR "Request GPIO failed, gpio: %d return :%d\n",
			dcdc_pin, err);
		return;
	}
	gpio_direction_output(dcdc_pin, 1);

	gpio_free(dcdc_pin);
	pr_info("pull up DCDC. \n");

	return;
}

#ifdef CONFIG_PROC_FS
static void sparrow_rf_reset(void)
{
	int dcdc_pin, rf_reset;
	int err;

	rf_reset = mfp_to_gpio(MFP_PIN_GPIO108);
	err = gpio_request(rf_reset, "RF_Reset");
	if (err) {
		gpio_free(rf_reset);
		printk(KERN_ERR "Request GPIO failed, gpio: %d return :%d\n",
			rf_reset, err);
		return;
	}

	dcdc_pin = mfp_to_gpio(MFP_PIN_RF_MFP14);
	err = gpio_request(dcdc_pin, "DCDC_Pin");
	if (err) {
		gpio_free(rf_reset);
		gpio_free(dcdc_pin);
		printk(KERN_ERR "Request GPIO failed, gpio: %d return :%d\n",
			dcdc_pin, err);
		return;
	}
	gpio_direction_output(dcdc_pin, 1);
	mdelay(1);
	gpio_direction_output(rf_reset, 0);
	mdelay(2);
	gpio_direction_output(rf_reset, 1);

	gpio_free(rf_reset);
	gpio_free(dcdc_pin);

	pr_info("reset sparrow rf\n");

	return;
}

static ssize_t sparrow_rf_write_proc(struct file *filp,
				const char *buff, size_t len, loff_t *off)
{
	sparrow_rf_reset();
	return len;
}

static void create_sparrow_rf_proc_file(void)
{
	struct proc_dir_entry *proc_file = NULL;

	proc_file = create_proc_entry("driver/sparrow_rf_reset", 0644, NULL);
	if (!proc_file) {
		pr_err("%s: create proc file failed\n", __func__);
		return;
	}

	proc_file->write_proc = (write_proc_t *)sparrow_rf_write_proc;
}

static void hsl_mfpr_set(void)
{
	mfp_config(ARRAY_AND_SIZE(pxa95x_hsl_mfp_cfg));
	pr_info("the MFP for MicroSD is switched to HSP for IML.\n");
}

static ssize_t hsl_mfpr_write_proc(struct file *filp,
				const char *buff, size_t len, loff_t *off)
{
	hsl_mfpr_set();
	return len;
}

static void create_hsl_mfpr_proc_file(void)
{
	struct proc_dir_entry *proc_file = NULL;

	proc_file = create_proc_entry("driver/hsl_mfpr", 0644, NULL);
	if (!proc_file) {
		pr_err("%s: create proc file failed\n", __func__);
		return;
	}

	proc_file->write_proc = (write_proc_t *)hsl_mfpr_write_proc;

}

static ssize_t pcm_mfp_write_proc(struct file *filp,
				const char *buff, size_t len, loff_t *off)
{
	char a;

	if (copy_from_user(&a, buff, 1))
		return -EINVAL;
	switch (a) {
	case '0':
		{
			unsigned long mfp = GPIO60_GPIO;
			unsigned int gpio = MFP_PIN_GPIO60;
			mfp_config(&mfp, 1);
			gpio_request(gpio, NULL);
			gpio_direction_input(gpio);
			gpio_free(gpio);
			printk(KERN_INFO "Switch MFP60 to GPIO input\n");
		break;
		}
	case '1':
		ssp3_mfp_init(false);
		printk(KERN_INFO "PCM port is switched to GSSP1\n");
		break;
	case '2':
		ssp3_mfp_init(true);
		printk(KERN_INFO "PCM port is switched to BSSP3\n");
		break;
	default:
		printk(KERN_INFO "[PCM_MFP] Error: invalid configuration\n");
		break;
	}
	return len;
}

static void create_pcm_mfp_proc_file(void)
{
	struct proc_dir_entry *proc_file = NULL;

	proc_file = create_proc_entry("driver/pcm_mfp", 0644, NULL);
	if (!proc_file) {
		pr_err("%s: create proc file failed\n", __func__);
		return;
	}

	proc_file->write_proc = (write_proc_t *)pcm_mfp_write_proc;

}

#endif

static int reboot_notifier_func(struct notifier_block *this,
		unsigned long code, void *cmd)
{
	int reg = (PM80X_BASE_PAGE << 8) | 0xef;

	if (cmd != NULL) {
		if (0 == strcmp(cmd, "recovery")) {
			pr_info("Enter recovery mode\n");
			pm80x_codec_reg_write(reg, 0x1);
		} else if (0 == strcmp(cmd, "autoflash")) {
			pr_info("Enter autoflash mode\n");
			pm80x_codec_reg_write(reg, 0x2);
		} else
			pm80x_codec_reg_write(reg, 0x0);
	} else
		pm80x_codec_reg_write(reg, 0x0);

	return 0;
}
static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_notifier_func,
};

static void __init init(void)
{
	/* init procidia */
	regulator_init_pm800();
	pr_info( \
		"[%s][%s]regulator_init_pm800 maxNum[%d] init\n",
		__FILE__, __func__, PM8XXX_REGULATOR_MAX);
	pxa978_usb_pdata.vbus = &pm80x_vbus;
	pxa978_usb_pdata.id = &pm80x_id;
	pxa978_usb_pdata.set_vbus = pm80x_set_vbus;

	register_reboot_notifier(&reboot_notifier);

	headsetflag_init_pm800();

	set_abu_init_func(abu_mfp_init);

	platform_device_add_data(&pxa95x_device_i2c1, &i2c1_pdata,
				 sizeof(i2c1_pdata));
	platform_device_add_data(&pxa95x_device_i2c2, &i2c2_pdata,
				 sizeof(i2c2_pdata));
	platform_device_add_data(&pxa95x_device_i2c3, &i2c3_pdata,
				 sizeof(i2c3_pdata));

	pxa95x_device_vMeta_devfreq.dev.platform_data =
				(void *)(&devfreq_vmeta_pdata);

	platform_add_devices(ARRAY_AND_SIZE(devices));
	register_i2c_board_info();

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
	pxa_set_keypad_info(&keypad_info);
	platform_device_register(&reset_keys_device);
#endif

	platform_device_register(&ariel_leds_gpio);

	init_cam();

#if defined(CONFIG_FB_PXA95x)
	init_lcd();
#endif
#if defined(CONFIG_LEDS_LM3530)
	lm3530_enable();
#endif

#if defined(CONFIG_MMC_SDHCI_PXAV2_TAVOR) || defined(CONFIG_MMC_SDHCI_PXAV3)
	init_mmc();
	if (is_wkr_nevo_2243()) {
		sd_normal_int_err_status0 = ioremap(0x55000030, 0x4);
		sd_normal_int_err_status1 = ioremap(0x55100030, 0x4);
		sd_normal_int_err_status2 = ioremap(0x55200030, 0x4);
	}

	wake_lock_init(&wifi_delayed_work_wake_lock,
					WAKE_LOCK_SUSPEND, "wifi_delayed_work");
#endif

	/* Init boot flash - sync mode in case of ONENAND */
	pxa_boot_flash_init(1);

#ifdef CONFIG_PM
	pxa95x_wakeup_register(&wakeup_ops);
#endif

#if (defined CONFIG_CMMB)
	/*spi device*/
	nevo_init_spi();
#endif

#ifdef CONFIG_USB_PXA_U2O
	pxa978_device_u2o.dev.platform_data = (void *)&pxa978_usb_pdata;
	platform_device_register(&pxa978_device_u2o);
#endif

#ifdef CONFIG_USB_PXA_U2O_OTG
	pxa978_device_u2ootg.dev.platform_data = (void *)&pxa978_usb_pdata;
	platform_device_register(&pxa978_device_u2ootg);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2O
	pxa978_device_u2oehci.dev.platform_data = (void *)&pxa978_usb_pdata;
	platform_device_register(&pxa978_device_u2oehci);
#endif

#ifdef CONFIG_PROC_FS
	create_sparrow_rf_proc_file();
	create_hsl_mfpr_proc_file();
	create_sirf_proc_file();
	create_pcm_mfp_proc_file();
#endif
	init_rfreset_gpio();

	pull_up_dcdc();
}

MACHINE_START(NEVOARIEL, "PXA978")
	.map_io		= pxa_map_io,
	.nr_irqs	= BOARD_NR_IRQS,
	.init_irq	= pxa95x_init_irq,
	.handle_irq	= pxa95x_handle_irq_intc,
	.timer		= &pxa_timer,
	.reserve	= pxa95x_mem_reserve,
	.init_machine	= init,
MACHINE_END

extern int mspm_idle_load(void);
static int __init board_pm_init(void)
{
	printk(KERN_INFO "Enable cpufreq and idle profiler.\n");
	cur_profiler = CPUFREQ_PROFILER;
	mspm_idle_load();
	return 0;
}

late_initcall(board_pm_init);
