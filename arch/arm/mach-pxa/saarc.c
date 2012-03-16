/*
 *  Copyright (C) 2007-2010 Marvell International Ltd.
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
#include <linux/mfd/88pm860x.h>
#include <linux/mfd/88pm80x.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/cwmi.h>
#include <linux/cwgd.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/regulator/machine.h>
#include <linux/sd8x_rfkill.h>
#include <linux/mmc/sdhci.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/i2c/adp8885.h>
#include <linux/proc_fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa930.h>
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
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/cmmb.h>

#include <plat/usb.h>

#include "devices.h"
#include "generic.h"

#include "panel_settings.h"
#ifdef CONFIG_RTC_DRV_PXA
#include <linux/rtc-pxa.h>
#endif


#define NEVOSAARC_NR_IRQS	(IRQ_BOARD_START + 40)

static struct pm860x_backlight_pdata backlight[] = {
	{
		/*backlight data*/
		.id	= PM8606_ID_BACKLIGHT,
		.iset = PM8606_WLED_CURRENT(0x12),
		.flags	= PM8606_BACKLIGHT1,
	},
	{
		/*keypad backlight data*/
		.id	= PM8606_ID_BACKLIGHT,
		.pwm = 0x10,
		.iset = PM8606_WLED_CURRENT(0x05),
		.flags	= PM8606_BACKLIGHT2,
	},
};

static struct pm860x_led_pdata led[] = {
	{
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_BLUE,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_BLUE,
	},
};

static void disable_rf(void)
{
}

static struct pm860x_power_pdata power = {
	.disable_rf_fn  = disable_rf,
};

static void init_rfreset_gpio(void)
{
	update_rfreset_gpio_num(MFP_PIN_GPIO112);
}
#if defined(CONFIG_SENSORS_CM3601)
static int cm3601_request_resource(unsigned char gpio_num, char *name)
{
	int ret = 0;
	ret = gpio_request(mfp_to_gpio(gpio_num), name);
	if (ret) {
			printk(KERN_ERR "%s: can't request GPIO %d.\n", __func__, gpio_num);
			return -1;
	}
	return ret;
}

static void cm3601_release_resource(unsigned char gpio_num)
{
	gpio_free(gpio_num);
}

static struct pm860x_cm3601_pdata cm3601_platform_info = {
	.gpio_cm3601_enable =   MFP_PIN_GPIO133,
	.gpio_ps_output =       MFP_PIN_GPIO135,
	.gpio_ps_enable =       MFP_PIN_GPIO134,
	.request_source =       cm3601_request_resource,
	.release_source =       cm3601_release_resource,
};
#endif

static struct pm860x_headset_pdata headset_platform_info	 = {
	.headset_flag = 0,
	/* headset switch */
	.headset_data[0].name = "h2w",
	.headset_data[0].gpio = 0,
	.headset_data[0].name_on = NULL,
	.headset_data[0].name_off = NULL,
	.headset_data[0].state_on = NULL,
	.headset_data[0].state_off = NULL,
	/* hook switch */
	.headset_data[1].name = "h3w",
	.headset_data[1].gpio = 0,
	.headset_data[1].name_on = NULL,
	.headset_data[1].name_off = NULL,
	.headset_data[1].state_on = NULL,
	.headset_data[1].state_off = NULL,
};


/* TODO: check the regulator data carefully */
#define PM8XXX_REGULATOR_MAX ((PM8607_ID_RG_MAX > PM800_ID_RG_MAX) ? \
		PM8607_ID_RG_MAX : PM800_ID_RG_MAX)

static struct regulator_consumer_supply regulator_supply[PM8XXX_REGULATOR_MAX];
static struct regulator_init_data regulator_data[PM8XXX_REGULATOR_MAX];

static int PM8607_ID_regulator_index[] = {
	PM8607_ID_BUCK1,
	PM8607_ID_BUCK2,
	PM8607_ID_BUCK3,
	PM8607_ID_LDO1,
	PM8607_ID_LDO2,
	PM8607_ID_LDO3,
	PM8607_ID_LDO4,
	PM8607_ID_LDO5,
	PM8607_ID_LDO6,
	PM8607_ID_LDO7,
	PM8607_ID_LDO8,
	PM8607_ID_LDO9,
	PM8607_ID_LDO10,
	PM8607_ID_LDO11,
	PM8607_ID_LDO12,
	PM8607_ID_LDO13,
	PM8607_ID_LDO14,
	PM8607_ID_LDO15,
};

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

#define REG_INIT(_id, _chip, _name, _min, _max, _always, _boot)\
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
}

static struct pm860x_rtc_pdata rtc = {
	.vrtc           = 1,
	.rtc_wakeup     = 0,
#ifdef CONFIG_RTC_DRV_PXA
	.sync		= pxa_rtc_sync_time,
#endif
};

static struct pm80x_rtc_pdata pm80x_rtc = {
	.vrtc           = 1,
	.rtc_wakeup     = 0,
#ifdef CONFIG_RTC_DRV_PXA
	.sync		= pxa_rtc_sync_time,
#endif
};

static struct pm860x_platform_data pm8607_info = {
	.backlight	= &backlight[0],
	.led		= &led[0],
	.power		= &power,
	.regulator	= regulator_data,
#if defined(CONFIG_SENSORS_CM3601)
	.cm3601		= &cm3601_platform_info,
#endif
	.headset = &headset_platform_info,
	.rtc		= &rtc,
	.companion_addr	= 0x10,

	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,

	.i2c_port	= PI2C_PORT,

	.num_leds	= ARRAY_SIZE(led),
	.num_backlights	= ARRAY_SIZE(backlight),
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
	pm80x_reg_write(chip->base_page, 0x0E, 0x00);
	/* Enable 32Khz-out-1 and resetoutn */
	pm80x_reg_write(chip->base_page, 0xE1, 0xB0);
	/* Enable 32Khz-out-3  low jitter */
	pm80x_reg_write(chip->base_page, 0x21, 0x20);
	/* Enable 32Khz-out-3 */
	pm80x_reg_write(chip->base_page, 0xE2, 0x22);

	return 0;
}

static struct pm80x_headset_pdata pm80x_headset = {
	.gpio	= 4,			/* GPIO 4 */
	.gpio_ctl = 0x32,		/* PM800 GPIO 4 */
	.gpio_enable_irq = (1 << 4),
	.gpio_set_mask = 0xff,
	.gpio_set_val = 0x08,
	.gpio_val_bit = (1 << 0),
};

static struct pm80x_platform_data pm800_info = {
	.headset = &pm80x_headset,
	.regulator	= regulator_data,
	.rtc  = &pm80x_rtc,
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
	switch (get_board_id()) {
	case OBM_DKB_2_NEVO_C0_BOARD:
		pm800_info.headset_flag = 1;
		break;
	default:
		pm800_info.headset_flag = 0;
		break;
	}
}
static void regulator_init_pm8607(void)
{
	int i = 0;

	if (PXA95x_USE_POWER_I2C != freq_mach_info.flags) {
		REG_SUPPLY_INIT(PM8607_ID_BUCK1, "v_buck1", "pxa95x-freq");
		REG_INIT(i++, PM8607_ID, BUCK1, 1000000, 1500000, 1, 1);
	}

	/* for hdmi */
	REG_SUPPLY_INIT(PM8607_ID_LDO12, "v_ihdmi", NULL);
	REG_INIT(i++, PM8607_ID, LDO12, 1200000, 3300000, 0, 0);

	switch (get_board_id()) {

	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		REG_SUPPLY_INIT(PM8607_ID_LDO1, "v_gps", NULL);
		REG_SUPPLY_INIT(PM8607_ID_LDO9, "v_cam", NULL);
		REG_SUPPLY_INIT(PM8607_ID_LDO13, "vmmc", "sdhci-pxa.1");
		REG_SUPPLY_INIT(PM8607_ID_LDO10, "v_cywee", NULL);

		REG_INIT(i++, PM8607_ID, LDO1, 1200000, 3300000, 0, 0);
		REG_INIT(i++, PM8607_ID, LDO9, 1800000, 3300000, 0, 0);
		REG_INIT(i++, PM8607_ID, LDO13, 1800000, 3300000, 0, 0);
		REG_INIT(i++, PM8607_ID, LDO10, 2800000, 2800000, 0, 0);
		printk(KERN_INFO "%s: select saarC NEVO ldo map\n", __func__);
	break;

	default:
		printk(KERN_ERR "%s: The board type is not defined!\n ", __func__);
		BUG();
	}

	pm8607_info.num_regulators = i;
}

static void regulator_init_pm800(void)
{
	int i = 0;

	REG_SUPPLY_INIT(PM800_ID_LDO18, "v_gps", NULL);
	REG_SUPPLY_INIT(PM800_ID_LDO16, "v_cam", NULL);
	REG_SUPPLY_INIT(PM800_ID_LDO13, "vmmc", "sdhci-pxa.1");
	REG_SUPPLY_INIT(PM800_ID_LDO8, "v_lcd", NULL);
	REG_SUPPLY_INIT(PM800_ID_LDO6, "v_ihdmi", NULL);
	/*
	for NFC integration - adding 3 regulators:
	 * Vdd_IO
	 * VBat
	 * VSim
	 since they are needed by NFC driver. to prevent harm to the system
	 i'm setting them to "unused" LDOs
	*/
	REG_SUPPLY_INIT(PM800_ID_LDO14, "Vdd_IO", NULL);
	REG_SUPPLY_INIT(PM800_ID_LDO15, "VBat", NULL);
	REG_SUPPLY_INIT(PM800_ID_LDO11, "v_cywee", NULL);

	REG_INIT(i++, PM800_ID, LDO18, 1200000, 3300000, 0, 0);
	REG_INIT(i++, PM800_ID, LDO16, 1800000, 3300000, 0, 0);
	REG_INIT(i++, PM800_ID, LDO13, 1800000, 3300000, 0, 0);
	REG_INIT(i++, PM800_ID, LDO8, 1800000, 3300000, 1, 1);
	REG_INIT(i++, PM800_ID, LDO6, 1200000, 3300000, 0, 0);
	REG_INIT(i++, PM800_ID, LDO14, 1800000, 3300000, 0, 0);
	REG_INIT(i++, PM800_ID, LDO15, 1800000, 3300000, 0, 0);

	REG_INIT(i++, PM800_ID, LDO11, 2800000, 2800000, 0, 0);

	switch (get_board_id()) {
	case OBM_DKB_2_NEVO_C0_BOARD:
		/* Turn on LDO12 and 17 before wifi regulator support added */
		REG_SUPPLY_INIT(PM800_ID_LDO2, "Vdd_CMMB12", NULL);
		REG_SUPPLY_INIT(PM800_ID_LDO9, "v_vibrator", NULL);
		REG_SUPPLY_INIT(PM800_ID_LDO12, "v_wifi_1v8", NULL);
		REG_SUPPLY_INIT(PM800_ID_LDO17, "v_wifi_3v3", NULL);
		REG_INIT(i++, PM800_ID, LDO2, 1200000, 3300000, 0, 0);
		REG_INIT(i++, PM800_ID, LDO9, 2800000, 2800000, 0, 0);
		REG_INIT(i++, PM800_ID, LDO12, 1800000, 1800000, 0, 0);
		REG_INIT(i++, PM800_ID, LDO17, 3300000, 3300000, 0, 0);
		break;
	default:
		REG_SUPPLY_INIT(PM800_ID_LDO9, "v_8787", NULL);
		REG_SUPPLY_INIT(PM800_ID_LDO10, "v_vibrator", NULL);
		REG_SUPPLY_INIT(PM800_ID_LDO17, "VSim", NULL);
		/*For BU turn on WiFi LDO always on,
		 *  need to fix after by WiFi Team*/
		REG_INIT(i++, PM800_ID, LDO9, 2400000, 3300000, 1, 1);
		REG_INIT(i++, PM800_ID, LDO10, 2800000, 2800000, 0, 0);
		REG_INIT(i++, PM800_ID, LDO17, 1800000, 3300000, 0, 0);
		break;
	}
	pr_info("%s: select saarC NEVO austica ldo map\n", __func__);

	pm800_info.num_regulators = i;
}

static struct i2c_board_info i2c1_860x_info[] = {
	{
		I2C_BOARD_INFO("88PM860x", 0x34),
		.platform_data	= &pm8607_info,
		.irq            = IRQ_PMIC_INT,
	},
};

static struct i2c_board_info i2c1_80x_info[] = {
	{
		I2C_BOARD_INFO("88PM80x", 0x34),
		.platform_data	= &pm800_info,
		.irq		= IRQ_PMIC_INT,
	},
};

static struct clk *clk_tout_s0;

/* specific 8787 power on/off setting for SAARB */
static void wifi_set_power(unsigned int on)
{
	unsigned long wlan_pd_mfp = 0;
	int gpio_power_down = mfp_to_gpio(MFP_PIN_GPIO70);
	int gpio_wifi_en = mfp_to_gpio(MFP_PIN_GPIO102);
	struct regulator *v_ldo12 = NULL;
	struct regulator *v_ldo17 = NULL;

	if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD) {
		v_ldo12 = regulator_get(NULL, "v_wifi_1v8");
		if (IS_ERR(v_ldo12)) {
			v_ldo12 = NULL;
			return;
		}
		v_ldo17 = regulator_get(NULL, "v_wifi_3v3");
		if (IS_ERR(v_ldo17)) {
			regulator_put(v_ldo12);
			v_ldo12 = NULL;
			v_ldo17 = NULL;
			return;
		}
	}

	wlan_pd_mfp = pxa3xx_mfp_read(gpio_power_down);

	if (on) {
		if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD) {
			regulator_enable(v_ldo12);
			regulator_enable(v_ldo17);
			gpio_request(gpio_wifi_en, "WIB_EN");
			gpio_direction_output(gpio_wifi_en, 1);
		}
		/* set wlan_pd pin to output high in low power
			mode to ensure 8787 is not power off in low power mode*/
		wlan_pd_mfp |= 0x100;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* enable 32KHz TOUT */
		clk_enable(clk_tout_s0);
	} else {
		/*set wlan_pd pin to output low in low power
			mode to save power in low power mode */
		wlan_pd_mfp &= ~0x100;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* disable 32KHz TOUT */
		clk_disable(clk_tout_s0);
		if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD) {
			gpio_direction_output(gpio_wifi_en, 0);
			gpio_free(gpio_wifi_en);
			regulator_disable(v_ldo12);
			regulator_disable(v_ldo17);
		}
	}
	if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD) {
		regulator_put(v_ldo12);
		regulator_put(v_ldo17);
	}
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

static struct sdhci_pxa_platdata mci0_platform_data = {
	.flags	= PXA_FLAG_CARD_PERMANENT |
				PXA_FLAG_SD_8_BIT_CAPABLE_SLOT |
				PXA_FLAG_ACITVE_IN_SUSPEND |
				PXA_FLAG_ENABLE_CLOCK_GATING |
				PXA_FLAG_KEEP_POWER_IN_SUSPEND,
};

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

	if(!host->vmmc) {
		printk(KERN_INFO"%s short circuit check not necessary \n",
			mmc_hostname(host->mmc));
		return 0;
	}

	if(mfp_num > MAX_SD_PINS) {
		printk(KERN_ERR"%s mfp_num(%d) > MAX_SD_PINS(%d) \n",
			__FUNCTION__,mfp_num, MAX_SD_PINS);
		return 1;
	}

	regulator_enabled = regulator_is_enabled(host->vmmc);
	if (regulator_enabled)
		regulator_disable(host->vmmc);

	for(i=0; i<mfp_num;i++) {
		mfp_bak[i] = mfp_read(mfp_start+i);
		val = mfp_bak[i];
		val &= ~(MFPR_PULL_MASK | MFPR_AF_MASK);
		if(pull_up)
			val |= MFPR_PULL_FLOAT;
		else
			val |= MFPR_PULL_HIGH;
		mfp_write(mfp_start+i, val);
		err = gpio_request(MFP_PIN(mfp_start+i), NULL);
		if (err) {
			printk(KERN_ERR"%s gpio_request[%d] fail\n",
				__FUNCTION__,MFP_PIN(mfp_start+i));
			ret = 1;
			goto out;
		}
		gpio_direction_input(MFP_PIN(mfp_start+i));
	}
	udelay(200);

	regulator_enable(host->vmmc);
	for(retry=0; retry < 3; retry ++) {
		/*retry 3 times, in case the voltage cannot reach in 300us*/
		udelay(300);/*TODO: check whether LDO can reach high voltage*/
		for(i=0; i<mfp_num;i++) {
			val = gpio_get_value(MFP_PIN(mfp_start+i));
			val &= 1 << (MFP_PIN(mfp_start+i) % 32);
			if (!val && (retry == 2)) {
				ret = 1;
				printk(KERN_ERR"%s GPIO[%d] short circuit \
					detected \n", __FUNCTION__,
					MFP_PIN(mfp_start+i));
			}
		}
		if (!ret) /* There are no short circuit detected*/
			break;
	}

	regulator_disable(host->vmmc);
out:
	for(i=0; i<mfp_num;i++) {
		gpio_free(MFP_PIN(mfp_start+i));
		mfp_write(mfp_start+i, mfp_bak[i]);
	}

	return ret;
}

/*
* According to SD spec, CLK, CMD, and DAT3-0 should be LOW before VDD becomes high.
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

	if(!host->vmmc)
		return 0;

	if(mfp_num > MAX_SD_PINS) {
		printk(KERN_ERR"%s mfp_num(%d) > MAX_SD_PINS(%d) \n",
			__FUNCTION__,mfp_num, MAX_SD_PINS);
		return 1;
	}

	regulator_enabled = regulator_is_enabled(host->vmmc);
	if (regulator_enabled)
		regulator_disable(host->vmmc);

	for(i=0; i<mfp_num;i++) {
		mfp_bak[i] = mfp_read(mfp_start+i);
		val = mfp_bak[i];
		val &= ~MFPR_AF_MASK;
		mfp_write(mfp_start+i, val);
		err = gpio_request(MFP_PIN(mfp_start+i), NULL);
		if (err) {
			printk(KERN_ERR"%s gpio_request[%d] fail\n",
				__FUNCTION__,MFP_PIN(mfp_start+i));
			ret = 1;
			goto out;
		}
		gpio_direction_output(MFP_PIN(mfp_start+i), 0);
		printk(KERN_ERR"gpio[%d] set as LOW before power on\n",
				MFP_PIN(mfp_start+i));
	}

	udelay(200);
	regulator_enable(host->vmmc);
	udelay(300);

out:
	for(i=0; i<mfp_num;i++) {
		gpio_free(MFP_PIN(mfp_start+i));
		mfp_write(mfp_start+i, mfp_bak[i]);
	}

	return ret;
}

static struct sdhci_pxa_platdata mci1_platform_data = {
	.flags = PXA_FLAG_ENABLE_CLOCK_GATING |
			PXA_FLAG_ACITVE_IN_SUSPEND,
	.ext_cd_gpio = mfp_to_gpio(MFP_PIN_GPIO123),
	.ext_cd_gpio_invert = 1,
	.mfp_start = 116,
	.mfp_num = 6,
	.pull_up = 0,
	.check_short_circuit = pxa_check_sd_short_circuit,
	.safe_regulator_on = pxa_safe_sd_on,
};

static struct sdhci_pxa_platdata mci2_platform_data = {
	.flags  = PXA_FLAG_CARD_PERMANENT,
	.pm_caps = MMC_PM_KEEP_POWER |
				MMC_PM_IRQ_ALWAYS_ON,
	.handle_cdint = pxa95x_handle_cdint,
};

static void __init init_mmc(void)
{
	int gpio_rst, gpio_pd;

	gpio_pd = mfp_to_gpio(MFP_PIN_GPIO70);
	if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD) {
		gpio_rst = mfp_to_gpio(MFP_PIN_GPIO97);
		mci1_platform_data.ext_cd_gpio = mfp_to_gpio(MFP_PIN_GPIO128);
	} else
		gpio_rst = mfp_to_gpio(MFP_PIN_GPIO99);

        if (cpu_is_pxa978_Cx())
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
	.vibrator_power = vibrator_set_power,
};

static int cywee_set_power(int on)
{
	struct regulator *v_ldo11;
	v_ldo11 = regulator_get(NULL, "v_cywee");

	if (IS_ERR(v_ldo11)) {
		v_ldo11 = NULL;
		return -EIO;
	}

	if (on) {
		regulator_enable(v_ldo11);
		msleep(100);
	} else {
		msleep(100);
		regulator_disable(v_ldo11);
	}
	regulator_put(v_ldo11);
	v_ldo11 = NULL;
	return 0;
}

static struct cwmi_platform_data cwmi_acc_data = {
	.set_power = cywee_set_power,
	.axes = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0},
};

static struct cwmi_platform_data cwmi_mag_data = {
	.set_power = cywee_set_power,
	.axes = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0},
};

static struct cwgd_platform_data cwgd_plat_data = {
	.set_power = cywee_set_power,
	.axes = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0},
};

static int ssd2531_ts_pins[] = { MFP_PIN_GPIO8, MFP_PIN_GPIO7 };

#if defined(CONFIG_HDMI_ADV7533)
static void adv7533_hdmi_reset(void)
{
	int pin0 = mfp_to_gpio(MFP_PIN_GPIO15);
	int pin1 = mfp_to_gpio(MFP_PIN_GPIO13);
	printk(KERN_INFO "hdmi: adv7533_hdmi_reset+\n");

	if (gpio_request(pin0, "hdmi-reset0")) {
		printk(KERN_ERR "hdmi: gpio_request0: failed!\n");
	}
	if (gpio_request(pin1, "hdmi-reset1")) {
		printk(KERN_ERR "hdmi: gpio_request1: failed!\n");
	}
	gpio_direction_output(pin1, 1);
	msleep(10);
	gpio_direction_output(pin0, 0);
	msleep(10);
	gpio_direction_output(pin0, 1);
	msleep(10);
	gpio_direction_output(pin0, 0);
	msleep(10);
	gpio_free(pin0);
	gpio_free(pin1);
	printk(KERN_INFO "hdmi: adv7533_hdmi_reset-\n");
}
#endif

#if defined(CONFIG_BACKLIGHT_ADP8885)
static struct adp8885_bl_channel_data adp8885_ch[] = {
	{	/* Display backlight */
		.sovp		= ADP8885_SOVP_ENABLE,
		.fb_dis		= ADP8885_FB_ENABLE,
		.dim_en		= ADP8885_DIM_DISABLE,
		.imax		= ADP8885_BL_MAX_CUR_uA(25600),
		.iset		= ADP8885_ISET_80,
		.fade_out	= ADP8885_FADE_510ms,
		.fade_in	= ADP8885_FADE_510ms,
		.iovr		= ADP8885_OVR_DISABLE,
	},
	{	/* Keypad backlight */
		.sovp		= ADP8885_SOVP_ENABLE,
		.fb_dis		= ADP8885_FB_ENABLE,
		.dim_en		= ADP8885_DIM_DISABLE,
		.imax		= ADP8885_BL_MAX_CUR_uA(25600),
		.iset		= ADP8885_ISET_40,
		.fade_out	= ADP8885_FADE_510ms,
		.fade_in	= ADP8885_FADE_510ms,
		.iovr		= ADP8885_OVR_DISABLE,
	},
};

static int adp8885_bl_enable(bool enable)
{
       static int gpio;
       if (!gpio) {
               gpio = mfp_to_gpio(MFP_PIN_GPIO72);
               if (gpio_request(gpio, "backlight_enable")) {
                       pr_err("%s: Request gpio #%d failed\n", __func__, gpio);
                       return -EIO;
               }
       }
       gpio_direction_output(gpio, (int)enable);
       msleep(1);
       return 0;
}

static struct adp8885_bl_platform_data adp8885_data = {
	.psm_en		= ADP8885_PWS_ENABLE,
	.fsw		= ADP8885_FSW_600KHZ,
	.ipeak		= ADP8885_IPEAK_500mA,
	.ovp_lvl	= ADP8885_OVP_LVL_28V,
	.ledout_h	= 0,
	.ledout_l	= 0,
	.ch		= adp8885_ch,
};
#endif

static struct i2c_board_info i2c2_info_DKB[] = {
#if defined(CONFIG_SENSORS_ROHM_BH1772)
	{
		I2C_BOARD_INFO("rohm_ls", 0x38),
		.irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO87)),
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_SSD2531)
	{
		I2C_BOARD_INFO("ssd2531_ts", 0x5c),
		.platform_data = ssd2531_ts_pins,
	},
#endif
#if defined(CONFIG_BACKLIGHT_ADP8885)
	{
		I2C_BOARD_INFO("adp8885", 0x3A), /* 0x74 */
		.platform_data = (void *)&adp8885_data,
	},
#endif
};

static struct i2c_board_info i2c2_info_C2[] = {
#if defined(CONFIG_TOUCHSCREEN_SSD2531)
	{
		I2C_BOARD_INFO("ssd2531_ts", 0x5c),
		.platform_data = ssd2531_ts_pins,
	},
#endif
};

static struct i2c_board_info i2c2_info_C25[] = {
#if defined(CONFIG_SENSORS_APDS990X_MRVL)
	{
		I2C_BOARD_INFO("apds990x", 0x39), /* 0x72 */
		.irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO87)),
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_SSD2531)
	{
		I2C_BOARD_INFO("ssd2531_ts", 0x5c),
		.platform_data = ssd2531_ts_pins,
	},
#endif
#if defined(CONFIG_BACKLIGHT_ADP8885)
	{
		I2C_BOARD_INFO("adp8885", 0x3A), /* 0x74 */
		.platform_data = (void *)&adp8885_data,
	},
#endif
};

static struct i2c_board_info i2c3_info[] = {
#if defined(CONFIG_SENSORS_CWMI)
	{
		I2C_BOARD_INFO("cwmi_acc", 0x19),
		.irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO11)),
		.platform_data = &cwmi_acc_data,
	},

	{
		I2C_BOARD_INFO("cwmi_mag", 0x1e),
		.irq = 0,/*gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO10)),*/
		.platform_data = &cwmi_mag_data,
	},
#endif
#if defined(CONFIG_SENSORS_CWGD)
	{
		I2C_BOARD_INFO("cwgd", 0x69),
		.irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO9)),
		.platform_data = &cwgd_plat_data,
	 },
#endif
#if defined(CONFIG_HDMI_ADV7533)
	{
		I2C_BOARD_INFO("adv7533-packet", 0x38),
	},
	{
		I2C_BOARD_INFO("adv7533-main", 0x39),
	},
	{
		I2C_BOARD_INFO("adv7533-cec_dsi", 0x2c),
	},
	{
		I2C_BOARD_INFO("adv7533-edid", 0x3f),
		.platform_data = adv7533_hdmi_reset,
	},
#endif

#if defined(CONFIG_C_TEC_OPTIC_TP)
	{
		I2C_BOARD_INFO("ctec_optic_tp", 0x33),
		.irq	= gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO100)),
	},
#endif
};

static void register_i2c_board_info(void)
{

	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		i2c_register_board_info(0, ARRAY_AND_SIZE(i2c1_860x_info));
		i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info_C2));
		break;

	case OBM_SAAR_C25_NEVO_B0_V10_BOARD:
	case OBM_EVB_NEVO_1_2_BOARD:
	case OBM_SAAR_C3_NEVO_C0_V10_BOARD:
	case OBM_SAAR_C3_NEVO_C0_V10_BOARD_533MHZ:
		i2c_register_board_info(0, ARRAY_AND_SIZE(i2c1_80x_info));
		i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info_C25));
		break;
	case OBM_DKB_2_NEVO_C0_BOARD:
		pm800_info.vibrator = &vibrator_pdata;
		i2c_register_board_info(0, ARRAY_AND_SIZE(i2c1_80x_info));
		i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info_DKB));
		break;

	default:
		pr_err("%s: Unknown board type!\n", __func__);
		BUG();
	}

	i2c_register_board_info(2, ARRAY_AND_SIZE(i2c3_info));
}

static struct i2c_pxa_platform_data i2c1_pdata = {
	.use_pio        = 0,
	.flags		= PXA_I2C_HIGH_MODE | PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
	.master_code	= (0x08 | 0x06), /*8 -highest, 0xF -lowest arbitration*/
};

static struct i2c_pxa_platform_data i2c2_pdata = {
	.use_pio	= 0,
	.flags		= PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
};

static struct i2c_pxa_platform_data i2c3_pdata = {
	.use_pio = 0,
	.flags = PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
};

static struct platform_device *devices[] __initdata = {
	&pxa95x_device_i2c1,
	&pxa95x_device_i2c2,
	&pxa95x_device_i2c3,
};

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)
#define STATUS2_VBUS        (1 << 4)

static int	read_vbus_val(void)
{
	int ret;
	ret = pm860x_codec_reg_read(2);
	if (ret & STATUS2_VBUS)
		ret = VBUS_HIGH;
	else
		ret = VBUS_LOW;
	return ret;
};

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

static struct mv_usb_addon_irq pm860x_id = {
	.irq	= IRQ_BOARD_START + PM8607_IRQ_GPADC2,
	.poll	= pm860x_read_id_val,
	.init	= pm860x_init_id,
};

static struct mv_usb_addon_irq pm860x_vbus = {
	.irq	= IRQ_BOARD_START + PM8607_IRQ_CHG,
	.poll	= read_vbus_val,
};


static struct mv_usb_platform_data pxa9xx_usb_pdata = {
	.clknum		= 3,
	.clkname	= pxa9xx_usb_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_OTG,
	.phy_init	= pxa9xx_usb_phy_init,
	.set_vbus	= NULL,
	.otg_force_a_bus_req = 1,
};
#endif

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_HOME),
	KEY(0, 1, KEY_SEND),
	KEY(0, 2, KEY_CAMERA),
	KEY(1, 0, KEY_WWW),
	KEY(1, 1, KEY_MENU),
	KEY(1, 2, KEY_CAMERA),
	KEY(2, 0, KEY_END),
	KEY(2, 1, KEY_VOLUMEUP),
	KEY(2, 2, KEY_VOLUMEUP),/*not exist*/
	KEY(3, 0, KEY_BACK),
	KEY(3, 1, KEY_VOLUMEDOWN),
	KEY(3, 2, KEY_VOLUMEDOWN),/*not exists*/
};

static struct pxa27x_keypad_platform_data keypad_info = {
	.matrix_key_rows = 4,
	.matrix_key_cols = 3,
	.matrix_key_map = matrix_key_map,
	.matrix_key_map_size = ARRAY_SIZE(matrix_key_map),
	.debounce_interval = 30,
	.active_low = 1,
	.direct_key_num = 5,
	.direct_key_map = { KEY_RESERVED,
						KEY_RESERVED,
						KEY_RESERVED,
						KEY_RESERVED,
						KEY_ENTER },
};

static unsigned int matrix_key_map_dkb2[] = {
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

static struct pxa27x_keypad_platform_data keypad_info_dkb2 = {
	.matrix_key_rows = 4,
	.matrix_key_cols = 3,
	.matrix_key_map = matrix_key_map_dkb2,
	.matrix_key_map_size = ARRAY_SIZE(matrix_key_map_dkb2),
	.debounce_interval = 30,
	.active_low = 1,
};

#endif /* CONFIG_KEYBOARD_PXA27x || CONFIG_KEYBOARD_PXA27x_MODULE */

/* Camera LDO */
static struct regulator *vcamera = NULL;
/* Camera sensor PowerDowN pins */
static int pwd_main, pwd_sub, pwd_isp;

#if defined(CONFIG_VIDEO_PXA955)
static int camera0_power(struct device *dev, int flag)
{
	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		/* Initialize AF_VCC */
		if (vcamera == NULL) {
			vcamera = regulator_get(NULL, "v_cam");
			if (IS_ERR(vcamera)) {
				return -ENODEV;
			}
		}

		/* Driver SWPD pin for main cam */
		if (flag) {
			gpio_direction_output(pwd_main, 0);	/* enable */
			msleep(1);
			regulator_enable(vcamera);
		} else {
			regulator_disable(vcamera);
			gpio_direction_output(pwd_main, 1);	/* disable */
		}
		return 0;
	/* Default for SAAR-C 2.5 and 3, carries M6MO+OV8820 and OV9740 */
	default:
		switch (flag) {
		case SENSOR_OPEN:
			gpio_direction_output(pwd_isp, 1);	/* enable */
			/* Wait 100ms here to let ISP boot up and working */
			msleep(100);
			break;
		case SENSOR_CLOSE:
			gpio_direction_output(pwd_isp, 0);	/* disable */
			mdelay(50);
			break;
		case ISP_SENSOR_OPEN:
			gpio_direction_output(pwd_main, 1);	/* enable */
			mdelay(50);
			break;
		case ISP_SENSOR_CLOSE:
			gpio_direction_output(pwd_main, 0);	/* disable */
			mdelay(50);
			break;
		}
		return 0;
	}
	return 0;
}

static int camera1_power(struct device *dev, int flag)
{
	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		/* Initialize AF_VCC */
		if (vcamera == NULL) {
			vcamera = regulator_get(NULL, "v_cam");
			if (IS_ERR(vcamera)) {
				return -ENODEV;
			}
		}
		/* Driver SWPD pin for sub cam */
		if (flag) {
			gpio_direction_output(pwd_sub, 0);
			msleep(2);
			regulator_enable(vcamera);
		} else {
			regulator_disable(vcamera);
			gpio_direction_output(pwd_sub, 1);
		}
		return 0;
	/* Default for SAAR-C 2.5 and 3, carries M6MO+OV8820 and OV9740 */
	default:
		if (flag) {
			gpio_direction_output(pwd_sub, 0);
			msleep(1);
		} else {
			gpio_direction_output(pwd_sub, 1);
		}
		return 0;
	}
	return 0;
}

static int icphd_power(struct device *dev, int flag)
{
	if (flag) {
		gpio_direction_output(pwd_main,	0);	/* Clear STANDBY */
		gpio_direction_output(pwd_isp, 1);
		msleep(100);
	} else {
		gpio_direction_output(pwd_isp, 0);
		gpio_direction_output(pwd_main,	1);	/* Clear STANDBY */
		msleep(10);
	}
	return 0;
}

static struct i2c_board_info camera_i2c[] = {
	{
		I2C_BOARD_INFO("m6mo", 0x1F),
	},
	{
		I2C_BOARD_INFO("ov9740", 0x10),
	},
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
	{
		I2C_BOARD_INFO("ov7692", 0x3c),
	},
	{
		I2C_BOARD_INFO("icp-hd", 0x3D),
	},
};

static struct pxa95x_csi_dev csidev[] = {
	{
		.id		= 0,
		.irq_num	= 71,
		.reg_start	= 0x50020000,
	},
	{
		.id		= 1,
		.irq_num	= 58,
		.reg_start	= 0x50022000,
	},
};

enum {
	ID_M6MO_SAARC	= 0,
	ID_M6MO_DKB,
	ID_OV9740,
	ID_OV5642,
	ID_OV7692,
	ID_ICPHD,
};

static struct sensor_platform_data camera_sensor[] = {
	[ID_M6MO_SAARC] = {/* M6MO on SaarC 3*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI \
				| SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO24,	/* M6MO PWD pin */
		.pin_aux	= MFP_PIN_GPIO18,	/* OV8820 PWD pin */
		.pin_irq	= MFP_PIN_GPIO102,	/* M6MO irq pin */
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "SaarC 3",
	},
	[ID_M6MO_DKB] = {/* M6MO on DKB 2*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI \
				| SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO24,	/* M6MO PWD pin */
		.pin_aux	= MFP_PIN_GPIO25,	/* OV8820 PWD pin */
		.pin_irq	= MFP_PIN_GPIO18,	/* M6MO irq pin */
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "DKB 2",
	},
	[ID_OV9740] = {/* OV9740 on SaarC3 and DKB2*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_FRONT | SENSOR_RES_LOW,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI \
				| SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[1],	/* connected to CSI1 */
		.pin_pwdn	= MFP_PIN_GPIO26,
		.af_cap		= 0,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "SaarC 2/3",
	},
	[ID_OV5642] = {/* OV5642 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI \
				| SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO25,
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "TRULY",
		.board_name	= "tavor",
	},
	[ID_OV7692] = {/* OV7692 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_FRONT | SENSOR_RES_LOW,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI \
				| SOCAM_MIPI_1LANE,
		.csi_ctlr	= &csidev[1],	/* connected to CSI1*/
		.pin_pwdn	= MFP_PIN_GPIO26,
		.af_cap		= 0,
		.mclk_mhz	= 26,
		.vendor_info	= "N/A",
		.board_name	= "SaarC 2",
	},
	[ID_ICPHD] = {/* Aptina on DKB 2*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI \
				| SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO24,	/* M6MO PWD pin */
		.pin_aux	= MFP_PIN_GPIO25,	/* OV8820 PWD pin */
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "DKB 2",
	},
};

static struct soc_camera_link iclink[] = {
	[ID_M6MO_SAARC] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[0],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "m6mo",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_M6MO_SAARC],
	},
	[ID_M6MO_DKB] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[0],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "m6mo",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_M6MO_DKB],
	},
	[ID_OV9740] = {
		.bus_id			= 1, /* Must match with the camera ID */
		.board_info		= &camera_i2c[1],
		.i2c_adapter_id		= 1,
		.power = camera1_power,
		.module_name		= "ov9740",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV9740],
	},
	[ID_OV5642] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[2],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "ov5642",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV5642],
	},
	[ID_OV7692] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[3],
		.i2c_adapter_id		= 1,
		.power = camera1_power,
		.module_name		= "ov7692",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV7692],
	},
	[ID_ICPHD] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[4],
		.i2c_adapter_id		= 1,
		.power = icphd_power,
		.module_name		= "icp-hd",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_ICPHD],
	},
};

static struct platform_device __attribute__((unused)) camera[] = {
	[ID_M6MO_SAARC] = {
		.name	= "soc-camera-pdrv",
		.id	= 0,
		.dev	= {
			.platform_data = &iclink[ID_M6MO_SAARC],
		},
	},
	[ID_M6MO_DKB] = {
		.name	= "soc-camera-pdrv",
		.id	= 0,
		.dev	= {
			.platform_data = &iclink[ID_M6MO_DKB],
		},
	},
	[ID_OV9740] = {
		.name	= "soc-camera-pdrv",
		.id	= 1,
		.dev	= {
			.platform_data = &iclink[ID_OV9740],
		},
	},
	[ID_OV5642] = {
		.name	= "soc-camera-pdrv",
		.id	= 0,
		.dev	= {
			.platform_data = &iclink[ID_OV5642],
		},
	},
	[ID_OV7692] = {
		.name	= "soc-camera-pdrv",
		.id	= 1,
		.dev	= {
			.platform_data = &iclink[ID_OV7692],
		},
	},
	[ID_ICPHD] = {
		.name	= "soc-camera-pdrv",
		.id	= 2,
		.dev	= {
			.platform_data = &iclink[ID_ICPHD],
		},
	},
};
#endif

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

	v_ldo1v2 = regulator_get(NULL, "Vdd_CMMB12");

	if (IS_ERR(v_ldo1v2)) {
		printk(KERN_ERR "cmmb: fail to get ldo1v2 handle!\n");
		return -EINVAL;
	}

	v_ldo1v8 = regulator_get(NULL, "Vdd_IO");

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

static void __init nevo_dkb_init_spi(void)
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

static void __init init_cam(void)
{
	/* PWD pin GPIO initialize */
	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		pwd_main = mfp_to_gpio(MFP_PIN_GPIO25);
		pwd_sub = mfp_to_gpio(MFP_PIN_GPIO26);
		pwd_isp	= 0;
		break;
	case OBM_DKB_2_NEVO_C0_BOARD:
		pwd_main = mfp_to_gpio(MFP_PIN_GPIO25);
		pwd_sub = mfp_to_gpio(MFP_PIN_GPIO26);
		pwd_isp	= mfp_to_gpio(MFP_PIN_GPIO24);
		break;
	default: /* For SaarC 2.5 and 3*/
		pwd_main = mfp_to_gpio(MFP_PIN_GPIO18);
		pwd_sub = mfp_to_gpio(MFP_PIN_GPIO26);
		pwd_isp	= mfp_to_gpio(MFP_PIN_GPIO24);
		break;
	}

	if (pwd_isp && gpio_request(pwd_isp, "CAM_ISP_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", pwd_isp);
		pwd_isp = 0;
		return;
	}

	/* Camera hold these GPIO forever, should not be accquired by others */
	if (pwd_main && gpio_request(pwd_main, "CAM_HI_SENSOR_PWD")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", pwd_main);
		pwd_main = 0;
		return;
	}
	if (pwd_sub && gpio_request(pwd_sub, "CAM_LOW_SENSOR_PWD")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", pwd_sub);
		pwd_sub = 0;
		return;
	}

#if defined(CONFIG_VIDEO_PXA955)
	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		printk(KERN_NOTICE "cam: saarc: Probing camera on SaarC 2\n");
#if defined(CONFIG_SOC_CAMERA_OV5642)
		platform_device_register(&camera[ID_OV5642]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV7692)
		platform_device_register(&camera[ID_OV7692]);
#endif
		break;
	case OBM_DKB_2_NEVO_C0_BOARD:
		printk(KERN_NOTICE "cam: saarc: Probing camera on DKB 2\n");
#if defined(CONFIG_SOC_CAMERA_ICPHD)
		platform_device_register(&camera[ID_ICPHD]);
#endif
#if defined(CONFIG_SOC_CAMERA_M6MO)
		platform_device_register(&camera[ID_M6MO_DKB]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV9740)
		platform_device_register(&camera[ID_OV9740]);
#endif
		break;
	default: /* For SaarC 2.5 and 3*/
		printk(KERN_NOTICE "cam: saarc: Probing camera on SaarC 3\n");
#if defined(CONFIG_SOC_CAMERA_ICPHD)
		platform_device_register(&camera[ID_ICPHD]);
#endif
#if defined(CONFIG_SOC_CAMERA_M6MO)
		platform_device_register(&camera[ID_M6MO_SAARC]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV9740)
		platform_device_register(&camera[ID_OV9740]);
#endif
		break;
	}
	platform_device_register(&pxa95x_device_cam0);
	platform_device_register(&pxa95x_device_cam1);
#endif
}

#if defined(CONFIG_FB_PXA95x)

#if defined(CONFIG_MV_IHDMI)
static int mv_ihdmi_format = 4;

static int hdtx_power(int en)
{
	struct regulator *v_ldo;
	int pin1 = mfp_to_gpio(MFP_PIN_GPIO13);

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
	printk(KERN_INFO "hdmi: turn %s charge pump 5V\n", en?"ON":"OFF");

	regulator_put(v_ldo);
	return 0;
}

#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data hdtx_uio_data = {
	.sspa_reg_base = 0x41900000,
	.itlc_reg_base = 0x4410c000,
	.gpio = MFP_PIN_GPIO112,
	.hdmi_v5p_power = hdtx_power,
	.edid_bus_num = 3,	/* edid on i2c-3*/
	.hpd_val = 0x10000,	/* hpd-gpio-112 pin, when plug in, its value is 1<<16*/
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
        .num_modes              = 1,
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
        .panel_power                    = hdtx_power,
#endif
        .invert_pixclock        = 1,
};

static struct pxa95xfb_mach_info ihdmi_ovly_info __initdata = {
	.id                     = "HDMI-Ovly",
	.num_modes              = 1,
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
};
#endif

#if defined(CONFIG_HDMI_ADV7533)
static struct pxa95xfb_mach_info adv7533_hdmi_ovly_info __initdata = {
	.id                     = "HDMI-Ovly",
	.modes                  = video_modes_adv7533,
	.num_modes              = ARRAY_SIZE(video_modes_adv7533),
	.pix_fmt_in             = PIX_FMTIN_YUV420,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_Active,
	/*as hdmi-ovly use same win4 with lcd-ovly, they should not open at the same time*/
	.window					= 4,
	.mixer_id				= 1,
	.zorder					= 0,
	.converter				= LCD_M2DSI1,
	.output					= OUTPUT_HDMI,
	.active					= 1,
	.invert_pixclock		= 1,
};
#endif

static void panel_power(int on)
{
	panel_power_trulywvga(1, on);
}

static void panel_reset(void)
{
	int reset_pin;
	int err;

	reset_pin = mfp_to_gpio(MFP_PIN_GPIO23);
	err = gpio_request(reset_pin, "DSI Reset");
	if (err) {
		gpio_free(reset_pin);
		printk(KERN_ERR "Request GPIO failed, gpio: %d return :%d\n",
		       reset_pin, err);
		return;
	}
	gpio_direction_output(reset_pin, 1);
	mdelay(1);
	gpio_direction_output(reset_pin, 0);
	mdelay(1);
	gpio_direction_output(reset_pin, 1);
	mdelay(10);
	gpio_free(reset_pin);
}

static struct pxa95xfb_mach_info lcd_info /*__initdata*/ = {
	.id = "Base",
	.modes = video_modes_trulywvga,
	.num_modes = ARRAY_SIZE(video_modes_trulywvga),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_24_RGB888,
	.panel_type = LCD_Controller_Active,
	.window = 0,
	.mixer_id = 0,
	.zorder = 1,
	.converter = LCD_M2PARALELL_CONVERTER,
	.output = OUTPUT_PANEL,
	.active = 1,
	.panel_power = panel_power,
	.invert_pixclock = 1,
	.reset = panel_reset,
};

static struct pxa95xfb_mach_info lcd_ovly_info /*__initdata*/ = {
	.id = "Ovly",
	.modes = video_modes_trulywvga,
	.num_modes = ARRAY_SIZE(video_modes_trulywvga),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_24_RGB888,
	.panel_type = LCD_Controller_Active,
	.window = 4,
	.mixer_id = 0,
	.zorder = 0,
	.converter = LCD_M2PARALELL_CONVERTER,
	.output = OUTPUT_PANEL,
	.active = 1,
	.panel_power = panel_power,
	.invert_pixclock = 1,
	.reset = panel_reset,
};

static void __init init_lcd(void)
{
	set_pxa95x_fb_info(&lcd_info);
	set_pxa95x_fb_ovly_info(&lcd_ovly_info, 0);
#if defined(CONFIG_MV_IHDMI)
#ifdef CONFIG_UIO_HDMI
	init_hdmi();
#else
	pxa_register_device(&pxa978_device_ihdmi, &mv_ihdmi_format);
#endif
	ihdmi_base_info.modes = &video_modes_ihdmi[mv_ihdmi_format-1];
	set_pxa95x_fb_ovly_info(&ihdmi_base_info, 1);
	ihdmi_ovly_info.modes = &video_modes_ihdmi[mv_ihdmi_format-1];
	set_pxa95x_fb_ovly_info(&ihdmi_ovly_info, 2);
#elif defined(CONFIG_HDMI_ADV7533)
	set_pxa95x_fb_ovly_info(&adv7533_hdmi_ovly_info, 1);
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

static mfp_cfg_t pxa95x_abu_mfp_cfg[] = {
	/* ABU of MG1 */
	GPIO63_ABU_RXD,
	GPIO64_ABU_TXD,
	GPIO65_GPIO,	/* no use for ABU/SSI, and configure GPIO70 to AF0 to save power when using ABU (~0.5mA) */
	GPIO66_ABU_FRM,
	GPIO67_ABU_CLK,
};

static mfp_cfg_t pxa95x_bssp2_mfp_cfg[] = {
	/* BSSP2 of MG1 */
	GPIO63_SSP2_RXD,
	GPIO64_SSP2_TXD,
	GPIO65_SSP2_SYSCLK,
	GPIO66_SSP2_FRM,
	GPIO67_SSP2_CLK,
};

static mfp_cfg_t bssp3_mfp_cfg[] = {
	/* BSSP3 of MG1*/
	GPIO58_BSSP3_CLK,
	GPIO59_BSSP3_FRM,
	GPIO60_BSSP3_TXD,
	GPIO61_BSSP3_RXD,
};

static mfp_cfg_t gssp1_mfp_cfg[] = {
	/* BSSP3 of MG1*/
	GPIO58_GSSP1_CLK,
	GPIO59_GSSP1_FRM,
	GPIO60_GSSP1_TXD,
	GPIO61_GSSP1_RXD,
};

static void abu_mfp_init(bool abu)
{
	if (abu)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa95x_abu_mfp_cfg));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa95x_bssp2_mfp_cfg));
}

static void ssp3_mfp_init(bool bssp)
{
	if (bssp)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(bssp3_mfp_cfg));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(gssp1_mfp_cfg));
}

#ifdef CONFIG_PM
static int init_wakeup(pm_wakeup_src_t *src)
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
	return 0;
}

u32 *sd_normal_int_err_status0, *sd_normal_int_err_status1, *sd_normal_int_err_status2;
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
		if (pxa95x_query_gwsr(53))
			src->bits.uart1 = 1;
		if (pxa95x_query_gwsr(123))
			src->bits.mmc1_cd = 1;
	}
	if (is_wkr_nevo_2243()) {
		/* Clear unexpected irq. */
		/* MMC0 interrupt pending. */
		if ((ICIP3 & (1 << (72 - 64)))) {
			writel(0xffffffff, sd_normal_int_err_status0);
		}

		/* MMC1 interrupt pending. */
		if ((ICIP3 & (1 << (73 - 64)))) {
			writel(0xffffffff, sd_normal_int_err_status1);
		}

		/* MMC2 interrupt pending. */
		if ((ICIP3 & (1 << (74 - 64)))) {
			writel(0xfffffeff, sd_normal_int_err_status2);
		}
		if (!is_mmc) {
			writel(0x100, sd_normal_int_err_status2);
		}
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
	if (enable) {
		if (src.bits.mkey) {
			static mfp_cfg_t key_edgeboth_cfg[] = {
				GPIO0_KP_MKIN_0 | MFP_LPM_EDGE_BOTH,
				GPIO2_KP_MKIN_1 | MFP_LPM_EDGE_BOTH,
				GPIO4_KP_MKIN_2 | MFP_LPM_EDGE_BOTH,
				GPIO6_KP_MKIN_3 | MFP_LPM_EDGE_BOTH,
				/*GPIO12_KP_DKIN_4 | MFP_LPM_EDGE_BOTH,*/
			};
			pxa3xx_mfp_config(ARRAY_AND_SIZE(key_edgeboth_cfg));
			ret |= PXA95x_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			static mfp_cfg_t key_edgenone_cfg[] = {
				GPIO0_KP_MKIN_0 | MFP_LPM_EDGE_NONE,
				GPIO2_KP_MKIN_1 | MFP_LPM_EDGE_NONE,
				GPIO4_KP_MKIN_2 | MFP_LPM_EDGE_NONE,
				GPIO6_KP_MKIN_3 | MFP_LPM_EDGE_NONE,
				/*GPIO12_KP_DKIN_4 | MFP_LPM_EDGE_NONE,*/
			};
			pxa3xx_mfp_config(ARRAY_AND_SIZE(key_edgenone_cfg));
		}
	}
	return ret;
}

static int mmc1_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		if (src.bits.mmc1_cd) {
			mfp_c = GPIO123_GPIO | MFP_LPM_EDGE_BOTH;
			pxa3xx_mfp_config(&mfp_c, 1);
			ret |= PXA95x_PM_WE_GENERIC(13);
		}
	} else {
		mfp_c = GPIO123_GPIO | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}

	return ret;
}

static int mmc3_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		if (src.bits.mmc3_dat1) {
			mfp_c = GPIO80_MMC3_DAT1 | MFP_LPM_EDGE_BOTH;
			pxa3xx_mfp_config(&mfp_c, 1);
			ret |= PXA95x_PM_WE_MMC3;
		}
	} else {
		mfp_c = GPIO80_MMC3_DAT1 | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}

	return ret;
}

static int uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t m;

	if (enable) {
		if (src.bits.uart1) {
			m = GPIO131_UART1_RXD | MFP_LPM_EDGE_FALL;
			pxa3xx_mfp_config(&m, 1);
			ret |= PXA95x_PM_WE_GENERIC(9);
		}
		if (src.bits.uart2) {
			m = GPIO94_UART3_RXD | MFP_LPM_EDGE_FALL;
			pxa3xx_mfp_config(&m, 1);
			/* note: on pxa930, uart2 use this bit */
			ret |= PXA95x_PM_WE_GENERIC(2);
		}
	} else {
		if (src.bits.uart1) {
			m = GPIO131_UART1_RXD | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
		if (src.bits.uart2) {
			m = GPIO94_UART3_RXD | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
	}
	return ret;
}

static int tsi_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t m;
	if (enable) {
		if (src.bits.tsi) {
			m = PMIC_INT_GPIO83 | MFP_LPM_FLOAT | MFP_LPM_EDGE_FALL;
			pxa3xx_mfp_config(&m, 1);
			ret |= PXA95x_PM_WE_GENERIC(3);
		}
	} else {
		if (src.bits.tsi) {
			m = PMIC_INT_GPIO83 | MFP_LPM_FLOAT | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
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

static struct pxa95x_peripheral_wakeup_ops wakeup_ops = {
	.init   = init_wakeup,
	.query  = query_wakeup,
	.ext    = ext_wakeup,
	.key    = key_wakeup,
	.mmc1    = mmc1_wakeup,
	.mmc3    = mmc3_wakeup,
	.uart   = uart_wakeup,
	.tsi    = tsi_wakeup,
	.cmwdt  = comm_wdt_wakeup,
};
#endif

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
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa95x_hsl_mfp_cfg));
	pr_info("the MFP for MicroSD is switched to HSP for IML. \n");
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
#endif

static int reboot_notifier_func(struct notifier_block *this,
		unsigned long code, void *cmd)
{
	int reg = (PM80X_BASE_PAGE << 8) | 0xef;

	if (cmd && (0 == strcmp(cmd, "recovery"))) {
		printk("Enter recovery mode\n");
		pm80x_codec_reg_write(reg, 0x1);
	} else
		pm80x_codec_reg_write(reg, 0x0);

	return 0;
}
static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_notifier_func,
};

static void __init init(void)
{
	if (get_pmic_id() >= PM800_CHIP_A0) {
		regulator_init_pm800();
		pr_info( \
			"[%s][%s]regulator_init_pm800 maxNum[%d] init\n",
			__FILE__, __func__, PM8XXX_REGULATOR_MAX);
		pxa9xx_usb_pdata.vbus = &pm80x_vbus;
		pxa9xx_usb_pdata.id = &pm80x_id;
		pxa9xx_usb_pdata.set_vbus = pm80x_set_vbus;

		register_reboot_notifier(&reboot_notifier);
	} else {
		regulator_init_pm8607();
		pr_info( \
			"[%s][%s]regulator_init_pm8607 maxNum[%d] init\n",
			__FILE__, __func__, PM8XXX_REGULATOR_MAX);
		pxa9xx_usb_pdata.vbus = &pm860x_vbus;
		pxa9xx_usb_pdata.id = &pm860x_id;
		pxa9xx_usb_pdata.set_vbus = pm860x_set_vbus;
	}

	if (get_pmic_id() >= PM800_CHIP_A0)
		headsetflag_init_pm800();
#if defined(CONFIG_BACKLIGHT_ADP8885)
	if (OBM_SAAR_C25_NEVO_B0_V10_BOARD == get_board_id())
		adp8885_data.num_chs = 1;
	else if (OBM_EVB_NEVO_1_2_BOARD == get_board_id() ||
			OBM_SAAR_C3_NEVO_C0_V10_BOARD == get_board_id() ||
			OBM_SAAR_C3_NEVO_C0_V10_BOARD_533MHZ == get_board_id() ||
			OBM_DKB_2_NEVO_C0_BOARD == get_board_id()) {
		adp8885_data.chip_enable = adp8885_bl_enable;
		adp8885_data.num_chs = 2;
	}
#endif

	/* adjust acc sensor axes */
	if (get_board_id() == OBM_SAAR_C3_NEVO_C0_V10_BOARD
	   || get_board_id() == OBM_SAAR_C3_NEVO_C0_V10_BOARD_533MHZ) {
		cwmi_acc_data.axes[0] = -1;
		cwmi_acc_data.axes[4] = 1;
		cwmi_acc_data.axes[8] = -1;
		cwmi_mag_data.axes[0] = -1;
		cwmi_mag_data.axes[4] = 1;
		cwmi_mag_data.axes[8] = -1;
		cwgd_plat_data.axes[0] = -1;
		cwgd_plat_data.axes[4] = 1;
		cwgd_plat_data.axes[8] = -1;
	}
	if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD) {
		i2c3_info[0].irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO10));
		cwmi_acc_data.axes[1] = -1;
		cwmi_acc_data.axes[3] = 1;
		cwmi_acc_data.axes[8] = 1;
		cwmi_mag_data.axes[1] = -1;
		cwmi_mag_data.axes[3] = 1;
		cwmi_mag_data.axes[8] = 1;
		cwgd_plat_data.axes[1] = 1;
		cwgd_plat_data.axes[3] = -1;
		cwgd_plat_data.axes[8] = 1;
	}

	set_abu_init_func(abu_mfp_init);
	set_ssp_init_func(ssp3_mfp_init);

	platform_device_add_data(&pxa95x_device_i2c1, &i2c1_pdata,
				 sizeof(i2c1_pdata));
	platform_device_add_data(&pxa95x_device_i2c2, &i2c2_pdata,
				 sizeof(i2c2_pdata));
	platform_device_add_data(&pxa95x_device_i2c3, &i2c3_pdata,
				 sizeof(i2c3_pdata));

	platform_add_devices(ARRAY_AND_SIZE(devices));
	register_i2c_board_info();

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
	if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD)
		pxa_set_keypad_info(&keypad_info_dkb2);
	else
		pxa_set_keypad_info(&keypad_info);
#endif

	init_cam();

#if defined(CONFIG_FB_PXA95x)
	init_lcd();
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
	if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD)
		nevo_dkb_init_spi();
#endif

#ifdef CONFIG_USB_PXA_U2O
	pxa9xx_device_u2o.dev.platform_data = (void *)&pxa9xx_usb_pdata;
	platform_device_register(&pxa9xx_device_u2o);
#endif

#ifdef CONFIG_USB_PXA_U2O_OTG
	pxa9xx_device_u2ootg.dev.platform_data = (void *)&pxa9xx_usb_pdata;
	platform_device_register(&pxa9xx_device_u2ootg);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2O
	pxa9xx_device_u2oehci.dev.platform_data = (void *)&pxa9xx_usb_pdata;
	platform_device_register(&pxa9xx_device_u2oehci);
#endif

#ifdef CONFIG_PROC_FS
	if (get_board_id() == OBM_DKB_2_NEVO_C0_BOARD) {
		create_sparrow_rf_proc_file();
		create_hsl_mfpr_proc_file();
	}
	create_sirf_proc_file();
#endif
	init_rfreset_gpio();
}

MACHINE_START(NEVOSAARC, "PXA978")
	.map_io		= pxa_map_io,
	.nr_irqs	= NEVOSAARC_NR_IRQS,
	.init_irq	= pxa95x_init_irq,
	.handle_irq	= pxa95x_handle_irq_intc,
	.timer		= &pxa_timer,
	.reserve	= pxa95x_mem_reserve,
	.init_machine	= init,
MACHINE_END

extern int mspm_idle_load(void);
static int __init saarc_pm_init(void)
{
	printk("Enable Power of Saar board.\n");
	switch (get_board_id()) {
		case OBM_DKB_2_NEVO_C0_BOARD:
			cur_profiler = CPUFREQ_PROFILER;
			mspm_idle_load();
			break;

		case OBM_SAAR_C3_NEVO_C0_V10_BOARD:
		case OBM_SAAR_C3_NEVO_C0_V10_BOARD_533MHZ:
		case OBM_EVB_NEVO_1_2_BOARD:
			cur_profiler = MSPM_PROFILER;
			break;
		default:
			pr_err("Bad board ID in %s\n", __func__);
			BUG();
	}
	return 0;
}

late_initcall(saarc_pm_init);
