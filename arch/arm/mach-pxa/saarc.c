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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/cwmi.h>
#include <linux/cwgd.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/regulator/machine.h>
#include <linux/sd8x_rfkill.h>
#include <linux/mmc/host.h>

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

#include <mach/audio.h>
#include <mach/usb-regs.h>
#include <plat/pxa27x_keypad.h>

#include <plat/usb.h>

#include "devices.h"
#include "generic.h"

#include "panel_settings.h"


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
static struct regulator_consumer_supply regulator_supply[PM8607_ID_RG_MAX];
static struct regulator_init_data regulator_data[PM8607_ID_RG_MAX];

static int regulator_index[] = {
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

#define REG_SUPPLY_INIT(_id, _name, _dev_name)		\
{							\
	int _i = _id;				\
	regulator_supply[_i].supply		= _name;		\
	regulator_supply[_i].dev_name	= _dev_name;	\
}

#define REG_INIT(_id, _name, _min, _max, _always, _boot)\
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
		&regulator_supply[PM8607_ID_##_name];	\
	regulator_data[_i].driver_data	=	\
		&regulator_index[PM8607_ID_##_name];	\
}

static struct pm860x_rtc_pdata rtc = {
	.vrtc           = 1,
	.rtc_wakeup     = 0,
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

extern struct pxa95x_freq_mach_info freq_mach_info;
static void regulator_init(void)
{
	int i = 0;

	if (PXA95x_USE_POWER_I2C != freq_mach_info.flags) {
		REG_SUPPLY_INIT(PM8607_ID_BUCK1, "v_buck1", "pxa95x-freq");
		REG_INIT(i++, BUCK1, 1000000, 1500000, 1, 1);
	}

	/* for hdmi */
	REG_SUPPLY_INIT(PM8607_ID_LDO12, "v_ihdmi", NULL);
	REG_INIT(i++, LDO12, 1200000, 3300000, 0, 0);

	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		REG_SUPPLY_INIT(PM8607_ID_LDO1, "v_gps", NULL);
		REG_SUPPLY_INIT(PM8607_ID_LDO9, "v_cam", NULL);
		REG_SUPPLY_INIT(PM8607_ID_LDO13, "vmmc", "sdhci-pxa.1");

		REG_INIT(i++, LDO1, 1200000, 3300000, 0, 0);
		REG_INIT(i++, LDO9, 1800000, 3300000, 0, 0);
		REG_INIT(i++, LDO13, 1800000, 3300000, 0, 0);
		printk(KERN_INFO "%s: select saarC NEVO ldo map\n", __func__);
	break;
	default:
		printk(KERN_ERR "%s: The board type is not defined!\n ", __func__);
		BUG();
	}

	pm8607_info.num_regulators = i;
}

static struct i2c_board_info i2c1_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &pm8607_info,
		.irq            = IRQ_PMIC_INT,
	},
};

static struct clk *clk_tout_s0;

/* specific 8787 power on/off setting for SAARB */
static void wifi_set_power(unsigned int on)
{
	unsigned long wlan_pd_mfp = 0;
	int gpio_power_down = mfp_to_gpio(MFP_PIN_GPIO70);

	wlan_pd_mfp = pxa3xx_mfp_read(gpio_power_down);

	if (on) {
		/* set wlan_pd pin to output high in low power
			mode to ensure 8787 is not power off in low power mode*/
		wlan_pd_mfp |= 0x100;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* enable 32KHz TOUT */
		clk_enable(clk_tout_s0);
		}
	else {
		/*set wlan_pd pin to output low in low power
			mode to save power in low power mode */
		wlan_pd_mfp &= ~0x100;
		pxa3xx_mfp_write(gpio_power_down, wlan_pd_mfp & 0xffff);

		/* disable 32KHz TOUT */
		clk_disable(clk_tout_s0);
	}
}

#if defined(CONFIG_MMC_SDHCI_PXAV2_TAVOR)
static struct sdhci_pxa_platdata mci0_platform_data = {
	.flags	= PXA_FLAG_CARD_PERMANENT |
				PXA_FLAG_SD_8_BIT_CAPABLE_SLOT |
				PXA_FLAG_ACITVE_IN_SUSPEND |
				PXA_FLAG_ENABLE_CLOCK_GATING,
};

static struct sdhci_pxa_platdata mci1_platform_data = {
	.flags = PXA_FLAG_ACITVE_IN_SUSPEND |
				PXA_FLAG_ENABLE_CLOCK_GATING,
	.ext_cd_gpio = mfp_to_gpio(MFP_PIN_GPIO123),
	.ext_cd_gpio_invert = 1,
};

static struct sdhci_pxa_platdata mci2_platform_data = {
	.flags  = PXA_FLAG_CARD_PERMANENT |
				PXA_FLAG_HS_NEED_WAKELOCK,
	.pm_caps = MMC_PM_KEEP_POWER |
				MMC_PM_IRQ_ALWAYS_ON,
};

static void __init init_mmc(void)
{
	/*add emmc only, need to add sdcard and sdio later*/
	pxa95x_set_mci_info(0, &mci0_platform_data);
	pxa95x_set_mci_info(1, &mci1_platform_data);
#ifdef CONFIG_SD8XXX_RFKILL
	add_sd8x_rfkill_device(mfp_to_gpio(MFP_PIN_GPIO70),
			mfp_to_gpio(MFP_PIN_GPIO99),
			&mci2_platform_data.pmmc,
			wifi_set_power);

	clk_tout_s0 = clk_get(NULL, "CLK_TOUT_S0");
	if (IS_ERR(clk_tout_s0))
		pr_err("init_mmc: unable to get tout_s0 clock");

	pxa95x_set_mci_info(2, &mci2_platform_data);
#endif
}
#endif

static struct cwmi_platform_data cwmi_acc_data = {
	.set_power = NULL,
	.axes = {
		1, 0, 0,
		0, -1, 0,
		0, 0, 1},
};

static struct cwmi_platform_data cwmi_mag_data = {
	.set_power = NULL,
	.axes = {
		-1, 0, 0,
		0, 1, 0,
		0, 0, -1},
};

static struct cwgd_platform_data cwgd_plat_data = {
	.set_power = NULL,
	.axes = {
		-1, 0, 0,
		0, 1, 0,
		0, 0, -1},
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

static struct i2c_board_info i2c2_info[] = {
#if defined(CONFIG_TOUCHSCREEN_SSD2531)
	{
	 .type = "ssd2531_ts",
	 .addr = 0x5c,
	 .platform_data = ssd2531_ts_pins,
	 },
#endif
};


static struct i2c_board_info i2c3_info[] = {
#if defined(CONFIG_SENSORS_CWMI)
	{
	 .type = "cwmi_acc",
	 .addr = 0x19,		/* Write addr 0x32, read addr 0x33 */
	 .irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO11)),
	 .platform_data = &cwmi_acc_data,
	 },

	{
	 .type = "cwmi_mag",
	 .addr = 0x1e,		/*write addr 0x3C, read addr 0x3D */
	 .irq = 0,/*gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO10)),*/
	 .platform_data = &cwmi_mag_data,
	 },
#endif
#if defined(CONFIG_SENSORS_CWGD)
	{
	 .type = "cwgd",
	 .addr = 0x69,		/*R1 mount(High) write=0xD2, Read=0xD3 */
	 .irq = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO9)),
	 .platform_data = &cwgd_plat_data,
	 },
#endif
#if defined(CONFIG_HDMI_ADV7533)
	{
		.type		= "adv7533-packet",
		.addr		= 0x38,
	},
	{
		.type		= "adv7533-main",
		.addr		= 0x39,
	},
	{
		.type		= "adv7533-cec-dsi",
		.addr		= 0x2c,
	},
	{
		.type		= "adv7533-edid",
		.addr		= 0x3f,
		.platform_data = adv7533_hdmi_reset,
	},
#endif

#if defined(CONFIG_C_TEC_OPTIC_TP)
	{
		.type	= "ctec_optic_tp",
		.addr	= 0x33,
		.irq	= gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO100)),
	},
#endif
};

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

static struct mv_usb_addon_irq pmic_vbus = {
	.irq	= IRQ_BOARD_START + PM8607_IRQ_CHG,
	.poll	= read_vbus_val,
};

static struct mv_usb_platform_data pxa9xx_usb_pdata = {
	.clknum		= 3,
	.clkname	= pxa9xx_usb_clock_name,
	.vbus		= &pmic_vbus,
	.mode		= MV_USB_MODE_OTG,
	.phy_init	= pxa9xx_usb_phy_init,
	.set_vbus	= NULL,
};
#endif

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_HOME),
	KEY(0, 1, KEY_SEND),
	KEY(0, 2, KEY_CAMERA),
	KEY(1, 0, KEY_WWW),
	KEY(1, 1, KEY_BACK),
	KEY(1, 2, KEY_CAMERA),
	KEY(2, 0, KEY_END),
	KEY(2, 1, KEY_VOLUMEUP),
	KEY(2, 2, KEY_VOLUMEUP),/*not exist*/
	KEY(3, 0, KEY_MENU),
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
#endif /* CONFIG_KEYBOARD_PXA27x || CONFIG_KEYBOARD_PXA27x_MODULE */

#if defined(CONFIG_VIDEO_PXA955)
static struct regulator *vcamera = NULL;
static int camera0_power(struct device *dev, int flag)
{
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO25);

	if (gpio_request(gpio_pin, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin);
		return -EIO;
	}

	/* Initialize AF_VCC */
	if (vcamera == NULL) {
		vcamera = regulator_get(NULL, "v_cam");
		if (IS_ERR(vcamera)) {
			return -ENODEV;
		}
	}

	if (flag) {
		gpio_direction_output(gpio_pin, 0);	/* enable */
		msleep(1);
		regulator_enable(vcamera);
	} else {
		regulator_disable(vcamera);
		gpio_direction_output(gpio_pin, 1);	/* disable */
	}

	gpio_free(gpio_pin);
	return 0;
}

static int camera1_power(struct device *dev, int flag)
{
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO26);

	if (gpio_request(gpio_pin, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin);
		return -EIO;
	}

	if (gpio_request(gpio_pin, "CAM_EANBLE_LOW_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin);
		return -EIO;
	}

	if (flag) {
		gpio_direction_output(gpio_pin, 0);
		msleep(1);
	} else {
		gpio_direction_output(gpio_pin, 1);
	}

	gpio_free(gpio_pin);
	return 0;
}

static struct i2c_board_info camera_i2c[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	}, {
		I2C_BOARD_INFO("ov7690", 0x21),
	},
};

static struct sensor_platform_data camera_sensor[] = {
	{	/* OV5642 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.intrfc_id	= 0,
		.bridge		= 1,
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "TRULY",
		.board_name	= "saarb",
	}, {	/* OV7690 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_FRONT | SENSOR_RES_LOW,
		.interface	= 0,
		.intrfc_id	= 0,
		.bridge		= 1,
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "N/A",
		.board_name	= "saarb",
	},
};

static struct soc_camera_link iclink[] = {
	{
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[0],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "ov5642",
		/* Configure this flag according to hardware connection */
		/* Both of the 2 MIPI lanes are connected to pxa95x on Saarb */
		.priv			= &camera_sensor[0],
	}, {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[1],
		.i2c_adapter_id		= 1,
		.power = camera1_power,
		.module_name		= "ov7690",
		/* ov7690 is using ov5642 as bridge, so config is the same*/
		.priv			= &camera_sensor[1],
	},
};

static struct platform_device camera[] = {
	{
		.name	= "soc-camera-pdrv",
		.id	= 0,
		.dev	= {
			.platform_data = &iclink[0],
		},
	}, {
		.name	= "soc-camera-pdrv",
		.id	= 1,
		.dev	= {
			.platform_data = &iclink[1],
		},
	},
};

static struct pxa95x_csi_dev csidev[] = {
	{
		.irq_num	= 71,
		.reg_start	= 0x50020000,
	},
	/*TODO: if there is 2 csi controller, add its info here*/
	/*
	{
		.irq_num	= 71,
		.reg_base	= 0x50020000,
	},
	*/
};

/*
* TODO: combine csi controller and camera controller, now we only have one
* csi controller on pxa95x, and two camera controllers
*/
static struct pxa95x_cam_pdata cam_pdata[] = {
	{
		.csidev		= &csidev[0],
	},
	/*TODO: if there is 2 sci controller, add its info here*/
	/*
	{
		.mclk_mhz	= 26,
		.csidev		= &csidev[1],
	},
	*/
};

#endif

static void __init init_cam(void)
{
#if defined(CONFIG_VIDEO_PXA955)

#if defined(CONFIG_SOC_CAMERA_OV5642)
	platform_device_register(&camera[0]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV7690)
	platform_device_register(&camera[1]);
#endif

	pxa95x_device_cam0.dev.platform_data = &cam_pdata[0];
	platform_device_register(&pxa95x_device_cam0);

	/* TODO: add sencond camera controller */
	/*pxa95x_device_cam1.dev.platform_data = &cam_pdata[1];*/
	/*platform_device_register(&pxa95x_device_cam1);*/
#endif
}

#if defined(CONFIG_FB_PXA95x)

#if defined(CONFIG_MV_IHDMI)
static int mv_ihdmi_format = 4;

static void hdtx_power(bool en)
{
	struct regulator *v_ldo;
	int pin1 = mfp_to_gpio(MFP_PIN_GPIO13);

	v_ldo = regulator_get(NULL, "v_ihdmi");
	if (IS_ERR(v_ldo)) {
		printk(KERN_ERR "hdmi: fail to get ldo handle!\n");
		return;
	}

	if (en) {
		regulator_set_voltage(v_ldo, 3300000, 3300000);
		regulator_enable(v_ldo);
		printk(KERN_INFO "hdmi: turn on ldo12 to 3.3v.\n");
	} else {
		printk(KERN_INFO "hdmi: turn off LDO\n");
		regulator_disable(v_ldo);
	}

	if (gpio_request(pin1, "hdmi-cp")) {
		printk(KERN_ERR "hdmi: hdmi-cp failed!\n");
		gpio_free(pin1);
		regulator_put(v_ldo);
		return;
	}

	gpio_direction_output(pin1, en);
	msleep(10);

	gpio_free(pin1);
	printk(KERN_INFO "hdmi: turn charge pump 5V to %d\n", en);

	regulator_put(v_ldo);
}

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
	.panel_power			= hdtx_power,
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
	pxa_register_device(&pxa978_device_ihdmi, &mv_ihdmi_format);
	ihdmi_ovly_info.modes = &video_modes_ihdmi[mv_ihdmi_format-1];
	set_pxa95x_fb_ovly_info(&ihdmi_ovly_info, 1);
#elif defined(CONFIG_HDMI_ADV7533)
	set_pxa95x_fb_ovly_info(&adv7533_hdmi_ovly_info, 1);
#endif
}
#endif

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

static int query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
{
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
		mfp_c = GPIO80_MMC3_DAT1 | MFP_LPM_EDGE_BOTH;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret |= PXA95x_PM_WE_MMC3;
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

static void __init init(void)
{
	regulator_init();

	set_abu_init_func(abu_mfp_init);
	set_ssp_init_func(ssp3_mfp_init);

	platform_device_add_data(&pxa95x_device_i2c1, &i2c1_pdata,
				 sizeof(i2c1_pdata));
	platform_device_add_data(&pxa95x_device_i2c2, &i2c2_pdata,
				 sizeof(i2c2_pdata));
	platform_device_add_data(&pxa95x_device_i2c3, &i2c3_pdata,
				 sizeof(i2c3_pdata));

	platform_add_devices(ARRAY_AND_SIZE(devices));
	i2c_register_board_info(0, ARRAY_AND_SIZE(i2c1_info));
	i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info));
	i2c_register_board_info(2, ARRAY_AND_SIZE(i2c3_info));

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
	pxa_set_keypad_info(&keypad_info);
#endif

	init_cam();

#if defined(CONFIG_FB_PXA95x)
	init_lcd();
#endif

#if defined(CONFIG_MMC_SDHCI_PXAV2_TAVOR)
	init_mmc();
#endif

	/* Init boot flash - sync mode in case of ONENAND */
	pxa_boot_flash_init(1);

#ifdef CONFIG_PM
	pxa95x_wakeup_register(&wakeup_ops);
#endif

#ifdef CONFIG_USB_PXA_U2O
	pxa9xx_device_u2o.dev.platform_data = (void *)&pxa9xx_usb_pdata;
	platform_device_register(&pxa9xx_device_u2o);
#endif
}

MACHINE_START(NEVOSAARC, "PXA978 Handheld Platform (aka SAAR C2)")
	.map_io		= pxa_map_io,
	.nr_irqs	= NEVOSAARC_NR_IRQS,
	.init_irq	= pxa95x_init_irq,
	.handle_irq	= pxa95x_handle_irq_intc,
	.timer		= &pxa_timer,
	.reserve	= pxa95x_mem_reserve,
	.init_machine	= init,
MACHINE_END
