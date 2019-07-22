/*
 *  linux/arch/arm/mach-pxa/saarb.c
 *
 *  Support for the Marvell Handheld Platform (aka SAARB)
 *
 *  Copyright (C) 2007-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/pxa95x-i2c.h>
#include <linux/mfd/88pm860x.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/regulator/machine.h>
#include <linux/sd8x_rfkill.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa930.h>
#include <mach/gpio.h>
#include <mach/pxa95xfb.h>
#include <mach/pxa95x_dvfm.h>

#include <mach/camera.h>
#include <media/soc_camera.h>
#include <linux/switch.h>

#include <mach/pxa3xx-regs.h>
#include <mach/usb-regs.h>
#include <mach/audio.h>
#include <plat/pxa27x_keypad.h>

#include <plat/usb.h>
#include "devices.h"
#include "generic.h"

#include "panel_settings.h"

#define SAARB_NR_IRQS	(IRQ_BOARD_START + 40)

static struct pm860x_touch_pdata touch = {
	.gpadc_prebias	= 1,
	.slot_cycle	= 1,
	.tsi_prebias	= 6,
	.pen_prebias	= 16,
	.pen_prechg	= 2,
	.res_x		= 300,
};

static struct pm860x_backlight_pdata backlight[] = {
	{
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(8),
		.flags	= PM8606_BACKLIGHT1,
	},
	{
		/*keypad backlight data*/
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(8),
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

static int cm3601_request_resource(unsigned char gpio_num, char *name)
{
	int ret = 0;
	ret = gpio_request(mfp_to_gpio(gpio_num), name);
	if (ret) {
		printk(KERN_ERR "%s: can't request GPIO %d.\n", __func__,gpio_num);
		return -1;
	}
	return ret;
}

static void cm3601_release_resource(unsigned char gpio_num){
	gpio_free(gpio_num);
}

static struct pm860x_cm3601_pdata cm3601_platform_info = {
	.gpio_cm3601_enable =	MFP_PIN_GPIO51,
	.gpio_ps_output =	MFP_PIN_GPIO50,
	.request_source	=	cm3601_request_resource,
	.release_source	=	cm3601_release_resource,
};

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
	.touch		= &touch,
	.backlight	= &backlight[0],
	.led		= &led[0],
	.power		= &power,
	.regulator	= regulator_data,
	.rtc		= &rtc,
#if defined(CONFIG_SENSORS_CM3601)
	.cm3601		= &cm3601_platform_info,
#endif
	.headset	= &headset_platform_info,
	.companion_addr	= 0x10,
	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,

	.i2c_port	= GI2C_PORT,
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

	switch (get_board_id()) {
	case OBM_SAAR_B_MG1_C0_V12_BOARD:
		REG_SUPPLY_INIT(PM8607_ID_LDO12, "v_hdmi", "1-003b");
		REG_SUPPLY_INIT(PM8607_ID_LDO13, "v_gps", NULL);
		REG_SUPPLY_INIT(PM8607_ID_LDO14, "vmmc", "sdhci-pxa.1");

		REG_INIT(i++, LDO12, 1200000, 3300000, 0, 0);
		REG_INIT(i++, LDO13, 1200000, 3300000, 0, 0);
		REG_INIT(i++, LDO14, 1800000, 3300000, 0, 0);

		printk(KERN_INFO "%s: select saarb v12 ldo map\n", __func__);
	break;
	case OBM_SAAR_B_MG2_A0_V13_BOARD:
	case OBM_SAAR_B_MG2_A0_V14_BOARD:
	case OBM_SAAR_B_MG2_B0_V15_BOARD:
	case OBM_SAAR_B_MG2_C0_V26_BOARD:
		REG_SUPPLY_INIT(PM8607_ID_LDO9, "vmmc", "sdhci-pxa.1");
		REG_SUPPLY_INIT(PM8607_ID_LDO13, "v_cam", NULL);
		REG_SUPPLY_INIT(PM8607_ID_LDO14, "v_gps", NULL);

		REG_INIT(i++, LDO9, 1800000, 3300000, 0, 0);
		REG_INIT(i++, LDO13, 1200000, 3300000, 0, 0);
		REG_INIT(i++, LDO14, 1800000, 3300000, 0, 0);
		printk(KERN_INFO "%s: select saarb v13 ldo map\n", __func__);
	break;
	default:
		printk(KERN_ERR "%s: The board type is not defined!\n ", __func__);
		BUG();
	}

	pm8607_info.num_regulators = i;
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
	.use_pio        = 0,
	.flags          = PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
};

static struct i2c_board_info i2c1_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &pm8607_info,
		.irq            = IRQ_PMIC_INT,
	},
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
	KEY(1, 3, KEY_0), KEY(0, 0, KEY_1), KEY(1, 0, KEY_2), KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4), KEY(1, 1, KEY_5), KEY(2, 1, KEY_6), KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8), KEY(2, 2, KEY_9),

	KEY(0, 3, KEY_KPASTERISK), 	/* * */
	KEY(2, 3, KEY_KPDOT),   	/* # */

	KEY(4, 0, KEY_HOME),
	KEY(3, 3, KEY_END),
	KEY(4, 1, KEY_BACK),

	KEY(3, 2, KEY_SEND),

	KEY(4, 4, KEY_SELECT),    /* volume rocker push */
	KEY(3, 4, KEY_VOLUMEUP),
	KEY(2, 4, KEY_VOLUMEDOWN),

	KEY(3, 0, KEY_F22),	/* soft1 */
	KEY(3, 1, KEY_F23),	/* soft2 */

	KEY(1, 4, KEY_CAMERA),      /* camera full push */
	KEY(0, 4, KEY_ZOOM),		/* camera half push */

	KEY(4, 3, KEY_WWW),		/* surf button */
	KEY(4, 2, KEY_OK),		/* ok button */

	KEY(0, 5, KEY_ENTER),       /* joystick push */
	KEY(4, 5, KEY_LEFT),
	KEY(3, 5, KEY_RIGHT),
	KEY(2, 5, KEY_UP),
	KEY(1, 5, KEY_DOWN),
};

static struct pxa27x_keypad_platform_data keypad_info = {
	.matrix_key_rows	= 5,
	.matrix_key_cols	= 6,
	.matrix_key_map		= matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(matrix_key_map),
	.debounce_interval	= 30,
};
#endif /* CONFIG_KEYBOARD_PXA27x || CONFIG_KEYBOARD_PXA27x_MODULE */

#if defined(CONFIG_VIDEO_PXA955)
/* Camera sensor PowerDowN pins */
static int pwd_main, pwd_sub;

static int camera0_power(struct device *dev, int flag)
{
	if (flag) {
		gpio_direction_output(pwd_main, 0);	/* enable */
		msleep(1);
	} else {
		gpio_direction_output(pwd_main, 1);	/* disable */
	}
	return 0;
}

static int camera1_power(struct device *dev, int flag)
{

	if (flag) {
		gpio_direction_output(pwd_main, 0);
		gpio_direction_output(pwd_sub, 0);
		msleep(1);
	} else {
		gpio_direction_output(pwd_main, 1);
		gpio_direction_output(pwd_sub, 1);
	}
	return 0;
}

static struct i2c_board_info camera_i2c[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3C),
	}, {
		I2C_BOARD_INFO("ov7690", 0x21),
	},
};

static struct pxa95x_csi_dev csidev[] = {
	{
		.irq_num	= 71,
		.reg_start	= 0x50020000,
	},
};

static struct sensor_platform_data camera_sensor[] = {
	{	/* OV5642 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa95x on SaarB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "TRULY",
		.board_name	= "tavor",
	},
	{	/* OV7690 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_FRONT | SENSOR_RES_LOW,
		/* OV7690 use OV5642 as bridge, so MIPI config is the same */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],
		.af_cap		= 0,
		.mclk_mhz	= 26,
		.vendor_info	= "N/A",
		.board_name	= "tavor",
	},
};

static struct soc_camera_link iclink[] = {
	{
		.bus_id			= 0, /* Must match with the SCI ID */
		.board_info		= &camera_i2c[0],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "ov5642",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[0],
	}, {
		.bus_id			= 0, /* Must match with the SCI ID */
		.board_info		= &camera_i2c[1],
		.i2c_adapter_id		= 1,
		.power = camera1_power,
		.module_name		= "ov7690",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
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
#endif

static void __init init_cam(void)
{
#if defined(CONFIG_VIDEO_PXA955)
	pwd_main = mfp_to_gpio(MFP_PIN_GPIO81);
	pwd_sub = mfp_to_gpio(MFP_PIN_GPIO14);

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

#if defined(CONFIG_SOC_CAMERA_OV5642)
	platform_device_register(&camera[0]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV7690)
	platform_device_register(&camera[1]);
#endif

	platform_device_register(&pxa95x_device_cam0);

	/* TODO: add sencond camera controller */
	/*pxa95x_device_cam1.dev.platform_data = &cam_pdata[1];*/
	/*platform_device_register(&pxa95x_device_cam1);*/
#endif
}

#if defined(CONFIG_FB_PXA95x)
static void panel_reset(void)
{
	int reset_pin;
	int err;

	reset_pin = mfp_to_gpio(MFP_PIN_GPIO20);
	err = gpio_request(reset_pin, "DSI Reset");
	if (err) {
		gpio_free(reset_pin);
		pr_err("Request GPIO failed, gpio: %d return :%d\n",
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

static void panel_power(int on)
{
	if (get_board_id() >= OBM_SAAR_B_MG2_B0_V15_BOARD)
		panel_power_trulywvga(1, on);
	else
		panel_power_tc3587(1, on);
}

static struct pxa95xfb_mach_info lcd_info __initdata = {
	.id                     = "Base",
	.modes                  = video_modes_tc3587,
	.num_modes              = ARRAY_SIZE(video_modes_tc3587),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_16_RGB565,
	.panel_type             = LCD_Controller_Active,
	.window                 = 0,
	.mixer_id               = 1,
	.zorder                 = 1,
	.converter              = LCD_M2DSI1,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power            = panel_power,
	.reset	                = panel_reset,
	.invert_pixclock        = 1,
};

static struct pxa95xfb_mach_info lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = video_modes_tc3587,
	.num_modes              = ARRAY_SIZE(video_modes_tc3587),
	.pix_fmt_in             = PIX_FMTIN_RGB_16,
	.pix_fmt_out            = PIX_FMTOUT_16_RGB565,
	.panel_type             = LCD_Controller_Active,
	.window                 = 4,
	.mixer_id               = 1,
	.zorder                 = 0,
	.converter              = LCD_M2DSI1,
	.output                 = OUTPUT_PANEL,
	.active                 = 1,
	.panel_power            = panel_power,
	.reset                  = panel_reset,
	.invert_pixclock	= 1,
};

static struct pxa95xfb_mach_info lcd_info_wvga /*__initdata*/ = {
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

static struct pxa95xfb_mach_info lcd_ovly_info_wvga /*__initdata*/ = {
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

static struct pxa95xfb_mach_info hdmi_ovly_info __initdata = {
	.id                     = "HDMI-Ovly",
	.modes                  = video_modes_si9226,
	.num_modes              = ARRAY_SIZE(video_modes_si9226),
	.pix_fmt_in             = PIX_FMTIN_YUV420,
	.pix_fmt_out            = PIX_FMTOUT_24_RGB888,
	.panel_type             = LCD_Controller_Active,
	/*as hdmi-ovly use same win4 with lcd-ovly, they should not open at the same time*/
	.window                 = 4,
	.mixer_id               = 0,
	.zorder                 = 1,
	.converter              = LCD_M2PARALELL_CONVERTER,
	.output                 = OUTPUT_HDMI,
	.active                 = 1,
	.invert_pixclock        = 1,
};


static void __init init_lcd(void)
{
	if (get_board_id() == OBM_SAAR_B_MG2_B0_V15_BOARD
		|| get_board_id() == OBM_SAAR_B_MG2_C0_V26_BOARD) {
		set_pxa95x_fb_info(&lcd_info_wvga);
		set_pxa95x_fb_ovly_info(&lcd_ovly_info_wvga, 0);
	} else {
		set_pxa95x_fb_info(&lcd_info);
		set_pxa95x_fb_ovly_info(&lcd_ovly_info, 0);
#ifdef CONFIG_HDMI_SI9226
		set_pxa95x_fb_ovly_info(&hdmi_ovly_info, 1);
#endif
	}
}
#endif

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
		if (pxa95x_query_gwsr(47))
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
				GPIO8_KP_MKIN_4 | MFP_LPM_EDGE_BOTH,
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
				GPIO8_KP_MKIN_4 | MFP_LPM_EDGE_NONE,
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
			mfp_c = GPIO47_GPIO | MFP_LPM_EDGE_BOTH;
			pxa3xx_mfp_config(&mfp_c, 1);
			ret |= PXA95x_PM_WE_GENERIC(13);
		}
	} else {
		mfp_c = GPIO47_GPIO | MFP_LPM_EDGE_NONE;
		pxa3xx_mfp_config(&mfp_c, 1);
	}

	return ret;
}

static int mmc3_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	mfp_cfg_t mfp_c;
	if (enable) {
		mfp_c = GPIO88_MMC3_DAT1 | MFP_LPM_EDGE_BOTH;
		pxa3xx_mfp_config(&mfp_c, 1);
		ret |= PXA95x_PM_WE_MMC3;
	} else {
		mfp_c = GPIO88_MMC3_DAT1 | MFP_LPM_EDGE_NONE;
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
			m = GPIO53_UART1_RXD | MFP_LPM_EDGE_FALL;
			pxa3xx_mfp_config(&m, 1);
			ret |= PXA95x_PM_WE_GENERIC(9);
		}
		if (src.bits.uart2) {
			m = GPIO45_UART3_RXD | MFP_LPM_EDGE_FALL;
			pxa3xx_mfp_config(&m, 1);
			/* note: on pxa930, uart2 use this bit */
		}
	} else {
		if (src.bits.uart1) {
			m = GPIO53_UART1_RXD | MFP_LPM_EDGE_NONE;
			pxa3xx_mfp_config(&m, 1);
		}
		if (src.bits.uart2) {
			m = GPIO45_UART3_RXD | MFP_LPM_EDGE_NONE;
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

#if defined(CONFIG_HDMI_SI9226)
static void SI9226_hdmi_power(struct device *dev, int on)
{
	static struct regulator *vhdmi;
	static int vhdmi_inited;

	if (get_board_id() < OBM_SAAR_B_MG2_A0_V13_BOARD && !vhdmi_inited) {
		vhdmi = regulator_get(dev, "v_hdmi");
		if (!IS_ERR(vhdmi))
			vhdmi_inited = 1;
		else
			pr_debug("hdmi: failed to get regulator\n");
	}

	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO43), "hdmi reset"))
		pr_debug("hdmi gpio_request: failed!\n");

	pr_info("SI9226 hdmi power %s\n", on ? "on" : "off");
	if (on) {
		if (get_board_id() < OBM_SAAR_B_MG2_A0_V13_BOARD) {
			if (!IS_ERR(vhdmi))
				regulator_enable(vhdmi);
			else
				pr_debug("hdmi: regulator is not correct\n");
			msleep(20);
		}
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO43), 0);
		msleep(1);
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO43), 1);
	} else {
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO43), 0);

		if (get_board_id() < OBM_SAAR_B_MG2_A0_V13_BOARD) {
			msleep(20);
			if (!IS_ERR(vhdmi))
				regulator_disable(vhdmi);
			else
				pr_debug("hdmi: regulator is not correct\n");
		}
	}
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO43));
	msleep(10);
}

#endif

static struct clk *clk_tout_s0;

/* specific 8787 power on/off setting for SAARB */
static void wifi_set_power(unsigned int on)
{
	unsigned long wlan_pd_mfp = 0;
	int gpio_power_down = mfp_to_gpio(MFP_PIN_GPIO77);

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
	.flags = PXA_FLAG_ENABLE_CLOCK_GATING,
	.ext_cd_gpio = mfp_to_gpio(MFP_PIN_GPIO47),
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
	add_sd8x_rfkill_device(mfp_to_gpio(MFP_PIN_GPIO77),
			mfp_to_gpio(MFP_PIN_GPIO67),
			&mci2_platform_data.pmmc,
			wifi_set_power);

	clk_tout_s0 = clk_get(NULL, "CLK_TOUT_S0");
	if (IS_ERR(clk_tout_s0))
		pr_err("init_mmc: unable to get tout_s0 clock");

	pxa95x_set_mci_info(2, &mci2_platform_data);
#endif
}

#endif

#if defined(CONFIG_SENSORS_LIS331DL)
static unsigned long lis33ldl_min_delay = 20;
#endif

/*{ TOUCHSCREEN_INT_GPIO, TOUCHSCREEN_RESET_GPIO };*/
static int ssd2531_ts_pins[2];

static struct i2c_board_info i2c2_info[] = {

#if defined(CONFIG_SENSORS_LIS331DL)
	{
		.type		= "lis331dl",
		.addr		= 0x1c,
		.platform_data	= &lis33ldl_min_delay,
	},
#endif

#if defined(CONFIG_SENSORS_YAS529)
	{
		.type		= "yas529",
		.addr		= 0x2e,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_SSD2531)
	{
	 .type = "ssd2531_ts",
	 .addr = 0x5c,
	 .platform_data = ssd2531_ts_pins,
	 },
#endif

#if defined(CONFIG_HDMI_SI9226)
	{
		.type		= "SI9226",
		.addr		= 0x3B,
		.platform_data = SI9226_hdmi_power,
	},
	{
		.type		= "SI9226-ctrl",
		.addr		= 0x66,
	},
#endif
};

static void set_i2c_touch(void)
{
	if (get_board_id() == OBM_SAAR_B_MG2_B0_V15_BOARD) {
		pm8607_info.touch = NULL;
		ssd2531_ts_pins[0] = MFP_PIN_GPIO144;
		ssd2531_ts_pins[1] = MFP_PIN_GPIO135;
	} else if (get_board_id() == OBM_SAAR_B_MG2_C0_V26_BOARD) {
		pm8607_info.touch = NULL;
		ssd2531_ts_pins[0] = MFP_PIN_GPIO154;
		ssd2531_ts_pins[1] = MFP_PIN_GPIO143;
	}
};

static mfp_cfg_t pxa95x_abu_mfp_cfg[] = {
	/* ABU of MG1 */
	GPIO68_ABU_RXD,
	GPIO69_ABU_TXD,
	GPIO70_GPIO,	/* no use for ABU/SSI, and configure GPIO70 to AF0 to save power when using ABU (~0.5mA) */
	GPIO71_ABU_FRM,
	GPIO72_ABU_CLK,
};

static mfp_cfg_t pxa95x_bssp2_mfp_cfg[] = {
	/* BSSP2 of MG1 */
	GPIO68_BSSP2_RXD,
	GPIO69_BSSP2_TXD,
	GPIO70_BSSP2_SYSCLK,
	GPIO71_BSSP2_FRM,
	GPIO72_BSSP2_CLK,
};

static mfp_cfg_t bssp3_mfp_cfg[] = {
	/* BSSP3 of MG1*/
	GPIO63_BSSP3_CLK,
	GPIO64_BSSP3_FRM,
	GPIO65_BSSP3_TXD,
	GPIO66_BSSP3_RXD,
};

static mfp_cfg_t gssp1_mfp_cfg[] = {
	/* BSSP3 of MG1*/
	GPIO63_GSSP1_CLK,
	GPIO64_GSSP1_FRM,
	GPIO65_GSSP1_TXD,
	GPIO66_GSSP1_RXD,
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

static void __init saarb_init(void)
{
	regulator_init();
	set_i2c_touch();

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

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
	pxa_set_keypad_info(&keypad_info);
#endif

#if defined(CONFIG_SENSORS_ORIENTATION)
	platform_device_register_simple("orientation", 0, NULL, 0);
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

MACHINE_START(SAARB, "PXA968")
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.nr_irqs	= SAARB_NR_IRQS,
	.init_irq       = pxa95x_init_irq,
	.handle_irq     = pxa95x_handle_irq_intc,
	.timer          = &pxa_timer,
	.reserve	= pxa95x_mem_reserve,
	.init_machine   = saarb_init,
MACHINE_END

