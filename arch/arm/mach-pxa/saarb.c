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
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/pxa95x-i2c.h>
#include <linux/mfd/88pm860x.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_data/pxa_sdhci.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa930.h>
#include <mach/gpio.h>
#include <mach/pxa95xfb.h>
#include <mach/soc_vmeta.h>
#include <mach/camera.h>
#include <media/soc_camera.h>
#include <linux/switch.h>

#include <plat/pxa27x_keypad.h>
#include <plat/pmem.h>

#include "devices.h"
#include "generic.h"

#include "panel_settings.h"

#define SAARB_NR_IRQS	(IRQ_BOARD_START + 40)

static struct pm860x_touch_pdata saarb_touch = {
	.gpadc_prebias	= 1,
	.slot_cycle	= 1,
	.tsi_prebias	= 6,
	.pen_prebias	= 16,
	.pen_prechg	= 2,
	.res_x		= 300,
};

static struct pm860x_backlight_pdata saarb_backlight[] = {
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

static struct pm860x_led_pdata saarb_led[] = {
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
	.gpio_en	= MFP_PIN_GPIO51,
	.gpio_out	= MFP_PIN_GPIO50,
	.request_source	= cm3601_request_resource,
	.release_source	= cm3601_release_resource,
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
static struct pm860x_platform_data saarb_pm8607_info = {
	.touch		= &saarb_touch,
	.backlight	= &saarb_backlight[0],
	.led		= &saarb_led[0],
	.power		= &power,
#if defined(CONFIG_SENSORS_CM3601)
	.cm3601		= &cm3601_platform_info,
#endif
	.headset	= &headset_platform_info,
	.companion_addr	= 0x10,
	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,

	.i2c_port	= GI2C_PORT,
	.num_leds	= ARRAY_SIZE(saarb_led),
	.num_backlights	= ARRAY_SIZE(saarb_backlight),
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
	.use_pio        = 0,
	.flags          = PXA_I2C_FAST_MODE | PXA_I2C_USING_FIFO_PIO_MODE,
};

static struct i2c_board_info saarb_i2c_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &saarb_pm8607_info,
		.irq		= gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO83)),
	},
};

#if defined(CONFIG_UIO_VMETA)
static struct vmeta_plat_data vmeta_plat_data = {
	.bus_irq_handler = pxa95x_vmeta_bus_irq_handler,
	.set_dvfm_constraint = pxa95x_vmeta_set_dvfm_constraint,
	.unset_dvfm_constraint = pxa95x_vmeta_unset_dvfm_constraint,
	.init_dvfm_constraint = pxa95x_vmeta_init_dvfm_constraint,
	.clean_dvfm_constraint = pxa95x_vmeta_clean_dvfm_constraint,
	.axi_clk_available = 1,
	.decrease_core_freq = pxa95x_vmeta_decrease_core_freq,
	.increase_core_freq = pxa95x_vmeta_increase_core_freq,
};

static void __init init_vmeta(void)
{
	pxa95x_set_vmeta_info(&vmeta_plat_data);
}
#endif /*(CONFIG_UIO_VMETA)*/

static struct platform_device *devices[] __initdata = {
	&pxa95x_device_i2c1,
	&pxa95x_device_i2c2,
	&pxa95x_device_i2c3,
};

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
static int camera0_power(struct device *dev, int flag)
{
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO81);

	if (gpio_request(gpio_pin, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin);
		return -EIO;
	}

	if (flag) {
		gpio_direction_output(gpio_pin, 0);	/* enable */
		msleep(1);
	} else {
		gpio_direction_output(gpio_pin, 1);	/* disable */
	}

	gpio_free(gpio_pin);
	return 0;
}

static int camera1_power(struct device *dev, int flag)
{
	int gpio_pin = mfp_to_gpio(MFP_PIN_GPIO14);
	int gpio_pin_mipi = mfp_to_gpio(MFP_PIN_GPIO81);

	if (gpio_request(gpio_pin, "CAM_EANBLE_LOW_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin);
		return -EIO;
	}

	if (gpio_request(gpio_pin_mipi, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "cam: Request GPIO failed,"
				"gpio: %d\n", gpio_pin_mipi);
		return -EIO;
	}

	if (flag) {
		gpio_direction_output(gpio_pin_mipi, 0);
		gpio_direction_output(gpio_pin, 0);
		msleep(1);
	} else {
		gpio_direction_output(gpio_pin_mipi, 1);
		gpio_direction_output(gpio_pin, 1);
	}

	gpio_free(gpio_pin_mipi);
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
	if (get_board_id() >= OBM_SAAR_B_MG2_B0_V15_BOARD) {
		set_pxa95x_fb_info(&lcd_info_wvga);
		set_pxa95x_fb_ovly_info(&lcd_ovly_info_wvga, 0);
	} else {
		set_pxa95x_fb_info(&lcd_info);
		set_pxa95x_fb_ovly_info(&lcd_ovly_info, 0);
	}
}
#endif

#if defined(CONFIG_MMC_SDHCI_PXAV2)
static struct sdhci_pxa_platdata mci0_platform_data = {
	.flags	= PXA_FLAG_CARD_PERMANENT,
};

static void __init init_mmc(void)
{
	/*add emmc only, need to add sdcard and sdio later*/
	pxa95x_set_mci_info(0, &mci0_platform_data);
}

#endif

#if defined(CONFIG_SENSORS_LIS331DL)
static unsigned long lis33ldl_min_delay = 20;
#endif

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

};


static void __init saarb_init(void)
{
	pxa_set_ffuart_info(NULL);
	platform_device_add_data(&pxa95x_device_i2c1, &i2c1_pdata,
				 sizeof(i2c1_pdata));
	platform_device_add_data(&pxa95x_device_i2c2, &i2c2_pdata,
				 sizeof(i2c2_pdata));
	platform_device_add_data(&pxa95x_device_i2c3, &i2c3_pdata,
				 sizeof(i2c3_pdata));
	platform_add_devices(ARRAY_AND_SIZE(devices));
	i2c_register_board_info(0, ARRAY_AND_SIZE(saarb_i2c_info));
	i2c_register_board_info(1, ARRAY_AND_SIZE(i2c2_info));

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif

#if defined(CONFIG_UIO_VMETA)
	init_vmeta();
#endif /*(CONFIG_UIO_VMETA)*/

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

#if defined(CONFIG_MMC_SDHCI_PXAV2)
	init_mmc();
#endif

	/* Init boot flash - sync mode in case of ONENAND */
	pxa_boot_flash_init(1);
}

MACHINE_START(SAARB, "PXA968 (PXA955) Handheld Platform (aka SAARB)")
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.nr_irqs	= SAARB_NR_IRQS,
	.init_irq       = pxa95x_init_irq,
	.handle_irq     = pxa95x_handle_irq_intc,
	.timer          = &pxa_timer,
	.reserve	= pxa95x_mem_reserve,
	.init_machine   = saarb_init,
MACHINE_END

