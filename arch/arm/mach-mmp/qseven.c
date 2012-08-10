/*
 *  linux/arch/arm/mach-mmp/qseven.c
 *  Dipen Patel, dpatel@marvell.com
 *  Derived from abilene.c
 *  Support for the Marvell MMP3 Qseven Platform.
 *
 *  Copyright (C) 2009-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/smc91x.h>

#include <linux/i2c/tsc2007.h>

#include <linux/pwm_backlight.h>
#include <linux/switch.h>
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || \
	defined(CONFIG_SENSORS_LSM303DLHC_MAG)
#include <linux/i2c/lsm303dlhc.h>
#endif
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
#include <linux/i2c/l3g4200d.h>
#endif
#if defined(CONFIG_SPI_PXA2XX)
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/ntrig_spi.h>
#endif
#include <linux/sd8x_rfkill.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp3.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <mach/soc_vmeta.h>
#include <mach/tc35876x.h>
#include <mach/camera.h>
#include <mach/isp_dev.h>
#include <mach/hsi_dev.h>
#include <plat/pmem.h>
#include <plat/usb.h>
#include <mach/sram.h>
#include <mach/axis_sensor.h>
#include <mach/uio_hdmi.h>
#include <media/soc_camera.h>
#include <mach/mmp3_pm.h>
#include <mach/gpio.h>

#include "common.h"
#include "onboard.h"

#define QSEVEN_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long qseven_pin_config[] __initdata = {
	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* TWSI5 not connected in rev 1 carrier board*/
	GPIO99_TWSI5_SCL,
	GPIO100_TWSI5_SDA,

	/* TWSI6 for HDMI */
	GPIO97_TWSI6_SCL,
	GPIO98_TWSI6_SDA,

	/* TWSI2 for camera */
	GPIO55_TWSI2_SCL,
	GPIO56_TWSI2_SDA,
	GPIO73_CAM_MCLK,
	GPIO72_GPIO,

	/* TWSI3 for audio codec on carrier card rev 2*/
	GPIO95_TWSI3_SCL,
	GPIO96_TWSI3_SDA,
	HDA_RST_N_GPIO_79, /*Not connected in carrier rev 1 board*/

	/* TWSI4 touch controller on carrier card*/
	TWSI4_SCL,
	TWSI4_SDA,
	TSI_INT_N, /*also smb_int_n in the card schema*/

	/*PWM3*/
	GPIO53_PWM3,
	/*PWM4 for Q7 board rev 1, not connected anywhere*/
	GPIO54_PWM4,

	/* SSPA1 (I2S) */
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/*ULPI QSEVEN rev 1*/
	ULPI_DATA_0_GPIO_66,
	ULPI_DATA_1_GPIO_65,
	ULPI_DATA_2_GPIO_64,
	ULPI_DATA_3_GPIO_63,
	ULPI_DATA_4_GPIO_62,
	ULPI_DATA_5_GPIO_61,
	ULPI_DATA_6_GPIO_60,
	ULPI_DATA_7_GPIO_59,
	ULPI_RST_N_GPIO_71,
	ULPI_CLK_GPIO_70,
	ULPI_DIR_GPIO_69,
	ULPI_NXT_GPIO_68,
	ULPI_STP_GPIO_67,

	/* SSP1 FOR NOR FLASH, NOT POPULTAED*/
	SSP1_RXD_GPIO_43,
	SSP1_TXD_GPIO_44,
	SSP1_CLK_GPIO_45,
	SSP1_FRM_GPIO_46,

	/* SSP3 FOR CAN*/
	GPIO74_SSP_CLK,
	GPIO75_SSP_FRM,
	GPIO76_SSP_TXD,
	GPIO77_SSP_RXD,

	/*Power button not connected in rev 1 carrier board*/
	GPIO80_GPIO,
	/*HDMI_HPD_N for hdmi detect, y no hdmi_cec*/
	GPIO81_GPIO,
	GPIO82_GPIO,
	GPIO83_GPIO,

	/*LED*/
	GPIO84_GPIO,
	GPIO85_GPIO,
	GPIO86_GPIO,
	GPIO87_GPIO,

};

static unsigned long mmc1_pin_config[] __initdata = {
	GPIO131_MMC1_DAT3,
	GPIO132_MMC1_DAT2,
	GPIO133_MMC1_DAT1,
	GPIO134_MMC1_DAT0,
	GPIO136_MMC1_CMD,
	GPIO135_MMC1_CLK,
	GPIO140_MMC1_CD | MFP_PULL_HIGH,
	GPIO141_MMC1_WP | MFP_PULL_HIGH,
	GPIO137_GPIO, /*drive low to enable power to card*/
};

/* MMC2 is used for WIB card */
static unsigned long mmc2_pin_config[] __initdata = {
	GPIO37_MMC2_DAT3,
	GPIO38_MMC2_DAT2,
	GPIO39_MMC2_DAT1,
	GPIO40_MMC2_DAT0,
	GPIO41_MMC2_CMD,
	GPIO42_MMC2_CLK,

	/* GPIO used for power */
	GPIO57_GPIO, /* WLAN_PD_N */
	WIFI_32K_CLK_OUT,
};

static unsigned long mmc3_pin_config[] __initdata = {
	GPIO108_MMC3_DAT7,
	GPIO109_MMC3_DAT6,
	GPIO161_MMC3_DAT5,
	GPIO163_MMC3_DAT4,
	GPIO111_MMC3_DAT3,
	GPIO110_MMC3_DAT2,
	GPIO162_MMC3_DAT1,
	GPIO164_MMC3_DAT0,
	GPIO145_MMC3_CMD,
	GPIO146_MMC3_CLK,
};

static struct sram_bank mmp3_audiosram_info = {
	.pool_name = "audio sram",
	.step = AUDIO_SRAM_GRANULARITY,
};

static struct sram_bank mmp3_videosram_info = {
	.pool_name = "mmp-videosram",
	.step = VIDEO_SRAM_GRANULARITY,
};

#ifdef CONFIG_UIO_VMETA
static struct vmeta_plat_data mmp_vmeta_plat_data = {
	.bus_irq_handler = NULL,
	.axi_clk_available = 0,
	.power_down_ms = 100,
};

static void __init mmp_init_vmeta(void)
{
	mmp_set_vmeta_info(&mmp_vmeta_plat_data);
}
#endif

static struct pxa27x_keypad_platform_data mmp3_keypad_info = {
	.direct_key_map = {
		KEY_BACK,
		KEY_MENU,
		KEY_HOME,
		KEY_SEARCH,
		KEY_VOLUMEUP,
		KEY_VOLUMEDOWN,
	},
	.direct_key_num = 6,
	.debounce_interval = 30,
	.active_low = 1,
};

static int qseven_pwm_init(struct device *dev)
{
	int lvds_blen, lvds_pplen;
	if (!cpu_is_mmp3_b0())
		return 0;

	lvds_blen = mfp_to_gpio(MFP_PIN_GPIO82);
	lvds_pplen = mfp_to_gpio(MFP_PIN_GPIO83);

	if (gpio_request(lvds_blen, "backlight enable")) {
		printk(KERN_INFO "gpio %d request failed\n", lvds_blen);
		return -1;
	}
	if (gpio_request(lvds_pplen, "lvds pplen")) {
		printk(KERN_INFO "gpio %d request failed\n", lvds_pplen);
		return -1;
	}

	gpio_direction_output(lvds_blen, 1);
	gpio_free(lvds_blen);

	gpio_direction_output(lvds_pplen, 1);
	gpio_free(lvds_pplen);

	return 0;
}

static struct platform_pwm_backlight_data qseven_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
	.init = qseven_pwm_init,
};

static struct platform_device qseven_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &qseven_lcd_backlight_data,
	},
};

#ifdef CONFIG_MMC_SDHCI_PXAV3
#include <linux/mmc/host.h>

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.clk_delay_cycles	= 0x1F,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags          = PXA_FLAG_CARD_PERMANENT,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
};

static int __init qseven_init_mmc(void)
{
	int sd_power_gpio = mfp_to_gpio(MFP_PIN_GPIO137);
	int wlan_pd_n = mfp_to_gpio(MFP_PIN_GPIO57);

	if (gpio_request(sd_power_gpio, "sd card power")) {
		printk(KERN_INFO "gpio %d request failed\n", sd_power_gpio);
		return -1;
	}

	gpio_direction_output(sd_power_gpio, 0);
	gpio_free(sd_power_gpio);

	mfp_config(ARRAY_AND_SIZE(mmc3_pin_config));
	mmp3_add_sdh(2, &mmp3_sdh_platdata_mmc2); /* eMMC */

	mfp_config(ARRAY_AND_SIZE(mmc1_pin_config));
	if (cpu_is_mmp3_b0())
		mmp3_sdh_platdata_mmc0.quirks =
					SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	mmp3_add_sdh(0, &mmp3_sdh_platdata_mmc0); /* SD/MMC */

	/* SDIO for WIFI card */
	mfp_config(ARRAY_AND_SIZE(mmc2_pin_config));
	mmp3_add_sdh(1, &mmp3_sdh_platdata_mmc1);

	if (gpio_request(wlan_pd_n, "wifi card power")) {
		printk(KERN_INFO "gpio %d request failed\n", wlan_pd_n);
		return -1;
	}
	gpio_direction_output(wlan_pd_n, 0);
	mdelay(100);
	gpio_direction_output(wlan_pd_n, 1);
	gpio_free(wlan_pd_n);
	return 0;
}
#endif /* CONFIG_MMC_SDHCI_PXAV3 */


#ifdef CONFIG_USB_SUPPORT

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)

static char *mmp3_usb_clock_name[] = {
	[0] = "U2OCLK",
};

static struct mv_usb_platform_data mmp3_usb_pdata = {
	.clknum		= 1,
	.clkname	= mmp3_usb_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_OTG,
	.phy_init	= pxa_usb_phy_init,
	.phy_deinit	= pxa_usb_phy_deinit,
};
#endif /*CONFIG_USB_PXA_U20*/

#endif

#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data mmp3_hdmi_info __initdata = {
	.sspa_reg_base = 0xD42A0C00,
	/* Fix me: gpio 81 lpm pull ? */
	.gpio = mfp_to_gpio(GPIO81_GPIO),
};
#endif

#define DMCU_SDRAM_TIMING1 0x80
#define DMCU_SDRAM_TIMING2 0x84
#define DMCU_SDRAM_TIMING3 0x88
#define DMCU_SDRAM_TIMING4 0x8c
#define DMCU_SDRAM_TIMING5 0x90
#define DMCU_SDRAM_TIMING6 0x94
#define DMCU_SDRAM_TIMING7 0x98
#define DMCU_PHY_CTRL3 0x220
#define DMCU_PHY_DQ_BYTE_SEL 0x300
#define DMCU_PHY_DLL_CTRL_BYTE1 0x304
#define DMCU_PHY_DLL_WL_SEL 0x380
#define DMCU_PHY_DLL_WL_CTRL0 0x384
#define ALLBITS (0xFFFFFFFF)

static struct dmc_regtable_entry khx1600c9s3k_2x400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911403CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x64660414},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC2003053},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x34F4A187},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x000F20C1},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005501},
};

static struct dmc_regtable_entry khx1600c9s3k_2x533mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911B03CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x74780564},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC200406C},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x3694DA09},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x00142101},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00006601},
};

static struct dmc_regtable_entry khx1600c9s3k_2x600mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x955E03CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x84890614},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC200487C},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x4762F24A},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x00162121},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00006601},
};

static struct dmc_regtable_entry khx1600c9s3k_4x400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x59A803CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0xB5B88812},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x610060A5},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x59D7430E},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x001D2181},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x02120501},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00008801},
};

static struct dmc_regtable_entry khx1600c9s3k_phy[] = {
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000000},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000001},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000002},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000003},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_CTRL3, ALLBITS, 0x00004055},
};

static struct dmc_regtable_entry khx1600c9s3k_wl[] = {
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000100},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x001A001A},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000101},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00160016},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000102},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x001A001A},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000103},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00190019},

};
static struct dmc_timing_entry khx1600c9s3k_table[] = {
	{
		.dsrc = 1,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x400mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x533mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
/*	{
		.dsrc = 2,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x600mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
	{
		.dsrc = 0,
		.mode4x = 1,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_4x400mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
*/
};

static void qseven_update_ddr_info(void)
{
	mmp3_pm_update_dram_timing_table(ARRAY_SIZE(khx1600c9s3k_table),
						khx1600c9s3k_table);
}

static void __init qseven_init(void)
{
	mfp_config(ARRAY_AND_SIZE(qseven_pin_config));
	qseven_update_ddr_info();

	/* on-chip devices */
	mmp3_add_uart(3);

	mmp3_add_keypad(&mmp3_keypad_info);

	mmp3_add_videosram(&mmp3_videosram_info);

	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&qseven_lcd_backlight_devices);
	mmp3_add_thermal();

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif

#ifdef CONFIG_UIO_VMETA
	mmp_init_vmeta();
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
	qseven_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

	platform_device_register(&mmp3_device_rtc);

#ifdef CONFIG_USB_PXA_U2O
	mmp3_device_u2o.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2o);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2O
	mmp3_device_u2oehci.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2oehci);

#ifdef CONFIG_USB_PXA_U2O_OTG
	mmp3_device_u2ootg.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2ootg);
#endif
#endif

	pxa_u3d_phy_disable();
}

MACHINE_START(QSEVEN, "Qseven")
	.map_io		= mmp_map_io,
	.nr_irqs	= QSEVEN_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= qseven_init,
MACHINE_END
