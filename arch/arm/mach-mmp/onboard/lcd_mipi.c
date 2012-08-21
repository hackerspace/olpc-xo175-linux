#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2.h>
#include <mach/mmp3.h>
#include <mach/pxa988.h>
#include <mach/tc35876x.h>
#include <mach/pxa168fb.h>
#include <mach/mmp2_plat_ver.h>
#include <mach/regs-mcu.h>

#ifdef CONFIG_MACH_EMEIDKB
#define CONFIG_VNC
#endif

#ifdef CONFIG_MACH_ABILENE
static struct fb_videomode video_modes_abilene[] = {
	[0] = {
		/* panel refresh rate should <= 55(Hz) */
		.refresh = 55,
		.xres = 1280,
		.yres = 800,
		.hsync_len = 2,
		.left_margin = 64,
		.right_margin = 64,
		.vsync_len = 2,
		.upper_margin = 8,
		.lower_margin = 8,
		.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		},
};
#endif

#ifdef CONFIG_MACH_YELLOWSTONE
static struct fb_videomode video_modes_yellowstone[] = {
	[0] = {
		 /* panel refresh rate should <= 55(Hz) */
		.refresh = 55,
		.xres = 1280,
		.yres = 800,
		.hsync_len = 2,
		.left_margin = 64,
		.right_margin = 64,
		.vsync_len = 2,
		.upper_margin = 8,
		.lower_margin = 8,
		.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		},
};
#endif

#ifdef CONFIG_MACH_THUNDERSTONEM
static struct fb_videomode video_modes_thunderstonem[] = {
	[0] = {
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.hsync_len = 2,
		.left_margin = 122,
		.right_margin = 122,
		.vsync_len = 8,
		.upper_margin = 16,
		.lower_margin = 16,
		.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		},
};
#endif

#ifdef CONFIG_MACH_ORCHID
static struct fb_videomode video_modes_orchid[] = {
	[0] = {
		 /* panel refresh rate should <= 55(Hz) */
		.refresh = 55,
		.xres = 540,
		.yres = 960,
		.hsync_len = 2,
		.left_margin = 50,
		.right_margin = 70,
		.vsync_len = 2,
		.upper_margin = 8,
		.lower_margin = 8,
		.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		},
};
#endif

#ifdef CONFIG_MACH_MK2
static struct fb_videomode video_modes_mk2[] = {
	[0] = {
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.hsync_len = 10,
		.left_margin = 160,
		.right_margin = 200,
		.vsync_len = 2,
		.upper_margin = 18,
		.lower_margin = 18,
		.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		},
};
#endif

#ifdef CONFIG_MACH_EMEIDKB
#ifdef CONFIG_VNC
static struct fb_videomode video_modes_emeidkb[] = {
	/* lpj032l001b HVGA mode info */
	[0] = {
		.refresh        = 60,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 10,
		.left_margin    = 15,
		.right_margin   = 10,
		.vsync_len      = 2,
		.upper_margin   = 4,
		.lower_margin   = 2,
		.sync		= 0,
	},
};
#else
static struct fb_videomode video_modes_emeidkb[] = {
	[0] = {
		 /* panel refresh rate should <= 55(Hz) */
		.refresh = 50,
		.xres = 540,
		.yres = 960,
		.hsync_len = 2,
		.left_margin = 50,
		.right_margin = 70,
		.vsync_len = 2,
		.upper_margin = 8,
		.lower_margin = 8,
		.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		},
};
#endif
#endif

#ifdef CONFIG_MACH_ABILENE
static int abilene_lvds_power(struct pxa168fb_info *fbi,
				unsigned int spi_gpio_cs,
				unsigned int spi_gpio_reset, int on)
{
	struct regulator *v_ldo10, *v_ldo19;
	int lcd_rst_n;

	/*
	 * FIXME: It is board related, baceuse zx will be replaced soon,
	 * it is temproary distinguished by cpu
	 */
	lcd_rst_n = mfp_to_gpio(GPIO128_LCD_RST);
	/* v_ldo19 AVDD_LVDS, 1.8V */
	v_ldo19 = regulator_get(NULL, "v_ldo19");
	/* v_ldo10, 2.8v */
	v_ldo10 = regulator_get(NULL, "v_ldo10");

	if (IS_ERR(v_ldo19) || IS_ERR(v_ldo10)) {
		pr_err("%s regulator get error!\n", __func__);
		v_ldo19 = NULL;
		v_ldo10 = NULL;
		return -EIO;
	}

	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", lcd_rst_n);
		return -EIO;
	}

	if (on) {
		/* v_ldo19 AVDD_LVDS, 1.8V */
		regulator_set_voltage(v_ldo19, 1800000, 1800000);
		regulator_enable(v_ldo19);

		regulator_set_voltage(v_ldo10, 2800000, 2800000);
		regulator_enable(v_ldo10);

		/* release panel from reset */
		gpio_direction_output(lcd_rst_n, 1);
	} else {
		/* disable v_ldo10 2.8v */
		regulator_disable(v_ldo10);
		regulator_put(v_ldo10);

		/* disable v_ldo19 1.8v */
		regulator_disable(v_ldo19);
		regulator_put(v_ldo19);

		gpio_direction_output(lcd_rst_n, 0);
	}

	gpio_free(lcd_rst_n);

	pr_debug("%s on %d\n", __func__, on);
	return 0;
}
#endif

#ifdef CONFIG_MACH_YELLOWSTONE
static int yellowstone_lvds_power(struct pxa168fb_info *fbi,
			     unsigned int spi_gpio_cs,
			     unsigned int spi_gpio_reset, int on)
{
	struct regulator *v_lcd, *v_1p8_ana;
	int lcd_rst_n;

	/*
	 * FIXME: It is board related, baceuse zx will be replaced soon,
	 * it is temproary distinguished by cpu
	 */
	lcd_rst_n = mfp_to_gpio(GPIO128_LCD_RST);
	/* V_1P8_ANA, AVDD_LVDS, 1.8v */
	v_1p8_ana = regulator_get(NULL, "V_1P8_ANA");
	/* V_LCD 3.3v */
	v_lcd = regulator_get(NULL, "V_LCD");

	if (IS_ERR(v_1p8_ana) || IS_ERR(v_lcd)) {
		pr_err("%s regulator get error!\n", __func__);
		v_1p8_ana = NULL;
		v_lcd = NULL;
		return -EIO;
	}

	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", lcd_rst_n);
		return -EIO;
	}

	if (on) {
		regulator_set_voltage(v_1p8_ana, 1800000, 1800000);
		regulator_enable(v_1p8_ana);

		regulator_set_voltage(v_lcd, 3300000, 3300000);
		regulator_enable(v_lcd);

		/* release panel from reset */
		gpio_direction_output(lcd_rst_n, 1);
	} else {
		/* disable v_ldo10 3.3v */
		regulator_disable(v_lcd);
		regulator_put(v_lcd);

		/* disable v_ldo19 1.8v */
		regulator_disable(v_1p8_ana);
		regulator_put(v_1p8_ana);

		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);
	}

	gpio_free(lcd_rst_n);

	pr_debug("%s on %d\n", __func__, on);
	return 0;
}
#endif

#ifdef CONFIG_MACH_THUNDERSTONEM
static int thunderstonem_lvds_power(struct pxa168fb_info *fbi,
				unsigned int spi_gpio_cs,
				unsigned int spi_gpio_reset, int on)
{
	static struct regulator *v_lcd, *v_1p8_ana, *v_3v3;
	int lcd_rst_n, lcd_en, lcd_stby;

	/*
	 * FIXME: It is board related, baceuse zx will be replaced soon,
	 * it is temproary distinguished by cpu
	 */
	lcd_rst_n = mfp_to_gpio(GPIO128_LCD_RST);
	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", lcd_rst_n);
		return -EIO;
	}

	lcd_en = mfp_to_gpio(GPIO84_GPIO);
	if (gpio_request(lcd_en, "lcd en gpio")) {
		pr_err("gpio %d request failed\n", lcd_en);
		goto gpio_free_rst;
	}

	lcd_stby = mfp_to_gpio(GPIO19_GPIO);
	if (gpio_request(lcd_stby, "lcd stby gpio")) {
		pr_err("gpio %d request failed\n", lcd_stby);
		goto gpio_free_en;
	}

	/* V_1P8_ANA, AVDD_LVDS, 1.8v */
	if (!v_1p8_ana) {
		v_1p8_ana = regulator_get(NULL, "AVDD_LVDS");
		if (IS_ERR(v_1p8_ana)) {
			pr_err("%s regulator get error!\n", __func__);
			v_1p8_ana = NULL;
			goto gpio_free_stby;
		}
	}
	/* V_LCD 3.3v */
	if (!v_lcd) {
		v_lcd = regulator_get(NULL, "V_LCD");
		if (IS_ERR(v_lcd)) {
			pr_err("%s regulator get error!\n", __func__);
			v_lcd = NULL;
			goto gpio_free_avdd;
		}
	}
	/* V_3V3, the source of LCD_VDDIO */
	if (!v_3v3) {
		v_3v3 = regulator_get(NULL, "V_3V3");
		if (IS_ERR(v_3v3)) {
			pr_err("%s regulator get error!\n", __func__);
			v_3v3 = NULL;
			goto gpio_free_v_lcd;
		}
	}


	if (on) {
		/* panel stanby mode exit */
		gpio_direction_output(lcd_stby, 1);

		/* panel enable */
		regulator_enable(v_3v3);
		gpio_direction_output(lcd_en, 1);

		regulator_set_voltage(v_1p8_ana, 1800000, 1800000);
		regulator_enable(v_1p8_ana);

		regulator_set_voltage(v_lcd, 3300000, 3300000);
		regulator_enable(v_lcd);

		/* release panel from reset */
		gpio_direction_output(lcd_rst_n, 1);
	} else {
		/* panel stanby mode enter */
		gpio_direction_output(lcd_stby, 0);
		/* panel disable */
		gpio_direction_output(lcd_en, 0);
		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);

		/* disable v_3v3 */
		regulator_disable(v_3v3);

		/* disable v_ldo10 3.3v */
		regulator_disable(v_lcd);

		/* disable v_ldo19 1.8v */
		regulator_disable(v_1p8_ana);
	}

	gpio_free(lcd_rst_n);
	gpio_free(lcd_en);
	gpio_free(lcd_stby);

	pr_debug("%s on %d\n", __func__, on);
	return 0;

gpio_free_v_lcd:
	regulator_put(v_lcd);

gpio_free_avdd:
	regulator_put(v_1p8_ana);

gpio_free_stby:
	gpio_free(lcd_stby);

gpio_free_en:
	gpio_free(lcd_en);

gpio_free_rst:
	gpio_free(lcd_rst_n);
	return -EIO;
}
#endif

#ifdef CONFIG_MACH_ORCHID
static int orchid_lcd_power(struct pxa168fb_info *fbi,
			     unsigned int spi_gpio_cs,
			     unsigned int spi_gpio_reset, int on)
{
	static struct regulator *lcd_iovdd, *lcd_avdd;
	int lcd_rst_n;

	lcd_rst_n = mfp_to_gpio(GPIO49_LCD_RST_N);
	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", lcd_rst_n);
		return -EIO;
	}

	/* LCD_IOVDD, 1.8v */
	if (!lcd_iovdd) {
		lcd_iovdd = regulator_get(NULL, "V_LDO15_1V8");
		if (IS_ERR(lcd_iovdd)) {
			pr_err("%s regulator get error!\n", __func__);
			lcd_iovdd = NULL;
			goto regu_lcd_iovdd;
		}
	}

	/* LCD_AVDD 3.1v */
	if (!lcd_avdd) {
		lcd_avdd = regulator_get(NULL, "V_LDO17_3V1");
		if (IS_ERR(lcd_avdd)) {
			pr_err("%s regulator get error!\n", __func__);
			lcd_avdd = NULL;
			goto regu_lcd_avdd;
		}
	}

	if (on) {
		regulator_set_voltage(lcd_iovdd, 1800000, 1800000);
		regulator_enable(lcd_iovdd);

		regulator_set_voltage(lcd_avdd, 3100000, 3100000);
		regulator_enable(lcd_avdd);

		mdelay(50);
		/* release panel from reset */
		gpio_direction_output(lcd_rst_n, 1);
		udelay(20);
		gpio_direction_output(lcd_rst_n, 0);
		udelay(50);
		gpio_direction_output(lcd_rst_n, 1);
	} else {
		/* disable LCD_AVDD 3.1v */
		regulator_disable(lcd_avdd);

		/* disable LCD_IOVDD 1.8v */
		regulator_disable(lcd_iovdd);

		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);
	}

	gpio_free(lcd_rst_n);
	pr_debug("%s on %d\n", __func__, on);

	return 0;

regu_lcd_iovdd:
	gpio_free(lcd_rst_n);

regu_lcd_avdd:
	regulator_put(lcd_iovdd);

	return -EIO;
}
#endif

#ifdef CONFIG_MACH_EMEIDKB
static int emeidkb_lcd_power(struct pxa168fb_info *fbi,
			     unsigned int spi_gpio_cs,
			     unsigned int spi_gpio_reset, int on)
{
	static struct regulator *lcd_iovdd, *lcd_avdd;
	int lcd_rst_n;

	/* FIXME:lcd reset,use GPIO_1 as lcd reset */
	lcd_rst_n = 1;
	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", lcd_rst_n);
		return -EIO;
	}

	/* FIXME:LCD_IOVDD, 1.8v */
	if (!lcd_iovdd) {
		lcd_iovdd = regulator_get(NULL, "v_ldo15");
		if (IS_ERR(lcd_iovdd)) {
			pr_err("%s regulator get error!\n", __func__);
			lcd_iovdd = NULL;
			goto regu_lcd_iovdd;
		}
	}

	/* FIXME:LCD_AVDD 3.1v */
	if (!lcd_avdd) {
		lcd_avdd = regulator_get(NULL, "v_ldo8");
		if (IS_ERR(lcd_avdd)) {
			pr_err("%s regulator get error!\n", __func__);
			lcd_avdd = NULL;
			goto regu_lcd_avdd;
		}
	}

	if (on) {
		regulator_set_voltage(lcd_iovdd, 1800000, 1800000);
		regulator_enable(lcd_iovdd);

		regulator_set_voltage(lcd_avdd, 3100000, 3100000);
		regulator_enable(lcd_avdd);

		mdelay(50);
		/* release panel from reset */
		gpio_direction_output(lcd_rst_n, 1);
		udelay(20);
		gpio_direction_output(lcd_rst_n, 0);
		udelay(50);
		gpio_direction_output(lcd_rst_n, 1);
	} else {
		/* disable LCD_AVDD 3.1v */
		regulator_disable(lcd_avdd);

		/* disable LCD_IOVDD 1.8v */
		regulator_disable(lcd_iovdd);

		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);
	}

	gpio_free(lcd_rst_n);
	pr_debug("%s on %d\n", __func__, on);

	return 0;

regu_lcd_avdd:
	regulator_put(lcd_iovdd);

regu_lcd_iovdd:
	gpio_free(lcd_rst_n);

	return -EIO;
}
#endif

#if defined(CONFIG_MACH_ABILENE) || defined(CONFIG_MACH_YELLOWSTONE) \
	|| defined(CONFIG_MACH_MK2)
static struct lvds_info lvdsinfo = {
	.src	= LVDS_SRC_PN,
	.fmt	= LVDS_FMT_18BIT,
};

#if defined(CONFIG_MACH_THUNDERSTONEM)
static struct lvds_info lvdsinfo_thunderstonem = {
    .src    = LVDS_SRC_PN,
    .fmt    = LVDS_FMT_24BIT,
};
#endif

static void lvds_hook(struct pxa168fb_mach_info *mi)
{
	mi->phy_type = LVDS;
	mi->phy_init = pxa688_lvds_init;
	mi->phy_info = (void *)&lvdsinfo;

	mi->modes->refresh = 60;

	if (machine_is_yellowstone()) {
		mi->phy_info = (void *)&lvdsinfo;
		mi->pxa168fb_lcd_power = yellowstone_lvds_power;
	} else if (machine_is_abilene()) {
		mi->phy_info = (void *)&lvdsinfo;
		mi->pxa168fb_lcd_power = abilene_lvds_power;
	} else if (machine_is_thunderstonem()) {
#if defined(CONFIG_MACH_THUNDERSTONEM)
		mi->pxa168fb_lcd_power = thunderstonem_lvds_power;
		mi->phy_info = (void *)&lvdsinfo_thunderstonem;
#endif
	}
}
#endif

#if defined(CONFIG_MACH_ABILENE) || defined(CONFIG_MACH_YELLOWSTONE) \
	|| defined(CONFIG_MACH_MK2) || defined(CONFIG_MACH_ORCHID) \
	|| defined(CONFIG_MACH_EMEIDKB)
static void dither_config(struct pxa168fb_mach_info *mi)
{
	struct lvds_info *lvds;
	struct dsi_info *dsi;
	int bpp;

	if (mi->phy_type == LVDS) {
		lvds = (struct lvds_info *)mi->phy_info;
		bpp = (lvds->fmt == LVDS_FMT_18BIT) ? 18 : 24;
	} else {
		dsi = (struct dsi_info *)mi->phy_info;
		bpp = dsi->bpp;
	}

	if (bpp < 24) {
		mi->dither_en = 1;
		/* dither table was related to resolution
		 * 4x4 table could be select for all cases.
		 * we can select 4x8 table if xres is much
		 * bigger than yres */
		mi->dither_table = DITHER_TBL_4X4;
		if (bpp == 18)
			mi->dither_mode = DITHER_MODE_RGB666;
		else if (bpp == 16)
			mi->dither_mode = DITHER_MODE_RGB565;
		else
			mi->dither_mode = DITHER_MODE_RGB444;
	}
}
#endif

/*
 * dsi bpp : rgb_mode
 *    16   : DSI_LCD_INPUT_DATA_RGB_MODE_565;
 *    24   : DSI_LCD_INPUT_DATA_RGB_MODE_888;
 */
static struct dsi_info dsiinfo = {
	.id = 1,
	.lanes = 4,
	.bpp = 16,
	.rgb_mode = DSI_LCD_INPUT_DATA_RGB_MODE_565,
	.burst_mode = DSI_BURST_MODE_BURST,
	.hbp_en = 1,
	.hfp_en = 1,
};

#ifdef CONFIG_MACH_ORCHID
static struct dsi_info orchid_dsiinfo = {
	.id = 2,
	.lanes = 2,
	.bpp = 24,
	.rgb_mode = DSI_LCD_INPUT_DATA_RGB_MODE_888,
	.burst_mode = DSI_BURST_MODE_BURST,
	.hbp_en = 1,
	.hfp_en = 1,
};
#endif

#ifdef CONFIG_MACH_EMEIDKB
/* emeidkb: only DSI1 and use lane0,lane1 */
static struct dsi_info emeidkb_dsiinfo = {
	.id = 1,
	.lanes = 2,
	.bpp = 24,
	.rgb_mode = DSI_LCD_INPUT_DATA_RGB_MODE_888,
	.burst_mode = DSI_BURST_MODE_BURST,
	.hbp_en = 1,
	.hfp_en = 1,
};
#endif

#ifdef CONFIG_TC35876X
static int tc358765_reset(struct pxa168fb_info *fbi)
{
	int gpio;

#ifdef CONFIG_MACH_BROWNSTONE
	gpio = mfp_to_gpio(GPIO83_LCD_RST);
#endif

#if defined(CONFIG_MACH_ABILENE) || defined(CONFIG_MACH_YELLOWSTONE) \
	|| defined(CONFIG_MACH_MK2) || defined(CONFIG_MACH_THUNDERSTONEM)
	gpio = mfp_to_gpio(GPIO128_LCD_RST);
#endif

	if (gpio_request(gpio, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 0);
	mdelay(10);
	gpio_direction_output(gpio, 1);
	mdelay(4);

	gpio_free(gpio);
	return 0;
}

static void tc358765_dump(void)
{
#if 0
	u32 val;

	pr_info("%s\n", __func__);
	tc35876x_read32(PPI_TX_RX_TA, &val);
	pr_info(" - PPI_TX_RX_TA = 0x%x\n", val);
	tc35876x_read32(PPI_LPTXTIMECNT, &val);
	pr_info(" - PPI_LPTXTIMECNT = 0x%x\n", val);
	tc35876x_read32(PPI_D0S_CLRSIPOCOUNT, &val);
	pr_info(" - PPI_D0S_CLRSIPOCOUNT = 0x%x\n", val);
	tc35876x_read32(PPI_D1S_CLRSIPOCOUNT, &val);
	pr_info(" - PPI_D1S_CLRSIPOCOUNT = 0x%x\n", val);

	tc35876x_read32(PPI_D2S_CLRSIPOCOUNT, &val);
	pr_info(" - PPI_D2S_CLRSIPOCOUNT = 0x%x\n", val);
	tc35876x_read32(PPI_D3S_CLRSIPOCOUNT, &val);
	pr_info(" - PPI_D3S_CLRSIPOCOUNT = 0x%x\n", val);

	tc35876x_read32(PPI_LANEENABLE, &val);
	pr_info(" - PPI_LANEENABLE = 0x%x\n", val);
	tc35876x_read32(DSI_LANEENABLE, &val);
	pr_info(" - DSI_LANEENABLE = 0x%x\n", val);
	tc35876x_read32(PPI_STARTPPI, &val);
	pr_info(" - PPI_STARTPPI = 0x%x\n", val);
	tc35876x_read32(DSI_STARTDSI, &val);
	pr_info(" - DSI_STARTDSI = 0x%x\n", val);

	tc35876x_read32(VPCTRL, &val);
	pr_info(" - VPCTRL = 0x%x\n", val);
	tc35876x_read32(HTIM1, &val);
	pr_info(" - HTIM1 = 0x%x\n", val);
	tc35876x_read32(HTIM2, &val);
	pr_info(" - HTIM2 = 0x%x\n", val);
	tc35876x_read32(VTIM1, &val);
	pr_info(" - VTIM1 = 0x%x\n", val);
	tc35876x_read32(VTIM2, &val);
	pr_info(" - VTIM2 = 0x%x\n", val);
	tc35876x_read32(VFUEN, &val);
	pr_info(" - VFUEN = 0x%x\n", val);
	tc35876x_read32(LVCFG, &val);
	pr_info(" - LVCFG = 0x%x\n", val);

	tc35876x_read32(DSI_INTSTAUS, &val);
	pr_info("!! - DSI_INTSTAUS= 0x%x BEFORE\n", val);
	tc35876x_write32(DSI_INTCLR, 0xFFFFFFFF);
	tc35876x_read32(DSI_INTSTAUS, &val);
	pr_info("!! - DSI_INTSTAUS= 0x%x AFTER\n", val);

	tc35876x_read32(DSI_LANESTATUS0, &val);
	pr_info(" - DSI_LANESTATUS0= 0x%x\n", val);
	tc35876x_read32(DSIERRCNT, &val);
	pr_info(" - DSIERRCNT= 0x%x\n", val);
	tc35876x_read32(DSIERRCNT, &val);
	pr_info(" - DSIERRCNT= 0x%x AGAIN\n", val);
	tc35876x_read32(SYSSTAT, &val);
	pr_info(" - SYSSTAT= 0x%x\n", val);
#endif
}

static int dsi_set_tc358765(struct pxa168fb_info *fbi)
{
	int status;
	struct fb_var_screeninfo *var = &(fbi->fb_info->var);
	struct dsi_info *di = &dsiinfo;
	u16 chip_id = 0;

	status = tc35876x_read16(TC358765_CHIPID_REG, &chip_id);
	if ((status < 0) || (chip_id != TC358765_CHIPID)) {
		pr_err("tc35876x unavailable! chip_id %x\n", chip_id);
		return -EIO;
	} else
		pr_debug("tc35876x(chip id:0x%02x) detected.\n", chip_id);

	/* REG 0x13C,DAT 0x000C000F */
	tc35876x_write32(PPI_TX_RX_TA, 0x00040004);
	/* REG 0x114,DAT 0x0000000A */
	tc35876x_write32(PPI_LPTXTIMECNT, 0x00000004);

	/* get middle value of mim-max value
	 * 0-0x13 for 2lanes-rgb888, 0-0x26 for 4lanes-rgb888
	 * 0-0x21 for 2lanes-rgb565, 0-0x25 for 4lanes-rgb565
	 */
	if (di->lanes == 4)
		status = 0x13;
	else if (di->bpp == 24)
		status = 0xa;
	else
		status = 0x11;
	/* REG 0x164,DAT 0x00000005 */
	tc35876x_write32(PPI_D0S_CLRSIPOCOUNT, status);
	/* REG 0x168,DAT 0x00000005 */
	tc35876x_write32(PPI_D1S_CLRSIPOCOUNT, status);
	if (di->lanes == 4) {
		/* REG 0x16C,DAT 0x00000005 */
		tc35876x_write32(PPI_D2S_CLRSIPOCOUNT, status);
		/* REG 0x170,DAT 0x00000005 */
		tc35876x_write32(PPI_D3S_CLRSIPOCOUNT, status);
	}

	/* REG 0x134,DAT 0x00000007 */
	tc35876x_write32(PPI_LANEENABLE, (di->lanes == 4) ? 0x1f : 0x7);
	/* REG 0x210,DAT 0x00000007 */
	tc35876x_write32(DSI_LANEENABLE, (di->lanes == 4) ? 0x1f : 0x7);

	/* REG 0x104,DAT 0x00000001 */
	tc35876x_write32(PPI_STARTPPI, 0x0000001);
	/* REG 0x204,DAT 0x00000001 */
	tc35876x_write32(DSI_STARTDSI, 0x0000001);

	/* REG 0x450,DAT 0x00012020, VSDELAY = 8 pixels,
	 * enable magic square if in_bpp == 24, out_bpp == 18 */
	tc35876x_write32(VPCTRL, 0x00800020 | (di->bpp == 24 ? 1 : 0));

	/* REG 0x454,DAT 0x00200008*/
	tc35876x_write32(HTIM1, ((var->left_margin) << 16)
			| var->hsync_len);

	/* REG 0x45C,DAT 0x00040004*/
	tc35876x_write32(VTIM1, ((var->upper_margin) << 16)
			| var->vsync_len);

	/* REG 0x49C,DAT 0x00000201 */
	tc35876x_write32(LVCFG, 0x00000001);

	/* dump register value */
	tc358765_dump();
	return 0;
}
#endif

#if defined(CONFIG_MACH_ORCHID) || defined(CONFIG_MACH_EMEIDKB)
static void panel_init_config(struct pxa168fb_info *fbi)
{
	enum dsi_packet_di data_type;
	enum dsi_packet_dcs_id dcs;

	set_dsi_low_power_mode(fbi);

	data_type = DSI_DI_DCS_WRITE_N;
	dcs = DSI_DCS_SLEEP_EXIT;
	dsi_send_cmd(fbi, data_type, dcs, 0);
	mdelay(200);

	dcs = DSI_DCS_DISPLAY_ON;
	dsi_send_cmd(fbi, data_type, dcs, 0);
}
#endif

#ifdef CONFIG_MACH_BROWNSTONE
static int lcd_twsi5_set(int en)
{
	int gpio;
	mfp_cfg_t mfp_gpio99_gpio = MFP_CFG_X(GPIO99, AF0, MEDIUM, PULL_HIGH)
								| MFP_PULL_HIGH;
	mfp_cfg_t mfp_gpio100_gpio = MFP_CFG_X(GPIO100, AF0, MEDIUM, PULL_HIGH)
								| MFP_PULL_HIGH;
	mfp_cfg_t mfp_gpio99_twsi5 = GPIO99_TWSI5_SCL;
	mfp_cfg_t mfp_gpio100_twsi5 = GPIO100_TWSI5_SDA;

	if (en) {
		mfp_config(&mfp_gpio99_twsi5, 1);
		mfp_config(&mfp_gpio100_twsi5, 1);
	} else {
		mfp_config(&mfp_gpio99_gpio, 1);
		mfp_config(&mfp_gpio100_gpio, 1);

		gpio = mfp_to_gpio(GPIO99_GPIO);
		if (gpio_request(gpio, "gpio99")) {
			printk(KERN_INFO "gpio %d request failed\n", gpio);
			return -1;
		}
		gpio_direction_output(gpio, 0);
		gpio_free(gpio);

		gpio = mfp_to_gpio(GPIO100_GPIO);
		if (gpio_request(gpio, "gpio100")) {
			printk(KERN_INFO "gpio %d request failed\n", gpio);
			return -1;
		}
		gpio_direction_output(gpio, 0);
		gpio_free(gpio);
	}
	return 0;
}
#endif

#if defined(CONFIG_MACH_ABILENE) || defined(CONFIG_MACH_YELLOWSTONE)
static int backlight_pwm_set(int en)
{
	int gpio;
	mfp_cfg_t gpio53_gpio = MFP_CFG_ALL(GPIO53, AF0,
				MEDIUM, PULL_HIGH, PULL_HIGH);

	mfp_config(&gpio53_gpio, 1);
	gpio = mfp_to_gpio(GPIO53_GPIO);
	if (gpio_request(gpio, "gpio53")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	if (en)
		gpio_direction_output(gpio, 1);
	else
		gpio_direction_output(gpio, 0);

	gpio_free(gpio);
	return 0;
}

static int abilene_lcd_power(struct pxa168fb_info *fbi,
			     unsigned int spi_gpio_cs,
			     unsigned int spi_gpio_reset, int on)
{
	struct regulator *v_ldo = NULL;
	int lcd_rst_n;
	/*
	 * FIXME: It is board related, baceuse zx will be replaced soon,
	 * it is temproary distinguished by cpu
	 */
	lcd_rst_n = mfp_to_gpio(GPIO128_LCD_RST);

	/* set LDOs 17 and 03 to 1.2V for MIPI Bridge */
	if (on) {
		/* v_ldo17 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo17");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else {
			regulator_enable(v_ldo);
			regulator_set_voltage(v_ldo, 1200000, 1200000);
			/* regulator_put(v_ldo); */
		}
		/* v_ldo3 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo3");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else {
			regulator_enable(v_ldo);
			regulator_set_voltage(v_ldo, 1200000, 1200000);
			/* regulator_put(v_ldo); */
		}
	} else {
		/* disable v_ldo03 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo3");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else
			regulator_disable(v_ldo);
		/* disable v_ldo17 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo17");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else
			regulator_disable(v_ldo);
	}

	/* set panel reset */
	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_rst_n);
		return -1;
	}
	if (on) {
		gpio_direction_output(lcd_rst_n, 1);
		/* FIXME workaround for Abilene Rev5 backlight issue */
#ifdef CONFIG_MACH_ABILENE
		if (machine_is_abilene() && cpu_is_mmp3_b0()) {
			mdelay(100);
			backlight_pwm_set(1);
		}
#endif
	} else {
		gpio_direction_output(lcd_rst_n, 0);
		/* FIXME workaround for Abilene Rev5 backlight issue */
#ifdef CONFIG_MACH_ABILENE
		if (machine_is_abilene() && cpu_is_mmp3_b0())
			backlight_pwm_set(0);
#endif
	}
	gpio_free(lcd_rst_n);

	printk(KERN_DEBUG "%s on %d\n", __func__, on);
	return 0;
}
#endif

#ifdef CONFIG_MACH_MK2
static int  mk2_lcd_power_en(int on)
{
	int vlcd_3v3_en;

	vlcd_3v3_en = mfp_to_gpio(GPIO152_VLCD_3V3);
	if (gpio_request(vlcd_3v3_en, "vlcd 3v3 gpio")) {
		pr_err("gpio %d request failed\n", vlcd_3v3_en);
		return -EIO;
	}

	if (on)
		gpio_direction_output(vlcd_3v3_en, 1);
	else
		gpio_direction_output(vlcd_3v3_en, 0);
	gpio_free(vlcd_3v3_en);

	return 0;
}

static int mk2_lcd_power(struct pxa168fb_info *fbi,
			     unsigned int spi_gpio_cs,
			     unsigned int spi_gpio_reset, int on)
{
	static struct regulator *mipi_1p2v = NULL,
		*mipi_logic_1p2v = NULL;
	int mipi_rst, bl_en;

	/*
	 * FIXME: It is board related, baceuse zx will be replaced soon,
	 * it is temproary distinguished by cpu
	 */
	mipi_rst = mfp_to_gpio(GPIO128_LCD_RST);
	if (gpio_request(mipi_rst, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", mipi_rst);
		return -EIO;
	}

	bl_en = mfp_to_gpio(GPIO17_BL_EN);
	if (gpio_request(bl_en, "lcd bl_en gpio")) {
		pr_err("gpio %d request failed\n", bl_en);
		goto gpio_req_bl_en;
	}

	/* pmic_1p2v_mipi, 1.2v */
	if (!mipi_1p2v) {
		mipi_1p2v = regulator_get(NULL, "PMIC_LDO1");
		if (IS_ERR(mipi_1p2v)) {
			pr_err("%s regulator get error!\n", __func__);
			mipi_1p2v = NULL;
			goto regu_mipi_1p2v;
		}
	}

	/* pmic_1p2v_mipi_logic, 1.2v */
	if (!mipi_logic_1p2v) {
		mipi_logic_1p2v = regulator_get(NULL, "PMIC_LDO7");
		if (IS_ERR(mipi_logic_1p2v)) {
			pr_err("%s regulator get error!\n", __func__);
			mipi_logic_1p2v = NULL;
			goto regu_mipi_logic_1p2v;
		}
	}

	if (on) {
		mk2_lcd_power_en(1);

		regulator_set_voltage(mipi_1p2v, 1200000, 1200000);
		regulator_enable(mipi_1p2v);

		regulator_set_voltage(mipi_logic_1p2v, 1200000, 1200000);
		regulator_enable(mipi_logic_1p2v);

		/* release panel from reset */
		gpio_direction_output(mipi_rst, 1);
		gpio_direction_output(bl_en, 1);
	} else {
		gpio_direction_output(bl_en, 0);
		/* set panel reset */
		gpio_direction_output(mipi_rst, 0);

		regulator_disable(mipi_logic_1p2v);

		regulator_disable(mipi_1p2v);

		mk2_lcd_power_en(0);
	}

	gpio_free(mipi_rst);
	gpio_free(bl_en);
	pr_debug("%s on %d\n", __func__, on);

	return 0;

regu_mipi_logic_1p2v:
	regulator_put(mipi_1p2v);

regu_mipi_1p2v:
	gpio_free(bl_en);

gpio_req_bl_en:
	gpio_free(mipi_rst);
	return -EIO;
}
#endif

static int dsi_init(struct pxa168fb_info *fbi)
{
#ifdef CONFIG_PXA688_PHY
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int ret = 0;

	/* reset DSI controller */
	dsi_reset(fbi, 1);
	mdelay(1);

	/* disable continuous clock */
	dsi_cclk_set(fbi, 0);

	/* dsi out of reset */
	dsi_reset(fbi, 0);

	/* turn on DSI continuous clock */
	dsi_cclk_set(fbi, 1);

	/* set dphy */
	dsi_set_dphy(fbi);

	/* init panel settings via dsi */
	if (mi->phy_type == DSI)
		mi->dsi_panel_config(fbi);

	/* put all lanes to LP-11 state  */
	dsi_lanes_enable(fbi, 0);
	dsi_lanes_enable(fbi, 1);

	/*  reset the bridge */
	if (mi->xcvr_reset) {
		mi->xcvr_reset(fbi);
		mdelay(10);
	}

	/* set dsi controller */
	dsi_set_controller(fbi);

	/* set dsi to dpi conversion chip */
	if (mi->phy_type == DSI2DPI) {
		ret = mi->dsi2dpi_set(fbi);
		if (ret < 0)
			pr_err("dsi2dpi_set error!\n");
	}
#endif
	return 0;
}

#if defined(CONFIG_MACH_ABILENE) || defined(CONFIG_MACH_YELLOWSTONE) \
	|| defined(CONFIG_MACH_MK2) || defined(CONFIG_MACH_ORCHID) \
	|| defined(CONFIG_MACH_EMEIDKB) || defined(CONFIG_MACH_THUNDERSTONEM)
static struct pxa168fb_mach_info mipi_lcd_info = {
	.id = "GFX Layer",
	.num_modes = 0,
	.modes = NULL,
	.sclk_div = 0xE0001108,
	.pix_fmt = PIX_FMT_RGB565,
	.isr_clear_mask	= LCD_ISR_CLEAR_MASK_PXA168,
	/* don't care about io_pin_allocation_mode and dumb_mode
	 * since the panel is hard connected with lcd panel path and
	 * dsi1 output
	 */
	.io_pad_ctrl = CFG_CYC_BURST_LEN16,
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena = 0,
	.invert_pixclock = 0,
	.panel_rbswap = 0,
	.active = 1,
	.enable_lcd = 1,
	.spi_gpio_cs = -1,
	.spi_gpio_reset = -1,
	.mmap = 1,
	.vdma_enable = 1,
	.sram_size  = 30 * 1024,
	.max_fb_size = 0,
	.phy_type = DSI2DPI,
	.phy_init = dsi_init,
#ifdef CONFIG_TC35876X
	.dsi2dpi_set = dsi_set_tc358765,
	.xcvr_reset = tc358765_reset,
#endif
	.phy_info = &dsiinfo,
	.sclk_src = 520000000,
};

static struct pxa168fb_mach_info mipi_lcd_ovly_info = {
	.id = "Video Layer",
	.num_modes = 0,
	.modes = NULL,
	.pix_fmt = PIX_FMT_RGB565,
	.io_pad_ctrl = CFG_CYC_BURST_LEN16,
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena = 0,
	.invert_pixclock = 0,
	.panel_rbswap = 0,
	.active = 1,
	.enable_lcd = 1,
	.spi_gpio_cs = -1,
	.spi_gpio_reset = -1,
	.mmap = 0,
	.max_fb_size = 0,
	.vdma_enable = 0,
	.sram_size = 30 * 1024,
};

#ifndef CONFIG_VNC
#define     DSI1_BITCLK(div)			((div)<<8)
#define     DSI1_BITCLK_DIV_MASK		0x00000F00
#define     CLK_INT_DIV(div)			(div)
#define     CLK_INT_DIV_MASK			0x000000FF
static void calculate_dsi_clk(struct pxa168fb_mach_info *mi)
{
	struct dsi_info *di = (struct dsi_info *)mi->phy_info;
	struct fb_videomode *modes = &mi->modes[0];
	u32 total_w, total_h, pclk2bclk_rate, byteclk, bitclk,
	    pclk_div, bitclk_div = 1;

	if (!di)
		return;

	/*
	 * When DSI is used to refresh panel, the timing configuration should
	 * follow the rules below:
	 * 1.Because Async fifo exists between the pixel clock and byte clock
	 *   domain, so there is no strict ratio requirement between pix_clk
	 *   and byte_clk, we just need to meet the following inequation to
	 *   promise the data supply from LCD controller:
	 *   pix_clk * (nbytes/pixel) >= byte_clk * lane_num
	 *   (nbyte/pixel: the real byte in DSI transmission)
	 *   a)16-bit format n = 2; b) 18-bit packed format n = 18/8 = 9/4;
	 *   c)18-bit unpacked format  n=3; d)24-bit format  n=3;
	 *   if lane_num = 1 or 2, we can configure pix_clk/byte_clk = 1:1 >
	 *   lane_num/nbytes/pixel
	 *   if lane_num = 3 or 4, we can configure pix_clk/byte_clk = 2:1 >
	 *   lane_num/nbytes/pixel
	 * 2.The horizontal sync for LCD is synchronized from DSI,
	 *    so the refresh rate calculation should base on the
	 *    configuration of DSI.
	 *    byte_clk = (h_total * nbytes/pixel) * v_total * fps / lane_num;
	 */
	total_w = modes->xres + modes->left_margin +
		 modes->right_margin + modes->hsync_len;
	total_h = modes->yres + modes->upper_margin +
		 modes->lower_margin + modes->vsync_len;

	pclk2bclk_rate = (di->lanes > 2) ? 2 : 1;
	byteclk = ((total_w * (di->bpp >> 3)) * total_h *
			 modes->refresh) / di->lanes;
	bitclk = byteclk << 3;

	/* The minimum of DSI pll is 150MHz */
	if (bitclk < 150000000)
		bitclk_div = 150000000 / bitclk + 1;

	mi->sclk_src = bitclk * bitclk_div;
	/*
	 * mi->sclk_src = pclk * pclk_div;
	 * pclk / bitclk  = pclk / (8 * byteclk) = pclk2bclk_rate / 8;
	 * pclk_div / bitclk_div = 8 / pclk2bclk_rate;
	 */
	pclk_div = (bitclk_div << 3) / pclk2bclk_rate;

	mi->sclk_div &= ~(DSI1_BITCLK_DIV_MASK | CLK_INT_DIV_MASK);
	mi->sclk_div |= DSI1_BITCLK(bitclk_div) | CLK_INT_DIV(pclk_div);
}

static void calculate_lvds_clk(struct pxa168fb_mach_info *mi)
{
	struct fb_videomode *modes = &mi->modes[0];
	u32 total_w, total_h, pclk, div, use_pll1;

	total_w = modes->xres + modes->left_margin +
		modes->right_margin + modes->hsync_len;
	total_h = modes->yres + modes->upper_margin +
		modes->lower_margin + modes->vsync_len;

	pclk = total_w * total_h * modes->refresh;

	/* use pll1 by default
	 * we could set a more flexible clocking options by selecting pll3 */
	use_pll1 = 1;
	if (use_pll1) {
		/* src clock is 800MHz */
		div = 800000000 / pclk;
		if (div * pclk < 800000000)
			div++;
		mi->sclk_src = 800000000;
		mi->sclk_div = 0x20000000 | div;
	} else {
		div = 150000000 / pclk;
		if (div * pclk < 150000000)
			div++;
		mi->sclk_src = pclk * div;
		mi->sclk_div = 0xe0000000 | div;
	}

	pr_debug("\n%s sclk_src %d sclk_div 0x%x\n", __func__,
			mi->sclk_src, mi->sclk_div);
}

static void calculate_lcd_sclk(struct pxa168fb_mach_info *mi)
{

	if (mi->phy_type & (DSI | DSI2DPI))
		calculate_dsi_clk(mi);
	else if (mi->phy_type & LVDS)
		calculate_lvds_clk(mi);
	else
		return;
}
#endif
#endif

#if defined(CONFIG_MACH_ABILENE) || defined(CONFIG_MACH_YELLOWSTONE) \
	|| defined(CONFIG_MACH_MK2) || defined(CONFIG_MACH_ORCHID) \
	|| defined(CONFIG_MACH_BROWNSTONE) || defined(CONFIG_MACH_THUNDERSTONEM)
static void vsmooth_init(int vsmooth_ch, int filter_ch)
{
#ifdef CONFIG_PXA688_MISC
	/* set TV path vertical smooth, panel2 as filter channel,
	 * vertical smooth is disabled by default to avoid underrun
	 * when video playback, to enable/disable graphics/video
	 * layer vertical smooth:
	 * echo g0/g1/v0/v1 > /sys/deivces/platform/pxa168-fb.1/misc
	 */
	fb_vsmooth = vsmooth_ch; fb_filter = filter_ch;
#endif
}
#endif

#define DDR_MEM_CTRL_BASE 0xD0000000
#define SDRAM_CONFIG_TYPE1_CS0 0x20	/* MMP3 */

#ifdef CONFIG_MACH_ABILENE
void __init abilene_add_lcd_mipi(void)
{
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL, lvds_en;
	struct dsi_info *dsi;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_abilene);
	fb->modes = video_modes_abilene;
	fb->max_fb_size = video_modes_abilene[0].xres *
		video_modes_abilene[0].yres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;
	fb->pxa168fb_lcd_power = &abilene_lcd_power;

	/* FIXME: select DSI2LVDS by default on abilene. */
	lvds_en = 0;
	if (cpu_is_mmp3_b0()) {
		if (lvds_en)
			lvds_hook(fb);
		dither_config(fb);
	}

	if (fb->phy_type & (DSI | DSI2DPI)) {
		dsi = (struct dsi_info *)fb->phy_info;
		dsi->master_mode = 1;
		dsi->hfp_en = 0;
		if (dsi->bpp == 16)
			video_modes_abilene[0].right_margin =
			(dsi->lanes == 4) ? 325 : 179;
		else if (dsi->bpp == 24)
			video_modes_abilene[0].right_margin =
			(dsi->lanes == 4) ? 206 : 116;
	}

	/* Re-calculate lcd clk source and divider
	 * according to dsi lanes and output format.
	 */
	calculate_lcd_sclk(fb);

	dmc_membase = ioremap(DDR_MEM_CTRL_BASE, 0x30);
	CSn_NO_COL = __raw_readl(dmc_membase + SDRAM_CONFIG_TYPE1_CS0) >> 4;
	CSn_NO_COL &= 0xF;
	if (CSn_NO_COL <= 0x2) {
		/*
		 *If DDR page size < 4KB,
		 *select no crossing 1KB boundary check
		 */
		fb->io_pad_ctrl |= CFG_BOUNDARY_1KB;
		ovly->io_pad_ctrl |= CFG_BOUNDARY_1KB;
	}
	iounmap(dmc_membase);

	/* add frame buffer drivers */
	mmp3_add_fb(fb);
	/* add overlay driver */
#ifdef CONFIG_PXA168_V4L2_OVERLAY
	mmp3_add_v4l2_ovly(ovly);
#else
	mmp3_add_fb_ovly(ovly);
#endif
	vsmooth_init(1, 2);
}
#endif

#ifdef CONFIG_MACH_YELLOWSTONE
void __init yellowstone_add_lcd_mipi(void)
{
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL;
	struct dsi_info *dsi;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_yellowstone);
	fb->modes = video_modes_yellowstone;
	fb->max_fb_size = video_modes_yellowstone[0].xres *
		video_modes_yellowstone[0].yres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;
	fb->pxa168fb_lcd_power = &abilene_lcd_power;

	if (cpu_is_mmp3_b0()) {
		lvds_hook(fb);
		dither_config(fb);
	}

	if (fb->phy_type & (DSI | DSI2DPI)) {
		dsi = (struct dsi_info *)fb->phy_info;
		dsi->master_mode = 1;
		dsi->hfp_en = 0;
		if (dsi->bpp == 16)
			video_modes_yellowstone[0].right_margin =
			(dsi->lanes == 4) ? 325 : 179;
		else if (dsi->bpp == 24)
			video_modes_yellowstone[0].right_margin =
			(dsi->lanes == 4) ? 206 : 116;
	}

	/* Re-calculate lcd clk source and divider
	 * according to dsi lanes and output format.
	 */
	calculate_lcd_sclk(fb);

	dmc_membase = ioremap(DDR_MEM_CTRL_BASE, 0x30);
	CSn_NO_COL = __raw_readl(dmc_membase + SDRAM_CONFIG_TYPE1_CS0) >> 4;
	CSn_NO_COL &= 0xF;
	if (CSn_NO_COL <= 0x2) {
		/*
		 *If DDR page size < 4KB,
		 *select no crossing 1KB boundary check
		 */
		fb->io_pad_ctrl |= CFG_BOUNDARY_1KB;
		ovly->io_pad_ctrl |= CFG_BOUNDARY_1KB;
	}
	iounmap(dmc_membase);

	/* add frame buffer drivers */
	mmp3_add_fb(fb);
	/* add overlay driver */
#ifdef CONFIG_PXA168_V4L2_OVERLAY
	mmp3_add_v4l2_ovly(ovly);
#else
	mmp3_add_fb_ovly(ovly);
#endif
	vsmooth_init(1, 2);
}
#endif

#ifdef CONFIG_MACH_THUNDERSTONEM
void __init thunderstonem_add_lcd_mipi(void)
{
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_thunderstonem);
	fb->modes = video_modes_thunderstonem;
	fb->max_fb_size = video_modes_thunderstonem[0].xres *
		video_modes_thunderstonem[0].yres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;

	lvds_hook(fb);

	/* Re-calculate lcd clk source and divider
	 * according to dsi lanes and output format.
	 */
	calculate_lcd_sclk(fb);

	dmc_membase = ioremap(DDR_MEM_CTRL_BASE, 0x30);
	CSn_NO_COL = __raw_readl(dmc_membase + SDRAM_CONFIG_TYPE1_CS0) >> 4;
	CSn_NO_COL &= 0xF;
	if (CSn_NO_COL <= 0x2) {
		/*
		 *If DDR page size < 4KB,
		 *select no crossing 1KB boundary check
		 */
		fb->io_pad_ctrl |= CFG_BOUNDARY_1KB;
		ovly->io_pad_ctrl |= CFG_BOUNDARY_1KB;
	}
	iounmap(dmc_membase);

	/* add frame buffer drivers */
	mmp3_add_fb(fb);
	/* add overlay driver */
#ifdef CONFIG_PXA168_V4L2_OVERLAY
	mmp3_add_v4l2_ovly(ovly);
#else
	mmp3_add_fb_ovly(ovly);
#endif
	vsmooth_init(1, 2);
}
#endif

#ifdef CONFIG_MACH_ORCHID
void __init orchid_add_lcd_mipi(void)
{
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL;
	struct dsi_info *dsi;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_orchid);
	fb->modes = video_modes_orchid;
	fb->max_fb_size = video_modes_orchid[0].xres *
		video_modes_orchid[0].yres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;

	fb->phy_type = DSI;
	fb->xcvr_reset = NULL;
	fb->phy_info = (void *)&orchid_dsiinfo;
	fb->dsi_panel_config = panel_init_config;
	fb->pxa168fb_lcd_power = orchid_lcd_power;
	dsi = (struct dsi_info *)fb->phy_info;
	dsi->master_mode = 1;
	dsi->hfp_en = 0;

	if (cpu_is_mmp3_b0())
		dither_config(fb);

	/* Re-calculate lcd clk source and divider
	 * according to dsi lanes and output format.
	 */
	calculate_lcd_sclk(fb);

	dmc_membase = ioremap(DDR_MEM_CTRL_BASE, 0x30);
	CSn_NO_COL = __raw_readl(dmc_membase + SDRAM_CONFIG_TYPE1_CS0) >> 4;
	CSn_NO_COL &= 0xF;
	if (CSn_NO_COL <= 0x2) {
		/*
		 *If DDR page size < 4KB,
		 *select no crossing 1KB boundary check
		 */
		fb->io_pad_ctrl |= CFG_BOUNDARY_1KB;
		ovly->io_pad_ctrl |= CFG_BOUNDARY_1KB;
	}
	iounmap(dmc_membase);

	/* add frame buffer drivers */
	mmp3_add_fb(fb);
	/* add overlay driver */
#ifdef CONFIG_PXA168_V4L2_OVERLAY
	mmp3_add_v4l2_ovly(ovly);
#else
	mmp3_add_fb_ovly(ovly);
#endif
	vsmooth_init(1, 2);
}
#endif

#ifdef CONFIG_MACH_MK2
void __init mk2_add_lcd_mipi(void)
{
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL, lvds_en;
	struct dsi_info *dsi;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_mk2);
	fb->modes = video_modes_mk2;
	fb->max_fb_size = video_modes_mk2[0].xres *
		video_modes_mk2[0].yres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;

	/* FIXME: V_LCD_3P3V enabled firstly,
	 * there would be a blank flicker if not */
	mk2_lcd_power_en(1);
	fb->pxa168fb_lcd_power = mk2_lcd_power;

	/* FIXME: select DSI2LVDS by default on mk2. */
	lvds_en = 0;
	if (cpu_is_mmp3_b0()) {
		if (lvds_en)
			lvds_hook(fb);
		dither_config(fb);
	}

	if (fb->phy_type & (DSI | DSI2DPI)) {
		dsi = (struct dsi_info *)fb->phy_info;
		dsi->master_mode = 1;
		dsi->hfp_en = 0;
	}

	/* Re-calculate lcd clk source and divider
	 * according to dsi lanes and output format.
	 */
	calculate_lcd_sclk(fb);

	dmc_membase = ioremap(DDR_MEM_CTRL_BASE, 0x30);
	CSn_NO_COL = __raw_readl(dmc_membase + SDRAM_CONFIG_TYPE1_CS0) >> 4;
	CSn_NO_COL &= 0xF;
	if (CSn_NO_COL <= 0x2) {
		/*
		 *If DDR page size < 4KB,
		 *select no crossing 1KB boundary check
		 */
		fb->io_pad_ctrl |= CFG_BOUNDARY_1KB;
		ovly->io_pad_ctrl |= CFG_BOUNDARY_1KB;
	}
	iounmap(dmc_membase);

	/* add frame buffer drivers */
	mmp3_add_fb(fb);
	/* add overlay driver */
#ifdef CONFIG_PXA168_V4L2_OVERLAY
	mmp3_add_v4l2_ovly(ovly);
#else
	mmp3_add_fb_ovly(ovly);
#endif
	vsmooth_init(1, 2);
}
#endif

#ifdef CONFIG_MACH_EMEIDKB
void __init emeidkb_add_lcd_mipi(void)
{
	unsigned int CSn_NO_COL;
	struct dsi_info *dsi;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_emeidkb);
	fb->modes = video_modes_emeidkb;
	fb->max_fb_size = video_modes_emeidkb[0].xres *
		video_modes_emeidkb[0].xres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;

	fb->phy_type = DSI;
	fb->xcvr_reset = NULL;
	fb->phy_info = (void *)&emeidkb_dsiinfo;
	fb->dsi_panel_config = panel_init_config;
	fb->pxa168fb_lcd_power = emeidkb_lcd_power;

	/* For EMEIDKB, there is not vdma */
	fb->vdma_enable = 0;
	fb->sram_size = 0;

	dsi = (struct dsi_info *)fb->phy_info;
	dsi->master_mode = 1;
	dsi->hfp_en = 0;

	dither_config(fb);
	/*
	 * Re-calculate lcd clk source and divider
	 * according to dsi lanes and output format.
	 */
#ifndef CONFIG_VNC
	calculate_lcd_sclk(fb);
#else
	/* FIXME:rewrite sclk_src, otherwise VNC will
	 * use 520000000 as sclk_src so that clock source
	 * will be set 624M */
	fb->sclk_src = 416000000;
	/* FIXME: change pixel clk divider for HVGA for fps 60 */
	fb->sclk_div = 0xE000141b;
#endif

	/*
	 * FIXME:EMEI dkb use display clk1 as clk source,
	 * which is from PLL1 416MHZ. PLL3 1GHZ will be used
	 * for cpu core,and can't be DSI clock source specially.
	 */
	fb->sclk_div &= 0x0fffffff;
	fb->sclk_div |= 0x40000000;

	CSn_NO_COL = __raw_readl(DMCU_VIRT_BASE + DMCU_SDRAM_CFG0_TYPE1) >> 4;
	CSn_NO_COL &= 0xF;
	if (CSn_NO_COL <= 0x2) {
		/*
		 *If DDR page size < 4KB,
		 *select no crossing 1KB boundary check
		 */
		fb->io_pad_ctrl |= CFG_BOUNDARY_1KB;
		ovly->io_pad_ctrl |= CFG_BOUNDARY_1KB;
	}

	/* add frame buffer drivers */
	pxa988_add_fb(fb);
	/* add overlay driver */
#ifdef CONFIG_PXA168_V4L2_OVERLAY
	pxa988_add_v4l2_ovly(ovly);
#else
	pxa988_add_fb_ovly(ovly);
#endif
}
#endif

#ifdef CONFIG_MACH_BROWNSTONE
static struct fb_videomode video_modes_brownstone[] = {
	[0] = {
		.refresh	= 60,
		.xres		= 1280,
		.yres		= 720,
		.hsync_len	= 2,
		.left_margin	= 12,	/* hbp */
		.right_margin	= 216,	/* hfp */
		.vsync_len	= 2,
		.upper_margin	= 10,	/* vbp */
		.lower_margin	= 4,	/* vfp */
		.sync		= 0,
	},
};

static struct regulator *lcd_pwr_ldo17;
static struct regulator *lcd_pwr_ldo3;
static struct regulator *led_pwr_v5p;

static int brownstone_lcd_power(struct pxa168fb_info *fbi,
	unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int lcd_rst_n = mfp_to_gpio(GPIO83_LCD_RST);

	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_rst_n);
		return -1;
	}

	if (!lcd_pwr_ldo17) {
		lcd_pwr_ldo17 = regulator_get(NULL, "v_ldo17");
		if (IS_ERR(lcd_pwr_ldo17)) {
			lcd_pwr_ldo17 = NULL;
			printk(KERN_ERR "v_ldo17 can't open!\n");
			goto out;
		}
	}
	if (!lcd_pwr_ldo3) {
		lcd_pwr_ldo3 = regulator_get(NULL, "v_ldo3");
		if (IS_ERR(lcd_pwr_ldo3)) {
			lcd_pwr_ldo3 = NULL;
			printk(KERN_ERR "v_ldo3 can't open!\n");
			goto out1;
		}
	}
	if (!led_pwr_v5p) {
		led_pwr_v5p = regulator_get(NULL, "v_5vp");
		if (IS_ERR(led_pwr_v5p)) {
			led_pwr_v5p = NULL;
			printk(KERN_ERR "v_5vp can't open!\n");
			goto out2;
		}
	}

	if (on) {
		/* enable regulator LDO17 to power VDDC and VDD_LVDS*_12 */
		regulator_set_voltage(lcd_pwr_ldo17, 1200000, 1200000);
		regulator_enable(lcd_pwr_ldo17);

		/* enable LDO3 to power AVDD12_DSI */
		regulator_set_voltage(lcd_pwr_ldo3, 1200000, 1200000);
		regulator_enable(lcd_pwr_ldo3);

		/* release reset */
		gpio_direction_output(lcd_rst_n, 1);

		/* enable 5V power supply */
		regulator_enable(led_pwr_v5p);

		/* config mfp GPIO99/GPIO100 as twsi5 */
		lcd_twsi5_set(1);
	} else {
		/* config mfp GPIO99/GPIO100 as normal gpio */
		lcd_twsi5_set(0);

		/* disable 5V power supply */
		regulator_disable(led_pwr_v5p);

		/* keep reset */
		gpio_direction_output(lcd_rst_n, 0);

		/* disable AVDD12_DSI voltage */
		regulator_disable(lcd_pwr_ldo3);

		/* disable regulator LDO17 */
		regulator_disable(lcd_pwr_ldo17);
	}

	gpio_free(lcd_rst_n);

	pr_debug("%s on %d\n", __func__, on);
	return 0;

out2:
	regulator_disable(lcd_pwr_ldo3);
	regulator_put(lcd_pwr_ldo3);
	lcd_pwr_ldo3 = NULL;
out1:
	regulator_disable(lcd_pwr_ldo17);
	regulator_put(lcd_pwr_ldo17);
	lcd_pwr_ldo17 = NULL;
out:
	gpio_free(lcd_rst_n);
	return -EIO;
}

#define LCD_ISR_CLEAR_MASK_PXA168   0xffffffff

static struct pxa168fb_mach_info mmp2_mipi_lcd_info __initdata = {
	.id			= "GFX Layer",
	.sclk_src		= 260000000,	/* 266MHz */
	.sclk_div		= 0x40000108,
	.num_modes		= ARRAY_SIZE(video_modes_brownstone),
	.modes			= video_modes_brownstone,
	.pix_fmt		= PIX_FMT_RGB565,
	.isr_clear_mask	= LCD_ISR_CLEAR_MASK_PXA168,
	/*
	 * don't care about io_pin_allocation_mode and dumb_mode
	 * since the panel is hard connected with lcd panel path and dsi1 output
	 */
	.io_pad_ctrl = CFG_CYC_BURST_LEN16,
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 0,
	.active			= 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.mmap			= 1,
	.max_fb_size		= 1280 * 720 * 8 + 4096,
	.vdma_enable		= 1,
	.sram_size		= 30 * 1024,
	.phy_type		= DSI2DPI,
	.phy_init		= dsi_init,
#ifdef CONFIG_TC35876X
	.dsi2dpi_set		= dsi_set_tc358765,
	.xcvr_reset		= tc358765_reset,
#else
#error Please select CONFIG_TC35876X in menuconfig to enable DSI bridge
#endif
	.phy_info		= &dsiinfo,
	.pxa168fb_lcd_power     = &brownstone_lcd_power,
#ifdef CONFIG_PXA688_CMU
	.cmu_cal = {{-1, 47, 2, 2},
		{44, 92, 25, 25},
		{0, 0, 0, 0}
	},
	.cmu_cal_letter_box = {{48, 47, 2, 2},
		{93, 92, 25, 25},
		{0, 0, 0, 0}
	},
	.ioctl			= pxa688_cmu_ioctl,
#endif
};

static struct pxa168fb_mach_info mmp2_mipi_lcd_ovly_info __initdata = {
	.id			= "Video Layer",
	.num_modes		= ARRAY_SIZE(video_modes_brownstone),
	.modes			= video_modes_brownstone,
	.pix_fmt		= PIX_FMT_RGB565,
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 0,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.mmap			= 0,
	.max_fb_size            = 1280 * 720 * 8 + 4096,
	.vdma_enable		= 0,
	.sram_size		= 30 * 1024,
};


#define SDRAM_CONFIG0_TYPE1 0x0020	/* MMP2 */

void __init brownstone_add_lcd_mipi(void)
{
	struct dsi_info *dsi = &dsiinfo;
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL;
	struct pxa168fb_mach_info *mi;

	if (board_is_mmp2_brownstone_rev5()) {
		video_modes_brownstone[0].yres = 800;
		video_modes_brownstone[0].lower_margin = 35;
		video_modes_brownstone[0].upper_margin = 15;

		mi = &mmp2_mipi_lcd_info;
		mi->max_fb_size = video_modes_brownstone[0].xres *
		video_modes_brownstone[0].yres * 8 + 4096;
		mi = &mmp2_mipi_lcd_ovly_info;
		mi->max_fb_size = video_modes_brownstone[0].xres *
		video_modes_brownstone[0].yres * 8 + 4096;
	}

	/*FIXME: set 24bpp output by default for mmp2 */
	dsi->bpp = 24;

	if (dsi->bpp == 24) {
		mmp2_mipi_lcd_info.sclk_src = 800000000 / (dsi->lanes / 2);
		dsi->rgb_mode = DSI_LCD_INPUT_DATA_RGB_MODE_888;
	} else {
		mmp2_mipi_lcd_info.sclk_src = 520000000 / (dsi->lanes / 2);
		dsi->rgb_mode = DSI_LCD_INPUT_DATA_RGB_MODE_565;
		video_modes_brownstone[0].right_margin = (dsi->lanes == 4) ? 315 : 178;
	}
	mmp2_mipi_lcd_info.sclk_div = 0x40000100 | ((dsi->lanes == 4) ? 4 : 8);

	dmc_membase = ioremap(DDR_MEM_CTRL_BASE, 0xfff);
	CSn_NO_COL = __raw_readl(dmc_membase + SDRAM_CONFIG0_TYPE1) >> 4;
	CSn_NO_COL &= 0xf;
	if (CSn_NO_COL <= 0x2) {
		/*
		*if DDR page size < 4KB, select no crossing 1KB boundary check
		*/
		mmp2_mipi_lcd_info.io_pad_ctrl |= CFG_BOUNDARY_1KB;
	}
	iounmap(dmc_membase);

#ifdef CONFIG_FB_PXA168
	/* lcd */
	mmp2_add_fb(&mmp2_mipi_lcd_info);
	mmp2_add_fb_ovly(&mmp2_mipi_lcd_ovly_info);
#endif
	vsmooth_init(1, 2);
}
#endif
