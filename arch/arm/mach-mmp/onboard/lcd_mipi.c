#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2.h>
#include <mach/mmp3.h>
#include <mach/tc35876x.h>
#include <mach/pxa168fb.h>
#include "../common.h"

static struct fb_videomode video_modes_abilene[] = {
	/* innolux WVGA mode info */
	[0] = {
		.refresh = 60,
		.xres = 1280,
		.yres = 720,
		.hsync_len = 2,
		.left_margin = 10,
		.right_margin = 216,
		.vsync_len = 2,
		.upper_margin = 10,
		.lower_margin = 4,
		.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		},
};

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

static int tc358765_reset(struct pxa168fb_info *fbi)
{
	int gpio;

#ifdef CONFIG_MACH_BROWNSTONE
	gpio = mfp_to_gpio(GPIO83_LCD_RST);
#endif

#if defined(CONFIG_MACH_ABILENE) || defined(CONFIG_MACH_YELLOWSTONE)
	gpio = mfp_to_gpio(GPIO128_LCD_RST);
#endif

	if (gpio_request(gpio, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 0);
	mdelay(100);
	gpio_direction_output(gpio, 1);
	mdelay(100);

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
#ifdef CONFIG_TC35876X
	struct fb_var_screeninfo *var = &(fbi->fb_info->var);
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_info *di = mi->dsi;
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

	/* REG 0x450,DAT 0x00012020, VSDELAY = 8 pixels */
	tc35876x_write32(VPCTRL, 0x00800020);

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
#endif
	return 0;
}

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
	if (on)
		gpio_direction_output(lcd_rst_n, 1);
	else
		gpio_direction_output(lcd_rst_n, 0);
	gpio_free(lcd_rst_n);

	printk(KERN_DEBUG "%s on %d\n", __func__, on);
	return 0;
}

static int dsi_init(struct pxa168fb_info *fbi)
{
#ifdef CONFIG_PXA688_DSI
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int ret = 0;

	/* reset DSI controller */
	dsi_reset(fbi, 1);
	mdelay(1);

	/* disable continuous clock */
	dsi_cclk_set(fbi, 0);

	/* dsi out of reset */
	dsi_reset(fbi, 0);

	/* set dphy */
	dsi_set_dphy(fbi);

	/* put all lanes to LP-11 state  */
	dsi_lanes_enable(fbi, 0);
	dsi_lanes_enable(fbi, 1);

	/*  reset the bridge */
	if (mi->xcvr_reset)
		mi->xcvr_reset(fbi);

	mdelay(10);
	/* set dsi controller */
	dsi_set_controller(fbi);

	/* turn on DSI continuous clock */
	dsi_cclk_set(fbi, 1);

	/* set dsi to dpi conversion chip */
	if (mi->phy_type == DSI2DPI) {
		ret = mi->dsi2dpi_set(fbi);
		if (ret < 0)
			pr_err("dsi2dpi_set error!\n");
	}
#endif
	return 0;
}

#define		LCD_ISR_CLEAR_MASK		0xffff00cc

static struct pxa168fb_mach_info mipi_lcd_info = {
	.id = "GFX Layer",
	.num_modes = 0,
	.modes = NULL,
	.sclk_div = 0xE0001108,
	.pix_fmt = PIX_FMT_RGB565,
	.isr_clear_mask	= LCD_ISR_CLEAR_MASK,
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
	.sram_paddr = 0,
	.sram_size  = 30 * 1024,
	.max_fb_size = 0,
	.phy_type = DSI2DPI,
	.phy_init = dsi_init,
	.dsi2dpi_set = dsi_set_tc358765,
	.xcvr_reset = tc358765_reset,
	.dsi = &dsiinfo,
	.pxa168fb_lcd_power = &abilene_lcd_power,
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
};

#define     DSI1_BITCLK(div)			((div)<<8)
#define     DSI1_BITCLK_DIV_MASK		0x00000F00
#define     CLK_INT_DIV(div)			(div)
#define     CLK_INT_DIV_MASK			0x000000FF
static void calculate_lcd_sclk(struct pxa168fb_mach_info *mi)
{
	struct dsi_info *di = mi->dsi;
	struct fb_videomode *modes = &mi->modes[0];
	u32 total_w, total_h, pclk2bclk_rate, byteclk, bitclk,
	    pclk_div, bitclk_div = 1;

	if (!di)
		return;

	if (di->lanes == 4) {
		if (di->bpp == 16)
			modes->right_margin = 325;
		else if (di->bpp == 24)
			modes->right_margin = 206;
	} else if (di->lanes == 2) {
		if (di->bpp == 16)
			modes->right_margin = 179;
		else if (di->bpp == 24)
			modes->right_margin = 116;
	}

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

#define DDR_MEM_CTRL_BASE 0xD0000000
#define SDRAM_CONFIG_TYPE1_CS0 0x20	/* MMP3 */

#ifdef CONFIG_MACH_ABILENE
void __init abilene_add_lcd_mipi(void)
{
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_abilene);
	fb->modes = video_modes_abilene;
	fb->max_fb_size = video_modes_abilene[0].xres *
		video_modes_abilene[0].xres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;

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
}
#endif

#ifdef CONFIG_MACH_YELLOWSTONE
void __init yellowstone_add_lcd_mipi(void)
{
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL;

	struct pxa168fb_mach_info *fb = &mipi_lcd_info, *ovly =
	    &mipi_lcd_ovly_info;

	fb->num_modes = ARRAY_SIZE(video_modes_yellowstone);
	fb->modes = video_modes_yellowstone;
	fb->max_fb_size = video_modes_yellowstone[0].xres *
		video_modes_abilene[0].xres * 8 + 4096;
	ovly->num_modes = fb->num_modes;
	ovly->modes = fb->modes;
	ovly->max_fb_size = fb->max_fb_size;

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
		.sync		= FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
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

	if (on) {
		/* enable regulator LDO17 to power VDDC and VDD_LVDS*_12 */
		lcd_pwr_ldo17 = regulator_get(NULL, "v_ldo17");
		if (IS_ERR(lcd_pwr_ldo17)) {
			lcd_pwr_ldo17 = NULL;
			printk(KERN_ERR "v_ldo17 can't open!\n");
			goto out;
		} else {
			regulator_set_voltage(lcd_pwr_ldo17, 1200000, 1200000);
			regulator_enable(lcd_pwr_ldo17);
		}

		/* enable LDO3 to power AVDD12_DSI */
		lcd_pwr_ldo3 = regulator_get(NULL, "v_ldo3");
		if (IS_ERR(lcd_pwr_ldo3)) {
			lcd_pwr_ldo3 = NULL;
			printk(KERN_ERR "v_ldo3 can't open!\n");
			goto out1;
		} else {
			regulator_set_voltage(lcd_pwr_ldo3, 1200000, 1200000);
			regulator_enable(lcd_pwr_ldo3);
		}

		/* release reset */
		gpio_direction_output(lcd_rst_n, 1);

		/* enable 5V power supply */
		led_pwr_v5p = regulator_get(NULL, "v_5vp");
		if (IS_ERR(led_pwr_v5p)) {
			led_pwr_v5p = NULL;
			printk(KERN_ERR "v_5vp can't open!\n");
			goto out2;
		} else
			regulator_enable(led_pwr_v5p);

		/* config mfp GPIO99/GPIO100 as twsi5 */
		lcd_twsi5_set(1);
	} else {
		/* config mfp GPIO99/GPIO100 as normal gpio */
		lcd_twsi5_set(0);

		/* disable 5V power supply */
		regulator_disable(led_pwr_v5p);
		regulator_put(led_pwr_v5p);

		/* keep reset */
		gpio_direction_output(lcd_rst_n, 0);

		/* disable AVDD12_DSI voltage */
		regulator_disable(lcd_pwr_ldo3);
		regulator_put(lcd_pwr_ldo3);

		/* disable regulator LDO17 */
		regulator_disable(lcd_pwr_ldo17);
		regulator_put(lcd_pwr_ldo17);
	}

	gpio_free(lcd_rst_n);

	pr_debug("%s on %d\n", __func__, on);
	return 0;

out2:
	regulator_disable(lcd_pwr_ldo3);
	regulator_put(lcd_pwr_ldo3);
out1:
	regulator_disable(lcd_pwr_ldo17);
	regulator_put(lcd_pwr_ldo17);
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
	.phy_type		= DSI2DPI,
	.phy_init		= dsi_init,
#ifdef CONFIG_TC35876X
	.dsi2dpi_set		= dsi_set_tc358765,
	.xcvr_reset		= tc358765_reset,
#else
#error Please select CONFIG_TC35876X in menuconfig to enable DSI bridge
#endif
	.dsi			= &dsiinfo,
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
};


#define SDRAM_CONFIG0_TYPE1 0x0020	/* MMP2 */

void __init brownstone_add_lcd_mipi(void)
{
	struct dsi_info *dsi = &dsiinfo;
	unsigned char __iomem *dmc_membase;
	unsigned int CSn_NO_COL;

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
#endif
}
#endif
