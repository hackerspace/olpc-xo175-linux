/*
 * linux/drivers/video/pxa168fb.c -- Marvell PXA168 LCD Controller
 *
 *  Copyright (C) 2008 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2009-02-16  adapted from original version for PXA168
 *		Green Wan <gwan@marvell.com>
 *              Jun Nie <njun@marvell.com>
 *		Kevin Liu <kliu5@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <plat/pm.h>
#include "pxa168fb_common.h"

#ifdef CONFIG_CPU_MMP2
#include <mach/mmp2_pm.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


/* interrupt timestamp collection to get:
 * 0. ITC_NONE		don't enable any timestamp collection
 * 1. ITC_INTERVAL	collect timestamps each time enter lcd interrupt
 *			service, normally only display done interrupt is
 *			enabled, should be same as LCD refresh period;
 *			while test results shows it takes longer time
 * 2. ITC_VSYNC		collect timestamp for display done interrupt and
 *			vsync interrupt
 * 3. ITC_GFX_DONE	collect timestamp for display done interrupt and
 *			graphics layer frame done(dma finish) interrupt
 * 4. ITC_VID_DONE	collect timestamp for display done interrupt and
 *			video layer frame done(dma finish) interrupt
 * 5. ITC_HANDLER	collect timestamp for display done interrupt and
 *			main handler finish time
 * usage:
 * "echo x > /sys/devices/platform/pxa168-fb.0/itc" to enable/disable
 * "cat /sys/devices/platform/pxa168-fb.0/itc" to get timestamps
 *
 * note: only panel path timestamp collection is supported now
 */
#define ITC_MAX_NUM		32
#define ITC_NONE		(irqtm_check == 0)
#define ITC_INTERVAL		(irqtm_check == 1)
#define ITC_VSYNC		(irqtm_check == 2)
#define ITC_GFX_DONE		(irqtm_check == 3)
#define ITC_VID_DONE		(irqtm_check == 4)
#define ITC_HANDLER		(irqtm_check == 5)
#define gettime(val, count, update) do {			\
	count %= ITC_MAX_NUM;					\
	do_gettimeofday(&val[update ? count++ : count]);	\
} while (0)
static int ct1, ct2, irqtm_check, irqtm_adjust = 1000;
static struct timeval t0, t1[ITC_MAX_NUM], t2[ITC_MAX_NUM];

/* interrupt number collection to get real frame rate */
#define VSYNC_CHECK_TIME	(10 * HZ)
#define DEBUG_VSYNC_PATH(id)	(gfx_info.fbi[(id)]->debug & 3)
#define DEBUG_ERR_IRQ(id)	(gfx_info.fbi[(id)]->debug & 4)
static int gfx_udflow_count;
static int vid_udflow_count;
static int axi_err_count;
static int vsync_check;
static int irq_count;
static int vsync_count;
static int dispd_count;
static int f0_count;
static int f1_count;
static int vf0_count;
static int vf1_count;
static struct timer_list vsync_timer;

static void vsync_check_timer(unsigned long data)
{
	int id = DEBUG_VSYNC_PATH(0);

	/* disable count interrupts */
	irq_mask_set(id, vsync_imask(id) | gf0_imask(id) |
		gf1_imask(id) | vf0_imask(id) | vf1_imask(id), 0);

	vsync_check = 0;
	del_timer(&vsync_timer);
	pr_info("fbi %d: irq_count %d\n", id, irq_count);
	pr_info("\tvsync_count %d\n",  vsync_count);
	pr_info("\tdispd_count %d\n",  dispd_count);
	pr_info("\tf0_count %d\n", f0_count);
	pr_info("\tf1_count %d\n", f1_count);
	pr_info("\tvf0_count %d\n", vf0_count);
	pr_info("\tvf1_count %d\n", vf1_count);
}

static void vsync_check_count(void)
{
	int path = DEBUG_VSYNC_PATH(0);
	int mask = vsync_imask(path) | gf0_imask(path) | gf1_imask(path)
		| vf0_imask(path) | vf1_imask(path);

	/* enable count interrupts */
	irq_mask_set(path, mask, mask);
	irq_status_clear(path, mask);

	/* clear counts */
	vsync_count = dispd_count = irq_count = 0;
	f0_count = f1_count = vf0_count = vf1_count = 0;
	ct1 = 0; memset(t1, 0, sizeof(t1));
	ct2 = 0; memset(t2, 0, sizeof(t2));

	/* trigger vsync check */
	vsync_check = 1;
	init_timer(&vsync_timer);
	vsync_timer.function = vsync_check_timer;
	mod_timer(&vsync_timer, jiffies + 10*HZ);
}

#define DEFAULT_REFRESH		60	/* Hz */

static unsigned int max_fb_size = 0;
static unsigned int fb_size_from_cmd = 0;

/* LCD mode switch
 * fb_share mode:
 * TV path graphics layer share same frame buffer with panel path
 */
int fb_share = 0;
struct fbi_info gfx_info;

struct lcd_regs *get_regs(int id)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[0];
	struct lcd_regs *regs;

	if (!fbi)
		return NULL;

	regs = (struct lcd_regs *)((unsigned)fbi->reg_base);

	if (id == 0)
		regs = (struct lcd_regs *)((unsigned)fbi->reg_base + 0xc0);
	if (id == 2)
		regs = (struct lcd_regs *)((unsigned)fbi->reg_base + 0x200);

	return regs;
}
u32 dma_ctrl_read(int id, int ctrl1)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[0];
	u32 reg = (u32)fbi->reg_base + dma_ctrl(ctrl1, id);

	return __raw_readl(reg);
}

void dma_ctrl_write(int id, int ctrl1, u32 value)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[0];
	u32 reg = (u32)fbi->reg_base + dma_ctrl(ctrl1, id);

	__raw_writel(value, reg);
}

void dma_ctrl_set(int id, int ctrl1, u32 mask, u32 value)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[0];
	u32 reg = (u32)fbi->reg_base + dma_ctrl(ctrl1, id);
	u32 tmp1, tmp2;

	tmp1 = tmp2 = __raw_readl(reg);
	tmp2 &= ~mask;
	tmp2 |= value;
	if (tmp1 != tmp2)
		__raw_writel(tmp2, reg);
}

void irq_mask_set(int id, u32 mask, u32 val)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[0];
	u32 temp = readl(fbi->reg_base + SPU_IRQ_ENA);

	temp &= ~mask; temp |= val;
	writel(temp, fbi->reg_base + SPU_IRQ_ENA);
}

void irq_status_clear(int id, u32 mask)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[0];
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	writel(mi->isr_clear_mask & (~mask), fbi->reg_base + SPU_IRQ_ISR);
}

u32 clk_reg(int id, u32 type)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[0];
	u32 offset = 0;

	switch (type) {
	case clk_sclk:
		offset = id ? (id & 1 ? LCD_TCLK_DIV : LCD_PN2_SCLK_DIV)
				: LCD_CFG_SCLK_DIV;
		break;
	case clk_lvds_rd:
		offset = LCD_LVDS_SCLK_DIV_RD;
		break;
	case clk_lvds_wr:
		offset = LCD_LVDS_SCLK_DIV_WR;
		break;
	case clk_tclk:
		if (id == 1)
			offset = LCD_TCLK_DIV;
		else if (id == 2)
			offset = LCD_PN2_TCLK_DIV;
		break;
	default:
		pr_err("%s path %d type %x not found\n", __func__, id, type);
		break;
	}

	if (offset)
		return (u32)fbi->reg_base + offset;

	return 0;
}

int lcd_clk_get(int id, u32 type)
{
	u32 reg = clk_reg(id, type), val;

	if (!reg)
		return 0;

	val = __raw_readl(reg);
	pr_debug("%s path %d type %d: 0x%x (@ 0x%x)\n", __func__,
		id, type, val, clk_reg(id, type));
	return val;
}

void lcd_clk_set(int id, u32 type, u32 mask, u32 val)
{
	u32 reg = clk_reg(id, type), tmp1, tmp2;

	if (!reg)
		return;

	tmp1 = tmp2 = __raw_readl(reg);
	tmp2 &= ~mask;
	tmp2 |= val;
	if (tmp1 != tmp2)
		__raw_writel(tmp2, reg);

	pr_debug("%s type %d mask %x val %x: 0x%x -> 0x%x (@ 0x%x)\n",
		__func__, type, mask, val, tmp1, tmp2,
		clk_reg(id, type) & 0xfff);
}

int pxa168fb_spi_send(struct pxa168fb_info *fbi, void *value,
		int count, unsigned int spi_gpio_cs)
{
	u32 x, spi_byte_len;
	u8 *cmd = (u8 *)value;
	int i, err, isr, iopad;
	unsigned int timeout = 0;

	if (spi_gpio_cs != -1) {
		err = gpio_request(spi_gpio_cs, "LCD_SPI_CS");
		if (err) {
			pr_err("failed to request GPIO for LCD CS\n");
			return -1;
		}
		gpio_direction_output(spi_gpio_cs, 1);
	}
	/* get spi data size */
	spi_byte_len = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
	spi_byte_len = (spi_byte_len >> 8) & 0xff;
	/* It should be (spi_byte_len + 7) >> 3, but spi controller
	 * request set one less than bit length */
	spi_byte_len = (spi_byte_len + 8) >> 3;
	/* spi command provided by platform should be 1, 2, or 4 byte aligned */
	if (spi_byte_len == 3)
		spi_byte_len = 4;

	/*After set mode1 it need a time to pull up the spi singals,
	 * or it would cause the wrong waveform when send spi command,
	 * especially on pxa910h*/
	iopad = readl(fbi->reg_base + SPU_IOPAD_CONTROL);
	if ((iopad & CFG_IOPADMODE_MASK) != PIN_MODE_DUMB_18_SPI)
		writel(PIN_MODE_DUMB_18_SPI |
			(iopad & ~CFG_IOPADMODE_MASK),
			fbi->reg_base + SPU_IOPAD_CONTROL);
	udelay(20);

	for (i = 0; i < count; i++) {
		if (spi_gpio_cs != -1)
			gpio_direction_output(spi_gpio_cs, 0);

		irq_status_clear(fbi->id, SPI_IRQ_MASK);

		switch (spi_byte_len) {
		case 1:
			writel(*cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		case 2:
			writel(*(u16 *)cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		case 4:
			writel(*(u32 *)cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		default:
			pr_err("Wrong spi bit length\n");
		}
		cmd += spi_byte_len;
		x = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
		x |= 0x1;
		writel(x, fbi->reg_base + LCD_SPU_SPI_CTRL);
		isr = readl(fbi->reg_base + SPU_IRQ_ISR);
		timeout = 0;
		while (!(isr & SPI_IRQ_ENA_MASK)) {
			udelay(100);
			isr = readl(fbi->reg_base + SPU_IRQ_ISR);
			if (timeout++ > 100) {
				pr_err("SPI IRQ may miss, just skip and send "
				"the following command, count %d!\n", i);
				break;
			}
		}
		irq_status_clear(fbi->id, SPI_IRQ_MASK);
		x = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
		x &= ~0x1;
		writel(x, fbi->reg_base + LCD_SPU_SPI_CTRL);
		if (spi_gpio_cs != -1)
			gpio_direction_output(spi_gpio_cs, 1);
	}

	if ((iopad & CFG_IOPADMODE_MASK) != PIN_MODE_DUMB_18_SPI)
		writel(iopad, fbi->reg_base + SPU_IOPAD_CONTROL);

	if (spi_gpio_cs != -1)
		gpio_free(spi_gpio_cs);
	return 0;
}

static int pxa168fb_power(struct pxa168fb_info *fbi,
		struct pxa168fb_mach_info *mi, int on)
{
	int ret = 0;
	pr_debug("fbi->active %d on %d\n", fbi->active, on);
	if ((mi->spi_ctrl != -1) && (mi->spi_ctrl & CFG_SPI_ENA_MASK))
		writel(mi->spi_ctrl, fbi->reg_base + LCD_SPU_SPI_CTRL);

	if ((mi->pxa168fb_lcd_power) && \
		((fbi->active && !on) || (!fbi->active && on)))
		ret = mi->pxa168fb_lcd_power(fbi, mi->spi_gpio_cs,
					mi->spi_gpio_reset, on);

	if ((mi->spi_ctrl != -1) && (mi->spi_ctrl & CFG_SPI_ENA_MASK))
		writel(mi->spi_ctrl & ~CFG_SPI_ENA_MASK,
			fbi->reg_base + LCD_SPU_SPI_CTRL);
	return ret;
}

static void set_mode(struct pxa168fb_info *fbi, struct fb_var_screeninfo *var,
		     const struct fb_videomode *mode, int pix_fmt, int ystretch)
{
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __func__);
	set_pix_fmt(var, pix_fmt);

	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = max(var->xres, var->xres_virtual);
	if (ystretch && !fb_share)
		var->yres_virtual = var->yres * 2;
	else
		var->yres_virtual = max(var->yres, var->yres_virtual);
	var->grayscale = 0;
	var->accel_flags = FB_ACCEL_NONE;
	var->pixclock = mode->pixclock;
	var->left_margin = mode->left_margin;
	var->right_margin = mode->right_margin;
	var->upper_margin = mode->upper_margin;
	var->lower_margin = mode->lower_margin;
	var->hsync_len = mode->hsync_len;
	var->vsync_len = mode->vsync_len;
	var->sync = mode->sync;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->rotate = FB_ROTATE_UR;
}

/*
 * The hardware clock divider has an integer and a fractional
 * stage:
 *
 *	clk2 = clk_in / integer_divider
 *	clk_out = clk2 * (1 - (fractional_divider >> 12))
 *
 * Calculate integer and fractional divider for given clk_in
 * and clk_out.
 */
static void set_clock_divider(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	u32 divider_int, needed_pixclk, val, x = 0;
	u64 div_result;

	/* check whether divider is fixed by platform */
	if (mi->sclk_div) {
		val = mi->sclk_div;
		/*for 480i and 576i, pixel clock should be half of
		 * the spec value because of pixel repetition */
		if ((var->yres == 480 || var->yres == 576) &&
			(var->vmode == FB_VMODE_INTERLACED)) {
			val &= ~0xf;
			val |= (mi->sclk_div & 0xf) << 1;
		}

		lcd_clk_set(fbi->id, clk_sclk, 0xffffffff, val);
		if (mi->phy_type & LVDS)
			lcd_clk_set(fbi->id, clk_lvds_wr, 0xffffffff, val);
		if (!var->pixclock) {
			divider_int = mi->sclk_div & CLK_INT_DIV_MASK;
			if (!divider_int)
				divider_int = 1;

			if (!clk_get_rate(fbi->clk)) {
				pr_err("%s: fbi->clk get rate null\n",
						__func__);
				return;
			}

			x = clk_get_rate(fbi->clk) / divider_int / 1000;
			var->pixclock = 1000000000 / x;
			pr_debug("%s pixclock %d x %d divider_int %d\n",
				__func__, var->pixclock, x, divider_int);
		}
		x = var->xres + var->left_margin + var->hsync_len +
			var->right_margin;
		fbi->frm_usec = var->pixclock * (var->yres + var->upper_margin
			+ var->vsync_len + var->lower_margin) / 1000 * x / 1000;
		return;
	}

	/* Notice: The field pixclock is used by linux fb
	 * is in pixel second. E.g. struct fb_videomode &
	 * struct fb_var_screeninfo
	 */

	/* Check input values */
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __func__);
	if (!var->pixclock) {
		pr_err("Input refresh or pixclock is wrong.\n");
		return;
	}

	/* Using PLL/AXI clock. */
	x = 0x40000000;

	/* Calc divider according to refresh rate */
	div_result = 1000000000000ll;
	do_div(div_result, var->pixclock);
	needed_pixclk = (u32)div_result;

	divider_int = clk_get_rate(fbi->clk) / needed_pixclk;

	/* check whether divisor is too small. */
	if (divider_int < 2) {
		pr_warning("Warning: clock source is too slow."
				 "Try smaller resolution\n");
		divider_int = 2;
	}

	/* Set setting to reg */
	x |= divider_int;
	if (fbi->id != 1)
		lcd_clk_set(fbi->id, clk_sclk, CLK_INT_DIV_MASK, x);
}

void enable_graphic_layer(int id)
{
	int val;

	val = check_modex_active(gfx_info.fbi[id]);
	if (!(dma_ctrl_read(id, 0) & CFG_GRA_ENA_MASK)) {
		dma_ctrl_set(id, 0, CFG_GRA_ENA_MASK, CFG_GRA_ENA(val));
	}
}

static void set_dumb_panel_control(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	u32 x;

	dev_dbg(info->dev, "Enter %s\n", __func__);

	/* Preserve enable flag */
	x = readl(fbi->reg_base + intf_ctrl(fbi->id)) & 0x00000001;
	x |= (fbi->is_blanked ? 0x7 : mi->dumb_mode) << 28;
	if (fbi->id == 1) {
		/* enable AXI urgent flag and vblank(for 3d formater) */
		x |= (0xff << 16) | TV_VBLNK_VALID_EN;
	} else {
		x |= mi->gpio_output_data << 20;
		x |= mi->gpio_output_mask << 12;
	}
	x |= mi->panel_rgb_reverse_lanes ? 0x00000080 : 0;
	x |= mi->invert_composite_blank ? 0x00000040 : 0;
	x |= (info->var.sync & FB_SYNC_COMP_HIGH_ACT) ? 0x00000020 : 0;
	x |= mi->invert_pix_val_ena ? 0x00000010 : 0;
	x |= (info->var.sync & FB_SYNC_VERT_HIGH_ACT) ? 0 : 0x00000008;
	x |= (info->var.sync & FB_SYNC_HOR_HIGH_ACT) ? 0 : 0x00000004;
	x |= mi->invert_pixclock ? 0x00000002 : 0;
	x |= 0x1;
	writel(x, fbi->reg_base + intf_ctrl(fbi->id));	/* FIXME */
}

static void set_tv_interlace(void)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[1];
	struct fb_info *info = fbi->fb_info;
	struct fb_var_screeninfo *v = &info->var;
	struct lcd_regs *regs = get_regs(fbi->id);
	int x, y, yres, interlaced = 0, vsync_ctrl;
	static int vdma_enabled, vdma_layer;
	struct pxa168fb_vdma_info *lcd_vdma = 0;

	dev_dbg(info->dev, "Enter %s\n", __func__);

	if (v->vmode & FB_VMODE_INTERLACED) {
		/* enable interlaced mode */
		interlaced = CFG_TV_INTERLACE_EN | CFG_TV_NIB;

		x = v->xres + v->right_margin + v->hsync_len + v->left_margin;

		/* interlaced mode, recalculate vertical pixels */
		yres = v->yres >> 1;
		y = yres + v->lower_margin + v->vsync_len + v->upper_margin;

		/* even field */
		writel(((y + 1) << 16) | yres,
			fbi->reg_base + LCD_TV_V_H_TOTAL_FLD);
		writel(((v->upper_margin) << 16) | (v->lower_margin),
			fbi->reg_base + LCD_TV_V_PORCH_FLD);
		vsync_ctrl = (x >> 1) - v->left_margin - v->hsync_len;
		writel(vsync_ctrl << 16 | vsync_ctrl,
			fbi->reg_base + LCD_TV_SEPXLCNT_FLD);

		/* odd field */
		writel((yres << 16) | v->xres, &regs->screen_active);
		writel((y << 16) | x, &regs->screen_size);
	}
	dma_ctrl_set(fbi->id, 1, CFG_TV_INTERLACE_EN | CFG_TV_NIB, interlaced);

	lcd_vdma = request_vdma(fbi->id, 0);
	if (!lcd_vdma) {
		lcd_vdma = request_vdma(fbi->id, 1);
		if (!lcd_vdma)
			return;
	}
	if (v->vmode & FB_VMODE_INTERLACED) {
		/* interlaced mode, TV path VDMA should be disabled */
		if (lcd_vdma->enable) {
			vdma_enabled = 1;
			vdma_layer = lcd_vdma->vid;
			pxa688_vdma_en(lcd_vdma, 0, lcd_vdma->vid);
		}
	} else if (vdma_enabled) {
		/* vdma recovery */
		vdma_enabled = 0;
		pxa688_vdma_en(lcd_vdma, 1, vdma_layer);
	}
}

static void set_dumb_screen_dimensions(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *v = &info->var;
	struct lcd_regs *regs = get_regs(fbi->id);
	struct dsi_info *di = (struct dsi_info *)mi->phy_info;
	int x, y, h_porch, vec = 10, vsync_ctrl;

	dev_dbg(info->dev, "Enter %s fb %d regs->screen_active 0x%p\n",
			__func__, fbi->id, &regs->screen_active);

	/* resolution, active */
	writel((v->yres << 16) | v->xres, &regs->screen_active);

	if (mi->phy_type & (DSI2DPI | DSI)) {
		vec = ((di->lanes <= 2) ? 1 : 2) * 10 * di->bpp / 8 / di->lanes;

		h_porch = (v->xres + v->right_margin) * vec / 10 - v->xres;
		h_porch = (v->left_margin * vec / 10) << 16 | h_porch;
		vsync_ctrl = 0x01330133;
	} else {
		h_porch = (v->left_margin) << 16 | v->right_margin;

		if ((fbi->id == 0) || (fbi->id == 2))
			vsync_ctrl = ((v->width + v->left_margin) << 16)
				| (v->width + v->left_margin);
		else
			vsync_ctrl = ((v->xres + v->right_margin) << 16)
				| (v->xres + v->right_margin);
	}
	/* h porch, left/right margin */
	writel(h_porch, &regs->screen_h_porch);

	/* v porch, upper/lower margin */
	writel((v->upper_margin << 16) | v->lower_margin,
			&regs->screen_v_porch);

	x = v->xres + v->right_margin + v->hsync_len + v->left_margin;
	x = x * vec / 10;
	y = v->yres + v->lower_margin + v->vsync_len + v->upper_margin;
	/* screen total size */
	writel((y << 16) | x, &regs->screen_size);

	/* vsync ctrl */
	writel(vsync_ctrl, &regs->vsync_ctrl);	/* FIXME */

}

static void pxa168fb_clear_framebuffer(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;

	memset(fbi->fb_start, 0, fbi->fb_size);
}

static int pxa168fb_set_par(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct regshadow *shadowreg = &fbi->shadowreg;
	int pix_fmt;
	u32 flags;

	dev_dbg(info->dev, "Enter %s, graphics layer\n", __func__);

	/* Determine which pixel format we're going to use */
	pix_fmt = determine_best_pix_fmt(var, fbi);
	if (pix_fmt < 0)
		return pix_fmt;
	fbi->pix_fmt = pix_fmt;
	set_pix_fmt(var, pix_fmt);

	if (!var->xres_virtual)
		var->xres_virtual = var->xres;
	if (!var->yres_virtual)
		var->yres_virtual = var->yres * 2;
	var->grayscale = 0;
	var->accel_flags = FB_ACCEL_NONE;
	var->rotate = FB_ROTATE_UR;

	/* Set additional mode info */
	if (fbi->pix_fmt == PIX_FMT_PSEUDOCOLOR)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;

	set_dumb_screen_dimensions(info);

	/* Calculate clock divisor. */
	set_clock_divider(fbi);

	/* Configure dumb panel ctrl regs & timings */
	set_dumb_panel_control(info);

	if (gfx_info.fbi[1] && (fbi->id == 1))
		set_tv_interlace();

	flags = UPDATE_ADDR | UPDATE_MODE | UPDATE_VIEW;
	pxa168fb_set_var(info, shadowreg, flags);

	if (!NEED_VSYNC(fbi))
		pxa168fb_set_regs(fbi, shadowreg);

	return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	return ((chan & 0xffff) >> (16 - bf->length)) << bf->offset;
}

static u32 to_rgb(u16 red, u16 green, u16 blue)
{
	red >>= 8; green >>= 8; blue >>= 8;

	return (red << 16) | (green << 8) | blue;
}

static int
pxa168fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		 unsigned int blue, unsigned int trans, struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	u32 val;

	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR && regno < 16) {
		val =  chan_to_field(red,   &info->var.red);
		val |= chan_to_field(green, &info->var.green);
		val |= chan_to_field(blue , &info->var.blue);
		fbi->pseudo_palette[regno] = val;
	}

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR && regno < 256) {
		val = to_rgb(red, green, blue);
		writel(val, fbi->reg_base + LCD_SPU_SRAM_WRDAT);
		writel(0x8300 | regno, fbi->reg_base + LCD_SPU_SRAM_CTRL);
	}

	return 0;
}

static int pxa168fb_active(struct pxa168fb_info *fbi, int active)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	dev_dbg(fbi->fb_info->dev, "Enter %s fbi[%d] active %d\n",
			__func__, fbi->id, active);
	if (!active && fbi->active) {
		atomic_set(&fbi->w_intr, 1);
		wake_up(&fbi->w_intr_wq);

		/* disable external panel power */
		if (pxa168fb_power(fbi, mi, 0))
			pr_err("%s %d pxa168fb_power control failed!\n",
				 __func__, __LINE__);

		fbi->active = 0;

		/* disable path clock */
		lcd_clk_set(fbi->id, clk_sclk, SCLK_DISABLE, SCLK_DISABLE);
	}

	if (active && !fbi->active) {
		/* enable path clock */
		lcd_clk_set(fbi->id, clk_sclk, SCLK_DISABLE, 0);

		/* enable external panel power */
		if (pxa168fb_power(fbi, mi, 1))
			pr_err("%s %d pxa168fb_power control failed!\n",
				 __func__, __LINE__);

		/* initialize external phy if needed */
		if (mi->phy_init && mi->phy_init(fbi)) {
			pr_err("%s fbi %d phy error\n", __func__, fbi->id);
			return -EIO;
		}

		fbi->active = 1;
		set_dma_active(fbi);
	}
	return 0;
}

static int pxa168fb_blank(int blank, struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
#ifdef CONFIG_PM
	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
			/* de-activate the device */
			pxa168fb_active(fbi, 0);

			/* allow system enter low power modes */
			pm_qos_update_request(&fbi->qos_idle_fb,
						PM_QOS_DEFAULT_VALUE);
			wake_unlock(&fbi->idle_lock);
			break;

	case FB_BLANK_UNBLANK:
			/* activate the device */
			pxa168fb_active(fbi, 1);

			/* avoid system enter low power modes */
			pm_qos_update_request(&fbi->qos_idle_fb,
						PM_QOS_CONSTRAINT);
			wake_lock(&fbi->idle_lock);
			break;
	default:
			break;
	}
	return 0;
#else
	fbi->is_blanked = (blank == FB_BLANK_UNBLANK) ? 0 : 1;
	set_dumb_panel_control(info);

	return 0;
#endif
}

static int pxa168fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;

	dev_dbg(info->dev, "Enter %s\n", __func__);

	set_start_address(info, var->xoffset, var->yoffset, &fbi->shadowreg);
	fbi->shadowreg.flags |= UPDATE_ADDR;
	if (fbi->shadowreg.flags == UPDATE_ADDR)
		/* only if address needs to be updated */
		pxa168fb_set_regs(fbi, &fbi->shadowreg);

	if (NEED_VSYNC(fbi))
		wait_for_vsync(fbi);

	return 0;
}

static irqreturn_t pxa168fb_handle_irq(int irq, void *dev_id)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)dev_id;
	u32 isr_en = readl(fbi->reg_base + SPU_IRQ_ISR) &
		readl(fbi->reg_base + SPU_IRQ_ENA);
	u32 id, dispd, err, sts, isr_delayed = 0;

	if (ITC_INTERVAL) {
		gettime(t1, ct1, 0);

		t0 = ct1 ? t1[ct1 - 1] : t1[ITC_MAX_NUM - 1];
		if (t0.tv_sec) {
			t2[ct2].tv_usec = t1[ct1].tv_usec -
				t0.tv_usec;
			if (t1[ct1].tv_sec > t0.tv_sec)
				t2[ct2].tv_usec += 1000000;
			if (t2[ct2].tv_usec > (gfx_info.fbi[0]->frm_usec
					+ irqtm_adjust))
				isr_delayed = 1;
		}
		ct1++; ct2++; ct2 %= ITC_MAX_NUM;
	}

	do {
		irq_status_clear(0, isr_en);
		/* display done irq */
		dispd = isr_en & display_done_imasks;
		if (dispd) {
			for (id = 0; id < 2; id++) {
				sts = dispd & display_done_imask(id);
				if (sts) {
					if (vsync_check &&
						id == DEBUG_VSYNC_PATH(0)) {
						if (!ITC_INTERVAL)
							gettime(t1, ct1, 1);
						dispd_count++;
					}

#if defined(CONFIG_CPU_MMP2) && defined(CONFIG_CPU_FREQ)
					wakeup_freq_seq();
#endif
#ifdef CONFIG_PXA168_V4L2_OVERLAY
					pxa168_v4l2_isr(id);
#endif
					pxa168_fb_isr(id);

					if (ITC_HANDLER && vsync_check &&
						id == DEBUG_VSYNC_PATH(0))
						gettime(t2, ct2, 1);
				}
			}
		}

		/* LCD under run error detect */
		err = isr_en & err_imasks;
		if (err) {
			for (id = 0; id < 3; id++) {
				if (err & gfx_udflow_imask(id)) {
					gfx_udflow_count++;
					if (DEBUG_ERR_IRQ(0))
						pr_err("fb%d gfx underflow\n", id);
				}
				if (err & vid_udflow_imask(id)) {
					vid_udflow_count++;
					if (DEBUG_ERR_IRQ(0))
						pr_err("fb%d vid underflow\n", id);
				}
			}
			if (err & AXI_BUS_ERROR_IRQ_ENA_MASK) {
				axi_err_count++;
				if (DEBUG_ERR_IRQ(0))
					pr_info("axi bus err\n");
			}
			if (err & AXI_LATENCY_TOO_LONG_IRQ_ENA_MASK) {
				axi_err_count++;
				if (DEBUG_ERR_IRQ(0))
					pr_info("axi lantency too long\n");
			}
		}

		/* count interrupts numbers in 10s */
		if (vsync_check) {
			id = DEBUG_VSYNC_PATH(0);
			if (isr_en & path_imasks(id))
				irq_count++;
			if (isr_en & gf0_imask(id)) {
				if (ITC_GFX_DONE)
					gettime(t2, ct2, 1);
				f0_count++;
			}
			if (isr_en & gf1_imask(id)) {
				if (ITC_GFX_DONE)
					gettime(t2, ct2, 1);
				f1_count++;
			}
			if (isr_en & vf0_imask(id)) {
				if (ITC_VID_DONE)
					gettime(t2, ct2, 1);
				vf0_count++;
			}
			if (isr_en & vf1_imask(id)) {
				if (ITC_VID_DONE)
					gettime(t2, ct2, 1);
				vf1_count++;
			}
			if (isr_en & vsync_imask(id)) {
				if (ITC_VSYNC)
					gettime(t2, ct2, 1);
				vsync_count++;
			}
		}
	} while (((isr_en = readl(gfx_info.fbi[0]->reg_base + SPU_IRQ_ISR)) &
			readl(gfx_info.fbi[0]->reg_base + SPU_IRQ_ENA) &
			(path_imasks(0) | path_imasks(1) | err_imasks)) &&
			!(irqtm_check && vsync_check));

	if (isr_delayed)
		printk(KERN_DEBUG "%lu.%lu - %lu.%lu) = %lu\n",
			t1[ct1].tv_sec, t1[ct1].tv_usec, t0.tv_sec,
			t0.tv_usec, t2[ct2].tv_usec);
	return IRQ_HANDLED;
}

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
static void debug_identify_called_ioctl(struct fb_info *info, int cmd,
					 unsigned long arg)
{
	switch (cmd) {
	case FB_IOCTL_CLEAR_FRAMEBUFFER:
		dev_dbg(info->dev, "FB_IOCTL_CLEAR_FRAMEBUFFER\n");
		break;
	case FB_IOCTL_PUT_SWAP_GRAPHIC_RED_BLUE:
		dev_dbg(info->dev, "FB_IOCTL_PUT_SWAP_GRAPHIC_RED_BLUE\
			 with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_SWAP_GRAPHIC_U_V:
		dev_dbg(info->dev, "FB_IOCTL_PUT_SWAP_GRAPHIC_U_V with arg = %08x\n",
			 (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_SWAP_GRAPHIC_Y_UV:
		dev_dbg(info->dev, "FB_IOCTL_PUT_SWAP_GRAPHIC_Y_UV with arg = %08x\n",
			 (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_VIDEO_ALPHABLEND:
		dev_dbg(info->dev, "FB_IOCTL_PUT_VIDEO_ALPHABLEND with arg = %08x\n",
			 (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_GLOBAL_ALPHABLEND:
		dev_dbg(info->dev, "FB_IOCTL_PUT_GLOBAL_ALPHABLEND with arg = %08x\n",
			 (unsigned int) arg);
		break;
	case FB_IOCTL_PUT_GRAPHIC_ALPHABLEND:
		dev_dbg(info->dev, "FB_IOCTL_PUT_GRAPHIC_ALPHABLEND with arg = %08x\n",
			 (unsigned int)arg);
		break;
	case FB_IOCTL_FLIP_VID_BUFFER:
		dev_dbg(info->dev, "FB_IOCTL_FLIP_VID_BUFFER with arg = %08x\n",
			 (unsigned int)arg);
		break;
	case FB_IOCTL_GET_FREELIST:
		dev_dbg(info->dev, "FB_IOCTL_GET_FREELIST with arg = %08x\n",
			 (unsigned int)arg);
		break;
	case FB_IOCTL_SWITCH_GRA_OVLY:
		dev_dbg(info->dev, "FB_IOCTL_SWITCH_GRA_OVLY with arg = %08x\n",
			 (unsigned int)arg);
		break;
	}
}
#endif

static int pxa168_graphic_ioctl(struct fb_info *info, unsigned int cmd,
				 unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int blendval;
	int val, mask, gra_on;
	unsigned long flags;
	unsigned char param;
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct mvdisp_partdisp grap;

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
	debug_identify_called_ioctl(info, cmd, arg);
#endif
	dev_dbg(info->dev, "%s cmd 0x%x\n", __func__, cmd);

	switch (cmd) {

	case FB_IOCTL_CLEAR_FRAMEBUFFER:
		pxa168fb_clear_framebuffer(info);
		break;
	case FB_IOCTL_WAIT_VSYNC:
		wait_for_vsync(fbi);
		break;
	case FB_IOCTL_WAIT_VSYNC_ON:
		fbi->wait_vsync = 1;
		break;
	case FB_IOCTL_WAIT_VSYNC_OFF:
		fbi->wait_vsync = 0;
		break;
	case FB_IOCTL_PUT_VIDEO_ALPHABLEND:
		/* This puts the blending control to the Video layer */
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(0) | CFG_ALPHA(0xff);
		dma_ctrl_set(fbi->id, 1, mask, val);
		break;

	case FB_IOCTL_PUT_GLOBAL_ALPHABLEND:
		/*  The userspace application can specify a byte value for
		 *  the amount of global blend between the video layer and
		 *  the graphic layer.
		 *  The alpha blending is per the formula below:
		 *  P = (V[P] * blendval/255) + (G[P] * (1 - blendval/255))
		 *
		 *  where: P = Pixel value, V = Video Layer, G = Graphic Layer
		 */
		blendval = (arg & 0xff);
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(2) | CFG_ALPHA(blendval);
		dma_ctrl_set(fbi->id, 1, mask, val);
		break;

	case FB_IOCTL_PUT_GRAPHIC_ALPHABLEND:
		/*  This puts the blending back to the default mode of
		 * allowing the graphic layer to do pixel level blending.
		 */
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(1) | CFG_ALPHA(0x0);
		dma_ctrl_set(fbi->id, 1, mask, val);
		break;

	case FB_IOCTL_SWAP_GRAPHIC_RED_BLUE:
		param = (arg & 0x1);
		mask = CFG_GRA_SWAPRB_MASK;
		val = CFG_GRA_SWAPRB(param);
		dma_ctrl_set(fbi->id, 0, mask, val);
		break;

	case FB_IOCTL_SWAP_GRAPHIC_U_V:
		param = (arg & 0x1);
		mask = CFG_GRA_SWAPUV_MASK;
		val = CFG_GRA_SWAPUV(param);
		dma_ctrl_set(fbi->id, 0, mask, val);
		break;

	case FB_IOCTL_SWAP_GRAPHIC_Y_UV:
		param = (arg & 0x1);
		mask = CFG_GRA_SWAPYU_MASK;
		val = CFG_GRA_SWAPYU(param);
		dma_ctrl_set(fbi->id, 0, mask, val);
		break;

	case FB_IOCTL_FLIP_VID_BUFFER:
		val = flip_buffer(info, arg);
		if (NEED_VSYNC(fbi))
			wait_for_vsync(fbi);
		return val;
	case FB_IOCTL_GET_FREELIST:
		return get_freelist(info, arg);

	case FB_IOCTL_SWITCH_GRA_OVLY:
		if (copy_from_user(&gra_on, argp, sizeof(int)))
			return -EFAULT;

		spin_lock_irqsave(&fbi->var_lock, flags);
		fbi->dma_on = gra_on ? 1 : 0;
		mask = CFG_GRA_ENA_MASK;
		val = CFG_GRA_ENA(check_modex_active(fbi));
		if (!val && gfx_info.fbi[0]->active) {
			pxa688_vdma_release(fbi->id, fbi->vid);
			/* switch off, disable DMA */
			dma_ctrl_set(fbi->id, 0, mask, val);
		}

		printk(KERN_DEBUG"SWITCH_GRA_OVLY fbi %d dma_on %d, val %d\n",
			fbi->id, fbi->dma_on, val);

		spin_unlock_irqrestore(&fbi->var_lock, flags);
		break;

	case FB_IOCTL_GRA_PARTDISP:
		if (copy_from_user(&grap, argp, sizeof(grap)))
			return -EFAULT;
		return pxa688fb_partdisp_set(grap);
		break;

	case FB_IOCTL_GAMMA_SET:
#ifdef CONFIG_PXA688_MISC
		if (copy_from_user(&fbi->gamma, argp, sizeof(fbi->gamma)))
			return -EFAULT;
		return gamma_set(fbi->id, fbi->gamma.flag, fbi->gamma.table);
#else
		return -EINVAL;
#endif
		break;

	default:
		if (mi->ioctl)
			return mi->ioctl(info, cmd, arg);
		else
			pr_warning("%s: unknown IOCTL 0x%x\n", __func__, cmd);
		break;

	}
	return 0;
}

static int pxa168fb_open(struct fb_info *info, int user)
{
	struct pxa168fb_mach_info *mi;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct _sVideoBufferAddr *new_addr = &fbi->surface.videoBufferAddr;

	if (fbi->debug & (1<<4))
		return 0;

	pr_info("%s GFX layer, fbi %d opened %d times ----\n",
		 __func__, fbi->id, atomic_read(&fbi->op_count));

	/* Save screen info */
	fbi->var_bak = *var;

	mi = fbi->dev->platform_data;

	memset(new_addr, 0, sizeof(struct _sVideoBufferAddr));
	fbi->surface.videoMode = -1;
	fbi->surface.viewPortInfo.srcWidth = var->xres;
	fbi->surface.viewPortInfo.srcHeight = var->yres;

	set_pix_fmt(var, fbi->pix_fmt);

	if (mutex_is_locked(&fbi->access_ok))
		mutex_unlock(&fbi->access_ok);

	/* increase open count */
	atomic_inc(&fbi->op_count);

	return 0;
}

static int pxa168fb_release(struct fb_info *info, int user)
{
	struct fb_var_screeninfo *var = &info->var;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)info->par;

	if (fbi->debug & (1<<4))
		return 0;

	pr_info("%s GFX layer, fbi %d opened %d times ----\n",
		__func__, fbi->id, atomic_read(&fbi->op_count));

	if (atomic_dec_and_test(&fbi->op_count))
		pxa688_vdma_release(fbi->id, fbi->vid);

	/* Turn off compatibility mode */
	var->nonstd &= ~0xff000000;
	fbi->compat_mode = 0;

	memset(&fbi->surface, 0, sizeof(struct _sOvlySurface));
	fbi->surface.videoMode = -1;

	/* clear buffer list */
	clear_buffer(fbi);

	/* Recovery screen info */
	*var = fbi->var_bak;

	fbi->pix_fmt = determine_best_pix_fmt(var, fbi);

	if (NEED_VSYNC(fbi))
		wait_for_vsync(fbi);

	return 0;
}

static struct fb_ops pxa168fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= pxa168fb_check_var,
	.fb_open	= pxa168fb_open,
	.fb_release	= pxa168fb_release,
	.fb_set_par	= pxa168fb_set_par,
	.fb_setcolreg	= pxa168fb_setcolreg,
	.fb_blank	= pxa168fb_blank,
	.fb_pan_display	= pxa168fb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_ioctl       = pxa168_graphic_ioctl,
};

static int pxa168fb_init_mode(struct fb_info *info,
			      struct pxa168fb_mach_info *mi)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int ret = 0;
	u32 refresh;
	const struct fb_videomode *m;

	dev_dbg(info->dev, "Enter %s\n", __func__);

	/* Set default value */
	refresh = DEFAULT_REFRESH;

	/* If has bootargs, apply it first */
	if (fbi->dft_vmode.xres && fbi->dft_vmode.yres &&
	    fbi->dft_vmode.refresh) {
		/* set data according bootargs */
		var->xres = fbi->dft_vmode.xres;
		var->yres = fbi->dft_vmode.yres;
		refresh = fbi->dft_vmode.refresh;
	}

	/* try to find best video mode. */
	m = fb_find_best_mode(&info->var, &info->modelist);
	if (m)
		fb_videomode_to_var(&info->var, m);

	/* Init settings. */
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres * 2;

#if 0
	if (!var->pixclock) {
		u32 total_w, total_h;
		u64 div_result;

		/* correct pixclock. */
		total_w = var->xres + var->left_margin + var->right_margin +
			var->hsync_len;
		total_h = var->yres + var->upper_margin + var->lower_margin +
			var->vsync_len;

		div_result = 1000000000000ll;
		do_div(div_result, total_w * total_h * refresh);
		var->pixclock = (u32)div_result;
	}
#endif
	return ret;
}

static void pxa168fb_set_default(struct pxa168fb_info *fbi,
		struct pxa168fb_mach_info *mi)
{
	struct lcd_regs *regs = get_regs(fbi->id);
	u32 dma_ctrl1 = 0x2012ff81, flag, tmp;

	/*
	 * LCD Global control(LCD_TOP_CTRL) should be configed before
	 * any other LCD registers read/write, or there maybe issues.
	 */
	tmp = readl(fbi->reg_base + LCD_TOP_CTRL);
	tmp |= 0xfff0;		/* FIXME */
	writel(tmp, fbi->reg_base + LCD_TOP_CTRL);

	/* Configure default register values */
	writel(mi->io_pad_ctrl, fbi->reg_base + SPU_IOPAD_CONTROL);
	/* enable 16 cycle burst length to get better formance */

	writel(0x00000000, &regs->blank_color);
	writel(0x00000000, &regs->g_1);
	writel(0x00000000, &regs->g_start);

	/* Configure default bits: vsync triggers DMA,
	 * power save enable, configure alpha registers to
	 * display 100% graphics, and set pixel command.
	 */
	if (fbi->id == 1) {
		if (mi->phy_type & (DSI2DPI | DSI))
			dma_ctrl1 = 0xa03eff00;
		else
			dma_ctrl1 = 0x203eff00;	/* FIXME */
	}

	/*
	 * vsync in LCD internal controller is always positive,
	 * we default configure dma trigger @vsync falling edge,
	 * so that DMA idle time between DMA frame done and
	 *  next DMA transfer begin can be as large as possible
	 */
	dma_ctrl1 |= CFG_VSYNC_INV_MASK;
	dma_ctrl_write(fbi->id, 1, dma_ctrl1);

	/*
	 * 1.enable multiple burst request in DMA AXI
	 * bus arbiter for faster read if not tv path;
	 * 2.enable horizontal smooth filter;
	 */
	tmp = CFG_GRA_HSMOOTH_MASK | CFG_DMA_HSMOOTH_MASK;
	flag = CFG_ARBFAST_ENA(1) | tmp;
	if (fbi->id != 1)
		dma_ctrl_set(fbi->id, 0, flag, flag);
	else
		dma_ctrl_set(fbi->id, 0, flag, tmp);
}

static int __init get_fb_size(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	max_fb_size = n;
	fb_size_from_cmd = 1;
	return 1;
}
__setup("fb_size=", get_fb_size);

static int __init get_fb_share(char *str)
{
	fb_share = 1;
	return 1;
}
__setup("fb_share", get_fb_share);

#ifdef CONFIG_PM

static int _pxa168fb_suspend(struct pxa168fb_info *fbi)
{
	struct fb_info *info = fbi->fb_info;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	u32 mask = CFG_GRA_ENA_MASK;

	/* notify others */
	fb_set_suspend(info, 1);

	/* stop dma transaction */
#ifndef CONFIG_PXA168_V4L2_OVERLAY
	mask |= CFG_DMA_ENA_MASK;
#endif
	dma_ctrl_set(fbi->id, 0, mask, 0);

	/* Before disable lcd clk, disable all lcd interrupts */
	fbi->irq_mask = readl(fbi->reg_base + SPU_IRQ_ENA);
	irq_mask_set(fbi->id, 0xffffffff, 0);

	/* disable external panel power */
	if (pxa168fb_power(fbi, mi, 0))
		pr_err("%s %d pxa168fb_power control failed!\n",
			 __func__, __LINE__);

	fbi->active = 0;

	/* disable pixel clock */
	lcd_clk_set(fbi->id, clk_sclk, SCLK_DISABLE, SCLK_DISABLE);

	/* disable clock */
	clk_disable(fbi->clk);

	pr_debug("pxa168fb.%d suspended\n", fbi->id);
	return 0;
}

static int _pxa168fb_resume(struct pxa168fb_info *fbi)
{
	struct fb_info *info = fbi->fb_info;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	/* enable clock */
	if (mi->sclk_src)
		clk_set_rate(fbi->clk, mi->sclk_src);
	clk_enable(fbi->clk);

	/* enable pixel clock */
	lcd_clk_set(fbi->id, clk_sclk, SCLK_DISABLE, 0);

	/* enable external panel power */
	if (pxa168fb_power(fbi, mi, 1))
		pr_err("%s %d pxa168fb_power control failed!\n",
			 __func__, __LINE__);

	/* register setting should retain so no need to set again.
	 * pxa168fb_set_par(info);
	 * pxa168fb_set_default(fbi, mi);
	 */

	/* initialize external phy if needed */
	if (mi->phy_init && mi->phy_init(fbi)) {
		pr_err("%s fbi %d phy error\n", __func__, fbi->id);
		return -EIO;
	}

	/*After enable lcd clk, restore lcd interrupts*/
	irq_mask_set(fbi->id, 0xffffffff, fbi->irq_mask);

	/* restore gamma correction table */
	gamma_set(fbi->id, fbi->gamma.flag, fbi->gamma.table);

	/* restore dma after resume */
	fbi->active = 1;
#ifndef CONFIG_ANDROID
	set_dma_active(fbi);
#ifndef CONFIG_PXA168_V4L2_OVERLAY
	set_dma_active(ovly_info.fbi[fbi->id]);
#endif
#endif

	/* notify others */
	fb_set_suspend(info, 0);

	pr_debug("pxa168fb.%d resumed.\n", fbi->id);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void pxa168fb_early_suspend(struct early_suspend *h)
{
	struct pxa168fb_info *fbi = container_of(h, struct pxa168fb_info,
						 early_suspend);

	_pxa168fb_suspend(fbi);
	pm_qos_update_request(&fbi->qos_idle_fb, PM_QOS_DEFAULT_VALUE);
	wake_unlock(&fbi->idle_lock);

	return;
}
static void pxa168fb_late_resume(struct early_suspend *h)
{
	struct pxa168fb_info *fbi = container_of(h, struct pxa168fb_info,
						 early_suspend);

	pm_qos_update_request(&fbi->qos_idle_fb, PM_QOS_CONSTRAINT);
	wake_lock(&fbi->idle_lock);
	_pxa168fb_resume(fbi);

	return;
}

#else

static int pxa168fb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);

	_pxa168fb_suspend(fbi);
	pdev->dev.power.power_state = mesg;
	pm_qos_update_request(&fbi->qos_idle_fb, PM_QOS_DEFAULT_VALUE);
	wake_unlock(&fbi->idle_lock);

	return 0;
}

static int pxa168fb_resume(struct platform_device *pdev)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);

	pm_qos_update_request(&fbi->qos_idle_fb, PM_QOS_CONSTRAINT);
	wake_lock(&fbi->idle_lock);
	_pxa168fb_resume(fbi);

	return 0;
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

#endif /* CONFIG_PM */

/************************************************************************/

static ssize_t lcd_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	struct lcd_regs *regs = get_regs(fbi->id);
	int id = fbi->id;

	printk("fbi %d base 0x%p, frm %luus\n", fbi->id,
			fbi->reg_base, fbi->frm_usec);
	pr_info("var\n");
	pr_info("\t xres              %4d yres              %4d\n",
			var->xres,		var->yres);
	pr_info("\t xres_virtual      %4d yres_virtual      %4d\n",
			var->xres_virtual,	var->yres_virtual);
	pr_info("\t xoffset           %4d yoffset           %4d\n",
			var->xoffset,		var->yoffset);
	pr_info("\t left_margin(hbp)  %4d right_margin(hfp) %4d\n",
			var->left_margin,	var->right_margin);
	pr_info("\t upper_margin(vbp) %4d lower_margin(vfp) %4d\n",
			var->upper_margin,	var->lower_margin);
	pr_info("\t hsync_len         %4d vsync_len         %4d\n",
			var->hsync_len,		var->vsync_len);
	pr_info("\t bits_per_pixel    %d\n", var->bits_per_pixel);
	pr_info("\t pixclock          %d\n", var->pixclock);
	pr_info("\t sync              0x%x\n", var->sync);
	pr_info("\t vmode             0x%x\n", var->vmode);
	pr_info("\t rotate            0x%x\n", var->rotate);
	pr_info("\n");

#ifdef CONFIG_PXA688_MISC
again:
	regs = get_regs(id);
#endif
	pr_info("video layer\n");
	pr_info("\tv_y0        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_y0) & 0xfff, readl(&regs->v_y0));
	pr_info("\tv_u0        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_u0) & 0xfff, readl(&regs->v_u0));
	pr_info("\tv_v0        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_v0) & 0xfff, readl(&regs->v_v0));
	pr_info("\tv_c0        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_c0) & 0xfff, readl(&regs->v_c0));
	pr_info("\tv_y1        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_y1) & 0xfff, readl(&regs->v_y1));
	pr_info("\tv_u1        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_u1) & 0xfff, readl(&regs->v_u1));
	pr_info("\tv_v1        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_v1) & 0xfff, readl(&regs->v_v1));
	pr_info("\tv_c1        ( @%3x ) 0x%x\n",
		 (int)(&regs->v_c1) & 0xfff, readl(&regs->v_c1));
	pr_info("\tv_pitch_yc  ( @%3x ) 0x%x\n",
		 (int)(&regs->v_pitch_yc) & 0xfff, readl(&regs->v_pitch_yc));
	pr_info("\tv_pitch_uv  ( @%3x ) 0x%x\n",
		 (int)(&regs->v_pitch_uv) & 0xfff, readl(&regs->v_pitch_uv));
	pr_info("\tv_start     ( @%3x ) 0x%x\n",
		 (int)(&regs->v_start) & 0xfff, readl(&regs->v_start));
	pr_info("\tv_size      ( @%3x ) 0x%x\n",
		 (int)(&regs->v_size) & 0xfff, readl(&regs->v_size));
	pr_info("\tv_size_z    ( @%3x ) 0x%x\n",
		 (int)(&regs->v_size_z) & 0xfff, readl(&regs->v_size_z));
	pr_info("\n");

	pr_info("graphic layer\n");
	pr_info("\tg_0       ( @%3x ) 0x%x\n", (int)(&regs->g_0) & 0xfff,
						 readl(&regs->g_0));
	pr_info("\tg_1       ( @%3x ) 0x%x\n", (int)(&regs->g_1) & 0xfff,
						 readl(&regs->g_1));
	pr_info("\tg_pitch   ( @%3x ) 0x%x\n", (int)(&regs->g_pitch) & 0xfff,
						 readl(&regs->g_pitch));
	pr_info("\tg_start   ( @%3x ) 0x%x\n", (int)(&regs->g_start) & 0xfff,
						 readl(&regs->g_start));
	pr_info("\tg_size    ( @%3x ) 0x%x\n", (int)(&regs->g_size) & 0xfff,
						 readl(&regs->g_size));
	pr_info("\tg_size_z  ( @%3x ) 0x%x\n", (int)(&regs->g_size_z) & 0xfff,
						 readl(&regs->g_size_z));
	pr_info("\n");

	pr_info("hardware cursor\n");
	pr_info("\thc_start  ( @%3x ) 0x%x\n", (int)(&regs->hc_start) & 0xfff,
						  readl(&regs->hc_start));
	pr_info("\thc_size   ( @%3x ) 0x%x\n", (int)(&regs->hc_size) & 0xfff,
						 readl(&regs->hc_size));
	pr_info("\n");

	pr_info("screen info\n");
	pr_info("\tscreen_size     ( @%3x ) 0x%x\n",
		(int)(&regs->screen_size) & 0xfff,
		 readl(&regs->screen_size));
	pr_info("\tscreen_active   ( @%3x ) 0x%x\n",
		(int)(&regs->screen_active) & 0xfff,
		 readl(&regs->screen_active));
	pr_info("\tscreen_h_porch  ( @%3x ) 0x%x\n",
		(int)(&regs->screen_h_porch) & 0xfff,
		 readl(&regs->screen_h_porch));
	pr_info("\tscreen_v_porch  ( @%3x ) 0x%x\n",
		(int)(&regs->screen_v_porch) & 0xfff,
		 readl(&regs->screen_v_porch));
	pr_info("\n");

	pr_info("color\n");
	pr_info("\tblank_color     ( @%3x ) 0x%x\n",
		 (int)(&regs->blank_color) & 0xfff,
		 readl(&regs->blank_color));
	pr_info("\thc_Alpha_color1 ( @%3x ) 0x%x\n",
		 (int)(&regs->hc_Alpha_color1) & 0xfff,
		 readl(&regs->hc_Alpha_color1));
	pr_info("\thc_Alpha_color2 ( @%3x ) 0x%x\n",
		 (int)(&regs->hc_Alpha_color2) & 0xfff,
		 readl(&regs->hc_Alpha_color2));
	pr_info("\tv_colorkey_y    ( @%3x ) 0x%x\n",
		 (int)(&regs->v_colorkey_y) & 0xfff,
		 readl(&regs->v_colorkey_y));
	pr_info("\tv_colorkey_u    ( @%3x ) 0x%x\n",
		 (int)(&regs->v_colorkey_u) & 0xfff,
		 readl(&regs->v_colorkey_u));
	pr_info("\tv_colorkey_v    ( @%3x ) 0x%x\n",
		 (int)(&regs->v_colorkey_v) & 0xfff,
		 readl(&regs->v_colorkey_v));
	pr_info("\n");

	pr_info("control\n");
	pr_info("\tvsync_ctrl    ( @%3x ) 0x%x\n",
		 (int)(&regs->vsync_ctrl) & 0xfff,
		 readl(&regs->vsync_ctrl));
	pr_info("\tdma_ctrl0     ( @%3x ) 0x%x\n",
		 (int)(dma_ctrl(0, id)) & 0xfff,
		 readl(fbi->reg_base + dma_ctrl0(id)));
	pr_info("\tdma_ctrl1     ( @%3x ) 0x%x\n",
		 (int)(dma_ctrl(1, id)) & 0xfff,
		 readl(fbi->reg_base + dma_ctrl1(id)));
	pr_info("\tintf_ctrl     ( @%3x ) 0x%x\n",
		 (int)(intf_ctrl(id)) & 0xfff,
		 readl(fbi->reg_base + intf_ctrl(id)));
	pr_info("\tirq_enable    ( @%3x ) 0x%8x\n",
		 (int)(SPU_IRQ_ENA) & 0xfff,
		 readl(fbi->reg_base + SPU_IRQ_ENA));
	pr_info("\tirq_status    ( @%3x ) 0x%8x\n",
		 (int)(SPU_IRQ_ISR) & 0xfff,
		 readl(fbi->reg_base + SPU_IRQ_ISR));
	pr_info("\tclk_sclk      ( @%3x ) 0x%x\n",
		 (int)(clk_reg(id, clk_sclk)) & 0xfff,
		 readl(clk_reg(id, clk_sclk)));
	if (clk_reg(id, clk_tclk))
		pr_info("\tclk_tclk      ( @%3x ) 0x%x\n",
			(int)(clk_reg(id, clk_tclk)) & 0xfff,
			readl(clk_reg(id, clk_tclk)));
	pr_info("\tclk_lvds      ( @%3x ) 0x%x\n",
		 (int)(clk_reg(id, clk_lvds_rd)) & 0xfff,
		 readl(clk_reg(id, clk_lvds_rd)));
	/* TV path registers */
	if (fbi->id == 1) {
		pr_info("\ntv path interlace related:\n");
		pr_info("\tv_h_total       ( @%3x ) 0x%8x\n",
			(int)(LCD_TV_V_H_TOTAL_FLD) & 0xfff,
			readl(fbi->reg_base + LCD_TV_V_H_TOTAL_FLD));
		pr_info("\tv_porch         ( @%3x ) 0x%8x\n",
			(int)(LCD_TV_V_PORCH_FLD) & 0xfff,
			readl(fbi->reg_base + LCD_TV_V_PORCH_FLD));
		pr_info("\tvsync_ctrl      ( @%3x ) 0x%8x\n",
			(int)(LCD_TV_SEPXLCNT_FLD) & 0xfff,
			readl(fbi->reg_base + LCD_TV_SEPXLCNT_FLD));
	}

	pr_info("\n");

#ifdef CONFIG_PXA688_MISC
	if (id == fb_vsmooth) {
		id = fb_filter;
		pr_info("========== fb_vsmooth = %d ==========\n", fb_vsmooth);
		goto again;
	}
#endif

	return sprintf(buf, "fb %d: active %d, debug 0x%x\n"
		"DEBUG_VSYNC_PATH %d DEBUG_ERR_IRQ %d DEBUG_TV_ACTIVE %d\n"
		"gfx_underflow %d vid_underflow %d axi_err %d dma_on %d\n",
		fbi->id, fbi->active, fbi->debug, DEBUG_VSYNC_PATH(fbi->id),
		DEBUG_ERR_IRQ(fbi->id), DEBUG_TV_ACTIVE(fbi->id),
		gfx_udflow_count, vid_udflow_count, axi_err_count, fbi->dma_on);
}

static ssize_t lcd_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);

	/* fbi->debug usage:
	 *	bit 0-1: indicates to count vsync number for which path, "0" "1"
	 *	0: count vsync number for pn path
	 *	1: count vsync number for tv path
	 *	2: count vsync number for p2 path, not valid yet
	 *	bit 2: show bus error interrupts on screen, "4"
	 *	bit 3: enable TV path graphics layer dma always, "8"
	 *	bit 4: enable do not restore var in release "16"
	 */
	sscanf(buf, "%d", &fbi->debug);

	return size;
}

static DEVICE_ATTR(lcd, S_IRUGO | S_IWUSR, lcd_show, lcd_store);

static ssize_t vsync_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);

	return sprintf(buf, "fbi %d wait vsync: %d\n",
			 fbi->id, fbi->wait_vsync);
}

static ssize_t vsync_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &fbi->wait_vsync) != 1)
		pr_err("%s %d erro input of wait vsync flag\n",\
			__func__, __LINE__);
	return size;
}
static DEVICE_ATTR(vsync, S_IRUGO | S_IWUSR, vsync_show, vsync_store);

static ssize_t itc_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	int i, j, s = 0, val;

	s += sprintf(buf + s, "irqtm_check - %d\n 0: none\n 1: irq intervals\n"
		" 2: display done .vs. vsync\n 3: display done .vs. gfx done\n"
		" 4: display done .vs. vid done\n 5: lcd main handler\n",
		irqtm_check);

	s += sprintf(buf + s, "\nt1 - display done, t2 - ");
	if (ITC_INTERVAL)
		s += sprintf(buf + s, "irq interval\n");
	else if (ITC_VSYNC)
		s += sprintf(buf + s, "vsync\n");
	else if (ITC_GFX_DONE)
		s += sprintf(buf + s, "gfx frame done\n");
	else if (ITC_VID_DONE)
		s += sprintf(buf + s, "vid frame done\n");
	else if (ITC_HANDLER)
		s += sprintf(buf + s, "main handler\n");
	else
		s += sprintf(buf + s, "none\n");

	s += sprintf(buf + s, "     t1     t2 :   ");
	if (ITC_INTERVAL)
		s += sprintf(buf + s, "t2-frm_usec\n");
	else
		s += sprintf(buf + s, "t2-t1\n");
	for (i = 0; i < ITC_MAX_NUM; i++) {
		if (ITC_INTERVAL) {
			val = 0;
			if ((int)t2[i].tv_usec > 10000)
				val = (int)t2[i].tv_usec - fbi->frm_usec;
		} else
			val = t2[i].tv_usec - t1[i].tv_usec;

		for (j = 0; j < ITC_MAX_NUM; j++) {
			val = (val < -10000 || val > 10000) ?
				(t2[j].tv_usec - t1[i].tv_usec) : val;
		}

		s += sprintf(buf + s, " %6ld %6ld :  %6d", t1[i].tv_usec,
			t2[i].tv_usec, val);
		s += sprintf(buf + s, "\n");
	}
	return s;
}

static ssize_t itc_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	sscanf(buf, "%d", &irqtm_check);

	if (irqtm_check && !ITC_INTERVAL)
		vsync_check_count();

	return size;
}
static DEVICE_ATTR(itc, S_IRUGO | S_IWUSR, itc_show, itc_store);

static struct proc_dir_entry *pxa168fb_proc;
static unsigned int proc_reg;
static int pxa168fb_proc_write(struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	struct fb_info *info = gfx_info.fbi[0]->fb_info;
	struct pxa168fb_mach_info *mi = gfx_info.fbi[0]->dev->platform_data;
	struct dsi_info *di = (struct dsi_info *)mi->phy_info;
	char kbuf[11], vol[11];
	int index, reg_val, i;
	unsigned addr = (unsigned)gfx_info.fbi[0]->reg_base;

	if (count >= 12)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	if ('-' == kbuf[0]) {
		memcpy(vol, kbuf+1, count-1);
		proc_reg = (int) simple_strtoul(vol, NULL, 16);
		pr_info("reg @ 0x%x: 0x%x\n", proc_reg,
			__raw_readl(addr + proc_reg));
		return count;
	} else if ('0' == kbuf[0] && 'x' == kbuf[1]) {
		/* set the register value */
		reg_val = (int)simple_strtoul(kbuf, NULL, 16);
		__raw_writel(reg_val, addr + proc_reg);
		pr_info("set reg @ 0x%x: 0x%x\n", proc_reg,
			__raw_readl(addr + proc_reg));
		return count;
	} else if ('t' == kbuf[0] && 'm' == kbuf[1]) {
		memcpy(vol, kbuf+2, count-2);
		/* set the register value */
		irqtm_adjust = (int) simple_strtoul(vol, NULL, 10);
		printk("irqtm_adjust: %d\n", irqtm_adjust);
		return count;
	} else if ('l' == kbuf[0]) {
		for (i = 0; i < 0x300; i += 4) {
			if (!(i % 16) && i)
				pr_info("\n0x%3x: ", i);
			pr_info(" %8x", __raw_readl(addr + i));
		}
		pr_info("\n");
		return count;
	} else if ('d' == kbuf[0]) {
		if (!di) {
			pr_info("fb0 no dsi info\n");
			return count;
		}
		addr = (unsigned)di->regs;
		for (i = 0x0; i < 0x200; i += 4) {
			if (!(i % 16))
				pr_info("\n0x%3x: ", i);
			pr_info(" %8x", __raw_readl(addr + i));
		}
		pr_info("\n");
		return count;
	}

	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
	case 4:
		memset(info->screen_base, 2, info->screen_size/2);
		memset(info->screen_base + info->screen_size/4, 8,
				info->screen_size/2);
		break;
	case 9:
		vsync_check_count();
		break;

	default:
		return -EINVAL;
	}

	return count;
}

static int pxa168fb_proc_read(char *buffer, char **buffer_location,
	off_t offset, int buffer_length, int *zero, void *ptr)
{
	if (offset > 0)
		return 0;

	return sprintf(buffer, "fb_share %d reg @ 0x%x: 0x%x\n", fb_share,
		 proc_reg, __raw_readl(gfx_info.fbi[0]->reg_base + proc_reg));
}

static int __devinit pxa168fb_probe(struct platform_device *pdev)
{
	struct pxa168fb_mach_info *mi;
	struct fb_info *info = 0;
	struct pxa168fb_info *fbi = 0;
	struct resource *res;
	struct clk *clk;
	int irq, irq_mask, irq_enable_value, ret = 0;
	struct dsi_info *di = NULL;
	static int proc_inited;
	struct pxa168fb_vdma_info *lcd_vdma = 0;

	mi = pdev->dev.platform_data;
	if (mi == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}
	/* if isr_clear_mask not initizlized, set to 0xffffffff as default */
	if (!mi->isr_clear_mask) {
		pr_err("%s: %s isr_clear_mask not been initialized!\n",
			__func__, mi->id);
		mi->isr_clear_mask = 0xffffffff;
	}

	clk = clk_get(NULL, "LCDCLK");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get LCDCLK");
		return PTR_ERR(clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		ret = -ENOENT;
		goto failed_put_clk;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		ret = -ENOENT;
		goto failed_put_clk;
	}

	info = framebuffer_alloc(sizeof(struct pxa168fb_info), &pdev->dev);
	if ((info == NULL) || (!info->par)) {
		ret = -ENOMEM;
		goto failed_put_clk;
	}

	/* Initialize private data */
	fbi = info->par;
	fbi->id = pdev->id;
	fbi->vid = 0;

	if (!fbi->id) {
		memset(&gfx_info, 0, sizeof(gfx_info));
		fbi->dma_on = 1;
	}

	gfx_info.fbi[fbi->id] = fbi;
	if (mi->phy_type & (DSI | DSI2DPI))
		di = (struct dsi_info *)mi->phy_info;
	if (di) {
		pr_info("fb%d dsi %d di->lanes %d di->bpp %d\n",
			fbi->id, di->id, di->lanes, di->bpp);
		if (di->id & 1)
			di->regs = (unsigned)ioremap_nocache\
			(DSI1_REGS_PHYSICAL_BASE, sizeof(struct dsi_regs));
		else
			di->regs = (unsigned)ioremap_nocache\
			(DSI2_REGS_PHYSICAL_BASE, sizeof(struct dsi_regs));
	}

	fbi->fb_info = info;
	platform_set_drvdata(pdev, fbi);
	fbi->clk = clk;
	fbi->dev = &pdev->dev;
	fbi->fb_info->dev = &pdev->dev;
	fbi->is_blanked = 0;
	fbi->active = mi->active;

	/* Initialize boot setting */
	fbi->dft_vmode.xres = mi->modes->xres;
	fbi->dft_vmode.yres = mi->modes->yres;
	fbi->dft_vmode.refresh = mi->modes->refresh;

	init_waitqueue_head(&fbi->w_intr_wq);
	mutex_init(&fbi->access_ok);

	pxa168fb_list_init(fbi);

	/* Initialise static fb parameters */
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
			FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	strcpy(info->fix.id, mi->id);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 1;
	info->fix.ypanstep = 1;
	info->fix.ywrapstep = 0;
	info->fix.mmio_start = res->start;
	info->fix.mmio_len = res->end - res->start + 1;
	info->fix.accel = FB_ACCEL_NONE;
	info->fbops = &pxa168fb_ops;
	info->pseudo_palette = fbi->pseudo_palette;

	/* Map LCD controller registers */
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		ret = -ENOMEM;
		goto failed_free_info;
	}

	/* Allocate framebuffer memory */
	if (!fb_size_from_cmd) {
		if (mi->max_fb_size)
			max_fb_size = mi->max_fb_size;
		else
			max_fb_size = DEFAULT_FB_SIZE;
	}
	if (fb_share) {
		/* fb_share mode, allocate more memory as frame buffer */
		max_fb_size = max(max_fb_size, 2 * 4 * (mi->modes->xres *
				(mi->modes->xres + mi->modes->yres)));
	}

	max_fb_size = PAGE_ALIGN(max_fb_size);
	if (mi->mmap)
		fbi->fb_size = max_fb_size;

	if (fb_share && (fbi->id == 1) && gfx_info.fbi[0]->fb_start) {
		fbi->fb_size = gfx_info.fbi[0]->fb_size;
		fbi->fb_start = gfx_info.fbi[0]->fb_start;
		fbi->fb_start_dma = gfx_info.fbi[0]->fb_start_dma;
		pr_info("--share--FB DMA buffer phy addr : %x\n",
			(unsigned int)fbi->fb_start_dma);
	} else if (mi->mmap) {
		fbi->fb_start = dma_alloc_writecombine(fbi->dev, max_fb_size,
				&fbi->fb_start_dma, GFP_KERNEL);

		if (!fbi->fb_start || !fbi->fb_start_dma) {
			fbi->fb_start = (void *)__get_free_pages(GFP_DMA |
				GFP_KERNEL, get_order(fbi->fb_size));
			fbi->fb_start_dma =
				(dma_addr_t)__virt_to_phys(fbi->fb_start);
		}

		if (fbi->fb_start == NULL) {
			pr_err("%s: no enough memory!\n", __func__);
			ret = -ENOMEM;
			goto failed_free_info;
		}
		pr_info("---------FB DMA buffer phy addr : %x\n",
			(unsigned int)fbi->fb_start_dma);

		memset(fbi->fb_start, 0, fbi->fb_size);
	}

	/* fill backup information for mode switch */
	fbi->fb_start_dma_bak = fbi->fb_start_dma;
	fbi->fb_start_bak = fbi->fb_start;

	info->fix.smem_start = fbi->fb_start_dma;
	info->fix.smem_len = fbi->fb_size;
	info->screen_base = fbi->fb_start;
	info->screen_size = fbi->fb_size;

	/* init qos with constraint */
	pm_qos_add_request(&fbi->qos_idle_fb, PM_QOS_CPU_DMA_LATENCY,
			PM_QOS_CONSTRAINT);
	/* init wake lock */
	wake_lock_init(&fbi->idle_lock, WAKE_LOCK_IDLE, dev_name(fbi->dev));
	/* avoid system enter low power modes */
	wake_lock(&fbi->idle_lock);

	/* Set video mode according to platform data */
	set_mode(fbi, &info->var, mi->modes, mi->pix_fmt, 1);

	fb_videomode_to_modelist(mi->modes, mi->num_modes, &info->modelist);

	/* init video mode data */
	pxa168fb_init_mode(info, mi);

	/* enable controller clock */
	if (mi->sclk_src)
		clk_set_rate(fbi->clk, mi->sclk_src);
	clk_enable(fbi->clk);
	pr_info("fb%d: sclk_src %d clk_get_rate = %d\n", fbi->id,
		mi->sclk_src, (int)clk_get_rate(fbi->clk));

	/* init vdma clock/sram, etc. */
	lcd_vdma = request_vdma(fbi->id, fbi->vid);
	if (lcd_vdma) {
		lcd_vdma->dev = fbi->dev;
		lcd_vdma->reg_base = fbi->reg_base;
		pxa688_vdma_init(lcd_vdma);
	} else
		pr_warn("path %d layer %d: request vdma fail\n", fbi->id, fbi->vid);

	/* Fill in sane defaults */
	pxa168fb_set_default(fbi, mi);	/* FIXME */
	pxa168fb_set_par(info);

	/* Allocate color map */
	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	/* Register irq handler */
	if (!fbi->id) {
		/* Clear the irq status before kernel startup */
		irq_status_clear(fbi->id, 0xFFFFFFFF);

		ret = request_irq(irq, pxa168fb_handle_irq, IRQF_DISABLED,
					 mi->id, fbi);
		if (ret < 0) {
			dev_err(&pdev->dev, "unable to request IRQ\n");
			ret = -ENXIO;
			goto failed_free_cmap;
		}
	}

	/* disable GFX interrupt
	*  enable display done & err interrupt */
	irq_mask = path_imasks(fbi->id) | err_imask(fbi->id);
	irq_enable_value = display_done_imask(fbi->id) | err_imask(fbi->id);
	irq_mask_set(fbi->id, irq_mask, irq_enable_value);
	fbi->wait_vsync = 1;

	/* enable power supply */
	fbi->active = 0;
	if (pxa168fb_power(fbi, mi, 1)) {
			ret = -EINVAL;
			goto failed_free_irq;
		}
	fbi->active = 1;

	/* initialize external phy if needed */
	if (mi->phy_init && mi->phy_init(fbi)) {
		pr_err("%s fbi %d phy error\n", __func__, fbi->id);
		ret = -EIO;
		goto failed_free_irq;
	}

	/* Register framebuffer */
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa168-fb: %d\n", ret);
		ret = -ENXIO;
		goto failed_free_irq;
	}
	pr_info("pxa168fb: frame buffer device was loaded"
		" to /dev/fb%d <%s>.\n", info->node, info->fix.id);

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* let TV path suspend first and resume late than panel path */
	fbi->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - fbi->id;
	fbi->early_suspend.suspend = pxa168fb_early_suspend;
	fbi->early_suspend.resume = pxa168fb_late_resume;
	register_early_suspend(&fbi->early_suspend);
#endif

#ifdef CONFIG_PXA688_PHY
	ret = device_create_file(&pdev->dev, &dev_attr_phy);
	if (ret < 0) {
		pr_err("device attr create fail: %d\n", ret);
		goto failed_free_irq;
	}
#endif

#ifdef CONFIG_PXA688_VDMA
	ret = device_create_file(&pdev->dev, &dev_attr_vdma);
	if (ret < 0) {
		pr_err("device attr create fail: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_PXA688_MISC
	ret = device_create_file(&pdev->dev, &dev_attr_misc);
	if (ret < 0) {
		pr_err("device attr misc create fail: %d\n", ret);
		return ret;
	}
#endif

	ret = device_create_file(&pdev->dev, &dev_attr_lcd);
	if (ret < 0) {
		pr_err("device attr lcd create fail: %d\n", ret);
		goto failed_free_irq;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_vsync);
	if (ret < 0) {
		pr_err("device attr create fail: %d\n", ret);
		goto failed_free_irq;
	}

	if (!fbi->id) {
		ret = device_create_file(&pdev->dev, &dev_attr_itc);
		if (ret < 0) {
			pr_err("device attr create fail: %d\n", ret);
			return ret;
		}
	}

	if (!proc_inited) {
		proc_inited = 1;
		pxa168fb_proc = create_proc_entry("pxa168fb" , 0666 , NULL);
		pxa168fb_proc->read_proc = pxa168fb_proc_read;
		pxa168fb_proc->write_proc = pxa168fb_proc_write;
	}

#ifdef CONFIG_ANDROID
	if (fbi->fb_start && (!fbi->id || !fb_share)) {
		fb_prepare_logo(info, 0);
		fb_show_logo(info, 0);
	}
#endif
	return 0;

failed_free_irq:
	free_irq(irq, fbi);
failed_free_cmap:
	fb_dealloc_cmap(&info->cmap);
failed_free_clk:
	pm_qos_remove_request(&fbi->qos_idle_fb);
	wake_lock_destroy(&fbi->idle_lock);
	clk_disable(fbi->clk);
	dma_free_writecombine(fbi->dev, PAGE_ALIGN(info->fix.smem_len),
				info->screen_base, info->fix.smem_start);
failed_free_info:
	platform_set_drvdata(pdev, NULL);

	if (fbi && fbi->reg_base) {
		iounmap(fbi->reg_base);
		kfree(fbi);
	}
	kfree(info);
failed_put_clk:
	clk_put(clk);
	pr_err("pxa168-fb: frame buffer device init failed\n");

	return ret;
}

static int __devexit pxa168fb_remove(struct platform_device *pdev)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);
	struct fb_info *info;
	int irq;
	unsigned int data;

	if (!fbi)
		return 0;

	/* disable DMA transfer */

	data = dma_ctrl_read(fbi->id, 0);
	data &= ~0x00000100;
	dma_ctrl_write(fbi->id, 0, data);

	info = fbi->fb_info;

	unregister_framebuffer(info);

	irq_mask_set(fbi->id, 0xffffffff, 0);

	if (info->cmap.len)
		fb_dealloc_cmap(&info->cmap);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, fbi);

	pm_qos_remove_request(&fbi->qos_idle_fb);
	wake_lock_destroy(&fbi->idle_lock);
	dma_free_writecombine(fbi->dev, PAGE_ALIGN(info->fix.smem_len),
				info->screen_base, info->fix.smem_start);

	iounmap(fbi->reg_base);

	clk_disable(fbi->clk);
	clk_put(fbi->clk);

	framebuffer_release(info);

	return 0;
}

static struct platform_driver pxa168fb_driver = {
	.driver		= {
		.name	= "pxa168-fb",
		.owner	= THIS_MODULE,
	},
	.probe		= pxa168fb_probe,
	.remove		= __devexit_p(pxa168fb_remove),
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= pxa168fb_suspend,
	.resume		= pxa168fb_resume,
#endif
#endif
};

static int __devinit pxa168fb_init(void)
{
	return platform_driver_register(&pxa168fb_driver);
}
module_init(pxa168fb_init);

static void __exit pxa168fb_exit(void)
{
	platform_driver_unregister(&pxa168fb_driver);
}
module_exit(pxa168fb_exit);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@marvell.com>");
MODULE_DESCRIPTION("Framebuffer driver for PXA168");
MODULE_LICENSE("GPL");
