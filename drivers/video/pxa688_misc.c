/*
 * linux/drivers/video/pxa688fb_misc.c -- Marvell PXA668 LCD Controller
 *
 * Copyright (C) Marvell Semiconductor Company.  All rights reserved.
 *
 * 2011-05-25  Jing Xiang <jxiang@marvell.com>
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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/console.h>
#include <linux/timer.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <mach/pxa168fb.h>

#include "pxa168fb.h"

/* graphic layer partial display, color format should be RGB565 */
int pxa688fb_partdisp_set(struct pxa168fb_gra_partdisp grap)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[1];
	struct fb_info *info = fbi->fb_info;
	struct fb_var_screeninfo *var = &info->var;
	struct lcd_regs *regs;
	u32 xres, yres, yres_z, color3_0, color7_4, color11_8, color15_12;
	u32 base, mask, gfx_fmt, bytespp, shift, offset, threshold, region, tmp;

	gfx_fmt = (dma_ctrl_read(grap.id, 0) & (0xf << 16)) >> 16;
	if (gfx_fmt == PIX_FMT_RGB565 || gfx_fmt == PIX_FMT_RGB1555 >> 1 ||
		gfx_fmt == PIX_FMT_YUV422PACK >> 1)
		bytespp = 16 >> 3;
	else if (gfx_fmt == PIX_FMT_RGB888PACK >> 1)
		bytespp = 24 >> 3;
	else if (gfx_fmt == PIX_FMT_RGB888UNPACK >> 1 ||
		gfx_fmt == PIX_FMT_RGBA888 >> 1)
		bytespp = 32 >> 3;
	else
		return -EINVAL;

	regs = get_regs(grap.id);
	xres = readl(&regs->g_size) & 0xfff;
	yres = (readl(&regs->g_size) & 0xfff0000) >> 16;
	yres_z = (readl(&regs->g_size_z) & 0xfff0000) >> 16;

	if (!yres)
		return -EINVAL;

	/* partial display region should be not larger than source size*/
	if (grap.horpix_start > xres)
		grap.horpix_start = xres;
	if (grap.horpix_end > xres)
		grap.horpix_end = xres;
	if (grap.vertline_start > yres)
		grap.vertline_start = yres;
	if (grap.vertline_end > yres)
		grap.vertline_end = yres;

	if (grap.id == 1 &&
		var->vmode & FB_VMODE_INTERLACED) {
		/* tv interlace mode */
		grap.vertline_start = grap.vertline_start >> 1;
		grap.vertline_end = grap.vertline_end >> 1;
	}

	/* adjust vertical start/end lines according to zoom size */
	grap.vertline_start = grap.vertline_start * yres_z / yres;
	grap.vertline_end = grap.vertline_end * yres_z / yres;

	/* adjust hortizontal start/end pixel number according to:
	 * 1. start pixel number should be
	 *    (DMA burst length / bytes per pixel) aligned.
	 * 2. (end pixel number - start pixel number -
	 *    path threshold / bytes per pixel) should be
	 *    64 / bytes per pixel aligned.
	 */
	if (grap.horpix_end > grap.horpix_start) {
		shift = (grap.id == 1 ? 14 : 10);
		offset = (grap.id == 1 ? 16 : 1);

		/* THRESHOLD_x: the least bytes to operate for
		 * horizontal partial display
		 */
		threshold = (grap.id == 1 ? THRESHOLD_TV : THRESHOLD_PN);
		base = (u32)fbi->reg_base +
			(grap.id == 2 ? PN2_IOPAD_CONTROL : LCD_TOP_CTRL);
		mask = readl(base) & (3 << shift);
		mask = (((mask >> shift) + 1) << 6) / bytespp;

		/* adjust horizontal start pixel number */
		grap.horpix_start /= mask;
		grap.horpix_start *= mask;

		/* adjust horizontal end pixel number */
		region = grap.horpix_end - grap.horpix_start;
		if (region  > (threshold / bytespp)) {
			region -= (threshold / bytespp);

			/* BURST_LEN: AXI burst size, platform dependent */
			if (region >= (BURST_LEN / bytespp)) {
				tmp = region % (BURST_LEN / bytespp);
				region /= (BURST_LEN / bytespp);
				region *= (BURST_LEN / bytespp);
				grap.horpix_end = grap.horpix_start + region +
					threshold / bytespp;
				if (grap.id == 1 && tmp >= (THRESHOLD_PN * 2 -
					THRESHOLD_TV) / bytespp)
					/* add extra 64 /bytespp for TV path*/
					grap.horpix_end += (BURST_LEN / bytespp);
			} else
				grap.horpix_end = grap.horpix_start +
					threshold / bytespp + offset;
		} else
			grap.horpix_end = grap.horpix_start;
	}

	color3_0 = grap.color & 0x000f;
	color7_4 = (grap.color & 0x00f0) >> 4;
	color11_8 = (grap.color & 0x0f00) >> 8;
	color15_12 = (grap.color & 0xf000) >> 12;

	/* horizontal register setting */
	mask = grap.horpix_start | (color3_0 << 12)
		| (grap.horpix_end << 16) | (color7_4 << 28);
	writel(mask, (u32)fbi->reg_base + gra_partdisp_ctrl_hor(grap.id));
	/* vertical register setting */
	mask = grap.vertline_start	| (color11_8  << 12)
		| (grap.vertline_end << 16) | (color15_12 << 28);
	writel(mask, (u32)fbi->reg_base + gra_partdisp_ctrl_ver(grap.id));

	return 0;
}

/* for partial display, only vertical lines need be updated
 * when zoom size changed */
void pxa688fb_partdisp_update(int id)
{
	struct pxa168fb_info *fbi = gfx_info.fbi[id];
	u32 base, mask, vertline_start, vertline_end,
		screen_active, yres, yres_bak;
	struct lcd_regs *regs;

	regs = get_regs(fbi->id);
	screen_active = readl(&regs->screen_active);
	if (!fbi->scrn_act_bak)
		fbi->scrn_act_bak = screen_active;
	if (fbi->scrn_act_bak == screen_active)
		/* no need to update partial display */
		return;

	base = (u32)fbi->reg_base;
	mask = readl(base + gra_partdisp_ctrl_ver(id));

	/* get original partial display vertical setting */
	vertline_start = mask & 0xfff;
	vertline_end = (mask & 0xfff0000) >> 16;

	/* get original/new vertical lines */
	yres_bak = (fbi->scrn_act_bak & 0x0fff0000) >> 16;
	yres = (screen_active & 0x0fff0000) >> 16;

	/* adjust partial display start/end vertical lines by
	 * new / original ratio */
	vertline_start = vertline_start * yres / yres_bak;
	vertline_end = vertline_end * yres / yres_bak;

	mask &= ~0xfff0fff;
	mask |= vertline_start | (vertline_end << 16);
	writel(mask, base + gra_partdisp_ctrl_ver(id));
	fbi->scrn_act_bak = screen_active;
}

ssize_t misc_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	struct pxa168fb_gra_partdisp grap;
	u32 mask;

	grap.id = fbi->id;

	mask = readl((u32)fbi->reg_base + gra_partdisp_ctrl_hor(grap.id));
	/* get horizontal start/end pixel number */
	grap.horpix_start = mask & 0xfff;
	grap.horpix_end = (mask & 0xfff0000) >> 16;
	/* get color bit 0~7 */
	grap.color = (mask & 0xf000) >> 12;
	grap.color |= ((mask & 0xf0000000) >> 28) << 4;

	mask = readl((u32)fbi->reg_base + gra_partdisp_ctrl_ver(grap.id));
	/* get vertical start/end line */
	grap.vertline_start = mask & 0xfff;
	grap.vertline_end = (mask & 0xfff0000) >> 16;
	/* get color bit 8~15 */
	grap.color |= ((mask & 0xf000) >> 12) << 8;
	grap.color |= ((mask & 0xf0000000) >> 28) << 12;

	return sprintf(buf, "fbi %d:\nhorpix_start:%d vertline_start:%d "
		"horpix_end:%d vertline_end:%d color:%d\n",
		grap.id, grap.horpix_start, grap.vertline_start,
		grap.horpix_end, grap.vertline_end, grap.color);
}
ssize_t misc_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	struct pxa168fb_gra_partdisp grap;
	char vol[30];

	if (size > 30) {
		pr_err("%s size = %d > max 30 chars\n", __func__, size);
		return size;
	}
	if ('p' == buf[0]) {
		memcpy(vol, (void *)((u32)buf + 1), size - 1);
		if (sscanf(vol, "%u %u %u %u %hu", &grap.horpix_start,
			&grap.vertline_start, &grap.horpix_end,
			&grap.vertline_end, &grap.color) != 5) {
			pr_err("partial display cmd should be like: "
				"p horpix_start vertline_start "
				"horpix_end verline_end color\n");
			return size;
		}
		grap.id = fbi->id;
		pxa688fb_partdisp_set(grap);
		pr_info("lcd_part_disp\n");
	} else
		pr_err("%s unknown command %s\n", __func__, buf);

	return size;
}
DEVICE_ATTR(misc, S_IRUGO | S_IWUSR, misc_show, misc_store);
