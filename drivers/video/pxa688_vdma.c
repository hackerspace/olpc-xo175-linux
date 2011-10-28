/*
 * linux/drivers/video/pxa168fb.c -- Marvell PXA168 LCD Controller
 *
 *  Copyright (C) 2008 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2009-02-16  adapted from original version for PXA168
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
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_PXA688_VDMA

#include "pxa168fb.h"
#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/pxa168fb.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <mach/gpio.h>

#include <asm/mach-types.h>
#include <mach/regs-apmu.h>
#include <mach/mfp-mmp2.h>
#include <mach/regs-mpmu.h>
#include <asm/mach-types.h>

/* malloc sram buffer for squ - 64 lines by default */
#include <mach/sram.h>
static u32 pxa688_vdma_squ_malloc(unsigned int *psize)
{
	u32 psqu = 0, vsqu;

	vsqu = (u32)sram_alloc("mmp-videosram", *psize, (dma_addr_t *)&psqu);

	if (vsqu) {
		return psqu;
	} else {
		pr_err("%s: sram malloc failed!\n", __func__);
		return 0;
	}
}

/* convert pix fmt to vmode */
static FBVideoMode pixfmt_to_vmode(int pix_fmt)
{
	switch(pix_fmt) {
	case PIX_FMT_RGB565:
		return FB_VMODE_RGB565;
	case PIX_FMT_BGR565:
		return FB_VMODE_BGR565;
	case PIX_FMT_RGB1555:
		return FB_VMODE_RGB1555;
	case PIX_FMT_BGR1555:
		return FB_VMODE_BGR1555;
	case PIX_FMT_RGB888PACK:
		return FB_VMODE_RGB888PACK;
	case PIX_FMT_BGR888PACK:
		return FB_VMODE_BGR888PACK;
	case PIX_FMT_RGB888UNPACK:
		return FB_VMODE_RGB888UNPACK;
	case PIX_FMT_BGR888UNPACK:
		return FB_VMODE_BGR888UNPACK;
	case PIX_FMT_RGBA888:
		return FB_VMODE_RGBA888;
	case PIX_FMT_BGRA888:
		return FB_VMODE_BGRA888;

	case PIX_FMT_YUV422PACK:
		return FB_VMODE_YUV422PACKED;
	case PIX_FMT_YVU422PACK:
		return FB_VMODE_YUV422PACKED_SWAPUV;
	case PIX_FMT_YUV422PLANAR:
		return FB_VMODE_YUV422PLANAR;
	case PIX_FMT_YVU422PLANAR:
		return FB_VMODE_YUV422PLANAR_SWAPUV;
	case PIX_FMT_YUV420PLANAR:
		return FB_VMODE_YUV420PLANAR;
	case PIX_FMT_YVU420PLANAR:
		return FB_VMODE_YUV420PLANAR_SWAPUV;
	case PIX_FMT_YUYV422PACK:
		return FB_VMODE_YUV422PACKED_SWAPYUorV;
	case PIX_FMT_YUV422PACK_IRE_90_270:
		return FB_VMODE_YUV422PACKED_IRE_90_270;
	default:
		return -1;
	};
}

#define vdma_ctrl(id)		(id ? VDMA_CTRL_2 : VDMA_CTRL_1)

static u32 lcd_pitch_read(struct pxa168fb_info *fbi)
{
	struct lcd_regs *regs = get_regs(fbi->id);
	u32 reg = (u32)&regs->g_pitch;

	if (fbi->vid)
		reg = (u32)&regs->v_pitch_yc;

	return __raw_readl(reg) & 0xffff;
}

static u32 lcd_height_read(struct pxa168fb_info *fbi)
{
	struct lcd_regs *regs = get_regs(fbi->id);
	u32 reg = (u32)&regs->g_size;

	if (fbi->vid)
		reg = (u32)&regs->v_size;

	return (__raw_readl(reg) & 0xfff0000) >> 16;
}

static u32 lcd_width_read(struct pxa168fb_info *fbi)
{
	struct lcd_regs *regs = get_regs(fbi->id);
	u32 reg = (u32)&regs->g_size;

	if (fbi->vid)
		reg = (u32)&regs->v_size;

	return __raw_readl(reg) & 0xfff;
}

static u32 vdma_ctrl_read(struct pxa168fb_info *fbi)
{
	u32 reg = (u32)fbi->reg_base + vdma_ctrl(fbi->id);

	return __raw_readl(reg);
}

static int __get_vdma_rot_ctrl(int vmode, int angle, int yuv_format)
{
	int rotation, flag = 0, reg = 0;

	if (!angle || (angle == 1))
		return 0;

	switch (angle) {
	case 90:
		rotation = 1;
		break;
	case 270:
		rotation = 0;
		break;
	case 180:
		rotation = 2;
		break;
	default:
		rotation = 0;
		break;
		/* return rotation; */
	}

	switch (vmode) {
	case FB_VMODE_RGB565:
	case FB_VMODE_BGR565:
	case FB_VMODE_RGB1555:
	case FB_VMODE_BGR1555:
		reg = 4 << 2;
		reg |= rotation << 11;
		break;
	case FB_VMODE_YUV422PACKED:
	case FB_VMODE_YUV422PACKED_SWAPUV:
	case FB_VMODE_YUV422PACKED_SWAPYUorV:
		flag = 1;
	case FB_VMODE_YUV422PACKED_IRE_90_270:
		reg = 1 << 5;
		reg |= 1 << 7;
		reg |= 3 << 2;
		reg |= rotation << 11;
		break;
	default:
		reg = 6 << 2;
		reg |= rotation << 11;
		reg |= 0 << 7;
		break;
	}

	if (vmode == FB_VMODE_YUV422PACKED_IRE_90_270 || flag == 1) {
		if (rotation == 2)
			reg |= 2 << 21;
		else
			reg |= 1 << 21;

		if (vmode == FB_VMODE_YUV422PACKED)
			yuv_format = 1;
		if (vmode == FB_VMODE_YUV422PACKED_SWAPUV)
			yuv_format = 2;
		if (vmode == FB_VMODE_YUV422PACKED_SWAPYUorV)
			yuv_format = 4;

		switch (yuv_format) {
		case 1:/*YUV_FORMAT_UYVY*/
			reg |= 1 << 9;
			break;
		case 2:/*YUV_FORMAT_VYUY*/
			reg |= 0 << 9;
			break;
		case 3:/*YUV_FORAMT_YVYU*/
			reg |= 3 << 9;
			break;
		case 4:/*YUV_FORMAT_YUYV*/
			reg |= 2 << 9;
			break;
		}
	}

	return reg;
}

static int __get_vdma_src_sz(struct pxa168fb_info *fbi,
		int vmode, int width, int height)
{
	int res = lcd_pitch_read(fbi) * height;

	if (vmode == FB_VMODE_YUV422PACKED_IRE_90_270)
		return res >> 1;
	else
		return res;
}

static int __get_vdma_sa(struct pxa168fb_info *fbi, int vmode,
		int width, int height, int bpp, int rotation)
{
	struct lcd_regs *regs = get_regs(fbi->id);
	int addr = 0;

	if (vmode == FB_VMODE_YUV422PACKED_IRE_90_270)
		bpp = 2;

	if (fbi->vid)
		addr = readl(&regs->v_y0);
	else
		addr = readl(&regs->g_0);

	switch (rotation) {
	case 90:
		break;
	case 180:
		addr += width * height * bpp - 1;
		break;
	case 270:
		addr += height * (width - 1) * bpp;
		break;
	}
	return addr;

}

static int __get_vdma_sz(struct pxa168fb_info *fbi, int vmode,
		int rotation, int width, int height, int bpp)
{
	int src_pitch = lcd_pitch_read(fbi);

	if (vmode == FB_VMODE_YUV422PACKED_IRE_90_270)
		bpp = 2;
	switch (rotation) {
	case 0:
	case 1:
		return height << 16 | src_pitch;
	case 90:
		return width << 16 | height;
	case 180:
		return width | height << 16;
	case 270:
		return width << 16 | height;
	}
	return 0;

}

static int __get_vdma_pitch(struct pxa168fb_info *fbi, int vmode,
		int rotation, int width, int height, int bpp)
{
	int src_bpp = bpp, src_pitch = lcd_pitch_read(fbi);

	if (vmode == FB_VMODE_YUV422PACKED_IRE_90_270)
		src_bpp = 2;
	switch (rotation) {
	case 0:
	case 1:
		return src_pitch | (width * bpp << 16);
	case 90:
		return (width * bpp) << 16 | (height * src_bpp);
	case 180:
		return width * src_bpp | (width * bpp << 16);
	case 270:
		return (width * bpp) << 16 | (height * src_bpp);
	}
	return 0;

}

static int __get_vdma_ctrl(struct pxa168fb_info *fbi, int rotation, int line)
{
	if (!rotation || (rotation == 1))
		return (line << 8) | 0xa1;
	else
		return (line << 8) | 0xa5;
}

static void pxa688_vdma_clkset(int en)
{
	if (en)
		writel(readl(APMU_LCD2_CLK_RES_CTRL) | 0x11b,
				APMU_LCD2_CLK_RES_CTRL);
}

static int pxa688_vdma_get_linenum(struct pxa168fb_info *fbi, int angle)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int mulfactor, lines, lines_exp;
	int pitch = lcd_pitch_read(fbi);
	int height = lcd_height_read(fbi);

	if (!pitch)
		return 0;
	if (!angle || (angle == 1))
		/* no rotation */
		mulfactor = 2;
	else
		mulfactor = 16;
	lines = (mi->sram_size / pitch)&(~(mulfactor-1));

	if (lines < 2)
		return 0; /* at least 2 lines*/
	if (lines > 64)
		lines = 64;

	for (lines_exp = 0; lines_exp < lines; lines_exp += mulfactor) {
		if (height%(lines-lines_exp) == 0)
			break;
	}
	if (lines_exp >= lines)
		return 32;
	lines -= lines_exp;
	pr_debug("lines %d lines_exp %d angle %d mulfactor %d height %d"
		" pitch %d sram_size %d\n", lines, lines_exp, angle,
		mulfactor, height, pitch, mi->sram_size);
	return lines;
}

static void pxa688_vdma_set(struct pxa168fb_info *fbi, u32 psqu,
	unsigned int lines, int pix_fmt, int rotation, unsigned format)
{
	struct vdma_regs *vdma = (struct vdma_regs *)((u32)fbi->reg_base
			+ VDMA_ARBR_CTRL);
	struct vdma_ch_regs *vdma_ch = fbi->id ? &vdma->ch2 : &vdma->ch1;
	unsigned int reg, width, height, bpp;
	FBVideoMode vmode;

	if (lines < 2) {
		pr_warn("%s lines = %d < 2???\n", __func__, lines);
		return;
	}

	width = lcd_width_read(fbi);
	height = lcd_height_read(fbi);
	bpp = lcd_pitch_read(fbi) / width;
	vmode = pixfmt_to_vmode(pix_fmt);
	if (vmode < 0) {
		pr_err("%s pix_fmt %d not supported\n", __func__, pix_fmt);
		return;
	}

	if (!psqu) {
		pxa688_vdma_release(fbi);
		return;
	} else {
		/* select video layer or graphics layer */
		reg = readl(fbi->reg_base + LCD_PN2_SQULN2_CTRL);
		if (fbi->vid)
			reg |= 1 << (24 + fbi->id);
		else
			reg &= ~(1 << (24 + fbi->id));
		writel(reg, fbi->reg_base + LCD_PN2_SQULN2_CTRL);
	}
	reg = (u32)psqu | ((lines/2-1)<<1 | 0x1);
	writel(reg, fbi->reg_base + squln_ctrl(fbi->id));

	pr_debug("%s psqu %x reg %x, width %d height %d bpp %d lines %d\n",
		__func__, psqu, reg, width, height, bpp, lines);

	/* src/dst addr */
	reg = __get_vdma_sa(fbi, vmode, width, height, bpp, rotation);
	writel(reg, &vdma_ch->src_addr);
	writel((u32)psqu, &vdma_ch->dst_addr);

	/* source size */
	reg = __get_vdma_src_sz(fbi, vmode, width, height);
	writel(reg, &vdma_ch->src_size);

	/* size */
	reg = __get_vdma_sz(fbi, vmode, rotation, width, height, bpp);
	writel(reg, &vdma_ch->dst_size);

	/* pitch */
	reg = __get_vdma_pitch(fbi, vmode, rotation, width, height, bpp);
	writel(reg, &vdma_ch->pitch);

	/* rotation ctrl */
	reg = __get_vdma_rot_ctrl(vmode, rotation, format);
	writel(reg, &vdma_ch->rot_ctrl);

	if (vmode == FB_VMODE_YUV422PACKED_SWAPYUorV && rotation == 180) {
		reg = dma_ctrl_read(fbi->id, 0);
		reg &= ~(0x1 << 2);
		dma_ctrl_write(fbi->id, 0, reg);
	}

	/* control */
	reg = __get_vdma_ctrl(fbi, rotation, lines);
	writel(reg, &vdma_ch->ctrl);
}

void pxa688_vdma_config(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	pr_debug("%s fbi %d vid %d vdma_enable %d active %d\n",
		__func__, fbi->id, fbi->vid, mi->vdma_enable, fbi->active);
	if (mi->vdma_enable && fbi->active) {
		int active = fbi->check_modex_active(fbi->id, fbi->active);
		if (active) {
			mi->vdma_lines = pxa688_vdma_get_linenum(fbi,
				fbi->surface.viewPortInfo.rotation);
			pxa688_vdma_set(fbi, mi->sram_paddr, mi->vdma_lines,
				fbi->pix_fmt, fbi->surface.viewPortInfo.rotation,
				fbi->surface.viewPortInfo.yuv_format);
		} else
			pxa688_vdma_release(fbi);
	}
}

void pxa688_vdma_init(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	if (mi->vdma_enable) {
		if (!mi->sram_paddr)
			mi->sram_paddr = pxa688_vdma_squ_malloc(&mi->sram_size);
		pr_info("vdma enabled, sram_paddr 0x%x sram_size 0x%x\n",
			mi->sram_paddr, mi->sram_size);
		pxa688_vdma_clkset(1);
	}
}

void pxa688_vdma_release(struct pxa168fb_info *fbi)
{
	struct vdma_regs *vdma = (struct vdma_regs *)((u32)fbi->reg_base
			+ VDMA_ARBR_CTRL);
	struct vdma_ch_regs *vdma_ch = fbi->id ? &vdma->ch2 : &vdma->ch1;
	unsigned reg; /*, isr, current_time, irq_mask; */

	if (!(vdma_ctrl_read(fbi) & 1))
		return;
#if 0
	isr = readl(fbi->reg_base + SPU_IRQ_ISR);
	if (fbi->id == 0)
		irq_mask = DMA_FRAME_IRQ0_MASK | DMA_FRAME_IRQ1_MASK;
	else if (fbi->id == 1)
		irq_mask = TV_DMA_FRAME_IRQ0_MASK | TV_DMA_FRAME_IRQ1_MASK;
	else
		irq_mask = PN2_DMA_FRAME_IRQ0_MASK | PN2_DMA_FRAME_IRQ1_MASK;
	irq_status_clear(fbi, irq_mask);
	current_time = jiffies;
	while ((readl(fbi->reg_base + SPU_IRQ_ISR) &	irq_mask) == 0) {
		if (jiffies_to_msecs(jiffies - current_time) > EOF_TIMEOUT) {
			pr_err("EOF not detected !");
			break;
		}
	}
#endif

	reg = readl(&vdma_ch->ctrl);
	reg &= ~0xF;
	writel(reg, &vdma_ch->ctrl);

	/* disable squ access */
	reg = readl(fbi->reg_base + squln_ctrl(fbi->id));
	reg &= (~0x1);
	writel(reg, fbi->reg_base + squln_ctrl(fbi->id));
	printk(KERN_DEBUG "%s fbi %d squln_ctrl %x\n", __func__,
		fbi->id, readl(fbi->reg_base + squln_ctrl(fbi->id)));
}

ssize_t vdma_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct vdma_regs *vdma = (struct vdma_regs *)((u32)fbi->reg_base
			+ VDMA_ARBR_CTRL);

	if (!mi || !mi->vdma_enable) {
		pr_info("vdma not enabled\n");
		goto out;
	}

	pr_info("vdma regs base 0x%p\n", vdma);
	pr_info("\tarbr_ctr       (@%3x):\t0x%x\n",
		 (int)(&vdma->arbr_ctr)&0xfff, readl(&vdma->arbr_ctr));
	pr_info("\tirq_raw        (@%3x):\t0x%x\n",
		 (int)(&vdma->irq_raw)&0xfff, readl(&vdma->irq_raw));
	pr_info("\tirq_mask       (@%3x):\t0x%x\n",
		 (int)(&vdma->irq_mask)&0xfff, readl(&vdma->irq_mask));
	pr_info("\tirq_status     (@%3x):\t0x%x\n",
		 (int)(&vdma->irq_status)&0xfff, readl(&vdma->irq_status));
	pr_info("\tmdma_arbr_ctrl (@%3x):\t0x%x\n",
	 (int)(&vdma->mdma_arbr_ctrl)&0xfff, readl(&vdma->mdma_arbr_ctrl));

	pr_info("\nch1 regs\n");
	pr_info("\tdc_saddr       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.dc_saddr)&0xfff, readl(&vdma->ch1.dc_saddr));
	pr_info("\tdc_size        (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.dc_size)&0xfff, readl(&vdma->ch1.dc_size));
	pr_info("\tctrl           (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.ctrl)&0xfff, readl(&vdma->ch1.ctrl));
	pr_info("\tsrc_size       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.src_size)&0xfff, readl(&vdma->ch1.src_size));
	pr_info("\tsrc_addr       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.src_addr)&0xfff, readl(&vdma->ch1.src_addr));
	pr_info("\tdst_addr       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.dst_addr)&0xfff, readl(&vdma->ch1.dst_addr));
	pr_info("\tdst_size       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.dst_size)&0xfff, readl(&vdma->ch1.dst_size));
	pr_info("\tpitch          (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.pitch)&0xfff, readl(&vdma->ch1.pitch));
	pr_info("\trot_ctrl       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch1.rot_ctrl)&0xfff, readl(&vdma->ch1.rot_ctrl));
	pr_info("\tram_ctrl0      (@%3x):\t0x%x\n",
	 (int)(&vdma->ch1.ram_ctrl0)&0xfff, readl(&vdma->ch1.ram_ctrl0));
	pr_info("\tram_ctrl1      (@%3x):\t0x%x\n",
	 (int)(&vdma->ch1.ram_ctrl1)&0xfff, readl(&vdma->ch1.ram_ctrl1));

	pr_info("\nch2 regs\n");
	pr_info("\tdc_saddr       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.dc_saddr)&0xfff, readl(&vdma->ch2.dc_saddr));
	pr_info("\tdc_size        (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.dc_size)&0xfff, readl(&vdma->ch2.dc_size));
	pr_info("\tctrl           (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.ctrl)&0xfff, readl(&vdma->ch2.ctrl));
	pr_info("\tsrc_size       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.src_size)&0xfff, readl(&vdma->ch2.src_size));
	pr_info("\tsrc_addr       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.src_addr)&0xfff, readl(&vdma->ch2.src_addr));
	pr_info("\tdst_addr       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.dst_addr)&0xfff, readl(&vdma->ch2.dst_addr));
	pr_info("\tdst_size       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.dst_size)&0xfff, readl(&vdma->ch2.dst_size));
	pr_info("\tpitch          (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.pitch)&0xfff, readl(&vdma->ch2.pitch));
	pr_info("\trot_ctrl       (@%3x):\t0x%x\n",
		 (int)(&vdma->ch2.rot_ctrl)&0xfff, readl(&vdma->ch2.rot_ctrl));
	pr_info("\tram_ctrl0      (@%3x):\t0x%x\n",
	 (int)(&vdma->ch2.ram_ctrl0)&0xfff, readl(&vdma->ch2.ram_ctrl0));
	pr_info("\tram_ctrl1      (@%3x):\t0x%x\n",
	 (int)(&vdma->ch2.ram_ctrl1)&0xfff, readl(&vdma->ch2.ram_ctrl1));

	pr_info("\nlcd related regs\n");
	pr_info("\tsquln_ctrl     (@%3x):\t0x%x\n",
		 (int)(fbi->reg_base + squln_ctrl(fbi->id))&0xfff,
		 readl(fbi->reg_base + squln_ctrl(fbi->id)));
	pr_info("\tpn2_squln2_ctrl(@%3x):\t0x%x\n",
		 (int)(fbi->reg_base + LCD_PN2_SQULN2_CTRL)&0xfff,
		 readl(fbi->reg_base + LCD_PN2_SQULN2_CTRL));

	pr_info("\nbasic info\n");
	pr_info("\tvdma_lines     %d\n", mi->vdma_lines);
	pr_info("\tsram_paddr     0x%x\n", mi->sram_paddr);
	pr_info("\tsram_size      0x%x\n", mi->sram_size);

out:
	return sprintf(buf, "%d\n", fbi->id);
}
ssize_t vdma_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	pr_info("%s\n", __func__);

	return size;
}
DEVICE_ATTR(vdma, S_IRUGO | S_IWUSR, vdma_show, vdma_store);

#endif
