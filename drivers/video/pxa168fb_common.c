#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include "pxa168fb_common.h"

/* buffer management:
 *    filterBufList: list return to upper layer which indicates buff is free
 *    freelist: list indicates buff is free
 *    waitlist: wait queue which indicates "using" buffer, will be writen in
 *              DMA register
 *    current: buffer on showing
 * Operation:
 *    flip: if !waitlist[0] || !waitlist[1] enqueue to waiting list;
 *          else enqueue the  waitlist[0] to freelist, new buf to waitlist
 *    get freelist: return freelist
 *    eof intr: enqueue current to freelist; dequeue waitlist[0] to current;
 *    buffers are protected spin_lock_irq disable/enable
 *    suspend: when lcd is suspend, move all buffers as "switched",
 *             but don't really set hw register.
 */

static int check_yuv_status(FBVideoMode videoMode, struct pxa168fb_info *fbi)
{
	u32 x;

	if (fbi->vid) {     /* now in video layer */
		x = dma_ctrl_read(fbi->id, 0);

		if (!((videoMode & 0xf00) >> 8)) {
			if ((x & 0xf0000)>>16 == 0x5) {
				/* graphic layer broadcast YUV */
				pr_warning(" vid layer: dma_ctrl0 0x%x\n", x);
				return -EFAULT;
			}
		}
	} else {
		x = dma_ctrl_read(fbi->id, 0);

		if (!((videoMode & 0xf00) >> 8)) {
			if ((x & 0xf00000)>>20 >= 0x5
					&& (x & 0xf00000)>>20 <= 0x7) {
				pr_warning(" gfx layer: dma_ctrl0 0x%x\n", x);
				return -EFAULT; /* video layer broadcast YUV */
			}
		}
	}

	return 0;
}

static void ovlysurface_clear_pitch(struct _sOvlySurface *surface)
{
	surface->viewPortInfo.yPitch = 0;
	surface->viewPortInfo.uPitch = 0;
	surface->viewPortInfo.vPitch = 0;
}

static void update_surface(struct _sOvlySurface *surface)
{
	int tmp;

	if (surface->viewPortInfo.rotation == 90 ||
		surface->viewPortInfo.rotation == 270) {
		surface->viewPortInfo.rotation = 360 -
		surface->viewPortInfo.rotation;
		tmp = surface->viewPortInfo.srcWidth;
		surface->viewPortInfo.srcWidth =
		surface->viewPortInfo.srcHeight;
		surface->viewPortInfo.srcHeight = tmp;
		switch (surface->videoMode) {
		case FB_VMODE_YUV422PACKED:
			surface->videoMode = FB_VMODE_YUV422PACKED_IRE_90_270;
			surface->viewPortInfo.yuv_format = 1;
			ovlysurface_clear_pitch(surface);
			break;
		case FB_VMODE_YUV422PACKED_SWAPUV:
			surface->videoMode = FB_VMODE_YUV422PACKED_IRE_90_270;
			surface->viewPortInfo.yuv_format = 2;
			ovlysurface_clear_pitch(surface);
			break;
		case FB_VMODE_YUV422PACKED_SWAPYUorV:
			surface->videoMode = FB_VMODE_YUV422PACKED_IRE_90_270;
			surface->viewPortInfo.yuv_format = 4;
			ovlysurface_clear_pitch(surface);
			break;
		default:
			surface->viewPortInfo.yuv_format = 0;
		}
	}
}

int unsupport_format(struct pxa168fb_info *fbi, struct _sViewPortInfo
		viewPortInfo, FBVideoMode videoMode)
{
	if (check_yuv_status(videoMode, fbi) < 0)
		return 1;

	if ((viewPortInfo.rotation == 0) || (viewPortInfo.rotation == 1)) {
		if (!fbi->vid) {
			/* In graphic layer now */
			switch (videoMode) {
			case FB_VMODE_YUV422PLANAR:
			case FB_VMODE_YUV420PLANAR:
				pr_err("Planar is not supported!\n");
				return 1;
			default:
				break;
			}
		}

		return 0;
	}

	if (viewPortInfo.srcHeight == 1080) {
		pr_err("1080P rotation is not supported!\n");
		return 1;
	}
	if (viewPortInfo.srcHeight == 720) {
		if (viewPortInfo.rotation == 180) {
			pr_err("720p rotation 180 is not supported!\n");
			return 1;
		}
	}

	switch (videoMode) {
	case FB_VMODE_YUV422PLANAR:
	case FB_VMODE_YUV422PLANAR_SWAPUV:
	case FB_VMODE_YUV422PLANAR_SWAPYUorV:
	case FB_VMODE_YUV420PLANAR:
	case FB_VMODE_YUV420PLANAR_SWAPUV:
	case FB_VMODE_YUV420PLANAR_SWAPYUorV:
		pr_err("Planar is not supported!\n");
		return 1;
	default:
		break;
	}
	return 0;
}

int convert_pix_fmt(u32 vmode)
{
/*	pr_info("vmode=%d\n", vmode); */
	switch (vmode) {
	case FB_VMODE_YUV422PACKED:
		return PIX_FMT_YUV422PACK;
	case FB_VMODE_YUV422PACKED_SWAPUV:
		return PIX_FMT_YVU422PACK;
	case FB_VMODE_YUV422PLANAR:
		return PIX_FMT_YUV422PLANAR;
	case FB_VMODE_YUV422PLANAR_SWAPUV:
		return PIX_FMT_YVU422PLANAR;
	case FB_VMODE_YUV420PLANAR:
		return PIX_FMT_YUV420PLANAR;
	case FB_VMODE_YUV420PLANAR_SWAPUV:
		return PIX_FMT_YVU420PLANAR;
	case FB_VMODE_YUV422PACKED_SWAPYUorV:
		return PIX_FMT_YUYV422PACK;
	case FB_VMODE_YUV422PACKED_IRE_90_270:
		return PIX_FMT_YUV422PACK_IRE_90_270;
	case FB_VMODE_RGB565:
		return PIX_FMT_RGB565;
	case FB_VMODE_BGR565:
		return PIX_FMT_BGR565;
	case FB_VMODE_RGB1555:
		return PIX_FMT_RGB1555;
	case FB_VMODE_BGR1555:
		return PIX_FMT_BGR1555;
	case FB_VMODE_RGB888PACK:
		return PIX_FMT_RGB888PACK;
	case FB_VMODE_BGR888PACK:
		return PIX_FMT_BGR888PACK;
	case FB_VMODE_RGBA888:
		return PIX_FMT_RGBA888;
	case FB_VMODE_BGRA888:
		return PIX_FMT_BGRA888;
	case FB_VMODE_RGB888UNPACK:
		return PIX_FMT_RGB888UNPACK;
	case FB_VMODE_BGR888UNPACK:
		return PIX_FMT_BGR888UNPACK;
	case FB_VMODE_YUV422PLANAR_SWAPYUorV:
	case FB_VMODE_YUV420PLANAR_SWAPYUorV:
	default:
		return -1;
	}
}

int set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt)
{
	switch (pix_fmt) {
	case PIX_FMT_RGB565:
		var->bits_per_pixel = 16;
		var->red.offset = 11;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_BGR565:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 11;   var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_RGB1555:
		var->bits_per_pixel = 16;
		var->red.offset = 10;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 5 << 20;
		break;
	case PIX_FMT_BGR1555:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 10;   var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 5 << 20;
		break;
	case PIX_FMT_RGB888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 6 << 20;
		break;
	case PIX_FMT_BGR888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 6 << 20;
		break;
	case PIX_FMT_RGB888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 7 << 20;
		break;
	case PIX_FMT_BGR888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 7 << 20;
		break;
	case PIX_FMT_RGBA888:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 8 << 20;
		break;
	case PIX_FMT_BGRA888:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 8 << 20;
		break;
	case PIX_FMT_YUYV422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 8;     var->red.length = 16;
		var->green.offset = 4;   var->green.length = 16;
		var->blue.offset = 0;   var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 9 << 20;
		break;
	case PIX_FMT_YVU422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 16;
		var->green.offset = 8;   var->green.length = 16;
		var->blue.offset = 12;   var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 9 << 20;
		break;
	case PIX_FMT_YUV422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 4;     var->red.length = 16;
		var->green.offset = 12;   var->green.length = 16;
		var->blue.offset = 0;    var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 9 << 20;
		break;
	case PIX_FMT_PSEUDOCOLOR:
		var->bits_per_pixel = 8;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 0;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		break;
	case PIX_FMT_YUV422PLANAR:
		var->bits_per_pixel = 16;
		var->red.offset = 8;	 var->red.length = 8;
		var->green.offset = 4;   var->green.length = 4;
		var->blue.offset = 0;   var->blue.length = 4;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 3 << 20;
		break;
	case PIX_FMT_YVU422PLANAR:
		var->bits_per_pixel = 16;
		var->red.offset = 0;	 var->red.length = 8;
		var->green.offset = 8;   var->green.length = 4;
		var->blue.offset = 12;   var->blue.length = 4;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 3 << 20;
		break;
	case PIX_FMT_YUV420PLANAR:
		var->bits_per_pixel = 12;
		var->red.offset = 4;	 var->red.length = 8;
		var->green.offset = 2;   var->green.length = 2;
		var->blue.offset = 0;   var->blue.length = 2;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 4 << 20;
		break;
	case PIX_FMT_YVU420PLANAR:
		var->bits_per_pixel = 12;
		var->red.offset = 0;	 var->red.length = 8;
		var->green.offset = 8;   var->green.length = 2;
		var->blue.offset = 10;   var->blue.length = 2;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 4 << 20;
		break;
	/*
	 * YUV422 Packed will be YUV444 Packed after
	 * IRE 90 and 270 degree rotation
	 */
	case PIX_FMT_YUV422PACK_IRE_90_270:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 7 << 20;
		break;
	default:
		return  -EINVAL;
	}

	return 0;
}

int determine_best_pix_fmt(struct fb_var_screeninfo *var,
			 unsigned int compat_mode)
{
	unsigned char pxa_format;

	/* compatibility switch: if var->nonstd MSB is 0xAA then skip to
	 * using the nonstd variable to select the color space
	 */
	if (compat_mode != 0x2625) {

		/*
		 * Pseudocolor mode?
		 */
		if (var->bits_per_pixel == 8)
			return PIX_FMT_PSEUDOCOLOR;
		/*
		 * Check for YUV422PACK.
		 */
		if (var->bits_per_pixel == 16 && var->red.length == 16 &&
		    var->green.length == 16 && var->blue.length == 16) {
			if (var->red.offset >= var->blue.offset) {
				if (var->red.offset == 4)
					return PIX_FMT_YUV422PACK;
				else
					return PIX_FMT_YUYV422PACK;
			} else
				return PIX_FMT_YVU422PACK;
		}
		/*
		 * Check for YUV422PLANAR.
		 */
		if (var->bits_per_pixel == 16 && var->red.length == 8 &&
		    var->green.length == 4 && var->blue.length == 4) {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMT_YUV422PLANAR;
			else
				return PIX_FMT_YVU422PLANAR;
		}

		/*
		 * Check for YUV420PLANAR.
		 */
		if (var->bits_per_pixel == 12 && var->red.length == 8 &&
		    var->green.length == 2 && var->blue.length == 2) {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMT_YUV420PLANAR;
			else
				return PIX_FMT_YVU420PLANAR;
		}
		/*
		 * Check for 565/1555.
		 */
		if (var->bits_per_pixel == 16 && var->red.length <= 5 &&
		    var->green.length <= 6 && var->blue.length <= 5) {
			if (var->transp.length == 0) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB565;
				else
					return PIX_FMT_BGR565;
			}

			if (var->transp.length == 1 && var->green.length <= 5) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB1555;
				else
					return PIX_FMT_BGR1555;
			}

			/* fall through */
		}

		/*
		 * Check for 888/A888.
		 */
		if (var->bits_per_pixel <= 32 && var->red.length <= 8 &&
		    var->green.length <= 8 && var->blue.length <= 8) {
			if (var->bits_per_pixel == 24 &&
				 var->transp.length == 0) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB888PACK;
				else
					return PIX_FMT_BGR888PACK;
			}

			if (var->bits_per_pixel == 32 &&
				 var->transp.offset == 24) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGBA888;
				else
					return PIX_FMT_BGRA888;
			} else {
				if (var->transp.length == 8) {
					if (var->red.offset >= var->blue.offset)
						return PIX_FMT_RGB888UNPACK;
					else
						return PIX_FMT_BGR888UNPACK;
				} else
					return PIX_FMT_YUV422PACK_IRE_90_270;

			}
			/* fall through */
		}
	} else {

		pxa_format = (var->nonstd >> 20) & 0xf;

		switch (pxa_format) {
		case 0:
			return PIX_FMT_RGB565;
			break;
		case 3:
			return PIX_FMT_YUV422PLANAR;
			break;
		case 4:
			return PIX_FMT_YUV420PLANAR;
			break;
		case 5:
			return PIX_FMT_RGB1555;
			break;
		case 6:
			return PIX_FMT_RGB888PACK;
			break;
		case 7:
			return PIX_FMT_RGB888UNPACK;
			break;
		case 8:
			return PIX_FMT_RGBA888;
			break;
		case 9:
			return PIX_FMT_YUV422PACK;
			break;

		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

int pxa168fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int pix_fmt;

	dev_dbg(fi->dev, "Enter %s\n", __func__);
	if (var->bits_per_pixel == 8) {
		pr_info("bits per pixel too small\n");
		return -EINVAL;
	}

	/* compatibility mode: if the MSB of var->nonstd is 0xAA then
	 * set xres_virtual and yres_virtual to xres and yres.
	 */

	if ((var->nonstd >> 24) == 0xAA)
		fbi->compat_mode = 0x2625;

	if ((var->nonstd >> 24) == 0x55)
		fbi->compat_mode = 0x0;

	/*
	 * Basic geometry sanity checks.
	 */

	if (var->xoffset + var->xres > var->xres_virtual) {
		pr_err("ERROR: xoffset(%d) + xres(%d) is greater than "
			"xres_virtual(%d)\n", var->xoffset, var->xres,
			var->xres_virtual);
		return -EINVAL;
	}
	if (var->yoffset + var->yres > var->yres_virtual) {
		pr_err("ERROR: yoffset(%d) + yres(%d) is greater than "
			"yres_virtual(%d)\n", var->yoffset, var->yres,
			var->yres_virtual);
		return -EINVAL;
	}

	if (var->xres + var->right_margin +
	    var->hsync_len + var->left_margin > 3500) {
		pr_err("ERROR: var->xres(%d) + var->right_margin(%d) + "
			"var->hsync_len(%d) + var->left_margin(%d) > 2048",
			var->xres, var->right_margin, var->hsync_len,
			var->left_margin);
		return -EINVAL;
	}
	if (var->yres + var->lower_margin +
	    var->vsync_len + var->upper_margin > 2500) {
		pr_err("var->yres(%d) + var->lower_margin(%d) + "
			"var->vsync_len(%d) + var->upper_margin(%d) > 2048",
			var->yres, var->lower_margin, var->vsync_len,
			var->upper_margin);
		return -EINVAL;
	}

	/*
	 * Check size of framebuffer.
	 */
	if (mi->mmap && (var->xres_virtual * var->yres_virtual *
	    (var->bits_per_pixel >> 3) > fbi->fb_size)) {
		pr_err("xres_virtual(%d) * yres_virtual(%d) * "
			"(bits_per_pixel(%d) >> 3) > max_fb_size(%d)",
			var->xres_virtual, var->yres_virtual,
			var->bits_per_pixel, fbi->fb_size);
		return -EINVAL;
	}

	/*
	 * Select most suitable hardware pixel format.
	 */
	pix_fmt = determine_best_pix_fmt(var, fbi->compat_mode);
	dev_dbg(fi->dev, "%s determine_best_pix_fmt returned: %d\n",
		 __func__, pix_fmt);
	if (pix_fmt < 0)
		return pix_fmt;

	return 0;
}

void pxa168fb_update_addr(struct pxa168fb_info *fbi,
	struct _sVideoBufferAddr *new_addr)
{
	fbi->new_addr[0] = (unsigned long)new_addr->startAddr[0];
	fbi->new_addr[1] = (unsigned long)new_addr->startAddr[1];
	fbi->new_addr[2] = (unsigned long)new_addr->startAddr[2];
}

int check_surface(struct fb_info *fi,
			FBVideoMode new_mode,
			struct _sViewPortInfo *new_info,
			struct _sViewPortOffset *new_offset,
			struct _sVideoBufferAddr *new_addr)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &fi->var;
	int changed = 0;

	dev_dbg(fi->dev, "Enter %s\n", __func__);

	/*
	 * check mode
	 */
	if (new_mode >= 0 && fbi->surface.videoMode != new_mode) {
		fbi->surface.videoMode = new_mode;
		fbi->pix_fmt = convert_pix_fmt(new_mode);
		set_pix_fmt(var, fbi->pix_fmt);
		changed = 1;
	}
	/*
	 * check view port settings.
	 */
	if (new_info && memcmp(&fbi->surface.viewPortInfo, new_info,
			sizeof(struct _sViewPortInfo))) {
		if (!(new_addr && new_addr->startAddr[0])) {
			if (mi->mmap && (((new_info->srcWidth *
			new_info->srcHeight * var->bits_per_pixel / 8) * 2)
			> fbi->fb_size)) {
				pr_err("%s: requested memory buffer size %d"
					"exceed the max limit %d!\n", __func__,
				(new_info->srcWidth * new_info->srcHeight
				 * var->bits_per_pixel / 4), fbi->fb_size);
				return changed;
			}
		}
		var->xres_virtual = new_info->srcWidth;
		var->yres_virtual = new_info->srcHeight * 2;
		var->xres = new_info->srcWidth;
		var->yres = new_info->srcHeight;
		fbi->surface.viewPortInfo = *new_info;
		changed = 1;
	}

	/*
	 * Check offset
	 */
	if (new_offset && memcmp(&fbi->surface.viewPortOffset, new_offset,
		sizeof(struct _sViewPortOffset))) {
		fbi->surface.viewPortOffset.xOffset = new_offset->xOffset;
		fbi->surface.viewPortOffset.yOffset = new_offset->yOffset;
		changed = 1;
	}
	/*
	 * Check buffer address
	 */
	if (new_addr && new_addr->startAddr[0] &&
	    fbi->new_addr[0] != (unsigned long)new_addr->startAddr[0]) {
		/*check overlay buffer address and pitch alignment*/
		if (((unsigned long)new_addr->startAddr[0] & 63) &&
			(fbi->surface.viewPortInfo.yPitch & 7) &&
			(fbi->surface.viewPortInfo.uPitch & 7)) {
			printk(KERN_WARNING "Ovly: the memory base 0x%08lx is"
			" not 64 bytes aligned, pitch is not 8 bytes aligned,"
			" video playback maybe wrong!\n",
			(unsigned long)new_addr->startAddr[0]);
		}

		pxa168fb_update_addr(fbi, new_addr);
		changed = 1;
	}

	return changed;
}

int check_surface_addr(struct fb_info *fi, struct _sOvlySurface *surface)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct _sVideoBufferAddr *new_addr = &surface->videoBufferAddr;
	int changed = 0;

	dev_dbg(fi->dev, "Enter %s\n", __func__);

	/* Check buffer address */
	if (new_addr && new_addr->startAddr[0] &&
	    fbi->new_addr[0] != (unsigned long)new_addr->startAddr[0]) {
		pxa168fb_update_addr(fbi, new_addr);
		changed = 1;
	}

	return changed;
}

int check_modex_active(struct pxa168fb_info *fbi)
{
	int active;

	if (!fbi)
		return 0;

	active = fbi->active && fbi->dma_on;

	pr_debug("%s fbi[%d] vid %d fbi->active %d"
		" dma_on %d: %d\n", __func__, fbi->id,
		fbi->vid, fbi->active, fbi->dma_on, active);
	return active;
}

void collectFreeBuf(struct pxa168fb_info *fbi,
		u8 *filterList[][3], struct _sSurfaceList *srflist)
{
	struct _sSurfaceList *surface_list = 0;
	struct _sOvlySurface *srf = 0;
	struct list_head *pos, *n;
	int i = 0;

	if (!filterList || !srflist)
		return;

	if (fbi->debug == 1)
		printk(KERN_DEBUG"%s fbi %d vid %d\n",
			 __func__, fbi->id, fbi->vid);

	list_for_each_safe(pos, n, &srflist->surfacelist) {
		surface_list = list_entry(pos, struct _sSurfaceList,
			surfacelist);
		list_del(pos);
		srf = &surface_list->surface;

		if (srf) {
			/* save ptrs which need to be freed.*/
			filterList[i][0] = srf->videoBufferAddr.startAddr[0];
			filterList[i][1] = srf->videoBufferAddr.startAddr[1];
			filterList[i][2] = srf->videoBufferAddr.startAddr[2];
			if (fbi->debug == 1)
				printk(KERN_DEBUG"buf %p will be returned\n",
				 srf->videoBufferAddr.startAddr[0]);
		}

		i++;

		if (i >= MAX_QUEUE_NUM)
			break;

		kfree(surface_list);
	}
}

void clearFilterBuf(u8 *ppBufList[][3], int iFlag)
{
	/* Check null pointer. */
	if (!ppBufList)
		return;
	if (RESET_BUF & iFlag)
		memset(ppBufList, 0, 3 * MAX_QUEUE_NUM * sizeof(u8 *));
}

void buf_clear(struct _sSurfaceList *srflist, int iFlag)
{
	struct _sSurfaceList *surface_list;
	struct list_head *pos, *n;

	/* Check null pointer. */
	if (!srflist)
		return;

	/* free */
	if (FREE_ENTRY & iFlag) {
		list_for_each_safe(pos, n, &srflist->surfacelist) {
			surface_list = list_entry(pos, struct _sSurfaceList,
				surfacelist);
			list_del(pos);
			kfree(surface_list);
		}
	}
}

void clear_buffer(struct pxa168fb_info *fbi)
{
	unsigned long flags;

	spin_lock_irqsave(&fbi->buf_lock, flags);
	clearFilterBuf(fbi->filterBufList, RESET_BUF);
	buf_clear(&fbi->buf_freelist, FREE_ENTRY);
	buf_clear(&fbi->buf_waitlist, FREE_ENTRY);
	kfree(fbi->buf_current);
	fbi->buf_current = 0;
	fbi->surface_set = 0;
	spin_unlock_irqrestore(&fbi->buf_lock, flags);
}

void buf_endframe(void *point)
{
	struct fb_info *fi = (struct fb_info *)point;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct _sOvlySurface *pOvlySurface;
	int ret;
	struct _sSurfaceList *surface_list = 0;

	if (list_empty(&fbi->buf_waitlist.surfacelist))
		return;

	surface_list = list_first_entry(&fbi->buf_waitlist.surfacelist,
		struct _sSurfaceList, surfacelist);
	pOvlySurface = &surface_list->surface;
	list_del(&surface_list->surfacelist);

	/* Update new surface settings */
	ret = fbi->update_buff(fi, pOvlySurface, 0);

	if (!ret) {
		/* enqueue current to freelist */
		if (fbi->buf_current)
			list_add_tail(&fbi->buf_current->surfacelist,
				&fbi->buf_freelist.surfacelist);

		fbi->buf_current = surface_list;
	} else {
		/* enqueue the repeated buffer to freelist */
		list_add_tail(&surface_list->surfacelist,
			&fbi->buf_freelist.surfacelist);
		pr_info("Detect a same surface flipped in, "
			"may flicker.\n");
	}
}

#if 0
static int get_list_count(struct _sSurfaceList *srflist)
{
	struct list_head *pos, *n;
	int count = 0;

	list_for_each_safe(pos, n, &srflist->surfacelist)
		count++;
	return count;
}
#endif

int flip_buffer(struct fb_info *info, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	unsigned long flags;
	struct _sSurfaceList *surface_list = 0, *srflist = 0;
	struct _sOvlySurface *surface = 0;
	u8 *start_addr[3], *input_data;
	u32 length;

	mutex_lock(&fbi->access_ok);
	surface_list = kmalloc(sizeof(struct _sSurfaceList),
			GFP_KERNEL);
	if (surface_list == NULL) {
		mutex_unlock(&fbi->access_ok);
		return -EFAULT;
	}
	surface = &surface_list->surface;

	/* Get user-mode data. */
	if (copy_from_user(surface, argp,
				sizeof(struct _sOvlySurface))) {
		kfree(surface_list);
		mutex_unlock(&fbi->access_ok);
		return -EFAULT;
	}
	if (unsupport_format(fbi, surface->viewPortInfo, surface->videoMode)) {
		kfree(surface_list);
		mutex_unlock(&fbi->access_ok);
		return -EFAULT;
	}

	update_surface(surface);

	length = surface->videoBufferAddr.length;
	start_addr[0] = surface->videoBufferAddr.startAddr[0];
	input_data = surface->videoBufferAddr.inputData;

	if (fbi->debug == 1)
		printk(KERN_DEBUG"fbi %d vid %d flip surface %p buf %p\n",
				fbi->id, fbi->vid, surface, start_addr[0]);
	/*
	 * Has DMA addr?
	 */
	if (start_addr[0] && (!input_data)) {
		spin_lock_irqsave(&fbi->buf_lock, flags);
#if 0
		if (get_list_count(&fbi->buf_waitlist) >= 2) {
			/*if there are more than two frames in waitlist, dequeue
			*the older frame and enqueue it to freelist,
			*then enqueue this frame to waitlist*/
#else
		while (!list_empty(&fbi->buf_waitlist.surfacelist)) {
			/* free the waitlist elements if any */
#endif
			srflist = list_first_entry(&fbi->buf_waitlist.surfacelist,
				struct _sSurfaceList, surfacelist);
			list_del(&srflist->surfacelist);
			list_add_tail(&srflist->surfacelist,
				&fbi->buf_freelist.surfacelist);
		}
		list_add_tail(&surface_list->surfacelist,
			&fbi->buf_waitlist.surfacelist);

		spin_unlock_irqrestore(&fbi->buf_lock, flags);
	} else {
		if (!mi->mmap) {
			pr_err("fbi %d(line %d): input err, mmap is not"
				" supported\n", fbi->id, __LINE__);
			kfree(surface_list);
			mutex_unlock(&fbi->access_ok);
			return -EINVAL;
		}

		/* update buffer */
		fbi->update_buff(info, surface, 1);

		/* copy buffer */
		if (input_data) {
			if (NEED_VSYNC(fbi))
				wait_for_vsync(fbi);
			/* if support hw DMA, replace this. */
			if (copy_from_user(fbi->fb_start,
						input_data, length)){
				kfree(surface_list);
				mutex_unlock(&fbi->access_ok);
				return -EFAULT;
			}
			kfree(surface_list);
			mutex_unlock(&fbi->access_ok);
			return 0;
		}

		/*
		 * if it has its own physical address,
		 * switch to this memory. don't support YUV planar format
		 * with split YUV buffers. but below code seems have no
		 * chancee to execute. - FIXME
		 */
		if (start_addr[0]) {
			if (fbi->mem_status)
				free_pages(
						(unsigned long)fbi->fb_start,
						get_order(fbi->fb_size));
			else
				dma_free_writecombine(fbi->dev,
						fbi->fb_size,
						fbi->fb_start,
						fbi->fb_start_dma);

			fbi->fb_start = __va(start_addr[0]);
			fbi->fb_size = length;
			fbi->fb_start_dma =
				(dma_addr_t)__pa(fbi->fb_start);
			fbi->mem_status = 1;
			info->fix.smem_start = fbi->fb_start_dma;
			info->fix.smem_len = fbi->fb_size;
			info->screen_base = fbi->fb_start;
			info->screen_size = fbi->fb_size;
		}
		kfree(surface_list);
	}

	mutex_unlock(&fbi->access_ok);

	return 0;
}

static void free_buf(struct pxa168fb_info *fbi)
{
	struct list_head *pos, *n;

	/* put all buffers into free list */
	list_for_each_safe(pos, n, &fbi->buf_waitlist.surfacelist) {
		list_del(pos);
		list_add_tail(pos, &fbi->buf_freelist.surfacelist);
	}

	if (fbi->buf_current) {
		list_add_tail(&fbi->buf_current->surfacelist,
			&fbi->buf_freelist.surfacelist);
		fbi->buf_current = 0;
	}

	/* clear some globals */
	memset(&fbi->surface, 0, sizeof(struct _sOvlySurface));
	fbi->new_addr[0] = 0; fbi->new_addr[1] = 0; fbi->new_addr[2] = 0;
}

int get_freelist(struct fb_info *info, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)info->par;
	unsigned long flags;

	if (fbi->debug == 1)
		printk(KERN_DEBUG"fbi %d vid %d get freelist\n",
				 fbi->id, fbi->vid);

	spin_lock_irqsave(&fbi->buf_lock, flags);

	/* when lcd is suspend, move all buffers as "switched"*/
	if (!(gfx_info.fbi[fbi->id]->active))
		buf_endframe(info);
	/* when video layer dma is off, free all buffers */
	if (!fbi->dma_on)
		free_buf(fbi);

	/* Collect expired frame to list */
	collectFreeBuf(fbi, fbi->filterBufList, &fbi->buf_freelist);
	if (copy_to_user(argp, fbi->filterBufList,
				3*MAX_QUEUE_NUM*sizeof(u8 *))) {
		spin_unlock_irqrestore(&fbi->buf_lock, flags);
		return -EFAULT;
	}
	clearFilterBuf(fbi->filterBufList, RESET_BUF);
	spin_unlock_irqrestore(&fbi->buf_lock, flags);

	if (fbi->debug == 1)
		printk(KERN_DEBUG"fbi %d vid %d get freelist end\n",
				fbi->id, fbi->vid);

	return 0;
}

void set_dma_active(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct lcd_regs *regs = get_regs(fbi->id);
	struct fb_var_screeninfo *v = &gfx_info.fbi[fbi->id]->fb_info->var;
	u32 flag = fbi->vid ? CFG_DMA_ENA_MASK : CFG_GRA_ENA_MASK;
	u32 enable = fbi->vid ? CFG_DMA_ENA(1) : CFG_GRA_ENA(1);
	u32 value, dma1, v_size_dst, screen_active, active = 0;

	if (fbi->new_addr[0] || mi->mmap ||
		 (!fbi->vid && fb_share && fbi->id == 1))
		active = check_modex_active(fbi);

	value = active ? enable : 0;
	/* if video layer is full screen without alpha blending
	 * then turn off graphics dma to save bandwidth */
	if (fbi->vid && active) {
		dma1 = dma_ctrl_read(fbi->id, 1);
		dma1 &= (CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK);
		if (dma1 == (CFG_ALPHA_MODE(2) | CFG_ALPHA(0xff))) {
			v_size_dst = readl(&regs->v_size_z);
			screen_active = readl(&regs->screen_active);
			if (v->vmode & FB_VMODE_INTERLACED) {
				screen_active = (screen_active & 0xffff) |
					(((screen_active >> 16) << 1) << 16);
			}
			if (v_size_dst == screen_active) {
				flag |= CFG_GRA_ENA_MASK;
				value &= ~CFG_GRA_ENA_MASK;
			} else if (check_modex_active(gfx_info.fbi[fbi->id])) {
				flag |= CFG_GRA_ENA_MASK;
				value |= CFG_GRA_ENA_MASK;
			}
		}
	}

	dma_ctrl_set(fbi->id, 0, flag, value);

	pr_debug("%s fbi %d: vid %d mask %x vaule %x fbi->active %d\
		 new_addr %lu\n", __func__, fbi->id, fbi->vid, flag,
		 value, fbi->active, fbi->new_addr[0]);
}

int dispd_dma_enabled(struct pxa168fb_info *fbi)
{
	if (irqs_disabled() || !fbi->active)
		return 0;

	/* check whether display done irq enabled */
	if (!(readl(fbi->reg_base + SPU_IRQ_ENA) &
		display_done_imask(fbi->id)))
		return 0;

	/* check whether path clock is disabled */
	if (readl(fbi->reg_base + clk_div(fbi->id)) & (SCLK_DISABLE))
		return 0;

	/* in modex dma may not be enabled */
	return dma_ctrl_read(fbi->id, 0) & (fbi->vid ?
		CFG_DMA_ENA_MASK : CFG_GRA_ENA_MASK);
}

void clear_dispd_irq(struct pxa168fb_info *fbi)
{
	int isr = readl(fbi->reg_base + SPU_IRQ_ISR);

	if ((isr & display_done_imask(fbi->id))) {
		irq_status_clear(fbi->id, display_done_imask(fbi->id));
		pr_info("fbi %d irq miss, clear isr %x\n", fbi->id, isr);
	}
}

void wait_for_vsync(struct pxa168fb_info *fbi)
{
	atomic_set(&fbi->w_intr, 0);
	pr_debug("fbi->id %d vid: %d\n", fbi->id, fbi->vid);

	wait_event_interruptible_timeout(fbi->w_intr_wq,
		atomic_read(&fbi->w_intr), HZ/20);

	/* handle timeout case, to w/a irq miss */
	if (atomic_read(&fbi->w_intr) == 0)
		clear_dispd_irq(fbi);
}

void pxa168fb_list_init(struct pxa168fb_info *fbi)
{
	INIT_LIST_HEAD(&fbi->buf_freelist.surfacelist);
	INIT_LIST_HEAD(&fbi->buf_waitlist.surfacelist);
	fbi->buf_current = 0;
}

void pxa168fb_misc_update(struct pxa168fb_info *fbi)
{
	pxa688_vdma_config(fbi);
	if (fbi->vid && vid_vsmooth)
		pxa688fb_vsmooth_set(fbi->id, 1, vid_vsmooth);
	if (!fbi->vid) {
		pxa688fb_partdisp_update(fbi->id);
		if (gfx_vsmooth)
			pxa688fb_vsmooth_set(fbi->id, 0, gfx_vsmooth);
	}
}

void set_start_address(struct fb_info *info,
	 int xoffset, int yoffset, int wait_vsync)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &info->var;
	struct lcd_regs *regs;
	unsigned long addr_y0 = 0, addr_u0 = 0, addr_v0 = 0;
	int  pixel_offset;

	dev_dbg(info->dev, "Enter %s\n", __func__);

	if (fbi->new_addr[0]) {
		addr_y0 = fbi->new_addr[0];
		addr_u0 = fbi->new_addr[1];
		addr_v0 = fbi->new_addr[2];
		if (fbi->debug == 1)
			printk(KERN_DEBUG"%s: buffer updated to %x\n",
				 __func__, (int)fbi->new_addr[0]);
	} else {
		if (!mi->mmap) {
			pr_debug("fbi %d(line %d): input err, mmap is not"
				" supported\n", fbi->id, __LINE__);
			return;
		}
		pixel_offset = (yoffset * var->xres_virtual) + xoffset;
		addr_y0 = fbi->fb_start_dma + (pixel_offset *
			(var->bits_per_pixel >> 3));
		if ((fbi->pix_fmt >= 12) && (fbi->pix_fmt <= 15))
			addr_u0 = addr_y0 + var->xres * var->yres;

		if ((fbi->pix_fmt >> 1) == 6)
			addr_v0 = addr_u0 + (var->xres * var->yres >> 1);
		else if ((fbi->pix_fmt >> 1) == 7)
			addr_v0 = addr_u0 + (var->xres * var->yres >> 2);
	}

	regs = get_regs(fbi->id);
	if (fbi->vid) {
		writel(addr_y0, &regs->v_y0);
		if (fbi->pix_fmt >= 12 && fbi->pix_fmt <= 15) {
			writel(addr_u0, &regs->v_u0);
			writel(addr_v0, &regs->v_v0);
		}
	} else
		writel(addr_y0, &regs->g_0);

	set_dma_active(fbi);

	if (wait_vsync)
		fbi->misc_update = 1;
	else
		pxa168fb_misc_update(fbi);

	/* return until the address take effect after vsync occurs */
	if (wait_vsync && NEED_VSYNC(fbi))
		wait_for_vsync(fbi);
}

void set_dma_control0(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi;
	struct _sOvlySurface *surface = &fbi->surface;
	u32 x = 0, x_bk = 0, pix_fmt;

	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __func__);

	if (surface->videoMode > 0)
		fbi->pix_fmt = convert_pix_fmt(surface->videoMode);

	mi = fbi->dev->platform_data;
	pix_fmt = fbi->pix_fmt;

	/* Get reg's current value */
	x_bk = x = dma_ctrl_read(fbi->id, 0);

	/* clear affected bits */
	x &= ~dma_mask(fbi->vid);

	/* enable horizontal smooth filter */
	x |= dma_hsmooth(fbi->vid, 1);

	/* If we are in a pseudo-color mode, we need to enable
	 * palette lookup  */
	if (pix_fmt == PIX_FMT_PSEUDOCOLOR)
		x |= dma_palette(1);

	/* Configure hardware pixel format */
	x |= dma_fmt(fbi->vid, (pix_fmt & ~0x1000) >> 1);

	/*
	 * color format in memory:
	 * PXA168/PXA910:
	 * PIX_FMT_YUV422PACK: UYVY(CbY0CrY1)
	 * PIX_FMT_YUV422PLANAR: YUV
	 * PIX_FMT_YUV420PLANAR: YUV
	 */
	if (((pix_fmt & ~0x1000) >> 1) == 5) {
		/* YUV422PACK, YVU422PACK, YUYV422PACK */
		x |= dma_csc(fbi->vid, 1);
		x |= dma_swaprb(fbi->vid, mi->panel_rbswap);
		if (pix_fmt & 0x1000)
			/* YUYV422PACK */
			x |= dma_swapyuv(fbi->vid, 1);
		else
			/* YVU422PACK */
			x |= dma_swapuv(fbi->vid, pix_fmt & 1);
	} else if (pix_fmt >= 12) {
		/* PIX_FMT_YUV422PACK_IRE_90_270 is here */
		if (!fbi->vid)
			pr_err("%s fmt %d not supported on graphics layer...\n",
				__func__, pix_fmt);
		/* PLANAR, YUV422PACK_IRE_90_270, PSEUDOCOLOR */
		x |= dma_csc(fbi->vid, 1);
		x |= dma_swapuv(fbi->vid, pix_fmt & 1);
		x |= dma_swaprb(fbi->vid, (mi->panel_rbswap));
	} else {
		/* RGB formats */
		/* Check red and blue pixel swap.
		 * 1. source data swap. BGR[M:L] rather than RGB[M:L]
		 *    is stored in memeory as source format.
		 * 2. panel output data swap
		 */
		x |= dma_swaprb(fbi->vid, ((pix_fmt & 1) ^ 1) ^
			 (mi->panel_rbswap));
	}

	/* clear reserved bits if not panel path */
	if (fbi->id)
		x &= ~CFG_ARBFAST_ENA(1);

	if (x_bk != x) {
		dma_ctrl_write(fbi->id, 0, x);
		pr_debug("set fbi %d, layer %d, dma ctrl0(0x%x -> 0x%x)\n",
			 fbi->id, fbi->vid, x_bk, x);
	}
}

void set_screen(struct pxa168fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	struct _sOvlySurface *surface = &fbi->surface;
	struct lcd_regs *regs;
	u32 xres, yres, xres_z, yres_z, xres_virtual, bits_per_pixel;
	u32 left = 0, top = 0, pitch[3], x;

	pitch[0] = surface->viewPortInfo.yPitch;
	pitch[1] = surface->viewPortInfo.uPitch;
	pitch[2] = surface->viewPortInfo.vPitch;

	var = &fbi->fb_info->var;
	regs = get_regs(fbi->id);
	xres = var->xres; yres = var->yres;
	xres_z = var->xres; yres_z = var->yres;
	xres_virtual = var->xres_virtual;
	bits_per_pixel = var->bits_per_pixel;

	/* xres_z = total - left - right */
	xres_z = xres_z - (left << 1);
	/* yres_z = yres_z - top - bottom */
	yres_z = yres_z - (top << 1);

	if (fbi->new_addr[0]) {
		xres = surface->viewPortInfo.srcWidth;
		yres = surface->viewPortInfo.srcHeight;
		var->xres_virtual = surface->viewPortInfo.srcWidth;
		var->yres_virtual = surface->viewPortInfo.srcHeight * 2;
		xres_virtual = surface->viewPortInfo.srcWidth;

		xres_z = surface->viewPortInfo.zoomXSize;
		yres_z = surface->viewPortInfo.zoomYSize;

		left = surface->viewPortOffset.xOffset;
		top = surface->viewPortOffset.yOffset;
		pr_debug("surface: xres %d xres_z %d"
			" yres %d yres_z %d\n left %d top %d\n",
			xres, xres_z, yres, yres_z, left, top);
	}

	dev_dbg(fbi->fb_info->dev, "adjust: xres %d xres_z %d"
		" yres %d yres_z %d\n left %d top %d\n",
		xres, xres_z, yres, yres_z, left, top);

	if (((fbi->pix_fmt & ~0x1000) >> 1) < 6) {
		pitch[0] = pitch[0] ? pitch[0] : (xres_virtual *
				 bits_per_pixel >> 3);
		pitch[1] = pitch[2] = 0;
	} else {
		pitch[0] = pitch[0] ? pitch[0] : xres;
		pitch[1] = pitch[1] ? pitch[1] : xres >> 1;
		pitch[2] = pitch[2] ? pitch[2] : xres >> 1;
	}
	if (fbi->vid) {
		/* start address on screen */
		writel((top << 16) | left, &regs->v_start);
		/* pitch, pixels per line */
		writel(pitch[0] & 0xFFFF, &regs->v_pitch_yc);
		writel(pitch[2] << 16 | pitch[1], &regs->v_pitch_uv);
		/* resolution, src size */
		writel((yres << 16) | xres, &regs->v_size);
		/* resolution, dst size */
		writel((yres_z << 16) | xres_z, &regs->v_size_z);

		/* enable two-level zoom down if the ratio exceed 2 */
		if (xres_z && var->bits_per_pixel) {
			int shift = (fbi->id == 1) ? 22 : 20;
			u32 reg = (fbi->id == 2) ? LCD_PN2_LAYER_ALPHA_SEL1 :\
				 LCD_AFA_ALL2ONE;

			x = readl(fbi->reg_base + reg);
			if (!(var->xres & 0x1) && ((var->xres >> 1) >= xres_z))
				x |= 1 << shift;
			else
				x &= ~(1 << shift);
			writel(x, fbi->reg_base + reg);
		}
	} else {
		/* start address on screen */
		writel((top << 16) | left, &regs->g_start);
		/* pitch, pixels per line */
		writel(pitch[0] & 0xFFFF, &regs->g_pitch);
		/* resolution, src size */
		writel((yres << 16) | xres, &regs->g_size);
		/* resolution, dst size */
		writel((yres_z << 16) | xres_z, &regs->g_size_z);
	}
}
