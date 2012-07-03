/*
 *  linux/drivers/video/pxa95xfb_ioctls.c
 */

#include "pxa95xfb.h"

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

/* buffer management:
 *    freelist: list return to upper layer which indicates buff is free
 *    waitlist: wait queue which indicates "using" buffer, waitlist[0] is write in dma desc
 *    current: buffer on panel
 * Operation:
 *    flip: if !waitlist[0] set buf; enqueue buf to waitlist
 *    get freelist: return freelist
 *    eof intr: enqueue current to freelist; dequeue waitlist[0] to current; if !new waitlist[0] set buf;
 *    buffers are protected by mutex: ?? how to protect in intr?
 * suspend:
 *    when suspend & get freelist, simulate eof intr one time
 */

static int convert_pix_fmt(u32 vmode)
{
	switch (vmode) {
		case FB_VMODE_YUV422PACKED:
			return PIX_FMTIN_YUV422IL;
		case FB_VMODE_YUV422PLANAR:
			return PIX_FMTIN_YUV422;
		case FB_VMODE_YUV420PLANAR:
			return PIX_FMTIN_YUV420;
			/* TODO - add U/V and R/B SWAP format and YUV444 */
		case FB_VMODE_RGB565:
			return PIX_FMTIN_RGB_16;
		case FB_VMODE_RGB888PACK:
			return PIX_FMTIN_RGB_24_PACK;
		case FB_VMODE_RGBA888:
			return PIX_FMTIN_RGB_32;
		case FB_VMODE_RGB888UNPACK:
			return PIX_FMTIN_RGB_24;
		default:
			printk(KERN_INFO "unsupported format!\n");
			return -1;
	}
}

/* pxa95x lcd controller could only resize in step. Adjust offset in such case to keep picture in center */
static void adjust_offset_for_resize(struct _sViewPortInfo *info, struct _sViewPortOffset *offset)
{
	int wstep, hstep;

	if ((info->zoomXSize == info->srcWidth && info->zoomYSize == info->srcHeight)
		|| !info->zoomXSize || !info->zoomYSize)
		return;

	wstep = (info->zoomXSize > info->srcWidth) ? (info->srcWidth /8) : (info->srcWidth /32);
	hstep = (info->zoomYSize > info->srcHeight) ? (info->srcHeight /8) : (info->srcHeight /32);

	offset->xOffset += (info->zoomXSize % wstep) / 2;
	offset->yOffset += (info->zoomYSize % hstep) / 2;
	info->zoomXSize = (info->zoomXSize / wstep) * wstep;
	info->zoomYSize = (info->zoomYSize / hstep) * hstep;
}

static int surface_is_changed(struct pxa95xfb_info *fbi,
		FBVideoMode new_mode,
		struct _sViewPortInfo *new_info,
		struct _sViewPortOffset *new_offset)
{
	if (new_info && new_offset)
		adjust_offset_for_resize(new_info, new_offset);

	/* check mode
	 * check view port settings.
	 * not check ycpitch as it's 0 sometimes from overlay-hal
	 * Check offset */
	return (new_mode >= 0 && fbi->surface.videoMode != new_mode)
			|| (new_info &&
			(fbi->surface.viewPortInfo.srcWidth != new_info->srcWidth ||
			 fbi->surface.viewPortInfo.srcHeight != new_info->srcHeight ||
			 fbi->surface.viewPortInfo.zoomXSize != new_info->zoomXSize ||
			 fbi->surface.viewPortInfo.zoomYSize != new_info->zoomYSize))
			|| (new_offset &&
			(fbi->surface.viewPortOffset.xOffset != new_offset->xOffset ||
			 fbi->surface.viewPortOffset.yOffset != new_offset->yOffset));
}

static int surface_is_valid(struct pxa95xfb_info *fbi,
		FBVideoMode new_mode,
		struct _sViewPortInfo *new_info,
		struct _sViewPortOffset *new_offset)
{
	return (new_mode >= 0 && convert_pix_fmt(new_mode) >= 0)
		|| (new_info &&
		(new_info->srcWidth || new_info->srcHeight ||
		new_info->zoomXSize || new_info->zoomYSize));
}


static void set_surface(struct pxa95xfb_info *fbi,
		FBVideoMode new_mode,
		struct _sViewPortInfo *new_info,
		struct _sViewPortOffset *new_offset)
{
	struct fb_var_screeninfo *var = &fbi->fb_info->var;

	if (new_info && new_offset)
		adjust_offset_for_resize(new_info, new_offset);


	/* check mode */
	if (new_mode >= 0) {
		printk(KERN_INFO "vmode=%d\n", new_mode);
		fbi->surface.videoMode = new_mode;
		fbi->pix_fmt = convert_pix_fmt(new_mode);
		lcdc_set_pix_fmt(var, fbi->pix_fmt);
	}

	/* check view port settings.*/
	if (new_info) {
		memcpy(&fbi->surface.viewPortInfo, new_info, sizeof(struct _sViewPortInfo));
		if (!new_info->yPitch)
			fbi->surface.viewPortInfo.yPitch = new_info->srcWidth*pix_fmt_to_bpp(fbi->pix_fmt);
		printk(KERN_INFO "Ovly update: [%d %d] - [%d %d], ycpitch %d\n",
			new_info->srcWidth, new_info->srcHeight,
			new_info->zoomXSize, new_info->zoomYSize,
			fbi->surface.viewPortInfo.yPitch);
	}

	/* Check offset */
	if (new_offset) {
		memcpy(&fbi->surface.viewPortOffset, new_offset, sizeof(struct _sViewPortOffset));
		printk(KERN_INFO "Ovly update offset [%d %d]\n",
			new_offset->xOffset, new_offset->yOffset);
	}
}

static inline void buf_print(struct pxa95xfb_info *fbi)
{
	int i;
	pr_info("Curent buff: %x \n", fbi->buf_current.y);

	pr_info("buf_waitlist:");
	for (i = 0; i < MAX_QUEUE_NUM; i++) {
		if (fbi->buf_waitlist[i].y)
			pr_info("<%d: %x> ", i, fbi->buf_waitlist[i].y);
	}
	pr_info("\n");

	pr_info("buf_freelist:");
	for (i = 0; i < MAX_QUEUE_NUM; i++) {
		if (fbi->buf_freelist[i].y)
			pr_info("<%d: %x> ", i, fbi->buf_freelist[i].y);
	}
	pr_info("\n");
}

static void buf_dequeue(struct buf_addr *list, struct buf_addr *buf)
{
	int i;

	if (!list){
		printk(KERN_ALERT "%s: invalid list\n", __func__);
		return;
	}

	memcpy(buf, list, sizeof(struct buf_addr));

	for (i = 1; i < MAX_QUEUE_NUM; i++) {
		if (!list[i].y) {
			memset(&list[i-1], 0, sizeof(struct buf_addr));
			break;
		}
		memcpy(&list[i-1], &list[i], sizeof(struct buf_addr));
		/*printk(KERN_INFO "%s: move buff %x from list[%d] to list[%d]\n", __func__, list[i].y, i, i-1);*/
	}

	if (i >= MAX_QUEUE_NUM)
		printk(KERN_ALERT "%s: buffer overflow\n",  __func__);

	/*printk(KERN_INFO "%s: dequeue: %x\n", __func__, buf->y);*/
}

static int buf_enqueue(struct buf_addr *list, struct buf_addr *buf)
{
	int i;

	if (!list){
		printk(KERN_ALERT "%s: invalid list\n", __func__);
		return -1;
	}

	for (i = 0; i < MAX_QUEUE_NUM; i++) {
		if (!list[i].y) {
			memcpy(&list[i], buf, sizeof(struct buf_addr));
			/*printk(KERN_INFO "%s: add buff %x to list[%d]\n", __func__, buf->y, i);*/
			return 0;
		}

		if (list[i].y == buf->y) {
			/* already in list, free this request. */
			/*printk(KERN_WARNING "%s: buff %x is same as list[%d]\n",
				__func__, buf->y, i);*/
			return 0;
		}
	}

	if (i >= MAX_QUEUE_NUM)
		printk(KERN_ALERT "%s: buffer overflow\n",  __func__);

	return -2;
}

static void buf_clear(struct buf_addr *list)
{
	/* Check null pointer. */
	if (list)
		memset(list, 0, MAX_QUEUE_NUM * sizeof(struct buf_addr));
}

/* fake endframe when suspend or overlay off */
static void buf_fake_endframe(void * p)
{
	struct pxa95xfb_info *fbi = p;
	struct buf_addr t;
	buf_dequeue(fbi->buf_waitlist, &t);
	while (t.y) {
		/*pr_info("%s: move %x to current, move %x to freelist\n",
			__func__, t.y, fbi->buf_current.y);*/
		/*enqueue current to freelist*/
		buf_enqueue(fbi->buf_freelist, &fbi->buf_current);
		/*dequeue waitlist[0] to current*/
		memcpy(&fbi->buf_current, &t, sizeof(struct buf_addr));
		buf_dequeue(fbi->buf_waitlist, &t);
	}

}

void lcdc_vid_buf_endframe(void * p)
{
	struct pxa95xfb_info *fbi = p;
	struct buf_addr t;
	buf_dequeue(fbi->buf_waitlist, &t);
	if (t.y) {
		/*printk(KERN_INFO "%s: move %x to current, move %x to freelist\n",
			__func__, t.y, fbi->buf_current.y);*/
		/*enqueue current to freelist*/
		buf_enqueue(fbi->buf_freelist, &fbi->buf_current);
		/*dequeue waitlist[0] to current*/
		memcpy(&fbi->buf_current, &t, sizeof(struct buf_addr));
	}

	/*if new waitlist[0] set buf*/
	if (fbi->buf_waitlist[0].y) {
		/*printk(KERN_INFO "%s: flip buf %x on\n", __func__, fbi->buf_waitlist.y);*/
		fbi->user_addr = fbi->buf_waitlist[0].y;
		lcdc_set_fr_addr(fbi);
	}
}

void lcdc_vid_clean(struct pxa95xfb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	unsigned long flags;

	fbi->user_addr = 0;
	fbi->surface.videoMode = -1;
	fbi->surface.viewPortInfo.srcWidth = var->xres;
	fbi->surface.viewPortInfo.yPitch = var->xres * pix_fmt_to_bpp(fbi->pix_fmt);
	fbi->surface.viewPortInfo.srcHeight = var->yres;
	spin_lock_irqsave(&fbi->buf_lock, flags);
	/* clear buffer list. */
	buf_clear(fbi->buf_freelist);
	buf_clear(fbi->buf_waitlist);
	memset(&fbi->buf_current, 0, sizeof(struct buf_addr));
	spin_unlock_irqrestore(&fbi->buf_lock, flags);
}

static void do_flip_baselay(struct pxa95xfb_info *fbi, struct _sOvlySurface *surface)
{
	struct buf_addr *start_addr = (struct buf_addr *)(surface->videoBufferAddr.startAddr);

	/* if controller is not switched on/really turn on, return */
	if (!fbi->on && !fbi->controller_on )
		return;

	fbi->user_addr = start_addr->y;
	lcdc_set_fr_addr(fbi);

	if (fbi->on && !fbi->controller_on) {
		printk(KERN_INFO "really turn on fb%d\n", fbi->id);
		set_surface(fbi, surface->videoMode,
					&surface->viewPortInfo,
					&surface->viewPortOffset);
		lcdc_set_lcd_controller(fbi);
		if (!conv_is_on(fbi))
			converter_onoff(fbi, 1);
		conv_ref_inc(fbi);
		fbi->controller_on = 1;
		return;
	}

	if (surface_is_changed(fbi, surface->videoMode,
				&surface->viewPortInfo,
				&surface->viewPortOffset)) {
		/* in this case, surface mode changed, need sync */
		set_surface(fbi, surface->videoMode,
				&surface->viewPortInfo,
				&surface->viewPortOffset);
		lcdc_set_lcd_controller(fbi);
	}
	return;
}

static void do_flip_overlay(struct pxa95xfb_info *fbi, struct _sOvlySurface *surface)
{
	unsigned long flags;
	struct buf_addr *start_addr = (struct buf_addr *)(surface->videoBufferAddr.startAddr);

	if (fbi->on && !fbi->controller_on) {
		printk(KERN_INFO "really turn on fb%d\n", fbi->id);
		set_surface(fbi, surface->videoMode,
					&surface->viewPortInfo,
					&surface->viewPortOffset);
		WARN_ON(fbi->buf_waitlist[0].y);
		spin_lock_irqsave(&fbi->buf_lock, flags);
		/*if !waitlist[0] enqueue buf to waitlist*/
		/*printk(KERN_INFO "%s: flip %x on\n",
		__func__, (u32)start_addr);*/
		fbi->user_addr = start_addr->y;
		lcdc_set_fr_addr(fbi);
		buf_enqueue(fbi->buf_waitlist, start_addr);
		buf_fake_endframe(fbi);
		spin_unlock_irqrestore(&fbi->buf_lock, flags);

		lcdc_set_lcd_controller(fbi);
		if (!conv_is_on(fbi))
			converter_onoff(fbi, 1);

		conv_ref_inc(fbi);
		fbi->controller_on = 1;
		return;
	}

	/* Fix the first green frames when camera preview */
	if( !fbi->controller_on ) {
		buf_enqueue(fbi->buf_freelist, start_addr);
		return;
	}

	if (surface_is_changed(fbi, surface->videoMode,
				&surface->viewPortInfo,
				&surface->viewPortOffset)) {
		/* in this case, surface mode changed, need sync */
		set_surface(fbi, surface->videoMode,
				&surface->viewPortInfo,
				&surface->viewPortOffset);
		spin_lock_irqsave(&fbi->buf_lock, flags);
		fbi->user_addr = start_addr->y;
		lcdc_set_fr_addr(fbi);
		buf_enqueue(fbi->buf_waitlist, start_addr);
		buf_fake_endframe(fbi);
		spin_unlock_irqrestore(&fbi->buf_lock, flags);
		lcdc_set_lcd_controller(fbi);
	} else {
		spin_lock_irqsave(&fbi->buf_lock, flags);
		/*if !waitlist[0] enqueue buf to waitlist*/
		if (!fbi->buf_waitlist[0].y) {
			/*printk(KERN_INFO "%s: flip %x on\n",
			__func__, start_addr->y);*/
			fbi->user_addr = start_addr->y;
			lcdc_set_fr_addr(fbi);
		}
		buf_enqueue(fbi->buf_waitlist, start_addr);
		spin_unlock_irqrestore(&fbi->buf_lock, flags);
	}
	return;
}

int pxa95xfb_ioctl(struct fb_info *fi, unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;
	static struct _sOvlySurface gOvlySurface;
	int on;
	unsigned long flags;

	switch (cmd) {
	case FB_IOCTL_WAIT_VSYNC_ON:
		fbi->vsync_en = 1;
		break;
	case FB_IOCTL_WAIT_VSYNC_OFF:
		fbi->vsync_en = 0;
		break;
	case FB_IOCTL_WAIT_VSYNC:
		if (arg > 0x3)
			printk(KERN_ERR "WAIT_VSYNC: invalid para %d\n", (u32)arg);
		else if (arg == 0)
			lcdc_wait_for_vsync(fbi->converter, 0);
		else if (arg== 1)
			lcdc_wait_for_vsync(pxa95xfbi[0]->converter, 0);
		else if (arg == 2)
			lcdc_wait_for_vsync(pxa95xfbi[2]->converter, 0);
		else if (arg == 3)
			lcdc_wait_for_vsync(pxa95xfbi[0]->converter, pxa95xfbi[2]->converter);
		break;
	case FB_IOCTL_GET_VIEWPORT_INFO:
		return copy_to_user(argp, &gOvlySurface.viewPortInfo,
				sizeof(struct _sViewPortInfo)) ? -EFAULT : 0;
	case FB_IOCTL_SET_VIEWPORT_INFO:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gOvlySurface.viewPortInfo, argp,
				sizeof(gOvlySurface.viewPortInfo))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_SET_VIDEO_MODE:
		/*Get data from user space.
		 *return error for not supported format
		 */
		if (copy_from_user(&gOvlySurface.videoMode,
				argp, sizeof(gOvlySurface.videoMode))
			|| convert_pix_fmt(gOvlySurface.videoMode) < 0)
			return -EFAULT;
		break;
	case FB_IOCTL_GET_VIDEO_MODE:
		return copy_to_user(argp, &gOvlySurface.videoMode,
				sizeof(u32)) ? -EFAULT : 0;
	case FB_IOCTL_SET_VID_OFFSET:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gOvlySurface.viewPortOffset,
			argp,
			sizeof(gOvlySurface.viewPortOffset))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_GET_VID_OFFSET:
		return copy_to_user(argp,
			&gOvlySurface.viewPortOffset,
			sizeof(struct _sViewPortOffset)) ? -EFAULT : 0;
	case FB_IOCTL_GET_SURFACE:
		return copy_to_user(argp, &gOvlySurface,
				sizeof(struct _sOvlySurface)) ? -EFAULT : 0;
	case FB_IOCTL_SET_SURFACE:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gOvlySurface, argp,
					sizeof(struct _sOvlySurface))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_FLIP_VID_BUFFER:
	{
		struct _sOvlySurface surface;

		mutex_lock(&fbi->access_ok);
		/* Get user-mode data. */
		if (copy_from_user(&surface, argp,
					sizeof(struct _sOvlySurface))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		if (!surface_is_valid(fbi, surface.videoMode,
					&surface.viewPortInfo,
					&surface.viewPortOffset)) {
			printk(KERN_INFO "surface is not valid in fb%d\n", fbi->id);
			mutex_unlock(&fbi->access_ok);
			return -EINVAL;
		}

		if (fb_is_baselay(fbi))
			do_flip_baselay(fbi, &surface);
		else
			do_flip_overlay(fbi, &surface);

		mutex_unlock(&fbi->access_ok);
		return 0;
	}
	case FB_IOCTL_GET_FREELIST:
		if (fb_is_baselay(fbi)) {
			if (copy_to_user(argp, fbi->buf_freelist,
				MAX_QUEUE_NUM * sizeof(struct buf_addr)))
				return -EFAULT;
			return 0;
		}

		spin_lock_irqsave(&fbi->buf_lock, flags);

		/* safe check: when lcd is suspend,
		 * move all buffers as "switched"*/
		if (!fbi->on)
			buf_fake_endframe(fbi);

		if (copy_to_user(argp, fbi->buf_freelist,
				MAX_QUEUE_NUM * sizeof(struct buf_addr))) {
			spin_unlock_irqrestore(&fbi->buf_lock, flags);
			return -EFAULT;
		}
		buf_clear(fbi->buf_freelist);
		spin_unlock_irqrestore(&fbi->buf_lock, flags);
		return 0;
	case FB_IOCTL_GET_BUFF_ADDR:
		return copy_to_user(argp, &fbi->surface.videoBufferAddr,
				sizeof(struct _sVideoBufferAddr)) ? -EFAULT : 0;
	case FB_IOCTL_SET_MEMORY_TOGGLE:
		break;
	case FB_IOCTL_GET_COLORKEYnALPHA:
		if (copy_to_user(argp, &fbi->ckey_alpha,
					sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;
		break;
	case FB_IOCTL_SET_COLORKEYnALPHA:
	{
		struct _sColorKeyNAlpha *ckey_alpha;
		if (copy_from_user(&fbi->ckey_alpha, argp,
					sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;
		ckey_alpha = &fbi->ckey_alpha;

		if (ckey_alpha->alphapath == FB_VID_PATH_ALPHA)
			fbi = overlay_in_same_path(fbi);
		else
			fbi = baselay_in_same_path(fbi);

		if (ckey_alpha->mode == FB_ENABLE_RGB_COLORKEY_MODE) {
			fbi->alphacolor = (ckey_alpha->Y_ColorAlpha << 16) |
				(ckey_alpha->U_ColorAlpha << 8) | ckey_alpha->V_ColorAlpha;
			fbi->alphamode = LCD_COLORKEY;
		} else if (ckey_alpha->mode == FB_DISABLE_COLORKEY_MODE) {
			if (ckey_alpha->alphapath == FB_CONFIG_ALPHA) {
				fbi->alphamode = LCD_WINALPHA;
				fbi->alphacolor = ckey_alpha->config;
			} else
				fbi->alphamode = LCD_PIXELALPHA;
		} else {
			pr_info("fb%d: unsupported mode\n", fbi->id);
			fbi->alphamode = LCD_ALPHA_INVALID;
		}

		if (fbi->controller_on)
			lcdc_set_lcd_controller(fbi);

		break;
	}
	case FB_IOCTL_SWITCH_VID_OVLY:
		/* fbi->id: 0,1 -> 1, 2,3 -> 3*/
	case FB_IOCTL_SWITCH_GRA_OVLY:
		/* fbi->id: 0,1 -> 0, 2,3 -> 2*/
		if (cmd == FB_IOCTL_SWITCH_VID_OVLY)
			fbi = overlay_in_same_path(fbi);
		else
			fbi = baselay_in_same_path(fbi);

		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&on, argp, sizeof(int))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		if (on != fbi->on) {
			fbi->on = on;
			printk(KERN_INFO "PXA95xfb%d: switch: %s\n",
				fbi->id, fbi->on ? "on" : "off");
			if (!fbi->on && fbi->controller_on) {
				conv_ref_dec(fbi);
				lcdc_set_lcd_controller(fbi);
				if (!conv_is_on(fbi))
					converter_onoff(fbi, 0);

				fbi->controller_on = 0;
				/* tricky workaround for id = 2/3 which shares same channel */
				if (fb_is_tv(fbi))
					fb_in_same_path(fbi)->controller_on = 0;
				fbi->user_addr = 0;
				lcdc_set_fr_addr(fbi);
			}
		}
		mutex_unlock(&fbi->access_ok);
		break;
	default:
		break;
	}

	return 0;
}
