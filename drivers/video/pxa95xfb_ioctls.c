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
		case FB_VMODE_YUV420PLANAR_SWAPUV:
			return PIX_FMTIN_YVU420;
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

static int surface_scaled(struct _sOvlySurface *surface)
{
	struct _sViewPortInfo *info = &surface->viewPortInfo;

	return (info->srcWidth != info->zoomXSize ||
		info->srcHeight != info->zoomYSize);
}

static void surface_print(struct _sOvlySurface *surface)
{
	pr_info("surface: [%d %d] - [%d %d], ycpitch %d\n",
		surface->viewPortInfo.srcWidth,
		surface->viewPortInfo.srcHeight,
		surface->viewPortInfo.zoomXSize,
		surface->viewPortInfo.zoomYSize,
		surface->viewPortInfo.yPitch);
	pr_info("surface offset [%d %d]\n",
		surface->viewPortOffset.xOffset,
		surface->viewPortOffset.yOffset);
	pr_info("surface videomode: %x\n",
		surface->videoMode);
	pr_info("surface addr: %x %x %x\n",
		(int)surface->videoBufferAddr.startAddr[0],
		(int)surface->videoBufferAddr.startAddr[1],
		(int)surface->videoBufferAddr.startAddr[2]);
}

static int surface_valid(struct pxa95xfb_info *fbi, struct _sOvlySurface *surface)
{
	if (convert_pix_fmt(surface->videoMode) < 0
		|| surface->viewPortInfo.srcWidth < 6
		|| surface->viewPortInfo.srcHeight < 2
		|| surface->viewPortInfo.zoomXSize <= 0
		|| surface->viewPortInfo.zoomYSize <= 0
		|| (fbi->window != 4 && surface_scaled(surface))
		|| surface->viewPortInfo.srcWidth * 4 < surface->viewPortInfo.zoomXSize
		|| surface->viewPortInfo.srcHeight * 4 < surface->viewPortInfo.zoomYSize
		|| surface->viewPortInfo.srcWidth > surface->viewPortInfo.zoomXSize * 4
		|| surface->viewPortInfo.srcHeight > surface->viewPortInfo.zoomYSize * 4) {
		pr_info("surface is not valid in fb%d\n", fbi->id);
		surface_print(surface);
		return 0;
	}

	return 1;
}

/* pxa95x lcd controller could only resize in step. Adjust offset in such case to keep picture in center */
static void surface_adjust(struct _sOvlySurface *surface)
{
	struct _sViewPortInfo *info = &surface->viewPortInfo;
	int wstep, hstep, wsrc;
	int bpp = pix_fmt_to_bpp(convert_pix_fmt(surface->videoMode));

	/* fill ypitch if not set */
	if (!info->yPitch)
		info->yPitch = info->srcWidth * bpp;

	/* adjust offset with step */
	if (!surface_scaled(surface))
		return;

	wsrc = info->yPitch / bpp;
	wstep = (info->zoomXSize > wsrc) ? (wsrc /8) : (wsrc /32);
	hstep = (info->zoomYSize > info->srcHeight) ? (info->srcHeight /8) : (info->srcHeight /32);
	if (!wstep) wstep = 1;
	if (!hstep) hstep = 1;

	surface->viewPortOffset.xOffset += (info->zoomXSize % wstep) / 2;
	surface->viewPortOffset.yOffset += (info->zoomYSize % hstep) / 2;
	info->zoomXSize = (info->zoomXSize / wstep) * wstep;
	info->zoomYSize = (info->zoomYSize / hstep) * hstep;
}

#define SURFACE_CONF_SIZE (sizeof(FBVideoMode)\
		+ sizeof(struct _sViewPortInfo)\
		+ sizeof(struct _sViewPortOffset))

static int surface_changed(struct pxa95xfb_info *fbi,
		struct _sOvlySurface *surface)
{
	return (memcmp(&fbi->surface, surface, SURFACE_CONF_SIZE));
}

static void surface_apply(struct pxa95xfb_info *fbi,
		struct _sOvlySurface *surface)
{
	struct fb_var_screeninfo *var = &fbi->fb_info->var;

	surface_print(surface);
	memcpy(&fbi->surface, surface, SURFACE_CONF_SIZE);

	/* set color format */
	fbi->pix_fmt = convert_pix_fmt(fbi->surface.videoMode);
	lcdc_set_pix_fmt(var, fbi->pix_fmt);
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

int hdmi_conv_on = 0;
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
		surface_apply(fbi, surface);
		lcdc_set_lcd_controller(fbi);
		if (!conv_is_on(fbi))
			converter_onoff(fbi, 1);
		conv_ref_inc(fbi);
		fbi->controller_on = 1;
		if (fb_is_tv(fbi))
			hdmi_conv_on = 1;
		return;
	}

	if (surface_changed(fbi, surface)) {
		/* in this case, surface mode changed, need sync */
		surface_apply(fbi, surface);
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
		surface_changed(fbi, surface);
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
		if (fb_is_tv(fbi))
			hdmi_conv_on = 1;
		return;
	}

	/* Fix the first green frames when camera preview */
	if( !fbi->controller_on ) {
		buf_enqueue(fbi->buf_freelist, start_addr);
		return;
	}

	if (surface_changed(fbi, surface)) {
		/* in this case, surface mode changed, need sync */
		surface_apply(fbi, surface);
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

		if (!surface_valid(fbi, &surface)) {
			mutex_unlock(&fbi->access_ok);
			return -EINVAL;
		}

		surface_adjust(&surface);

		if (fbi->dump)
			dump_buffer(fbi, 0);

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
			if (fb_is_tv(fbi)) {
				if (0 == on)
					hdmi_conv_on = 0;
			}
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
		dev_info(fbi->dev, "ioctl: command %x un-supported\n", cmd);
		break;
	}

	return 0;
}
