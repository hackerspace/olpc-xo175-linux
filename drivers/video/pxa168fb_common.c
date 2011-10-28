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

static int check_yuv_status(FBVideoMode videoMode, struct pxa168fb_info *fbi,
		int video_layer)
{
	u32 x;

	if (video_layer) {     /* now in video layer */
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
		viewPortInfo, FBVideoMode videoMode, int video_layer)
{
	if (check_yuv_status(videoMode, fbi, video_layer) < 0)
		return 1;

	if ((viewPortInfo.rotation == 0) || (viewPortInfo.rotation == 1)) {
		if (!video_layer) {
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

void clear_buffer(struct pxa168fb_info *fbi, int video_layer)
{
	unsigned long flags;

	spin_lock_irqsave(&fbi->buf_lock, flags);
	clearFilterBuf(fbi->filterBufList, RESET_BUF);
	buf_clear(&fbi->buf_freelist, FREE_ENTRY);
	buf_clear(&fbi->buf_waitlist, FREE_ENTRY);
	kfree(fbi->buf_current);
	fbi->buf_current = 0;
	kfree(fbi->buf_retired);
	fbi->buf_retired = 0;
	fbi->surface_set = 0;
	if (video_layer)
		ovly_info.wait_peer = 0;
	else
		gfx_info.wait_peer = 0;
	spin_unlock_irqrestore(&fbi->buf_lock, flags);
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

void pxa168fb_update_addr(struct pxa168fb_info *fbi,
	struct _sVideoBufferAddr *new_addr, int video_layer)
{
	fbi->new_addr[0] = (unsigned long)new_addr->startAddr[0];
	fbi->new_addr[1] = (unsigned long)new_addr->startAddr[1];
	fbi->new_addr[2] = (unsigned long)new_addr->startAddr[2];

	if (FB_MODE_DUP) {
		struct pxa168fb_info *fbi_dual;
		if (video_layer)             /* video layer */
			fbi_dual = ovly_info.fbi[fb_dual];
		else
			fbi_dual = gfx_info.fbi[fb_dual];

		fbi_dual->new_addr[0] = fbi->new_addr[0];
		fbi_dual->new_addr[1] = fbi->new_addr[1];
		fbi_dual->new_addr[2] = fbi->new_addr[2];
	}
}

int check_surface(struct fb_info *fi,
			FBVideoMode new_mode,
			struct _sViewPortInfo *new_info,
			struct _sViewPortOffset *new_offset,
			struct _sVideoBufferAddr *new_addr,
			int video_layer)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &fi->var;
	struct pxa168fb_info *fbi_dual;
	int changed = 0;

	dev_dbg(fi->dev, "Enter %s\n", __func__);

	if (video_layer)
		fbi_dual = ovly_info.fbi[fb_dual];
	else
		fbi_dual = gfx_info.fbi[fb_dual];

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

		pxa168fb_update_addr(fbi, new_addr, video_layer);
		changed = 1;
	}

	return changed;
}

int check_surface_addr(struct fb_info *fi,
			struct _sOvlySurface *surface, int video_layer)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct _sVideoBufferAddr *new_addr = &surface->videoBufferAddr;
	int changed = 0;

	dev_dbg(fi->dev, "Enter %s\n", __func__);

	/* Check buffer address */
	if (new_addr && new_addr->startAddr[0] &&
	    fbi->new_addr[0] != (unsigned long)new_addr->startAddr[0]) {
		pxa168fb_update_addr(fbi, new_addr, video_layer);
		changed = 1;
	}

	return changed;
}

#ifdef OVLY_TASKLET
void pxa168fb_ovly_task(unsigned long data)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)data;
#else
void pxa168fb_ovly_work(struct work_struct *w)
{
	struct pxa168fb_info *fbi = container_of(w,
		struct pxa168fb_info, buf_work);
#endif
	unsigned long flags;

	if (fbi->debug == 1)
		printk(KERN_DEBUG"%s fbi %d buf_retired %p\n",
			__func__, fbi->id, fbi->buf_retired);

	if (fbi->buf_retired) {
		/* enqueue current to freelist */
		spin_lock_irqsave(&fbi->buf_lock, flags);
		list_add_tail(&fbi->buf_retired->surfacelist,
			&fbi->buf_freelist.surfacelist);
		fbi->buf_retired = 0;
		ovly_info.wait_peer = 0;
		gfx_info.wait_peer = 0;
		spin_unlock_irqrestore(&fbi->buf_lock, flags);
	}
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
		pr_info("%s\n", __func__);

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
				pr_info("%s: buffer "
					"%p will be returned\n", __func__,
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

void buf_endframe(void *point, int video_layer)
{
	struct fb_info *fi = (struct fb_info *)point;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct _sOvlySurface *pOvlySurface;
	int ret;
	struct _sSurfaceList *surface_list = 0;

	if (fbi->buf_retired) {
		if (video_layer)
			ovly_info.retire_err++;
		else
			gfx_info.retire_err++;
	} else {
		if (list_empty(&fbi->buf_waitlist.surfacelist))
			return;

		surface_list = list_first_entry(&fbi->buf_waitlist.surfacelist,
			struct _sSurfaceList, surfacelist);
		pOvlySurface = &surface_list->surface;
		list_del(&surface_list->surfacelist);

		/* Update new surface settings */
		ret = fbi->update_buff(fi, pOvlySurface, 0);

		if (!ret) {
			fbi->buf_retired = fbi->buf_current;
			fbi->buf_current = surface_list;
			if (WAIT_PEER && FB_MODE_DUP) {
				if (video_layer)
					ovly_info.wait_peer = 1;
				else
					gfx_info.wait_peer = 1;
			} else {
#ifdef OVLY_TASKLET
				tasklet_schedule(&fbi->ovly_task);
#else
				if (fbi->system_work)
					schedule_work(&fbi->buf_work);
				else
					queue_work(fbi->work_q,
						&fbi->fb_info->queue);
#endif
			}
		} else {
			/* enqueue the repeated buffer to freelist */
			list_add_tail(&surface_list->surfacelist,
				&fbi->buf_freelist.surfacelist);
			pr_info("Detect a same surface flipped in, "
				"may flicker.\n");
		}
	}
}

static int get_list_count(struct _sSurfaceList *srflist)
{
	struct list_head *pos, *n;
	int count = 0;

	list_for_each_safe(pos, n, &srflist->surfacelist)
		count++;
	return count;
}

int flip_buffer(struct fb_info *info, unsigned long arg,
		int video_layer)
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
	if (unsupport_format(fbi, surface->viewPortInfo, surface->videoMode,
				video_layer)) {
		kfree(surface_list);
		mutex_unlock(&fbi->access_ok);
		return -EFAULT;
	}

	update_surface(surface);

	length = surface->videoBufferAddr.length;
	start_addr[0] = surface->videoBufferAddr.startAddr[0];
	input_data = surface->videoBufferAddr.inputData;

	if (fbi->debug == 1)
		printk(KERN_DEBUG"flip surface %p buf %p\n",
				surface, start_addr[0]);
	/*
	 * Has DMA addr?
	 */
	if (start_addr[0] && (!input_data)) {
		spin_lock_irqsave(&fbi->buf_lock, flags);
		if (get_list_count(&fbi->buf_waitlist) >= 2) {
			/*if there are more than two frames in waitlist, dequeue
			*the older frame and enqueue it to freelist,
			*then enqueue this frame to waitlist*/
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
			if (NEED_VSYNC(fbi, video_layer))
				wait_for_vsync(fbi, video_layer);
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
	unsigned long flags;

	spin_lock_irqsave(&fbi->buf_lock, flags);

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

	if (fbi->buf_retired) {
		list_add_tail(&fbi->buf_retired->surfacelist,
			&fbi->buf_freelist.surfacelist);
		fbi->buf_retired = 0;
	}

	/* clear some globals */
	ovly_info.wait_peer = 0;
	spin_unlock_irqrestore(&fbi->buf_lock, flags);

	memset(&fbi->surface, 0, sizeof(struct _sOvlySurface));
	fbi->new_addr[0] = 0; fbi->new_addr[1] = 0; fbi->new_addr[2] = 0;
	if (FB_MODE_DUP) {
		memset(&ovly_info.fbi[fb_dual]->surface, 0,
			sizeof(struct _sOvlySurface));
		ovly_info.fbi[fb_dual]->new_addr[0] = 0;
		ovly_info.fbi[fb_dual]->new_addr[1] = 0;
		ovly_info.fbi[fb_dual]->new_addr[2] = 0;
	}
}

int get_freelist(struct fb_info *info, unsigned long arg,
		int video_layer)
{
	void __user *argp = (void __user *)arg;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)info->par;
	unsigned long flags;

	if (fbi->debug == 1)
		printk(KERN_DEBUG"get freelist\n");

	spin_lock_irqsave(&fbi->buf_lock, flags);

	/* when lcd is suspend, move all buffers as "switched"*/
	if (!(gfx_info.fbi[fbi->id]->active))
		buf_endframe(info, video_layer);
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
		printk(KERN_DEBUG"get freelist end\n");

	return 0;
}

void set_dma_active(struct pxa168fb_info *fbi, int video_layer)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	u32 flag = video_layer ? CFG_DMA_ENA_MASK : CFG_GRA_ENA_MASK;
	u32 enable = video_layer ? CFG_DMA_ENA(1) : CFG_GRA_ENA(1);
	u32 active = 0;

	if (fbi->new_addr[0] || mi->mmap || (!video_layer &&
		 (fb_mode || fb_share) && fbi->id == fb_dual))
		active = fbi->check_modex_active(fbi->id, fbi->active
					&& fbi->dma_on);

	dma_ctrl_set(fbi->id, 0, flag, active ? enable : 0);

	pr_debug("%s fbi %d: active %d fbi->active %d, new_addr %lu\n",
		__func__, fbi->id, active, fbi->active, fbi->new_addr[0]);
}

int dispd_dma_enabled(struct pxa168fb_info *fbi, int video_layer)
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
	return dma_ctrl_read(fbi->id, 0) & (video_layer ?
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

void wait_for_vsync(struct pxa168fb_info *fbi, int video_layer)
{
	struct pxa168fb_info *fbi_dual =
		video_layer ? ovly_info.fbi[fb_dual] : gfx_info.fbi[fb_dual];

again:
	atomic_set(&fbi->w_intr, 0);
	pr_debug("fbi->id %d layer: %d\n", fbi->id, video_layer);

	wait_event_interruptible_timeout(fbi->w_intr_wq,
		atomic_read(&fbi->w_intr), HZ/20);

	/* handle timeout case, to w/a irq miss */
	if (atomic_read(&fbi->w_intr) == 0)
		clear_dispd_irq(fbi);

	if (FB_MODE_DUP && NEED_VSYNC(fbi_dual, video_layer)) {
		fbi = fbi_dual;
		goto again;
	}
}

void pxa168fb_list_init(struct pxa168fb_info *fbi)
{
	INIT_LIST_HEAD(&fbi->buf_freelist.surfacelist);
	INIT_LIST_HEAD(&fbi->buf_waitlist.surfacelist);
	fbi->buf_retired = fbi->buf_current = 0;
}
