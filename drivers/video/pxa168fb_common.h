#ifndef _PXA168FB_COMMON_
#define _PXA168FB_COMMON_

#define OVLY_TASKLET

#include <mach/pxa168fb.h>
#include "pxa168fb.h"

/* mirror mode related info/flags */
/* wait dual path's interrupt before put the buffer into freelist */
#define WAIT_PEER	(fb_mode && ovly_info.flag & 1)
/* wait dual path's interrupt once in ioctl of get free list */
#define WAIT_FREE	(fb_mode && ovly_info.flag & 2)
#define ROTATE_IGNORE	(fb_mode && ovly_info.flag & 4)
#define RESET_BUF	0x1
#define FREE_ENTRY	0x2

#define NEED_VSYNC(fbi, vid)	(fbi->wait_vsync && dispd_dma_enabled(fbi, vid))

int unsupport_format(struct pxa168fb_info *fbi, struct _sViewPortInfo
		viewPortInfo, FBVideoMode videoMode, int video_or_graphic);
u8 *buf_dequeue(u8 **ppBufList);
int buf_enqueue(u8 **ppBufList, u8 *pBuf);
u8 *buf_gethead(u8 **ppBufList);
int convert_pix_fmt(u32 vmode);
int set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt);
void pxa168fb_update_addr(struct pxa168fb_info *fbi,
	struct _sVideoBufferAddr *new_addr, int video_or_graphic);
int check_surface(struct fb_info *fi,
			FBVideoMode new_mode,
			struct _sViewPortInfo *new_info,
			struct _sViewPortOffset *new_offset,
			struct _sVideoBufferAddr *new_addr,
			int video_or_graphic);
int check_surface_addr(struct fb_info *fi,
			struct _sOvlySurface *surface, int video_or_graphic);
#ifdef OVLY_TASKLET
void pxa168fb_ovly_task(unsigned long data);
#else
void pxa168fb_ovly_work(struct work_struct *w);
#endif

void collectFreeBuf(struct pxa168fb_info *fbi,
		u8 *filterList[][3], u8 **freeList);
void clearFilterBuf(u8 *ppBufList[][3], int iFlag);
void buf_endframe(void *point, int video_or_graphic);
int flip_buffer(struct fb_info *info, unsigned long arg,
		int video_or_graphic);
int get_freelist(struct fb_info *info, unsigned long arg,
		int video_or_graphic);
void dual_pos_zoom(struct pxa168fb_info *fbi,
	struct _sOvlySurface *surface,
	int *xzoom, int *yzoom, int *xpos, int *ypos);
void buf_clear(u8 **ppBufList, int iFlag);
void clear_buffer(struct pxa168fb_info *fbi, int video_layer);
void set_dma_active(struct pxa168fb_info *fbi, int video_layer);
int dispd_dma_enabled(struct pxa168fb_info *fbi, int video_layer);
void wait_for_vsync(struct pxa168fb_info *fbi, int video_layer);

#endif
