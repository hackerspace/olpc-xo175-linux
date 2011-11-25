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

#define NEED_VSYNC(fbi)	(fbi->wait_vsync && dispd_dma_enabled(fbi))

#define DEBUG_TV_ACTIVE(id)	(gfx_info.fbi[(id)]->debug & 8)
extern int unsupport_format(struct pxa168fb_info *fbi,
	 struct _sViewPortInfo viewPortInfo, FBVideoMode videoMode);
extern int convert_pix_fmt(u32 vmode);
extern int set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt);
extern int determine_best_pix_fmt(struct fb_var_screeninfo *var,
	unsigned int compat_mode);
extern int pxa168fb_check_var(struct fb_var_screeninfo *var,
	 struct fb_info *fi);
extern int check_surface(struct fb_info *fi, FBVideoMode new_mode,
	struct _sViewPortInfo *new_info, struct _sViewPortOffset *new_offset,
	struct _sVideoBufferAddr *new_addr);
extern int check_surface_addr(struct fb_info *fi,
	 struct _sOvlySurface *surface);
extern int check_modex_active(struct pxa168fb_info *fbi);
#ifdef OVLY_TASKLET
void pxa168fb_ovly_task(unsigned long data);
#else
void pxa168fb_ovly_work(struct work_struct *w);
#endif

extern void buf_endframe(void *point);
extern void clear_buffer(struct pxa168fb_info *fbi);
extern void pxa168fb_list_init(struct pxa168fb_info *fbi);
extern int flip_buffer(struct fb_info *info, unsigned long arg);
extern int get_freelist(struct fb_info *info, unsigned long arg);

extern void set_dma_active(struct pxa168fb_info *fbi);
extern int dispd_dma_enabled(struct pxa168fb_info *fbi);
extern void wait_for_vsync(struct pxa168fb_info *fbi);
extern void pxa168fb_misc_update(struct pxa168fb_info *fbi);
extern void set_start_address(struct fb_info *info, int xoffset,
			 int yoffset, int wait_vsync);
extern void set_dma_control0(struct pxa168fb_info *fbi);
extern void set_screen(struct pxa168fb_info *fbi);
#endif
