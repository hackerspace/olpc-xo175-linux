/*
 * linux/arch/arm/mach-pxa/include/mach/pxa95xfb.h
 *
 *  Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_PXA95xFB_H
#define __ASM_MACH_PXA95xFB_H
/* ---------------------------------------------- */
/*              Header Files                      */
/* ---------------------------------------------- */
#include <linux/fb.h>

#define PXA95xFB_FB_NUM           3

typedef enum {
	LCD_MIXER_DISABLE = 0,
	LCD_M2PARALELL_CONVERTER,
	LCD_M2DSI0,
	LCD_M2DSI1,
	LCD_M2HDMI,
}pxa95xfb_mixer_output_port;

typedef enum {
	PIX_FMTOUT_8_R8_G8_B8 = 0,
	PIX_FMTOUT_8_R5G3_G5B3_B5R3,
	PIX_FMTOUT_16_RGB565,
	PIX_FMTOUT_16_R8G8_B8R8_G8B8,
	PIX_FMTOUT_18_RGB666,
	PIX_FMTOUT_24_RGB888,
	PIX_FMTOUT_8_R5G3_G3R5,
}LCD_Controller_OP_FOR;

typedef enum {
	PIX_FMTIN_RGB_16 = 0,
	PIX_FMTIN_RGB_24,
	PIX_FMTIN_RGB_32,
	PIX_FMTIN_RGB_24_PACK,
	PIX_FMTIN_YUV420,
	PIX_FMTIN_YUV422,
	PIX_FMTIN_YUV444,
	PIX_FMTIN_YUV422IL,
	PIX_FMT_PSEUDOCOLOR = 20,
}LCD_Controller_SRC_FOR;

typedef enum {
       LCD_Controller_Active = 0,
       LCD_Controller_Smart,
       LCD_Controller_TV_HDMI,
}LCD_Controller_Display_Device;

#define OUTPUT_PANEL 0
#define OUTPUT_HDMI 1

/*
 * PXA fb machine information
 */
struct pxa95xfb_mach_info {
	char	id[16];
	unsigned int	sclk_clock;

	int		num_modes;
	struct fb_videomode *modes;

	/*
	 * Pix_fmt
	 */
	unsigned	pix_fmt_in;	/* PXA95x */
	unsigned	pix_fmt_out;

	/*
	 * I/O pin allocation.
	 */
	unsigned	io_pin_allocation_mode:4;

	/*
	 * Dumb panel -- assignment of R/G/B component info to the 24
	 * available external data lanes.
	 */
	unsigned	dumb_mode:4;
	unsigned	panel_type:2;	/* PXA95x */
	unsigned	panel_rgb_reverse_lanes:1;

	/*
	 * Dumb panel -- GPIO output data.
	 */
	unsigned	gpio_output_mask:8;
	unsigned	gpio_output_data:8;

	/* PXA95x mixer configure */
	int	window;
	int			mixer_id;
	int	zorder;
	pxa95xfb_mixer_output_port      converter;
	int		output;

	/*
	 * Dumb panel -- configurable output signal polarity.
	 */
	unsigned	invert_composite_blank:1;
	unsigned	invert_pix_val_ena:1;
	unsigned	invert_pixclock:1;
	unsigned	invert_vsync:1;
	unsigned	invert_hsync:1;
	unsigned	panel_rbswap:1;
	unsigned	active:1;
	unsigned	enable_lcd:1;
	/*
	 * SPI control
	 */
	unsigned int	spi_ctrl;
	unsigned int	spi_gpio_cs;
	unsigned int 	spi_gpio_reset;
	/*
	 * power on/off function.
	 */
	void (*panel_power)(int);
	void (*reset)(void);
};

void set_pxa95x_fb_info(void *info);
void set_pxa95x_fb_ovly_info(struct pxa95xfb_mach_info *info, int id);

/* ---------------------------------------------- */
/*              IOCTL Definition                  */
/* ---------------------------------------------- */
#define FB_IOC_MAGIC                        'm'
#define FB_IOCTL_CONFIG_CURSOR              _IO(FB_IOC_MAGIC, 0)
#define FB_IOCTL_DUMP_REGS                  _IO(FB_IOC_MAGIC, 1)
#define FB_IOCTL_CLEAR_IRQ                  _IO(FB_IOC_MAGIC, 2)

/*
 * There are many video mode supported.
 */
#define FB_IOCTL_SET_VIDEO_MODE             _IO(FB_IOC_MAGIC, 3)
#define FB_IOCTL_GET_VIDEO_MODE             _IO(FB_IOC_MAGIC, 4)
/* Request a new video buffer from driver. User program needs to free
 * this memory.
 */
#define FB_IOCTL_CREATE_VID_BUFFER          _IO(FB_IOC_MAGIC, 5)

/* Configure viewport in driver. */
#define FB_IOCTL_SET_VIEWPORT_INFO          _IO(FB_IOC_MAGIC, 6)
#define FB_IOCTL_GET_VIEWPORT_INFO          _IO(FB_IOC_MAGIC, 7)

/* Flip the video buffer from user mode. Vide buffer can be separated into:
 * a. Current-used buffer - user program put any data into it. It will be
 *    displayed immediately.
 * b. Requested from driver but not current-used - user programe can put any
 *    data into it. It will be displayed after calling
 *    FB_IOCTL_FLIP_VID_BUFFER.
 *    User program should free this memory when they don't use it any more.
 * c. User program alloated - user program can allocated a contiguos DMA
 *    buffer to store its video data. And flip it to driver. Notices that
 *    this momory should be free by user programs. Driver won't take care of
 *    this.
 */
#define FB_IOCTL_FLIP_VID_BUFFER            _IO(FB_IOC_MAGIC, 8)

/* Get the current buffer information. User program could use it to display
 * anything directly. If developer wants to allocate multiple video layers,
 * try to use FB_IOCTL_CREATE_VID_BUFFER  to request a brand new video
 * buffer.
 */
#define FB_IOCTL_GET_BUFF_ADDR              _IO(FB_IOC_MAGIC, 9)

/* Get/Set offset position of screen */
#define FB_IOCTL_SET_VID_OFFSET             _IO(FB_IOC_MAGIC, 10)
#define FB_IOCTL_GET_VID_OFFSET             _IO(FB_IOC_MAGIC, 11)

/* Turn on the memory toggle function to improve the frame rate while playing
 * movie.
 */
#define FB_IOCTL_SET_MEMORY_TOGGLE          _IO(FB_IOC_MAGIC, 12)

/* Turn on the memory toggle function to improve the frame rate while playing
 * movie.
 */
#define FB_IOCTL_SET_COLORKEYnALPHA         _IO(FB_IOC_MAGIC, 13)
#define FB_IOCTL_GET_COLORKEYnALPHA         _IO(FB_IOC_MAGIC, 14)
#define FB_IOCTL_SWITCH_GRA_OVLY            _IO(FB_IOC_MAGIC, 15)
#define FB_IOCTL_SWITCH_VID_OVLY            _IO(FB_IOC_MAGIC, 16)

/* For VPro integration */
#define FB_IOCTL_GET_FREELIST               _IO(FB_IOC_MAGIC, 17)

/* Wait for vsync happen. */
#define FB_IOCTL_WAIT_VSYNC                 _IO(FB_IOC_MAGIC, 18)

/* clear framebuffer: Makes resolution or color space changes look nicer */
#define FB_IOCTL_CLEAR_FRAMEBUFFER              _IO(FB_IOC_MAGIC, 19)

/* Wait for vsync each time pan display */
#define FB_IOCTL_WAIT_VSYNC_ON              _IO(FB_IOC_MAGIC, 20)
#define FB_IOCTL_WAIT_VSYNC_OFF             _IO(FB_IOC_MAGIC, 21)

/* Get and set the display surface */
#define FB_IOCTL_GET_SURFACE			_IO(FB_IOC_MAGIC, 22)
#define FB_IOCTL_SET_SURFACE			_IO(FB_IOC_MAGIC, 23)

/* Graphic partial display ctrl */
#define FB_IOCTL_GRA_PARTDISP			_IO(FB_IOC_MAGIC, 24)

/* Global alpha blend controls - Maintaining compatibility with existing
   user programs. */
#define FB_IOCTL_PUT_VIDEO_ALPHABLEND            0xeb
#define FB_IOCTL_PUT_GLOBAL_ALPHABLEND           0xe1
#define FB_IOCTL_PUT_GRAPHIC_ALPHABLEND          0xe2

/* color swapping */
#define FB_IOCTL_SWAP_GRAPHIC_RED_BLUE       0xe3
#define FB_IOCTL_SWAP_GRAPHIC_U_V            0xe4
#define FB_IOCTL_SWAP_GRAPHIC_Y_UV           0xe5
#define FB_IOCTL_SWAP_VIDEO_RED_BLUE         0xe6
#define FB_IOCTL_SWAP_VIDEO_U_V              0xe7
#define FB_IOCTL_SWAP_VIDEO_Y_UV             0xe8

/* colorkey compatibility */
#define FB_IOCTL_GET_CHROMAKEYS		0xe9
#define FB_IOCTL_PUT_CHROMAKEYS		0xea

/* cmu operation */
#define FB_IOCTL_CMU_SWITCH		0xf1
#define FB_IOCTL_CMU_WRITE		0xf2
#define FB_IOCTL_CMU_READ		0xf3
#define FB_IOCTL_CMU_SET_ROUTE		0xf4
#define FB_IOCTL_CMU_SET_PIP		0xf5
#define FB_IOCTL_CMU_GET_RES		0xf6
#define FB_IOCTL_CMU_SET_LETTER_BOX 0xf7

/* gamma correction */
#define FB_IOCTL_GAMMA_SET 		0xff

#define FB_VMODE_RGB565			0x100
#define FB_VMODE_BGR565                 0x101
#define FB_VMODE_RGB1555		0x102
#define FB_VMODE_BGR1555                0x103
#define FB_VMODE_RGB888PACK		0x104
#define FB_VMODE_BGR888PACK		0x105
#define FB_VMODE_RGB888UNPACK		0x106
#define FB_VMODE_BGR888UNPACK		0x107
#define FB_VMODE_RGBA888		0x108
#define FB_VMODE_BGRA888		0x109

#define FB_VMODE_YUV422PACKED               0x0
#define FB_VMODE_YUV422PACKED_SWAPUV        0x1
#define FB_VMODE_YUV422PACKED_SWAPYUorV     0x2
#define FB_VMODE_YUV422PLANAR               0x3
#define FB_VMODE_YUV422PLANAR_SWAPUV        0x4
#define FB_VMODE_YUV422PLANAR_SWAPYUorV     0x5
#define FB_VMODE_YUV420PLANAR               0x6
#define FB_VMODE_YUV420PLANAR_SWAPUV        0x7
#define FB_VMODE_YUV420PLANAR_SWAPYUorV     0x8
#define FB_VMODE_YUV422PACKED_IRE_90_270    0x9

#define FB_HWCMODE_1BITMODE                 0x0
#define FB_HWCMODE_2BITMODE                 0x1

#define FB_DISABLE_COLORKEY_MODE            0x0
#define FB_ENABLE_Y_COLORKEY_MODE           0x1
#define FB_ENABLE_U_COLORKEY_MODE           0x2
#define FB_ENABLE_V_COLORKEY_MODE           0x4
#define FB_ENABLE_RGB_COLORKEY_MODE         0x3
#define FB_ENABLE_R_COLORKEY_MODE           0x5
#define FB_ENABLE_G_COLORKEY_MODE           0x6
#define FB_ENABLE_B_COLORKEY_MODE           0x7

#define FB_VID_PATH_ALPHA		0x0
#define FB_GRA_PATH_ALPHA		0x1
#define FB_CONFIG_ALPHA			0x2

#define FB_SYNC_COLORKEY_TO_CHROMA          1
#define FB_SYNC_CHROMA_TO_COLORKEY          2

/* ---------------------------------------------- */
/*              Data Structure                    */
/* ---------------------------------------------- */
/*
 * The follow structures are used to pass data from
 * user space into the kernel for the creation of
 * overlay surfaces and setting the video mode.
 */

#define FBVideoMode int

struct _sViewPortInfo {
	unsigned short srcWidth;        /* video source size */
	unsigned short srcHeight;
	unsigned short zoomXSize;       /* size after zooming */
	unsigned short zoomYSize;
	unsigned short yPitch;
	unsigned short uPitch;
	unsigned short vPitch;
	unsigned int rotation;
	unsigned int yuv_format;
};

struct _sViewPortOffset {
	unsigned short xOffset;         /* position on screen */
	unsigned short yOffset;
};

struct _sVideoBufferAddr {
	unsigned char   frameID;        /* which frame wants */
	 /* new buffer (PA). 3 addr for YUV planar */
	unsigned char *startAddr[3];
	unsigned char *inputData;       /* input buf address (VA) */
	unsigned int length;            /* input data's length */
};

struct _sColorKeyNAlpha {
	unsigned int mode;
	unsigned int alphapath;
	unsigned int config;
	unsigned int Y_ColorAlpha;
	unsigned int U_ColorAlpha;
	unsigned int V_ColorAlpha;
};

struct _sOvlySurface {
	FBVideoMode videoMode;
	struct _sViewPortInfo viewPortInfo;
	struct _sViewPortOffset viewPortOffset;
	struct _sVideoBufferAddr videoBufferAddr;
};

#endif /* __ASM_MACH_PXA95xFB_H */
