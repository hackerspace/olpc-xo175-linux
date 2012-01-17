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

#define PXA95xFB_FB_NUM           4

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
#endif /* __ASM_MACH_PXA95xFB_H */
