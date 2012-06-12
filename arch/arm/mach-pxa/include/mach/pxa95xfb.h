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

typedef enum {
	LCD_Controller_READ_STATUS = 0,
	LCD_Controller_AUTO_STATUS_CHECK,
	LCD_Controller_WAIT_FOR_BUSY,
	LCD_Controller_READ_FRAME_BUFFER,
	LCD_Controller_LOAD_MATCH_REGISTER,
	LCD_Controller_COMMAND_WRITE,
	LCD_Controller_DATA_WRITE,
	LCD_Controller_LINE_DATA_WRITE,
	LCD_Controller_WAIT_FOR_VSYNC,
	LCD_Controller_SET_DLY_MULT,
	LCD_Controller_NO_OPERATION,
	LCD_Controller_INTERRUPT_THE_PROCESSOR,
	LCD_Controller_SET_GPIO,
	LCD_Controller_EXECUTE_LOOP_BUFFER = 0x10,
	LCD_Controller_FLUSH_LOOP_BUFFER,
	LCD_Controller_START_LABEL,
	LCD_Controller_GOTO_START,
	LCD_Controller_COMMAND_WRITE_HOLD = 0x15,
	LCD_Controller_DISABLE_OUTPUT = 0x1D,
} LCD_Controller_DSI_COMMAND;

typedef enum {
	LCD_Controller_VSYNC_START = 0x1,
	LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER = 0x05,
	LCD_Controller_DCS_READ_NO_PARAMETER = 0x06,
	LCD_Controller_END_OF_TRANSMITION = 0x08,
	LCD_Controller_NULL_PACKET = 0x09,
	LCD_Controller_RGB_565_PACKET = 0x0E,
	LCD_Controller_VSYNC_END = 0x11,
	LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER = 0x15,
	LCD_Controller_BLANKING_PACKET = 0x19,
	LCD_Controller_RGB_666_PACKET = 0x1E,
	LCD_Controller_HSYNC_START = 0x21,
	LCD_Controller_GENERIC_SHORT_WRITE_TWO_PARAMETERS = 0x23,
	LCD_Controller_GENERIC_LONG_WRITE = 0x29,
	LCD_Controller_RGB_666_LOOSELY_PACKET = 0x2E,
	LCD_Controller_HSYNC_END = 0x31,
	LCD_Controller_TURN_ON_PERIPHERAL = 0x32,
	LCD_Controller_SET_MAXIMUM_RETURN_PACKET_SIZE = 0x37,
	LCD_Controller_DCS_LONG_WRITE = 0x39,
	LCD_Controller_RGB_888_PACKET = 0x3E,
	LCD_Controller_LONG_PACKET_TBD = 0x69,
} LCD_Controller_DSI_DATA;

typedef enum {
	LCD_Controller_DCS_NOP = 0x0,
	LCD_Controller_DCS_SOFT_RESET = 0x01,
	LCD_Controller_DCS_GET_DIAGNOSTIC_RESULT = 0x0F,
	LCD_Controller_DCS_ENTER_SLEEP_MODE = 0x10,
	LCD_Controller_DCS_EXIT_SLEEP_MODE = 0x11,
	LCD_Controller_DCS_SET_DISPLAY_OFF = 0x28,
	LCD_Controller_DCS_SET_DISPLAY_ON = 0x29,
	LCD_Controller_DCS_SET_COLUMN_ADDRESS = 0x2A,
	LCD_Controller_DCS_SET_PAGE_ADDRESS = 0x2B,
	LCD_Controller_DCS_WRITE_MEMORY_START = 0x2C,
	LCD_Controller_DCS_SET_TEAR_ON = 0x35,
	LCD_Controller_DCS_GET_PIXEL_FORMAT = 0x3A,
	LCD_Controller_DCS_WRITE_MEMORY_CONTINUE = 0x3C,
	LCD_Controller_DCS_SET_THSSI_ON = 0x81,
	LCD_Controller_DCS_DISPLAY_BUFFER_IO_CONTROL = 0x82,
	LCD_Controller_DCS_SET_OUTPUT_VERTICAL_TIMINGS = 0x8b,
	LCD_Controller_DCS_SET_OUTPUT_HORIZONTAL_TIMINGS = 0x92,
	LCD_Controller_DCS_ENABLE_SET_SPECIAL_COMMAND = 0x9D,
	LCD_Controller_DCS_SET_OUTPUT_PIXEL_CLOCK_FREQUENCY = 0x9E,
	LCD_Controller_DCS_WRITE_EDISCO_REGISTER = 0xFD,
} LCD_Controller_DCS_COMMANDS;

#define BOARD_INIT_MAX_DATA_BYTES   35

typedef enum {
	DSI_MODE_COMMAND_DCS,
	DSI_MODE_COMMAND_PACKET,
	DSI_MODE_VIDEO_NON_BURST,
	DSI_MODE_VIDEO_BURST,
} LCD_Controller_DSI_Mode;

#define DSI_IS_COMMAND_MODE(x) ((x) < DSI_MODE_VIDEO_NON_BURST)
#define DSI_IS_VIDEO_MODE(x) ((x) >= DSI_MODE_VIDEO_NON_BURST)

typedef enum {
	LCD_Controller_DSI_1LANE = 0,
	LCD_Controller_DSI_2LANE,
	LCD_Controller_DSI_3LANE,
	LCD_Controller_DSI_4LANE,
} LCD_Controller_DSI_DATA_LANES;

/*
 * PXA fb machine information
 */
struct pxa95xfb_mach_info {
	char	id[16];
	unsigned int	sclk_clock;

	int		num_modes;
	struct fb_videomode *modes;
	int		init_mode;

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

	unsigned int dither_tbl_4x8;
	unsigned int dither_en;

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
	/*
	 * DSI configure
	 */
	u8 *dsi_init_cmds;
	u8 *dsi_sleep_cmds;
	LCD_Controller_DSI_Mode dsi_mode;
	LCD_Controller_DSI_DATA_LANES dsi_lane_nr;
};

void set_pxa95x_fb_info(void *info);
void set_pxa95x_fb_ovly_info(struct pxa95xfb_mach_info *info, int id);
#endif /* __ASM_MACH_PXA95xFB_H */
