#include <linux/fb.h>
#include <mach/pxa95xfb.h>
#include <mach/mfp.h>
#include <linux/gpio.h>
#include <linux/delay.h>

u8 init_board_byd[][BOARD_INIT_MAX_DATA_BYTES] = {
	{1, LCD_Controller_GENERIC_SHORT_WRITE_TWO_PARAMETERS, 0xB0, 0x04},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x03, 0x00,
	 0xB3, 0x02, 0x00},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x03, 0x00,
	 0xB6, 0x52, 0x83},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x05, 0x00,
	 0xB7, 0x00, 0x00, 0x11, 0x25},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x15, 0x00,
	 0xB8, 0x00, 0x09, 0x09, 0xFF, 0xFF, 0xD8, 0xD8, 0x02, 0x18,
	 0x10, 0x10, 0x37, 0x5A, 0x87, 0xBE, 0xFF, 0x00, 0x00, 0x00,
	 0x00},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x05, 0x00,
	 0xB9, 0x00, 0xFF, 0x02, 0x08},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x10, 0x00,
	 0xC1, 0x23, 0x31, 0x99, 0x21, 0x20, 0x00, 0x10, 0x28, 0x0C,
	 0x0C, 0x00, 0x00, 0x00, 0x21, 0x01},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x07, 0x00,
	 0xC2, 0x10, 0x06, 0x06, 0x01, 0x03, 0x00},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x19, 0x00,
	 0xC8, 0x04, 0x10, 0x18, 0x20, 0x2E, 0x46, 0x3C, 0x28, 0x1F,
	 0x18, 0x10, 0x04, 0x04, 0x10, 0x18, 0x20, 0x2E, 0x46, 0x3C,
	 0x28, 0x1F, 0x18, 0x10, 0x04},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x19, 0x00,
	 0xC9, 0x04, 0x10, 0x18, 0x20, 0x2E, 0x46, 0x3C, 0x28, 0x1F,
	 0x18, 0x10, 0x04, 0x04, 0x10, 0x18, 0x20, 0x2E, 0x46, 0x3C,
	 0x28, 0x1F, 0x18, 0x10, 0x04},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x19, 0x00,
	 0xCA, 0x04, 0x10, 0x18, 0x20, 0x2E, 0x46, 0x3C, 0x28, 0x1F,
	 0x18, 0x10, 0x04, 0x04, 0x10, 0x18, 0x20, 0x2E, 0x46, 0x3C,
	 0x28, 0x1F, 0x18, 0x10, 0x04},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x11, 0x00,
	 0xD0, 0x29, 0x03, 0xCE, 0xA6, 0x0C, 0x43, 0x20, 0x10, 0x01,
	 0x00, 0x01, 0x01, 0x00, 0x03, 0x01, 0x00},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x08, 0x00,
	 0xD1, 0x18, 0x0C, 0x23, 0x03, 0x75, 0x02, 0x50},
	{1, LCD_Controller_GENERIC_SHORT_WRITE_TWO_PARAMETERS, 0xD3, 0x33},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x03, 0x00,
	 0xD5, 0x28, 0x28},
	{1, LCD_Controller_GENERIC_LONG_WRITE, 0x03, 0x00,
	 0xDE, 0x01, 0x41},
	{1, LCD_Controller_GENERIC_SHORT_WRITE_TWO_PARAMETERS, 0xE6, 0x51},
	{1, LCD_Controller_GENERIC_SHORT_WRITE_TWO_PARAMETERS, 0xFA, 0x03},
	{0xFF, LCD_Controller_GENERIC_LONG_WRITE, 0x02, 0x00,
	 0xD6, 0x28},
	{100, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0},
	{1, LCD_Controller_DCS_LONG_WRITE, 0x05, 0x00,
	 0x2A, 0x00, 0x00, 0x01, 0xDF},
	{1, LCD_Controller_DCS_LONG_WRITE, 0x05, 0x00,
	 0x2B, 0x00, 0x00, 0x03, 0x1F},
	{1, LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER, 0x35, 0x00},
	{1, LCD_Controller_DCS_LONG_WRITE, 0x03, 0x00,
	 0x44, 0x01, 0x90},
	{1, LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER, 0x36, 0x00},
	{1, LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER, 0x3A, 0x77},
	{0xFF, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0x11},
	{200, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0},
	{0xFF, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0x29},
	{50, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0},
	{1, LCD_Controller_DCS_LONG_WRITE, 0x01, 0x00, 0x2C},
	{0xFF, 0xFF},
};

static struct fb_videomode video_modes_byd[] = {
	[0] = {
		.pixclock	= 39325,
		.refresh	= 60,
		.xres		= 480,
		.yres		= 800,
		.hsync_len	= 10,
		.left_margin	= 10,
		.right_margin	= 10,
		.vsync_len	= 3,
		.upper_margin	= 14,
		.lower_margin	= 14,
	},
};

static void panel_reset_byd(void)
{
	int reset_pin;
	int err;

	pr_info("%s: resetting panel\n", __func__);
	reset_pin = mfp_to_gpio(MFP_PIN_GPIO23);
	err = gpio_request(reset_pin, "DSI Reset");
	if (err) {
		gpio_free(reset_pin);
		printk(KERN_ERR "Request GPIO failed, gpio: %d return :%d\n",
		       reset_pin, err);
		return;
	}
	gpio_direction_output(reset_pin, 1);
	mdelay(1);
	gpio_direction_output(reset_pin, 0);
	mdelay(1);
	gpio_direction_output(reset_pin, 1);
	mdelay(10);
	gpio_free(reset_pin);
}

static struct pxa95xfb_mach_info lcd_info __initdata = {
	.id = "Base",
	.modes = video_modes_byd,
	.num_modes = ARRAY_SIZE(video_modes_byd),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_24_RGB888,
	.panel_type = LCD_Controller_Smart,
	.window = 0,
	.mixer_id = 1,
	.zorder = 1,
	.converter = LCD_M2DSI1,
	.output = OUTPUT_PANEL,
	.active = 1,
	.invert_pixclock = 1,
	.reset = panel_reset_byd,
	.dsi_init_cmds = init_board_byd,
	.dsi_mode = DSI_MODE_COMMAND_DCS,
	.dsi_lane_nr = LCD_Controller_DSI_2LANE,
};

static struct pxa95xfb_mach_info lcd_ovly_info __initdata = {
	.id = "Ovly",
	.modes = video_modes_byd,
	.num_modes = ARRAY_SIZE(video_modes_byd),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_24_RGB888,
	.panel_type = LCD_Controller_Smart,
	.window = 4,
	.mixer_id = 1,
	.zorder = 0,
	.converter = LCD_M2DSI1,
	.output = OUTPUT_PANEL,
	.active = 0,
	.invert_pixclock = 1,
	.reset = panel_reset_byd,
	.dsi_mode = DSI_MODE_COMMAND_DCS,
	.dsi_lane_nr = LCD_Controller_DSI_2LANE,
};

void __init pxa95x_add_lcd_byd(void)
{
	set_pxa95x_fb_info(&lcd_info);
	set_pxa95x_fb_ovly_info(&lcd_ovly_info, 0);
}
