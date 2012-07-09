#include <linux/fb.h>
#include <mach/pxa95xfb.h>
#include <mach/mfp.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>

u8 init_board_ls043[][BOARD_INIT_MAX_DATA_BYTES] =
{
	//wait, command, parameter0,  1st,  2nd,  3rd,  4th,  5th  ,  6th  , 7th  , 8th,  9th,  10th
	{180, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0x11},		/* sleep out180*/
	{1, LCD_Controller_DCS_SHORT_WRITE_WITH_PARAMETER, 0x3a, 0x55},	/* RGB565*/
	{50, LCD_Controller_DCS_SHORT_WRITE_NO_PARAMETER, 0x29},		/* display on	50*/
	{0xff,0xff},
};

struct fb_videomode video_modes_ls043[] = {
	[0] = {
		.pixclock       = 23812,
		.refresh        = 30,
		.xres           = 540,
		.yres           = 960,
		.hsync_len      = 6, /*HSW*/
		.left_margin    = 21, /*BLW*/
		.right_margin   = 179, /*ELW*/
		.vsync_len      = 2, /*VSW*/
		.upper_margin   = 32, /*BFW*/
		.lower_margin   = 32, /*EFW*/
		.sync           = 0,
	},
};

static void panel_power_ls043(int on)
{
	struct regulator *v_ldo;
	v_ldo = regulator_get(NULL, "v_ls043");
	if (IS_ERR(v_ldo)) {
		printk(KERN_ERR "lcd: fail to get ldo handle!\n");
		return;
	}

	if (on) {
		regulator_set_voltage(v_ldo, 3100000, 3100000);
		regulator_enable(v_ldo);
	} else {
		regulator_disable(v_ldo);
	}

	regulator_put(v_ldo);
	printk(KERN_INFO "lcd: turn %s SHARP ls043 panel.\n", on?"on":"off");
}

static struct pxa95xfb_mach_info lcd_info /*__initdata*/ = {
	.id = "Base",
	.modes = video_modes_ls043,
	.num_modes = ARRAY_SIZE(video_modes_ls043),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_16_RGB565,
	.panel_type = LCD_Controller_Active,
	.window = 0,
	.mixer_id = 1,
	.zorder = 1,
	.converter = LCD_M2DSI1,
	.output = OUTPUT_PANEL,
	.active = 1,
	.panel_power = panel_power_ls043,
	.invert_pixclock = 1,
	.dsi_clock_val = (((23812000) * 8)/1000000)+1,
	.dsi_init_cmds = init_board_ls043,
	.dsi_mode = DSI_MODE_VIDEO_NON_BURST,
	.dsi_lane_nr = LCD_Controller_DSI_2LANE,
};

static struct pxa95xfb_mach_info lcd_ovly_info /*__initdata*/ = {
	.id = "Ovly",
	.modes = video_modes_ls043,
	.num_modes = ARRAY_SIZE(video_modes_ls043),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_16_RGB565,
	.panel_type = LCD_Controller_Active,
	.window = 4,
	.mixer_id = 1,
	.zorder = 0,
	.converter = LCD_M2DSI1,
	.output = OUTPUT_PANEL,
	.active = 1,
	.panel_power = panel_power_ls043,
	.invert_pixclock = 1,
	.dsi_clock_val = (((23812000) * 8)/1000000)+1,
	.dsi_init_cmds = init_board_ls043,
	.dsi_mode = DSI_MODE_VIDEO_NON_BURST,
	.dsi_lane_nr = LCD_Controller_DSI_2LANE,
};

void __init pxa95x_add_lcd_sharp(void)
{
	set_pxa95x_fb_info(&lcd_info);
	set_pxa95x_fb_ovly_info(&lcd_ovly_info, 0);
}
