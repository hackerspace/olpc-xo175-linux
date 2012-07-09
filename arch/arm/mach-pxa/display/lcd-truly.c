#include <linux/fb.h>
#include <mach/pxa95xfb.h>
#include <mach/mfp.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>

extern void panel_power_trulywvga(int ssp_port, int on);

static void panel_power_truly(int on)
{
	static struct regulator *vlcd;
	if (!vlcd) {
		vlcd = regulator_get(NULL, "v_lcd_cywee_touch");
		if (IS_ERR(vlcd)) {
			printk(KERN_ERR "lcd: fail to get ldo handle!\n");
			vlcd = NULL;
		}
	}
	if (on) {
		if (vlcd)
			regulator_enable(vlcd);
		panel_power_trulywvga(1, on);
	} else {
		panel_power_trulywvga(1, on);
		if (vlcd)
			regulator_disable(vlcd);
	}
}

static void panel_reset_truly(void)
{
	int reset_pin;
	int err;

	reset_pin = mfp_to_gpio(MFP_PIN_GPIO23);
	err = gpio_request(reset_pin, "DSI Reset");
	if (err) {
		gpio_free(reset_pin);
		printk(KERN_ERR "Request GPIO failed, gpio: %d return :%d\n",
		       reset_pin, err);
		return;
	}
	gpio_direction_output(reset_pin, 1);
	msleep(1);
	gpio_direction_output(reset_pin, 0);
	msleep(1);
	gpio_direction_output(reset_pin, 1);
	msleep(10);
	gpio_free(reset_pin);
}

struct fb_videomode video_modes_trulywvga[] = {
	[0] = {
		.pixclock = 41701,
		.refresh = 60,
		.xres = 480,
		.yres = 800,
		.hsync_len = 19,
		.left_margin = 40,
		.right_margin = 59,
		.vsync_len = 9,
		.upper_margin = 4,
		.lower_margin = 9,
		.sync = 0,
	},
};

static struct pxa95xfb_mach_info lcd_info /*__initdata*/ = {
	.id = "Base",
	.modes = video_modes_trulywvga,
	.num_modes = ARRAY_SIZE(video_modes_trulywvga),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_24_RGB888,
	.panel_type = LCD_Controller_Active,
	.window = 0,
	.mixer_id = 0,
	.zorder = 1,
	.converter = LCD_M2PARALELL_CONVERTER,
	.output = OUTPUT_PANEL,
	.active = 1,
	.panel_power = panel_power_truly,
	.invert_pixclock = 1,
	.reset = panel_reset_truly,
};

static struct pxa95xfb_mach_info lcd_ovly_info /*__initdata*/ = {
	.id = "Ovly",
	.modes = video_modes_trulywvga,
	.num_modes = ARRAY_SIZE(video_modes_trulywvga),
	.pix_fmt_in = PIX_FMTIN_RGB_16,
	.pix_fmt_out = PIX_FMTOUT_24_RGB888,
	.panel_type = LCD_Controller_Active,
	.window = 4,
	.mixer_id = 0,
	.zorder = 0,
	.converter = LCD_M2PARALELL_CONVERTER,
	.output = OUTPUT_PANEL,
	.active = 1,
	.panel_power = panel_power_truly,
	.invert_pixclock = 1,
	.reset = panel_reset_truly,
};

void __init pxa95x_add_lcd_truly(void)
{
	set_pxa95x_fb_info(&lcd_info);
	set_pxa95x_fb_ovly_info(&lcd_ovly_info, 0);
}
