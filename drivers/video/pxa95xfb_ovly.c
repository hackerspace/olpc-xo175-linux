/*
 *  linux/drivers/video/pxa95xfb_ovly.c
 */

#include "pxa95xfb.h"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

static int pxa95xfb_vid_open(struct fb_info *fi, int user)
{
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
	fbi->open_count ++;

	if(mutex_is_locked(&fbi->access_ok))
		mutex_unlock(&fbi->access_ok);

	lcdc_vid_clean(fbi);

	return 0;
}

static int pxa95xfb_vid_release(struct fb_info *fi, int user)
{
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;
	struct fb_var_screeninfo *var = &fi->var;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);

	lcdc_vid_clean(fbi);

	/* Turn off compatibility mode */
	var->nonstd &= ~0xff000000;
	fbi->open_count --;
	return 0;
}

static int pxa95xfb_vid_blank(int blank, struct fb_info *info)
{
	struct pxa95xfb_info *fbi = info->par;

	fbi->is_blanked = (blank == FB_BLANK_UNBLANK) ? 0 : 1;

	return 0;	/* TODO */
}

static struct fb_ops pxa95xfb_vid_ops = {
	.owner		= THIS_MODULE,
	.fb_open        = pxa95xfb_vid_open,
	.fb_release     = pxa95xfb_vid_release,
	.fb_blank		= pxa95xfb_vid_blank,
	.fb_ioctl       = pxa95xfb_ioctl,
	.fb_set_par		= pxa95xfb_set_par,
	.fb_check_var	= pxa95xfb_check_var,
	.fb_setcolreg	= pxa95xfb_setcolreg,	/* TODO */
	.fb_pan_display	= pxa95xfb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int __devinit pxa95xfb_vid_probe(struct platform_device *pdev)
{
	struct pxa95xfb_mach_info *mi;
	struct fb_info *info = 0;
	struct pxa95xfb_info *fbi = 0;
	struct resource *res;
	int irq_conv, ret = 0, i;
	struct pxa95xfb_conv_info *conv;
	int max_x, max_y;
	max_x = max_y = 0;

	mi = pdev->dev.platform_data;
	if (mi == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}
	printk(KERN_INFO "vid probe: %s.%d: %s\n", pdev->name, pdev->id, mi->id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		return -ENOENT;
	}

	if(mi->converter != LCD_MIXER_DISABLE){
		irq_conv = platform_get_irq(pdev, mi->converter);
		if (irq_conv < 0) {
			dev_err(&pdev->dev, "no IRQ defined\n");
			ret = -ENOMEM;
			goto failed;
		}
	}else{
		dev_err(&pdev->dev, "no converter defined\n");
		ret = -ENOMEM;
		goto failed;
	}

	info = framebuffer_alloc(sizeof(struct pxa95xfb_info), &pdev->dev);
	if (info == NULL){
		ret = -ENOMEM;
		goto failed;
	}

	/* Initialize private data */
	fbi = info->par;
	if(!fbi) {
		ret = -EINVAL;
		goto failed;
	}
	memset(fbi, 0, sizeof(fbi));

	fbi->fb_info = info;
	platform_set_drvdata(pdev, fbi);
	fbi->dev = &pdev->dev;
	fbi->id = pdev->id+1;
	fbi->on = 0;
	fbi->controller_on = fbi->on;
	fbi->open_count = fbi->on;/*if fbi on as default, open count +1 as default*/
	fbi->window = mi->window;
	fbi->zorder = mi->zorder;
	fbi->mixer_id = mi->mixer_id;
	fbi->converter = mi->converter;

	fbi->vsync_en = 0;
	fbi->eof_intr_en = 1;
	if (!fb_is_baselay(fbi))
		fbi->eof_handler = lcdc_vid_buf_endframe;
	spin_lock_init(&fbi->buf_lock);
	mutex_init(&fbi->access_ok);

	/* Map LCD controller registers.*/
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		ret = -ENOMEM;
		goto failed;
	}

	for(i = 0; i < mi->num_modes; i++) {
		if ((max_x * max_y) < (mi->modes[i].xres * mi->modes[i].yres)) {
			max_x = mi->modes[i].xres;
			max_y = mi->modes[i].yres;
		}
	}
	/* Allocate framebuffer memory.*/
	fbi->fb_size = PAGE_ALIGN(max_x * max_y * 4 + PAGE_SIZE);
	fbi->fb_start = lcdc_alloc_framebuffer(fbi->fb_size + PAGE_SIZE,
				&fbi->fb_start_dma);
	if (fbi->fb_start == NULL) {
		ret = -ENOMEM;
		goto failed;
	}
	memset(fbi->fb_start, 0x0, fbi->fb_size);
	fbi->fb_start = fbi->fb_start + PAGE_SIZE;
	fbi->fb_start_dma = fbi->fb_start_dma + PAGE_SIZE;

	/* Set video mode and init var*/
	for(i = 0; i < mi->num_modes; i++)
		lcdc_correct_pixclock(&mi->modes[i]);
	fb_videomode_to_modelist(mi->modes, mi->num_modes, &info->modelist);

	lcdc_set_pix_fmt(&info->var, mi->pix_fmt_in);
	fb_videomode_to_var(&info->var, &mi->modes[mi->init_mode]);
	memcpy(&fbi->mode, &mi->modes[mi->init_mode], sizeof(struct fb_videomode));

	/* fix to 2* yres */
	info->var.yres_virtual = info->var.yres * 2;
	if (mi->output == OUTPUT_HDMI) {
		info->var.yres_virtual = info->var.yres;
	}

	/* Initialise static fb parameters.*/
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
		FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	strcpy(info->fix.id, mi->id);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 0;
	info->fix.ypanstep = info->var.yres;
	info->fix.ywrapstep = 0;
	info->fix.mmio_start = res->start;
	info->fix.mmio_len = res->end - res->start + 1;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.smem_start = fbi->fb_start_dma;
	info->fix.smem_len = fbi->fb_size;
	info->fix.visual = (fbi->pix_fmt == PIX_FMT_PSEUDOCOLOR)?
		FB_VISUAL_PSEUDOCOLOR: FB_VISUAL_TRUECOLOR;
	info->fix.line_length = info->var.xres_virtual * info->var.bits_per_pixel / 8;

	info->fbops = &pxa95xfb_vid_ops;
	info->pseudo_palette = fbi->pseudo_palette;
	info->screen_base = fbi->fb_start;
	info->screen_size = fbi->fb_size;

	pxa95xfbi[fbi->id] = fbi;

	/*init converter*/
	conv = &pxa95xfb_conv[fbi->converter -1];
	if(!conv->inited){
		conv->inited = 1;
		conv->output = mi->output;
		conv->irq = irq_conv;
		conv->pix_fmt_out = mi->pix_fmt_out;
		conv->active = mi->active;
		conv->invert_pixclock = mi->invert_pixclock;
		conv->panel_rbswap = mi->panel_rbswap;
		conv->panel_type = mi->panel_type;
		conv->power = mi->panel_power;
		conv->reset = mi->reset;
		conv->dsi_init_cmds = mi->dsi_init_cmds;
		conv->dsi_sleep_cmds = mi->dsi_sleep_cmds;
		conv->conf_dsi_video_mode = mi->dsi_mode;
		conv->dsi_lanes = mi->dsi_lane_nr;

		converter_init(fbi);
	}

	/* turn on when boot */
	if(fbi->on) {
		lcdc_set_fr_addr(fbi);
		lcdc_set_lcd_controller(fbi);
		if (!conv_is_on(fbi))
			converter_onoff(fbi, 1);
		conv_ref_inc(fbi);
	}

	/* Allocate color map.*/
	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed_free_irq_conv;
	}

	/* Register framebuffer.*/
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa95x-fb: %d\n", ret);
		ret = -ENXIO;
		goto failed_free_cmap;
	}
	printk(KERN_INFO "pxa95xfb_ovly: frame buffer device %s was loaded"
			" to /dev/fb%d <%s>.\n", conv->output?"HDMI":"PANEL", info->node, info->fix.id);


	mvdisp_debug_init(&pdev->dev);

	return 0;

failed_free_cmap:
	fb_dealloc_cmap(&info->cmap);
failed_free_irq_conv:
	free_irq(irq_conv, fbi);
failed:
	pr_err("pxa95xfb-ovly: frame buffer device init failed\n");
	platform_set_drvdata(pdev, NULL);

	if (fbi && fbi->reg_base) {
		iounmap(fbi->reg_base);
		kfree(fbi);
	}

	return ret;
}

#ifdef CONFIG_PM
extern void unset_dvfm_constraint(void);

static int pxa95xfb_ovly_suspend(struct platform_device *dev, pm_message_t state)
{
	struct pxa95xfb_info *fbi = platform_get_drvdata(dev);

	/* disable fb hdmi when suspend */
	if (fb_is_tv(fbi) && fbi->controller_on) {
		fbi->on = 0;
		conv_ref_dec(fbi);
		lcdc_set_lcd_controller(fbi);
		if (!conv_is_on(fbi))
			converter_onoff(fbi, 0);
		fbi->controller_on = 0;
		fbi->user_addr = 0;
		lcdc_set_fr_addr(fbi);
		unset_dvfm_constraint();
	}

	return 0;
}

static int pxa95xfb_ovly_resume(struct platform_device *dev)
{
	return 0;
}
#endif

static struct platform_driver pxa95xfb_vid_driver = {
	.driver		= {
		.name	= "pxa95xfb-ovly",
		.owner	= THIS_MODULE,
	},
	.probe		= pxa95xfb_vid_probe,
#ifdef CONFIG_PM
	.suspend	= pxa95xfb_ovly_suspend,
	.resume		= pxa95xfb_ovly_resume,
#endif
};

static int __devinit pxa95xfb_vid_init(void)
{
	return platform_driver_register(&pxa95xfb_vid_driver);
}
/* module_init(pxa95xfb_init); */
late_initcall(pxa95xfb_vid_init);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@marvell.com>");
MODULE_DESCRIPTION("Framebuffer driver for PXA95x");
MODULE_LICENSE("GPL");
