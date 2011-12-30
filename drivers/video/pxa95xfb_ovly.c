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
	struct clk *clk = NULL;
	int irq_conv, ret = 0, i;
	struct pxa95xfb_conv_info *conv;

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

	if (mi->converter == LCD_M2DSI0) {
		clk = clk_get(&pdev->dev, "PXA95x_DSI0CLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get DSI0 CLK");
			goto failed;
		}
	} else if (mi->converter == LCD_M2DSI1) {
		clk = clk_get(&pdev->dev, "PXA95x_DSI1CLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get DSI1 CLK");
			goto failed;
		}
	} else if (mi->converter == LCD_M2HDMI) {
		clk = clk_get(&pdev->dev, "PXA95x_iHDMICLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get internal HDMI CLK");
			goto failed;
		}
	}

	if(mi->converter != LCD_MIXER_DISABLE){
		irq_conv = platform_get_irq(pdev, mi->converter);
		if (irq_conv < 0) {
			dev_err(&pdev->dev, "no IRQ defined\n");
			ret = -ENOMEM;
			goto failed_free_clk;
		}
	}else{
		dev_err(&pdev->dev, "no converter defined\n");
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	info = framebuffer_alloc(sizeof(struct pxa95xfb_info), &pdev->dev);
	if (info == NULL){
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	/* Initialize private data */
	fbi = info->par;
	if(!fbi) {
		ret = -EINVAL;
		goto failed_free_clk;
	}
	fbi->fb_info = info;
	platform_set_drvdata(pdev, fbi);
	fbi->dev = &pdev->dev;
	fbi->id = pdev->id+1;
	fbi->on = 0;
	fbi->controller_on = 0;
	fbi->active = 0;
	fbi->open_count = fbi->on;/*if fbi on as default, open count +1 as default*/
	fbi->is_blanked = 0;
	fbi->suspend = 0;
	fbi->debug = 0;
	fbi->window = mi->window;
	fbi->zorder = mi->zorder;
	fbi->mixer_id = mi->mixer_id;
	fbi->converter = mi->converter;
	fbi->user_addr = 0;
	fbi->vsync_en = 0;
	fbi->eof_intr_en = 1;
	fbi->eof_handler = lcdc_vid_buf_endframe;

	mutex_init(&fbi->access_ok);
	init_waitqueue_head(&fbi->w_intr_wq);

	memset(&fbi->surface, 0, sizeof(fbi->surface));
	memset(&fbi->mode, 0, sizeof(struct fb_videomode));

	/* Map LCD controller registers.*/
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	/* Allocate framebuffer memory.*/
	fbi->fb_size = PAGE_ALIGN(mi->modes[0].xres * mi->modes[0].yres * 4 + PAGE_SIZE);
	fbi->fb_start = lcdc_alloc_framebuffer(fbi->fb_size + PAGE_SIZE,
				&fbi->fb_start_dma);
	if (fbi->fb_start == NULL) {
		ret = -ENOMEM;
		goto failed_free_clk;
	}
	memset(fbi->fb_start, 0x0, fbi->fb_size);
	fbi->fb_start = fbi->fb_start + PAGE_SIZE;
	fbi->fb_start_dma = fbi->fb_start_dma + PAGE_SIZE;

	/* Initialise static fb parameters.*/
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
		FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	strcpy(info->fix.id, mi->id);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 1;
	info->fix.ypanstep = 1;
	info->fix.ywrapstep = 0;
	info->fix.mmio_start = res->start;
	info->fix.mmio_len = res->end - res->start + 1;
	info->fix.accel = FB_ACCEL_NONE;
	info->fbops = &pxa95xfb_vid_ops;
	info->pseudo_palette = fbi->pseudo_palette;
	info->fix.smem_start = fbi->fb_start_dma;
	info->fix.smem_len = fbi->fb_size;
	info->screen_base = fbi->fb_start;
	info->screen_size = fbi->fb_size;

	for(i = 0; i < mi->num_modes; i++)
		lcdc_correct_pixclock(&mi->modes[i]);
	fb_videomode_to_modelist(mi->modes, mi->num_modes, &info->modelist);
	/* init var: according to modes[0] */
	lcdc_set_pix_fmt(&info->var, mi->pix_fmt_in);
	lcdc_set_mode_to_var(fbi, &info->var, &mi->modes[0]);
	memcpy(&fbi->mode, &mi->modes[0], sizeof(struct fb_videomode));
	info->var.xoffset = info->var.yoffset = 0;

	pxa95xfbi[fbi->id] = fbi;

	/*init converter*/
	conv = &pxa95xfb_conv[fbi->converter -1];
	if(!conv->inited){
		conv->inited = 1;
		conv->output = mi->output;
		conv->clk = clk;
		conv->irq = irq_conv;
		conv->pix_fmt_out = mi->pix_fmt_out;
		conv->active = mi->active;
		conv->invert_pixclock = mi->invert_pixclock;
		conv->panel_rbswap = mi->panel_rbswap;
		conv->panel_type = mi->panel_type;
		conv->power = mi->panel_power;
		conv->reset = mi->reset;

		converter_init(fbi);
	}

	if(fbi->on)
		converter_openclose(fbi, 1);

	pxa95xfb_set_par(info);

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

	return 0;

failed_free_cmap:
	fb_dealloc_cmap(&info->cmap);
failed_free_irq_conv:
	free_irq(irq_conv, fbi);
failed_free_clk:
	clk_disable(clk);
	clk_put(clk);
failed:
	pr_err("pxa95xfb-ovly: frame buffer device init failed\n");
	platform_set_drvdata(pdev, NULL);

	if (fbi && fbi->reg_base) {
		iounmap(fbi->reg_base);
		kfree(fbi);
	}

	return ret;
}

static struct platform_driver pxa95xfb_vid_driver = {
	.driver		= {
		.name	= "pxa95xfb-ovly",
		.owner	= THIS_MODULE,
	},
	.probe		= pxa95xfb_vid_probe,
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
