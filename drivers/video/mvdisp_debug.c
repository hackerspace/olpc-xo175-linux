/*
 *  linux/drivers/video/mvdisp_debug.c
 */

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "pxa95xfb.h"
struct bmp_info_header {
	unsigned int    biSize;
	int             biWidth;
	int             biHeight;
	short           biPlanes;
	short           biBitCount;
	unsigned int    biCompression;
	unsigned int    biSizeImage;
	int             biXPelsPerMeter;
	int             biYPelsPerMeter;
	unsigned int    biClrUsed;
	unsigned int    biClrImportant;
} __attribute((packed));

struct bmp_file_header {
	short           bfType;
	unsigned int    bfSize;
	short           bfReserved1;
	short           bfReserved2;
	unsigned int    bfOffBits;
} __attribute((packed));

static void dump_bmp(char *name, unsigned char *src, int fmt, int w, int h)
{
	struct bmp_file_header    file_header;
	struct bmp_info_header    info_header;
	int ret, i, j, size = w * h * 3;
	unsigned char *src_line, *dst_line, *dst;
	struct file *f;
	mm_segment_t old_fs;

	memset(&file_header, 0, sizeof(struct bmp_file_header));
	file_header.bfType	   = 0x4D42;
	file_header.bfSize	   = sizeof(struct bmp_file_header);
	file_header.bfReserved1   = 0;
	file_header.bfReserved2   = 0;
	file_header.bfOffBits	   = sizeof(struct bmp_file_header)
		+ sizeof(struct bmp_info_header);

	memset(&info_header, 0, sizeof(struct bmp_info_header));
	info_header.biSize	   = sizeof(struct bmp_info_header);
	info_header.biWidth	   = w;
	info_header.biHeight	   = h;
	info_header.biPlanes	   = 1;
	info_header.biBitCount    = 24;
	info_header.biCompression = 0;/*BI_RGB;*/

	dst = vmalloc(size);
	if (!dst) {
		vfree(dst);
		return ;
	}

	dst_line = dst + (h-1) * w * 3;
	src_line = src;

	switch (fmt)
	{
	case PIX_FMTIN_RGB_16:
		for(i = 0; i < h; i++) {
			unsigned char *s = src_line;
			unsigned char *d = dst_line;
			for(j = 0; j < w; j++) {
				unsigned short rgb = *(unsigned short*)s;
				d[0] = ((rgb & 0x001f) << 3);
				d[1] = ((rgb & 0x07e0) >> 3);
				d[2] = ((rgb & 0xf800) >> 8);
				s += 2;
				d += 3;
			}
			src_line += 2*w;
			dst_line -= 3*w;
		}
		break;
	case PIX_FMTIN_RGB_24:
	case PIX_FMTIN_RGB_32:
		for(i = 0; i < h; i++) {
			unsigned char *s = src_line;
			unsigned char *d = dst_line;
			for(j = 0; j < w; j++) {
				d[0] = s[0];
				d[1] = s[1];
				d[2] = s[2];
				s += 4;
				d += 3;
			}
			src_line += 4*w;
			dst_line -= 3*w;
		}
		break;
	case PIX_FMTIN_RGB_24_PACK:
		for(i = 0; i < h; i++) {
			memcpy(dst_line, src_line, 3*w);
			src_line += 3*w;
			dst_line -= 3*w;
		}
		break;

	default:
		pr_err("format not supported\n");
		break;
	}

	printk("creat %s\n", name);
	f = filp_open(name, O_RDWR | O_CREAT | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		pr_err("dump fail: fail to open\n");
		vfree(dst);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	ret = f->f_op->write(f, (const char *)&file_header, sizeof(struct bmp_file_header), &f->f_pos);
	ret = f->f_op->write(f, (const char *)&info_header, sizeof(struct bmp_info_header), &f->f_pos);
	ret = f->f_op->write(f, dst, size, &f->f_pos);

	set_fs(old_fs);
	if (ret < 0)
		pr_err("dump fail: fail to write\n");
	filp_close(f, NULL);
	vfree(dst);
}

static void dump_yuv(char *name, unsigned char *src, int fmt, int w, int h)
{
	struct file *f;
	int ret, size;
	mm_segment_t old_fs;

	switch (fmt)
	{
	case PIX_FMTIN_YUV420:
	case PIX_FMTIN_YVU420:
	case PIX_FMTIN_YUV422:
	case PIX_FMTIN_YUV422IL:
		size = w*h*2;
		break;
	case PIX_FMTIN_YUV444:
		size = w*h*3;
		break;
	default:
		pr_err("format not supported\n");
		break;
	}

	printk("creat %s\n", name);
	f = filp_open(name, O_RDWR | O_CREAT | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		pr_err("dump fail: fail to open\n");
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	ret = f->f_op->write(f, src, size, &f->f_pos);

	set_fs(old_fs);
	if (ret < 0)
		pr_err("dump fail: fail to write\n");
	filp_close(f, NULL);
}

void dump_buffer(struct pxa95xfb_info *fbi, int yoffset)
{
	static int c;
	char name[128];
	unsigned char *src;
	int w = fbi->mode.xres, h = fbi->mode.yres;

	if (fbi->user_addr) {
		printk("buffer dump... using user pointer\n");
		w = fbi->surface.viewPortInfo.srcWidth;
		h = fbi->surface.viewPortInfo.srcHeight;
		src = (unsigned char *)phys_to_virt(fbi->user_addr);
	} else {
		printk("buffer dump... using fb buffer\n");
		w = fbi->mode.xres;
		h = fbi->mode.yres;
		src = (unsigned char *)(fbi->fb_start + yoffset * pix_fmt_to_bpp(fbi->pix_fmt) * w);
	}

	if (format_is_yuv(fbi->pix_fmt)) {
		sprintf(name, "/data/fb%d.%04d.yuv", fbi->id, c++);
		dump_yuv(name, src, fbi->pix_fmt, w, h);
	} else {
		sprintf(name, "/data/fb%d.%04d.bmp", fbi->id, c++);
		dump_bmp(name, src, fbi->pix_fmt, w, h);
	}
}

static void dump_regs_controller(struct pxa95xfb_info *fbi)
{
	/* LCD CTL related */
	printk("LCD_CTL = %08x\n", readl(fbi->reg_base + LCD_CTL));
	printk("LCD_CTL_INT_STS = %08x\n", readl(fbi->reg_base + LCD_CTL_INT_STS));

	/* LCD fetch related */
	printk("LCD_FETCH_CTL%d = %08x\n", fbi->window,
		readl(fbi->reg_base + LCD_FETCH_CTL0 + 0x40 * fbi->window));
	printk("LCD_NXT_DESC_ADDR%d = %08x\n", fbi->window,
		readl(fbi->reg_base + LCD_NXT_DESC_ADDR0 + 0x40 * fbi->window));
	printk("LCD_FETCH_STS%d = %08x\n", fbi->window,
		readl(fbi->reg_base + LCD_FETCH_STS0 + 0x40 * fbi->window));
	printk("LCD_FR_ADDR%d = %08x\n", fbi->window,
		readl(fbi->reg_base + LCD_FR_ADDR0 + 0x40 * fbi->window));
	printk("LCD_FETCH_INT_STS0 = %08x\n", readl(fbi->reg_base + LCD_FETCH_INT_STS0));
	printk("LCD_FETCH_INT_STS1 = %08x\n", readl(fbi->reg_base + LCD_FETCH_INT_STS1));

	/* LCD alpha related */
	printk("LCD_CH%d_ALPHA = %08x\n", fbi->window,
		readl(fbi->reg_base + LCD_CH0_ALPHA + 0x4 * fbi->window));
	printk("LCD_CH%d_CLR_MATCH = %08x\n", fbi->window,
		readl(fbi->reg_base + LCD_CH0_CLR_MATCH + 0x4 * fbi->window));

	/* LCD window related */
	if (fbi->window == 4) {
		printk("LCD_WIN4_CTL = %08x\n", readl(fbi->reg_base + LCD_WIN4_CTL));
		printk("LCD_WIN4_CFG = %08x\n", readl(fbi->reg_base + LCD_WIN4_CFG));
		printk("LCD_WIN4_CROP0 = %08x\n", readl(fbi->reg_base + LCD_WIN4_CROP0));
		printk("LCD_WIN4_CROP1 = %08x\n", readl(fbi->reg_base + LCD_WIN4_CROP1));
		printk("LCD_GWIN_INT_STS = %08x\n", readl(fbi->reg_base + LCD_GWIN_INT_STS));
		printk("LCD_WIN4_INT_STS = %08x\n", readl(fbi->reg_base + LCD_WIN4_INT_STS));

	} else {
		printk("LCD_WIN%d_CTL = %08x\n", fbi->window,
			readl(fbi->reg_base + LCD_WIN0_CTL + 0x40 * fbi->window));
		printk("LCD_WIN%d_CFG = %08x\n", fbi->window,
			readl(fbi->reg_base + LCD_WIN0_CFG + 0x40 * fbi->window));
		printk("LCD_WIN%d_CROP0 = %08x\n", fbi->window,
			readl(fbi->reg_base + LCD_WIN0_CROP0 + 0x40 * fbi->window));
		printk("LCD_WIN%d_CROP1 = %08x\n", fbi->window,
			readl(fbi->reg_base + LCD_WIN0_CROP1 + 0x40 * fbi->window));
		printk("LCD_GWIN_INT_STS = %08x\n", readl(fbi->reg_base + LCD_GWIN_INT_STS));
		printk("LCD_WIN%d_INT_STS = %08x\n", fbi->window,
			readl(fbi->reg_base + LCD_WIN0_INT_STS + 0x40 * fbi->window));
	}

	/* LCD mixer related */
	printk("LCD_MIXER%d_CTL0 = %08x\n", fbi->mixer_id,
		readl(fbi->reg_base + LCD_MIXER0_CTL0 + 0x100 * fbi->mixer_id));
	printk("LCD_MIXER%d_CTL1 = %08x\n", fbi->mixer_id,
		readl(fbi->reg_base + LCD_MIXER0_CTL1 + 0x100 * fbi->mixer_id));
	printk("LCD_MIXER%d_CTL2 = %08x\n", fbi->mixer_id,
		readl(fbi->reg_base + LCD_MIXER0_CTL2 + 0x100 * fbi->mixer_id));
	printk("LCD_MIXER%d_STS = %08x\n", fbi->mixer_id,
		readl(fbi->reg_base + LCD_MIXER0_STS + 0x100 * fbi->mixer_id));
	printk("LCD_MIXER%d_STS1 = %08x\n", fbi->mixer_id,
		readl(fbi->reg_base + LCD_MIXER0_STS1 + 0x100 * fbi->mixer_id));

	printk("LCD_MIXER%d_OL%d_CFG0 = %08x\n", fbi->mixer_id, fbi->zorder,
		readl(fbi->reg_base + LCD_MIXER0_BP_CFG0 + 0x100 * fbi->mixer_id + 0x8 * fbi->zorder));
	printk("LCD_MIXER%d_OL%d_CFG1 = %08x\n", fbi->mixer_id, fbi->zorder,
		readl(fbi->reg_base + LCD_MIXER0_BP_CFG1 + 0x100 * fbi->mixer_id + 0x8 * fbi->zorder));
	printk("LCD_GMIX_INT_STS = %08x\n", readl(fbi->reg_base + LCD_GMIX_INT_STS));
	printk("LCD_MIXER%d_INT_STS = %08x\n", fbi->mixer_id,
		readl(fbi->reg_base + LCD_MIXER0_INT_STS + 0x100 * fbi->mixer_id));
}

static void dump_regs_parallel(struct pxa95xfb_info *fbi)
{
	printk("LCD_CONV0_CTL = %08x\n", readl(fbi->reg_base + LCD_CONV0_CTL));
	printk("LCD_MIXER0_TIM0 = %08x\n", readl(fbi->reg_base + LCD_MIXER0_TIM0));
	printk("LCD_MIXER0_TIM1 = %08x\n", readl(fbi->reg_base + LCD_MIXER0_TIM1));
	printk("LCD_MIXER0_INT_STS = %08x\n", readl(fbi->reg_base + LCD_MIXER0_INT_STS));
	printk("LCD_DITHER_CTL = %08x\n", readl(fbi->reg_base + LCD_DITHER_CTL));
}

static void dump_regs_dsi(struct pxa95xfb_info *fbi)
{
	void *conv_base =
		CONVERTER_BASE_ADDRESS(fbi->reg_base, fbi->converter);

	printk("LCD_DSI_D%dSCR0 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxSCR0_OFFSET));
	printk("LCD_DSI_D%dCONV_GEN_NULL_BLANK = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxCONV_GEN_NULL_BLANK_OFFSET));
	printk("LCD_DSI_D%dCONV_RD0 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxCONV_DSI_RD0_OFFSET));
	printk("LCD_DSI_D%dCONV_RD1 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxCONV_DSI_RD1_OFFSET));
	printk("LCD_DSI_D%dCONV_FIFO_THRESH = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxCONV_FIFO_THRESH_OFFSET));
	printk("LCD_DSI_D%dCONV_FIFO_THRESH_INT = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxCONV_FIFO_THRESH_INT_OFFSET));
	printk("LCD_DSI_D%dSCR1 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxSCR1_OFFSET));
	printk("LCD_DSI_D%dSSR = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxSSR_OFFSET));
	printk("LCD_DSI_D%d_G_CTL = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_Dx_G_CTL_OFFSET));
	printk("LCD_DSI_D%d_G_DAT_RED = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_Dx_G_DAT_RED_OFFSET));
	printk("LCD_DSI_D%d_G_DAT_GREEN = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_Dx_G_DAT_GREEN_OFFSET));
	printk("LCD_DSI_D%d_G_DAT_BLUE = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_Dx_G_DAT_BLUE_OFFSET));
	printk("LCD_DSI_D%dCFIF = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxCFIF_OFFSET));
	printk("LCD_DSI_D%dTRIG = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxTRIG_OFFSET));
	printk("LCD_DSI_D%dPRSR = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxPRSR_OFFSET));
	printk("LCD_DSI_D%dTEINTCNT = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxTEINTCNT_OFFSET));
	printk("LCD_DSI_D%dINEN = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxINEN_OFFSET));
	printk("LCD_DSI_D%dINST0 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxINST0_OFFSET));
	printk("LCD_DSI_D%dINST1 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxINST1_OFFSET));
	printk("LCD_DSI_D%dADAT = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxADAT_OFFSET));
	printk("LCD_DSI_D%dTIM0 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxTIM0_OFFSET));
	printk("LCD_DSI_D%dTIM1 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxTIM1_OFFSET));
	printk("LCD_DSI_D%dPHY_TIM0 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxPHY_TIM0_OFFSET));
	printk("LCD_DSI_D%dPHY_TIM1 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxPHY_TIM1_OFFSET));
	printk("LCD_DSI_D%dVSYNC_JITT_TIM0 = %08x\n", fbi->converter - 1,
		readl(conv_base + LCD_DSI_DxVSYNC_JITT_TIM0));

	if (fbi->converter == LCD_M2DSI1)
		printk("DSI_DITHER_CTL = %08x\n", readl(conv_base + LCD_DSI_DITHER_CTL));
}

static void dump_regs_ihdmi(struct pxa95xfb_info *fbi)
{
	void *conv_base =
		CONVERTER_BASE_ADDRESS(fbi->reg_base, fbi->converter);

	printk("HDMI_CONV_CTL = %08x\n", readl(conv_base + HDMI_CONV_CTL));
	printk("HDMI_MIXER_TIM0 = %08x\n", readl(conv_base + HDMI_MIXER_TIM0));
	printk("HDMI_MIXER_TIM1 = %08x\n", readl(conv_base + HDMI_MIXER_TIM1));
	printk("HDMI_MIXER_TIM2 = %08x\n", readl(conv_base + HDMI_MIXER_TIM2));
	printk("HDMI_MIXER_TIM3 = %08x\n", readl(conv_base + HDMI_MIXER_TIM3));
	printk("HDMI_MIXER_TIM4 = %08x\n", readl(conv_base + HDMI_MIXER_TIM4));
	printk("HDMI_CONV_INT_STS = %08x\n", readl(conv_base + HDMI_CONV_INT_STS));
	printk("HDMI_CONV_CTL = %08x\n", readl(conv_base + HDMI_CONV_CTL));
	printk("HDMI_CLK_DIV = %08x\n", readl(conv_base + HDMI_CLK_DIV));
	printk("HDMI_PCLK_DIV = %08x\n", readl(conv_base + HDMI_PCLK_DIV));
	printk("HDMI_TCLK_DIV = %08x\n", readl(conv_base + HDMI_TCLK_DIV));
	printk("HDMI_PRCLK_DIV = %08x\n", readl(conv_base + HDMI_PRCLK_DIV));
	printk("HDMI_MIXER_TIM4 = %08x\n", readl(conv_base + HDMI_MIXER_TIM4));
}

static void dump_regs(struct pxa95xfb_info *fbi)
{
	switch (fbi->converter)
	{
	case LCD_M2PARALELL_CONVERTER:
		printk("dump regs...using parallel interface\n");
		dump_regs_controller(fbi);
		dump_regs_parallel(fbi);
		break;
	case LCD_M2DSI0:
	case LCD_M2DSI1:
		printk("dump regs...using parallel interface\n");
		dump_regs_controller(fbi);
		dump_regs_dsi(fbi);
		break;
	case LCD_M2HDMI:
		printk("dump regs...using internal HDMI interface\n");
		dump_regs_controller(fbi);
		dump_regs_ihdmi(fbi);
	default:
		printk("dump regs...converter disabled stop dump\n");
		break;
	}
}

static void dump_fb_info(struct pxa95xfb_info *fbi)
{
	printk("fb%d info:\n", fbi->id);
	printk("\tfetch%d, ol%d on mixer%d\n", fbi->window, fbi->zorder, fbi->mixer_id);
	switch (fbi->converter)
	{
	case LCD_M2PARALELL_CONVERTER:
		printk("\tParallel interface\n");
		break;
	case LCD_M2DSI0:
		printk("\tDSI0 interface\n");
		break;
	case LCD_M2DSI1:
		printk("\tDSI1 interface\n");
		break;
	case LCD_M2HDMI:
		printk("\tInternal HDMI interface\n");
		break;
	default:
		printk("\tconverter disabled\n");
		break;
	}

	printk("\tOutput resolution %d X %d\n", fbi->mode.xres, fbi->mode.yres);
	printk("buffer status:\n");
	if (fbi->user_addr) {
		printk("\tbuffer user pointer phys addr %08x\n", (int)fbi->user_addr);
		printk("\tbuffer size %d X %d\n",
			fbi->surface.viewPortInfo.srcWidth, fbi->surface.viewPortInfo.srcHeight);
		printk("\tbuffer zoom dst %d X %d\n",
			fbi->surface.viewPortInfo.zoomXSize, fbi->surface.viewPortInfo.zoomYSize);
		printk("\tbuffer position %d X %d\n",
			fbi->surface.viewPortOffset.xOffset, fbi->surface.viewPortOffset.yOffset);
	} else {
		printk("\tbuffer fb buffer phys addr %08x\n", fbi->fb_start_dma);
		printk("\tbuffer size %d X %d\n",
			fbi->mode.xres, fbi->mode.yres);
	}
	switch (fbi->pix_fmt)
	{
	case PIX_FMTIN_RGB_16:
		printk("\tbuffer format RGB565\n");
		break;
	case PIX_FMTIN_RGB_24:
		printk("\tbuffer format XRGB888\n");
		break;
	case PIX_FMTIN_RGB_32:
		printk("\tbuffer format ARGB888\n");
		break;
	case PIX_FMTIN_RGB_24_PACK:
		printk("\tbuffer format RGB888\n");
		break;
	case PIX_FMTIN_YUV420:
		printk("\tbuffer format YUV420p\n");
		break;
	case PIX_FMTIN_YVU420:
		printk("\tbuffer format YVU420\n");
		break;
	case PIX_FMTIN_YUV422:
		printk("\tbuffer format YUV422p\n");
		break;
	case PIX_FMTIN_YUV444:
		printk("\tbuffer format YUV444p\n");
		break;
	case PIX_FMTIN_YUV422IL:
		printk("\tbuffer format YUV422il\n");
		break;
	default:
		printk("\tbuffer format not supported\n");
		break;
	}
}

static void dump_helper(void)
{
	printk("***how to dump****:\n");
	printk("\tdump = 0 would dump infos\n");
	printk("\tdump = 1 would dump registers\n");
	printk("\tdump = 2 would dump buffer instantly\n");
	printk("\tdump = 3 would start dump when each frame passed down\n");
	printk("\tdump = 4 would stop continous dump\n");
}

/* dump support:
 * echo <value> > /sys/class/graphics/fb0/device/dump
 * 0. dump fb info:
 * 1. reg dump
 * 2. dump buffer instantly, support both yuv/rgb and user pointer/fb buffer
 * 3 .start dump when each frame passed down
 * 4. stop continous dump
 */
static ssize_t dump_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	dump_helper();
	return 0;
}

static ssize_t dump_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa95xfb_info *fbi = dev_get_drvdata(dev);
	int dump;
	sscanf(buf, "%d", &dump);
	switch (dump)
	{
	case 0:
		dump_fb_info(fbi);
		break;
	case 1:
		dump_regs(fbi);
		break;
	case 2:
		dump_buffer(fbi, 0);
		break;
	case 3:
		fbi->dump = 1;
		break;
	case 4:
		fbi->dump = 0;
		break;
	default:
		printk("dump command %d is not recoginzed\n", dump);
		dump_helper();
		break;
	}

	return size;
}

static DEVICE_ATTR(dump, S_IRUGO | S_IWUSR, dump_show, dump_store);

int mvdisp_debug_init(struct device *dev)
{
	return device_create_file(dev, &dev_attr_dump);
}
