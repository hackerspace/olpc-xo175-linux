/*
 * ov2656 timing setting
 *
 * Copyright (c) 2010 Marvell Ltd.
 * Angela Wan <jwan@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/string.h>
#include "ov2656.h"

static int bus_type_index;
static struct resv_size ov2656_resv[] = {
	{   0,    0},
	{ 176,  144},	/* QCIF */
	{ 240,	160},	/* QHVGA */
	{ 320,  240},	/* QVGA */
	{ 352,  288},	/* CIF */
	{ 480,  320},	/* HALF_VGA */
	{ 640,  480},	/* VGA */
	{ 720,  480},	/* D1 */
	{ 800,  480},	/* WVGA */
	{ 800,  600},	/* SVGA */
	{1280,  720},	/* 720P */
	{1600,  1200},	/* UXVGA */
};

static struct regval_list ov2656_fmt_yuv422[] = {

	{0x308c, 0x80},
	{0x308d, 0x0e},
	{0x30b0, 0xff},
	{0x30b1, 0xff},
	{0x30b2, 0x24},
	{0x3082, 0x01},		//non-defined in spec
	{0x30f4, 0x01},		//non-defined in spec
	{0x3090, 0x33},		//defined "Reserved" in spec
	{0x30a8, 0x56},		//non-defined
	{0x3093, 0x00},		//reserved
	{0x307e, 0xe5},
	{0x3079, 0x00},
	{0x30aa, 0x42},		//non-defined
	{0x3017, 0x40},
	{0x30f3, 0x82},		//non-defined
	{0x306a, 0x0c},		//reserved
	{0x306d, 0x00},
	{0x336a, 0x3c},		//reserved
	{0x3076, 0x6a},
	{0x30d9, 0x8c},		//non-defined
	{0x3016, 0x82},
	{0x304e, 0x88},		//non-defined
	{0x30f1, 0x82},		//reserved
	{0x3011, 0x02},

	//AEC/AGC
	{0x3013, 0xff},
	{0x3015, 0x02},		//AGC max gain = 8x 
	{0x301c, 0x13},
	{0x301d, 0x17},
	{0x3070, 0x3e},
	{0x3072, 0x34},

	//others
	{0x3300, 0xfc},
	{0x3302, 0x01},
	{0x3400, 0x02},
	{0x3601, 0x30},
	{0x30f3, 0x83},

	{0x304e, 0x04},		//; [7] DVP_CLK_snr
	{0x309e, 0x08}, 	//disable lp_rx
	{0x3606, 0x00}, 	//disable dvp
	{0x3084, 0x01}, 	//scale_div_man_en
	{0x3010, 0x80}, 	//scale_div_man
	{0x3011, 0x00}, 	//sys_div /4
	{0x3634, 0x26},

	//;MIPI
	{0x363b, 0x01},	//
	{0x3630, 0x35},	//

	{0x3023, 0x06},	//  ;array output vertical start L
	{0x3012, 0x10},	//  ;<4>vario
	{0x302A, 0x02},	//  ;VTS high-byte
	{0x302B, 0x6a},	//  ;VTS low-byte
	{0x306f, 0x14},	//  ;<6:4>window,<3:0>targ_s
	{0x3026, 0x02},	//  ;array output height H
	{0x3027, 0x61},	//  ;array output height L
	{0x308a, 0x02},	//  ;image output height H
	{0x308b, 0x58},	//  ;image output height L

	{0x3319, 0x0c},	// ; for awb timing   
	{0x331d, 0x4c},	// ; for awb timing 

	{0x3302, 0x11},
	{0x3317, 0x25},	// ;
	{0x3318, 0x80},	// ;

	{OV2656_END_ADDR, OV2656_END_VAL},
};

static struct regval_list ov2656_yuv_low[] = {

	{0x3012, 0x10},		//  ;<4>vario
	{0x302A, 0x02},		//  ;VTS high-byte
	{0x302B, 0x6a},		//  ;VTS low-byte
	{0x3023, 0x06},
	{0x3026, 0x02},		//  ;array output height H
	{0x3027, 0x61},		//  ;array output height L
	{0x3088, 0x03},		//  ;image output width H
	{0x3089, 0x20},		//  ;image output width L
	{0x308a, 0x02},		//  ;image output height H
	{0x308b, 0x58},		//  ;image output height L

	{0x3314, 0x00},		//
	{0x3317, 0x25},		//
	{0x3318, 0x80},

	{OV2656_END_ADDR, OV2656_END_VAL},
};

static struct regval_list ov2656_yuv_high[] = {

	{0x3012, 0x00},		//  ;<4>vario
	{0x302A, 0x04},		//  ;VTS high-byte
	{0x302B, 0xd4},		//  ;VTS low-byte
	{0x3023, 0x0c},
	{0x3026, 0x04},		//  ;array output height H		//1212
	{0x3027, 0xbc},		//  ;array output height L
	{0x3088, 0x06},		//  ;image output width H		//1600
	{0x3089, 0x40},		//  ;image output width L
	{0x308a, 0x04},		//  ;image output height H		//1200
	{0x308b, 0xb0},		//  ;image output height L

	{0x3314, 0x00},		//
	{0x3317, 0x4b},		//1200
	{0x3318, 0x00},

	{OV2656_END_ADDR, OV2656_END_VAL},
};

static struct regval_list ov2656_color_effect[] = {

	//;AWB
	{0x3320, 0x98}, 
	{0x3321, 0x11},
	{0x3322, 0x92},
	{0x3323, 0x01},
	{0x3324, 0x96},
	{0x3325, 0x02},
	{0x3326, 0xff},
	{0x3327, 0x0d},
	{0x3328, 0x0f},
	{0x3329, 0x0f},
	{0x332a, 0x59},
	{0x332b, 0x68},
	{0x332c, 0x19},
	{0x332d, 0xba},
	{0x332e, 0x3a},
	{0x332f, 0x3e},
	{0x3330, 0x4a},
	{0x3331, 0x50},
	{0x3332, 0xf0},
	{0x3333, 0x0a}, 
	{0x3334, 0xf0},
	{0x3335, 0xf0},
	{0x3336, 0xf0},
	{0x3337, 0x40},
	{0x3338, 0x40},
	{0x3339, 0x40},
	{0x333a, 0x00}, 
	{0x333b, 0x00},  

	//;Color Matrix
	{0x3380, 0x28}, 
	{0x3381, 0x48},  
	{0x3382, 0x10}, 
	{0x3383, 0x2c},
	{0x3384, 0x7c},
	{0x3385, 0xa8},
	{0x3386, 0xa2},
	{0x3387, 0xa1},
	{0x3388, 0x01},
	{0x3389, 0x98},
	{0x338a, 0x1 },

	//;Gamma
	{0x334f, 0x21}, 
	{0x3340, 0x06},
	{0x3341, 0x11},
	{0x3342, 0x22},
	{0x3343, 0x33},
	{0x3344, 0x46},
	{0x3345, 0x53},
	{0x3346, 0x5f},
	{0x3347, 0x6d},
	{0x3348, 0x7a},
	{0x3349, 0x8e},
	{0x334a, 0x9f},
	{0x334b, 0xad},
	{0x334c, 0xc5},
	{0x334d, 0xd6},
	{0x334e, 0xe7},

	//;Lens correction
	{0x3350, 0x2f}, 
	{0x3351, 0x2B},  
	{0x3352, 0xC2},  
	{0x3353, 0x1D},  
	{0x3354, 0x01},  
	{0x3355, 0x85},  
	{0x3356, 0x30},   
	{0x3357, 0x2B},   
	{0x3358, 0xaf},   
	{0x3359, 0x19},   
	{0x335a, 0x00},  
	{0x335b, 0x85},   
	{0x335c, 0x2f},   
	{0x335d, 0x2b},   
	{0x335e, 0xce},   
	{0x335f, 0x19},   
	{0x3360, 0x00},  
	{0x3361, 0x85},   
	{0x3362, 0x00},   
	{0x3363, 0x00},   
	{0x3364, 0x03},   
	{0x3365, 0x02},  
	{0x3366, 0x00},  

	//;UVadjust
	{0x3301, 0xff},
	{0x338B, 0x1f},
	{0x338c, 0x10},
	{0x338d, 0x40},

	//;Sharpness/De-noise
	{0x3370, 0xd0}, 
	{0x3371, 0x00},
	{0x3372, 0x00},
	{0x3373, 0x40},
	{0x3374, 0x10},
	{0x3375, 0x10},

	//;Sharpness
	{0x3376, 0x05}, 
	{0x3377, 0x00},
	{0x3378, 0x04},
	{0x3379, 0x70},


	//;black sun 
	//;Avdd 2.55~3.0V
	{0x3090, 0x03}, 
	{0x30a8, 0x54},
	{0x30aa, 0x82},
	{0x30a3, 0x91},
	{0x30a1, 0x41},

	//;BLC
	{0x3069, 0x84}, 
	{0x307c, 0x10},
	{0x3087, 0x02},

	//;AEC/AGC
	{0x3013, 0xff}, 
	{0x301c, 0x1a},
	{0x301d, 0x1f},
	{0x3070, 0x2e},
	{0x3072, 0x26},

	//;AE Target
	{0x3018, 0x4d}, 
	{0x3019, 0x3d},
	{0x301a, 0x83},

	//;D5060       
	{0x3014, 0x0c}, 
	//{0x3015, 0x53},
	{0x30af, 0x10},
	{0x304a, 0x00},
	{0x304f, 0x00},

	//;AE Window
	{0x3030, 0xaa}, 
	{0x3031, 0xff},
	{0x3032, 0xff},
	{0x3033, 0xaa},

	//;AE Window Size
	{0x3038, 0x01}, 
	{0x3039, 0xc6},
	{0x303a, 0x00},
	{0x303b, 0x86},
	//{0x303c, 0x01},
	//{0x303d, 0x2c},
	//{0x303e, 0x00},
	//{0x303f, 0xe1},

	{OV2656_END_ADDR, OV2656_END_VAL},
};

static struct regval_list ov2656_res_qcif[] =
{
	{0x3088, 0x00},		//  ;image output width H
	{0x3089, 0xb0},		//  ;image output width L
	{0x308a, 0x00},		//  ;image output height H
	{0x308b, 0x90},		//  ;image output height L

	{OV2656_END_ADDR, OV2656_END_VAL},
};

static struct regval_list ov2656_res_qvga[] =
{
	{0x3088, 0x01},		//  ;image output width H
	{0x3089, 0x40},		//  ;image output width L
	{0x308a, 0x00},		//  ;image output height H
	{0x308b, 0xf0},		//  ;image output height L

	{OV2656_END_ADDR, OV2656_END_VAL}
};

static struct regval_list ov2656_res_vga[] =
{
	{0x3088, 0x02},		// ;
	{0x3089, 0x80},		// ;
	{0x308a, 0x01},		// ;
	{0x308b, 0xe0},		// ;

	{OV2656_END_ADDR, OV2656_END_VAL}
};

static struct regval_list ov2656_res_d1[] =	//720x480
{
	{0x3088, 0x02},		// ;
	{0x3089, 0xd0},		// ;
	{0x308a, 0x01},		// ;
	{0x308b, 0xe0},		// ;

	{OV2656_END_ADDR, OV2656_END_VAL}
};

static struct regval_list ov2656_res_wvga[] =	//800x480
{
	{0x3088, 0x03},		// ;
	{0x3089, 0x20},		// ;
	{0x308a, 0x01},		// ;
	{0x308b, 0xe0},		// ;

	{OV2656_END_ADDR, OV2656_END_VAL}
};

static struct regval_list ov2656_res_svga[] =	//800x600
{
	{0x3088, 0x03},		// ;
	{0x3089, 0x20},		// ;
	{0x308a, 0x02},		// ;
	{0x308b, 0x58},		// ;

	{OV2656_END_ADDR, OV2656_END_VAL}
};

static struct regval_list ov2656_res_720p[] =
{
	{0x3088, 0x05},		//  ;image output width H		//1280
	{0x3089, 0x00},		//  ;image output width L
	{0x308a, 0x02},		//  ;image output height H		//720
	{0x308b, 0xd0},		//  ;image output height L
	
	{0x3314, 0x0a},
	{0x3317, 0x38},

	{0x331a, 0x50},		//1280
	{0x331b, 0x2d},		//720
	{0x331c, 0x00},

	{OV2656_END_ADDR, OV2656_END_VAL}
};

static struct regval_list ov2656_res_uxvga[] =	//1600x1200
{
	{0x3088, 0x06},		//  ;image output width H		//1600
	{0x3089, 0x40},		//  ;image output width L
	{0x308a, 0x04},		//  ;image output height H		//1200
	{0x308b, 0xb0},		//  ;image output height L

	{0x3314, 0x00},
	{0x3317, 0x4b},

	{0x331a, 0x64},
	{0x331b, 0x4b},
	{0x331c, 0x00},

	{OV2656_END_ADDR, OV2656_END_VAL}
};

static struct ov2656_win_size ov2656_sizes_yuv[] = {
	/* QCIF */
	{
		.resv = OV2656_FMT_QCIF,
		.regs = ov2656_res_qcif,
	},
	/* QVGA */
	{
		.resv = OV2656_FMT_QVGA,
		.regs = ov2656_res_qvga,
	},
	/* VGA */
	{
		.resv = OV2656_FMT_VGA,
		.regs = ov2656_res_vga,
	},
	/* D1 */
	{
		.resv = OV2656_FMT_D1,
		.regs = ov2656_res_d1,
	},
	/* Wvga */
	{
		.resv = OV2656_FMT_WVGA,
		.regs = ov2656_res_wvga,
	},
	/* Svga */
	{
		.resv = OV2656_FMT_SVGA,
		.regs = ov2656_res_svga,
	},
	/* 720p */
	{
		.resv = OV2656_FMT_720P,
		.regs = ov2656_res_720p,
	},
	/* UXVGA 1600*1200 */
	{
		.resv = OV2656_FMT_UXVGA,
		.regs = ov2656_res_uxvga,
	}
};

static struct ov2656_format ov2656_pxa2128_mipi_fmts[] = {
	{
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.fmt_support = RESV(OV2656_FMT_UXVGA),
		.regs = ov2656_yuv_high,
	},
	{
		.code = V4L2_MBUS_FMT_YVYU8_2X8,
		.fmt_support = RESV(OV2656_FMT_UXVGA),
		.regs = ov2656_yuv_high,
	},
	{
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.fmt_support = RESV(OV2656_FMT_720P),
		.regs = ov2656_yuv_high,
	},
	{
		.code = V4L2_MBUS_FMT_YVYU8_2X8,
		.fmt_support = RESV(OV2656_FMT_720P),
		.regs = ov2656_yuv_high,
	},
	{
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.regs = ov2656_yuv_low,
	},
	{
		.code = V4L2_MBUS_FMT_YVYU8_2X8,
		.regs = ov2656_yuv_low,
	}
};

static struct ov2656_config ov2656_cfg[] = {
	{
		.name = "mk2-mipi",
		.init = ov2656_fmt_yuv422,
		.fmt = ov2656_pxa2128_mipi_fmts,
		.fmt_size = ARRAY_SIZE(ov2656_pxa2128_mipi_fmts),
		.yuv_res = ov2656_sizes_yuv,
		.yuv_res_size = ARRAY_SIZE(ov2656_sizes_yuv),
	},
};

int ov2656_select_bus_type(const char *name)
{
	int i, size, ret = 0;
	size = ARRAY_SIZE(ov2656_cfg);
	for (i = 0; i < size; i++) {
		if (name && !strcmp(name, ov2656_cfg[i].name)) {
			bus_type_index = i;
			break;
		}
	}
	if (i == size)
		ret = -EINVAL;
	printk(KERN_NOTICE "cam: ov2656: select sensor as \"%s\"\n", name);
	return ret;
}

int ov2656_set_yuv_res_array(struct v4l2_frmsizeenum *fsize)
{
	int size = ov2656_cfg[bus_type_index].yuv_res_size;
	struct ov2656_win_size *res;

	if (fsize->index >= size)
		return -EINVAL;

	res = &(ov2656_cfg[bus_type_index].yuv_res[fsize->index]);
	if (res) {
		fsize->discrete.height = ov2656_resv[res->resv].height;
		fsize->discrete.width = ov2656_resv[res->resv].width;
		return 0;
	}

	return -EINVAL;
}

struct regval_list *ov2656_get_global_init_regs()
{
	return ov2656_cfg[bus_type_index].init;
}

struct regval_list *ov2656_get_global_color_effect(void)
{
	return ov2656_color_effect;
}

static int resv_match(unsigned int fmt_support, int witdh, int height)
{
	int i, end, size_fmt;
	if (fmt_support == 0)
		return 1;

	size_fmt = sizeof(fmt_support) * 8;
	end = (OV2656_FMT_END > size_fmt) ? size_fmt : OV2656_FMT_END;
	for (i = 0; i < end; i++)
		if ((fmt_support & (1 << i))
		    && (ov2656_resv[i].width == witdh)
		    && (ov2656_resv[i].height == height)) {
			return 1;
		}

	return 0;
}

struct regval_list *ov2656_get_fmt_regs(enum v4l2_mbus_pixelcode code,
				 int witdh, int height)
{
	int i, size, fmt_support;
	size = ov2656_cfg[bus_type_index].fmt_size;
	if (!size)
		return NULL;
	for (i = 0; i < size; i++) {
		fmt_support = ov2656_cfg[bus_type_index].fmt[i].fmt_support;
		if ((code == ov2656_cfg[bus_type_index].fmt[i].code)
		    && resv_match(fmt_support, witdh, height)) {
			return ov2656_cfg[bus_type_index].fmt[i].regs;
		}
	}

	return NULL;
}

struct regval_list *ov2656_get_fmt_default_setting(enum v4l2_mbus_pixelcode code)
{
	int i, size;

	size = ov2656_cfg[bus_type_index].fmt_size;
	if (!size)
		return NULL;
	for (i = 0; i < size; i++) {
		if (code == ov2656_cfg[bus_type_index].fmt[i].code)
			return ov2656_cfg[bus_type_index].fmt[i].def_set;
	}

	return NULL;
}

struct regval_list *ov2656_get_yuv_size_regs(int width, int height)
{
	int i, size;
	struct resv_size *resv;

	size = ov2656_cfg[bus_type_index].yuv_res_size;
	for (i = 0; i < size; i++) {
		resv = &ov2656_resv[ov2656_cfg[bus_type_index].yuv_res[i].resv];
		if (width == resv->width && height == resv->height)
			return ov2656_cfg[bus_type_index].yuv_res[i].regs;
	}

	return NULL;
}

struct regval_list *ov2656_get_yuv_lane_set(int width, int height)
{
	int i, size;
	struct resv_size *resv;

	size = ov2656_cfg[bus_type_index].yuv_res_size;
	for (i = 0; i < size; i++) {
		resv = &ov2656_resv[ov2656_cfg[bus_type_index].yuv_res[i].resv];
		if (width == resv->width && height == resv->height)
			return ov2656_cfg[bus_type_index].yuv_res[i].lane_set;
	}

	return NULL;
}

struct regval_list *ov2656_get_yuv_resolution_regs(int width, int height)
{
	int i, size;
	struct resv_size *resv;

	size = ov2656_cfg[bus_type_index].yuv_res_size;
	for (i = 0; i < size; i++) {
		resv = &ov2656_resv[ov2656_cfg[bus_type_index].yuv_res[i].resv];
		if (width == resv->width && height == resv->height)
			return ov2656_cfg[bus_type_index].yuv_res[i].regs_resolution;
	}

	return NULL;
}

struct regval_list *ov2656_get_mipi_set_regs()
{
	if (ov2656_cfg[bus_type_index].mipi_lane)
		return ov2656_cfg[bus_type_index].mipi_lane->mipi_set_regs;
	else
		return NULL;
}

struct regval_list *ov2656_get_mipi_lane_regs(int num)
{
	if (ov2656_cfg[bus_type_index].mipi_lane) {
		if (num == 1)
			return ov2656_cfg[bus_type_index].mipi_lane->lane1_regs;
		else if (num == 2)
			return ov2656_cfg[bus_type_index].mipi_lane->lane2_regs;
		else
			return NULL;
	} else
		return NULL;
}
