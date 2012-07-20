/*
 * OmniVision OV9740 Camera Sensor Driver Based on ECS Framework
 *
 * Copyright (C) 2012 Marvell Corporation
 *
 * Based on driver/media/video/ov9640.c camera driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <mach/camera.h>
#include "ecs-subdev.h"

/* General Status Registers */
#define OV9740_MODEL_ID_HI		0x0000
#define OV9740_MODEL_ID_LO		0x0001
#define OV9740_REVISION_NUMBER		0x0002
#define OV9740_MANUFACTURER_ID		0x0003
#define OV9740_SMIA_VERSION		0x0004

/* General Setup Registers */
#define OV9740_MODE_SELECT		0x0100
#define OV9740_IMAGE_ORT		0x0101
#define OV9740_SOFTWARE_RESET		0x0103
#define OV9740_GRP_PARAM_HOLD		0x0104
#define OV9740_MSK_CORRUP_FM		0x0105

/* Timing Setting */
#define OV9740_FRM_LENGTH_LN_HI		0x0340 /* VTS */
#define OV9740_FRM_LENGTH_LN_LO		0x0341 /* VTS */
#define OV9740_LN_LENGTH_PCK_HI		0x0342 /* HTS */
#define OV9740_LN_LENGTH_PCK_LO		0x0343 /* HTS */
#define OV9740_X_ADDR_START_HI		0x0344
#define OV9740_X_ADDR_START_LO		0x0345
#define OV9740_Y_ADDR_START_HI		0x0346
#define OV9740_Y_ADDR_START_LO		0x0347
#define OV9740_X_ADDR_END_HI		0x0348
#define OV9740_X_ADDR_END_LO		0x0349
#define OV9740_Y_ADDR_END_HI		0x034A
#define OV9740_Y_ADDR_END_LO		0x034B
#define OV9740_X_OUTPUT_SIZE_HI		0x034C
#define OV9740_X_OUTPUT_SIZE_LO		0x034D
#define OV9740_Y_OUTPUT_SIZE_HI		0x034E
#define OV9740_Y_OUTPUT_SIZE_LO		0x034F

/* IO Control Registers */
#define OV9740_IO_CREL00		0x3002
#define OV9740_IO_CREL01		0x3004
#define OV9740_IO_CREL02		0x3005
#define OV9740_IO_MIPI2L		0x301F
#define OV9740_IO_OUTPUT_SEL01		0x3026
#define OV9740_IO_OUTPUT_SEL02		0x3027

/* AWB Registers */
#define OV9740_AWB_MANUAL_CTRL		0x3406

/* Analog Control Registers */
#define OV9740_ANALOG_CTRL01		0x3601
#define OV9740_ANALOG_CTRL02		0x3602
#define OV9740_ANALOG_CTRL03		0x3603
#define OV9740_ANALOG_CTRL04		0x3604
#define OV9740_ANALOG_CTRL10		0x3610
#define OV9740_ANALOG_CTRL12		0x3612
#define OV9740_ANALOG_CTRL20		0x3620
#define OV9740_ANALOG_CTRL21		0x3621
#define OV9740_ANALOG_CTRL22		0x3622
#define OV9740_ANALOG_CTRL30		0x3630
#define OV9740_ANALOG_CTRL31		0x3631
#define OV9740_ANALOG_CTRL32		0x3632
#define OV9740_ANALOG_CTRL33		0x3633

/* Sensor Control */
#define OV9740_SENSOR_CTRL03		0x3703
#define OV9740_SENSOR_CTRL04		0x3704
#define OV9740_SENSOR_CTRL05		0x3705
#define OV9740_SENSOR_CTRL07		0x3707

/* Timing Control */
#define OV9740_TIMING_CTRL17		0x3817
#define OV9740_TIMING_CTRL19		0x3819
#define OV9740_TIMING_CTRL33		0x3833
#define OV9740_TIMING_CTRL35		0x3835

/* Banding Filter */
#define OV9740_AEC_MAXEXPO_60_H		0x3A02
#define OV9740_AEC_MAXEXPO_60_L		0x3A03
#define OV9740_AEC_B50_STEP_HI		0x3A08
#define OV9740_AEC_B50_STEP_LO		0x3A09
#define OV9740_AEC_B60_STEP_HI		0x3A0A
#define OV9740_AEC_B60_STEP_LO		0x3A0B
#define OV9740_AEC_CTRL0D		0x3A0D
#define OV9740_AEC_CTRL0E		0x3A0E
#define OV9740_AEC_MAXEXPO_50_H		0x3A14
#define OV9740_AEC_MAXEXPO_50_L		0x3A15

/* AEC/AGC Control */
#define OV9740_AEC_ENABLE		0x3503
#define OV9740_GAIN_CEILING_01		0x3A18
#define OV9740_GAIN_CEILING_02		0x3A19
#define OV9740_AEC_HI_THRESHOLD		0x3A11
#define OV9740_AEC_3A1A			0x3A1A
#define OV9740_AEC_CTRL1B_WPT2		0x3A1B
#define OV9740_AEC_CTRL0F_WPT		0x3A0F
#define OV9740_AEC_CTRL10_BPT		0x3A10
#define OV9740_AEC_CTRL1E_BPT2		0x3A1E
#define OV9740_AEC_LO_THRESHOLD		0x3A1F

/* BLC Control */
#define OV9740_BLC_AUTO_ENABLE		0x4002
#define OV9740_BLC_MODE			0x4005

/* Output Format select */
#define OV9740_DATA_FORMAT		0x4300

/* VFIFO */
#define OV9740_VFIFO_READ_START_HI	0x4608
#define OV9740_VFIFO_READ_START_LO	0x4609

/* DVP Control */
#define OV9740_DVP_VSYNC_CTRL02		0x4702
#define OV9740_DVP_VSYNC_MODE		0x4704
#define OV9740_DVP_VSYNC_CTRL06		0x4706

/* PLL Setting */
#define OV9740_PLL_MODE_CTRL01		0x3104
#define OV9740_PRE_PLL_CLK_DIV		0x0305
#define OV9740_PLL_MULTIPLIER		0x0307
#define OV9740_VT_SYS_CLK_DIV		0x0303
#define OV9740_VT_PIX_CLK_DIV		0x0301
#define OV9740_PLL_CTRL3010		0x3010
#define OV9740_VFIFO_CTRL00		0x460E

/* ISP Control */
#define OV9740_ISP_CTRL00		0x5000
#define OV9740_ISP_CTRL01		0x5001
#define OV9740_ISP_CTRL03		0x5003
#define OV9740_ISP_CTRL05		0x5005
#define OV9740_ISP_CTRL12		0x5012
#define OV9740_ISP_CTRL19		0x5019
#define OV9740_ISP_CTRL1A		0x501A
#define OV9740_ISP_CTRL1E		0x501E
#define OV9740_ISP_CTRL1F		0x501F
#define OV9740_ISP_CTRL20		0x5020
#define OV9740_ISP_CTRL21		0x5021

/* AWB */
#define OV9740_AWB_CTRL00		0x5180
#define OV9740_AWB_CTRL01		0x5181
#define OV9740_AWB_CTRL02		0x5182
#define OV9740_AWB_CTRL03		0x5183
#define OV9740_AWB_ADV_CTRL01		0x5184
#define OV9740_AWB_ADV_CTRL02		0x5185
#define OV9740_AWB_ADV_CTRL03		0x5186
#define OV9740_AWB_ADV_CTRL04		0x5187
#define OV9740_AWB_ADV_CTRL05		0x5188
#define OV9740_AWB_ADV_CTRL06		0x5189
#define OV9740_AWB_ADV_CTRL07		0x518A
#define OV9740_AWB_ADV_CTRL08		0x518B
#define OV9740_AWB_ADV_CTRL09		0x518C
#define OV9740_AWB_ADV_CTRL10		0x518D
#define OV9740_AWB_ADV_CTRL11		0x518E
#define OV9740_AWB_CTRL0F		0x518F
#define OV9740_AWB_CTRL10		0x5190
#define OV9740_AWB_CTRL11		0x5191
#define OV9740_AWB_CTRL12		0x5192
#define OV9740_AWB_CTRL13		0x5193
#define OV9740_AWB_CTRL14		0x5194

/* MIPI Control */
#define OV9740_MIPI_CTRL00		0x4800
#define OV9740_MIPI_3837		0x3837
#define OV9740_MIPI_CTRL01		0x4801
#define OV9740_MIPI_CTRL03		0x4803
#define OV9740_MIPI_CTRL05		0x4805
#define OV9740_VFIFO_RD_CTRL		0x4601
#define OV9740_MIPI_CTRL_3012		0x3012
#define OV9740_SC_CMMM_MIPI_CTR		0x3014

static const struct reg_tab_wb ov9740_init_req[] = {
	{OV9740_SOFTWARE_RESET,		0x01},
};

static const struct reg_tab_wb ov9740_init_done[] = {
	/* Banding Filter */
	{ OV9740_AEC_B50_STEP_HI,	0x00 },
	{ OV9740_AEC_B50_STEP_LO,	0xe8 },
	{ OV9740_AEC_CTRL0E,		0x03 },
	{ OV9740_AEC_MAXEXPO_50_H,	0x15 },
	{ OV9740_AEC_MAXEXPO_50_L,	0xc6 },
	{ OV9740_AEC_B60_STEP_HI,	0x00 },
	{ OV9740_AEC_B60_STEP_LO,	0xc0 },
	{ OV9740_AEC_CTRL0D,		0x04 },
	{ OV9740_AEC_MAXEXPO_60_H,	0x18 },
	{ OV9740_AEC_MAXEXPO_60_L,	0x20 },

	/* LC */
	{ 0x5842, 0x02 }, { 0x5843, 0x5e }, { 0x5844, 0x04 }, { 0x5845, 0x32 },
	{ 0x5846, 0x03 }, { 0x5847, 0x29 }, { 0x5848, 0x02 }, { 0x5849, 0xcc },

	/* Un-documented OV9740 registers */
	{ 0x5800, 0x29 }, { 0x5801, 0x25 }, { 0x5802, 0x20 }, { 0x5803, 0x21 },
	{ 0x5804, 0x26 }, { 0x5805, 0x2e }, { 0x5806, 0x11 }, { 0x5807, 0x0c },
	{ 0x5808, 0x09 }, { 0x5809, 0x0a }, { 0x580A, 0x0e }, { 0x580B, 0x16 },
	{ 0x580C, 0x06 }, { 0x580D, 0x02 }, { 0x580E, 0x00 }, { 0x580F, 0x00 },
	{ 0x5810, 0x04 }, { 0x5811, 0x0a }, { 0x5812, 0x05 }, { 0x5813, 0x02 },
	{ 0x5814, 0x00 }, { 0x5815, 0x00 }, { 0x5816, 0x03 }, { 0x5817, 0x09 },
	{ 0x5818, 0x0f }, { 0x5819, 0x0a }, { 0x581A, 0x07 }, { 0x581B, 0x08 },
	{ 0x581C, 0x0b }, { 0x581D, 0x14 }, { 0x581E, 0x28 }, { 0x581F, 0x23 },
	{ 0x5820, 0x1d }, { 0x5821, 0x1e }, { 0x5822, 0x24 }, { 0x5823, 0x2a },
	{ 0x5824, 0x4f }, { 0x5825, 0x6f }, { 0x5826, 0x5f }, { 0x5827, 0x7f },
	{ 0x5828, 0x9f }, { 0x5829, 0x5f }, { 0x582A, 0x8f }, { 0x582B, 0x9e },
	{ 0x582C, 0x8f }, { 0x582D, 0x9f }, { 0x582E, 0x4f }, { 0x582F, 0x87 },
	{ 0x5830, 0x86 }, { 0x5831, 0x97 }, { 0x5832, 0xae }, { 0x5833, 0x3f },
	{ 0x5834, 0x8e }, { 0x5835, 0x7c }, { 0x5836, 0x7e }, { 0x5837, 0xaf },
	{ 0x5838, 0x8f }, { 0x5839, 0x8f }, { 0x583A, 0x9f }, { 0x583B, 0x7f },
	{ 0x583C, 0x5f },

	/* Y Gamma */
	{ 0x5480, 0x07 }, { 0x5481, 0x18 }, { 0x5482, 0x2c }, { 0x5483, 0x4e },
	{ 0x5484, 0x5e }, { 0x5485, 0x6b }, { 0x5486, 0x77 }, { 0x5487, 0x82 },
	{ 0x5488, 0x8c }, { 0x5489, 0x95 }, { 0x548A, 0xa4 }, { 0x548B, 0xb1 },
	{ 0x548C, 0xc6 }, { 0x548D, 0xd8 }, { 0x548E, 0xe9 },

	/* UV Gamma */
	{ 0x5490, 0x0f }, { 0x5491, 0xff }, { 0x5492, 0x0d }, { 0x5493, 0x05 },
	{ 0x5494, 0x07 }, { 0x5495, 0x1a }, { 0x5496, 0x04 }, { 0x5497, 0x01 },
	{ 0x5498, 0x03 }, { 0x5499, 0x53 }, { 0x549A, 0x02 }, { 0x549B, 0xeb },
	{ 0x549C, 0x02 }, { 0x549D, 0xa0 }, { 0x549E, 0x02 }, { 0x549F, 0x67 },
	{ 0x54A0, 0x02 }, { 0x54A1, 0x3b }, { 0x54A2, 0x02 }, { 0x54A3, 0x18 },
	{ 0x54A4, 0x01 }, { 0x54A5, 0xe7 }, { 0x54A6, 0x01 }, { 0x54A7, 0xc3 },
	{ 0x54A8, 0x01 }, { 0x54A9, 0x94 }, { 0x54AA, 0x01 }, { 0x54AB, 0x72 },
	{ 0x54AC, 0x01 }, { 0x54AD, 0x57 },

	/* AWB */
	{ OV9740_AWB_CTRL00,		0xf0 },
	{ OV9740_AWB_CTRL01,		0x00 },
	{ OV9740_AWB_CTRL02,		0x41 },
	{ OV9740_AWB_CTRL03,		0x42 },
	{ OV9740_AWB_ADV_CTRL01,	0x8a },
	{ OV9740_AWB_ADV_CTRL02,	0x61 },
	{ OV9740_AWB_ADV_CTRL03,	0xce },
	{ OV9740_AWB_ADV_CTRL04,	0xa8 },
	{ OV9740_AWB_ADV_CTRL05,	0x17 },
	{ OV9740_AWB_ADV_CTRL06,	0x1f },
	{ OV9740_AWB_ADV_CTRL07,	0x27 },
	{ OV9740_AWB_ADV_CTRL08,	0x41 },
	{ OV9740_AWB_ADV_CTRL09,	0x34 },
	{ OV9740_AWB_ADV_CTRL10,	0xf0 },
	{ OV9740_AWB_ADV_CTRL11,	0x10 },
	{ OV9740_AWB_CTRL0F,		0xff },
	{ OV9740_AWB_CTRL10,		0x00 },
	{ OV9740_AWB_CTRL11,		0xff },
	{ OV9740_AWB_CTRL12,		0x00 },
	{ OV9740_AWB_CTRL13,		0xff },
	{ OV9740_AWB_CTRL14,		0x00 },

	/* CIP */
	{ 0x530D, 0x12 },

	/* CMX */
	{ 0x5380, 0x01 }, { 0x5381, 0x00 }, { 0x5382, 0x00 }, { 0x5383, 0x17 },
	{ 0x5384, 0x00 }, { 0x5385, 0x01 }, { 0x5386, 0x00 }, { 0x5387, 0x00 },
	{ 0x5388, 0x00 }, { 0x5389, 0xe0 }, { 0x538A, 0x00 }, { 0x538B, 0x20 },
	{ 0x538C, 0x00 }, { 0x538D, 0x00 }, { 0x538E, 0x00 }, { 0x538F, 0x16 },
	{ 0x5390, 0x00 }, { 0x5391, 0x9c }, { 0x5392, 0x00 }, { 0x5393, 0xa0 },
	{ 0x5394, 0x18 },

	/* 50/60 Detection */
	{ 0x3C0A, 0x9c }, { 0x3C0B, 0x3f },

	/* Output Select */
	{ OV9740_IO_OUTPUT_SEL01,	0x00 },
	{ OV9740_IO_OUTPUT_SEL02,	0x00 },
	{ OV9740_IO_CREL00,		0x00 },
	{ OV9740_IO_CREL01,		0x00 },
	{ OV9740_IO_CREL02,		0x00 },

	/* AWB Control */
	{ OV9740_AWB_MANUAL_CTRL,	0x00 },

	/* Analog Control */
	{ OV9740_ANALOG_CTRL03,		0xaa },
	{ OV9740_ANALOG_CTRL32,		0x2f },
	{ OV9740_ANALOG_CTRL20,		0x66 },
	{ OV9740_ANALOG_CTRL21,		0xc0 },
	{ OV9740_ANALOG_CTRL31,		0x52 },
	{ OV9740_ANALOG_CTRL33,		0x50 },
	{ OV9740_ANALOG_CTRL30,		0xca },
	{ OV9740_ANALOG_CTRL04,		0x0c },
	{ OV9740_ANALOG_CTRL01,		0x40 },
	{ OV9740_ANALOG_CTRL02,		0x16 },
	{ OV9740_ANALOG_CTRL10,		0xa1 },
	{ OV9740_ANALOG_CTRL12,		0x24 },
	{ OV9740_ANALOG_CTRL22,		0x9f },

	/* Sensor Control */
	{ OV9740_SENSOR_CTRL03,		0x42 },
	{ OV9740_SENSOR_CTRL04,		0x10 },
	{ OV9740_SENSOR_CTRL05,		0x45 },
	{ OV9740_SENSOR_CTRL07,		0x14 },

	/* Timing Control */
	{ OV9740_TIMING_CTRL33,		0x04 },
	{ OV9740_TIMING_CTRL35,		0x02 },
	{ OV9740_TIMING_CTRL19,		0x6e },
	{ OV9740_TIMING_CTRL17,		0x94 },

	/* AEC/AGC Control */
	{ OV9740_AEC_ENABLE,		0x10 },
	{ OV9740_GAIN_CEILING_01,	0x00 },
	{ OV9740_GAIN_CEILING_02,	0x7f },
	{ OV9740_AEC_HI_THRESHOLD,	0xa0 },
	{ OV9740_AEC_3A1A,		0x05 },
	{ OV9740_AEC_CTRL1B_WPT2,	0x50 },
	{ OV9740_AEC_CTRL0F_WPT,	0x50 },
	{ OV9740_AEC_CTRL10_BPT,	0x4c },
	{ OV9740_AEC_CTRL1E_BPT2,	0x4c },
	{ OV9740_AEC_LO_THRESHOLD,	0x26 },

	/* BLC Control */
	{ OV9740_BLC_AUTO_ENABLE,	0x45 },
	{ OV9740_BLC_MODE,		0x18 },

	/* DVP Control */
	{ OV9740_DVP_VSYNC_CTRL02,	0x04 },
	{ OV9740_DVP_VSYNC_MODE,	0x00 },
	{ OV9740_DVP_VSYNC_CTRL06,	0x08 },

	/* PLL Setting */
	{ OV9740_PLL_MODE_CTRL01,	0x20 },
	{ OV9740_PRE_PLL_CLK_DIV,	0x03 },
	{ OV9740_PLL_MULTIPLIER,	0x4c },
	{ OV9740_VT_SYS_CLK_DIV,	0x01 },
	{ OV9740_VT_PIX_CLK_DIV,	0x08 },

	/* Timing Setting */
	/* VTS */
	{ OV9740_FRM_LENGTH_LN_HI,	0x03 },
	{ OV9740_FRM_LENGTH_LN_LO,	0x07 },
	/* HTS */
	{ OV9740_LN_LENGTH_PCK_HI,	0x06 },
	{ OV9740_LN_LENGTH_PCK_LO,	0x62 },
};

static const struct reg_tab_wb ov9740_stm_on[] = {
	{OV9740_MODE_SELECT,		0x01},
};

static const struct reg_tab_wb ov9740_stm_off[] = {
	{OV9740_MODE_SELECT,		0x00},
};

static const struct reg_tab_wb ov9740_ort_none[] = {
	{OV9740_IMAGE_ORT,		0x00},
};

static const struct reg_tab_wb ov9740_ort_mirr[] = {
	{OV9740_IMAGE_ORT,		0x01},
};

static const struct reg_tab_wb ov9740_ort_flip[] = {
	{OV9740_IMAGE_ORT,		0x02},
};

static const struct reg_tab_wb ov9740_ort_flipmirr[] = {
	{OV9740_IMAGE_ORT,		0x03},
};

static const struct reg_tab_wb ov9740_fmt_yuyv[] __attribute__((unused)) = {
	{OV9740_DATA_FORMAT,		0x30},
};

static const struct reg_tab_wb ov9740_fmt_yvyu[] __attribute__((unused)) = {
	{OV9740_DATA_FORMAT,		0x31},
};

static const struct reg_tab_wb ov9740_fmt_uyvy[] __attribute__((unused)) = {
	{OV9740_DATA_FORMAT,		0x32},
};

static const struct reg_tab_wb ov9740_fmt_vyuy[] __attribute__((unused)) = {
	{OV9740_DATA_FORMAT,		0x33},
};

static const struct reg_tab_wb ov9740_res_qcif[] = {
	{ OV9740_X_ADDR_START_HI,	0x00 },
	{ OV9740_X_ADDR_START_LO,	0xa8 },
	{ OV9740_Y_ADDR_START_HI,	0x00 },
	{ OV9740_Y_ADDR_START_LO,	0x00 },
	{ OV9740_X_ADDR_END_HI,		0x04 },
	{ OV9740_X_ADDR_END_LO,		0x67 },
	{ OV9740_Y_ADDR_END_HI,		0x02 },
	{ OV9740_Y_ADDR_END_LO,		0xcf },
	{ OV9740_X_OUTPUT_SIZE_HI,	0x00 },
	{ OV9740_X_OUTPUT_SIZE_LO,	0xB0 },
	{ OV9740_Y_OUTPUT_SIZE_HI,	0x00 },
	{ OV9740_Y_OUTPUT_SIZE_LO,	0x90 },
	{ OV9740_ISP_CTRL1E,		0x03 },
	{ OV9740_ISP_CTRL1F,		0xc0 },
	{ OV9740_ISP_CTRL20,		0x02 },
	{ OV9740_ISP_CTRL21,		0xd0 },
	{ OV9740_VFIFO_READ_START_HI,	0x01 },
	{ OV9740_VFIFO_READ_START_LO,	0x50 },
	{ OV9740_ISP_CTRL00,		0xff },
	{ OV9740_ISP_CTRL01,		0xff },
	{ OV9740_ISP_CTRL03,		0xff },
};

static const struct reg_tab_wb ov9740_res_cif[] = {
	{ OV9740_X_ADDR_START_HI,	0x00 },
	{ OV9740_X_ADDR_START_LO,	0xa0 },
	{ OV9740_Y_ADDR_START_HI,	0x00 },
	{ OV9740_Y_ADDR_START_LO,	0x00 },
	{ OV9740_X_ADDR_END_HI,		0x04 },
	{ OV9740_X_ADDR_END_LO,		0x63 },
	{ OV9740_Y_ADDR_END_HI,		0x02 },
	{ OV9740_Y_ADDR_END_LO,		0xd3 },
	{ OV9740_X_OUTPUT_SIZE_HI,	0x01 },
	{ OV9740_X_OUTPUT_SIZE_LO,	0x60 },
	{ OV9740_Y_OUTPUT_SIZE_HI,	0x01 },
	{ OV9740_Y_OUTPUT_SIZE_LO,	0x20 },
	{ OV9740_ISP_CTRL1E,		0x03 },
	{ OV9740_ISP_CTRL1F,		0xc0 },
	{ OV9740_ISP_CTRL20,		0x02 },
	{ OV9740_ISP_CTRL21,		0xd0 },
	{ OV9740_VFIFO_READ_START_HI,	0x01 },
	{ OV9740_VFIFO_READ_START_LO,	0x40 },
	{ OV9740_ISP_CTRL00,		0xff },
	{ OV9740_ISP_CTRL01,		0xff },
	{ OV9740_ISP_CTRL03,		0xff },
};

static const struct reg_tab_wb ov9740_res_hqvga[] = {
	{ OV9740_X_ADDR_START_HI,	0x00 },
	{ OV9740_X_ADDR_START_LO,	0xa0 },
	{ OV9740_Y_ADDR_START_HI,	0x00 },
	{ OV9740_Y_ADDR_START_LO,	0x00 },
	{ OV9740_X_ADDR_END_HI,		0x04 },
	{ OV9740_X_ADDR_END_LO,		0x63 },
	{ OV9740_Y_ADDR_END_HI,		0x02 },
	{ OV9740_Y_ADDR_END_LO,		0xd3 },
	{ OV9740_X_OUTPUT_SIZE_HI,	0x00 },
	{ OV9740_X_OUTPUT_SIZE_LO,	0xF0 },
	{ OV9740_Y_OUTPUT_SIZE_HI,	0x00 },
	{ OV9740_Y_OUTPUT_SIZE_LO,	0xA0 },
	{ OV9740_ISP_CTRL1E,		0x03 },
	{ OV9740_ISP_CTRL1F,		0xc0 },
	{ OV9740_ISP_CTRL20,		0x02 },
	{ OV9740_ISP_CTRL21,		0xd0 },
	{ OV9740_VFIFO_READ_START_HI,	0x01 },
	{ OV9740_VFIFO_READ_START_LO,	0x40 },
	{ OV9740_ISP_CTRL00,		0xff },
	{ OV9740_ISP_CTRL01,		0xff },
	{ OV9740_ISP_CTRL03,		0xff },
};

static const struct reg_tab_wb ov9740_res_qvga[] = {
	{ OV9740_X_ADDR_START_HI,	0x00 },
	{ OV9740_X_ADDR_START_LO,	0xa0 },
	{ OV9740_Y_ADDR_START_HI,	0x00 },
	{ OV9740_Y_ADDR_START_LO,	0x00 },
	{ OV9740_X_ADDR_END_HI,		0x04 },
	{ OV9740_X_ADDR_END_LO,		0x63 },
	{ OV9740_Y_ADDR_END_HI,		0x02 },
	{ OV9740_Y_ADDR_END_LO,		0xd3 },
	{ OV9740_X_OUTPUT_SIZE_HI,	0x01 },
	{ OV9740_X_OUTPUT_SIZE_LO,	0x40 },
	{ OV9740_Y_OUTPUT_SIZE_HI,	0x00 },
	{ OV9740_Y_OUTPUT_SIZE_LO,	0xF0 },
	{ OV9740_ISP_CTRL1E,		0x03 },
	{ OV9740_ISP_CTRL1F,		0xc0 },
	{ OV9740_ISP_CTRL20,		0x02 },
	{ OV9740_ISP_CTRL21,		0xd0 },
	{ OV9740_VFIFO_READ_START_HI,	0x01 },
	{ OV9740_VFIFO_READ_START_LO,	0x40 },
	{ OV9740_ISP_CTRL00,		0xff },
	{ OV9740_ISP_CTRL01,		0xff },
	{ OV9740_ISP_CTRL03,		0xff },
};

static const struct reg_tab_wb ov9740_res_vga[] = {
	{ OV9740_X_ADDR_START_HI,	0x00 },
	{ OV9740_X_ADDR_START_LO,	0xa0 },
	{ OV9740_Y_ADDR_START_HI,	0x00 },
	{ OV9740_Y_ADDR_START_LO,	0x00 },
	{ OV9740_X_ADDR_END_HI,		0x04 },
	{ OV9740_X_ADDR_END_LO,		0x63 },
	{ OV9740_Y_ADDR_END_HI,		0x02 },
	{ OV9740_Y_ADDR_END_LO,		0xd3 },
	{ OV9740_X_OUTPUT_SIZE_HI,	0x02 },
	{ OV9740_X_OUTPUT_SIZE_LO,	0x80 },
	{ OV9740_Y_OUTPUT_SIZE_HI,	0x01 },
	{ OV9740_Y_OUTPUT_SIZE_LO,	0xe0 },
	{ OV9740_ISP_CTRL1E,		0x03 },
	{ OV9740_ISP_CTRL1F,		0xc0 },
	{ OV9740_ISP_CTRL20,		0x02 },
	{ OV9740_ISP_CTRL21,		0xd0 },
	{ OV9740_VFIFO_READ_START_HI,	0x01 },
	{ OV9740_VFIFO_READ_START_LO,	0x40 },
	{ OV9740_ISP_CTRL00,		0xff },
	{ OV9740_ISP_CTRL01,		0xff },
	{ OV9740_ISP_CTRL03,		0xff },
};

static const struct reg_tab_wb ov9740_res_360p[] = {
	{ OV9740_X_ADDR_START_HI,	0x00 },
	{ OV9740_X_ADDR_START_LO,	0x00 },
	{ OV9740_Y_ADDR_START_HI,	0x00 },
	{ OV9740_Y_ADDR_START_LO,	0x00 },
	{ OV9740_X_ADDR_END_HI,		0x05 },
	{ OV9740_X_ADDR_END_LO,		0x03 },
	{ OV9740_Y_ADDR_END_HI,		0x02 },
	{ OV9740_Y_ADDR_END_LO,		0xd3 },
	{ OV9740_X_OUTPUT_SIZE_HI,	0x02 },
	{ OV9740_X_OUTPUT_SIZE_LO,	0x80 },
	{ OV9740_Y_OUTPUT_SIZE_HI,	0x01 },
	{ OV9740_Y_OUTPUT_SIZE_LO,	0x68 },
	{ OV9740_ISP_CTRL1E,		0x05 },
	{ OV9740_ISP_CTRL1F,		0x00 },
	{ OV9740_ISP_CTRL20,		0x02 },
	{ OV9740_ISP_CTRL21,		0xd0 },
	{ OV9740_VFIFO_READ_START_HI,	0x02 },
	{ OV9740_VFIFO_READ_START_LO,	0x30 },
	{ OV9740_ISP_CTRL00,		0xff },
	{ OV9740_ISP_CTRL01,		0xef },
	{ OV9740_ISP_CTRL03,		0xff },
};

static const struct reg_tab_wb ov9740_res_720p[] = {
	{ OV9740_X_ADDR_START_HI,	0x00 },
	{ OV9740_X_ADDR_START_LO,	0x00 },
	{ OV9740_Y_ADDR_START_HI,	0x00 },
	{ OV9740_Y_ADDR_START_LO,	0x00 },
	{ OV9740_X_ADDR_END_HI,		0x05 },
	{ OV9740_X_ADDR_END_LO,		0x03 },
	{ OV9740_Y_ADDR_END_HI,		0x02 },
	{ OV9740_Y_ADDR_END_LO,		0xd3 },
	{ OV9740_X_OUTPUT_SIZE_HI,	0x05 },
	{ OV9740_X_OUTPUT_SIZE_LO,	0x00 },
	{ OV9740_Y_OUTPUT_SIZE_HI,	0x02 },
	{ OV9740_Y_OUTPUT_SIZE_LO,	0xd0 },
	{ OV9740_ISP_CTRL1E,		0x05 },
	{ OV9740_ISP_CTRL1F,		0x00 },
	{ OV9740_ISP_CTRL20,		0x02 },
	{ OV9740_ISP_CTRL21,		0xd0 },
	{ OV9740_VFIFO_READ_START_HI,	0x02 },
	{ OV9740_VFIFO_READ_START_LO,	0x30 },
	{ OV9740_ISP_CTRL00,		0xff },
	{ OV9740_ISP_CTRL01,		0xef },
	{ OV9740_ISP_CTRL03,		0xff },
};

static const struct reg_tab_wb ov9740_mipi_1lane_def[] = {
	/* 330Mhz MIPI clock */
	/* PLL1 scale divider Was 0x01, change to 0x81 to
	 * disable MIPI clock auto tweaking */
	{ OV9740_PLL_CTRL3010,		0x81 },
	{ OV9740_VFIFO_CTRL00,		0x82 },
	/* MIPI Control */
	{ OV9740_MIPI_CTRL00,		0x44 },
	{ OV9740_MIPI_3837,		0x01 },
	{ OV9740_MIPI_CTRL01,		0x0f },
	{ OV9740_MIPI_CTRL03,		0x05 },
	{ OV9740_MIPI_CTRL05,		0x10 },
	{ OV9740_VFIFO_RD_CTRL,		0x16 },
	{ OV9740_MIPI_CTRL_3012,	0x70 },
	{ OV9740_SC_CMMM_MIPI_CTR,	0x01 },
};

static const struct reg_tab_wb ov9740_mipi_2lane_def[] = {
	/* 2lane 330Mhz MIPI clock */
	{ OV9740_IO_MIPI2L,		0x43 },
	/* PLL1 scale divider Was 0x01, change to 0x81 to
	 * disable MIPI clock auto tweaking */
	{ OV9740_PLL_CTRL3010,		0x81 },
	{ OV9740_VFIFO_CTRL00,		0x82 },
	/* MIPI Control */
	{ OV9740_MIPI_CTRL00,		0x44 },
	{ OV9740_MIPI_3837,		0x01 },
	{ OV9740_MIPI_CTRL01,		0x0f },
	{ OV9740_MIPI_CTRL03,		0x05 },
	{ OV9740_MIPI_CTRL05,		0x10 },
	{ OV9740_VFIFO_RD_CTRL,		0x16 },
	{ OV9740_MIPI_CTRL_3012,	0x70 },
	{ OV9740_SC_CMMM_MIPI_CTR,	0x01 },
};
/**************************** register section end ****************************/

enum {
	OV9740_PROP_INIT,		/* Initialize sequence */
	OV9740_PROP_STM,		/* Stream on/off */
	OV9740_PROP_ORT,		/* Image orientation */
	OV9740_PROP_FMT,		/* Output format */
	OV9740_PROP_RES,		/* Output resolution */
	OV9740_PROP_IF,			/* Interface behavior */
	OV9740_PROP_MIPI = OV9740_PROP_IF,	/* MIPI timing */
	OV9740_PROP_END,
};

enum {
	OV9740_INIT_REQ	= 0,
	OV9740_INIT_DONE,
	OV9740_INIT_END,	/* don't use it*/
};

enum {
	OV9740_STM_OFF	= 0,
	OV9740_STM_ON,
	OV9740_STM_END,	/* don't use it*/
};

enum {
	OV9740_ORT_NONE	= 0,
	OV9740_ORT_MIRR,
	OV9740_ORT_FLIP,
	OV9740_ORT_FLIPMIRR,
	OV9740_ORT_END,	/* don't use it*/
};

enum {
	OV9740_FMT_YUYV	= 0,
	OV9740_FMT_YVYU,
	OV9740_FMT_UYVY,
	OV9740_FMT_VYUY,
	OV9740_FMT_END,	/* don't use it*/
};

enum {	/* 0x3808~0x380F */
	OV9740_RES_QCIF = 0,
	OV9740_RES_HQVGA,
	OV9740_RES_QVGA,
	OV9740_RES_CIF,
	OV9740_RES_VGA,
	OV9740_RES_360P,
	OV9740_RES_720P,
	OV9740_RES_END,	/* don't use it*/
};

enum {	/* 0x4800~0x48FF */
	OV9740_IF_DEFAULT	= 0,
	OV9740_IF_END,

	OV9740_MIPI_1LANE_DEF	= OV9740_IF_DEFAULT,
	OV9740_MIPI_1LANE_END	= OV9740_IF_END,

	OV9740_MIPI_2LANE_DEF	= OV9740_IF_DEFAULT,
	OV9740_MIPI_2LANE_END	= OV9740_IF_END,
};

enum {	/* 0x4800~0x48FF */
	OV9740_ST_INVALID	= 0, /* place holder, don't use 0 as state id*/
	OV9740_ST_UYVY_QCIF,
	OV9740_ST_UYVY_CIF,
	OV9740_ST_UYVY_HQVGA,
	OV9740_ST_UYVY_QVGA,
	OV9740_ST_UYVY_VGA,
	OV9740_ST_UYVY_360P,
	OV9740_ST_UYVY_720P,
	OV9740_ST_END,	/* don't use it*/
};

/********************************** settings **********************************/
static struct ecs_default_fmt_info ov9740_fmt_info_table[] = {
	[OV9740_FMT_YUYV] = {
		.code	= V4L2_MBUS_FMT_YUYV8_2X8,
		.clrspc	= V4L2_COLORSPACE_JPEG,
		.fourcc	= V4L2_PIX_FMT_YUYV,
	},
	[OV9740_FMT_YVYU] = {
		.code	= V4L2_MBUS_FMT_YVYU8_2X8,
		.clrspc	= V4L2_COLORSPACE_JPEG,
		.fourcc	= V4L2_PIX_FMT_YVYU,
	},
	[OV9740_FMT_UYVY] = {
		.code	= V4L2_MBUS_FMT_UYVY8_2X8,
		.clrspc	= V4L2_COLORSPACE_JPEG,
		.fourcc	= V4L2_PIX_FMT_UYVY,
	},
	[OV9740_FMT_VYUY] = {
		.code	= V4L2_MBUS_FMT_VYUY8_2X8,
		.clrspc	= V4L2_COLORSPACE_JPEG,
		.fourcc	= V4L2_PIX_FMT_VYUY,
	},
};

static struct ecs_default_res_info ov9740_res_info_table[] = {
	[OV9740_RES_QCIF] = {
		.h_act	= 176,
		.v_act	= 144,
	},
	[OV9740_RES_CIF] = {
		.h_act	= 352,
		.v_act	= 288,
	},
	[OV9740_RES_HQVGA] = {
		.h_act	= 240,
		.v_act	= 160,
	},
	[OV9740_RES_QVGA] = {
		.h_act	= 320,
		.v_act	= 240,
	},
	[OV9740_RES_VGA] = {
		.h_act	= 640,
		.v_act	= 480,
	},
	[OV9740_RES_360P] = {
		.h_act	= 640,
		.v_act	= 360,
	},
	[OV9740_RES_720P] = {
		.h_act	= 1280,
		.v_act	= 720,
	},
};

#ifdef CONFIG_PXA95x
static struct mipi_phy ov9740_mipi_1lane_info_table[] = {
	{ /* OV9740 default mipi PHY config, based on 330Mhz MIPI clock */
		.cl_termen	= 0x00,
		.cl_settle	= 0x0C,
		.hs_termen	= 0x0D,
		.hs_settle	= 0x19,
		.hs_rx_to	= 0xFFFF,
		.lane		= 1,
	},
	{ /* For 160Mhz */
		.cl_termen	= 0x00,
		.cl_settle	= 0x0C,
		.hs_termen	= 0x07,
		.hs_settle	= 0x0D,
		.hs_rx_to	= 0xFFFF,
		.lane		= 1,
	},
	{ /* For 80Mhz */
		.cl_termen	= 0x00,
		.cl_settle	= 0x0C,
		.hs_termen	= 0x03,
		.hs_settle	= 0x0B,
		.hs_rx_to	= 0xFFFF,
		.lane		= 1,
	},
	{/* This phy configuration is workable on 100~330Mhz clock */
		.cl_termen	= 0x00,
		.cl_settle	= 0x0C,
		.hs_termen	= 0x08,
		.hs_settle	= 0x10,
		.hs_rx_to	= 0xFFFF,
		.lane		= 1,
	}
};
#endif

#define OV9740_DECLARE_INIT_SETTING(VAL, val) \
	__DECLARE_SETTING(OV9740, ov9740, INIT, init, VAL, val)
static struct ecs_setting ov9740_init_stn_table[OV9740_INIT_END] = {
	OV9740_DECLARE_INIT_SETTING(REQ, req),
	OV9740_DECLARE_INIT_SETTING(DONE, done),
};

#define OV9740_DECLARE_STM_SETTING(VAL, val) \
	__DECLARE_SETTING(OV9740, ov9740, STM, stm, VAL, val)
static struct ecs_setting ov9740_stm_stn_table[OV9740_STM_END] = {
	OV9740_DECLARE_STM_SETTING(OFF, off),
	OV9740_DECLARE_STM_SETTING(ON, on),
};

#define OV9740_DECLARE_ORT_SETTING(VAL, val) \
	__DECLARE_SETTING(OV9740, ov9740, ORT, ort, VAL, val)
static struct ecs_setting ov9740_ort_stn_table[OV9740_ORT_END] = {
	OV9740_DECLARE_ORT_SETTING(NONE, none),
	OV9740_DECLARE_ORT_SETTING(MIRR, mirr),
	OV9740_DECLARE_ORT_SETTING(FLIP, flip),
	OV9740_DECLARE_ORT_SETTING(FLIPMIRR, flipmirr),
};

#define OV9740_DECLARE_FMT_SETTING(VAL, val) \
	__DECLARE_SETTING_VS_INFO(OV9740, ov9740, FMT, fmt, VAL, val)
static struct ecs_setting ov9740_fmt_stn_table[OV9740_FMT_END] = {
	OV9740_DECLARE_FMT_SETTING(YUYV, yuyv),
	OV9740_DECLARE_FMT_SETTING(YVYU, yvyu),
	OV9740_DECLARE_FMT_SETTING(UYVY, uyvy),
	OV9740_DECLARE_FMT_SETTING(VYUY, vyuy),
};

#define OV9740_DECLARE_RES_SETTING(VAL, val) \
	__DECLARE_SETTING_VS_INFO(OV9740, ov9740, RES, res, VAL, val)
static struct ecs_setting ov9740_res_stn_table[OV9740_RES_END] = {
	OV9740_DECLARE_RES_SETTING(QCIF, qcif),
	OV9740_DECLARE_RES_SETTING(CIF,	cif),
	OV9740_DECLARE_RES_SETTING(HQVGA, hqvga),
	OV9740_DECLARE_RES_SETTING(QVGA, qvga),
	OV9740_DECLARE_RES_SETTING(VGA, vga),
	OV9740_DECLARE_RES_SETTING(360P, 360p),
	OV9740_DECLARE_RES_SETTING(720P, 720p),
};

#define OV9740_DECLARE_MIPI_1LANE_SETTING(VAL, val) \
	__DECLARE_SETTING_VS_INFO(OV9740, ov9740, MIPI_1LANE, mipi_1lane, \
								VAL, val)
#ifdef CONFIG_PXA95x
static struct ecs_setting ov9740_mipi_1lane_stn_table[OV9740_MIPI_1LANE_END] = {
	OV9740_DECLARE_MIPI_1LANE_SETTING(DEF, def),
};
#endif

#define OV9740_DECLARE_MIPI_2LANE_SETTING(VAL, val) \
	__DECLARE_SETTING(OV9740, ov9740, MIPI_2LANE, mipi_2lane, VAL, val)
static struct ecs_setting ov9740_mipi_2lane_stn_table[OV9740_MIPI_2LANE_END] = {
	OV9740_DECLARE_MIPI_2LANE_SETTING(DEF, def),
};


/********************************* properties *********************************/
#define OV9740_DECLARE_PROP(PPT, ppt, NAME, REGH, REGL, SPEC) \
	__DECLARE_PROPERTY(OV9740, ov9740, PPT, ppt, NAME, REGH, REGL, SPEC)
static struct ecs_property ov9740_property_table[OV9740_PROP_END] = {
	OV9740_DECLARE_PROP(INIT, init,	INITIALIZE,	0,	0,	1),
	OV9740_DECLARE_PROP(STM, stm,	STREAM,		0,	0,	1),
	OV9740_DECLARE_PROP(ORT, ort,	ORIENTATION,	0,	0,	1),
	OV9740_DECLARE_PROP(FMT, fmt,	FORMAT,		0,	0,	1),
	OV9740_DECLARE_PROP(RES, res,	RESOLUTION,	0,	0,	1),
	__DEVLARE_VIRTUAL_PROPERTY(OV9740, IF, INTERFACE, 1),
};

/*********************************** states ***********************************/
static struct ecs_state_cfg ov9740_state_uyvy_qcif[] = {
	{OV9740_PROP_INIT,	OV9740_INIT_DONE},
	{OV9740_PROP_FMT,	OV9740_FMT_UYVY},
	{OV9740_PROP_RES,	OV9740_RES_QCIF},
	{OV9740_PROP_IF,	OV9740_IF_DEFAULT},
};

static struct ecs_state_cfg ov9740_state_uyvy_cif[] = {
	{OV9740_PROP_INIT,	OV9740_INIT_DONE},
	{OV9740_PROP_FMT,	OV9740_FMT_UYVY},
	{OV9740_PROP_RES,	OV9740_RES_CIF},
	{OV9740_PROP_IF,	OV9740_IF_DEFAULT},
};

static struct ecs_state_cfg ov9740_state_uyvy_qvga[] = {
	{OV9740_PROP_INIT,	OV9740_INIT_DONE},
	{OV9740_PROP_FMT,	OV9740_FMT_UYVY},
	{OV9740_PROP_RES,	OV9740_RES_QVGA},
	{OV9740_PROP_IF,	OV9740_IF_DEFAULT},
};

static struct ecs_state_cfg ov9740_state_uyvy_vga[] = {
	{OV9740_PROP_INIT,	OV9740_INIT_DONE},
	{OV9740_PROP_FMT,	OV9740_FMT_UYVY},
	{OV9740_PROP_RES,	OV9740_RES_VGA},
	{OV9740_PROP_IF,	OV9740_IF_DEFAULT},
};

static struct ecs_state_cfg ov9740_state_uyvy_hqvga[] = {
	{OV9740_PROP_INIT,	OV9740_INIT_DONE},
	{OV9740_PROP_FMT,	OV9740_FMT_UYVY},
	{OV9740_PROP_RES,	OV9740_RES_HQVGA},
	{OV9740_PROP_IF,	OV9740_IF_DEFAULT},
};

static struct ecs_state_cfg ov9740_state_uyvy_360p[] = {
	{OV9740_PROP_INIT,	OV9740_INIT_DONE},
	{OV9740_PROP_FMT,	OV9740_FMT_UYVY},
	{OV9740_PROP_RES,	OV9740_RES_360P},
	{OV9740_PROP_IF,	OV9740_IF_DEFAULT},
};

static struct ecs_state_cfg ov9740_state_uyvy_720p[] = {
	{OV9740_PROP_INIT,	OV9740_INIT_DONE},
	{OV9740_PROP_FMT,	OV9740_FMT_UYVY},
	{OV9740_PROP_RES,	OV9740_RES_720P},
	{OV9740_PROP_IF,	OV9740_IF_DEFAULT},
};

#define OV9740_DECLARE_STATE(VAL, val) \
	__DECLARE_STATE(OV9740, ov9740, VAL, val)
static struct ecs_state ov9740_state_table[OV9740_ST_END] = {
	OV9740_DECLARE_STATE(UYVY_QCIF, uyvy_qcif),
	OV9740_DECLARE_STATE(UYVY_CIF, uyvy_cif),
	OV9740_DECLARE_STATE(UYVY_HQVGA, uyvy_hqvga),
	OV9740_DECLARE_STATE(UYVY_QVGA, uyvy_qvga),
	OV9740_DECLARE_STATE(UYVY_VGA, uyvy_vga),
	OV9740_DECLARE_STATE(UYVY_360P, uyvy_360p),
	OV9740_DECLARE_STATE(UYVY_720P, uyvy_720p),
};

static struct x_i2c ov9740_xic = {
	.ident_addr	= {OV9740_MODEL_ID_HI, OV9740_MODEL_ID_LO},
	.ident_mask	= {0xFF, 0xFF},
	.ident_data	= {0x97, 0x40},
	.ident_regs	= 2,
	.reset_addr	= OV9740_SOFTWARE_RESET,
	.reset_mask	= 0x01,
	.read		= xic_read_wb,
	.write		= xic_write_wb,
	.write_array	= xic_write_array_wb,
	.write_burst	= xic_write_burst_wb,
	.detect		= xic_detect_wb,
};

/* This struct is acually the code to instantize ECS to ov9740 driver */
static struct ecs_sensor generic_ov9740 = {
	.name		= "generic-ov9740",
	.speculate	= 1,
	.prop_tab	= ov9740_property_table,
	.prop_num	= OV9740_PROP_END,
	.state_tab	= ov9740_state_table,
	.state_num	= OV9740_ST_END,
	.state_now	= UNSET,
	.hw_ctx		= &ov9740_xic,
};

static struct ecs_property nevo_spec_prop[] = {
	{
		.name		= "MIPI_1LANE",
		.id		= OV9740_PROP_MIPI,
		.stn_tab	= ov9740_mipi_1lane_stn_table,
		.stn_num	= OV9740_MIPI_1LANE_END,
		.speculate	= 1,
	},
};

static struct ecs_property mmp3_spec_prop[] = {
	{
		.name		= "MIPI_2LANE",
		.id		= OV9740_PROP_MIPI,
		.stn_tab	= ov9740_mipi_2lane_stn_table,
		.stn_num	= OV9740_MIPI_2LANE_END,
		.speculate	= 1,
	},
};

static struct ecs_sensor nevo_spec __attribute__((unused)) = {
	.name		= "nevo specific settings",
	.prop_tab	= nevo_spec_prop,
	.prop_num	= ARRAY_SIZE(nevo_spec_prop),
};

static struct ecs_sensor mmp3_spec __attribute__((unused)) = {
	.name		= "mmp3 specific settings",
	.prop_tab	= mmp3_spec_prop,
	.prop_num	= ARRAY_SIZE(mmp3_spec_prop),
};
/********************************** ECS done **********************************/

/* x_subdev related */
/* The supported format*resolutions mapping table */
static struct xsd_cid ov9740_cid_list[] = {
	{
		.cid = V4L2_CID_PRIVATE_GET_MIPI_PHY,
		.prop = {OV9740_PROP_IF},
	},
	{
		.cid = V4L2_CID_VFLIP,
		.prop = {OV9740_PROP_ORT, OV9740_ORT_NONE},
	},
	{
		.cid = V4L2_CID_HFLIP,
		.prop = {OV9740_PROP_ORT, OV9740_ORT_NONE},
	},
};
static __attribute__((unused)) int ov9740_state_list[] = {
	OV9740_ST_UYVY_QCIF,
	OV9740_ST_UYVY_CIF,
	OV9740_ST_UYVY_HQVGA,
	OV9740_ST_UYVY_QVGA,
	OV9740_ST_UYVY_VGA,
	OV9740_ST_UYVY_360P,
	OV9740_ST_UYVY_720P,
};
static struct v4l2_mbus_framefmt ov9740_fmt_map[OV9740_ST_END];
static int ov9740_enum_map[OV9740_FMT_END * 2];

static struct x_subdev ov9740_xsd = {
	.ecs		= &generic_ov9740,
	.state_list	= ov9740_state_list,
	.state_cnt	= ARRAY_SIZE(ov9740_state_list),
	.cid_list	= ov9740_cid_list,
	.cid_cnt	= ARRAY_SIZE(ov9740_cid_list),
	.state_map	= ov9740_fmt_map,
	.enum_map	= ov9740_enum_map,
	.init_id	= OV9740_PROP_INIT,
	.fmt_id		= OV9740_PROP_FMT,
	.res_id		= OV9740_PROP_RES,
	.str_id		= OV9740_PROP_STM,
	.get_fmt_code	= &ecs_subdev_default_get_fmt_code,
	.get_res_desc	= &ecs_subdev_default_get_res_desc,
};
/********************************** xsd done **********************************/

struct ov9740 {
	struct x_subdev xsd;

	int				ident;
	u16				model;
	u8				revision;
	u8				manid;
	u8				smiaver;

	bool				flag_vflip;
	bool				flag_hflip;
	int				res_idx;
};

static inline struct ov9740 *to_ov9740(const struct i2c_client *client)
{
	struct x_subdev *xsd = container_of(i2c_get_clientdata(client), \
					struct x_subdev, subdev);
	return container_of(xsd, struct ov9740, xsd);
}

/* Alter bus settings on camera side */
static int ov9740_set_bus_param(struct soc_camera_device *icd,
				unsigned long flags)
{
	return 0;
}

/* Request bus settings on camera side */
static unsigned long ov9740_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8 |
		SOCAM_MIPI | SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE;

	/* If soc_camera_link::priv is pointing to sensor_platform_data */
	/* copy sensor_platform_data::interface to soc_camera_link::flags */
	if (icl->flags & 0x80000000) {
		struct sensor_platform_data *sensor;
		sensor = icl->priv;
		icl->flags |= sensor->interface;
	}
	return soc_camera_apply_sensor_flags(icl, flags);
}

static struct soc_camera_ops ov9740_ops = {
	.set_bus_param		= ov9740_set_bus_param,
	.query_bus_param	= ov9740_query_bus_param,
};

static int ov9740_video_probe(struct soc_camera_device *icd,
			      struct i2c_client *client)
{
	struct ov9740 *priv = to_ov9740(client);
	int ret;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface) {
		dev_err(&client->dev, "Parent missing or invalid!\n");
		ret = -ENODEV;
		goto err;
	}

	ret = (*ov9740_xic.detect)(&ov9740_xic);
	if (ret)
		goto err;
	dev_err(&client->dev, "OmniVision ov9740 sensor detected\n");
	priv->ident = V4L2_IDENT_OV9740;

err:
	return ret;
}

/*
 * i2c_driver function
 */
static int ov9740_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov9740 *ov9740;
	struct soc_camera_device *icd	= client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "Missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}

	ov9740 = kzalloc(sizeof(struct ov9740), GFP_KERNEL);
	if (!ov9740) {
		dev_err(&client->dev, "Failed to allocate private data!\n");
		return -ENOMEM;
	}

	ov9740->xsd = ov9740_xsd;	/* setup xsd based on generic */
	v4l2_i2c_subdev_init(&ov9740->xsd.subdev, client, &ecs_subdev_ops);

	icd->ops = &ov9740_ops;

	ov9740_xic.client = v4l2_get_subdevdata(&ov9740->xsd.subdev);

	ret = ov9740_video_probe(icd, client);
	if (ret)
		goto out;

#ifdef CONFIG_PXA95x
	/* use mipi 1lane as interface */
	ret = ecs_sensor_merge(ov9740->xsd.ecs, &nevo_spec);
	if (ret < 0)
		goto out;
#endif
#ifdef CONFIG_CPU_MMP3
	/* use MMP3 profile */
	ret = ecs_sensor_merge(ov9740->xsd.ecs, &mmp3_spec);
	if (ret < 0)
		goto out;
#endif
	/* ECS driver setup */
	ov9740->xsd.ecs = &generic_ov9740;
	ecs_subdev_init(&ov9740->xsd);
	ecs_sensor_reset(ov9740->xsd.ecs);

	return ret;

out:
	icd->ops = NULL;
	kfree(ov9740);
	return ret;
}

static int ov9740_remove(struct i2c_client *client)
{
	struct ov9740 *priv = i2c_get_clientdata(client);

	ecs_subdev_remove(&priv->xsd);
	kfree(priv);

	return 0;
}

static const struct i2c_device_id ov9740_id[] = {
	{ "ov9740", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9740_id);

static struct i2c_driver ov9740_i2c_driver = {
	.driver = {
		.name = "ov9740",
	},
	.probe    = ov9740_probe,
	.remove   = ov9740_remove,
	.id_table = ov9740_id,
};

static int __init ov9740_module_init(void)
{
	return i2c_add_driver(&ov9740_i2c_driver);
}

static void __exit ov9740_module_exit(void)
{
	i2c_del_driver(&ov9740_i2c_driver);
}

module_init(ov9740_module_init);
module_exit(ov9740_module_exit);

MODULE_DESCRIPTION("SoC Camera driver for OmniVision OV9740 based on ECS");
MODULE_AUTHOR("Jiaquan Su <achew@nvidia.com>");
MODULE_LICENSE("GPL v2");
