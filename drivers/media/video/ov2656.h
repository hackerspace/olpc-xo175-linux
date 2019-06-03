#ifndef OV2656_H_
#define OV2656_H_

#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>

#define REG_PIDH	0x300a
#define REG_PIDL	0x300b

#define REG_SYS		0x3012
#define SYS_RESET	(1<<7)
#define REG_TM11	0x3086
#define SYS_SWPD	(1<<0)

#define OV2656_END_ADDR		0xFFFF
#define OV2656_END_VAL		0xFF
#define END_SYMBOL		{OV2656_END_ADDR, OV2656_END_VAL}

struct regval_list {
	u16 reg_num;
	unsigned char value;
};

struct resv_size {
	int width;
	int height;
};

#define RESV(x)		(1 << (x))
enum ov2656_resv_support {
	OV2656_FMT_QCIF = 1,
	OV2656_FMT_QHVGA,
	OV2656_FMT_QVGA,
	OV2656_FMT_CIF,
	OV2656_FMT_HALF_VGA,
	OV2656_FMT_VGA,
	OV2656_FMT_D1,
	OV2656_FMT_WVGA,
	OV2656_FMT_SVGA,
	OV2656_FMT_720P,
	OV2656_FMT_UXVGA,	/* 1600*1200 */

	OV2656_FMT_END,
};

struct ov2656_format {
	enum v4l2_mbus_pixelcode	code;
	unsigned int fmt_support;
	struct regval_list	*regs;
	struct regval_list	*def_set;
	struct regval_list	*color_effect;
};

struct ov2656_mipi {
	struct regval_list *mipi_set_regs;
	struct regval_list *lane1_regs;
	struct regval_list *lane2_regs;
};

struct ov2656_win_size {
	enum ov2656_resv_support	resv;
	struct regval_list *regs;

	struct regval_list *regs_resolution;

	/* Update mipi clock setting to fix fps downgrade issue
	   on brownstone rev5 boards */
	struct regval_list  *lane_set;
};

struct ov2656_config {
	const char name[32];
	struct ov2656_format *fmt;
	int fmt_size;
	struct ov2656_mipi *mipi_lane;
	struct ov2656_win_size *yuv_res;
	int yuv_res_size;
	struct regval_list	*init;
};

struct ov2656 {
	struct v4l2_subdev subdev;
	int model;	/* V4L2_IDENT_OV2656* codes from v4l2-chip-ident.h */
	struct v4l2_rect rect;
	u32 pixfmt;
	struct i2c_client *client;
	struct soc_camera_device icd;
	struct regval_list *init;
	struct regval_list *regs_fmt;
	struct regval_list *regs_size;
	struct regval_list *regs_default;
	struct regval_list *regs_lane_set;
	struct regval_list *regs_resolution;
	struct regval_list *regs_mipi_set;
	struct regval_list *regs_mipi_lane;
	struct regval_list *regs_color_effect;
};

/* ov2656 has only one fixed colorspace per pixelcode */
struct ov2656_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

extern int ov2656_read(struct i2c_client *c, u16 reg, unsigned char *value);
extern int ov2656_write(struct i2c_client *c, u16 reg, unsigned char value);
extern int ov2656_select_bus_type(const char *name);
extern int ov2656_set_yuv_res_array(struct v4l2_frmsizeenum *fsize);
extern struct regval_list *ov2656_get_global_init_regs(void);
extern struct regval_list *ov2656_get_fmt_regs(enum v4l2_mbus_pixelcode code,
					int width, int height);
extern struct regval_list *ov2656_get_fmt_default_setting( \
						enum v4l2_mbus_pixelcode code);
extern struct regval_list *ov2656_get_yuv_size_regs(int width, int height);
extern struct regval_list *ov2656_get_yuv_lane_set(int width, int height);
extern struct ov2656_win_size *ov2656_get_yuv_size_array(void);
extern struct regval_list *ov2656_get_mipi_set_regs(void);
extern struct regval_list *ov2656_get_mipi_lane_regs(int num);
extern struct regval_list *ov2656_get_yuv_resolution_regs(int width, int height);
extern struct regval_list *ov2656_get_global_color_effect(void);
#endif
