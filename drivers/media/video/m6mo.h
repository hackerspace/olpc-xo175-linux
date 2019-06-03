#ifndef _M6MO_H_
#define _M6MO_H_

#include <mach/camera.h>

enum {
	ISP_REG_READ		= 1,
	ISP_REG_WRITE		= 2,
	ISP_MEM_READ_8		= 3,
	ISP_MEM_WRITE_8		= 4,
	ISP_MEM_READ_16		= 5,
	ISP_MEM_WRITE_16	= 6,
	ISP_MEM_READ_32		= 7,
	ISP_MEM_WRITE_32	= 8,
};

enum _m6mo_state {
	M6MO_UNINIT		= 0,
	M6MO_SETUP,
	M6MO_PRE_MNT,
	M6MO_MONITOR,
	M6MO_PRE_JPG,
	M6MO_CAP_JPG,
	M6MO_CAP_YUV,
};

/*
00 SYSP
01 MNIA
02 MNIB
03 EXPO
06 WBCT
07 EXIF
0A AFCT
0B STPP
0C STPC
0D SNSR
0F FLSH
*/
enum category_parameter {
/* Category 0: System parameter */
	SYSP_CUSTOMER_CODE	= 0x0000,
	SYSP_PROJECT_CODE	= 0x0001,
	SYSP_MODE		= 0x000B,
		MODE_INIT		= 0,
		MODE_SETIN		= 1,
		MODE_MONIT		= 2,
		MODE_STILL		= 3,
	SYSP_STATUS		= 0x000C,
		STAT_INIT		= 0,
		STAT_SETIN		= 1,
		STAT_MONIT		= 2,
		STAT_FOCUS		= 3,
		STAT_FACED		= 4,
		STAT_MTCAP		= 5,
		STAT_SGCAP		= 6,
		STAT_PREVW		= 7,
	SYSP_IRQ_EN		= 0x0010,
	SYSP_IRQ_ROOT		= 0x0012,
	SYSP_IRQ_STATE		= 0x001C,
		IS_MODE			= (1<<0),
		IS_AF			= (1<<1),
		IS_ZOOM			= (1<<2),
		IS_CAP			= (1<<3),
		IS_SYNC			= (1<<4),
		IS_FD			= (1<<5),
		IS_LENS			= (1<<6),
		IS_SOUND		= (1<<7),

/* Category 1: Monitor and still parameter A */
	MONT_OUTPUT_IF		= 0x0100,
		OUTPUT_YUV		= 0,
		OUTPUT_HDMI		= 1,
		OUTPUT_MIPI		= 2,
	MONT_SIZE		= 0x0101,
		SIZE_QCIF		= 0x05,
		SIZE_CIF		= 0x0E,
		SIZE_QVGA		= 0x09,
		SIZE_VGA_30FPS		= 0x17,
		SIZE_SVGA		= 0x1F,	/* 800X600*/
		SIZE_720P		= 0x21,
		SIZE_1080P		= 0x28,
		SIZE_VGA_60FPS		= 0x32,
	MONT_FPS		= 0x0102,
		FPS_AUTO		= 0x01,
		FPS_30			= 0x02,
		FPS_15			= 0x03,
		FPS_12			= 0x04,
		FPS_10			= 0x05,
		FPS_7_5			= 0x06,
	MONT_SHD		= 0x0106,
		SHD_OFF			= 0,
		SHD_ON			= 1,
		SHD_MANUAL		= 2,
	MONT_ROTATE		= 0x012D,
		ROT_NONE		= 0,
		ROT_CLK		= 1,
		ROT_ANTICLK		= 2,
/* Category 1: Monitor and still parameter B */
	MONT_ZOOM_SET		= 0x0201,
	MONT_ZOOM_POS		= 0x0202,
	MONT_ZOOM_STEP		= 0x0203,
	MONT_FLIP		= 0x0252,
	MONT_MIRROR		= 0x0253,

/* Category 7: EXIF information */
	EXIF_HEAD		= 0x0700,
	EXIF_TAIL		= 0x072F,
	EXIF_SIZE		= EXIF_TAIL - EXIF_HEAD,

/* category B: Still picture parameter */
	STPP_SET_FMT		= 0x0B00,
		FMT_YUV422		= 0x00,
		FMT_JPEG_H422		= 0x01,
		FMT_JPEG_H420		= 0x02,
		FMT_RAW			= 0x07,
	STPP_MAIN_SIZE		= 0x0B01,
		MAIN_SIZE_QQVGA		= 0x00,
		MAIN_SIZE_QVGA		= 0x02,
		MAIN_SIZE_VGA		= 0x09,
		MAIN_SIZE_XGA		= 0x0F,	/* 1024 X 768 */
		MAIN_SIZE_1M			= 0x14,	/* 1280 X 960 */
		MAIN_SIZE_2M			= 0x17,	/* 1600 X 1200 */
		MAIN_SIZE_3M			= 0x1B,	/* 2048 X 1536 */
		MAIN_SIZE_5M			= 0x1F,	/* 2560 X 1920 */
		MAIN_SIZE_8M			= 0x25,	/* 3264 X 2448 */
	STPP_JPEG_SIZE_MAX3	= 0x0B0F,
	STPP_JPEG_SIZE_MAX2	= 0x0B10,
	STPP_JPEG_SIZE_MAX1	= 0x0B11,
	STPP_JPEG_SIZE_MAX0	= 0x0B12,
	STPP_JPEG_SIZE_MIN3	= 0x0B13,
	STPP_JPEG_SIZE_MIN2	= 0x0B14,
	STPP_JPEG_SIZE_MIN1	= 0x0B15,
	STPP_JPEG_SIZE_MIN0	= 0x0B16,
	STPP_JPEG_SIZE_MAX	= STPP_JPEG_SIZE_MAX3,
	STPP_JPEG_SIZE_MIN	= STPP_JPEG_SIZE_MIN3,

/* category C: Still picture control */
	STPC_MODE		= 0x0C00,
		CAP_SIGNAL		= 0x00,
		CAP_MUILT		= 0x01,
	STPC_SEL_MF		= 0x0C06,
	STPC_TRANS_START	= 0x0C09,
		TRANS_MAIN		= 0x01,
		TRANS_PREV		= 0x02,
		TRANS_THUM		= 0x03,
	STPC_GET_SIZE		= 0x0C0D,

/* category D: Production and endurence test */
	SENSOR_REG_RW		= 0x0D33,
		RW_READ			= 1,
		RW_WRITE		= 2,
	SENSOR_REG_ADRH		= 0x0D34,
	SENSOR_REG_ADRL		= 0x0D35,
	SENSOR_REG_DATA		= 0x0D36,

/* category F: Flash ROM */
	CAM_START		= 0x0F12,
/* All above address is invalid*/
	INVALID_ADDR		= 0x1000,
};

enum m6mo_operations {
	OPER_NOOP = 0,	/* Noop */
	OPER_RESET,	/* ISP reset */
	OPER_READ,	/* Register read */
	OPER_WRITE,	/* Register write */
	OPER_SET,	/* Register bit set */
	OPER_CLEAR,	/* Register bit clear */
	OPER_SLEEP,	/* Sleep in ms, set addr for duration*/
	OPER_WAIT_INT,	/* Wait for interrupt */
	OPER_TRIGGER,	/* Trigger setting to be downloaded to sensor */
	/* TODO: ALWAYS add oper names BEFORE OPER_END */
	OPER_END,
};

#define ISP_REG_ADDR_TYPE	u16
#define ISP_REG_DATA_TYPE	u8
#define SENSOR_REG_ADDR_TYPE	u16
#define SENSOR_REG_DATA_TYPE	u8

struct isp_oper {
	enum m6mo_operations	oper;
	ISP_REG_ADDR_TYPE	addr;
	ISP_REG_DATA_TYPE	data;
};

#define WD_BYTES(x) ((x)>>8)&0xFF, (x)&0xFF
#define DW_BYTES(x) WD_BYTES(((x)>>16)&0xFFFF), WD_BYTES((x)&0xFFFF)

#define BUFFER_DEPTH	8

__attribute__ ((unused)) struct isp_oper YUV_VGA[] = {
	{OPER_RESET},
	{OPER_WRITE,	MONT_OUTPUT_IF,	OUTPUT_MIPI	},
	{OPER_WRITE,	MONT_SIZE,	SIZE_VGA_30FPS	},
	{OPER_WRITE,	MONT_FPS, 	FPS_30	},
	{OPER_WRITE,	0x0106, 	0x01	},
	{OPER_WRITE,	MONT_SHD,	SHD_ON	},
	{OPER_WRITE,	0x0304, 	0x01	},
	{OPER_WRITE,	0x0307, 	0x01	},
	{OPER_WRITE,	0x012D, 	0x00	},
	{OPER_WRITE,	SYSP_IRQ_EN,	0	},
	{OPER_WRITE,	SYSP_MODE,	MODE_MONIT	},
	{OPER_SLEEP,	0,		100	},
	{OPER_TRIGGER},
	{OPER_END}
};

__attribute__ ((unused)) struct isp_oper YUV_720P[] = {
	{OPER_RESET},
	{OPER_WRITE,	MONT_OUTPUT_IF,	OUTPUT_MIPI	},
	{OPER_WRITE,	MONT_SIZE,	SIZE_720P	},
	{OPER_WRITE,	MONT_FPS, 	FPS_30	},
	{OPER_WRITE,	0x0106, 	0x01	},
	{OPER_WRITE,	MONT_SHD,	SHD_ON	},
	{OPER_WRITE,	0x0304, 	0x01	},
	{OPER_WRITE,	0x0307, 	0x01	},
	{OPER_WRITE,	0x012D, 	0x00	},
	{OPER_WRITE,	SYSP_IRQ_EN,	0	},
	{OPER_WRITE,	SYSP_MODE,	MODE_MONIT	},
	{OPER_SLEEP,	0,		100	},
	{OPER_TRIGGER},
	{OPER_END}
};

__attribute__ ((unused)) struct isp_oper YUV_1080P[] = {
	{OPER_RESET},
	{OPER_WRITE,	MONT_OUTPUT_IF,	OUTPUT_MIPI	},
	{OPER_WRITE,	MONT_SIZE,	SIZE_1080P	},
	{OPER_WRITE,	MONT_FPS, 	FPS_30	},
	{OPER_WRITE,	0x0106, 	0x01	},
	{OPER_WRITE,	MONT_SHD,	SHD_ON	},
	{OPER_WRITE,	0x0304, 	0x01	},
	{OPER_WRITE,	0x0307, 	0x01	},
	{OPER_WRITE,	0x012D, 	0x00	},
	{OPER_WRITE,	SYSP_IRQ_EN,	0	},
	{OPER_WRITE,	SYSP_MODE,	MODE_MONIT	},
	{OPER_SLEEP,	0,		100	},
	{OPER_TRIGGER},
	{OPER_END}
};

struct m6mo_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct m6mo_info {
	struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;
	int (*power)(struct device *, int);

	int model;	/* V4L2_IDENT_m6mo* codes from v4l2-chip-ident.h */
	struct v4l2_rect rect;
	u32 pixfmt;
	const struct m6mo_datafmt *curfmt;
	const struct m6mo_datafmt *fmts;
	int num_fmts;

	struct regval_list *regs_fmt;
	struct regval_list *regs_size;

	enum _m6mo_state isp_state;
	struct work_struct frame_wq;
};

#endif /* _M6MO_H_ */
