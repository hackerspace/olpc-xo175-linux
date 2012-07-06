#ifndef _MARVELL_CAMERA_H_
#define _MARVELL_CAMERA_H_

#include <linux/delay.h>

/* Sensor installation related */
enum {
	SENSOR_USED		= (1 << 31),
	SENSOR_UNUSED		= 0,
	SENSOR_POS_LEFT		= (1 << 2),
	SENSOR_POS_RIGHT	= 0,
	SENSOR_POS_FRONT	= (1 << 1),
	SENSOR_POS_BACK		= 0,
	SENSOR_RES_HIGH		= (1 << 0),
	SENSOR_RES_LOW		= 0,
};



/* MIPI related */
struct csi_dphy_desc {
	u32 clk_mul;
	u32 clk_div;	/* clock_lane_freq = input_clock * clk_mul / clk_div */
	u32 cl_prepare;
	u32 cl_zero;
	u32 hs_prepare;
	u32 hs_zero;
	u32 nr_lane;	/* When set to 0, S/W will try to figure out a value */
};

/*
 *Add macro definiton for sensor power.
 *Plese note the POWER_OFF and POWER_ON
 *value is fixed since in soc_camera.c
 *the value is directly used.
 */
#define POWER_OFF		0
#define POWER_ON		1
#define POWER_SAVING		2
#define POWER_RESTORE		3

/* V4L2 related */
#define V4L2_CID_PRIVATE_FIRMWARE_DOWNLOAD	(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_GET_MIPI_PHY		(V4L2_CID_PRIVATE_BASE + 1)

/* Misc */
/* sleep function for sensor power sequence, only provide 100us precision */
/* According to Documentation/timers/timers-howto.txt, we should choose *sleep
 * family function according to the time:
 * >= 20ms	msleep
 * >= 10us	usleep
 * <10us	udelay
 * for camera power usecase, we mostly need 1~2ms for power on/off sequence and
 * 100~500us for software reset, no precision control below 10us is used,
 * so sleep_range is good enough for us */
#define usleep(x) \
do { \
	unsigned int left = (x); \
	if (left >= 20000) { \
		msleep(left-20000); \
		left -= 20000; \
	} \
	if (left >= 100) \
		usleep_range(left, left+100); \
} while (0)

#endif
