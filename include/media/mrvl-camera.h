#ifndef _MARVELL_CAMERA_H_
#define _MARVELL_CAMERA_H_

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



/* V4L2 related */
#define V4L2_CID_PRIVATE_FIRMWARE_DOWNLOAD	(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_GET_MIPI_PHY		(V4L2_CID_PRIVATE_BASE + 1)

#endif
