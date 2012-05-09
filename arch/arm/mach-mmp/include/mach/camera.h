#ifndef __ASM_ARCH_CAMERA_H__
#define __ASM_ARCH_CAMERA_H__

struct mv_cam_pdata {
#ifdef CONFIG_CPU_MMP3
	struct clk	*clk;
#else
	struct clk	*clk[3];
#endif
	char		*name;
	int		clk_enabled;
	int		dphy[3];
	int		bus_type;
	int		ccic_num_flag; /*ccic1:0; ccic2:1 */
	int		mipi_enabled;
	int		dma_burst;
	int		qos_req_min;
	int		mclk_min;
	int		mclk_src;
	void		(*controller_power)(int on);
	int		(*init_clk)(struct device *dev, int init);
	void		(*enable_clk)(struct device *dev, int on);
	int		(*get_mclk_src)(int src);
};
struct clk;
struct sensor_platform_data {
	int id;
	int interface;	/* MIPI or DVP flags*/
	int (*power_on)(int, int);
	int (*platform_set)(int, struct clk*);
};

struct mipi_phy {
	u16 cl_termen;
	u16 cl_settle;
	u16 cl_miss;
	u16 hs_termen;
	u16 hs_settle;
	u16 hs_rx_to;
	u16 lane;	/* When set to 0, S/W will try to figure out a value */
	u16 vc;		/* Virtual channel */
	u16 dt1;	/* Data type 1: For video or main data type */
	u16 dt2;	/* Data type 2: For thumbnail or auxiliry data type */
};
#define V4L2_CID_PRIVATE_GET_MIPI_PHY	(V4L2_CID_PRIVATE_BASE + 1)
void mv_set_sensor_attached(bool sensor_attached);
int isppwr_power_control(int on);

struct csi_dphy_desc {
	u32 clk_mul;
	u32 clk_div;	/* clock_lane_freq = input_clock * clk_mul / clk_div */
	u32 cl_prepare;
	u32 cl_zero;
	u32 hs_prepare;
	u32 hs_zero;
	u32 nr_lane;	/* When set to 0, S/W will try to figure out a value */
};

#endif

