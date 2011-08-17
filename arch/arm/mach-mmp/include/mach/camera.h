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
	int (*power_on)(int, int);
	int (*platform_set)(int, struct clk*);
};

#endif

