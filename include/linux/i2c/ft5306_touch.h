#ifndef __LINUX_I2C_FT5306_TOUCH_H
#define __LINUX_I2C_FT5306_TOUCH_H

/*
 * power func:	power on / off touch
 * reset func:	reset touch
 * abs_x_max:	max value of abs_x
 * abs_y_max:	max value of abs_y
 * abs_flag:	convert the frame of axes
 * 0:	no change
 * 1:	convert the frame of axes 90 degree by clockwise
 * 2:	convert the frame of axes 180 degree by clockwise
 * 3:	convert the frame of axes 270 degree by clockwise
 */
struct ft5306_touch_platform_data {
	int (*power)(int);
	void (*reset)(void);
	int abs_x_max;
	int abs_y_max;
	int abs_flag;
};

#endif
