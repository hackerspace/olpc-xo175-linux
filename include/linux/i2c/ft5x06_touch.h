#ifndef __LINUX_I2C_FT5X06_TOUCH_H
#define __LINUX_I2C_FT5X06_TOUCH_H

#include <linux/input.h>
/*
 * power func:	power on / off touch
 * reset func:	reset touch
 * abs_x_max:	max value of abs_x
 * abs_y_max:	max value of abs_y
 * set_virtual_key func: set virtual keys
 */
struct ft5x06_touch_platform_data {
	int (*power)(int);
	void (*reset)(void);
	int abs_x_max;
	int abs_y_max;
	int (*set_virtual_key)(struct input_dev *);
};

#endif
