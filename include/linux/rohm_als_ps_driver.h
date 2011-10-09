/******************************************************************************
 * MODULE     : rohm_als_driver.h
 * FUNCTION   : Light Sensor driver of BH1780
 * PROGRAMMED : sensor application development group
 * DATE(ORG)  : Jun-09-2011(Jun-09-2011)
 * REMARKS    :
 * C-FORM     : 1.00A
 * COPYRIGHT  : Copyright (C) 2011 ROHM CO.,LTD.
 *            : This software is licensed under the terms of the GNU General Public
 *            : License version 2, as published by the Free Software Foundation, and
 *            : may be copied, distributed, and modified under those terms.
 *            :
 *            : This program is distributed in the hope that it will be useful,
 *            : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *            : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *            : GNU General Public License for more details.
 * HISTORY    :
 * 1.00A Jun-09-2011  SEI   Made a new file
 *****************************************************************************/
#ifndef _ROHM_ALS_PS_DRIVER_H_
#define _ROHM_ALS_PS_DRIVER_H_

#define ROHM_I2C_NAME "rohm_ls"

#define _ROHM_BH1772_

#if (defined(_ROHM_BH1771_) || defined(_ROHM_BH1772_))
#define ROHM_I2C_Address (0x38)	/*7 bits slave address 011 1000 */
#endif
struct ROHM_I2C_platform_data {
	int (*power) (int on);	/* Only valid in first array entry */
	uint32_t flags;
	uint32_t fuzz_x;	/* 0x10000 = screen width */
	uint32_t fuzz_y;	/* 0x10000 = screen height */
	int fuzz_p;
	int fuzz_w;
};

struct rohm_ls_data {
	uint16_t addr;
	struct i2c_client *client;
	int use_irq;
	unsigned int als_poll_delay;
	unsigned int ps_delay;
	int (*power) (int on);
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct delayed_work als_work;
	struct work_struct ps_work;
	struct delayed_work ps_delay_work;
	unsigned char als_enable;
	unsigned char ps_enable;
	unsigned char ps_data;
	unsigned short als_data;
	unsigned char interrupt;
	unsigned char device_suspend;
	unsigned char als_meas_time;
	unsigned char ps_meas_time;
	struct mutex sensor_lock;
};

/* cmd definition of ioctl */
#define IOCTL_APP_SET_TIMER (0)

#endif /* _ROHM_ALS_PS_DRIVER_H_ */
