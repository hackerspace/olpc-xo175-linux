/*
*  cm3217a.h - CM3217a light and proximity sensor driver
*
*  Copyright (C) Yifan Zhang  zhangyf@marvell.com
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#ifndef __CM3217A_H__
#define __CM3217A_H__

#include <mach/cm3217a.h>

#define ON 1
#define OFF 0
#define INITIAL_LSB_DATA 0x10
#define INITIAL_MSB_DATA 0x02
#define ALS_SHUTDOWN__MSK   0x01
#define ALS_DATA_MODE__MSK  0x02 
#define ALS_INTER_TIME__MSK 0x0C  
#define ALS_THRESH_WIN__MSK 0x30  
#define ALS_GAIN__MSK   	0xC0
#define ALS_SHUTDOWN__POS   0
#define ALS_DATA_MODE__POS  1 
#define ALS_INTER_TIME__POS 2  
#define ALS_THRESH_WIN__POS 4  
#define ALS_GAIN__POS   	6 

#define ALS_INT__MSK       0x08  
#define ALS_INT__POS       3  
#define SAMPLE_INTERVAL 50

#define CM3217a_GET_BITSLICE(regvar, bitname)\
			(regvar & bitname##__MSK) >> bitname##__POS
#define CM3217a_SET_BITSLICE(regvar, bitname, val)\
		  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

struct i2c_cm3217a {

	 struct cm3217a_platform_data *pdata;

	 struct i2c_client *g_client_msb;
	 struct i2c_client *g_client_lsb;
	 struct input_dev *g_als;
	 struct timer_list timer_als;
	 struct mutex als_lock;
	 unsigned char als_user_count;
	 char  g_als_status;
	 unsigned int als_sample_interval;
	 /* Add by Marco @ 20110308 for storing the value of active attr */
	 int active;
};
#endif

