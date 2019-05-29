/*
*  cm3623.h - CM3623 light and proximity sensor driver
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
#ifndef __CM3623_H__
#define __CM3623_H__

#define CM3623_IR_INIT_REG		(0x22 >> 1)
#define CM3623_IR_NORMAL_MODE		0x10
#define CM3623_IR_PS_HIGH_LOW_MODE	0x20

#define CM3623_ALS_WRITE_CMD_REG	(0x20 >> 1)
#define ALS_SD_ALS_SHUTDOWN		(1<<0)
#define ALS_WDM_WORD_MODE		(1<<1)
#define ALS_IT0_ALS_INTEGRATE_TIME	(1<<2)
#define ALS_IT1_ALS_INTEGRATE_TIME	(1<<3)
#define ALS_THD0_THRESHOLD_WINDOW	(1<<4)
#define ALS_THD1_THRESHOLD_WINDOW	(1<<5)
#define ALS_GAIN0			(1<<6)
#define ALS_GAIN1			(1<<7)

#define ALS_IT_ALS			(ALS_IT0_ALS_INTEGRATE_TIME | \
					 ALS_IT1_ALS_INTEGRATE_TIME)
#define ALS_THD_ALS			(ALS_THD0_THRESHOLD_WINDOW | \
					 ALS_THD1_THRESHOLD_WINDOW)
#define ALS_GAIN_ALS			(ALS_GAIN0 | ALS_GAIN1)

#define ALS_CMD_INITIAL_STATE		(ALS_WDM_WORD_MODE | \
					 ALS_SD_ALS_SHUTDOWN)
#define ALS_INTEGRATE_TIME_100		(0)
#define ALS_INTEGRATE_TIME_200		(ALS_IT0_ALS_INTEGRATE_TIME)
#define ALS_INTEGRATE_TIME_400		(ALS_IT1_ALS_INTEGRATE_TIME)
#define ALS_INTEGRATE_TIME_800		(ALS_IT0_ALS_INTEGRATE_TIME | \
					 ALS_IT1_ALS_INTEGRATE_TIME)

#define CM3623_ALS_READ_MSB_REG		(0x21 >> 1)
#define CM3623_ALS_READ_LSB_REG		(0x23 >> 1)

#define CM3623_ARA_INTERRUPT_REG	(0x19 >> 1)

#define CM3623_PS_WRITE_CMD_REG		(0xB0 >> 1)
#define CM3623_PS_READ_PS_REG		(0xB1 >> 1)
#define CM3623_PS_THRESHOLD		(0xB2 >> 1)

#define PS_SD_PS_SHUTDOWN		(1<<0)
#define PS_SD_RESERVED1			(1<<1)
#define PS_INT_PS_INTERRUPTION		(1<<2)
#define PS_INT_ALS_INTERRUPTION		(1<<3)
#define PS_IT0_PS_INTEGRATE_TIME	(1<<4)
#define PS_IT1_PS_INTEGRATE_TIME	(1<<5)
#define PS_DR0_LED_DUTY_RATIO		(1<<6)
#define PS_DR1_LED_DUTY_RATIO		(1<<7)

#define PS_IT_PS			(PS_IT0_PS_INTEGRATE_TIME | \
					 PS_IT1_PS_INTEGRATE_TIME)

#define PS_DR_PS			(PS_DR0_LED_DUTY_RATIO | \
					 PS_DR1_LED_DUTY_RATIO)
#define PS_CMD_INITIAL_STATE		(PS_SD_PS_SHUTDOWN)

#define PS_TURN_OFF			(0)

#define PS_SHUTDOWN__POS   0
#define PS_INT__MSK        0x04
#define PS_INT__POS        2
#define ALS_INT__MSK       0x08
#define ALS_INT__POS       3
#define PS_INTER_TIME__MSK 0x30
#define PS_INTER_TIME__POS 4
#define PS_THRESH_WIN__MSK 0xC0
#define PS_THRESH_WIN__POS 6

#define ON 1
#define OFF 0

#define ALS_SAMPLE_INTERVAL 200
#define PS_SAMPLE_INTERVAL 200

struct i2c_cm3623 {
	struct i2c_client *g_client_ps_cmd_or_data;
	struct i2c_client *g_client_als_cmd_or_msb;
	struct i2c_client *g_client_init_or_lsb;
	struct i2c_client *g_client_int;
	struct i2c_client *g_client_ps_threshold;
	struct input_dev *g_ps;
	struct input_dev *g_als;
	struct mutex active_lock;
	unsigned char als_user_count;
	unsigned char ps_user_count;
	unsigned char g_ps_state;
	unsigned char g_als_state;
	unsigned int ps_sample_interval;
	unsigned int als_sample_interval;

	struct axis_sensor_platform_data *pdata;
};

#endif
