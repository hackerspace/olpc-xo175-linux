/* linux/input/crucial_tec_otp.h
 *
 * platform data for crucial tec optical track pad device.
 *
 * Copyright (c) 2011 Marvell
 *
 */

#ifndef __LINUX_I2C_CRUCIAL_TEC_OTP_H
#define __LINUX_I2C_CRUCIAL_TEC_OTP_H

/* Board specific touch screen initial values */
struct ctec_optic_tp_platform_data {
	int			pd_gpio;
	int			rst_gpio;
	int			irq_gpio;
	int			ok_btn_gpio;
};

#endif /*  __LINUX_I2C_CRUCIAL_TEC_OTP_H */
