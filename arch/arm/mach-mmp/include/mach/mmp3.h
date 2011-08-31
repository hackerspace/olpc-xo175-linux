#ifndef __ASM_MACH_MMP3_H
#define __ASM_MACH_MMP3_H

struct sys_timer;

extern struct sys_timer mmp3_timer;
extern void __init mmp3_init_gic(void);
extern void __init mmp3_init_irq(void);

#include <linux/i2c.h>
#include <linux/i2c/pxa-i2c.h>
#include <mach/devices.h>
#include <mach/cputype.h>
#include <plat/pxa27x_keypad.h>
#include <plat/pxa3xx_nand.h>

extern struct pxa_device_desc mmp3_device_uart1;
extern struct pxa_device_desc mmp3_device_uart2;
extern struct pxa_device_desc mmp3_device_uart3;
extern struct pxa_device_desc mmp3_device_uart4;
extern struct pxa_device_desc mmp3_device_twsi1;
extern struct pxa_device_desc mmp3_device_twsi2;
extern struct pxa_device_desc mmp3_device_twsi3;
extern struct pxa_device_desc mmp3_device_twsi4;
extern struct pxa_device_desc mmp3_device_twsi5;
extern struct pxa_device_desc mmp3_device_twsi6;
extern struct pxa_device_desc mmp3_device_nand;
extern struct pxa_device_desc mmp3_device_pwm1;
extern struct pxa_device_desc mmp3_device_pwm2;
extern struct pxa_device_desc mmp3_device_pwm3;
extern struct pxa_device_desc mmp3_device_pwm4;
extern struct pxa_device_desc mmp3_device_keypad;

extern struct platform_device mmp3_device_rtc;

static inline int mmp3_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &mmp3_device_uart1; break;
	case 2: d = &mmp3_device_uart2; break;
	case 3: d = &mmp3_device_uart3; break;
	case 4: d = &mmp3_device_uart4; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int mmp3_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 1: d = &mmp3_device_twsi1; break;
	case 2: d = &mmp3_device_twsi2; break;
	case 3: d = &mmp3_device_twsi3; break;
	case 4: d = &mmp3_device_twsi4; break;
	case 5: d = &mmp3_device_twsi5; break;
	case 6: d = &mmp3_device_twsi6; break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id - 1, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int mmp3_add_nand(struct pxa3xx_nand_platform_data *info)
{
	return pxa_register_device(&mmp3_device_nand, info, sizeof(*info));
}

static inline int mmp3_add_pwm(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1:
		d = &mmp3_device_pwm1;
		break;
	case 2:
		d = &mmp3_device_pwm2;
		break;
	case 3:
		d = &mmp3_device_pwm3;
		break;
	case 4:
		d = &mmp3_device_pwm4;
		break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

extern void mmp3_clear_keypad_wakeup(void);
static inline int mmp3_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	data->clear_wakeup_event = mmp3_clear_keypad_wakeup;
	return pxa_register_device(&mmp3_device_keypad, data, sizeof(*data));
}

#endif /* __ASM_MACH_MMP2_H */
