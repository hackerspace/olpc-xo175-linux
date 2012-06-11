#ifndef __ASM_CPU_PXA988_H
#define __ASM_CPU_PXA988_H

struct sys_timer;

extern struct sys_timer pxa988_timer;
extern void __init pxa988_init_gic(void);
extern void __init pxa988_init_irq(void);
extern void __init pxa988_reserve(void);

#include <linux/i2c.h>
#include <linux/i2c/pxa-i2c.h>
#include <mach/devices.h>
#include <mach/cputype.h>
#include <mach/regs-apbc.h>
#include <mach/sram.h>
#include <plat/pxa27x_keypad.h>
#include <mach/pxa168fb.h>

extern struct pxa_device_desc pxa988_device_uart1;
extern struct pxa_device_desc pxa988_device_uart2;
extern struct pxa_device_desc pxa988_device_uart3;
extern struct pxa_device_desc pxa988_device_keypad;
extern struct pxa_device_desc pxa988_device_twsi0;
extern struct pxa_device_desc pxa988_device_twsi1;
extern struct pxa_device_desc pxa988_device_twsi2;
extern struct pxa_device_desc pxa988_device_asram;
extern struct pxa_device_desc pxa988_device_vsram;
extern struct pxa_device_desc pxa988_device_fb;
extern struct pxa_device_desc pxa988_device_fb_ovly;

extern void pxa988_clear_keypad_wakeup(void);

static inline int pxa988_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1:
		d = &pxa988_device_uart1;
		break;
	case 2:
		d = &pxa988_device_uart2;
		break;
	case 3:
		d = &pxa988_device_uart3;
		break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int pxa988_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	data->clear_wakeup_event = pxa988_clear_keypad_wakeup;
	return pxa_register_device(&pxa988_device_keypad, data, sizeof(*data));
}

static inline int pxa988_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 0:
		d = &pxa988_device_twsi0;
		break;
	case 1:
		d = &pxa988_device_twsi1;
		break;
	case 2:
		d = &pxa988_device_twsi2;
		break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int pxa988_add_asram(struct sram_bank *data)
{
	return pxa_register_device(&pxa988_device_asram, data, sizeof(*data));
}

static inline int pxa988_add_vsram(struct sram_bank *data)
{
	return pxa_register_device(&pxa988_device_vsram, data, sizeof(*data));
}

static inline int pxa988_add_fb(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&pxa988_device_fb, mi, sizeof(*mi));
}

static inline int pxa988_add_fb_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&pxa988_device_fb_ovly, mi, sizeof(*mi));
}

#endif /* __ASM_CPU_PXA988_H */

