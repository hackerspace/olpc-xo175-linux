#ifndef __ASM_MACH_PXA910_H
#define __ASM_MACH_PXA910_H

struct sys_timer;

extern struct sys_timer pxa910_timer;
extern void __init pxa910_init_irq(void);
extern void pxa910_ripc_lock(void);
extern void pxa910_ripc_unlock(void);
extern int pxa910_ripc_trylock(void);

#include <linux/i2c.h>
#include <linux/i2c/pxa-i2c.h>
#include <mach/devices.h>
#include <plat/pxa3xx_nand.h>
#include <plat/pxa27x_keypad.h>
#include <linux/platform_data/pxa_sdhci.h>

extern struct pxa_device_desc pxa910_device_uart1;
extern struct pxa_device_desc pxa910_device_uart2;
extern struct pxa_device_desc pxa910_device_twsi0;
extern struct pxa_device_desc pxa910_device_twsi1;
extern struct pxa_device_desc pxa910_device_pwm1;
extern struct pxa_device_desc pxa910_device_pwm2;
extern struct pxa_device_desc pxa910_device_pwm3;
extern struct pxa_device_desc pxa910_device_pwm4;
extern struct pxa_device_desc pxa910_device_nand;
extern struct pxa_device_desc pxa910_device_keypad;
extern struct pxa_device_desc pxa910_device_sdh0;
extern struct pxa_device_desc pxa910_device_sdh1;
extern struct pxa_device_desc pxa910_device_sdh2;
extern struct pxa_device_desc pxa910_device_cnm;

extern struct platform_device pxa910_device_rtc;
extern struct platform_device pxa910_device_1wire;

extern void pxa910_clear_keypad_wakeup(void);

static inline int pxa910_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &pxa910_device_uart1; break;
	case 2: d = &pxa910_device_uart2; break;
	}

	if (d == NULL)
		return -EINVAL;

	return pxa_register_device(d, NULL, 0);
}

static inline int pxa910_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 0: d = &pxa910_device_twsi0; break;
	case 1: d = &pxa910_device_twsi1; break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int pxa910_add_pwm(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &pxa910_device_pwm1; break;
	case 2: d = &pxa910_device_pwm2; break;
	case 3: d = &pxa910_device_pwm3; break;
	case 4: d = &pxa910_device_pwm4; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int pxa910_add_nand(struct pxa3xx_nand_platform_data *info)
{
	return pxa_register_device(&pxa910_device_nand, info, sizeof(*info));
}

static inline int pxa910_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	data->clear_wakeup_event = pxa910_clear_keypad_wakeup;
	return pxa_register_device(&pxa910_device_keypad, data, sizeof(*data));
}

static inline void pxa910_add_1wire(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_1wire);
	if (ret)
		dev_err(&pxa910_device_1wire.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_sdh(int id, struct sdhci_pxa_platdata *data)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &pxa910_device_sdh0; break;
	case 1: d = &pxa910_device_sdh1; break;
	case 2: d = &pxa910_device_sdh2; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int pxa910_add_cnm(void)
{
	return pxa_register_device(&pxa910_device_cnm, NULL, 0);
}

#endif /* __ASM_MACH_PXA910_H */
