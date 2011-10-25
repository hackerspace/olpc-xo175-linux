#ifndef __ASM_MACH_MMP2_H
#define __ASM_MACH_MMP2_H

#include <linux/platform_data/pxa_sdhci.h>

struct sys_timer;

extern struct sys_timer mmp2_timer;
extern void __init mmp2_init_icu(void);
extern void __init mmp2_init_irq(void);
extern void mmp2_clear_pmic_int(void);
extern void __init mmp2_reserve(void);

#include <linux/i2c.h>
#include <linux/i2c/pxa-i2c.h>
#include <mach/devices.h>
#include <mach/pxa168fb.h>
#include <mach/uio_hdmi.h>
#include <plat/pxa27x_keypad.h>
#include <mach/sram.h>

extern struct pxa_device_desc mmp2_device_uart1;
extern struct pxa_device_desc mmp2_device_uart2;
extern struct pxa_device_desc mmp2_device_uart3;
extern struct pxa_device_desc mmp2_device_uart4;
extern struct pxa_device_desc mmp2_device_twsi1;
extern struct pxa_device_desc mmp2_device_twsi2;
extern struct pxa_device_desc mmp2_device_twsi3;
extern struct pxa_device_desc mmp2_device_twsi4;
extern struct pxa_device_desc mmp2_device_twsi5;
extern struct pxa_device_desc mmp2_device_twsi6;
extern struct pxa_device_desc mmp2_device_sdh0;
extern struct pxa_device_desc mmp2_device_sdh1;
extern struct pxa_device_desc mmp2_device_sdh2;
extern struct pxa_device_desc mmp2_device_sdh3;
extern struct pxa_device_desc mmp2_device_pwm1;
extern struct pxa_device_desc mmp2_device_pwm2;
extern struct pxa_device_desc mmp2_device_pwm3;
extern struct pxa_device_desc mmp2_device_pwm4;
extern struct pxa_device_desc mmp2_device_fb;
extern struct pxa_device_desc mmp2_device_fb_ovly;
extern struct pxa_device_desc mmp2_device_fb_tv;
extern struct pxa_device_desc mmp2_device_fb_tv_ovly;
extern struct pxa_device_desc mmp2_device_hdmi;
extern struct pxa_device_desc mmp2_device_keypad;
extern struct pxa_device_desc mmp2_device_sspa1;
extern struct pxa_device_desc mmp2_device_sspa2;
extern struct pxa_device_desc mmp2_device_audiosram;
extern struct platform_device mmp_device_asoc_sspa1;
extern struct platform_device mmp_device_asoc_sspa2;
extern struct platform_device mmp_device_asoc_platform;
extern struct pxa_device_desc mmp2_device_thsens;

extern struct platform_device pxa168_device_u2o;
extern struct platform_device pxa168_device_u2oehci;
extern struct platform_device pxa168_device_u2ootg;

static inline int mmp2_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &mmp2_device_uart1; break;
	case 2: d = &mmp2_device_uart2; break;
	case 3: d = &mmp2_device_uart3; break;
	case 4: d = &mmp2_device_uart4; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int mmp2_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 1: d = &mmp2_device_twsi1; break;
	case 2: d = &mmp2_device_twsi2; break;
	case 3: d = &mmp2_device_twsi3; break;
	case 4: d = &mmp2_device_twsi4; break;
	case 5: d = &mmp2_device_twsi5; break;
	case 6: d = &mmp2_device_twsi6; break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id - 1, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int mmp2_add_sdhost(int id, struct sdhci_pxa_platdata *data)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &mmp2_device_sdh0; break;
	case 1: d = &mmp2_device_sdh1; break;
	case 2: d = &mmp2_device_sdh2; break;
	case 3: d = &mmp2_device_sdh3; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int mmp2_add_pwm(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &mmp2_device_pwm1; break;
	case 2: d = &mmp2_device_pwm2; break;
	case 3: d = &mmp2_device_pwm3; break;
	case 4: d = &mmp2_device_pwm4; break;
	default:
			return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int mmp2_add_fb(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp2_device_fb, mi, sizeof(*mi));
}

static inline int mmp2_add_fb_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp2_device_fb_ovly, mi, sizeof(*mi));
}

static inline int mmp2_add_fb_tv(struct pxa168fb_mach_info *mi)
{
       return pxa_register_device(&mmp2_device_fb_tv, mi, sizeof(*mi));
}

static inline int mmp2_add_fb_tv_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp2_device_fb_tv_ovly, mi, sizeof(*mi));
}

static inline int mmp2_add_hdmi(struct uio_hdmi_platform_data *data)
{
		return pxa_register_device(&mmp2_device_hdmi, data, sizeof(*data));
}

static inline int mmp2_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	return pxa_register_device(&mmp2_device_keypad, data, sizeof(*data));
}

static inline int mmp2_add_thermal_sensor(void)
{
	return pxa_register_device(&mmp2_device_thsens, NULL, 0);
}

static inline int mmp2_add_audiosram(struct sram_bank *data)
{
	return pxa_register_device(&mmp2_device_audiosram, data, sizeof(*data));
}

static inline int mmp2_add_sspa(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &mmp2_device_sspa1; break;
	case 2: d = &mmp2_device_sspa2; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

#endif /* __ASM_MACH_MMP2_H */

