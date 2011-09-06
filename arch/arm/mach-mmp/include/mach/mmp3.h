#ifndef __ASM_MACH_MMP3_H
#define __ASM_MACH_MMP3_H

struct sys_timer;

extern struct sys_timer mmp3_timer;
extern void __init mmp3_init_gic(void);
extern void __init mmp3_init_irq(void);
extern void __init mmp3_reserve(void);

#include <linux/i2c.h>
#include <linux/i2c/pxa-i2c.h>
#include <mach/devices.h>
#include <mach/cputype.h>
#include <mach/regs-apbc.h>
#include <mach/pxa168fb.h>
#include <mach/uio_hdmi.h>
#include <plat/pxa27x_keypad.h>
#include <plat/pxa3xx_nand.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <mach/sram.h>
#include <mach/camera.h>

#define IOPWRDOM_VIRT_BASE	(APB_VIRT_BASE + 0x1e800)
#define PAD_1V8			(1 << 2)
#define PAD_POWERDOWN		(1 << 0)
#define AIB_HSIC3_IO_REG	0x0000
#define AIB_HSIC2_IO_REG	0x0004
#define AIB_GPIO2_IO_REG	0x000C
#define AIB_GPIO3_IO_REG	0x0010
#define AIB_TWSI_IO_REG		0x0014
#define AIB_SDMMC_IO_REG	0x001c
#define AIB_NAND_IO_REG		0x0020
#define AIB_USIM_IO_REG		0x002c
#define AIB_BB_IO_REG		0x0030

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
extern struct pxa_device_desc mmp3_device_sdh0;
extern struct pxa_device_desc mmp3_device_sdh1;
extern struct pxa_device_desc mmp3_device_sdh2;
extern struct pxa_device_desc mmp3_device_sdh3;
extern struct pxa_device_desc mmp3_device_camera0;
extern struct pxa_device_desc mmp3_device_camera1;
extern struct pxa_device_desc mmp3_device_pwm1;
extern struct pxa_device_desc mmp3_device_pwm2;
extern struct pxa_device_desc mmp3_device_pwm3;
extern struct pxa_device_desc mmp3_device_pwm4;
extern struct pxa_device_desc mmp3_device_keypad;
extern struct pxa_device_desc mmp3_device_fb;
extern struct pxa_device_desc mmp3_device_fb_ovly;
extern struct pxa_device_desc mmp3_device_v4l2_ovly;
extern struct pxa_device_desc mmp3_device_fb_tv;
extern struct pxa_device_desc mmp3_device_fb_tv_ovly;
extern struct pxa_device_desc mmp3_device_v4l2_tv_ovly;
extern struct pxa_device_desc mmp3_device_hdmi;
extern struct pxa_device_desc mmp3_device_videosram;

extern struct platform_device mmp3_device_rtc;
extern struct pxa_device_desc mmp3_device_sspa1;
extern struct pxa_device_desc mmp3_device_sspa2;
extern struct pxa_device_desc mmp3_device_audiosram;
extern struct platform_device mmp3_device_u2o;
extern struct platform_device mmp3_device_u2ootg;
extern struct platform_device mmp3_device_u2oehci;
extern struct platform_device mmp3_hsic1_device;

static inline void mmp3_io_domain_1v8(u16 reg, int set)
{
	u32 tmp;

	writel(MAGIC_ASFAR, APBC_MMP2_ASFAR);
	writel(MAGIC_ASSAR, APBC_MMP2_ASSAR);
	tmp = readl(IOPWRDOM_VIRT_BASE + reg);

	if (set)
		tmp |= PAD_1V8;
	else
		tmp &= ~PAD_1V8;

	writel(MAGIC_ASFAR, APBC_MMP2_ASFAR);
	writel(MAGIC_ASSAR, APBC_MMP2_ASSAR);
	writel(tmp, IOPWRDOM_VIRT_BASE + reg);
}

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

static inline int mmp3_add_cam(int id, struct mv_cam_pdata *cam)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0:
		d = &mmp3_device_camera0; break;
	case 1:
		d = &mmp3_device_camera1; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, cam, sizeof(*cam));
}

static inline int mmp3_add_nand(struct pxa3xx_nand_platform_data *info)
{
	return pxa_register_device(&mmp3_device_nand, info, sizeof(*info));
}

static inline int mmp3_add_sdh(int id, struct sdhci_pxa_platdata *data)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &mmp3_device_sdh0; break;
	case 1: d = &mmp3_device_sdh1; break;
	case 2: d = &mmp3_device_sdh2; break;
	case 3: d = &mmp3_device_sdh3; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, data, sizeof(*data));
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

static inline int mmp3_add_fb(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp3_device_fb, mi, sizeof(*mi));
}

static inline int mmp3_add_fb_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp3_device_fb_ovly, mi, sizeof(*mi));
}

static inline int mmp3_add_v4l2_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp3_device_v4l2_ovly, mi, sizeof(*mi));
}

static inline int mmp3_add_fb_tv(struct pxa168fb_mach_info *mi)
{
       return pxa_register_device(&mmp3_device_fb_tv, mi, sizeof(*mi));
}

static inline int mmp3_add_fb_tv_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp3_device_fb_tv_ovly, mi, sizeof(*mi));
}

static inline int mmp3_add_v4l2_tv_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp3_device_v4l2_tv_ovly, mi, sizeof(*mi));
}

static inline int mmp3_add_hdmi(struct uio_hdmi_platform_data *data)
{
        return pxa_register_device(&mmp3_device_hdmi, data, sizeof(*data));
}

extern void mmp3_clear_keypad_wakeup(void);
static inline int mmp3_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	data->clear_wakeup_event = mmp3_clear_keypad_wakeup;
	return pxa_register_device(&mmp3_device_keypad, data, sizeof(*data));
}

static inline int mmp3_add_audiosram(struct sram_bank *data)
{
	return pxa_register_device(&mmp3_device_audiosram, data, sizeof(*data));
}

static inline int mmp3_add_sspa(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1:
		d = &mmp3_device_sspa1;
		break;
	case 2:
		d = &mmp3_device_sspa2;
		break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int mmp3_add_videosram(struct sram_bank *data)
{
	return pxa_register_device(&mmp3_device_videosram, data, sizeof(*data));
}

#endif /* __ASM_MACH_MMP2_H */
