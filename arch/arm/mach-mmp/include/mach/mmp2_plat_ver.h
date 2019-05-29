#ifndef __ASM_MACH_MMP2_PLAT_VER_H
#define __ASM_MACH_MMP2_PLAT_VER_H

#include <asm/mach-types.h>
#include <linux/gpio.h>

extern int mmp2_platform_version;

static inline int mmp2_get_platform_version(void)
{
	int gpio_vers0 = mfp_to_gpio(GPIO125_VERS0);
	int gpio_vers1 = mfp_to_gpio(GPIO126_VERS1);
	int gpio_vers2 = mfp_to_gpio(GPIO127_VERS2);
	int gpio_vers3 = mfp_to_gpio(GPIO128_VERS3);

	if (gpio_request(gpio_vers0, "vers0")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_vers0);
		return -1;
	}

	if (gpio_request(gpio_vers1, "vers1")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_vers1);
		gpio_free(gpio_vers0);
		return -1;
	}

	if (gpio_request(gpio_vers2, "vers2")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_vers2);
		gpio_free(gpio_vers0);
		gpio_free(gpio_vers1);
		return -1;
	}

	if (gpio_request(gpio_vers3, "vers3")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_vers3);
		gpio_free(gpio_vers0);
		gpio_free(gpio_vers1);
		gpio_free(gpio_vers2);
		return -1;
	}

	gpio_direction_input(gpio_vers0);
	gpio_direction_input(gpio_vers1);
	gpio_direction_input(gpio_vers2);
	gpio_direction_input(gpio_vers3);

	mmp2_platform_version = ((!!gpio_get_value(gpio_vers3))<<3) |
		((!!gpio_get_value(gpio_vers2))<<2) |
		((!!gpio_get_value(gpio_vers1))<<1) |
		((!!gpio_get_value(gpio_vers0))<<0);

	gpio_free(gpio_vers0);
	gpio_free(gpio_vers1);
	gpio_free(gpio_vers2);
	gpio_free(gpio_vers3);

	return 0;
}

static inline int board_is_mmp2_brownstone_rev1(void){
	return (machine_is_brownstone() && (0x9 == mmp2_platform_version));
}

static inline int board_is_mmp2_brownstone_rev2(void){
	return (machine_is_brownstone() && (0x7 == mmp2_platform_version));
}

static inline int board_is_mmp2_brownstone_rev4(void){
	return (machine_is_brownstone() && (0x3 == mmp2_platform_version));
}

static inline int board_is_mmp2_brownstone_rev5(void){
	return (machine_is_brownstone() && (0xe == mmp2_platform_version));
}

#endif /* __ASM_MACH_MMP2_PLAT_VER_H */

