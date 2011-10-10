/*
 * linux/arch/arm/mach-mmp/include/mach/timex.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
#define CLOCK_TICK_RATE		6500000
#else
#define CLOCK_TICK_RATE		3250000
#endif

/*
 * Since we need to delay 3 cycle of tickes to get the update of timer
 * we get the calculated result based on the slowest freq here, 3250000
 * This 3.25M timer's tick happen every 0.307us, and 3 times of this value
 * should be still smaller than 1us, so delay 1us is enough for all
 * fast timer here
 */
#if (CLOCK_TICK_RATE >= 3250000)
#define DELAY_US		1
#else
#error "We current don't support this CLOCK_TICK_RATE!!!"
#endif
