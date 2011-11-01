#ifndef __MACH_MMP2_FUSE_H
#define __MACH_MMP2_FUSE_H

#include <mach/regs-fuse.h>

#define MMP2_PROFILE_SHIFT_VAL          3
#define MMP2_PROFILE_NUM                8

#define MMP2_PROFILE_ADJUST_MASK	(0x7)
#define MMP2_PROFILE_ADJUST_NONE	(0x0)
#define MMP2_PROFILE_ADJUST_INC_1	(0x3)
#define MMP2_PROFILE_ADJUST_DEC_1	(0x7)

#define MMP2_MAX_FREQ_SHIFT		(14)
#define MMP2_MAX_FREQ_MASK		(0x3)
#define MMP2_MAX_FREQ_988MHZ		(0x0)
#define MMP2_MAX_FREQ_800MHZ		(0x1)

#define MMP2_PPT_ULTRA_LOW_MIPS		0
#define MMP2_PPT_LOW_MIPS		1
#define MMP2_PPT_VG_MIPS		2
#define MMP2_PPT_VG_HIGH_MIPS		3
#define MMP2_PPT_ULTRA_HIGH_MIPS	4
#define MMP2_PRODUCT_POINT_NUM          5

extern unsigned int mmp2_read_profile(void);
extern int mmp2_get_voltage(unsigned int profile, unsigned int product_point);
extern unsigned int mmp2_read_max_freq(void);

#endif /* __MACH_MMP2_FUSE_H  */
