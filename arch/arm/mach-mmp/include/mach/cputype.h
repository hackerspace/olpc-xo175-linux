#ifndef __ASM_MACH_CPUTYPE_H
#define __ASM_MACH_CPUTYPE_H

#include <asm/cputype.h>

/*
 *  CPU   Stepping   CPU_ID      CHIP_ID
 *
 * PXA168    S0    0x56158400   0x0000C910
 * PXA168    A0    0x56158400   0x00A0A168
 * PXA910    Y1    0x56158400   0x00F2C920
 * PXA910    A0    0x56158400   0x00F2C910
 * PXA910    A1    0x56158400   0x00A0C910
 * PXA920    Y0    0x56158400   0x00F2C920
 * PXA920    A0    0x56158400   0x00A0C920
 * PXA920    A1    0x56158400   0x00A1C920
 * MMP2	     Z0	   0x560f5811   0x00F00410
 * MMP2      Z1    0x560f5811   0x00E00410
 * MMP2      A0    0x560f5811   0x00A0A610
 */

extern unsigned int mmp_chip_id;
extern unsigned int mmp_fuse_id;
extern unsigned int mmp_soc_stepping;
extern unsigned int mmp_soc_profile;
extern unsigned int mmp_1g_svc;

#ifdef CONFIG_CPU_PXA168
static inline int cpu_is_pxa168(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x84) &&
		((mmp_chip_id & 0xfff) == 0x168);
}
#else
#define cpu_is_pxa168()	(0)
#endif

#ifdef CONFIG_CPU_PXA910
static inline int cpu_is_pxa910_family(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x84) &&
		(((mmp_chip_id & 0xfff) == 0x910));
}

static inline int cpu_is_pxa920_family(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x84) &&
		(((mmp_chip_id & 0xfff) == 0x920));
}

static inline int cpu_is_pxa910(void)
{
	if (cpu_is_pxa910_family() && ((mmp_fuse_id & 0x0000f000) == 0x00003000))
		return 0;
	if (cpu_is_pxa910_family())
		return 1;
	return 0;
}

static inline int cpu_is_pxa920(void)
{
	if (cpu_is_pxa920_family() && ((mmp_fuse_id & 0x3000000)) == 0x3000000)
		return 0;
	if (cpu_is_pxa920_family() && (mmp_1g_svc == 0x0) &&
		((mmp_fuse_id & 0x0000f000) != 0x00003000))
		return 1;
	return 0;
}

static inline int cpu_is_pxa921(void)
{
	if (cpu_is_pxa920_family() && ((mmp_fuse_id & 0x3000000) == 0x3000000))
		return 0;
	if (cpu_is_pxa920_family() && ((mmp_1g_svc != 0x0) ||
		((mmp_fuse_id & 0x0000f000) == 0x00003000)))
		return 1;
	return 0;
}

static inline int cpu_is_pxa918(void)
{
	return (cpu_is_pxa920_family() &&
		((mmp_fuse_id & 0x03000000) == 0x03000000));
}

static inline int cpu_is_pxa910h(void)
{
	return (cpu_is_pxa910_family() &&
		((mmp_fuse_id & 0x0000f000) == 0x00003000));
}

#else
#define cpu_is_pxa910_family()	(0)
#define cpu_is_pxa920_family()	(0)
#endif

#ifdef CONFIG_CPU_MMP2
static inline int cpu_is_mmp2(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x58);
}
#else
#define cpu_is_mmp2()	(0)
#endif

#ifdef CONFIG_CPU_MMP3
static inline int cpu_is_mmp3(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x58);
}

static inline int cpu_is_mmp3_a0(void)
{
	if (cpu_is_mmp3() && ((mmp_chip_id & 0x00ff0000) == 0x00a00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp3_b0(void)
{
	if (cpu_is_mmp3() && ((mmp_chip_id & 0x00ff0000) == 0x00b00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_mmp3_b0p(void)
{
	if (cpu_is_mmp3() && mmp_soc_stepping == 0x423050)
		return 1;
	else
		return 0;
}
#else
#define cpu_is_mmp3(id)	(0)
#endif

#ifdef CONFIG_CPU_PXA988
static inline int cpu_is_pxa988(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc09) &&
		(((mmp_chip_id & 0xffff) == 0xc988) ||
		((mmp_chip_id & 0xffff) == 0xc928));
}
static inline int cpu_is_pxa986(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc09) &&
		(((mmp_chip_id & 0xffff) == 0xc986) ||
		((mmp_chip_id & 0xffff) == 0xc926));
}
#else
#define cpu_is_pxa988()	(0)
#define cpu_is_pxa986()	(0)
#endif

#endif /* __ASM_MACH_CPUTYPE_H */
