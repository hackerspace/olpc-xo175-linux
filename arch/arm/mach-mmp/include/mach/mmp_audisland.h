#ifndef __MACH_MMP_AUDISLAND_H
#define __MACH_MMP_AUDISLAND_H

#include <mach/addr-map.h>

#if defined(CONFIG_CPU_MMP2)

#define AUD_PHYS_BASE		(AXI_PHYS_BASE + 0xA0000)
#define AUD_VIRT_BASE		(AXI_VIRT_BASE + 0xA0000)
#define AUD_PHYS_SIZE		0x00003000

#elif defined(CONFIG_CPU_MMP3)

#define AUD_PHYS_BASE		0xc0ffd000
#define AUD_VIRT_BASE		0xfeffd000
#define AUD_PHYS_SIZE		0x00003000
#define AUD_PHYS_BASE2		0xc0140000
#define AUD_VIRT_BASE2		0xfef40000
#define AUD_PHYS_SIZE2		0x00010000

#else
#error "unknown platform for audio island"
#endif

#endif /* __MACH_MMP_AUDISLAND_H */
