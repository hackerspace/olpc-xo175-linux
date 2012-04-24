/*  cmd_ca9_idle.c: PXA9XX memory and memory throughput tests
 *
 *	Copyright (C) 2008 MARVELL Corporation (antone@marvell.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <mach/ca9_asm.h>
#include <asm/outercache.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <linux/cpu_pm.h>
#include <mach/pxa95x_pm.h>
#include <asm/io.h>
#include <mach/pxa3xx-regs.h>
#include <linux/delay.h>
#include <asm/hardware/cache-l2x0.h>

/* Part of the code based on the below ARM code and modified */
/*
 * Copyright (C) 2008-2010 ARM Limited
 *
 * This software is provided 'as-is', without any express or implied
 * warranties including the implied warranties of satisfactory quality,
 * fitness for purpose or non infringement.  In no event will  ARM be
 * liable for any damages arising from the use of this software.
 *
 * Permission is granted to anyone to use, copy and modify this software for
 * any purpose, and to redistribute the software, subject to the following
 * restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *		claim that you wrote the original software. If you use this software
 *		in a product, an acknowledgment in the product documentation would be
 *		appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *		misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
static int pxa978_suspend_finish(unsigned long pwrmode)
{
	if (pwrmode & PWRMODE_L2_DIS_IN_C2) {
		/* In L2$ non-retentive mode, two option:
		 *1. clean all before c2, inv all after c2
		 *2. flush all before c2
		 *but we mush use the first one. for after power on, L2 is
		 *in an unpredicatable state of all data, tag, status bit
		 */
		/*this will actually call clean_all() */
		outer_clean_range(0, 0xFFFFFFFF);
	} else if ((pwrmode & 0x07) == PXA95x_PM_S0D1C2) {
		/*WR for NEVO-2344. l2$ content lost in D1*/
		outer_disable();
	}
	pxa978_cpu_suspend(pwrmode);
	return 0;
}

void c2_address_unremap(void)
{
	__raw_writel(0, remap_c2_reg);
}

void c2_address_remap(void)
{
	unsigned c2_addr = VirtualToPhysical(get_c2_sram_base());
	__raw_writel(((c2_addr >> 13) | 1) & 0x1FFF, remap_c2_reg);
}

void pxa978_pm_enter(unsigned long pwrmode)
{
	unsigned int debug_context[DEBUG_DATA_SIZE / sizeof(unsigned int)];
	unsigned int pmu_context[PMU_DATA_SIZE / sizeof(unsigned int)];

	c2_address_remap();
	save_pxa978_debug((unsigned int *)&debug_context);
	save_performance_monitors((unsigned int *)&pmu_context);
	cpu_pm_enter();
	cpu_suspend(pwrmode, pxa978_suspend_finish);

	cpu_pm_exit();
	restore_performance_monitors((unsigned int *)&pmu_context);
	restore_pxa978_debug((unsigned int *)&debug_context);
	c2_address_unremap();
}
