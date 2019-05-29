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
#include <mach/ca9_regs.h>
#include <mach/pxa95x_pm.h>
#include <asm/outercache.h>
#include <mach/pxa3xx-regs.h>
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

#define __nop() do { asm volatile ("nop" : : : "memory"); } while (0)

void flushL2VaRange(phys_addr_t start, phys_addr_t end)
{
	outer_flush_range(VirtualToPhysical(start), VirtualToPhysical(end));
}

void cleanL2VaRange(phys_addr_t start, phys_addr_t end)
{
	/*actually this will clean all because the range > L2 cache size*/
	outer_clean_range(0, 0x10000000);
}
void invalidL2All(void)
{
	outer_inv_all();
}

#define PWRMODE_L2_DIS_IN_C2 0x40

void ca9_enter_idle(unsigned int pwrmode, unsigned int sramaddr, unsigned int l2c_base_address)
{
	struct pl310_context pl310;
	unsigned int remap_addr = VirtualToPhysical(sramaddr);
	unsigned int pmu_context[PMU_DATA_SIZE/sizeof(unsigned int)];
	unsigned int vfp_context[VFP_DATA_SIZE/sizeof(unsigned int)];
	unsigned int debug_context[DEBUG_DATA_SIZE/sizeof(unsigned int)];
	/* Remap 0 physical to SRAM so reset will runs the C2 restore code */
	*remap_c2_reg = ((remap_addr>>13)|1)&0x1fff;
	save_vfp((unsigned int *)&vfp_context);
	save_ca9_debug((unsigned int *)&debug_context);
	save_performance_monitors((unsigned int *)&pmu_context);

	ca9_enter_c2_wrapper(&pl310, l2c_base_address, pwrmode & PWRMODE_L2_DIS_IN_C2, sramaddr);

	restore_performance_monitors((unsigned int *)&pmu_context);
	restore_ca9_debug((unsigned int *)&debug_context);
	restore_vfp((unsigned int *)&vfp_context);
	*remap_c2_reg = 0;
}
