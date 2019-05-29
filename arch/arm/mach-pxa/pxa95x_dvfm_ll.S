@
@ This program is free software; you can redistribute it and/or modify
@ it under the terms of the GNU General Public License as published by
@ the Free Software Foundation; either version 2 of the License, or
@ (at your option) any later version.
@
@ This program is distributed in the hope that it will be useful,
@ but WITHOUT ANY WARRANTY; without even the implied warranty of
@ MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@ GNU General Public License for more details.
@
@ You should have received a copy of the GNU General Public License
@ along with this program; if not, write to the Free Software
@ Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
@
@
@  FILENAME:       pxa95x_dvfm_ll.S
@
@  PURPOSE:        Provides low level DVFM primitive functions written
@		   specifically for the Monahans/Zylonite processor/platform.
@
@******************************************************************************


@
@ List of primitive functions in this module:
@
    .global pxa_clkcfg_read
    .global pxa_clkcfg_write

.equ     CLKMGR_ACCR_OFFSET,0x0000
.equ     CLKMGR_ACSR_OFFSET,0x0004
.equ     CLKMGR_CCLKCFG_OFFSET,0x0040

.equ     DMEMC_MDCNFG_OFFSET, 0x0000
.equ     DMEMC_DDRHCAL_OFFSET,0x0060
.equ     BPMU_PWRMODE_OFFSET,0x0080

    .text
@
@ UINT32 pxa_clkcfg_read
@
@ Description:
@ 	This routine reads the designated PMU register via CoProcesser 14.
@
@ Input Parameters:
@
@ Returns:
@	r0 - clkcfg value
@
@ Registers Modified:
@	CoProcessor Register Modified: None
@	General Purpose Registers Modified: None
@
@

pxa_clkcfg_read:
	mrc  p14, 0, r0, c6, c0, 0		@ Read clkcfg
	bx lr          @ return



@
@ void pxa_clkcfg_write
@
@ Description:
@ 	This routine writes to the designated ClkCFG register via
@	CoProcesser 14.
@
@ Input Parameters:
@       r0 - arg1 - Value to write to ClkCFG register
@	r1 - arg2 - Clk manager base address for PXA955 implementation

@ Returns:
@	None
@
@ Registers Modified:
@	CoProcessor Register Modified: ClkCFG Register
@	General Purpose Registers Modified: None
@
@ NOTE
@	Error checking not included
@

pxa_clkcfg_write:
	@test whether it's PXA955
	mrc	p15, 0, r2, c0, c0
	and r2,#0x000f0000 @ARMv7 architecture
	cmp r2,#0x000f0000
	beq pxa_clkcfg_write_pxa955

	mcr	 p14, 0, r0, c6, c0, 0	@ Write ClkCFG CO-processor
	b pxa_clkcfg_write_end

pxa_clkcfg_write_pxa955:
	str     r0, [r1, #CLKMGR_CCLKCFG_OFFSET] @ Store to CCLKCFG register
verify_write:
	ldr     r2, [r1, #CLKMGR_CCLKCFG_OFFSET] @ load CCLKCFG register
	cmp	r0, r2
	bne 	verify_write			 @ verify that the write succedded

pxa_clkcfg_write_end:
	bx	lr				 @ return

