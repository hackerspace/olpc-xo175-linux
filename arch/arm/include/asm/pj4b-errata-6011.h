#ifndef __ASMARM_PJ4B_ERRATA_6011_H
#define __ASMARM_PJ4B_ERRATA_6011_H

#ifdef __ASSEMBLY__
	.macro  pj4b_6011_ldrex, p0, p1, p2
	mrs	\p2, cpsr
	cpsid	i
	ldrex  \p0, [\p1]
	ldrex  \p0, [\p1]
	ldrex  \p0, [\p1]
	ldrex  \p0, [\p1]
	ldrex  \p0, [\p1]
	msr    cpsr_c, \p2
	.endm

	.macro  pj4b_6011_ldrexb, p0, p1, p2
	mrs	\p2, cpsr
	cpsid	i
	ldrexb  \p0, [\p1]
	ldrexb  \p0, [\p1]
	ldrexb  \p0, [\p1]
	ldrexb  \p0, [\p1]
	ldrexb  \p0, [\p1]
	msr    cpsr_c, \p2
	.endm
#endif

#define pj4b_6011_ldrexb(p0, p1, p2) \
	"mrs    "#p2", cpsr\n" \
	"cpsid  i\n" \
	"ldrexb "#p0", ["#p1"]\n" \
	"ldrexb "#p0", ["#p1"]\n" \
	"ldrexb "#p0", ["#p1"]\n" \
	"ldrexb "#p0", ["#p1"]\n" \
	"ldrexb "#p0", ["#p1"]\n" \
	"msr    cpsr_c, "#p2"\n" \

#define pj4b_6011_ldrexh(p0, p1, p2) \
	"mrs    "#p2", cpsr\n" \
	"cpsid  i\n" \
	"ldrexh "#p0", ["#p1"]\n" \
	"ldrexh "#p0", ["#p1"]\n" \
	"ldrexh "#p0", ["#p1"]\n" \
	"ldrexh "#p0", ["#p1"]\n" \
	"ldrexh "#p0", ["#p1"]\n" \
	"msr    cpsr_c, "#p2"\n" \

#define pj4b_6011_ldrex(p0, p1, p2) \
	"mrs    "#p2", cpsr\n" \
	"cpsid	i\n" \
	"ldrex	"#p0", ["#p1"]\n" \
	"ldrex	"#p0", ["#p1"]\n" \
	"ldrex	"#p0", ["#p1"]\n" \
	"ldrex	"#p0", ["#p1"]\n" \
	"ldrex	"#p0", ["#p1"]\n" \
	"msr    cpsr_c, "#p2"\n" \

#define pj4b_6011_ldrexd(p0, p1, p2, p3)  \
	"mrs    "#p3", cpsr\n" \
	"cpsid	i\n" \
	"ldrexd	"#p0", "#p1", ["#p2"]\n" \
	"ldrexd	"#p0", "#p1", ["#p2"]\n" \
	"ldrexd	"#p0", "#p1", ["#p2"]\n" \
	"ldrexd	"#p0", "#p1", ["#p2"]\n" \
	"ldrexd "#p0", "#p1", ["#p2"]\n" \
	"msr    cpsr_c, "#p3"\n" \

#endif
