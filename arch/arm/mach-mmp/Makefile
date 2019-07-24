#
# Makefile for Marvell's PXA168 processors line
#

obj-y				+= common.o devices.o reset.o pmu.o
obj-$(CONFIG_FB_PXA168)		+= onboard/

# SoC support
obj-$(CONFIG_CPU_PXA168)	+= pxa168.o irq-pxa168.o
obj-$(CONFIG_CPU_PXA910)	+= pxa910.o irq-pxa168.o sram.o pxa910-squ.o \
clock.o
obj-$(CONFIG_CPU_PXA988)	+= pxa988.o sram.o pxa910-squ.o reset-pxa988.o pxa988_lowpower.o \
clock-pxa988.o wakeupgen.o acpuclock-pxa988.o gpio-edge.o head_v7.o \
dvfs-pxa988.o stime.o
obj-$(CONFIG_CPU_MMP2)		+= mmp2.o irq-mmp2.o clock.o mmp_dma.o \
sram.o mmp2_sspa.o mmp2_fuse.o clock-audio.o
#obj-$(CONFIG_CPU_MMP3)		+= mmp3.o irq-mmp3.o mmp_dma.o mmp2_sspa.o \
#sram.o delay-mmp3.o clock-mmp3.o dvfs-mmp3.o mmp3_coherent_fix.o
obj-$(CONFIG_CPU_MMP3)		+= mmp3.o irq-mmp3.o delay-mmp3.o clock-mmp3.o dvfs-mmp3.o mmp3_coherent_fix.o

ifeq ($(CONFIG_SMP),y)
obj-$(CONFIG_CPU_MMP3)		+= platsmp.o mmp3_headsmp.o reset-mmp3.o
obj-$(CONFIG_CPU_PXA988)	+= platsmp.o pxa988_headsmp.o
endif

ifeq ($(CONFIG_HOTPLUG_CPU),y)
obj-$(CONFIG_CPU_MMP3)		+= mmp3_hotplug.o
obj-$(CONFIG_CPU_PXA988)	+= pxa988_hotplug.o
endif
ifeq ($(CONFIG_SOC_LOCAL_TIMERS),y)
obj-$(CONFIG_CPU_MMP3)		+= soc_time.o
else ifneq ($(CONFIG_CPU_PXA988),y)
obj-y				+= time.o
obj-$(CONFIG_LOCAL_TIMERS)	+= localtimer.o
endif

ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_CPU_MMP3)		+= pm-mmp3.o
obj-$(CONFIG_CPU_MMP3)		+= suspend.o
obj-$(CONFIG_CPU_MMP2)		+= pm-mmp2.o pm-mmp2-ll.o
obj-$(CONFIG_CPU_PXA988)	+= sleep-pxa988.o

ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_CPU_MMP2)		+= cpuidle-mmp2.o
obj-$(CONFIG_CPU_PXA988)	+= cpuidle-pxa988.o
endif
ifeq ($(CONFIG_CPU_FREQ),y)
obj-$(CONFIG_CPU_MMP2)		+= cpufreq-mmp2.o fc_mmp2.o board_fc_ops.o mmp2_fc_entry.o brownstone_fc_ll.o brownstone_fc_ops.o
endif

endif

ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_CPU_MMP3)		+= cpuidle-mmp3.o
endif

# board support
obj-$(CONFIG_MACH_ASPENITE)	+= aspenite.o
obj-$(CONFIG_MACH_ZYLONITE2)	+= aspenite.o
obj-$(CONFIG_MACH_AVENGERS_LITE)+= avengers_lite.o
obj-$(CONFIG_MACH_TAVOREVB)	+= tavorevb.o
obj-$(CONFIG_MACH_TTC_DKB)	+= ttc_dkb.o
obj-$(CONFIG_MACH_BROWNSTONE)	+= brownstone.o
obj-$(CONFIG_MACH_FLINT)	+= flint.o
obj-$(CONFIG_MACH_MARVELL_JASPER) += jasper.o
obj-$(CONFIG_MACH_TETON_BGA)	+= teton_bga.o
obj-$(CONFIG_MACH_ABILENE)	+= abilene.o
obj-$(CONFIG_MACH_QSEVEN)	+= qseven.o
obj-$(CONFIG_MACH_MK2)		+= mk2.o
obj-$(CONFIG_MACH_YELLOWSTONE)	+= yellowstone.o
obj-$(CONFIG_MACH_ORCHID)	+= orchid.o
obj-$(CONFIG_MACH_THUNDERSTONEM)+= thunderstonem.o
obj-$(CONFIG_MACH_EMEIDKB)	+= board-emeidkb.o
obj-$(CONFIG_CPU_FREQ_MMP3)	+= cpufreq-mmp3.o
obj-$(CONFIG_CPU_FREQ_PXA910)	+= cpufreq-pxa910.o acpuclock-pxa910.o acpuclock-pxa910_ll.o
ifeq ($(CONFIG_CPU_FREQ),y)
obj-$(CONFIG_CPU_PXA988)	+= cpufreq-pxa988.o
endif
ifeq ($(CONFIG_SUSPEND),y)
obj-$(CONFIG_CPU_PXA910)	+= pxa910_pm.o
obj-$(CONFIG_CPU_PXA988)	+= pm-pxa988.o
endif
