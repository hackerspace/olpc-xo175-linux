#
# Makefile for Marvell's PXA168 processors line
#

obj-y				+= common.o clock.o devices.o time.o onboard/

# SoC support
obj-$(CONFIG_CPU_PXA168)	+= pxa168.o irq-pxa168.o
obj-$(CONFIG_CPU_PXA910)	+= pxa910.o irq-pxa168.o
obj-$(CONFIG_CPU_MMP2)		+= mmp2.o irq-mmp2.o
obj-$(CONFIG_CPU_MMP3)		+= mmp3.o irq-mmp3.o mmp2_dma.o mmp2_sspa.o sram.o delay-mmp3.o
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_LOCAL_TIMERS)	+= localtimer.o

ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_CPU_MMP3)		+= pm-mmp3.o
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
obj-$(CONFIG_MACH_YELLOWSTONE)	+= yellowstone.o
obj-$(CONFIG_CPU_FREQ_MMP3)	+= cpufreq-mmp3.o
obj-$(CONFIG_CPU_IDLE)		+= cpuidle-mmp3.o
obj-$(CONFIG_CPU_FREQ_PXA910)	+= cpufreq-pxa910.o acpuclock-pxa910.o acpuclock-pxa910_ll.o
obj-$(CONFIG_SUSPEND)		+= pxa910_pm.o
