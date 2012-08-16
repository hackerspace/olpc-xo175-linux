obj-$(CONFIG_PM_DEVFREQ)	+= devfreq.o
obj-$(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND)	+= governor_simpleondemand.o
obj-$(CONFIG_DEVFREQ_GOV_PERFORMANCE)	+= governor_performance.o
obj-$(CONFIG_DEVFREQ_GOV_POWERSAVE)	+= governor_powersave.o
obj-$(CONFIG_DEVFREQ_GOV_USERSPACE)	+= governor_userspace.o
ifeq ($(CONFIG_CPU_PXA978), y)
obj-$(CONFIG_DDR_DEVFREQ)		+= mck3_memorybus.o
else
obj-$(CONFIG_DDR_DEVFREQ)		+= mck4_memorybus.o
endif
obj-$(CONFIG_VMETA_DEVFREQ)		+= devfreq-vmeta.o
