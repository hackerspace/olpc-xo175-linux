# SPDX-License-Identifier: GPL-2.0
armada-y	:= armada_crtc.o armada_drv.o armada_fb.o armada_fbdev.o \
		   armada_gem.o armada_overlay.o armada_plane.o armada_trace.o
armada-y	+= armada_510.o
armada-y	+= armada_610.o
armada-$(CONFIG_DEBUG_FS) += armada_debugfs.o

obj-$(CONFIG_DRM_ARMADA) := armada.o

armada-rmem-$(CONFIG_DRM_ARMADA) += armada_rmem.o
obj-y += $(armada-rmem-y) $(armada-rmem-m)
