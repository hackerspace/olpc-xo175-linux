
ifeq ($(CONFIG_CRASH_DUMP),y)
	__ADDRBASE := 0x06000000
else
ifeq ($(CONFIG_TZ_HYPERVISOR),y)
	__ADDRBASE := 0x00200000
else
	__ADDRBASE := 0x00000000
endif
endif

__ZRELADDR := $(shell /bin/bash -c 'printf "0x%08x" \
	$$[$(TEXT_OFFSET) + $(__ADDRBASE)]')

zreladdr-y := $(__ZRELADDR)
