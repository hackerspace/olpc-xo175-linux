/*
 * linux/arch/arm/mach-mmp/include/mach/wistron.h
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_WISTRON_H
#define __ASM_MACH_WISTRON_H

#define PROJ_PROC_INFO_DIR "sysinfo"

#define PROJ_PROC_OS_VER		PROJ_PROC_INFO_DIR"/os_version"
#define PROJ_PROC_BOOTLOADER_VER	PROJ_PROC_INFO_DIR"/bootloader_ver"
#define PROJ_PROC_PCB_VER		PROJ_PROC_INFO_DIR"/pcb_ver"
#define PROJ_PROC_KBC_VER		PROJ_PROC_INFO_DIR"/kbc_ver"
#define PROJ_PROC_DIGITIZER_VER		PROJ_PROC_INFO_DIR"/digitizer_fw_ver"
#define PROJ_PROC_CHARGING_STOP		PROJ_PROC_INFO_DIR"/charging_stop"
#define PROJ_PROC_LEDON			PROJ_PROC_INFO_DIR"/ledon"
#define PROJ_PROC_MANUFACTURER		PROJ_PROC_INFO_DIR"/manufacturer"
#define PROJ_PROC_NVS			PROJ_PROC_INFO_DIR"/wis_NVS"
#define PROJ_PROC_SN_MB			PROJ_PROC_INFO_DIR"/serial_mb"
#define PROJ_PROC_SN_SYSTEM		PROJ_PROC_INFO_DIR"/serial_system"
#define PROJ_PROC_KBC_FIRMWARE		PROJ_PROC_INFO_DIR"/kbc_firmware"
#define PROJ_PROC_KEY_TEST		PROJ_PROC_INFO_DIR"/homebtnmode"
#define PROJ_PROC_MS                    PROJ_PROC_INFO_DIR"/ms"

#define MANUFACTURER			"Wistron"

#define KBC_FIRMWARE_UPDATE_DIR		"/data/data/com.wistron.kbcUpdate"

#define SIZE_BOOTBLOCK		(8192)
#define SIZE_USERCODE		(40960)
#define BUFFER_SIZE		SIZE_USERCODE	//MAX file size

void create_wis_proj_procfs(void);

#define WIS_NVS_HEADER_SIZE	32
#define SN_SIZE			32
#define MAC_SIZE		32
#define LANGUAGE_SIZE		8
#define REGION_SIZE		8
#define BASEBAND_SIZE		12
//#define NVS_OFFSET 18349568	//(0x8c00 - 0x1) * BLOCK_SIZE
#define NVS_OFFSET 15922609664 //0x1DA87DF * BLOCK_SIZE,18349568	//(0x8c00 - 0x1) * BLOCK_SIZE
#define EMMC_BLOCK_SIZE 512

struct WISTRON_NVS		//Note !! Hard define nvs size MAX = 1 block size (512 byte)
{
	char		wisNvs[WIS_NVS_HEADER_SIZE];	//a fixed string idefined as WISNVS
	char		sn_mb[SN_SIZE];			//mainboard serial number
	char		sn_system[SN_SIZE];		//system serial number
	char	        bt_mac[MAC_SIZE];		//
	char	        sn_customer[SN_SIZE];		//customer serial number
	char	        language[LANGUAGE_SIZE];	//language
	char	        region[REGION_SIZE];		//region
        char            baseband[BASEBAND_SIZE];	//wifi-only or 3g sku
};

#define KBC_FIRMWARE_UPDATE_NORMAL 		0x0
#define KBC_FIRMWARE_UPDATING_USERCODE		0x1
#define KBC_FIRMWARE_UPDATING_BOOTBLOCK		0x2
#define KBC_FIRMWARE_UPDATE_USERCODE_FAIL	0x11
#define KBC_FIRMWARE_UPDATE_BOOTBLOCK_FAIL	0x21
#define KBC_FIRMWARE_UPDATE_USERCODE_OK		0x12
#define KBC_FIRMWARE_UPDATE_BOOTBLOCK_OK	0x22
#define KBC_FIRMWARE_UPDATE_USERCODE_NOT_FOUND	0x13
#define KBC_FIRMWARE_UPDATE_BOOTBLOCK_NOT_FOUND	0x23

#endif
