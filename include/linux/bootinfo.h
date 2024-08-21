#ifndef __LINUX_BOOT_INFO_H_
#define __LINUX_BOOT_INFO_H_
#include <linux/types.h>

enum hw_platform_type {
	HW_PLATFORM_EVB = 0,
	HW_PLATFORM_EVT = 2,
	HW_PLATFORM_DVT1 = 4,
	HW_PLATFORM_DVT1_2 = 6,
#ifdef CONFIG_HW_BOARD_ID_ROW_PRC
	HW_PLATFORM_PRC_DVT2 = 8,
	HW_PLATFORM_ROW_DVT2 = 9,
	HW_PLATFORM_PRC_PVT = 10,
	HW_PLATFORM_ROW_PVT = 11,
#else  /* CONFIG_HW_BOARD_ID_ROW_PRC */
	HW_PLATFORM_DVT2 = 8,
	HW_PLATFORM_PVT = 10,
#endif /* CONFIG_HW_BOARD_ID_ROW_PRC */
	HW_PLATFORM_UNKNOWN,
};

extern bool hw_pm_debug_enable(void);
extern enum hw_platform_type hw_get_platform_type(void);
/*
enum boot_mode_type {
	BOOT_NORMAL,
	BOOT_CHARGING,
	BOOT_RECOVERY,
	BOOT_FASTBOOTD,
        BOOT_FACTORY,
	BOOT_FFBM,
	BOOT_ALARM
}*/

#endif /* __LINUX_BOOT_INFO_H_ */
