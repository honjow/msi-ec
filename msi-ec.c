// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * msi-ec.c - Embedded Controller driver for MSI laptops.
 *
 * This driver registers a platform driver at /sys/devices/platform/msi-ec
 * The list of supported attributes is available in the docs/sysfs-platform-msi-ec file
 *
 * In addition to these platform device attributes the driver
 * registers itself in the Linux power_supply subsystem and is
 * available to userspace under /sys/class/power_supply/<power_supply>:
 *
 *   charge_control_start_threshold
 *   charge_control_end_threshold
 *
 * This driver also registers available led class devices for
 * mute, micmute and keyboard_backlight leds
 *
 * This driver might not work on other laptops produced by MSI. Also, and until
 * future enhancements, no DMI data are used to identify your compatibility
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "ec_memory_configuration.h"

#include <acpi/battery.h>
#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rtc.h>
#include <linux/string_choices.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

static DEFINE_MUTEX(ec_set_by_mask_mutex);
static DEFINE_MUTEX(ec_unset_by_mask_mutex);
static DEFINE_MUTEX(ec_set_bit_mutex);
// ============================================================ //
// Hwmon device data
// ============================================================ //

struct msi_ec_hwmon_data {
	struct device *dev;
	const char *name;
};

static struct msi_ec_hwmon_data *hwmon_data;
static struct device *hwmon_dev;

#define SM_ECO_NAME		"eco"
#define SM_COMFORT_NAME		"comfort"
#define SM_SPORT_NAME		"sport"
#define SM_TURBO_NAME		"turbo"

#define FM_AUTO_NAME		"auto"
#define FM_SILENT_NAME		"silent"
#define FM_BASIC_NAME		"basic"
#define FM_ADVANCED_NAME	"advanced"

static const char *ALLOWED_FW_0[] __initconst = {
	"14C1EMS1.012", // Prestige 14 A10SC
	"14C1EMS1.101",
	"14C1EMS1.102",
	NULL
};

static struct msi_ec_conf CONF0 __initdata = {
	.allowed_fw = ALLOWED_FW_0, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN, // 0xd5 needs testing
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_BASIC_NAME,    0x4d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_1[] __initconst = {
	"17F2EMS1.103", // GF75 Thin 9SC
	"17F2EMS1.104",
	"17F2EMS1.106",
	"17F2EMS1.107",
	NULL
};

static struct msi_ec_conf CONF1 __initdata = {
	.allowed_fw = ALLOWED_FW_1, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_BASIC_NAME,    0x4d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_2[] __initconst = {
	"1552EMS1.115", // Modern 15 A11M
	"1552EMS1.118",
	"1552EMS1.119",
	"1552EMS1.120",
	NULL
};

static struct msi_ec_conf CONF2 __initdata = {
	.allowed_fw = ALLOWED_FW_2, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xD2, // because WMI2 device
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_BASIC_NAME,    0x4d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c, // ?
		.bl_modes         = { 0x00, 0x08 }, // ?
		.max_mode         = 1, // ?
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_3[] __initconst = {
	"1592EMS1.111", // Summit E16 Flip A12UCT / A12MT
	NULL
};

static struct msi_ec_conf CONF3 __initdata = {
	.allowed_fw = ALLOWED_FW_3, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_BASIC_NAME,    0x4d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_4[] __initconst = {
	"16V4EMS1.114", // GS66 Stealth 11UE
	NULL
};

static struct msi_ec_conf CONF4 __initdata = {
	.allowed_fw = ALLOWED_FW_4, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = MSI_EC_ADDR_UNKNOWN, // supported, but unknown
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = { // may be supported, but address is unknown
		.address = MSI_EC_ADDR_UNKNOWN,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68, // needs testing
		.rt_fan_speed_address = 0x71, // needs testing
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNKNOWN,
		.mute_led_address    = MSI_EC_ADDR_UNKNOWN,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN, // ?
		.bl_modes         = { 0x00, 0x08 }, // ?
		.max_mode         = 1, // ?
		.bl_state_address = MSI_EC_ADDR_UNSUPP, // 0xd3, not functional
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_5[] __initconst = {
	"158LEMS1.103", // Alpha 15 B5EE / B5EEK
	"158LEMS1.105",
	"158LEMS1.106",
	NULL
};

static struct msi_ec_conf CONF5 __initdata = {
	.allowed_fw = ALLOWED_FW_5, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNKNOWN,
		.rt_fan_speed_address = MSI_EC_ADDR_UNKNOWN,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP, // 0xf3, not functional (RGB)
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_6[] __initconst = {
	"1542EMS1.102", // GP66 Leopard 10UG / 10UE / 10UH
	"1542EMS1.104",
	NULL
};

static struct msi_ec_conf CONF6 __initdata = {
	.allowed_fw = ALLOWED_FW_6, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xd5,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = MSI_EC_ADDR_UNSUPP,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP, // not functional (RGB)
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_7[] __initconst = {
	"17FKEMS1.108", // Bravo 17 A4DDR / A4DDK
	"17FKEMS1.109",
	"17FKEMS1.10A",
	NULL
};

static struct msi_ec_conf CONF7 __initdata = {
	.allowed_fw = ALLOWED_FW_7, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN, // 0xd5 but has its own set of modes
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d }, // d may not be relevant
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNKNOWN,
		.rt_fan_speed_address = MSI_EC_ADDR_UNKNOWN,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_8[] __initconst = {
	"14F1EMS1.114", // Summit E14 Evo A12M
	"14F1EMS1.115",
	"14F1EMS1.116",
	"14F1EMS1.117",
	"14F1EMS1.118",
	"14F1EMS1.119",
	"14F1EMS1.120",
	NULL
};

static struct msi_ec_conf CONF8 __initdata = {
	.allowed_fw = ALLOWED_FW_8, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x80 }, // 00 - on, 80 - 10 sec auto off
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_9[] __initconst = {
	"14JKEMS1.104", // Modern 14 C5M
	NULL
};

static struct msi_ec_conf CONF9 __initdata = {
	.allowed_fw = ALLOWED_FW_9, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // unsupported or enabled by ECO shift
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // not presented in MSI app
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_10[] __initconst = {
	"1582EMS1.107", // Katana GF66 11UC / 11UD
	NULL
};

static struct msi_ec_conf CONF10 __initdata = {
	.allowed_fw = ALLOWED_FW_10, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = MSI_EC_ADDR_UNSUPP,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xe5,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_11[] __initconst = {
	"16S6EMS1.111", // Prestige 15 A11SCX
	NULL
};

static struct msi_ec_conf CONF11 __initdata = {
	.allowed_fw = ALLOWED_FW_11, // WMI2 based
	.charge_control_address = 0xD7,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNKNOWN,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x4d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = {},
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_12[] __initconst = {
	"16R6EMS1.104", // GF63 Thin 11UC
	"16R6EMS1.106",
	"16R6EMS1.107",
	NULL
};

static struct msi_ec_conf CONF12 __initdata = {
	.allowed_fw = ALLOWED_FW_12, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // 0xeb
		.mask    = 0x0f, // 00, 0f
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_13[] __initconst = {
	"1594EMS1.109", // Prestige 16 Studio A13VE
	NULL
};

static struct msi_ec_conf CONF13 __initdata = {
	.allowed_fw = ALLOWED_FW_13, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4, // 0x00-0x10
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super battery
			{ SM_COMFORT_NAME, 0xc1 }, // balanced
			{ SM_TURBO_NAME,   0xc4 }, // extreme
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP,
		.mask    = 0x0f, // 00, 0f
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c, // KB auto turn off
		.bl_modes         = { 0x00, 0x08 }, // always on; off after 10 sec
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_14[] __initconst = {
	"17L2EMS1.108", // Katana 17 B11UCX, Katana GF76 11UC
	NULL
};

static struct msi_ec_conf CONF14 __initdata = {
	.allowed_fw = ALLOWED_FW_14, // WMI2 based
	.charge_control_address = 0xd7,
	// .usb_share  {
	// 	.address      = 0xbf, // states: 0x08 || 0x28
	// 	.bit          = 5,
	// }
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8, // states: 0x40 || 0x50
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98, // states: 0x02 || 0x82
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2, // Performance Level
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // Low
			{ SM_COMFORT_NAME, 0xc1 }, // Medium
			{ SM_SPORT_NAME,   0xc0 }, // High
			{ SM_TURBO_NAME,   0xc4 }, // Turbo
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // enabled by Low Performance Level
		// .address = 0xeb, // states: 0x00 || 0x0f
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c, // states: 0x00 || 0x02
		.mute_led_address    = 0x2d, // states: 0x04 || 0x06
		.bit                 = 1,
	},
	.kbd_bl = {
		// .bl_mode_address  = 0x2c, // ?
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 }, // ? always on; off after 10 sec
		.max_mode         = 1, // ?
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_15[] __initconst = {
	"15CKEMS1.108", // Delta 15 A5EFK
	NULL
};

static struct msi_ec_conf CONF15 __initdata = {
	.allowed_fw = ALLOWED_FW_15, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xa5 }, // super battery
			{ SM_COMFORT_NAME, 0xa1 }, // balanced
			{ SM_TURBO_NAME,   0xa0 }, // extreme
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN,
		.mask    = 0x0f
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2d,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x01 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP, // RGB
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_16[] __initconst = {
	"155LEMS1.105", // Modern 15 A5M
	"155LEMS1.106",
	NULL
};

static struct msi_ec_conf CONF16 __initdata = {
	.allowed_fw = ALLOWED_FW_16, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN, // 0xed
		.mask    = 0x0f, // a5, a4, a2
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNKNOWN,
		.rt_fan_speed_address = MSI_EC_ADDR_UNKNOWN,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_17[] __initconst = {
	"15K1IMS1.110", // Cyborg 15 A12VF
	"15K1IMS1.112", // Cyborg 15 A13VFK
	"15K1IMS1.113", // Cyborg 15 A13VF
	NULL
};

static struct msi_ec_conf CONF17 __initdata = {
	.allowed_fw = ALLOWED_FW_17, // WMI2 based
	.charge_control_address = 0xd7,
	// .usb_share  {
	// 	.address      = 0xbf, // states: 0x08 || 0x28
	// 	.bit          = 5,
	// }, // Like Katana 17 B11UCX
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4, // 0x01-0x11
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super battery
			{ SM_COMFORT_NAME, 0xc1 }, // balanced
			{ SM_TURBO_NAME,   0xc4 }, // extreme
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb, // 0x0F ( on ) or 0x00 ( off ) on 0xEB
		.mask    = 0x0f, // 00, 0f
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
		// n/rpm register is C9
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c, // KB auto turn off
		.bl_modes         = { 0x00, 0x08 }, // always on; off after 10 sec
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_18[] __initconst = {
	"15HKEMS1.104", // Modern 15 B7M
	NULL
};

static struct msi_ec_conf CONF18 __initdata = {
	.allowed_fw = ALLOWED_FW_18, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // unsupported or enabled by ECO shift
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // not presented in MSI app
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_19[] __initconst = {
	"1543EMS1.113", // GP66 Leopard 11UG / 11U*
	NULL
};

static struct msi_ec_conf CONF19 __initdata = {
	.allowed_fw = ALLOWED_FW_19, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNKNOWN,
		.mute_led_address    = MSI_EC_ADDR_UNKNOWN,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = {},
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_20[] __initconst = {
	"1581EMS1.107", // Katana GF66 11UE / 11UG
	NULL
};

static struct msi_ec_conf CONF20 __initdata = {
	.allowed_fw = ALLOWED_FW_20, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = { // tested
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = { // tested
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = { // tested
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = { // tested
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = { // tested
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = { // tested
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68, // tested
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80, // tested
		.rt_fan_speed_address = 0x89,
	},
	.leds = { // tested
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = { // tested
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // reason: no such setting in the "MSI Center", checked in version 2.0.35
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_21[] __initconst = {
	"16R3EMS1.102", // GF63 Thin 9SC
	"16R3EMS1.104",
	NULL
};

static struct msi_ec_conf CONF21 __initdata = {
	.allowed_fw = ALLOWED_FW_21, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_BASIC_NAME,    0x4d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // Only mode is solid red
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_22[] __initconst = {
	"17LLEMS1.106", // Alpha 17 B5EEK
	NULL
};

static struct msi_ec_conf CONF22 __initdata = {
	.allowed_fw = ALLOWED_FW_22, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super_battery = 0xa5
			{ SM_COMFORT_NAME, 0xc1 }, // super_battery = 0xa4
			{ SM_SPORT_NAME,   0xc1 }, // super_battery = 0xa1
			{ SM_TURBO_NAME,   0xc4 }, // super_battery = 0xa0
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN, // knwon. 0xd5.
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP, // RGB
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_23[] __initconst = {
	"16WKEMS1.105", // MSI Bravo 15 A4DDR (issue #134)
	NULL
};

static struct msi_ec_conf CONF23 __initdata = {
	.allowed_fw = ALLOWED_FW_23, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP, // not in MSI app
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			// values can also be 0x81... when booting on Linux
			{ SM_COMFORT_NAME, 0xc1 }, // Silent / Balanced / AI
			{ SM_ECO_NAME,     0xc2 }, // Super Battery
			{ SM_TURBO_NAME,   0xc4 }, // Performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // enabled by "Super Battery" shift mode?
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			// 'd' is not relevant, values can also be 0x00... or 0x03...
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71, // target speed
		// current RPM speed is 480000/x
		// with x 2 bytes at 0xcc and 0xcd
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
		// current RPM speed is 480000/x
		// with x 2 bytes at 0xca and 0xcb
	},
	.leds = {
		// No LED indicator
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = MSI_EC_ADDR_UNSUPP,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // not in MSI Center
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_24[] __initconst = {
	"14D1EMS1.103", // Modern 14 B10MW (#100)
	NULL
};

static struct msi_ec_conf CONF24 __initdata = {
	.allowed_fw = ALLOWED_FW_24, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2E,
		.block_address = 0x2F,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xBF,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xC2 }, // Super Battery
			{ SM_COMFORT_NAME, 0xC1 }, // + Silent
			{ SM_SPORT_NAME,   0xC0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // not 0xD5, tested
		.mask    = 0x0f,
	},
	.fan_mode = { // Creator Center sets 0x?0 instead of 0x?D
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2B,
		.mute_led_address    = 0x2C,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xF3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_25[] __initconst = {
	"14F1EMS1.209", // Summit E14 Flip Evo A13MT
	"14F1EMS1.211",
	NULL
};

static struct msi_ec_conf CONF25 __initdata = {
	.allowed_fw = ALLOWED_FW_25, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 }, // 00 - on, 08 - 10 sec auto off
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_26[] __initconst = {
	"14DLEMS1.105", // Modern 14 B5M
	NULL
};

static struct msi_ec_conf CONF26 __initdata = {
	.allowed_fw = ALLOWED_FW_26, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // Super Battery
			{ SM_COMFORT_NAME, 0xc1 }, // Silent / Balanced / AI
			{ SM_SPORT_NAME,   0xc0 }, // Performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // 0x33 switches between 0x0D and 0x05
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // not presented in MSI app
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_27[] __initconst = {
	"17S2IMS1.113", // Raider GE78 HX Smart Touchpad 13V
	NULL
};

static struct msi_ec_conf CONF27 __initdata = {
	.allowed_fw = ALLOWED_FW_27, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_28[] __initconst = {
	"1822EMS1.105", // Titan 18 HX A14V
	"1822EMS1.109", // WMI 2.8
	"1822EMS1.111",
	"1822EMS1.112",
	"1822EMS1.114",
	"1822EMS1.115",
	NULL
};

static struct msi_ec_conf CONF28 __initdata = {
	.allowed_fw = ALLOWED_FW_28,
	.charge_control_address = 0xd7,
	// .usb_share  {
	// 	.address      = 0xbf, // states: 0x08 || 0x28
	// 	.bit          = 5,
	// }, // Like Katana 17 B11UCX
	.webcam = {
		.address       = MSI_EC_ADDR_UNSUPP,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4, // 0x01-0x11
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super battery
			{ SM_COMFORT_NAME, 0xc1 }, // balanced
			{ SM_TURBO_NAME,   0xc4 }, // extreme
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb, // 0x0F ( on ) or 0x00 ( off ) on 0xEB
		.mask    = 0x0f, // 00, 0f
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
		// n/rpm register is C9
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // KB auto turn off
		.bl_modes         = { 0x00, 0x08 }, // always on; off after 10 sec
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP, // bugged RGB
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_29[] __initconst = {
	"16V5EMS1.107", // MSI GS66 12UGS
	NULL
};

static struct msi_ec_conf CONF29 __initdata = {
	.allowed_fw = ALLOWED_FW_29,
	.charge_control_address = 0xd7,
	// .usb_share  {
	// 	.address      = 0xbf,
	// 	.bit          = 5,
	// },
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super battery
			{ SM_COMFORT_NAME, 0xc1 }, // balanced
			{ SM_TURBO_NAME,   0xc4 }, // extreme
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = MSI_EC_ADDR_UNSUPP,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_30[] __initconst = {
	"17Q2IMS1.10D", // Titan GT77HX 13VH
	NULL
};

static struct msi_ec_conf CONF30 __initdata = {
	.allowed_fw = ALLOWED_FW_30, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // eco works as expected (much slower, uses less power and lower fan speeds)
			{ SM_COMFORT_NAME, 0xc1 }, // comfort, sport, and turbo all seem to be the same
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNKNOWN,
		.mute_led_address    = MSI_EC_ADDR_UNKNOWN,
		.bit = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNKNOWN,
		.bl_modes         = {},
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_31[] __initconst = {
	"16Q4EMS1.110", // GS65 Stealth
	NULL
};

static struct msi_ec_conf CONF31 __initdata = {
	.allowed_fw = ALLOWED_FW_31,
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4, // 0x00-0x10
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super battery
			{ SM_COMFORT_NAME, 0xc1 }, // balanced
			{ SM_TURBO_NAME,   0xc4 }, // extreme
			{ SM_SPORT_NAME,   0xc0 }, // sport
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // Function not shown in dragon center
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_BASIC_NAME,    0x4c },
			{ FM_AUTO_NAME,     0x0c },
			{ FM_ADVANCED_NAME, 0x8c },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
		// n/rpm register is C9
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = MSI_EC_ADDR_UNSUPP,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // KB auto turn off
		.bl_modes         = { 0x00, 0x08 }, // always on; off after 10 sec
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x81,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_32[] __initconst = {
	"158PIMS1.207", // Bravo 15 B7E
	"158PIMS1.112", // Bravo 15 B7ED
	NULL
};

static struct msi_ec_conf CONF32 __initdata = {
	.allowed_fw = ALLOWED_FW_32,
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = MSI_EC_ADDR_UNSUPP,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { },
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_33[] __initconst = {
	"17N1EMS1.109", // MSI Creator Z17 A12UGST
	NULL
};

static struct msi_ec_conf CONF33 __initdata = {
	.allowed_fw = ALLOWED_FW_33,
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xD2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x4d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_34[] __initconst = {
	"14C6EMS1.109", // Prestige 14 Evo A12M
	NULL
};

static struct msi_ec_conf CONF34 __initdata = {
	.allowed_fw = ALLOWED_FW_34,
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super battery
			{ SM_COMFORT_NAME, 0xc1 }, // silent / balanced
			{ SM_SPORT_NAME,   0xc0 }, // high performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d }, // super battery, balanced and auto high performance modes
			{ FM_SILENT_NAME,   0x1d }, // silent mode
			{ FM_ADVANCED_NAME, 0x4d }, // advanced high performance mode
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNKNOWN,
		.rt_fan_speed_address = MSI_EC_ADDR_UNKNOWN,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 }, // always on / off after 10 sec
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_35[] __initconst = {
	"15M2IMS1.113", // Raider GE68HX 13VG
	NULL
};

static struct msi_ec_conf CONF35 __initdata = {
	.allowed_fw = ALLOWED_FW_35, // WMI2 based
	.charge_control_address = 0xd7,
	// .usb_share = {
	//  	.address      = 0xbf, // states: 0x08 || 0x28
	//  	.bit          = 5,
	// },
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP, // not in MSI app
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_COMFORT_NAME, 0xc1 }, // Silent / Balanced / AI
			{ SM_ECO_NAME,     0xc2 }, // Super Battery
			{ SM_TURBO_NAME,   0xc4 }, // Performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
		// Fan rpm is 480000 / value at combined: c8..c9
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
		// Fan rpm is 480000 / value at combined: ca..cb
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_36[] __initconst = {
	"1585EMS1.115", // MSI Katana 15 B13VFK
	NULL
};

static struct msi_ec_conf CONF36 __initdata = {
	.allowed_fw = ALLOWED_FW_36, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP, // not supported but it is already controlled by hardware
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true, // true because FN key is on right side
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xD2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68, // CPU temperature
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80, // GPU temperature
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_37[] __initconst = {
	"15M1IMS1.113", // Vector GP68 HX 12V
	NULL
};

static struct msi_ec_conf CONF37 __initdata = {
	.allowed_fw = ALLOWED_FW_37, // WMI2 based
	.charge_control_address = 0xd7,
	// .usb_share  {
	// 	.address      = 0xbf, // states: 0x08 || 0x28
	// 	.bit          = 5,
	// }
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = { // also on address 0x91 (?) = 0x5f - normal, 0x50 - silent
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_38[] __initconst = {
	"17E8IMS1.106", // GL75 Leopard 10SCXR/MS-17E8
	"17E8EMS1.101",
	NULL
};

static struct msi_ec_conf CONF38 __initdata = {
	.allowed_fw = ALLOWED_FW_38, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			{ SM_TURBO_NAME,   0xc4 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNKNOWN,
	},
	.fan_mode = {
		.address = 0xf4,
		.modes = {
			{ FM_AUTO_NAME,     0x00 },
			{ FM_ADVANCED_NAME, 0x80 },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNKNOWN,
		.mute_led_address    = MSI_EC_ADDR_UNKNOWN,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};
static const char *ALLOWED_FW_39[] __initconst = {
	"16R8IMS1.117", // Thin GF63 12UC & Thin GF63 12UCX
	NULL
};

static struct msi_ec_conf CONF39 __initdata = {
	.allowed_fw = ALLOWED_FW_39, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,      0xc2},
			{ SM_COMFORT_NAME,  0xc1},
			{ SM_TURBO_NAME,    0xc4},
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,      0x0d},
			{ FM_SILENT_NAME,    0x1d},
			{ FM_ADVANCED_NAME,  0x8d},
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = MSI_EC_ADDR_UNSUPP,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { },
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_40[] __initconst = {
	"17S1IMS1.105", // Raider GE78HX 13VI
	NULL
};

static struct msi_ec_conf CONF40 __initdata = {
	.allowed_fw = ALLOWED_FW_40, // WMI2 based
	.charge_control_address = 0xd7,
	// .usb_share = {
	//  	.address      = 0xbf, // states: 0x08 || 0x28
	//  	.bit          = 5,
	// },
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP, // not in MSI app
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_COMFORT_NAME, 0xc1 }, // Silent / Balanced / AI
			{ SM_ECO_NAME,     0xc2 }, // Super Battery
			{ SM_TURBO_NAME,   0xc4 }, // Performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
		// Fan rpm is 480000 / value at combined: c8..c9
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
		// Fan rpm is 480000 / value at combined: ca..cb
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_401[] __initconst = {
	"1T52EMS1.104", // MSI Claw 8 AI+ A2VM
	NULL
};

static struct msi_ec_conf CONF401 __initdata = {
	.allowed_fw = ALLOWED_FW_401,
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = MSI_EC_ADDR_UNSUPP,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = MSI_EC_ADDR_UNSUPP,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 },
			{ SM_COMFORT_NAME, 0xc1 },
			{ SM_SPORT_NAME,   0xc0 },
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x00 },
			{ FM_SILENT_NAME,   0x10 },
			{ FM_ADVANCED_NAME, 0x80 },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address       = 0x68,
		.rt_fan_speed_address  = 0x71,
		.fan_curve = {
			.speed_start_address = 0x72,
			.temperature_start_address = 0x6a,
			.entries_count = 7,
			.max_speed = 150,
			.apply_strategy = CURVE_APPLY_STRATEGY_RESET_ON_AUTO
		}
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
		.fan_curve = {
			.speed_start_address = 0x8a,
			.temperature_start_address = 0x82,
			.entries_count = 7,
			.max_speed = 150,
			.apply_strategy = CURVE_APPLY_STRATEGY_RESET_ON_AUTO
		}
	},
	.leds = {
		.micmute_led_address = MSI_EC_ADDR_UNSUPP,
		.mute_led_address    = MSI_EC_ADDR_UNSUPP,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // KB auto turn off
		.bl_modes         = { 0x00, 0x08 }, // always on; off after 10 sec
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_41[] __initconst = {
	"15M1IMS2.111", // MSI Vector 16 HX A14VHG
	NULL
};

static struct msi_ec_conf CONF41 __initdata = {
	.allowed_fw = ALLOWED_FW_41, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_COMFORT_NAME, 0xc1 }, // Silent / Balanced / AI
			{ SM_TURBO_NAME,   0xc4 }, // Performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // Function not shown in dragon center
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_42[] __initconst = {
	"14L1EMS1.307", // Modern 14 H D13M
	"14L1EMS1.308",
	NULL
};

static struct msi_ec_conf CONF42 __initdata = {
	.allowed_fw = ALLOWED_FW_42, // WMI2 based
	.charge_control_address = 0xd7,
	.webcam = {
		.address       = MSI_EC_ADDR_UNSUPP,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // super battery
			{ SM_COMFORT_NAME, 0xc1 }, // balanced + silent + ai
			{ SM_TURBO_NAME,   0xc4 }, // extreme performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = MSI_EC_ADDR_UNSUPP,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = 0x2c,
		.bl_modes         = { 0x00, 0x08 }, // 00 - on, 08 - 10 sec auto off
		.max_mode         = 1,
		.bl_state_address = 0xd3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_43[] __initconst = {
	"14DKEMS1.104", // Modern 14 B4MW
	NULL
};

static struct msi_ec_conf CONF43 __initdata = {
	.allowed_fw = ALLOWED_FW_43, // WMI1 based
	.charge_control_address = 0xef,
	.webcam = {
		.address       = 0x2e,
		.block_address = 0x2f,
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xbf,
		.bit     = 4,
		.invert  = false,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xf2,
		.modes = {
			{ SM_ECO_NAME,     0xc2 }, // Super Battery
			{ SM_COMFORT_NAME, 0xc1 }, // Silent / Balanced / AI
			{ SM_SPORT_NAME,   0xc0 }, // Performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = MSI_EC_ADDR_UNSUPP, // 0x33 switches between 0x0D and 0x05
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
	},
	.gpu = {
		.rt_temp_address      = MSI_EC_ADDR_UNSUPP,
		.rt_fan_speed_address = MSI_EC_ADDR_UNSUPP,
	},
	.leds = {
		.micmute_led_address = 0x2b,
		.mute_led_address    = 0x2c,
		.bit                 = 2,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP, // not presented in MSI app
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = 0xf3,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static const char *ALLOWED_FW_44[] __initconst = {
	"17LNIMS1.505", // Katana A17 AI B8VF
	NULL
};

static struct msi_ec_conf CONF44 __initdata = {
	.allowed_fw = ALLOWED_FW_44, // WMI2 based
	.charge_control_address = 0xd7,
	// .usb_share = {
	//  	.address      = 0xbf, // states: 0x08 || 0x28
	//  	.bit          = 5,
	// },
	.webcam = {
		.address       = 0x2e,
		.block_address = MSI_EC_ADDR_UNSUPP, // not in MSI app
		.bit           = 1,
	},
	.fn_win_swap = {
		.address = 0xe8,
		.bit     = 4,
		.invert  = true,
	},
	.cooler_boost = {
		.address = 0x98,
		.bit     = 7,
	},
	.shift_mode = {
		.address = 0xd2,
		.modes = {
			{ SM_COMFORT_NAME, 0xc1 }, // Silent / Balanced / AI
			{ SM_ECO_NAME,     0xc2 }, // Super Battery
			{ SM_TURBO_NAME,   0xc4 }, // Performance
			MSI_EC_MODE_NULL
		},
	},
	.super_battery = {
		.address = 0xeb,
		.mask    = 0x0f,
	},
	.fan_mode = {
		.address = 0xd4,
		.modes = {
			{ FM_AUTO_NAME,     0x0d },
			{ FM_SILENT_NAME,   0x1d },
			{ FM_ADVANCED_NAME, 0x8d },
			MSI_EC_MODE_NULL
		},
	},
	.cpu = {
		.rt_temp_address      = 0x68,
		.rt_fan_speed_address = 0x71,
		// Fan rpm is 480000 / value at combined: c8..c9
	},
	.gpu = {
		.rt_temp_address      = 0x80,
		.rt_fan_speed_address = 0x89,
		// Fan rpm is 480000 / value at combined: ca..cb
	},
	.leds = {
		.micmute_led_address = 0x2c,
		.mute_led_address    = 0x2d,
		.bit                 = 1,
	},
	.kbd_bl = {
		.bl_mode_address  = MSI_EC_ADDR_UNSUPP,
		.bl_modes         = { 0x00, 0x08 },
		.max_mode         = 1,
		.bl_state_address = MSI_EC_ADDR_UNSUPP,
		.state_base_value = 0x80,
		.max_state        = 3,
	},
};

static struct msi_ec_conf *CONFIGURATIONS[] __initdata = {
	&CONF0,
	&CONF1,
	&CONF2,
	&CONF3,
	&CONF4,
	&CONF5,
	&CONF6,
	&CONF7,
	&CONF8,
	&CONF9,
	&CONF10,
	&CONF11,
	&CONF12,
	&CONF13,
	&CONF14,
	&CONF15,
	&CONF16,
	&CONF17,
	&CONF18,
	&CONF19,
	&CONF20,
	&CONF21,
	&CONF22,
	&CONF23,
	&CONF24,
	&CONF25,
	&CONF26,
	&CONF27,
	&CONF28,
	&CONF29,
	&CONF30,
	&CONF31,
	&CONF32,
	&CONF33,
	&CONF34,
	&CONF35,
	&CONF36,
	&CONF37,
	&CONF38,
	&CONF39,
	&CONF40,
	&CONF401,
	&CONF41,
	&CONF42,
	&CONF43,
	&CONF44,
	NULL
};

static bool conf_loaded = false;
static struct msi_ec_conf conf; // current configuration

static bool charge_control_supported = false;

static bool fan_mode_is_available(const char *mode);
static int create_fan_curve_attrs(struct device *dev);
static void remove_fan_curve_attrs(struct device *dev);

static char *firmware = NULL;
module_param(firmware, charp, 0);
MODULE_PARM_DESC(firmware, "Load a configuration for a specified firmware version");

static bool debug = false;
module_param(debug, bool, 0);
MODULE_PARM_DESC(debug, "Load the driver in the debug mode, exporting the debug attributes");

// ============================================================ //
// Helper functions
// ============================================================ //

static int ec_read_seq(u8 addr, u8 *buf, u8 len)
{
	int result;
	for (u8 i = 0; i < len; i++) {
		result = ec_read(addr + i, buf + i);
		if (result < 0)
			return result;
	}
	return 0;
}

static int ec_set_by_mask(u8 addr, u8 mask)
{
	int result;
	u8 stored;

	mutex_lock(&ec_set_by_mask_mutex);
	result = ec_read(addr, &stored);
	if (result < 0)
		goto unlock;

	stored |= mask;
	result = ec_write(addr, stored);

unlock:
	mutex_unlock(&ec_set_by_mask_mutex);
	return result;
}

static int ec_unset_by_mask(u8 addr, u8 mask)
{
	int result;
	u8 stored;

	mutex_lock(&ec_unset_by_mask_mutex);
	result = ec_read(addr, &stored);
	if (result < 0)
		goto unlock;

	stored &= ~mask;
	result = ec_write(addr, stored);

unlock:
	mutex_unlock(&ec_unset_by_mask_mutex);
	return result;
}

static int ec_check_by_mask(u8 addr, u8 mask, bool *output)
{
	int result;
	u8 stored;

	result = ec_read(addr, &stored);
	if (result < 0)
		return result;

	*output = ((stored & mask) == mask);

	return 0;
}

static int ec_set_bit(u8 addr, u8 bit, bool value)
{
	int result;
	u8 stored;

	mutex_lock(&ec_set_bit_mutex);
	result = ec_read(addr, &stored);
	if (result < 0)
		goto unlock;

	if (value)
		stored |= BIT(bit);
	else
		stored &= ~BIT(bit);

	result = ec_write(addr, stored);

unlock:
	mutex_unlock(&ec_set_bit_mutex);
	return result;
}

static int ec_check_bit(u8 addr, u8 bit, bool *output)
{
	int result;
	u8 stored;

	result = ec_read(addr, &stored);
	if (result < 0)
		return result;

	*output = stored & BIT(bit);

	return 0;
}

static int ec_get_firmware_version(u8 buf[MSI_EC_FW_VERSION_LENGTH + 1])
{
	int result;

	memset(buf, 0, MSI_EC_FW_VERSION_LENGTH + 1);
	result = ec_read_seq(MSI_EC_FW_VERSION_ADDRESS, buf,
			     MSI_EC_FW_VERSION_LENGTH);
	if (result < 0)
		return result;

	return MSI_EC_FW_VERSION_LENGTH + 1;
}

static inline const char *str_left_right(bool v)
{
	return v ? "left" : "right";
}

static int direction_is_left(const char *s, bool *res)
{
	if (!s)
		return -EINVAL;

	switch (s[0]) {
	case 'l':
	case 'L':
		*res = true;
		return 0;
	case 'r':
	case 'R':
		*res = false;
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

// ============================================================ //
// Sysfs power_supply subsystem
// ============================================================ //

static int get_end_threshold(u8 *out)
{
	u8 rdata;
	int result;

	result = ec_read(conf.charge_control_address, &rdata);
	if (result < 0)
		return result;

	rdata &= ~BIT(7); // last 7 bits contain the threshold

	// the thresholds are unknown
	if (rdata == 0)
		return -ENODATA;

	if (rdata < 10 || rdata > 100)
		return -EINVAL;

	*out = rdata;
	return 0;
}

static int set_end_threshold(u8 value)
{
	if (value < 10 || value > 100)
		return -EINVAL;

	return ec_write(conf.charge_control_address, value | BIT(7));
}

static ssize_t
charge_control_start_threshold_show(struct device *device,
				    struct device_attribute *attr, char *buf)
{
	int result;
	u8 threshold;

	result = get_end_threshold(&threshold);

	if (result == -ENODATA)
		return sysfs_emit(buf, "0\n");
	else if (result < 0)
		return result;

	return sysfs_emit(buf, "%u\n", threshold - 10);
}

static ssize_t
charge_control_start_threshold_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int result;
	u8 threshold;

	result = kstrtou8(buf, 10, &threshold);
	if (result < 0)
		return result;

	result = set_end_threshold(threshold + 10);
	if (result < 0)
		return result;

	return count;
}

static ssize_t charge_control_end_threshold_show(struct device *device,
						 struct device_attribute *attr,
						 char *buf)
{
	int result;
	u8 threshold;

	result = get_end_threshold(&threshold);

	if (result == -ENODATA)
		return sysfs_emit(buf, "0\n");
	else if (result < 0)
		return result;

	return sysfs_emit(buf, "%u\n", threshold);
}

static ssize_t charge_control_end_threshold_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	int result;
	u8 threshold;

	result = kstrtou8(buf, 10, &threshold);
	if (result < 0)
		return result;

	result = set_end_threshold(threshold);
	if (result < 0)
		return result;

	return count;
}

static DEVICE_ATTR_RW(charge_control_start_threshold);
static DEVICE_ATTR_RW(charge_control_end_threshold);

static struct attribute *msi_battery_attrs[] = {
	&dev_attr_charge_control_start_threshold.attr,
	&dev_attr_charge_control_end_threshold.attr,
	NULL
};

ATTRIBUTE_GROUPS(msi_battery);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0))
static int msi_battery_add(struct power_supply *battery,
			   struct acpi_battery_hook *hook)
#else
static int msi_battery_add(struct power_supply *battery)
#endif
{
	return device_add_groups(&battery->dev, msi_battery_groups);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0))
static int msi_battery_remove(struct power_supply *battery,
			      struct acpi_battery_hook *hook)
#else
static int msi_battery_remove(struct power_supply *battery)
#endif
{
	device_remove_groups(&battery->dev, msi_battery_groups);
	return 0;
}

static struct acpi_battery_hook battery_hook = {
	.add_battery = msi_battery_add,
	.remove_battery = msi_battery_remove,
	.name = MSI_EC_DRIVER_NAME,
};

// ============================================================ //
// Sysfs platform device attributes (root)
// ============================================================ //

static ssize_t webcam_common_show(u8 address, char *buf, bool inverted)
{
	int result;
	bool value;

	result = ec_check_bit(address, conf.webcam.bit, &value);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%s\n", str_on_off(value ^ inverted));
}

static ssize_t webcam_common_store(u8 address,
				   const char *buf,
				   size_t count,
				   bool inverted)
{
	int result;
	bool value;

	result = kstrtobool(buf, &value);
	if (result)
		return result;

	result = ec_set_bit(address, conf.webcam.bit, value ^ inverted);
	if (result < 0)
		return result;

	return count;
}

static ssize_t webcam_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	return webcam_common_show(conf.webcam.address, buf, false);
}

static ssize_t webcam_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	return webcam_common_store(conf.webcam.address, buf, count, false);
}

static ssize_t webcam_block_show(struct device *device,
				 struct device_attribute *attr,
				 char *buf)
{
	return webcam_common_show(conf.webcam.block_address, buf, true);
}

static ssize_t webcam_block_store(struct device *dev,
				  struct device_attribute *attr,
			          const char *buf, size_t count)
{
	return webcam_common_store(conf.webcam.block_address, buf, count, true);
}

static ssize_t fn_key_show(struct device *device, struct device_attribute *attr,
			   char *buf)
{
	int result;
	bool value;

	result = ec_check_bit(conf.fn_win_swap.address, conf.fn_win_swap.bit, &value);
	if (result < 0)
		return result;

	value ^= conf.fn_win_swap.invert; // invert the direction for some laptops
	value = !value; // fn key position is the opposite of win key

	return sysfs_emit(buf, "%s\n", str_left_right(value));
}

static ssize_t fn_key_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int result;
	bool value;

	result = direction_is_left(buf, &value);
	if (result < 0)
		return result;

	value ^= conf.fn_win_swap.invert; // invert the direction for some laptops
	value = !value; // fn key position is the opposite of win key

	result = ec_set_bit(conf.fn_win_swap.address, conf.fn_win_swap.bit, value);

	if (result < 0)
		return result;

	return count;
}

static ssize_t win_key_show(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	int result;
	bool value;

	result = ec_check_bit(conf.fn_win_swap.address, conf.fn_win_swap.bit, &value);
	if (result < 0)
		return result;

	value ^= conf.fn_win_swap.invert; // invert the direction for some laptops

	return sysfs_emit(buf, "%s\n", str_left_right(value));
}

static ssize_t win_key_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int result;
	bool value;

	result = direction_is_left(buf, &value);
	if (result < 0)
		return result;

	value ^= conf.fn_win_swap.invert; // invert the direction for some laptops

	result = ec_set_bit(conf.fn_win_swap.address, conf.fn_win_swap.bit, value);

	if (result < 0)
		return result;

	return count;
}

static ssize_t cooler_boost_show(struct device *device,
				 struct device_attribute *attr, char *buf)
{
	int result;
	bool value;

	result = ec_check_bit(conf.cooler_boost.address, conf.cooler_boost.bit, &value);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%s\n", str_on_off(value));
}

static ssize_t cooler_boost_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int result;
	bool value;

	result = kstrtobool(buf, &value);
	if (result)
		return result;

	result = ec_set_bit(conf.cooler_boost.address, conf.cooler_boost.bit, value);
	if (result < 0)
		return result;

	return count;
}

static ssize_t available_shift_modes_show(struct device *device,
				          struct device_attribute *attr,
				          char *buf)
{
	int result = 0;
	int count = 0;

	for (int i = 0; conf.shift_mode.modes[i].name; i++) {
		// NULL entries have NULL name

		result = sysfs_emit_at(buf, count, "%s\n", conf.shift_mode.modes[i].name);
		if (result < 0)
			return result;
		count += result;
	}

	return count;
}

static ssize_t shift_mode_show(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	u8 rdata;
	int result;

	result = ec_read(conf.shift_mode.address, &rdata);
	if (result < 0)
		return result;

	if (rdata == 0x80)
		return sysfs_emit(buf, "%s\n", "unspecified");

	for (int i = 0; conf.shift_mode.modes[i].name; i++) {
		// NULL entries have NULL name

		if (rdata == conf.shift_mode.modes[i].value) {
			return sysfs_emit(buf, "%s\n", conf.shift_mode.modes[i].name);
		}
	}

	return sysfs_emit(buf, "%s (%i)\n", "unknown", rdata);
}

static ssize_t shift_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int result;

	for (int i = 0; conf.shift_mode.modes[i].name; i++) {
		// NULL entries have NULL name

		if (sysfs_streq(conf.shift_mode.modes[i].name, buf)) {
			result = ec_write(conf.shift_mode.address,
					  conf.shift_mode.modes[i].value);
			if (result < 0)
				return result;

			return count;
		}
	}

	return -EINVAL;
}

static ssize_t super_battery_show(struct device *device,
				  struct device_attribute *attr, char *buf)
{
	int result;
	bool enabled;

	result = ec_check_by_mask(conf.super_battery.address,
				  conf.super_battery.mask,
				  &enabled);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%s\n", str_on_off(enabled));
}

static ssize_t super_battery_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int result;
	bool value;

	result = kstrtobool(buf, &value);
	if (result)
		return result;

	if (value)
		result = ec_set_by_mask(conf.super_battery.address,
				        conf.super_battery.mask);
	else
		result = ec_unset_by_mask(conf.super_battery.address,
					  conf.super_battery.mask);

	if (result < 0)
		return result;

	return count;
}

static ssize_t available_fan_modes_show(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	int result = 0;
	int count = 0;

	for (int i = 0; conf.fan_mode.modes[i].name; i++) {
		// NULL entries have NULL name

		result = sysfs_emit_at(buf, count, "%s\n", conf.fan_mode.modes[i].name);
		if (result < 0)
			return result;
		count += result;
	}

	return count;
}

static int fan_mode_get(const char **const dst) {
	u8 rdata;
	int result;

	result = ec_read(conf.fan_mode.address, &rdata);
	if (result < 0)
		return result;
	for (int i = 0; conf.fan_mode.modes[i].name; i++) {
		// NULL entries have NULL name

		if (rdata == conf.fan_mode.modes[i].value) {
			*dst = conf.fan_mode.modes[i].name;
			return 0;
		}
	}

	if (rdata == 0) return MSI_EC_ADDR_UNSUPP;
	return rdata;
}

static ssize_t fan_mode_show(struct device *device,
			     struct device_attribute *attr, char *buf)
{
	const char *attr_name;
	int status = fan_mode_get(&attr_name);

	if (status < 0) return status;
	else if (status == 0) return sysfs_emit(buf, "%s\n", attr_name);
	else return sysfs_emit(buf, "%s (%i)\n", "unknown", status);
}

/**
 * Callback, used to swap curve to default when fan mode is changed.
 */
static int curve_fan_mode_change(const char *mode);

static ssize_t fan_mode_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int result;

	for (int i = 0; conf.fan_mode.modes[i].name; i++) {
		// NULL entries have NULL name

		if (sysfs_streq(conf.fan_mode.modes[i].name, buf)) {
			const char *mode = conf.fan_mode.modes[i].name;

			result = curve_fan_mode_change(mode);
			if (result < 0)
				return result;

			result = ec_write(conf.fan_mode.address,
					  conf.fan_mode.modes[i].value);
			if (result < 0)
				return result;

			return count;
		}
	}

	return -EINVAL;
}

static ssize_t fw_version_show(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	u8 rdata[MSI_EC_FW_VERSION_LENGTH + 1];
	int result;

	result = ec_get_firmware_version(rdata);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%s\n", rdata);
}

static ssize_t fw_release_date_show(struct device *device,
				    struct device_attribute *attr, char *buf)
{
	u8 rdate[MSI_EC_FW_DATE_LENGTH + 1];
	u8 rtime[MSI_EC_FW_TIME_LENGTH + 1];
	int result;
	struct rtc_time time;

	memset(rdate, 0, sizeof(rdate));
	result = ec_read_seq(MSI_EC_FW_DATE_ADDRESS, rdate,
			     MSI_EC_FW_DATE_LENGTH);
	if (result < 0)
		return result;

	result = sscanf(rdate, "%02d%02d%04d", &time.tm_mon, &time.tm_mday, &time.tm_year);
	if (result != 3)
		return -ENODATA;

	/* the number of months since January and number of years since 1900 */
	time.tm_mon -= 1;
	time.tm_year -= 1900;

	memset(rtime, 0, sizeof(rtime));
	result = ec_read_seq(MSI_EC_FW_TIME_ADDRESS, rtime,
			     MSI_EC_FW_TIME_LENGTH);
	if (result < 0)
		return result;

	result = sscanf(rtime, "%02d:%02d:%02d", &time.tm_hour, &time.tm_min, &time.tm_sec);
	if (result != 3)
		return -ENODATA;

	return sysfs_emit(buf, "%ptR\n", &time);
}

static DEVICE_ATTR_RW(webcam);
static DEVICE_ATTR_RW(webcam_block);
static DEVICE_ATTR_RW(fn_key);
static DEVICE_ATTR_RW(win_key);
static DEVICE_ATTR_RW(cooler_boost);
static DEVICE_ATTR_RO(available_shift_modes);
static DEVICE_ATTR_RW(shift_mode);
static DEVICE_ATTR_RW(super_battery);
static DEVICE_ATTR_RO(available_fan_modes);
static DEVICE_ATTR_RW(fan_mode);
static DEVICE_ATTR_RO(fw_version);
static DEVICE_ATTR_RO(fw_release_date);

static struct attribute *msi_root_attrs[] = {
	&dev_attr_webcam.attr,
	&dev_attr_webcam_block.attr,
	&dev_attr_fn_key.attr,
	&dev_attr_win_key.attr,
	&dev_attr_cooler_boost.attr,
	&dev_attr_available_shift_modes.attr,
	&dev_attr_shift_mode.attr,
	&dev_attr_super_battery.attr,
	&dev_attr_available_fan_modes.attr,
	&dev_attr_fan_mode.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_release_date.attr,
	NULL
};

// ============================================================ //
// Sysfs platform device attributes (fan curves)
// ============================================================ //
struct curve_pack {
	struct msi_ec_fan_curve *curve;
	u8 *curve_temp;
	u8 *curve_temp_default;
	u8 *curve_fan_speed;
	u8 *curve_fan_speed_default;
};

static int is_curve_allowed(struct msi_ec_fan_curve curve) {
	if (
		curve.speed_start_address == MSI_EC_ADDR_UNSUPP || curve.speed_start_address == 0 || 
		curve.temperature_start_address == MSI_EC_ADDR_UNSUPP || curve.temperature_start_address == 0 || 
		curve.entries_count <= 0 || curve.entries_count > CURVE_MAX_ENTRIES 
	) {
		return 0;
	}

	return 1;
}
/**
 * Synchronizes (gets and stores) ec curve to local in-memory curves.
 */
static int sync_ec_curve(struct msi_ec_fan_curve curve, u8 *fan_speed_buf, u8 *temperature_buf) {
	if (!is_curve_allowed(curve)) return -EINVAL;

	for (int i = 0, j = curve.speed_start_address; i < curve.entries_count; i++, j++) {
		if (ec_read(j, &fan_speed_buf[i])) return -EIO;
	}
	for (int i = 0, j = curve.temperature_start_address; i < curve.entries_count - 1; i++, j++) {
		if (ec_read(j, &temperature_buf[i])) return -EIO;
	}

	return 0;
}

/**
 * Writes curve from buffers to ec.
 */
static int push_ec_curve(struct msi_ec_fan_curve curve, const u8 *fan_speed_buf, const u8 *temperature_buf) {
	if (!is_curve_allowed(curve)) return -EINVAL;

	for (int i = 0, j = curve.speed_start_address; i < curve.entries_count; i++, j++) {
		if (ec_write(j, fan_speed_buf[i])) return -EIO;
	}
	for (int i = 0, j = curve.temperature_start_address; i < curve.entries_count - 1; i++, j++) {
		if (ec_write(j, temperature_buf[i])) return -EIO;
	}

	return 0;
}

/**
 * A wrapper for sync_ec_curve. Checks ability and safety to overwrite curve buffers.
 */
static int sync_ec_curve_safe(struct msi_ec_fan_curve curve, u8 *fan_speed_buf, u8 *temperature_buf) {
	const char *fan_mode;

	if (curve.apply_strategy == CURVE_APPLY_STRATEGY_RESET_ON_AUTO) {
		if (fan_mode_get(&fan_mode)) return -ENODATA;

		if (strcmp(fan_mode, FM_ADVANCED_NAME)) return 0;
	}

	return sync_ec_curve(curve, fan_speed_buf, temperature_buf);
}

/**
 * A wrapper for push_ec_curve. Checks ability and safety to write curve.
 */
static int push_ec_curve_safe(struct msi_ec_fan_curve curve, const u8 *fan_speed_buf, const u8 *temperature_buf) {
	const char *fan_mode;

	pr_info("msi-ec: (push_ec_curve_safe) checking fan mode, apply_strategy: %d\n", curve.apply_strategy);
	if (curve.apply_strategy == CURVE_APPLY_STRATEGY_RESET_ON_AUTO) {
		if (fan_mode_get(&fan_mode)) return -ENODATA;

		if (strcmp(fan_mode, FM_ADVANCED_NAME)) {
			pr_info("msi-ec: (push_ec_curve_safe) fan mode is [%s], not advanced, skipping\n", fan_mode);
			return 0;
		}
	}

	return push_ec_curve(curve, fan_speed_buf, temperature_buf);
}



/**
 * Curve is represented in format:
 * s0 t1 s1 t2 s2 t3 s3 ... t(n-1) s(n-1) t(n) s(n)
 *
 * Notice that here is no leading temperature as it represents `less_than_t1`
 */

static ssize_t print_curve(const u8 *fan_speed_buf, const u8 *temperature_buf, int entries, char *buf) {
	char str[128];
	int i = 0;

	int sz = 2 * entries - 1;
	for (int j = 0, sc = 0, tc = 0; j < sz; j++) {
		if (j % 2 == 0) {
			i += sprintf(str + i, "%d ", fan_speed_buf[sc++]);
		} else {
			i += sprintf(str + i, "%d ", temperature_buf[tc++]);
		}
	}
	str[i - 1] = '\0';

	return sysfs_emit(buf, "%s\n", str);
}

static ssize_t read_curve(u8 *fan_speed_buf, u8 *temperature_buf, int entries, const char *buf, size_t count) {
	int sz = 2 * entries - 1;
	int data[2 * CURVE_MAX_ENTRIES];

	const char *sdata = buf;
	int offset;
	for (int i = 0; i < sz; i++) {
		if (sdata >= buf + count) return -EINVAL;
		if (sscanf(sdata, "%u%n", &data[i], &offset) != 1)
			return -EINVAL;

		if (data[i] >= 256) return -EINVAL;
		sdata += offset;
	}
	if (sdata < buf + count && *sdata == '\n') sdata++;
	if (sdata != buf + count) return -EINVAL;

	u8 temp_speed_buf[CURVE_MAX_ENTRIES];
	u8 temp_temperature_buf[CURVE_MAX_ENTRIES];

	for (int j = 0, sc = 0, tc = 0; j < sz; j++) {
		if (j % 2 == 0) {
			temp_speed_buf[sc++] = (u8)data[j];
		} else {
			temp_temperature_buf[tc++] = (u8)data[j];
		}
	}

	int late_temp = 0;
	// Validate buffs.
	for (int i = 0; i < entries - 1; i++) {
		if (late_temp >= temp_temperature_buf[i] || temp_temperature_buf[i] > 100) return -EINVAL;
		late_temp = temp_temperature_buf[i];
	}
	for (int i = 0; i < entries; i++) {
		if (temp_speed_buf[i] > 150) return -EINVAL;
	}

	for (int i = 0; i < entries; i++) {
		fan_speed_buf[i] = temp_speed_buf[i];
		temperature_buf[i] = temp_temperature_buf[i];
	}

	return count;
}

static ssize_t curve_show(struct curve_pack curve_data, char *buf) {
	int status = sync_ec_curve_safe(*curve_data.curve, curve_data.curve_fan_speed, curve_data.curve_temp);
	if (status < 0) return status;

	return print_curve(curve_data.curve_fan_speed, curve_data.curve_temp, curve_data.curve->entries_count, buf);
}
static ssize_t curve_store(struct curve_pack curve_data, const char *buf, size_t count) {

	ssize_t scount = read_curve(curve_data.curve_fan_speed, curve_data.curve_temp, conf.cpu.fan_curve.entries_count, buf, count);
	if (scount < 0) return scount;

	int status = push_ec_curve_safe(*curve_data.curve, curve_data.curve_fan_speed, curve_data.curve_temp);
	if (status < 0) return status;

	return scount;
}

static int curve_init(struct curve_pack curve_data) {
	if (is_curve_allowed(*curve_data.curve)) {
		int status = sync_ec_curve(*curve_data.curve, 
			     curve_data.curve_fan_speed_default, curve_data.curve_temp_default);

		if (status < 0) return status;

		pr_info("msi-ec: Initialized curve with %d entries (addr: speed=%d, temp=%d)\n", 
                curve_data.curve->entries_count,
                curve_data.curve->speed_start_address,
                curve_data.curve->temperature_start_address);

        for (int i = 0; i < curve_data.curve->entries_count; i++) {
            pr_info("msi-ec:   Point %d: Fan speed = %d%%, Temp = %d°C\n", 
                    i + 1, 
                    curve_data.curve_fan_speed_default[i],
                    i < curve_data.curve->entries_count - 1 ? curve_data.curve_temp_default[i] : 0);
        }

		for (int i = 0; i < CURVE_MAX_ENTRIES; i++) {
			curve_data.curve_fan_speed[i] = curve_data.curve_fan_speed_default[i];
			curve_data.curve_temp[i] = curve_data.curve_temp_default[i];
		}
	}

	return 0;
}

static int curve_destroy(struct curve_pack curve_data) {
	if (is_curve_allowed(*curve_data.curve)) {
		pr_info("msi-ec: Destroying curve with %d entries (addr: speed=%d, temp=%d)\n", 
                curve_data.curve->entries_count,
                curve_data.curve->speed_start_address,
                curve_data.curve->temperature_start_address);
        
        for (int i = 0; i < curve_data.curve->entries_count; i++) {
            pr_info("msi-ec:   Point %d: Fan speed = %d%%, Temp = %d°C\n", 
                    i + 1, 
                    curve_data.curve_fan_speed_default[i],
                    i < curve_data.curve->entries_count - 1 ? curve_data.curve_temp_default[i] : 0);
        }
		int status = push_ec_curve(*curve_data.curve,
			curve_data.curve_fan_speed_default, curve_data.curve_temp_default);

		if (status < 0) return status;
		for (int i = 0; i < CURVE_MAX_ENTRIES; i++) {
			curve_data.curve_fan_speed[i] = curve_data.curve_fan_speed_default[i];
			curve_data.curve_temp[i] = curve_data.curve_temp_default[i];
		}
	}
	return 0;
}



/**
 * Used to store curve data.
 */
static u8 cpu_curve_temp[CURVE_MAX_ENTRIES];
static u8 cpu_curve_fan_speed[CURVE_MAX_ENTRIES];

/**
 * Default settings of curve. Used to backup curve on module exit and fan mode switch 
 * (to avoid issues on some devices).
 */
static u8 cpu_curve_temp_default[CURVE_MAX_ENTRIES];
static u8 cpu_curve_fan_speed_default[CURVE_MAX_ENTRIES];
struct curve_pack cpu_curve_package = {
	.curve = &conf.cpu.fan_curve,
	.curve_temp = cpu_curve_temp,
	.curve_fan_speed = cpu_curve_fan_speed,
	.curve_temp_default = cpu_curve_temp_default,
	.curve_fan_speed_default = cpu_curve_fan_speed_default
};

static ssize_t dev_attr_cpu_curve_show(struct device *dev, struct device_attribute *attr, 
				       char *buf) {
	return curve_show(cpu_curve_package, buf);
}
static ssize_t dev_attr_cpu_curve_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count) {
	return curve_store(cpu_curve_package, buf, count);
}


static struct device_attribute dev_attr_cpu_curve = {
	.attr = {
		.name = "curve",
		.mode = 0644,
	},
	.show = dev_attr_cpu_curve_show,
	.store = dev_attr_cpu_curve_store,
};

/**
 * Used to store curve data.
 */
static u8 gpu_curve_temp[CURVE_MAX_ENTRIES];
static u8 gpu_curve_fan_speed[CURVE_MAX_ENTRIES];

/**
 * Default settings of curve. Used to backup curve on module exit and fan mode switch 
 * (to avoid issues on some devices).
 */
static u8 gpu_curve_temp_default[CURVE_MAX_ENTRIES];
static u8 gpu_curve_fan_speed_default[CURVE_MAX_ENTRIES];
struct curve_pack gpu_curve_package = {
	.curve = &conf.gpu.fan_curve,
	.curve_temp = gpu_curve_temp,
	.curve_fan_speed = gpu_curve_fan_speed,
	.curve_temp_default = gpu_curve_temp_default,
	.curve_fan_speed_default = gpu_curve_fan_speed_default
};

static ssize_t dev_attr_gpu_curve_show(struct device *dev, struct device_attribute *attr, 
				       char *buf) {
	return curve_show(gpu_curve_package, buf);
}
static ssize_t dev_attr_gpu_curve_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count) {
	return curve_store(gpu_curve_package, buf, count);
}

static struct device_attribute dev_attr_gpu_curve = {
	.attr = {
		.name = "curve",
		.mode = 0644,
	},
	.show = dev_attr_gpu_curve_show,
	.store = dev_attr_gpu_curve_store,
};

const struct {
	struct curve_pack *cpu_curve;	
	struct curve_pack *gpu_curve;	
} all_curves = {
	.cpu_curve = &cpu_curve_package,
	.gpu_curve = &gpu_curve_package,
};
#define ALL_CURVES_COUNT 2

static int curve_fan_mode_change(const char *mode) {
	struct curve_pack *all_curves_list[ALL_CURVES_COUNT]= {
		all_curves.cpu_curve,
		all_curves.gpu_curve
	};

	if (!strcmp(mode, FM_ADVANCED_NAME)) {
		for (int i = 0; i < ALL_CURVES_COUNT; i++) {
			struct curve_pack curve_data = *all_curves_list[i];

			if (curve_data.curve->apply_strategy == CURVE_APPLY_STRATEGY_RESET_ON_AUTO &&
				is_curve_allowed(*curve_data.curve)) {

				int status = push_ec_curve(*curve_data.curve, 
					curve_data.curve_fan_speed, curve_data.curve_temp);
				if (status < 0) {
					return status;
				}
			}

		}

	} else {
		for (int i = 0; i < ALL_CURVES_COUNT; i++) {
			struct curve_pack curve_data = *all_curves_list[i];

			if (curve_data.curve->apply_strategy == CURVE_APPLY_STRATEGY_RESET_ON_AUTO &&
				is_curve_allowed(*curve_data.curve)) {

				sync_ec_curve_safe(*curve_data.curve, curve_data.curve_fan_speed, curve_data.curve_temp);
				int status = push_ec_curve(*curve_data.curve, 
					curve_data.curve_fan_speed_default, curve_data.curve_temp_default);
				if (status < 0) {
					return status;
				}
			}

		}
	}

	return 0;
}

// ============================================================ //
// Sysfs platform device attributes (cpu)
// ============================================================ //

static ssize_t cpu_realtime_temperature_show(struct device *device,
					     struct device_attribute *attr,
					     char *buf)
{
	u8 rdata;
	int result;

	result = ec_read(conf.cpu.rt_temp_address, &rdata);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%i\n", rdata);
}

static ssize_t cpu_realtime_fan_speed_show(struct device *device,
					   struct device_attribute *attr,
					   char *buf)
{
	u8 rdata;
	int result;

	result = ec_read(conf.cpu.rt_fan_speed_address, &rdata);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%i\n", rdata);
}

static struct device_attribute dev_attr_cpu_realtime_temperature = {
	.attr = {
		.name = "realtime_temperature",
		.mode = 0444,
	},
	.show = cpu_realtime_temperature_show,
};

static struct device_attribute dev_attr_cpu_realtime_fan_speed = {
	.attr = {
		.name = "realtime_fan_speed",
		.mode = 0444,
	},
	.show = cpu_realtime_fan_speed_show,
};

static struct attribute *msi_cpu_attrs[] = {
	&dev_attr_cpu_realtime_temperature.attr,
	&dev_attr_cpu_realtime_fan_speed.attr,
	&dev_attr_cpu_curve.attr,
	NULL
};

// ============================================================ //
// Sysfs platform device attributes (gpu)
// ============================================================ //

static ssize_t gpu_realtime_temperature_show(struct device *device,
					     struct device_attribute *attr,
					     char *buf)
{
	u8 rdata;
	int result;

	result = ec_read(conf.gpu.rt_temp_address, &rdata);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%i\n", rdata);
}

static ssize_t gpu_realtime_fan_speed_show(struct device *device,
					   struct device_attribute *attr,
					   char *buf)
{
	u8 rdata;
	int result;

	result = ec_read(conf.gpu.rt_fan_speed_address, &rdata);
	if (result < 0)
		return result;

	return sysfs_emit(buf, "%i\n", rdata);
}

static struct device_attribute dev_attr_gpu_realtime_temperature = {
	.attr = {
		.name = "realtime_temperature",
		.mode = 0444,
	},
	.show = gpu_realtime_temperature_show,
};

static struct device_attribute dev_attr_gpu_realtime_fan_speed = {
	.attr = {
		.name = "realtime_fan_speed",
		.mode = 0444,
	},
	.show = gpu_realtime_fan_speed_show,
};

static struct attribute *msi_gpu_attrs[] = {
	&dev_attr_gpu_realtime_temperature.attr,
	&dev_attr_gpu_realtime_fan_speed.attr,
	&dev_attr_gpu_curve.attr,
	NULL
};

// ============================================================ //
// Sysfs platform device attributes (debug)
// ============================================================ //

// Prints an EC memory dump in form of a table
static ssize_t ec_dump_show(struct device *device,
			    struct device_attribute *attr,
			    char *buf)
{
	int count = 0;
	char ascii_row[16]; // not null-terminated

	// print header
	count += sysfs_emit(
		buf,
		"|      | _0 _1 _2 _3 _4 _5 _6 _7 _8 _9 _a _b _c _d _e _f\n"
		"|------+------------------------------------------------\n");

	// print dump
	for (u8 i = 0x0; i <= 0xf; i++) {
		u8 addr_base = i * 16;

		count += sysfs_emit_at(buf, count, "| %#x_ |", i);
		for (u8 j = 0x0; j <= 0xf; j++) {
			u8 rdata;
			int result = ec_read(addr_base + j, &rdata);
			if (result < 0)
				return result;

			count += sysfs_emit_at(buf, count, " %02x", rdata);
			ascii_row[j] = isascii(rdata) && isgraph(rdata) ? rdata : '.';
		}

		count += sysfs_emit_at(buf, count, "  |%.16s|\n", ascii_row);
	}

	return count;
}

// stores a value in the specified EC memory address. Format: "xx=xx", xx - hex u8
static ssize_t ec_set_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	if (count > 6) // "xx=xx\n" - 6 chars
		return -EINVAL;

	int result;

	char addr_s[3], val_s[3];
	result = sscanf(buf, "%2s=%2s", addr_s, val_s);
	if (result != 2)
		return -EINVAL;

	u8 addr, val;

	// convert addr
	result = kstrtou8(addr_s, 16, &addr);
	if (result < 0)
		return result;

	// convert val
	result = kstrtou8(val_s, 16, &val);
	if (result < 0)
		return result;

	// write val to EC[addr]
	result = ec_write(addr, val);
	if (result < 0)
		return result;

	return count;
}

// ec_get. stores the specified EC memory address. MAY BE UNSAFE!!!
static u8 ec_get_addr;

// ec_get. reads and stores the specified EC memory address. Format: "xx", xx - hex u8
static ssize_t ec_get_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	if (count > 3) // "xx\n" - 3 chars
		return -EINVAL;

	int result;
	char addr_s[3];

	result = sscanf(buf, "%2s", addr_s);
	if (result != 1)
		return -EINVAL;

	// convert addr
	result = kstrtou8(addr_s, 16, &ec_get_addr);
	if (result < 0)
		return result;

	return count;
};

// ec_get. prints value of previously stored EC memory address
static ssize_t ec_get_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	u8 rdata;
	int result;

	result = ec_read(ec_get_addr, &rdata);
	if (result < 0)
		return result;

	//	return sysfs_emit(buf, "%02x=%02x\n", ec_get_addr, rdata);
	return sysfs_emit(buf, "%02x\n", rdata);
};

static DEVICE_ATTR_RO(ec_dump);
static DEVICE_ATTR_WO(ec_set);
static DEVICE_ATTR_RW(ec_get);

static struct attribute *msi_debug_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_ec_dump.attr,
	&dev_attr_ec_set.attr,
	&dev_attr_ec_get.attr,
	NULL
};

// ============================================================ //
// Sysfs leds subsystem
// ============================================================ //

static int micmute_led_sysfs_set(struct led_classdev *led_cdev,
				 enum led_brightness brightness)
{
	int result;

	result = ec_set_bit(conf.leds.micmute_led_address, conf.leds.bit, brightness);

	if (result < 0)
		return result;

	return 0;
}

static int mute_led_sysfs_set(struct led_classdev *led_cdev,
			      enum led_brightness brightness)
{
	int result;

	result = ec_set_bit(conf.leds.mute_led_address, conf.leds.bit, brightness);

	if (result < 0)
		return result;

	return 0;
}

static enum led_brightness kbd_bl_sysfs_get(struct led_classdev *led_cdev)
{
	u8 rdata;
	int result = ec_read(conf.kbd_bl.bl_state_address, &rdata);
	if (result < 0)
		return 0;
	return rdata & MSI_EC_KBD_BL_STATE_MASK;
}

static int kbd_bl_sysfs_set(struct led_classdev *led_cdev,
			    enum led_brightness brightness)
{
	// By default, on an unregister event,
	// kernel triggers the setter with 0 brightness.
	if (led_cdev->flags & LED_UNREGISTERING)
		return 0;

	u8 wdata;
	if (brightness < 0 || brightness > 3)
		return -1;
	wdata = conf.kbd_bl.state_base_value | brightness;
	return ec_write(conf.kbd_bl.bl_state_address, wdata);
}

static struct led_classdev micmute_led_cdev = {
	.name = "platform::micmute",
	.max_brightness = 1,
	.brightness_set_blocking = &micmute_led_sysfs_set,
	.default_trigger = "audio-micmute",
};

static struct led_classdev mute_led_cdev = {
	.name = "platform::mute",
	.max_brightness = 1,
	.brightness_set_blocking = &mute_led_sysfs_set,
	.default_trigger = "audio-mute",
};

static struct led_classdev msiacpi_led_kbdlight = {
	.name = "msiacpi::kbd_backlight",
	.max_brightness = 3,
	.flags = LED_BRIGHT_HW_CHANGED,
	.brightness_set_blocking = &kbd_bl_sysfs_set,
	.brightness_get = &kbd_bl_sysfs_get,
};

// ============================================================ //
// Sysfs platform driver
// ============================================================ //

static umode_t msi_ec_is_visible(struct kobject *kobj,
				 struct attribute *attr,
				 int idx)
{
	int address;

	if (!conf_loaded)
		return 0;

	/* root group */
	if (attr == &dev_attr_webcam.attr)
		address = conf.webcam.address;

	else if (attr == &dev_attr_webcam_block.attr)
		address = conf.webcam.block_address;

	else if (attr == &dev_attr_fn_key.attr ||
		 attr == &dev_attr_win_key.attr)
		address = conf.fn_win_swap.address;

	else if (attr == &dev_attr_cooler_boost.attr)
		address = conf.cooler_boost.address;

	else if (attr == &dev_attr_available_shift_modes.attr ||
		 attr == &dev_attr_shift_mode.attr)
		address = conf.shift_mode.address;

	else if (attr == &dev_attr_super_battery.attr)
		address = conf.super_battery.address;

	else if (attr == &dev_attr_available_fan_modes.attr ||
		 attr == &dev_attr_fan_mode.attr)
		address = conf.fan_mode.address;

	/* cpu group */
	else if (attr == &dev_attr_cpu_realtime_temperature.attr)
		address = conf.cpu.rt_temp_address;

	else if (attr == &dev_attr_cpu_realtime_fan_speed.attr)
		address = conf.cpu.rt_fan_speed_address;

	/* gpu group */
	else if (attr == &dev_attr_gpu_realtime_temperature.attr)
		address = conf.gpu.rt_temp_address;

	else if (attr == &dev_attr_gpu_realtime_fan_speed.attr)
		address = conf.gpu.rt_fan_speed_address;

	/* default */
	else
		return attr->mode;

	return address == MSI_EC_ADDR_UNSUPP ? 0 : attr->mode;
}

static struct attribute_group msi_root_group = {
	.is_visible = msi_ec_is_visible,
	.attrs = msi_root_attrs,
};

static struct attribute_group msi_cpu_group = {
	.name = "cpu",
	.is_visible = msi_ec_is_visible,
	.attrs = msi_cpu_attrs,
};
static struct attribute_group msi_gpu_group = {
	.name = "gpu",
	.is_visible = msi_ec_is_visible,
	.attrs = msi_gpu_attrs,
};

static const struct attribute_group msi_debug_group = {
	.name = "debug",
	.attrs = msi_debug_attrs,
};

/* the debug group is created separately if needed */
static const struct attribute_group *msi_platform_groups[] = {
	&msi_root_group,
	&msi_cpu_group,
	&msi_gpu_group,
	NULL
};

static int __init msi_platform_probe(struct platform_device *pdev)
{
	int status;

	if (debug) {
		int result = sysfs_create_group(&pdev->dev.kobj,
						&msi_debug_group);
		if (result < 0)
			return result;
	}

	status = curve_init(cpu_curve_package);
	if (status < 0) return status;

	status = curve_init(gpu_curve_package);
	if (status < 0) return status;

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0))
static void msi_platform_remove(struct platform_device *pdev)
#else
static int msi_platform_remove(struct platform_device *pdev)
#endif
{
	if (debug)
		sysfs_remove_group(&pdev->dev.kobj, &msi_debug_group);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
	return 0;
#endif
}

static struct platform_device *msi_platform_device;

static struct platform_driver msi_platform_driver = {
	.driver = {
		.name = MSI_EC_DRIVER_NAME,
		.dev_groups = msi_platform_groups,
	},
	.remove = msi_platform_remove,
};

// ============================================================ //
// Module load/unload
// ============================================================ //

// must be called before msi_platform_probe()
static int __init load_configuration(void)
{
	int result;

	char *ver;
	char ver_by_ec[MSI_EC_FW_VERSION_LENGTH + 1]; // to store version read from EC

	if (firmware) {
		// use fw version passed as a parameter
		ver = firmware;
	} else {
		// get fw version from EC
		result = ec_get_firmware_version(ver_by_ec);
		if (result < 0)
			return result;

		ver = ver_by_ec;
	}

	// load the suitable configuration, if exists
	for (int i = 0; CONFIGURATIONS[i]; i++) {
		if (match_string(CONFIGURATIONS[i]->allowed_fw, -1, ver) != -EINVAL) {
			memcpy(&conf,
			       CONFIGURATIONS[i],
			       sizeof(struct msi_ec_conf));
			conf.allowed_fw = NULL;
			conf_loaded = true;
			return 0;
		}
	}

	// debug mode works regardless of whether the firmware is supported
	if (debug)
		return 0;

	pr_err("Your firmware version is not supported!\n");
	return -EOPNOTSUPP;
}

// ============================================================ //
// Hwmon functions (curve)
// ============================================================ //

enum {
	PWM_ENABLE_FULL = 0,
	PWM_ENABLE_MANUAL,
	PWM_ENABLE_AUTO,
	PWM_ENABLE_SILENT,
	PWM_ENABLE_BASIC,
};

static int virtual_hwmon_pwm_enable[2] = {-1, -1};

// Array to hold dynamically created attributes
static struct msi_ec_curve_attr *curve_attrs;
static int curve_attrs_count;

static ssize_t pwm_enable_available_show(struct device *dev, struct device_attribute *attr, char *buf);

// Forward declarations
static ssize_t curve_attr_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t curve_attr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

// Fan curve points count attributes
static ssize_t pwm1_auto_points_count_show(struct device *dev, 
                                         struct device_attribute *attr,
                                         char *buf)
{
    return sysfs_emit(buf, "%d\n", conf.cpu.fan_curve.entries_count);
}

static ssize_t pwm2_auto_points_count_show(struct device *dev, 
                                         struct device_attribute *attr,
                                         char *buf)
{
    return sysfs_emit(buf, "%d\n", conf.gpu.fan_curve.entries_count);
}

static DEVICE_ATTR_RO(pwm_enable_available);
static DEVICE_ATTR_RO(pwm1_auto_points_count);
static DEVICE_ATTR_RO(pwm2_auto_points_count);

// Common show function for all curve point attributes
static ssize_t curve_attr_show(struct device *dev, 
                              struct device_attribute *attr,
                              char *buf)
{
    struct msi_ec_curve_attr *curve_attr = 
        container_of(attr, struct msi_ec_curve_attr, dev_attr);
    int fan = curve_attr->fan;
    int point = curve_attr->point;
    int is_pwm = curve_attr->is_pwm;

    u8 value;

	struct curve_pack *curve_data = (fan == 0) ? &cpu_curve_package : &gpu_curve_package;
	struct msi_ec_fan_curve *conf_curve = (fan == 0) ? &conf.cpu.fan_curve : &conf.gpu.fan_curve;

	int max_speed = conf_curve->max_speed;
	max_speed = max_speed > 0 ? max_speed : 100;

	pr_debug("msi-ec: curve_attr_show - fan=%d, point=%d, is_pwm=%d; max_speed=%d\n", 
			 fan, point, is_pwm, max_speed);

	if (is_pwm) {
		if (point < 1 || point > conf_curve->entries_count) {
			pr_err("msi-ec: Invalid PWM point: %d, max allowed: %d\n", 
					 point, conf_curve->entries_count);
			return -EINVAL;
		}

		value = curve_data->curve_fan_speed[point - 1];
		unsigned int orig_val = value;
		value = value * 255 / max_speed;
		pr_debug("msi-ec: Reading PWM from buffer, index: %d, scaled value: %u (from %u)\n", 
                 point - 1, value, orig_val);
	} else {
		if (point < 1 || point >= conf_curve->entries_count) {
			pr_err("msi-ec: Invalid temperature point: %d, max allowed: %d\n", 
					 point, conf_curve->entries_count);
			return -EINVAL;
		}

		value = curve_data->curve_temp[point - 1];
		pr_debug("msi-ec: Reading temperature from buffer, index: %d, value: %u\n", 
                 point - 1, value);
	}

    return sysfs_emit(buf, "%u\n", value);
}

// Common store function for all curve point attributes
static ssize_t curve_attr_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
    struct msi_ec_curve_attr *curve_attr = 
        container_of(attr, struct msi_ec_curve_attr, dev_attr);
    int fan = curve_attr->fan;
    int point = curve_attr->point;
    int is_pwm = curve_attr->is_pwm;

	pr_info("msi-ec: (curve_attr_store) fan=%d, point=%d, is_pwm=%d\n", 
			fan, point, is_pwm);

	unsigned long val;
    int ret;

	struct curve_pack *curve_data = (fan == 0) ? &cpu_curve_package : &gpu_curve_package;
	struct msi_ec_fan_curve *curve = curve_data->curve;
	u8 *fan_speed_buf = curve_data->curve_fan_speed;
	u8 *temp_buf = curve_data->curve_temp;
	unsigned int max_speed = curve->max_speed;
	max_speed = max_speed > 0 ? max_speed : 100;

    // Parse value from user
    ret = kstrtoul(buf, 10, &val);
    if (ret < 0) {
		pr_err("msi-ec: Failed to parse value from user\n");
		return ret;
	}
    
    // Select proper address based on fan, point and type (PWM or temp)
    if (fan == 0) { // CPU fan
        if (is_pwm) {
            // Make sure point is within range
            if (point < 1 || point > conf.cpu.fan_curve.entries_count) {
				pr_debug("msi-ec: Invalid CPU PWM point: %d, max allowed: %d\n", 
				         point, conf.cpu.fan_curve.entries_count);
				return -EINVAL;
			}
                
            // Validate PWM value (0-255)
            if (val > 255)
                return -EINVAL;

			unsigned long orig_val = val;
			val = val * max_speed / 255;

			fan_speed_buf[point - 1] = (u8)val;
			pr_info("msi-ec: Updating CPU PWM in buffer, index: %d, scaled value: %lu (from %lu)\n", 
                     point - 1, val, orig_val);
        } else {
            // Last point has no temperature value
            if (point < 1 || point >= conf.cpu.fan_curve.entries_count) {
				pr_debug("msi-ec: Invalid CPU temperature point: %d, max allowed: %d\n", 
						 point, conf.cpu.fan_curve.entries_count);
				return -EINVAL;
			}
                
            // Validate temperature value (0-100°C)
            if (val > 100)
                return -EINVAL;

			temp_buf[point - 1] = (u8)val;
			pr_info("msi-ec: Updating CPU temperature in buffer, index: %d, value: %lu\n", 
                     point - 1, val);
        }
    } else { // GPU fan
        // Similar validation for GPU fan...
        if (is_pwm) {
            if (point < 1 || point > conf.gpu.fan_curve.entries_count)
                return -EINVAL;
                
            if (val > 255)
                return -EINVAL;
			
			unsigned long orig_val = val;
			val = val * max_speed / 255;

			fan_speed_buf[point - 1] = (u8)val;
			pr_debug("msi-ec: Updating GPU PWM in buffer, index: %d, scaled value: %lu (from %lu)\n", 
			         point - 1, val, orig_val);
        } else {
            if (point < 1 || point >= conf.gpu.fan_curve.entries_count)
                return -EINVAL;
                
            if (val > 100)
                return -EINVAL;

			temp_buf[point - 1] = (u8)val;
			pr_debug("msi-ec: Updating GPU temperature in buffer, index: %d, value: %lu\n", 
                     point - 1, val);
        }
    }

	// Push curve to EC safely
	ret = push_ec_curve_safe(*curve, fan_speed_buf, temp_buf);
    if (ret < 0) {
        pr_err("msi-ec: Failed to push curve to EC, error: %d\n", ret);
        return ret;
    }
    
    return count;
}

// Create fan curve attributes based on configuration
static int create_fan_curve_attrs(struct device *dev)
{
    int i, idx = 0;
    int cpu_points = conf.cpu.fan_curve.entries_count;
    int gpu_points = conf.gpu.fan_curve.entries_count;
    char name[32];
    int ret;

	pr_debug("msi-ec: create_fan_curve_attrs - CPU points: %d, GPU points: %d\n", 
			 cpu_points, gpu_points);
    
    // Calculate total number of attributes to create
    // CPU: PWM for all points + temp for all except last + points count
    // GPU: Same structure
    curve_attrs_count = cpu_points + (cpu_points - 1) + 
                       gpu_points + (gpu_points - 1) + 2;
    
    // Allocate memory for attributes
    curve_attrs = kzalloc(sizeof(*curve_attrs) * curve_attrs_count, GFP_KERNEL);
    if (!curve_attrs) {
		pr_debug("msi-ec: Failed to allocate memory for curve attributes\n");
		return -ENOMEM;
	}
	
    // Create attributes for CPU fan curve points
    for (i = 1; i <= cpu_points; i++) {
        // PWM attributes
        snprintf(name, sizeof(name), "pwm1_auto_point%d_pwm", i);
        
        curve_attrs[idx].fan = 0;  // CPU
        curve_attrs[idx].point = i;
        curve_attrs[idx].is_pwm = 1;
        
        curve_attrs[idx].dev_attr.attr.name = 
            kstrdup(name, GFP_KERNEL);
        if (!curve_attrs[idx].dev_attr.attr.name) {
            ret = -ENOMEM;
            goto cleanup;
        }
        
        curve_attrs[idx].dev_attr.attr.mode = 0644;
        curve_attrs[idx].dev_attr.show = curve_attr_show;
        curve_attrs[idx].dev_attr.store = curve_attr_store;
        
        ret = device_create_file(dev, &curve_attrs[idx].dev_attr);
        if (ret < 0)
            goto cleanup;
            
        idx++;
        
        // Temperature attributes (except for last point)
        if (i < cpu_points) {
            snprintf(name, sizeof(name), "pwm1_auto_point%d_temp", i);
            
            curve_attrs[idx].fan = 0;  // CPU
            curve_attrs[idx].point = i;
            curve_attrs[idx].is_pwm = 0;
            
            curve_attrs[idx].dev_attr.attr.name = 
                kstrdup(name, GFP_KERNEL);
            if (!curve_attrs[idx].dev_attr.attr.name) {
                ret = -ENOMEM;
                goto cleanup;
            }
            
            curve_attrs[idx].dev_attr.attr.mode = 0644;
            curve_attrs[idx].dev_attr.show = curve_attr_show;
            curve_attrs[idx].dev_attr.store = curve_attr_store;
            
            ret = device_create_file(dev, &curve_attrs[idx].dev_attr);
            if (ret < 0)
                goto cleanup;
                
            idx++;
        }
    }
    
    // Similarly create attributes for GPU fan curve points
    for (i = 1; i <= gpu_points; i++) {
        // PWM attributes
        snprintf(name, sizeof(name), "pwm2_auto_point%d_pwm", i);
        
        curve_attrs[idx].fan = 1;  // GPU
        curve_attrs[idx].point = i;
        curve_attrs[idx].is_pwm = 1;
        
        curve_attrs[idx].dev_attr.attr.name = 
            kstrdup(name, GFP_KERNEL);
        if (!curve_attrs[idx].dev_attr.attr.name) {
            ret = -ENOMEM;
            goto cleanup;
        }
        
        curve_attrs[idx].dev_attr.attr.mode = 0644;
        curve_attrs[idx].dev_attr.show = curve_attr_show;
        curve_attrs[idx].dev_attr.store = curve_attr_store;
        
        ret = device_create_file(dev, &curve_attrs[idx].dev_attr);
        if (ret < 0)
            goto cleanup;
            
        idx++;
        
        // Temperature attributes (except for last point)
        if (i < gpu_points) {
            snprintf(name, sizeof(name), "pwm2_auto_point%d_temp", i);
            
            curve_attrs[idx].fan = 1;  // GPU
            curve_attrs[idx].point = i;
            curve_attrs[idx].is_pwm = 0;
            
            curve_attrs[idx].dev_attr.attr.name = 
                kstrdup(name, GFP_KERNEL);
            if (!curve_attrs[idx].dev_attr.attr.name) {
                ret = -ENOMEM;
                goto cleanup;
            }
            
            curve_attrs[idx].dev_attr.attr.mode = 0644;
            curve_attrs[idx].dev_attr.show = curve_attr_show;
            curve_attrs[idx].dev_attr.store = curve_attr_store;
            
            ret = device_create_file(dev, &curve_attrs[idx].dev_attr);
            if (ret < 0)
                goto cleanup;
                
            idx++;
        }
    }
    
    // Create points count attributes
    ret = device_create_file(dev, &dev_attr_pwm1_auto_points_count);
    if (ret < 0)
        goto cleanup;
        
    ret = device_create_file(dev, &dev_attr_pwm2_auto_points_count);
    if (ret < 0) {
        device_remove_file(dev, &dev_attr_pwm1_auto_points_count);
        goto cleanup;
    }
    
    return 0;
    
cleanup:
    // Remove already created attributes
    for (i = 0; i < idx; i++) {
		if (curve_attrs[i].dev_attr.attr.name) {
			kfree(curve_attrs[i].dev_attr.attr.name);
			// device_remove_file(dev, &curve_attrs[i].dev_attr);
		}
    }
    kfree(curve_attrs);
    curve_attrs = NULL;
	curve_attrs_count = 0;
    return ret;
}

// Remove dynamically created attributes
static void remove_fan_curve_attrs(struct device *dev)
{
    int i;

	if (!dev) {
        pr_debug("msi-ec: Cannot remove attributes, device pointer is NULL\n");
        return;
    }

	if (!curve_attrs) {
        pr_debug("msi-ec: No fan curve attributes to remove\n");
        return;
    }
    
    pr_debug("msi-ec: Removing %d fan curve attributes\n", curve_attrs_count);
    for (i = 0; i < curve_attrs_count; i++) {
        if (curve_attrs[i].dev_attr.attr.name) {
            // device_remove_file(dev, &curve_attrs[i].dev_attr);
            kfree(curve_attrs[i].dev_attr.attr.name);
        }
    }
    kfree(curve_attrs);
    curve_attrs = NULL;
    curve_attrs_count = 0;
	pr_debug("msi-ec: Successfully removed all fan curve attributes\n");
}

// ============================================================ //
// Hwmon functions (other)
// ============================================================ //

// Check if a specific fan mode is available in the configuration
static bool fan_mode_is_available(const char *mode)
{
    int i;
    
    for (i = 0; i < ARRAY_SIZE(conf.fan_mode.modes); i++) {
        if (strcmp(conf.fan_mode.modes[i].name, mode) == 0)
            return true;
    }
    
    return false;
}

static umode_t msi_ec_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
                                       u32 attr, int channel)
{
	switch (type) {
	case hwmon_temp:
		if (attr == hwmon_temp_input && 
		    ((channel == 0 && conf.cpu.rt_temp_address != MSI_EC_ADDR_UNSUPP) || 
		     (channel == 1 && conf.gpu.rt_temp_address != MSI_EC_ADDR_UNSUPP)))
			return 0444;
		break;
	case hwmon_fan:
		if (attr == hwmon_fan_input &&
		    ((channel == 0 && conf.cpu.rt_fan_speed_address != MSI_EC_ADDR_UNSUPP) ||
		     (channel == 1 && conf.gpu.rt_fan_speed_address != MSI_EC_ADDR_UNSUPP)))
			return 0444;
		else if (attr == hwmon_fan_label &&
			 ((channel == 0 && conf.cpu.rt_fan_speed_address != MSI_EC_ADDR_UNSUPP) ||
			  (channel == 1 && conf.gpu.rt_fan_speed_address != MSI_EC_ADDR_UNSUPP)))
			return 0444;
		break;
	case hwmon_pwm:
		if (attr == hwmon_pwm_enable &&
		    ((channel == 0 && conf.cpu.rt_fan_speed_address != MSI_EC_ADDR_UNSUPP) ||
		     (channel == 1 && conf.gpu.rt_fan_speed_address != MSI_EC_ADDR_UNSUPP)))
			return 0644;
		break;
	default:
		break;
	}
	
	return 0;
}

// Helper function to set fan mode using mode name
static int set_fan_mode(const char *mode)
{
    int result;

    for (int i = 0; i < ARRAY_SIZE(conf.fan_mode.modes); i++) {
        // NULL entries have NULL name
        if (conf.fan_mode.modes[i].name && 
            strcmp(conf.fan_mode.modes[i].name, mode) == 0) {
            
            result = curve_fan_mode_change(mode);
            if (result < 0)
                return result;

            result = ec_write(conf.fan_mode.address,
                              conf.fan_mode.modes[i].value);
            if (result < 0)
                return result;

            return 0;
        }
    }
    
    return -EINVAL; // Invalid mode
}

// Helper function to set cooler boost mode
static int set_cooler_boost(bool enable)
{
    int result;
    
    if (conf.cooler_boost.address == MSI_EC_ADDR_UNSUPP)
        return -EINVAL;
    
    result = ec_set_bit(conf.cooler_boost.address, conf.cooler_boost.bit, enable);
    if (result < 0)
        return result;

    return 0;
}

static int msi_ec_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
                             u32 attr, int channel, long val)
{
    int result = 0;
    
    switch (type) {
    case hwmon_fan:
		break;
	case hwmon_pwm:
        if (attr == hwmon_pwm_enable) {
            if (channel == 0 || channel == 1) { // CPU and GPU fans share mode control
                // Change fan mode based on val
				virtual_hwmon_pwm_enable[channel] = val;

                switch (val) {
                case PWM_ENABLE_FULL: // Full speed mode - Cooler Boost
                    if (conf.cooler_boost.address != MSI_EC_ADDR_UNSUPP)
                        result = set_cooler_boost(true);
                    else
                        return -EINVAL;
					virtual_hwmon_pwm_enable[0] = PWM_ENABLE_FULL;
					virtual_hwmon_pwm_enable[1] = PWM_ENABLE_FULL;
                    break;
                case PWM_ENABLE_MANUAL: // Manual mode - Advanced
                    result = set_cooler_boost(false);
                    if (fan_mode_is_available(FM_ADVANCED_NAME))
                        result = set_fan_mode(FM_ADVANCED_NAME);
                    else
                        return -EINVAL;
					virtual_hwmon_pwm_enable[1] = PWM_ENABLE_MANUAL;
					virtual_hwmon_pwm_enable[0] = PWM_ENABLE_MANUAL;
                    break;
                case PWM_ENABLE_AUTO: // Automatic mode - Auto
                    result = set_cooler_boost(false);
					// If both channels are set to automatic mode, apply the change
					if (virtual_hwmon_pwm_enable[1] == PWM_ENABLE_AUTO && 
					    virtual_hwmon_pwm_enable[0] == PWM_ENABLE_AUTO) {
						if (fan_mode_is_available(FM_AUTO_NAME))
						    result = set_fan_mode(FM_AUTO_NAME);
						else
						    return -EINVAL;
					}
                    break;
                case PWM_ENABLE_SILENT: // Silent mode
                    result = set_cooler_boost(false);
                    if (fan_mode_is_available(FM_SILENT_NAME))
                        result = set_fan_mode(FM_SILENT_NAME);
                    else
                        return -EINVAL;
					virtual_hwmon_pwm_enable[0] = PWM_ENABLE_SILENT;
					virtual_hwmon_pwm_enable[1] = PWM_ENABLE_SILENT;
                    break;
                case PWM_ENABLE_BASIC: // Basic mode
                    result = set_cooler_boost(false);
                    if (fan_mode_is_available(FM_BASIC_NAME))
                        result = set_fan_mode(FM_BASIC_NAME);
                    else
                        return -EINVAL;
					virtual_hwmon_pwm_enable[0] = PWM_ENABLE_BASIC;
					virtual_hwmon_pwm_enable[1] = PWM_ENABLE_BASIC;
                    break;
                default:
                    return -EINVAL;
                }
            }
            return result;
        }
        break;
    default:
        break;
    }
    
    return -EINVAL;
}

static int msi_ec_hwmon_read_string(struct device *dev, enum hwmon_sensor_types type,
                                   u32 attr, int channel, const char **str)
{
    switch (type) {
    case hwmon_fan:
        if (attr == hwmon_fan_label) {
            if (channel == 0) {
                *str = "cpu_fan";
                return 0;
            } else if (channel == 1) {
                *str = "gpu_fan";
                return 0;
            }
        }
        break;
    default:
        break;
    }
    
    return -EINVAL;
}

static int msi_ec_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
                             u32 attr, int channel, long *val)
{
	u8 value;
	u8 high_byte, low_byte;
	u16 fan_value;
	int result;
	
	switch (type) {
	case hwmon_temp:
		if (attr == hwmon_temp_input) {
			if (channel == 0 && conf.cpu.rt_temp_address != MSI_EC_ADDR_UNSUPP) {
				result = ec_read_seq(conf.cpu.rt_temp_address, &value, 1);
				if (result < 0)
					return result;
				*val = value * 1000; // Convert to millidegree Celsius
				return 0;
			} else if (channel == 1 && conf.gpu.rt_temp_address != MSI_EC_ADDR_UNSUPP) {
				result = ec_read_seq(conf.gpu.rt_temp_address, &value, 1);
				if (result < 0)
					return result;
				*val = value * 1000; // Convert to millidegree Celsius
				return 0;
			}
		}
		break;
	case hwmon_fan:
		if (attr == hwmon_fan_input) {
			if (channel == 0) { // CPU fan
				// Read high byte (0xC8)
				result = ec_read_seq(0xC8, &high_byte, 1);
				if (result < 0)
					return result;
				
				// Read low byte (0xC9)
				result = ec_read_seq(0xC9, &low_byte, 1);
				if (result < 0)
					return result;
				
				// Combine into complete fan speed value
				fan_value = (high_byte << 8) | low_byte;
				
				// If value is 0, fan might be stopped
				if (fan_value == 0)
					*val = 0;
				else
					*val = 480000 / fan_value; // Apply formula RPM = 480000 / value
				
				return 0;
			} else if (channel == 1) { // GPU fan
				// Read high byte (0xCA)
				result = ec_read_seq(0xCA, &high_byte, 1);
				if (result < 0)
					return result;
				
				// Read low byte (0xCB)
				result = ec_read_seq(0xCB, &low_byte, 1);
				if (result < 0)
					return result;
				
				// Combine into complete fan speed value
				fan_value = (high_byte << 8) | low_byte;
				
				// If value is 0, fan might be stopped
				if (fan_value == 0)
					*val = 0;
				else
					*val = 480000 / fan_value; // Apply formula RPM = 480000 / value
				
				return 0;
			}
		}
		break;
	case hwmon_pwm:
		if (attr == hwmon_pwm_enable) {
			// CPU and GPU fans share mode control, so return the same value for both channels
    		if (channel == 0 || channel == 1) {
				if (virtual_hwmon_pwm_enable[channel] >= 0) {
                	*val = virtual_hwmon_pwm_enable[channel];
                	return 0;
            	}

				int result;
				bool cooler_boost_enabled = false;
				const char *mode_name = NULL;
    		    if (conf.cooler_boost.address != MSI_EC_ADDR_UNSUPP) {
					result = ec_check_bit(conf.cooler_boost.address, conf.cooler_boost.bit, &cooler_boost_enabled);
					if (result < 0)
						return result;
				}
    		    if (cooler_boost_enabled) {
					*val = PWM_ENABLE_FULL; // Full speed mode
					virtual_hwmon_pwm_enable[1] = PWM_ENABLE_FULL;
					virtual_hwmon_pwm_enable[0] = PWM_ENABLE_FULL;
					return 0;
				} else {
					result = fan_mode_get(&mode_name);
					if (result < 0)
						return result;
					
					if (strcmp(mode_name, FM_ADVANCED_NAME) == 0)
						*val = PWM_ENABLE_MANUAL; // Manual mode
					else if (strcmp(mode_name, FM_AUTO_NAME) == 0)
						*val = PWM_ENABLE_AUTO; // Automatic mode
					else if (strcmp(mode_name, FM_SILENT_NAME) == 0)
						*val = PWM_ENABLE_SILENT; // Silent mode
					else if (strcmp(mode_name, FM_BASIC_NAME) == 0)
						*val = PWM_ENABLE_BASIC; // Basic mode
					else
						*val = -1; // Unknown mode
					virtual_hwmon_pwm_enable[1] = *val;
					virtual_hwmon_pwm_enable[0] = *val;
					return 0;
				}
			}
		}
		break;
	default:
		break;
	}
	
	return -EOPNOTSUPP;
}

static ssize_t pwm_enable_available_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = 0;
    const char *mode_names[5] = {NULL}; // Index corresponds to mode value, 0 is not used
    
    // First collect all mode names
    for (int i = 0; conf.fan_mode.modes[i].name; i++) {
        if (strcmp(conf.fan_mode.modes[i].name, FM_ADVANCED_NAME) == 0)
            mode_names[1] = conf.fan_mode.modes[i].name;  // Manual mode
        else if (strcmp(conf.fan_mode.modes[i].name, FM_AUTO_NAME) == 0)
            mode_names[2] = conf.fan_mode.modes[i].name;  // Auto mode
        else if (strcmp(conf.fan_mode.modes[i].name, FM_SILENT_NAME) == 0)
            mode_names[3] = conf.fan_mode.modes[i].name;  // Silent mode
        else if (strcmp(conf.fan_mode.modes[i].name, FM_BASIC_NAME) == 0)
            mode_names[4] = conf.fan_mode.modes[i].name;  // Basic mode
    }
    
    // Add Cooler Boost (if supported)
    if (conf.cooler_boost.address != MSI_EC_ADDR_UNSUPP) {
        mode_names[0] = "full";
    }
    
    // Output in order
    for (int i = 0; i <= 4; i++) {
        if (mode_names[i]) {
            len += scnprintf(buf + len, PAGE_SIZE - len, 
                          "%d: %s\n", i, mode_names[i]);
        }
    }
    
    return len;
}

static const struct hwmon_ops msi_ec_hwmon_ops = {
	.is_visible = msi_ec_hwmon_is_visible,
	.read = msi_ec_hwmon_read,
	.read_string = msi_ec_hwmon_read_string,
	.write = msi_ec_hwmon_write,
};

static const struct hwmon_channel_info *msi_ec_hwmon_info[] = {
	HWMON_CHANNEL_INFO(temp,
                  HWMON_T_INPUT,  // CPU temperature
                  HWMON_T_INPUT), // GPU temperature
	HWMON_CHANNEL_INFO(fan,
				HWMON_F_INPUT | HWMON_F_LABEL,  // CPU fan
				HWMON_F_INPUT | HWMON_F_LABEL), // GPU fan
	HWMON_CHANNEL_INFO(pwm,
				HWMON_PWM_ENABLE,  // CPU fan PWM
				HWMON_PWM_ENABLE), // GPU fan PWM
	NULL
};

static const struct hwmon_chip_info msi_ec_hwmon_chip_info = {
	.ops = &msi_ec_hwmon_ops,
	.info = msi_ec_hwmon_info,
};

static struct attribute *msi_ec_hwmon_attrs[] = {
    &dev_attr_pwm_enable_available.attr,
    NULL
};

static const struct attribute_group msi_ec_hwmon_group = {
    .attrs = msi_ec_hwmon_attrs,
};

static const struct attribute_group *msi_ec_hwmon_groups[] = {
    &msi_ec_hwmon_group,
    NULL
};

static int __init msi_ec_init(void)
{
	int result;

	result = load_configuration();
	if (result < 0)
		return result;

	msi_platform_device = platform_create_bundle(&msi_platform_driver,
						     msi_platform_probe,
						     NULL, 0, NULL, 0);
	if (IS_ERR(msi_platform_device))
		return PTR_ERR(msi_platform_device);

	pr_info("module_init\n");
	if (!conf_loaded)
		return 0;

	/*
	 * Additional check: battery thresholds are supported only if
	 * the 7th bit is set.
	 */
	if (conf.charge_control_address != MSI_EC_ADDR_UNSUPP) {
		result = ec_check_bit(conf.charge_control_address, 7,
				      &charge_control_supported);
		if (result < 0)
			return result;
	}

	if (charge_control_supported)
		battery_hook_register(&battery_hook);

	// register LED classdevs
	if (conf.leds.micmute_led_address != MSI_EC_ADDR_UNSUPP)
		led_classdev_register(&msi_platform_device->dev,
				      &micmute_led_cdev);

	if (conf.leds.mute_led_address != MSI_EC_ADDR_UNSUPP)
		led_classdev_register(&msi_platform_device->dev,
				      &mute_led_cdev);

	if (conf.kbd_bl.bl_state_address != MSI_EC_ADDR_UNSUPP)
		led_classdev_register(&msi_platform_device->dev,
				      &msiacpi_led_kbdlight);

	pr_info("msi-ec: Registering hwmon device\n");

	// register hwmon device
	hwmon_data = devm_kzalloc(&msi_platform_device->dev, sizeof(*hwmon_data), GFP_KERNEL);
	if (!hwmon_data)
		return -ENOMEM;
	
	hwmon_data->dev = &msi_platform_device->dev;
	hwmon_data->name = MSI_EC_HWMON_NAME;
	
	dev_set_drvdata(&msi_platform_device->dev, hwmon_data);
	
	hwmon_dev = hwmon_device_register_with_info(&msi_platform_device->dev,
						      hwmon_data->name, hwmon_data,
						      &msi_ec_hwmon_chip_info, 
						      msi_ec_hwmon_groups);
	
	if (IS_ERR(hwmon_dev)) {
		int err = PTR_ERR(hwmon_dev);
		pr_err("msi-ec: hwmon device register failed with error %d\n", err);
		return err;
	}

	// Add fan curve attributes if advanced mode is available
	if (!IS_ERR(hwmon_dev) && fan_mode_is_available(FM_ADVANCED_NAME)) {
		int result = create_fan_curve_attrs(hwmon_dev);
		if (result < 0) {
			hwmon_device_unregister(hwmon_dev);
			return result;
		}
	}

	pr_info("msi-ec: hwmon device registered successfully\n");

	return 0;
}

static void __exit msi_ec_exit(void)
{
	if (conf_loaded) {
		// unregister LED classdevs
		if (conf.leds.micmute_led_address != MSI_EC_ADDR_UNSUPP)
			led_classdev_unregister(&micmute_led_cdev);

		if (conf.leds.mute_led_address != MSI_EC_ADDR_UNSUPP)
			led_classdev_unregister(&mute_led_cdev);

		if (conf.kbd_bl.bl_state_address != MSI_EC_ADDR_UNSUPP)
			led_classdev_unregister(&msiacpi_led_kbdlight);

		if (charge_control_supported)
			battery_hook_unregister(&battery_hook);
	}

	if (hwmon_dev) {
        pr_info("msi-ec: Removing fan curve attributes before unregistering hwmon\n");
        remove_fan_curve_attrs(hwmon_dev);
    }

	// unregister hwmon device
	if (hwmon_dev)
		hwmon_device_unregister(hwmon_dev);

	// Destroy curve and load default settings.
	curve_destroy(cpu_curve_package);
	curve_destroy(gpu_curve_package);

	platform_device_unregister(msi_platform_device);
	platform_driver_unregister(&msi_platform_driver);

	pr_info("module_exit\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jose Angel Pastrana <japp0005@red.ujaen.es>");
MODULE_AUTHOR("Aakash Singh <mail@singhaakash.dev>");
MODULE_AUTHOR("Nikita Kravets <teackot@gmail.com>");
MODULE_DESCRIPTION("MSI Embedded Controller");
MODULE_VERSION("0.09");

module_init(msi_ec_init);
module_exit(msi_ec_exit);
