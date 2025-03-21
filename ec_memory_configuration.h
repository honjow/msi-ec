#ifndef __MSI_EC_REGISTERS_CONFIG__
#define __MSI_EC_REGISTERS_CONFIG__

#include <linux/types.h>
#include <linux/device.h>

#define MSI_EC_DRIVER_NAME "msi-ec"
#define MSI_EC_HWMON_NAME "msi_ec"

#define MSI_EC_ADDR_UNKNOWN 0xff01 // unknown address
#define MSI_EC_ADDR_UNSUPP  0xff01 // unsupported parameter

// Firmware info addresses are universal
#define MSI_EC_FW_VERSION_ADDRESS 0xa0
#define MSI_EC_FW_DATE_ADDRESS    0xac
#define MSI_EC_FW_TIME_ADDRESS    0xb4
#define MSI_EC_FW_VERSION_LENGTH  12
#define MSI_EC_FW_DATE_LENGTH     8
#define MSI_EC_FW_TIME_LENGTH     8

struct msi_ec_webcam_conf {
	int address;
	int block_address;
	int bit;
};

struct msi_ec_fn_win_swap_conf {
	int address;
	int bit;
	bool invert;
};

struct msi_ec_cooler_boost_conf {
	int address;
	int bit;
};

#define MSI_EC_MODE_NULL { NULL, 0 }
struct msi_ec_mode {
	const char *name;
	int value;
};

#define MSI_EC_SHIFT_MODE_NAME_LIMIT 20
struct msi_ec_shift_mode_conf {
	int address;
	struct msi_ec_mode modes[5]; // fixed size for easier hard coding
};

struct msi_ec_super_battery_conf {
	int address;
	int mask;
};

struct msi_ec_fan_mode_conf {
	int address;
	struct msi_ec_mode modes[5]; // fixed size for easier hard coding
};

/**
 * Curve maximum entries (should be more than real maximum for extensibility).
 */
#define CURVE_MAX_ENTRIES 16

/**
 * Persists curve in 
 */
#define CURVE_APPLY_STRATEGY_NORMAL 0

/**
 * Resets curve from ec to default when auto mode is turned on.
 * Required on some devices where auto mode is broken by custom curve in EC.
 */
#define CURVE_APPLY_STRATEGY_RESET_ON_AUTO 1

// Curve start address and entries count.
struct msi_ec_fan_curve {
	int speed_start_address;
	int temperature_start_address;
	int entries_count;

	// Defaults to CURVE_APPLY_STRATEGY_NORMAL
	int apply_strategy;

	int max_speed;
};

struct msi_ec_cpu_conf {
	int rt_temp_address;
	int rt_fan_speed_address; // realtime % RPM
	struct msi_ec_fan_curve fan_curve;
};

struct msi_ec_gpu_conf {
	int rt_temp_address;
	int rt_fan_speed_address; // realtime % RPM
	struct msi_ec_fan_curve fan_curve;
};

struct msi_ec_led_conf {
	int micmute_led_address;
	int mute_led_address;
	int bit;
};

#define MSI_EC_KBD_BL_STATE_MASK 0x3
struct msi_ec_kbd_bl_conf {
	int bl_mode_address;
	int bl_modes[2];
	int max_mode;

	int bl_state_address;
	int state_base_value;
	int max_state;
};

struct msi_ec_conf {
	const char **allowed_fw;

	int charge_control_address;
	struct msi_ec_webcam_conf         webcam;
	struct msi_ec_fn_win_swap_conf    fn_win_swap;
	struct msi_ec_cooler_boost_conf   cooler_boost;
	struct msi_ec_shift_mode_conf     shift_mode;
	struct msi_ec_super_battery_conf  super_battery;
	struct msi_ec_fan_mode_conf       fan_mode;
	struct msi_ec_cpu_conf            cpu;
	struct msi_ec_gpu_conf            gpu;
	struct msi_ec_led_conf            leds;
	struct msi_ec_kbd_bl_conf         kbd_bl;
};

// Struct to hold curve point attribute information
struct msi_ec_curve_attr {
    struct device_attribute dev_attr;
    int fan;        // Fan index (0=CPU, 1=GPU)
    int point;      // Curve point index (1-based)
    int is_pwm;     // 1=PWM, 0=temperature
};

#endif // __MSI_EC_REGISTERS_CONFIG__
