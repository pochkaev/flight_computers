#pragma once

#include <Arduino.h>

// Firmware identity
#define ROCKET_FW_VERSION       "rv10.20260509c"
#define NAND_RECORD_FORMAT_V4   4
#define ATTITUDE_ESTIMATOR_VERSION 3

// LoRa RFM95
#define LORA_FREQUENCY_HZ   915E6
#define LORA_CS_PIN         10
#define LORA_RST_PIN        9
#define LORA_DIO0_PIN       2
#define LORA_SPI_FREQ_HZ    8000000

// GPS GT-U7
#define GPS_SERIAL          Serial1
#define GPS_BAUD            9600

// External status LED
// Recommended wiring: pin -> 330R -> LED anode, LED cathode -> GND
#define STATUS_LED_PIN      3
#define STATUS_LED_ACTIVE_HIGH 1

// Battery monitor
#define VBAT_PIN            A0
// Two-resistor divider for compact rocket wiring:
// Battery+ -> 100k -> A0 -> 47k -> GND
#define VBAT_R1_OHMS        100000.0f
#define VBAT_R2_OHMS        47000.0f
#define ADC_REF_V           3.3f
#define ADC_MAX_COUNTS      4095.0f

// Battery pack auto-detection:
// Use hysteresis so the detected pack does not flap near the boundary.
#define BATT_2S_DETECT_UP_V   5.15f
#define BATT_2S_DETECT_DOWN_V 4.85f

// Per-pack thresholds
#define BATT_1S_WARN_V      3.85f
#define BATT_1S_CRIT_V      3.70f
#define BATT_2S_WARN_V      6.8f
#define BATT_2S_CRIT_V      6.4f
#define BATT_WARN_HYST_V    0.08f
#define BATT_CRIT_HYST_V    0.05f
#define BATT_FILTER_ALPHA   0.15f

// Sensor settings
#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define MS5607_ADDR_0       0x76
#define MS5607_ADDR_1       0x77

// Timing
#define IMU_UPDATE_MS       5
#define BARO_UPDATE_MS      20
#define BARO_TEMP_UPDATE_MS 200
#define MS5607_CONVERSION_US 10000
#define BATT_UPDATE_MS      100
#define FLIGHT_TX_MS        200
#define NAV_TX_MS           1000
#define STATUS_TX_MS        2000
#define IDENTITY_TX_MS      5000
#define RECOVERY_FLIGHT_TX_MS 1000
#define RECOVERY_NAV_TX_MS    2000
#define RECOVERY_STATUS_TX_MS 5000
#define SD_LOG_UPDATE_MS       200
#define NAND_LOG_UPDATE_MS     20
#define NAND_IMU_LOG_UPDATE_MS 5
#define RECOVERY_SD_LOG_UPDATE_MS 2000
#define RECOVERY_NAND_LOG_UPDATE_MS 2000
#define LOG_FLUSH_MS        1000
#define STATUS_PRINT_MS     1000
#define LED_UPDATE_MS       50

// NAND log rotation:
// Before opening a new NAND flight log, delete oldest /fltNNNN.bin files
// until there is enough free space and the file count is below the cap.
// V4 writes full-state plus stream-specific IMU, baro, GPS, battery,
// flight-event, and telemetry records. This is roughly 45 MB/hour before
// filesystem overhead with the current rates.
#define NAND_ROTATE_ENABLE       1
#define NAND_MIN_FREE_BYTES      (16UL * 1024UL * 1024UL)
#define NAND_MAX_LOG_FILES       96
#define NAND_LOG_CACHE_BYTES     16384

// Sensor freshness windows
#define BARO_STALE_MS       1500
#define IMU_STALE_MS        1500
#define GPS_STALE_MS        5000

// Attitude estimator:
// During boost the accelerometer is dominated by thrust, not gravity. Only use
// accelerometer/magnetometer correction when measured acceleration is close to
// 1g; otherwise coast on gyro integration.
#define IMU_ACCEL_CORRECT_MIN_G 0.75f
#define IMU_ACCEL_CORRECT_MAX_G 1.25f
#define IMU_GYRO_ALPHA          0.98f
#define IMU_MAG_YAW_ALPHA       0.995f
#define IMU_MAG_CORRECT_MIN_UT  10.0f
#define IMU_MAG_CORRECT_MAX_UT  90.0f
#define IMU_ACCEL_RANGE_G       16
#define IMU_GYRO_RANGE_DPS      2000
#define IMU_MAG_RANGE_GAUSS     4

// Flight-state confirmation thresholds
#define LAUNCH_ACCEL_G            2.0f
#define LAUNCH_ACCEL_VEL_MPS      8.0f
#define LAUNCH_REL_ALT_M          3.0f
#define LAUNCH_VEL_MPS            8.0f
#define LAUNCH_CONFIRM_MS         120
#define LAUNCH_PAD_SETTLE_MS      2500
#define LAUNCH_POWERON_INHIBIT_MS 60000u
#define LAUNCH_PAD_STILL_ARM_MS   30000u
#define LAUNCH_PAD_STILL_ACCEL_ERR_G 0.20f
#define LAUNCH_PAD_STILL_GYRO_DPS 12.0f
#define LAUNCH_TREND_MS           150
#define LAUNCH_TREND_MIN_M        2.50f
#define BARO_VEL_WINDOW_MS        120
#define BARO_MAX_RAW_VEL_MPS      120.0f
#define COAST_VEL_MPS             5.0f
#define COAST_MIN_AFTER_LAUNCH_MS 750
#define COAST_CONFIRM_MS          250
#define APOGEE_VEL_MPS           -0.5f
#define APOGEE_MIN_REL_ALT_M      30.0f
#define APOGEE_CONFIRM_MS         250
#define ENABLE_LOW_ENERGY_DIRECT_LANDED 0
#define LOW_ENERGY_DIRECT_LANDED_MAX_ALT_M 8.0f
#define LANDED_ABS_VEL_MPS        0.7f
#define LANDED_MAX_REL_ALT_M      20.0f
#define LANDED_MIN_AFTER_LAUNCH_MS 3000
#define LANDED_CONFIRM_MS         2000
#define LANDED_STILL_CONFIRM_MS   3000
#define LANDED_STILL_ACCEL_ERR_G  0.20f
#define LANDED_STILL_GYRO_DPS     12.0f

// GPS as secondary altitude reference:
// GPS is not used to trigger flight states. It is baselined on the pad,
// logged as an independent relative altitude, and used only to flag
// obvious baro/GPS disagreement for post-flight review.
#define GPS_ALT_BASE_ALPHA         0.05f
#define GPS_BARO_DIVERGE_M         75.0f

// Serial debug:
// 0 = disabled for flight
// 1 = boot + basic status line
// 2 = boot + verbose sensor/status line for bench debugging
#define SERIAL_DEBUG_LEVEL  2

// Rocket identity
#define DEFAULT_ROCKET_NAME   "shadow"
// Optional SD config file on the rocket module.
// Example line: rocket_name=shadow
#define ROCKET_CONFIG_PATH    "/rocket_config.txt"

// Packet types
#define PKT_TYPE_FLIGHT_V7  0x01
#define PKT_TYPE_NAV_V7     0x02
#define PKT_TYPE_STATUS_V8  0x03
#define PKT_TYPE_IDENTITY_V1 0x04

// Health bits for status packet
#define HEALTH_BARO_OK      (1u << 0)
#define HEALTH_IMU_OK       (1u << 1)
#define HEALTH_GPS_OK       (1u << 2)
#define HEALTH_SD_OK        (1u << 3)
#define HEALTH_NAND_OK      (1u << 4)
#define HEALTH_LOG_OK       (1u << 5)
#define HEALTH_BATT_OK      (1u << 6)

// Launch readiness status sent in the reserved status-packet bytes.
#define LAUNCH_STATUS_INHIBIT     0
#define LAUNCH_STATUS_WAIT_STILL  1
#define LAUNCH_STATUS_READY       2
#define LAUNCH_STATUS_FLIGHT      3
