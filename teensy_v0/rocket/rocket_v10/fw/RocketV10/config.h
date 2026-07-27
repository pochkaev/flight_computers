#pragma once

#include <Arduino.h>
#include "rocket_console.h"

// Route existing command/status output through the selected console transport.
// rocket_console.cpp does not include this macro and accesses native USB.
#define Serial rocketConsole

// Firmware identity
#define ROCKET_FW_VERSION       "rv10.20260727b"
#define NAND_RECORD_FORMAT_V4   4
#define ATTITUDE_ESTIMATOR_VERSION 7

// LoRa RFM95
#define LORA_FREQUENCY_HZ   915E6
#define LORA_CS_PIN         10
#define LORA_RST_PIN        9
#define LORA_DIO0_PIN       2
#define LORA_SPI_FREQ_HZ    8000000
#define LORA_TX_TIMEOUT_MS  1000u
#define LORA_TX_POWER_DBM   17
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH_HZ 125000L
#define LORA_CODING_RATE_DENOMINATOR 5

// GPS GT-U7
#define GPS_SERIAL          Serial1
#define GPS_BAUD            9600
#define UART_SERVICE_BAUD   115200

// External status LED
// Recommended wiring: pin -> 330R -> LED anode, LED cathode -> GND
#define STATUS_LED_PIN      3
#define STATUS_LED_ACTIVE_HIGH 1

// Finder buzzer and local service button.
// Recommended buzzer wiring for small active/passive piezo:
//   D5 -> buzzer +, buzzer - -> GND
// For louder buzzers, drive D5 into a small NPN/MOSFET instead of powering
// the buzzer directly from the Teensy pin.
#define BUZZER_PIN          5
#define BUZZER_USE_TONE     1
#define BUTTON_PIN          4
#define BUTTON_ACTIVE_LOW   1
#define BUTTON_RESET_HOLD_MS 2000u

// Removable SAFE/ARM pin microswitch.
// Wiring: switch C -> GND, switch NC -> D16. With INPUT_PULLUP, a pin that
// presses the lever opens NC and reads HIGH (SAFE); removing the pin closes
// NC and reads LOW (ARM).
#define ARM_SWITCH_PIN          16
#define ARM_SWITCH_SAFE_LEVEL   HIGH
#define ARM_SWITCH_DEBOUNCE_MS  500u
#define ARM_SWITCH_DISARM_MS    1000u
#define ARM_SAFE_BOOT_REQUIRED  1
#define LANDED_FINDER_BEEP_ENABLE 1
#define LANDED_FINDER_FAST_MS 180000u
#define LANDED_FINDER_MEDIUM_MS 600000u
#define LANDED_FINDER_FAST_PERIOD_MS 2500u
#define LANDED_FINDER_MEDIUM_PERIOD_MS 8000u
#define LANDED_FINDER_SLOW_PERIOD_MS 30000u

// HPR-style pyro/event channels.
// Current safe default is log-only: pins are initialized to the inactive level,
// but no output pulse is produced until PYRO_OUTPUT_ENABLE and the matching
// per-channel PYRO_CHx_OUTPUT_ENABLE are both set to 1.
// Function letters:
//   N = disabled, A = apogee/drogue, M = main, B = booster separation,
//   I = sustainer ignition, 1 = airstart 1, 2 = airstart 2.
// Use a MOSFET/transistor driver, gate/base pulldown, external arming switch,
// and current-limited pyro battery before enabling real outputs. Do not drive
// an e-match directly from a Teensy GPIO pin.
#define PYRO_OUTPUT_ENABLE       0
#define PYRO_ACTIVE_HIGH         1
#define PYRO_FIRE_MS             1000u
#define PYRO_FLIGHT_PROFILE      'S'   // S = single-stage, 2 = two-stage, A = airstart
#define PYRO_APOGEE_DELAY_MS     1000u
#define PYRO_MAIN_MIN_AFTER_APOGEE_MS 1000u
#define PYRO_STAGING_MIN_REL_ALT_M    20.0f
#define PYRO_STAGING_MAX_TILT_DEG     45.0f
#define PYRO_BOOSTER_SEP_DELAY_MS     1000u
#define PYRO_SUSTAINER_FIRE_DELAY_MS  1000u
#define PYRO_AIRSTART1_EVENT          'I'   // I = liftoff, B = booster burnout/coast
#define PYRO_AIRSTART1_DELAY_MS       1000u
#define PYRO_AIRSTART2_EVENT          '1'   // 1 = airstart 1 fired, B = airstart 1 burnout proxy
#define PYRO_AIRSTART2_DELAY_MS       1000u

#define PYRO_CH1_PIN         6
#define PYRO_CH1_FUNC        'A'
#define PYRO_CH1_LOG_ENABLE  1
#define PYRO_CH1_OUTPUT_ENABLE 0
#define PYRO_CH2_PIN         7
#define PYRO_CH2_FUNC        'M'
#define PYRO_CH2_LOG_ENABLE  1
#define PYRO_CH2_OUTPUT_ENABLE 0
#define PYRO_CH3_PIN         8
#define PYRO_CH3_FUNC        'N'
#define PYRO_CH3_LOG_ENABLE  1
#define PYRO_CH3_OUTPUT_ENABLE 0
#define PYRO_CH4_PIN         15
#define PYRO_CH4_FUNC        'N'
#define PYRO_CH4_LOG_ENABLE  1
#define PYRO_CH4_OUTPUT_ENABLE 0

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
#define NAND_LOG_UPDATE_MS     100
#define NAND_IMU_LOG_UPDATE_MS 5
#define NAND_MAG_LOG_UPDATE_MS 40
#define RECOVERY_SD_LOG_UPDATE_MS 2000
#define RECOVERY_NAND_LOG_UPDATE_MS 2000
#define LOG_FLUSH_MS        1000
#define STATUS_PRINT_MS     1000
#define LED_UPDATE_MS       50

// Flight-critical recorder policy:
// - NAND is the in-flight binary black box.
// - SD remains available for configuration and SAFE-only export, but is not
//   opened or written by the runtime logger.
// - The 200 Hz IMU record already contains attitude angles, so the separate
//   200 Hz quaternion stream is disabled until the recorder is redesigned.
#define SD_RUNTIME_LOG_ENABLE       0
#define SD_CONFIG_LOAD_ENABLE       0
#define NAND_ATTITUDE_LOG_ENABLE    0
#define NAND_PERIODIC_HEADER_UPDATE 0

// Runtime timing acceptance instrumentation.
#define FLIGHT_KERNEL_DEADLINE_US 5000u
#define LOOP_MAJOR_STALL_US      100000u

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

// LittleFS QSPI writes have measured worst-case pauses near 80 ms. Once
// physically ARMED, hold flight records in RAM and commit after a terminal
// state so filesystem latency cannot delay flight-state evaluation.
#define NAND_FLIGHT_RAM1_BYTES        (160UL * 1024UL)
#define NAND_FLIGHT_RAM2_BYTES        (416UL * 1024UL)
#define NAND_PRELAUNCH_RAM_BYTES       (64UL * 1024UL)
#define NAND_FLIGHT_CRITICAL_RESERVE_BYTES (64UL * 1024UL)
#define NAND_DESCENT_IMU_LOG_UPDATE_MS 20u

// Sensor freshness windows
#define BARO_STALE_MS       1500
#define IMU_STALE_MS        1500
#define GPS_STALE_MS        5000

// Phase-aware quaternion estimator. During boost the accelerometer is
// dominated by thrust, not gravity, so attitude propagates from the gyro.
// Accel/mag references are accepted only when both magnitude and innovation
// checks pass. These values affect attitude diagnostics only; launch/recovery
// continue to use the existing raw sensor evidence.
#define IMU_ACCEL_CORRECT_MIN_G 0.82f
#define IMU_ACCEL_CORRECT_MAX_G 1.18f
#define IMU_ACCEL_INNOVATION_MAX_DEG 35.0f
#define IMU_MAG_INNOVATION_MAX_DEG 45.0f
#define IMU_ACCEL_CORRECTION_RATE 3.0f
#define IMU_MAG_CORRECTION_RATE   0.7f
#define IMU_STATIONARY_ACCEL_TOLERANCE_G 0.05f
#define IMU_STATIONARY_GYRO_MAX_DPS 3.0f
#define IMU_GYRO_BIAS_LEARNING_RATE 0.15f
#define IMU_MAG_CORRECT_MIN_UT  10.0f
#define IMU_MAG_CORRECT_MAX_UT  90.0f
#define IMU_ACCEL_RANGE_G       16
#define IMU_GYRO_RANGE_DPS      2000
#define IMU_MAG_RANGE_GAUSS     4

// Flight-state confirmation thresholds
#define LAUNCH_AXIAL_ACCEL_G      1.35f
#define LAUNCH_ACCEL_VEL_MPS      8.0f
#define LAUNCH_REL_ALT_M          3.0f
#define LAUNCH_VEL_MPS            8.0f
#define LAUNCH_BARO_REL_ALT_M     8.0f
#define LAUNCH_BARO_VEL_MPS       8.0f
#define LAUNCH_OBVIOUS_REL_ALT_M  25.0f
#define LAUNCH_OBVIOUS_VEL_MPS    2.0f
#define LAUNCH_CONFIRM_MS         120
#define LAUNCH_PAD_SETTLE_MS      2500
#define LAUNCH_POWERON_INHIBIT_MS 60000u
#define LAUNCH_PAD_STILL_ARM_MS   10000u
#define LAUNCH_PAD_STILL_ACCEL_ERR_G 0.20f
#define LAUNCH_PAD_STILL_GYRO_DPS 12.0f
#define LAUNCH_PAD_VERTICAL_MIN_G 0.70f
#define LAUNCH_PAD_VERTICAL_MAX_G 1.30f
#define LAUNCH_PAD_TRANSVERSE_MAX_G 0.45f
#define LAUNCH_TREND_MS           150
#define LAUNCH_TREND_MIN_M        2.50f
#define LAUNCH_IMU_CONFIRM_MS      50u
#define LAUNCH_IMU_MIN_DELTA_V_MPS 1.0f
#define LAUNCH_IMU_EVIDENCE_MAX_AGE_MS 30u
#define LAUNCH_CANDIDATE_TIMEOUT_MS 3000u
#define BARO_VEL_WINDOW_MS        120
#define BARO_MAX_RAW_VEL_MPS      120.0f
#define BARO_STATE_EVIDENCE_MAX_AGE_MS 80u
#define COAST_VEL_MPS             5.0f
#define COAST_MIN_AFTER_LAUNCH_MS 750
#define COAST_CONFIRM_MS          250
#define APOGEE_VEL_MPS           -0.5f
#define APOGEE_MIN_REL_ALT_M      30.0f
#define APOGEE_CONFIRM_MS         250
#define RECOVERY_SUBSONIC_COAST_REL_ALT_M 25.0f
#define RECOVERY_SUBSONIC_COAST_VEL_MPS 8.0f
#define RECOVERY_NEAR_APOGEE_REL_ALT_M 25.0f
#define RECOVERY_NEAR_APOGEE_FALLBACK_REL_ALT_M 60.0f
#define RECOVERY_NEAR_APOGEE_ABS_VEL_MPS 2.0f
#define RECOVERY_DESCENT_REL_ALT_M 25.0f
#define RECOVERY_DESCENT_VEL_MPS  -8.0f
#define RECOVERY_UNDER_DROGUE_REL_ALT_M 25.0f
#define RECOVERY_UNDER_DROGUE_FAST_VEL_MPS -8.0f
#define RECOVERY_POST_FLIGHT_MAX_REL_ALT_M 20.0f
#define RECOVERY_CLASSIFY_CONFIRM_MS 1000u
#define RECOVERY_POST_FLIGHT_CONFIRM_MS 5000u
#define DUAL_DEPLOY_MAIN_ALT_M    153.0f
#define DUAL_DEPLOY_MAIN_MIN_APOGEE_MARGIN_M 20.0f
#define ENABLE_LOW_ENERGY_DIRECT_LANDED 0
#define LOW_ENERGY_DIRECT_LANDED_MAX_ALT_M 8.0f
#define LANDED_ABS_VEL_MPS        0.7f
#define LANDED_MAX_REL_ALT_M      20.0f
#define LANDED_MIN_AFTER_LAUNCH_MS 3000
#define LANDED_CONFIRM_MS         2000
#define LANDED_STILL_CONFIRM_MS   3000
#define LANDED_STILL_ACCEL_ERR_G  0.20f
#define LANDED_STILL_GYRO_DPS     12.0f
#define TOUCHDOWN_IMPACT_G        3.0f
#define TOUCHDOWN_MIN_AFTER_APOGEE_MS 15000u
#define TOUCHDOWN_SOFT_VEL_MPS    1.5f
#define TOUCHDOWN_SOFT_CONFIRM_MS 3000u
#define TOUCHDOWN_POST_FLIGHT_CONFIRM_MS 5000u
#define TOUCHDOWN_LANDED_CONFIRM_MS 10000u

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
#ifndef SERIAL_DEBUG_LEVEL
#define SERIAL_DEBUG_LEVEL  0
#endif

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
#define PKT_TYPE_PYRO_CONFIG_V1 0x05
#define PKT_TYPE_PYRO_EVENT_V1  0x06

#define PYRO_CONFIG_TX_MS 10000u

// Health bits for status packet
#define HEALTH_BARO_OK      (1u << 0)
#define HEALTH_IMU_OK       (1u << 1)
#define HEALTH_GPS_OK       (1u << 2)
#define HEALTH_SD_OK        (1u << 3)
#define HEALTH_NAND_OK      (1u << 4)
#define HEALTH_LOG_OK       (1u << 5)
#define HEALTH_BATT_OK      (1u << 6)

// Launch readiness status sent in the reserved status-packet bytes.
#define LAUNCH_STATUS_BOOT_WAIT       0
#define LAUNCH_STATUS_SAFE_REQUIRED   1
#define LAUNCH_STATUS_SAFE            2
#define LAUNCH_STATUS_PAD_SETTLE      3
#define LAUNCH_STATUS_SENSOR_FAULT    4
#define LAUNCH_STATUS_LOG_FAULT       5
#define LAUNCH_STATUS_BATT_CRIT       6
#define LAUNCH_STATUS_HOLD_VERTICAL   7
#define LAUNCH_STATUS_HOLD_STILL      8
#define LAUNCH_STATUS_ARMING          9
#define LAUNCH_STATUS_READY          10
#define LAUNCH_STATUS_LAUNCH_CHECK   11
#define LAUNCH_STATUS_FLIGHT         12
