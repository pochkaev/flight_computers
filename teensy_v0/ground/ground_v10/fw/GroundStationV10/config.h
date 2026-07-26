#pragma once
#include <Arduino.h>

// -------------------------------------------------------------
// Global Configuration for Ground Station
// -------------------------------------------------------------
#define GROUND_FW_VERSION "gv10.20260726j"

// Pins
#define LORA_CS_PIN        10
#define LORA_RST_PIN       9
#define LORA_DIO0_PIN      2

#define SD_CS_PIN          4
#define BUTTON_PIN         5   // Service / page button (active LOW)
#define GROUND_BUZZER_PIN  26  // ARM reminder / status buzzer
#define GROUND_BUZZER_USE_TONE 1

// Launch controller / RS-485 power module (Master)
// Uses MAX3485 / MAX485 transceiver.
#define PWR_RS485_DE_RE_PIN 6      // DE+RE control
#define PWR_RS485_SERIAL    Serial2
#define PWR_RS485_BAUD      9600

// ARM / START switches and LEDs (active-LOW inputs)
#define PWR_ARM_A_PIN       20
#define PWR_ARM_B_PIN       21
#define PWR_START_A_PIN     16
#define PWR_START_B_PIN     17
// Use pins within D1-D23 range for LEDs
#define PWR_LED_A_PIN       22
#define PWR_LED_B_PIN       23

// Local battery measurement for ground-station 1S/2S/3S Li-Po.
// Divider selected from the available resistor kit:
// Vbat -> 100k -> ADC -> 22k -> GND
#define PWR_VBAT_PIN        A0
#define GND_VBAT_R1_OHMS    100000.0f
#define GND_VBAT_R2_OHMS    22000.0f
// Measured calibration from real battery voltage and A0 voltage pairs:
// Vbat = A0_V * 5.61783593 - 0.06534455
#define GND_VBAT_A0_SLOPE   5.61783593f
#define GND_VBAT_A0_OFFSET -0.06534455f
#define ADC_REF_V           3.3f
#define ADC_MAX_COUNTS      4095.0f
#define GND_VBAT_SAMPLE_MS  50u
#define GND_VBAT_AVG_SAMPLES 40u

// LoRa
#define LORA_FREQUENCY     915E6
#define LORA_SPI_FREQ      8000000
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH_HZ 125000L
#define LORA_CODING_RATE_DENOMINATOR 5
#define PKT_TYPE_FLIGHT_V7 0x01
#define PKT_TYPE_NAV_V7    0x02
#define PKT_TYPE_STATUS_V8 0x03
#define PKT_TYPE_IDENTITY_V1 0x04
#define PKT_TYPE_PYRO_CONFIG_V1 0x05
#define PKT_TYPE_PYRO_EVENT_V1  0x06

// LoRa payload CRC is intentionally disabled in the paired flight profile.
// Never latch a flight phase from one otherwise plausible packet.
#define ROCKET_FLIGHT_CONFIRM_PACKETS 3u
#define ROCKET_LANDED_CONFIRM_PACKETS 3u
#define ROCKET_SAFE_RESET_CONFIRM_STATUS_PACKETS 2u

// Logging / timing
#define GPS_WAIT_MS        60000      // 60s to wait for GPS time
#define LINK_LOST_MS       10000      // No rocket packets for 10s = signal lost
#define PAD_PRELOG_MS      5000       // Pre-flight PAD logging
#define PAD_LOSTLOG_MS     30000      // Logging when signal lost
#define FLIGHT_LOG_MS      200        // Flight log cadence
#define RECOVERY_LOG_MS    1000       // Recovery log cadence
#define GROUND_LOG_MS      30000      // Independent ground-only log cadence
#define PWR_FIRE_LOG_MS    100        // High-rate power samples after START
#define PWR_FIRE_LOG_WINDOW_MS 3000   // Duration of high-rate power samples
#define ARM_HEARTBEAT_ON_MS 150       // Normal armed heartbeat beep length
#define ARM_HEARTBEAT_PERIOD_MS 500   // Normal armed heartbeat cadence
#define ARM_URGENT_AFTER_MS 60000     // Faster cadence after ARM is on too long
#define ARM_URGENT_ON_MS 150          // Urgent armed heartbeat beep length
#define ARM_URGENT_PERIOD_MS 300      // Urgent armed heartbeat cadence

// Display
#define UI_UPDATE_MS       250        // Refresh rate (ms)
#define UI_MANUAL_TIMEOUT_MS 15000    // After manual page change, return after 15s
#define UI_RESET_HOLD_MS     3000     // Hold button to reset link/phase
// Automatically focus Launch page for N ms after arming
#define UI_LAUNCH_VIEW_MS    15000
// Force a UI refresh at least this often so
// time-based fields (ages, timers) still update.
#define UI_AUTO_REFRESH_MS   1000
// How long to keep [LANDED] page before returning to default
#define UI_LANDED_VIEW_MS    30000
// RS-485 turnaround timings (us)
#define PWR_DE_PRE_US        20
#define PWR_DE_POST_US       300
// TFT display (ILI9341 240x320)
#define TFT_CS_PIN          3
#define TFT_DC_PIN          15
#define TFT_RST_PIN         28

// Battery placeholders
#define GROUND_BATT_VOLTAGE   11.1f
#define ROCKET_BATT_VOLTAGE    7.0f

// Ground battery status inference. Mirrors the RocketV10 style and extends it
// to 3S packs used by the ground station.
#define GND_BATT_2S_DETECT_UP_V   5.15f
#define GND_BATT_2S_DETECT_DOWN_V 4.85f
#define GND_BATT_3S_DETECT_UP_V   8.60f
#define GND_BATT_3S_DETECT_DOWN_V 8.20f
// Current ground power path loses a strict 3.3V rail below about 4.28V input,
// so 1S is only treated as safe while there is regulator headroom.
#define GND_BATT_1S_WARN_V        4.45f
#define GND_BATT_1S_CRIT_V        4.30f
#define GND_BATT_2S_WARN_V        6.8f
#define GND_BATT_2S_CRIT_V        6.4f
#define GND_BATT_3S_WARN_V        10.2f
#define GND_BATT_3S_CRIT_V        9.6f
#define GND_BATT_WARN_HYST_V      0.08f
#define GND_BATT_CRIT_HYST_V      0.05f

// Rocket battery status inference from status-packet voltage.
// These mirror the current RocketV10 thresholds. The status packet only sends
// batt_mv plus HEALTH_BATT_OK, so ground derives WARN vs CRIT locally.
#define ROCKET_BATT_2S_DETECT_UP_V   5.15f
#define ROCKET_BATT_1S_WARN_V        3.85f
#define ROCKET_BATT_1S_CRIT_V        3.70f
#define ROCKET_BATT_2S_WARN_V        6.8f
#define ROCKET_BATT_2S_CRIT_V        6.4f

// Default rocket name
#define DEFAULT_ROCKET_NAME   "shadow"
// Optional SD config file on the ground station.
// Example line: rocket_name=shadow
#define GROUND_CONFIG_PATH    "/ground_config.txt"

// Sensors
#define SEA_LEVEL_PRESSURE_HPA   1013.25f

// Ground sensors / GPS cadence
// BMP180 baro/temperature read period on ground module
#define GND_SENSORS_UPDATE_MS    2000
// Ground GPS (Serial1 -> TinyGPS++) read cadence
#define GND_GPS_UPDATE_MS        1000
#define GND_GPS_STALE_MS         5000

// Serial debug:
// 0 = disabled for field use
// 1 = boot + important status messages
// 2 = verbose packet/status debug for bench work
#ifndef SERIAL_DEBUG_LEVEL
#define SERIAL_DEBUG_LEVEL 0
#endif

// Enable / disable RS-485 power module integration (1=on, 0=off)
#define ENABLE_POWER_MODULE 1

// Serial debug macros
#if SERIAL_DEBUG_LEVEL >= 1
  #define DBG1(x) do { Serial.println(x); } while (0)
#else
  #define DBG1(x) do {} while (0)
#endif

#if SERIAL_DEBUG_LEVEL >= 2
  #define DBG2(x) do { Serial.println(x); } while (0)
#else
  #define DBG2(x) do {} while (0)
#endif

// Legacy alias so old DBG3 call sites still map to verbose level 2.
#if SERIAL_DEBUG_LEVEL >= 2
  #define DBG3(x) do { Serial.println(x); } while (0)
#else
  #define DBG3(x) do {} while (0)
#endif
