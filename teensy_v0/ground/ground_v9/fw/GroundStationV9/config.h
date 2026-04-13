#pragma once
#include <Arduino.h>

// -------------------------------------------------------------
// Global Configuration for Ground Station
// -------------------------------------------------------------

// Pins
#define LORA_CS_PIN        10
#define LORA_RST_PIN       9
#define LORA_DIO0_PIN      2

#define SD_CS_PIN          4
#define BUTTON_PIN         5   // Service / page button (active LOW)

// Launch controller / RS-485 power module (Master)
// Uses MAX3485 / MAX485 transceiver.
#define PWR_RS485_DE_RE_PIN 6      // DE+RE control
#define PWR_RS485_SERIAL    Serial2
#define PWR_RS485_BAUD      9600

// ARM / START switches and LEDs (active-LOW inputs)
#define PWR_ARM_A_PIN       20
#define PWR_ARM_B_PIN       21
#define PWR_START_A_PIN     22
#define PWR_START_B_PIN     23
// Use pins within D1-D23 range for LEDs
#define PWR_LED_A_PIN       16
#define PWR_LED_B_PIN       17

// Local battery measurement for ground-station 3S Li-Po.
// Default divider is conservative for up to about 14 V:
// Vbat -> 330k -> ADC -> 100k -> GND
#define PWR_VBAT_PIN        A0
// Ground battery divider (Vbat -> ADC). Change these to match your resistors.
#define GND_VBAT_R1_OHMS    330000.0f
#define GND_VBAT_R2_OHMS    100000.0f
#define ADC_REF_V           3.3f
#define ADC_MAX_COUNTS      4095.0f

// LoRa
#define LORA_FREQUENCY     915E6
#define LORA_SPI_FREQ      8000000
#define PKT_TYPE_FLIGHT_V7 0x01
#define PKT_TYPE_NAV_V7    0x02
#define PKT_TYPE_STATUS_V8 0x03

// Logging / timing
#define GPS_WAIT_MS        60000      // 60s to wait for GPS time
#define LINK_LOST_MS       10000      // No rocket packets for 10s = signal lost
#define PAD_PRELOG_MS      5000       // Pre-flight PAD logging
#define PAD_LOSTLOG_MS     30000      // Logging when signal lost

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

// Default rocket name
#define DEFAULT_ROCKET_NAME   "shadow"

// Sensors
#define SEA_LEVEL_PRESSURE_HPA   1013.25f

// Ground sensors / GPS cadence
// BMP180 baro/temperature read period on ground module
#define GND_SENSORS_UPDATE_MS    2000
// Ground GPS (Serial1 -> TinyGPS++) read cadence
#define GND_GPS_UPDATE_MS        1000

// Debug level: 0..3
#define DEBUG_LEVEL 2

// Enable / disable RS-485 power module integration (1=on, 0=off)
#define ENABLE_POWER_MODULE 1

// Debug macros
#if DEBUG_LEVEL >= 1
  #define DBG1(x) do { Serial.println(x); } while (0)
#else
  #define DBG1(x) do {} while (0)
#endif

#if DEBUG_LEVEL >= 2
  #define DBG2(x) do { Serial.println(x); } while (0)
#else
  #define DBG2(x) do {} while (0)
#endif

#if DEBUG_LEVEL >= 3
  #define DBG3(x) do { Serial.println(x); } while (0)
#else
  #define DBG3(x) do {} while (0)
#endif
