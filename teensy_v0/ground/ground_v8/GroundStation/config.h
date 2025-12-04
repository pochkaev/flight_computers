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
#define BUTTON_PIN         5   // Page change button (active LOW)

// LoRa
#define LORA_FREQUENCY     915E6
#define LORA_SPI_FREQ      8000000

// Logging / timing
#define GPS_WAIT_MS        60000      // 60s to wait for GPS time
#define LINK_LOST_MS       10000      // No packets for 10s = signal lost
#define PAD_PRELOG_MS      5000       // Pre-flight PAD logging
#define PAD_LOSTLOG_MS     30000      // Logging when signal lost

// Display
#define UI_UPDATE_MS       150        // Refresh rate (ms)
#define BIG_FONT           u8g2_font_logisoso20_tr
#define SMALL_FONT         u8g2_font_6x10_tr
#define UI_MANUAL_TIMEOUT_MS 30000    // After manual page change, return after 30s
#define UI_RESET_HOLD_MS     3000     // Hold button to reset link/phase
// Many 0.96" 128x64 I2C modules are SH1106 (column offset) even when sold as SSD1306.
// Set to 1 for SH1106 driver, 0 to force SSD1306 driver.
#define OLED_IS_SH1106     1

// Battery placeholders
#define GROUND_BATT_VOLTAGE   10.0f
#define ROCKET_BATT_VOLTAGE    7.0f

// Default rocket name
#define DEFAULT_ROCKET_NAME   "ROCKET"

// Sensors
#define SEA_LEVEL_PRESSURE_HPA   1013.25f

// Debug level: 0..3
#define DEBUG_LEVEL 2

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
