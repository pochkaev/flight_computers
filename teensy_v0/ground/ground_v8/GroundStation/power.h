#pragma once
#include <Arduino.h>
#include "config.h"

// Init / periodic update for RS485 power module master
void power_init();
void power_update();

// Link / status helpers for UI
bool power_link_fresh();

extern bool pwr_armA_seen;
extern bool pwr_onA;
extern bool pwr_armB_seen;
extern bool pwr_onB;
extern bool pwr_key_ok;
extern bool pwr_presA;
extern bool pwr_presB;
extern bool pwr_faultAny;

extern uint8_t pwr_vbat_x10;   // ignition battery x10 (from power module)
extern uint8_t pwr_ia_x10;     // lane A current x10 or peak
extern uint8_t pwr_ib_x10;     // lane B current x10 or peak

extern float   pwr_localVbat;  // local ground module battery voltage (optional)

// Approximate RS-485 receive rate (frames per second)
extern uint16_t pwr_rxRate;

// Overall arm state for Launch page / auto focus
extern bool     pwr_anyArmed;       // any ARM switch currently active
extern uint32_t pwr_lastArmOnMs;    // millis() when ARM last went active
