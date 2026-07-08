#pragma once

#include <Arduino.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include "state.h"
#include "config.h"

// Global TinyGPS++ instance (defined in radio.cpp)
extern TinyGPSPlus gps;

// Init / update
void radio_init();
void radio_update();

// Phase
FlightPhase radio_getPhase();

// Rocket name
extern char rocketName[16];

// Rocket link state
extern bool rocketHasFix;
extern bool rocketLaunched;
extern bool rocketLanded;
extern uint32_t rocketLastPacketMs;

// RSSI
extern int lastFlightRssi;
extern int lastNavRssi;
extern int lastCombinedRssi;

// Rocket navigation
extern double rktLat;
extern double rktLon;
extern float  rktAltGpsM;
extern float  rktAltBaroM;
extern uint8_t rktFixType;
extern uint8_t rktSats;
extern float  rktHdop;
extern uint16_t rktFlags;
extern float  rktVelMs;
extern float  rktMaxAltM;
extern float  rktMaxVelMs;
extern float  rktBaseAltM;   // Altitude at pad (captured before/at launch)
extern RocketFlightState rocketFlightState;

// Rocket health (inferred)
extern bool rocketImuOk;
extern bool rocketBaroOk;

// Battery placeholder
extern float rocketBattV;

// Ground GPS
extern double gndLat;
extern double gndLon;
extern float  gndAltGpsM;
extern uint8_t gndFixType;
extern uint8_t gndSats;
extern float  gndHdop;

// Distance / bearing
extern float distanceToRocketM;
extern float bearingToRocketDeg;

// Internal callback
void radio_onReceive(int packetSize);

// Helpers
float calculateDistanceM(double lat1, double lon1, double lat2, double lon2);
float calculateBearingDeg(double lat1, double lon1, double lat2, double lon2);

// Reset local rocket state (e.g., after long hold)
void radio_resetState();
