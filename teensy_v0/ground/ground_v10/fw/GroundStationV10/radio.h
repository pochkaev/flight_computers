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
extern int lastStatusRssi;
extern int lastCombinedRssi;
extern uint32_t lastFlightPacketMs;
extern uint32_t lastNavPacketMs;
extern uint32_t lastStatusPacketMs;
extern uint32_t lastPyroConfigPacketMs;
extern uint32_t lastPyroEventPacketMs;
extern uint32_t flightRxCount;
extern uint32_t navRxCount;
extern uint32_t statusRxCount;
extern uint32_t flightMissedCount;
extern uint32_t navMissedCount;
extern uint32_t statusMissedCount;
extern uint16_t flightRxRate;
extern uint16_t navRxRate;
extern uint16_t statusRxRate;

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
extern bool rocketGpsOk;
extern bool rocketSdOk;
extern bool rocketNandOk;
extern bool rocketLogOk;
extern bool rocketBattOk;
extern bool rocketBattWarn;
extern bool rocketBattCrit;
extern uint32_t rocketStatusLastMs;
extern uint8_t rocketLaunchStatus;
extern uint16_t rocketLaunchWaitS;

extern bool rocketPyroConfigValid;
extern uint8_t rocketPyroChannelCount;
extern uint8_t rocketPyroOutputEnabled;
extern uint8_t rocketPyroActiveHigh;
extern uint16_t rocketPyroFireMs;
extern uint16_t rocketPyroApogeeDelayMs;
extern uint16_t rocketPyroMainMinAfterApogeeMs;
extern uint16_t rocketPyroMainAltM;
extern char rocketPyroFlightProfile;
extern char rocketPyroChannelFunc[4];
extern uint8_t rocketPyroChannelPin[4];
extern uint8_t rocketPyroChannelLogMask;
extern uint8_t rocketPyroChannelOutputMask;
extern uint8_t rocketLastPyroEventType;
extern uint8_t rocketLastPyroEventChannel;
extern char rocketLastPyroEventFunction;
extern uint32_t rocketLastPyroEventSeq;
extern bool rocketPyroFlashPending;

// Battery placeholder
extern float rocketBattV;

// Ground GPS
extern double gndLat;
extern double gndLon;
extern float  gndAltGpsM;
extern uint8_t gndFixType;
extern uint8_t gndSats;
extern float  gndHdop;
extern uint32_t gndGpsChars;
extern uint32_t gndGpsPassed;
extern uint32_t gndGpsFailed;
extern uint32_t gndGpsSentencesWithFix;
extern bool gndGpsLocValid;
extern bool gndGpsAltValid;
extern bool gndGpsDateValid;
extern bool gndGpsTimeValid;

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
