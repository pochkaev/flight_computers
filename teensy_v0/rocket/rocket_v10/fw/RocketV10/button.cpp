#include "button.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "pyro.h"
#include "state.h"
#include "ui.h"

extern FlightState flightState;
extern FlightState lastFlightState;
extern uint16_t flightFlags;
extern bool haveAlt;
extern bool haveGpsBaseAlt;
extern bool logsFinalized;
extern bool logOk;
extern bool sdOk;
extern bool nandOk;
extern bool buttonPrevPressed;
extern bool buttonResetFired;
extern uint32_t buttonDownMs;
extern uint32_t tLaunchMs;
extern uint32_t tApogeeMs;
extern uint32_t launchDetectSinceMs;
extern uint32_t coastDetectSinceMs;
extern uint32_t apogeeDetectSinceMs;
extern uint32_t landedDetectSinceMs;
extern uint32_t landedStillSinceMs;
extern uint32_t recoverySubsonicCoastSinceMs;
extern uint32_t recoveryNearApogeeSinceMs;
extern uint32_t recoveryDescendingBallisticSinceMs;
extern uint32_t recoveryUnderDrogueSinceMs;
extern uint32_t recoveryPostFlightSinceMs;
extern uint32_t tBoosterBurnoutMs;
extern uint32_t tBoosterSeparationMs;
extern uint32_t tAirStart1Ms;
extern uint32_t padSettleStartMs;
extern uint32_t velRefMs;
extern bool apogeeChargeLogged;
extern bool mainChargeLogged;
extern bool boosterBurnoutLogged;
extern bool boosterSeparationLogged;
extern bool sustainerIgnitionLogged;
extern bool airStart1Logged;
extern bool airStart2Logged;
extern bool recoverySubsonicCoastLogged;
extern bool recoveryNearApogeeLogged;
extern bool recoveryDescendingBallisticLogged;
extern bool recoveryUnderDrogueLogged;
extern bool recoveryPostFlightLogged;
extern float maxAltM;
extern float maxVelMps;
extern float apogeeAltM;
extern float baseAltM;
extern float filtAlt;
extern float velRefAltM;
extern float velZ;
extern float gpsBaseAltM;
extern float gpsRelAltM;
extern float baroGpsDeltaM;
extern bool baroGpsDiverged;

void finalizeLogFiles(NandCloseReason closeReason);
void resetRelAltHistory(uint32_t nowMs, float relAlt);
void clearLaunchArmGate();

static bool flightStateAllowsButtonReset() {
  return flightState == FS_IDLE || flightState == FS_PAD ||
         flightState == FS_POST_FLIGHT_GROUND ||
         flightState == FS_LANDED || flightState == FS_ABORT;
}

static void resetForNextFlightFromButton() {
  const uint32_t nowMs = millis();
  finalizeLogFiles(NAND_CLOSE_SERVICE);

  flightState = haveAlt ? FS_PAD : FS_IDLE;
  lastFlightState = flightState;
  flightFlags = 0;
  tLaunchMs = 0;
  tApogeeMs = 0;
  maxAltM = 0.0f;
  maxVelMps = 0.0f;
  apogeeAltM = NAN;
  launchDetectSinceMs = 0;
  coastDetectSinceMs = 0;
  apogeeDetectSinceMs = 0;
  landedDetectSinceMs = 0;
  landedStillSinceMs = 0;
  recoverySubsonicCoastSinceMs = 0;
  recoveryNearApogeeSinceMs = 0;
  recoveryDescendingBallisticSinceMs = 0;
  recoveryUnderDrogueSinceMs = 0;
  recoveryPostFlightSinceMs = 0;
  apogeeChargeLogged = false;
  mainChargeLogged = false;
  boosterBurnoutLogged = false;
  boosterSeparationLogged = false;
  sustainerIgnitionLogged = false;
  airStart1Logged = false;
  airStart2Logged = false;
  tBoosterBurnoutMs = 0;
  tBoosterSeparationMs = 0;
  tAirStart1Ms = 0;
  recoverySubsonicCoastLogged = false;
  recoveryNearApogeeLogged = false;
  recoveryDescendingBallisticLogged = false;
  recoveryUnderDrogueLogged = false;
  recoveryPostFlightLogged = false;
  stopPyroOutputs();
  setFinderBeeper(false);

  if (haveAlt) {
    baseAltM = filtAlt;
    velRefAltM = filtAlt;
    velRefMs = nowMs;
    velZ = 0.0f;
    padSettleStartMs = nowMs;
    resetRelAltHistory(nowMs, 0.0f);
  }
  clearLaunchArmGate();

  haveGpsBaseAlt = false;
  gpsBaseAltM = NAN;
  gpsRelAltM = 0.0f;
  baroGpsDeltaM = NAN;
  baroGpsDiverged = false;

  logsFinalized = false;
  logOk = sdOk || nandOk;
  startBeepPattern(2);

  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.println("RocketV10 button reset: prepared for next flight");
  }
}

void updateButtonTask() {
  const bool rawPressed =
#if BUTTON_ACTIVE_LOW
    digitalRead(BUTTON_PIN) == LOW;
#else
    digitalRead(BUTTON_PIN) == HIGH;
#endif

  const uint32_t nowMs = millis();
  if (rawPressed && !buttonPrevPressed) {
    buttonDownMs = nowMs;
    buttonResetFired = false;
  }

  if (rawPressed && !buttonResetFired &&
      (uint32_t)(nowMs - buttonDownMs) >= BUTTON_RESET_HOLD_MS) {
    if (flightStateAllowsButtonReset()) {
      resetForNextFlightFromButton();
    } else {
      startBeepPattern(3);
    }
    buttonResetFired = true;
  }

  if (!rawPressed && buttonPrevPressed) {
    if (!buttonResetFired) {
      setFinderBeeper(false);
      buzzerWrite(false);
    }
    buttonResetFired = false;
  }
  buttonPrevPressed = rawPressed;
}
