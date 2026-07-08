#include "pyro.h"

#include "config.h"
#include "state.h"
#include "telemetry.h"

#include <Arduino.h>
#include <math.h>

extern FlightState flightState;
extern uint16_t flightFlags;
extern bool apogeeChargeLogged;
extern bool mainChargeLogged;
extern bool boosterBurnoutLogged;
extern bool boosterSeparationLogged;
extern bool sustainerIgnitionLogged;
extern bool airStart1Logged;
extern bool airStart2Logged;
extern bool pyroChannelActive[4];
extern uint32_t tLaunchMs;
extern uint32_t tApogeeMs;
extern uint32_t tBoosterBurnoutMs;
extern uint32_t tBoosterSeparationMs;
extern uint32_t tAirStart1Ms;
extern uint32_t pyroChannelStartMs[4];
extern float roll;
extern float pitch;

float currentBaroRelAltM();
void logNandEventBinary(uint32_t nowMs, uint8_t eventType, FlightState fromState,
                        FlightState toState, NandCloseReason closeReason);

static const uint8_t PYRO_CHANNEL_COUNT = 4;
static const uint8_t pyroChannelPins[PYRO_CHANNEL_COUNT] = {
  PYRO_CH1_PIN, PYRO_CH2_PIN, PYRO_CH3_PIN, PYRO_CH4_PIN
};
static const char pyroChannelFuncs[PYRO_CHANNEL_COUNT] = {
  PYRO_CH1_FUNC, PYRO_CH2_FUNC, PYRO_CH3_FUNC, PYRO_CH4_FUNC
};
static const bool pyroChannelLogEnabled[PYRO_CHANNEL_COUNT] = {
  PYRO_CH1_LOG_ENABLE != 0, PYRO_CH2_LOG_ENABLE != 0,
  PYRO_CH3_LOG_ENABLE != 0, PYRO_CH4_LOG_ENABLE != 0
};
static const bool pyroChannelOutputEnabled[PYRO_CHANNEL_COUNT] = {
  PYRO_CH1_OUTPUT_ENABLE != 0, PYRO_CH2_OUTPUT_ENABLE != 0,
  PYRO_CH3_OUTPUT_ENABLE != 0, PYRO_CH4_OUTPUT_ENABLE != 0
};
static const uint16_t pyroChannelLoggedFlags[PYRO_CHANNEL_COUNT] = {
  FLAG_PYRO_CH1_LOGGED, FLAG_PYRO_CH2_LOGGED,
  FLAG_PYRO_CH3_LOGGED, FLAG_PYRO_CH4_LOGGED
};

static bool pyroOutputEnabled() {
  return PYRO_OUTPUT_ENABLE != 0;
}

static void writePyroPin(uint8_t pin, bool active) {
  const bool pinHigh =
#if PYRO_ACTIVE_HIGH
      active;
#else
      !active;
#endif
  digitalWrite(pin, pinHigh ? HIGH : LOW);
}

void stopPyroOutputs() {
  for (uint8_t i = 0; i < PYRO_CHANNEL_COUNT; ++i) {
    pyroChannelActive[i] = false;
    writePyroPin(pyroChannelPins[i], false);
  }
}

void setupPyroOutputs() {
  for (uint8_t i = 0; i < PYRO_CHANNEL_COUNT; ++i) {
    pinMode(pyroChannelPins[i], OUTPUT);
  }
  stopPyroOutputs();
}

static uint8_t pyroChannelLogEvent(uint8_t channelIndex) {
  return EVT_PYRO_CHANNEL1_LOG + channelIndex;
}

static uint8_t pyroChannelOutputOnEvent(uint8_t channelIndex) {
  return EVT_PYRO_CHANNEL1_OUTPUT_ON + channelIndex;
}

static uint8_t pyroChannelOutputOffEvent(uint8_t channelIndex) {
  return EVT_PYRO_CHANNEL1_OUTPUT_OFF + channelIndex;
}

static bool pyroStagingSafetyOk() {
  const float relAlt = currentBaroRelAltM();
  const float tiltDeg = max(fabsf(roll), fabsf(pitch)) * 57.2957795f;
  return (relAlt >= PYRO_STAGING_MIN_REL_ALT_M) &&
         (tiltDeg <= PYRO_STAGING_MAX_TILT_DEG) &&
         flightState != FS_LANDED &&
         flightState != FS_ABORT;
}

static void requestPyroFunction(uint32_t nowMs, char func, uint8_t functionEventType) {
  logNandEventBinary(nowMs, functionEventType, flightState, flightState, NAND_CLOSE_NONE);
  telemetryNotifyPyroEvent(functionEventType, 0xFF, func);
  for (uint8_t i = 0; i < PYRO_CHANNEL_COUNT; ++i) {
    if (pyroChannelFuncs[i] != func || pyroChannelFuncs[i] == 'N') continue;

    if (pyroChannelLogEnabled[i]) {
      flightFlags |= pyroChannelLoggedFlags[i];
      logNandEventBinary(nowMs, pyroChannelLogEvent(i), flightState, flightState, NAND_CLOSE_NONE);
      telemetryNotifyPyroEvent(pyroChannelLogEvent(i), i, pyroChannelFuncs[i]);
    }

    if (!pyroOutputEnabled() || !pyroChannelOutputEnabled[i] || pyroChannelActive[i]) continue;
    pyroChannelActive[i] = true;
    pyroChannelStartMs[i] = nowMs;
    writePyroPin(pyroChannelPins[i], true);
    logNandEventBinary(nowMs, pyroChannelOutputOnEvent(i), flightState, flightState, NAND_CLOSE_NONE);
    telemetryNotifyPyroEvent(pyroChannelOutputOnEvent(i), i, pyroChannelFuncs[i]);
  }
}

void updatePyroOutputs(uint32_t nowMs) {
  if (!pyroOutputEnabled()) {
    bool anyActive = false;
    for (uint8_t i = 0; i < PYRO_CHANNEL_COUNT; ++i) anyActive |= pyroChannelActive[i];
    if (anyActive) stopPyroOutputs();
    return;
  }

  for (uint8_t i = 0; i < PYRO_CHANNEL_COUNT; ++i) {
    if (pyroChannelActive[i] && (uint32_t)(nowMs - pyroChannelStartMs[i]) >= PYRO_FIRE_MS) {
      pyroChannelActive[i] = false;
      writePyroPin(pyroChannelPins[i], false);
      logNandEventBinary(nowMs, pyroChannelOutputOffEvent(i), flightState, flightState, NAND_CLOSE_NONE);
      telemetryNotifyPyroEvent(pyroChannelOutputOffEvent(i), i, pyroChannelFuncs[i]);
    }
  }
}

void logApogeeChargeEvent(uint32_t nowMs) {
  if (apogeeChargeLogged) return;
  if (tApogeeMs != 0 && (uint32_t)(nowMs - tApogeeMs) < PYRO_APOGEE_DELAY_MS) return;
  apogeeChargeLogged = true;
  flightFlags |= FLAG_DUAL_DEPLOY_APOGEE_LOG;
  requestPyroFunction(nowMs, 'A', EVT_DUAL_DEPLOY_APOGEE_CHARGE_LOG);
  flightState = FS_DUAL_DEPLOY_APOGEE_LOGGED;
}

void logMainChargeEvent(uint32_t nowMs) {
  if (mainChargeLogged) return;
  mainChargeLogged = true;
  flightFlags |= FLAG_DUAL_DEPLOY_MAIN_LOG;
  requestPyroFunction(nowMs, 'M', EVT_DUAL_DEPLOY_MAIN_CHARGE_LOG);
  flightState = FS_DUAL_DEPLOY_MAIN_LOGGED;
}

void markBoosterBurnoutEvent(uint32_t nowMs) {
  if (boosterBurnoutLogged) return;
  boosterBurnoutLogged = true;
  tBoosterBurnoutMs = nowMs;
}

static void requestStagingOrInhibit(uint32_t nowMs, char func, uint8_t eventType) {
  if (pyroStagingSafetyOk()) {
    requestPyroFunction(nowMs, func, eventType);
  } else {
    logNandEventBinary(nowMs, EVT_PYRO_STAGING_INHIBIT_LOG, flightState, flightState, NAND_CLOSE_NONE);
    telemetryNotifyPyroEvent(EVT_PYRO_STAGING_INHIBIT_LOG, 0xFF, func);
  }
}

void updateHprStylePyroEvents(uint32_t nowMs) {
  if (PYRO_FLIGHT_PROFILE == '2') {
    if (!boosterSeparationLogged &&
        tBoosterBurnoutMs != 0 &&
        (uint32_t)(nowMs - tBoosterBurnoutMs) >= PYRO_BOOSTER_SEP_DELAY_MS) {
      boosterSeparationLogged = true;
      tBoosterSeparationMs = nowMs;
      requestPyroFunction(nowMs, 'B', EVT_PYRO_BOOSTER_SEPARATION_LOG);
    }

    if (!sustainerIgnitionLogged &&
        tBoosterSeparationMs != 0 &&
        (uint32_t)(nowMs - tBoosterSeparationMs) >= PYRO_SUSTAINER_FIRE_DELAY_MS) {
      sustainerIgnitionLogged = true;
      requestStagingOrInhibit(nowMs, 'I', EVT_PYRO_SUSTAINER_IGNITION_LOG);
    }
  }

  if (PYRO_FLIGHT_PROFILE == 'A') {
    const uint32_t airStart1BaseMs =
        (PYRO_AIRSTART1_EVENT == 'B') ? tBoosterBurnoutMs : tLaunchMs;
    if (!airStart1Logged &&
        airStart1BaseMs != 0 &&
        (uint32_t)(nowMs - airStart1BaseMs) >= PYRO_AIRSTART1_DELAY_MS) {
      airStart1Logged = true;
      tAirStart1Ms = nowMs;
      requestStagingOrInhibit(nowMs, '1', EVT_PYRO_AIRSTART1_LOG);
    }

    uint32_t airStart2BaseMs = 0;
    if (PYRO_AIRSTART2_EVENT == 'B') {
      airStart2BaseMs = tBoosterBurnoutMs;
    } else if (PYRO_AIRSTART2_EVENT == '1') {
      airStart2BaseMs = tAirStart1Ms;
    }
    if (!airStart2Logged &&
        airStart2BaseMs != 0 &&
        (uint32_t)(nowMs - airStart2BaseMs) >= PYRO_AIRSTART2_DELAY_MS) {
      airStart2Logged = true;
      requestStagingOrInhibit(nowMs, '2', EVT_PYRO_AIRSTART2_LOG);
    }
  }
}
