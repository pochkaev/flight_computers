#include "ui.h"

#include "config.h"

extern LedMode ledMode;
extern uint32_t ledModeSinceMs;
extern bool finderBeeperActive;
extern uint32_t finderBeeperStartMs;
extern uint32_t beepPatternStartMs;
extern uint8_t beepPattern;
extern bool serviceModeActive;
extern bool serviceModeSuccess;
extern bool nandOk;
extern bool logOk;
extern bool baroOk;
extern bool imuOk;
extern bool loraOk;
extern bool batteryCrit;
extern bool armSwitchSafe;
extern FlightState flightState;
extern uint32_t nandFlightCriticalDroppedRecords;

bool isBaroFresh();
bool isImuFresh();
uint8_t currentLaunchStatus(uint16_t &waitSecondsOut);
static void landedFinderSignal(uint32_t activeMs, bool &on, uint16_t &freq);

void writeStatusLed(bool on) {
#if STATUS_LED_ACTIVE_HIGH
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
#else
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
#endif
}

void setLedMode(LedMode mode) {
  if (ledMode == mode) return;
  ledMode = mode;
  ledModeSinceMs = millis();
}

const char *ledModeName() {
  switch (ledMode) {
    case LED_MODE_BOOT: return "BOOT";
    case LED_MODE_SAFE: return "SAFE";
    case LED_MODE_ARMING: return "ARMING";
    case LED_MODE_READY: return "READY_FLIGHT";
    case LED_MODE_SERVICE:
    case LED_MODE_SUCCESS: return "SERVICE";
    case LED_MODE_LANDED: return "LANDED";
    case LED_MODE_CRITICAL:
    default: return "CRITICAL";
  }
}

void updateLedModeFromHealth() {
  const uint32_t nowMs = millis();
  const bool criticalFault =
      flightState == FS_ABORT ||
      !nandOk ||
      ((!serviceModeActive && !serviceModeSuccess) && !logOk) ||
      !baroOk || !imuOk || !loraOk ||
      !isBaroFresh() || !isImuFresh() ||
      batteryCrit ||
      nandFlightCriticalDroppedRecords > 0;
  if (criticalFault) {
    setLedMode(LED_MODE_CRITICAL);
    return;
  }
  if (serviceModeActive) {
    setLedMode(LED_MODE_SERVICE);
    return;
  }
  if (serviceModeSuccess) {
    if ((uint32_t)(nowMs - ledModeSinceMs) <= 3000u) {
      setLedMode(LED_MODE_SUCCESS);
      return;
    }
    serviceModeSuccess = false;
  }

  if (finderBeeperActive || flightState == FS_LANDED) {
    setLedMode(LED_MODE_LANDED);
    return;
  }

  uint16_t waitSeconds = 0;
  const uint8_t launchStatus = currentLaunchStatus(waitSeconds);
  (void)waitSeconds;
  const bool flightActive =
      flightState >= FS_ASCENT && flightState < FS_POST_FLIGHT_GROUND;
  if (launchStatus == LAUNCH_STATUS_READY ||
      launchStatus == LAUNCH_STATUS_LAUNCH_CHECK ||
      launchStatus == LAUNCH_STATUS_FLIGHT ||
      flightActive) {
    setLedMode(LED_MODE_READY);
  } else if (armSwitchSafe) {
    setLedMode(LED_MODE_SAFE);
  } else {
    setLedMode(LED_MODE_ARMING);
  }
}

bool startupHardwareOk() {
  return nandOk && baroOk && imuOk && loraOk && !batteryCrit;
}

void updateStatusLed() {
  static uint32_t lastLedMs = 0;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastLedMs) < LED_UPDATE_MS) return;
  lastLedMs = nowMs;

  bool on = false;
  uint32_t phase = 0;
  switch (ledMode) {
    case LED_MODE_BOOT:
      on = ((nowMs / 500u) % 2u) == 0u;
      break;
    case LED_MODE_SAFE:
      phase = nowMs % 2000u;
      on = phase < 80u;
      break;
    case LED_MODE_ARMING:
      on = ((nowMs / 500u) % 2u) == 0u;
      break;
    case LED_MODE_READY:
      on = true;
      break;
    case LED_MODE_SERVICE:
    case LED_MODE_SUCCESS:
      phase = nowMs % 1200u;
      on = (phase < 120u) || (phase >= 240u && phase < 360u);
      break;
    case LED_MODE_LANDED: {
      uint16_t unusedFreq = 0;
      landedFinderSignal(nowMs - finderBeeperStartMs, on, unusedFreq);
      break;
    }
    case LED_MODE_CRITICAL:
    default:
      on = ((nowMs / 100u) % 2u) == 0u;
      break;
  }

  writeStatusLed(on);
}

void buzzerWrite(bool on, uint16_t freqHz) {
#if BUZZER_USE_TONE
  if (on) tone(BUZZER_PIN, freqHz);
  else noTone(BUZZER_PIN);
#else
  (void)freqHz;
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#endif
}

static void buzzerDcWrite(bool on) {
#if BUZZER_USE_TONE
  noTone(BUZZER_PIN);
#endif
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

void startBeepPattern(uint8_t pattern) {
  beepPattern = pattern;
  beepPatternStartMs = millis();
}

static void playBlockingBuzzerPulse(uint16_t durationMs, uint16_t pauseMs = 0) {
  buzzerDcWrite(true);
  delay(durationMs);
  buzzerDcWrite(false);
  if (pauseMs > 0) delay(pauseMs);
}

void playStartupSound(bool ok) {
  if (ok) {
    playBlockingBuzzerPulse(260, 260);
    playBlockingBuzzerPulse(260, 260);
    playBlockingBuzzerPulse(520, 0);
  } else {
    playBlockingBuzzerPulse(700, 300);
    playBlockingBuzzerPulse(700, 300);
    playBlockingBuzzerPulse(1200, 0);
  }
}

static bool noteActive(uint32_t t, uint32_t startMs, uint32_t durationMs) {
  return t >= startMs && t < (startMs + durationMs);
}

void setFinderBeeper(bool active) {
  finderBeeperActive = active;
  if (active) finderBeeperStartMs = millis();
  else buzzerWrite(false);
}

static uint32_t landedFinderPeriodMs(uint32_t activeMs) {
  if (activeMs < LANDED_FINDER_FAST_MS) return LANDED_FINDER_FAST_PERIOD_MS;
  if (activeMs < LANDED_FINDER_MEDIUM_MS) return LANDED_FINDER_MEDIUM_PERIOD_MS;
  return LANDED_FINDER_SLOW_PERIOD_MS;
}

static void landedFinderSignal(uint32_t activeMs, bool &on, uint16_t &freq) {
  const uint32_t periodMs = landedFinderPeriodMs(activeMs);
  const uint32_t phase = activeMs % periodMs;

  if (activeMs < LANDED_FINDER_FAST_MS) {
    on = noteActive(phase, 0u, 320u) || noteActive(phase, 520u, 320u) || noteActive(phase, 1040u, 520u);
    freq = noteActive(phase, 1040u, 520u) ? 3000 : 2200;
  } else if (activeMs < LANDED_FINDER_MEDIUM_MS) {
    on = noteActive(phase, 0u, 450u) || noteActive(phase, 700u, 450u);
    freq = noteActive(phase, 700u, 450u) ? 2600 : 1800;
  } else {
    on = noteActive(phase, 0u, 900u);
    freq = 1800;
  }
}

void updateBuzzer() {
  const uint32_t nowMs = millis();
  bool on = false;
  uint16_t freq = 2400;

  if (beepPattern != 0) {
    const uint32_t t = nowMs - beepPatternStartMs;
    switch (beepPattern) {
      case 1:
        on = noteActive(t, 0u, 180u) || noteActive(t, 280u, 180u) || noteActive(t, 560u, 260u);
        if (noteActive(t, 0u, 180u)) freq = 2200;
        else if (noteActive(t, 280u, 180u)) freq = 2700;
        else freq = 3300;
        if (t >= 980u) beepPattern = 0;
        break;
      case 2:
        on = noteActive(t, 0u, 160u) || noteActive(t, 260u, 160u) || noteActive(t, 520u, 320u);
        if (noteActive(t, 0u, 160u)) freq = 1800;
        else if (noteActive(t, 260u, 160u)) freq = 2400;
        else freq = 3200;
        if (t >= 1000u) beepPattern = 0;
        break;
      case 3:
        on = noteActive(t, 0u, 420u) || noteActive(t, 620u, 420u);
        freq = 850;
        if (t >= 1250u) beepPattern = 0;
        break;
      case 4:
        on = noteActive(t, 0u, 260u) || noteActive(t, 420u, 260u) ||
             noteActive(t, 840u, 520u);
        if (noteActive(t, 0u, 260u)) freq = 2200;
        else if (noteActive(t, 420u, 260u)) freq = 1500;
        else freq = 900;
        if (t >= 1600u) beepPattern = 0;
        break;
      default:
        beepPattern = 0;
        break;
    }
  }

#if LANDED_FINDER_BEEP_ENABLE
  if (beepPattern == 0 && finderBeeperActive) {
    landedFinderSignal(nowMs - finderBeeperStartMs, on, freq);
  }
#endif

  buzzerWrite(on, freq);
}
