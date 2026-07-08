#include "battery.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "state.h"

extern int lastBattRaw;
extern float lastBattPinV;
extern float rocketBattRawV;
extern float rocketBattV;
extern BatteryPackType batteryPack;
extern bool batteryWarn;
extern bool batteryCrit;

static float readBatteryVoltage() {
  lastBattRaw = analogRead(VBAT_PIN);
  lastBattPinV = (float)lastBattRaw * ADC_REF_V / ADC_MAX_COUNTS;
  return lastBattPinV * (VBAT_R1_OHMS + VBAT_R2_OHMS) / VBAT_R2_OHMS;
}

static BatteryPackType detectBatteryPack(float battV) {
  if (!isfinite(battV) || battV <= 0.0f) return BATT_PACK_UNKNOWN;
  if (batteryPack == BATT_PACK_2S) {
    return (battV <= BATT_2S_DETECT_DOWN_V) ? BATT_PACK_1S : BATT_PACK_2S;
  }
  if (batteryPack == BATT_PACK_1S) {
    return (battV >= BATT_2S_DETECT_UP_V) ? BATT_PACK_2S : BATT_PACK_1S;
  }
  return (battV >= BATT_2S_DETECT_UP_V) ? BATT_PACK_2S : BATT_PACK_1S;
}

static bool thresholdLowWithHysteresis(float value, float threshold, float hysteresis, bool alreadyLow) {
  if (!isfinite(value) || value <= 0.0f) return true;
  if (alreadyLow) return value < (threshold + hysteresis);
  return value < threshold;
}

static void updateBatteryStatus(float battV) {
  batteryPack = detectBatteryPack(battV);
  if (batteryPack == BATT_PACK_1S) {
    batteryWarn = thresholdLowWithHysteresis(battV, BATT_1S_WARN_V, BATT_WARN_HYST_V, batteryWarn);
    batteryCrit = thresholdLowWithHysteresis(battV, BATT_1S_CRIT_V, BATT_CRIT_HYST_V, batteryCrit);
  } else if (batteryPack == BATT_PACK_2S) {
    batteryWarn = thresholdLowWithHysteresis(battV, BATT_2S_WARN_V, BATT_WARN_HYST_V, batteryWarn);
    batteryCrit = thresholdLowWithHysteresis(battV, BATT_2S_CRIT_V, BATT_CRIT_HYST_V, batteryCrit);
  } else {
    batteryWarn = true;
    batteryCrit = true;
  }
}

const char *batteryPackName() {
  switch (batteryPack) {
    case BATT_PACK_1S: return "1S";
    case BATT_PACK_2S: return "2S";
    default: return "UNK";
  }
}

void sampleBatteryTask() {
  rocketBattRawV = readBatteryVoltage();
  if (!isfinite(rocketBattV) || rocketBattV <= 0.0f) {
    rocketBattV = rocketBattRawV;
  } else {
    rocketBattV = (1.0f - BATT_FILTER_ALPHA) * rocketBattV + BATT_FILTER_ALPHA * rocketBattRawV;
  }
  updateBatteryStatus(rocketBattV);
}
