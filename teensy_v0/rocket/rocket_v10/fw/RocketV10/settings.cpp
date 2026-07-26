#include "settings.h"

#include <EEPROM.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "button.h"
#include "state.h"
#include "storage.h"
#include "timing.h"
#include "ui.h"

static const uint32_t SETTINGS_MAGIC = 0x52563130u;  // RV10
static const uint16_t SETTINGS_VERSION = 3;
static const int EEPROM_ADDRESS = 0;

extern FlightState flightState;

RocketRuntimeSettings rocketSettings = {};

static uint32_t checksumFor(const RocketRuntimeSettings &settings) {
  const uint8_t *bytes = (const uint8_t *)&settings;
  const size_t length = offsetof(RocketRuntimeSettings, checksum);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

static bool allowedBandwidth(uint32_t value) {
  return value == 62500u || value == 125000u ||
         value == 250000u || value == 500000u;
}

static uint32_t minimumFlightPeriod(uint8_t sf) {
  if (sf >= 10) return 1500u;
  if (sf == 9) return 800u;
  return 200u;
}

static void loadDefaults() {
  rocketSettings = {};
  rocketSettings.magic = SETTINGS_MAGIC;
  rocketSettings.version = SETTINGS_VERSION;
  rocketSettings.size = sizeof(rocketSettings);
  rocketSettings.loraSf = LORA_SPREADING_FACTOR;
  rocketSettings.loraCr = LORA_CODING_RATE_DENOMINATOR;
  rocketSettings.txPowerDbm = LORA_TX_POWER_DBM;
  rocketSettings.loraBandwidthHz = LORA_SIGNAL_BANDWIDTH_HZ;
  rocketSettings.flightTxMs = FLIGHT_TX_MS;
  rocketSettings.navTxMs = NAV_TX_MS;
  rocketSettings.statusTxMs = STATUS_TX_MS;
  rocketSettings.launchAccelG = LAUNCH_AXIAL_ACCEL_G;
  rocketSettings.launchImuConfirmMs = LAUNCH_IMU_CONFIRM_MS;
  rocketSettings.launchImuDeltaVMps = LAUNCH_IMU_MIN_DELTA_V_MPS;
  rocketSettings.launchBaroAltM = LAUNCH_BARO_REL_ALT_M;
  rocketSettings.launchBaroVelMps = LAUNCH_BARO_VEL_MPS;
  rocketSettings.apogeeMinAltM = APOGEE_MIN_REL_ALT_M;
  rocketSettings.mainAltM = DUAL_DEPLOY_MAIN_ALT_M;
  rocketSettings.checksum = checksumFor(rocketSettings);
}

static bool validSettings(const RocketRuntimeSettings &s) {
  return s.magic == SETTINGS_MAGIC &&
         s.version == SETTINGS_VERSION &&
         s.size == sizeof(s) &&
         s.loraSf >= 6 && s.loraSf <= 10 &&
         s.loraCr >= 5 && s.loraCr <= 8 &&
         s.txPowerDbm >= 2 && s.txPowerDbm <= 17 &&
         allowedBandwidth(s.loraBandwidthHz) &&
         s.flightTxMs >= minimumFlightPeriod(s.loraSf) && s.flightTxMs <= 5000u &&
         s.navTxMs >= 500u && s.navTxMs <= 10000u &&
         s.statusTxMs >= 500u && s.statusTxMs <= 10000u &&
         isfinite(s.launchAccelG) && s.launchAccelG >= 1.10f && s.launchAccelG <= 5.0f &&
         s.launchImuConfirmMs >= 20u && s.launchImuConfirmMs <= 1000u &&
         isfinite(s.launchImuDeltaVMps) &&
         s.launchImuDeltaVMps >= 0.30f && s.launchImuDeltaVMps <= 5.0f &&
         isfinite(s.launchBaroAltM) && s.launchBaroAltM >= 2.0f && s.launchBaroAltM <= 100.0f &&
         isfinite(s.launchBaroVelMps) && s.launchBaroVelMps >= 1.0f && s.launchBaroVelMps <= 100.0f &&
         isfinite(s.apogeeMinAltM) && s.apogeeMinAltM >= 10.0f && s.apogeeMinAltM <= 1000.0f &&
         isfinite(s.mainAltM) && s.mainAltM >= 20.0f && s.mainAltM <= 1000.0f &&
         s.checksum == checksumFor(s);
}

static void saveSettings() {
  rocketSettings.checksum = checksumFor(rocketSettings);
  EEPROM.put(EEPROM_ADDRESS, rocketSettings);
}

static bool configurationUnlocked() {
  const bool physicalSafe =
      digitalRead(ARM_SWITCH_PIN) == ARM_SWITCH_SAFE_LEVEL;
  const bool preflight =
      flightState == FS_IDLE || flightState == FS_PAD;
  return physicalSafe && preflight;
}

static bool requireUnlocked() {
  if (configurationUnlocked()) return true;
  Serial.println("ERR LOCKED: physical SAFE and IDLE/PAD required");
  return false;
}

static void showSettings() {
  Serial.println("CFG ROCKET");
  Serial.print("FW "); Serial.println(ROCKET_FW_VERSION);
  Serial.print("UPTIME_MS "); Serial.println(millis());
  Serial.print("CONFIG_UNLOCKED "); Serial.println(configurationUnlocked() ? 1 : 0);
  Serial.print("FLIGHT_STATE "); Serial.println((uint8_t)flightState);
  Serial.print("LED_MODE "); Serial.println(ledModeName());
  Serial.print("LORA_OK "); Serial.println(loraOk ? 1 : 0);
  Serial.println("LORA_CRC 0");
  Serial.print("LORA_TX_BUSY "); Serial.println(loraTxBusy ? 1 : 0);
  Serial.print("TX_FLIGHT_SEQ "); Serial.println(flightSeq);
  Serial.print("TX_NAV_SEQ "); Serial.println(navSeq);
  Serial.print("TX_STATUS_SEQ "); Serial.println(statusSeq);
  Serial.print("TX_IDENTITY_SEQ "); Serial.println(identitySeq);
  Serial.print("TX_PYROCFG_SEQ "); Serial.println(pyroConfigSeq);
  Serial.print("LORA_SF "); Serial.println(rocketSettings.loraSf);
  Serial.print("LORA_BW "); Serial.println(rocketSettings.loraBandwidthHz);
  Serial.print("LORA_CR "); Serial.println(rocketSettings.loraCr);
  Serial.print("TX_POWER_DBM "); Serial.println(rocketSettings.txPowerDbm);
  Serial.print("FLIGHT_TX_MS "); Serial.println(rocketSettings.flightTxMs);
  Serial.print("NAV_TX_MS "); Serial.println(rocketSettings.navTxMs);
  Serial.print("STATUS_TX_MS "); Serial.println(rocketSettings.statusTxMs);
  Serial.print("LAUNCH_ACCEL_G "); Serial.println(rocketSettings.launchAccelG, 2);
  Serial.print("LAUNCH_IMU_CONFIRM_MS "); Serial.println(rocketSettings.launchImuConfirmMs);
  Serial.print("LAUNCH_IMU_DELTA_V_MPS "); Serial.println(rocketSettings.launchImuDeltaVMps, 2);
  Serial.print("LAUNCH_BARO_ALT_M "); Serial.println(rocketSettings.launchBaroAltM, 1);
  Serial.print("LAUNCH_BARO_VEL_MPS "); Serial.println(rocketSettings.launchBaroVelMps, 1);
  Serial.print("APOGEE_MIN_ALT_M "); Serial.println(rocketSettings.apogeeMinAltM, 1);
  Serial.print("MAIN_ALT_M "); Serial.println(rocketSettings.mainAltM, 1);
  Serial.print("LOOP_GAP_MAX_US "); Serial.println(runtimeTiming.loop_gap_max_us);
  Serial.print("LOOP_DEADLINE_MISSES "); Serial.println(runtimeTiming.loop_deadline_misses);
  Serial.print("LOOP_MAJOR_STALLS "); Serial.println(runtimeTiming.loop_major_stalls);
  Serial.print("NAND_WRITE_MAX_US "); Serial.println(runtimeTiming.nand_write_max_us);
  Serial.print("NAND_FLUSH_MAX_US "); Serial.println(runtimeTiming.nand_flush_max_us);
  Serial.print("SD_RUNTIME_LOG "); Serial.println(SD_RUNTIME_LOG_ENABLE ? 1 : 0);
  Serial.println("PYRO_CONTROL NONE");
}

static void showHelp() {
  Serial.println("SHOW");
  Serial.println("SET LORA_SF 7");
  Serial.println("SET LORA_BW 125000");
  Serial.println("SET LORA_CR 5");
  Serial.println("SET TX_POWER_DBM 17");
  Serial.println("SET FLIGHT_TX_MS 200");
  Serial.println("SET NAV_TX_MS 1000");
  Serial.println("SET STATUS_TX_MS 2000");
  Serial.println("SET LAUNCH_ACCEL_G 1.35");
  Serial.println("SET LAUNCH_IMU_CONFIRM_MS 50");
  Serial.println("SET LAUNCH_IMU_DELTA_V_MPS 1.0");
  Serial.println("SET LAUNCH_BARO_ALT_M 8.0");
  Serial.println("SET LAUNCH_BARO_VEL_MPS 8.0");
  Serial.println("SET APOGEE_MIN_ALT_M 30.0");
  Serial.println("SET MAIN_ALT_M 153.0");
  Serial.println("SAVE");
  Serial.println("DEFAULTS");
  Serial.println("FLIGHT RESET CONFIRM");
  Serial.println("ERASE NAND CONFIRM");
  Serial.println("ERASE SD CONFIRM");
  Serial.println("ERASE ALL CONFIRM");
  Serial.println("STORAGE STATUS");
  Serial.println("TIMING");
  Serial.println("TIMING RESET");
  Serial.println("NAND LIST");
  Serial.println("NAND INFO <index>");
  Serial.println("NAND READ <index> <offset> <length|0-to-end>");
  Serial.println("NAND EXPORT SD ALL SUMMARY");
  Serial.println("NAND EXPORT SD ALL FULL");
  Serial.println("NAND EXPORT SD <index> SUMMARY|FULL");
  Serial.println("NAND ERASE ALL CONFIRM");
  Serial.println("SD MOUNT");
  Serial.println("SD LIST");
  Serial.println("SD INFO <filename>");
  Serial.println("SD READ <filename> <offset> <length|0-to-end>");
  Serial.println("SD ERASE LOGS CONFIRM");
  Serial.println("Mutating/storage commands require physical SAFE and IDLE/PAD");
}

static bool parseLong(const char *text, long &value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  value = strtol(text, &end, 10);
  return end && *end == '\0';
}

static bool parseFloat(const char *text, float &value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return end && *end == '\0' && isfinite(value);
}

static void handleSet(char *key, char *valueText) {
  if (!requireUnlocked()) return;
  if (!key || !valueText) {
    Serial.println("ERR SET syntax");
    return;
  }
  if (valueText[0] == '-') {
    Serial.println("ERR SET key/range or incompatible RF cadence");
    return;
  }

  RocketRuntimeSettings candidate = rocketSettings;
  long integerValue = 0;
  float floatValue = 0.0f;
  bool radioChanged = false;
  bool recognized = true;

  if (strcmp(key, "LORA_SF") == 0 && parseLong(valueText, integerValue) &&
      integerValue >= 6 && integerValue <= 10) {
    candidate.loraSf = (uint8_t)integerValue;
    radioChanged = true;
  } else if (strcmp(key, "LORA_BW") == 0 && parseLong(valueText, integerValue) &&
             allowedBandwidth((uint32_t)integerValue)) {
    candidate.loraBandwidthHz = (uint32_t)integerValue;
    radioChanged = true;
  } else if (strcmp(key, "LORA_CR") == 0 && parseLong(valueText, integerValue) &&
             integerValue >= 5 && integerValue <= 8) {
    candidate.loraCr = (uint8_t)integerValue;
    radioChanged = true;
  } else if (strcmp(key, "TX_POWER_DBM") == 0 && parseLong(valueText, integerValue) &&
             integerValue >= 2 && integerValue <= 17) {
    candidate.txPowerDbm = (uint8_t)integerValue;
    radioChanged = true;
  } else if (strcmp(key, "FLIGHT_TX_MS") == 0 && parseLong(valueText, integerValue) &&
             integerValue >= 200 && integerValue <= 5000) {
    candidate.flightTxMs = (uint32_t)integerValue;
  } else if (strcmp(key, "NAV_TX_MS") == 0 && parseLong(valueText, integerValue) &&
             integerValue >= 500 && integerValue <= 10000) {
    candidate.navTxMs = (uint32_t)integerValue;
  } else if (strcmp(key, "STATUS_TX_MS") == 0 && parseLong(valueText, integerValue) &&
             integerValue >= 500 && integerValue <= 10000) {
    candidate.statusTxMs = (uint32_t)integerValue;
  } else if (strcmp(key, "LAUNCH_ACCEL_G") == 0 && parseFloat(valueText, floatValue)) {
    candidate.launchAccelG = floatValue;
  } else if (strcmp(key, "LAUNCH_IMU_CONFIRM_MS") == 0 && parseLong(valueText, integerValue) &&
             integerValue >= 20 && integerValue <= 1000) {
    candidate.launchImuConfirmMs = (uint32_t)integerValue;
  } else if (strcmp(key, "LAUNCH_IMU_DELTA_V_MPS") == 0 &&
             parseFloat(valueText, floatValue)) {
    candidate.launchImuDeltaVMps = floatValue;
  } else if (strcmp(key, "LAUNCH_BARO_ALT_M") == 0 && parseFloat(valueText, floatValue)) {
    candidate.launchBaroAltM = floatValue;
  } else if (strcmp(key, "LAUNCH_BARO_VEL_MPS") == 0 && parseFloat(valueText, floatValue)) {
    candidate.launchBaroVelMps = floatValue;
  } else if (strcmp(key, "APOGEE_MIN_ALT_M") == 0 && parseFloat(valueText, floatValue)) {
    candidate.apogeeMinAltM = floatValue;
  } else if (strcmp(key, "MAIN_ALT_M") == 0 && parseFloat(valueText, floatValue)) {
    candidate.mainAltM = floatValue;
  } else {
    recognized = false;
  }

  candidate.checksum = checksumFor(candidate);
  if (!recognized || !validSettings(candidate)) {
    Serial.println("ERR SET key/range or incompatible RF cadence");
    return;
  }

  rocketSettings = candidate;
  if (radioChanged) applyRocketRadioSettings();
  Serial.println("OK SET (use SAVE to persist)");
}

static void handleErase(char *target, char *confirmation, char *extra) {
  if (!target || !confirmation || extra ||
      strcmp(confirmation, "CONFIRM") != 0) {
    Serial.println("ERR ERASE syntax; use ERASE NAND|SD|ALL CONFIRM");
    return;
  }

  const bool eraseNand = strcmp(target, "NAND") == 0 ||
                         strcmp(target, "ALL") == 0;
  const bool eraseSd = strcmp(target, "SD") == 0 ||
                       strcmp(target, "ALL") == 0;
  if (!eraseNand && !eraseSd) {
    Serial.println("ERR ERASE target");
    return;
  }
  if (!requireUnlocked()) return;

  Serial.println("ERASE BEGIN");
  uint32_t nandRemoved = 0;
  uint32_t sdRemoved = 0;
  const bool ok = storageEraseLogs(eraseNand, eraseSd,
                                   nandRemoved, sdRemoved);
  Serial.print(ok ? "OK ERASE" : "ERR ERASE");
  Serial.print(" NAND_REMOVED "); Serial.print(nandRemoved);
  Serial.print(" SD_REMOVED "); Serial.println(sdRemoved);
}

static void handleNand(char *subcommand, char **save) {
  if (!subcommand) {
    Serial.println("ERR NAND syntax");
    return;
  }
  if (!requireUnlocked()) return;

  if (strcmp(subcommand, "LIST") == 0) {
    if (strtok_r(nullptr, " \t", save)) {
      Serial.println("ERR NAND LIST syntax");
      return;
    }
    storageListNand(Serial);
    return;
  }

  if (strcmp(subcommand, "INFO") == 0) {
    char *target = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    long index = 0;
    if (!target || extra || !parseLong(target, index) || index <= 0) {
      Serial.println("ERR NAND INFO syntax");
      return;
    }
    storageInfoNand((uint32_t)index, Serial);
    return;
  }

  if (strcmp(subcommand, "READ") == 0) {
    char *target = strtok_r(nullptr, " \t", save);
    char *offsetText = strtok_r(nullptr, " \t", save);
    char *lengthText = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    long index = 0, offset = 0, length = 0;
    if (!target || !offsetText || !lengthText || extra ||
        !parseLong(target, index) || index <= 0 ||
        !parseLong(offsetText, offset) || offset < 0 ||
        !parseLong(lengthText, length) || length < 0) {
      Serial.println("ERR NAND READ syntax");
      return;
    }
    storageReadNand((uint32_t)index, (uint32_t)offset,
                    (uint32_t)length, Serial);
    return;
  }

  if (strcmp(subcommand, "EXPORT") == 0) {
    char *destination = strtok_r(nullptr, " \t", save);
    char *target = strtok_r(nullptr, " \t", save);
    char *mode = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    if (!destination || strcmp(destination, "SD") != 0 ||
        !target || !mode || extra ||
        (strcmp(mode, "SUMMARY") != 0 && strcmp(mode, "FULL") != 0)) {
      Serial.println("ERR NAND EXPORT syntax");
      return;
    }
    int32_t index = -1;
    if (strcmp(target, "ALL") != 0) {
      long parsed = 0;
      if (!parseLong(target, parsed) || parsed <= 0 || parsed > INT32_MAX) {
        Serial.println("ERR NAND EXPORT target");
        return;
      }
      index = (int32_t)parsed;
    }
    Serial.println("NAND EXPORT BEGIN");
    uint32_t exported = 0;
    uint32_t skipped = 0;
    uint32_t failed = 0;
    const bool ok = storageExportNandToSd(
        index, strcmp(mode, "FULL") == 0, exported, skipped, failed);
    Serial.print(ok ? "OK NAND EXPORT" : "ERR NAND EXPORT");
    Serial.print(" EXPORTED "); Serial.print(exported);
    Serial.print(" SKIPPED "); Serial.print(skipped);
    Serial.print(" FAILED "); Serial.println(failed);
    return;
  }

  if (strcmp(subcommand, "ERASE") == 0) {
    char *target = strtok_r(nullptr, " \t", save);
    char *confirmation = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    if (!target || strcmp(target, "ALL") != 0 ||
        !confirmation || strcmp(confirmation, "CONFIRM") != 0 || extra) {
      Serial.println("ERR NAND ERASE syntax; use NAND ERASE ALL CONFIRM");
      return;
    }
    uint32_t nandRemoved = 0;
    uint32_t sdRemoved = 0;
    Serial.println("NAND ERASE BEGIN");
    const bool ok = storageEraseLogs(true, false, nandRemoved, sdRemoved);
    Serial.print(ok ? "OK NAND ERASE" : "ERR NAND ERASE");
    Serial.print(" REMOVED "); Serial.println(nandRemoved);
    return;
  }

  Serial.println("ERR NAND command");
}

static void handleSd(char *subcommand, char **save) {
  if (!subcommand) {
    Serial.println("ERR SD syntax");
    return;
  }
  if (!requireUnlocked()) return;

  if (strcmp(subcommand, "MOUNT") == 0) {
    if (strtok_r(nullptr, " \t", save)) {
      Serial.println("ERR SD MOUNT syntax");
      return;
    }
    Serial.println(storageMountSd() ? "OK SD MOUNT" : "ERR SD MOUNT");
    return;
  }

  if (strcmp(subcommand, "LIST") == 0) {
    if (strtok_r(nullptr, " \t", save)) {
      Serial.println("ERR SD LIST syntax");
      return;
    }
    storageListSd(Serial);
    return;
  }

  if (strcmp(subcommand, "INFO") == 0) {
    char *name = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    if (!name || extra) {
      Serial.println("ERR SD INFO syntax");
      return;
    }
    storageInfoSd(name, Serial);
    return;
  }

  if (strcmp(subcommand, "READ") == 0) {
    char *name = strtok_r(nullptr, " \t", save);
    char *offsetText = strtok_r(nullptr, " \t", save);
    char *lengthText = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    long offset = 0, length = 0;
    if (!name || !offsetText || !lengthText || extra ||
        !parseLong(offsetText, offset) || offset < 0 ||
        !parseLong(lengthText, length) || length < 0) {
      Serial.println("ERR SD READ syntax");
      return;
    }
    storageReadSd(name, (uint32_t)offset, (uint32_t)length, Serial);
    return;
  }

  if (strcmp(subcommand, "ERASE") == 0) {
    char *target = strtok_r(nullptr, " \t", save);
    char *confirmation = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    if (!target || strcmp(target, "LOGS") != 0 ||
        !confirmation || strcmp(confirmation, "CONFIRM") != 0 || extra) {
      Serial.println("ERR SD ERASE syntax; use SD ERASE LOGS CONFIRM");
      return;
    }
    uint32_t nandRemoved = 0;
    uint32_t sdRemoved = 0;
    Serial.println("SD ERASE BEGIN");
    const bool ok = storageEraseLogs(false, true, nandRemoved, sdRemoved);
    Serial.print(ok ? "OK SD ERASE" : "ERR SD ERASE");
    Serial.print(" REMOVED "); Serial.println(sdRemoved);
    return;
  }

  Serial.println("ERR SD command");
}

static void handleLine(char *line) {
  char *save = nullptr;
  char *command = strtok_r(line, " \t", &save);
  if (!command) return;

  if (strcmp(command, "SHOW") == 0) {
    showSettings();
  } else if (strcmp(command, "HELP") == 0) {
    showHelp();
  } else if (strcmp(command, "FLIGHT") == 0) {
    char *subcommand = strtok_r(nullptr, " \t", &save);
    char *confirmation = strtok_r(nullptr, " \t", &save);
    char *extra = strtok_r(nullptr, " \t", &save);
    if (!subcommand || strcmp(subcommand, "RESET") != 0 ||
        !confirmation || strcmp(confirmation, "CONFIRM") != 0 || extra) {
      Serial.println("ERR FLIGHT RESET syntax; use FLIGHT RESET CONFIRM");
    } else if (digitalRead(ARM_SWITCH_PIN) != ARM_SWITCH_SAFE_LEVEL) {
      Serial.println("ERR LOCKED: physical SAFE required");
    } else {
      Serial.println("FLIGHT RESET BEGIN");
      resetForNextFlight();
      Serial.println("OK FLIGHT RESET");
    }
  } else if (strcmp(command, "STORAGE") == 0) {
    char *subcommand = strtok_r(nullptr, " \t", &save);
    char *extra = strtok_r(nullptr, " \t", &save);
    if (!subcommand || strcmp(subcommand, "STATUS") != 0 || extra) {
      Serial.println("ERR STORAGE syntax; use STORAGE STATUS");
    } else if (requireUnlocked()) {
      storagePrintStatus(Serial);
    }
  } else if (strcmp(command, "TIMING") == 0) {
    char *subcommand = strtok_r(nullptr, " \t", &save);
    char *extra = strtok_r(nullptr, " \t", &save);
    if (extra || (subcommand && strcmp(subcommand, "RESET") != 0)) {
      Serial.println("ERR TIMING syntax");
    } else if (subcommand) {
      if (!requireUnlocked()) return;
      timingReset();
      Serial.println("OK TIMING RESET");
    } else {
      timingPrint(Serial);
    }
  } else if (strcmp(command, "SAVE") == 0) {
    if (!requireUnlocked()) return;
    saveSettings();
    Serial.println("OK SAVED");
  } else if (strcmp(command, "DEFAULTS") == 0) {
    if (!requireUnlocked()) return;
    loadDefaults();
    applyRocketRadioSettings();
    Serial.println("OK DEFAULTS (use SAVE to persist)");
  } else if (strcmp(command, "SET") == 0) {
    handleSet(strtok_r(nullptr, " \t", &save),
              strtok_r(nullptr, " \t", &save));
  } else if (strcmp(command, "ERASE") == 0) {
    handleErase(strtok_r(nullptr, " \t", &save),
                strtok_r(nullptr, " \t", &save),
                strtok_r(nullptr, " \t", &save));
  } else if (strcmp(command, "NAND") == 0) {
    handleNand(strtok_r(nullptr, " \t", &save), &save);
  } else if (strcmp(command, "SD") == 0) {
    handleSd(strtok_r(nullptr, " \t", &save), &save);
  } else {
    Serial.println("ERR command; use HELP");
  }
}

void rocketSettingsInit() {
  RocketRuntimeSettings stored = {};
  EEPROM.get(EEPROM_ADDRESS, stored);
  if (validSettings(stored)) {
    rocketSettings = stored;
  } else {
    loadDefaults();
    saveSettings();
  }
}

void rocketSettingsTask() {
  static char line[128] = {};
  static uint8_t length = 0;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (length > 0) {
        line[length] = '\0';
        handleLine(line);
        length = 0;
      }
    } else if (length < sizeof(line) - 1) {
      line[length++] = c;
    } else {
      length = 0;
      Serial.println("ERR line too long");
    }
  }
}
