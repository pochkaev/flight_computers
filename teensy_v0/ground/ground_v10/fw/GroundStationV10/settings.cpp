#include "settings.h"

#include <EEPROM.h>
#include <LoRa.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "radio.h"
#include "sdlog.h"
#include "timekeeper.h"

static const uint32_t SETTINGS_MAGIC = 0x47563130u;  // GV10
static const uint16_t SETTINGS_VERSION = 2;
static const int EEPROM_ADDRESS = 0;

GroundRuntimeSettings groundSettings = {};

static uint32_t checksumFor(const GroundRuntimeSettings &settings) {
  const uint8_t *bytes = (const uint8_t *)&settings;
  const size_t length = offsetof(GroundRuntimeSettings, checksum);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

static void loadDefaults() {
  groundSettings = {};
  groundSettings.magic = SETTINGS_MAGIC;
  groundSettings.version = SETTINGS_VERSION;
  groundSettings.size = sizeof(groundSettings);
  groundSettings.timezoneOffsetMin = -300;
  // Existing Ground RTCs were set to local clock time. GPS synchronization
  // changes the RTC basis to UTC and persists rtcIsUtc=1.
  groundSettings.rtcIsUtc = 0;
  groundSettings.loraSf = LORA_SPREADING_FACTOR;
  groundSettings.loraCr = LORA_CODING_RATE_DENOMINATOR;
  groundSettings.loraBandwidthHz = LORA_SIGNAL_BANDWIDTH_HZ;
  groundSettings.linkLostMs = LINK_LOST_MS;
  groundSettings.checksum = checksumFor(groundSettings);
}

static bool validSettings(const GroundRuntimeSettings &settings) {
  return settings.magic == SETTINGS_MAGIC &&
         settings.version == SETTINGS_VERSION &&
         settings.size == sizeof(settings) &&
         settings.timezoneOffsetMin >= -720 &&
         settings.timezoneOffsetMin <= 840 &&
         settings.rtcIsUtc <= 1 &&
         settings.loraSf >= 6 && settings.loraSf <= 12 &&
         settings.loraCr >= 5 && settings.loraCr <= 8 &&
         settings.loraBandwidthHz >= 62500u &&
         settings.loraBandwidthHz <= 500000u &&
         settings.linkLostMs >= 2000u &&
         settings.linkLostMs <= 60000u &&
         settings.checksum == checksumFor(settings);
}

static void saveSettings() {
  groundSettings.checksum = checksumFor(groundSettings);
  EEPROM.put(EEPROM_ADDRESS, groundSettings);
}

static void showSettings() {
  GroundDateTime dt = {};
  Serial.println("CFG GROUND");
  Serial.print("FW "); Serial.println(GROUND_FW_VERSION);
  Serial.print("TZ_OFFSET_MIN "); Serial.println(groundSettings.timezoneOffsetMin);
  Serial.print("RTC_IS_UTC "); Serial.println(groundSettings.rtcIsUtc);
  if (timekeeper_getCentralDateTime(dt)) {
    char stamp[32];
    snprintf(stamp, sizeof(stamp), "%04u-%02u-%02u %02u:%02u:%02u",
             (unsigned int)dt.year, (unsigned int)dt.month, (unsigned int)dt.day,
             (unsigned int)dt.hour, (unsigned int)dt.minute, (unsigned int)dt.second);
    Serial.print("LOCAL_TIME "); Serial.println(stamp);
  }
  Serial.print("LORA_SF "); Serial.println(groundSettings.loraSf);
  Serial.print("LORA_BW "); Serial.println(groundSettings.loraBandwidthHz);
  Serial.print("LORA_CR "); Serial.println(groundSettings.loraCr);
  Serial.println("LORA_CRC 0");
  Serial.print("LORA_OK "); Serial.println(groundLoraOk ? 1 : 0);
  Serial.print("LORA_REARM_COUNT "); Serial.println(groundRadioRearmCount);
  Serial.print("LINK_LOST_MS "); Serial.println(groundSettings.linkLostMs);
  const bool linkFresh = rocketLastPacketMs != 0 &&
                         (uint32_t)(millis() - rocketLastPacketMs) <= groundSettings.linkLostMs;
  Serial.print("ROCKET_LINK_FRESH "); Serial.println(linkFresh ? 1 : 0);
  Serial.print("ROCKET_PACKET_AGE_MS ");
  if (rocketLastPacketMs == 0) Serial.println(-1);
  else Serial.println((uint32_t)(millis() - rocketLastPacketMs));
  Serial.print("RX_FLIGHT "); Serial.println(flightRxCount);
  Serial.print("RX_NAV "); Serial.println(navRxCount);
  Serial.print("RX_STATUS "); Serial.println(statusRxCount);
  Serial.print("RSSI_LAST "); Serial.println(lastCombinedRssi);
  Serial.print("STORAGE_SERVICE_SAFE ");
  Serial.println(sdlog_serviceSafe() ? 1 : 0);
}

static void showHelp() {
  Serial.println("SHOW");
  Serial.println("SET TZ_OFFSET_MIN -300");
  Serial.println("SET LORA_SF 7");
  Serial.println("SET LORA_BW 125000");
  Serial.println("SET LORA_CR 5");
  Serial.println("SET LINK_LOST_MS 10000");
  Serial.println("TIME LOCAL 2026-07-24 22:47:00");
  Serial.println("TIME UTC 2026-07-25 03:47:00");
  Serial.println("SAVE");
  Serial.println("DEFAULTS");
  Serial.println("STORAGE STATUS");
  Serial.println("SD MOUNT");
  Serial.println("SD LIST");
  Serial.println("SD INFO <filename>");
  Serial.println("SD READ <filename> <offset> <length|0-to-end>");
  Serial.println("SD ERASE LOGS CONFIRM");
  Serial.println("SD commands require both ARM switches SAFE and START released");
}

static bool parseLong(const char *text, long &value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  value = strtol(text, &end, 10);
  return end && *end == '\0';
}

static void applyRadioSettings() {
  radio_applySettings();
}

static void handleSet(char *key, char *valueText) {
  long value = 0;
  if (!key || !parseLong(valueText, value)) {
    Serial.println("ERR SET syntax");
    return;
  }

  if (strcmp(key, "TZ_OFFSET_MIN") == 0 && value >= -720 && value <= 840) {
    groundSettings.timezoneOffsetMin = (int16_t)value;
  } else if (strcmp(key, "LORA_SF") == 0 && value >= 6 && value <= 12) {
    groundSettings.loraSf = (uint8_t)value;
    applyRadioSettings();
  } else if (strcmp(key, "LORA_BW") == 0 &&
             (value == 62500 || value == 125000 || value == 250000 || value == 500000)) {
    groundSettings.loraBandwidthHz = (uint32_t)value;
    applyRadioSettings();
  } else if (strcmp(key, "LORA_CR") == 0 && value >= 5 && value <= 8) {
    groundSettings.loraCr = (uint8_t)value;
    applyRadioSettings();
  } else if (strcmp(key, "LINK_LOST_MS") == 0 && value >= 2000 && value <= 60000) {
    groundSettings.linkLostMs = (uint32_t)value;
  } else {
    Serial.println("ERR SET key/range");
    return;
  }
  Serial.println("OK SET (use SAVE to persist)");
}

static void handleTime(char *basis, char *dateText, char *timeText) {
  if (!basis || !dateText || !timeText) {
    Serial.println("ERR TIME syntax");
    return;
  }
  unsigned int yearValue = 0, monthValue = 0, dayValue = 0;
  unsigned int hourValue = 0, minuteValue = 0, secondValue = 0;
  char extra = '\0';
  if (sscanf(dateText, "%u-%u-%u%c", &yearValue, &monthValue, &dayValue, &extra) != 3 ||
      sscanf(timeText, "%u:%u:%u%c", &hourValue, &minuteValue, &secondValue, &extra) != 3) {
    Serial.println("ERR TIME format");
    return;
  }
  const bool utcBasis = strcmp(basis, "UTC") == 0;
  const bool localBasis = strcmp(basis, "LOCAL") == 0;
  if ((!utcBasis && !localBasis) ||
      yearValue > 65535u || monthValue > 255u || dayValue > 255u ||
      hourValue > 255u || minuteValue > 255u || secondValue > 255u ||
      !timekeeper_setDateTime((uint16_t)yearValue, (uint8_t)monthValue, (uint8_t)dayValue,
                              (uint8_t)hourValue, (uint8_t)minuteValue,
                              (uint8_t)secondValue, utcBasis)) {
    Serial.println("ERR TIME value");
    return;
  }
  Serial.println(utcBasis ? "OK TIME UTC" : "OK TIME LOCAL");
}

static bool requireGroundStorageSafe() {
  if (sdlog_serviceSafe()) return true;
  Serial.println("ERR LOCKED: set both channels SAFE and release START");
  return false;
}

static void handleSd(char *subcommand, char **save) {
  if (!subcommand) {
    Serial.println("ERR SD syntax");
    return;
  }
  if (!requireGroundStorageSafe()) return;

  if (strcmp(subcommand, "MOUNT") == 0) {
    if (strtok_r(nullptr, " \t", save)) {
      Serial.println("ERR SD MOUNT syntax");
      return;
    }
    Serial.println(sdlog_mount() ? "OK SD MOUNT" : "ERR SD MOUNT");
    return;
  }
  if (strcmp(subcommand, "LIST") == 0) {
    if (strtok_r(nullptr, " \t", save)) {
      Serial.println("ERR SD LIST syntax");
      return;
    }
    sdlog_list(Serial);
    return;
  }
  if (strcmp(subcommand, "INFO") == 0) {
    char *name = strtok_r(nullptr, " \t", save);
    if (!name || strtok_r(nullptr, " \t", save)) {
      Serial.println("ERR SD INFO syntax");
      return;
    }
    sdlog_info(name, Serial);
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
    sdlog_read(name, (uint32_t)offset, (uint32_t)length, Serial);
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
    uint32_t removed = 0;
    const bool ok = sdlog_eraseLogs(removed);
    Serial.print(ok ? "OK SD ERASE REMOVED " : "ERR SD ERASE REMOVED ");
    Serial.println(removed);
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
  } else if (strcmp(command, "SAVE") == 0) {
    saveSettings();
    Serial.println("OK SAVED");
  } else if (strcmp(command, "DEFAULTS") == 0) {
    const uint8_t rtcBasis = groundSettings.rtcIsUtc;
    loadDefaults();
    groundSettings.rtcIsUtc = rtcBasis;
    applyRadioSettings();
    Serial.println("OK DEFAULTS (use SAVE to persist)");
  } else if (strcmp(command, "SET") == 0) {
    handleSet(strtok_r(nullptr, " \t", &save), strtok_r(nullptr, " \t", &save));
  } else if (strcmp(command, "TIME") == 0) {
    handleTime(strtok_r(nullptr, " \t", &save),
               strtok_r(nullptr, " \t", &save),
               strtok_r(nullptr, " \t", &save));
  } else if (strcmp(command, "STORAGE") == 0) {
    char *subcommand = strtok_r(nullptr, " \t", &save);
    char *extra = strtok_r(nullptr, " \t", &save);
    if (!subcommand || strcmp(subcommand, "STATUS") != 0 || extra) {
      Serial.println("ERR STORAGE syntax; use STORAGE STATUS");
    } else {
      sdlog_printStatus(Serial);
    }
  } else if (strcmp(command, "SD") == 0) {
    handleSd(strtok_r(nullptr, " \t", &save), &save);
  } else {
    Serial.println("ERR command; use HELP");
  }
}

void groundSettingsInit() {
  GroundRuntimeSettings stored = {};
  EEPROM.get(EEPROM_ADDRESS, stored);
  if (validSettings(stored)) {
    groundSettings = stored;
  } else {
    loadDefaults();
    saveSettings();
  }
}

void groundSettingsTask() {
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

void groundSettingsMarkRtcUtc(bool isUtc, bool persist) {
  const uint8_t value = isUtc ? 1u : 0u;
  if (groundSettings.rtcIsUtc == value) return;
  groundSettings.rtcIsUtc = value;
  if (persist) saveSettings();
}

int16_t groundSettingsTimezoneOffsetMin() {
  return groundSettings.timezoneOffsetMin;
}

bool groundSettingsRtcIsUtc() {
  return groundSettings.rtcIsUtc != 0;
}

uint32_t groundSettingsLinkLostMs() {
  return groundSettings.linkLostMs;
}
