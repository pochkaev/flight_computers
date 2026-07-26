#include "storage.h"

#include <LittleFS.h>
#include <LoRa.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "ui.h"
#include "settings.h"
#include "timing.h"

static int16_t scaledInt16Saturated(float value, float scale) {
  const float scaled = value * scale;
  if (!isfinite(scaled)) return 0;
  if (scaled >= 32767.0f) return INT16_MAX;
  if (scaled <= -32768.0f) return INT16_MIN;
  return (int16_t)lroundf(scaled);
}

static int16_t angleRadToCdeg(float angleRad) {
  if (!isfinite(angleRad)) return 0;
  while (angleRad > PI) angleRad -= 2.0f * PI;
  while (angleRad < -PI) angleRad += 2.0f * PI;
  return scaledInt16Saturated(angleRad, 5729.57795f);
}

#define HAS_LITTLEFS_QPINAND 1

bool sdOk = false;
bool nandOk = false;
bool logOk = false;
bool sdLogOk = false;
bool nandLogOk = false;
bool logsFinalized = false;
File sdLogFile;
File nandLogFile;
char nandLogPath[32] = "";
uint32_t nextLogIndex = 1;
uint32_t nandRecordCount = 0;
uint8_t nandLogCache[NAND_LOG_CACHE_BYTES];
uint16_t nandLogCacheBytes = 0;
uint16_t nandRecordSequence = 0;
static uint8_t nandFlightRam1[NAND_FLIGHT_RAM1_BYTES];
DMAMEM static uint8_t nandFlightRam2[NAND_FLIGHT_RAM2_BYTES] __attribute__((aligned(32)));
uint32_t nandFlightRamBytes = 0;
uint32_t nandFlightRamDroppedRecords = 0;
static uint32_t nandFlightRamRecordCount = 0;
uint32_t nandFlightCriticalRamBytes = 0;
uint32_t nandFlightCriticalDroppedRecords = 0;
uint32_t nandFlightNonCriticalDroppedRecords = 0;
static uint32_t nandFlightCriticalRecordCount = 0;
uint32_t currentLogIndex = 0;
uint32_t nandLogOpenMs = 0;
uint32_t lastLogFlushMs = 0;
uint32_t lastGpsLoggedMs = 0;
uint32_t lastGpsCharsLogged = 0;
uint32_t lastGpsPassLogged = 0;
uint32_t lastGpsFailLogged = 0;
LittleFS_QPINAND qspiNand;

static const uint8_t NAND_EVENT_QUEUE_LEN = 32;
static NandEventRecordV4 nandEventQueue[NAND_EVENT_QUEUE_LEN] = {};
static uint8_t nandEventQueueHead = 0;
static uint8_t nandEventQueueTail = 0;
static uint8_t nandEventQueueCount = 0;
uint32_t nandEventDroppedCount = 0;

extern bool serviceModeActive;
extern bool serviceModeSuccess;
extern bool serviceModeFailed;
extern bool haveImuEstimate;
extern char rocketName[16];
extern uint32_t flightSeq;
extern uint32_t navSeq;
extern uint32_t statusSeq;
extern uint32_t identitySeq;
extern uint32_t lastGpsFixMs;
extern uint32_t lastGpsDataMs;
extern TinyGPSPlus gps;
extern FlightState flightState;
extern FlightState lastFlightState;
extern uint16_t flightFlags;
extern uint16_t diagFlags;
extern bool gpsHasFix;
extern bool haveGpsBaseAlt;
extern bool baroGpsDiverged;
extern bool batteryWarn;
extern bool batteryCrit;
extern bool attitudeAccelCorrectionActive;
extern bool attitudeMagCorrectionActive;
extern bool attitudeGyroOnly;
extern bool armSwitchSafe;
extern uint8_t gpsFixType;
extern uint8_t gpsSats;
extern BatteryPackType batteryPack;
extern float filtAlt;
extern float filtTempC;
extern float filtPressurePa;
extern float velZ;
extern float rocketBattV;
extern float rocketBattRawV;
extern float lastBattPinV;
extern float last_ax;
extern float last_ay;
extern float last_az;
extern float last_gx;
extern float last_gy;
extern float last_gz;
extern float last_mx;
extern float last_my;
extern float last_mz;
extern float roll;
extern float pitch;
extern float yaw;
extern float attitudeQw;
extern float attitudeQx;
extern float attitudeQy;
extern float attitudeQz;
extern float gpsHdop;
extern double gpsLatDeg;
extern double gpsLonDeg;
extern float gpsAltM;
extern float gpsSpeedMps;
extern float gpsBaseAltM;
extern float gpsRelAltM;
extern float baroGpsDeltaM;

uint16_t buildHealthFlags();
float currentBaroRelAltM();
bool taskDue(uint32_t nowMs, uint32_t &lastRunMs, uint32_t periodMs);

static int16_t quantizeUnitI16(float value) {
  value = max(-1.0f, min(1.0f, value));
  return (int16_t)lroundf(value * 32767.0f);
}

int extractPrefixedIndex(const char *name, const char *prefix) {
  const char *p = strstr(name, prefix);
  if (!p) return -1;
  p += strlen(prefix);
  int idx = 0;
  while (*p >= '0' && *p <= '9') {
    idx = idx * 10 + (*p - '0');
    ++p;
  }
  return idx > 0 ? idx : -1;
}

void scanSdLogFiles() {
  File root = SD.open("/");
  if (!root) return;
  uint32_t maxIdx = 0;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      int idx = extractPrefixedIndex(f.name(), "rocket_flight");
      if (idx < 0) idx = extractPrefixedIndex(f.name(), "flight");
      if (idx > (int)maxIdx) maxIdx = idx;
    }
    f.close();
  }
  root.close();
  if (maxIdx >= nextLogIndex) nextLogIndex = maxIdx + 1;
}

bool verifySdFilesystem() {
  File f = SD.open("/rocket_sd.txt", FILE_WRITE);
  if (!f) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("SD: open write failed");
    return false;
  }
  f.println("RocketV10");
  f.close();

  f = SD.open("/rocket_sd.txt", FILE_READ);
  if (!f) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("SD: open read failed");
    return false;
  }
  while (f.available()) {
    (void)f.read();
  }
  f.close();
  return true;
}

void trimAscii(char *s) {
  if (!s) return;
  char *start = s;
  while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') start++;
  if (start != s) memmove(s, start, strlen(start) + 1);
  size_t len = strlen(s);
  while (len > 0) {
    char c = s[len - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    s[--len] = '\0';
  }
}

void copyFixedString(char *dst, size_t dstSize, const char *src) {
  if (!dst || dstSize == 0) return;
  if (!src) src = "";
  size_t i = 0;
  while (i + 1 < dstSize && src[i] != '\0') {
    dst[i] = src[i];
    i++;
  }
  dst[i] = '\0';
}

static bool applyRocketNameConfig(const char *value) {
  if (!value) return false;

  char cleaned[sizeof(rocketName)] = {};
  size_t out = 0;
  for (const char *p = value; *p && out < sizeof(cleaned) - 1; ++p) {
    char c = *p;
    bool allowed =
        (c >= 'A' && c <= 'Z') ||
        (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') ||
        c == '_' || c == '-' || c == '.';
    if (allowed) cleaned[out++] = c;
  }

  if (out == 0) return false;
  cleaned[out] = '\0';
  strncpy(rocketName, cleaned, sizeof(rocketName) - 1);
  rocketName[sizeof(rocketName) - 1] = '\0';
  return true;
}

static void parseRocketConfigLine(char *line) {
  if (!line) return;
  char *comment = strchr(line, '#');
  if (comment) *comment = '\0';
  trimAscii(line);
  if (line[0] == '\0') return;

  char *eq = strchr(line, '=');
  if (!eq) return;
  *eq = '\0';
  char *key = line;
  char *value = eq + 1;
  trimAscii(key);
  trimAscii(value);

  if (strcmp(key, "rocket_name") == 0 ||
      strcmp(key, "rocket") == 0 ||
      strcmp(key, "name") == 0) {
    applyRocketNameConfig(value);
  }
}

void loadRocketConfig() {
  if (!sdOk) return;

  File f = SD.open(ROCKET_CONFIG_PATH, FILE_READ);
  if (!f) return;

  char line[96];
  uint8_t pos = 0;
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\n' || c == '\r') {
      if (pos > 0) {
        line[pos] = '\0';
        parseRocketConfigLine(line);
        pos = 0;
      }
    } else if (pos < sizeof(line) - 1) {
      line[pos++] = c;
    }
  }
  if (pos > 0) {
    line[pos] = '\0';
    parseRocketConfigLine(line);
  }
  f.close();

  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("Rocket config name=");
    Serial.println(rocketName);
  }
}

static bool parseBoolValue(const String &value, bool defaultValue) {
  if (value == "1" || value == "true" || value == "yes") return true;
  if (value == "0" || value == "false" || value == "no") return false;
  return defaultValue;
}

ServiceRequest loadServiceRequest() {
  ServiceRequest req = {};
  if (!sdOk) return req;

  File f = SD.open("/nand_ops.txt", FILE_READ);
  if (!f) return req;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("#")) continue;
    int eq = line.indexOf('=');
    if (eq <= 0) continue;

    String key = line.substring(0, eq);
    String value = line.substring(eq + 1);
    key.trim();
    value.trim();
    key.toLowerCase();
    value.toLowerCase();

    if (key == "version") {
      req.valid = (value == "1");
    } else if (key == "operation_id") {
      req.operationId = (uint32_t)value.toInt();
    } else if (key == "export_to_sd" || key == "copy_to_sd") {
      req.exportToSd = parseBoolValue(value, req.exportToSd);
    } else if (key == "erase_nand_after_export" || key == "clean_nand") {
      req.eraseNandAfterExport = parseBoolValue(value, req.eraseNandAfterExport);
    } else if (key == "export_latest_only" || key == "latest_only") {
      req.exportLatestOnly = parseBoolValue(value, req.exportLatestOnly);
    } else if (key == "export_imu" || key == "copy_imu") {
      req.exportImu = parseBoolValue(value, req.exportImu);
    } else if (key == "require_nand_ok") {
      req.requireNandOk = parseBoolValue(value, req.requireNandOk);
    } else if (key == "require_sd_ok") {
      req.requireSdOk = parseBoolValue(value, req.requireSdOk);
    }
  }

  f.close();
  if (!req.valid) return ServiceRequest();
  return req;
}

void writeServiceResult(const ServiceRequest &req, bool exportDone, uint32_t exportCount,
                        uint32_t exportSkipped, uint32_t exportFailed,
                        bool eraseDone, uint32_t eraseCount, const char *statusText) {
  if (!sdOk) return;
  SD.remove("/nand_ops_result.txt");
  File f = SD.open("/nand_ops_result.txt", FILE_WRITE);
  if (!f) return;
  f.print("version=1\noperation_id=");
  f.print((unsigned long)req.operationId);
  f.print("\nnand_ok=");
  f.print(nandOk ? "1" : "0");
  f.print("\nsd_ok=");
  f.print(sdOk ? "1" : "0");
  f.print("\nexport_requested=");
  f.print(req.exportToSd ? "1" : "0");
  f.print("\nexport_latest_only=");
  f.print(req.exportLatestOnly ? "1" : "0");
  f.print("\nexport_imu=");
  f.print(req.exportImu ? "1" : "0");
  f.print("\nexport_done=");
  f.print(exportDone ? "1" : "0");
  f.print("\nexport_count=");
  f.print((unsigned long)exportCount);
  f.print("\nexport_skipped=");
  f.print((unsigned long)exportSkipped);
  f.print("\nexport_failed=");
  f.print((unsigned long)exportFailed);
  f.print("\nerase_requested=");
  f.print(req.eraseNandAfterExport ? "1" : "0");
  f.print("\nerase_done=");
  f.print(eraseDone ? "1" : "0");
  f.print("\nerase_count=");
  f.print((unsigned long)eraseCount);
  f.print("\nstatus=");
  f.print(statusText);
  f.print("\n");
  f.close();
}

void metadataFromHeaderV3(const NandLogHeaderV3 &header, NandLogMetadata &meta) {
  meta = {};
  meta.header_version = header.version;
  meta.header_size = header.header_size;
  meta.record_size = header.record_size;
  meta.flags = header.flags;
  meta.flight_index = header.flight_index;
  meta.boot_ms = header.boot_ms;
  meta.open_ms = header.open_ms;
  meta.close_ms = header.close_ms;
  meta.record_count = header.record_count;
  meta.final_state = header.final_state;
  meta.close_reason = header.close_reason;
  copyFixedString(meta.firmware_version, sizeof(meta.firmware_version), header.firmware_version);
  copyFixedString(meta.rocket_name, sizeof(meta.rocket_name), header.rocket_name);
  meta.imu_hz = header.imu_hz;
  meta.baro_hz = header.baro_hz;
  meta.nand_log_hz = header.nand_log_hz;
  meta.sd_log_hz = header.sd_log_hz;
  meta.flight_tx_hz_x10 = header.flight_tx_hz_x10;
  meta.nav_tx_hz_x10 = header.nav_tx_hz_x10;
  meta.status_tx_hz_x10 = header.status_tx_hz_x10;
  meta.identity_tx_hz_x10 = header.identity_tx_hz_x10;
  meta.recovery_flight_tx_hz_x10 = header.recovery_flight_tx_hz_x10;
  meta.recovery_nav_tx_hz_x10 = header.recovery_nav_tx_hz_x10;
  meta.recovery_status_tx_hz_x10 = header.recovery_status_tx_hz_x10;
  meta.recovery_nand_log_hz_x10 = header.recovery_nand_log_hz_x10;
  meta.recovery_sd_log_hz_x10 = header.recovery_sd_log_hz_x10;
  meta.gyro_range_dps = header.gyro_range_dps;
  meta.accel_range_g = header.accel_range_g;
  meta.mag_range_gauss = header.mag_range_gauss;
  meta.estimator_version = header.estimator_version;
  meta.record_format = header.record_format;
}

void writeNandExportMetadata(File &dst, const NandLogMetadata &meta) {
  dst.print("# nand_header_version=");
  dst.println(meta.header_version);
  dst.print("# firmware_version=");
  dst.println(meta.firmware_version);
  dst.print("# rocket_name=");
  dst.println(meta.rocket_name);
  dst.print("# flight_index=");
  dst.println((unsigned long)meta.flight_index);
  dst.print("# record_format=");
  dst.println(meta.record_format);
  dst.print("# record_size=");
  dst.println(meta.record_size);
  dst.print("# record_count=");
  dst.println((unsigned long)meta.record_count);
  dst.print("# finalized=");
  dst.println((meta.flags & NAND_LOG_FLAG_FINALIZED) ? 1 : 0);
  dst.print("# final_state=");
  dst.println(meta.final_state);
  dst.print("# close_reason=");
  dst.println(meta.close_reason);
  dst.print("# boot_ms=");
  dst.println((unsigned long)meta.boot_ms);
  dst.print("# open_ms=");
  dst.println((unsigned long)meta.open_ms);
  dst.print("# close_ms=");
  dst.println((unsigned long)meta.close_ms);
  if (meta.imu_hz) {
    dst.print("# imu_hz=");
    dst.println(meta.imu_hz);
    dst.print("# baro_hz=");
    dst.println(meta.baro_hz);
    dst.print("# nand_log_hz=");
    dst.println(meta.nand_log_hz);
    dst.print("# sd_log_hz=");
    dst.println(meta.sd_log_hz);
    dst.print("# flight_tx_hz_x10=");
    dst.println(meta.flight_tx_hz_x10);
    dst.print("# nav_tx_hz_x10=");
    dst.println(meta.nav_tx_hz_x10);
    dst.print("# status_tx_hz_x10=");
    dst.println(meta.status_tx_hz_x10);
    dst.print("# identity_tx_hz_x10=");
    dst.println(meta.identity_tx_hz_x10);
    dst.print("# recovery_flight_tx_hz_x10=");
    dst.println(meta.recovery_flight_tx_hz_x10);
    dst.print("# recovery_nav_tx_hz_x10=");
    dst.println(meta.recovery_nav_tx_hz_x10);
    dst.print("# recovery_status_tx_hz_x10=");
    dst.println(meta.recovery_status_tx_hz_x10);
    dst.print("# recovery_nand_log_hz_x10=");
    dst.println(meta.recovery_nand_log_hz_x10);
    dst.print("# recovery_sd_log_hz_x10=");
    dst.println(meta.recovery_sd_log_hz_x10);
    dst.print("# accel_range_g=");
    dst.println(meta.accel_range_g);
    dst.print("# gyro_range_dps=");
    dst.println(meta.gyro_range_dps);
    dst.print("# mag_range_gauss=");
    dst.println(meta.mag_range_gauss);
    dst.print("# estimator_version=");
    dst.println(meta.estimator_version);
  }
}

// Moved storage implementation from RocketV10.ino.
static bool findOldestNandLog(char *path, size_t pathLen, uint32_t &logCount) {
  logCount = 0;
  if (!path || pathLen == 0) return false;
  path[0] = '\0';

  File root = qspiNand.open("/");
  if (!root) return false;

  bool found = false;
  int oldestIdx = INT_MAX;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      const char *name = f.name();
      int idx = extractPrefixedIndex(name, "rocket_flt");
      if (idx < 0) idx = extractPrefixedIndex(name, "flt");
      if (idx > 0 && strstr(name, ".bin")) {
        logCount++;
        if (idx < oldestIdx) {
          oldestIdx = idx;
          strncpy(path, name, pathLen - 1);
          path[pathLen - 1] = '\0';
          found = true;
        }
      }
    }
    f.close();
  }
  root.close();
  return found;
}

static uint64_t nandFreeBytes() {
  uint64_t total = qspiNand.totalSize();
  uint64_t used = qspiNand.usedSize();
  return (used < total) ? (total - used) : 0;
}

static bool rotateNandLogsIfNeeded() {
#if NAND_ROTATE_ENABLE
  bool ok = true;
  for (uint8_t attempt = 0; attempt < 32; ++attempt) {
    uint32_t logCount = 0;
    char oldestPath[32];
    bool haveOldest = findOldestNandLog(oldestPath, sizeof(oldestPath), logCount);
    bool tooManyLogs = (logCount >= NAND_MAX_LOG_FILES);
    bool tooLittleFree = (nandFreeBytes() < (uint64_t)NAND_MIN_FREE_BYTES);
    if (!tooManyLogs && !tooLittleFree) return ok;
    if (!haveOldest) return false;
    if (!qspiNand.remove(oldestPath)) {
      ok = false;
      break;
    }
  }
  return ok && nandFreeBytes() >= (uint64_t)NAND_MIN_FREE_BYTES;
#else
  return true;
#endif
}

static void scanNandLogFiles() {
  File root = qspiNand.open("/");
  if (!root) return;
  uint32_t maxIdx = 0;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      int idx = extractPrefixedIndex(f.name(), "rocket_flt");
      if (idx < 0) idx = extractPrefixedIndex(f.name(), "flt");
      if (idx > (int)maxIdx) maxIdx = idx;
    }
    f.close();
  }
  root.close();
  if (maxIdx >= nextLogIndex) nextLogIndex = maxIdx + 1;
}

static bool rewriteNandLogHeader(bool finalized, NandCloseReason closeReason) {
#if HAS_LITTLEFS_QPINAND
  if (!nandLogFile) return false;
  const uint32_t timingStartUs = micros();

  NandLogHeaderV3 header = {};
  memcpy(header.magic, "RV10NLG", 8);
  header.version = 4;
  header.header_size = sizeof(header);
  header.record_size = 0;
  header.flags = finalized ? NAND_LOG_FLAG_FINALIZED : 0;
  header.flight_index = currentLogIndex;
  header.boot_ms = nandLogOpenMs;
  header.open_ms = nandLogOpenMs;
  header.close_ms = finalized ? millis() : 0;
  header.record_count = nandRecordCount;
  header.final_state = (uint8_t)flightState;
  header.close_reason = (uint8_t)closeReason;
  copyFixedString(header.firmware_version, sizeof(header.firmware_version), ROCKET_FW_VERSION);
  copyFixedString(header.rocket_name, sizeof(header.rocket_name), rocketName);
  header.imu_hz = 1000u / IMU_UPDATE_MS;
  header.baro_hz = 1000u / BARO_UPDATE_MS;
  header.nand_log_hz = 1000u / NAND_LOG_UPDATE_MS;
  header.sd_log_hz = SD_RUNTIME_LOG_ENABLE ? (1000u / SD_LOG_UPDATE_MS) : 0u;
  header.flight_tx_hz_x10 = 10000u / rocketSettings.flightTxMs;
  header.nav_tx_hz_x10 = 10000u / rocketSettings.navTxMs;
  header.status_tx_hz_x10 = 10000u / rocketSettings.statusTxMs;
  header.identity_tx_hz_x10 = 10000u / IDENTITY_TX_MS;
  header.recovery_flight_tx_hz_x10 = 10000u / RECOVERY_FLIGHT_TX_MS;
  header.recovery_nav_tx_hz_x10 = 10000u / RECOVERY_NAV_TX_MS;
  header.recovery_status_tx_hz_x10 = 10000u / RECOVERY_STATUS_TX_MS;
  header.recovery_nand_log_hz_x10 = 10000u / RECOVERY_NAND_LOG_UPDATE_MS;
  header.recovery_sd_log_hz_x10 = 10000u / RECOVERY_SD_LOG_UPDATE_MS;
  header.gyro_range_dps = IMU_GYRO_RANGE_DPS;
  header.accel_range_g = IMU_ACCEL_RANGE_G;
  header.mag_range_gauss = IMU_MAG_RANGE_GAUSS;
  header.estimator_version = ATTITUDE_ESTIMATOR_VERSION;
  header.record_format = NAND_RECORD_FORMAT_V4;

  size_t endPos = nandLogFile.size();
  if (!nandLogFile.seek(0)) {
    timingRecordStorage(STORAGE_TIMING_NAND_HEADER, micros() - timingStartUs);
    return false;
  }
  if (nandLogFile.write((const uint8_t *)&header, sizeof(header)) != sizeof(header)) {
    timingRecordStorage(STORAGE_TIMING_NAND_HEADER, micros() - timingStartUs);
    return false;
  }
  if (!nandLogFile.seek(endPos)) {
    timingRecordStorage(STORAGE_TIMING_NAND_HEADER, micros() - timingStartUs);
    return false;
  }
  nandLogFile.flush();
  timingRecordStorage(STORAGE_TIMING_NAND_HEADER, micros() - timingStartUs);
  return true;
#else
  (void)finalized;
  (void)closeReason;
  return false;
#endif
}

static bool flushNandLogCache() {
#if HAS_LITTLEFS_QPINAND
  if (nandLogCacheBytes == 0) return true;
  if (!nandLogFile) {
    nandLogCacheBytes = 0;
    return false;
  }

  const size_t bytesToWrite = nandLogCacheBytes;
  const uint32_t timingStartUs = micros();
  if (nandLogFile.write((const uint8_t *)nandLogCache, bytesToWrite) != bytesToWrite) {
    timingRecordStorage(STORAGE_TIMING_NAND_WRITE, micros() - timingStartUs);
    nandLogCacheBytes = 0;
    nandLogOk = false;
    nandLogFile.close();
    return false;
  }
  timingRecordStorage(STORAGE_TIMING_NAND_WRITE, micros() - timingStartUs);
  nandLogCacheBytes = 0;
  return true;
#else
  return false;
#endif
}

static bool isTerminalFlightState() {
  return flightState == FS_LANDED || flightState == FS_ABORT;
}

static bool flightRamCaptureActive() {
  if (isTerminalFlightState()) return false;
  // Once the log header/file is prepared, PAD recording is a rolling RAM
  // pre-launch window. Do not let routine SAFE bench logging touch LittleFS.
  if (flightState == FS_PAD && nandLogFile) return true;
  return !armSwitchSafe || (flightState != FS_IDLE && flightState != FS_PAD);
}

static void copyToFlightRam(uint32_t offset, const void *record, uint16_t size) {
  const uint8_t *source = (const uint8_t *)record;
  if (offset < NAND_FLIGHT_RAM1_BYTES) {
    const uint32_t first =
        min((uint32_t)size, (uint32_t)NAND_FLIGHT_RAM1_BYTES - offset);
    memcpy(nandFlightRam1 + offset, source, first);
    offset += first;
    source += first;
    size -= (uint16_t)first;
  }
  if (size > 0) {
    memcpy(nandFlightRam2 + (offset - NAND_FLIGHT_RAM1_BYTES), source, size);
  }
}

static bool isCriticalFlightRecord(const void *record, uint16_t size) {
  if (!record || size < 1) return false;
  const uint8_t type = *((const uint8_t *)record);
  return type == NAND_RECORD_FULL_STATE_V4 ||
         type == NAND_RECORD_BARO_V4 ||
         type == NAND_RECORD_BATT_V4 ||
         type == NAND_RECORD_EVENT_V4;
}

static bool appendFlightRamRecord(const void *record, uint16_t size) {
  const uint32_t totalCapacity =
      (uint32_t)NAND_FLIGHT_RAM1_BYTES + (uint32_t)NAND_FLIGHT_RAM2_BYTES;
  const uint32_t primaryCapacity =
      totalCapacity - (uint32_t)NAND_FLIGHT_CRITICAL_RESERVE_BYTES;

  // A long armed wait must not consume the flight buffer. Keep a recent
  // bounded pre-launch window; after launch the buffer becomes linear.
  if (flightState == FS_PAD &&
      nandFlightRamBytes + size > NAND_PRELAUNCH_RAM_BYTES) {
    const uint32_t bufferedRecords =
        nandFlightRamRecordCount + nandFlightCriticalRecordCount;
    if (nandRecordCount >= bufferedRecords) {
      nandRecordCount -= bufferedRecords;
    }
    nandFlightRamBytes = 0;
    nandFlightRamRecordCount = 0;
    nandFlightCriticalRamBytes = 0;
    nandFlightCriticalRecordCount = 0;
  }

  if (nandFlightRamBytes + size <= primaryCapacity) {
    copyToFlightRam(nandFlightRamBytes, record, size);
    nandFlightRamBytes += size;
    nandFlightRamRecordCount++;
    nandRecordCount++;
    return true;
  }

  if (isCriticalFlightRecord(record, size) &&
      nandFlightCriticalRamBytes + size <= NAND_FLIGHT_CRITICAL_RESERVE_BYTES) {
    copyToFlightRam(primaryCapacity + nandFlightCriticalRamBytes, record, size);
    nandFlightCriticalRamBytes += size;
    nandFlightCriticalRecordCount++;
    nandRecordCount++;
    return true;
  }

  nandFlightRamDroppedRecords++;
  if (isCriticalFlightRecord(record, size)) nandFlightCriticalDroppedRecords++;
  else nandFlightNonCriticalDroppedRecords++;
  // Storage saturation must never disable flight logic or close the
  // pre-opened NAND file. Report the loss through status telemetry/service.
  return true;
}

static bool flushFlightRamRange(uint32_t start, uint32_t length) {
  uint32_t offset = start;
  const uint32_t end = start + length;
  while (offset < end) {
    const uint8_t *source;
    uint32_t available;
    if (offset < NAND_FLIGHT_RAM1_BYTES) {
      source = nandFlightRam1 + offset;
      available = NAND_FLIGHT_RAM1_BYTES - offset;
    } else {
      source = nandFlightRam2 + (offset - NAND_FLIGHT_RAM1_BYTES);
      available = end - offset;
    }
    const uint32_t remaining = end - offset;
    const size_t chunk = (size_t)min(min(available, remaining),
                                     (uint32_t)NAND_LOG_CACHE_BYTES);
    const uint32_t timingStartUs = micros();
    if (nandLogFile.write(source, chunk) != chunk) {
      timingRecordStorage(STORAGE_TIMING_NAND_WRITE, micros() - timingStartUs);
      nandLogOk = false;
      return false;
    }
    timingRecordStorage(STORAGE_TIMING_NAND_WRITE, micros() - timingStartUs);
    offset += (uint32_t)chunk;
  }
  return true;
}

static bool flushFlightRamToNand() {
#if HAS_LITTLEFS_QPINAND
  if (nandFlightRamBytes == 0 && nandFlightCriticalRamBytes == 0) return true;
  if (!nandLogFile) return false;

  // Preserve record order: pre-arm cache, RAM flight capture, then terminal
  // records accumulated in the normal cache.
  if (!flushNandLogCache()) return false;

  const uint32_t totalCapacity =
      (uint32_t)NAND_FLIGHT_RAM1_BYTES + (uint32_t)NAND_FLIGHT_RAM2_BYTES;
  const uint32_t primaryCapacity =
      totalCapacity - (uint32_t)NAND_FLIGHT_CRITICAL_RESERVE_BYTES;
  if (!flushFlightRamRange(0, nandFlightRamBytes)) return false;
  if (!flushFlightRamRange(primaryCapacity, nandFlightCriticalRamBytes)) return false;

  nandFlightRamBytes = 0;
  nandFlightRamRecordCount = 0;
  nandFlightCriticalRamBytes = 0;
  nandFlightCriticalRecordCount = 0;
  return true;
#else
  return false;
#endif
}

static bool appendNandRecord(const void *record, uint16_t size) {
#if HAS_LITTLEFS_QPINAND
  if (flightRamCaptureActive()) {
    return appendFlightRamRecord(record, size);
  }
  if ((nandFlightRamBytes > 0 || nandFlightCriticalRamBytes > 0) &&
      !flushFlightRamToNand()) return false;
  if (size > NAND_LOG_CACHE_BYTES) return false;
  if ((uint32_t)nandLogCacheBytes + size > NAND_LOG_CACHE_BYTES) {
    if (!flushNandLogCache()) return false;
  }
  memcpy(nandLogCache + nandLogCacheBytes, record, size);
  nandLogCacheBytes += size;
  nandRecordCount++;
  return true;
#else
  (void)record;
  (void)size;
  return false;
#endif
}

void finalizeLogFiles(NandCloseReason closeReason) {
  if (logsFinalized) return;

  if (sdLogFile) {
    sdLogFile.flush();
    sdLogFile.close();
  }
  sdLogOk = false;

#if HAS_LITTLEFS_QPINAND
  if (nandLogFile) {
    flushFlightRamToNand();
    flushNandLogCache();
    nandLogOk = rewriteNandLogHeader(true, closeReason);
    nandLogFile.close();
  }
  nandLogCacheBytes = 0;
  nandLogOk = false;
#endif

  logOk = true;
  logsFinalized = true;
}

static void ensureLogOpen() {
  if (logsFinalized) return;
  if (flightRamCaptureActive()) return;
  bool openedAny = false;
#if SD_RUNTIME_LOG_ENABLE
  if (sdOk && !sdLogFile) {
    char filename[32];
    snprintf(filename, sizeof(filename), "rocket_flight%04lu.csv", (unsigned long)nextLogIndex);
    sdLogFile = SD.open(filename, FILE_WRITE);
    if (sdLogFile) {
      currentLogIndex = nextLogIndex;
      sdLogOk = true;
      openedAny = true;
      sdLogFile.println("ms,state,flags,alt_m,rel_alt_m,vel_mps,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,lat,lon,gps_alt,gps_rel_alt,gps_base_alt,baro_gps_delta,baro_gps_diverge,diag_flags,batt_v,health,gps_chars,gps_pass,gps_fail,gps_fix_age_ms,gps_loc_valid,gps_alt_valid,gps_date_valid,gps_time_valid");
      sdLogFile.flush();
    } else {
      sdLogOk = false;
    }
  }
#endif

#if HAS_LITTLEFS_QPINAND
  if (nandOk && !nandLogFile) {
    if (!rotateNandLogsIfNeeded()) {
      nandLogOk = false;
      logOk = sdLogOk || nandLogOk || logsFinalized;
      return;
    }

    char filename[32];
    snprintf(filename, sizeof(filename), "/rocket_flt%04lu.bin", (unsigned long)nextLogIndex);
    strncpy(nandLogPath, filename, sizeof(nandLogPath) - 1);
    nandLogPath[sizeof(nandLogPath) - 1] = '\0';
    nandLogFile = qspiNand.open(filename, FILE_WRITE);
    if (nandLogFile) {
      NandLogHeaderV3 header = {};
      memcpy(header.magic, "RV10NLG", 8);
      header.version = 4;
      header.header_size = sizeof(header);
      header.record_size = 0;
      header.flags = 0;
      header.flight_index = nextLogIndex;
      header.boot_ms = millis();
      header.open_ms = header.boot_ms;
      header.close_ms = 0;
      header.record_count = 0;
      header.final_state = (uint8_t)flightState;
      header.close_reason = (uint8_t)NAND_CLOSE_NONE;
      copyFixedString(header.firmware_version, sizeof(header.firmware_version), ROCKET_FW_VERSION);
      copyFixedString(header.rocket_name, sizeof(header.rocket_name), rocketName);
      header.imu_hz = 1000u / IMU_UPDATE_MS;
      header.baro_hz = 1000u / BARO_UPDATE_MS;
      header.nand_log_hz = 1000u / NAND_LOG_UPDATE_MS;
      header.sd_log_hz = SD_RUNTIME_LOG_ENABLE ? (1000u / SD_LOG_UPDATE_MS) : 0u;
      header.flight_tx_hz_x10 = 10000u / rocketSettings.flightTxMs;
      header.nav_tx_hz_x10 = 10000u / rocketSettings.navTxMs;
      header.status_tx_hz_x10 = 10000u / rocketSettings.statusTxMs;
      header.identity_tx_hz_x10 = 10000u / IDENTITY_TX_MS;
      header.recovery_flight_tx_hz_x10 = 10000u / RECOVERY_FLIGHT_TX_MS;
      header.recovery_nav_tx_hz_x10 = 10000u / RECOVERY_NAV_TX_MS;
      header.recovery_status_tx_hz_x10 = 10000u / RECOVERY_STATUS_TX_MS;
      header.recovery_nand_log_hz_x10 = 10000u / RECOVERY_NAND_LOG_UPDATE_MS;
      header.recovery_sd_log_hz_x10 = 10000u / RECOVERY_SD_LOG_UPDATE_MS;
      header.gyro_range_dps = IMU_GYRO_RANGE_DPS;
      header.accel_range_g = IMU_ACCEL_RANGE_G;
      header.mag_range_gauss = IMU_MAG_RANGE_GAUSS;
      header.estimator_version = ATTITUDE_ESTIMATOR_VERSION;
      header.record_format = NAND_RECORD_FORMAT_V4;
      nandRecordCount = 0;
      nandLogCacheBytes = 0;
      nandRecordSequence = 0;
      currentLogIndex = nextLogIndex;
      nandLogOpenMs = header.open_ms;
      nandLogOk = nandLogFile.write((const uint8_t *)&header, sizeof(header)) == sizeof(header);
      if (!nandLogOk) {
        nandLogFile.close();
      } else {
        openedAny = true;
      }
    } else {
      nandLogOk = false;
    }
  }
#endif

  if (openedAny) {
    nextLogIndex++;
  }
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

static void logSdCsv() {
  ensureLogOpen();
  const uint32_t nowMs = millis();
  const float relAlt = currentBaroRelAltM();
  const uint16_t healthFlags = buildHealthFlags();

  if (sdLogFile) {
    char line[640];
    snprintf(line, sizeof(line),
             "%lu,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%u,%lu,%lu,%lu,%lu,%u,%u,%u,%u",
             (unsigned long)nowMs,
             (unsigned int)flightState,
             (unsigned int)flightFlags,
             filtAlt,
             relAlt,
             velZ,
             last_ax,
             last_ay,
             last_az,
             last_gx,
             last_gy,
             last_gz,
             last_mx,
             last_my,
             last_mz,
             roll * 57.2957795f,
             pitch * 57.2957795f,
             yaw * 57.2957795f,
             gpsLatDeg,
             gpsLonDeg,
             gpsAltM,
             gpsRelAltM,
             haveGpsBaseAlt ? gpsBaseAltM : NAN,
             baroGpsDeltaM,
             baroGpsDiverged ? 1u : 0u,
             (unsigned int)diagFlags,
             rocketBattV,
             (unsigned int)healthFlags,
             (unsigned long)gps.charsProcessed(),
             (unsigned long)gps.passedChecksum(),
             (unsigned long)gps.failedChecksum(),
             (unsigned long)(gpsHasFix ? 0u : (lastGpsFixMs ? (nowMs - lastGpsFixMs) : 0xFFFFFFFFu)),
             gps.location.isValid() ? 1u : 0u,
             gps.altitude.isValid() ? 1u : 0u,
             gps.date.isValid() ? 1u : 0u,
             gps.time.isValid() ? 1u : 0u);
    const uint32_t timingStartUs = micros();
    const bool writeOk = sdLogFile.println(line);
    timingRecordStorage(STORAGE_TIMING_SD_WRITE, micros() - timingStartUs);
    if (!writeOk) {
      sdLogOk = false;
      sdLogFile.close();
    }
  }
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

static void fillNandFlightRecord(NandFlightRecordV3 &rec, uint32_t nowMs,
                                 float relAlt, uint16_t healthFlags) {
  rec = {};
  rec.ms = nowMs;
  rec.health_flags = healthFlags;
  rec.flight_flags = flightFlags;
  rec.alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
  rec.rel_alt_cm = (int32_t)lroundf(relAlt * 100.0f);
  rec.vel_cms = scaledInt16Saturated(velZ, 100.0f);
  rec.temp_centi_c = scaledInt16Saturated(filtTempC, 100.0f);
  rec.pressure_pa_x10 = (uint32_t)lroundf(filtPressurePa * 10.0f);
  rec.ax_cms2 = scaledInt16Saturated(last_ax, 100.0f);
  rec.ay_cms2 = scaledInt16Saturated(last_ay, 100.0f);
  rec.az_cms2 = scaledInt16Saturated(last_az, 100.0f);
  rec.gx_cdeg = scaledInt16Saturated(last_gx, 100.0f);
  rec.gy_cdeg = scaledInt16Saturated(last_gy, 100.0f);
  rec.gz_cdeg = scaledInt16Saturated(last_gz, 100.0f);
  rec.roll_cdeg = angleRadToCdeg(roll);
  rec.pitch_cdeg = angleRadToCdeg(pitch);
  rec.gps_lat_e7 = (int32_t)llround(gpsLatDeg * 1e7);
  rec.gps_lon_e7 = (int32_t)llround(gpsLonDeg * 1e7);
  rec.gps_alt_cm = (int32_t)lroundf(gpsAltM * 100.0f);
  rec.gps_rel_alt_cm = haveGpsBaseAlt ? (int32_t)lroundf(gpsRelAltM * 100.0f) : INT32_MIN;
  rec.baro_gps_delta_cm = isfinite(baroGpsDeltaM) ? (int32_t)lroundf(baroGpsDeltaM * 100.0f) : INT32_MIN;
  rec.gps_speed_cms = scaledInt16Saturated(gpsSpeedMps, 100.0f);
  rec.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
  rec.diag_flags = diagFlags;
  rec.mx_centiuT = scaledInt16Saturated(last_mx, 100.0f);
  rec.my_centiuT = scaledInt16Saturated(last_my, 100.0f);
  rec.mz_centiuT = scaledInt16Saturated(last_mz, 100.0f);
  rec.yaw_cdeg = angleRadToCdeg(yaw);
  rec.state = (uint8_t)flightState;
  rec.gps_fix_type = gpsFixType;
  rec.gps_sats = gpsSats;
  rec.battery_pack = (uint8_t)batteryPack;
}

static void logNandBinary() {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile || flightRamCaptureActive()) {
    NandFullStateRecordV4 rec = {};
    rec.type = NAND_RECORD_FULL_STATE_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    fillNandFlightRecord(rec.data, millis(), currentBaroRelAltM(), buildHealthFlags());
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    }
  }
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

void logNandImuBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  if (flightState == FS_LANDED || flightState == FS_ABORT) return;
  ensureLogOpen();
  if (nandLogFile || flightRamCaptureActive()) {
    NandImuWideRecordV4 rec = {};
    rec.type = NAND_RECORD_IMU_WIDE_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.ax_cms2 = scaledInt16Saturated(last_ax, 100.0f);
    rec.ay_cms2 = scaledInt16Saturated(last_ay, 100.0f);
    rec.az_cms2 = scaledInt16Saturated(last_az, 100.0f);
    rec.gx_mdeg = (int32_t)lroundf(last_gx * 1000.0f);
    rec.gy_mdeg = (int32_t)lroundf(last_gy * 1000.0f);
    rec.gz_mdeg = (int32_t)lroundf(last_gz * 1000.0f);
    rec.roll_cdeg = angleRadToCdeg(roll);
    rec.pitch_cdeg = angleRadToCdeg(pitch);
    rec.yaw_cdeg = angleRadToCdeg(yaw);
    rec.state = (uint8_t)flightState;
    rec.flags = 0;
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    }
  }
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

void logNandBaroBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile || flightRamCaptureActive()) {
    NandBaroRecordV4 rec = {};
    rec.type = NAND_RECORD_BARO_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
    rec.rel_alt_cm = (int32_t)lroundf(currentBaroRelAltM() * 100.0f);
    rec.vel_cms = scaledInt16Saturated(velZ, 100.0f);
    rec.temp_centi_c = scaledInt16Saturated(filtTempC, 100.0f);
    rec.pressure_pa_x10 = (uint32_t)lroundf(filtPressurePa * 10.0f);
    rec.diag_flags = diagFlags;
    rec.state = (uint8_t)flightState;
    rec.flags = 0;
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    }
  }
#else
  (void)nowMs;
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

void logNandGpsBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  if (lastGpsDataMs == 0 || (uint32_t)(nowMs - lastGpsLoggedMs) < rocketSettings.navTxMs) return;
  ensureLogOpen();
  if (nandLogFile || flightRamCaptureActive()) {
    const uint32_t charsNow = gps.charsProcessed();
    const uint32_t passNow = gps.passedChecksum();
    const uint32_t failNow = gps.failedChecksum();
    NandGpsRecordV4 rec = {};
    rec.type = NAND_RECORD_GPS_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.lat_e7 = (int32_t)llround(gpsLatDeg * 1e7);
    rec.lon_e7 = (int32_t)llround(gpsLonDeg * 1e7);
    rec.alt_cm = (int32_t)lroundf(gpsAltM * 100.0f);
    rec.rel_alt_cm = haveGpsBaseAlt ? (int32_t)lroundf(gpsRelAltM * 100.0f) : INT32_MIN;
    rec.baro_gps_delta_cm = isfinite(baroGpsDeltaM) ? (int32_t)lroundf(baroGpsDeltaM * 100.0f) : INT32_MIN;
    rec.speed_cms = scaledInt16Saturated(gpsSpeedMps, 100.0f);
    const uint32_t fixAgeMs = gpsHasFix ? 0u : (lastGpsFixMs ? (nowMs - lastGpsFixMs) : 0xFFFFFFFFu);
    rec.fix_age_ms_x10 = (uint16_t)min(fixAgeMs / 10u, 65535u);
    rec.chars_delta = (uint16_t)min(charsNow - lastGpsCharsLogged, 65535u);
    rec.pass_delta = (uint16_t)min(passNow - lastGpsPassLogged, 65535u);
    rec.fail_delta = (uint16_t)min(failNow - lastGpsFailLogged, 65535u);
    rec.fix_type = gpsFixType;
    rec.sats = gpsSats;
    rec.hdop_x10 = (uint8_t)min((uint32_t)lroundf(gpsHdop * 10.0f), 255u);
    rec.flags = 0;
    if (gps.location.isValid()) rec.flags |= 1u << 0;
    if (gps.altitude.isValid()) rec.flags |= 1u << 1;
    if (gps.date.isValid()) rec.flags |= 1u << 2;
    if (gps.time.isValid()) rec.flags |= 1u << 3;
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    } else {
      lastGpsLoggedMs = nowMs;
      lastGpsCharsLogged = charsNow;
      lastGpsPassLogged = passNow;
      lastGpsFailLogged = failNow;
    }
  }
#else
  (void)nowMs;
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

void logNandBatteryBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile || flightRamCaptureActive()) {
    NandBatteryRecordV4 rec = {};
    rec.type = NAND_RECORD_BATT_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
    rec.raw_mv = (uint16_t)lroundf(rocketBattRawV * 1000.0f);
    rec.pin_mv = (uint16_t)lroundf(lastBattPinV * 1000.0f);
    rec.pack = (uint8_t)batteryPack;
    rec.status_flags = 0;
    if (batteryWarn) rec.status_flags |= 1u << 0;
    if (batteryCrit) rec.status_flags |= 1u << 1;
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    }
  }
#else
  (void)nowMs;
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

void logNandEventBinary(uint32_t nowMs, uint8_t eventType, FlightState fromState,
                        FlightState toState, NandCloseReason closeReason) {
#if HAS_LITTLEFS_QPINAND
  // Flight and pyro code enqueue only. Filesystem work is deferred until
  // storageTask(), after the flight kernel and output shutoff have run.
  if (nandEventQueueCount >= NAND_EVENT_QUEUE_LEN) {
    nandEventDroppedCount++;
    return;
  }
  NandEventRecordV4 &rec = nandEventQueue[nandEventQueueTail];
  rec = {};
  rec.type = NAND_RECORD_EVENT_V4;
  rec.size = sizeof(rec);
  rec.ms = nowMs;
  rec.event_type = eventType;
  rec.from_state = (uint8_t)fromState;
  rec.to_state = (uint8_t)toState;
  rec.close_reason = (uint8_t)closeReason;
  rec.flight_flags = flightFlags;
  rec.diag_flags = diagFlags;
  rec.rel_alt_cm = (int32_t)lroundf(currentBaroRelAltM() * 100.0f);
  rec.vel_cms = scaledInt16Saturated(velZ, 100.0f);
  rec.health_flags = buildHealthFlags();
  nandEventQueueTail = (uint8_t)((nandEventQueueTail + 1u) % NAND_EVENT_QUEUE_LEN);
  nandEventQueueCount++;
#else
  (void)nowMs;
  (void)eventType;
  (void)fromState;
  (void)toState;
  (void)closeReason;
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

static void drainNandEventQueue() {
#if HAS_LITTLEFS_QPINAND
  if (nandEventQueueCount == 0) return;
  ensureLogOpen();
  while ((nandLogFile || flightRamCaptureActive()) && nandEventQueueCount > 0) {
    NandEventRecordV4 &rec = nandEventQueue[nandEventQueueHead];
    rec.sequence = nandRecordSequence++;
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
      return;
    }
    nandEventQueueHead = (uint8_t)((nandEventQueueHead + 1u) % NAND_EVENT_QUEUE_LEN);
    nandEventQueueCount--;
  }
#endif
}

static const char *eventTypeName(uint8_t eventType) {
  switch (eventType) {
    case EVT_STATE_CHANGE: return "STATE_CHANGE";
    case EVT_RECOVERY_SUBSONIC_COAST: return "RECOVERY_SUBSONIC_COAST";
    case EVT_RECOVERY_NEAR_APOGEE: return "RECOVERY_NEAR_APOGEE";
    case EVT_RECOVERY_DESCENDING_BALLISTIC: return "RECOVERY_DESCENDING_BALLISTIC";
    case EVT_RECOVERY_UNDER_DROGUE: return "RECOVERY_UNDER_DROGUE";
    case EVT_RECOVERY_POST_FLIGHT_GROUND: return "RECOVERY_POST_FLIGHT_GROUND";
    case EVT_DUAL_DEPLOY_APOGEE_CHARGE_LOG: return "DUAL_DEPLOY_APOGEE_CHARGE_LOG";
    case EVT_DUAL_DEPLOY_MAIN_CHARGE_LOG: return "DUAL_DEPLOY_MAIN_CHARGE_LOG";
    case EVT_PYRO_BOOSTER_SEPARATION_LOG: return "PYRO_BOOSTER_SEPARATION_LOG";
    case EVT_PYRO_SUSTAINER_IGNITION_LOG: return "PYRO_SUSTAINER_IGNITION_LOG";
    case EVT_PYRO_AIRSTART1_LOG: return "PYRO_AIRSTART1_LOG";
    case EVT_PYRO_AIRSTART2_LOG: return "PYRO_AIRSTART2_LOG";
    case EVT_PYRO_STAGING_INHIBIT_LOG: return "PYRO_STAGING_INHIBIT_LOG";
    case EVT_PYRO_CHANNEL1_LOG: return "PYRO_CHANNEL1_LOG";
    case EVT_PYRO_CHANNEL2_LOG: return "PYRO_CHANNEL2_LOG";
    case EVT_PYRO_CHANNEL3_LOG: return "PYRO_CHANNEL3_LOG";
    case EVT_PYRO_CHANNEL4_LOG: return "PYRO_CHANNEL4_LOG";
    case EVT_PYRO_CHANNEL1_OUTPUT_ON: return "PYRO_CHANNEL1_OUTPUT_ON";
    case EVT_PYRO_CHANNEL2_OUTPUT_ON: return "PYRO_CHANNEL2_OUTPUT_ON";
    case EVT_PYRO_CHANNEL3_OUTPUT_ON: return "PYRO_CHANNEL3_OUTPUT_ON";
    case EVT_PYRO_CHANNEL4_OUTPUT_ON: return "PYRO_CHANNEL4_OUTPUT_ON";
    case EVT_PYRO_CHANNEL1_OUTPUT_OFF: return "PYRO_CHANNEL1_OUTPUT_OFF";
    case EVT_PYRO_CHANNEL2_OUTPUT_OFF: return "PYRO_CHANNEL2_OUTPUT_OFF";
    case EVT_PYRO_CHANNEL3_OUTPUT_OFF: return "PYRO_CHANNEL3_OUTPUT_OFF";
    case EVT_PYRO_CHANNEL4_OUTPUT_OFF: return "PYRO_CHANNEL4_OUTPUT_OFF";
    default: return "UNKNOWN";
  }
}

void logNandTelemetryBinary(uint32_t nowMs, uint8_t packetType, uint32_t packetSeq) {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile || flightRamCaptureActive()) {
    NandTelemetryRecordV4 rec = {};
    rec.type = NAND_RECORD_TELEM_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.packet_type = packetType;
    rec.state = (uint8_t)flightState;
    rec.health_flags = buildHealthFlags();
    rec.packet_seq = packetSeq;
    rec.flight_seq = flightSeq;
    rec.nav_seq = navSeq;
    rec.status_seq = statusSeq;
    rec.identity_seq = identitySeq;
    rec.last_rssi_dbm = (int16_t)LoRa.packetRssi();
    rec.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    }
  }
#else
  (void)nowMs;
  (void)packetType;
  (void)packetSeq;
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

void logNandAttitudeBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  if (!haveImuEstimate || flightState == FS_LANDED || flightState == FS_ABORT) return;
  ensureLogOpen();
  if (nandLogFile || flightRamCaptureActive()) {
    NandAttitudeRecordV4 rec = {};
    rec.type = NAND_RECORD_ATTITUDE_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.qw_i16 = quantizeUnitI16(attitudeQw);
    rec.qx_i16 = quantizeUnitI16(attitudeQx);
    rec.qy_i16 = quantizeUnitI16(attitudeQy);
    rec.qz_i16 = quantizeUnitI16(attitudeQz);
    rec.roll_cdeg = angleRadToCdeg(roll);
    rec.pitch_cdeg = angleRadToCdeg(pitch);
    rec.yaw_cdeg = angleRadToCdeg(yaw);
    rec.diag_flags = diagFlags;
    rec.state = (uint8_t)flightState;
    rec.flags = 0;
    if (attitudeAccelCorrectionActive) rec.flags |= 1u << 0;
    if (attitudeMagCorrectionActive) rec.flags |= 1u << 1;
    if (attitudeGyroOnly) rec.flags |= 1u << 2;
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    }
  }
#else
  (void)nowMs;
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

static void flushLogsIfDue() {
  const uint32_t nowMs = millis();
  // No explicit filesystem flush or header seek is allowed after physical ARM
  // or during flight. Sequential cache writes may still occur and are timed.
  const bool flightCriticalMode =
      !armSwitchSafe || (flightState != FS_IDLE && flightState != FS_PAD);
  if (flightCriticalMode || flightRamCaptureActive()) return;

  if ((nowMs - lastLogFlushMs) >= LOG_FLUSH_MS) {
    lastLogFlushMs = nowMs;
#if SD_RUNTIME_LOG_ENABLE
    if (sdLogFile) {
      const uint32_t timingStartUs = micros();
      sdLogFile.flush();
      timingRecordStorage(STORAGE_TIMING_SD_FLUSH, micros() - timingStartUs);
    }
#endif
#if HAS_LITTLEFS_QPINAND
    if (nandLogFile) {
      flushNandLogCache();
#if NAND_PERIODIC_HEADER_UPDATE
      rewriteNandLogHeader(false, NAND_CLOSE_NONE);
#endif
      const uint32_t timingStartUs = micros();
      nandLogFile.flush();
      timingRecordStorage(STORAGE_TIMING_NAND_FLUSH, micros() - timingStartUs);
    }
#endif
  }
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

static bool verifyNandFilesystem() {
  File f = qspiNand.open("/rocket.txt", FILE_WRITE);
  if (!f) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND: open write failed");
    return false;
  }
  f.println("RocketV10");
  f.close();

  f = qspiNand.open("/rocket.txt", FILE_READ);
  if (!f) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND: open read failed");
    return false;
  }
  while (f.available()) {
    (void)f.read();
  }
  f.close();
  return true;
}

static bool isPlausibleNandRecordV3(const NandFlightRecordV3 &rec) {
  if (rec.state > FS_ABORT) return false;
  if (rec.ms > 7UL * 24UL * 60UL * 60UL * 1000UL) return false;
  if (rec.alt_cm < -100000L || rec.alt_cm > 1000000L) return false;
  if (rec.rel_alt_cm < -50000L || rec.rel_alt_cm > 1000000L) return false;
  if (rec.pressure_pa_x10 < 500000UL || rec.pressure_pa_x10 > 1200000UL) return false;
  if (rec.batt_mv > 15000U) return false;
  if (rec.battery_pack > BATT_PACK_2S) return false;
  if (rec.gps_sats > 32U) return false;
  return true;
}

static NandExportResult exportOneNandLogToSd(const char *nandName, uint32_t operationId, bool exportImu) {
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("NAND export: open ");
    Serial.println(nandName);
  }
  File src = qspiNand.open(nandName, FILE_READ);
  if (!src) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: open failed");
    return NAND_EXPORT_FAILED;
  }

  NandLogHeaderV3 header = {};
  NandLogMetadata meta = {};
  if (src.read((uint8_t *)&header, sizeof(header)) != (int)sizeof(header)) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: header short read, skipped");
    src.close();
    return NAND_EXPORT_SKIPPED;
  }
  if (memcmp(header.magic, "RV10NLG", 8) != 0 ||
      header.version != 4 ||
      header.header_size != sizeof(NandLogHeaderV3) ||
      header.record_format != NAND_RECORD_FORMAT_V4) {
    if (SERIAL_DEBUG_LEVEL >= 1) {
      Serial.print("NAND export: unsupported header version=");
      Serial.print(header.version);
      Serial.print(" header_size=");
      Serial.print(header.header_size);
      Serial.print(" record_format=");
      Serial.println(header.record_format);
    }
    src.close();
    return NAND_EXPORT_SKIPPED;
  }
  metadataFromHeaderV3(header, meta);
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("NAND export: header ok flight=");
    Serial.print((unsigned long)meta.flight_index);
    Serial.print(" records=");
    Serial.print((unsigned long)meta.record_count);
    Serial.print(" size=");
    Serial.print((unsigned long)src.size());
    Serial.print(" finalized=");
    Serial.println((meta.flags & NAND_LOG_FLAG_FINALIZED) ? 1 : 0);
  }

  if (src.size() < (uint64_t)meta.header_size + 2) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: no payload, skipped");
    src.close();
    return NAND_EXPORT_SKIPPED;
  }
  const uint64_t fileSize = src.size();
  const uint64_t payloadBytes = fileSize - meta.header_size;
  const uint32_t maxPossibleRecords = (uint32_t)(payloadBytes / 2);
  if (meta.record_count == 0 || meta.record_count > maxPossibleRecords) {
    if (SERIAL_DEBUG_LEVEL >= 1) {
      Serial.print("NAND export: impossible record_count=");
      Serial.print((unsigned long)meta.record_count);
      Serial.print(" max_possible=");
      Serial.println((unsigned long)maxPossibleRecords);
    }
    src.close();
    return NAND_EXPORT_SKIPPED;
  }
  if (!src.seek(meta.header_size)) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: seek payload failed");
    src.close();
    return NAND_EXPORT_FAILED;
  }

  char csvName[48];
  snprintf(csvName, sizeof(csvName), "rocket_nand_%04lu_op%lu.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  char imuCsvName[52];
  snprintf(imuCsvName, sizeof(imuCsvName), "rocket_nand_%04lu_op%lu_imu.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  char baroCsvName[56];
  snprintf(baroCsvName, sizeof(baroCsvName), "rocket_nand_%04lu_op%lu_baro.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  char gpsCsvName[56];
  snprintf(gpsCsvName, sizeof(gpsCsvName), "rocket_nand_%04lu_op%lu_gps.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  char battCsvName[56];
  snprintf(battCsvName, sizeof(battCsvName), "rocket_nand_%04lu_op%lu_batt.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  char eventCsvName[58];
  snprintf(eventCsvName, sizeof(eventCsvName), "rocket_nand_%04lu_op%lu_event.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  char telemCsvName[58];
  snprintf(telemCsvName, sizeof(telemCsvName), "rocket_nand_%04lu_op%lu_telem.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  char attitudeCsvName[56];
  snprintf(attitudeCsvName, sizeof(attitudeCsvName), "rocket_nand_%04lu_op%lu_att.csv",
           (unsigned long)meta.flight_index, (unsigned long)operationId);
  SD.remove(csvName);
  SD.remove(imuCsvName);
  SD.remove(baroCsvName);
  SD.remove(gpsCsvName);
  SD.remove(battCsvName);
  SD.remove(eventCsvName);
  SD.remove(telemCsvName);
  SD.remove(attitudeCsvName);
  File dst = SD.open(csvName, FILE_WRITE);
  if (!dst) {
    if (SERIAL_DEBUG_LEVEL >= 1) {
      Serial.print("NAND export: open csv failed ");
      Serial.println(csvName);
    }
    src.close();
    return NAND_EXPORT_FAILED;
  }
  File imuDst;
  File baroDst;
  File gpsDst;
  File battDst;
  File eventDst;
  File telemDst;
  File attitudeDst;
  if (exportImu) {
    imuDst = SD.open(imuCsvName, FILE_WRITE);
    baroDst = SD.open(baroCsvName, FILE_WRITE);
    gpsDst = SD.open(gpsCsvName, FILE_WRITE);
    battDst = SD.open(battCsvName, FILE_WRITE);
    eventDst = SD.open(eventCsvName, FILE_WRITE);
    telemDst = SD.open(telemCsvName, FILE_WRITE);
    attitudeDst = SD.open(attitudeCsvName, FILE_WRITE);
    if (!imuDst || !baroDst || !gpsDst || !battDst || !eventDst || !telemDst || !attitudeDst) {
      if (SERIAL_DEBUG_LEVEL >= 1) {
        Serial.println("NAND export: open detail csv failed");
      }
      if (imuDst) imuDst.close();
      if (baroDst) baroDst.close();
      if (gpsDst) gpsDst.close();
      if (battDst) battDst.close();
      if (eventDst) eventDst.close();
      if (telemDst) telemDst.close();
      if (attitudeDst) attitudeDst.close();
      dst.close();
      src.close();
      return NAND_EXPORT_FAILED;
    }
  }
  writeNandExportMetadata(dst, meta);
  dst.println("ms,state,flags,health,diag_flags,alt_m,rel_alt_m,vel_mps,temp_c,pres_pa,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,gps_fix,sats,lat,lon,gps_alt_m,gps_rel_alt_m,baro_gps_delta_m,gps_speed_mps,batt_v,pack");
  if (imuDst) {
    writeNandExportMetadata(imuDst, meta);
    imuDst.println("ms,seq,state,ax,ay,az,gx,gy,gz,roll,pitch,yaw");
  }
  if (baroDst) {
    writeNandExportMetadata(baroDst, meta);
    baroDst.println("ms,seq,state,alt_m,rel_alt_m,vel_mps,temp_c,pres_pa,diag_flags");
  }
  if (gpsDst) {
    writeNandExportMetadata(gpsDst, meta);
    gpsDst.println("ms,seq,fix,sats,hdop,lat,lon,gps_alt_m,gps_rel_alt_m,baro_gps_delta_m,gps_speed_mps,fix_age_ms,chars_delta,pass_delta,fail_delta,flags");
  }
  if (battDst) {
    writeNandExportMetadata(battDst, meta);
    battDst.println("ms,seq,batt_v,raw_v,pin_v,pack,status_flags");
  }
  if (eventDst) {
    writeNandExportMetadata(eventDst, meta);
    eventDst.println("ms,seq,event_type,event_name,from_state,to_state,close_reason,flags,diag_flags,rel_alt_m,vel_mps,health");
  }
  if (telemDst) {
    writeNandExportMetadata(telemDst, meta);
    telemDst.println("ms,seq,packet_type,packet_seq,state,health,flight_seq,nav_seq,status_seq,identity_seq,last_rssi_dbm,batt_v");
  }
  if (attitudeDst) {
    writeNandExportMetadata(attitudeDst, meta);
    attitudeDst.println("ms,seq,state,qw,qx,qy,qz,roll,pitch,yaw,diag_flags,flags,accel_corr,mag_corr,gyro_only");
  }
  dst.flush();
  if (imuDst) imuDst.flush();
  if (baroDst) baroDst.flush();
  if (gpsDst) gpsDst.flush();
  if (battDst) battDst.flush();
  if (eventDst) eventDst.flush();
  if (telemDst) telemDst.flush();
  if (attitudeDst) attitudeDst.flush();
  bool wroteFull = false;
  bool wroteImu = false;

#define CLOSE_DETAIL_EXPORT_FILES() do { \
  if (dst) dst.close(); \
  if (imuDst) imuDst.close(); \
  if (baroDst) baroDst.close(); \
  if (gpsDst) gpsDst.close(); \
  if (battDst) battDst.close(); \
  if (eventDst) eventDst.close(); \
  if (telemDst) telemDst.close(); \
  if (attitudeDst) attitudeDst.close(); \
} while (0)

  uint64_t offset = meta.header_size;
  uint32_t recordsRead = 0;
  uint32_t fullRows = 0;
  uint32_t imuRows = 0;
  uint32_t baroRows = 0;
  uint32_t gpsRows = 0;
  uint32_t battRows = 0;
  uint32_t eventRows = 0;
  uint32_t telemRows = 0;
  uint32_t attitudeRows = 0;
  uint32_t badTypeRows = 0;
  uint32_t implausibleRows = 0;
  while (offset + 2 <= fileSize && recordsRead < meta.record_count) {
    updateStatusLed();
    if (!src.seek(offset)) {
      if (SERIAL_DEBUG_LEVEL >= 1) {
        Serial.print("NAND export: seek record failed offset=");
        Serial.println((unsigned long)offset);
      }
      CLOSE_DETAIL_EXPORT_FILES();
      src.close();
      return NAND_EXPORT_FAILED;
    }
    uint8_t type = 0;
    uint8_t size = 0;
    if (src.read(&type, 1) != 1 || src.read(&size, 1) != 1) {
      if (SERIAL_DEBUG_LEVEL >= 1) {
        Serial.print("NAND export: record header read failed offset=");
        Serial.println((unsigned long)offset);
      }
      CLOSE_DETAIL_EXPORT_FILES();
      src.close();
      return NAND_EXPORT_FAILED;
    }
    if (size < 2 || offset + size > fileSize) {
      if (SERIAL_DEBUG_LEVEL >= 1) {
        Serial.print("NAND export: bad record size type=");
        Serial.print(type);
        Serial.print(" size=");
        Serial.print(size);
        Serial.print(" offset=");
        Serial.println((unsigned long)offset);
      }
      break;
    }
    if (!src.seek(offset)) {
      if (SERIAL_DEBUG_LEVEL >= 1) {
        Serial.print("NAND export: seek record body failed offset=");
        Serial.println((unsigned long)offset);
      }
      CLOSE_DETAIL_EXPORT_FILES();
      src.close();
      return NAND_EXPORT_FAILED;
    }

    if (type == NAND_RECORD_FULL_STATE_V4 && size == sizeof(NandFullStateRecordV4)) {
      NandFullStateRecordV4 wrapped = {};
      if (src.read((uint8_t *)&wrapped, sizeof(wrapped)) != (int)sizeof(wrapped)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: full record read failed");
        CLOSE_DETAIL_EXPORT_FILES();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      const NandFlightRecordV3 &rec = wrapped.data;
      if (!isPlausibleNandRecordV3(rec)) {
        implausibleRows++;
        offset += size;
        recordsRead++;
        continue;
      }
      const float gpsRelAlt = (rec.gps_rel_alt_cm == INT32_MIN) ? NAN : rec.gps_rel_alt_cm / 100.0f;
      const float baroGpsDelta = (rec.baro_gps_delta_cm == INT32_MIN) ? NAN : rec.baro_gps_delta_cm / 100.0f;
      char line[512];
      snprintf(line, sizeof(line),
               "%lu,%u,%u,%u,%u,"
               "%.2f,%.2f,%.2f,%.2f,%.1f,"
               "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
               "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
               "%u,%u,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.3f,%u",
               (unsigned long)rec.ms,
               (unsigned int)rec.state,
               (unsigned int)rec.flight_flags,
               (unsigned int)rec.health_flags,
               (unsigned int)rec.diag_flags,
               rec.alt_cm / 100.0f,
               rec.rel_alt_cm / 100.0f,
               rec.vel_cms / 100.0f,
               rec.temp_centi_c / 100.0f,
               rec.pressure_pa_x10 / 10.0f,
               rec.ax_cms2 / 100.0f,
               rec.ay_cms2 / 100.0f,
               rec.az_cms2 / 100.0f,
               rec.gx_cdeg / 100.0f,
               rec.gy_cdeg / 100.0f,
               rec.gz_cdeg / 100.0f,
               rec.mx_centiuT / 100.0f,
               rec.my_centiuT / 100.0f,
               rec.mz_centiuT / 100.0f,
               rec.roll_cdeg / 100.0f,
               rec.pitch_cdeg / 100.0f,
               rec.yaw_cdeg / 100.0f,
               (unsigned int)rec.gps_fix_type,
               (unsigned int)rec.gps_sats,
               rec.gps_lat_e7 / 1e7,
               rec.gps_lon_e7 / 1e7,
               rec.gps_alt_cm / 100.0f,
               gpsRelAlt,
               baroGpsDelta,
               rec.gps_speed_cms / 100.0f,
               rec.batt_mv / 1000.0f,
               (unsigned int)rec.battery_pack);
      if (!dst.println(line)) {
        dst.close();
        if (imuDst) imuDst.close();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      wroteFull = true;
      fullRows++;
    } else if (type == NAND_RECORD_IMU_V4 && size == sizeof(NandImuRecordV4)) {
      NandImuRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: imu record read failed");
        CLOSE_DETAIL_EXPORT_FILES();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (imuDst) {
        char line[192];
        snprintf(line, sizeof(line),
                 "%lu,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.state,
                 rec.ax_cms2 / 100.0f,
                 rec.ay_cms2 / 100.0f,
                 rec.az_cms2 / 100.0f,
                 rec.gx_cdeg / 100.0f,
                 rec.gy_cdeg / 100.0f,
                 rec.gz_cdeg / 100.0f,
                 rec.roll_cdeg / 100.0f,
                 rec.pitch_cdeg / 100.0f,
                 rec.yaw_cdeg / 100.0f);
        if (!imuDst.println(line)) {
          if (dst) dst.close();
          imuDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      if (exportImu) wroteImu = true;
      imuRows++;
    } else if (type == NAND_RECORD_IMU_WIDE_V4 && size == sizeof(NandImuWideRecordV4)) {
      NandImuWideRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: wide imu record read failed");
        CLOSE_DETAIL_EXPORT_FILES();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (imuDst) {
        char line[192];
        snprintf(line, sizeof(line),
                 "%lu,%u,%u,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.state,
                 rec.ax_cms2 / 100.0f,
                 rec.ay_cms2 / 100.0f,
                 rec.az_cms2 / 100.0f,
                 rec.gx_mdeg / 1000.0f,
                 rec.gy_mdeg / 1000.0f,
                 rec.gz_mdeg / 1000.0f,
                 rec.roll_cdeg / 100.0f,
                 rec.pitch_cdeg / 100.0f,
                 rec.yaw_cdeg / 100.0f);
        if (!imuDst.println(line)) {
          if (dst) dst.close();
          imuDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      if (exportImu) wroteImu = true;
      imuRows++;
    } else if (type == NAND_RECORD_BARO_V4 && size == sizeof(NandBaroRecordV4)) {
      NandBaroRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: baro record read failed");
        if (dst) dst.close();
        if (imuDst) imuDst.close();
        if (baroDst) baroDst.close();
        if (gpsDst) gpsDst.close();
        if (battDst) battDst.close();
        if (eventDst) eventDst.close();
        if (telemDst) telemDst.close();
        if (attitudeDst) attitudeDst.close();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (baroDst) {
        char line[192];
        snprintf(line, sizeof(line), "%lu,%u,%u,%.2f,%.2f,%.2f,%.2f,%.1f,%u",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.state,
                 rec.alt_cm / 100.0f,
                 rec.rel_alt_cm / 100.0f,
                 rec.vel_cms / 100.0f,
                 rec.temp_centi_c / 100.0f,
                 rec.pressure_pa_x10 / 10.0f,
                 (unsigned int)rec.diag_flags);
        if (!baroDst.println(line)) {
          if (dst) dst.close();
          if (imuDst) imuDst.close();
          baroDst.close();
          if (gpsDst) gpsDst.close();
          if (battDst) battDst.close();
          if (eventDst) eventDst.close();
          if (telemDst) telemDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      baroRows++;
    } else if (type == NAND_RECORD_GPS_V4 && size == sizeof(NandGpsRecordV4)) {
      NandGpsRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: gps record read failed");
        if (dst) dst.close();
        if (imuDst) imuDst.close();
        if (baroDst) baroDst.close();
        if (gpsDst) gpsDst.close();
        if (battDst) battDst.close();
        if (eventDst) eventDst.close();
        if (telemDst) telemDst.close();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (gpsDst) {
        const float gpsRelAlt = (rec.rel_alt_cm == INT32_MIN) ? NAN : rec.rel_alt_cm / 100.0f;
        const float baroGpsDelta = (rec.baro_gps_delta_cm == INT32_MIN) ? NAN : rec.baro_gps_delta_cm / 100.0f;
        char line[256];
        snprintf(line, sizeof(line), "%lu,%u,%u,%u,%.1f,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%lu,%u,%u,%u,%u",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.fix_type,
                 (unsigned int)rec.sats,
                 rec.hdop_x10 / 10.0f,
                 rec.lat_e7 / 1e7,
                 rec.lon_e7 / 1e7,
                 rec.alt_cm / 100.0f,
                 gpsRelAlt,
                 baroGpsDelta,
                 rec.speed_cms / 100.0f,
                 (unsigned long)rec.fix_age_ms_x10 * 10UL,
                 (unsigned int)rec.chars_delta,
                 (unsigned int)rec.pass_delta,
                 (unsigned int)rec.fail_delta,
                 (unsigned int)rec.flags);
        if (!gpsDst.println(line)) {
          if (dst) dst.close();
          if (imuDst) imuDst.close();
          if (baroDst) baroDst.close();
          gpsDst.close();
          if (battDst) battDst.close();
          if (eventDst) eventDst.close();
          if (telemDst) telemDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      gpsRows++;
    } else if (type == NAND_RECORD_BATT_V4 && size == sizeof(NandBatteryRecordV4)) {
      NandBatteryRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: battery record read failed");
        if (dst) dst.close();
        if (imuDst) imuDst.close();
        if (baroDst) baroDst.close();
        if (gpsDst) gpsDst.close();
        if (battDst) battDst.close();
        if (eventDst) eventDst.close();
        if (telemDst) telemDst.close();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (battDst) {
        char line[128];
        snprintf(line, sizeof(line), "%lu,%u,%.3f,%.3f,%.3f,%u,%u",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 rec.batt_mv / 1000.0f,
                 rec.raw_mv / 1000.0f,
                 rec.pin_mv / 1000.0f,
                 (unsigned int)rec.pack,
                 (unsigned int)rec.status_flags);
        if (!battDst.println(line)) {
          if (dst) dst.close();
          if (imuDst) imuDst.close();
          if (baroDst) baroDst.close();
          if (gpsDst) gpsDst.close();
          battDst.close();
          if (eventDst) eventDst.close();
          if (telemDst) telemDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      battRows++;
    } else if (type == NAND_RECORD_EVENT_V4 && size == sizeof(NandEventRecordV4)) {
      NandEventRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: event record read failed");
        if (dst) dst.close();
        if (imuDst) imuDst.close();
        if (baroDst) baroDst.close();
        if (gpsDst) gpsDst.close();
        if (battDst) battDst.close();
        if (eventDst) eventDst.close();
        if (telemDst) telemDst.close();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (eventDst) {
        char line[192];
        snprintf(line, sizeof(line), "%lu,%u,%u,%s,%u,%u,%u,%u,%u,%.2f,%.2f,%u",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.event_type,
                 eventTypeName(rec.event_type),
                 (unsigned int)rec.from_state,
                 (unsigned int)rec.to_state,
                 (unsigned int)rec.close_reason,
                 (unsigned int)rec.flight_flags,
                 (unsigned int)rec.diag_flags,
                 rec.rel_alt_cm / 100.0f,
                 rec.vel_cms / 100.0f,
                 (unsigned int)rec.health_flags);
        if (!eventDst.println(line)) {
          if (dst) dst.close();
          if (imuDst) imuDst.close();
          if (baroDst) baroDst.close();
          if (gpsDst) gpsDst.close();
          if (battDst) battDst.close();
          eventDst.close();
          if (telemDst) telemDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      eventRows++;
    } else if (type == NAND_RECORD_TELEM_V4 && size == sizeof(NandTelemetryRecordV4)) {
      NandTelemetryRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: telemetry record read failed");
        if (dst) dst.close();
        if (imuDst) imuDst.close();
        if (baroDst) baroDst.close();
        if (gpsDst) gpsDst.close();
        if (battDst) battDst.close();
        if (eventDst) eventDst.close();
        if (telemDst) telemDst.close();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (telemDst) {
        char line[224];
        snprintf(line, sizeof(line), "%lu,%u,%u,%lu,%u,%u,%lu,%lu,%lu,%lu,%d,%.3f",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.packet_type,
                 (unsigned long)rec.packet_seq,
                 (unsigned int)rec.state,
                 (unsigned int)rec.health_flags,
                 (unsigned long)rec.flight_seq,
                 (unsigned long)rec.nav_seq,
                 (unsigned long)rec.status_seq,
                 (unsigned long)rec.identity_seq,
                 (int)rec.last_rssi_dbm,
                 rec.batt_mv / 1000.0f);
        if (!telemDst.println(line)) {
          if (dst) dst.close();
          if (imuDst) imuDst.close();
          if (baroDst) baroDst.close();
          if (gpsDst) gpsDst.close();
          if (battDst) battDst.close();
          if (eventDst) eventDst.close();
          telemDst.close();
          if (attitudeDst) attitudeDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      telemRows++;
    } else if (type == NAND_RECORD_ATTITUDE_V4 && size == sizeof(NandAttitudeRecordV4)) {
      NandAttitudeRecordV4 rec = {};
      if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
        if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: attitude record read failed");
        if (dst) dst.close();
        if (imuDst) imuDst.close();
        if (baroDst) baroDst.close();
        if (gpsDst) gpsDst.close();
        if (battDst) battDst.close();
        if (eventDst) eventDst.close();
        if (telemDst) telemDst.close();
        if (attitudeDst) attitudeDst.close();
        src.close();
        return NAND_EXPORT_FAILED;
      }
      if (attitudeDst) {
        const bool accelCorr = rec.flags & (1u << 0);
        const bool magCorr = rec.flags & (1u << 1);
        const bool gyroOnly = rec.flags & (1u << 2);
        char line[224];
        snprintf(line, sizeof(line), "%lu,%u,%u,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.state,
                 rec.qw_i16 / 32767.0f,
                 rec.qx_i16 / 32767.0f,
                 rec.qy_i16 / 32767.0f,
                 rec.qz_i16 / 32767.0f,
                 rec.roll_cdeg / 100.0f,
                 rec.pitch_cdeg / 100.0f,
                 rec.yaw_cdeg / 100.0f,
                 (unsigned int)rec.diag_flags,
                 (unsigned int)rec.flags,
                 accelCorr ? 1u : 0u,
                 magCorr ? 1u : 0u,
                 gyroOnly ? 1u : 0u);
        if (!attitudeDst.println(line)) {
          if (dst) dst.close();
          if (imuDst) imuDst.close();
          if (baroDst) baroDst.close();
          if (gpsDst) gpsDst.close();
          if (battDst) battDst.close();
          if (eventDst) eventDst.close();
          if (telemDst) telemDst.close();
          attitudeDst.close();
          src.close();
          return NAND_EXPORT_FAILED;
        }
      }
      attitudeRows++;
    } else {
      badTypeRows++;
      if (SERIAL_DEBUG_LEVEL >= 2 && badTypeRows <= 8) {
        Serial.print("NAND export: unknown record type=");
        Serial.print(type);
        Serial.print(" size=");
        Serial.print(size);
        Serial.print(" offset=");
        Serial.println((unsigned long)offset);
      }
    }
    offset += size;
    recordsRead++;
    if (SERIAL_DEBUG_LEVEL >= 1 && (recordsRead % 1000u) == 0) {
      Serial.print("NAND export: progress records=");
      Serial.print((unsigned long)recordsRead);
      Serial.print("/");
      Serial.print((unsigned long)meta.record_count);
      Serial.print(" full=");
      Serial.print((unsigned long)fullRows);
      Serial.print(" imu=");
      Serial.print((unsigned long)imuRows);
      Serial.print(" att=");
      Serial.print((unsigned long)attitudeRows);
      Serial.print(" bad=");
      Serial.print((unsigned long)badTypeRows);
      Serial.print(" implausible=");
      Serial.println((unsigned long)implausibleRows);
    }
  }

  dst.flush();
  dst.close();
  if (imuDst) {
    imuDst.flush();
    imuDst.close();
  }
  if (baroDst) {
    baroDst.flush();
    baroDst.close();
  }
  if (gpsDst) {
    gpsDst.flush();
    gpsDst.close();
  }
  if (battDst) {
    battDst.flush();
    battDst.close();
  }
  if (eventDst) {
    eventDst.flush();
    eventDst.close();
  }
  if (telemDst) {
    telemDst.flush();
    telemDst.close();
  }
  if (attitudeDst) {
    attitudeDst.flush();
    attitudeDst.close();
  }
  src.close();
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("NAND export: done records=");
    Serial.print((unsigned long)recordsRead);
    Serial.print(" full=");
    Serial.print((unsigned long)fullRows);
    Serial.print(" imu=");
    Serial.print((unsigned long)imuRows);
    Serial.print(" baro=");
    Serial.print((unsigned long)baroRows);
    Serial.print(" gps=");
    Serial.print((unsigned long)gpsRows);
    Serial.print(" batt=");
    Serial.print((unsigned long)battRows);
    Serial.print(" event=");
    Serial.print((unsigned long)eventRows);
    Serial.print(" telem=");
    Serial.print((unsigned long)telemRows);
    Serial.print(" att=");
    Serial.print((unsigned long)attitudeRows);
    Serial.print(" bad=");
    Serial.print((unsigned long)badTypeRows);
    Serial.print(" implausible=");
    Serial.println((unsigned long)implausibleRows);
  }
  if (!wroteFull && !wroteImu) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export: no rows written, skipped");
    return NAND_EXPORT_SKIPPED;
  }
#undef CLOSE_DETAIL_EXPORT_FILES
  return NAND_EXPORT_EXPORTED;
}

static bool exportAllNandLogsToSd(uint32_t operationId, bool latestOnly, bool exportImu, uint32_t &exportedCount,
                                  uint32_t &skippedCount, uint32_t &failedCount) {
  exportedCount = 0;
  skippedCount = 0;
  failedCount = 0;
  File root = qspiNand.open("/");
  if (!root) return false;

  int latestIndex = -1;
  if (latestOnly) {
    while (true) {
      File f = root.openNextFile();
      if (!f) break;
      if (!f.isDirectory()) {
        const char *name = f.name();
        int idx = extractPrefixedIndex(name, "rocket_flt");
        if (idx < 0) idx = extractPrefixedIndex(name, "flt");
        if (idx > latestIndex && strstr(name, ".bin")) latestIndex = idx;
      }
      f.close();
    }
    root.close();
    root = qspiNand.open("/");
    if (!root) return false;
    if (SERIAL_DEBUG_LEVEL >= 1) {
      Serial.print("NAND export all: latest_only index=");
      Serial.println(latestIndex);
    }
  }

  bool allOk = true;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      const char *name = f.name();
      int idx = extractPrefixedIndex(name, "rocket_flt");
      if (idx < 0) idx = extractPrefixedIndex(name, "flt");
      if (idx > 0 && strstr(name, ".bin")) {
        if (latestOnly && idx != latestIndex) {
          f.close();
          skippedCount++;
          continue;
        }
        char path[32];
        strncpy(path, name, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
        f.close();
        if (SERIAL_DEBUG_LEVEL >= 1) {
          Serial.print("NAND export all: file ");
          Serial.println(path);
        }
        NandExportResult result = exportOneNandLogToSd(path, operationId, exportImu);
        if (result == NAND_EXPORT_EXPORTED) {
          exportedCount++;
          if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export all: result exported");
        } else if (result == NAND_EXPORT_SKIPPED) {
          skippedCount++;
          if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export all: result skipped");
        } else {
          failedCount++;
          allOk = false;
          if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND export all: result failed");
        }
        continue;
      }
    }
    f.close();
  }
  root.close();
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("NAND export all: complete exported=");
    Serial.print((unsigned long)exportedCount);
    Serial.print(" skipped=");
    Serial.print((unsigned long)skippedCount);
    Serial.print(" failed=");
    Serial.println((unsigned long)failedCount);
  }
  return allOk;
}

static bool eraseAllNandLogs(uint32_t &removedCount) {
  removedCount = 0;
  File root = qspiNand.open("/");
  if (!root) return false;

  bool allOk = true;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      const char *name = f.name();
      int idx = extractPrefixedIndex(name, "rocket_flt");
      if (idx < 0) idx = extractPrefixedIndex(name, "flt");
      if (idx > 0 && strstr(name, ".bin")) {
        char path[32];
        strncpy(path, name, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
        f.close();
        if (SERIAL_DEBUG_LEVEL >= 1) {
          Serial.print("NAND erase: remove ");
          Serial.println(path);
        }
        if (qspiNand.remove(path)) {
          removedCount++;
        } else {
          allOk = false;
          if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND erase: remove failed");
        }
        continue;
      }
    }
    f.close();
  }
  root.close();
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("NAND erase: complete removed=");
    Serial.print((unsigned long)removedCount);
    Serial.print(" ok=");
    Serial.println(allOk ? 1 : 0);
  }
  return allOk;
}

static bool hasSuffix(const char *text, const char *suffix) {
  if (!text || !suffix) return false;
  const size_t textLen = strlen(text);
  const size_t suffixLen = strlen(suffix);
  return textLen >= suffixLen &&
         strcmp(text + textLen - suffixLen, suffix) == 0;
}

static bool isRocketSdLogName(const char *name) {
  if (!name || !hasSuffix(name, ".csv")) return false;
  if (strstr(name, "rocket_nand_") != nullptr) return true;

  int idx = extractPrefixedIndex(name, "rocket_flight");
  if (idx < 0) idx = extractPrefixedIndex(name, "flight");
  return idx > 0;
}

static bool eraseAllSdLogs(uint32_t &removedCount) {
  removedCount = 0;
  File root = SD.open("/");
  if (!root) return false;

  bool allOk = true;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory() && isRocketSdLogName(f.name())) {
      char path[96];
      strncpy(path, f.name(), sizeof(path) - 1);
      path[sizeof(path) - 1] = '\0';
      f.close();
      if (SD.remove(path)) {
        removedCount++;
      } else {
        allOk = false;
      }
      continue;
    }
    f.close();
  }
  root.close();
  return allOk;
}

bool storageEraseLogs(bool eraseNand, bool eraseSd,
                      uint32_t &nandRemoved, uint32_t &sdRemoved) {
  nandRemoved = 0;
  sdRemoved = 0;
  if (!eraseNand && !eraseSd) return false;
  if ((eraseNand && !nandOk) || (eraseSd && !sdOk)) return false;

  // Close both files before deleting either backend. Retained media will
  // restart on the next loop with an index consistent across SD and NAND.
  finalizeLogFiles(NAND_CLOSE_SERVICE);

  bool ok = true;
  if (eraseNand) ok = eraseAllNandLogs(nandRemoved) && ok;
  if (eraseSd) ok = eraseAllSdLogs(sdRemoved) && ok;

  nandLogCacheBytes = 0;
  nandRecordCount = 0;
  nandRecordSequence = 0;
  nandLogPath[0] = '\0';
  currentLogIndex = 0;
  nextLogIndex = 1;
  lastLogFlushMs = millis();
  lastGpsLoggedMs = 0;
  nandEventQueueHead = 0;
  nandEventQueueTail = 0;
  nandEventQueueCount = 0;
  nandEventDroppedCount = 0;
  nandFlightRamBytes = 0;
  nandFlightRamRecordCount = 0;
  nandFlightCriticalRamBytes = 0;
  nandFlightCriticalRecordCount = 0;
  nandFlightRamDroppedRecords = 0;
  nandFlightCriticalDroppedRecords = 0;
  nandFlightNonCriticalDroppedRecords = 0;
  logsFinalized = false;
  sdLogOk = false;
  nandLogOk = false;
  logOk = false;

  if (sdOk) scanSdLogFiles();
#if HAS_LITTLEFS_QPINAND
  if (nandOk) scanNandLogFiles();
#endif
  return ok;
}

static void resumeLoggingAfterSerialService() {
  nandLogCacheBytes = 0;
  nandFlightRamBytes = 0;
  nandFlightRamRecordCount = 0;
  nandFlightCriticalRamBytes = 0;
  nandFlightCriticalRecordCount = 0;
  nandRecordCount = 0;
  nandRecordSequence = 0;
  nandLogPath[0] = '\0';
  currentLogIndex = 0;
  lastLogFlushMs = millis();
  lastGpsLoggedMs = 0;
  logsFinalized = false;
  sdLogOk = false;
  nandLogOk = false;
  logOk = false;
  if (sdOk) scanSdLogFiles();
#if HAS_LITTLEFS_QPINAND
  if (nandOk) scanNandLogFiles();
#endif
}

void storagePrintStatus(Stream &out) {
  out.println("STORAGE STATUS");
  out.print("SD_OK "); out.println(sdOk ? 1 : 0);
  out.print("SD_RUNTIME_LOG "); out.println(SD_RUNTIME_LOG_ENABLE ? 1 : 0);
  out.print("NAND_OK "); out.println(nandOk ? 1 : 0);
  out.print("NAND_TOTAL_BYTES ");
  out.println(nandOk ? (uint32_t)qspiNand.totalSize() : 0u);
  out.print("NAND_USED_BYTES ");
  out.println(nandOk ? (uint32_t)qspiNand.usedSize() : 0u);
  out.print("NAND_FREE_BYTES ");
  out.println(nandOk ? (uint32_t)nandFreeBytes() : 0u);
  out.print("LOG_OK "); out.println(logOk ? 1 : 0);
  out.print("NAND_LOG_OK "); out.println(nandLogOk ? 1 : 0);
  out.print("LOG_FINALIZED "); out.println(logsFinalized ? 1 : 0);
  out.print("CURRENT_LOG_INDEX "); out.println(currentLogIndex);
  out.print("NEXT_LOG_INDEX "); out.println(nextLogIndex);
  out.print("NAND_CACHE_BYTES "); out.println(nandLogCacheBytes);
  out.print("FLIGHT_RAM_BYTES "); out.println(nandFlightRamBytes);
  out.print("FLIGHT_RAM_PRIMARY_CAPACITY ");
  out.println((uint32_t)NAND_FLIGHT_RAM1_BYTES +
              (uint32_t)NAND_FLIGHT_RAM2_BYTES -
              (uint32_t)NAND_FLIGHT_CRITICAL_RESERVE_BYTES);
  out.print("FLIGHT_RAM_CRITICAL_BYTES "); out.println(nandFlightCriticalRamBytes);
  out.print("FLIGHT_RAM_CRITICAL_CAPACITY ");
  out.println((uint32_t)NAND_FLIGHT_CRITICAL_RESERVE_BYTES);
  out.print("FLIGHT_RAM_DROPPED "); out.println(nandFlightRamDroppedRecords);
  out.print("FLIGHT_RAM_DROPPED_CRITICAL ");
  out.println(nandFlightCriticalDroppedRecords);
  out.print("FLIGHT_RAM_DROPPED_NONCRITICAL ");
  out.println(nandFlightNonCriticalDroppedRecords);
  out.print("NAND_RECORD_COUNT "); out.println(nandRecordCount);
  out.print("EVENT_QUEUE_DEPTH "); out.println(nandEventQueueCount);
  out.print("EVENT_DROPPED "); out.println(nandEventDroppedCount);
  out.println("STORAGE STATUS END");
}

void storageListNand(Stream &out) {
  out.println("NAND LIST BEGIN");
#if HAS_LITTLEFS_QPINAND
  if (!nandOk) {
    out.println("ERR NAND unavailable");
    out.println("NAND LIST END COUNT 0");
    return;
  }
  File root = qspiNand.open("/");
  if (!root) {
    out.println("ERR NAND root");
    out.println("NAND LIST END COUNT 0");
    return;
  }
  uint32_t count = 0;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      const char *name = f.name();
      int idx = extractPrefixedIndex(name, "rocket_flt");
      if (idx < 0) idx = extractPrefixedIndex(name, "flt");
      if (idx > 0 && strstr(name, ".bin")) {
        NandLogHeaderV3 header = {};
        const bool headerOk =
            f.seek(0) &&
            f.read((uint8_t *)&header, sizeof(header)) == (int)sizeof(header) &&
            memcmp(header.magic, "RV10NLG", 8) == 0;
        out.print("NAND_LOG INDEX "); out.print(idx);
        out.print(" NAME "); out.print(name);
        out.print(" BYTES "); out.print((uint32_t)f.size());
        out.print(" HEADER_OK "); out.print(headerOk ? 1 : 0);
        if (headerOk) {
          out.print(" RECORDS "); out.print(header.record_count);
          out.print(" FINALIZED ");
          out.print((header.flags & NAND_LOG_FLAG_FINALIZED) ? 1 : 0);
          out.print(" STATE "); out.print(header.final_state);
          out.print(" FW "); out.print(header.firmware_version);
        }
        out.println();
        count++;
      }
    }
    f.close();
  }
  root.close();
  out.print("NAND LIST END COUNT "); out.println(count);
#else
  out.println("ERR NAND unsupported");
  out.println("NAND LIST END COUNT 0");
#endif
}

void storageListSd(Stream &out) {
  out.println("SD LIST BEGIN");
  if (!sdOk) {
    out.println("ERR SD unavailable");
    out.println("SD LIST END COUNT 0");
    return;
  }
  File root = SD.open("/");
  if (!root) {
    out.println("ERR SD root");
    out.println("SD LIST END COUNT 0");
    return;
  }
  uint32_t count = 0;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    out.print(f.isDirectory() ? "SD_DIR NAME " : "SD_FILE NAME ");
    out.print(f.name());
    if (!f.isDirectory()) {
      out.print(" BYTES ");
      out.print((uint32_t)f.size());
    }
    out.println();
    count++;
    f.close();
  }
  root.close();
  out.print("SD LIST END COUNT "); out.println(count);
}

struct __attribute__((packed)) SerialTransferFrame {
  char magic[4];
  uint8_t version;
  uint8_t backend;
  uint16_t flags;
  uint32_t sequence;
  uint32_t offset;
  uint16_t length;
  uint16_t reserved;
  uint32_t crc32;
};
static_assert(sizeof(SerialTransferFrame) == 24, "SerialTransferFrame size mismatch");

static uint32_t updateCrc32(uint32_t crc, const uint8_t *data, size_t length) {
  while (length-- > 0) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
  }
  return crc;
}

static bool safeStorageName(const char *name) {
  if (!name || !*name || strlen(name) > 80) return false;
  if (strstr(name, "..") || strchr(name, '/') || strchr(name, '\\')) return false;
  for (const char *p = name; *p; ++p) {
    const char c = *p;
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')) {
      return false;
    }
  }
  return true;
}

static bool nandPathForIndex(uint32_t index, char *path, size_t pathSize) {
  if (!nandOk || index == 0 || !path || pathSize < 20) return false;
  snprintf(path, pathSize, "/rocket_flt%04lu.bin", (unsigned long)index);
  File f = qspiNand.open(path, FILE_READ);
  if (f) {
    f.close();
    return true;
  }
  snprintf(path, pathSize, "/flt%04lu.bin", (unsigned long)index);
  f = qspiNand.open(path, FILE_READ);
  if (!f) return false;
  f.close();
  return true;
}

static bool streamFileRange(File &file, const char *backendName,
                            const char *objectName, uint8_t backend,
                            uint32_t offset, uint32_t requestedLength,
                            Stream &out) {
  const uint32_t fileSize = (uint32_t)file.size();
  if (offset > fileSize) {
    out.println("ERR READ offset");
    return false;
  }
  uint32_t transferLength = fileSize - offset;
  if (requestedLength != 0 && requestedLength < transferLength) {
    transferLength = requestedLength;
  }
  if (!file.seek(offset)) {
    out.println("ERR READ seek");
    return false;
  }

  out.print("XFER BEGIN BACKEND "); out.print(backendName);
  out.print(" NAME "); out.print(objectName);
  out.print(" SIZE "); out.print(fileSize);
  out.print(" OFFSET "); out.print(offset);
  out.print(" LENGTH "); out.print(transferLength);
  out.println(" CHUNK 1024");

  uint8_t payload[1024];
  uint32_t sent = 0;
  uint32_t sequence = 0;
  uint32_t runningCrc = 0xFFFFFFFFu;
  while (sent < transferLength) {
    if (digitalRead(ARM_SWITCH_PIN) != ARM_SWITCH_SAFE_LEVEL ||
        (flightState != FS_IDLE && flightState != FS_PAD)) {
      out.println("\nXFER ERROR LOCKED");
      return false;
    }
    const uint16_t wanted =
        (uint16_t)min((uint32_t)sizeof(payload), transferLength - sent);
    const int got = file.read(payload, wanted);
    if (got <= 0) {
      out.println("\nXFER ERROR READ");
      return false;
    }
    const uint16_t payloadLength = (uint16_t)got;
    const uint32_t payloadCrc =
        updateCrc32(0xFFFFFFFFu, payload, payloadLength) ^ 0xFFFFFFFFu;
    runningCrc = updateCrc32(runningCrc, payload, payloadLength);

    SerialTransferFrame frame = {};
    memcpy(frame.magic, "RVXF", 4);
    frame.version = 1;
    frame.backend = backend;
    frame.flags = (sent + payloadLength == transferLength) ? 1u : 0u;
    frame.sequence = sequence++;
    frame.offset = offset + sent;
    frame.length = payloadLength;
    frame.crc32 = payloadCrc;
    out.write((const uint8_t *)&frame, sizeof(frame));
    out.write(payload, payloadLength);
    sent += payloadLength;
  }

  out.print("\nXFER END BYTES "); out.print(sent);
  out.print(" CRC32 ");
  char crcText[9];
  snprintf(crcText, sizeof(crcText), "%08lX",
           (unsigned long)(runningCrc ^ 0xFFFFFFFFu));
  out.println(crcText);
  return true;
}

bool storageInfoNand(uint32_t index, Stream &out) {
  char path[32];
  if (!nandPathForIndex(index, path, sizeof(path))) {
    out.println("ERR NAND INFO not found");
    return false;
  }
  File f = qspiNand.open(path, FILE_READ);
  if (!f) {
    out.println("ERR NAND INFO open");
    return false;
  }
  NandLogHeaderV3 header = {};
  const bool headerOk =
      f.read((uint8_t *)&header, sizeof(header)) == (int)sizeof(header) &&
      memcmp(header.magic, "RV10NLG", 8) == 0;
  out.print("NAND INFO INDEX "); out.print(index);
  out.print(" NAME "); out.print(path[0] == '/' ? path + 1 : path);
  out.print(" BYTES "); out.print((uint32_t)f.size());
  out.print(" HEADER_OK "); out.print(headerOk ? 1 : 0);
  if (headerOk) {
    out.print(" RECORDS "); out.print(header.record_count);
    out.print(" FINALIZED ");
    out.print((header.flags & NAND_LOG_FLAG_FINALIZED) ? 1 : 0);
    out.print(" STATE "); out.print(header.final_state);
    out.print(" FW "); out.print(header.firmware_version);
  }
  out.println();
  f.close();
  return true;
}

bool storageInfoSd(const char *name, Stream &out) {
  if (!sdOk || !safeStorageName(name)) {
    out.println("ERR SD INFO name");
    return false;
  }
  File f = SD.open(name, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    out.println("ERR SD INFO not found");
    return false;
  }
  out.print("SD INFO NAME "); out.print(name);
  out.print(" BYTES "); out.println((uint32_t)f.size());
  f.close();
  return true;
}

bool storageReadNand(uint32_t index, uint32_t offset, uint32_t length, Stream &out) {
  if (nandLogFile && index == currentLogIndex) {
    out.println("ERR NAND READ active log");
    return false;
  }
  char path[32];
  if (!nandPathForIndex(index, path, sizeof(path))) {
    out.println("ERR NAND READ not found");
    return false;
  }
  File f = qspiNand.open(path, FILE_READ);
  if (!f) {
    out.println("ERR NAND READ open");
    return false;
  }
  const bool ok = streamFileRange(
      f, "NAND", path[0] == '/' ? path + 1 : path, 1, offset, length, out);
  f.close();
  return ok;
}

bool storageReadSd(const char *name, uint32_t offset, uint32_t length, Stream &out) {
  if (!sdOk || !safeStorageName(name)) {
    out.println("ERR SD READ name");
    return false;
  }
  File f = SD.open(name, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    out.println("ERR SD READ not found");
    return false;
  }
  const bool ok = streamFileRange(f, "SD", name, 2, offset, length, out);
  f.close();
  return ok;
}

bool storageMountSd() {
  if (sdLogFile) {
    sdLogFile.close();
  }
  sdLogOk = false;
  sdOk = SD.begin(BUILTIN_SDCARD);
  if (sdOk) {
    sdOk = verifySdFilesystem();
  }
  if (sdOk) {
    scanSdLogFiles();
  }
  return sdOk;
}

bool storageExportNandToSd(int32_t index, bool includeDetail,
                           uint32_t &exportedCount, uint32_t &skippedCount,
                           uint32_t &failedCount) {
  exportedCount = 0;
  skippedCount = 0;
  failedCount = 0;
  if (!nandOk || !sdOk) return false;

  finalizeLogFiles(NAND_CLOSE_SERVICE);
  const uint32_t operationId = millis();
  bool ok = true;
  if (index < 0) {
    ok = exportAllNandLogsToSd(operationId, false, includeDetail,
                               exportedCount, skippedCount, failedCount);
  } else {
    char path[32];
    snprintf(path, sizeof(path), "/rocket_flt%04ld.bin", (long)index);
    NandExportResult result =
        exportOneNandLogToSd(path, operationId, includeDetail);
    if (result == NAND_EXPORT_SKIPPED) {
      snprintf(path, sizeof(path), "/flt%04ld.bin", (long)index);
      result = exportOneNandLogToSd(path, operationId, includeDetail);
    }
    if (result == NAND_EXPORT_EXPORTED) exportedCount = 1;
    else if (result == NAND_EXPORT_SKIPPED) skippedCount = 1;
    else failedCount = 1;
    ok = result == NAND_EXPORT_EXPORTED;
  }
  resumeLoggingAfterSerialService();
  return ok && failedCount == 0;
}

void processServiceModeIfRequested() {
  ServiceRequest req = loadServiceRequest();
  if (!req.valid) return;

  serviceModeActive = true;
  serviceModeFailed = false;
  setLedMode(LED_MODE_SERVICE);
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("Service op id=");
    Serial.println((unsigned long)req.operationId);
  }

  bool statusOk = true;
  bool exportDone = false;
  bool eraseDone = false;
  uint32_t exportCount = 0;
  uint32_t exportSkipped = 0;
  uint32_t exportFailed = 0;
  uint32_t eraseCount = 0;

  if (req.requireSdOk && !sdOk) statusOk = false;
  if (req.requireNandOk && !nandOk) statusOk = false;
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("Service checks: sdOk=");
    Serial.print(sdOk ? 1 : 0);
    Serial.print(" nandOk=");
    Serial.print(nandOk ? 1 : 0);
    Serial.print(" statusOk=");
    Serial.println(statusOk ? 1 : 0);
  }

#if HAS_LITTLEFS_QPINAND
  if (statusOk && req.exportToSd) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("Service export: finalize active logs");
    finalizeLogFiles(NAND_CLOSE_SERVICE);
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("Service export: start");
    exportDone = exportAllNandLogsToSd(req.operationId, req.exportLatestOnly, req.exportImu,
                                       exportCount, exportSkipped, exportFailed);
    statusOk = statusOk && exportDone;
    if (SERIAL_DEBUG_LEVEL >= 1) {
      Serial.print("Service export: done=");
      Serial.print(exportDone ? 1 : 0);
      Serial.print(" exported=");
      Serial.print((unsigned long)exportCount);
      Serial.print(" skipped=");
      Serial.print((unsigned long)exportSkipped);
      Serial.print(" failed=");
      Serial.println((unsigned long)exportFailed);
    }
  }

  if (statusOk && req.eraseNandAfterExport) {
    if (!req.exportToSd || !exportDone) {
      statusOk = false;
    } else {
      if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("Service erase: start");
      eraseDone = eraseAllNandLogs(eraseCount);
      statusOk = statusOk && eraseDone;
      if (SERIAL_DEBUG_LEVEL >= 1) {
        Serial.print("Service erase: done=");
        Serial.print(eraseDone ? 1 : 0);
        Serial.print(" count=");
        Serial.println((unsigned long)eraseCount);
      }
    }
  }
#else
  (void)exportCount;
  (void)exportSkipped;
  (void)exportFailed;
  (void)eraseCount;
#endif

  writeServiceResult(req, exportDone, exportCount, exportSkipped, exportFailed,
                     eraseDone, eraseCount, statusOk ? "success" : "failed");
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("Service result written status=");
    Serial.println(statusOk ? "success" : "failed");
  }
  if (statusOk) {
    SD.remove("/nand_ops.txt");
    serviceModeSuccess = true;
    setLedMode(LED_MODE_SUCCESS);
  } else {
    serviceModeFailed = true;
    updateLedModeFromHealth();
  }
  serviceModeActive = false;
}

void setupStorage() {
#if HAS_LITTLEFS_QPINAND
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND: begin...");
  delay(20);
  nandOk = qspiNand.begin();
  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("NAND: begin=");
    Serial.println(nandOk ? "1" : "0");
  }
  if (nandOk) {
    nandOk = verifyNandFilesystem();
    if (SERIAL_DEBUG_LEVEL >= 1) {
      Serial.print("NAND: total=");
      Serial.println(qspiNand.totalSize());
      Serial.print("NAND: used=");
      Serial.println(qspiNand.usedSize());
      Serial.println(nandOk ? "NAND: verify OK" : "NAND: verify FAILED");
    }
    if (nandOk) {
      scanNandLogFiles();
    }
  }
#else
  nandOk = false;
#endif

  storageMountSd();
  if (sdOk) {
#if SD_CONFIG_LOAD_ENABLE
    loadRocketConfig();
#endif
  }
  logOk = false;
  sdLogOk = false;
  nandLogOk = false;
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println(sdOk ? "SD: verify OK" : "SD: FAIL");
}

void storageTask() {
  const uint32_t timingStartUs = micros();
  static uint32_t lastSdLogMs = 0;
  static uint32_t lastNandLogMs = 0;
  const uint32_t nowMs = millis();
  const bool recoveryMode = (flightState == FS_LANDED);
  const uint32_t sdLogPeriodMs = recoveryMode ? RECOVERY_SD_LOG_UPDATE_MS : SD_LOG_UPDATE_MS;
  const uint32_t nandLogPeriodMs = recoveryMode ? RECOVERY_NAND_LOG_UPDATE_MS : NAND_LOG_UPDATE_MS;

  NandCloseReason finalizeReason = NAND_CLOSE_NONE;
  if (flightState != lastFlightState) {
    logNandEventBinary(nowMs, EVT_STATE_CHANGE, lastFlightState, flightState, NAND_CLOSE_NONE);
    if (flightState == FS_LANDED) {
      setFinderBeeper(true);
      finalizeReason = NAND_CLOSE_LANDED;
    } else if (flightState == FS_ABORT) {
      finalizeReason = NAND_CLOSE_ABORT;
    }
    lastFlightState = flightState;
  }

  drainNandEventQueue();

  if (taskDue(nowMs, lastNandLogMs, nandLogPeriodMs)) {
    logNandBinary();
  }

#if SD_RUNTIME_LOG_ENABLE
  if (taskDue(nowMs, lastSdLogMs, sdLogPeriodMs)) {
    logSdCsv();
  }
#else
  (void)lastSdLogMs;
  (void)sdLogPeriodMs;
#endif

  if (finalizeReason != NAND_CLOSE_NONE) {
    finalizeLogFiles(finalizeReason);
  }

  flushLogsIfDue();
  timingRecordStorage(STORAGE_TIMING_TASK, micros() - timingStartUs);
}
