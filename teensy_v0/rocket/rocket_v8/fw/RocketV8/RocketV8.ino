#include "config.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <math.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <LittleFS.h>
#define HAS_ADAFRUIT_LSM9DS1 1
#define HAS_LITTLEFS_QPINAND 1

enum FlightState : uint8_t {
  FS_IDLE = 0,
  FS_PAD,
  FS_ASCENT,
  FS_COAST,
  FS_DESCENT,
  FS_LANDED,
  FS_ABORT
};

enum BatteryPackType : uint8_t {
  BATT_PACK_UNKNOWN = 0,
  BATT_PACK_1S,
  BATT_PACK_2S
};

enum LedMode : uint8_t {
  LED_MODE_BOOT = 0,
  LED_MODE_READY,
  LED_MODE_SERVICE,
  LED_MODE_SUCCESS,
  LED_MODE_ERROR_SD,
  LED_MODE_ERROR_NAND,
  LED_MODE_ERROR_GENERAL
};

static const uint16_t FLAG_LAUNCH = 1 << 0;
static const uint16_t FLAG_APOGEE = 1 << 1;
static const uint16_t FLAG_LANDED = 1 << 2;

struct __attribute__((packed)) FlightPacketV7 {
  uint8_t  version;
  uint8_t  state;
  uint16_t flags;
  uint32_t seq;
  uint32_t ms;
  int32_t  alt_cm;
  int16_t  vel_cms;
  int16_t  ax_cms2;
  int16_t  ay_cms2;
  int16_t  az_cms2;
  int16_t  gx_cdeg;
  int16_t  gy_cdeg;
  int16_t  gz_cdeg;
  int16_t  roll_cdeg;
  int16_t  pitch_cdeg;
};
static_assert(sizeof(FlightPacketV7) == 34, "FlightPacketV7 size mismatch");

struct __attribute__((packed)) NavPacketV7 {
  uint8_t  version;
  uint8_t  gps_fix_type;
  uint8_t  gps_sats;
  uint8_t  gps_hdop_x10;
  uint32_t seq;
  uint32_t ms;
  int32_t  gps_lat_e7;
  int32_t  gps_lon_e7;
  int32_t  gps_alt_cm;
  int32_t  baro_alt_cm;
  uint32_t last_fix_age_ms;
};
static_assert(sizeof(NavPacketV7) == 32, "NavPacketV7 size mismatch");

struct __attribute__((packed)) StatusPacketV8 {
  uint8_t  version;
  uint8_t  state;
  uint16_t health_flags;
  uint32_t seq;
  uint32_t ms;
  uint16_t batt_mv;
  uint8_t  gps_sats;
  uint8_t  reserved0;
  int16_t  last_rssi_dbm;
  uint16_t reserved1;
};
static_assert(sizeof(StatusPacketV8) == 20, "StatusPacketV8 size mismatch");

struct __attribute__((packed)) NandLogHeaderV1 {
  char     magic[8];
  uint16_t version;
  uint16_t header_size;
  uint16_t record_size;
  uint16_t reserved0;
  uint32_t flight_index;
  uint32_t boot_ms;
};
static_assert(sizeof(NandLogHeaderV1) == 24, "NandLogHeaderV1 size mismatch");

struct __attribute__((packed)) NandFlightRecordV1 {
  uint32_t ms;
  uint16_t health_flags;
  uint16_t flight_flags;
  int32_t  alt_cm;
  int32_t  rel_alt_cm;
  int16_t  vel_cms;
  int16_t  temp_centi_c;
  uint32_t pressure_pa_x10;
  int16_t  ax_cms2;
  int16_t  ay_cms2;
  int16_t  az_cms2;
  int16_t  gx_cdeg;
  int16_t  gy_cdeg;
  int16_t  gz_cdeg;
  int16_t  roll_cdeg;
  int16_t  pitch_cdeg;
  int32_t  gps_lat_e7;
  int32_t  gps_lon_e7;
  int32_t  gps_alt_cm;
  int16_t  gps_speed_cms;
  uint16_t batt_mv;
  uint8_t  state;
  uint8_t  gps_fix_type;
  uint8_t  gps_sats;
  uint8_t  battery_pack;
};
static_assert(sizeof(NandFlightRecordV1) == 60, "NandFlightRecordV1 size mismatch");

struct ServiceRequest {
  bool valid = false;
  uint32_t operationId = 0;
  bool exportToSd = false;
  bool eraseNandAfterExport = false;
  bool requireNandOk = true;
  bool requireSdOk = true;
};

class MS5607Sensor {
public:
  bool begin() {
    address_ = 0;
    if (!probe(MS5607_ADDR_0) && !probe(MS5607_ADDR_1)) {
      return false;
    }
    reset();
    delay(10);
    for (uint8_t i = 0; i < 8; ++i) {
      prom_[i] = readPromWord(i);
    }
    return prom_[1] != 0 && prom_[1] != 0xFFFF;
  }

  bool read(float &tempC, float &pressurePa, float &altM) {
    if (!address_) return false;

    uint32_t d1 = 0;
    uint32_t d2 = 0;
    if (!readAdc(0x48, d1)) return false; // D1 OSR=4096
    if (!readAdc(0x58, d2)) return false; // D2 OSR=4096

    const int64_t C1 = prom_[1];
    const int64_t C2 = prom_[2];
    const int64_t C3 = prom_[3];
    const int64_t C4 = prom_[4];
    const int64_t C5 = prom_[5];
    const int64_t C6 = prom_[6];

    int64_t dT = (int64_t)d2 - (C5 << 8);
    int64_t temp = 2000 + ((dT * C6) >> 23);
    // MS5607 uses one-bit larger OFF/SENS scaling than MS5611-style code.
    // The previous shifts produced pressure near 0.5x real value.
    int64_t off = (C2 << 17) + ((C4 * dT) >> 6);
    int64_t sens = (C1 << 16) + ((C3 * dT) >> 7);

    int64_t t2 = 0;
    int64_t off2 = 0;
    int64_t sens2 = 0;
    if (temp < 2000) {
      int64_t dtLow = temp - 2000;
      t2 = (dT * dT) >> 31;
      off2 = (5 * dtLow * dtLow);
      sens2 = (5 * dtLow * dtLow) >> 1;
      if (temp < -1500) {
        int64_t dtVeryLow = temp + 1500;
        off2 += (7 * dtVeryLow * dtVeryLow) << 1;
        sens2 += 11 * dtVeryLow * dtVeryLow;
      }
    }

    temp -= t2;
    off -= off2;
    sens -= sens2;

    int32_t pressure = (int32_t)(((((int64_t)d1 * sens) >> 21) - off) >> 15);
    tempC = temp / 100.0f;
    pressurePa = (float)pressure;

    float pressureHpa = pressurePa / 100.0f;
    altM = 44330.0f * (1.0f - powf(pressureHpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));
    return isfinite(tempC) && isfinite(pressurePa) && isfinite(altM);
  }

private:
  uint8_t address_ = 0;
  uint16_t prom_[8] = {};

  bool probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) return false;
    address_ = addr;
    return true;
  }

  void reset() {
    Wire.beginTransmission(address_);
    Wire.write(0x1E);
    Wire.endTransmission();
  }

  uint16_t readPromWord(uint8_t index) {
    uint8_t cmd = 0xA0 + (index * 2);
    Wire.beginTransmission(address_);
    Wire.write(cmd);
    if (Wire.endTransmission() != 0) return 0;
    Wire.requestFrom((int)address_, 2);
    if (Wire.available() < 2) return 0;
    return (Wire.read() << 8) | Wire.read();
  }

  bool readAdc(uint8_t cmd, uint32_t &value) {
    Wire.beginTransmission(address_);
    Wire.write(cmd);
    if (Wire.endTransmission() != 0) return false;
    delay(10);

    Wire.beginTransmission(address_);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((int)address_, 3);
    if (Wire.available() < 3) return false;
    value = ((uint32_t)Wire.read() << 16) |
            ((uint32_t)Wire.read() << 8) |
            (uint32_t)Wire.read();
    return true;
  }
};

TinyGPSPlus gps;
MS5607Sensor ms5607;

#if HAS_ADAFRUIT_LSM9DS1
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(&Wire);
#endif

#if HAS_LITTLEFS_QPINAND
LittleFS_QPINAND qspiNand;
#endif

volatile bool loraTxBusy = false;
bool loraOk = false;
bool sdOk = false;
bool nandOk = false;
bool logOk = false;
bool sdLogOk = false;
bool nandLogOk = false;
bool imuOk = false;
bool baroOk = false;
bool serviceModeActive = false;
bool serviceModeSuccess = false;
bool serviceModeFailed = false;

LedMode ledMode = LED_MODE_BOOT;
uint32_t ledModeSinceMs = 0;

File sdLogFile;
File nandLogFile;
uint32_t nextLogIndex = 1;

FlightState flightState = FS_IDLE;
uint16_t flightFlags = 0;

bool haveAlt = false;
bool haveGoodFix = false;
bool haveImuEstimate = false;

int lastBattRaw = 0;
float lastBattPinV = 0.0f;
BatteryPackType batteryPack = BATT_PACK_UNKNOWN;
bool batteryWarn = false;
bool batteryCrit = false;

float filtAlt = 0.0f;
float filtTempC = 0.0f;
float filtPressurePa = 0.0f;
float velZ = 0.0f;
float baseAltM = NAN;
float maxAltM = 0.0f;
float maxVelMps = 0.0f;
float lastAltRaw = 0.0f;
float rocketBattV = 0.0f;

float last_ax = 0.0f;
float last_ay = 0.0f;
float last_az = 9.80665f;
float last_gx = 0.0f;
float last_gy = 0.0f;
float last_gz = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;

bool gpsHasFix = false;
uint8_t gpsFixType = 0;
uint8_t gpsSats = 0;
float gpsHdop = 99.9f;
double gpsLatDeg = 0.0;
double gpsLonDeg = 0.0;
float gpsAltM = 0.0f;
float gpsSpeedMps = 0.0f;
double lastFixLatDeg = 0.0;
double lastFixLonDeg = 0.0;
float lastFixAltM = 0.0f;
uint32_t lastFixTimeMs = 0;

uint32_t flightSeq = 0;
uint32_t navSeq = 0;
uint32_t statusSeq = 0;
uint32_t tLaunchMs = 0;
uint32_t tApogeeMs = 0;
uint32_t lastLogFlushMs = 0;

static float readBatteryVoltage() {
  lastBattRaw = analogRead(VBAT_PIN);
  lastBattPinV = (float)lastBattRaw * ADC_REF_V / ADC_MAX_COUNTS;
  float vPin = lastBattPinV;
  return vPin * (VBAT_R1_OHMS + VBAT_R2_OHMS) / VBAT_R2_OHMS;
}

static BatteryPackType detectBatteryPack(float battV) {
  if (!isfinite(battV) || battV <= 0.0f) return BATT_PACK_UNKNOWN;
  return (battV >= BATT_2S_DETECT_V) ? BATT_PACK_2S : BATT_PACK_1S;
}

static void updateBatteryStatus(float battV) {
  batteryPack = detectBatteryPack(battV);
  batteryWarn = false;
  batteryCrit = false;

  if (batteryPack == BATT_PACK_1S) {
    batteryWarn = battV < BATT_1S_WARN_V;
    batteryCrit = battV < BATT_1S_CRIT_V;
  } else if (batteryPack == BATT_PACK_2S) {
    batteryWarn = battV < BATT_2S_WARN_V;
    batteryCrit = battV < BATT_2S_CRIT_V;
  }
}

static const char *batteryPackName() {
  switch (batteryPack) {
    case BATT_PACK_1S: return "1S";
    case BATT_PACK_2S: return "2S";
    default: return "UNK";
  }
}

static uint16_t buildHealthFlags() {
  uint16_t flags = 0;
  if (baroOk) flags |= HEALTH_BARO_OK;
  if (imuOk)  flags |= HEALTH_IMU_OK;
  if (gpsHasFix) flags |= HEALTH_GPS_OK;
  if (sdOk)   flags |= HEALTH_SD_OK;
  if (nandOk) flags |= HEALTH_NAND_OK;
  if (sdLogOk || nandLogOk) flags |= HEALTH_LOG_OK;
  if (!batteryWarn) flags |= HEALTH_BATT_OK;
  return flags;
}

static void writeStatusLed(bool on) {
#if STATUS_LED_ACTIVE_HIGH
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
#else
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
#endif
}

static void setLedMode(LedMode mode) {
  if (ledMode == mode) return;
  ledMode = mode;
  ledModeSinceMs = millis();
}

static void updateLedModeFromHealth() {
  if (serviceModeActive) return;
  if (!nandOk) {
    setLedMode(LED_MODE_ERROR_NAND);
  } else if (!sdOk) {
    setLedMode(LED_MODE_ERROR_SD);
  } else if (!baroOk || !imuOk || !loraOk) {
    setLedMode(LED_MODE_ERROR_GENERAL);
  } else {
    setLedMode(LED_MODE_READY);
  }
}

static void updateStatusLed() {
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
    case LED_MODE_READY:
      phase = nowMs % 2000u;
      on = phase < 80u;
      break;
    case LED_MODE_SERVICE:
      on = ((nowMs / 100u) % 2u) == 0u;
      break;
    case LED_MODE_SUCCESS:
      on = true;
      break;
    case LED_MODE_ERROR_SD:
      phase = nowMs % 1200u;
      on = (phase < 120u) || (phase >= 240u && phase < 360u);
      break;
    case LED_MODE_ERROR_NAND:
      phase = nowMs % 1400u;
      on = (phase < 100u) || (phase >= 200u && phase < 300u) || (phase >= 400u && phase < 500u);
      break;
    case LED_MODE_ERROR_GENERAL:
    default:
      on = ((nowMs / 250u) % 2u) == 0u;
      break;
  }

  if (serviceModeSuccess && (uint32_t)(nowMs - ledModeSinceMs) > 3000u) {
    serviceModeSuccess = false;
    updateLedModeFromHealth();
  }

  writeStatusLed(on);
}

void onLoraTxDone() {
  loraTxBusy = false;
}

static int extractPrefixedIndex(const char *name, const char *prefix) {
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

static void scanSdLogFiles() {
  File root = SD.open("/");
  if (!root) return;
  uint32_t maxIdx = 0;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      int idx = extractPrefixedIndex(f.name(), "flight");
      if (idx > (int)maxIdx) maxIdx = idx;
    }
    f.close();
  }
  root.close();
  if (maxIdx >= nextLogIndex) nextLogIndex = maxIdx + 1;
}

#if HAS_LITTLEFS_QPINAND
static void scanNandLogFiles() {
  File root = qspiNand.open("/");
  if (!root) return;
  uint32_t maxIdx = 0;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      int idx = extractPrefixedIndex(f.name(), "flt");
      if (idx > (int)maxIdx) maxIdx = idx;
    }
    f.close();
  }
  root.close();
  if (maxIdx >= nextLogIndex) nextLogIndex = maxIdx + 1;
}
#endif

static void ensureLogOpen() {
  bool openedAny = false;
  if (sdOk && !sdLogFile) {
    char filename[32];
    snprintf(filename, sizeof(filename), "flight%lu.csv", (unsigned long)nextLogIndex);
    sdLogFile = SD.open(filename, FILE_WRITE);
    if (sdLogFile) {
      sdLogOk = true;
      openedAny = true;
      sdLogFile.println("ms,state,flags,alt_m,rel_alt_m,vel_mps,ax,ay,az,gx,gy,gz,roll,pitch,lat,lon,gps_alt,batt_v,health");
      sdLogFile.flush();
    } else {
      sdLogOk = false;
    }
  }

#if HAS_LITTLEFS_QPINAND
  if (nandOk && !nandLogFile) {
    char filename[32];
    snprintf(filename, sizeof(filename), "/flt%04lu.bin", (unsigned long)nextLogIndex);
    nandLogFile = qspiNand.open(filename, FILE_WRITE);
    if (nandLogFile) {
      NandLogHeaderV1 header = {};
      memcpy(header.magic, "RV8NLOG", 8);
      header.version = 1;
      header.header_size = sizeof(header);
      header.record_size = sizeof(NandFlightRecordV1);
      header.flight_index = nextLogIndex;
      header.boot_ms = millis();
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
  logOk = sdLogOk || nandLogOk;
}

static void logCsv() {
  ensureLogOpen();
  const uint32_t nowMs = millis();
  float relAlt = filtAlt;
  if (!isnan(baseAltM)) relAlt = filtAlt - baseAltM;
  const uint16_t healthFlags = buildHealthFlags();

  if (sdLogFile) {
    char line[288];
    snprintf(line, sizeof(line),
             "%lu,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.7f,%.7f,%.2f,%.2f,%.2f,%u",
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
             roll * 57.2957795f,
             pitch * 57.2957795f,
             gpsLatDeg,
             gpsLonDeg,
             gpsAltM,
             rocketBattV,
             (unsigned int)healthFlags);
    if (!sdLogFile.println(line)) {
      sdLogOk = false;
      sdLogFile.close();
    }
  }

#if HAS_LITTLEFS_QPINAND
  if (nandLogFile) {
    NandFlightRecordV1 rec = {};
    rec.ms = nowMs;
    rec.health_flags = healthFlags;
    rec.flight_flags = flightFlags;
    rec.alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
    rec.rel_alt_cm = (int32_t)lroundf(relAlt * 100.0f);
    rec.vel_cms = (int16_t)lroundf(velZ * 100.0f);
    rec.temp_centi_c = (int16_t)lroundf(filtTempC * 100.0f);
    rec.pressure_pa_x10 = (uint32_t)lroundf(filtPressurePa * 10.0f);
    rec.ax_cms2 = (int16_t)lroundf(last_ax * 100.0f);
    rec.ay_cms2 = (int16_t)lroundf(last_ay * 100.0f);
    rec.az_cms2 = (int16_t)lroundf(last_az * 100.0f);
    rec.gx_cdeg = (int16_t)lroundf(last_gx * 100.0f);
    rec.gy_cdeg = (int16_t)lroundf(last_gy * 100.0f);
    rec.gz_cdeg = (int16_t)lroundf(last_gz * 100.0f);
    rec.roll_cdeg = (int16_t)lroundf(roll * 5729.57795f);
    rec.pitch_cdeg = (int16_t)lroundf(pitch * 5729.57795f);
    rec.gps_lat_e7 = (int32_t)llround(gpsLatDeg * 1e7);
    rec.gps_lon_e7 = (int32_t)llround(gpsLonDeg * 1e7);
    rec.gps_alt_cm = (int32_t)lroundf(gpsAltM * 100.0f);
    rec.gps_speed_cms = (int16_t)lroundf(gpsSpeedMps * 100.0f);
    rec.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
    rec.state = (uint8_t)flightState;
    rec.gps_fix_type = gpsFixType;
    rec.gps_sats = gpsSats;
    rec.battery_pack = (uint8_t)batteryPack;
    if (nandLogFile.write((const uint8_t *)&rec, sizeof(rec)) != sizeof(rec)) {
      nandLogOk = false;
      nandLogFile.close();
    }
  }
#endif

  if ((nowMs - lastLogFlushMs) >= LOG_FLUSH_MS) {
    lastLogFlushMs = nowMs;
    if (sdLogFile) sdLogFile.flush();
#if HAS_LITTLEFS_QPINAND
    if (nandLogFile) nandLogFile.flush();
#endif
  }
  logOk = sdLogOk || nandLogOk;
}

static bool taskDue(uint32_t nowMs, uint32_t &lastRunMs, uint32_t periodMs) {
  if ((uint32_t)(nowMs - lastRunMs) < periodMs) return false;
  lastRunMs = nowMs;
  return true;
}

static void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("LoRa: not found");
    loraOk = false;
    return;
  }
  LoRa.setSPIFrequency(LORA_SPI_FREQ_HZ);
  LoRa.onTxDone(onLoraTxDone);
  loraOk = true;
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("LoRa: OK");
}

#if HAS_LITTLEFS_QPINAND
static bool verifyNandFilesystem() {
  File f = qspiNand.open("/rocket.txt", FILE_WRITE);
  if (!f) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("NAND: open write failed");
    return false;
  }
  f.println("RocketV8");
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
#endif

static bool verifySdFilesystem() {
  File f = SD.open("/rocket_sd.txt", FILE_WRITE);
  if (!f) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("SD: open write failed");
    return false;
  }
  f.println("RocketV8");
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

static bool parseBoolValue(const String &value, bool defaultValue) {
  if (value == "1" || value == "true" || value == "yes") return true;
  if (value == "0" || value == "false" || value == "no") return false;
  return defaultValue;
}

static ServiceRequest loadServiceRequest() {
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

#if HAS_LITTLEFS_QPINAND
static bool exportOneNandLogToSd(const char *nandName, uint32_t operationId) {
  File src = qspiNand.open(nandName, FILE_READ);
  if (!src) return false;

  NandLogHeaderV1 header = {};
  if (src.read((uint8_t *)&header, sizeof(header)) != (int)sizeof(header)) {
    src.close();
    return false;
  }
  if (memcmp(header.magic, "RV8NLOG", 8) != 0 || header.record_size != sizeof(NandFlightRecordV1)) {
    src.close();
    return false;
  }

  char csvName[48];
  snprintf(csvName, sizeof(csvName), "nand_%04lu_op%lu.csv",
           (unsigned long)header.flight_index, (unsigned long)operationId);
  File dst = SD.open(csvName, FILE_WRITE);
  if (!dst) {
    src.close();
    return false;
  }

  dst.println("ms,state,flags,health,alt_m,rel_alt_m,vel_mps,temp_c,pres_pa,ax,ay,az,gx,gy,gz,roll,pitch,gps_fix,sats,lat,lon,gps_alt_m,gps_speed_mps,batt_v,pack");
  NandFlightRecordV1 rec = {};
  while (src.available() >= (int)sizeof(rec)) {
    updateStatusLed();
    if (src.read((uint8_t *)&rec, sizeof(rec)) != (int)sizeof(rec)) {
      dst.close();
      src.close();
      return false;
    }
    char line[320];
    snprintf(line, sizeof(line),
             "%lu,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%.7f,%.7f,%.2f,%.2f,%.3f,%u",
             (unsigned long)rec.ms,
             (unsigned int)rec.state,
             (unsigned int)rec.flight_flags,
             (unsigned int)rec.health_flags,
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
             rec.roll_cdeg / 100.0f,
             rec.pitch_cdeg / 100.0f,
             (unsigned int)rec.gps_fix_type,
             (unsigned int)rec.gps_sats,
             rec.gps_lat_e7 / 1e7,
             rec.gps_lon_e7 / 1e7,
             rec.gps_alt_cm / 100.0f,
             rec.gps_speed_cms / 100.0f,
             rec.batt_mv / 1000.0f,
             (unsigned int)rec.battery_pack);
    if (!dst.println(line)) {
      dst.close();
      src.close();
      return false;
    }
  }

  dst.flush();
  dst.close();
  src.close();
  return true;
}

static bool exportAllNandLogsToSd(uint32_t operationId, uint32_t &exportedCount) {
  exportedCount = 0;
  File root = qspiNand.open("/");
  if (!root) return false;

  bool allOk = true;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      const char *name = f.name();
      int idx = extractPrefixedIndex(name, "flt");
      if (idx > 0 && strstr(name, ".bin")) {
        if (exportOneNandLogToSd(name, operationId)) {
          exportedCount++;
        } else {
          allOk = false;
        }
      }
    }
    f.close();
  }
  root.close();
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
      int idx = extractPrefixedIndex(name, "flt");
      if (idx > 0 && strstr(name, ".bin")) {
        if (qspiNand.remove(name)) {
          removedCount++;
        } else {
          allOk = false;
        }
      }
    }
    f.close();
  }
  root.close();
  return allOk;
}
#endif

static void writeServiceResult(const ServiceRequest &req, bool exportDone, uint32_t exportCount,
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
  f.print("\nexport_done=");
  f.print(exportDone ? "1" : "0");
  f.print("\nexport_count=");
  f.print((unsigned long)exportCount);
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

static void processServiceModeIfRequested() {
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
  uint32_t eraseCount = 0;

  if (req.requireSdOk && !sdOk) statusOk = false;
  if (req.requireNandOk && !nandOk) statusOk = false;

#if HAS_LITTLEFS_QPINAND
  if (statusOk && req.exportToSd) {
    exportDone = exportAllNandLogsToSd(req.operationId, exportCount);
    statusOk = statusOk && exportDone;
  }

  if (statusOk && req.eraseNandAfterExport) {
    if (!req.exportToSd || !exportDone) {
      statusOk = false;
    } else {
      eraseDone = eraseAllNandLogs(eraseCount);
      statusOk = statusOk && eraseDone;
    }
  }
#else
  (void)exportCount;
  (void)eraseCount;
#endif

  writeServiceResult(req, exportDone, exportCount, eraseDone, eraseCount, statusOk ? "success" : "failed");
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

static void setupStorage() {
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

  sdOk = SD.begin(BUILTIN_SDCARD);
  if (sdOk) {
    sdOk = verifySdFilesystem();
  }
  if (sdOk) {
    scanSdLogFiles();
  }
  logOk = false;
  sdLogOk = false;
  nandLogOk = false;
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println(sdOk ? "SD: verify OK" : "SD: FAIL");
}

static void setupBaro() {
  baroOk = ms5607.begin();
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println(baroOk ? "MS5607: OK" : "MS5607: missing");
}

static bool probeI2cAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void setupImu() {
#if HAS_ADAFRUIT_LSM9DS1
  const bool imuAgSeen = probeI2cAddress(0x6B);
  const bool imuMagSeen = probeI2cAddress(0x1E);

  if (SERIAL_DEBUG_LEVEL >= 1) {
    Serial.print("LSM9DS1 probe AG(0x6B)=");
    Serial.print(imuAgSeen ? "1" : "0");
    Serial.print(" MAG(0x1E)=");
    Serial.println(imuMagSeen ? "1" : "0");
  }

  if (lsm.begin()) {
    lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_4G);
    lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_500DPS);
    lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);
    imuOk = true;
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("LSM9DS1: begin OK");
  } else {
    imuOk = false;
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("LSM9DS1: begin FAILED");
  }
#else
  imuOk = false;
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("LSM9DS1: library missing at compile time");
#endif
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println(imuOk ? "LSM9DS1: OK" : "LSM9DS1: unavailable");
}

static void printDebugStatus() {
  if (SERIAL_DEBUG_LEVEL <= 0) return;

  float relAlt = filtAlt;
  if (!isnan(baseAltM)) relAlt = filtAlt - baseAltM;

  if (SERIAL_DEBUG_LEVEL == 1) {
    Serial.print("state=");
    Serial.print((int)flightState);
    Serial.print(" alt=");
    Serial.print(filtAlt, 2);
    Serial.print(" vel=");
    Serial.print(velZ, 2);
    Serial.print(" gpsFix=");
    Serial.print((int)gpsFixType);
    Serial.print(" sats=");
    Serial.print((int)gpsSats);
    Serial.print(" batt=");
    Serial.print(rocketBattV, 2);
    Serial.print(" sd=");
    Serial.print(sdOk ? "1" : "0");
    Serial.print(" nand=");
    Serial.println(nandOk ? "1" : "0");
    return;
  }

  Serial.print("dbg ms=");
  Serial.print(millis());
  Serial.print(" state=");
  Serial.print((int)flightState);
  Serial.print(" flags=");
  Serial.print((unsigned int)flightFlags);
  Serial.print(" alt=");
  Serial.print(filtAlt, 2);
  Serial.print(" relAlt=");
  Serial.print(relAlt, 2);
  Serial.print(" vel=");
  Serial.print(velZ, 2);
  Serial.print(" tempC=");
  Serial.print(filtTempC, 2);
  Serial.print(" presPa=");
  Serial.print(filtPressurePa, 1);
  Serial.print(" ax=");
  Serial.print(last_ax, 2);
  Serial.print(" ay=");
  Serial.print(last_ay, 2);
  Serial.print(" az=");
  Serial.print(last_az, 2);
  Serial.print(" gx=");
  Serial.print(last_gx, 2);
  Serial.print(" gy=");
  Serial.print(last_gy, 2);
  Serial.print(" gz=");
  Serial.print(last_gz, 2);
  Serial.print(" rollDeg=");
  Serial.print(roll * 57.2957795f, 2);
  Serial.print(" pitchDeg=");
  Serial.print(pitch * 57.2957795f, 2);
  Serial.print(" gpsFix=");
  Serial.print((int)gpsFixType);
  Serial.print(" sats=");
  Serial.print((int)gpsSats);
  Serial.print(" hdop=");
  Serial.print(gpsHdop, 1);
  Serial.print(" lat=");
  Serial.print(gpsLatDeg, 7);
  Serial.print(" lon=");
  Serial.print(gpsLonDeg, 7);
  Serial.print(" gpsAlt=");
  Serial.print(gpsAltM, 2);
  Serial.print(" gpsSpd=");
  Serial.print(gpsSpeedMps, 2);
  Serial.print(" batt=");
  Serial.print(rocketBattV, 2);
  Serial.print(" pack=");
  Serial.print(batteryPackName());
  Serial.print(" bwarn=");
  Serial.print(batteryWarn ? "1" : "0");
  Serial.print(" bcrit=");
  Serial.print(batteryCrit ? "1" : "0");
  Serial.print(" battRaw=");
  Serial.print(lastBattRaw);
  Serial.print(" battPin=");
  Serial.print(lastBattPinV, 3);
  Serial.print(" health=");
  Serial.print(buildHealthFlags());
  Serial.print(" lora=");
  Serial.print(loraOk ? "1" : "0");
  Serial.print(" baro=");
  Serial.print(baroOk ? "1" : "0");
  Serial.print(" imu=");
  Serial.print(imuOk ? "1" : "0");
  Serial.print(" gps=");
  Serial.print(gpsHasFix ? "1" : "0");
  Serial.print(" sd=");
  Serial.print(sdOk ? "1" : "0");
  Serial.print(" sdlog=");
  Serial.print(sdLogOk ? "1" : "0");
  Serial.print(" nlog=");
  Serial.print(nandLogOk ? "1" : "0");
  Serial.print(" log=");
  Serial.print(logOk ? "1" : "0");
  Serial.print(" nand=");
  Serial.println(nandOk ? "1" : "0");
}

static void updateGps() {
  while (GPS_SERIAL.available() > 0) {
    gps.encode((char)GPS_SERIAL.read());
  }

  bool locValid = gps.location.isValid();
  bool altValid = gps.altitude.isValid();
  gpsHasFix = locValid;
  gpsLatDeg = locValid ? gps.location.lat() : 0.0;
  gpsLonDeg = locValid ? gps.location.lng() : 0.0;
  gpsAltM = altValid ? gps.altitude.meters() : 0.0f;
  gpsSpeedMps = gps.speed.isValid() ? gps.speed.mps() : 0.0f;
  gpsSats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  gpsHdop = gps.hdop.isValid() ? (gps.hdop.value() * 0.01f) : 99.9f;
  gpsFixType = locValid ? (altValid ? 3 : 2) : 0;

  if (gpsHasFix) {
    haveGoodFix = true;
    lastFixLatDeg = gpsLatDeg;
    lastFixLonDeg = gpsLonDeg;
    lastFixAltM = gpsAltM;
    lastFixTimeMs = millis();
  }
}

static void updateImu(float dt) {
#if HAS_ADAFRUIT_LSM9DS1
  if (!imuOk) return;

  sensors_event_t accel;
  sensors_event_t mag;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  last_ax = accel.acceleration.x;
  last_ay = accel.acceleration.y;
  last_az = accel.acceleration.z;
  last_gx = gyro.gyro.x * 57.2957795f;
  last_gy = gyro.gyro.y * 57.2957795f;
  last_gz = gyro.gyro.z * 57.2957795f;

  float ax_g = last_ax / 9.80665f;
  float ay_g = last_ay / 9.80665f;
  float az_g = last_az / 9.80665f;
  float rollAcc = atan2f(ay_g, az_g);
  float pitchAcc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

  if (!haveImuEstimate) {
    roll = rollAcc;
    pitch = pitchAcc;
    haveImuEstimate = true;
  } else {
    const float alpha = 0.96f;
    roll = alpha * (roll + (last_gx * 0.017453293f) * dt) + (1.0f - alpha) * rollAcc;
    pitch = alpha * (pitch + (last_gy * 0.017453293f) * dt) + (1.0f - alpha) * pitchAcc;
  }
#else
  (void)dt;
#endif
}

static void updateBaroAndState(float dt) {
  float tempC = 0.0f;
  float pressurePa = 0.0f;
  float altM = 0.0f;
  if (!ms5607.read(tempC, pressurePa, altM)) {
    baroOk = false;
    return;
  }

  baroOk = true;
  if (!haveAlt) {
    haveAlt = true;
    filtAlt = altM;
    lastAltRaw = altM;
    filtTempC = tempC;
    filtPressurePa = pressurePa;
    velZ = 0.0f;
    baseAltM = altM;
    flightState = FS_PAD;
    return;
  }

  filtAlt = 0.90f * filtAlt + 0.10f * altM;
  filtTempC = 0.90f * filtTempC + 0.10f * tempC;
  filtPressurePa = 0.90f * filtPressurePa + 0.10f * pressurePa;

  float rawVel = (altM - lastAltRaw) / dt;
  lastAltRaw = altM;
  velZ = 0.80f * velZ + 0.20f * rawVel;

  float relAlt = filtAlt;
  if (!isnan(baseAltM)) relAlt = filtAlt - baseAltM;

  float accMagG = sqrtf(last_ax * last_ax + last_ay * last_ay + last_az * last_az) / 9.80665f;
  uint32_t nowMs = millis();

  switch (flightState) {
    case FS_IDLE:
    case FS_PAD:
      if (accMagG > 2.0f || (relAlt > 8.0f && velZ > 12.0f)) {
        flightState = FS_ASCENT;
        flightFlags |= FLAG_LAUNCH;
        tLaunchMs = nowMs;
        maxAltM = relAlt;
        maxVelMps = max(0.0f, velZ);
      }
      break;

    case FS_ASCENT:
      if (relAlt > maxAltM) maxAltM = relAlt;
      if (velZ > maxVelMps) maxVelMps = velZ;
      if (velZ < 5.0f && (nowMs - tLaunchMs) > 750) {
        flightState = FS_COAST;
      }
      break;

    case FS_COAST:
      if (relAlt > maxAltM) maxAltM = relAlt;
      if (velZ > maxVelMps) maxVelMps = velZ;
      if (velZ < -0.5f && relAlt > 30.0f) {
        flightState = FS_DESCENT;
        flightFlags |= FLAG_APOGEE;
        tApogeeMs = nowMs;
      }
      break;

    case FS_DESCENT:
      if (fabsf(velZ) < 0.7f && relAlt < 20.0f && (nowMs - tLaunchMs) > 3000) {
        flightState = FS_LANDED;
        flightFlags |= FLAG_LANDED;
      }
      break;

    case FS_LANDED:
    case FS_ABORT:
    default:
      break;
  }
}

static void sendFlightTelemetry() {
  if (!loraOk || loraTxBusy || !haveAlt) return;

  FlightPacketV7 pkt = {};
  pkt.version = 7;
  pkt.state = (uint8_t)flightState;
  pkt.flags = flightFlags;
  pkt.seq = flightSeq++;
  pkt.ms = millis();
  pkt.alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
  pkt.vel_cms = (int16_t)lroundf(velZ * 100.0f);
  pkt.ax_cms2 = (int16_t)lroundf(last_ax * 100.0f);
  pkt.ay_cms2 = (int16_t)lroundf(last_ay * 100.0f);
  pkt.az_cms2 = (int16_t)lroundf(last_az * 100.0f);
  pkt.gx_cdeg = (int16_t)lroundf(last_gx * 100.0f);
  pkt.gy_cdeg = (int16_t)lroundf(last_gy * 100.0f);
  pkt.gz_cdeg = (int16_t)lroundf(last_gz * 100.0f);
  pkt.roll_cdeg = (int16_t)lroundf(roll * 5729.57795f);
  pkt.pitch_cdeg = (int16_t)lroundf(pitch * 5729.57795f);

  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_FLIGHT_V7);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
}

static void sendNavTelemetry() {
  if (!loraOk || loraTxBusy) return;

  NavPacketV7 pkt = {};
  pkt.version = 7;
  pkt.gps_fix_type = gpsFixType;
  pkt.gps_sats = gpsSats;
  int hdopX10 = (int)lroundf(gpsHdop * 10.0f);
  if (hdopX10 < 0) hdopX10 = 0;
  if (hdopX10 > 255) hdopX10 = 255;
  pkt.gps_hdop_x10 = (uint8_t)hdopX10;
  pkt.seq = navSeq++;
  pkt.ms = millis();
  pkt.gps_lat_e7 = (int32_t)llround(gpsLatDeg * 1e7);
  pkt.gps_lon_e7 = (int32_t)llround(gpsLonDeg * 1e7);
  pkt.gps_alt_cm = (int32_t)lroundf(gpsAltM * 100.0f);
  pkt.baro_alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
  pkt.last_fix_age_ms = haveGoodFix ? (millis() - lastFixTimeMs) : 0xFFFFFFFFu;

  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_NAV_V7);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
}

static void sendStatusTelemetry() {
  if (!loraOk || loraTxBusy) return;

  StatusPacketV8 pkt = {};
  pkt.version = 8;
  pkt.state = (uint8_t)flightState;
  pkt.health_flags = buildHealthFlags();
  pkt.seq = statusSeq++;
  pkt.ms = millis();
  pkt.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
  pkt.gps_sats = gpsSats;
  pkt.last_rssi_dbm = 0;

  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_STATUS_V8);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
}

static void sampleBatteryTask() {
  rocketBattV = readBatteryVoltage();
  updateBatteryStatus(rocketBattV);
}

static void sampleGpsTask() {
  updateGps();
}

static void sampleImuTask(float dtImu) {
  updateImu(dtImu);
}

static void sampleBaroTask(float dtBaro) {
  updateBaroAndState(dtBaro);
}

static void telemetryTask() {
  static uint32_t lastFlightTxMs = 0;
  static uint32_t lastNavTxMs = 0;
  static uint32_t lastStatusTxMs = 0;
  const uint32_t nowMs = millis();

  if (taskDue(nowMs, lastFlightTxMs, FLIGHT_TX_MS)) {
    sendFlightTelemetry();
  }
  if (taskDue(nowMs, lastNavTxMs, NAV_TX_MS)) {
    sendNavTelemetry();
  }
  if (taskDue(nowMs, lastStatusTxMs, STATUS_TX_MS)) {
    sendStatusTelemetry();
  }
}

static void storageTask() {
  static uint32_t lastLogMs = 0;
  const uint32_t nowMs = millis();
  if (taskDue(nowMs, lastLogMs, LOG_UPDATE_MS)) {
    logCsv();
  }
}

static void serialDebugTask() {
  static uint32_t lastPrintMs = 0;
  if (SERIAL_DEBUG_LEVEL <= 0) return;

  const uint32_t nowMs = millis();
  if (taskDue(nowMs, lastPrintMs, STATUS_PRINT_MS)) {
    printDebugStatus();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STATUS_LED_PIN, OUTPUT);
  writeStatusLed(false);
  setLedMode(LED_MODE_BOOT);

  Wire.begin();
  GPS_SERIAL.begin(GPS_BAUD);

#if defined(__IMXRT1062__)
  analogReadResolution(12);
#endif
  pinMode(VBAT_PIN, INPUT);

  setupLoRa();
  setupStorage();
  processServiceModeIfRequested();
  setupBaro();
  setupImu();

  sampleBatteryTask();
  updateLedModeFromHealth();

  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("RocketV8 ready");
}

void loop() {
  static uint32_t lastImuMs = 0;
  static uint32_t lastBaroMs = 0;
  static uint32_t lastBattMs = 0;
  static uint32_t lastImuUs = micros();

  sampleGpsTask();

  uint32_t nowUs = micros();
  float dtImu = (nowUs - lastImuUs) * 1e-6f;
  if (dtImu <= 0.0f || dtImu > 0.05f) dtImu = 0.01f;

  uint32_t nowMs = millis();
  if (taskDue(nowMs, lastBattMs, BATT_UPDATE_MS)) {
    sampleBatteryTask();
  }

  if (taskDue(nowMs, lastImuMs, IMU_UPDATE_MS)) {
    lastImuUs = nowUs;
    sampleImuTask(dtImu);
  }

  if ((uint32_t)(nowMs - lastBaroMs) >= BARO_UPDATE_MS) {
    float dtBaro = (nowMs - lastBaroMs) / 1000.0f;
    if (dtBaro <= 0.0f || dtBaro > 0.25f) dtBaro = 0.05f;
    lastBaroMs = nowMs;
    sampleBaroTask(dtBaro);
  }

  telemetryTask();
  storageTask();
  serialDebugTask();
  updateStatusLed();
}
