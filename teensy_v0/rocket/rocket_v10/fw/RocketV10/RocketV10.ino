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

static const uint16_t DIAG_BARO_GPS_DIVERGE = 1 << 0;
static const uint16_t DIAG_ATT_ACCEL_CORR = 1 << 1;
static const uint16_t DIAG_ATT_MAG_CORR = 1 << 2;
static const uint16_t DIAG_ATT_GYRO_ONLY = 1 << 3;
static const uint8_t NAND_RECORD_FULL_STATE_V4 = 1;
static const uint8_t NAND_RECORD_IMU_V4 = 2;
static const uint8_t NAND_RECORD_BARO_V4 = 3;
static const uint8_t NAND_RECORD_GPS_V4 = 4;
static const uint8_t NAND_RECORD_BATT_V4 = 5;
static const uint8_t NAND_RECORD_EVENT_V4 = 6;
static const uint8_t NAND_RECORD_TELEM_V4 = 7;
static const uint8_t NAND_RECORD_ATTITUDE_V4 = 8;

enum BaroReadStatus {
  BARO_READ_WAITING = 0,
  BARO_READ_SAMPLE = 1,
  BARO_READ_ERROR = 2
};

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
  uint8_t  launch_status;
  int16_t  last_rssi_dbm;
  uint16_t launch_wait_s;
};
static_assert(sizeof(StatusPacketV8) == 20, "StatusPacketV8 size mismatch");

struct __attribute__((packed)) IdentityPacketV1 {
  uint8_t  version;
  uint8_t  name_len;
  uint16_t reserved0;
  uint32_t seq;
  uint32_t ms;
  char     name[16];
};
static_assert(sizeof(IdentityPacketV1) == 28, "IdentityPacketV1 size mismatch");

struct __attribute__((packed)) NandLogHeaderV3 {
  char     magic[8];
  uint16_t version;
  uint16_t header_size;
  uint16_t record_size;
  uint16_t flags;
  uint32_t flight_index;
  uint32_t boot_ms;
  uint32_t open_ms;
  uint32_t close_ms;
  uint32_t record_count;
  uint8_t  final_state;
  uint8_t  close_reason;
  uint16_t reserved0;
  uint32_t reserved1;
  char     firmware_version[16];
  char     rocket_name[16];
  uint16_t imu_hz;
  uint16_t baro_hz;
  uint16_t nand_log_hz;
  uint16_t sd_log_hz;
  uint16_t flight_tx_hz_x10;
  uint16_t nav_tx_hz_x10;
  uint16_t status_tx_hz_x10;
  uint16_t identity_tx_hz_x10;
  uint16_t recovery_flight_tx_hz_x10;
  uint16_t recovery_nav_tx_hz_x10;
  uint16_t recovery_status_tx_hz_x10;
  uint16_t recovery_nand_log_hz_x10;
  uint16_t recovery_sd_log_hz_x10;
  uint16_t gyro_range_dps;
  uint8_t  accel_range_g;
  uint8_t  mag_range_gauss;
  uint8_t  estimator_version;
  uint8_t  record_format;
  uint8_t  reserved2[20];
};
static_assert(sizeof(NandLogHeaderV3) == 128, "NandLogHeaderV3 size mismatch");

struct NandLogMetadata {
  uint16_t header_version = 0;
  uint16_t header_size = 0;
  uint16_t record_size = 0;
  uint16_t flags = 0;
  uint32_t flight_index = 0;
  uint32_t boot_ms = 0;
  uint32_t open_ms = 0;
  uint32_t close_ms = 0;
  uint32_t record_count = 0;
  uint8_t final_state = 0;
  uint8_t close_reason = 0;
  char firmware_version[16] = "";
  char rocket_name[16] = "";
  uint16_t imu_hz = 0;
  uint16_t baro_hz = 0;
  uint16_t nand_log_hz = 0;
  uint16_t sd_log_hz = 0;
  uint16_t flight_tx_hz_x10 = 0;
  uint16_t nav_tx_hz_x10 = 0;
  uint16_t status_tx_hz_x10 = 0;
  uint16_t identity_tx_hz_x10 = 0;
  uint16_t recovery_flight_tx_hz_x10 = 0;
  uint16_t recovery_nav_tx_hz_x10 = 0;
  uint16_t recovery_status_tx_hz_x10 = 0;
  uint16_t recovery_nand_log_hz_x10 = 0;
  uint16_t recovery_sd_log_hz_x10 = 0;
  uint16_t gyro_range_dps = 0;
  uint8_t accel_range_g = 0;
  uint8_t mag_range_gauss = 0;
  uint8_t estimator_version = 0;
  uint8_t record_format = 0;
};

struct __attribute__((packed)) NandFlightRecordV3 {
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
  int32_t  gps_rel_alt_cm;
  int32_t  baro_gps_delta_cm;
  int16_t  gps_speed_cms;
  uint16_t batt_mv;
  uint16_t diag_flags;
  int16_t  mx_centiuT;
  int16_t  my_centiuT;
  int16_t  mz_centiuT;
  int16_t  yaw_cdeg;
  uint8_t  state;
  uint8_t  gps_fix_type;
  uint8_t  gps_sats;
  uint8_t  battery_pack;
};
static_assert(sizeof(NandFlightRecordV3) == 78, "NandFlightRecordV3 size mismatch");

struct __attribute__((packed)) NandFullStateRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  NandFlightRecordV3 data;
};
static_assert(sizeof(NandFullStateRecordV4) == 82, "NandFullStateRecordV4 size mismatch");

struct __attribute__((packed)) NandImuRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int16_t ax_cms2;
  int16_t ay_cms2;
  int16_t az_cms2;
  int16_t gx_cdeg;
  int16_t gy_cdeg;
  int16_t gz_cdeg;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandImuRecordV4) == 28, "NandImuRecordV4 size mismatch");

struct __attribute__((packed)) NandBaroRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int32_t alt_cm;
  int32_t rel_alt_cm;
  int16_t vel_cms;
  int16_t temp_centi_c;
  uint32_t pressure_pa_x10;
  uint16_t diag_flags;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandBaroRecordV4) == 28, "NandBaroRecordV4 size mismatch");

struct __attribute__((packed)) NandGpsRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_cm;
  int32_t rel_alt_cm;
  int32_t baro_gps_delta_cm;
  int16_t speed_cms;
  uint16_t fix_age_ms_x10;
  uint16_t chars_delta;
  uint16_t pass_delta;
  uint16_t fail_delta;
  uint8_t fix_type;
  uint8_t sats;
  uint8_t hdop_x10;
  uint8_t flags;
};
static_assert(sizeof(NandGpsRecordV4) == 42, "NandGpsRecordV4 size mismatch");

struct __attribute__((packed)) NandBatteryRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint16_t batt_mv;
  uint16_t raw_mv;
  uint16_t pin_mv;
  uint8_t pack;
  uint8_t status_flags;
};
static_assert(sizeof(NandBatteryRecordV4) == 16, "NandBatteryRecordV4 size mismatch");

struct __attribute__((packed)) NandEventRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint8_t event_type;
  uint8_t from_state;
  uint8_t to_state;
  uint8_t close_reason;
  uint16_t flight_flags;
  uint16_t diag_flags;
  int32_t rel_alt_cm;
  int16_t vel_cms;
  uint16_t health_flags;
};
static_assert(sizeof(NandEventRecordV4) == 24, "NandEventRecordV4 size mismatch");

struct __attribute__((packed)) NandTelemetryRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint8_t packet_type;
  uint8_t state;
  uint16_t health_flags;
  uint32_t packet_seq;
  uint32_t flight_seq;
  uint32_t nav_seq;
  uint32_t status_seq;
  uint32_t identity_seq;
  int16_t last_rssi_dbm;
  uint16_t batt_mv;
};
static_assert(sizeof(NandTelemetryRecordV4) == 36, "NandTelemetryRecordV4 size mismatch");

struct __attribute__((packed)) NandAttitudeRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int16_t qw_i16;
  int16_t qx_i16;
  int16_t qy_i16;
  int16_t qz_i16;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
  uint16_t diag_flags;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandAttitudeRecordV4) == 26, "NandAttitudeRecordV4 size mismatch");

struct ServiceRequest {
  bool valid = false;
  uint32_t operationId = 0;
  bool exportToSd = false;
  bool eraseNandAfterExport = false;
  bool exportLatestOnly = false;
  bool exportImu = true;
  bool requireNandOk = true;
  bool requireSdOk = true;
};

enum NandLogFlags : uint16_t {
  NAND_LOG_FLAG_FINALIZED = 1u << 0
};

enum NandCloseReason : uint8_t {
  NAND_CLOSE_NONE = 0,
  NAND_CLOSE_LANDED = 1,
  NAND_CLOSE_ABORT = 2,
  NAND_CLOSE_SERVICE = 3,
  NAND_CLOSE_ERROR = 4
};

enum NandExportResult : uint8_t {
  NAND_EXPORT_EXPORTED = 0,
  NAND_EXPORT_SKIPPED = 1,
  NAND_EXPORT_FAILED = 2
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

  BaroReadStatus update(float &tempC, float &pressurePa, float &altM) {
    if (!address_) return BARO_READ_ERROR;

    uint32_t nowUs = micros();
    if (conversion_ == CONV_NONE) {
      return startNextConversion() ? BARO_READ_WAITING : BARO_READ_ERROR;
    }

    if ((uint32_t)(nowUs - conversionStartUs_) < MS5607_CONVERSION_US) {
      return BARO_READ_WAITING;
    }

    uint32_t adc = 0;
    uint8_t completed = conversion_;
    conversion_ = CONV_NONE;
    if (!readAdcResult(adc)) return BARO_READ_ERROR;

    if (completed == CONV_D2) {
      d2_ = adc;
      haveD2_ = true;
      lastTempMs_ = millis();
      return startNextConversion() ? BARO_READ_WAITING : BARO_READ_ERROR;
    }

    d1_ = adc;
    if (!haveD2_) {
      return startNextConversion() ? BARO_READ_WAITING : BARO_READ_ERROR;
    }

    if (!calculateSample(tempC, pressurePa, altM)) return BARO_READ_ERROR;
    if (!startNextConversion()) return BARO_READ_ERROR;
    return BARO_READ_SAMPLE;
  }

private:
  static const uint8_t CONV_NONE = 0;
  static const uint8_t CONV_D1 = 1;
  static const uint8_t CONV_D2 = 2;

  uint8_t address_ = 0;
  uint16_t prom_[8] = {};
  uint8_t conversion_ = CONV_NONE;
  uint32_t conversionStartUs_ = 0;
  uint32_t lastTempMs_ = 0;
  uint32_t d1_ = 0;
  uint32_t d2_ = 0;
  bool haveD2_ = false;

  bool calculateSample(float &tempC, float &pressurePa, float &altM) {
    const int64_t C1 = prom_[1];
    const int64_t C2 = prom_[2];
    const int64_t C3 = prom_[3];
    const int64_t C4 = prom_[4];
    const int64_t C5 = prom_[5];
    const int64_t C6 = prom_[6];

    int64_t dT = (int64_t)d2_ - (C5 << 8);
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

    int32_t pressure = (int32_t)(((((int64_t)d1_ * sens) >> 21) - off) >> 15);
    tempC = temp / 100.0f;
    pressurePa = (float)pressure;

    float pressureHpa = pressurePa / 100.0f;
    altM = 44330.0f * (1.0f - powf(pressureHpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));
    return isfinite(tempC) && isfinite(pressurePa) && isfinite(altM);
  }

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

  bool startNextConversion() {
    uint32_t nowMs = millis();
    bool tempDue = !haveD2_ || (uint32_t)(nowMs - lastTempMs_) >= BARO_TEMP_UPDATE_MS;
    return startConversion(tempDue ? 0x58 : 0x48, tempDue ? CONV_D2 : CONV_D1);
  }

  bool startConversion(uint8_t cmd, uint8_t conversion) {
    Wire.beginTransmission(address_);
    Wire.write(cmd);
    if (Wire.endTransmission() != 0) return false;
    conversion_ = conversion;
    conversionStartUs_ = micros();
    return true;
  }

  bool readAdcResult(uint32_t &value) {
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
bool finderBeeperActive = false;
bool buttonPrevPressed = false;
bool buttonResetFired = false;
uint32_t buttonDownMs = 0;
uint32_t finderBeeperStartMs = 0;
uint32_t beepPatternStartMs = 0;
uint8_t beepPattern = 0;

File sdLogFile;
File nandLogFile;
char nandLogPath[32] = "";
uint32_t nextLogIndex = 1;
uint32_t nandRecordCount = 0;
#if HAS_LITTLEFS_QPINAND
uint8_t nandLogCache[NAND_LOG_CACHE_BYTES];
uint16_t nandLogCacheBytes = 0;
uint16_t nandRecordSequence = 0;
#endif
bool logsFinalized = false;
uint32_t currentLogIndex = 0;
uint32_t nandLogOpenMs = 0;
char rocketName[16] = DEFAULT_ROCKET_NAME;

FlightState flightState = FS_IDLE;
FlightState lastFlightState = FS_IDLE;
uint16_t flightFlags = 0;

bool haveAlt = false;
bool haveGoodFix = false;
bool haveImuEstimate = false;

int lastBattRaw = 0;
float lastBattPinV = 0.0f;
float rocketBattRawV = 0.0f;
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
float apogeeAltM = NAN;
float lastAltRaw = 0.0f;
float velRefAltM = 0.0f;
uint32_t velRefMs = 0;
float rocketBattV = 0.0f;

float last_ax = 0.0f;
float last_ay = 0.0f;
float last_az = 9.80665f;
float last_gx = 0.0f;
float last_gy = 0.0f;
float last_gz = 0.0f;
float last_mx = 0.0f;
float last_my = 0.0f;
float last_mz = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float attitudeQw = 1.0f;
float attitudeQx = 0.0f;
float attitudeQy = 0.0f;
float attitudeQz = 0.0f;
bool attitudeAccelCorrectionActive = false;
bool attitudeMagCorrectionActive = false;
bool attitudeGyroOnly = true;

bool gpsHasFix = false;
uint8_t gpsFixType = 0;
uint8_t gpsSats = 0;
float gpsHdop = 99.9f;
double gpsLatDeg = 0.0;
double gpsLonDeg = 0.0;
float gpsAltM = 0.0f;
float gpsSpeedMps = 0.0f;
float gpsBaseAltM = NAN;
float gpsRelAltM = 0.0f;
float baroGpsDeltaM = NAN;
uint16_t diagFlags = 0;
bool haveGpsBaseAlt = false;
bool baroGpsDiverged = false;
double lastFixLatDeg = 0.0;
double lastFixLonDeg = 0.0;
float lastFixAltM = 0.0f;
uint32_t lastFixTimeMs = 0;

uint32_t flightSeq = 0;
uint32_t navSeq = 0;
uint32_t statusSeq = 0;
uint32_t identitySeq = 0;
uint32_t tLaunchMs = 0;
uint32_t tApogeeMs = 0;
uint32_t lastLogFlushMs = 0;
uint32_t launchDetectSinceMs = 0;
uint32_t coastDetectSinceMs = 0;
uint32_t apogeeDetectSinceMs = 0;
uint32_t landedDetectSinceMs = 0;
uint32_t landedStillSinceMs = 0;
uint32_t launchArmStillSinceMs = 0;
uint32_t launchArmedMs = 0;
uint32_t lastGpsDataMs = 0;
uint32_t lastImuSampleMs = 0;
uint32_t lastBaroSampleMs = 0;
uint32_t lastGpsFixMs = 0;
uint32_t lastGpsBaseUpdateMs = 0;
uint32_t lastGpsLoggedMs = 0;
uint32_t lastGpsCharsLogged = 0;
uint32_t lastGpsPassLogged = 0;
uint32_t lastGpsFailLogged = 0;
uint32_t padSettleStartMs = 0;
bool launchArmed = false;

static const uint8_t REL_ALT_HISTORY_COUNT = 8;
float relAltHistoryM[REL_ALT_HISTORY_COUNT] = {};
uint32_t relAltHistoryMs[REL_ALT_HISTORY_COUNT] = {};
uint8_t relAltHistoryIndex = 0;
bool relAltHistoryFilled = false;

static float readBatteryVoltage() {
  lastBattRaw = analogRead(VBAT_PIN);
  lastBattPinV = (float)lastBattRaw * ADC_REF_V / ADC_MAX_COUNTS;
  float vPin = lastBattPinV;
  return vPin * (VBAT_R1_OHMS + VBAT_R2_OHMS) / VBAT_R2_OHMS;
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

static float wrapPi(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

static float angleDelta(float from, float to) {
  return wrapPi(to - from);
}

static int16_t quantizeUnitI16(float value) {
  value = max(-1.0f, min(1.0f, value));
  return (int16_t)lroundf(value * 32767.0f);
}

static float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static void normalizeAttitudeQuat() {
  const float norm = sqrtf(attitudeQw * attitudeQw + attitudeQx * attitudeQx +
                           attitudeQy * attitudeQy + attitudeQz * attitudeQz);
  if (!isfinite(norm) || norm < 1.0e-6f) {
    attitudeQw = 1.0f;
    attitudeQx = 0.0f;
    attitudeQy = 0.0f;
    attitudeQz = 0.0f;
    return;
  }
  const float inv = 1.0f / norm;
  attitudeQw *= inv;
  attitudeQx *= inv;
  attitudeQy *= inv;
  attitudeQz *= inv;
}

static void setAttitudeQuatFromEuler(float rollRad, float pitchRad, float yawRad) {
  const float cr = cosf(rollRad * 0.5f);
  const float sr = sinf(rollRad * 0.5f);
  const float cp = cosf(pitchRad * 0.5f);
  const float sp = sinf(pitchRad * 0.5f);
  const float cy = cosf(yawRad * 0.5f);
  const float sy = sinf(yawRad * 0.5f);

  attitudeQw = cr * cp * cy + sr * sp * sy;
  attitudeQx = sr * cp * cy - cr * sp * sy;
  attitudeQy = cr * sp * cy + sr * cp * sy;
  attitudeQz = cr * cp * sy - sr * sp * cy;
  normalizeAttitudeQuat();
}

static void updateEulerFromAttitudeQuat() {
  normalizeAttitudeQuat();
  const float sinrCosp = 2.0f * (attitudeQw * attitudeQx + attitudeQy * attitudeQz);
  const float cosrCosp = 1.0f - 2.0f * (attitudeQx * attitudeQx + attitudeQy * attitudeQy);
  roll = atan2f(sinrCosp, cosrCosp);

  const float sinp = 2.0f * (attitudeQw * attitudeQy - attitudeQz * attitudeQx);
  pitch = asinf(clampFloat(sinp, -1.0f, 1.0f));

  const float sinyCosp = 2.0f * (attitudeQw * attitudeQz + attitudeQx * attitudeQy);
  const float cosyCosp = 1.0f - 2.0f * (attitudeQy * attitudeQy + attitudeQz * attitudeQz);
  yaw = atan2f(sinyCosp, cosyCosp);
  if (yaw < 0.0f) yaw += 2.0f * PI;
}

static void integrateAttitudeQuatGyro(float gxRadS, float gyRadS, float gzRadS, float dt) {
  const float halfDt = 0.5f * dt;
  const float qw = attitudeQw;
  const float qx = attitudeQx;
  const float qy = attitudeQy;
  const float qz = attitudeQz;

  attitudeQw += (-qx * gxRadS - qy * gyRadS - qz * gzRadS) * halfDt;
  attitudeQx += ( qw * gxRadS + qy * gzRadS - qz * gyRadS) * halfDt;
  attitudeQy += ( qw * gyRadS - qx * gzRadS + qz * gxRadS) * halfDt;
  attitudeQz += ( qw * gzRadS + qx * gyRadS - qy * gxRadS) * halfDt;
  normalizeAttitudeQuat();
}

static void copyFixedString(char *dst, size_t dstSize, const char *src) {
  if (!dst || dstSize == 0) return;
  if (!src) src = "";
  size_t i = 0;
  while (i + 1 < dstSize && src[i] != '\0') {
    dst[i] = src[i];
    i++;
  }
  dst[i] = '\0';
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

static bool conditionHeld(bool active, uint32_t nowMs, uint32_t &sinceMs, uint32_t holdMs) {
  if (!active) {
    sinceMs = 0;
    return false;
  }
  if (sinceMs == 0) {
    sinceMs = nowMs;
    return holdMs == 0;
  }
  return (uint32_t)(nowMs - sinceMs) >= holdMs;
}

static const char *batteryPackName() {
  switch (batteryPack) {
    case BATT_PACK_1S: return "1S";
    case BATT_PACK_2S: return "2S";
    default: return "UNK";
  }
}

static bool isRecent(uint32_t lastMs, uint32_t staleMs) {
  return lastMs != 0 && (uint32_t)(millis() - lastMs) <= staleMs;
}

static bool isBaroFresh() {
  return isRecent(lastBaroSampleMs, BARO_STALE_MS);
}

static bool isImuFresh() {
  return isRecent(lastImuSampleMs, IMU_STALE_MS);
}

static bool isGpsFresh() {
  return isRecent(lastGpsDataMs, GPS_STALE_MS);
}

static void resetRelAltHistory(uint32_t nowMs, float relAlt) {
  for (uint8_t i = 0; i < REL_ALT_HISTORY_COUNT; ++i) {
    relAltHistoryM[i] = relAlt;
    relAltHistoryMs[i] = nowMs;
  }
  relAltHistoryIndex = 0;
  relAltHistoryFilled = true;
}

static void clearLaunchArmGate() {
  launchArmed = false;
  launchArmedMs = 0;
  launchArmStillSinceMs = 0;
  launchDetectSinceMs = 0;
}

static void setLaunchArmGate(uint32_t nowMs, float currentAltM) {
  launchArmed = true;
  launchArmedMs = nowMs;
  baseAltM = filtAlt;
  velRefAltM = currentAltM;
  velRefMs = nowMs;
  velZ = 0.0f;
  resetRelAltHistory(nowMs, 0.0f);
}

static uint16_t secondsCeilRemaining(uint32_t elapsedMs, uint32_t targetMs) {
  if (elapsedMs >= targetMs) return 0;
  uint32_t remainingMs = targetMs - elapsedMs;
  uint32_t seconds = (remainingMs + 999u) / 1000u;
  return seconds > 65535u ? 65535u : (uint16_t)seconds;
}

static uint8_t currentLaunchStatus(uint16_t &waitSecondsOut) {
  waitSecondsOut = 0;
  const uint32_t nowMs = millis();
  if (flightState != FS_IDLE && flightState != FS_PAD) {
    return LAUNCH_STATUS_FLIGHT;
  }
  if (nowMs < LAUNCH_POWERON_INHIBIT_MS) {
    waitSecondsOut = secondsCeilRemaining(nowMs, LAUNCH_POWERON_INHIBIT_MS);
    return LAUNCH_STATUS_INHIBIT;
  }
  if (launchArmed) {
    return LAUNCH_STATUS_READY;
  }
  if (launchArmStillSinceMs != 0) {
    waitSecondsOut = secondsCeilRemaining((uint32_t)(nowMs - launchArmStillSinceMs), LAUNCH_PAD_STILL_ARM_MS);
  } else {
    waitSecondsOut = LAUNCH_PAD_STILL_ARM_MS / 1000u;
  }
  return LAUNCH_STATUS_WAIT_STILL;
}

static void pushRelAltHistory(uint32_t nowMs, float relAlt) {
  relAltHistoryM[relAltHistoryIndex] = relAlt;
  relAltHistoryMs[relAltHistoryIndex] = nowMs;
  relAltHistoryIndex = (uint8_t)((relAltHistoryIndex + 1u) % REL_ALT_HISTORY_COUNT);
  if (relAltHistoryIndex == 0) relAltHistoryFilled = true;
}

static bool relAltAtLeastAgo(uint32_t nowMs, uint32_t ageMs, float &relAltOut) {
  if (!relAltHistoryFilled && relAltHistoryIndex == 0) return false;

  bool found = false;
  uint32_t bestAgeMs = 0xFFFFFFFFu;
  const uint8_t count = relAltHistoryFilled ? REL_ALT_HISTORY_COUNT : relAltHistoryIndex;
  for (uint8_t i = 0; i < count; ++i) {
    const uint32_t sampleAgeMs = nowMs - relAltHistoryMs[i];
    if (sampleAgeMs >= ageMs && sampleAgeMs < bestAgeMs) {
      bestAgeMs = sampleAgeMs;
      relAltOut = relAltHistoryM[i];
      found = true;
    }
  }
  return found;
}

static float currentBaroRelAltM() {
  if (isnan(baseAltM)) return filtAlt;
  return filtAlt - baseAltM;
}

static bool gpsAltitudeUsable() {
  return gpsHasFix && gpsFixType >= 3 && gps.altitude.isValid() && isGpsFresh();
}

static void updateGpsAltitudeReference() {
  if (!gpsAltitudeUsable()) {
    baroGpsDeltaM = NAN;
    baroGpsDiverged = false;
    diagFlags &= (uint16_t)~DIAG_BARO_GPS_DIVERGE;
    return;
  }

  const uint32_t nowMs = millis();
  if (flightState == FS_IDLE || flightState == FS_PAD) {
    if (!haveGpsBaseAlt) {
      gpsBaseAltM = gpsAltM;
      haveGpsBaseAlt = true;
      lastGpsBaseUpdateMs = nowMs;
    } else if ((uint32_t)(nowMs - lastGpsBaseUpdateMs) >= 800u) {
      gpsBaseAltM = (1.0f - GPS_ALT_BASE_ALPHA) * gpsBaseAltM + GPS_ALT_BASE_ALPHA * gpsAltM;
      lastGpsBaseUpdateMs = nowMs;
    }
  }

  if (haveGpsBaseAlt) {
    gpsRelAltM = gpsAltM - gpsBaseAltM;
    baroGpsDeltaM = currentBaroRelAltM() - gpsRelAltM;
    baroGpsDiverged = fabsf(baroGpsDeltaM) > GPS_BARO_DIVERGE_M;
  } else {
    gpsRelAltM = 0.0f;
    baroGpsDeltaM = NAN;
    baroGpsDiverged = false;
  }

  if (baroGpsDiverged) {
    diagFlags |= DIAG_BARO_GPS_DIVERGE;
  } else {
    diagFlags &= (uint16_t)~DIAG_BARO_GPS_DIVERGE;
  }
}

static uint16_t buildHealthFlags() {
  uint16_t flags = 0;
  if (baroOk && isBaroFresh()) flags |= HEALTH_BARO_OK;
  if (imuOk && isImuFresh())  flags |= HEALTH_IMU_OK;
  if (gpsHasFix && isGpsFresh()) flags |= HEALTH_GPS_OK;
  if (sdOk)   flags |= HEALTH_SD_OK;
  if (nandOk) flags |= HEALTH_NAND_OK;
  if (logOk) flags |= HEALTH_LOG_OK;
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
  } else if (!baroOk || !imuOk || !loraOk || !isBaroFresh() || !isImuFresh()) {
    setLedMode(LED_MODE_ERROR_GENERAL);
  } else {
    setLedMode(LED_MODE_READY);
  }
}

static bool startupHardwareOk() {
  return nandOk && sdOk && baroOk && imuOk && loraOk;
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

static void buzzerWrite(bool on, uint16_t freqHz = 2400) {
#if BUZZER_USE_TONE
  if (on) tone(BUZZER_PIN, freqHz);
  else noTone(BUZZER_PIN);
#else
  (void)freqHz;
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#endif
}

static void startBeepPattern(uint8_t pattern) {
  beepPattern = pattern;
  beepPatternStartMs = millis();
}

static void playBlockingTone(uint16_t durationMs, uint16_t freqHz, uint16_t pauseMs = 0) {
  buzzerWrite(true, freqHz);
  delay(durationMs);
  buzzerWrite(false);
  if (pauseMs > 0) delay(pauseMs);
}

static void playStartupSound(bool ok) {
  if (ok) {
    playBlockingTone(180, 2200, 120);
    playBlockingTone(180, 2700, 120);
    playBlockingTone(320, 3300, 0);
  } else {
    playBlockingTone(260, 2200, 160);
    playBlockingTone(260, 1500, 160);
    playBlockingTone(600, 900, 0);
  }
}

static bool noteActive(uint32_t t, uint32_t startMs, uint32_t durationMs) {
  return t >= startMs && t < (startMs + durationMs);
}

static void setFinderBeeper(bool active) {
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

static void updateBuzzer() {
  const uint32_t nowMs = millis();
  bool on = false;
  uint16_t freq = 2400;

  if (beepPattern != 0) {
    const uint32_t t = nowMs - beepPatternStartMs;
    switch (beepPattern) {
      case 1: // boot OK: rising "ready" melody
        on = noteActive(t, 0u, 180u) || noteActive(t, 280u, 180u) || noteActive(t, 560u, 260u);
        if (noteActive(t, 0u, 180u)) freq = 2200;
        else if (noteActive(t, 280u, 180u)) freq = 2700;
        else freq = 3300;
        if (t >= 980u) beepPattern = 0;
        break;
      case 2: // button reset accepted: clear rising confirmation
        on = noteActive(t, 0u, 160u) || noteActive(t, 260u, 160u) || noteActive(t, 520u, 320u);
        if (noteActive(t, 0u, 160u)) freq = 1800;
        else if (noteActive(t, 260u, 160u)) freq = 2400;
        else freq = 3200;
        if (t >= 1000u) beepPattern = 0;
        break;
      case 3: // button reset refused during active flight: low warning
        on = noteActive(t, 0u, 420u) || noteActive(t, 620u, 420u);
        freq = 850;
        if (t >= 1250u) beepPattern = 0;
        break;
      case 4: // startup failure: falling warning
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
      int idx = extractPrefixedIndex(f.name(), "rocket_flight");
      if (idx < 0) idx = extractPrefixedIndex(f.name(), "flight");
      if (idx > (int)maxIdx) maxIdx = idx;
    }
    f.close();
  }
  root.close();
  if (maxIdx >= nextLogIndex) nextLogIndex = maxIdx + 1;
}

#if HAS_LITTLEFS_QPINAND
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
#endif

static bool rewriteNandLogHeader(bool finalized, NandCloseReason closeReason) {
#if HAS_LITTLEFS_QPINAND
  if (!nandLogFile) return false;

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
  header.sd_log_hz = 1000u / SD_LOG_UPDATE_MS;
  header.flight_tx_hz_x10 = 10000u / FLIGHT_TX_MS;
  header.nav_tx_hz_x10 = 10000u / NAV_TX_MS;
  header.status_tx_hz_x10 = 10000u / STATUS_TX_MS;
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
  if (!nandLogFile.seek(0)) return false;
  if (nandLogFile.write((const uint8_t *)&header, sizeof(header)) != sizeof(header)) return false;
  if (!nandLogFile.seek(endPos)) return false;
  nandLogFile.flush();
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
  if (nandLogFile.write((const uint8_t *)nandLogCache, bytesToWrite) != bytesToWrite) {
    nandLogCacheBytes = 0;
    nandLogOk = false;
    nandLogFile.close();
    return false;
  }
  nandLogCacheBytes = 0;
  return true;
#else
  return false;
#endif
}

static bool appendNandRecord(const void *record, uint16_t size) {
#if HAS_LITTLEFS_QPINAND
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

static void finalizeLogFiles(NandCloseReason closeReason) {
  if (logsFinalized) return;

  if (sdLogFile) {
    sdLogFile.flush();
    sdLogFile.close();
  }
  sdLogOk = false;

#if HAS_LITTLEFS_QPINAND
  if (nandLogFile) {
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

static bool flightStateAllowsButtonReset() {
  return flightState == FS_IDLE || flightState == FS_PAD ||
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

static void updateButtonTask() {
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

static void ensureLogOpen() {
  if (logsFinalized) return;
  bool openedAny = false;
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
      header.sd_log_hz = 1000u / SD_LOG_UPDATE_MS;
      header.flight_tx_hz_x10 = 10000u / FLIGHT_TX_MS;
      header.nav_tx_hz_x10 = 10000u / NAV_TX_MS;
      header.status_tx_hz_x10 = 10000u / STATUS_TX_MS;
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
    if (!sdLogFile.println(line)) {
      sdLogOk = false;
      sdLogFile.close();
    }
  }
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

#if HAS_LITTLEFS_QPINAND
static void fillNandFlightRecord(NandFlightRecordV3 &rec, uint32_t nowMs,
                                 float relAlt, uint16_t healthFlags) {
  rec = {};
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
  rec.gps_rel_alt_cm = haveGpsBaseAlt ? (int32_t)lroundf(gpsRelAltM * 100.0f) : INT32_MIN;
  rec.baro_gps_delta_cm = isfinite(baroGpsDeltaM) ? (int32_t)lroundf(baroGpsDeltaM * 100.0f) : INT32_MIN;
  rec.gps_speed_cms = (int16_t)lroundf(gpsSpeedMps * 100.0f);
  rec.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
  rec.diag_flags = diagFlags;
  rec.mx_centiuT = (int16_t)lroundf(last_mx * 100.0f);
  rec.my_centiuT = (int16_t)lroundf(last_my * 100.0f);
  rec.mz_centiuT = (int16_t)lroundf(last_mz * 100.0f);
  rec.yaw_cdeg = (int16_t)lroundf(yaw * 5729.57795f);
  rec.state = (uint8_t)flightState;
  rec.gps_fix_type = gpsFixType;
  rec.gps_sats = gpsSats;
  rec.battery_pack = (uint8_t)batteryPack;
}
#endif

static void logNandBinary() {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile) {
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

static void logNandImuBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  if (flightState == FS_LANDED || flightState == FS_ABORT) return;
  ensureLogOpen();
  if (nandLogFile) {
    NandImuRecordV4 rec = {};
    rec.type = NAND_RECORD_IMU_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.ax_cms2 = (int16_t)lroundf(last_ax * 100.0f);
    rec.ay_cms2 = (int16_t)lroundf(last_ay * 100.0f);
    rec.az_cms2 = (int16_t)lroundf(last_az * 100.0f);
    rec.gx_cdeg = (int16_t)lroundf(last_gx * 100.0f);
    rec.gy_cdeg = (int16_t)lroundf(last_gy * 100.0f);
    rec.gz_cdeg = (int16_t)lroundf(last_gz * 100.0f);
    rec.roll_cdeg = (int16_t)lroundf(roll * 5729.57795f);
    rec.pitch_cdeg = (int16_t)lroundf(pitch * 5729.57795f);
    rec.yaw_cdeg = (int16_t)lroundf(yaw * 5729.57795f);
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

static void logNandBaroBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile) {
    NandBaroRecordV4 rec = {};
    rec.type = NAND_RECORD_BARO_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
    rec.rel_alt_cm = (int32_t)lroundf(currentBaroRelAltM() * 100.0f);
    rec.vel_cms = (int16_t)lroundf(velZ * 100.0f);
    rec.temp_centi_c = (int16_t)lroundf(filtTempC * 100.0f);
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

static void logNandGpsBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  if (lastGpsDataMs == 0 || (uint32_t)(nowMs - lastGpsLoggedMs) < NAV_TX_MS) return;
  ensureLogOpen();
  if (nandLogFile) {
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
    rec.speed_cms = (int16_t)lroundf(gpsSpeedMps * 100.0f);
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

static void logNandBatteryBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile) {
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

static void logNandEventBinary(uint32_t nowMs, uint8_t eventType, FlightState fromState,
                               FlightState toState, NandCloseReason closeReason) {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile) {
    NandEventRecordV4 rec = {};
    rec.type = NAND_RECORD_EVENT_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.event_type = eventType;
    rec.from_state = (uint8_t)fromState;
    rec.to_state = (uint8_t)toState;
    rec.close_reason = (uint8_t)closeReason;
    rec.flight_flags = flightFlags;
    rec.diag_flags = diagFlags;
    rec.rel_alt_cm = (int32_t)lroundf(currentBaroRelAltM() * 100.0f);
    rec.vel_cms = (int16_t)lroundf(velZ * 100.0f);
    rec.health_flags = buildHealthFlags();
    if (!appendNandRecord(&rec, sizeof(rec))) {
      nandLogOk = false;
      if (nandLogFile) nandLogFile.close();
    }
  }
#else
  (void)nowMs;
  (void)eventType;
  (void)fromState;
  (void)toState;
  (void)closeReason;
#endif
  logOk = sdLogOk || nandLogOk || logsFinalized;
}

static void logNandTelemetryBinary(uint32_t nowMs, uint8_t packetType, uint32_t packetSeq) {
#if HAS_LITTLEFS_QPINAND
  ensureLogOpen();
  if (nandLogFile) {
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

static void logNandAttitudeBinary(uint32_t nowMs) {
#if HAS_LITTLEFS_QPINAND
  if (!haveImuEstimate || flightState == FS_LANDED || flightState == FS_ABORT) return;
  ensureLogOpen();
  if (nandLogFile) {
    NandAttitudeRecordV4 rec = {};
    rec.type = NAND_RECORD_ATTITUDE_V4;
    rec.size = sizeof(rec);
    rec.sequence = nandRecordSequence++;
    rec.ms = nowMs;
    rec.qw_i16 = quantizeUnitI16(attitudeQw);
    rec.qx_i16 = quantizeUnitI16(attitudeQx);
    rec.qy_i16 = quantizeUnitI16(attitudeQy);
    rec.qz_i16 = quantizeUnitI16(attitudeQz);
    rec.roll_cdeg = (int16_t)lroundf(roll * 5729.57795f);
    rec.pitch_cdeg = (int16_t)lroundf(pitch * 5729.57795f);
    rec.yaw_cdeg = (int16_t)lroundf(yaw * 5729.57795f);
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
  if ((nowMs - lastLogFlushMs) >= LOG_FLUSH_MS) {
    lastLogFlushMs = nowMs;
    if (sdLogFile) sdLogFile.flush();
#if HAS_LITTLEFS_QPINAND
    if (nandLogFile) {
      flushNandLogCache();
      rewriteNandLogHeader(false, NAND_CLOSE_NONE);
      nandLogFile.flush();
    }
#endif
  }
  logOk = sdLogOk || nandLogOk || logsFinalized;
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
#endif

static bool verifySdFilesystem() {
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

static void trimAscii(char *s) {
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

static void loadRocketConfig() {
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
    Serial.print("Rocket name: ");
    Serial.println(rocketName);
  }
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

#if HAS_LITTLEFS_QPINAND
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

static void metadataFromHeaderV3(const NandLogHeaderV3 &header, NandLogMetadata &meta) {
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

static void writeNandExportMetadata(File &dst, const NandLogMetadata &meta) {
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
    eventDst.println("ms,seq,event_type,from_state,to_state,close_reason,flags,diag_flags,rel_alt_m,vel_mps,health");
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
        snprintf(line, sizeof(line), "%lu,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%u",
                 (unsigned long)rec.ms,
                 (unsigned int)rec.sequence,
                 (unsigned int)rec.event_type,
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
#endif

static void writeServiceResult(const ServiceRequest &req, bool exportDone, uint32_t exportCount,
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
    loadRocketConfig();
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
    lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_16G);
    lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_2000DPS);
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

  float relAlt = currentBaroRelAltM();

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
  Serial.print(" maxAlt=");
  Serial.print(maxAltM, 2);
  Serial.print(" apogeeAlt=");
  if (isnan(apogeeAltM)) {
    Serial.print("nan");
  } else {
    Serial.print(apogeeAltM, 2);
  }
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
  Serial.print(" mx=");
  Serial.print(last_mx, 2);
  Serial.print(" my=");
  Serial.print(last_my, 2);
  Serial.print(" mz=");
  Serial.print(last_mz, 2);
  Serial.print(" rollDeg=");
  Serial.print(roll * 57.2957795f, 2);
  Serial.print(" pitchDeg=");
  Serial.print(pitch * 57.2957795f, 2);
  Serial.print(" yawDeg=");
  Serial.print(yaw * 57.2957795f, 2);
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
  Serial.print(" gpsRelAlt=");
  Serial.print(gpsRelAltM, 2);
  Serial.print(" gpsBaseAlt=");
  if (haveGpsBaseAlt) {
    Serial.print(gpsBaseAltM, 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" baroGpsDelta=");
  if (isfinite(baroGpsDeltaM)) {
    Serial.print(baroGpsDeltaM, 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" baroGpsDiv=");
  Serial.print(baroGpsDiverged ? "1" : "0");
  Serial.print(" gpsSpd=");
  Serial.print(gpsSpeedMps, 2);
  Serial.print(" gpsChars=");
  Serial.print((unsigned long)gps.charsProcessed());
  Serial.print(" gpsPass=");
  Serial.print((unsigned long)gps.passedChecksum());
  Serial.print(" gpsFail=");
  Serial.print((unsigned long)gps.failedChecksum());
  Serial.print(" gpsLocValid=");
  Serial.print(gps.location.isValid() ? "1" : "0");
  Serial.print(" gpsAltValid=");
  Serial.print(gps.altitude.isValid() ? "1" : "0");
  Serial.print(" gpsDateValid=");
  Serial.print(gps.date.isValid() ? "1" : "0");
  Serial.print(" gpsTimeValid=");
  Serial.print(gps.time.isValid() ? "1" : "0");
  Serial.print(" gpsFixAgeMs=");
  if (gpsHasFix) {
    Serial.print("0");
  } else if (lastGpsFixMs != 0) {
    Serial.print((unsigned long)(millis() - lastGpsFixMs));
  } else {
    Serial.print("-1");
  }
  Serial.print(" battRawV=");
  Serial.print(rocketBattRawV, 2);
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
  Serial.print(" gpsFresh=");
  Serial.print(isGpsFresh() ? "1" : "0");
  Serial.print(" sd=");
  Serial.print(sdOk ? "1" : "0");
  Serial.print(" sdlog=");
  Serial.print(sdLogOk ? "1" : "0");
  Serial.print(" nlog=");
  Serial.print(nandLogOk ? "1" : "0");
  Serial.print(" log=");
  Serial.print(logOk ? "1" : "0");
  Serial.print(" nand=");
  Serial.print(nandOk ? "1" : "0");
  Serial.print(" imuFresh=");
  Serial.print(isImuFresh() ? "1" : "0");
  Serial.print(" baroFresh=");
  Serial.print(isBaroFresh() ? "1" : "0");
  uint16_t launchWaitS = 0;
  uint8_t launchStatus = currentLaunchStatus(launchWaitS);
  Serial.print(" launchStatus=");
  Serial.print((unsigned int)launchStatus);
  Serial.print(" launchReady=");
  Serial.print(launchStatus == LAUNCH_STATUS_READY ? "1" : "0");
  Serial.print(" launchWaitS=");
  Serial.print((unsigned int)launchWaitS);
  Serial.print(" launchArmMs=");
  Serial.print((unsigned long)launchArmedMs);
  Serial.print(" launchStillMs=");
  Serial.print((unsigned long)launchArmStillSinceMs);
  Serial.print(" logFinal=");
  Serial.println(logsFinalized ? "1" : "0");
}

static void updateGps() {
  bool sawBytes = false;
  while (GPS_SERIAL.available() > 0) {
    sawBytes = true;
    gps.encode((char)GPS_SERIAL.read());
  }
  if (sawBytes) {
    lastGpsDataMs = millis();
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
    lastGpsFixMs = lastFixTimeMs;
  }

  updateGpsAltitudeReference();
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
  last_mx = mag.magnetic.x;
  last_my = mag.magnetic.y;
  last_mz = mag.magnetic.z;

  float ax_g = last_ax / 9.80665f;
  float ay_g = last_ay / 9.80665f;
  float az_g = last_az / 9.80665f;
  const float accMagG = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  const bool accelCorrectionOk = accMagG >= IMU_ACCEL_CORRECT_MIN_G &&
                                 accMagG <= IMU_ACCEL_CORRECT_MAX_G;
  const float magMagUt = sqrtf(last_mx * last_mx + last_my * last_my + last_mz * last_mz);
  const bool magCorrectionOk = accelCorrectionOk &&
                               magMagUt >= IMU_MAG_CORRECT_MIN_UT &&
                               magMagUt <= IMU_MAG_CORRECT_MAX_UT;
  float rollAcc = atan2f(ay_g, az_g);
  float pitchAcc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));
  const bool initializingEstimate = !haveImuEstimate;

  if (initializingEstimate) {
    roll = rollAcc;
    pitch = pitchAcc;
    const float cpInit = cosf(pitch);
    const float spInit = sinf(pitch);
    const float crInit = cosf(roll);
    const float srInit = sinf(roll);
    const float magXInit = last_mx * cpInit + last_mz * spInit;
    const float magYInit = last_mx * srInit * spInit + last_my * crInit - last_mz * srInit * cpInit;
    yaw = atan2f(-magYInit, magXInit);
    setAttitudeQuatFromEuler(roll, pitch, yaw);
    updateEulerFromAttitudeQuat();
    haveImuEstimate = true;
  } else {
    integrateAttitudeQuatGyro(last_gx * 0.017453293f,
                              last_gy * 0.017453293f,
                              last_gz * 0.017453293f,
                              dt);
    updateEulerFromAttitudeQuat();

    float correctedRoll = roll;
    float correctedPitch = pitch;
    float correctedYaw = yaw;
    if (accelCorrectionOk) {
      correctedRoll = wrapPi(roll + (1.0f - IMU_GYRO_ALPHA) * angleDelta(roll, rollAcc));
      correctedPitch = wrapPi(pitch + (1.0f - IMU_GYRO_ALPHA) * angleDelta(pitch, pitchAcc));
    }

    if (magCorrectionOk) {
      const float cp = cosf(correctedPitch);
      const float sp = sinf(correctedPitch);
      const float cr = cosf(correctedRoll);
      const float sr = sinf(correctedRoll);
      const float magX = last_mx * cp + last_mz * sp;
      const float magY = last_mx * sr * sp + last_my * cr - last_mz * sr * cp;
      const float yawMag = atan2f(-magY, magX);
      correctedYaw = wrapPi(yaw + (1.0f - IMU_MAG_YAW_ALPHA) * angleDelta(yaw, yawMag));
    }

    setAttitudeQuatFromEuler(correctedRoll, correctedPitch, correctedYaw);
    updateEulerFromAttitudeQuat();
  }

  attitudeAccelCorrectionActive = accelCorrectionOk;
  attitudeMagCorrectionActive = magCorrectionOk;
  attitudeGyroOnly = !accelCorrectionOk && !magCorrectionOk;
  if (attitudeAccelCorrectionActive) diagFlags |= DIAG_ATT_ACCEL_CORR;
  else diagFlags &= (uint16_t)~DIAG_ATT_ACCEL_CORR;
  if (attitudeMagCorrectionActive) diagFlags |= DIAG_ATT_MAG_CORR;
  else diagFlags &= (uint16_t)~DIAG_ATT_MAG_CORR;
  if (attitudeGyroOnly) diagFlags |= DIAG_ATT_GYRO_ONLY;
  else diagFlags &= (uint16_t)~DIAG_ATT_GYRO_ONLY;

  lastImuSampleMs = millis();
#else
  (void)dt;
#endif
}

static void updateBaroAndState(float dt) {
  (void)dt;
  float tempC = 0.0f;
  float pressurePa = 0.0f;
  float altM = 0.0f;
  BaroReadStatus status = ms5607.update(tempC, pressurePa, altM);
  if (status == BARO_READ_WAITING) {
    return;
  }
  if (status == BARO_READ_ERROR) {
    baroOk = false;
    return;
  }

  baroOk = true;
  uint32_t nowMs = millis();
  lastBaroSampleMs = nowMs;
  if (!haveAlt) {
    haveAlt = true;
    filtAlt = altM;
    lastAltRaw = altM;
    velRefAltM = altM;
    velRefMs = nowMs;
    filtTempC = tempC;
    filtPressurePa = pressurePa;
    velZ = 0.0f;
    baseAltM = altM;
    padSettleStartMs = nowMs;
    clearLaunchArmGate();
    resetRelAltHistory(nowMs, 0.0f);
    flightState = FS_PAD;
    return;
  }

  filtAlt = 0.90f * filtAlt + 0.10f * altM;
  filtTempC = 0.90f * filtTempC + 0.10f * tempC;
  filtPressurePa = 0.90f * filtPressurePa + 0.10f * pressurePa;

  if (velRefMs == 0) {
    velRefAltM = altM;
    velRefMs = nowMs;
  }
  const uint32_t velWindowMs = nowMs - velRefMs;
  if (velWindowMs >= BARO_VEL_WINDOW_MS) {
    const float rawVel = (altM - velRefAltM) / (velWindowMs / 1000.0f);
    if (fabsf(rawVel) > BARO_MAX_RAW_VEL_MPS) {
      velRefAltM = altM;
      velRefMs = nowMs;
      return;
    }
    lastAltRaw = altM;
    velRefAltM = altM;
    velRefMs = nowMs;
    velZ = 0.80f * velZ + 0.20f * rawVel;
  }

  float relAlt = currentBaroRelAltM();
  updateGpsAltitudeReference();

  float accMagG = sqrtf(last_ax * last_ax + last_ay * last_ay + last_az * last_az) / 9.80665f;
  float gyroMagDps = sqrtf(last_gx * last_gx + last_gy * last_gy + last_gz * last_gz);
  float trendRelAltM = relAlt;
  const bool launchTrendOk = relAltAtLeastAgo(nowMs, LAUNCH_TREND_MS, trendRelAltM) &&
                             ((relAlt - trendRelAltM) >= LAUNCH_TREND_MIN_M);
  const bool padSettled = padSettleStartMs != 0 &&
                          ((uint32_t)(nowMs - padSettleStartMs) >= LAUNCH_PAD_SETTLE_MS);
  const bool launchStillCond = fabsf(accMagG - 1.0f) <= LAUNCH_PAD_STILL_ACCEL_ERR_G &&
                               gyroMagDps <= LAUNCH_PAD_STILL_GYRO_DPS;
  const bool powerOnInhibitDone = nowMs >= LAUNCH_POWERON_INHIBIT_MS;
  const bool launchKinematicsCond = launchTrendOk &&
                                    (relAlt > LAUNCH_REL_ALT_M) &&
                                    (velZ > LAUNCH_VEL_MPS) &&
                                    (accMagG >= LAUNCH_ACCEL_G);
  if (flightState == FS_IDLE || flightState == FS_PAD) {
    if (!powerOnInhibitDone || !padSettled) {
      clearLaunchArmGate();
    } else if (!launchArmed) {
      if (conditionHeld(launchStillCond, nowMs, launchArmStillSinceMs, LAUNCH_PAD_STILL_ARM_MS)) {
        setLaunchArmGate(nowMs, altM);
        relAlt = currentBaroRelAltM();
      }
    } else if (!launchStillCond && !launchKinematicsCond) {
      clearLaunchArmGate();
    }
  }
  const bool launchReady = powerOnInhibitDone && padSettled && launchArmed;
  const bool launchCond = launchReady && launchKinematicsCond;
  const bool coastCond = (velZ < COAST_VEL_MPS) && ((uint32_t)(nowMs - tLaunchMs) > COAST_MIN_AFTER_LAUNCH_MS);
  const bool apogeeCond = (velZ < APOGEE_VEL_MPS) && (relAlt > APOGEE_MIN_REL_ALT_M);
  const bool stillCond = fabsf(accMagG - 1.0f) <= LANDED_STILL_ACCEL_ERR_G &&
                         gyroMagDps <= LANDED_STILL_GYRO_DPS;
  const bool landedBaseCond = (fabsf(velZ) < LANDED_ABS_VEL_MPS) &&
                          (relAlt < LANDED_MAX_REL_ALT_M) &&
                          ((uint32_t)(nowMs - tLaunchMs) > LANDED_MIN_AFTER_LAUNCH_MS);
  const bool landedStillCond = conditionHeld(stillCond, nowMs, landedStillSinceMs, LANDED_STILL_CONFIRM_MS);
  const bool landedCond = landedBaseCond && landedStillCond;

  switch (flightState) {
    case FS_IDLE:
    case FS_PAD:
      if (conditionHeld(launchCond, nowMs, launchDetectSinceMs, LAUNCH_CONFIRM_MS)) {
        flightState = FS_ASCENT;
        flightFlags |= FLAG_LAUNCH;
        tLaunchMs = nowMs;
        maxAltM = relAlt;
        maxVelMps = max(0.0f, velZ);
        apogeeAltM = NAN;
        coastDetectSinceMs = 0;
        apogeeDetectSinceMs = 0;
        landedDetectSinceMs = 0;
        landedStillSinceMs = 0;
      }
      break;

    case FS_ASCENT:
      if (relAlt > maxAltM) maxAltM = relAlt;
      if (velZ > maxVelMps) maxVelMps = velZ;
      if (conditionHeld(coastCond, nowMs, coastDetectSinceMs, COAST_CONFIRM_MS)) {
        flightState = FS_COAST;
        apogeeDetectSinceMs = 0;
      }
      break;

    case FS_COAST:
      if (relAlt > maxAltM) maxAltM = relAlt;
      if (velZ > maxVelMps) maxVelMps = velZ;
      if (conditionHeld(apogeeCond, nowMs, apogeeDetectSinceMs, APOGEE_CONFIRM_MS)) {
        flightState = FS_DESCENT;
        flightFlags |= FLAG_APOGEE;
        tApogeeMs = nowMs;
        apogeeAltM = maxAltM;
        landedDetectSinceMs = 0;
        landedStillSinceMs = 0;
#if ENABLE_LOW_ENERGY_DIRECT_LANDED
      } else if (maxAltM <= LOW_ENERGY_DIRECT_LANDED_MAX_ALT_M &&
                 conditionHeld(landedCond, nowMs, landedDetectSinceMs, LANDED_CONFIRM_MS)) {
        // Optional bench-only path for low-energy throws.
        flightState = FS_LANDED;
        flightFlags |= FLAG_LANDED;
#endif
      }
      break;

    case FS_DESCENT:
      if (conditionHeld(landedCond, nowMs, landedDetectSinceMs, LANDED_CONFIRM_MS)) {
        flightState = FS_LANDED;
        flightFlags |= FLAG_LANDED;
      }
      break;

    case FS_LANDED:
    case FS_ABORT:
    default:
      break;
  }

  pushRelAltHistory(nowMs, relAlt);
}

static bool sendFlightTelemetry() {
  if (!loraOk || loraTxBusy || !haveAlt) return false;

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

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_FLIGHT_V7, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_FLIGHT_V7);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendNavTelemetry() {
  if (!loraOk || loraTxBusy) return false;

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

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_NAV_V7, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_NAV_V7);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendStatusTelemetry() {
  if (!loraOk || loraTxBusy) return false;

  StatusPacketV8 pkt = {};
  pkt.version = 8;
  pkt.state = (uint8_t)flightState;
  pkt.health_flags = buildHealthFlags();
  pkt.seq = statusSeq++;
  pkt.ms = millis();
  pkt.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
  pkt.gps_sats = gpsSats;
  uint16_t launchWaitS = 0;
  pkt.launch_status = currentLaunchStatus(launchWaitS);
  pkt.launch_wait_s = launchWaitS;
  pkt.last_rssi_dbm = 0;

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_STATUS_V8, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_STATUS_V8);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendIdentityTelemetry() {
  if (!loraOk || loraTxBusy) return false;

  IdentityPacketV1 pkt = {};
  pkt.version = 1;
  while (pkt.name_len < sizeof(pkt.name) && rocketName[pkt.name_len] != '\0') {
    pkt.name_len++;
  }
  pkt.seq = identitySeq++;
  pkt.ms = millis();
  strncpy(pkt.name, rocketName, sizeof(pkt.name));

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_IDENTITY_V1, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_IDENTITY_V1);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static void sampleBatteryTask() {
  rocketBattRawV = readBatteryVoltage();
  if (!isfinite(rocketBattV) || rocketBattV <= 0.0f) {
    rocketBattV = rocketBattRawV;
  } else {
    rocketBattV = (1.0f - BATT_FILTER_ALPHA) * rocketBattV + BATT_FILTER_ALPHA * rocketBattRawV;
  }
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
  static uint32_t lastIdentityTxMs = 0;
  const uint32_t nowMs = millis();
  const bool recoveryMode = (flightState == FS_LANDED);
  const uint32_t flightTxPeriodMs = recoveryMode ? RECOVERY_FLIGHT_TX_MS : FLIGHT_TX_MS;
  const uint32_t navTxPeriodMs = recoveryMode ? RECOVERY_NAV_TX_MS : NAV_TX_MS;
  const uint32_t statusTxPeriodMs = recoveryMode ? RECOVERY_STATUS_TX_MS : STATUS_TX_MS;

  if (!loraOk || loraTxBusy) return;

  if (lastIdentityTxMs == 0 || (uint32_t)(nowMs - lastIdentityTxMs) >= IDENTITY_TX_MS) {
    if (sendIdentityTelemetry()) lastIdentityTxMs = nowMs;
    return;
  }

  if ((uint32_t)(nowMs - lastStatusTxMs) >= statusTxPeriodMs) {
    if (sendStatusTelemetry()) lastStatusTxMs = nowMs;
    return;
  }

  if ((uint32_t)(nowMs - lastNavTxMs) >= navTxPeriodMs) {
    if (sendNavTelemetry()) lastNavTxMs = nowMs;
    return;
  }

  if ((uint32_t)(nowMs - lastFlightTxMs) >= flightTxPeriodMs) {
    if (sendFlightTelemetry()) lastFlightTxMs = nowMs;
    return;
  }
}

static void storageTask() {
  static uint32_t lastSdLogMs = 0;
  static uint32_t lastNandLogMs = 0;
  const uint32_t nowMs = millis();
  const bool recoveryMode = (flightState == FS_LANDED);
  const uint32_t sdLogPeriodMs = recoveryMode ? RECOVERY_SD_LOG_UPDATE_MS : SD_LOG_UPDATE_MS;
  const uint32_t nandLogPeriodMs = recoveryMode ? RECOVERY_NAND_LOG_UPDATE_MS : NAND_LOG_UPDATE_MS;

  if (flightState != lastFlightState) {
    logNandEventBinary(nowMs, 1, lastFlightState, flightState, NAND_CLOSE_NONE);
    if (flightState == FS_LANDED) {
      setFinderBeeper(true);
      finalizeLogFiles(NAND_CLOSE_LANDED);
    } else if (flightState == FS_ABORT) {
      finalizeLogFiles(NAND_CLOSE_ABORT);
    }
    lastFlightState = flightState;
  }

  if (taskDue(nowMs, lastNandLogMs, nandLogPeriodMs)) {
    logNandBinary();
  }

  if (taskDue(nowMs, lastSdLogMs, sdLogPeriodMs)) {
    logSdCsv();
  }

  flushLogsIfDue();
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
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);
#if BUTTON_ACTIVE_LOW
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#else
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
#endif

  Wire.begin();
  GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1);

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
  playStartupSound(startupHardwareOk());

  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("RocketV10 ready");
}

void loop() {
  static uint32_t lastImuMs = 0;
  static uint32_t lastBaroMs = 0;
  static uint32_t lastBattMs = 0;
  static uint32_t lastNandImuLogMs = 0;
  static uint32_t lastImuUs = micros();

  sampleGpsTask();

  uint32_t nowUs = micros();
  float dtImu = (nowUs - lastImuUs) * 1e-6f;
  if (dtImu <= 0.0f || dtImu > 0.05f) dtImu = 0.01f;

  uint32_t nowMs = millis();
  if (taskDue(nowMs, lastBattMs, BATT_UPDATE_MS)) {
    sampleBatteryTask();
    logNandBatteryBinary(nowMs);
  }

  if (taskDue(nowMs, lastImuMs, IMU_UPDATE_MS)) {
    lastImuUs = nowUs;
    sampleImuTask(dtImu);
    if (taskDue(nowMs, lastNandImuLogMs, NAND_IMU_LOG_UPDATE_MS)) {
      logNandImuBinary(nowMs);
      logNandAttitudeBinary(nowMs);
    }
  }

  if ((uint32_t)(nowMs - lastBaroMs) >= BARO_UPDATE_MS) {
    float dtBaro = (nowMs - lastBaroMs) / 1000.0f;
    if (dtBaro <= 0.0f || dtBaro > 0.25f) dtBaro = BARO_UPDATE_MS / 1000.0f;
    lastBaroMs = nowMs;
    sampleBaroTask(dtBaro);
    if (isBaroFresh()) logNandBaroBinary(nowMs);
  }

  logNandGpsBinary(nowMs);

  telemetryTask();
  storageTask();
  updateButtonTask();
  serialDebugTask();
  updateLedModeFromHealth();
  updateStatusLed();
  updateBuzzer();
}
