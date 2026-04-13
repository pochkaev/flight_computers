#include "config.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <math.h>

#if __has_include(<Adafruit_LSM9DS1.h>)
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#define HAS_ADAFRUIT_LSM9DS1 1
#else
#define HAS_ADAFRUIT_LSM9DS1 0
#endif

#if __has_include(<LittleFS.h>)
#include <LittleFS.h>
#define HAS_LITTLEFS_QPINAND 1
#else
#define HAS_LITTLEFS_QPINAND 0
#endif

enum FlightState : uint8_t {
  FS_IDLE = 0,
  FS_PAD,
  FS_ASCENT,
  FS_COAST,
  FS_DESCENT,
  FS_LANDED,
  FS_ABORT
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
    int64_t off = (C2 << 16) + ((C4 * dT) >> 7);
    int64_t sens = (C1 << 15) + ((C3 * dT) >> 8);

    int64_t t2 = 0;
    int64_t off2 = 0;
    int64_t sens2 = 0;
    if (temp < 2000) {
      int64_t dtLow = temp - 2000;
      t2 = (dT * dT) >> 31;
      off2 = (5 * dtLow * dtLow) >> 1;
      sens2 = (5 * dtLow * dtLow) >> 2;
      if (temp < -1500) {
        int64_t dtVeryLow = temp + 1500;
        off2 += 7 * dtVeryLow * dtVeryLow;
        sens2 += (11 * dtVeryLow * dtVeryLow) >> 1;
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
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#endif

#if HAS_LITTLEFS_QPINAND
LittleFS_QPINAND qspiNand;
#endif

volatile bool loraTxBusy = false;
bool loraOk = false;
bool sdOk = false;
bool nandOk = false;
bool logOk = false;
bool imuOk = false;
bool baroOk = false;

File sdLogFile;
uint32_t nextLogIndex = 1;

FlightState flightState = FS_IDLE;
uint16_t flightFlags = 0;

bool haveAlt = false;
bool haveGoodFix = false;
bool haveImuEstimate = false;

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

static float readBatteryVoltage() {
  int raw = analogRead(VBAT_PIN);
  float vPin = (float)raw * ADC_REF_V / ADC_MAX_COUNTS;
  return vPin * (VBAT_R1_OHMS + VBAT_R2_OHMS) / VBAT_R2_OHMS;
}

static uint16_t buildHealthFlags() {
  uint16_t flags = 0;
  if (baroOk) flags |= HEALTH_BARO_OK;
  if (imuOk)  flags |= HEALTH_IMU_OK;
  if (gpsHasFix) flags |= HEALTH_GPS_OK;
  if (sdOk)   flags |= HEALTH_SD_OK;
  if (nandOk) flags |= HEALTH_NAND_OK;
  if (logOk)  flags |= HEALTH_LOG_OK;
  if (rocketBattV >= BATT_WARN_V) flags |= HEALTH_BATT_OK;
  return flags;
}

void onLoraTxDone() {
  loraTxBusy = false;
}

static int extractLogIndex(const char *name) {
  const char *p = strstr(name, "flight");
  if (!p) return -1;
  p += 6;
  int idx = 0;
  while (*p >= '0' && *p <= '9') {
    idx = idx * 10 + (*p - '0');
    ++p;
  }
  return idx > 0 ? idx : -1;
}

static void scanLogFiles() {
  File root = SD.open("/");
  if (!root) return;
  uint32_t maxIdx = 0;
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      int idx = extractLogIndex(f.name());
      if (idx > (int)maxIdx) maxIdx = idx;
    }
    f.close();
  }
  root.close();
  nextLogIndex = maxIdx + 1;
}

static void ensureLogOpen() {
  if (!sdOk || sdLogFile) return;
  char filename[32];
  snprintf(filename, sizeof(filename), "flight%lu.csv", (unsigned long)nextLogIndex);
  sdLogFile = SD.open(filename, FILE_WRITE);
  if (!sdLogFile) {
    logOk = false;
    return;
  }
  logOk = true;
  nextLogIndex++;
  sdLogFile.println("ms,state,flags,alt_m,vel_mps,ax,ay,az,gx,gy,gz,roll,pitch,lat,lon,gps_alt,batt_v");
  sdLogFile.flush();
}

static void logCsv() {
  ensureLogOpen();
  if (!sdLogFile) return;
  char line[256];
  snprintf(line, sizeof(line),
           "%lu,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.7f,%.7f,%.2f,%.2f",
           (unsigned long)millis(),
           (unsigned int)flightState,
           (unsigned int)flightFlags,
           filtAlt,
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
           rocketBattV);
  sdLogFile.println(line);
  if ((millis() % 1000) < LOG_UPDATE_MS) {
    sdLogFile.flush();
  }
}

static void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    Serial.println("LoRa: not found");
    loraOk = false;
    return;
  }
  LoRa.setSPIFrequency(LORA_SPI_FREQ_HZ);
  LoRa.onTxDone(onLoraTxDone);
  loraOk = true;
}

static void setupStorage() {
  sdOk = SD.begin(BUILTIN_SDCARD);
  if (sdOk) {
    scanLogFiles();
  }
  logOk = false;

#if HAS_LITTLEFS_QPINAND
  nandOk = qspiNand.begin();
#else
  nandOk = false;
#endif
}

static void setupBaro() {
  baroOk = ms5607.begin();
  Serial.println(baroOk ? "MS5607: OK" : "MS5607: missing");
}

static void setupImu() {
#if HAS_ADAFRUIT_LSM9DS1
  if (lsm.begin()) {
    lsm.setupAccel(LSM9DS1_ACCELRANGE_4G);
    lsm.setupGyro(LSM9DS1_GYROSCALE_500DPS);
    lsm.setupMag(LSM9DS1_MAGGAIN_4GAUSS);
    imuOk = true;
  } else {
    imuOk = false;
  }
#else
  imuOk = false;
#endif
  Serial.println(imuOk ? "LSM9DS1: OK" : "LSM9DS1: unavailable");
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

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  GPS_SERIAL.begin(GPS_BAUD);

#if defined(__IMXRT1062__)
  analogReadResolution(12);
#endif
  pinMode(VBAT_PIN, INPUT);

  setupLoRa();
  setupStorage();
  setupBaro();
  setupImu();

  rocketBattV = readBatteryVoltage();

  Serial.println("RocketV8 ready");
}

void loop() {
  static uint32_t lastBaroMs = 0;
  static uint32_t lastFlightTxMs = 0;
  static uint32_t lastNavTxMs = 0;
  static uint32_t lastStatusTxMs = 0;
  static uint32_t lastLogMs = 0;
  static uint32_t lastPrintMs = 0;
  static uint32_t lastImuUs = micros();

  rocketBattV = readBatteryVoltage();
  updateGps();

  uint32_t nowUs = micros();
  float dtImu = (nowUs - lastImuUs) * 1e-6f;
  if (dtImu <= 0.0f || dtImu > 0.05f) dtImu = 0.01f;
  lastImuUs = nowUs;
  updateImu(dtImu);

  uint32_t nowMs = millis();
  if (nowMs - lastBaroMs >= BARO_UPDATE_MS) {
    float dtBaro = (nowMs - lastBaroMs) / 1000.0f;
    if (dtBaro <= 0.0f || dtBaro > 0.25f) dtBaro = 0.05f;
    lastBaroMs = nowMs;
    updateBaroAndState(dtBaro);
  }

  if (nowMs - lastFlightTxMs >= FLIGHT_TX_MS) {
    lastFlightTxMs = nowMs;
    sendFlightTelemetry();
  }
  if (nowMs - lastNavTxMs >= NAV_TX_MS) {
    lastNavTxMs = nowMs;
    sendNavTelemetry();
  }
  if (nowMs - lastStatusTxMs >= STATUS_TX_MS) {
    lastStatusTxMs = nowMs;
    sendStatusTelemetry();
  }
  if (nowMs - lastLogMs >= LOG_UPDATE_MS) {
    lastLogMs = nowMs;
    logCsv();
  }

  if (nowMs - lastPrintMs >= STATUS_PRINT_MS) {
    lastPrintMs = nowMs;
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
  }

  delay(1);
}
