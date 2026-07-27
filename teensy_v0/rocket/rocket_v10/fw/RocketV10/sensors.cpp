#include "sensors.h"

#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "flight.h"
#include "imu_service.h"
#include "settings.h"
#include "state.h"

#define HAS_ADAFRUIT_LSM9DS1 1

enum BaroReadStatus {
  BARO_READ_WAITING = 0,
  BARO_READ_SAMPLE = 1,
  BARO_READ_ERROR = 2
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
    return isfinite(tempC) && isfinite(pressurePa) && isfinite(altM) &&
           tempC >= -60.0f && tempC <= 100.0f &&
           pressurePa >= 10000.0f && pressurePa <= 120000.0f &&
           altM >= -1000.0f && altM <= 20000.0f;
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

static bool isRecent(uint32_t lastMs, uint32_t staleMs) {
  return lastMs != 0 && (uint32_t)(millis() - lastMs) <= staleMs;
}

bool isBaroFresh() {
  return isRecent(lastBaroSampleMs, BARO_STALE_MS);
}

bool isImuFresh() {
  return isRecent(lastImuSampleMs, IMU_STALE_MS);
}

bool isGpsFresh() {
  return isRecent(lastGpsDataMs, GPS_STALE_MS);
}

static bool gpsAltitudeUsable() {
  return gpsHasFix && gpsFixType >= 3 && gps.altitude.isValid() &&
         gps.altitude.age() <= GPS_STALE_MS && isGpsFresh();
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

void setupBaro() {
  baroOk = ms5607.begin();
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println(baroOk ? "MS5607: OK" : "MS5607: missing");
}

static bool probeI2cAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool readI2cRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)address, 1) != 1 || Wire.available() < 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

void setupImu() {
  imuServiceInit();
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
    lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_16G,
                   Adafruit_LSM9DS1::LSM9DS1_ACCELDATARATE_238HZ);
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

static void updateGps() {
  if (rocketConsole.maintenanceActive()) return;
  bool sawBytes = false;
  static char serviceLine[32] = {};
  static uint8_t serviceLength = 0;
  while (GPS_SERIAL.available() > 0) {
    sawBytes = true;
    const char c = (char)GPS_SERIAL.read();
    gps.encode(c);
    if (c == '\r' || c == '\n') {
      if (serviceLength > 0) {
        serviceLine[serviceLength] = '\0';
        if (strcmp(serviceLine, "SERVICE UART CONFIRM") == 0) {
          serviceLength = 0;
          rocketSettingsActivateUartService();
          return;
        }
        serviceLength = 0;
      }
    } else if (c >= 32 && c <= 126) {
      if (serviceLength < sizeof(serviceLine) - 1) {
        serviceLine[serviceLength++] = c;
      } else {
        serviceLength = 0;
      }
    }
  }
  if (sawBytes) {
    lastGpsDataMs = millis();
  }

  bool locValid = gps.location.isValid() && gps.location.age() <= GPS_STALE_MS;
  bool altValid = gps.altitude.isValid() && gps.altitude.age() <= GPS_STALE_MS;
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

static bool updateImu() {
#if HAS_ADAFRUIT_LSM9DS1
  if (!imuOk) return false;

  // LSM9DS1 XG STATUS_REG: bit 0 = new accel, bit 1 = new gyro.
  // Accept only a coherent accel+gyro update instead of repeatedly logging
  // the last register values at the scheduler rate.
  uint8_t xgStatus = 0;
  if (readI2cRegister(0x6B, 0x17, xgStatus) &&
      (xgStatus & 0x03u) != 0x03u) {
    imuServiceRecordNotReady();
    return false;
  }
  // LIS3MDL STATUS_REG bit 3 indicates a fresh XYZ magnetometer sample.
  uint8_t magStatus = 0;
  const bool magFresh =
      readI2cRegister(0x1E, 0x27, magStatus) && (magStatus & 0x08u);

  sensors_event_t accel;
  sensors_event_t mag;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  const uint32_t sampleUs = micros();

  const float ax = accel.acceleration.x;
  const float ay = accel.acceleration.y;
  const float az = accel.acceleration.z;
  const float gx = gyro.gyro.x * 57.2957795f;
  const float gy = gyro.gyro.y * 57.2957795f;
  const float gz = gyro.gyro.z * 57.2957795f;
  const float mx = mag.magnetic.x;
  const float my = mag.magnetic.y;
  const float mz = mag.magnetic.z;
  const bool validSample =
      isfinite(ax) && isfinite(ay) && isfinite(az) &&
      isfinite(gx) && isfinite(gy) && isfinite(gz) &&
      isfinite(mx) && isfinite(my) && isfinite(mz) &&
      fabsf(ax) <= 200.0f && fabsf(ay) <= 200.0f && fabsf(az) <= 200.0f &&
      fabsf(gx) <= 2500.0f && fabsf(gy) <= 2500.0f && fabsf(gz) <= 2500.0f &&
      fabsf(mx) <= 2000.0f && fabsf(my) <= 2000.0f && fabsf(mz) <= 2000.0f;
  if (!validSample) {
    imuServiceRecordInvalidSample();
    return false;
  }

  last_ax = ax;
  last_ay = ay;
  last_az = az;
  last_gx = gx;
  last_gy = gy;
  last_gz = gz;
  last_mx = mx;
  last_my = my;
  last_mz = mz;

  static uint32_t previousSampleUs = 0;
  uint32_t dtUs = previousSampleUs == 0
                      ? IMU_UPDATE_MS * 1000u
                      : (uint32_t)(sampleUs - previousSampleUs);
  previousSampleUs = sampleUs;
  imuServiceProcessSample(ax, ay, az, gx, gy, gz, mx, my, mz,
                          sampleUs, dtUs, magFresh);
  lastImuSampleMs = millis();
  return true;
#else
  return false;
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
    } else {
      lastAltRaw = altM;
      velRefAltM = altM;
      velRefMs = nowMs;
      velZ = 0.80f * velZ + 0.20f * rawVel;
    }
  }

  updateGpsAltitudeReference();
}

void sampleGpsTask() {
  updateGps();
}

bool sampleImuTask() {
  return updateImu();
}

void sampleBaroTask(float dtBaro) {
  updateBaroAndState(dtBaro);
}
