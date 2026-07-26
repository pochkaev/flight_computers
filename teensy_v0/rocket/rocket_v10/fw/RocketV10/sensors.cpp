#include "sensors.h"

#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "flight.h"
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

static float wrapPi(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

static float angleDelta(float from, float to) {
  return wrapPi(to - from);
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

void setupImu() {
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

static void updateGps() {
  bool sawBytes = false;
  while (GPS_SERIAL.available() > 0) {
    sawBytes = true;
    gps.encode((char)GPS_SERIAL.read());
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

static void updateImu(float dt) {
#if HAS_ADAFRUIT_LSM9DS1
  if (!imuOk) return;

  sensors_event_t accel;
  sensors_event_t mag;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

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
  if (!validSample) return;

  last_ax = ax;
  last_ay = ay;
  last_az = az;
  last_gx = gx;
  last_gy = gy;
  last_gz = gz;
  last_mx = mx;
  last_my = my;
  last_mz = mz;

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

void sampleImuTask(float dtImu) {
  updateImu(dtImu);
}

void sampleBaroTask(float dtBaro) {
  updateBaroAndState(dtBaro);
}
