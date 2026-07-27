#include "imu_service.h"

#include <EEPROM.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "imu_frame.h"
#include "state.h"

static const uint32_t IMU_CAL_MAGIC = 0x494D4331u;  // IMC1
static const uint16_t IMU_CAL_VERSION = 1;
static const int IMU_CAL_EEPROM_ADDRESS = 256;
static const uint32_t IMU_ALIGNMENT_MAGIC = 0x494D4131u;  // IMA1
static const uint16_t IMU_ALIGNMENT_VERSION = 2;
static const int IMU_ALIGNMENT_EEPROM_ADDRESS = 384;
static const float GRAVITY_MPS2 = 9.80665f;
static const uint32_t GYRO_CAL_REQUIRED_SAMPLES = 2000u;
static const uint32_t ACCEL_FACE_REQUIRED_SAMPLES = 200u;
static const uint32_t MAG_CAL_MIN_SAMPLES = 1000u;
static const float BENCH_MAX_ERROR_DEG = 8.0f;

enum CalibrationMode : uint8_t {
  CAL_MODE_NONE = 0,
  CAL_MODE_GYRO,
  CAL_MODE_ACCEL_FACE,
  CAL_MODE_MAG,
};

ImuCalibrationData imuCalibration = {};
ImuAlignmentData imuAlignment = {};
ImuRuntimeData imuRuntime = {};
AttitudeEstimatorState attitudeEstimator = {};

static CalibrationMode calibrationMode = CAL_MODE_NONE;
static uint32_t calibrationAcceptedSamples = 0;
static uint32_t calibrationRejectedSamples = 0;
static double calibrationSum[9] = {};
static double calibrationSumSquares[3] = {};
static float accelFaces[6][3] = {};
static uint8_t accelFaceMask = 0;
static float magMinimum[3] = {};
static float magMaximum[3] = {};
static bool streamEnabled = false;
static uint32_t lastStreamMs = 0;
static bool benchReferenceValid = false;
static Quaternionf benchReference = {1.0f, 0.0f, 0.0f, 0.0f};

static uint32_t checksumFor(const ImuCalibrationData &calibration) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&calibration);
  const size_t length = offsetof(ImuCalibrationData, checksum);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

static uint32_t checksumFor(const ImuAlignmentData &alignment) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&alignment);
  const size_t length = offsetof(ImuAlignmentData, checksum);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

static void loadCalibrationDefaults() {
  imuCalibration = {};
  imuCalibration.magic = IMU_CAL_MAGIC;
  imuCalibration.version = IMU_CAL_VERSION;
  imuCalibration.size = sizeof(imuCalibration);
  for (uint8_t axis = 0; axis < 3; ++axis) {
    imuCalibration.accelScale[axis] = 1.0f;
    imuCalibration.magScale[axis] = 1.0f;
  }
  imuCalibration.checksum = checksumFor(imuCalibration);
}

static void loadAlignmentDefaults() {
  imuAlignment = {};
  imuAlignment.magic = IMU_ALIGNMENT_MAGIC;
  imuAlignment.version = IMU_ALIGNMENT_VERSION;
  imuAlignment.size = sizeof(imuAlignment);
  imuAlignment.sensorToAirframe[0] = 1.0f;
  imuAlignment.checksum = checksumFor(imuAlignment);
}

static bool finiteArray(const float *values, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    if (!isfinite(values[i])) return false;
  }
  return true;
}

static bool validAlignment(const ImuAlignmentData &alignment) {
  if (alignment.magic != IMU_ALIGNMENT_MAGIC ||
      alignment.version != IMU_ALIGNMENT_VERSION ||
      alignment.size != sizeof(alignment) ||
      alignment.valid > 1u ||
      alignment.checksum != checksumFor(alignment) ||
      !finiteArray(alignment.sensorToAirframe, 4)) {
    return false;
  }
  const Quaternionf q = {
      alignment.sensorToAirframe[0],
      alignment.sensorToAirframe[1],
      alignment.sensorToAirframe[2],
      alignment.sensorToAirframe[3],
  };
  return fabsf(quaternionNorm(q) - 1.0f) <= 0.01f;
}

static bool validCalibration(const ImuCalibrationData &calibration) {
  if (calibration.magic != IMU_CAL_MAGIC ||
      calibration.version != IMU_CAL_VERSION ||
      calibration.size != sizeof(calibration) ||
      (calibration.validFlags &
       ~(IMU_CAL_GYRO_VALID | IMU_CAL_ACCEL_VALID | IMU_CAL_MAG_VALID)) != 0 ||
      calibration.checksum != checksumFor(calibration) ||
      !finiteArray(calibration.gyroBiasDps, 3) ||
      !finiteArray(calibration.accelBiasMps2, 3) ||
      !finiteArray(calibration.accelScale, 3) ||
      !finiteArray(calibration.magBiasUt, 3) ||
      !finiteArray(calibration.magScale, 3)) {
    return false;
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (fabsf(calibration.gyroBiasDps[axis]) > 50.0f ||
        fabsf(calibration.accelBiasMps2[axis]) > 5.0f ||
        calibration.accelScale[axis] < 0.5f ||
        calibration.accelScale[axis] > 1.5f ||
        fabsf(calibration.magBiasUt[axis]) > 500.0f ||
        calibration.magScale[axis] < 0.2f ||
        calibration.magScale[axis] > 5.0f) {
      return false;
    }
  }
  return true;
}

Quaternionf imuServiceAirframeQuaternion() {
  if (!imuAlignment.valid) return attitudeEstimator.q;
  const Quaternionf sensorToAirframe = {
      imuAlignment.sensorToAirframe[0],
      imuAlignment.sensorToAirframe[1],
      imuAlignment.sensorToAirframe[2],
      imuAlignment.sensorToAirframe[3],
  };
  // attitudeEstimator.q maps sensor-frame vectors into the world frame.
  // sensorToAirframe maps sensor-frame vectors into the rocket airframe.
  // Therefore its conjugate maps airframe vectors into the sensor frame.
  return quaternionNormalize(quaternionMultiply(
      attitudeEstimator.q, quaternionConjugate(sensorToAirframe)));
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

static void applyCalibration(float rawAx, float rawAy, float rawAz,
                             float rawGx, float rawGy, float rawGz,
                             float rawMx, float rawMy, float rawMz) {
  const float rawAccel[3] = {rawAx, rawAy, rawAz};
  const float rawGyro[3] = {rawGx, rawGy, rawGz};
  const float rawMag[3] = {rawMx, rawMy, rawMz};
  float calibratedAccel[3] = {};
  float calibratedGyro[3] = {};
  float calibratedMag[3] = {};
  for (uint8_t axis = 0; axis < 3; ++axis) {
    calibratedAccel[axis] =
        (rawAccel[axis] - imuCalibration.accelBiasMps2[axis]) *
        imuCalibration.accelScale[axis];
    calibratedGyro[axis] =
        rawGyro[axis] - imuCalibration.gyroBiasDps[axis];
    calibratedMag[axis] =
        (rawMag[axis] - imuCalibration.magBiasUt[axis]) *
        imuCalibration.magScale[axis];
  }
  imuRuntime.calibratedAxMps2 = calibratedAccel[0];
  imuRuntime.calibratedAyMps2 = calibratedAccel[1];
  imuRuntime.calibratedAzMps2 = calibratedAccel[2];
  imuRuntime.calibratedGxDps = calibratedGyro[0];
  imuRuntime.calibratedGyDps = calibratedGyro[1];
  imuRuntime.calibratedGzDps = calibratedGyro[2];
  imuRuntime.calibratedMxUt = calibratedMag[0];
  imuRuntime.calibratedMyUt = calibratedMag[1];
  imuRuntime.calibratedMzUt = calibratedMag[2];
}

static void clearCalibrationAccumulator() {
  calibrationAcceptedSamples = 0;
  calibrationRejectedSamples = 0;
  memset(calibrationSum, 0, sizeof(calibrationSum));
  memset(calibrationSumSquares, 0, sizeof(calibrationSumSquares));
}

static void finishGyroCalibration() {
  if (calibrationAcceptedSamples < GYRO_CAL_REQUIRED_SAMPLES) return;
  float standardDeviation[3] = {};
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const double mean = calibrationSum[axis] / calibrationAcceptedSamples;
    const double variance =
        calibrationSumSquares[axis] / calibrationAcceptedSamples - mean * mean;
    standardDeviation[axis] = sqrtf((float)fmax(0.0, variance));
    imuCalibration.gyroBiasDps[axis] = (float)mean;
  }
  calibrationMode = CAL_MODE_NONE;
  if (standardDeviation[0] > 0.5f ||
      standardDeviation[1] > 0.5f ||
      standardDeviation[2] > 0.5f) {
    Serial.print("ERR IMU CAL GYRO unstable std_dps=");
    Serial.print(standardDeviation[0], 3); Serial.print(",");
    Serial.print(standardDeviation[1], 3); Serial.print(",");
    Serial.println(standardDeviation[2], 3);
    return;
  }
  imuCalibration.validFlags |= IMU_CAL_GYRO_VALID;
  imuCalibration.checksum = checksumFor(imuCalibration);
  attitudeEstimatorSetGyroBias(attitudeEstimator, 0.0f, 0.0f, 0.0f);
  Serial.print("OK IMU CAL GYRO bias_dps=");
  Serial.print(imuCalibration.gyroBiasDps[0], 4); Serial.print(",");
  Serial.print(imuCalibration.gyroBiasDps[1], 4); Serial.print(",");
  Serial.print(imuCalibration.gyroBiasDps[2], 4);
  Serial.println(" use IMU CAL SAVE");
}

static void finishAccelFace() {
  if (calibrationAcceptedSamples < ACCEL_FACE_REQUIRED_SAMPLES) return;
  float mean[3] = {};
  for (uint8_t axis = 0; axis < 3; ++axis) {
    mean[axis] = (float)(calibrationSum[axis] / calibrationAcceptedSamples);
  }
  uint8_t dominantAxis = 0;
  if (fabsf(mean[1]) > fabsf(mean[dominantAxis])) dominantAxis = 1;
  if (fabsf(mean[2]) > fabsf(mean[dominantAxis])) dominantAxis = 2;
  const uint8_t slot = dominantAxis * 2u + (mean[dominantAxis] >= 0.0f ? 1u : 0u);
  const uint8_t otherA = (dominantAxis + 1u) % 3u;
  const uint8_t otherB = (dominantAxis + 2u) % 3u;
  calibrationMode = CAL_MODE_NONE;
  if (fabsf(mean[dominantAxis]) < 0.75f * GRAVITY_MPS2 ||
      fabsf(mean[otherA]) > 0.35f * GRAVITY_MPS2 ||
      fabsf(mean[otherB]) > 0.35f * GRAVITY_MPS2) {
    Serial.println("ERR IMU CAL ACCEL face not level/still");
    return;
  }
  memcpy(accelFaces[slot], mean, sizeof(mean));
  accelFaceMask |= (uint8_t)(1u << slot);
  Serial.print("OK IMU CAL ACCEL FACE axis=");
  Serial.print(dominantAxis == 0 ? "X" : dominantAxis == 1 ? "Y" : "Z");
  Serial.print(" sign=");
  Serial.print(mean[dominantAxis] >= 0.0f ? "+" : "-");
  Serial.print(" faces=");
  Serial.print(__builtin_popcount((unsigned int)accelFaceMask));
  Serial.println("/6");

  if (accelFaceMask != 0x3Fu) return;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float negative = accelFaces[axis * 2u][axis];
    const float positive = accelFaces[axis * 2u + 1u][axis];
    const float span = positive - negative;
    if (!isfinite(span) || span < 1.5f * GRAVITY_MPS2) {
      Serial.println("ERR IMU CAL ACCEL invalid six-face span");
      return;
    }
    imuCalibration.accelBiasMps2[axis] = 0.5f * (positive + negative);
    imuCalibration.accelScale[axis] = (2.0f * GRAVITY_MPS2) / span;
  }
  imuCalibration.validFlags |= IMU_CAL_ACCEL_VALID;
  imuCalibration.checksum = checksumFor(imuCalibration);
  Serial.println("OK IMU CAL ACCEL COMPLETE use IMU CAL SAVE");
}

static void observeCalibrationSample(float ax, float ay, float az,
                                     float gx, float gy, float gz,
                                     float mx, float my, float mz,
                                     bool magFresh) {
  if (calibrationMode == CAL_MODE_NONE) return;
  if (!configurationUnlocked()) {
    calibrationMode = CAL_MODE_NONE;
    Serial.println("ERR IMU CAL cancelled: SAFE/preflight lost");
    return;
  }

  const float accelMagnitude =
      sqrtf(ax * ax + ay * ay + az * az) / GRAVITY_MPS2;
  const float gyroMagnitude = sqrtf(gx * gx + gy * gy + gz * gz);
  if (calibrationMode == CAL_MODE_GYRO ||
      calibrationMode == CAL_MODE_ACCEL_FACE) {
    const bool still =
        fabsf(accelMagnitude - 1.0f) <= 0.08f && gyroMagnitude <= 3.0f;
    if (!still) {
      calibrationRejectedSamples++;
      return;
    }
  }

  const float values[9] = {gx, gy, gz, ax, ay, az, mx, my, mz};
  if (calibrationMode == CAL_MODE_GYRO) {
    for (uint8_t axis = 0; axis < 3; ++axis) {
      calibrationSum[axis] += values[axis];
      calibrationSumSquares[axis] += values[axis] * values[axis];
    }
    calibrationAcceptedSamples++;
    finishGyroCalibration();
  } else if (calibrationMode == CAL_MODE_ACCEL_FACE) {
    for (uint8_t axis = 0; axis < 3; ++axis) {
      calibrationSum[axis] += values[axis + 3u];
    }
    calibrationAcceptedSamples++;
    finishAccelFace();
  } else if (calibrationMode == CAL_MODE_MAG) {
    if (!magFresh) return;
    const float mag[3] = {mx, my, mz};
    for (uint8_t axis = 0; axis < 3; ++axis) {
      if (mag[axis] < magMinimum[axis]) magMinimum[axis] = mag[axis];
      if (mag[axis] > magMaximum[axis]) magMaximum[axis] = mag[axis];
    }
    calibrationAcceptedSamples++;
  }
}

void imuServiceInit() {
  ImuCalibrationData stored = {};
  EEPROM.get(IMU_CAL_EEPROM_ADDRESS, stored);
  if (validCalibration(stored)) {
    imuCalibration = stored;
  } else {
    loadCalibrationDefaults();
  }
  ImuAlignmentData storedAlignment = {};
  EEPROM.get(IMU_ALIGNMENT_EEPROM_ADDRESS, storedAlignment);
  if (validAlignment(storedAlignment)) {
    imuAlignment = storedAlignment;
  } else {
    loadAlignmentDefaults();
  }
  imuRuntime = {};
  attitudeEstimatorReset(attitudeEstimator);
  // The calibration is subtracted before samples enter the estimator.
  attitudeEstimatorSetGyroBias(attitudeEstimator, 0.0f, 0.0f, 0.0f);
}

void imuServiceRecordInvalidSample() {
  imuRuntime.invalidSampleCount++;
}

void imuServiceRecordNotReady() {
  imuRuntime.notReadyPollCount++;
}

void imuServiceProcessSample(float axMps2, float ayMps2, float azMps2,
                             float gxDps, float gyDps, float gzDps,
                             float mxUt, float myUt, float mzUt,
                             uint32_t sampleUs, uint32_t dtUs,
                             bool magFresh) {
  if (imuRuntime.firstSampleUs == 0) imuRuntime.firstSampleUs = sampleUs;
  imuRuntime.sampleUs = sampleUs;
  imuRuntime.sampleCount++;
  if (magFresh) {
    const uint32_t magDtUs = imuRuntime.lastMagSampleUs == 0
                                 ? 0u
                                 : (uint32_t)(sampleUs - imuRuntime.lastMagSampleUs);
    imuRuntime.magDtUs = magDtUs > 65535u ? 65535u : (uint16_t)magDtUs;
    imuRuntime.lastMagSampleUs = sampleUs;
    imuRuntime.magSampleCount++;
  }
  imuRuntime.dtUs = dtUs > 65535u ? 65535u : (uint16_t)dtUs;
  if (dtUs > imuRuntime.maxDtUs) imuRuntime.maxDtUs = dtUs;
  const bool sampleGap = dtUs > (IMU_UPDATE_MS * 1000u * 2u);
  if (sampleGap) {
    const uint32_t expectedUs = IMU_UPDATE_MS * 1000u;
    const uint32_t missed = dtUs / expectedUs;
    if (missed > 1u) imuRuntime.missedSampleCount += missed - 1u;
  }
  const float accelLimitMps2 = IMU_ACCEL_RANGE_G * GRAVITY_MPS2 * 0.97f;
  const float gyroLimitDps = IMU_GYRO_RANGE_DPS * 0.97f;
  const bool accelSaturated =
      fabsf(axMps2) >= accelLimitMps2 ||
      fabsf(ayMps2) >= accelLimitMps2 ||
      fabsf(azMps2) >= accelLimitMps2;
  const bool gyroSaturated =
      fabsf(gxDps) >= gyroLimitDps ||
      fabsf(gyDps) >= gyroLimitDps ||
      fabsf(gzDps) >= gyroLimitDps;
  if (accelSaturated || gyroSaturated) imuRuntime.saturationSampleCount++;

  observeCalibrationSample(axMps2, ayMps2, azMps2,
                           gxDps, gyDps, gzDps,
                           mxUt, myUt, mzUt, magFresh);
  applyCalibration(axMps2, ayMps2, azMps2,
                   gxDps, gyDps, gzDps,
                   mxUt, myUt, mzUt);

  const bool boostPhase = flightState == FS_ASCENT;
  const bool preflight = flightState == FS_IDLE || flightState == FS_PAD;
  const bool recoveryGround =
      flightState == FS_POST_FLIGHT_GROUND || flightState == FS_LANDED;
  const bool physicalSafe =
      digitalRead(ARM_SWITCH_PIN) == ARM_SWITCH_SAFE_LEVEL;
  float bodyMxUt = 0.0f;
  float bodyMyUt = 0.0f;
  float bodyMzUt = 0.0f;
  lsm9ds1MagToAgFrame(
      imuRuntime.calibratedMxUt, imuRuntime.calibratedMyUt,
      imuRuntime.calibratedMzUt, bodyMxUt, bodyMyUt, bodyMzUt);
  AttitudeEstimatorInput input = {
      imuRuntime.calibratedAxMps2 / GRAVITY_MPS2,
      imuRuntime.calibratedAyMps2 / GRAVITY_MPS2,
      imuRuntime.calibratedAzMps2 / GRAVITY_MPS2,
      imuRuntime.calibratedGxDps,
      imuRuntime.calibratedGyDps,
      imuRuntime.calibratedGzDps,
      bodyMxUt,
      bodyMyUt,
      bodyMzUt,
      dtUs * 1.0e-6f,
      !boostPhase,
      (preflight || recoveryGround) && magFresh,
      preflight && physicalSafe,
      accelSaturated,
      gyroSaturated,
  };
  const AttitudeEstimatorConfig estimatorConfig = {
      IMU_ACCEL_CORRECT_MIN_G,
      IMU_ACCEL_CORRECT_MAX_G,
      IMU_ACCEL_INNOVATION_MAX_DEG,
      IMU_MAG_CORRECT_MIN_UT,
      IMU_MAG_CORRECT_MAX_UT,
      IMU_MAG_INNOVATION_MAX_DEG,
      IMU_ACCEL_CORRECTION_RATE,
      IMU_MAG_CORRECTION_RATE,
      IMU_STATIONARY_ACCEL_TOLERANCE_G,
      IMU_STATIONARY_GYRO_MAX_DPS,
      IMU_GYRO_BIAS_LEARNING_RATE,
  };
  attitudeEstimatorUpdate(attitudeEstimator, estimatorConfig, input);

  const Quaternionf airframeQ = imuServiceAirframeQuaternion();
  attitudeQw = airframeQ.w;
  attitudeQx = airframeQ.x;
  attitudeQy = airframeQ.y;
  attitudeQz = airframeQ.z;
  quaternionToEuler(airframeQ, roll, pitch, yaw);
  haveImuEstimate = attitudeEstimator.initialized;
  attitudeAccelCorrectionActive =
      attitudeEstimator.accelCorrectionActive;
  attitudeMagCorrectionActive =
      attitudeEstimator.magCorrectionActive;
  attitudeGyroOnly = attitudeEstimator.gyroOnly;
  imuRuntime.accelInnovationDeg =
      attitudeEstimator.accelInnovationDeg;
  imuRuntime.magInnovationDeg =
      attitudeEstimator.magInnovationDeg;
  imuRuntime.confidence = attitudeEstimator.confidence;

  uint16_t flags = 0;
  if (attitudeEstimator.accelCorrectionActive)
    flags |= IMU_QUALITY_ACCEL_CORRECTION;
  if (attitudeEstimator.magCorrectionActive)
    flags |= IMU_QUALITY_MAG_CORRECTION;
  if (attitudeEstimator.gyroOnly) flags |= IMU_QUALITY_GYRO_ONLY;
  if (accelSaturated) flags |= IMU_QUALITY_ACCEL_SATURATED;
  if (gyroSaturated) flags |= IMU_QUALITY_GYRO_SATURATED;
  if (sampleGap) flags |= IMU_QUALITY_SAMPLE_GAP;
  if (attitudeEstimator.accelRejected)
    flags |= IMU_QUALITY_ACCEL_REJECTED;
  if (attitudeEstimator.magRejected)
    flags |= IMU_QUALITY_MAG_REJECTED;
  if (attitudeEstimator.stationary) flags |= IMU_QUALITY_STATIONARY;
  if (imuCalibration.validFlags & IMU_CAL_GYRO_VALID)
    flags |= IMU_QUALITY_GYRO_CALIBRATED;
  if (imuCalibration.validFlags & IMU_CAL_ACCEL_VALID)
    flags |= IMU_QUALITY_ACCEL_CALIBRATED;
  if (imuCalibration.validFlags & IMU_CAL_MAG_VALID)
    flags |= IMU_QUALITY_MAG_CALIBRATED;
  if (magFresh) flags |= IMU_QUALITY_MAG_FRESH;
  if (imuAlignment.valid) flags |= IMU_QUALITY_AIRFRAME_ALIGNED;
  imuRuntime.qualityFlags = flags;

  if (attitudeAccelCorrectionActive) diagFlags |= DIAG_ATT_ACCEL_CORR;
  else diagFlags &= (uint16_t)~DIAG_ATT_ACCEL_CORR;
  if (attitudeMagCorrectionActive) diagFlags |= DIAG_ATT_MAG_CORR;
  else diagFlags &= (uint16_t)~DIAG_ATT_MAG_CORR;
  if (attitudeGyroOnly) diagFlags |= DIAG_ATT_GYRO_ONLY;
  else diagFlags &= (uint16_t)~DIAG_ATT_GYRO_ONLY;
}

void imuServicePrintStatus(Stream &output) {
  output.println("IMU STATUS");
  output.print("CAL_FLAGS "); output.println(imuCalibration.validFlags);
  output.print("CAL_GYRO_BIAS_DPS ");
  output.print(imuCalibration.gyroBiasDps[0], 5); output.print(",");
  output.print(imuCalibration.gyroBiasDps[1], 5); output.print(",");
  output.println(imuCalibration.gyroBiasDps[2], 5);
  output.print("EST_GYRO_RESIDUAL_BIAS_DPS ");
  output.print(attitudeEstimator.gyroBiasDps[0], 5); output.print(",");
  output.print(attitudeEstimator.gyroBiasDps[1], 5); output.print(",");
  output.println(attitudeEstimator.gyroBiasDps[2], 5);
  output.print("CAL_ACCEL_BIAS_MPS2 ");
  output.print(imuCalibration.accelBiasMps2[0], 5); output.print(",");
  output.print(imuCalibration.accelBiasMps2[1], 5); output.print(",");
  output.println(imuCalibration.accelBiasMps2[2], 5);
  output.print("CAL_ACCEL_SCALE ");
  output.print(imuCalibration.accelScale[0], 6); output.print(",");
  output.print(imuCalibration.accelScale[1], 6); output.print(",");
  output.println(imuCalibration.accelScale[2], 6);
  output.print("CAL_MAG_BIAS_UT ");
  output.print(imuCalibration.magBiasUt[0], 4); output.print(",");
  output.print(imuCalibration.magBiasUt[1], 4); output.print(",");
  output.println(imuCalibration.magBiasUt[2], 4);
  output.print("CAL_MAG_SCALE ");
  output.print(imuCalibration.magScale[0], 6); output.print(",");
  output.print(imuCalibration.magScale[1], 6); output.print(",");
  output.println(imuCalibration.magScale[2], 6);
  output.print("MAG_AG_FRAME_UT ");
  float bodyMxUt = 0.0f;
  float bodyMyUt = 0.0f;
  float bodyMzUt = 0.0f;
  lsm9ds1MagToAgFrame(
      imuRuntime.calibratedMxUt, imuRuntime.calibratedMyUt,
      imuRuntime.calibratedMzUt, bodyMxUt, bodyMyUt, bodyMzUt);
  output.print(bodyMxUt, 4); output.print(",");
  output.print(bodyMyUt, 4); output.print(",");
  output.println(bodyMzUt, 4);
  output.print("ALIGN_VALID "); output.println(imuAlignment.valid);
  output.print("ALIGN_SENSOR_TO_AIRFRAME ");
  output.print(imuAlignment.sensorToAirframe[0], 7); output.print(",");
  output.print(imuAlignment.sensorToAirframe[1], 7); output.print(",");
  output.print(imuAlignment.sensorToAirframe[2], 7); output.print(",");
  output.println(imuAlignment.sensorToAirframe[3], 7);
  output.print("SAMPLES "); output.println(imuRuntime.sampleCount);
  const float elapsedS =
      (uint32_t)(imuRuntime.sampleUs - imuRuntime.firstSampleUs) * 1.0e-6f;
  output.print("SAMPLE_RATE_HZ ");
  output.println(elapsedS > 0.0f
                     ? (imuRuntime.sampleCount - 1u) / elapsedS
                     : 0.0f,
                 2);
  output.print("MAG_RATE_HZ ");
  output.println(elapsedS > 0.0f
                     ? imuRuntime.magSampleCount / elapsedS
                     : 0.0f,
                 2);
  output.print("INVALID_SAMPLES "); output.println(imuRuntime.invalidSampleCount);
  output.print("NOT_READY_POLLS "); output.println(imuRuntime.notReadyPollCount);
  output.print("MAG_SAMPLES "); output.println(imuRuntime.magSampleCount);
  output.print("MISSED_SAMPLES "); output.println(imuRuntime.missedSampleCount);
  output.print("SATURATION_SAMPLES "); output.println(imuRuntime.saturationSampleCount);
  output.print("LAST_DT_US "); output.println(imuRuntime.dtUs);
  output.print("MAX_DT_US "); output.println(imuRuntime.maxDtUs);
  output.print("QUALITY_FLAGS "); output.println(imuRuntime.qualityFlags);
  output.print("CONFIDENCE "); output.println(imuRuntime.confidence, 3);
  output.print("ACCEL_INNOVATION_DEG ");
  output.println(imuRuntime.accelInnovationDeg, 2);
  output.print("MAG_INNOVATION_DEG ");
  output.println(imuRuntime.magInnovationDeg, 2);
  output.print("MAG_HEALTHY ");
  output.println(attitudeEstimator.magHealthy ? 1 : 0);
  output.print("Q_SENSOR ");
  output.print(attitudeEstimator.q.w, 7); output.print(",");
  output.print(attitudeEstimator.q.x, 7); output.print(",");
  output.print(attitudeEstimator.q.y, 7); output.print(",");
  output.println(attitudeEstimator.q.z, 7);
  const Quaternionf airframeQ = imuServiceAirframeQuaternion();
  output.print("Q ");
  output.print(airframeQ.w, 7); output.print(",");
  output.print(airframeQ.x, 7); output.print(",");
  output.print(airframeQ.y, 7); output.print(",");
  output.println(airframeQ.z, 7);
  output.print("Q_NORM "); output.println(quaternionNorm(airframeQ), 7);
  output.print("CAL_MODE "); output.println((uint8_t)calibrationMode);
  output.print("CAL_ACCEPTED "); output.println(calibrationAcceptedSamples);
  output.print("CAL_REJECTED "); output.println(calibrationRejectedSamples);
  output.print("ACCEL_FACES "); output.print(__builtin_popcount((unsigned int)accelFaceMask));
  output.println("/6");
  output.print("STREAM "); output.println(streamEnabled ? 1 : 0);
  output.print("BENCH_REFERENCE "); output.println(benchReferenceValid ? 1 : 0);
}

static void startGyroCalibration() {
  clearCalibrationAccumulator();
  calibrationMode = CAL_MODE_GYRO;
  Serial.println("IMU CAL GYRO BEGIN keep rocket completely still for 10 seconds");
}

static void startAccelCalibration() {
  calibrationMode = CAL_MODE_NONE;
  accelFaceMask = 0;
  memset(accelFaces, 0, sizeof(accelFaces));
  Serial.println("IMU CAL ACCEL BEGIN use IMU CAL ACCEL ADD in each of six faces");
}

static void addAccelFace() {
  clearCalibrationAccumulator();
  calibrationMode = CAL_MODE_ACCEL_FACE;
  Serial.println("IMU CAL ACCEL FACE BEGIN hold still for 1 second");
}

static void startMagCalibration() {
  clearCalibrationAccumulator();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    magMinimum[axis] = 1.0e9f;
    magMaximum[axis] = -1.0e9f;
  }
  calibrationMode = CAL_MODE_MAG;
  Serial.println("IMU CAL MAG BEGIN rotate assembled rocket through all orientations");
}

static void stopMagCalibration() {
  if (calibrationMode != CAL_MODE_MAG) {
    Serial.println("ERR IMU CAL MAG not running");
    return;
  }
  calibrationMode = CAL_MODE_NONE;
  if (calibrationAcceptedSamples < MAG_CAL_MIN_SAMPLES) {
    Serial.println("ERR IMU CAL MAG too few samples");
    return;
  }
  float halfRange[3] = {};
  for (uint8_t axis = 0; axis < 3; ++axis) {
    halfRange[axis] = 0.5f * (magMaximum[axis] - magMinimum[axis]);
    if (!isfinite(halfRange[axis]) || halfRange[axis] < 10.0f) {
      Serial.println("ERR IMU CAL MAG insufficient rotation coverage");
      return;
    }
  }
  const float averageRadius =
      (halfRange[0] + halfRange[1] + halfRange[2]) / 3.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    imuCalibration.magBiasUt[axis] =
        0.5f * (magMaximum[axis] + magMinimum[axis]);
    imuCalibration.magScale[axis] = averageRadius / halfRange[axis];
  }
  imuCalibration.validFlags |= IMU_CAL_MAG_VALID;
  imuCalibration.checksum = checksumFor(imuCalibration);
  Serial.println("OK IMU CAL MAG COMPLETE use IMU CAL SAVE");
}

static bool parseFloatValue(const char *text, float &value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return end && *end == '\0' && isfinite(value);
}

static void benchCheck(const char *axisText, const char *degreesText) {
  if (!benchReferenceValid || !haveImuEstimate) {
    Serial.println("ERR IMU BENCH use IMU BENCH ZERO first");
    return;
  }
  if (!axisText || !degreesText || axisText[1] != '\0') {
    Serial.println("ERR IMU BENCH CHECK syntax");
    return;
  }
  uint8_t axis = 3;
  if (axisText[0] == 'X') axis = 0;
  else if (axisText[0] == 'Y') axis = 1;
  else if (axisText[0] == 'Z') axis = 2;
  float degrees = 0.0f;
  if (axis > 2 || !parseFloatValue(degreesText, degrees) ||
      degrees < -360.0f || degrees > 360.0f) {
    Serial.println("ERR IMU BENCH CHECK axis/degrees");
    return;
  }
  const Quaternionf current = imuServiceAirframeQuaternion();
  const Quaternionf expected = quaternionMultiply(
      benchReference, quaternionFromAxisAngle(axis, degrees));
  const float angularError =
      quaternionAngularErrorDeg(current, expected);
  const float normError =
      fabsf(quaternionNorm(current) - 1.0f);
  const bool passed =
      angularError <= BENCH_MAX_ERROR_DEG &&
      normError <= 0.005f &&
      imuRuntime.confidence >= 0.60f &&
      !(imuRuntime.qualityFlags &
        (IMU_QUALITY_ACCEL_SATURATED | IMU_QUALITY_GYRO_SATURATED));
  Serial.print(passed ? "PASS IMU BENCH" : "FAIL IMU BENCH");
  Serial.print(" AXIS "); Serial.print(axisText);
  Serial.print(" EXPECT_DEG "); Serial.print(degrees, 2);
  Serial.print(" ERROR_DEG "); Serial.print(angularError, 3);
  Serial.print(" Q_NORM_ERROR "); Serial.print(normError, 6);
  Serial.print(" CONFIDENCE "); Serial.print(imuRuntime.confidence, 3);
  Serial.print(" FLAGS "); Serial.println(imuRuntime.qualityFlags);
}

bool imuServiceHandleCommand(char *subcommand, char **save) {
  if (!subcommand) {
    Serial.println("ERR IMU syntax");
    return true;
  }
  if (!requireUnlocked()) return true;

  if (strcmp(subcommand, "STATUS") == 0) {
    if (strtok_r(nullptr, " \t", save)) {
      Serial.println("ERR IMU STATUS syntax");
    } else {
      imuServicePrintStatus(Serial);
    }
    return true;
  }

  if (strcmp(subcommand, "STREAM") == 0) {
    char *operation = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    if (!operation || extra ||
        (strcmp(operation, "START") != 0 &&
         strcmp(operation, "STOP") != 0)) {
      Serial.println("ERR IMU STREAM use START|STOP");
    } else {
      streamEnabled = strcmp(operation, "START") == 0;
      Serial.println(streamEnabled ? "OK IMU STREAM START" :
                                     "OK IMU STREAM STOP");
    }
    return true;
  }

  if (strcmp(subcommand, "CAL") == 0) {
    char *target = strtok_r(nullptr, " \t", save);
    char *operation = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    if (!target) {
      Serial.println("ERR IMU CAL syntax");
    } else if (strcmp(target, "GYRO") == 0 && !operation) {
      startGyroCalibration();
    } else if (strcmp(target, "ACCEL") == 0 &&
               operation && !extra &&
               strcmp(operation, "START") == 0) {
      startAccelCalibration();
    } else if (strcmp(target, "ACCEL") == 0 &&
               operation && !extra &&
               strcmp(operation, "ADD") == 0) {
      addAccelFace();
    } else if (strcmp(target, "MAG") == 0 &&
               operation && !extra &&
               strcmp(operation, "START") == 0) {
      startMagCalibration();
    } else if (strcmp(target, "MAG") == 0 &&
               operation && !extra &&
               strcmp(operation, "STOP") == 0) {
      stopMagCalibration();
    } else if (strcmp(target, "SAVE") == 0 && !operation) {
      imuCalibration.checksum = checksumFor(imuCalibration);
      if (!validCalibration(imuCalibration)) {
        Serial.println("ERR IMU CAL invalid values; not saved");
      } else {
        EEPROM.put(IMU_CAL_EEPROM_ADDRESS, imuCalibration);
        Serial.println("OK IMU CAL SAVED");
      }
    } else if (strcmp(target, "RESET") == 0 &&
               operation && !extra &&
               strcmp(operation, "CONFIRM") == 0) {
      loadCalibrationDefaults();
      EEPROM.put(IMU_CAL_EEPROM_ADDRESS, imuCalibration);
      attitudeEstimatorReset(attitudeEstimator);
      accelFaceMask = 0;
      calibrationMode = CAL_MODE_NONE;
      Serial.println("OK IMU CAL RESET");
    } else {
      Serial.println("ERR IMU CAL syntax");
    }
    return true;
  }

  if (strcmp(subcommand, "ALIGN") == 0) {
    char *operation = strtok_r(nullptr, " \t", save);
    char *confirmation = strtok_r(nullptr, " \t", save);
    char *extra = strtok_r(nullptr, " \t", save);
    if (operation && strcmp(operation, "STATUS") == 0 &&
        !confirmation && !extra) {
      Serial.print("IMU ALIGN VALID "); Serial.println(imuAlignment.valid);
      Serial.print("IMU ALIGN SENSOR_TO_AIRFRAME ");
      Serial.print(imuAlignment.sensorToAirframe[0], 7); Serial.print(",");
      Serial.print(imuAlignment.sensorToAirframe[1], 7); Serial.print(",");
      Serial.print(imuAlignment.sensorToAirframe[2], 7); Serial.print(",");
      Serial.println(imuAlignment.sensorToAirframe[3], 7);
    } else if (operation && strcmp(operation, "CAPTURE") == 0 &&
               confirmation && strcmp(confirmation, "CONFIRM") == 0 &&
               !extra) {
      const uint16_t saturationFlags =
          IMU_QUALITY_ACCEL_SATURATED | IMU_QUALITY_GYRO_SATURATED;
      const bool magRecent =
          imuRuntime.lastMagSampleUs != 0 &&
          (uint32_t)(imuRuntime.sampleUs - imuRuntime.lastMagSampleUs) <=
              100000u;
      if (!haveImuEstimate || !attitudeEstimator.stationary ||
          !attitudeEstimator.accelCorrectionActive ||
          !attitudeEstimator.magHealthy || !magRecent ||
          imuRuntime.confidence < 0.90f ||
          (imuRuntime.qualityFlags & saturationFlags)) {
        Serial.println(
            "ERR IMU ALIGN requires still rocket, accel+mag correction, "
            "confidence >=0.90");
      } else {
        const Quaternionf sensorToAirframe =
            quaternionNormalize(attitudeEstimator.q);
        imuAlignment.valid = 1;
        imuAlignment.sensorToAirframe[0] = sensorToAirframe.w;
        imuAlignment.sensorToAirframe[1] = sensorToAirframe.x;
        imuAlignment.sensorToAirframe[2] = sensorToAirframe.y;
        imuAlignment.sensorToAirframe[3] = sensorToAirframe.z;
        imuAlignment.checksum = checksumFor(imuAlignment);
        EEPROM.put(IMU_ALIGNMENT_EEPROM_ADDRESS, imuAlignment);
        benchReferenceValid = false;
        Serial.println("OK IMU ALIGN CAPTURED AND SAVED");
      }
    } else if (operation && strcmp(operation, "RESET") == 0 &&
               confirmation && strcmp(confirmation, "CONFIRM") == 0 &&
               !extra) {
      loadAlignmentDefaults();
      EEPROM.put(IMU_ALIGNMENT_EEPROM_ADDRESS, imuAlignment);
      benchReferenceValid = false;
      Serial.println("OK IMU ALIGN RESET");
    } else {
      Serial.println(
          "ERR IMU ALIGN use STATUS|CAPTURE CONFIRM|RESET CONFIRM");
    }
    return true;
  }

  if (strcmp(subcommand, "BENCH") == 0) {
    char *operation = strtok_r(nullptr, " \t", save);
    if (operation && strcmp(operation, "ZERO") == 0 &&
        !strtok_r(nullptr, " \t", save)) {
      if (!haveImuEstimate) {
        Serial.println("ERR IMU BENCH estimator unavailable");
      } else {
        benchReference = imuServiceAirframeQuaternion();
        benchReferenceValid = true;
        Serial.println("OK IMU BENCH ZERO");
      }
    } else if (operation && strcmp(operation, "CHECK") == 0) {
      char *axis = strtok_r(nullptr, " \t", save);
      char *degrees = strtok_r(nullptr, " \t", save);
      char *extra = strtok_r(nullptr, " \t", save);
      if (extra) Serial.println("ERR IMU BENCH CHECK syntax");
      else benchCheck(axis, degrees);
    } else {
      Serial.println("ERR IMU BENCH use ZERO or CHECK X|Y|Z degrees");
    }
    return true;
  }

  Serial.println("ERR IMU command");
  return true;
}

void imuServiceTask() {
  if (!configurationUnlocked()) {
    streamEnabled = false;
    if (calibrationMode != CAL_MODE_NONE) {
      calibrationMode = CAL_MODE_NONE;
    }
    return;
  }
  if (!streamEnabled) return;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastStreamMs) < 50u) return;
  lastStreamMs = nowMs;
  Serial.print("IMU_DATA,");
  Serial.print(imuRuntime.sampleUs);
  Serial.print(","); Serial.print(imuRuntime.dtUs);
  Serial.print(","); Serial.print(last_ax, 4);
  Serial.print(","); Serial.print(last_ay, 4);
  Serial.print(","); Serial.print(last_az, 4);
  Serial.print(","); Serial.print(last_gx, 4);
  Serial.print(","); Serial.print(last_gy, 4);
  Serial.print(","); Serial.print(last_gz, 4);
  const Quaternionf airframeQ = imuServiceAirframeQuaternion();
  Serial.print(","); Serial.print(airframeQ.w, 7);
  Serial.print(","); Serial.print(airframeQ.x, 7);
  Serial.print(","); Serial.print(airframeQ.y, 7);
  Serial.print(","); Serial.print(airframeQ.z, 7);
  Serial.print(","); Serial.print(roll * 57.2957795f, 3);
  Serial.print(","); Serial.print(pitch * 57.2957795f, 3);
  Serial.print(","); Serial.print(yaw * 57.2957795f, 3);
  Serial.print(","); Serial.print(imuRuntime.confidence, 3);
  Serial.print(","); Serial.println(imuRuntime.qualityFlags);
}
