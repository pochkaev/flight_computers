#pragma once

#include <Arduino.h>

#include "attitude_estimator.h"

enum ImuCalibrationFlags : uint16_t {
  IMU_CAL_GYRO_VALID = 1u << 0,
  IMU_CAL_ACCEL_VALID = 1u << 1,
  IMU_CAL_MAG_VALID = 1u << 2,
};

enum ImuQualityFlags : uint16_t {
  IMU_QUALITY_ACCEL_CORRECTION = 1u << 0,
  IMU_QUALITY_MAG_CORRECTION = 1u << 1,
  IMU_QUALITY_GYRO_ONLY = 1u << 2,
  IMU_QUALITY_ACCEL_SATURATED = 1u << 3,
  IMU_QUALITY_GYRO_SATURATED = 1u << 4,
  IMU_QUALITY_SAMPLE_GAP = 1u << 5,
  IMU_QUALITY_ACCEL_REJECTED = 1u << 6,
  IMU_QUALITY_MAG_REJECTED = 1u << 7,
  IMU_QUALITY_STATIONARY = 1u << 8,
  IMU_QUALITY_GYRO_CALIBRATED = 1u << 9,
  IMU_QUALITY_ACCEL_CALIBRATED = 1u << 10,
  IMU_QUALITY_MAG_CALIBRATED = 1u << 11,
  IMU_QUALITY_MAG_FRESH = 1u << 12,
  IMU_QUALITY_AIRFRAME_ALIGNED = 1u << 13,
};

struct ImuCalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint16_t validFlags;
  uint16_t reserved;
  float gyroBiasDps[3];
  float accelBiasMps2[3];
  float accelScale[3];
  float magBiasUt[3];
  float magScale[3];
  uint32_t checksum;
};

struct ImuRuntimeData {
  float calibratedAxMps2;
  float calibratedAyMps2;
  float calibratedAzMps2;
  float calibratedGxDps;
  float calibratedGyDps;
  float calibratedGzDps;
  float calibratedMxUt;
  float calibratedMyUt;
  float calibratedMzUt;
  float accelInnovationDeg;
  float magInnovationDeg;
  float confidence;
  uint32_t firstSampleUs;
  uint32_t sampleUs;
  uint32_t lastMagSampleUs;
  uint32_t sampleCount;
  uint32_t invalidSampleCount;
  uint32_t notReadyPollCount;
  uint32_t magSampleCount;
  uint32_t missedSampleCount;
  uint32_t saturationSampleCount;
  uint32_t maxDtUs;
  uint16_t dtUs;
  uint16_t magDtUs;
  uint16_t qualityFlags;
};

struct ImuAlignmentData {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint16_t valid;
  uint16_t reserved;
  float sensorToAirframe[4];
  uint32_t checksum;
};

extern ImuCalibrationData imuCalibration;
extern ImuAlignmentData imuAlignment;
extern ImuRuntimeData imuRuntime;
extern AttitudeEstimatorState attitudeEstimator;

Quaternionf imuServiceAirframeQuaternion();
void imuServiceInit();
void imuServiceProcessSample(float axMps2, float ayMps2, float azMps2,
                             float gxDps, float gyDps, float gzDps,
                             float mxUt, float myUt, float mzUt,
                             uint32_t sampleUs, uint32_t dtUs,
                             bool magFresh);
void imuServiceRecordInvalidSample();
void imuServiceRecordNotReady();
void imuServiceTask();
void imuServicePrintStatus(Stream &output);
bool imuServiceHandleCommand(char *subcommand, char **save);
