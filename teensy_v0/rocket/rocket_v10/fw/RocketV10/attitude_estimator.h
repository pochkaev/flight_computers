#pragma once

#include <stdint.h>

struct Quaternionf {
  float w;
  float x;
  float y;
  float z;
};

struct AttitudeEstimatorConfig {
  float accelMinG;
  float accelMaxG;
  float accelInnovationMaxDeg;
  float magMinUt;
  float magMaxUt;
  float magInnovationMaxDeg;
  float accelCorrectionRate;
  float magCorrectionRate;
  float stationaryAccelToleranceG;
  float stationaryGyroMaxDps;
  float biasLearningRate;
};

struct AttitudeEstimatorInput {
  float axG;
  float ayG;
  float azG;
  float gxDps;
  float gyDps;
  float gzDps;
  float mxUt;
  float myUt;
  float mzUt;
  float dtS;
  bool allowAccelCorrection;
  bool allowMagCorrection;
  bool allowBiasLearning;
  bool accelSaturated;
  bool gyroSaturated;
};

struct AttitudeEstimatorState {
  Quaternionf q;
  float gyroBiasDps[3];
  float magReferenceWorld[3];
  float rollRad;
  float pitchRad;
  float yawRad;
  float accelInnovationDeg;
  float magInnovationDeg;
  float confidence;
  uint16_t initializationAttempts;
  bool initialized;
  bool magReferenceValid;
  bool accelCorrectionActive;
  bool magCorrectionActive;
  bool magHealthy;
  bool gyroOnly;
  bool accelRejected;
  bool magRejected;
  bool stationary;
};

void attitudeEstimatorReset(AttitudeEstimatorState &state);
void attitudeEstimatorSetGyroBias(AttitudeEstimatorState &state,
                                  float xDps, float yDps, float zDps);
void attitudeEstimatorUpdate(AttitudeEstimatorState &state,
                             const AttitudeEstimatorConfig &config,
                             const AttitudeEstimatorInput &input);

Quaternionf quaternionNormalize(Quaternionf value);
Quaternionf quaternionMultiply(const Quaternionf &a, const Quaternionf &b);
Quaternionf quaternionConjugate(const Quaternionf &value);
Quaternionf quaternionFromAxisAngle(uint8_t axis, float degrees);
float quaternionNorm(const Quaternionf &value);
float quaternionAngularErrorDeg(const Quaternionf &a, const Quaternionf &b);
void quaternionToEuler(const Quaternionf &input,
                       float &roll, float &pitch, float &yaw);
