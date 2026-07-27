#include "attitude_estimator.h"

#include <math.h>

static const float PI_F = 3.14159265358979323846f;
static const float DEG_TO_RAD_F = PI_F / 180.0f;
static const float RAD_TO_DEG_F = 180.0f / PI_F;

static float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float quaternionNorm(const Quaternionf &value) {
  return sqrtf(value.w * value.w + value.x * value.x +
               value.y * value.y + value.z * value.z);
}

Quaternionf quaternionNormalize(Quaternionf value) {
  const float norm = quaternionNorm(value);
  if (!isfinite(norm) || norm < 1.0e-7f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  const float inverse = 1.0f / norm;
  value.w *= inverse;
  value.x *= inverse;
  value.y *= inverse;
  value.z *= inverse;
  return value;
}

Quaternionf quaternionMultiply(const Quaternionf &a, const Quaternionf &b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

Quaternionf quaternionConjugate(const Quaternionf &value) {
  return {value.w, -value.x, -value.y, -value.z};
}

Quaternionf quaternionFromAxisAngle(uint8_t axis, float degrees) {
  const float half = degrees * DEG_TO_RAD_F * 0.5f;
  const float sine = sinf(half);
  Quaternionf value = {cosf(half), 0.0f, 0.0f, 0.0f};
  if (axis == 0) value.x = sine;
  else if (axis == 1) value.y = sine;
  else value.z = sine;
  return quaternionNormalize(value);
}

float quaternionAngularErrorDeg(const Quaternionf &a, const Quaternionf &b) {
  const Quaternionf na = quaternionNormalize(a);
  const Quaternionf nb = quaternionNormalize(b);
  float dot = na.w * nb.w + na.x * nb.x + na.y * nb.y + na.z * nb.z;
  dot = fabsf(clampFloat(dot, -1.0f, 1.0f));
  return 2.0f * acosf(dot) * RAD_TO_DEG_F;
}

static Quaternionf quaternionFromRotationVector(float xRad, float yRad,
                                                float zRad) {
  const float angle = sqrtf(xRad * xRad + yRad * yRad + zRad * zRad);
  if (!isfinite(angle) || angle < 1.0e-9f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  const float scale = sinf(0.5f * angle) / angle;
  return quaternionNormalize(
      {cosf(0.5f * angle), xRad * scale, yRad * scale, zRad * scale});
}

static void rotateVector(const Quaternionf &rotation,
                         float x, float y, float z,
                         float &outX, float &outY, float &outZ) {
  const Quaternionf normalized = quaternionNormalize(rotation);
  const Quaternionf vector = {0.0f, x, y, z};
  const Quaternionf rotated = quaternionMultiply(
      quaternionMultiply(normalized, vector),
      quaternionConjugate(normalized));
  outX = rotated.x;
  outY = rotated.y;
  outZ = rotated.z;
}

static Quaternionf quaternionFromVectorAlignment(float fromX, float fromY,
                                                 float fromZ, float toX,
                                                 float toY, float toZ) {
  const float fromNorm =
      sqrtf(fromX * fromX + fromY * fromY + fromZ * fromZ);
  const float toNorm = sqrtf(toX * toX + toY * toY + toZ * toZ);
  if (fromNorm < 1.0e-7f || toNorm < 1.0e-7f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  fromX /= fromNorm;
  fromY /= fromNorm;
  fromZ /= fromNorm;
  toX /= toNorm;
  toY /= toNorm;
  toZ /= toNorm;
  const float dot =
      clampFloat(fromX * toX + fromY * toY + fromZ * toZ, -1.0f, 1.0f);
  if (dot > 0.999999f) return {1.0f, 0.0f, 0.0f, 0.0f};
  if (dot < -0.999999f) {
    float axisX = 0.0f;
    float axisY = fromZ;
    float axisZ = -fromY;
    float axisNorm =
        sqrtf(axisX * axisX + axisY * axisY + axisZ * axisZ);
    if (axisNorm < 1.0e-6f) {
      axisX = -fromZ;
      axisY = 0.0f;
      axisZ = fromX;
      axisNorm = sqrtf(axisX * axisX + axisZ * axisZ);
    }
    return quaternionFromRotationVector(axisX / axisNorm * PI_F,
                                        axisY / axisNorm * PI_F,
                                        axisZ / axisNorm * PI_F);
  }
  const float crossX = fromY * toZ - fromZ * toY;
  const float crossY = fromZ * toX - fromX * toZ;
  const float crossZ = fromX * toY - fromY * toX;
  return quaternionNormalize({1.0f + dot, crossX, crossY, crossZ});
}

static Quaternionf quaternionFromRotationMatrix(
    float r00, float r01, float r02,
    float r10, float r11, float r12,
    float r20, float r21, float r22) {
  Quaternionf q = {};
  const float trace = r00 + r11 + r22;
  if (trace > 0.0f) {
    const float scale = sqrtf(trace + 1.0f) * 2.0f;
    q.w = 0.25f * scale;
    q.x = (r21 - r12) / scale;
    q.y = (r02 - r20) / scale;
    q.z = (r10 - r01) / scale;
  } else if (r00 > r11 && r00 > r22) {
    const float scale = sqrtf(1.0f + r00 - r11 - r22) * 2.0f;
    q.w = (r21 - r12) / scale;
    q.x = 0.25f * scale;
    q.y = (r01 + r10) / scale;
    q.z = (r02 + r20) / scale;
  } else if (r11 > r22) {
    const float scale = sqrtf(1.0f + r11 - r00 - r22) * 2.0f;
    q.w = (r02 - r20) / scale;
    q.x = (r01 + r10) / scale;
    q.y = 0.25f * scale;
    q.z = (r12 + r21) / scale;
  } else {
    const float scale = sqrtf(1.0f + r22 - r00 - r11) * 2.0f;
    q.w = (r10 - r01) / scale;
    q.x = (r02 + r20) / scale;
    q.y = (r12 + r21) / scale;
    q.z = 0.25f * scale;
  }
  return quaternionNormalize(q);
}

static bool quaternionFromAccelMag(float axG, float ayG, float azG,
                                   float mxUt, float myUt, float mzUt,
                                   Quaternionf &result) {
  const float accelNorm = sqrtf(axG * axG + ayG * ayG + azG * azG);
  const float magNorm = sqrtf(mxUt * mxUt + myUt * myUt + mzUt * mzUt);
  if (accelNorm < 1.0e-7f || magNorm < 1.0e-7f) return false;

  const float gravityX = axG / accelNorm;
  const float gravityY = ayG / accelNorm;
  const float gravityZ = azG / accelNorm;
  const float magX = mxUt / magNorm;
  const float magY = myUt / magNorm;
  const float magZ = mzUt / magNorm;
  const float verticalMag =
      magX * gravityX + magY * gravityY + magZ * gravityZ;
  float northX = magX - verticalMag * gravityX;
  float northY = magY - verticalMag * gravityY;
  float northZ = magZ - verticalMag * gravityZ;
  const float horizontalNorm =
      sqrtf(northX * northX + northY * northY + northZ * northZ);
  if (!isfinite(horizontalNorm) || horizontalNorm < 0.05f) return false;
  northX /= horizontalNorm;
  northY /= horizontalNorm;
  northZ /= horizontalNorm;

  // The body-frame basis is north, east, up. Rows of the body-to-world
  // rotation are those unit vectors because the world basis is X=north,
  // Y=east, Z=up.
  const float eastX = gravityY * northZ - gravityZ * northY;
  const float eastY = gravityZ * northX - gravityX * northZ;
  const float eastZ = gravityX * northY - gravityY * northX;
  result = quaternionFromRotationMatrix(
      northX, northY, northZ,
      eastX, eastY, eastZ,
      gravityX, gravityY, gravityZ);
  return true;
}

void quaternionToEuler(const Quaternionf &input,
                       float &roll, float &pitch, float &yaw) {
  const Quaternionf q = quaternionNormalize(input);
  const float sinrCosp = 2.0f * (q.w * q.x + q.y * q.z);
  const float cosrCosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  roll = atan2f(sinrCosp, cosrCosp);

  const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  pitch = asinf(clampFloat(sinp, -1.0f, 1.0f));

  const float sinyCosp = 2.0f * (q.w * q.z + q.x * q.y);
  const float cosyCosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  yaw = atan2f(sinyCosp, cosyCosp);
  if (yaw < 0.0f) yaw += 2.0f * PI_F;
}

static Quaternionf integrateGyro(const Quaternionf &input,
                                 float gxRadS, float gyRadS, float gzRadS,
                                 float dtS) {
  const Quaternionf q = quaternionNormalize(input);
  const float halfDt = 0.5f * dtS;
  return quaternionNormalize({
      q.w + (-q.x * gxRadS - q.y * gyRadS - q.z * gzRadS) * halfDt,
      q.x + (q.w * gxRadS + q.y * gzRadS - q.z * gyRadS) * halfDt,
      q.y + (q.w * gyRadS - q.x * gzRadS + q.z * gxRadS) * halfDt,
      q.z + (q.w * gzRadS + q.x * gyRadS - q.y * gxRadS) * halfDt,
  });
}

void attitudeEstimatorReset(AttitudeEstimatorState &state) {
  state = {};
  state.q = {1.0f, 0.0f, 0.0f, 0.0f};
  state.accelInnovationDeg = 180.0f;
  state.magInnovationDeg = 180.0f;
  state.gyroOnly = true;
}

void attitudeEstimatorSetGyroBias(AttitudeEstimatorState &state,
                                  float xDps, float yDps, float zDps) {
  state.gyroBiasDps[0] = xDps;
  state.gyroBiasDps[1] = yDps;
  state.gyroBiasDps[2] = zDps;
}

void attitudeEstimatorUpdate(AttitudeEstimatorState &state,
                             const AttitudeEstimatorConfig &config,
                             const AttitudeEstimatorInput &input) {
  float dtS = input.dtS;
  if (!isfinite(dtS) || dtS <= 0.0f) dtS = 0.005f;
  if (dtS > 0.05f) dtS = 0.05f;

  const float accelMagnitude =
      sqrtf(input.axG * input.axG + input.ayG * input.ayG +
            input.azG * input.azG);
  const float magMagnitude =
      sqrtf(input.mxUt * input.mxUt + input.myUt * input.myUt +
            input.mzUt * input.mzUt);
  const float correctedGx = input.gxDps - state.gyroBiasDps[0];
  const float correctedGy = input.gyDps - state.gyroBiasDps[1];
  const float correctedGz = input.gzDps - state.gyroBiasDps[2];
  const float gyroMagnitude =
      sqrtf(correctedGx * correctedGx + correctedGy * correctedGy +
            correctedGz * correctedGz);

  state.stationary =
      !input.accelSaturated && !input.gyroSaturated &&
      fabsf(accelMagnitude - 1.0f) <= config.stationaryAccelToleranceG &&
      gyroMagnitude <= config.stationaryGyroMaxDps;
  if (input.allowBiasLearning && state.stationary) {
    const float fraction = 1.0f - expf(-config.biasLearningRate * dtS);
    state.gyroBiasDps[0] +=
        fraction * (input.gxDps - state.gyroBiasDps[0]);
    state.gyroBiasDps[1] +=
        fraction * (input.gyDps - state.gyroBiasDps[1]);
    state.gyroBiasDps[2] +=
        fraction * (input.gzDps - state.gyroBiasDps[2]);
  }

  const bool accelMagnitudeOk =
      !input.accelSaturated && isfinite(accelMagnitude) &&
      accelMagnitude >= config.accelMinG &&
      accelMagnitude <= config.accelMaxG;
  const bool magMagnitudeOk =
      isfinite(magMagnitude) && magMagnitude >= config.magMinUt &&
      magMagnitude <= config.magMaxUt;

  if (!state.initialized) {
    state.initializationAttempts++;
    Quaternionf accelMagQ = {};
    if (accelMagnitudeOk && magMagnitudeOk &&
        input.allowMagCorrection &&
        quaternionFromAccelMag(input.axG, input.ayG, input.azG,
                               input.mxUt, input.myUt, input.mzUt,
                               accelMagQ)) {
      state.q = accelMagQ;
      state.magReferenceWorld[0] = 1.0f;
      state.magReferenceWorld[1] = 0.0f;
      state.magReferenceWorld[2] = 0.0f;
      state.magReferenceValid = true;
      state.magHealthy = true;
      state.magInnovationDeg = 0.0f;
      state.initialized = true;
    } else if (state.initializationAttempts >= 40u) {
      state.q = accelMagnitudeOk
                    ? quaternionFromVectorAlignment(
                          input.axG, input.ayG, input.azG,
                          0.0f, 0.0f, 1.0f)
                    : Quaternionf{1.0f, 0.0f, 0.0f, 0.0f};
      state.initialized = true;
    } else {
      state.confidence = 0.10f;
      return;
    }
  } else {
    state.q = integrateGyro(state.q,
                            correctedGx * DEG_TO_RAD_F,
                            correctedGy * DEG_TO_RAD_F,
                            correctedGz * DEG_TO_RAD_F, dtS);
  }

  quaternionToEuler(state.q, state.rollRad, state.pitchRad, state.yawRad);
  state.accelCorrectionActive = false;
  state.magCorrectionActive = false;
  state.accelRejected = false;
  state.accelInnovationDeg = 180.0f;

  if (input.allowAccelCorrection && accelMagnitudeOk) {
    const float measuredX = input.axG / accelMagnitude;
    const float measuredY = input.ayG / accelMagnitude;
    const float measuredZ = input.azG / accelMagnitude;
    float predictedX = 0.0f;
    float predictedY = 0.0f;
    float predictedZ = 0.0f;
    rotateVector(quaternionConjugate(state.q), 0.0f, 0.0f, 1.0f,
                 predictedX, predictedY, predictedZ);
    const float dot = clampFloat(measuredX * predictedX +
                                     measuredY * predictedY +
                                     measuredZ * predictedZ,
                                 -1.0f, 1.0f);
    const float innovationRad = acosf(dot);
    state.accelInnovationDeg = innovationRad * RAD_TO_DEG_F;
    if (state.accelInnovationDeg <= config.accelInnovationMaxDeg) {
      float correctionX = measuredY * predictedZ - measuredZ * predictedY;
      float correctionY = measuredZ * predictedX - measuredX * predictedZ;
      float correctionZ = measuredX * predictedY - measuredY * predictedX;
      const float correctionNorm =
          sqrtf(correctionX * correctionX + correctionY * correctionY +
                correctionZ * correctionZ);
      if (correctionNorm > 1.0e-7f && innovationRad > 1.0e-7f) {
        const float fraction =
            1.0f - expf(-config.accelCorrectionRate * dtS);
        const float correctionAngle = fraction * innovationRad;
        correctionX *= correctionAngle / correctionNorm;
        correctionY *= correctionAngle / correctionNorm;
        correctionZ *= correctionAngle / correctionNorm;
        state.q = quaternionNormalize(quaternionMultiply(
            state.q, quaternionFromRotationVector(
                         correctionX, correctionY, correctionZ)));
      }
      state.accelCorrectionActive = true;
      quaternionToEuler(state.q, state.rollRad, state.pitchRad, state.yawRad);
    } else {
      state.accelRejected = true;
    }
  } else if (input.allowAccelCorrection) {
    state.accelRejected = true;
  }

  if (input.allowMagCorrection && state.accelCorrectionActive &&
      magMagnitudeOk) {
    state.magRejected = false;
    float magWorldX = 0.0f;
    float magWorldY = 0.0f;
    float magWorldZ = 0.0f;
    rotateVector(state.q, input.mxUt / magMagnitude,
                 input.myUt / magMagnitude, input.mzUt / magMagnitude,
                 magWorldX, magWorldY, magWorldZ);
    const float horizontalNorm =
        sqrtf(magWorldX * magWorldX + magWorldY * magWorldY);
    if (!state.magReferenceValid && horizontalNorm > 0.05f) {
      state.magReferenceWorld[0] = magWorldX / horizontalNorm;
      state.magReferenceWorld[1] = magWorldY / horizontalNorm;
      state.magReferenceWorld[2] = 0.0f;
      state.magReferenceValid = true;
    }
    if (state.magReferenceValid && horizontalNorm > 0.05f) {
      magWorldX /= horizontalNorm;
      magWorldY /= horizontalNorm;
      const float dot = clampFloat(
          magWorldX * state.magReferenceWorld[0] +
              magWorldY * state.magReferenceWorld[1],
          -1.0f, 1.0f);
      const float crossZ =
          magWorldX * state.magReferenceWorld[1] -
          magWorldY * state.magReferenceWorld[0];
      const float yawErrorRad = atan2f(crossZ, dot);
      state.magInnovationDeg = fabsf(yawErrorRad) * RAD_TO_DEG_F;
      if (state.magInnovationDeg <= config.magInnovationMaxDeg) {
        const float fraction =
            1.0f - expf(-config.magCorrectionRate * dtS);
        const Quaternionf correction = quaternionFromRotationVector(
            0.0f, 0.0f, fraction * yawErrorRad);
        state.q = quaternionNormalize(
            quaternionMultiply(correction, state.q));
        state.magCorrectionActive = true;
        state.magHealthy = true;
        quaternionToEuler(state.q, state.rollRad, state.pitchRad,
                          state.yawRad);
      } else {
        state.magRejected = true;
        state.magHealthy = false;
      }
    } else {
      state.magRejected = true;
      state.magHealthy = false;
    }
  } else if (input.allowMagCorrection) {
    state.magRejected = true;
    state.magHealthy = false;
  }

  state.gyroOnly =
      !state.accelCorrectionActive && !state.magCorrectionActive;
  float confidence = state.gyroOnly ? 0.35f : 0.72f;
  if (state.magHealthy) confidence = 0.95f;
  if (state.stationary && state.accelCorrectionActive) confidence += 0.05f;
  if (input.accelSaturated || input.gyroSaturated) confidence *= 0.35f;
  if (state.accelRejected) confidence *= 0.8f;
  if (state.magRejected) confidence *= 0.9f;
  state.confidence = clampFloat(confidence, 0.0f, 1.0f);
  state.q = quaternionNormalize(state.q);
}
