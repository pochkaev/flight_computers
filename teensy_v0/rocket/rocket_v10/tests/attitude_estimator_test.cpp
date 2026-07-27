#include "../fw/RocketV10/attitude_estimator.h"
#include "../fw/RocketV10/imu_frame.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "FAIL %s\n", message);
    std::exit(1);
  }
  std::printf("PASS %s\n", message);
}

static AttitudeEstimatorConfig config() {
  return {
      0.82f, 1.18f, 35.0f,
      10.0f, 90.0f, 45.0f,
      3.0f, 0.7f,
      0.05f, 3.0f, 0.15f,
  };
}

static AttitudeEstimatorInput levelInput() {
  return {
      0.0f, 0.0f, 1.0f,
      0.0f, 0.0f, 0.0f,
      25.0f, 0.0f, 40.0f,
      0.005f,
      true, true, true,
      false, false,
  };
}

static void rotateVector(const Quaternionf &rotation,
                         float x, float y, float z,
                         float &outX, float &outY, float &outZ) {
  const Quaternionf vector = {0.0f, x, y, z};
  const Quaternionf rotated = quaternionMultiply(
      quaternionMultiply(rotation, vector), quaternionConjugate(rotation));
  outX = rotated.x;
  outY = rotated.y;
  outZ = rotated.z;
}

int main() {
  float bodyMx = 0.0f;
  float bodyMy = 0.0f;
  float bodyMz = 0.0f;
  lsm9ds1MagToAgFrame(-12.5f, 31.0f, -42.0f,
                      bodyMx, bodyMy, bodyMz);
  require(std::fabs(bodyMx - 12.5f) < 1.0e-6f &&
              std::fabs(bodyMy + 31.0f) < 1.0e-6f &&
              std::fabs(bodyMz + 42.0f) < 1.0e-6f,
          "LSM9DS1 magnetometer axes map into accel gyro frame");

  const Quaternionf sensorToAirframe =
      quaternionMultiply(quaternionFromAxisAngle(2, 20.0f),
                         quaternionFromAxisAngle(0, -7.0f));
  const Quaternionf airframeMotion = quaternionFromAxisAngle(1, 90.0f);
  const Quaternionf sensorToWorld =
      quaternionMultiply(airframeMotion, sensorToAirframe);
  const Quaternionf aligned = quaternionMultiply(
      sensorToWorld, quaternionConjugate(sensorToAirframe));
  require(quaternionAngularErrorDeg(aligned, airframeMotion) < 0.001f,
          "fixed sensor mounting quaternion produces airframe motion");

  AttitudeEstimatorInput invertedInput = levelInput();
  invertedInput.axG = 0.0001f;
  invertedInput.ayG = 0.0f;
  invertedInput.azG = -1.0f;
  invertedInput.mxUt = 25.0f;
  invertedInput.myUt = 5.0f;
  invertedInput.mzUt = -40.0f;
  AttitudeEstimatorState invertedA;
  attitudeEstimatorReset(invertedA);
  attitudeEstimatorUpdate(invertedA, config(), invertedInput);
  require(invertedA.initialized && invertedA.magHealthy,
          "gravity plus magnetometer initializes inverted pose");
  float worldX = 0.0f;
  float worldY = 0.0f;
  float worldZ = 0.0f;
  rotateVector(invertedA.q, invertedInput.axG, invertedInput.ayG,
               invertedInput.azG, worldX, worldY, worldZ);
  require(std::fabs(worldX) < 0.001f && std::fabs(worldY) < 0.001f &&
              worldZ > 0.999f,
          "startup quaternion maps measured gravity to world up");
  rotateVector(invertedA.q, invertedInput.mxUt, invertedInput.myUt,
               invertedInput.mzUt, worldX, worldY, worldZ);
  require(worldX > 0.0f && std::fabs(worldY) < 0.001f,
          "startup quaternion maps horizontal magnetic field to world X");

  AttitudeEstimatorState invertedB;
  attitudeEstimatorReset(invertedB);
  invertedInput.axG = -0.0001f;
  attitudeEstimatorUpdate(invertedB, config(), invertedInput);
  require(quaternionAngularErrorDeg(invertedA.q, invertedB.q) < 0.05f,
          "inverted startup yaw is stable across gravity noise");

  AttitudeEstimatorState state;
  attitudeEstimatorReset(state);
  AttitudeEstimatorInput input = levelInput();
  for (int i = 0; i < 400; ++i) {
    attitudeEstimatorUpdate(state, config(), input);
  }
  require(std::fabs(quaternionNorm(state.q) - 1.0f) < 1.0e-5f,
          "stationary quaternion remains normalized");
  require(state.accelCorrectionActive && state.magCorrectionActive,
          "trusted stationary references are accepted");

  const float acceptedMagInnovation = state.magInnovationDeg;
  input.allowMagCorrection = false;
  attitudeEstimatorUpdate(state, config(), input);
  require(state.magHealthy && !state.magCorrectionActive &&
              std::fabs(state.magInnovationDeg - acceptedMagInnovation) <
                  1.0e-6f,
          "magnetometer health persists between fresh samples");

  const Quaternionf start = state.q;
  input.allowAccelCorrection = false;
  input.allowMagCorrection = false;
  input.allowBiasLearning = false;
  input.gxDps = 90.0f;
  for (int i = 0; i < 200; ++i) {
    attitudeEstimatorUpdate(state, config(), input);
  }
  const Quaternionf expected90 =
      quaternionMultiply(start, quaternionFromAxisAngle(0, 90.0f));
  require(quaternionAngularErrorDeg(state.q, expected90) < 0.25f,
          "90 degree gyro rotation matches expected quaternion");

  input.gxDps = -90.0f;
  for (int i = 0; i < 200; ++i) {
    attitudeEstimatorUpdate(state, config(), input);
  }
  require(quaternionAngularErrorDeg(state.q, start) < 0.35f,
          "opposite rotation returns to start");

  attitudeEstimatorReset(state);
  input = levelInput();
  input.axG = 0.0f;
  input.ayG = 0.0f;
  input.azG = -1.0f;
  input.mxUt = 25.0f;
  input.myUt = 0.0f;
  input.mzUt = -40.0f;
  attitudeEstimatorUpdate(state, config(), input);
  const Quaternionf verticalStart = state.q;
  float worldMagX = 0.0f;
  float worldMagY = 0.0f;
  float worldMagZ = 0.0f;
  rotateVector(verticalStart, input.mxUt, input.myUt, input.mzUt,
               worldMagX, worldMagY, worldMagZ);
  for (int i = 1; i <= 200; ++i) {
    const Quaternionf truth = quaternionMultiply(
        verticalStart, quaternionFromAxisAngle(2, -0.45f * i));
    const Quaternionf worldToBody = quaternionConjugate(truth);
    rotateVector(worldToBody, 0.0f, 0.0f, 1.0f,
                 input.axG, input.ayG, input.azG);
    rotateVector(worldToBody, worldMagX, worldMagY, worldMagZ,
                 input.mxUt, input.myUt, input.mzUt);
    input.gxDps = 0.0f;
    input.gyDps = 0.0f;
    input.gzDps = -90.0f;
    attitudeEstimatorUpdate(state, config(), input);
  }
  const Quaternionf expectedVerticalRoll = quaternionMultiply(
      verticalStart, quaternionFromAxisAngle(2, -90.0f));
  require(quaternionAngularErrorDeg(state.q, expectedVerticalRoll) < 0.5f,
          "vertical 90 degree roll avoids Euler singularity");
  require(state.accelCorrectionActive && state.magCorrectionActive &&
              state.confidence >= 0.90f,
          "vertical roll keeps trusted vector corrections active");

  const Quaternionf beforeBoost = state.q;
  input = levelInput();
  input.axG = 8.0f;
  input.azG = 0.0f;
  input.mxUt = 300.0f;
  for (int i = 0; i < 100; ++i) {
    attitudeEstimatorUpdate(state, config(), input);
  }
  require(!state.accelCorrectionActive && !state.magCorrectionActive,
          "boost and magnetic disturbance corrections are rejected");
  require(quaternionAngularErrorDeg(state.q, beforeBoost) < 0.1f,
          "rejected disturbance does not rotate quaternion");

  attitudeEstimatorReset(state);
  input = levelInput();
  input.gxDps = 1.5f;
  for (int i = 0; i < 4000; ++i) {
    attitudeEstimatorUpdate(state, config(), input);
  }
  require(std::fabs(state.gyroBiasDps[0] - 1.5f) < 0.15f,
          "stationary gyro bias converges");
  require(std::fabs(quaternionNorm(state.q) - 1.0f) < 1.0e-5f,
          "long run quaternion remains normalized");

  std::printf("ALL QUATERNION ACCEPTANCE CHECKS PASSED\n");
  return 0;
}
