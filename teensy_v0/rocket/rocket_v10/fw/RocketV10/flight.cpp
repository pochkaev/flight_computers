#include "flight.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "pyro.h"
#include "sensors.h"
#include "state.h"
#include "storage.h"

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

static void setLaunchArmGate(uint32_t nowMs, float currentAltM) {
  launchArmed = true;
  launchArmedMs = nowMs;
  launchArmMotionSinceMs = 0;
  baseAltM = filtAlt;
  velRefAltM = currentAltM;
  velRefMs = nowMs;
  velZ = 0.0f;
  resetRelAltHistory(nowMs, 0.0f);
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

static void markRecoveredLaunch(uint32_t nowMs, float relAlt) {
  if (!(flightFlags & FLAG_LAUNCH)) {
    tLaunchMs = nowMs;
  }
  flightFlags |= FLAG_LAUNCH | FLAG_RECOVERY_CLASSIFIED;
  if (relAlt > maxAltM) maxAltM = relAlt;
  if (velZ > maxVelMps) maxVelMps = velZ;
}


void resetRelAltHistory(uint32_t nowMs, float relAlt) {
  for (uint8_t i = 0; i < REL_ALT_HISTORY_COUNT; ++i) {
    relAltHistoryM[i] = relAlt;
    relAltHistoryMs[i] = nowMs;
  }
  relAltHistoryIndex = 0;
  relAltHistoryFilled = true;
}

void clearLaunchArmGate() {
  launchArmed = false;
  launchArmedMs = 0;
  launchArmStillSinceMs = 0;
  launchArmMotionSinceMs = 0;
  launchDetectSinceMs = 0;
}

static uint16_t secondsCeilRemaining(uint32_t elapsedMs, uint32_t targetMs) {
  if (elapsedMs >= targetMs) return 0;
  uint32_t remainingMs = targetMs - elapsedMs;
  uint32_t seconds = (remainingMs + 999u) / 1000u;
  return seconds > 65535u ? 65535u : (uint16_t)seconds;
}

uint8_t currentLaunchStatus(uint16_t &waitSecondsOut) {
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

float currentBaroRelAltM() {
  if (isnan(baseAltM)) return filtAlt;
  return filtAlt - baseAltM;
}

uint16_t buildHealthFlags() {
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

bool taskDue(uint32_t nowMs, uint32_t &lastRunMs, uint32_t periodMs) {
  if ((uint32_t)(nowMs - lastRunMs) < periodMs) return false;
  lastRunMs = nowMs;
  return true;
}

void updateFlightStateFromBaroSample(uint32_t nowMs, float currentAltM) {
  float relAlt = currentBaroRelAltM();
  float accMagG = sqrtf(last_ax * last_ax + last_ay * last_ay + last_az * last_az) / 9.80665f;
  const float axialAccelG = (-last_az) / 9.80665f;
  float gyroMagDps = sqrtf(last_gx * last_gx + last_gy * last_gy + last_gz * last_gz);
  float trendRelAltM = relAlt;
  const bool launchTrendOk = relAltAtLeastAgo(nowMs, LAUNCH_TREND_MS, trendRelAltM) &&
                             ((relAlt - trendRelAltM) >= LAUNCH_TREND_MIN_M);
  const bool padSettled = padSettleStartMs != 0 &&
                          ((uint32_t)(nowMs - padSettleStartMs) >= LAUNCH_PAD_SETTLE_MS);
  const bool launchStillCond = fabsf(accMagG - 1.0f) <= LAUNCH_PAD_STILL_ACCEL_ERR_G &&
                               gyroMagDps <= LAUNCH_PAD_STILL_GYRO_DPS;
  const bool powerOnInhibitDone = nowMs >= LAUNCH_POWERON_INHIBIT_MS;
  const bool launchAccelCond = launchTrendOk &&
                               (relAlt > LAUNCH_REL_ALT_M) &&
                               (velZ > LAUNCH_VEL_MPS) &&
                               (axialAccelG >= LAUNCH_AXIAL_ACCEL_G);
  const bool launchBaroCond = launchTrendOk &&
                              (relAlt > LAUNCH_BARO_REL_ALT_M) &&
                              (velZ > LAUNCH_BARO_VEL_MPS);
  const bool launchObviousCond = (relAlt > LAUNCH_OBVIOUS_REL_ALT_M) &&
                                 (velZ > LAUNCH_OBVIOUS_VEL_MPS);
  const bool launchKinematicsCond = launchAccelCond || launchBaroCond || launchObviousCond;
  if (flightState == FS_IDLE || flightState == FS_PAD) {
    if (!powerOnInhibitDone || !padSettled) {
      clearLaunchArmGate();
    } else if (!launchArmed) {
      if (conditionHeld(launchStillCond, nowMs, launchArmStillSinceMs, LAUNCH_PAD_STILL_ARM_MS)) {
        setLaunchArmGate(nowMs, currentAltM);
        relAlt = currentBaroRelAltM();
      }
    } else if (launchStillCond || launchKinematicsCond) {
      launchArmMotionSinceMs = 0;
    } else {
      if (launchArmMotionSinceMs == 0) launchArmMotionSinceMs = nowMs;
      if ((uint32_t)(nowMs - launchArmMotionSinceMs) > LAUNCH_ARM_MOTION_GRACE_MS) {
        clearLaunchArmGate();
      }
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

  const bool recoverySubsonicCoastCond = launchTrendOk &&
                                         (relAlt > RECOVERY_SUBSONIC_COAST_REL_ALT_M) &&
                                         (velZ > RECOVERY_SUBSONIC_COAST_VEL_MPS);
  const bool recoveryNearApogeeCond = (relAlt > RECOVERY_NEAR_APOGEE_REL_ALT_M) &&
                                      (fabsf(velZ) < RECOVERY_NEAR_APOGEE_ABS_VEL_MPS) &&
                                      ((flightFlags & FLAG_LAUNCH) ||
                                       maxAltM > RECOVERY_NEAR_APOGEE_REL_ALT_M ||
                                       relAlt > RECOVERY_NEAR_APOGEE_FALLBACK_REL_ALT_M);
  const bool recoveryDescendingBallisticCond = (relAlt > RECOVERY_DESCENT_REL_ALT_M) &&
                                               (velZ < RECOVERY_DESCENT_VEL_MPS);
  const bool recoveryUnderDrogueCond = (flightState == FS_DESCENT_BALLISTIC ||
                                        flightState == FS_DUAL_DEPLOY_APOGEE_LOGGED ||
                                        flightState == FS_DUAL_DEPLOY_MAIN_LOGGED) &&
                                       (relAlt > RECOVERY_UNDER_DROGUE_REL_ALT_M) &&
                                       (velZ < APOGEE_VEL_MPS) &&
                                       (velZ > RECOVERY_UNDER_DROGUE_FAST_VEL_MPS);
  const bool recoveryPostFlightCond = ((flightFlags & FLAG_LAUNCH) || maxAltM > RECOVERY_POST_FLIGHT_MAX_REL_ALT_M) &&
                                      (relAlt < RECOVERY_POST_FLIGHT_MAX_REL_ALT_M) &&
                                      (fabsf(velZ) < LANDED_ABS_VEL_MPS) &&
                                      stillCond;

  const bool subsonicCoastRecovered =
      conditionHeld(recoverySubsonicCoastCond, nowMs, recoverySubsonicCoastSinceMs, RECOVERY_CLASSIFY_CONFIRM_MS);
  const bool nearApogeeRecovered =
      conditionHeld(recoveryNearApogeeCond, nowMs, recoveryNearApogeeSinceMs, RECOVERY_CLASSIFY_CONFIRM_MS);
  const bool descendingBallisticRecovered =
      conditionHeld(recoveryDescendingBallisticCond, nowMs, recoveryDescendingBallisticSinceMs, RECOVERY_CLASSIFY_CONFIRM_MS);
  const bool underDrogueRecovered =
      conditionHeld(recoveryUnderDrogueCond, nowMs, recoveryUnderDrogueSinceMs, RECOVERY_CLASSIFY_CONFIRM_MS);
  const bool postFlightRecovered =
      conditionHeld(recoveryPostFlightCond, nowMs, recoveryPostFlightSinceMs, RECOVERY_POST_FLIGHT_CONFIRM_MS);

  if (subsonicCoastRecovered &&
      (flightState == FS_IDLE || flightState == FS_PAD || flightState == FS_ASCENT) &&
      !recoverySubsonicCoastLogged) {
    const FlightState fromState = flightState;
    markRecoveredLaunch(nowMs, relAlt);
    flightState = FS_SUBSONIC_COAST;
    recoverySubsonicCoastLogged = true;
    markBoosterBurnoutEvent(nowMs);
    logNandEventBinary(nowMs, EVT_RECOVERY_SUBSONIC_COAST, fromState, flightState, NAND_CLOSE_NONE);
    coastDetectSinceMs = 0;
    apogeeDetectSinceMs = 0;
  }

  if (nearApogeeRecovered &&
      (flightState == FS_IDLE || flightState == FS_PAD || flightState == FS_ASCENT || flightState == FS_COAST) &&
      !recoveryNearApogeeLogged) {
    const FlightState fromState = flightState;
    markRecoveredLaunch(nowMs, relAlt);
    flightState = FS_NEAR_APOGEE;
    recoveryNearApogeeLogged = true;
    markBoosterBurnoutEvent(nowMs);
    logNandEventBinary(nowMs, EVT_RECOVERY_NEAR_APOGEE, fromState, flightState, NAND_CLOSE_NONE);
    apogeeDetectSinceMs = 0;
  }

  if (descendingBallisticRecovered &&
      (flightState == FS_IDLE || flightState == FS_PAD || flightState == FS_ASCENT || flightState == FS_COAST) &&
      !recoveryDescendingBallisticLogged) {
    const FlightState fromState = flightState;
    markRecoveredLaunch(nowMs, relAlt);
    flightState = FS_DESCENT_BALLISTIC;
    flightFlags |= FLAG_APOGEE;
    if (tApogeeMs == 0) tApogeeMs = nowMs;
    apogeeAltM = max(maxAltM, relAlt);
    recoveryDescendingBallisticLogged = true;
    landedDetectSinceMs = 0;
    landedStillSinceMs = 0;
    logNandEventBinary(nowMs, EVT_RECOVERY_DESCENDING_BALLISTIC, fromState, flightState, NAND_CLOSE_NONE);
    markBoosterBurnoutEvent(nowMs);
    logApogeeChargeEvent(nowMs);
  }

  if (underDrogueRecovered && !recoveryUnderDrogueLogged) {
    const FlightState fromState = flightState;
    recoveryUnderDrogueLogged = true;
    flightFlags |= FLAG_UNDER_DROGUE;
    flightState = FS_UNDER_DROGUE;
    logNandEventBinary(nowMs, EVT_RECOVERY_UNDER_DROGUE, fromState, flightState, NAND_CLOSE_NONE);
  }

  const bool mainChargeCond = (flightState == FS_DESCENT_BALLISTIC ||
                              flightState == FS_UNDER_DROGUE ||
                              flightState == FS_DUAL_DEPLOY_APOGEE_LOGGED) &&
                              !mainChargeLogged &&
                              apogeeChargeLogged &&
                              ((flightFlags & FLAG_APOGEE) != 0) &&
                              (tApogeeMs != 0) &&
                              ((uint32_t)(nowMs - tApogeeMs) >= PYRO_MAIN_MIN_AFTER_APOGEE_MS) &&
                              (maxAltM >= (DUAL_DEPLOY_MAIN_ALT_M + DUAL_DEPLOY_MAIN_MIN_APOGEE_MARGIN_M)) &&
                              (relAlt <= DUAL_DEPLOY_MAIN_ALT_M) &&
                              (relAlt > LANDED_MAX_REL_ALT_M) &&
                              (velZ < APOGEE_VEL_MPS);
  if (mainChargeCond) {
    logMainChargeEvent(nowMs);
  }

  if (postFlightRecovered &&
      flightState != FS_LANDED &&
      flightState != FS_ABORT &&
      !recoveryPostFlightLogged) {
    const FlightState fromState = flightState;
    recoveryPostFlightLogged = true;
    flightFlags |= FLAG_POST_FLIGHT | FLAG_LANDED;
    flightState = FS_POST_FLIGHT_GROUND;
    logNandEventBinary(nowMs, EVT_RECOVERY_POST_FLIGHT_GROUND, fromState, flightState, NAND_CLOSE_NONE);
  }

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
        markBoosterBurnoutEvent(nowMs);
        apogeeDetectSinceMs = 0;
      }
      break;

    case FS_COAST:
    case FS_SUBSONIC_COAST:
    case FS_NEAR_APOGEE:
      if (relAlt > maxAltM) maxAltM = relAlt;
      if (velZ > maxVelMps) maxVelMps = velZ;
      if (conditionHeld(apogeeCond, nowMs, apogeeDetectSinceMs, APOGEE_CONFIRM_MS)) {
        flightState = FS_DESCENT_BALLISTIC;
        flightFlags |= FLAG_APOGEE;
        tApogeeMs = nowMs;
        apogeeAltM = maxAltM;
        landedDetectSinceMs = 0;
        landedStillSinceMs = 0;
        markBoosterBurnoutEvent(nowMs);
        logApogeeChargeEvent(nowMs);
#if ENABLE_LOW_ENERGY_DIRECT_LANDED
      } else if (maxAltM <= LOW_ENERGY_DIRECT_LANDED_MAX_ALT_M &&
                 conditionHeld(landedCond, nowMs, landedDetectSinceMs, LANDED_CONFIRM_MS)) {
        // Optional bench-only path for low-energy throws.
        flightState = FS_LANDED;
        flightFlags |= FLAG_LANDED;
#endif
      }
      break;

    case FS_DESCENT_BALLISTIC:
    case FS_UNDER_DROGUE:
    case FS_DUAL_DEPLOY_APOGEE_LOGGED:
    case FS_DUAL_DEPLOY_MAIN_LOGGED:
    case FS_POST_FLIGHT_GROUND:
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

  if ((flightFlags & FLAG_APOGEE) != 0) {
    logApogeeChargeEvent(nowMs);
  }
  updateHprStylePyroEvents(nowMs);

  pushRelAltHistory(nowMs, relAlt);
}
