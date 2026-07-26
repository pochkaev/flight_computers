#include "flight.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "pyro.h"
#include "sensors.h"
#include "state.h"
#include "storage.h"
#include "ui.h"
#include "settings.h"

static float launchImuDeltaVMps = 0.0f;
static uint32_t launchImuIntegratedSampleMs = 0;

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
  launchCandidateActive = false;
  launchCandidateSinceMs = 0;
  launchImuSinceMs = 0;
  launchImuDeltaVMps = 0.0f;
  launchImuIntegratedSampleMs = 0;
}

void updateArmSwitchTask(uint32_t nowMs) {
  const bool rawSafe = digitalRead(ARM_SWITCH_PIN) == ARM_SWITCH_SAFE_LEVEL;
  static bool lastRawSafe = rawSafe;

  if (rawSafe != lastRawSafe) {
    lastRawSafe = rawSafe;
    armSwitchRawSinceMs = nowMs;
  } else if (armSwitchRawSinceMs == 0) {
    armSwitchRawSinceMs = nowMs;
  }

  if ((uint32_t)(nowMs - armSwitchRawSinceMs) >= ARM_SWITCH_DEBOUNCE_MS) {
    armSwitchSafe = rawSafe;
  }

  if (armSwitchSafe) {
    if (armSwitchSafeSinceMs == 0) armSwitchSafeSinceMs = nowMs;
    armSafeObservedSinceBoot = true;
    if ((flightState == FS_LANDED || flightState == FS_POST_FLIGHT_GROUND) &&
        (uint32_t)(nowMs - armSwitchSafeSinceMs) >= ARM_SWITCH_DISARM_MS) {
      setFinderBeeper(false);
    }
  } else {
    armSwitchSafeSinceMs = 0;
  }

  if (armSwitchSafe) diagFlags |= DIAG_ARM_SWITCH_SAFE;
  else diagFlags &= (uint16_t)~DIAG_ARM_SWITCH_SAFE;
  if (armSafeObservedSinceBoot) diagFlags |= DIAG_ARM_SAFE_OBSERVED;
  else diagFlags &= (uint16_t)~DIAG_ARM_SAFE_OBSERVED;
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
    return LAUNCH_STATUS_BOOT_WAIT;
  }
  if (!armSafeObservedSinceBoot) {
    return LAUNCH_STATUS_SAFE_REQUIRED;
  }
  if (armSwitchSafe) {
    return LAUNCH_STATUS_SAFE;
  }
  if (launchCandidateActive) {
    return LAUNCH_STATUS_LAUNCH_CHECK;
  }
  if (launchArmed) {
    return LAUNCH_STATUS_READY;
  }

  const bool padSettled = padSettleStartMs != 0 &&
                          (uint32_t)(nowMs - padSettleStartMs) >= LAUNCH_PAD_SETTLE_MS;
  if (!padSettled) {
    const uint32_t elapsedMs = padSettleStartMs == 0 ? 0 : (uint32_t)(nowMs - padSettleStartMs);
    waitSecondsOut = secondsCeilRemaining(elapsedMs, LAUNCH_PAD_SETTLE_MS);
    return LAUNCH_STATUS_PAD_SETTLE;
  }
  if (!baroOk || !imuOk || !isBaroFresh() || !isImuFresh()) {
    return LAUNCH_STATUS_SENSOR_FAULT;
  }
  if (!logOk) {
    return LAUNCH_STATUS_LOG_FAULT;
  }
  if (batteryCrit) {
    return LAUNCH_STATUS_BATT_CRIT;
  }

  const float accMagG =
      sqrtf(last_ax * last_ax + last_ay * last_ay + last_az * last_az) / 9.80665f;
  const float axialAccelG = (-last_az) / 9.80665f;
  const float transverseAccelG =
      sqrtf(last_ax * last_ax + last_ay * last_ay) / 9.80665f;
  const float gyroMagDps =
      sqrtf(last_gx * last_gx + last_gy * last_gy + last_gz * last_gz);
  const bool vertical =
      axialAccelG >= LAUNCH_PAD_VERTICAL_MIN_G &&
      axialAccelG <= LAUNCH_PAD_VERTICAL_MAX_G &&
      transverseAccelG <= LAUNCH_PAD_TRANSVERSE_MAX_G;
  if (!vertical) {
    return LAUNCH_STATUS_HOLD_VERTICAL;
  }
  const bool still =
      fabsf(accMagG - 1.0f) <= LAUNCH_PAD_STILL_ACCEL_ERR_G &&
      gyroMagDps <= LAUNCH_PAD_STILL_GYRO_DPS;
  if (!still) {
    return LAUNCH_STATUS_HOLD_STILL;
  }

  if (launchArmStillSinceMs != 0) {
    waitSecondsOut = secondsCeilRemaining((uint32_t)(nowMs - launchArmStillSinceMs), LAUNCH_PAD_STILL_ARM_MS);
  } else {
    waitSecondsOut = LAUNCH_PAD_STILL_ARM_MS / 1000u;
  }
  return LAUNCH_STATUS_ARMING;
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

void updateFlightStateTask(uint32_t nowMs) {
  if (!haveAlt) return;

  float relAlt = currentBaroRelAltM();
  float accMagG = sqrtf(last_ax * last_ax + last_ay * last_ay + last_az * last_az) / 9.80665f;
  const float axialAccelG = (-last_az) / 9.80665f;
  float gyroMagDps = sqrtf(last_gx * last_gx + last_gy * last_gy + last_gz * last_gz);
  const bool baroEvidenceCurrent =
      lastBaroSampleMs != 0 &&
      (uint32_t)(nowMs - lastBaroSampleMs) <= BARO_STATE_EVIDENCE_MAX_AGE_MS;
  float trendRelAltM = relAlt;
  const bool launchTrendOk = baroEvidenceCurrent &&
                             relAltAtLeastAgo(nowMs, LAUNCH_TREND_MS, trendRelAltM) &&
                             ((relAlt - trendRelAltM) >= LAUNCH_TREND_MIN_M);
  const bool padSettled = padSettleStartMs != 0 &&
                          ((uint32_t)(nowMs - padSettleStartMs) >= LAUNCH_PAD_SETTLE_MS);
  const bool launchStillCond = fabsf(accMagG - 1.0f) <= LAUNCH_PAD_STILL_ACCEL_ERR_G &&
                               gyroMagDps <= LAUNCH_PAD_STILL_GYRO_DPS;
  const float transverseAccelG = sqrtf(last_ax * last_ax + last_ay * last_ay) / 9.80665f;
  const bool launchVerticalCond = axialAccelG >= LAUNCH_PAD_VERTICAL_MIN_G &&
                                  axialAccelG <= LAUNCH_PAD_VERTICAL_MAX_G &&
                                  transverseAccelG <= LAUNCH_PAD_TRANSVERSE_MAX_G;
  const bool launchHealthCond = baroOk && imuOk && isBaroFresh() && isImuFresh() &&
                                logOk && !batteryCrit;
  const bool powerOnInhibitDone = nowMs >= LAUNCH_POWERON_INHIBIT_MS;
  // A cached high-acceleration sample must not satisfy the hold timer after
  // the IMU stops updating. The age cap is shorter than the confirmation time.
  const bool launchImuCond =
      axialAccelG >= rocketSettings.launchAccelG &&
      lastImuSampleMs != 0 &&
      (uint32_t)(nowMs - lastImuSampleMs) <= LAUNCH_IMU_EVIDENCE_MAX_AGE_MS;
  const bool launchAccelCond = launchTrendOk &&
                               (relAlt > LAUNCH_REL_ALT_M) &&
                               (velZ > LAUNCH_VEL_MPS) &&
                               (axialAccelG >= rocketSettings.launchAccelG);
  const bool launchBaroCond = launchTrendOk &&
                              (relAlt > rocketSettings.launchBaroAltM) &&
                              (velZ > rocketSettings.launchBaroVelMps);
  const bool launchObviousCond = baroEvidenceCurrent &&
                                 (relAlt > LAUNCH_OBVIOUS_REL_ALT_M) &&
                                 (velZ > LAUNCH_OBVIOUS_VEL_MPS);
  const bool launchKinematicsCond = launchAccelCond || launchBaroCond || launchObviousCond;
  const bool armRequestValid = armSafeObservedSinceBoot && !armSwitchSafe;
  if (flightState == FS_IDLE || flightState == FS_PAD) {
    if ((!armRequestValid || !powerOnInhibitDone || !padSettled) && !launchCandidateActive) {
      clearLaunchArmGate();
    } else if (!launchArmed) {
      const bool padVerifyCond = launchStillCond && launchVerticalCond && launchHealthCond;
      if (conditionHeld(padVerifyCond, nowMs, launchArmStillSinceMs, LAUNCH_PAD_STILL_ARM_MS)) {
        setLaunchArmGate(nowMs, lastAltRaw);
        relAlt = currentBaroRelAltM();
      }
    }
  }
  const bool launchReady = powerOnInhibitDone && padSettled && launchArmed;
  if (launchReady && !launchCandidateActive && (launchImuCond || launchKinematicsCond)) {
    launchCandidateActive = true;
    launchCandidateSinceMs = nowMs;
    launchDetectSinceMs = 0;
    launchImuSinceMs = 0;
    launchImuDeltaVMps = 0.0f;
    launchImuIntegratedSampleMs = 0;
  }
  if (launchCandidateActive &&
      lastImuSampleMs != 0 &&
      lastImuSampleMs != launchImuIntegratedSampleMs) {
    if (!launchImuCond) {
      // Separate bumps must not accumulate into a synthetic motor impulse.
      launchImuDeltaVMps = 0.0f;
    } else if (launchImuIntegratedSampleMs != 0) {
      const uint32_t sampleDtMs = lastImuSampleMs - launchImuIntegratedSampleMs;
      if (sampleDtMs <= LAUNCH_IMU_EVIDENCE_MAX_AGE_MS) {
        const float netAxialMps2 =
            max(0.0f, axialAccelG - 1.0f) * 9.80665f;
        launchImuDeltaVMps += netAxialMps2 * (sampleDtMs * 0.001f);
      } else {
        launchImuDeltaVMps = 0.0f;
      }
    }
    launchImuIntegratedSampleMs = lastImuSampleMs;
  }
  const bool launchImuConfirmed =
      launchCandidateActive &&
      conditionHeld(launchImuCond, nowMs, launchImuSinceMs,
                    rocketSettings.launchImuConfirmMs) &&
      launchImuDeltaVMps >= rocketSettings.launchImuDeltaVMps;
  const bool launchBaroConfirmed =
      launchCandidateActive &&
      conditionHeld(launchKinematicsCond, nowMs, launchDetectSinceMs, LAUNCH_CONFIRM_MS);
  const bool launchConfirmed =
      launchImuConfirmed || launchBaroConfirmed;
  if (launchCandidateActive && !launchConfirmed &&
      (uint32_t)(nowMs - launchCandidateSinceMs) >= LAUNCH_CANDIDATE_TIMEOUT_MS) {
    launchCandidateActive = false;
    launchCandidateSinceMs = 0;
    launchDetectSinceMs = 0;
    launchImuSinceMs = 0;
    launchImuDeltaVMps = 0.0f;
    launchImuIntegratedSampleMs = 0;
  }
  const bool coastCond = baroEvidenceCurrent &&
                         lastBaroSampleMs > tLaunchMs &&
                         (velZ < COAST_VEL_MPS) &&
                         ((uint32_t)(nowMs - tLaunchMs) > COAST_MIN_AFTER_LAUNCH_MS);
  const bool apogeeCond = baroEvidenceCurrent &&
                          (velZ < APOGEE_VEL_MPS) &&
                          (relAlt > rocketSettings.apogeeMinAltM);
  const bool stillCond = fabsf(accMagG - 1.0f) <= LANDED_STILL_ACCEL_ERR_G &&
                         gyroMagDps <= LANDED_STILL_GYRO_DPS;
  const bool landedBaseCond = baroEvidenceCurrent &&
                          (fabsf(velZ) < LANDED_ABS_VEL_MPS) &&
                          (relAlt < LANDED_MAX_REL_ALT_M) &&
                          ((uint32_t)(nowMs - tLaunchMs) > LANDED_MIN_AFTER_LAUNCH_MS);
  const bool landedStillCond = conditionHeld(stillCond, nowMs, landedStillSinceMs, LANDED_STILL_CONFIRM_MS);
  const bool landedCond = landedBaseCond && landedStillCond;
  const bool descentState = flightState == FS_DESCENT_BALLISTIC ||
                            flightState == FS_UNDER_DROGUE ||
                            flightState == FS_DUAL_DEPLOY_APOGEE_LOGGED ||
                            flightState == FS_DUAL_DEPLOY_MAIN_LOGGED ||
                            flightState == FS_POST_FLIGHT_GROUND;
  const bool touchdownTimeOk = tApogeeMs != 0 &&
                               (uint32_t)(nowMs - tApogeeMs) >= TOUCHDOWN_MIN_AFTER_APOGEE_MS;
  const bool touchdownImpact = descentState && touchdownTimeOk &&
                               accMagG >= TOUCHDOWN_IMPACT_G;
  const bool touchdownSoft = baroEvidenceCurrent && descentState &&
                             relAlt < LANDED_MAX_REL_ALT_M &&
                             fabsf(velZ) < TOUCHDOWN_SOFT_VEL_MPS;
  const bool touchdownSoftConfirmed =
      conditionHeld(touchdownSoft, nowMs, touchdownSoftSinceMs, TOUCHDOWN_SOFT_CONFIRM_MS);
  if (!touchdownCandidateActive && (touchdownImpact || touchdownSoftConfirmed)) {
    touchdownCandidateActive = true;
    touchdownImpactQualified = touchdownImpact;
    touchdownCandidateSinceMs = nowMs;
    touchdownStableSinceMs = 0;
    setFinderBeeper(true);
  }
  if (touchdownCandidateActive && touchdownImpact) {
    touchdownImpactQualified = true;
  }
  const bool touchdownFinalZone = touchdownImpactQualified ||
                                  relAlt < LOW_ENERGY_DIRECT_LANDED_MAX_ALT_M;
  const bool touchdownStable = baroEvidenceCurrent && touchdownFinalZone &&
                               fabsf(velZ) < TOUCHDOWN_SOFT_VEL_MPS;
  const bool touchdownPostFlight =
      touchdownCandidateActive &&
      conditionHeld(touchdownStable, nowMs, touchdownStableSinceMs,
                    TOUCHDOWN_POST_FLIGHT_CONFIRM_MS);

  const bool recoverySubsonicCoastCond = launchTrendOk &&
                                         (relAlt > RECOVERY_SUBSONIC_COAST_REL_ALT_M) &&
                                         (velZ > RECOVERY_SUBSONIC_COAST_VEL_MPS);
  const bool recoveryNearApogeeCond = baroEvidenceCurrent &&
                                      (relAlt > RECOVERY_NEAR_APOGEE_REL_ALT_M) &&
                                      (fabsf(velZ) < RECOVERY_NEAR_APOGEE_ABS_VEL_MPS) &&
                                      ((flightFlags & FLAG_LAUNCH) ||
                                       maxAltM > RECOVERY_NEAR_APOGEE_REL_ALT_M ||
                                       relAlt > RECOVERY_NEAR_APOGEE_FALLBACK_REL_ALT_M);
  const bool recoveryDescendingBallisticCond = baroEvidenceCurrent &&
                                               (relAlt > RECOVERY_DESCENT_REL_ALT_M) &&
                                               (velZ < RECOVERY_DESCENT_VEL_MPS);
  const bool recoveryUnderDrogueCond = baroEvidenceCurrent &&
                                       (flightState == FS_DESCENT_BALLISTIC ||
                                        flightState == FS_DUAL_DEPLOY_APOGEE_LOGGED ||
                                        flightState == FS_DUAL_DEPLOY_MAIN_LOGGED) &&
                                       (relAlt > RECOVERY_UNDER_DROGUE_REL_ALT_M) &&
                                       (velZ < APOGEE_VEL_MPS) &&
                                       (velZ > RECOVERY_UNDER_DROGUE_FAST_VEL_MPS);
  const bool recoveryPostFlightCond = baroEvidenceCurrent &&
                                      ((flightFlags & FLAG_LAUNCH) || maxAltM > RECOVERY_POST_FLIGHT_MAX_REL_ALT_M) &&
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

  const bool mainChargeCond = baroEvidenceCurrent &&
                              (flightState == FS_DESCENT_BALLISTIC ||
                              flightState == FS_UNDER_DROGUE ||
                              flightState == FS_DUAL_DEPLOY_APOGEE_LOGGED) &&
                              !mainChargeLogged &&
                              apogeeChargeLogged &&
                              ((flightFlags & FLAG_APOGEE) != 0) &&
                              (tApogeeMs != 0) &&
                              ((uint32_t)(nowMs - tApogeeMs) >= PYRO_MAIN_MIN_AFTER_APOGEE_MS) &&
                              (maxAltM >= (rocketSettings.mainAltM + DUAL_DEPLOY_MAIN_MIN_APOGEE_MARGIN_M)) &&
                              (relAlt <= rocketSettings.mainAltM) &&
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
      if (launchConfirmed) {
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
        launchCandidateActive = false;
        launchCandidateSinceMs = 0;
        launchImuSinceMs = 0;
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
      if (touchdownPostFlight && flightState != FS_POST_FLIGHT_GROUND) {
        flightState = FS_POST_FLIGHT_GROUND;
        flightFlags |= FLAG_POST_FLIGHT;
      }
      if ((touchdownCandidateActive &&
           (uint32_t)(nowMs - touchdownCandidateSinceMs) >= TOUCHDOWN_LANDED_CONFIRM_MS &&
           touchdownStable) ||
          conditionHeld(landedCond, nowMs, landedDetectSinceMs, LANDED_CONFIRM_MS)) {
        flightState = FS_LANDED;
        flightFlags |= FLAG_POST_FLIGHT | FLAG_LANDED;
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

  if (launchCandidateActive) diagFlags |= DIAG_LAUNCH_CANDIDATE;
  else diagFlags &= (uint16_t)~DIAG_LAUNCH_CANDIDATE;
  if (touchdownCandidateActive) diagFlags |= DIAG_TOUCHDOWN_CANDIDATE;
  else diagFlags &= (uint16_t)~DIAG_TOUCHDOWN_CANDIDATE;

  // Preserve a barometer-sample history for altitude trend checks, while
  // allowing IMU launch confirmation and time-based state work to run at the
  // main-loop rate even when a pressure conversion is waiting or has failed.
  static uint32_t lastHistoryBaroSampleMs = 0;
  if (lastBaroSampleMs != 0 && lastBaroSampleMs != lastHistoryBaroSampleMs) {
    lastHistoryBaroSampleMs = lastBaroSampleMs;
    pushRelAltHistory(lastBaroSampleMs, relAlt);
  }
}
