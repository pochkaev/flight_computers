#pragma once

#include <Arduino.h>

enum FlightState : uint8_t {
  FS_IDLE = 0,
  FS_PAD,
  FS_ASCENT,
  FS_COAST,
  FS_SUBSONIC_COAST,
  FS_NEAR_APOGEE,
  FS_DESCENT_BALLISTIC,
  FS_UNDER_DROGUE,
  FS_DUAL_DEPLOY_APOGEE_LOGGED,
  FS_DUAL_DEPLOY_MAIN_LOGGED,
  FS_POST_FLIGHT_GROUND,
  FS_LANDED,
  FS_ABORT
};

enum BatteryPackType : uint8_t {
  BATT_PACK_UNKNOWN = 0,
  BATT_PACK_1S,
  BATT_PACK_2S
};

enum LedMode : uint8_t {
  LED_MODE_BOOT = 0,
  LED_MODE_SAFE,
  LED_MODE_ARMING,
  LED_MODE_READY,
  LED_MODE_SERVICE,
  LED_MODE_SUCCESS,
  LED_MODE_LANDED,
  LED_MODE_CRITICAL
};

static const uint16_t FLAG_LAUNCH = 1 << 0;
static const uint16_t FLAG_APOGEE = 1 << 1;
static const uint16_t FLAG_LANDED = 1 << 2;
static const uint16_t FLAG_DUAL_DEPLOY_APOGEE_LOG = 1 << 3;
static const uint16_t FLAG_DUAL_DEPLOY_MAIN_LOG = 1 << 4;
static const uint16_t FLAG_RECOVERY_CLASSIFIED = 1 << 5;
static const uint16_t FLAG_UNDER_DROGUE = 1 << 6;
static const uint16_t FLAG_POST_FLIGHT = 1 << 7;
static const uint16_t FLAG_PYRO_CH1_LOGGED = 1 << 8;
static const uint16_t FLAG_PYRO_CH2_LOGGED = 1 << 9;
static const uint16_t FLAG_PYRO_CH3_LOGGED = 1 << 10;
static const uint16_t FLAG_PYRO_CH4_LOGGED = 1 << 11;

static const uint16_t DIAG_BARO_GPS_DIVERGE = 1 << 0;
static const uint16_t DIAG_ATT_ACCEL_CORR = 1 << 1;
static const uint16_t DIAG_ATT_MAG_CORR = 1 << 2;
static const uint16_t DIAG_ATT_GYRO_ONLY = 1 << 3;
static const uint16_t DIAG_ARM_SWITCH_SAFE = 1 << 4;
static const uint16_t DIAG_ARM_SAFE_OBSERVED = 1 << 5;
static const uint16_t DIAG_LAUNCH_CANDIDATE = 1 << 6;
static const uint16_t DIAG_TOUCHDOWN_CANDIDATE = 1 << 7;

enum FlightEventType : uint8_t {
  EVT_STATE_CHANGE = 1,
  EVT_RECOVERY_SUBSONIC_COAST = 20,
  EVT_RECOVERY_NEAR_APOGEE = 21,
  EVT_RECOVERY_DESCENDING_BALLISTIC = 22,
  EVT_RECOVERY_UNDER_DROGUE = 23,
  EVT_RECOVERY_POST_FLIGHT_GROUND = 24,
  EVT_DUAL_DEPLOY_APOGEE_CHARGE_LOG = 40,
  EVT_DUAL_DEPLOY_MAIN_CHARGE_LOG = 41,
  EVT_PYRO_BOOSTER_SEPARATION_LOG = 42,
  EVT_PYRO_SUSTAINER_IGNITION_LOG = 43,
  EVT_PYRO_AIRSTART1_LOG = 44,
  EVT_PYRO_AIRSTART2_LOG = 45,
  EVT_PYRO_STAGING_INHIBIT_LOG = 46,
  EVT_PYRO_CHANNEL1_LOG = 50,
  EVT_PYRO_CHANNEL2_LOG = 51,
  EVT_PYRO_CHANNEL3_LOG = 52,
  EVT_PYRO_CHANNEL4_LOG = 53,
  EVT_PYRO_CHANNEL1_OUTPUT_ON = 60,
  EVT_PYRO_CHANNEL2_OUTPUT_ON = 61,
  EVT_PYRO_CHANNEL3_OUTPUT_ON = 62,
  EVT_PYRO_CHANNEL4_OUTPUT_ON = 63,
  EVT_PYRO_CHANNEL1_OUTPUT_OFF = 64,
  EVT_PYRO_CHANNEL2_OUTPUT_OFF = 65,
  EVT_PYRO_CHANNEL3_OUTPUT_OFF = 66,
  EVT_PYRO_CHANNEL4_OUTPUT_OFF = 67
};

enum NandCloseReason : uint8_t {
  NAND_CLOSE_NONE = 0,
  NAND_CLOSE_LANDED = 1,
  NAND_CLOSE_ABORT = 2,
  NAND_CLOSE_SERVICE = 3,
  NAND_CLOSE_ERROR = 4
};

extern volatile bool loraTxBusy;
extern bool loraOk;
extern bool imuOk;
extern bool baroOk;
extern bool serviceModeActive;
extern bool serviceModeSuccess;
extern bool serviceModeFailed;

extern LedMode ledMode;
extern uint32_t ledModeSinceMs;
extern bool finderBeeperActive;
extern bool buttonPrevPressed;
extern bool buttonResetFired;
extern uint32_t buttonDownMs;
extern uint32_t finderBeeperStartMs;
extern uint32_t beepPatternStartMs;
extern uint8_t beepPattern;

extern char rocketName[16];

extern FlightState flightState;
extern FlightState lastFlightState;
extern uint16_t flightFlags;

extern bool haveAlt;
extern bool haveGoodFix;
extern bool haveImuEstimate;

extern int lastBattRaw;
extern float lastBattPinV;
extern float rocketBattRawV;
extern BatteryPackType batteryPack;
extern bool batteryWarn;
extern bool batteryCrit;

extern float filtAlt;
extern float filtTempC;
extern float filtPressurePa;
extern float velZ;
extern float baseAltM;
extern float maxAltM;
extern float maxVelMps;
extern float apogeeAltM;
extern float lastAltRaw;
extern float velRefAltM;
extern uint32_t velRefMs;
extern float rocketBattV;

extern float last_ax;
extern float last_ay;
extern float last_az;
extern float last_gx;
extern float last_gy;
extern float last_gz;
extern float last_mx;
extern float last_my;
extern float last_mz;
extern float roll;
extern float pitch;
extern float yaw;
extern float attitudeQw;
extern float attitudeQx;
extern float attitudeQy;
extern float attitudeQz;
extern bool attitudeAccelCorrectionActive;
extern bool attitudeMagCorrectionActive;
extern bool attitudeGyroOnly;

extern bool gpsHasFix;
extern uint8_t gpsFixType;
extern uint8_t gpsSats;
extern float gpsHdop;
extern double gpsLatDeg;
extern double gpsLonDeg;
extern float gpsAltM;
extern float gpsSpeedMps;
extern float gpsBaseAltM;
extern float gpsRelAltM;
extern float baroGpsDeltaM;
extern uint16_t diagFlags;
extern bool haveGpsBaseAlt;
extern bool baroGpsDiverged;
extern double lastFixLatDeg;
extern double lastFixLonDeg;
extern float lastFixAltM;
extern uint32_t lastFixTimeMs;

extern uint32_t flightSeq;
extern uint32_t navSeq;
extern uint32_t statusSeq;
extern uint32_t identitySeq;
extern uint32_t pyroConfigSeq;
extern uint32_t pyroEventSeq;
extern uint32_t tLaunchMs;
extern uint32_t tApogeeMs;
extern uint32_t launchDetectSinceMs;
extern uint32_t coastDetectSinceMs;
extern uint32_t apogeeDetectSinceMs;
extern uint32_t landedDetectSinceMs;
extern uint32_t landedStillSinceMs;
extern uint32_t recoverySubsonicCoastSinceMs;
extern uint32_t recoveryNearApogeeSinceMs;
extern uint32_t recoveryDescendingBallisticSinceMs;
extern uint32_t recoveryUnderDrogueSinceMs;
extern uint32_t recoveryPostFlightSinceMs;
extern uint32_t launchArmStillSinceMs;
extern uint32_t launchArmMotionSinceMs;
extern uint32_t launchArmedMs;
extern uint32_t lastGpsDataMs;
extern uint32_t lastImuSampleMs;
extern uint32_t lastBaroSampleMs;
extern uint32_t lastGpsFixMs;
extern uint32_t lastGpsBaseUpdateMs;
extern uint32_t padSettleStartMs;
extern bool launchArmed;
extern bool armSwitchSafe;
extern bool armSafeObservedSinceBoot;
extern bool launchCandidateActive;
extern bool touchdownCandidateActive;
extern bool touchdownImpactQualified;
extern uint32_t armSwitchRawSinceMs;
extern uint32_t armSwitchSafeSinceMs;
extern uint32_t launchCandidateSinceMs;
extern uint32_t launchImuSinceMs;
extern uint32_t touchdownCandidateSinceMs;
extern uint32_t touchdownSoftSinceMs;
extern uint32_t touchdownStableSinceMs;
extern bool apogeeChargeLogged;
extern bool mainChargeLogged;
extern bool boosterBurnoutLogged;
extern bool boosterSeparationLogged;
extern bool sustainerIgnitionLogged;
extern bool airStart1Logged;
extern bool airStart2Logged;
extern bool pyroChannelActive[4];
extern bool recoverySubsonicCoastLogged;
extern bool recoveryNearApogeeLogged;
extern bool recoveryDescendingBallisticLogged;
extern bool recoveryUnderDrogueLogged;
extern bool recoveryPostFlightLogged;
extern uint32_t tBoosterBurnoutMs;
extern uint32_t tBoosterSeparationMs;
extern uint32_t tAirStart1Ms;
extern uint32_t pyroChannelStartMs[4];

static const uint8_t REL_ALT_HISTORY_COUNT = 8;
extern float relAltHistoryM[REL_ALT_HISTORY_COUNT];
extern uint32_t relAltHistoryMs[REL_ALT_HISTORY_COUNT];
extern uint8_t relAltHistoryIndex;
extern bool relAltHistoryFilled;
