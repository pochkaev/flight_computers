#include "state.h"

#include "config.h"

#include <math.h>

volatile bool loraTxBusy = false;
bool loraOk = false;
bool imuOk = false;
bool baroOk = false;
bool serviceModeActive = false;
bool serviceModeSuccess = false;
bool serviceModeFailed = false;

LedMode ledMode = LED_MODE_BOOT;
uint32_t ledModeSinceMs = 0;
bool finderBeeperActive = false;
bool buttonPrevPressed = false;
bool buttonResetFired = false;
uint32_t buttonDownMs = 0;
uint32_t finderBeeperStartMs = 0;
uint32_t beepPatternStartMs = 0;
uint8_t beepPattern = 0;

char rocketName[16] = DEFAULT_ROCKET_NAME;

FlightState flightState = FS_IDLE;
FlightState lastFlightState = FS_IDLE;
uint16_t flightFlags = 0;

bool haveAlt = false;
bool haveGoodFix = false;
bool haveImuEstimate = false;

int lastBattRaw = 0;
float lastBattPinV = 0.0f;
float rocketBattRawV = 0.0f;
BatteryPackType batteryPack = BATT_PACK_UNKNOWN;
bool batteryWarn = false;
bool batteryCrit = false;

float filtAlt = 0.0f;
float filtTempC = 0.0f;
float filtPressurePa = 0.0f;
float velZ = 0.0f;
float baseAltM = NAN;
float maxAltM = 0.0f;
float maxVelMps = 0.0f;
float apogeeAltM = NAN;
float lastAltRaw = 0.0f;
float velRefAltM = 0.0f;
uint32_t velRefMs = 0;
float rocketBattV = 0.0f;

float last_ax = 0.0f;
float last_ay = 0.0f;
float last_az = 9.80665f;
float last_gx = 0.0f;
float last_gy = 0.0f;
float last_gz = 0.0f;
float last_mx = 0.0f;
float last_my = 0.0f;
float last_mz = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float attitudeQw = 1.0f;
float attitudeQx = 0.0f;
float attitudeQy = 0.0f;
float attitudeQz = 0.0f;
bool attitudeAccelCorrectionActive = false;
bool attitudeMagCorrectionActive = false;
bool attitudeGyroOnly = true;

bool gpsHasFix = false;
uint8_t gpsFixType = 0;
uint8_t gpsSats = 0;
float gpsHdop = 99.9f;
double gpsLatDeg = 0.0;
double gpsLonDeg = 0.0;
float gpsAltM = 0.0f;
float gpsSpeedMps = 0.0f;
float gpsBaseAltM = NAN;
float gpsRelAltM = 0.0f;
float baroGpsDeltaM = NAN;
uint16_t diagFlags = 0;
bool haveGpsBaseAlt = false;
bool baroGpsDiverged = false;
double lastFixLatDeg = 0.0;
double lastFixLonDeg = 0.0;
float lastFixAltM = 0.0f;
uint32_t lastFixTimeMs = 0;

uint32_t flightSeq = 0;
uint32_t navSeq = 0;
uint32_t statusSeq = 0;
uint32_t identitySeq = 0;
uint32_t pyroConfigSeq = 0;
uint32_t pyroEventSeq = 0;
uint32_t tLaunchMs = 0;
uint32_t tApogeeMs = 0;
uint32_t launchDetectSinceMs = 0;
uint32_t coastDetectSinceMs = 0;
uint32_t apogeeDetectSinceMs = 0;
uint32_t landedDetectSinceMs = 0;
uint32_t landedStillSinceMs = 0;
uint32_t recoverySubsonicCoastSinceMs = 0;
uint32_t recoveryNearApogeeSinceMs = 0;
uint32_t recoveryDescendingBallisticSinceMs = 0;
uint32_t recoveryUnderDrogueSinceMs = 0;
uint32_t recoveryPostFlightSinceMs = 0;
uint32_t launchArmStillSinceMs = 0;
uint32_t launchArmMotionSinceMs = 0;
uint32_t launchArmedMs = 0;
uint32_t armSwitchRawSinceMs = 0;
uint32_t armSwitchSafeSinceMs = 0;
uint32_t launchCandidateSinceMs = 0;
uint32_t launchImuSinceMs = 0;
uint32_t touchdownCandidateSinceMs = 0;
uint32_t touchdownSoftSinceMs = 0;
uint32_t touchdownStableSinceMs = 0;
uint32_t lastGpsDataMs = 0;
uint32_t lastImuSampleMs = 0;
uint32_t lastBaroSampleMs = 0;
uint32_t lastGpsFixMs = 0;
uint32_t lastGpsBaseUpdateMs = 0;
uint32_t padSettleStartMs = 0;
bool launchArmed = false;
bool armSwitchSafe = false;
bool armSafeObservedSinceBoot = false;
bool launchCandidateActive = false;
bool touchdownCandidateActive = false;
bool touchdownImpactQualified = false;
bool apogeeChargeLogged = false;
bool mainChargeLogged = false;
bool boosterBurnoutLogged = false;
bool boosterSeparationLogged = false;
bool sustainerIgnitionLogged = false;
bool airStart1Logged = false;
bool airStart2Logged = false;
bool pyroChannelActive[4] = {};
bool recoverySubsonicCoastLogged = false;
bool recoveryNearApogeeLogged = false;
bool recoveryDescendingBallisticLogged = false;
bool recoveryUnderDrogueLogged = false;
bool recoveryPostFlightLogged = false;
uint32_t tBoosterBurnoutMs = 0;
uint32_t tBoosterSeparationMs = 0;
uint32_t tAirStart1Ms = 0;
uint32_t pyroChannelStartMs[4] = {};

float relAltHistoryM[REL_ALT_HISTORY_COUNT] = {};
uint32_t relAltHistoryMs[REL_ALT_HISTORY_COUNT] = {};
uint8_t relAltHistoryIndex = 0;
bool relAltHistoryFilled = false;
