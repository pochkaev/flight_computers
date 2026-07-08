#include "config.h"
#include "battery.h"
#include "button.h"
#include "state.h"
#include "storage.h"
#include "sensors.h"
#include "flight.h"
#include "telemetry.h"
#include "pyro.h"
#include "ui.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <math.h>

void onLoraTxDone() {
  loraTxBusy = false;
}

static void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("LoRa: not found");
    loraOk = false;
    return;
  }
  LoRa.setSPIFrequency(LORA_SPI_FREQ_HZ);
  LoRa.onTxDone(onLoraTxDone);
  loraOk = true;
  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("LoRa: OK");
}

static void printDebugStatus() {
  if (SERIAL_DEBUG_LEVEL <= 0) return;

  float relAlt = currentBaroRelAltM();

  if (SERIAL_DEBUG_LEVEL == 1) {
    Serial.print("state=");
    Serial.print((int)flightState);
    Serial.print(" alt=");
    Serial.print(filtAlt, 2);
    Serial.print(" vel=");
    Serial.print(velZ, 2);
    Serial.print(" gpsFix=");
    Serial.print((int)gpsFixType);
    Serial.print(" sats=");
    Serial.print((int)gpsSats);
    Serial.print(" batt=");
    Serial.print(rocketBattV, 2);
    Serial.print(" sd=");
    Serial.print(sdOk ? "1" : "0");
    Serial.print(" nand=");
    Serial.println(nandOk ? "1" : "0");
    return;
  }

  Serial.print("dbg ms=");
  Serial.print(millis());
  Serial.print(" state=");
  Serial.print((int)flightState);
  Serial.print(" flags=");
  Serial.print((unsigned int)flightFlags);
  Serial.print(" alt=");
  Serial.print(filtAlt, 2);
  Serial.print(" relAlt=");
  Serial.print(relAlt, 2);
  Serial.print(" maxAlt=");
  Serial.print(maxAltM, 2);
  Serial.print(" apogeeAlt=");
  if (isnan(apogeeAltM)) {
    Serial.print("nan");
  } else {
    Serial.print(apogeeAltM, 2);
  }
  Serial.print(" vel=");
  Serial.print(velZ, 2);
  Serial.print(" tempC=");
  Serial.print(filtTempC, 2);
  Serial.print(" presPa=");
  Serial.print(filtPressurePa, 1);
  Serial.print(" ax=");
  Serial.print(last_ax, 2);
  Serial.print(" ay=");
  Serial.print(last_ay, 2);
  Serial.print(" az=");
  Serial.print(last_az, 2);
  Serial.print(" gx=");
  Serial.print(last_gx, 2);
  Serial.print(" gy=");
  Serial.print(last_gy, 2);
  Serial.print(" gz=");
  Serial.print(last_gz, 2);
  Serial.print(" mx=");
  Serial.print(last_mx, 2);
  Serial.print(" my=");
  Serial.print(last_my, 2);
  Serial.print(" mz=");
  Serial.print(last_mz, 2);
  Serial.print(" rollDeg=");
  Serial.print(roll * 57.2957795f, 2);
  Serial.print(" pitchDeg=");
  Serial.print(pitch * 57.2957795f, 2);
  Serial.print(" yawDeg=");
  Serial.print(yaw * 57.2957795f, 2);
  Serial.print(" gpsFix=");
  Serial.print((int)gpsFixType);
  Serial.print(" sats=");
  Serial.print((int)gpsSats);
  Serial.print(" hdop=");
  Serial.print(gpsHdop, 1);
  Serial.print(" lat=");
  Serial.print(gpsLatDeg, 7);
  Serial.print(" lon=");
  Serial.print(gpsLonDeg, 7);
  Serial.print(" gpsAlt=");
  Serial.print(gpsAltM, 2);
  Serial.print(" gpsRelAlt=");
  Serial.print(gpsRelAltM, 2);
  Serial.print(" gpsBaseAlt=");
  if (haveGpsBaseAlt) {
    Serial.print(gpsBaseAltM, 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" baroGpsDelta=");
  if (isfinite(baroGpsDeltaM)) {
    Serial.print(baroGpsDeltaM, 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" baroGpsDiv=");
  Serial.print(baroGpsDiverged ? "1" : "0");
  Serial.print(" gpsSpd=");
  Serial.print(gpsSpeedMps, 2);
  Serial.print(" gpsChars=");
  Serial.print((unsigned long)gps.charsProcessed());
  Serial.print(" gpsPass=");
  Serial.print((unsigned long)gps.passedChecksum());
  Serial.print(" gpsFail=");
  Serial.print((unsigned long)gps.failedChecksum());
  Serial.print(" gpsLocValid=");
  Serial.print(gps.location.isValid() ? "1" : "0");
  Serial.print(" gpsAltValid=");
  Serial.print(gps.altitude.isValid() ? "1" : "0");
  Serial.print(" gpsDateValid=");
  Serial.print(gps.date.isValid() ? "1" : "0");
  Serial.print(" gpsTimeValid=");
  Serial.print(gps.time.isValid() ? "1" : "0");
  Serial.print(" gpsFixAgeMs=");
  if (gpsHasFix) {
    Serial.print("0");
  } else if (lastGpsFixMs != 0) {
    Serial.print((unsigned long)(millis() - lastGpsFixMs));
  } else {
    Serial.print("-1");
  }
  Serial.print(" battRawV=");
  Serial.print(rocketBattRawV, 2);
  Serial.print(" batt=");
  Serial.print(rocketBattV, 2);
  Serial.print(" pack=");
  Serial.print(batteryPackName());
  Serial.print(" bwarn=");
  Serial.print(batteryWarn ? "1" : "0");
  Serial.print(" bcrit=");
  Serial.print(batteryCrit ? "1" : "0");
  Serial.print(" battRaw=");
  Serial.print(lastBattRaw);
  Serial.print(" battPin=");
  Serial.print(lastBattPinV, 3);
  Serial.print(" health=");
  Serial.print(buildHealthFlags());
  Serial.print(" lora=");
  Serial.print(loraOk ? "1" : "0");
  Serial.print(" baro=");
  Serial.print(baroOk ? "1" : "0");
  Serial.print(" imu=");
  Serial.print(imuOk ? "1" : "0");
  Serial.print(" gps=");
  Serial.print(gpsHasFix ? "1" : "0");
  Serial.print(" gpsFresh=");
  Serial.print(isGpsFresh() ? "1" : "0");
  Serial.print(" sd=");
  Serial.print(sdOk ? "1" : "0");
  Serial.print(" sdlog=");
  Serial.print(sdLogOk ? "1" : "0");
  Serial.print(" nlog=");
  Serial.print(nandLogOk ? "1" : "0");
  Serial.print(" log=");
  Serial.print(logOk ? "1" : "0");
  Serial.print(" nand=");
  Serial.print(nandOk ? "1" : "0");
  Serial.print(" imuFresh=");
  Serial.print(isImuFresh() ? "1" : "0");
  Serial.print(" baroFresh=");
  Serial.print(isBaroFresh() ? "1" : "0");
  uint16_t launchWaitS = 0;
  uint8_t launchStatus = currentLaunchStatus(launchWaitS);
  Serial.print(" launchStatus=");
  Serial.print((unsigned int)launchStatus);
  Serial.print(" launchReady=");
  Serial.print(launchStatus == LAUNCH_STATUS_READY ? "1" : "0");
  Serial.print(" launchWaitS=");
  Serial.print((unsigned int)launchWaitS);
  Serial.print(" launchArmMs=");
  Serial.print((unsigned long)launchArmedMs);
  Serial.print(" launchStillMs=");
  Serial.print((unsigned long)launchArmStillSinceMs);
  Serial.print(" logFinal=");
  Serial.println(logsFinalized ? "1" : "0");
}

static void serialDebugTask() {
  static uint32_t lastPrintMs = 0;
  if (SERIAL_DEBUG_LEVEL <= 0) return;

  const uint32_t nowMs = millis();
  if (taskDue(nowMs, lastPrintMs, STATUS_PRINT_MS)) {
    printDebugStatus();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STATUS_LED_PIN, OUTPUT);
  writeStatusLed(false);
  setLedMode(LED_MODE_BOOT);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);
  setupPyroOutputs();
#if BUTTON_ACTIVE_LOW
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#else
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
#endif

  Wire.begin();
  GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1);

#if defined(__IMXRT1062__)
  analogReadResolution(12);
#endif
  pinMode(VBAT_PIN, INPUT);

  setupLoRa();
  setupStorage();
  processServiceModeIfRequested();
  setupBaro();
  setupImu();

  sampleBatteryTask();
  updateLedModeFromHealth();
  playStartupSound(startupHardwareOk());

  if (SERIAL_DEBUG_LEVEL >= 1) Serial.println("RocketV10 ready");
}

void loop() {
  static uint32_t lastImuMs = 0;
  static uint32_t lastBaroMs = 0;
  static uint32_t lastBattMs = 0;
  static uint32_t lastNandImuLogMs = 0;
  static uint32_t lastImuUs = micros();

  sampleGpsTask();

  uint32_t nowUs = micros();
  float dtImu = (nowUs - lastImuUs) * 1e-6f;
  if (dtImu <= 0.0f || dtImu > 0.05f) dtImu = 0.01f;

  uint32_t nowMs = millis();
  if (taskDue(nowMs, lastBattMs, BATT_UPDATE_MS)) {
    sampleBatteryTask();
    logNandBatteryBinary(nowMs);
  }

  if (taskDue(nowMs, lastImuMs, IMU_UPDATE_MS)) {
    lastImuUs = nowUs;
    sampleImuTask(dtImu);
    if (taskDue(nowMs, lastNandImuLogMs, NAND_IMU_LOG_UPDATE_MS)) {
      logNandImuBinary(nowMs);
      logNandAttitudeBinary(nowMs);
    }
  }

  if ((uint32_t)(nowMs - lastBaroMs) >= BARO_UPDATE_MS) {
    float dtBaro = (nowMs - lastBaroMs) / 1000.0f;
    if (dtBaro <= 0.0f || dtBaro > 0.25f) dtBaro = BARO_UPDATE_MS / 1000.0f;
    lastBaroMs = nowMs;
    sampleBaroTask(dtBaro);
    if (isBaroFresh()) logNandBaroBinary(nowMs);
  }

  logNandGpsBinary(nowMs);

  telemetryTask();
  storageTask();
  updateButtonTask();
  updatePyroOutputs(nowMs);
  serialDebugTask();
  updateLedModeFromHealth();
  updateStatusLed();
  updateBuzzer();
}
