#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP3XX.h>

namespace Pins {
const uint8_t SD_CS = 0;
const uint8_t IMU_CS = 1;
const uint8_t BUTTON = 2;
const uint8_t BUZZER = 3;
const uint8_t BMP_CS = 6;
const uint8_t STATUS_LED = LED_BUILTIN;
}

namespace Timing {
const uint32_t SAMPLE_PERIOD_MS = 50;
const uint32_t STATUS_PERIOD_MS = 250;
const uint32_t BUTTON_HOLD_MS = 1000;
const uint32_t BUTTON_RESET_HOLD_MS = 4000;
const uint32_t GPS_BAUD = 9600;
}

namespace BuildConfig {
const bool IMU_ENABLED = true;
const bool IMU_REQUIRED = false;
}

namespace Thresholds {
const float ARM_ALT_SETTLE_S = 2.0f;
const float LAUNCH_ACCEL_G = 2.5f;
const float LAUNCH_ALT_GAIN_M = 5.0f;
const float APOGEE_FALL_M = 0.75f;
const float COAST_TIMEOUT_ALT_FALL_M = 0.30f;
const float LANDED_GYRO_DPS = 12.0f;
const float LANDED_ALT_BAND_M = 2.0f;
const uint32_t DESCENT_CONFIRM_MS = 300;
const uint32_t COAST_TIMEOUT_MS = 8000;
const uint32_t LANDED_TIME_MS = 8000;
}

enum FlightPhase : uint8_t {
  PHASE_BOOT,
  PHASE_IDLE,
  PHASE_ARMED,
  PHASE_BOOST,
  PHASE_COAST,
  PHASE_DESCENT,
  PHASE_LANDED,
  PHASE_ERROR
};

struct ImuSample {
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  bool ok = false;
};

Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
File logFile;

FlightPhase phase = PHASE_BOOT;
ImuSample imu;

float groundPressurePa = 101325.0f;
float groundAltitudeM = 0.0f;
float filteredAltitudeM = 0.0f;
float maxAltitudeM = 0.0f;
float lastLoggedAltitudeM = 0.0f;
float latestPressurePa = 101325.0f;

uint32_t lastSampleMs = 0;
uint32_t lastStatusMs = 0;
uint32_t buttonPressStartMs = 0;
uint32_t launchDetectedMs = 0;
uint32_t descentCandidateMs = 0;
uint32_t landedCandidateMs = 0;
uint32_t lastLandedBeaconMs = 0;
uint32_t flushCounter = 0;

bool sdReady = false;
bool bmpReady = false;
bool imuReady = false;
bool gpsSeen = false;
bool buttonShortHandled = false;
bool buttonLongHandled = false;
bool systemReady = false;

const uint8_t ICM_REG_WHO_AM_I = 0x75;
const uint8_t ICM_REG_PWR_MGMT_1 = 0x6B;
const uint8_t ICM_REG_SMPLRT_DIV = 0x19;
const uint8_t ICM_REG_CONFIG = 0x1A;
const uint8_t ICM_REG_GYRO_CONFIG = 0x1B;
const uint8_t ICM_REG_ACCEL_CONFIG = 0x1C;
const uint8_t ICM_REG_ACCEL_CONFIG2 = 0x1D;
const uint8_t ICM_REG_USER_CTRL = 0x6A;
const uint8_t ICM_REG_I2C_IF = 0x70;
const uint8_t ICM_REG_ACCEL_XOUT_H = 0x3B;

const float ACCEL_LSB_PER_G = 2048.0f;
const float GYRO_LSB_PER_DPS = 16.4f;

const char *phaseName(FlightPhase p) {
  switch (p) {
    case PHASE_BOOT: return "BOOT";
    case PHASE_IDLE: return "IDLE";
    case PHASE_ARMED: return "ARMED";
    case PHASE_BOOST: return "BOOST";
    case PHASE_COAST: return "COAST";
    case PHASE_DESCENT: return "DESCENT";
    case PHASE_LANDED: return "LANDED";
    case PHASE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void setPhase(FlightPhase nextPhase) {
  if (phase == nextPhase) {
    return;
  }
  phase = nextPhase;
  Serial.print("PHASE=");
  Serial.println(phaseName(phase));
}

void buzz(uint16_t durationMs, uint16_t frequency = 2400) {
  tone(Pins::BUZZER, frequency, durationMs);
}

void fatalError(const __FlashStringHelper *message) {
  Serial.println(message);
  setPhase(PHASE_ERROR);
}

void printInitStatus(const __FlashStringHelper *name, bool ok) {
  Serial.print(name);
  Serial.print(": ");
  Serial.println(ok ? F("OK") : F("FAIL"));
}

void printInitDisabled(const __FlashStringHelper *name) {
  Serial.print(name);
  Serial.println(F(": DISABLED"));
}

void printStartupSummary() {
  Serial.println(F("INIT SUMMARY"));
  printInitStatus(F("BMP390"), bmpReady);
  if (BuildConfig::IMU_ENABLED) {
    printInitStatus(F("ICM-20602"), imuReady);
  } else {
    printInitDisabled(F("ICM-20602"));
  }
  printInitStatus(F("SD"), sdReady);
  Serial.print(F("SYSTEM READY: "));
  Serial.println(systemReady ? F("YES") : F("NO"));
}

uint8_t icmReadReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::IMU_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(Pins::IMU_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void icmWriteReg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::IMU_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(Pins::IMU_CS, HIGH);
  SPI.endTransaction();
}

void icmReadBytes(uint8_t reg, uint8_t *buffer, size_t length) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pins::IMU_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(Pins::IMU_CS, HIGH);
  SPI.endTransaction();
}

bool initImu() {
  digitalWrite(Pins::IMU_CS, HIGH);
  delay(100);

  const uint8_t whoAmIPreReset = icmReadReg(ICM_REG_WHO_AM_I);
  Serial.print("ICM WHOAMI pre-reset=0x");
  Serial.println(whoAmIPreReset, HEX);

  icmWriteReg(ICM_REG_PWR_MGMT_1, 0x80);
  delay(100);
  const uint8_t whoAmIPostReset = icmReadReg(ICM_REG_WHO_AM_I);
  Serial.print("ICM WHOAMI post-reset=0x");
  Serial.println(whoAmIPostReset, HEX);

  icmWriteReg(ICM_REG_I2C_IF, 0x40);
  delay(10);
  icmWriteReg(ICM_REG_PWR_MGMT_1, 0x01);
  delay(10);
  icmWriteReg(ICM_REG_USER_CTRL, 0x00);
  icmWriteReg(ICM_REG_SMPLRT_DIV, 0x04);
  icmWriteReg(ICM_REG_CONFIG, 0x03);
  icmWriteReg(ICM_REG_GYRO_CONFIG, 0x18);
  icmWriteReg(ICM_REG_ACCEL_CONFIG, 0x18);
  icmWriteReg(ICM_REG_ACCEL_CONFIG2, 0x03);

  const uint8_t whoAmI = icmReadReg(ICM_REG_WHO_AM_I);
  Serial.print("ICM WHOAMI=0x");
  Serial.println(whoAmI, HEX);
  Serial.print("ICM PWR_MGMT_1=0x");
  Serial.println(icmReadReg(ICM_REG_PWR_MGMT_1), HEX);
  Serial.print("ICM I2C_IF=0x");
  Serial.println(icmReadReg(ICM_REG_I2C_IF), HEX);

  return whoAmI == 0x12;
}

ImuSample readImu() {
  ImuSample sample;
  uint8_t raw[14] = {0};
  icmReadBytes(ICM_REG_ACCEL_XOUT_H, raw, sizeof(raw));

  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
  int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
  int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

  sample.ax_g = (float)ax / ACCEL_LSB_PER_G;
  sample.ay_g = (float)ay / ACCEL_LSB_PER_G;
  sample.az_g = (float)az / ACCEL_LSB_PER_G;
  sample.gx_dps = (float)gx / GYRO_LSB_PER_DPS;
  sample.gy_dps = (float)gy / GYRO_LSB_PER_DPS;
  sample.gz_dps = (float)gz / GYRO_LSB_PER_DPS;
  sample.ok = true;
  return sample;
}

bool openNextLogFile() {
  char name[20];
  for (uint8_t i = 0; i < 100; ++i) {
    snprintf(name, sizeof(name), "FLIGHT%02u.CSV", i);
    if (!SD.exists(name)) {
      logFile = SD.open(name, FILE_WRITE);
      if (logFile) {
        logFile.println("ms,phase,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,baro_alt_m,max_alt_m,baro_temp_c,pressure_pa,gps_fix,gps_sats,lat_deg,lon_deg,gps_alt_m,gps_speed_mps,gps_course_deg");
        logFile.flush();
        Serial.print("LOG=");
        Serial.println(name);
        return true;
      }
    }
  }
  return false;
}

void serviceGps() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    gps.encode(c);
    gpsSeen = true;
  }
}

bool updateBarometer(float &altitudeM, float &temperatureC, float &pressurePa) {
  if (!bmp.performReading()) {
    return false;
  }
  temperatureC = bmp.temperature;
  pressurePa = bmp.pressure;
  latestPressurePa = pressurePa;
  if (phase == PHASE_IDLE) {
    // While idle on the bench, keep the baro reference slowly centered near zero.
    groundPressurePa = 0.98f * groundPressurePa + 0.02f * pressurePa;
  }
  altitudeM = 44330.0f * (1.0f - powf(pressurePa / groundPressurePa, 0.1903f)) + groundAltitudeM;
  filteredAltitudeM = 0.85f * filteredAltitudeM + 0.15f * altitudeM;
  if (filteredAltitudeM > maxAltitudeM) {
    maxAltitudeM = filteredAltitudeM;
  }
  return true;
}

bool captureGroundReference(uint8_t samples = 16, uint16_t sampleDelayMs = 40) {
  float pressureSum = 0.0f;
  uint8_t goodSamples = 0;

  for (uint8_t i = 0; i < samples; ++i) {
    if (bmp.performReading()) {
      pressureSum += bmp.pressure;
      ++goodSamples;
    }
    delay(sampleDelayMs);
  }

  if (goodSamples == 0) {
    return false;
  }

  groundPressurePa = pressureSum / goodSamples;
  latestPressurePa = groundPressurePa;
  groundAltitudeM = 0.0f;
  filteredAltitudeM = 0.0f;
  maxAltitudeM = 0.0f;
  lastLoggedAltitudeM = 0.0f;

  Serial.print(F("GROUND PRESSURE PA="));
  Serial.println(groundPressurePa, 2);
  return true;
}

bool resetForNextFlight() {
  Serial.println(F("RESET FOR NEXT FLIGHT"));

  if (logFile) {
    logFile.flush();
    logFile.close();
  }

  sdReady = SD.begin(Pins::SD_CS);
  if (sdReady) {
    sdReady = openNextLogFile();
  }
  if (!sdReady) {
    fatalError(F("Reset failed: SD reopen failed"));
    return false;
  }

  imu = ImuSample();
  launchDetectedMs = 0;
  descentCandidateMs = 0;
  landedCandidateMs = 0;
  lastLandedBeaconMs = 0;
  flushCounter = 0;
  lastSampleMs = 0;
  lastStatusMs = 0;
  lastLoggedAltitudeM = 0.0f;
  filteredAltitudeM = 0.0f;
  maxAltitudeM = 0.0f;
  groundAltitudeM = 0.0f;

  if (!captureGroundReference()) {
    fatalError(F("Reset failed: ground reference capture failed"));
    return false;
  }

  setPhase(PHASE_IDLE);
  buzz(120, 2200);
  delay(140);
  buzz(120, 2600);
  delay(140);
  buzz(120, 3000);
  Serial.println(F("READY FOR NEXT FLIGHT"));
  return true;
}

void serviceLandedBeacon(uint32_t nowMs) {
  if (phase != PHASE_LANDED) {
    return;
  }

  if (nowMs - lastLandedBeaconMs >= 3000) {
    lastLandedBeaconMs = nowMs;
    buzz(120, 2200);
    delay(140);
    buzz(120, 2200);
  }
}

float accelMagnitudeG(const ImuSample &s) {
  return sqrtf(s.ax_g * s.ax_g + s.ay_g * s.ay_g + s.az_g * s.az_g);
}

float gyroMagnitudeDps(const ImuSample &s) {
  return sqrtf(s.gx_dps * s.gx_dps + s.gy_dps * s.gy_dps + s.gz_dps * s.gz_dps);
}

void updateFlightState(uint32_t nowMs) {
  if (phase == PHASE_IDLE || phase == PHASE_ARMED) {
    const bool accelLaunch = accelMagnitudeG(imu) > Thresholds::LAUNCH_ACCEL_G;
    const bool altLaunch = (filteredAltitudeM - groundAltitudeM) > Thresholds::LAUNCH_ALT_GAIN_M;
    if (accelLaunch || altLaunch) {
      if (!captureGroundReference(8, 20)) {
        Serial.println(F("Launch-time ground reference refresh failed"));
      }
      launchDetectedMs = nowMs;
      setPhase(PHASE_BOOST);
      buzz(200, 2800);
    }
  } else if (phase == PHASE_BOOST) {
    if (nowMs - launchDetectedMs > 1500) {
      setPhase(PHASE_COAST);
    }
  } else if (phase == PHASE_COAST) {
    const float altDropM = maxAltitudeM - filteredAltitudeM;
    const bool nominalApogeeDetected = altDropM > Thresholds::APOGEE_FALL_M;
    const bool timeoutApogeeDetected =
        (nowMs - launchDetectedMs) > Thresholds::COAST_TIMEOUT_MS &&
        altDropM > Thresholds::COAST_TIMEOUT_ALT_FALL_M;

    if (nominalApogeeDetected || timeoutApogeeDetected) {
      if (descentCandidateMs == 0) {
        descentCandidateMs = nowMs;
      }
      if (nowMs - descentCandidateMs > Thresholds::DESCENT_CONFIRM_MS) {
        setPhase(PHASE_DESCENT);
        buzz(300, 1800);
      }
    } else {
      descentCandidateMs = 0;
    }
  } else if (phase == PHASE_DESCENT) {
    const bool lowMotion = gyroMagnitudeDps(imu) < Thresholds::LANDED_GYRO_DPS;
    const bool nearGround = fabsf(filteredAltitudeM - groundAltitudeM) < Thresholds::LANDED_ALT_BAND_M;
    if (lowMotion && nearGround) {
      if (landedCandidateMs == 0) {
        landedCandidateMs = nowMs;
      }
      if (nowMs - landedCandidateMs > Thresholds::LANDED_TIME_MS) {
        setPhase(PHASE_LANDED);
        Serial.print(F("MAX ALTITUDE M="));
        Serial.println(maxAltitudeM, 2);
        buzz(600, 1400);
      }
    } else {
      landedCandidateMs = 0;
    }
  }
}

void logSample(uint32_t nowMs, float altitudeM, float temperatureC, float pressurePa) {
  if (!sdReady || !logFile) {
    return;
  }

  const bool gpsFix = gps.location.isValid() && gps.location.age() < 2000;

  logFile.print(nowMs);
  logFile.print(',');
  logFile.print(phaseName(phase));
  logFile.print(',');
  logFile.print(imu.ax_g, 4);
  logFile.print(',');
  logFile.print(imu.ay_g, 4);
  logFile.print(',');
  logFile.print(imu.az_g, 4);
  logFile.print(',');
  logFile.print(imu.gx_dps, 3);
  logFile.print(',');
  logFile.print(imu.gy_dps, 3);
  logFile.print(',');
  logFile.print(imu.gz_dps, 3);
  logFile.print(',');
  logFile.print(altitudeM, 2);
  logFile.print(',');
  logFile.print(maxAltitudeM, 2);
  logFile.print(',');
  logFile.print(temperatureC, 2);
  logFile.print(',');
  logFile.print(pressurePa, 2);
  logFile.print(',');
  logFile.print(gpsFix ? 1 : 0);
  logFile.print(',');
  logFile.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
  logFile.print(',');
  logFile.print(gps.location.isValid() ? gps.location.lat() : 0.0, 6);
  logFile.print(',');
  logFile.print(gps.location.isValid() ? gps.location.lng() : 0.0, 6);
  logFile.print(',');
  logFile.print(gps.altitude.isValid() ? gps.altitude.meters() : 0.0, 2);
  logFile.print(',');
  logFile.print(gps.speed.isValid() ? gps.speed.mps() : 0.0, 2);
  logFile.print(',');
  logFile.println(gps.course.isValid() ? gps.course.deg() : 0.0, 2);

  if (++flushCounter >= 20) {
    logFile.flush();
    flushCounter = 0;
  }
  lastLoggedAltitudeM = altitudeM;
}

void updateUi(uint32_t nowMs) {
  bool ledState = false;

  switch (phase) {
    case PHASE_BOOT:
      ledState = ((nowMs / 100) % 2) == 0;
      break;
    case PHASE_IDLE:
      ledState = ((nowMs / 1000) % 2) == 0;
      break;
    case PHASE_ARMED:
      ledState = ((nowMs / 200) % 2) == 0;
      break;
    case PHASE_BOOST:
    case PHASE_COAST:
      ledState = true;
      break;
    case PHASE_DESCENT:
      ledState = ((nowMs / 300) % 2) == 0;
      break;
    case PHASE_LANDED:
      ledState = ((nowMs / 1200) % 2) == 0;
      break;
    case PHASE_ERROR:
      ledState = ((nowMs / 80) % 2) == 0;
      break;
  }

  digitalWrite(Pins::STATUS_LED, ledState ? HIGH : LOW);
}

void serviceButton(uint32_t nowMs) {
  const bool pressed = digitalRead(Pins::BUTTON) == LOW;

  if (pressed && buttonPressStartMs == 0) {
    buttonPressStartMs = nowMs;
  }

  if (!pressed) {
    buttonPressStartMs = 0;
    buttonShortHandled = false;
    buttonLongHandled = false;
    return;
  }

  const uint32_t heldMs = nowMs - buttonPressStartMs;

  if (!buttonLongHandled && heldMs >= Timing::BUTTON_RESET_HOLD_MS && phase != PHASE_ERROR) {
    buttonLongHandled = true;
    buttonShortHandled = true;
    resetForNextFlight();
    return;
  }

  if (!buttonShortHandled && heldMs >= Timing::BUTTON_HOLD_MS) {
    buttonShortHandled = true;
    if (phase == PHASE_IDLE) {
      if (!captureGroundReference()) {
        Serial.println(F("Ground reference capture failed"));
        buzz(700, 1200);
        return;
      }
      Serial.println(F("Ground reference refreshed by button"));
      buzz(150, 2400);
    }
  }
}

void printStatus(uint32_t nowMs, float altitudeM) {
  if (nowMs - lastStatusMs < Timing::STATUS_PERIOD_MS) {
    return;
  }
  lastStatusMs = nowMs;

  Serial.print("t=");
  Serial.print(nowMs);
  Serial.print(" phase=");
  Serial.print(phaseName(phase));
  Serial.print(" alt=");
  Serial.print(altitudeM, 2);
  Serial.print(" max_alt=");
  Serial.print(maxAltitudeM, 2);
  Serial.print(" a_g=");
  Serial.print(accelMagnitudeG(imu), 2);
  Serial.print(" sats=");
  Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
  Serial.print(" gps=");
  Serial.println(gps.location.isValid() ? 1 : 0);
}

void setup() {
  pinMode(Pins::STATUS_LED, OUTPUT);
  pinMode(Pins::BUTTON, INPUT_PULLUP);
  pinMode(Pins::BUZZER, OUTPUT);
  pinMode(Pins::SD_CS, OUTPUT);
  pinMode(Pins::BMP_CS, OUTPUT);
  if (BuildConfig::IMU_ENABLED) {
    pinMode(Pins::IMU_CS, OUTPUT);
  }

  digitalWrite(Pins::STATUS_LED, LOW);
  digitalWrite(Pins::SD_CS, HIGH);
  digitalWrite(Pins::BMP_CS, HIGH);
  if (BuildConfig::IMU_ENABLED) {
    digitalWrite(Pins::IMU_CS, HIGH);
  }

  Serial.begin(115200);
  delay(1000);
  Serial.println("XIAO SAMD21 rocket flight computer boot");

  SPI.begin();
  Serial1.begin(Timing::GPS_BAUD);

  bmpReady = bmp.begin_SPI(Pins::BMP_CS);
  if (bmpReady) {
    bmpReady = true;
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  printInitStatus(F("BMP390"), bmpReady);

  if (BuildConfig::IMU_ENABLED) {
    imuReady = initImu();
    printInitStatus(F("ICM-20602"), imuReady);
  } else {
    imuReady = false;
    printInitDisabled(F("ICM-20602"));
  }

  sdReady = SD.begin(Pins::SD_CS);
  if (sdReady) {
    sdReady = openNextLogFile();
  }
  printInitStatus(F("SD"), sdReady);

  systemReady = bmpReady && sdReady && (!BuildConfig::IMU_REQUIRED || imuReady);
  printStartupSummary();

  if (!systemReady) {
    fatalError(F("One or more required devices failed initialization"));
    Serial.println(F("Bench note: connect one device at a time and re-test."));
    return;
  }

  if (!captureGroundReference()) {
    fatalError(F("Ground reference capture failed"));
    Serial.println(F("Bench note: check BMP390 wiring and power stability."));
    return;
  }

  if (phase != PHASE_ERROR) {
    setPhase(PHASE_IDLE);
    buzz(120, 2500);
    delay(160);
    buzz(120, 3200);
  }
}

void loop() {
  const uint32_t nowMs = millis();

  updateUi(nowMs);
  serviceLandedBeacon(nowMs);

  if (phase == PHASE_ERROR) {
    static uint32_t lastErrorPrintMs = 0;
    if (nowMs - lastErrorPrintMs >= 1500) {
      lastErrorPrintMs = nowMs;
      Serial.println(F("ERROR MODE: waiting for reset after wiring or power changes"));
    }
    delay(20);
    return;
  }

  serviceGps();
  serviceButton(nowMs);

  if (nowMs - lastSampleMs < Timing::SAMPLE_PERIOD_MS) {
    return;
  }
  lastSampleMs = nowMs;

  imu = readImu();

  float altitudeM = lastLoggedAltitudeM;
  float temperatureC = 0.0f;
  float pressurePa = groundPressurePa;

  if (!updateBarometer(altitudeM, temperatureC, pressurePa)) {
    Serial.println("BMP390 read failed");
  }

  updateFlightState(nowMs);
  logSample(nowMs, altitudeM, temperatureC, pressurePa);
  printStatus(nowMs, altitudeM);
}
