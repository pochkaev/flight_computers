// =====================================================
// Rocket Flight Computer - Teensy 4.0
// LoRa (RFM95) + Baro (BMP390/BMP280/BMP180) + IMU (ICM-20602/MPU-6050) + GPS (GT-U7)
// Telemetry V6 binary format (must match ground station)
//
// Wiring summary:
//   LoRa RFM95: CS=10, RST=9, DIO0=2
//   I2C:        SDA=18, SCL=19 (baro + IMU)
//   GPS GT-U7:  TX -> pin 0 (RX1), RX -> pin 1 (TX1, optional)
//
// =====================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>

#include <Adafruit_BMP280.h>   // BMP280
#include <Adafruit_BMP085.h>   // BMP180
#include <Adafruit_BMP3XX.h>   // BMP390

// ----------------- LoRa config -----------------
const long LORA_FREQUENCY = 915E6;
const int LORA_CS_PIN    = 10;
const int LORA_RST_PIN   = 9;
const int LORA_DIO0_PIN  = 2;
volatile bool loraTxBusy = false;


// ----------------- Altimeter -------------------
Adafruit_BMP280 bmp280;
Adafruit_BMP085 bmp180;
Adafruit_BMP3XX bmp390;

enum AltimeterType {
  ALT_NONE = 0,
  ALT_BMP390,
  ALT_BMP280,
  ALT_BMP180
};
AltimeterType altimeterType = ALT_NONE;

// Approx sea level pressure, hPa
const float SEA_LEVEL_HPA = 1013.25f;

// ----------------- IMU (ICM-20602 / MPU-6050) --
enum ImuType {
  IMU_NONE = 0,
  IMU_ICM20602,
  IMU_MPU6050
};
ImuType imuType = IMU_NONE;
uint8_t imuAddr = 0x68;

const uint8_t REG_WHO_AM_I   = 0x75;
const uint8_t REG_PWR_MGMT1  = 0x6B;
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_GYRO_CONFIG   = 0x1B;
const uint8_t REG_ACCEL_CONFIG  = 0x1C;
const uint8_t REG_ACCEL_XOUT_H  = 0x3B;

const float ACCEL_SENS_4G = 8192.0f; // LSB/g
const float GYRO_SENS_500 = 65.5f;   // LSB/(deg/s)
const float DEG2RAD       = 3.14159265358979f / 180.0f;

// Last IMU values
float last_ax = 0.0f, last_ay = 0.0f, last_az = 0.0f;
float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;

// Simple complementary filter estimate
float roll  = 0.0f; // rad
float pitch = 0.0f; // rad
bool  haveImuEstimate = false;

// ----------------- Flight state machine --------
enum FlightState : uint8_t {
  FS_IDLE = 0,
  FS_PAD,
  FS_ASCENT,
  FS_COAST,
  FS_DESCENT,
  FS_LANDED,
  FS_ABORT
};

FlightState flightState = FS_IDLE;

uint16_t flightFlags = 0;
const uint16_t FLAG_LAUNCH  = 1 << 0;
const uint16_t FLAG_APOGEE  = 1 << 1;
const uint16_t FLAG_LANDED  = 1 << 2;

// Altitude / velocity filtering
bool   haveAlt   = false;
float  filtAlt   = 0.0f;
float  filtTemp  = 0.0f;
float  velZ      = 0.0f;    // m/s
float  lastAltRaw = 0.0f;

uint32_t tLaunchMs = 0;
uint32_t tApogeeMs = 0;

// ----------------- GPS (GT-U7 + TinyGPS++) ----
TinyGPSPlus gps;

// "current" GPS snapshot
bool    gpsHasFix    = false;
uint8_t gpsFixType   = 0;     // 0=no, 2=2D, 3=3D
uint8_t gpsSats      = 0;
float   gpsLatDeg    = 0.0f;
float   gpsLonDeg    = 0.0f;
float   gpsAltM      = 0.0f;
float   gpsSpeedMps  = 0.0f;
float   gpsHdop      = 99.99f;

// last known good fix (for recovery)
bool     haveGoodFix   = false;
float    lastFixLatDeg = 0.0f;
float    lastFixLonDeg = 0.0f;
float    lastFixAltM   = 0.0f;
uint32_t lastFixTimeMs = 0;

// ----------------- Telemetry V6 struct --------
struct TelemetryPacketV6 {
  uint8_t  version;   // 6
  uint8_t  state;     // FlightState
  uint16_t flags;     // bitfield
  uint32_t seq;
  uint32_t ms;

  float    alt;       // filtered baro altitude (m)
  float    temp;      // filtered temp (°C)
  float    vel_z;     // vertical speed (m/s)

  float    ax;        // m/s^2
  float    ay;
  float    az;
  float    gx;        // deg/s
  float    gy;
  float    gz;
  float    roll_deg;  // fused attitude
  float    pitch_deg;

  uint8_t  gps_has_fix;    // 0/1
  uint8_t  gps_fix_type;   // 0=no, 2=2D, 3=3D
  uint8_t  gps_sats;
  uint8_t  gps_reserved;   // padding

  float    gps_lat_deg;    // current fix (if valid) else 0
  float    gps_lon_deg;
  float    gps_alt_m;
  float    gps_speed_mps;
  float    gps_hdop;       // e.g. 1.14

  float    gps_last_lat_deg;   // last known good fix
  float    gps_last_lon_deg;
  float    gps_last_alt_m;
  uint32_t gps_last_fix_age_ms; // ms since last good fix
};

static_assert(sizeof(TelemetryPacketV6) == 96, "TelemetryPacketV6 must be 96 bytes");

uint32_t packetSeq = 0;

// ----------------- I2C helpers ----------------
uint8_t imuRead8(uint8_t reg) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)imuAddr, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void imuWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// ----------------- LoRa setup -----------------
void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    // Hard fail if LoRa not found
    while (true) {
      delay(1000);
    }
  }

  // optional, but fine for Teensy 4.0
  LoRa.setSPIFrequency(8E6);

  // TX-done callback (uses DIO0 pin)
  LoRa.onTxDone(onLoraTxDone);

  // Initially not busy
  loraTxBusy = false;
}


void onLoraTxDone() {
  loraTxBusy = false;
}

// ----------------- Altimeter setup ------------

void setupAltimeter() {
  Wire.begin();

  // Try BMP390 on 0x76 / 0x77
  if (bmp390.begin_I2C(0x76)) {
    altimeterType = ALT_BMP390;
    bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp390.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("Altimeter: BMP390 @ 0x76");
    return;
  }
  if (bmp390.begin_I2C(0x77)) {
    altimeterType = ALT_BMP390;
    bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp390.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("Altimeter: BMP390 @ 0x77");
    return;
  }

  // Try BMP280 on 0x76 / 0x77
  if (bmp280.begin(0x76)) {
    altimeterType = ALT_BMP280;
    bmp280.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,   // Temp
      Adafruit_BMP280::SAMPLING_X4,   // Pressure
      Adafruit_BMP280::FILTER_X4,
      Adafruit_BMP280::STANDBY_MS_63
    );
    Serial.println("Altimeter: BMP280 @ 0x76");
    return;
  }
  if (bmp280.begin(0x77)) {
    altimeterType = ALT_BMP280;
    bmp280.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X4,
      Adafruit_BMP280::FILTER_X4,
      Adafruit_BMP280::STANDBY_MS_63
    );
    Serial.println("Altimeter: BMP280 @ 0x77");
    return;
  }

  // Try BMP180 (Adafruit_BMP085)
  if (bmp180.begin()) {
    altimeterType = ALT_BMP180;
    Serial.println("Altimeter: BMP180");
    return;
  }

  altimeterType = ALT_NONE;
  Serial.println("Altimeter: NONE");
}

bool readAltimeterOnce(float &alt_m, float &temp_c) {
  if (altimeterType == ALT_NONE) return false;

  if (altimeterType == ALT_BMP390) {
    if (!bmp390.performReading()) return false;
    float pressure_hPa = bmp390.pressure / 100.0f;
    temp_c = bmp390.temperature;
    alt_m  = 44330.0f * (1.0f - pow(pressure_hPa / SEA_LEVEL_HPA, 0.1903f));
    return true;
  }

  if (altimeterType == ALT_BMP280) {
    float pressure_hPa = bmp280.readPressure() / 100.0f;
    temp_c = bmp280.readTemperature();
    alt_m  = bmp280.readAltitude(SEA_LEVEL_HPA);
    return true;
  }

  if (altimeterType == ALT_BMP180) {
    float pressure_hPa = bmp180.readPressure() / 100.0f;
    temp_c = bmp180.readTemperature();
    alt_m  = bmp180.readAltitude(SEA_LEVEL_HPA * 100.0f);
    return true;
  }

  return false;
}

// ----------------- IMU setup ------------------
void setupImu() {
  uint8_t addrs[2] = {0x68, 0x69};
  imuType = IMU_NONE;

  for (int i = 0; i < 2; i++) {
    uint8_t addr = addrs[i];
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;

    imuAddr = addr;
    uint8_t who = imuRead8(REG_WHO_AM_I);

    if (who == 0x12) {
      imuType = IMU_ICM20602;
      Serial.print("IMU: ICM-20602 @ 0x");
      Serial.println(imuAddr, HEX);
      break;
    } else if (who == 0x68) {
      imuType = IMU_MPU6050;
      Serial.print("IMU: MPU-6050 @ 0x");
      Serial.println(imuAddr, HEX);
      break;
    }
  }

  if (imuType == IMU_NONE) {
    Serial.println("IMU: NONE");
    return;
  }

  // Wake & configure
  imuWrite8(REG_PWR_MGMT1, 0x00);
  imuWrite8(REG_SMPLRT_DIV, 0x07);  // ~125 Hz
  imuWrite8(REG_GYRO_CONFIG,  0x08); // ±500 dps
  imuWrite8(REG_ACCEL_CONFIG, 0x08); // ±4 g
}

bool readImuRaw(int16_t &rawAx, int16_t &rawAy, int16_t &rawAz,
                int16_t &rawGx, int16_t &rawGy, int16_t &rawGz) {
  if (imuType == IMU_NONE) return false;

  Wire.beginTransmission(imuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t N = 14;
  Wire.requestFrom((int)imuAddr, (int)N);
  if (Wire.available() < N) return false;

  rawAx = (Wire.read() << 8) | Wire.read();
  rawAy = (Wire.read() << 8) | Wire.read();
  rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read(); (void)rawTemp;
  rawGx = (Wire.read() << 8) | Wire.read();
  rawGy = (Wire.read() << 8) | Wire.read();
  rawGz = (Wire.read() << 8) | Wire.read();
  return true;
}

// ----------------- IMU + complementary filter --
void updateImuEstimate(float dt) {
  int16_t rax, ray, raz, rgx, rgy, rgz;
  if (!readImuRaw(rax, ray, raz, rgx, rgy, rgz)) return;

  float ax_g = (float)rax / ACCEL_SENS_4G;
  float ay_g = (float)ray / ACCEL_SENS_4G;
  float az_g = (float)raz / ACCEL_SENS_4G;

  float gx_dps = (float)rgx / GYRO_SENS_500;
  float gy_dps = (float)rgy / GYRO_SENS_500;
  float gz_dps = (float)rgz / GYRO_SENS_500;

  const float g = 9.80665f;
  last_ax = ax_g * g;
  last_ay = ay_g * g;
  last_az = az_g * g;
  last_gx = gx_dps;
  last_gy = gy_dps;
  last_gz = gz_dps;

  float roll_acc  = atan2f(ay_g, az_g);
  float pitch_acc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

  float gx_rad = gx_dps * DEG2RAD;
  float gy_rad = gy_dps * DEG2RAD;

  if (!haveImuEstimate) {
    roll  = roll_acc;
    pitch = pitch_acc;
    haveImuEstimate = true;
  } else {
    const float alpha = 0.95f;
    roll  = alpha * (roll  + gx_rad * dt) + (1.0f - alpha) * roll_acc;
    pitch = alpha * (pitch + gy_rad * dt) + (1.0f - alpha) * pitch_acc;
  }
}

// ----------------- GPS update ------------------
unsigned long lastGpsDiagMs = 0;

void updateGps() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    gps.encode(c);
  }

  bool locValid   = gps.location.isValid();
  bool altValid   = gps.altitude.isValid();
  bool spdValid   = gps.speed.isValid();
  bool satsValid  = gps.satellites.isValid();
  bool hdopValid  = gps.hdop.isValid();

  gpsHasFix   = locValid;
  gpsLatDeg   = locValid ? gps.location.lat()    : 0.0f;
  gpsLonDeg   = locValid ? gps.location.lng()    : 0.0f;
  gpsAltM     = altValid ? gps.altitude.meters() : 0.0f;
  gpsSpeedMps = spdValid ? gps.speed.mps()       : 0.0f;
  gpsSats     = satsValid ? (uint8_t)gps.satellites.value() : 0;
  gpsHdop     = hdopValid ? (float)gps.hdop.value() * 0.01f : 99.99f;

  if (locValid && altValid)      gpsFixType = 3;
  else if (locValid)             gpsFixType = 2;
  else                           gpsFixType = 0;

  uint32_t nowMs = millis();
  if (gpsHasFix) {
    haveGoodFix   = true;
    lastFixLatDeg = gpsLatDeg;
    lastFixLonDeg = gpsLonDeg;
    lastFixAltM   = altValid ? gpsAltM : lastFixAltM;
    lastFixTimeMs = nowMs;
  }

  static unsigned long lastPrint = 0;
  if (nowMs - lastPrint >= 1000) {
    lastPrint = nowMs;
    Serial.print("GPS diag: chars=");
    Serial.print(gps.charsProcessed());
    Serial.print("  locValid=");
    Serial.print(locValid);
    Serial.print("  satsValid=");
    Serial.print(satsValid);
    Serial.print("  hdopValid=");
    Serial.print(hdopValid);
    Serial.print("  sats=");
    if (satsValid) Serial.print(gpsSats); else Serial.print(-1);
    Serial.print("  hdop=");
    Serial.print(gpsHdop);
    Serial.print("  lat=");
    if (locValid) Serial.print(gpsLatDeg, 6); else Serial.print(0.0, 6);
    Serial.print("  lon=");
    if (locValid) Serial.print(gpsLonDeg, 6); else Serial.print(0.0, 6);
    Serial.print("  alt=");
    if (altValid) Serial.print(gpsAltM); else Serial.print(0.0);
    Serial.println();
  }
}


// ----------------- Altitude + flight logic -----
void updateAltAndState(float dt) {
  float alt, temp;
  if (!readAltimeterOnce(alt, temp)) return;

  if (!haveAlt) {
    haveAlt   = true;
    filtAlt   = alt;
    filtTemp  = temp;
    lastAltRaw = alt;
    velZ      = 0.0f;
    flightState = FS_PAD;
    return;
  }

  const float aAlt = 0.9f;
  filtAlt  = aAlt * filtAlt  + (1.0f - aAlt) * alt;
  filtTemp = 0.9f * filtTemp + 0.1f * temp;

  float rawVel = (alt - lastAltRaw) / dt;
  lastAltRaw = alt;
  velZ = 0.8f * velZ + 0.2f * rawVel;

  float accMag = sqrtf(last_ax * last_ax + last_ay * last_ay + last_az * last_az);
  const float g = 9.80665f;
  float accMg = accMag / g;

  uint32_t nowMs = millis();

  switch (flightState) {
    case FS_IDLE:
    case FS_PAD:
      if (accMg > 2.0f) {
        flightState = FS_ASCENT;
        flightFlags |= FLAG_LAUNCH;
        tLaunchMs = nowMs;
      }
      break;

    case FS_ASCENT:
      if (accMg < 1.2f && velZ > 1.0f) {
        flightState = FS_COAST;
      }
      break;

    case FS_COAST:
      if (velZ < -0.5f && filtAlt > 30.0f && (nowMs - tLaunchMs) > 1000) {
        flightState = FS_DESCENT;
        flightFlags |= FLAG_APOGEE;
        tApogeeMs = nowMs;
      }
      break;

    case FS_DESCENT:
      if (fabsf(velZ) < 0.5f &&
          fabsf(accMg - 1.0f) < 0.2f &&
          (nowMs - tLaunchMs) > 2000) {
        flightState = FS_LANDED;
        flightFlags |= FLAG_LANDED;
      }
      break;

    case FS_LANDED:
    case FS_ABORT:
    default:
      break;
  }
}

// ----------------- Telemetry send --------------
void sendTelemetry() {
  // If previous TX still in progress, skip this frame
  if (loraTxBusy) {
    return;
  }

  TelemetryPacketV6 pkt = {};  // zero everything

  pkt.version = 6;
  pkt.state   = (uint8_t)flightState;
  pkt.flags   = flightFlags;
  pkt.seq     = packetSeq++;
  pkt.ms      = millis();

  pkt.alt   = filtAlt;
  pkt.temp  = filtTemp;
  pkt.vel_z = velZ;

  pkt.ax = last_ax;
  pkt.ay = last_ay;
  pkt.az = last_az;
  pkt.gx = last_gx;
  pkt.gy = last_gy;
  pkt.gz = last_gz;

  pkt.roll_deg  = roll  * (180.0f / 3.14159265358979f);
  pkt.pitch_deg = pitch * (180.0f / 3.14159265358979f);

  pkt.gps_has_fix  = gpsHasFix ? 1 : 0;
  pkt.gps_fix_type = gpsFixType;
  pkt.gps_sats     = gpsSats;
  pkt.gps_reserved = 0;

  pkt.gps_lat_deg   = gpsLatDeg;
  pkt.gps_lon_deg   = gpsLonDeg;
  pkt.gps_alt_m     = gpsAltM;
  pkt.gps_speed_mps = gpsSpeedMps;
  pkt.gps_hdop      = gpsHdop;

  if (haveGoodFix) {
    uint32_t nowMs = millis();
    pkt.gps_last_lat_deg    = lastFixLatDeg;
    pkt.gps_last_lon_deg    = lastFixLonDeg;
    pkt.gps_last_alt_m      = lastFixAltM;
    pkt.gps_last_fix_age_ms = nowMs - lastFixTimeMs;
  } else {
    pkt.gps_last_lat_deg    = 0.0f;
    pkt.gps_last_lon_deg    = 0.0f;
    pkt.gps_last_alt_m      = 0.0f;
    pkt.gps_last_fix_age_ms = 0xFFFFFFFF;
  }

  // Start async TX
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket(true);    // true = ASYNC, returns immediately

  // Debug
  Serial.print("TX SEQ=");
  Serial.print(pkt.seq);
  Serial.print(" state=");
  Serial.print((int)pkt.state);
  Serial.print(" alt=");
  Serial.print(pkt.alt);
  Serial.print(" velZ=");
  Serial.print(pkt.vel_z);
  Serial.print(" gpsFix=");
  Serial.print((int)pkt.gps_has_fix);
  Serial.print(" sats=");
  Serial.print((int)pkt.gps_sats);
  Serial.print(" lastAge=");
  Serial.println(pkt.gps_last_fix_age_ms);
}


// ----------------- setup / loop ---------------
void setup() {
  Serial.begin(115200);   // Non-blocking if no USB
  Serial1.begin(9600);
  delay(1000); 

  setupLoRa();
  setupAltimeter();
  setupImu();

  // GPS on Serial1 (GT-U7)

  Serial.println("Rocket V6 initialized");
}

void loop() {
  static uint32_t lastImuMicros = micros();
  static uint32_t lastStateMs   = millis();
  static uint32_t lastTelemMs   = millis();

  // IMU update at high rate
  uint32_t nowMicros = micros();
  float dtImu = (nowMicros - lastImuMicros) * 1e-6f;
  if (dtImu <= 0.0f || dtImu > 0.05f) dtImu = 0.01f;
  lastImuMicros = nowMicros;
  updateImuEstimate(dtImu);

  // GPS every loop (just feed TinyGPS++)
  updateGps();

  uint32_t nowMs = millis();

  // Alt + state at ~20 Hz
  if (nowMs - lastStateMs >= 50) {
    float dtState = (nowMs - lastStateMs) / 1000.0f;
    lastStateMs = nowMs;
    updateAltAndState(dtState);
  }

  // Telemetry at ~5 Hz
  if (nowMs - lastTelemMs >= 200) {
    lastTelemMs = nowMs;
    sendTelemetry();
  }

  delay(1);
}
