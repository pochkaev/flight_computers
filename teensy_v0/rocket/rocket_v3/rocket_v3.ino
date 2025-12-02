#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

// ------------- LoRa config -------------
const long LORA_FREQUENCY = 915E6;
const int LORA_CS_PIN    = 10;
const int LORA_RST_PIN   = 9;
const int LORA_DIO0_PIN  = 2;

// ------------- Altimeter libs -------------
#include <Adafruit_BMP280.h>    // BMP280
#include <Adafruit_BMP085.h>    // BMP180
#include <Adafruit_BMP3XX.h>    // BMP390

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

const float SEA_LEVEL_HPA = 1013.25f;

// ------------- IMU (ICM-20602 / MPU-6050) -------------
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

// ------------- Flight state machine -------------
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

// Altitude / velocity filter
bool   haveAlt   = false;
float  filtAlt   = 0.0f;
float  filtTemp  = 0.0f;
float  velZ      = 0.0f;   // m/s
float  lastAltRaw = 0.0f;

// IMU last samples + attitude
float last_ax = 0, last_ay = 0, last_az = 0;
float last_gx = 0, last_gy = 0, last_gz = 0;
float roll = 0.0f;   // rad
float pitch = 0.0f;  // rad
bool  haveImuEstimate = false;

uint32_t tLaunchMs = 0;
uint32_t tApogeeMs = 0;

// ------------- Telemetry V4 -------------
struct TelemetryPacketV4 {
  uint8_t  version;   // 4
  uint8_t  state;     // FlightState
  uint16_t flags;     // bitfield
  uint32_t seq;
  uint32_t ms;
  float    alt;       // m (filtered)
  float    temp;      // °C (filtered)
  float    vel_z;     // m/s (filtered)
  float    ax;        // m/s^2
  float    ay;
  float    az;
  float    gx;        // deg/s
  float    gy;
  float    gz;
  float    roll_deg;  // fused
  float    pitch_deg; // fused
};

uint32_t packetSeq = 0;

// ------------- I2C helpers -------------
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

// ------------- LoRa setup -------------
void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    while (true) {}
  }
}

// ------------- Altimeter -------------
void setupAltimeter() {
  Wire.begin(); // SDA=18, SCL=19

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

  if (bmp280.begin(0x76)) {
    altimeterType = ALT_BMP280;
    bmp280.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X4,
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

// ------------- IMU setup -------------
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

  imuWrite8(REG_PWR_MGMT1, 0x00);
  imuWrite8(REG_SMPLRT_DIV, 0x07); // ~125 Hz
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

// ------------- IMU + complementary filter -------------
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
    const float alpha = 0.95f; // slightly more responsive
    roll  = alpha * (roll  + gx_rad * dt) + (1.0f - alpha) * roll_acc;
    pitch = alpha * (pitch + gy_rad * dt) + (1.0f - alpha) * pitch_acc;
  }
}

// ------------- Altitude + state update -------------
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

  // Low-pass filtered altitude & temp
  const float aAlt = 0.9f;
  filtAlt  = aAlt * filtAlt  + (1.0f - aAlt) * alt;
  filtTemp = 0.9f * filtTemp + 0.1f * temp;

  // Vertical velocity estimate
  float rawVel = (alt - lastAltRaw) / dt;   // m/s
  lastAltRaw = alt;
  velZ = 0.8f * velZ + 0.2f * rawVel;

  // Acc magnitude in g
  float accMag = sqrtf(last_ax * last_ax + last_ay * last_ay + last_az * last_az);
  const float g = 9.80665f;
  float accMg = accMag / g;

  uint32_t nowMs = millis();

  switch (flightState) {
    case FS_IDLE:
    case FS_PAD:
      // launch when we see > ~2 g total
      if (accMg > 2.0f) {
        flightState = FS_ASCENT;
        flightFlags |= FLAG_LAUNCH;
        tLaunchMs = nowMs;
      }
      break;

    case FS_ASCENT:
      // burnout → coast when accel near 1 g and still going up
      if (accMg < 1.2f && velZ > 1.0f) {
        flightState = FS_COAST;
      }
      break;

    case FS_COAST:
      // apogee / descent when velocity clearly negative
      if (velZ < -0.5f && filtAlt > 30.0f && (nowMs - tLaunchMs) > 1000) {
        flightState = FS_DESCENT;
        flightFlags |= FLAG_APOGEE;
        tApogeeMs = nowMs;
      }
      break;

    case FS_DESCENT:
      // landed when almost no movement and accel ~1 g
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

// ------------- Telemetry send -------------
void sendTelemetry() {
  TelemetryPacketV4 pkt;
  pkt.version = 4;
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

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  // Debug over USB if connected
  Serial.print("TX SEQ=");
  Serial.print(pkt.seq);
  Serial.print(" state=");
  Serial.print((int)pkt.state);
  Serial.print(" alt=");
  Serial.print(pkt.alt);
  Serial.print(" velZ=");
  Serial.println(pkt.vel_z);
}

void setup() {
  Serial.begin(115200);
  setupLoRa();
  setupAltimeter();
  setupImu();
}

void loop() {
  static uint32_t lastImuMicros = micros();
  static uint32_t lastStateMs   = millis();
  static uint32_t lastTelemMs   = millis();

  uint32_t nowMicros = micros();
  float dtImu = (nowMicros - lastImuMicros) * 1e-6f;
  if (dtImu <= 0.0f || dtImu > 0.05f) dtImu = 0.01f;
  lastImuMicros = nowMicros;
  updateImuEstimate(dtImu);

  uint32_t nowMs = millis();

  if (nowMs - lastStateMs >= 50) {   // 20 Hz state update
    float dtState = (nowMs - lastStateMs) / 1000.0f;
    lastStateMs = nowMs;
    updateAltAndState(dtState);
  }

  if (nowMs - lastTelemMs >= 50) {   // 20 Hz telemetry
    lastTelemMs = nowMs;
    sendTelemetry();
  }

  delay(1);
}
