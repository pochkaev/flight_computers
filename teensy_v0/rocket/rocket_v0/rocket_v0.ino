#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

// ------------- LoRa config -------------
const long LORA_FREQUENCY = 915E6;

// Pin mapping for Teensy 4.0
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

// Sea level pressure for altitude calculation (hPa)
const float SEA_LEVEL_HPA = 1013.25f;

// ------------- IMU (ICM-20602 / MPU-6050) -------------
enum ImuType {
  IMU_NONE = 0,
  IMU_ICM20602,
  IMU_MPU6050
};

ImuType imuType = IMU_NONE;
uint8_t imuAddr = 0x68;   // will be 0x68 or 0x69

// Common IMU registers
const uint8_t REG_WHO_AM_I  = 0x75;
const uint8_t REG_PWR_MGMT1 = 0x6B;
const uint8_t REG_SMPLRT_DIV= 0x19;
const uint8_t REG_GYRO_CONFIG   = 0x1B;
const uint8_t REG_ACCEL_CONFIG  = 0x1C;
const uint8_t REG_ACCEL_XOUT_H  = 0x3B;

// For ±4g / ±500 dps
const float ACCEL_SENS_4G = 8192.0f; // LSB/g
const float GYRO_SENS_500 = 65.5f;   // LSB/(deg/s)

// ------------- Telemetry packet -------------
struct TelemetryPacketV2 {
  uint8_t  version;   // 2
  uint32_t seq;
  uint32_t ms;
  float    alt;       // m
  float    temp;      // C
  float    ax;        // m/s^2
  float    ay;
  float    az;
  float    gx;        // deg/s
  float    gy;
  float    gz;
};

uint32_t packetSeq = 0;

// ------------- Helper: I2C read/write -------------
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
    // LoRa init failed, stop here for now
    while (true) { }
  }
}

// ------------- Altimeter autodetect -------------
void setupAltimeter() {
  Wire.begin(); // Teensy default I2C: SDA=18, SCL=19

  // Try BMP390 on 0x76 then 0x77
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

  // Try BMP280 on 0x76 then 0x77
  if (bmp280.begin(0x76)) {
    altimeterType = ALT_BMP280;
    bmp280.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,   // temp
      Adafruit_BMP280::SAMPLING_X4,   // pressure
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

  // Try BMP180
  if (bmp180.begin()) {
    altimeterType = ALT_BMP180;
    Serial.println("Altimeter: BMP180");
    return;
  }

  altimeterType = ALT_NONE;
  Serial.println("Altimeter: NONE");
}

bool readAltimeter(float &alt_m, float &temp_c) {
  if (altimeterType == ALT_NONE) {
    alt_m  = 0.0f;
    temp_c = 0.0f;
    return false;
  }

  if (altimeterType == ALT_BMP390) {
    if (!bmp390.performReading()) return false;
    float pressure_hPa = bmp390.pressure / 100.0f;
    temp_c = bmp390.temperature;
    alt_m = 44330.0f * (1.0f - pow(pressure_hPa / SEA_LEVEL_HPA, 0.1903f));
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

// ------------- IMU autodetect & setup -------------
void setupImu() {
  // we assume Wire.begin() already called in setupAltimeter()
  uint8_t addrs[2] = {0x68, 0x69};

  imuType = IMU_NONE;
  for (int i = 0; i < 2; i++) {
    uint8_t addr = addrs[i];

    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) {
      continue; // no device answer
    }

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
    } else {
      // Not one of our expected IDs, keep looking
      imuType = IMU_NONE;
    }
  }

  if (imuType == IMU_NONE) {
    Serial.println("IMU: NONE");
    return;
  }

  // Common init: wake up, set sample rate, ranges
  // Wake up: clear sleep bit in PWR_MGMT_1
  imuWrite8(REG_PWR_MGMT1, 0x00);   // use internal oscillator, sleep=0

  // Sample rate: SMPLRT_DIV
  imuWrite8(REG_SMPLRT_DIV, 0x07);  // 1kHz / (1+7) = 125 Hz

  // Gyro config: ±500 dps (bits[4:3] = 01)
  imuWrite8(REG_GYRO_CONFIG, 0x08);

  // Accel config: ±4 g (bits[4:3] = 01)
  imuWrite8(REG_ACCEL_CONFIG, 0x08);
}

// ------------- IMU read -------------
bool readImu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (imuType == IMU_NONE) {
    ax = ay = az = gx = gy = gz = 0.0f;
    return false;
  }

  // Read 14 bytes: accel(6) + temp(2) + gyro(6)
  Wire.beginTransmission(imuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t N = 14;
  Wire.requestFrom((int)imuAddr, (int)N);
  if (Wire.available() < N) {
    return false;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read(); // not used now
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Convert to physical units
  float ax_g = (float)rawAx / ACCEL_SENS_4G;
  float ay_g = (float)rawAy / ACCEL_SENS_4G;
  float az_g = (float)rawAz / ACCEL_SENS_4G;

  float gx_dps = (float)rawGx / GYRO_SENS_500;
  float gy_dps = (float)rawGy / GYRO_SENS_500;
  float gz_dps = (float)rawGz / GYRO_SENS_500;

  // Convert accel to m/s^2
  const float g = 9.80665f;
  ax = ax_g * g;
  ay = ay_g * g;
  az = az_g * g;

  gx = gx_dps;
  gy = gy_dps;
  gz = gz_dps;

  return true;
}

// ------------- Telemetry send -------------
void sendTelemetry() {
  TelemetryPacketV2 pkt;
  pkt.version = 2;
  pkt.seq     = packetSeq++;
  pkt.ms      = millis();

  // Altimeter
  float alt, temp;
  bool altOk = readAltimeter(alt, temp);
  pkt.alt  = altOk ? alt  : 0.0f;
  pkt.temp = altOk ? temp : 0.0f;

  // IMU
  float ax, ay, az, gx, gy, gz;
  bool imuOk = readImu(ax, ay, az, gx, gy, gz);
  pkt.ax = imuOk ? ax : 0.0f;
  pkt.ay = imuOk ? ay : 0.0f;
  pkt.az = imuOk ? az : 0.0f;
  pkt.gx = imuOk ? gx : 0.0f;
  pkt.gy = imuOk ? gy : 0.0f;
  pkt.gz = imuOk ? gz : 0.0f;

  // Send over LoRa as raw bytes
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  // Optional debug
  Serial.print("TX SEQ=");
  Serial.print(pkt.seq);
  Serial.print(" alt=");
  Serial.print(pkt.alt);
  Serial.print(" temp=");
  Serial.print(pkt.temp);
  Serial.print(" ax=");
  Serial.print(pkt.ax);
  Serial.print(" ay=");
  Serial.print(pkt.ay);
  Serial.print(" az=");
  Serial.print(pkt.az);
  Serial.print(" gx=");
  Serial.print(pkt.gx);
  Serial.print(" gy=");
  Serial.print(pkt.gy);
  Serial.print(" gz=");
  Serial.print(pkt.gz);
  Serial.print(" altOK=");
  Serial.print(altOk ? "Y" : "N");
  Serial.print(" imuOK=");
  Serial.println(imuOk ? "Y" : "N");
}

void setup() {
  Serial.begin(115200);  // no waiting for USB
  setupLoRa();
  setupAltimeter();
  setupImu();
}

void loop() {
  sendTelemetry();
  delay(500); // 2 Hz
}
