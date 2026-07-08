#include <Arduino.h>
#include <Wire.h>

static const uint8_t kLsm9ds1AgAddrs[] = {0x6A, 0x6B};
static const uint8_t kLsm9ds1MagAddrs[] = {0x1C, 0x1E};
static const uint8_t kWhoAmIReg = 0x0F;

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  value = Wire.read();
  return true;
}

static void printDivider() {
  Serial.println("--------------------------------------------------");
}

static void scanBus() {
  Serial.println("I2C scan:");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    if (i2cProbe(addr)) {
      Serial.print("  found 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      ++found;
    }
  }
  if (!found) {
    Serial.println("  no I2C devices found");
  }
}

static void probeLsm9ds1() {
  Serial.println("LSM9DS1 direct probe:");

  bool anyAg = false;
  for (uint8_t addr : kLsm9ds1AgAddrs) {
    Serial.print("  AG  0x");
    if (addr < 16) Serial.print('0');
    Serial.print(addr, HEX);
    if (!i2cProbe(addr)) {
      Serial.println("  no response");
      continue;
    }
    anyAg = true;
    uint8_t who = 0;
    if (i2cReadReg(addr, kWhoAmIReg, who)) {
      Serial.print("  WHO_AM_I=0x");
      if (who < 16) Serial.print('0');
      Serial.println(who, HEX);
    } else {
      Serial.println("  read failed");
    }
  }

  bool anyMag = false;
  for (uint8_t addr : kLsm9ds1MagAddrs) {
    Serial.print("  MAG 0x");
    if (addr < 16) Serial.print('0');
    Serial.print(addr, HEX);
    if (!i2cProbe(addr)) {
      Serial.println("  no response");
      continue;
    }
    anyMag = true;
    uint8_t who = 0;
    if (i2cReadReg(addr, kWhoAmIReg, who)) {
      Serial.print("  WHO_AM_I=0x");
      if (who < 16) Serial.print('0');
      Serial.println(who, HEX);
    } else {
      Serial.println("  read failed");
    }
  }

  Serial.println("Expected LSM9DS1 values:");
  Serial.println("  AG  WHO_AM_I = 0x68");
  Serial.println("  MAG WHO_AM_I = 0x3D");

  if (!anyAg && !anyMag) {
    Serial.println("Result: no LSM9DS1 address responded on the bus");
  } else if (!anyAg || !anyMag) {
    Serial.println("Result: only part of the chip responded; check address straps and wiring");
  } else {
    Serial.println("Result: both AG and MAG addresses responded");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }

  delay(500);
  Wire.begin();
  Wire.setClock(100000);
}

void loop() {
  printDivider();
  Serial.println("Teensy 4.1 IMU diagnostic");
  Serial.println("Bus: Wire on SDA=18, SCL=19");
  printDivider();
  scanBus();
  printDivider();
  probeLsm9ds1();
  printDivider();
  delay(3000);
}
