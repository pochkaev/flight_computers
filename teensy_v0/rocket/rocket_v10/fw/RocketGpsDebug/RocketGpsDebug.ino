#include <Arduino.h>
#include "src/TinyGPS++.h"

// Rocket GPS-only debug firmware for Teensy 4.1.
// Keeps the test isolated from LoRa, IMU, barometer, SD, and NAND.

#define GPS_SERIAL Serial1
#define GPS_BAUD 9600
#define STATUS_LED_PIN 3
#define STATUS_PRINT_MS 1000
#define GPS_STALE_MS 3000

TinyGPSPlus gps;

static uint32_t lastPrintMs = 0;
static uint32_t lastGpsByteMs = 0;
static uint32_t lastFixMs = 0;
static bool ledState = false;
static char nmeaLine[128];
static uint8_t nmeaLen = 0;

static void printHexByte(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

static void sendUbx(const uint8_t *msg, size_t len) {
  GPS_SERIAL.write(msg, len);
  GPS_SERIAL.flush();
}

static void sendUbxPacket(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t payloadLen) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;

  auto updateChecksum = [&](uint8_t b) {
    ckA = (uint8_t)(ckA + b);
    ckB = (uint8_t)(ckB + ckA);
  };

  GPS_SERIAL.write(0xB5);
  GPS_SERIAL.write(0x62);

  GPS_SERIAL.write(cls);
  updateChecksum(cls);

  GPS_SERIAL.write(id);
  updateChecksum(id);

  uint8_t lenLo = (uint8_t)(payloadLen & 0xFF);
  uint8_t lenHi = (uint8_t)((payloadLen >> 8) & 0xFF);
  GPS_SERIAL.write(lenLo);
  updateChecksum(lenLo);
  GPS_SERIAL.write(lenHi);
  updateChecksum(lenHi);

  for (uint16_t i = 0; i < payloadLen; ++i) {
    GPS_SERIAL.write(payload[i]);
    updateChecksum(payload[i]);
  }

  GPS_SERIAL.write(ckA);
  GPS_SERIAL.write(ckB);
  GPS_SERIAL.flush();
}

static bool waitForUbxAck(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
  uint8_t ack[10] = {0};
  size_t pos = 0;
  uint32_t startMs = millis();

  while ((millis() - startMs) < timeoutMs) {
    while (GPS_SERIAL.available() > 0) {
      uint8_t c = (uint8_t)GPS_SERIAL.read();
      handleNmeaByte((char)c);
      gps.encode((char)c);
      lastGpsByteMs = millis();

      if (pos == 0 && c != 0xB5) continue;
      if (pos == 1 && c != 0x62) {
        pos = 0;
        continue;
      }

      ack[pos++] = c;
      if (pos >= sizeof(ack)) {
        if (ack[2] == 0x05 && ack[3] == 0x01 && ack[6] == cls && ack[7] == id) {
          Serial.print("UBX ACK ");
          printHexByte(cls);
          Serial.print(' ');
          printHexByte(id);
          Serial.println();
          return true;
        }
        if (ack[2] == 0x05 && ack[3] == 0x00 && ack[6] == cls && ack[7] == id) {
          Serial.print("UBX NACK ");
          printHexByte(cls);
          Serial.print(' ');
          printHexByte(id);
          Serial.println();
          return false;
        }
        pos = 0;
      }
    }
  }

  Serial.print("UBX ACK TIMEOUT ");
  printHexByte(cls);
  Serial.print(' ');
  printHexByte(id);
  Serial.println();
  return false;
}

static void configureUbloxMinimal() {
  // CFG-NAV5: set dynamic model to airborne <1g, keep other fields default-ish.
  const uint8_t cfgNav5[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
    0x01, 0x00,       // mask: apply dyn model
    0x06,             // dynModel: airborne <1g
    0x03,             // fixMode: auto 2D/3D
    0x00, 0x00, 0x00, 0x00,
    0x10, 0x27, 0x00, 0x00,
    0x05, 0x00,
    0xFA, 0x00,
    0xFA, 0x00,
    0x64, 0x00,
    0x2C, 0x01,
    0x00, 0x00,
    0x00, 0x00,
    0x10, 0x27, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x13, 0x76
  };

  // CFG-RATE: 1000 ms measurement rate, GPS time reference.
  const uint8_t cfgRate1Hz[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xE8, 0x03,       // measRate 1000 ms
    0x01, 0x00,       // navRate
    0x01, 0x00,       // timeRef = GPS
    0x01, 0x39
  };

  Serial.println("UBX minimal config start");
  sendUbx(cfgNav5, sizeof(cfgNav5));
  waitForUbxAck(0x06, 0x24, 1500);
  sendUbx(cfgRate1Hz, sizeof(cfgRate1Hz));
  waitForUbxAck(0x06, 0x08, 1500);
  Serial.println("UBX minimal config end");
}

static void factoryResetUblox() {
  // UBX-CFG-CFG:
  // clear permanent config to defaults, then load defaults into current config.
  // Lower 16 bits are the documented config sections on older u-blox generations.
  const uint8_t cfgCfgFactory[] = {
    0xFF, 0xFF, 0x00, 0x00, // clearMask
    0x00, 0x00, 0x00, 0x00, // saveMask
    0xFF, 0xFF, 0x00, 0x00, // loadMask
    0x17                    // deviceMask: BBR + Flash + EEPROM + SPI flash
  };

  Serial.println("UBX factory reset start");
  sendUbxPacket(0x06, 0x09, cfgCfgFactory, sizeof(cfgCfgFactory));
  waitForUbxAck(0x06, 0x09, 2000);
  Serial.println("UBX factory reset end");
}

static void coldRestartUblox() {
  // UBX-CFG-RST: controlled software reset, cold start.
  const uint8_t cfgRstCold[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
    0xFF, 0xFF, // navBbrMask: clear all backup data
    0x09,       // resetMode: controlled software reset (GNSS only)
    0x00,       // reserved
    0x15, 0x6F
  };

  Serial.println("UBX cold restart start");
  sendUbx(cfgRstCold, sizeof(cfgRstCold));
  waitForUbxAck(0x06, 0x04, 1500);
  Serial.println("UBX cold restart end");
}

static bool gpsBytesFresh() {
  return lastGpsByteMs != 0 && (millis() - lastGpsByteMs) <= GPS_STALE_MS;
}

static void updateLed() {
  if (gps.location.isValid()) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }

  if (!gpsBytesFresh()) {
    digitalWrite(STATUS_LED_PIN, LOW);
    return;
  }

  ledState = !ledState;
  digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
}

static void printStatus() {
  Serial.print("gpsdbg ms=");
  Serial.print(millis());
  Serial.print(" bytesFresh=");
  Serial.print(gpsBytesFresh() ? "1" : "0");
  Serial.print(" chars=");
  Serial.print((unsigned long)gps.charsProcessed());
  Serial.print(" pass=");
  Serial.print((unsigned long)gps.passedChecksum());
  Serial.print(" fail=");
  Serial.print((unsigned long)gps.failedChecksum());
  Serial.print(" fixSent=");
  Serial.print((unsigned long)gps.sentencesWithFix());
  Serial.print(" locValid=");
  Serial.print(gps.location.isValid() ? "1" : "0");
  Serial.print(" altValid=");
  Serial.print(gps.altitude.isValid() ? "1" : "0");
  Serial.print(" dateValid=");
  Serial.print(gps.date.isValid() ? "1" : "0");
  Serial.print(" timeValid=");
  Serial.print(gps.time.isValid() ? "1" : "0");
  Serial.print(" sats=");
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print("-1");
  }
  Serial.print(" hdop=");
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.value() * 0.01f, 2);
  } else {
    Serial.print("-1");
  }
  Serial.print(" lat=");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 7);
  } else {
    Serial.print("nan");
  }
  Serial.print(" lon=");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lng(), 7);
  } else {
    Serial.print("nan");
  }
  Serial.print(" altM=");
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters(), 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" spdMps=");
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.mps(), 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" lastByteAgeMs=");
  if (lastGpsByteMs != 0) {
    Serial.print((unsigned long)(millis() - lastGpsByteMs));
  } else {
    Serial.print("-1");
  }
  Serial.print(" fixAgeMs=");
  if (gps.location.isValid()) {
    Serial.print("0");
  } else if (lastFixMs != 0) {
    Serial.print((unsigned long)(millis() - lastFixMs));
  } else {
    Serial.print("-1");
  }
  Serial.println();
}

static void handleNmeaByte(char c) {
  if (c == '\r') return;

  if (c == '$') {
    nmeaLen = 0;
    if (nmeaLen < sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaLen++] = c;
    }
    return;
  }

  if (c == '\n') {
    if (nmeaLen > 0) {
      nmeaLine[nmeaLen] = '\0';
      Serial.print("NMEA ");
      Serial.println(nmeaLine);
    }
    nmeaLen = 0;
    return;
  }

  if (nmeaLen > 0 && nmeaLen < sizeof(nmeaLine) - 1) {
    nmeaLine[nmeaLen++] = c;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  GPS_SERIAL.begin(GPS_BAUD);

  Serial.println("RocketGpsDebug ready");
  delay(200);
  factoryResetUblox();
  delay(1500);
  coldRestartUblox();
  delay(2000);
}

void loop() {
  while (GPS_SERIAL.available() > 0) {
    char c = (char)GPS_SERIAL.read();
    handleNmeaByte(c);
    gps.encode(c);
    lastGpsByteMs = millis();
  }

  if (gps.location.isValid()) {
    lastFixMs = millis();
  }

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastPrintMs) >= STATUS_PRINT_MS) {
    lastPrintMs = nowMs;
    printStatus();
  }

  updateLed();
}
