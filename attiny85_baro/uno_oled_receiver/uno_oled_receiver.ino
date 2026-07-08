#include <Arduino.h>
#include <AltSoftSerial.h>
#include <Wire.h>
#include <U8g2lib.h>

// ----------------------------
//   USER CONFIG
// ----------------------------
#define BARO_RX_PIN 8      // AltSoftSerial RX is fixed on D8 for Uno
#define BARO_TX_PIN 9      // AltSoftSerial TX is fixed on D9 for Uno
#define BARO_BAUD   9600
#define DEBUG_BAUD  115200
#define OLED_IS_SH1106 1

#if OLED_IS_SH1106
U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
#else
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
#endif

AltSoftSerial baroSerial;

struct Telemetry {
  bool valid = false;
  char temp[12] = "--.--";
  long pressurePa = 0;
  long altitudeM = 0;
  long padM = 0;
  long maxM = 0;
  long deltaM = 0;
  long recordM = 0;
  unsigned long lastPacketMs = 0;
};

Telemetry telemetry;

char rxLine[96];
uint8_t rxLen = 0;
char lastScreen[8][17];
unsigned long lastHeartbeatMs = 0;

void initScreenCache() {
  for (uint8_t row = 0; row < 8; row++) {
    for (uint8_t col = 0; col < 16; col++) {
      lastScreen[row][col] = '\0';
    }
    lastScreen[row][16] = '\0';
  }
}

void writeRow(uint8_t row, const char* text) {
  char padded[17];
  uint8_t i = 0;
  while (i < 16 && text[i]) {
    padded[i] = text[i];
    i++;
  }
  while (i < 16) {
    padded[i++] = ' ';
  }
  padded[16] = '\0';

  if (strncmp(lastScreen[row], padded, 16) != 0) {
    display.setCursor(0, row);
    display.print(padded);
    strncpy(lastScreen[row], padded, 17);
  }
}

bool extractField(const char* src, const char* key, char* out, size_t outSize) {
  const char* p = strstr(src, key);
  if (!p) return false;
  p += strlen(key);

  size_t n = 0;
  while (*p && *p != ' ' && *p != '\r' && *p != '\n' && n + 1 < outSize) {
    out[n++] = *p++;
  }
  out[n] = '\0';
  return n > 0;
}

bool extractLong(const char* src, const char* key, long* out) {
  char buf[16];
  if (!extractField(src, key, buf, sizeof(buf))) return false;
  *out = atol(buf);
  return true;
}

bool parseTelemetryLine(const char* line, Telemetry* t) {
  char tempBuf[12];
  long pressurePa;
  long altitudeM;
  long padM;
  long maxM;
  long deltaM;
  long recordM;

  if (!extractField(line, "T=", tempBuf, sizeof(tempBuf))) return false;
  if (!extractLong(line, "P=", &pressurePa)) return false;
  if (!extractLong(line, "A=", &altitudeM)) return false;
  if (!extractLong(line, "PAD=", &padM)) return false;
  if (!extractLong(line, "MAX=", &maxM)) return false;
  if (!extractLong(line, "dA=", &deltaM)) return false;
  if (!extractLong(line, "REC=", &recordM)) return false;

  strncpy(t->temp, tempBuf, sizeof(t->temp) - 1);
  t->temp[sizeof(t->temp) - 1] = '\0';
  t->pressurePa = pressurePa;
  t->altitudeM = altitudeM;
  t->padM = padM;
  t->maxM = maxM;
  t->deltaM = deltaM;
  t->recordM = recordM;
  t->valid = true;
  t->lastPacketMs = millis();
  Serial.print("PARSED: ");
  Serial.println(line);
  return true;
}

void handleIncomingSerial() {
  while (baroSerial.available()) {
    char c = (char)baroSerial.read();

    Serial.print("RX 0x");
    if ((uint8_t)c < 16) Serial.print('0');
    Serial.print((uint8_t)c, HEX);
    Serial.print(" '");
    if (c >= 32 && c <= 126) {
      Serial.write(c);
    } else if (c == '\n') {
      Serial.print("\\n");
    } else if (c == '\r') {
      Serial.print("\\r");
    } else {
      Serial.print('.');
    }
    Serial.println("'");

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      rxLine[rxLen] = '\0';
      if (rxLen > 0) {
        Serial.print("LINE: ");
        Serial.println(rxLine);
        if (!parseTelemetryLine(rxLine, &telemetry)) {
          Serial.println("PARSE FAILED");
        }
      }
      rxLen = 0;
      continue;
    }

    if (rxLen + 1 < sizeof(rxLine)) {
      rxLine[rxLen++] = c;
    } else {
      rxLen = 0;
    }
  }
}

void drawWaitingScreen() {
  writeRow(0, "ATtiny85 Baro");
  writeRow(1, "");
  writeRow(2, "Waiting data");
  writeRow(3, "");
  writeRow(4, "Uno RX D8");
  writeRow(5, "");
  writeRow(6, "OLED I2C OK");
  writeRow(7, "");
}

void drawTelemetryScreen() {
  char buf[32];
  unsigned long ageMs = millis() - telemetry.lastPacketMs;

  snprintf(buf, sizeof(buf), "T:%sC  A:%ldm", telemetry.temp, telemetry.altitudeM);
  writeRow(0, buf);

  snprintf(buf, sizeof(buf), "P:%ld Pa", telemetry.pressurePa);
  writeRow(1, buf);

  snprintf(buf, sizeof(buf), "PAD:%ld  MAX:%ld", telemetry.padM, telemetry.maxM);
  writeRow(2, buf);

  snprintf(buf, sizeof(buf), "dA:%ld   REC:%ld", telemetry.deltaM, telemetry.recordM);
  writeRow(3, buf);

  if (ageMs < 2000) {
    snprintf(buf, sizeof(buf), "LINK OK  %lums", ageMs);
  } else {
    snprintf(buf, sizeof(buf), "STALE    %lums", ageMs);
  }
  writeRow(4, "");
  writeRow(5, buf);
  writeRow(6, "");
  writeRow(7, "");
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(200);
  Serial.println("Uno OLED receiver");
  Serial.print("Debug Serial @ ");
  Serial.println(DEBUG_BAUD);
  Serial.print("AltSoftSerial RX pin D");
  Serial.println(BARO_RX_PIN);
  Serial.print("Baro baud ");
  Serial.println(BARO_BAUD);

  pinMode(BARO_RX_PIN, INPUT);
  baroSerial.begin(BARO_BAUD);
  Serial.println("AltSoftSerial started");
  Serial.print("D");
  Serial.print(BARO_RX_PIN);
  Serial.print(" idle=");
  Serial.println(digitalRead(BARO_RX_PIN));

  Wire.begin();
  Serial.println("Wire.begin() OK");
  display.begin();
  display.setPowerSave(0);
  display.setFont(u8x8_font_chroma48medium8_r);
  initScreenCache();
  display.clearDisplay();
  drawWaitingScreen();
  Serial.println("OLED init done");
}

void loop() {
  handleIncomingSerial();

  if (millis() - lastHeartbeatMs >= 1000) {
    lastHeartbeatMs = millis();
    Serial.print("alive valid=");
    Serial.print(telemetry.valid ? "1" : "0");
    Serial.print(" avail=");
    Serial.print(baroSerial.available());
    Serial.print(" D");
    Serial.print(BARO_RX_PIN);
    Serial.print("=");
    Serial.println(digitalRead(BARO_RX_PIN));
  }

  if (telemetry.valid) {
    drawTelemetryScreen();
  } else {
    drawWaitingScreen();
  }
}
