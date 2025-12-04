#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>
#include <math.h>
#include <SD.h>
#include <U8g2lib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----------------- LoRa config -----------------
const long LORA_FREQUENCY = 915E6;
const int  LORA_CS_PIN    = 10;
const int  LORA_RST_PIN   = 9;
const int  LORA_DIO0_PIN  = 2;

const uint8_t PKT_TYPE_FLIGHT_V7 = 0x01;
const uint8_t PKT_TYPE_NAV_V7    = 0x02;

// ----------------- SD card config -------------
const int SD_CS_PIN = 4;
File logFile;
bool sdOk = false;
uint32_t sdLogLines = 0;
uint32_t nextFlightIndex = 1;

// Logging mode / GPS time logic
enum LogMode {
  LOG_MODE_WAIT = 0,     // waiting for GPS time or timeout
  LOG_MODE_UNSTAMPED,    // using flightN.log
  LOG_MODE_STAMPED       // using YYYYMMDD_HHMMSS_flightN.log
};

LogMode logMode = LOG_MODE_WAIT;
uint32_t bootTimeMs = 0;
bool gpsDateTimeValid = false;
const uint32_t GPS_WAIT_MS = 60000; // 60s to wait for GPS time before fallback

// ----------------- OLED (SH1106 + U8g2) -------
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ----------------- Page button ----------------
const int BUTTON_PIN = 5;   // connect to GND when pressed
const uint8_t NUM_PAGES = 5;
volatile uint8_t currentPage = 0;

// ----------------- GPS (ground) ----------------
TinyGPSPlus gps;   // ground GPS on Serial1

bool    gpsHasFix    = false;
uint8_t gpsFixType   = 0;
uint8_t gpsSats      = 0;
float   gpsLatDeg    = 0.0f;
float   gpsLonDeg    = 0.0f;
float   gpsAltM      = 0.0f;
float   gpsSpeedMps  = 0.0f;
float   gpsHdop      = 99.99f;

bool     haveGoodFix   = false;
float    lastFixLatDeg = 0.0f;
float    lastFixLonDeg = 0.0f;
float    lastFixAltM   = 0.0f;
uint32_t lastFixTimeMs = 0;

// ----------------- BMP180 (Adafruit_BMP085) ----
Adafruit_BMP085 bmp180;
bool hasBaro = false;

// ----------------- Flight states ---------------
enum FlightState : uint8_t {
  FS_IDLE = 0,
  FS_PAD,
  FS_ASCENT,
  FS_COAST,
  FS_DESCENT,
  FS_LANDED,
  FS_ABORT
};

const char* stateName(uint8_t s) {
  switch (s) {
    case FS_IDLE:    return "IDLE";
    case FS_PAD:     return "PAD";
    case FS_ASCENT:  return "ASCENT";
    case FS_COAST:   return "COAST";
    case FS_DESCENT: return "DESCENT";
    case FS_LANDED:  return "LANDED";
    case FS_ABORT:   return "ABORT";
    default:         return "UNKNOWN";
  }
}

const uint16_t FLAG_LAUNCH  = 1 << 0;
const uint16_t FLAG_APOGEE  = 1 << 1;
const uint16_t FLAG_LANDED  = 1 << 2;

// ----------------- Telemetry V7 structs --------
struct __attribute__((packed)) FlightPacketV7 {
  uint8_t  version;    // 7
  uint8_t  state;      // FlightState
  uint16_t flags;      // bitfield
  uint32_t seq;
  uint32_t ms;

  int32_t  alt_cm;
  int16_t  vel_cms;

  int16_t  ax_cms2;
  int16_t  ay_cms2;
  int16_t  az_cms2;

  int16_t  gx_cdeg;
  int16_t  gy_cdeg;
  int16_t  gz_cdeg;

  int16_t  roll_cdeg;
  int16_t  pitch_cdeg;
};
static_assert(sizeof(FlightPacketV7) == 34, "FlightPacketV7 must be 34 bytes");

struct __attribute__((packed)) NavPacketV7 {
  uint8_t  version;       // 7
  uint8_t  gps_fix_type;  // 0,2,3
  uint8_t  gps_sats;
  uint8_t  gps_hdop_x10;  // hdop * 10

  uint32_t seq;
  uint32_t ms;

  int32_t  gps_lat_e7;
  int32_t  gps_lon_e7;
  int32_t  gps_alt_cm;
  int32_t  baro_alt_cm;

  uint32_t last_fix_age_ms;
};
static_assert(sizeof(NavPacketV7) == 32, "NavPacketV7 must be 32 bytes");

// LoRa receive buffers
volatile bool       loraFlightPending = false;
volatile bool       loraNavPending    = false;
FlightPacketV7      loraFlightPkt;
NavPacketV7         loraNavPkt;
int                 loraFlightRssi = 0;
int                 loraNavRssi    = 0;

// ----------------- Display snapshot values -----
// Rocket dynamics
float   dispRocketAltM   = 0.0f;
float   dispRocketVelMps = 0.0f;
uint8_t dispRocketState  = FS_IDLE;
int     dispFlightRssi   = 0;

float   dispAxMs2 = 0.0f;
float   dispAyMs2 = 0.0f;
float   dispAzMs2 = 0.0f;
float   dispGxDps = 0.0f;
float   dispGyDps = 0.0f;
float   dispGzDps = 0.0f;
float   dispRollDeg  = 0.0f;
float   dispPitchDeg = 0.0f;
uint16_t dispFlightFlags = 0;

// Rocket nav
int     dispNavRssi      = 0;
uint8_t dispGpsFixType   = 0;
uint8_t dispGpsSats      = 0;
float   dispGpsHdop      = 99.9f;
double  dispGpsLat       = 0.0;
double  dispGpsLon       = 0.0;
float   dispGpsAltM      = 0.0f;
float   dispRocketBaroM  = 0.0f;
uint32_t dispLastFixAgeMs = 0;

// Ground PAD alt
float   dispGroundGpsAltM  = 0.0f;
float   dispGroundBaroAltM = 0.0f;

// Distance ground ↔ rocket
float   dispDistanceM = NAN;

// ----------------- LoRa stats & phase tracking -
uint32_t flightPktCount   = 0;
uint32_t navPktCount      = 0;
uint32_t flightMissed     = 0;
uint32_t navMissed        = 0;
uint32_t lastFlightSeq    = 0;
uint32_t lastNavSeq       = 0;
bool     haveLastFlightSeq = false;
bool     haveLastNavSeq    = false;

int      dispLoRaRssi     = -200;   // recent RSSI (dBm)
uint32_t lastFlightMsDisp = 0;
uint32_t lastNavMsDisp    = 0;

// rocket phase via flags
bool rocketLaunched = false;
bool rocketLanded   = false;

// last time we got *any* rocket packet
uint32_t lastRocketRxMs = 0;

// decimation counters for pre/post flight logging
uint32_t prePostFlightFlightLogDecim = 0;
uint32_t prePostFlightNavLogDecim    = 0;

// ----------------- SD helpers ------------------

// Extract <number> from "...flight<number>.log"
int extractFlightIndex(const char *name) {
  if (!name) return -1;

  const char *p = strstr(name, "flight");
  if (!p) return -1;

  p += 6; // skip "flight"

  if (*p < '0' || *p > '9') {
    return -1;
  }

  long idx = 0;
  while (*p >= '0' && *p <= '9') {
    idx = idx * 10 + (*p - '0');
    p++;
    if (idx > 1000000) break;
  }

  if (idx <= 0) return -1;
  return (int)idx;
}

void scanForExistingLogs() {
  nextFlightIndex = 1;
  File root = SD.open("/");
  if (!root) {
    Serial.println("SD: cannot open root");
    return;
  }

  uint32_t maxIndex = 0;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      const char *name = entry.name();
      int idx = extractFlightIndex(name);
      if (idx > 0 && (uint32_t)idx > maxIndex) {
        maxIndex = (uint32_t)idx;
      }
    }
    entry.close();
  }

  root.close();

  nextFlightIndex = maxIndex + 1;
  Serial.print("SD: next flight index = ");
  Serial.println(nextFlightIndex);
}

// Create log file lazily when first log line is written.
void ensureLogFile() {
  if (!sdOk) return;
  if (logFile) return;

  uint32_t now = millis();

  // Decide mode if still waiting
  if (logMode == LOG_MODE_WAIT) {
    if (gpsDateTimeValid) {
      logMode = LOG_MODE_STAMPED;
      Serial.println("SD: opening stamped log (GPS time valid)");
    } else if (now - bootTimeMs > GPS_WAIT_MS) {
      logMode = LOG_MODE_UNSTAMPED;
      Serial.println("SD: GPS timeout -> using un-stamped log");
    } else {
      // still waiting: do not open yet
      return;
    }
  }

  char filename[48];
  uint32_t idx = nextFlightIndex++;
  bool useStamped = (logMode == LOG_MODE_STAMPED) && gpsDateTimeValid;

  if (useStamped) {
    int year   = gps.date.year();
    int month  = gps.date.month();
    int day    = gps.date.day();
    int hour   = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    snprintf(filename, sizeof(filename),
             "%04d%02d%02d_%02d%02d%02d_flight%lu.log",
             year, month, day,
             hour, minute, second,
             (unsigned long)idx);
  } else {
    snprintf(filename, sizeof(filename),
             "flight%lu.log",
             (unsigned long)idx);
  }

  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.print("SD: open failed: ");
    Serial.println(filename);
    sdOk = false;
    return;
  }

  Serial.print("SD: logging to ");
  Serial.println(filename);

  logFile.println("# Ground log start");
  logFile.flush();
  sdLogLines++;
}

void logLineToSd(const char *line) {
  if (!sdOk) return;

  if (!logFile) {
    ensureLogFile();
    if (!logFile) return;  // still waiting or failed
  }

  logFile.println(line);
  logFile.flush();
  sdLogLines++;
}

void setupSd() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);  // deselect SD

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD: init failed");
    sdOk = false;
    return;
  }

  sdOk = true;
  Serial.println("SD: init OK");

  scanForExistingLogs();
  logMode = LOG_MODE_WAIT;
}

// ----------------- OLED (U8g2) helpers ---------
void setupOled() {
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 12, "Ground V7");
  display.drawStr(0, 24, "SH1106 OK");
  display.drawStr(0, 36, "BTN: change page");
  display.sendBuffer();
}

void drawHeader(const char *title, uint8_t pageIdx) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s  P%u", title, (unsigned)pageIdx);

  display.setDrawColor(1);
  display.drawBox(0, 0, 128, 11);
  display.setDrawColor(0);
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(2, 9, buf);
  display.setDrawColor(1);
}

inline int rowY(uint8_t rowIdx) {
  return 20 + rowIdx * 10;
}

void drawPageOverview() {
  char buf[48];
  display.setFont(u8g2_font_6x10_tr);

  drawHeader("Overview", 0);

  snprintf(buf, sizeof(buf), "State:%s  F_Rssi:%d",
           stateName(dispRocketState), dispFlightRssi);
  display.drawStr(0, rowY(0), buf);

  snprintf(buf, sizeof(buf), "Alt: %.1f m  Vel:%.1f",
           dispRocketAltM, dispRocketVelMps);
  display.drawStr(0, rowY(1), buf);

  snprintf(buf, sizeof(buf), "GPSfix:%u  Sats:%u  H:%.1f",
           (unsigned)dispGpsFixType,
           (unsigned)dispGpsSats,
           dispGpsHdop);
  display.drawStr(0, rowY(2), buf);

  snprintf(buf, sizeof(buf), "Rkt Lat:%.3f", dispGpsLat);
  display.drawStr(0, rowY(3), buf);

  snprintf(buf, sizeof(buf), "Rkt Lon:%.3f", dispGpsLon);
  display.drawStr(0, rowY(4), buf);
}

void drawPageDynamics() {
  char buf[48];
  display.setFont(u8g2_font_6x10_tr);

  drawHeader("Dynamics", 1);

  snprintf(buf, sizeof(buf), "Ax:%.1f  Ay:%.1f  Az:%.1f",
           dispAxMs2, dispAyMs2, dispAzMs2);
  display.drawStr(0, rowY(0), buf);

  snprintf(buf, sizeof(buf), "Gx:%.1f  Gy:%.1f  Gz:%.1f",
           dispGxDps, dispGyDps, dispGzDps);
  display.drawStr(0, rowY(1), buf);

  snprintf(buf, sizeof(buf), "Roll:%.1f  Pitch:%.1f",
           dispRollDeg, dispPitchDeg);
  display.drawStr(0, rowY(2), buf);

  snprintf(buf, sizeof(buf), "Flags:0x%04X", dispFlightFlags);
  display.drawStr(0, rowY(3), buf);

  snprintf(buf, sizeof(buf), "NavRssi:%d", dispNavRssi);
  display.drawStr(0, rowY(4), buf);
}

void formatDistance(char *buf, size_t n, float distM) {
  if (!isfinite(distM) || distM < 0.0f) {
    snprintf(buf, n, "Dist: ---");
    return;
  }
  if (distM < 1000.0f) {
    snprintf(buf, n, "Dist: %.0f m", distM);
  } else {
    float km = distM / 1000.0f;
    snprintf(buf, n, "Dist: %.2f km", km);
  }
}

void drawPageNav() {
  char buf[48];
  display.setFont(u8g2_font_6x10_tr);

  drawHeader("Nav / Alt", 2);

  snprintf(buf, sizeof(buf), "Rkt GPS:  %.1f m", dispGpsAltM);
  display.drawStr(0, rowY(0), buf);

  snprintf(buf, sizeof(buf), "Gnd GPS:  %.1f m", dispGroundGpsAltM);
  display.drawStr(0, rowY(1), buf);

  formatDistance(buf, sizeof(buf), dispDistanceM);
  display.drawStr(0, rowY(2), buf);

  snprintf(buf, sizeof(buf), "Rkt Baro: %.1f m", dispRocketBaroM);
  display.drawStr(0, rowY(3), buf);

  snprintf(buf, sizeof(buf), "Gnd Baro: %.1f m", dispGroundBaroAltM);
  display.drawStr(0, rowY(4), buf);
}

int rssiToWidth(int rssi) {
  if (rssi < -120) rssi = -120;
  if (rssi > -40)  rssi = -40;
  float frac = (float)(rssi + 120) / 80.0f;  // 0..1
  int w = (int)(frac * 120.0f + 0.5f);
  if (w < 0) w = 0;
  if (w > 120) w = 120;
  return w;
}

const char* rssiQualityText(int rssi) {
  if (rssi < -110) return "BAD";
  if (rssi < -95)  return "OK";
  if (rssi < -80)  return "GOOD";
  return "EXCELLENT";
}

void drawPageLoRa() {
  char buf[48];
  display.setFont(u8g2_font_6x10_tr);

  drawHeader("LoRa / SD", 3);

  snprintf(buf, sizeof(buf), "F pkts:%lu  miss:%lu",
           (unsigned long)flightPktCount,
           (unsigned long)flightMissed);
  display.drawStr(0, rowY(0), buf);

  snprintf(buf, sizeof(buf), "N pkts:%lu  miss:%lu",
           (unsigned long)navPktCount,
           (unsigned long)navMissed);
  display.drawStr(0, rowY(1), buf);

  uint32_t now = millis();
  uint32_t ageF = lastFlightMsDisp ? (now - lastFlightMsDisp) / 1000 : 0xFFFFFFFFUL;
  uint32_t ageN = lastNavMsDisp    ? (now - lastNavMsDisp) / 1000    : 0xFFFFFFFFUL;

  if (lastFlightMsDisp) {
    snprintf(buf, sizeof(buf), "Last F: %lus ago", (unsigned long)ageF);
  } else {
    snprintf(buf, sizeof(buf), "Last F: ---");
  }
  display.drawStr(0, rowY(2), buf);

  if (lastNavMsDisp) {
    snprintf(buf, sizeof(buf), "Last N: %lus ago", (unsigned long)ageN);
  } else {
    snprintf(buf, sizeof(buf), "Last N: ---");
  }
  display.drawStr(0, rowY(3), buf);

  snprintf(buf, sizeof(buf), "SD lines:%lu %s",
           (unsigned long)sdLogLines,
           sdOk ? "OK" : "ERR");
  display.drawStr(0, rowY(4), buf);
}

void drawPageSignal() {
  char buf[48];
  display.setFont(u8g2_font_6x10_tr);

  drawHeader("Signal", 4);

  snprintf(buf, sizeof(buf), "RSSI: %d dBm", dispLoRaRssi);
  display.drawStr(10, rowY(0), buf);

  const char* qtext = rssiQualityText(dispLoRaRssi);
  snprintf(buf, sizeof(buf), "Link: %s", qtext);
  display.drawStr(10, rowY(1), buf);

  int barX = 4;
  int barY = rowY(2) + 6;
  int barW = 120;
  int barH = 14;

  int fillW = rssiToWidth(dispLoRaRssi);

  display.drawFrame(barX, barY, barW, barH);
  if (fillW > 2) {
    display.drawBox(barX + 1, barY + 1, fillW - 2, barH - 2);
  }

  display.drawStr(4, 62, "-120");
  display.drawStr(52, 62, "-80");
  display.drawStr(100, 62, "-40");
}

void updateOled() {
  static uint32_t lastUpdateMs = 0;
  uint32_t now = millis();
  if (now - lastUpdateMs < 150) return;  // ~6-7 Hz
  lastUpdateMs = now;

  display.clearBuffer();

  switch (currentPage) {
    case 0: drawPageOverview(); break;
    case 1: drawPageDynamics(); break;
    case 2: drawPageNav();      break;
    case 3: drawPageLoRa();     break;
    case 4: drawPageSignal();   break;
    default: currentPage = 0;   drawPageOverview(); break;
  }

  display.sendBuffer();
}

// ----------------- Button handling -------------
void updateButton() {
  static uint8_t lastStableState = HIGH;
  static uint8_t lastReading     = HIGH;
  static uint32_t lastDebounceTime = 0;
  const uint32_t debounceDelay = 25;

  uint8_t reading = digitalRead(BUTTON_PIN);
  uint32_t now = millis();

  if (reading != lastReading) {
    lastDebounceTime = now;
    lastReading = reading;
  }

  if ((now - lastDebounceTime) > debounceDelay) {
    if (reading != lastStableState) {
      lastStableState = reading;
      if (lastStableState == LOW) {
        currentPage = (currentPage + 1) % NUM_PAGES;
      }
    }
  }
}

// ----------------- LoRa RX callback ------------
void onLoraReceive(int packetSize) {
  if (packetSize <= 0) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  int first = LoRa.read();
  if (first < 0) {
    while (LoRa.available()) LoRa.read();
    return;
  }
  uint8_t pktType = (uint8_t)first;
  int remaining = packetSize - 1;

  if (pktType == PKT_TYPE_FLIGHT_V7) {
    if (remaining != (int)sizeof(FlightPacketV7)) {
      while (LoRa.available()) LoRa.read();
      return;
    }

    uint8_t *p = (uint8_t*)&loraFlightPkt;
    int i = 0;
    while (LoRa.available() && i < remaining) {
      p[i++] = LoRa.read();
    }
    if (i != remaining) {
      while (LoRa.available()) LoRa.read();
      return;
    }

    loraFlightRssi    = LoRa.packetRssi();
    loraFlightPending = true;
    return;
  }
  else if (pktType == PKT_TYPE_NAV_V7) {
    if (remaining != (int)sizeof(NavPacketV7)) {
      while (LoRa.available()) LoRa.read();
      return;
    }

    uint8_t *p = (uint8_t*)&loraNavPkt;
    int i = 0;
    while (LoRa.available() && i < remaining) {
      p[i++] = LoRa.read();
    }
    if (i != remaining) {
      while (LoRa.available()) LoRa.read();
      return;
    }

    loraNavRssi    = LoRa.packetRssi();
    loraNavPending = true;
    return;
  }
  else {
    while (LoRa.available()) LoRa.read();
  }
}

// ----------------- LoRa setup ------------------
void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed");
    while (true) {}
  }

  LoRa.setSPIFrequency(8E6);

  LoRa.onReceive(onLoraReceive);
  LoRa.receive();

  Serial.println("LoRa RX initialized");
}

// ----------------- Baro helpers ----------------
void setupBaro() {
  if (bmp180.begin()) {
    hasBaro = true;
    Serial.println("Ground: BMP180 detected");
  } else {
    hasBaro = false;
    Serial.println("Ground: BMP180 not found");
  }
}

bool readGroundBaro(float &alt_m) {
  if (!hasBaro) return false;
  alt_m = bmp180.readAltitude(101325.0); // Pa
  return true;
}

// ----------------- GPS update ------------------
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

  // Track GPS date/time appearance
  bool nowHasDateTime = gps.date.isValid() && gps.time.isValid();
  static bool hadDateTimeBefore = false;
  if (nowHasDateTime && !hadDateTimeBefore) {
    gpsDateTimeValid = true;
    Serial.println("GPS date/time became valid");

    // If we were logging to un-stamped file, close it and switch
    if (logMode == LOG_MODE_UNSTAMPED) {
      if (logFile) {
        logFile.flush();
        logFile.close();
      }
      logFile = File();
      logMode = LOG_MODE_STAMPED;
      Serial.println("Switching to stamped log mode; next log -> new dated file");
    }
  }
  hadDateTimeBefore = nowHasDateTime;

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
    if (locValid) Serial.print(gpsLonDeg, 6); else Serial.print(0.0);
    Serial.print("  alt=");
    if (altValid) Serial.print(gpsAltM); else Serial.print(0.0);
    Serial.println();
  }
}

// ----------------- PAD status logging ----------
void maybePrintPadLine() {
  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();

  uint32_t interval = 1000;
  bool inFlightPhase = rocketLaunched && !rocketLanded;

  if (inFlightPhase) {
    interval = 1000;      // 1s during flight
  } else if (rocketLanded) {
    interval = 10000;     // 10s after landing
  } else {
    interval = 5000;      // 5s pre-launch
  }

  if ((now - lastRocketRxMs) > 30000) {
    interval = 30000;     // 30s if no packets for 30s
  }

  if (now - lastPrintMs < interval) return;
  lastPrintMs = now;

  bool locValid  = gps.location.isValid();
  bool altValid  = gps.altitude.isValid();
  bool satsValid = gps.satellites.isValid();
  bool hdopValid = gps.hdop.isValid();
  bool dateValid = gps.date.isValid();
  bool timeValid = gps.time.isValid();

  uint8_t sats = satsValid ? (uint8_t)gps.satellites.value() : 0;
  float hdop   = hdopValid ? (float)gps.hdop.value() * 0.01f : 99.99f;

  uint8_t fixType = 0;
  if (locValid && altValid)      fixType = 3;
  else if (locValid)             fixType = 2;

  double lat = locValid ? gps.location.lat() : 0.0;
  double lon = locValid ? gps.location.lng() : 0.0;
  float gpsAlt = altValid ? gps.altitude.meters() : 0.0f;

  float bmpAlt = 0.0f;
  bool haveBmpAlt = readGroundBaro(bmpAlt);

  char tbuf[32];
  if (dateValid && timeValid) {
    snprintf(tbuf, sizeof(tbuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    snprintf(tbuf, sizeof(tbuf), "INVALID");
  }

  char bmpAltStr[16];
  if (haveBmpAlt) {
    snprintf(bmpAltStr, sizeof(bmpAltStr), "%.2f", bmpAlt);
  } else {
    snprintf(bmpAltStr, sizeof(bmpAltStr), "NaN");
  }

  char line[256];
  snprintf(line, sizeof(line),
           "PAD, gps_fix=%d, gps_fix_type=%u, sats=%u, hdop=%.2f, "
           "lat=%.6f, lon=%.6f, gps_alt=%.2f, bmp_alt=%s, t_utc=%s",
           locValid ? 1 : 0,
           fixType,
           sats,
           hdop,
           lat,
           lon,
           gpsAlt,
           bmpAltStr,
           tbuf);

  Serial.println(line);
  logLineToSd(line);

  dispGpsFixType     = fixType;
  dispGpsSats        = sats;
  dispGpsHdop        = hdop;
  dispGroundGpsAltM  = gpsAlt;
  dispGroundBaroAltM = haveBmpAlt ? bmpAlt : NAN;
}

// ---- distance helper (haversine) ----
float distanceMetersHaversine(double lat1Deg, double lon1Deg,
                              double lat2Deg, double lon2Deg) {
  const double R = 6371000.0; // Earth radius (m)
  double lat1 = lat1Deg * (M_PI / 180.0);
  double lon1 = lon1Deg * (M_PI / 180.0);
  double lat2 = lat2Deg * (M_PI / 180.0);
  double lon2 = lon2Deg * (M_PI / 180.0);

  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;

  double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
             cos(lat1) * cos(lat2) *
             sin(dlon / 2.0) * sin(dlon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return (float)(R * c);
}

// ----------------- LoRa FLIGHT handler ---------
void handleLoRaFlight() {
  if (!loraFlightPending) return;

  noInterrupts();
  FlightPacketV7 pkt = loraFlightPkt;
  int rssi           = loraFlightRssi;
  loraFlightPending  = false;
  interrupts();

  uint32_t now = millis();
  lastRocketRxMs = now;

  // track phase from flags
  if (pkt.flags & FLAG_LAUNCH) {
    rocketLaunched = true;
  }
  if (pkt.flags & FLAG_LANDED) {
    rocketLanded = true;
  }

  float alt_m   = pkt.alt_cm  / 100.0f;
  float vel_ms  = pkt.vel_cms / 100.0f;
  float ax_ms2  = pkt.ax_cms2 / 100.0f;
  float ay_ms2  = pkt.ay_cms2 / 100.0f;
  float az_ms2  = pkt.az_cms2 / 100.0f;
  float gx_dps  = pkt.gx_cdeg / 100.0f;
  float gy_dps  = pkt.gy_cdeg / 100.0f;
  float gz_dps  = pkt.gz_cdeg / 100.0f;
  float roll_degs  = pkt.roll_cdeg  / 100.0f;
  float pitch_degs = pkt.pitch_cdeg / 100.0f;

  char line[256];
  snprintf(line, sizeof(line),
           "ROCKET_FLIGHT, V=%u, STATE=%s, SEQ=%lu, ms=%lu, "
           "alt=%.2f, vel=%.2f, "
           "ax=%.2f, ay=%.2f, az=%.2f, "
           "gx=%.2f, gy=%.2f, gz=%.2f, "
           "roll=%.2f, pitch=%.2f, FLAGS=0x%04X, RSSI=%d",
           pkt.version,
           stateName(pkt.state),
           (unsigned long)pkt.seq,
           (unsigned long)pkt.ms,
           alt_m, vel_ms,
           ax_ms2, ay_ms2, az_ms2,
           gx_dps, gy_dps, gz_dps,
           roll_degs, pitch_degs,
           pkt.flags,
           rssi);

  Serial.println(line);

  bool inFlightPhase = rocketLaunched && !rocketLanded;
  bool logThis = inFlightPhase;
  if (!inFlightPhase) {
    prePostFlightFlightLogDecim++;
    if (prePostFlightFlightLogDecim >= 10) {
      prePostFlightFlightLogDecim = 0;
      logThis = true;
    }
  }

  if (logThis) {
    logLineToSd(line);
  }

  if (haveLastFlightSeq && pkt.seq > lastFlightSeq + 1) {
    flightMissed += (pkt.seq - lastFlightSeq - 1);
  }
  lastFlightSeq = pkt.seq;
  haveLastFlightSeq = true;
  flightPktCount++;
  lastFlightMsDisp = now;

  dispRocketAltM   = alt_m;
  dispRocketVelMps = vel_ms;
  dispRocketState  = pkt.state;
  dispFlightRssi   = rssi;

  dispAxMs2 = ax_ms2;
  dispAyMs2 = ay_ms2;
  dispAzMs2 = az_ms2;
  dispGxDps = gx_dps;
  dispGyDps = gy_dps;
  dispGzDps = gz_dps;
  dispRollDeg  = roll_degs;
  dispPitchDeg = pitch_degs;
  dispFlightFlags = pkt.flags;

  dispLoRaRssi = rssi;
}

// ----------------- LoRa NAV handler ------------
void handleLoRaNav() {
  if (!loraNavPending) return;

  noInterrupts();
  NavPacketV7 pkt = loraNavPkt;
  int rssi        = loraNavRssi;
  loraNavPending  = false;
  interrupts();

  uint32_t now = millis();
  lastRocketRxMs = now;

  double lat = pkt.gps_lat_e7 / 1e7;
  double lon = pkt.gps_lon_e7 / 1e7;
  float  gps_alt_m  = pkt.gps_alt_cm  / 100.0f;
  float  baro_alt_m = pkt.baro_alt_cm / 100.0f;
  float  hdop = pkt.gps_hdop_x10 / 10.0f;

  char line[256];
  snprintf(line, sizeof(line),
           "ROCKET_NAV, V=%u, SEQ=%lu, ms=%lu, "
           "fix_type=%u, sats=%u, hdop=%.1f, "
           "lat=%.6f, lon=%.6f, gps_alt=%.2f, baro_alt=%.2f, "
           "last_fix_age_ms=%lu, RSSI=%d",
           pkt.version,
           (unsigned long)pkt.seq,
           (unsigned long)pkt.ms,
           pkt.gps_fix_type,
           pkt.gps_sats,
           hdop,
           lat, lon,
           gps_alt_m, baro_alt_m,
           (unsigned long)pkt.last_fix_age_ms,
           rssi);

  Serial.println(line);

  bool inFlightPhase = rocketLaunched && !rocketLanded;
  bool logThis = inFlightPhase;
  if (!inFlightPhase) {
    prePostFlightNavLogDecim++;
    if (prePostFlightNavLogDecim >= 10) {
      prePostFlightNavLogDecim = 0;
      logThis = true;
    }
  }
  if (logThis) {
    logLineToSd(line);
  }

  if (haveLastNavSeq && pkt.seq > lastNavSeq + 1) {
    navMissed += (pkt.seq - lastNavSeq - 1);
  }
  lastNavSeq = pkt.seq;
  haveLastNavSeq = true;
  navPktCount++;
  lastNavMsDisp = now;

  dispNavRssi       = rssi;
  dispGpsFixType    = pkt.gps_fix_type;
  dispGpsSats       = pkt.gps_sats;
  dispGpsHdop       = hdop;
  dispGpsLat        = lat;
  dispGpsLon        = lon;
  dispGpsAltM       = gps_alt_m;
  dispRocketBaroM   = baro_alt_m;
  dispLastFixAgeMs  = pkt.last_fix_age_ms;

  dispLoRaRssi = rssi;

  if (gpsHasFix) {
    dispDistanceM = distanceMetersHaversine(gpsLatDeg, gpsLonDeg, lat, lon);
  } else {
    dispDistanceM = NAN;
  }
}

// ----------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);    // GPS

  Wire.begin();           // I2C for BMP180 + SH1106

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  delay(500);

  setupLoRa();
  setupBaro();
  setupSd();
  setupOled();

  bootTimeMs = millis();

  Serial.println("Ground V7: OLED+LoRa+SD+GPS-aware logging");
}

void loop() {
  updateButton();
  updateGps();

  maybePrintPadLine();

  handleLoRaFlight();
  handleLoRaNav();

  updateOled();
}
