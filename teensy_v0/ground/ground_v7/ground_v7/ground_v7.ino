#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>
#include <math.h>

// ----------------- LoRa config -----------------
const long LORA_FREQUENCY = 915E6;
const int  LORA_CS_PIN    = 10;
const int  LORA_RST_PIN   = 9;
const int  LORA_DIO0_PIN  = 2;

const uint8_t PKT_TYPE_FLIGHT_V7 = 0x01;
const uint8_t PKT_TYPE_NAV_V7    = 0x02;

// ----------------- GPS (ground) ----------------
TinyGPSPlus gps;   // ground GPS on Serial1

// snapshot (for PAD line)
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

// ----------------- LoRa RX callback -----------------
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
    // unknown type
    while (LoRa.available()) LoRa.read();
  }
}

// ----------------- LoRa setup -----------------
void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    while (true) {}  // hard fail if LoRa not found
  }

  LoRa.setSPIFrequency(8E6);

  LoRa.onReceive(onLoraReceive);
  LoRa.receive();

  Serial.println("LoRa RX initialized");
}

// -----------------  Baro helpers ---------
void setupBaro() {
  Wire.begin();
  if (bmp180.begin()) {
    hasBaro = true;
    Serial.println("Ground: BMP180 detected");
  } else {
    hasBaro = false;
    Serial.println("Ground: BMP180 not found");
  }
}

// Read current ground barometric altitude (m); returns true if valid
bool readGroundBaro(float &alt_m) {
  if (!hasBaro) return false;
  alt_m = bmp180.readAltitude(101325.0);
  return true;
}

// ----------------- Ground GPS update -------------
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

// ----------------- PAD line (ground status) ----
void maybePrintPadLine() {
  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();
  if (now - lastPrintMs < 1000) return;
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

  Serial.print("PAD, gps_fix=");
  Serial.print(locValid ? 1 : 0);
  Serial.print(", gps_fix_type=");
  Serial.print(fixType);
  Serial.print(", sats=");
  Serial.print(sats);
  Serial.print(", hdop=");
  Serial.print(hdop, 2);

  Serial.print(", lat=");
  Serial.print(lat, 6);
  Serial.print(", lon=");
  Serial.print(lon, 6);
  Serial.print(", gps_alt=");
  Serial.print(gpsAlt, 2);

  Serial.print(", bmp_alt=");
  if (haveBmpAlt) {
    Serial.print(bmpAlt, 2);
  } else {
    Serial.print("NaN");
  }

  Serial.print(", t_utc=");
  if (dateValid && timeValid) {
    int year  = gps.date.year();
    int month = gps.date.month();
    int day   = gps.date.day();
    int hour  = gps.time.hour();
    int minute= gps.time.minute();
    int second= gps.time.second();

    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             year, month, day, hour, minute, second);
    Serial.print(buf);
  } else {
    Serial.print("INVALID");
  }

  Serial.println();
}

// ----------------- LoRa FLIGHT handler  --------
void handleLoRaFlight() {
  if (!loraFlightPending) return;

  noInterrupts();
  FlightPacketV7 pkt = loraFlightPkt;
  int rssi           = loraFlightRssi;
  loraFlightPending  = false;
  interrupts();

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

  Serial.print("ROCKET_FLIGHT, V=");
  Serial.print(pkt.version);
  Serial.print(", STATE=");
  Serial.print(stateName(pkt.state));
  Serial.print(", SEQ=");
  Serial.print(pkt.seq);
  Serial.print(", ms=");
  Serial.print(pkt.ms);

  Serial.print(", alt=");
  Serial.print(alt_m, 2);
  Serial.print(", vel=");
  Serial.print(vel_ms, 2);

  Serial.print(", ax=");
  Serial.print(ax_ms2, 2);
  Serial.print(", ay=");
  Serial.print(ay_ms2, 2);
  Serial.print(", az=");
  Serial.print(az_ms2, 2);

  Serial.print(", gx=");
  Serial.print(gx_dps, 2);
  Serial.print(", gy=");
  Serial.print(gy_dps, 2);
  Serial.print(", gz=");
  Serial.print(gz_dps, 2);

  Serial.print(", roll=");
  Serial.print(roll_degs, 2);
  Serial.print(", pitch=");
  Serial.print(pitch_degs, 2);

  Serial.print(", FLAGS=0x");
  Serial.print(pkt.flags, HEX);
  Serial.print(", RSSI=");
  Serial.println(rssi);
}

// ----------------- LoRa NAV handler ------------
void handleLoRaNav() {
  if (!loraNavPending) return;

  noInterrupts();
  NavPacketV7 pkt = loraNavPkt;
  int rssi        = loraNavRssi;
  loraNavPending  = false;
  interrupts();

  double lat = pkt.gps_lat_e7 / 1e7;
  double lon = pkt.gps_lon_e7 / 1e7;
  float  gps_alt_m  = pkt.gps_alt_cm  / 100.0f;
  float  baro_alt_m = pkt.baro_alt_cm / 100.0f;
  float  hdop = pkt.gps_hdop_x10 / 10.0f;

  Serial.print("ROCKET_NAV, V=");
  Serial.print(pkt.version);
  Serial.print(", SEQ=");
  Serial.print(pkt.seq);
  Serial.print(", ms=");
  Serial.print(pkt.ms);

  Serial.print(", fix_type=");
  Serial.print(pkt.gps_fix_type);
  Serial.print(", sats=");
  Serial.print(pkt.gps_sats);
  Serial.print(", hdop=");
  Serial.print(hdop, 1);

  Serial.print(", lat=");
  Serial.print(lat, 6);
  Serial.print(", lon=");
  Serial.print(lon, 6);
  Serial.print(", gps_alt=");
  Serial.print(gps_alt_m, 2);
  Serial.print(", baro_alt=");
  Serial.print(baro_alt_m, 2);

  Serial.print(", last_fix_age_ms=");
  Serial.print(pkt.last_fix_age_ms);

  Serial.print(", RSSI=");
  Serial.println(rssi);
}

// ----------------- setup / loop -----------------
void setup() {
  Serial.begin(115200);

  // Ground GPS on Serial1
  Serial1.begin(9600);
  delay(1000);

  setupLoRa();
  setupBaro();

  Serial.println("Ground V7 initialized");
}

void loop() {
  // Keep feeding GPS parser
  updateGps();

  // Periodically print pad info
  maybePrintPadLine();

  // Handle any LoRa packets from rocket
  handleLoRaFlight();
  handleLoRaNav();
}
