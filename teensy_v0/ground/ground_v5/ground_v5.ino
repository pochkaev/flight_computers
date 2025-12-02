#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>

// ----------------- LoRa config -----------------
const long LORA_FREQUENCY = 915E6;
const int LORA_CS_PIN    = 10;
const int LORA_RST_PIN   = 9;
const int LORA_DIO0_PIN  = 2;

// ----------------- GPS (GT-U7) -----------------
TinyGPSPlus gps;   // ground GPS on Serial1

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

// ----------------- Telemetry V6 packet --------
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
volatile bool loraPacketPending = false;
TelemetryPacketV6 loraPkt;
int loraPktRssi = 0;


// ----------------- LoRa setup -----------------

void onLoraReceive(int packetSize) {
  // Only handle exact-size packets
  if (packetSize != sizeof(TelemetryPacketV6)) {
    // Drain garbage
    while (LoRa.available()) LoRa.read();
    return;
  }

  // Read raw bytes into our global struct
  uint8_t *p = (uint8_t*)&loraPkt;
  int i = 0;
  while (LoRa.available() && i < packetSize) {
    p[i++] = LoRa.read();
  }

  // If for some reason we didn't get all bytes, drop it
  if (i != packetSize) {
    // drain any remainder
    while (LoRa.available()) LoRa.read();
    return;
  }

  loraPktRssi = LoRa.packetRssi();
  loraPacketPending = true;
}


void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    while (true) {}  // hard fail if LoRa not found
  }

  // Optional but fine for Teensy 4.0
  LoRa.setSPIFrequency(8E6);

  // Register RX callback and go into continuous RX mode
  LoRa.onReceive(onLoraReceive);
  LoRa.receive();

  Serial.println("LoRa RX initialized");
}


// -----------------  Baro helpers ---------

void setupBaro() {

  // BMP180 / BMP085 baro
  Wire.begin();
  if (bmp180.begin()) {
    hasBaro = true;
    Serial.println("Ground: BMP180 detected");
  } else {
    hasBaro = false;
    Serial.println("Ground: BMP180 not found");
  }
}

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


// Read current ground barometric altitude (m); returns true if valid
bool readGroundBaro(float &alt_m) {
  if (!hasBaro) return false;
  // Adafruit_BMP085 reports altitude relative to given sea level pressure
  // Use 101325 Pa as default; you can tweak this if you know local pressure.
  alt_m = bmp180.readAltitude(101325.0);
  return true;
}

// Print one PAD line once per second with:
//  - ground GPS (lat/lon/gps_alt) if valid
//  - ground baro altitude if available
//  - GPS time (UTC) if valid
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
  if (locValid && altValid)      fixType = 3;  // 3D
  else if (locValid)             fixType = 2;  // 2D

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

  // GPS UTC time
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

// ----------------- LoRa RX -> Print rocket telemetry -----
void handleLoRa() {
  if (!loraPacketPending) return;

  // Copy under interrupt lock so ISR doesn't modify mid-print
  noInterrupts();
  TelemetryPacketV6 pkt = loraPkt;
  int rssi = loraPktRssi;
  loraPacketPending = false;
  interrupts();

  // Now print using the local copy
  Serial.print("ROCKET, ");
  Serial.print("V=");      Serial.print(pkt.version);
  Serial.print(", STATE=");Serial.print(stateName(pkt.state));
  Serial.print(", SEQ=");  Serial.print(pkt.seq);
  Serial.print(", ms=");   Serial.print(pkt.ms);
  Serial.print(", alt=");  Serial.print(pkt.alt);
  Serial.print(", temp="); Serial.print(pkt.temp);
  Serial.print(", vel=");  Serial.print(pkt.vel_z);

  Serial.print(", ax=");   Serial.print(pkt.ax);
  Serial.print(", ay=");   Serial.print(pkt.ay);
  Serial.print(", az=");   Serial.print(pkt.az);
  Serial.print(", gx=");   Serial.print(pkt.gx);
  Serial.print(", gy=");   Serial.print(pkt.gy);
  Serial.print(", gz=");   Serial.print(pkt.gz);
  Serial.print(", roll="); Serial.print(pkt.roll_deg);
  Serial.print(", pitch=");Serial.print(pkt.pitch_deg);

  Serial.print(", gps_fix=");       Serial.print(pkt.gps_has_fix);
  Serial.print(", gps_fix_type=");  Serial.print(pkt.gps_fix_type);
  Serial.print(", gps_sats=");      Serial.print(pkt.gps_sats);
  Serial.print(", gps_hdop=");      Serial.print(pkt.gps_hdop);

  Serial.print(", gps_lat=");       Serial.print(pkt.gps_lat_deg, 6);
  Serial.print(", gps_lon=");       Serial.print(pkt.gps_lon_deg, 6);
  Serial.print(", gps_alt=");       Serial.print(pkt.gps_alt_m);
  Serial.print(", gps_spd=");       Serial.print(pkt.gps_speed_mps);

  Serial.print(", gps_last_lat=");  Serial.print(pkt.gps_last_lat_deg, 6);
  Serial.print(", gps_last_lon=");  Serial.print(pkt.gps_last_lon_deg, 6);
  Serial.print(", gps_last_alt=");  Serial.print(pkt.gps_last_alt_m);
  Serial.print(", gps_last_age_ms="); Serial.print(pkt.gps_last_fix_age_ms);

  Serial.print(", FLAGS=0x");       Serial.print(pkt.flags, HEX);
  Serial.print(", RSSI=");          Serial.println(rssi);
}


// ----------------- setup / loop -----------------
void setup() {
  Serial.begin(115200);
  
  // Ground GPS on Serial1
  Serial1.begin(9600);
  delay(1000); 

  setupLoRa();
  setupBaro();

  Serial.println("Ground V6 initialized");

}

void loop() {
  // Keep feeding GPS parser
  updateGps();

  // Periodically print pad info
  maybePrintPadLine();

  // Handle any LoRa packets from rocket
  handleLoRa();
}
