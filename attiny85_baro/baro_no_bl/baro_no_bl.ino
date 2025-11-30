#include <TinyWireM.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <stdbool.h>

// ----------------------------
//   CONFIG
// ----------------------------
#define BMP_ADDR            0x77
#define SEA_LEVEL_PRESSURE  101325L    // Pa - can tweak before flight

// I/O pins
#define TX_PIN        1   // PB1 - software serial TX
#define I2C_SDA_PIN   0   // PB0 - TinyWireM uses this
#define I2C_SCL_PIN   2   // PB2 - TinyWireM uses this
#define BTN_RESET_PIN 3   // PB3 - clear EEPROM if held low at power-on

// Altitude filter
#define ALT_FILTER_SHIFT   0   // alpha = 1 / (2^3) = 1/8 (0-debug, 2-prod)

// Launch / apogee detection
#define LAUNCH_ALT_THRESH  1   // m above pad to consider "launched" (1-debug, 5-prod)
#define APOGEE_FALL_SAMPLES 3 // how many consecutive falling samples to confirm apogee (3-debug, 10-prod)
#define EEPROM_RECORD_DELTA 1  // m improvement required to update recordMaxAlt (1-debug, 5-prod)

// ----------------------------
//   EEPROM FLIGHT DATA
// ----------------------------
typedef struct {
  int16_t padAlt;        // pad altitude (m)
  int16_t apogeeAlt;     // apogee altitude (m)
  int16_t deltaAlt;      // apogee - pad (m)
  int16_t recordMaxAlt;  // best apogee ever (m)
} FlightData;

FlightData flightData;
FlightData EEMEM eeFlightData;
uint8_t   EEMEM eeMagic;
#define FLIGHT_MAGIC 0x42

void clearFlightData() {
  flightData.padAlt       = 0;
  flightData.apogeeAlt    = 0;
  flightData.deltaAlt     = 0;
  flightData.recordMaxAlt = 0;
  eeprom_write_byte(&eeMagic, 0xFF);  // invalidate
}

void loadFlightData() {
  uint8_t m = eeprom_read_byte(&eeMagic);
  if (m == FLIGHT_MAGIC) {
    eeprom_read_block(&flightData, &eeFlightData, sizeof(FlightData));
  } else {
    flightData.padAlt       = 0;
    flightData.apogeeAlt    = 0;
    flightData.deltaAlt     = 0;
    flightData.recordMaxAlt = 0;
  }
}

void saveFlightData(int16_t padAlt, int16_t apogeeAlt) {
  flightData.padAlt    = padAlt;
  flightData.apogeeAlt = apogeeAlt;
  int16_t d = apogeeAlt - padAlt;
  if (d < 0) d = 0;
  flightData.deltaAlt = d;

  // Update lifetime record only if improved by >= EEPROM_RECORD_DELTA meters
  if (apogeeAlt > flightData.recordMaxAlt + EEPROM_RECORD_DELTA) {
    flightData.recordMaxAlt = apogeeAlt;
  }

  eeprom_write_block(&flightData, &eeFlightData, sizeof(FlightData));
  eeprom_write_byte(&eeMagic, FLIGHT_MAGIC);
}

// ----------------------------
//   SOFTWARE SERIAL (TX only)
// ----------------------------
void serialBegin() {
  DDRB |= (1 << TX_PIN);
  PORTB |= (1 << TX_PIN);  // idle = HIGH
}

void serialWrite(uint8_t data) {
  // 9600 baud @ ~16 MHz: ~104 µs per bit
  PORTB &= ~(1 << TX_PIN);       // start bit
  delayMicroseconds(104);

  for (uint8_t i = 0; i < 8; i++) {
    if (data & 1) PORTB |= (1 << TX_PIN);
    else          PORTB &= ~(1 << TX_PIN);
    data >>= 1;
    delayMicroseconds(104);
  }

  PORTB |= (1 << TX_PIN);        // stop bit
  delayMicroseconds(104);
}

void serialPrint(const char* s) {
  while (*s) serialWrite(*s++);
}

void serialPrintInt(int32_t v) {
  char buf[12];
  uint8_t i = 0;
  bool neg = false;

  if (v < 0) { neg = true; v = -v; }

  do {
    buf[i++] = (v % 10) + '0';
    v /= 10;
  } while (v > 0);

  if (neg) buf[i++] = '-';

  while (i--) serialWrite(buf[i]);
}

// value is in 1/100 units, e.g. 2345 => "23.45"
void serialPrintFloat100(int32_t value) {
  int32_t whole = value / 100;
  int32_t frac  = value % 100;
  if (frac < 0) frac = -frac;

  serialPrintInt(whole);
  serialWrite('.');
  if (frac < 10) serialWrite('0');
  serialPrintInt(frac);
}

// ----------------------------
//   I2C HELPERS (TinyWireM)
// ----------------------------
uint8_t bmpRead8(uint8_t reg) {
  TinyWireM.beginTransmission(BMP_ADDR);
  TinyWireM.write(reg);
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(BMP_ADDR, (uint8_t)1);
  while (TinyWireM.available() < 1) { }
  return TinyWireM.read();
}

uint16_t bmpRead16(uint8_t reg) {
  TinyWireM.beginTransmission(BMP_ADDR);
  TinyWireM.write(reg);
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(BMP_ADDR, (uint8_t)2);
  while (TinyWireM.available() < 2) { }
  uint8_t msb = TinyWireM.read();
  uint8_t lsb = TinyWireM.read();
  return (uint16_t)msb << 8 | lsb;
}

void bmpWrite8(uint8_t reg, uint8_t val) {
  TinyWireM.beginTransmission(BMP_ADDR);
  TinyWireM.write(reg);
  TinyWireM.write(val);
  TinyWireM.endTransmission();
}

// ----------------------------
//   BMP085/BMP180 CALIB DATA
//   (same names as Adafruit lib)
// ----------------------------
int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
uint8_t  oversampling = 0;  // ultra low power (OSS=0)

bool bmpBegin() {
  uint8_t id = bmpRead8(0xD0);
  if (id != 0x55) {
    return false;
  }

  ac1 = (int16_t)bmpRead16(0xAA);
  ac2 = (int16_t)bmpRead16(0xAC);
  ac3 = (int16_t)bmpRead16(0xAE);
  ac4 =          bmpRead16(0xB0);
  ac5 =          bmpRead16(0xB2);
  ac6 =          bmpRead16(0xB4);
  b1  = (int16_t)bmpRead16(0xB6);
  b2  = (int16_t)bmpRead16(0xB8);
  mb  = (int16_t)bmpRead16(0xBA);
  mc  = (int16_t)bmpRead16(0xBC);
  md  = (int16_t)bmpRead16(0xBE);

  return true;
}

// ----------------------------
//   RAW READS & COMPENSATION
// ----------------------------
uint16_t bmpReadRawTemperature() {
  bmpWrite8(0xF4, 0x2E);
  delay(5);
  return bmpRead16(0xF6);
}

uint32_t bmpReadRawPressure() {
  uint32_t raw;

  bmpWrite8(0xF4, 0x34 + (oversampling << 6));
  delay(5); // enough for OSS=0

  raw = bmpRead16(0xF6);
  raw <<= 8;
  raw |= bmpRead8(0xF8);
  raw >>= (8 - oversampling);

  return raw;
}

int32_t bmpComputeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
}

// Temperature in 0.1 °C from B5
int16_t bmpTemp10FromB5(int32_t B5) {
  return (int16_t)((B5 + 8) >> 4); // 0.1°C
}

// Pressure in Pa, using existing B5 (no second temp calc)
int32_t bmpReadPressurePa(int32_t B5) {
  int32_t UT, UP, B3, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  // We already calculated B5 from UT outside

  B6 = B5 - 4000;

  X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  UP = (int32_t)bmpReadRawPressure();
  B7 = ((uint32_t)UP - (uint32_t)B3) * (uint32_t)(50000UL >> oversampling);

  if (B7 < 0x80000000UL) {
    p = (int32_t)((B7 * 2UL) / B4);
  } else {
    p = (int32_t)((B7 / B4) * 2UL);
  }

  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038L) >> 16;
  X2 = (-7357L * p) >> 16;

  p = p + ((X1 + X2 + 3791L) >> 4);

  return p;
}

// ----------------------------
//   ALTITUDE (approx, meters)
// ----------------------------
// altitude ≈ (P0 - P) * 0.0833 m/Pa = (P0 - P) * 833 / 10000
int16_t calcAltitude(int32_t pressure) {
  if (pressure <= 0) return 0;
  int32_t dp = SEA_LEVEL_PRESSURE - pressure;
  if (dp <= 0) return 0;
  int32_t alt = (dp * 833L) / 10000L;
  if (alt > 32767) alt = 32767;
  return (int16_t)alt;
}

// ----------------------------
//   GLOBALS FOR FILTER & FLIGHT STATE
// ----------------------------
static int32_t altFiltered32 = 0;
static int16_t padAlt = 0;
static int16_t maxAlt = 0;

static bool firstSample   = true;
static bool launched      = false;
static bool apogeeSaved   = false;
static int16_t lastAlt    = 0;
static uint8_t fallingCnt = 0;

// ----------------------------
//   PAD ALTITUDE CALIBRATION
// ----------------------------
// Take several samples at startup to estimate pad altitude
void calibratePadAltitude() {
  const uint8_t N = 16;
  int32_t sumAlt = 0;

  for (uint8_t i = 0; i < N; i++) {
    uint16_t UT = bmpReadRawTemperature();
    int32_t B5 = bmpComputeB5((int32_t)UT);
    int16_t t10 = bmpTemp10FromB5(B5);  // not used, but keeps flow similar
    (void)t10;
    int32_t pres = bmpReadPressurePa(B5);
    int16_t alt  = calcAltitude(pres);

    sumAlt += alt;
    delay(50);
  }

  padAlt = (int16_t)(sumAlt / N);
  maxAlt = padAlt;
  altFiltered32 = padAlt;
  lastAlt = padAlt;
}

// ----------------------------
//   SETUP / LOOP
// ----------------------------
void setup() {
  TinyWireM.begin();
  serialBegin();

  pinMode(BTN_RESET_PIN, INPUT_PULLUP);

  // If reset button held at power-up, clear EEPROM flight data
  if (digitalRead(BTN_RESET_PIN) == LOW) {
    clearFlightData();
  }

  loadFlightData();

  serialPrint("Init BMP...");
  if (!bmpBegin()) {
    serialPrint("FAIL\r\n");
    while (1) {
      delay(1000);
    }
  }
  serialPrint("OK\r\n");

  calibratePadAltitude();
}

void loop() {
  // 1) Read raw temp and pressure, reuse B5
  uint16_t UT = bmpReadRawTemperature();
  int32_t B5  = bmpComputeB5((int32_t)UT);
  int16_t t10 = bmpTemp10FromB5(B5);      // 0.1 °C
  int32_t pres = bmpReadPressurePa(B5);   // Pa

  // 2) Compute raw altitude (integer meters)
  int16_t alt = calcAltitude(pres);

  // 3) Low-pass filter altitude
  if (firstSample) {
    altFiltered32 = alt;
    firstSample = false;
  } else {
    altFiltered32 += ((int32_t)alt - altFiltered32) >> ALT_FILTER_SHIFT; // alpha=1/8
  }
  int16_t altF = (int16_t)altFiltered32;

  // 4) Launch detection
  if (!launched && (altF > padAlt + LAUNCH_ALT_THRESH)) {
    launched = true;
    apogeeSaved = false;
    maxAlt = altF;
    fallingCnt = 0;
  }

  // 5) Track max altitude (current flight) and detect apogee
  if (launched) {
    if (altF > maxAlt) {
      maxAlt = altF;
    }

    if (altF < lastAlt - 1) {
      if (fallingCnt < 255) fallingCnt++;
    } else if (altF > lastAlt + 1) {
      fallingCnt = 0;
    }

    if (!apogeeSaved && fallingCnt > APOGEE_FALL_SAMPLES) {
      // We are past apogee: save flight data once
      saveFlightData(padAlt, maxAlt);
      apogeeSaved = true;
      launched = false; // or keep true if you want continuous flight state
    }
  }

  lastAlt = altF;

  // 6) Telemetry output
  serialPrint("T=");
  // t10 is 0.1°C; to print as XX.XX use scale x10 => 0.01°C (last digit often 0)
  serialPrintFloat100((int32_t)t10 * 10);
  serialPrint("C P=");
  serialPrintInt(pres);
  serialPrint("Pa A=");
  serialPrintInt(altF);
  serialPrint("m PAD=");
  serialPrintInt(padAlt);
  serialPrint("m MAX=");
  serialPrintInt(maxAlt);
  serialPrint("m dA=");
  serialPrintInt(maxAlt - padAlt);
  serialPrint("m REC=");
  serialPrintInt(flightData.recordMaxAlt);
  serialPrint("m\r\n");

  delay(200);  // ~5 Hz sample rate
}
