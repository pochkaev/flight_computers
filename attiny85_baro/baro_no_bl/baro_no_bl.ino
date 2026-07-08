#include <TinyWireM.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <stdbool.h>

// ----------------------------
//   CONFIG
// ----------------------------
#define SEA_LEVEL_PRESSURE  101325L    // Pa - can tweak before flight

// I/O pins
#define TX_PIN        1   // PB1 - software serial TX
#define I2C_SDA_PIN   0   // PB0 - TinyWireM uses this
#define I2C_SCL_PIN   2   // PB2 - TinyWireM uses this
#define BTN_RESET_PIN 3   // PB3 - clear EEPROM if held low at power-on / long press in runtime
#define LONG_PRESS_TICKS  8   // ~2 seconds at the current loop rate

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
static uint8_t sensorAddr = 0x76;   // MS5607 supports 0x76 or 0x77

uint8_t sensorRead8(uint8_t reg) {
  TinyWireM.beginTransmission(sensorAddr);
  TinyWireM.write(reg);
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(sensorAddr, (uint8_t)1);
  while (TinyWireM.available() < 1) { }
  return TinyWireM.read();
}

uint16_t sensorRead16(uint8_t reg) {
  TinyWireM.beginTransmission(sensorAddr);
  TinyWireM.write(reg);
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(sensorAddr, (uint8_t)2);
  while (TinyWireM.available() < 2) { }
  uint8_t msb = TinyWireM.read();
  uint8_t lsb = TinyWireM.read();
  return (uint16_t)msb << 8 | lsb;
}

uint32_t sensorRead24(uint8_t reg) {
  TinyWireM.beginTransmission(sensorAddr);
  TinyWireM.write(reg);
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(sensorAddr, (uint8_t)3);
  while (TinyWireM.available() < 3) { }

  uint32_t value = (uint32_t)TinyWireM.read() << 16;
  value |= (uint32_t)TinyWireM.read() << 8;
  value |= TinyWireM.read();
  return value;
}

void sensorWriteCommand(uint8_t cmd) {
  TinyWireM.beginTransmission(sensorAddr);
  TinyWireM.write(cmd);
  TinyWireM.endTransmission();
}

bool sensorPresentAt(uint8_t addr) {
  TinyWireM.beginTransmission(addr);
  return TinyWireM.endTransmission() == 0;
}

// ----------------------------
//   MS5607 CALIBRATION + READS
// ----------------------------
uint16_t ms5607Prom[8];
const uint8_t ms5607Osr = 0x08;  // OSR=4096, highest resolution

void ms5607Reset() {
  sensorWriteCommand(0x1E);
  delay(3);  // datasheet reset time is 2.8 ms
}

uint16_t ms5607ReadPromWord(uint8_t index) {
  return sensorRead16(0xA0 + (index << 1));
}

uint32_t ms5607ReadAdc() {
  return sensorRead24(0x00);
}

uint32_t ms5607StartConversion(uint8_t baseCmd) {
  sensorWriteCommand(baseCmd | ms5607Osr);
  delay(10);  // OSR=4096 conversion time
  return ms5607ReadAdc();
}

bool ms5607Begin() {
  if (sensorPresentAt(0x76)) {
    sensorAddr = 0x76;
  } else if (sensorPresentAt(0x77)) {
    sensorAddr = 0x77;
  } else {
    return false;
  }

  ms5607Reset();

  for (uint8_t i = 0; i < 8; i++) {
    ms5607Prom[i] = ms5607ReadPromWord(i);
  }

  // Require actual calibration words, not all-zero/all-ones PROM.
  if (ms5607Prom[1] == 0 || ms5607Prom[1] == 0xFFFF) return false;
  if (ms5607Prom[2] == 0 || ms5607Prom[2] == 0xFFFF) return false;
  if (ms5607Prom[3] == 0 || ms5607Prom[3] == 0xFFFF) return false;
  if (ms5607Prom[4] == 0 || ms5607Prom[4] == 0xFFFF) return false;
  if (ms5607Prom[5] == 0 || ms5607Prom[5] == 0xFFFF) return false;
  if (ms5607Prom[6] == 0 || ms5607Prom[6] == 0xFFFF) return false;

  return true;
}

// Reads temperature in 0.1C and pressure in Pa.
bool ms5607Read(int16_t* t10Out, int32_t* pressurePaOut) {
  uint32_t D1 = ms5607StartConversion(0x40);  // pressure
  uint32_t D2 = ms5607StartConversion(0x50);  // temperature

  if (D1 == 0 || D2 == 0) {
    return false;
  }

  int64_t dT   = (int64_t)D2 - ((int64_t)ms5607Prom[5] << 8);
  int64_t TEMP = 2000LL + ((dT * (int64_t)ms5607Prom[6]) >> 23);                 // 0.01C
  int64_t OFF  = ((int64_t)ms5607Prom[2] << 17) + ((dT * (int64_t)ms5607Prom[4]) >> 6);
  int64_t SENS = ((int64_t)ms5607Prom[1] << 16) + ((dT * (int64_t)ms5607Prom[3]) >> 7);

  // Second-order compensation from the MS5607 datasheet.
  if (TEMP < 2000) {
    int64_t tLow = TEMP - 2000;
    int64_t tLow2 = tLow * tLow;
    int64_t T2 = (dT * dT) >> 31;
    int64_t OFF2 = (61LL * tLow2) >> 4;
    int64_t SENS2 = 2LL * tLow2;

    if (TEMP < -1500) {
      int64_t tVeryLow = TEMP + 1500;
      int64_t tVeryLow2 = tVeryLow * tVeryLow;
      OFF2 += 15LL * tVeryLow2;
      SENS2 += 8LL * tVeryLow2;
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;
  }

  int64_t pressure100 = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;              // 0.01 mbar
  int32_t pressurePa = (int32_t)pressure100;                                      // 0.01 mbar == 1 Pa

  *t10Out = (int16_t)(TEMP / 10);  // 0.01C -> 0.1C
  *pressurePaOut = pressurePa;
  return true;
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
static bool resetHandled  = false;
static uint8_t btnPressTicks = 0;

void resetFlightTracking() {
  maxAlt = padAlt;
  altFiltered32 = padAlt;
  firstSample = true;
  launched = false;
  apogeeSaved = false;
  lastAlt = padAlt;
  fallingCnt = 0;
}

void clearFlightDataRuntime() {
  clearFlightData();
  loadFlightData();
  calibratePadAltitude();
  resetFlightTracking();
  serialPrint("RESET ALTITUDE\r\n");
}

void handleResetButton() {
  bool pressed = (digitalRead(BTN_RESET_PIN) == LOW);

  if (!pressed) {
    btnPressTicks = 0;
    resetHandled = false;
    return;
  }

  if (btnPressTicks < 255) {
    btnPressTicks++;
  }

  if (!resetHandled && btnPressTicks >= LONG_PRESS_TICKS) {
    resetHandled = true;
    clearFlightDataRuntime();
  }
}

// ----------------------------
//   PAD ALTITUDE CALIBRATION
// ----------------------------
// Take several samples at startup to estimate pad altitude
void calibratePadAltitude() {
  const uint8_t N = 16;
  int32_t sumAlt = 0;
  uint8_t validSamples = 0;

  for (uint8_t i = 0; i < N; i++) {
    int16_t t10;
    int32_t pres;
    if (!ms5607Read(&t10, &pres)) {
      continue;
    }
    int16_t alt = calcAltitude(pres);

    sumAlt += alt;
    validSamples++;
    delay(50);
  }

  if (validSamples == 0) {
    padAlt = 0;
  } else {
    padAlt = (int16_t)(sumAlt / validSamples);
  }
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

  serialPrint("Init MS5607...");
  if (!ms5607Begin()) {
    serialPrint("FAIL\r\n");
    while (1) {
      delay(1000);
    }
  }
  serialPrint("OK\r\n");

  calibratePadAltitude();
}

void loop() {
  handleResetButton();

  // 1) Read temperature and pressure from MS5607
  int16_t t10;
  int32_t pres;
  if (!ms5607Read(&t10, &pres)) {
    serialPrint("Sensor read fail\r\n");
    delay(500);
    return;
  }

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
