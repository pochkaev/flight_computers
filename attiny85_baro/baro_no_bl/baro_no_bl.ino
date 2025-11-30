#include <TinyWireM.h>
#include <avr/eeprom.h>

// ATTinyCore's binary.h defines B1 as 1; we need B1 as a variable
#undef B1

// ----------------------------
//   CONFIG
// ----------------------------
#define BMP_ADDR            0x77
#define SEA_LEVEL_PRESSURE  101325L   // Pa, sea-level. Adjust per launch if you want.

// I/O pins
#define TX_PIN   1   // PB1, physical pin 6: software serial TX
#define BTN_PIN  3   // PB3 reset max altitude button

// ----------------------------
//   MAX ALTITUDE EEPROM
// ----------------------------
#define MAX_ALT_MAGIC 0x5A

int16_t maxAlt = 0;

int16_t EEMEM eeMaxAlt;
uint8_t EEMEM eeMaxMagic;

void loadMaxAltitude() {
  uint8_t m = eeprom_read_byte(&eeMaxMagic);
  if (m == MAX_ALT_MAGIC) {
    maxAlt = (int16_t)eeprom_read_word((const uint16_t*)&eeMaxAlt);
  } else {
    maxAlt = 0;
  }
}

void saveMaxAltitude() {
  eeprom_write_word((uint16_t*)&eeMaxAlt, (uint16_t)maxAlt);
  eeprom_write_byte(&eeMaxMagic, MAX_ALT_MAGIC);
}

// ----------------------------
//   SOFTWARE SERIAL (TX only)
// ----------------------------
void serialBegin() {
  DDRB |= (1 << TX_PIN);
  PORTB |= (1 << TX_PIN);  // idle = HIGH
}

void serialWrite(uint8_t data) {
  PORTB &= ~(1 << TX_PIN);         // start bit
  delayMicroseconds(104);          // ~9600 baud @ ~16MHz

  for (uint8_t i = 0; i < 8; i++) {
    if (data & 1) PORTB |= (1 << TX_PIN);
    else          PORTB &= ~(1 << TX_PIN);
    data >>= 1;
    delayMicroseconds(104);
  }

  PORTB |= (1 << TX_PIN);          // stop bit
  delayMicroseconds(104);
}

void serialPrint(const char* s) {
  while (*s) serialWrite(*s++);
}

void serialPrintInt(int32_t v) {
  char buf[12];
  int i = 0;
  bool neg = false;

  if (v < 0) { neg = true; v = -v; }

  do {
    buf[i++] = (v % 10) + '0';
    v /= 10;
  } while (v > 0);

  if (neg) buf[i++] = '-';

  while (--i >= 0) serialWrite(buf[i]);
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
//   (mirroring Adafruit's read8/read16, but simpler)
// ----------------------------
uint8_t bmpRead8(uint8_t reg) {
  TinyWireM.beginTransmission(BMP_ADDR);
  TinyWireM.write(reg);
  TinyWireM.endTransmission();           // STOP

  TinyWireM.requestFrom(BMP_ADDR, (uint8_t)1);
  while (TinyWireM.available() < 1) { }
  return TinyWireM.read();
}

uint16_t bmpRead16(uint8_t reg) {
  TinyWireM.beginTransmission(BMP_ADDR);
  TinyWireM.write(reg);
  TinyWireM.endTransmission();           // STOP

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
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
uint8_t oversampling = 0;  // we use ultra-low-power (OSS=0) for simplicity

bool bmpBegin() {
  // Check chip ID (0x55 per BMP085/BMP180 datasheet)
  uint8_t id = bmpRead8(0xD0);
  if (id != 0x55) {
    return false;
  }

  // Read calibration data exactly like Adafruit does
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
//   (ported from Adafruit_BMP085)
// ----------------------------
int32_t bmpComputeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
}

uint16_t bmpReadRawTemperature() {
  bmpWrite8(0xF4, 0x2E);   // control: temperature
  delay(5);
  return bmpRead16(0xF6);
}

uint32_t bmpReadRawPressure() {
  uint32_t raw;

  bmpWrite8(0xF4, 0x34 + (oversampling << 6));   // control: pressure + OSS
  // conversion time: 5ms at OSS=0
  delay(5);

  raw  = bmpRead16(0xF6);
  raw <<= 8;
  raw |= bmpRead8(0xF8);
  raw >>= (8 - oversampling);

  return raw;
}

// Returns temperature in 0.1°C (same as your original bmpReadTemp10)
int32_t bmpReadTemp10() {
  int32_t UT = (int32_t)bmpReadRawTemperature();
  int32_t B5 = bmpComputeB5(UT);
  return (B5 + 8) >> 4;   // 0.1°C
}

// Returns pressure in Pa (same as Adafruit_BMP085::readPressure, but OSS=0)
int32_t bmpReadPressurePa() {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = (int32_t)bmpReadRawTemperature();
  UP = (int32_t)bmpReadRawPressure();

  B5 = bmpComputeB5(UT);
  B6 = B5 - 4000;

  X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
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

  return p;  // Pa
}

// ----------------------------
//   ALTITUDE (approx, meters)
// ----------------------------
// Simple integer approximation around sea level:
// altitude ≈ (P0 - P) * 0.0833 m/Pa = (P0 - P) * 833 / 10000
int32_t calcAltitude(int32_t pressure) {
  if (pressure <= 0) return 0;
  int32_t dp = SEA_LEVEL_PRESSURE - pressure;
  // If local pressure is higher than sea-level pressure => below reference -> 0
  if (dp <= 0) return 0;
  return (dp * 833L) / 10000L;
}

// ----------------------------
//   SETUP / LOOP
// ----------------------------
void setup() {
  TinyWireM.begin();
  serialBegin();

  pinMode(BTN_PIN, INPUT_PULLUP);

  serialPrint("Init BMP...");
  if (!bmpBegin()) {
    serialPrint("FAIL\r\n");
    while (1) {
      // hang if sensor not found
      delay(1000);
    }
  }
  serialPrint("OK\r\n");

  // Reset max altitude if button held at power-up
  if (digitalRead(BTN_PIN) == LOW) {
    maxAlt = 0;
    saveMaxAltitude();
  }

  loadMaxAltitude();
}

void loop() {
  int32_t t10  = bmpReadTemp10();        // 0.1 °C
  int32_t pres = bmpReadPressurePa();    // Pa
  int32_t alt  = calcAltitude(pres);     // m (approx)

  // Update max altitude (with 1m hysteresis)
  if (alt > maxAlt + 1) {
    maxAlt = (int16_t)alt;
    saveMaxAltitude();
  }

  // Print in roughly the same style you're using
  serialPrint("T=");
  // t10 is 0.1°C → multiply by 10 to show XX.XX (last digit often 0)
  serialPrintFloat100(t10 * 10);
  serialPrint("C P=");
  serialPrintInt(pres);
  serialPrint("Pa A=");
  serialPrintInt(alt);
  serialPrint("m MAX=");
  serialPrintInt(maxAlt);
  serialPrint("m\r\n");

  delay(500);
}
