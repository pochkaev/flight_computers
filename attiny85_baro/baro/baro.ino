#include <DigiUSB.h>
#include <TinyWireM.h>

#define BMP150_ADDRESS 0x77

void serviceUSB(int ms) {
  for(int i=0; i<ms; i++) {
    DigiUSB.refresh();
    delay(1);
  }
}

uint16_t read16(uint8_t reg) {
  TinyWireM.beginTransmission(BMP150_ADDRESS);
  TinyWireM.send(reg);
  TinyWireM.endTransmission();
  serviceUSB(10);
  
  TinyWireM.requestFrom(BMP150_ADDRESS, (uint8_t)2);
  return (TinyWireM.receive() << 8) | TinyWireM.receive();
}

void write8(uint8_t reg, uint8_t value) {
  TinyWireM.beginTransmission(BMP150_ADDRESS);
  TinyWireM.send(reg);
  TinyWireM.send(value);
  TinyWireM.endTransmission();
  serviceUSB(5);
}

// Simple integer-based altitude calculation (no floating point)
int32_t calculateAltitude(int32_t pressure) {
  // Simplified altitude formula using integer math:
  // altitude ≈ 44330 * (101325 - pressure) / 101325
  // This gives approximate altitude in meters
  int32_t altitude = (44330L * (101325L - pressure)) / 101325L;
  return altitude;
}

void setup() {
  DigiUSB.begin();
  serviceUSB(3000);
  DigiUSB.println("Altitude Monitor");
  DigiUSB.println("================");
  
  TinyWireM.begin();
  serviceUSB(100);
}

void loop() {
  DigiUSB.refresh();
  
  // Read pressure (simplified)
  write8(0xF4, 0x34); // Standard pressure measurement
  serviceUSB(20);
  
  // Read raw pressure
  uint32_t rawPressure = read16(0xF6);
  
  // Convert to approximate Pa (simplified)
  int32_t pressure = rawPressure * 100; // Rough conversion
  
  // Calculate altitude
  int32_t altitude = calculateAltitude(pressure);
  
  // Display altitude
  DigiUSB.print("Altitude: ");
  DigiUSB.print(altitude);
  DigiUSB.println(" m");
  
  serviceUSB(2000);
}