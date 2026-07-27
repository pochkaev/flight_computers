#pragma once

#include <Arduino.h>

// Native USB command console with an optional SAFE-only maintenance UART.
// Exactly one input/output transport is active, so GPS bytes can never be
// interpreted as service commands.
class RocketConsole : public Stream {
 public:
  using Print::write;

  void begin(unsigned long baud);
  void useMaintenancePort(Stream &port);
  bool maintenanceActive() const;

  int available() override;
  int read() override;
  int peek() override;
  void flush() override;
  size_t write(uint8_t value) override;

 private:
  Stream *maintenancePort_ = nullptr;
};

extern RocketConsole rocketConsole;

