#include "rocket_console.h"

RocketConsole rocketConsole;

void RocketConsole::begin(unsigned long baud) {
  ::Serial.begin(baud);
}

void RocketConsole::useMaintenancePort(Stream &port) {
  maintenancePort_ = &port;
}

bool RocketConsole::maintenanceActive() const {
  return maintenancePort_ != nullptr;
}

int RocketConsole::available() {
  return maintenancePort_ ? maintenancePort_->available()
                          : ::Serial.available();
}

int RocketConsole::read() {
  return maintenancePort_ ? maintenancePort_->read() : ::Serial.read();
}

int RocketConsole::peek() {
  return maintenancePort_ ? maintenancePort_->peek() : ::Serial.peek();
}

void RocketConsole::flush() {
  if (maintenancePort_) maintenancePort_->flush();
  else ::Serial.flush();
}

size_t RocketConsole::write(uint8_t value) {
  return maintenancePort_ ? maintenancePort_->write(value)
                          : ::Serial.write(value);
}

