#pragma once

#include <Arduino.h>
#include "state.h"

void writeStatusLed(bool on);
void setLedMode(LedMode mode);
void updateLedModeFromHealth();
const char *ledModeName();
bool startupHardwareOk();
void updateStatusLed();

void buzzerWrite(bool on, uint16_t freqHz = 2400);
void startBeepPattern(uint8_t pattern);
void playStartupSound(bool ok);
void setFinderBeeper(bool active);
void updateBuzzer();
