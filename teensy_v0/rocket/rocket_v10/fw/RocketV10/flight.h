#pragma once

#include <Arduino.h>

void resetRelAltHistory(uint32_t nowMs, float relAlt);
void clearLaunchArmGate();
uint8_t currentLaunchStatus(uint16_t &waitSecondsOut);
float currentBaroRelAltM();
uint16_t buildHealthFlags();
bool taskDue(uint32_t nowMs, uint32_t &lastRunMs, uint32_t periodMs);
void updateFlightStateFromBaroSample(uint32_t nowMs, float currentAltM);
