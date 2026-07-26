#pragma once

#include <Arduino.h>

void resetRelAltHistory(uint32_t nowMs, float relAlt);
void clearLaunchArmGate();
void updateArmSwitchTask(uint32_t nowMs);
uint8_t currentLaunchStatus(uint16_t &waitSecondsOut);
float currentBaroRelAltM();
uint16_t buildHealthFlags();
bool taskDue(uint32_t nowMs, uint32_t &lastRunMs, uint32_t periodMs);
void updateFlightStateTask(uint32_t nowMs);
