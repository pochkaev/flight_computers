#pragma once

#include <Arduino.h>

void setupPyroOutputs();
void stopPyroOutputs();
void updatePyroOutputs(uint32_t nowMs);
void logApogeeChargeEvent(uint32_t nowMs);
void logMainChargeEvent(uint32_t nowMs);
void markBoosterBurnoutEvent(uint32_t nowMs);
void updateHprStylePyroEvents(uint32_t nowMs);
