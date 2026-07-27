#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

extern TinyGPSPlus gps;

void setupBaro();
void setupImu();
bool isBaroFresh();
bool isImuFresh();
bool isGpsFresh();
void sampleGpsTask();
bool sampleImuTask();
void sampleBaroTask(float dtBaro);
