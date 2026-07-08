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
void sampleImuTask(float dtImu);
void sampleBaroTask(float dtBaro);
