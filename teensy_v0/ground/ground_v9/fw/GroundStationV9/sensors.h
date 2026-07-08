#pragma once
#include <Arduino.h>

extern float gndTempC;
extern float gndPressureHpa;
extern float gndAltBaroM;

void sensors_init();
void sensors_update();
void sensors_read(float &tempC, float &pressureHpa, float &altM);
