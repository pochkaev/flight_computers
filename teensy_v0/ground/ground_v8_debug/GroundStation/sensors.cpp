#include "sensors.h"
#include "config.h"
#include "ui.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>

static Adafruit_BMP085 bmp180;
static bool bmpOk = false;

float gndTempC = NAN;
float gndPressureHpa = NAN;
float gndAltBaroM = NAN;
static uint32_t lastSensorsMs = 0;

void sensors_init() {
    Wire.begin();
    bmpOk = bmp180.begin();
    if (bmpOk) DBG1("BMP180 detected");
    else       DBG1("BMP180 NOT found");
}

void sensors_update() {
    if (!bmpOk) return;
    uint32_t now = millis();
    if (now - lastSensorsMs < GND_SENSORS_UPDATE_MS) return;
    lastSensorsMs = now;

    gndTempC = bmp180.readTemperature();
    float pressurePa = bmp180.readPressure();
    gndPressureHpa = pressurePa / 100.0f;
    gndAltBaroM = bmp180.readAltitude(SEA_LEVEL_PRESSURE_HPA * 100.0f);
    ui_markDirty();
}

void sensors_read(float &tempC, float &pressureHpa, float &altM) {
    sensors_update();
    tempC = gndTempC;
    pressureHpa = gndPressureHpa;
    altM = gndAltBaroM;
}
