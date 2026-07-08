#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "radio.h"
#include "sensors.h"
#include "sdlog.h"
#include "ui.h"
#include "power.h"

static uint32_t lastPadLogMs = 0;
static uint32_t lastLostLogMs = 0;

#if DEBUG_PROFILE
// Simple per-loop timing buckets (microseconds)
static uint32_t profLoopStartUs   = 0;
static uint32_t profSensorsUs     = 0;
static uint32_t profRadioUs       = 0;
static uint32_t profPowerUs       = 0;
static uint32_t profLoggingUs     = 0;
static uint32_t profUiUs          = 0;
static uint32_t profLoopTotalUs   = 0;
#endif

void log_pad() {
    char line[256];
    snprintf(line, sizeof(line),
        "PAD,lat=%.6f,lon=%.6f,gnd_alt_gps=%.1f,gnd_alt_baro=%.1f,"
        "temp=%.1f,pres=%.1f,sats=%d,hdop=%.2f",
        gndLat, gndLon, gndAltGpsM, gndAltBaroM,
        gndTempC, gndPressureHpa, gndSats, gndHdop
    );
    sdlog_write(line);
}

void log_flight() {
    char line[256];
    snprintf(line, sizeof(line),
        "FLG,alt_baro=%.1f,vel=%.1f,fix=%d,sats=%d,hdop=%.1f,"
        "gps_alt=%.1f,baro_alt=%.1f,lat=%.6f,lon=%.6f,"
        "flags=%u,rssi=%d",
        rktAltBaroM, rktVelMs, rktFixType, rktSats, rktHdop,
        rktAltGpsM, rktAltBaroM, rktLat, rktLon,
        rktFlags, lastCombinedRssi
    );
    sdlog_write(line);
}

void log_nav() {
    char line[256];
    snprintf(line, sizeof(line),
        "NAV,lat=%.6f,lon=%.6f,gps_alt=%.1f,baro_alt=%.1f,"
        "fix=%d,sats=%d,hdop=%.1f,rssi=%d",
        rktLat, rktLon, rktAltGpsM, rktAltBaroM,
        rktFixType, rktSats, rktHdop, lastCombinedRssi
    );
    sdlog_write(line);
}

void log_lost() {
    char line[256];
    snprintf(line, sizeof(line),
        "LOST,age=%.1f,last_lat=%.6f,last_lon=%.6f",
        (millis()-rocketLastPacketMs)/1000.0f, rktLat, rktLon
    );
    sdlog_write(line);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    DBG1("Booting GroundStation...");

    sensors_init();
    radio_init();
    sdlog_init();
#if ENABLE_POWER_MODULE
    power_init();
#endif
    ui_init();

    DBG1("GroundStation READY");
}

void loop() {
    uint32_t t0 = micros();
    sensors_update();
    uint32_t t1 = micros();
    radio_update();
    uint32_t t2 = micros();
#if ENABLE_POWER_MODULE
    power_update();
#endif
    uint32_t tAfterPower = micros();

    if (!sdlog_hasGpsTime && gps.date.isValid() && gps.time.isValid()) {
        DBG1("GPS TIME ACQUIRED → timestamp logs");
        sdlog_onGpsTimeAvailable();
    }

    uint32_t now = millis();
    static FlightPhase lastPhase = PHASE_PREFLIGHT;
    FlightPhase ph = radio_getPhase();

    if (ph != lastPhase) {
        DBG1(String("PHASE → ") + String(ph));
        lastPhase = ph;
    }

    switch (ph) {
        case PHASE_PREFLIGHT:
            if (now - lastPadLogMs > PAD_PRELOG_MS) {
                lastPadLogMs = now;
                log_pad();
            }
            break;
        case PHASE_FLIGHT:
            log_flight();
            break;
        case PHASE_RECOVERY:
            log_nav();
            break;
        case PHASE_LOST:
            if (now - lastLostLogMs > PAD_LOSTLOG_MS) {
                lastLostLogMs = now;
                log_lost();
            }
            break;
    }

#if DEBUG_GND_STATUS
    static uint32_t lastDebugMs = 0;
    if (now - lastDebugMs > 1000) {
        lastDebugMs = now;
        Serial.print("GND: GPSfix=");
        Serial.print((int)gndFixType);
        Serial.print(" sats=");
        Serial.print((int)gndSats);
        Serial.print(" hdop=");
        Serial.print(gndHdop, 2);
        Serial.print(" alt_gps=");
        Serial.print(gndAltGpsM, 1);
        Serial.print(" alt_baro=");
        Serial.print(gndAltBaroM, 1);
        Serial.println("m");
    }
#endif

    uint32_t tBeforeUi = micros();
    ui_update();

#if DEBUG_PROFILE
    uint32_t tEnd = micros();

    profSensorsUs   = t1 - t0;
    profRadioUs     = t2 - t1;
#if ENABLE_POWER_MODULE
    profPowerUs     = tAfterPower - t2;
#else
    profPowerUs     = 0;
#endif
    // Approximate logging cost as everything between end of power and start of UI
    profLoggingUs   = tBeforeUi - tAfterPower;
    profUiUs        = tEnd - tBeforeUi;
    profLoopTotalUs = tEnd - t0;

    static uint32_t lastProfMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastProfMs > 200) { // log profiling every 200 ms
        lastProfMs = nowMs;
        char line[128];
        snprintf(line, sizeof(line),
                 "PROF,loop_us=%lu,sens=%lu,radio=%lu,pwr=%lu,log=%lu,ui=%lu",
                 (unsigned long)profLoopTotalUs,
                 (unsigned long)profSensorsUs,
                 (unsigned long)profRadioUs,
                 (unsigned long)profPowerUs,
                 (unsigned long)profLoggingUs,
                 (unsigned long)profUiUs);
        sdlog_write(line);
    }
#endif
}
