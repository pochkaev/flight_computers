#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "radio.h"
#include "sensors.h"
#include "sdlog.h"
#include "ui.h"

static uint32_t lastPadLogMs = 0;
static uint32_t lastLostLogMs = 0;

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
    ui_init();

    DBG1("GroundStation READY");
}

void loop() {
    sensors_update();
    radio_update();

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

    ui_update();
}
