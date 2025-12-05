#include "ui.h"
#include "config.h"
#include "radio.h"
#include "sensors.h"
#include "sdlog.h"
#include "power.h"

#if OLED_IS_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

static uint32_t lastUiMs = 0;
static uint8_t currentPage = PAGE_PREFLIGHT;
static bool buttonPrev = HIGH;
static uint32_t buttonDownMs = 0;
static uint32_t manualOverrideMs = 0;
static bool buttonResetFired = false;

static void drawPreflight();
static void drawRocketStatus();
static void drawFlight();
static void drawRecovery();
static void drawLost();
static void drawLaunch();
static void drawRocketDiag();
static void drawDiag();
static void drawSignal();
static void drawPeaks();

void ui_init() {
    u8g2.begin();
    u8g2.setFont(SMALL_FONT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    currentPage = PAGE_PREFLIGHT;
}

static void updateButton() {
    bool b = digitalRead(BUTTON_PIN);
    if (!b && buttonPrev) {
        buttonDownMs = millis();
        buttonResetFired = false;
    }
    if (!b) {
        uint32_t held = millis() - buttonDownMs;
        if (!buttonResetFired && held >= UI_RESET_HOLD_MS) {
            radio_resetState();
            currentPage = PAGE_PREFLIGHT;
            manualOverrideMs = 0;
            buttonResetFired = true;
        }
    }
    if (b && !buttonPrev) {
        uint32_t held = millis() - buttonDownMs;
        if (!buttonResetFired && held < UI_RESET_HOLD_MS) {
            ui_nextPage();
            manualOverrideMs = millis();
        }
        buttonResetFired = false;
    }
    buttonPrev = b;
}

void ui_nextPage() {
    currentPage = (currentPage + 1) % PAGE_COUNT;
}

static const char* rocketOverallStatus() {
    uint32_t age = millis() - rocketLastPacketMs;
    if (age > LINK_LOST_MS) return "FAULT";
    if (rktFixType < 2)     return "WARN";
    if (!rocketImuOk || !rocketBaroOk) return "WARN";
    return "OK";
}

static const char* rocketStateName() {
    switch (rocketFlightState) {
        case FS_IDLE:    return "IDLE";
        case FS_PAD:     return "PAD";
        case FS_ASCENT:  return "ASCENT";
        case FS_COAST:   return "COAST";
        case FS_DESCENT: return "DESCENT";
        case FS_LANDED:  return "LANDED";
        case FS_ABORT:   return "ABORT";
        default:         return "UNK";
    }
}

void ui_update() {
    uint32_t now = millis();
    if (now - lastUiMs < UI_UPDATE_MS) return;
    lastUiMs = now;

    updateButton();

    bool manualActive = false;
    if (manualOverrideMs) {
        uint32_t dt = now - manualOverrideMs;
        if (dt < UI_MANUAL_TIMEOUT_MS) {
            manualActive = true;
        } else {
            manualOverrideMs = 0;
            // When manual expires, return to phase-driven page if we were on pages 2..last
            if (currentPage >= PAGE_FLIGHT) {
                FlightPhase ph = radio_getPhase();
                switch (ph) {
                    case PHASE_PREFLIGHT: currentPage = PAGE_PREFLIGHT; break;
                    case PHASE_FLIGHT:    currentPage = PAGE_FLIGHT;    break;
                    case PHASE_RECOVERY:  currentPage = PAGE_RECOVERY;  break;
                    case PHASE_LOST:      currentPage = PAGE_LOST;      break;
                }
            }
        }
    }

    if (!manualActive) {
        FlightPhase ph = radio_getPhase();
        switch (ph) {
            case PHASE_PREFLIGHT: currentPage = PAGE_PREFLIGHT; break;
            case PHASE_FLIGHT:    currentPage = PAGE_FLIGHT;    break;
            case PHASE_RECOVERY:  currentPage = PAGE_RECOVERY;  break;
            case PHASE_LOST:      currentPage = PAGE_LOST;      break;
        }
    }

    u8g2.clearBuffer();
    switch (currentPage) {
        case PAGE_PREFLIGHT:      drawPreflight();      break;
        case PAGE_ROCKET_STATUS:  drawRocketStatus();   break;
        case PAGE_FLIGHT:         drawFlight();         break;
        case PAGE_RECOVERY:       drawRecovery();       break;
        case PAGE_LOST:           drawLost();           break;
        case PAGE_LAUNCH:         drawLaunch();         break;
        case PAGE_ROCKET_DIAG:    drawRocketDiag();     break;
        case PAGE_DIAG:           drawDiag();           break;
        case PAGE_SIGNAL:         drawSignal();         break;
        case PAGE_PEAKS:          drawPeaks();          break;
    }
    u8g2.sendBuffer();
}

static void drawPreflight() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[Gnd Module]");
    u8g2.drawStr(90, 10, rocketName);

    char b[64];
    snprintf(b, sizeof(b), "BATT: %.1fV GPS:%dSV",
             GROUND_BATT_VOLTAGE, gndSats);
    u8g2.drawStr(0, 22, b);

    snprintf(b, sizeof(b), "ALT G:%.1fm B:%.1fm", gndAltGpsM, gndAltBaroM);
    u8g2.drawStr(0, 34, b);

    snprintf(b, sizeof(b), "T:%.1fC P:%.1fhPa", gndTempC, gndPressureHpa);
    u8g2.drawStr(0, 46, b);

    snprintf(b, sizeof(b), "ROCKET STATUS: %s", rocketOverallStatus());
    u8g2.drawStr(0, 58, b);
}

static void drawRocketStatus() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[ROCKET]");
    u8g2.drawStr(90, 10, rocketName);

    char b[64];
    snprintf(b, sizeof(b), "BATT: %.1fV GPS:%dSV", rocketBattV, rktSats);
    u8g2.drawStr(0, 22, b);

    snprintf(b, sizeof(b), "ALT G:%.1fm B:%.1fm", rktAltGpsM, rktAltBaroM);
    u8g2.drawStr(0, 34, b);

    snprintf(b, sizeof(b), "STATE: %s", rocketStateName());
    u8g2.drawStr(0, 46, b);

    snprintf(b, sizeof(b), "LORA:%ddBm Age:%.1fs",
             lastCombinedRssi, (millis()-rocketLastPacketMs)/1000.0f);
    u8g2.drawStr(0, 58, b);

    char flags[16] = "";
    if (rktFlags & FLAG_LAUNCH) strcat(flags, "LCH ");
    if (rktFlags & FLAG_APOGEE) strcat(flags, "APO ");
    if (rktFlags & FLAG_LANDED) strcat(flags, "LND ");
    u8g2.drawStr(70, 58, flags);
}

static void drawFlight() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[FLIGHT]");

    float relAlt = rktAltBaroM;
    if (!isnan(rktBaseAltM)) relAlt = rktAltBaroM - rktBaseAltM;
    if (relAlt < 0) relAlt = 0; // clamp if baro drifted

    u8g2.setFont(BIG_FONT);
    char b[32];
    snprintf(b, sizeof(b), "%.1f m AGL", relAlt);
    u8g2.drawStr(0, 40, b);

    u8g2.setFont(SMALL_FONT);
    snprintf(b, sizeof(b), "V: %.1f m/s RSSI:%d", rktVelMs, lastCombinedRssi);
    u8g2.drawStr(0, 60, b);
}

static void drawRecovery() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[LANDED]");
    u8g2.drawStr(90, 10, rocketName);

    char b[32];
    snprintf(b, sizeof(b), "Lat: %.6f", rktLat);
    u8g2.drawStr(0, 22, b);

    snprintf(b, sizeof(b), "Lon: %.6f", rktLon);
    u8g2.drawStr(0, 34, b);

    snprintf(b, sizeof(b), "GPS ALT: %.1fm", rktAltGpsM);
    u8g2.drawStr(0, 46, b);

    snprintf(b, sizeof(b), "BARO ALT: %.1fm", rktAltBaroM);
    u8g2.drawStr(0, 58, b);
}

static void drawLost() {
    u8g2.setFont(BIG_FONT);
    u8g2.drawStr(0, 40, "SIGNAL LOST");

    u8g2.setFont(SMALL_FONT);
    char b[32];
    snprintf(b, sizeof(b), "Age: %.1fs",
             (millis()-rocketLastPacketMs)/1000.0f);
    u8g2.drawStr(0, 60, b);
}

static void drawLaunch() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[LAUNCH]");

    char b[40];

    // Top: ignition battery from power module + local ground batt
    snprintf(b, sizeof(b), "Ign:%.1fV Bat:%.1fV",
             pwr_vbat_x10 / 10.0f,
             pwr_localVbat);
    u8g2.drawStr(0, 22, b);

    // Link / fault marker on the right
    char marker = '!';
    if (power_link_fresh()) {
        marker = pwr_faultAny ? 'F' : '*';
    }
    char m[2] = {marker, '\0'};
    u8g2.drawStr(114, 22, m);

    // Lane A
    snprintf(b, sizeof(b), "A:%s %s I:%.1f(%s)",
             pwr_key_ok ? (pwr_armA_seen ? "ARM" : "SAFE") : "LOCK",
             pwr_onA ? "[ON]" : "[  ]",
             pwr_ia_x10 / 10.0f,
             pwr_presA ? "OK" : "--");
    u8g2.drawStr(0, 36, b);

    // Lane B
    snprintf(b, sizeof(b), "B:%s %s I:%.1f(%s)",
             pwr_key_ok ? (pwr_armB_seen ? "ARM" : "SAFE") : "LOCK",
             pwr_onB ? "[ON]" : "[  ]",
             pwr_ib_x10 / 10.0f,
             pwr_presB ? "OK" : "--");
    u8g2.drawStr(0, 48, b);

}

static void drawDiag() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[DIAG]");

    char b[32];
    snprintf(b, sizeof(b), "GND TEMP: %.1fC", gndTempC);
    u8g2.drawStr(0, 22, b);

    snprintf(b, sizeof(b), "GND PRES: %.1fhPa", gndPressureHpa);
    u8g2.drawStr(0, 34, b);

    snprintf(b, sizeof(b), "RSSI FL:%d NV:%d", lastFlightRssi, lastNavRssi);
    u8g2.drawStr(0, 46, b);

    snprintf(b, sizeof(b), "Log:#%lu Lines:%lu",
             (unsigned long)currentLogIndex,
             (unsigned long)logLineCount);
    u8g2.drawStr(0, 58, b);
}

static void drawRocketDiag() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[ROCKET DIAG]");

    char b[32];
    snprintf(b, sizeof(b), "State: %s", rocketStateName());
    u8g2.drawStr(0, 22, b);

    snprintf(b, sizeof(b), "Baro: %.1fm V:%.1fm/s", rktAltBaroM, rktVelMs);
    u8g2.drawStr(0, 34, b);

    snprintf(b, sizeof(b), "GPS: %dSV HDOP:%.1f", rktSats, rktHdop);
    u8g2.drawStr(0, 46, b);

    snprintf(b, sizeof(b), "GPS Alt: %.1fm", rktAltGpsM);
    u8g2.drawStr(0, 58, b);
}

static void drawSignal() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[SIGNAL]");

    char b[32];
    snprintf(b, sizeof(b), "RSSI: %d", lastCombinedRssi);
    u8g2.drawStr(0, 22, b);

    int rssi = lastCombinedRssi;
    if (rssi < -120) rssi = -120;
    if (rssi > -40)  rssi = -40;
    int barWidth = map(rssi, -120, -40, 0, 100);
    u8g2.drawFrame(0, 26, 104, 8);
    u8g2.drawBox(2, 28, barWidth, 4);

    snprintf(b, sizeof(b), "Dist: %.1fm", distanceToRocketM);
    u8g2.drawStr(0, 42, b);

    snprintf(b, sizeof(b), "Bear: %.1f deg", bearingToRocketDeg);
    u8g2.drawStr(0, 54, b);

    snprintf(b, sizeof(b), "Fix: %dSV(%.1f)", rktSats, rktHdop);
    u8g2.drawStr(0, 64, b);
}

static void drawPeaks() {
    u8g2.setFont(SMALL_FONT);
    u8g2.drawStr(0, 10, "[PEAK]");

    char b[32];
    snprintf(b, sizeof(b), "Max Alt: %.1fm", rktMaxAltM);
    u8g2.drawStr(0, 22, b);

    snprintf(b, sizeof(b), "Max Vel: %.1fm/s", rktMaxVelMs);
    u8g2.drawStr(0, 34, b);

    const char *state = rocketLanded ? "LANDED" : (rocketLaunched ? "FLIGHT" : "PREP");
    snprintf(b, sizeof(b), "State: %s", state);
    u8g2.drawStr(0, 46, b);
}
