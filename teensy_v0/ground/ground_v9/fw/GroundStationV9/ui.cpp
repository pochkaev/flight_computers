#include "ui.h"
#include "config.h"
#include "radio.h"
#include "sensors.h"
#include "sdlog.h"
#include "power.h"
#include <ILI9341_t3.h>
#include <SPI.h>

ILI9341_t3 tft(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

static uint32_t lastUiMs = 0;
static uint32_t lastUiAutoDirtyMs = 0;
static uint8_t currentPage = PAGE_PREFLIGHT;
static uint8_t defaultPage = PAGE_PREFLIGHT;
static uint8_t lastRenderedPage = 0xFF;
static bool pageChangedThisFrame = false;
static bool buttonPrev = HIGH;
static uint32_t buttonDownMs = 0;
static uint32_t manualOverrideMs = 0;
static bool buttonResetFired = false;
static bool uiDirty = true;

// Landed page hold timing
static bool landedSeen = false;
static uint32_t landedStartMs = 0;

static const uint16_t COLOR_BG = ILI9341_BLACK;
static const uint16_t COLOR_HDR = ILI9341_BLUE;
static const uint16_t COLOR_TEXT = ILI9341_WHITE;
static const uint16_t COLOR_ACCENT = ILI9341_YELLOW;
static const uint16_t COLOR_OK = ILI9341_GREEN;
static const uint16_t COLOR_WARN = ILI9341_YELLOW;
static const uint16_t COLOR_BAD = ILI9341_RED;

static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
static const int HDR_H = 26;

static void drawPreflight();
static void drawRocketStatus();
static void drawFlight();
static void drawRecovery();
static void drawLost();
static void drawLaunch();
static void drawSignal();

void ui_init() {
    tft.begin();
    tft.setRotation(1); // landscape: 320x240
    tft.fillScreen(COLOR_BG);
    tft.setTextWrap(false);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    currentPage = PAGE_PREFLIGHT;
    lastUiMs = millis();
    lastUiAutoDirtyMs = lastUiMs;
    uiDirty = true;
}

void ui_markDirty() {
    uiDirty = true;
}

static void drawHeader(const char *title, const char *rightText, bool force) {
    if (!force) {
        tft.setTextColor(COLOR_TEXT, COLOR_BG);
        return;
    }
    tft.fillRect(0, 0, SCREEN_W, HDR_H, COLOR_HDR);
    tft.setTextColor(COLOR_TEXT, COLOR_HDR);
    tft.setTextSize(2);
    tft.setCursor(6, 6);
    tft.print(title);
    if (rightText && rightText[0]) {
        int16_t x = SCREEN_W - (strlen(rightText) * 6 * 2) - 6;
        if (x < 140) x = 140;
        tft.setCursor(x, 6);
        tft.print(rightText);
    }
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
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
    if (!rocketImuOk || !rocketBaroOk || !rocketGpsOk) return "WARN";
    if (!rocketSdOk || !rocketNandOk || !rocketLogOk) return "WARN";
    return "OK";
}

static uint16_t rocketOverallStatusColor() {
    const char *status = rocketOverallStatus();
    if (strcmp(status, "OK") == 0) return COLOR_OK;
    if (strcmp(status, "WARN") == 0) return COLOR_WARN;
    return COLOR_BAD;
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
    if (now - lastUiMs < UI_UPDATE_MS) {
        return;
    }
    lastUiMs = now;

    uint8_t prevPage = currentPage;

    updateButton();

    // If ARM just turned on, immediately jump to Launch
    // and cancel any manual override timer.
    static bool prevAnyArmed = false;
    bool anyArmed = pwr_anyArmed;
    if (anyArmed && !prevAnyArmed) {
        currentPage = PAGE_LAUNCH;
        manualOverrideMs = 0;
        uiDirty = true;
    }
    prevAnyArmed = anyArmed;

    // Manual override window
    bool manualActive = false;
    if (manualOverrideMs) {
        uint32_t dt = now - manualOverrideMs;
        if (dt < UI_MANUAL_TIMEOUT_MS) {
            manualActive = true;
        } else {
            manualOverrideMs = 0;
        }
    }

    // --- Compute default page based on system state ---
    FlightPhase ph = radio_getPhase();

    if (pwr_anyArmed) {
        // While armed, Launch is the default page.
        defaultPage = PAGE_LAUNCH;
    } else {
        switch (ph) {
            case PHASE_FLIGHT:
                landedSeen = false;
                defaultPage = PAGE_FLIGHT;
                break;
            case PHASE_RECOVERY:
                if (!landedSeen) {
                    landedSeen = true;
                    landedStartMs = now;
                }
                if (now - landedStartMs < UI_LANDED_VIEW_MS) {
                    defaultPage = PAGE_RECOVERY;
                } else {
                    defaultPage = PAGE_PREFLIGHT;
                }
                break;
            case PHASE_LOST:
                landedSeen = false;
                defaultPage = PAGE_LOST;
                break;
            case PHASE_PREFLIGHT:
            default:
                landedSeen = false;
                // Idle: always fall back to first page.
                defaultPage = PAGE_PREFLIGHT;
                break;
        }
    }

    // If not in manual mode, follow default page.
    if (!manualActive) {
        currentPage = defaultPage;
    }

    // Force an occasional refresh so time-based
    // fields (ages, timers) don't freeze.
    if (now - lastUiAutoDirtyMs >= UI_AUTO_REFRESH_MS) {
        uiDirty = true;
        lastUiAutoDirtyMs = now;
    }

    if (currentPage != prevPage) {
        uiDirty = true;
    }

    if (!uiDirty) {
        return;
    }
    uiDirty = false;

    pageChangedThisFrame = (currentPage != lastRenderedPage);
    if (pageChangedThisFrame) {
        tft.fillScreen(COLOR_BG);
        lastRenderedPage = currentPage;
    }
    switch (currentPage) {
        case PAGE_PREFLIGHT:      drawPreflight();      break;
        case PAGE_ROCKET_STATUS:  drawRocketStatus();   break;
        case PAGE_FLIGHT:         drawFlight();         break;
        case PAGE_RECOVERY:       drawRecovery();       break;
        case PAGE_LOST:           drawLost();           break;
        case PAGE_LAUNCH:         drawLaunch();         break;
        case PAGE_SIGNAL:         drawSignal();         break;
    }
}

static void drawPreflight() {
    drawHeader("[GND MODULE]", rocketName, pageChangedThisFrame);

    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 8);
    tft.print("BATT:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(pwr_localVbat, 2);
    tft.print("V");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("  GPS:");
    tft.print((int)gndSats);
    tft.print("SV");

    tft.setCursor(10, HDR_H + 36);
    tft.print("ALT G:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(gndAltGpsM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("  B:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(gndAltBaroM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m");

    tft.setCursor(10, HDR_H + 64);
    tft.print("TEMP:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(gndTempC, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("C  PRES:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(gndPressureHpa, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("hPa");

    tft.setCursor(10, HDR_H + 92);
    if (sdOK && currentLogIndex > 0) {
        tft.print("SD: #");
        tft.print((unsigned long)currentLogIndex);
        tft.print("  Lines:");
        tft.print((unsigned long)logLineCount);
    } else if (sdOK) {
        tft.print("SD: OK (no file)");
    } else {
        tft.print("SD: ---");
    }
}

static void drawRocketStatus() {
    drawHeader("[ROCKET]", rocketName, pageChangedThisFrame);

    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 8);
    tft.print("BATT:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rocketBattV, 2);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("V  GPS:");
    tft.print((int)rktSats);
    tft.print("SV");

    tft.setCursor(10, HDR_H + 36);
    tft.print("ALT G:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltGpsM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("  B:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltBaroM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m");

    tft.setCursor(10, HDR_H + 64);
    tft.print("STATE: ");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rocketStateName());
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    tft.setCursor(10, HDR_H + 92);
    tft.print("LORA: ");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(lastCombinedRssi);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" dBm  AGE:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((millis()-rocketLastPacketMs)/1000.0f, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("s");

    tft.setCursor(10, HDR_H + 120);
    if (rktFlags & FLAG_LAUNCH) tft.print("LCH ");
    if (rktFlags & FLAG_APOGEE) tft.print("APO ");
    if (rktFlags & FLAG_LANDED) tft.print("LND ");

    tft.setCursor(10, HDR_H + 148);
    tft.setTextColor(rocketOverallStatusColor(), COLOR_BG);
    tft.print("SYS ");
    tft.setTextColor(rocketGpsOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("GPS ");
    tft.setTextColor(rocketImuOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("IMU ");
    tft.setTextColor(rocketBaroOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("BARO");

    tft.setCursor(10, HDR_H + 176);
    tft.setTextColor(rocketLogOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("LOG ");
    tft.setTextColor(rocketSdOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("SD ");
    tft.setTextColor(rocketNandOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("NAND");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawFlight() {
    drawHeader("[FLIGHT]", nullptr, pageChangedThisFrame);

    float relAlt = rktAltBaroM;
    if (!isnan(rktBaseAltM)) relAlt = rktAltBaroM - rktBaseAltM;
    if (relAlt < 0) relAlt = 0; // clamp if baro drifted

    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.setTextSize(4);
    tft.setCursor(10, HDR_H + 20);
    tft.print(relAlt, 1);
    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 80);
    tft.print("m AGL");

    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setCursor(10, HDR_H + 120);
    tft.print("V:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktVelMs, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m/s  RSSI:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(lastCombinedRssi);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawRecovery() {
    drawHeader("[LANDED]", rocketName, pageChangedThisFrame);

    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 8);
    tft.print("MAX ALT:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktMaxAltM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m");

    tft.setCursor(10, HDR_H + 36);
    tft.print("MAX V:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktMaxVelMs, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m/s");

    tft.setCursor(10, HDR_H + 64);
    tft.print("ALT G:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltGpsM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("  B:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltBaroM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m");

    tft.setCursor(10, HDR_H + 92);
    tft.print("LAT:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktLat, 6);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    tft.setCursor(10, HDR_H + 120);
    tft.print("LON:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktLon, 6);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawLost() {
    drawHeader("[LOST]", rocketName, pageChangedThisFrame);

    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 8);
    tft.print("AGE:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((millis()-rocketLastPacketMs)/1000.0f, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" s");

    tft.setCursor(10, HDR_H + 36);
    tft.print("LAT:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktLat, 6);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    tft.setCursor(10, HDR_H + 64);
    tft.print("LON:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktLon, 6);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    tft.setCursor(10, HDR_H + 92);
    tft.print("ALT G:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltGpsM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("  B:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltBaroM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m");
}

static void drawLaunch() {
    drawHeader("[LAUNCH]", rocketName, pageChangedThisFrame);

    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 8);
    tft.print("IGN:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(pwr_vbat_x10 / 10.0f, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("V  LNK:");
    if (power_link_fresh()) {
        tft.setTextColor(COLOR_OK, COLOR_BG);
        tft.print("OK");
    } else {
        tft.setTextColor(COLOR_BAD, COLOR_BG);
        tft.print("---");
    }
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    tft.setCursor(10, HDR_H + 40);
    if (pwr_faultAny) {
        tft.setTextColor(COLOR_BAD, COLOR_BG);
        tft.print("A: FAULT  ");
        tft.print(pwr_ia_x10 / 10.0f, 1);
        tft.print("A");
        tft.setCursor(10, HDR_H + 68);
        tft.print("B: FAULT  ");
        tft.print(pwr_ib_x10 / 10.0f, 1);
        tft.print("A");
        tft.setTextColor(COLOR_TEXT, COLOR_BG);
    } else {
        tft.print("A:");
        tft.setTextColor(pwr_key_ok ? COLOR_OK : COLOR_WARN, COLOR_BG);
        tft.print(pwr_key_ok ? (pwr_armA_seen ? "ARM " : "SAFE") : "LOCK");
        tft.setTextColor(COLOR_TEXT, COLOR_BG);
        tft.print(" ");
        tft.print(pwr_onA ? "[ON]" : "[  ]");
        tft.print(" I:");
        tft.setTextColor(COLOR_ACCENT, COLOR_BG);
        tft.print(pwr_ia_x10 / 10.0f, 1);
        tft.setTextColor(COLOR_TEXT, COLOR_BG);
        tft.print(" ");
        tft.print(pwr_presA ? "(OK)" : "(--)");

        tft.setCursor(10, HDR_H + 68);
        tft.print("B:");
        tft.setTextColor(pwr_key_ok ? COLOR_OK : COLOR_WARN, COLOR_BG);
        tft.print(pwr_key_ok ? (pwr_armB_seen ? "ARM " : "SAFE") : "LOCK");
        tft.setTextColor(COLOR_TEXT, COLOR_BG);
        tft.print(" ");
        tft.print(pwr_onB ? "[ON]" : "[  ]");
        tft.print(" I:");
        tft.setTextColor(COLOR_ACCENT, COLOR_BG);
        tft.print(pwr_ib_x10 / 10.0f, 1);
        tft.setTextColor(COLOR_TEXT, COLOR_BG);
        tft.print(" ");
        tft.print(pwr_presB ? "(OK)" : "(--)");
    }

    tft.setCursor(10, HDR_H + 104);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("RS485 RX: ");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((unsigned int)pwr_rxRate);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("/s");
}

static void drawSignal() {
    drawHeader("[SIGNAL]", nullptr, pageChangedThisFrame);

    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 8);
    tft.print("RSSI:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(lastCombinedRssi);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" dBm");

    int rssi = lastCombinedRssi;
    if (rssi < -120) rssi = -120;
    if (rssi > -40)  rssi = -40;
    int barWidth = map(rssi, -120, -40, 0, 260);
    tft.drawRect(10, HDR_H + 36, 280, 12, COLOR_TEXT);
    tft.fillRect(12, HDR_H + 38, barWidth, 8, COLOR_OK);

    tft.setCursor(10, HDR_H + 64);
    tft.print("DIST:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(distanceToRocketM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m");

    tft.setCursor(10, HDR_H + 92);
    tft.print("BEAR:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(bearingToRocketDeg, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" deg");

    tft.setCursor(10, HDR_H + 120);
    tft.print("GPS: ");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((int)rktSats);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("SV  HDOP:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktHdop, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

// No separate PEAK page; max altitude / velocity
// are shown on the [LANDED] page.
