#include "ui.h"
#include "config.h"
#include "radio.h"
#include "sensors.h"
#include "sdlog.h"
#include "power.h"
#include "timekeeper.h"
#include "settings.h"
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
static uint32_t pyroFlashStartMs = 0;

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
static void drawPyroConfig();
static void drawFlight();
static void drawRecovery();
static void drawLost();
static void drawLaunch();
static void drawSignal();
static void drawStatusStrip();
static void clearPageBody();

static void clearPageBody() {
    tft.fillRect(0, HDR_H, SCREEN_W, SCREEN_H - HDR_H - 20, COLOR_BG);
}

static void clearTextRow(int y, int h = 24) {
    tft.fillRect(0, y, SCREEN_W, h, COLOR_BG);
}

static float packetAgeS(uint32_t lastMs) {
    if (lastMs == 0) return -1.0f;
    return (millis() - lastMs) / 1000.0f;
}

static bool rocketLinkFresh() {
    return rocketLastPacketMs != 0 && (millis() - rocketLastPacketMs) <= groundSettingsLinkLostMs();
}

static void formatGroundTime(char *out, size_t n) {
    GroundDateTime dt;
    if (timekeeper_getCentralDateTime(dt)) {
        snprintf(out, n, "%02u:%02u", (unsigned int)dt.hour, (unsigned int)dt.minute);
    } else {
        snprintf(out, n, "--:--");
    }
}

static float rocketRelAltM() {
    float relAlt = rktAltBaroM;
    if (!isnan(rktBaseAltM)) relAlt = rktAltBaroM - rktBaseAltM;
    if (relAlt < 0) relAlt = 0.0f;
    return relAlt;
}

void ui_init() {
    tft.begin();
    tft.setRotation(3); // landscape flipped: 320x240
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
    if (age > groundSettingsLinkLostMs()) return "FAULT";
    if (rocketBattCrit) return "CRIT";
    if (rocketBattWarn) return "WARN";
    if (!rocketImuOk || !rocketBaroOk) return "WARN";
    if (!rocketSdOk || !rocketNandOk || !rocketLogOk) return "WARN";
    return "OK";
}

static uint16_t rocketOverallStatusColor() {
    const char *status = rocketOverallStatus();
    if (strcmp(status, "OK") == 0) return COLOR_OK;
    if (strcmp(status, "WARN") == 0) return COLOR_WARN;
    return COLOR_BAD;
}

static const char* rocketLinkStatus() {
    if (rocketLastPacketMs == 0) return "---";
    uint32_t age = millis() - rocketLastPacketMs;
    if (age > groundSettingsLinkLostMs()) return "LOST";
    return "OK";
}

static uint16_t rocketLinkStatusColor() {
    const char *status = rocketLinkStatus();
    if (strcmp(status, "OK") == 0) return COLOR_OK;
    if (strcmp(status, "---") == 0) return COLOR_WARN;
    return COLOR_BAD;
}

static const char* rocketStateName() {
    switch (rocketFlightState) {
        case FS_IDLE:                       return "IDLE";
        case FS_PAD:                        return "PAD";
        case FS_ASCENT:                     return "ASCENT";
        case FS_COAST:                      return "COAST";
        case FS_SUBSONIC_COAST:             return "SUB COAST";
        case FS_NEAR_APOGEE:                return "APOGEE";
        case FS_DESCENT_BALLISTIC:          return "BALLISTIC";
        case FS_UNDER_DROGUE:               return "DROGUE";
        case FS_DUAL_DEPLOY_APOGEE_LOGGED:  return "APOGEE LOG";
        case FS_DUAL_DEPLOY_MAIN_LOGGED:    return "MAIN LOG";
        case FS_POST_FLIGHT_GROUND:         return "POST FLT";
        case FS_LANDED:                     return "LANDED";
        case FS_ABORT:                      return "ABORT";
        default:                            return "UNK";
    }
}

static const char* pyroFunctionName(char func) {
    switch (func) {
        case 'A': return "APOGEE";
        case 'M': return "MAIN";
        case 'B': return "BOOST SEP";
        case 'I': return "SUST IGN";
        case '1': return "AIRSTART1";
        case '2': return "AIRSTART2";
        case 'N': return "DISABLED";
        default: return "UNKNOWN";
    }
}

static const char* pyroEventName(uint8_t eventType) {
    switch (eventType) {
        case 40: return "APOGEE LOG";
        case 41: return "MAIN LOG";
        case 42: return "BOOST SEP";
        case 43: return "SUST IGN";
        case 44: return "AIRSTART1";
        case 45: return "AIRSTART2";
        case 46: return "STAGE INHIBIT";
        case 50: return "CH1 LOG";
        case 51: return "CH2 LOG";
        case 52: return "CH3 LOG";
        case 53: return "CH4 LOG";
        case 60: return "CH1 ON";
        case 61: return "CH2 ON";
        case 62: return "CH3 ON";
        case 63: return "CH4 ON";
        case 64: return "CH1 OFF";
        case 65: return "CH2 OFF";
        case 66: return "CH3 OFF";
        case 67: return "CH4 OFF";
        default: return "PYRO EVENT";
    }
}

static bool drawPyroFlashIfActive(uint32_t now) {
    if (rocketPyroFlashPending) {
        rocketPyroFlashPending = false;
        pyroFlashStartMs = now;
        lastRenderedPage = 0xFF;
    }
    if (pyroFlashStartMs == 0) return false;

    const uint32_t elapsed = now - pyroFlashStartMs;
    const uint32_t redMs = 550;
    const uint32_t darkMs = 250;
    const uint32_t cycleMs = redMs + darkMs;
    const uint32_t totalMs = cycleMs * 3;
    if (elapsed >= totalMs) {
        pyroFlashStartMs = 0;
        uiDirty = true;
        return false;
    }

    bool red = (elapsed % cycleMs) < redMs;
    tft.fillScreen(red ? COLOR_BAD : COLOR_BG);
    if (red) {
        tft.setTextColor(COLOR_TEXT, COLOR_BAD);
        tft.setTextSize(3);
        tft.setCursor(28, 84);
        tft.print("PYRO EVENT");
        tft.setTextSize(2);
        tft.setCursor(28, 128);
        tft.print(pyroEventName(rocketLastPyroEventType));
    }
    return true;
}

static const char* rocketLaunchStatusName() {
    switch (rocketLaunchStatus) {
        case LAUNCH_STATUS_BOOT_WAIT:     return "BOOT";
        case LAUNCH_STATUS_SAFE_REQUIRED: return "INSERT PIN";
        case LAUNCH_STATUS_SAFE:          return "SAFE";
        case LAUNCH_STATUS_PAD_SETTLE:    return "PAD SETTLE";
        case LAUNCH_STATUS_SENSOR_FAULT:  return "SENSOR ERR";
        case LAUNCH_STATUS_LOG_FAULT:     return "LOG ERR";
        case LAUNCH_STATUS_BATT_CRIT:     return "BATT CRIT";
        case LAUNCH_STATUS_HOLD_VERTICAL: return "VERTICAL";
        case LAUNCH_STATUS_HOLD_STILL:    return "HOLD STILL";
        case LAUNCH_STATUS_ARMING:        return "ARMING";
        case LAUNCH_STATUS_READY:         return "READY";
        case LAUNCH_STATUS_LAUNCH_CHECK:  return "LAUNCH CHECK";
        case LAUNCH_STATUS_FLIGHT:        return "FLIGHT";
        default:                          return "UNKNOWN";
    }
}

static uint16_t pyroFunctionColor(char func) {
    switch (func) {
        case 'A':
        case 'M':
        case 'B':
        case 'I':
        case '1':
        case '2':
            return COLOR_OK;
        case 'N':
            return COLOR_WARN;
        default:
            return COLOR_BAD;
    }
}

static void printPyroChannelRow(uint8_t ch, int y) {
    const uint8_t idx = ch - 1;
    const bool valid = idx < rocketPyroChannelCount;
    const char func = valid ? rocketPyroChannelFunc[idx] : 'N';
    const bool logEnabled = (rocketPyroChannelLogMask & (1u << idx)) != 0;
    const bool outputEnabled = (rocketPyroChannelOutputMask & (1u << idx)) != 0;

    clearTextRow(y, 24);
    tft.setTextSize(2);
    tft.setCursor(10, y + 2);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print((unsigned int)ch);
    tft.print(" ");
    tft.setTextColor(pyroFunctionColor(func), COLOR_BG);
    tft.print(pyroFunctionName(func));
    tft.setTextColor(outputEnabled ? COLOR_BAD : (logEnabled ? COLOR_ACCENT : COLOR_WARN), COLOR_BG);
    tft.setCursor(242, y + 2);
    if (outputEnabled) tft.print("FIRE");
    else tft.print(logEnabled ? "LOGS" : "NOLOG");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static uint16_t rocketLaunchStatusColor() {
    switch (rocketLaunchStatus) {
        case LAUNCH_STATUS_READY:
        case LAUNCH_STATUS_FLIGHT:
            return COLOR_OK;
        case LAUNCH_STATUS_SENSOR_FAULT:
        case LAUNCH_STATUS_LOG_FAULT:
        case LAUNCH_STATUS_BATT_CRIT:
            return COLOR_BAD;
        case LAUNCH_STATUS_BOOT_WAIT:
        case LAUNCH_STATUS_SAFE_REQUIRED:
        case LAUNCH_STATUS_SAFE:
        case LAUNCH_STATUS_PAD_SETTLE:
        case LAUNCH_STATUS_HOLD_VERTICAL:
        case LAUNCH_STATUS_HOLD_STILL:
        case LAUNCH_STATUS_ARMING:
        case LAUNCH_STATUS_LAUNCH_CHECK:
            return COLOR_WARN;
        default:
            return COLOR_BAD;
    }
}

static const char* rocketReadyDisplayName() {
    if (rocketFlightState == FS_PAD) return rocketLaunchStatusName();
    return rocketStateName();
}

static uint16_t rocketReadyDisplayColor() {
    if (rocketFlightState == FS_PAD) return rocketLaunchStatusColor();
    return COLOR_ACCENT;
}

void ui_update() {
    uint32_t now = millis();
    if (now - lastUiMs < UI_UPDATE_MS) {
        return;
    }
    lastUiMs = now;

    if (drawPyroFlashIfActive(now)) {
        return;
    }

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
                defaultPage = PAGE_RECOVERY;
                break;
            case PHASE_LOST:
                landedSeen = false;
                defaultPage = PAGE_LOST;
                break;
            case PHASE_PREFLIGHT:
            default:
                landedSeen = false;
                // No rocket packets: show ground module. Rocket present on pad:
                // show the launch-ready rocket dashboard.
                defaultPage = (rocketLastPacketMs != 0) ? PAGE_ROCKET_STATUS : PAGE_PREFLIGHT;
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
        case PAGE_PYRO_CONFIG:    drawPyroConfig();     break;
        case PAGE_FLIGHT:         drawFlight();         break;
        case PAGE_RECOVERY:       drawRecovery();       break;
        case PAGE_LOST:           drawLost();           break;
        case PAGE_LAUNCH:         drawLaunch();         break;
        case PAGE_SIGNAL:         drawSignal();         break;
    }
    drawStatusStrip();
}

static void drawPreflight() {
    drawHeader("[GND MODULE]", rocketName, pageChangedThisFrame);
    if (pageChangedThisFrame) clearPageBody();

    tft.setTextSize(2);
    clearTextRow(HDR_H + 4, 28);
    tft.setCursor(10, HDR_H + 8);
    tft.print("BATT:");
    if (pwr_localBattCrit) tft.setTextColor(COLOR_BAD, COLOR_BG);
    else if (pwr_localBattWarn) tft.setTextColor(COLOR_WARN, COLOR_BG);
    else tft.setTextColor(COLOR_OK, COLOR_BG);
    tft.print(pwr_localVbat, 2);
    tft.print("V");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" ");
    tft.setTextColor(pwr_localBattCrit ? COLOR_BAD : (pwr_localBattWarn ? COLOR_WARN : COLOR_ACCENT), COLOR_BG);
    tft.print(power_local_battery_pack_name());
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" GPS:");
    tft.print((int)gndSats);
    tft.print("SV");

    clearTextRow(HDR_H + 32, 28);
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

    clearTextRow(HDR_H + 60, 28);
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

    clearTextRow(HDR_H + 88, 28);
    tft.setCursor(10, HDR_H + 92);
    tft.print("LINK:");
    tft.setTextColor(rocketLinkStatusColor(), COLOR_BG);
    tft.print(rocketLinkStatus());
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" RSSI:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(lastCombinedRssi);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 116, 28);
    tft.setCursor(10, HDR_H + 120);
    tft.print("RX F/N/S:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((unsigned int)flightRxRate);
    tft.print("/");
    tft.print((unsigned int)navRxRate);
    tft.print("/");
    tft.print((unsigned int)statusRxRate);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 144, 28);
    tft.setCursor(10, HDR_H + 148);
    tft.print("MISS:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((unsigned long)flightMissedCount);
    tft.print("/");
    tft.print((unsigned long)navMissedCount);
    tft.print("/");
    tft.print((unsigned long)statusMissedCount);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 172, 28);
    tft.setCursor(10, HDR_H + 176);
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

    char timeText[8];
    formatGroundTime(timeText, sizeof(timeText));
    tft.setTextColor(timekeeper_hasTime() ? COLOR_ACCENT : COLOR_WARN, COLOR_BG);
    tft.setCursor(SCREEN_W - (5 * 12) - 10, HDR_H + 176);
    tft.print(timeText);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawRocketStatus() {
    drawHeader("[ROCKET]", rocketName, true);
    if (pageChangedThisFrame) clearPageBody();

    float relAlt = rocketRelAltM();

    clearTextRow(HDR_H + 8, 42);
    const char *readyName = rocketReadyDisplayName();
    tft.setTextSize(strlen(readyName) > 10 ? 2 : 3);
    tft.setCursor(10, HDR_H + 14);
    tft.setTextColor(rocketReadyDisplayColor(), COLOR_BG);
    tft.print(readyName);
    if (rocketFlightState == FS_PAD &&
        (rocketLaunchStatus == LAUNCH_STATUS_BOOT_WAIT ||
         rocketLaunchStatus == LAUNCH_STATUS_PAD_SETTLE ||
         rocketLaunchStatus == LAUNCH_STATUS_ARMING)) {
        tft.setTextSize(2);
        tft.print(" ");
        tft.print((unsigned int)rocketLaunchWaitS);
        tft.print("s");
    }

    tft.setTextSize(2);
    tft.setCursor(240, HDR_H + 12);
    tft.print("SYS");
    tft.setCursor(240, HDR_H + 32);
    tft.setTextColor(rocketOverallStatusColor(), COLOR_BG);
    tft.print(rocketOverallStatus());
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 54, 28);
    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 58);
    tft.print("BATT:");
    if (rocketBattCrit) tft.setTextColor(COLOR_BAD, COLOR_BG);
    else if (rocketBattWarn) tft.setTextColor(COLOR_WARN, COLOR_BG);
    else tft.setTextColor(COLOR_OK, COLOR_BG);
    tft.print(rocketBattV, 2);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("V  GPS:");
    tft.setTextColor(rocketGpsOk ? COLOR_OK : COLOR_WARN, COLOR_BG);
    tft.print((int)rktSats);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("SV ");
    tft.setTextColor(rocketGpsOk ? COLOR_OK : COLOR_WARN, COLOR_BG);
    tft.print(rocketGpsOk ? "OK" : "---");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 84, 28);
    tft.setCursor(10, HDR_H + 88);
    tft.print("LINK:");
    tft.setTextColor(rocketLinkStatusColor(), COLOR_BG);
    tft.print(rocketLinkStatus());
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" RSSI:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(lastCombinedRssi);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" AGE:");
    float age = packetAgeS(rocketLastPacketMs);
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    if (age < 0) tft.print("-");
    else tft.print(age, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 114, 28);
    tft.setCursor(10, HDR_H + 118);
    tft.print("AGL:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(relAlt, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("m  VEL:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktVelMs, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("m/s");

    clearTextRow(HDR_H + 144, 24);
    tft.setCursor(10, HDR_H + 148);
    tft.print("BARO:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltBaroM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("m  RX:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((unsigned int)flightRxRate);
    tft.print("/");
    tft.print((unsigned int)navRxRate);
    tft.print("/");
    tft.print((unsigned int)statusRxRate);

    clearTextRow(HDR_H + 170, 28);
    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 174);
    tft.setTextColor(rocketImuOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("IMU ");
    tft.setTextColor(rocketBaroOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("BARO ");
    tft.setTextColor(rocketLogOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("LOG ");
    tft.setTextColor(rocketSdOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("SD ");
    tft.setTextColor(rocketNandOk ? COLOR_OK : COLOR_BAD, COLOR_BG);
    tft.print("NAND");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawPyroConfig() {
    drawHeader("[PYRO CFG]", rocketName, pageChangedThisFrame);
    if (pageChangedThisFrame) clearPageBody();

    if (!rocketPyroConfigValid) {
        clearTextRow(HDR_H + 56, 40);
        tft.setTextSize(3);
        tft.setCursor(16, HDR_H + 62);
        tft.setTextColor(COLOR_WARN, COLOR_BG);
        tft.print("NO PYRO CONFIG");
        tft.setTextColor(COLOR_TEXT, COLOR_BG);
        clearTextRow(HDR_H + 112, 28);
        tft.setTextSize(2);
        tft.setCursor(16, HDR_H + 116);
        tft.print("Wait for rocket LoRa");
        return;
    }

    clearTextRow(HDR_H + 6, 26);
    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 10);
    tft.print("MAIN:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((unsigned int)rocketPyroMainAltM);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("m  GLOBAL:");
    tft.setTextColor(rocketPyroOutputEnabled ? COLOR_BAD : COLOR_WARN, COLOR_BG);
    tft.print(rocketPyroOutputEnabled ? "FIRE" : "LOGS");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 32, 26);
    tft.setCursor(10, HDR_H + 36);
    tft.print("APG DELAY:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rocketPyroApogeeDelayMs / 1000.0f, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("s  WAIT:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rocketPyroMainMinAfterApogeeMs / 1000.0f, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("s");

    clearTextRow(HDR_H + 58, 26);
    tft.setCursor(10, HDR_H + 62);
    tft.print("PULSE DUR:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rocketPyroFireMs / 1000.0f, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("s");

    printPyroChannelRow(1, HDR_H + 86);
    printPyroChannelRow(2, HDR_H + 112);
    printPyroChannelRow(3, HDR_H + 138);
    printPyroChannelRow(4, HDR_H + 164);

    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawFlight() {
    drawHeader("[FLIGHT]", rocketStateName(), pageChangedThisFrame);
    if (pageChangedThisFrame) clearPageBody();

    float relAlt = rocketRelAltM();

    clearTextRow(HDR_H + 12, 80);
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.setTextSize(4);
    tft.setCursor(10, HDR_H + 20);
    tft.print(relAlt, 1);
    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 80);
    tft.print("m AGL");

    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    clearTextRow(HDR_H + 116, 28);
    tft.setCursor(10, HDR_H + 120);
    tft.print("V:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktVelMs, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print(" m/s  RSSI:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(lastCombinedRssi);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 144, 28);
    tft.setCursor(10, HDR_H + 148);
    tft.print("D:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    if (isnan(distanceToRocketM)) tft.print("---");
    else tft.print(distanceToRocketM, 0);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("m  BRG:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    if (isnan(bearingToRocketDeg)) tft.print("---");
    else tft.print(bearingToRocketDeg, 0);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    clearTextRow(HDR_H + 172, 28);
    tft.setCursor(10, HDR_H + 176);
    tft.print("AGE F/N/S:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    float ageF = packetAgeS(lastFlightPacketMs);
    float ageN = packetAgeS(lastNavPacketMs);
    float ageS = packetAgeS(lastStatusPacketMs);
    if (ageF < 0) tft.print("-"); else tft.print(ageF, 1);
    tft.print("/");
    if (ageN < 0) tft.print("-"); else tft.print(ageN, 1);
    tft.print("/");
    if (ageS < 0) tft.print("-"); else tft.print(ageS, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawRecovery() {
    drawHeader("[LANDED]", rocketName, pageChangedThisFrame);
    if (pageChangedThisFrame) clearPageBody();

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
    if (pageChangedThisFrame) clearPageBody();

    tft.setTextSize(2);
    tft.setCursor(10, HDR_H + 8);
    tft.print("AGE:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print((millis()-rocketLastPacketMs)/1000.0f, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("s  RSSI:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(lastCombinedRssi);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    tft.setCursor(10, HDR_H + 36);
    tft.print("BATT:");
    if (rocketBattCrit) tft.setTextColor(COLOR_BAD, COLOR_BG);
    else if (rocketBattWarn) tft.setTextColor(COLOR_WARN, COLOR_BG);
    else tft.setTextColor(COLOR_OK, COLOR_BG);
    tft.print(rocketBattV, 2);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("V  GPS:");
    tft.setTextColor(rocketHasFix ? COLOR_OK : COLOR_WARN, COLOR_BG);
    tft.print((int)rktSats);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("SV");

    tft.setCursor(10, HDR_H + 64);
    tft.print("DIST:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    if (isnan(distanceToRocketM)) tft.print("---");
    else tft.print(distanceToRocketM, 0);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("m  BRG:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    if (isnan(bearingToRocketDeg)) tft.print("---");
    else tft.print(bearingToRocketDeg, 0);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

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

    tft.setCursor(10, HDR_H + 148);
    tft.print("ALT:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rktAltGpsM, 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.print("m  AGL:");
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.print(rocketRelAltM(), 1);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);

    tft.setCursor(10, HDR_H + 176);
    tft.setTextColor(COLOR_WARN, COLOR_BG);
    tft.print("HOLD BTN 3S RESET");
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

static void drawLaunch() {
    drawHeader("[LAUNCH]", rocketName, pageChangedThisFrame);
    if (pageChangedThisFrame) clearPageBody();

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
        tft.setTextColor(pwr_armA_rearmRequired ? COLOR_WARN :
                         (pwr_key_ok ? COLOR_OK : COLOR_WARN), COLOR_BG);
        tft.print(pwr_armA_rearmRequired ? "REARM" :
                  (pwr_key_ok ? (pwr_armA_seen ? "ARM " : "SAFE") : "LOCK"));
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
        tft.setTextColor(pwr_armB_rearmRequired ? COLOR_WARN :
                         (pwr_key_ok ? COLOR_OK : COLOR_WARN), COLOR_BG);
        tft.print(pwr_armB_rearmRequired ? "REARM" :
                  (pwr_key_ok ? (pwr_armB_seen ? "ARM " : "SAFE") : "LOCK"));
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
    if (pageChangedThisFrame) clearPageBody();

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

static void drawStatusStrip() {
    const int y = SCREEN_H - 20;
    uint16_t color = COLOR_OK;
    const char *label = "OK";
    char detail[36];
    detail[0] = '\0';

    if (pwr_localBattCrit) {
        color = COLOR_BAD;
        label = "GND BATT CRIT";
        snprintf(detail, sizeof(detail), "%s %.2fV", power_local_battery_pack_name(), pwr_localVbat);
    } else if (pwr_localBattWarn) {
        color = COLOR_WARN;
        label = "GND BATT WARN";
        snprintf(detail, sizeof(detail), "%s %.2fV", power_local_battery_pack_name(), pwr_localVbat);
    } else if (rocketLastPacketMs == 0) {
        color = COLOR_WARN;
        label = "NO ROCKET LINK";
    } else if (!rocketLinkFresh()) {
        color = COLOR_BAD;
        label = "LINK LOST";
        snprintf(detail, sizeof(detail), "%.1fs", packetAgeS(rocketLastPacketMs));
    } else if (rocketBattCrit) {
        color = COLOR_BAD;
        label = "BATT CRIT";
        snprintf(detail, sizeof(detail), "%.2fV", rocketBattV);
    } else if (rocketBattWarn) {
        color = COLOR_WARN;
        label = "BATT WARN";
        snprintf(detail, sizeof(detail), "%.2fV", rocketBattV);
    } else if (!rocketImuOk || !rocketBaroOk || !rocketSdOk || !rocketNandOk || !rocketLogOk) {
        color = COLOR_WARN;
        label = "SYS WARN";
    } else {
        color = COLOR_OK;
        label = "LINK OK";
        snprintf(detail, sizeof(detail), "RSSI %d", lastCombinedRssi);
    }

    tft.fillRect(0, y, SCREEN_W, 20, color);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK, color);
    tft.setCursor(6, y + 3);
    tft.print(label);
    if (detail[0]) {
        int16_t x = SCREEN_W - (strlen(detail) * 12) - 6;
        if (x < 170) x = 170;
        tft.setCursor(x, y + 3);
        tft.print(detail);
    }
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
}

// No separate PEAK page; max altitude / velocity
// are shown on the [LANDED] page.
