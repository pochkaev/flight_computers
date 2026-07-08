#include "radio.h"
#include "config.h"
#include "ui.h"

TinyGPSPlus gps;
char rocketName[16] = DEFAULT_ROCKET_NAME;

bool rocketHasFix = false;
bool rocketLaunched = false;
bool rocketLanded = false;
uint32_t rocketLastPacketMs = 0;

int lastFlightRssi = -200;
int lastNavRssi    = -200;
int lastStatusRssi = -200;
int lastCombinedRssi = -200;
uint32_t lastFlightPacketMs = 0;
uint32_t lastNavPacketMs = 0;
uint32_t lastStatusPacketMs = 0;
uint32_t lastIdentityPacketMs = 0;
uint32_t lastPyroConfigPacketMs = 0;
uint32_t lastPyroEventPacketMs = 0;
uint32_t flightRxCount = 0;
uint32_t navRxCount = 0;
uint32_t statusRxCount = 0;
uint32_t flightMissedCount = 0;
uint32_t navMissedCount = 0;
uint32_t statusMissedCount = 0;
uint16_t flightRxRate = 0;
uint16_t navRxRate = 0;
uint16_t statusRxRate = 0;

double rktLat = 0.0;
double rktLon = 0.0;
float  rktAltGpsM = 0.0f;
float  rktAltBaroM = 0.0f;
uint8_t rktFixType = 0;
uint8_t rktSats = 0;
float  rktHdop = 99.9f;
uint16_t rktFlags = 0;
float  rktVelMs = 0.0f;
float  rktMaxAltM = 0.0f;
float  rktMaxVelMs = 0.0f;
float  rktBaseAltM = NAN;
RocketFlightState rocketFlightState = FS_IDLE;

bool rocketImuOk = true;
bool rocketBaroOk = true;
bool rocketGpsOk = false;
bool rocketSdOk = false;
bool rocketNandOk = false;
bool rocketLogOk = false;
bool rocketBattOk = true;
bool rocketBattWarn = false;
bool rocketBattCrit = false;
uint32_t rocketStatusLastMs = 0;
uint8_t rocketLaunchStatus = LAUNCH_STATUS_INHIBIT;
uint16_t rocketLaunchWaitS = 0;

bool rocketPyroConfigValid = false;
uint8_t rocketPyroChannelCount = 0;
uint8_t rocketPyroOutputEnabled = 0;
uint8_t rocketPyroActiveHigh = 1;
uint16_t rocketPyroFireMs = 0;
uint16_t rocketPyroApogeeDelayMs = 0;
uint16_t rocketPyroMainMinAfterApogeeMs = 0;
uint16_t rocketPyroMainAltM = 0;
char rocketPyroFlightProfile = 'S';
char rocketPyroChannelFunc[4] = {'N', 'N', 'N', 'N'};
uint8_t rocketPyroChannelPin[4] = {};
uint8_t rocketPyroChannelLogMask = 0;
uint8_t rocketPyroChannelOutputMask = 0;
uint8_t rocketLastPyroEventType = 0;
uint8_t rocketLastPyroEventChannel = 0xFF;
char rocketLastPyroEventFunction = 'N';
uint32_t rocketLastPyroEventSeq = 0;
bool rocketPyroFlashPending = false;

float rocketBattV = ROCKET_BATT_VOLTAGE;

double gndLat = 0.0;
double gndLon = 0.0;
float  gndAltGpsM = 0.0f;
uint8_t gndFixType = 0;
uint8_t gndSats = 0;
float  gndHdop = 99.9f;
uint32_t gndGpsChars = 0;
uint32_t gndGpsPassed = 0;
uint32_t gndGpsFailed = 0;
uint32_t gndGpsSentencesWithFix = 0;
bool gndGpsLocValid = false;
bool gndGpsAltValid = false;
bool gndGpsDateValid = false;
bool gndGpsTimeValid = false;

float distanceToRocketM = NAN;
float bearingToRocketDeg = NAN;

volatile bool flightPending = false;
volatile bool navPending    = false;
volatile bool statusPending = false;
volatile bool identityPending = false;
volatile bool pyroConfigPending = false;
volatile bool pyroEventPending = false;

FlightPacketV7 flightBuf;
NavPacketV7    navBuf;
StatusPacketV8 statusBuf;
IdentityPacketV1 identityBuf;
PyroConfigPacketV1 pyroConfigBuf;
PyroEventPacketV1 pyroEventBuf;
static bool haveFlightSeq = false;
static bool haveNavSeq = false;
static bool haveStatusSeq = false;
static uint32_t lastFlightSeq = 0;
static uint32_t lastNavSeq = 0;
static uint32_t lastStatusSeq = 0;
static const uint32_t MAX_REASONABLE_SEQ_GAP = 1000;
static uint16_t flightRxCountWindow = 0;
static uint16_t navRxCountWindow = 0;
static uint16_t statusRxCountWindow = 0;
static uint32_t lastRateMs = 0;

static bool isReasonableRocketBaroAlt(float altM) {
    return isfinite(altM) && altM > -500.0f && altM < 10000.0f;
}

static bool isReasonableRocketRelAlt(float relAltM) {
    return isfinite(relAltM) && relAltM > -100.0f && relAltM < 10000.0f;
}

static float rocketRelAltFromBaro() {
    if (isnan(rktBaseAltM)) return rktAltBaroM;
    return rktAltBaroM - rktBaseAltM;
}

static const char *flightStateName(RocketFlightState st) {
    switch (st) {
        case FS_IDLE:                      return "IDLE";
        case FS_PAD:                       return "PAD";
        case FS_ASCENT:                    return "ASCENT";
        case FS_COAST:                     return "COAST";
        case FS_SUBSONIC_COAST:            return "SUBSONIC_COAST";
        case FS_NEAR_APOGEE:               return "NEAR_APOGEE";
        case FS_DESCENT_BALLISTIC:         return "DESCENT_BALLISTIC";
        case FS_UNDER_DROGUE:              return "UNDER_DROGUE";
        case FS_DUAL_DEPLOY_APOGEE_LOGGED: return "DUAL_DEPLOY_APOGEE_LOGGED";
        case FS_DUAL_DEPLOY_MAIN_LOGGED:   return "DUAL_DEPLOY_MAIN_LOGGED";
        case FS_POST_FLIGHT_GROUND:        return "POST_FLIGHT_GROUND";
        case FS_LANDED:                    return "LANDED";
        case FS_ABORT:                     return "ABORT";
        default:                           return "UNK";
    }
}

static void debugFlightPacket() {
    char line[128];
    snprintf(line, sizeof(line),
             "FLIGHT state=%s alt=%.1f vel=%.1f flags=%u rssi=%d",
             flightStateName(rocketFlightState),
             rktAltBaroM,
             rktVelMs,
             (unsigned int)rktFlags,
             lastFlightRssi);
    DBG2(line);
}

static void debugNavPacket() {
    char line[160];
    snprintf(line, sizeof(line),
             "NAV fix=%u sats=%u hdop=%.1f lat=%.6f lon=%.6f gpsAlt=%.1f rssi=%d",
             (unsigned int)rktFixType,
             (unsigned int)rktSats,
             rktHdop,
             rktLat,
             rktLon,
             rktAltGpsM,
             lastNavRssi);
    DBG2(line);
}

static void debugStatusPacket() {
    char line[192];
    snprintf(line, sizeof(line),
             "STATUS state=%s launch=%u wait=%u batt=%.2f battStatus=%s gps=%u imu=%u baro=%u sd=%u nand=%u log=%u rssi=%d",
             flightStateName(rocketFlightState),
             (unsigned int)rocketLaunchStatus,
             (unsigned int)rocketLaunchWaitS,
             rocketBattV,
             rocketBattCrit ? "CRIT" : (rocketBattWarn ? "WARN" : "OK"),
             rocketGpsOk ? 1u : 0u,
             rocketImuOk ? 1u : 0u,
             rocketBaroOk ? 1u : 0u,
             rocketSdOk ? 1u : 0u,
             rocketNandOk ? 1u : 0u,
             rocketLogOk ? 1u : 0u,
             lastStatusRssi);
    DBG2(line);
}

static const char *pyroFuncName(char func) {
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

static void debugPyroConfigPacket() {
    char line[160];
    snprintf(line, sizeof(line),
             "PYROCFG out=%u profile=%c ch1=%s ch2=%s ch3=%s ch4=%s",
             (unsigned int)rocketPyroOutputEnabled,
             rocketPyroFlightProfile,
             pyroFuncName(rocketPyroChannelFunc[0]),
             pyroFuncName(rocketPyroChannelFunc[1]),
             pyroFuncName(rocketPyroChannelFunc[2]),
             pyroFuncName(rocketPyroChannelFunc[3]));
    DBG2(line);
}

static void debugPyroEventPacket() {
    char line[128];
    snprintf(line, sizeof(line),
             "PYROEV type=%u ch=%u func=%s seq=%lu",
             (unsigned int)rocketLastPyroEventType,
             (unsigned int)rocketLastPyroEventChannel,
             pyroFuncName(rocketLastPyroEventFunction),
             (unsigned long)rocketLastPyroEventSeq);
    DBG1(line);
}

static bool applyReceivedRocketName(const IdentityPacketV1 &pkt) {
    if (pkt.version != 1) return false;

    uint8_t maxLen = pkt.name_len;
    if (maxLen > sizeof(pkt.name)) maxLen = sizeof(pkt.name);

    char cleaned[sizeof(rocketName)] = {};
    size_t out = 0;
    for (uint8_t i = 0; i < maxLen && out < sizeof(cleaned) - 1; ++i) {
        char c = pkt.name[i];
        if (c == '\0') break;
        bool allowed =
            (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '_' || c == '-' || c == '.';
        if (allowed) cleaned[out++] = c;
    }

    if (out == 0) return false;
    cleaned[out] = '\0';
    if (strncmp(rocketName, cleaned, sizeof(rocketName)) == 0) return false;
    strncpy(rocketName, cleaned, sizeof(rocketName) - 1);
    rocketName[sizeof(rocketName) - 1] = '\0';
    return true;
}

static void updateRocketBatteryStatusFromVoltage() {
    const bool pack2s = rocketBattV >= ROCKET_BATT_2S_DETECT_UP_V;
    const float warnV = pack2s ? ROCKET_BATT_2S_WARN_V : ROCKET_BATT_1S_WARN_V;
    const float critV = pack2s ? ROCKET_BATT_2S_CRIT_V : ROCKET_BATT_1S_CRIT_V;

    rocketBattCrit = rocketBattV <= critV;
    rocketBattWarn = !rocketBattCrit && rocketBattV <= warnV;
    rocketBattOk = !rocketBattWarn && !rocketBattCrit;
}

static void updateSeqStats(uint32_t seq, bool &haveSeq, uint32_t &lastSeq, uint32_t &missedCount) {
    if (!haveSeq) {
        haveSeq = true;
        lastSeq = seq;
        return;
    }
    if (seq <= lastSeq) {
        // Treat sequence regressions as a sender reset/reboot, not packet loss.
        lastSeq = seq;
        return;
    }
    uint32_t delta = seq - lastSeq;
    if (delta > MAX_REASONABLE_SEQ_GAP) {
        // Treat huge forward jumps as sender reset/desync/corrupt packet, not real packet loss.
        lastSeq = seq;
        return;
    }
    if (delta > 1u) {
        missedCount += (delta - 1u);
    }
    lastSeq = seq;
}

void radio_onReceive(int packetSize) {
    if (packetSize <= 0) {
        while (LoRa.available()) LoRa.read();
        return;
    }

    int typeByte = LoRa.read();
    int remaining = packetSize - 1;

    if (typeByte == PKT_TYPE_FLIGHT_V7) {
        if (remaining != (int)sizeof(FlightPacketV7)) {
            while (LoRa.available()) LoRa.read();
            return;
        }
        uint8_t *p = (uint8_t*)&flightBuf;
        for (int i=0; i<remaining && LoRa.available(); ++i) {
            p[i] = LoRa.read();
        }
        flightPending = true;
        lastFlightRssi = LoRa.packetRssi();
        lastCombinedRssi = lastFlightRssi;
    }
    else if (typeByte == PKT_TYPE_NAV_V7) {
        if (remaining != (int)sizeof(NavPacketV7)) {
            while (LoRa.available()) LoRa.read();
            return;
        }
        uint8_t *p = (uint8_t*)&navBuf;
        for (int i=0; i<remaining && LoRa.available(); ++i) {
            p[i] = LoRa.read();
        }
        navPending = true;
        lastNavRssi = LoRa.packetRssi();
        lastCombinedRssi = lastNavRssi;
    }
    else if (typeByte == PKT_TYPE_STATUS_V8) {
        if (remaining != (int)sizeof(StatusPacketV8)) {
            while (LoRa.available()) LoRa.read();
            return;
        }
        uint8_t *p = (uint8_t*)&statusBuf;
        for (int i=0; i<remaining && LoRa.available(); ++i) {
            p[i] = LoRa.read();
        }
        statusPending = true;
        lastStatusRssi = LoRa.packetRssi();
        lastCombinedRssi = lastStatusRssi;
    }
    else if (typeByte == PKT_TYPE_IDENTITY_V1) {
        if (remaining != (int)sizeof(IdentityPacketV1)) {
            while (LoRa.available()) LoRa.read();
            return;
        }
        uint8_t *p = (uint8_t*)&identityBuf;
        for (int i=0; i<remaining && LoRa.available(); ++i) {
            p[i] = LoRa.read();
        }
        identityPending = true;
        lastCombinedRssi = LoRa.packetRssi();
    }
    else if (typeByte == PKT_TYPE_PYRO_CONFIG_V1) {
        if (remaining != (int)sizeof(PyroConfigPacketV1)) {
            while (LoRa.available()) LoRa.read();
            return;
        }
        uint8_t *p = (uint8_t*)&pyroConfigBuf;
        for (int i=0; i<remaining && LoRa.available(); ++i) {
            p[i] = LoRa.read();
        }
        pyroConfigPending = true;
        lastCombinedRssi = LoRa.packetRssi();
    }
    else if (typeByte == PKT_TYPE_PYRO_EVENT_V1) {
        if (remaining != (int)sizeof(PyroEventPacketV1)) {
            while (LoRa.available()) LoRa.read();
            return;
        }
        uint8_t *p = (uint8_t*)&pyroEventBuf;
        for (int i=0; i<remaining && LoRa.available(); ++i) {
            p[i] = LoRa.read();
        }
        pyroEventPending = true;
        lastCombinedRssi = LoRa.packetRssi();
    }
    else {
        while (LoRa.available()) LoRa.read();
    }
}

void radio_init() {
    Serial1.begin(9600);

    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQUENCY)) {
        DBG1("LoRa init FAILED – running without radio");
        return;  // Continue running with whatever data we have
    }
    LoRa.setSPIFrequency(LORA_SPI_FREQ);
    LoRa.onReceive(radio_onReceive);
    LoRa.receive();

    DBG1("LoRa + GPS initialized");
}

float calculateDistanceM(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double p1 = lat1 * (PI/180.0);
    double p2 = lat2 * (PI/180.0);
    double dp = (lat2 - lat1) * (PI/180.0);
    double dl = (lon2 - lon1) * (PI/180.0);

    double a = sin(dp/2)*sin(dp/2) +
               cos(p1)*cos(p2)*sin(dl/2)*sin(dl/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return (float)(R * c);
}

float calculateBearingDeg(double lat1, double lon1, double lat2, double lon2) {
    double p1 = lat1*(PI/180.0);
    double p2 = lat2*(PI/180.0);
    double dl = (lon2-lon1)*(PI/180.0);

    double y = sin(dl)*cos(p2);
    double x = cos(p1)*sin(p2) - sin(p1)*cos(p2)*cos(dl);
    double br = atan2(y,x) * (180.0/PI);
    if (br < 0) br += 360.0;
    return (float)br;
}

void radio_update() {
    bool launchedBefore = rocketLaunched;

    // Ground GPS:
    // Always feed TinyGPS++ with incoming bytes so it can parse sentences,
    // but only update our copied values at the configured cadence.
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }
    gndGpsChars = gps.charsProcessed();
    gndGpsPassed = gps.passedChecksum();
    gndGpsFailed = gps.failedChecksum();
    gndGpsSentencesWithFix = gps.sentencesWithFix();
    gndGpsLocValid = gps.location.isValid();
    gndGpsAltValid = gps.altitude.isValid();
    gndGpsDateValid = gps.date.isValid();
    gndGpsTimeValid = gps.time.isValid();

    static uint32_t lastGpsMs = 0;
    uint32_t now = millis();
    if (now - lastRateMs >= 1000) {
        lastRateMs = now;
        flightRxRate = flightRxCountWindow;
        navRxRate = navRxCountWindow;
        statusRxRate = statusRxCountWindow;
        flightRxCountWindow = 0;
        navRxCountWindow = 0;
        statusRxCountWindow = 0;
        ui_markDirty();
    }
    if (now - lastGpsMs >= GND_GPS_UPDATE_MS) {
        lastGpsMs = now;
        if (gndGpsLocValid) {
            gndLat = gps.location.lat();
            gndLon = gps.location.lng();
        }
        if (gndGpsAltValid) {
            gndAltGpsM = gps.altitude.meters();
        }
        gndSats = gps.satellites.isValid() ? gps.satellites.value() : 0;
        gndHdop = gps.hdop.isValid() ? (gps.hdop.value() * 0.01f) : 99.9f;
        gndFixType = gndGpsLocValid ? (gndGpsAltValid ? 3 : 2) : 0;
        ui_markDirty();
    }
    static uint32_t lastGpsDbgMs = 0;
    if (SERIAL_DEBUG_LEVEL >= 1 && (now - lastGpsDbgMs) >= 5000) {
        lastGpsDbgMs = now;
        char line[160];
        snprintf(line, sizeof(line),
                 "GND GPS chars=%lu pass=%lu fail=%lu fix=%u sats=%u hdop=%.1f loc=%u alt=%u date=%u time=%u",
                 (unsigned long)gndGpsChars,
                 (unsigned long)gndGpsPassed,
                 (unsigned long)gndGpsFailed,
                 (unsigned int)gndFixType,
                 (unsigned int)gndSats,
                 gndHdop,
                 gndGpsLocValid ? 1u : 0u,
                 gndGpsAltValid ? 1u : 0u,
                 gndGpsDateValid ? 1u : 0u,
                 gndGpsTimeValid ? 1u : 0u);
        DBG1(line);
    }

    if (identityPending) {
        noInterrupts();
        IdentityPacketV1 pkt = identityBuf;
        identityPending = false;
        interrupts();

        lastIdentityPacketMs = millis();
        if (applyReceivedRocketName(pkt)) {
            ui_markDirty();
        }
    }

    if (pyroConfigPending) {
        noInterrupts();
        PyroConfigPacketV1 pkt = pyroConfigBuf;
        pyroConfigPending = false;
        interrupts();

        if (pkt.version == 1) {
            rocketLastPacketMs = millis();
            lastPyroConfigPacketMs = rocketLastPacketMs;
            rocketPyroConfigValid = true;
            rocketPyroChannelCount = pkt.channel_count > 4 ? 4 : pkt.channel_count;
            rocketPyroOutputEnabled = pkt.output_enabled;
            rocketPyroActiveHigh = pkt.active_high;
            rocketPyroFireMs = pkt.fire_ms;
            rocketPyroApogeeDelayMs = pkt.apogee_delay_ms;
            rocketPyroMainMinAfterApogeeMs = pkt.main_min_after_apogee_ms;
            rocketPyroMainAltM = pkt.main_alt_m;
            rocketPyroFlightProfile = pkt.flight_profile;
            for (uint8_t i = 0; i < 4; ++i) {
                rocketPyroChannelFunc[i] = pkt.channel_func[i];
                rocketPyroChannelPin[i] = pkt.channel_pin[i];
            }
            rocketPyroChannelLogMask = pkt.channel_log_mask;
            rocketPyroChannelOutputMask = pkt.channel_output_mask;
            debugPyroConfigPacket();
            ui_markDirty();
        }
    }

    if (pyroEventPending) {
        noInterrupts();
        PyroEventPacketV1 pkt = pyroEventBuf;
        pyroEventPending = false;
        interrupts();

        if (pkt.version == 1) {
            rocketLastPacketMs = millis();
            lastPyroEventPacketMs = rocketLastPacketMs;
            rocketLastPyroEventType = pkt.event_type;
            rocketLastPyroEventChannel = pkt.channel_index;
            rocketLastPyroEventFunction = pkt.function;
            rocketLastPyroEventSeq = pkt.seq;
            rocketFlightState = (RocketFlightState)pkt.state;
            rktFlags = pkt.flags;
            rocketPyroFlashPending = true;
            debugPyroEventPacket();
            ui_markDirty();
        }
    }

    // Flight packet
    if (flightPending) {
        noInterrupts();
        FlightPacketV7 pkt = flightBuf;
        flightPending = false;
        interrupts();

        rocketLastPacketMs = millis();
        lastFlightPacketMs = rocketLastPacketMs;
        flightRxCount++;
        flightRxCountWindow++;
        updateSeqStats(pkt.seq, haveFlightSeq, lastFlightSeq, flightMissedCount);
        rocketLaunched = (pkt.flags & FLAG_LAUNCH);
        rocketLanded   = (pkt.flags & FLAG_LANDED);
        rktFlags       = pkt.flags;
        rktAltBaroM    = pkt.alt_cm / 100.0f;
        rktVelMs       = pkt.vel_cms / 100.0f;
        rocketFlightState = (RocketFlightState)pkt.state;

        rocketImuOk  = true;
        rocketBaroOk = true;

        debugFlightPacket();
        ui_markDirty();
    }

    // Nav packet
    if (navPending) {
        noInterrupts();
        NavPacketV7 pkt = navBuf;
        navPending = false;
        interrupts();

        rocketLastPacketMs = millis();
        lastNavPacketMs = rocketLastPacketMs;
        navRxCount++;
        navRxCountWindow++;
        updateSeqStats(pkt.seq, haveNavSeq, lastNavSeq, navMissedCount);

        rktFixType   = pkt.gps_fix_type;
        rktSats      = pkt.gps_sats;
        rktHdop      = pkt.gps_hdop_x10 / 10.0f;
        rktLat       = pkt.gps_lat_e7 / 1e7;
        rktLon       = pkt.gps_lon_e7 / 1e7;
        rktAltGpsM   = pkt.gps_alt_cm / 100.0f;
        rktAltBaroM  = pkt.baro_alt_cm / 100.0f;
        rocketHasFix = (rktFixType >= 2);

        debugNavPacket();
        ui_markDirty();
    }

    if (statusPending) {
        noInterrupts();
        StatusPacketV8 pkt = statusBuf;
        statusPending = false;
        interrupts();

        rocketStatusLastMs = millis();
        lastStatusPacketMs = rocketStatusLastMs;
        statusRxCount++;
        statusRxCountWindow++;
        updateSeqStats(pkt.seq, haveStatusSeq, lastStatusSeq, statusMissedCount);
        rocketFlightState = (RocketFlightState)pkt.state;
        rocketBattV = pkt.batt_mv / 1000.0f;
        rktSats = pkt.gps_sats;
        rocketLaunchStatus = pkt.launch_status;
        rocketLaunchWaitS = pkt.launch_wait_s;
        rocketBaroOk = (pkt.health_flags & HEALTH_BARO_OK);
        rocketImuOk  = (pkt.health_flags & HEALTH_IMU_OK);
        rocketGpsOk  = (pkt.health_flags & HEALTH_GPS_OK);
        rocketSdOk   = (pkt.health_flags & HEALTH_SD_OK);
        rocketNandOk = (pkt.health_flags & HEALTH_NAND_OK);
        rocketLogOk  = (pkt.health_flags & HEALTH_LOG_OK);
        updateRocketBatteryStatusFromVoltage();
        if ((pkt.health_flags & HEALTH_BATT_OK) == 0 && rocketBattOk) {
            rocketBattWarn = true;
            rocketBattOk = false;
        }
        debugStatusPacket();
        ui_markDirty();
    }

    // Capture pad altitude before or at launch for relative AGL
    if (!rocketLaunched) {
        if (isReasonableRocketBaroAlt(rktAltBaroM)) {
            rktBaseAltM = rktAltBaroM;
        }
    } else if (isnan(rktBaseAltM) && isReasonableRocketBaroAlt(rktAltBaroM)) {
        rktBaseAltM = rktAltBaroM;
    }

    const float relAltM = rocketRelAltFromBaro();
    const bool relAltOk = isReasonableRocketRelAlt(relAltM);

    if (!launchedBefore && rocketLaunched) {
        rktMaxAltM = relAltOk ? max(0.0f, relAltM) : 0.0f;
        rktMaxVelMs = (isfinite(rktVelMs) && rktVelMs < 1000.0f) ? max(0.0f, rktVelMs) : 0.0f;
    }

    if (rocketLaunched) {
        if (relAltOk && relAltM > rktMaxAltM) rktMaxAltM = relAltM;
        if (isfinite(rktVelMs) && rktVelMs > rktMaxVelMs && rktVelMs < 1000.0f) rktMaxVelMs = rktVelMs;
    }

    // Distance / bearing
    if (gndFixType > 0 && rocketHasFix) {
        distanceToRocketM  = calculateDistanceM(gndLat, gndLon, rktLat, rktLon);
        bearingToRocketDeg = calculateBearingDeg(gndLat, gndLon, rktLat, rktLon);
    } else {
        distanceToRocketM  = NAN;
        bearingToRocketDeg = NAN;
    }
}

FlightPhase radio_getPhase() {
    if (rocketLastPacketMs == 0) return PHASE_PREFLIGHT;

    uint32_t age = millis() - rocketLastPacketMs;

    if (age > LINK_LOST_MS) return PHASE_LOST;
    if (!rocketLaunched)    return PHASE_PREFLIGHT;
    if (rocketLanded)       return PHASE_RECOVERY;
    return PHASE_FLIGHT;
}

void radio_resetState() {
    rocketHasFix = false;
    rocketLaunched = false;
    rocketLanded = false;
    rocketLastPacketMs = 0;

    rktLat = 0.0;
    rktLon = 0.0;
    rktAltGpsM = 0.0f;
    rktAltBaroM = 0.0f;
    rktFixType = 0;
    rktSats = 0;
    rktHdop = 99.9f;
    rktFlags = 0;
    rktVelMs = 0.0f;
    rktMaxAltM = 0.0f;
    rktMaxVelMs = 0.0f;
    rktBaseAltM = NAN;
    rocketFlightState = FS_IDLE;

    lastFlightRssi = -200;
    lastNavRssi = -200;
    lastStatusRssi = -200;
    lastCombinedRssi = -200;
    lastFlightPacketMs = 0;
    lastNavPacketMs = 0;
    lastStatusPacketMs = 0;
    lastIdentityPacketMs = 0;
    lastPyroConfigPacketMs = 0;
    lastPyroEventPacketMs = 0;
    flightRxCount = 0;
    navRxCount = 0;
    statusRxCount = 0;
    flightMissedCount = 0;
    navMissedCount = 0;
    statusMissedCount = 0;
    flightRxRate = 0;
    navRxRate = 0;
    statusRxRate = 0;
    haveFlightSeq = false;
    haveNavSeq = false;
    haveStatusSeq = false;
    lastFlightSeq = 0;
    lastNavSeq = 0;
    lastStatusSeq = 0;
    flightRxCountWindow = 0;
    navRxCountWindow = 0;
    statusRxCountWindow = 0;
    lastRateMs = millis();

    distanceToRocketM = NAN;
    bearingToRocketDeg = NAN;
    rocketBattV = ROCKET_BATT_VOLTAGE;
    rocketImuOk = true;
    rocketBaroOk = true;
    rocketGpsOk = false;
    rocketSdOk = false;
    rocketNandOk = false;
    rocketLogOk = false;
    rocketBattOk = true;
    rocketBattWarn = false;
    rocketBattCrit = false;
    rocketStatusLastMs = 0;
    rocketLaunchStatus = LAUNCH_STATUS_INHIBIT;
    rocketLaunchWaitS = 0;
    rocketPyroConfigValid = false;
    rocketPyroFlashPending = false;
    rocketPyroChannelOutputMask = 0;

    ui_markDirty();
}
