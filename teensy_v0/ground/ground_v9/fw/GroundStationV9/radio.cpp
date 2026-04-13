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
uint32_t rocketStatusLastMs = 0;

float rocketBattV = ROCKET_BATT_VOLTAGE;

double gndLat = 0.0;
double gndLon = 0.0;
float  gndAltGpsM = 0.0f;
uint8_t gndFixType = 0;
uint8_t gndSats = 0;
float  gndHdop = 99.9f;

float distanceToRocketM = NAN;
float bearingToRocketDeg = NAN;

volatile bool flightPending = false;
volatile bool navPending    = false;
volatile bool statusPending = false;

FlightPacketV7 flightBuf;
NavPacketV7    navBuf;
StatusPacketV8 statusBuf;

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
        DBG2("RX FLIGHT PKT");
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
        DBG2("RX NAV PKT");
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
        DBG2("RX STATUS PKT");
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

    static uint32_t lastGpsMs = 0;
    uint32_t now = millis();
    if (now - lastGpsMs >= GND_GPS_UPDATE_MS) {
        lastGpsMs = now;
        gndLat = gps.location.lat();
        gndLon = gps.location.lng();
        gndAltGpsM = gps.altitude.meters();
        gndSats = gps.satellites.value();
        gndHdop = gps.hdop.value() * 0.01f;
        gndFixType = gps.location.isValid() ? (gps.altitude.isValid() ? 3 : 2) : 0;
        ui_markDirty();
    }

    // Flight packet
    if (flightPending) {
        noInterrupts();
        FlightPacketV7 pkt = flightBuf;
        flightPending = false;
        interrupts();

        rocketLastPacketMs = millis();
        rocketLaunched = (pkt.flags & FLAG_LAUNCH);
        rocketLanded   = (pkt.flags & FLAG_LANDED);
        rktFlags       = pkt.flags;
        rktAltBaroM    = pkt.alt_cm / 100.0f;
        rktVelMs       = pkt.vel_cms / 100.0f;
        rocketFlightState = (RocketFlightState)pkt.state;

        rocketImuOk  = true;
        rocketBaroOk = true;

        DBG3(String("FLIGHT: alt=") + String(pkt.alt_cm/100.0f) +
            " vel=" + String(pkt.vel_cms/100.0f));
        ui_markDirty();
    }

    // Nav packet
    if (navPending) {
        noInterrupts();
        NavPacketV7 pkt = navBuf;
        navPending = false;
        interrupts();

        rocketLastPacketMs = millis();

        rktFixType   = pkt.gps_fix_type;
        rktSats      = pkt.gps_sats;
        rktHdop      = pkt.gps_hdop_x10 / 10.0f;
        rktLat       = pkt.gps_lat_e7 / 1e7;
        rktLon       = pkt.gps_lon_e7 / 1e7;
        rktAltGpsM   = pkt.gps_alt_cm / 100.0f;
        rktAltBaroM  = pkt.baro_alt_cm / 100.0f;
        rocketHasFix = (rktFixType >= 2);

        DBG3(String("NAV: lat=") + String(rktLat,6) +
             " lon=" + String(rktLon,6));
        ui_markDirty();
    }

    if (statusPending) {
        noInterrupts();
        StatusPacketV8 pkt = statusBuf;
        statusPending = false;
        interrupts();

        rocketStatusLastMs = millis();
        rocketBattV = pkt.batt_mv / 1000.0f;
        rktSats = pkt.gps_sats;
        rocketBaroOk = (pkt.health_flags & HEALTH_BARO_OK);
        rocketImuOk  = (pkt.health_flags & HEALTH_IMU_OK);
        rocketGpsOk  = (pkt.health_flags & HEALTH_GPS_OK);
        rocketSdOk   = (pkt.health_flags & HEALTH_SD_OK);
        rocketNandOk = (pkt.health_flags & HEALTH_NAND_OK);
        rocketLogOk  = (pkt.health_flags & HEALTH_LOG_OK);
        ui_markDirty();
    }

    // Capture pad altitude before or at launch for relative AGL
    if (!rocketLaunched) {
        if (!isnan(rktAltBaroM)) {
            rktBaseAltM = rktAltBaroM;
        }
    } else if (isnan(rktBaseAltM) && !isnan(rktAltBaroM)) {
        rktBaseAltM = rktAltBaroM;
    }

    if (!launchedBefore && rocketLaunched) {
        rktMaxAltM = rktAltBaroM;
        rktMaxVelMs = max(0.0f, rktVelMs);
    }

    if (rocketLaunched) {
        if (rktAltBaroM > rktMaxAltM) rktMaxAltM = rktAltBaroM;
        if (rktVelMs > rktMaxVelMs)   rktMaxVelMs = rktVelMs;
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

    distanceToRocketM = NAN;
    bearingToRocketDeg = NAN;
    rocketBattV = ROCKET_BATT_VOLTAGE;
    rocketImuOk = true;
    rocketBaroOk = true;
    rocketGpsOk = false;
    rocketSdOk = false;
    rocketNandOk = false;
    rocketLogOk = false;
    rocketStatusLastMs = 0;

    ui_markDirty();
}
