#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "radio.h"
#include "sensors.h"
#include "sdlog.h"
#include "ui.h"
#include "power.h"
#include "timekeeper.h"
#include "settings.h"

static uint32_t lastPadLogMs = 0;
static uint32_t lastLostLogMs = 0;
static uint32_t lastFlightLogMs = 0;
static uint32_t lastRecoveryLogMs = 0;
static uint32_t lastGroundLogMs = 0;

static const char *phaseName(FlightPhase ph) {
    switch (ph) {
        case PHASE_PREFLIGHT: return "PREFLIGHT";
        case PHASE_FLIGHT: return "FLIGHT";
        case PHASE_RECOVERY: return "RECOVERY";
        case PHASE_LOST: return "LOST";
        default: return "UNK";
    }
}

static const char *rocketStateLogName() {
    switch (rocketFlightState) {
        case FS_IDLE: return "IDLE";
        case FS_PAD: return "PAD";
        case FS_ASCENT: return "ASCENT";
        case FS_COAST: return "COAST";
        case FS_SUBSONIC_COAST: return "SUBSONIC_COAST";
        case FS_NEAR_APOGEE: return "NEAR_APOGEE";
        case FS_DESCENT_BALLISTIC: return "DESCENT_BALLISTIC";
        case FS_UNDER_DROGUE: return "UNDER_DROGUE";
        case FS_DUAL_DEPLOY_APOGEE_LOGGED: return "DUAL_DEPLOY_APOGEE_LOGGED";
        case FS_DUAL_DEPLOY_MAIN_LOGGED: return "DUAL_DEPLOY_MAIN_LOGGED";
        case FS_POST_FLIGHT_GROUND: return "POST_FLIGHT_GROUND";
        case FS_LANDED: return "LANDED";
        case FS_ABORT: return "ABORT";
        default: return "UNK";
    }
}

static const char *rocketBattStatusLogName() {
    if (rocketBattCrit) return "CRIT";
    if (rocketBattWarn) return "WARN";
    if (rocketBattOk) return "OK";
    return "UNK";
}

static const char *groundBattStatusLogName() {
    if (pwr_localBattCrit) return "CRIT";
    if (pwr_localBattWarn) return "WARN";
    return "OK";
}

static float ageSeconds(uint32_t lastMs) {
    if (lastMs == 0) return -1.0f;
    return (millis() - lastMs) / 1000.0f;
}

static float rocketRelAltM() {
    if (isnan(rktBaseAltM)) return rktAltBaroM;
    return rktAltBaroM - rktBaseAltM;
}

void log_snapshot(const char *type, FlightPhase ph) {
    char line[768];
    snprintf(line, sizeof(line),
        "%s,ms=%lu,ground_fw=%s,phase=%s,rocket_state=%s,flags=%u,"
        "rkt_alt_baro=%.1f,rkt_rel_alt=%.1f,rkt_vel=%.1f,"
        "rkt_lat=%.6f,rkt_lon=%.6f,rkt_gps_alt=%.1f,rkt_fix=%u,rkt_sats=%u,rkt_hdop=%.1f,"
        "gnd_lat=%.6f,gnd_lon=%.6f,gnd_alt_gps=%.1f,gnd_alt_baro=%.1f,gnd_temp_c=%.1f,gnd_sats=%u,gnd_hdop=%.1f,"
        "distance_m=%.1f,bearing_deg=%.1f,"
        "rssi_f=%d,rssi_n=%d,rssi_s=%d,rssi_last=%d,"
        "age_f=%.1f,age_n=%.1f,age_s=%.1f,age_last=%.1f,"
        "rx_f=%lu,rx_n=%lu,rx_s=%lu,rate_f=%u,rate_n=%u,rate_s=%u,"
        "miss_f=%lu,miss_n=%lu,miss_s=%lu,"
        "rkt_batt=%.2f,rkt_batt_status=%s,launch_status=%u,launch_wait_s=%u,"
        "ok_gps=%u,ok_imu=%u,ok_baro=%u,ok_sd=%u,ok_nand=%u,ok_log=%u,"
        "gnd_gps_chars=%lu,gnd_gps_pass=%lu,gnd_gps_fail=%lu,gnd_loc_valid=%u,"
        "time_src=%s,time_valid=%u",
        type,
        (unsigned long)millis(),
        GROUND_FW_VERSION,
        phaseName(ph),
        rocketStateLogName(),
        (unsigned int)rktFlags,
        rktAltBaroM,
        rocketRelAltM(),
        rktVelMs,
        rktLat,
        rktLon,
        rktAltGpsM,
        (unsigned int)rktFixType,
        (unsigned int)rktSats,
        rktHdop,
        gndLat,
        gndLon,
        gndAltGpsM,
        gndAltBaroM,
        gndTempC,
        (unsigned int)gndSats,
        gndHdop,
        distanceToRocketM,
        bearingToRocketDeg,
        lastFlightRssi,
        lastNavRssi,
        lastStatusRssi,
        lastCombinedRssi,
        ageSeconds(lastFlightPacketMs),
        ageSeconds(lastNavPacketMs),
        ageSeconds(lastStatusPacketMs),
        ageSeconds(rocketLastPacketMs),
        (unsigned long)flightRxCount,
        (unsigned long)navRxCount,
        (unsigned long)statusRxCount,
        (unsigned int)flightRxRate,
        (unsigned int)navRxRate,
        (unsigned int)statusRxRate,
        (unsigned long)flightMissedCount,
        (unsigned long)navMissedCount,
        (unsigned long)statusMissedCount,
        rocketBattV,
        rocketBattStatusLogName(),
        (unsigned int)rocketLaunchStatus,
        (unsigned int)rocketLaunchWaitS,
        rocketGpsOk ? 1u : 0u,
        rocketImuOk ? 1u : 0u,
        rocketBaroOk ? 1u : 0u,
        rocketSdOk ? 1u : 0u,
        rocketNandOk ? 1u : 0u,
        rocketLogOk ? 1u : 0u,
        (unsigned long)gndGpsChars,
        (unsigned long)gndGpsPassed,
        (unsigned long)gndGpsFailed,
        gndGpsLocValid ? 1u : 0u,
        timekeeper_source_name(),
        timekeeper_hasTime() ? 1u : 0u
    );
    sdlog_write(line);
}

void log_ground() {
    char line[512];
#if ENABLE_POWER_MODULE
    snprintf(line, sizeof(line),
        "GND,ms=%lu,ground_fw=%s,"
        "gnd_lat=%.6f,gnd_lon=%.6f,gnd_alt_gps=%.1f,gnd_alt_baro=%.1f,gnd_temp_c=%.1f,gnd_sats=%u,gnd_hdop=%.1f,"
        "gnd_gps_chars=%lu,gnd_gps_pass=%lu,gnd_gps_fail=%lu,gnd_loc_valid=%u,"
        "gnd_v=%.2f,gnd_pack=%s,gnd_batt_status=%s,ign_v=%.1f,ia=%.1f,ib=%.1f,"
        "pwr_key=%u,presA=%u,presB=%u,fault=%u,onA=%u,onB=%u,pwr_link=%u,pwr_rx_rate=%u,pwr_tx_gap_ms=%u,pwr_tx_gap_max_ms=%u,"
        "rocket_seen=%u,rocket_age_s=%.1f,time_src=%s,time_valid=%u",
        (unsigned long)millis(),
        GROUND_FW_VERSION,
        gndLat,
        gndLon,
        gndAltGpsM,
        gndAltBaroM,
        gndTempC,
        (unsigned int)gndSats,
        gndHdop,
        (unsigned long)gndGpsChars,
        (unsigned long)gndGpsPassed,
        (unsigned long)gndGpsFailed,
        gndGpsLocValid ? 1u : 0u,
        pwr_localVbat,
        power_local_battery_pack_name(),
        groundBattStatusLogName(),
        pwr_vbat_x10 / 10.0f,
        pwr_ia_x10 / 10.0f,
        pwr_ib_x10 / 10.0f,
        pwr_key_ok ? 1u : 0u,
        pwr_presA ? 1u : 0u,
        pwr_presB ? 1u : 0u,
        pwr_faultAny ? 1u : 0u,
        pwr_onA ? 1u : 0u,
        pwr_onB ? 1u : 0u,
        power_link_fresh() ? 1u : 0u,
        (unsigned int)pwr_rxRate,
        (unsigned int)pwr_lastTxGapMs,
        (unsigned int)pwr_maxTxGapMs,
        rocketLastPacketMs != 0 ? 1u : 0u,
        ageSeconds(rocketLastPacketMs),
        timekeeper_source_name(),
        timekeeper_hasTime() ? 1u : 0u);
#else
    snprintf(line, sizeof(line),
        "GND,ms=%lu,ground_fw=%s,"
        "gnd_lat=%.6f,gnd_lon=%.6f,gnd_alt_gps=%.1f,gnd_alt_baro=%.1f,gnd_temp_c=%.1f,gnd_sats=%u,gnd_hdop=%.1f,"
        "gnd_gps_chars=%lu,gnd_gps_pass=%lu,gnd_gps_fail=%lu,gnd_loc_valid=%u,"
        "gnd_v=%.2f,gnd_pack=%s,gnd_batt_status=%s,"
        "rocket_seen=%u,rocket_age_s=%.1f,time_src=%s,time_valid=%u",
        (unsigned long)millis(),
        GROUND_FW_VERSION,
        gndLat,
        gndLon,
        gndAltGpsM,
        gndAltBaroM,
        gndTempC,
        (unsigned int)gndSats,
        gndHdop,
        (unsigned long)gndGpsChars,
        (unsigned long)gndGpsPassed,
        (unsigned long)gndGpsFailed,
        gndGpsLocValid ? 1u : 0u,
        pwr_localVbat,
        power_local_battery_pack_name(),
        groundBattStatusLogName(),
        rocketLastPacketMs != 0 ? 1u : 0u,
        ageSeconds(rocketLastPacketMs),
        timekeeper_source_name(),
        timekeeper_hasTime() ? 1u : 0u);
#endif
    sdlog_write_now(line);
}

void log_pad() {
    log_snapshot("PAD", PHASE_PREFLIGHT);
}

void log_flight() {
    log_snapshot("FLG", PHASE_FLIGHT);
}

void log_nav() {
    log_snapshot("NAV", PHASE_RECOVERY);
}

void log_lost() {
    log_snapshot("LOST", PHASE_LOST);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    DBG1(String("Booting GroundStation V10 ") + GROUND_FW_VERSION);

    groundSettingsInit();
    sensors_init();
    radio_init();
    timekeeper_init();
    sdlog_init();
#if ENABLE_POWER_MODULE
    power_init();
#endif
    ui_init();

    DBG1("GroundStation V10 READY");
}

void loop() {
#if ENABLE_POWER_MODULE
    power_update();
#endif
    groundSettingsTask();
    sensors_update();
#if ENABLE_POWER_MODULE
    power_update();
#endif
    radio_update();
#if ENABLE_POWER_MODULE
    power_update();
#endif
    bool timeChanged = timekeeper_update();
#if ENABLE_POWER_MODULE
    power_update();
#endif

    if (timeChanged && timekeeper_hasTime()) {
        DBG1(String("TIME ") + timekeeper_source_name() + " ACQUIRED -> timestamp logs");
        sdlog_onGpsTimeAvailable();
    }

    uint32_t now = millis();
    static FlightPhase lastPhase = PHASE_PREFLIGHT;
    FlightPhase ph = radio_getPhase();

    if (lastGroundLogMs == 0 || now - lastGroundLogMs > GROUND_LOG_MS) {
        lastGroundLogMs = now;
        log_ground();
    }

    if (ph != lastPhase) {
        DBG1(String("PHASE → ") + String(ph));
        lastPhase = ph;
    }

    switch (ph) {
        case PHASE_PREFLIGHT:
            // Before first rocket packet, ground-only rows do not help flight
            // reconstruction and can fill the SD with idle bench time.
            if (rocketLastPacketMs != 0 && now - lastPadLogMs > PAD_PRELOG_MS) {
                lastPadLogMs = now;
                log_pad();
            }
            break;
        case PHASE_FLIGHT:
            if (now - lastFlightLogMs > FLIGHT_LOG_MS) {
                lastFlightLogMs = now;
                log_flight();
            }
            break;
        case PHASE_RECOVERY:
            if (now - lastRecoveryLogMs > RECOVERY_LOG_MS) {
                lastRecoveryLogMs = now;
                log_nav();
            }
            break;
        case PHASE_LOST:
            if (now - lastLostLogMs > PAD_LOSTLOG_MS) {
                lastLostLogMs = now;
                log_lost();
            }
            break;
    }

#if ENABLE_POWER_MODULE
    power_update();
#endif
    ui_update();
#if ENABLE_POWER_MODULE
    power_update();
#endif
}
