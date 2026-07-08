#pragma once
#include <Arduino.h>

// Flight phases for auto-page switching
enum FlightPhase : uint8_t {
    PHASE_PREFLIGHT = 0,
    PHASE_FLIGHT,
    PHASE_RECOVERY,
    PHASE_LOST
};

// UI pages
enum PageIndex : uint8_t {
    PAGE_PREFLIGHT = 0,
    PAGE_ROCKET_STATUS,
    PAGE_PYRO_CONFIG,
    PAGE_LAUNCH,
    PAGE_FLIGHT,
    PAGE_RECOVERY,
    PAGE_LOST,
    PAGE_SIGNAL,
    PAGE_COUNT
};

// Rocket flight state (from rocket telemetry)
enum RocketFlightState : uint8_t {
    FS_IDLE = 0,
    FS_PAD,
    FS_ASCENT,
    FS_COAST,
    FS_SUBSONIC_COAST,
    FS_NEAR_APOGEE,
    FS_DESCENT_BALLISTIC,
    FS_UNDER_DROGUE,
    FS_DUAL_DEPLOY_APOGEE_LOGGED,
    FS_DUAL_DEPLOY_MAIN_LOGGED,
    FS_POST_FLIGHT_GROUND,
    FS_LANDED,
    FS_ABORT
};

// Rocket flags from flight packet
const uint16_t FLAG_LAUNCH = 1 << 0;
const uint16_t FLAG_APOGEE = 1 << 1;
const uint16_t FLAG_LANDED = 1 << 2;
const uint16_t FLAG_DUAL_DEPLOY_APOGEE_LOG = 1 << 3;
const uint16_t FLAG_DUAL_DEPLOY_MAIN_LOG = 1 << 4;
const uint16_t FLAG_PYRO_CH1_LOGGED = 1 << 8;
const uint16_t FLAG_PYRO_CH2_LOGGED = 1 << 9;
const uint16_t FLAG_PYRO_CH3_LOGGED = 1 << 10;
const uint16_t FLAG_PYRO_CH4_LOGGED = 1 << 11;

// Rocket status flags from the new rocket_v8 status packet
const uint16_t HEALTH_BARO_OK = 1 << 0;
const uint16_t HEALTH_IMU_OK  = 1 << 1;
const uint16_t HEALTH_GPS_OK  = 1 << 2;
const uint16_t HEALTH_SD_OK   = 1 << 3;
const uint16_t HEALTH_NAND_OK = 1 << 4;
const uint16_t HEALTH_LOG_OK  = 1 << 5;
const uint16_t HEALTH_BATT_OK = 1 << 6;

enum RocketLaunchStatus : uint8_t {
    LAUNCH_STATUS_INHIBIT = 0,
    LAUNCH_STATUS_WAIT_STILL,
    LAUNCH_STATUS_READY,
    LAUNCH_STATUS_FLIGHT
};

// Telemetry packets (must match rocket firmware)
struct __attribute__((packed)) FlightPacketV7 {
    uint8_t  version;
    uint8_t  state;
    uint16_t flags;

    uint32_t seq;
    uint32_t ms;

    int32_t  alt_cm;
    int16_t  vel_cms;

    int16_t  ax_cms2, ay_cms2, az_cms2;
    int16_t  gx_cdeg, gy_cdeg, gz_cdeg;

    int16_t  roll_cdeg;
    int16_t  pitch_cdeg;
};

struct __attribute__((packed)) NavPacketV7 {
    uint8_t  version;
    uint8_t  gps_fix_type;
    uint8_t  gps_sats;
    uint8_t  gps_hdop_x10;

    uint32_t seq;
    uint32_t ms;

    int32_t  gps_lat_e7;
    int32_t  gps_lon_e7;
    int32_t  gps_alt_cm;
    int32_t  baro_alt_cm;

    uint32_t last_fix_age_ms;
};

struct __attribute__((packed)) StatusPacketV8 {
    uint8_t  version;
    uint8_t  state;
    uint16_t health_flags;

    uint32_t seq;
    uint32_t ms;

    uint16_t batt_mv;
    uint8_t  gps_sats;
    uint8_t  launch_status;
    int16_t  last_rssi_dbm;
    uint16_t launch_wait_s;
};

struct __attribute__((packed)) IdentityPacketV1 {
    uint8_t  version;
    uint8_t  name_len;
    uint16_t reserved0;

    uint32_t seq;
    uint32_t ms;

    char     name[16];
};

struct __attribute__((packed)) PyroConfigPacketV1 {
    uint8_t  version;
    uint8_t  channel_count;
    uint8_t  output_enabled;
    uint8_t  active_high;
    uint32_t seq;
    uint32_t ms;
    uint16_t fire_ms;
    uint16_t apogee_delay_ms;
    uint16_t main_min_after_apogee_ms;
    uint16_t main_alt_m;
    char     flight_profile;
    char     channel_func[4];
    uint8_t  channel_pin[4];
    uint8_t  channel_log_mask;
    uint8_t  channel_output_mask;
};
static_assert(sizeof(PyroConfigPacketV1) == 31, "PyroConfigPacketV1 size mismatch");

struct __attribute__((packed)) PyroEventPacketV1 {
    uint8_t  version;
    uint8_t  event_type;
    uint8_t  channel_index;
    char     function;
    uint32_t seq;
    uint32_t ms;
    uint8_t  state;
    uint16_t flags;
    uint8_t  output_enabled;
};
static_assert(sizeof(PyroEventPacketV1) == 16, "PyroEventPacketV1 size mismatch");
