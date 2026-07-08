#pragma once

#include <Arduino.h>

struct __attribute__((packed)) FlightPacketV7 {
  uint8_t  version;
  uint8_t  state;
  uint16_t flags;
  uint32_t seq;
  uint32_t ms;
  int32_t  alt_cm;
  int16_t  vel_cms;
  int16_t  ax_cms2;
  int16_t  ay_cms2;
  int16_t  az_cms2;
  int16_t  gx_cdeg;
  int16_t  gy_cdeg;
  int16_t  gz_cdeg;
  int16_t  roll_cdeg;
  int16_t  pitch_cdeg;
};
static_assert(sizeof(FlightPacketV7) == 34, "FlightPacketV7 size mismatch");

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
static_assert(sizeof(NavPacketV7) == 32, "NavPacketV7 size mismatch");

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
static_assert(sizeof(StatusPacketV8) == 20, "StatusPacketV8 size mismatch");

struct __attribute__((packed)) IdentityPacketV1 {
  uint8_t  version;
  uint8_t  name_len;
  uint16_t reserved0;
  uint32_t seq;
  uint32_t ms;
  char     name[16];
};
static_assert(sizeof(IdentityPacketV1) == 28, "IdentityPacketV1 size mismatch");

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

void telemetryNotifyPyroEvent(uint8_t eventType, uint8_t channelIndex, char function);
void telemetryTask();
