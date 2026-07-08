#include "telemetry.h"

#include <LoRa.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "state.h"

extern volatile bool loraTxBusy;
extern bool loraOk;
extern bool haveAlt;
extern bool haveGoodFix;
extern FlightState flightState;
extern uint16_t flightFlags;
extern uint32_t flightSeq;
extern uint32_t navSeq;
extern uint32_t statusSeq;
extern uint32_t identitySeq;
extern uint32_t pyroConfigSeq;
extern uint32_t pyroEventSeq;
extern float filtAlt;
extern float velZ;
extern float rocketBattV;
extern float last_ax;
extern float last_ay;
extern float last_az;
extern float last_gx;
extern float last_gy;
extern float last_gz;
extern float roll;
extern float pitch;
extern uint8_t gpsFixType;
extern uint8_t gpsSats;
extern float gpsHdop;
extern double gpsLatDeg;
extern double gpsLonDeg;
extern float gpsAltM;
extern uint32_t lastFixTimeMs;
extern char rocketName[16];

uint16_t buildHealthFlags();
uint8_t currentLaunchStatus(uint16_t &waitSecondsOut);
void logNandTelemetryBinary(uint32_t nowMs, uint8_t packetType, uint32_t packetSeq);

static const uint8_t PYRO_EVENT_QUEUE_LEN = 8;
static PyroEventPacketV1 pendingPyroEventPkts[PYRO_EVENT_QUEUE_LEN] = {};
static uint8_t pendingPyroEventHead = 0;
static uint8_t pendingPyroEventCount = 0;

static char pyroFunctionForEvent(uint8_t eventType) {
  switch (eventType) {
    case EVT_DUAL_DEPLOY_APOGEE_CHARGE_LOG:
    case EVT_PYRO_CHANNEL1_LOG:
    case EVT_PYRO_CHANNEL1_OUTPUT_ON:
    case EVT_PYRO_CHANNEL1_OUTPUT_OFF:
      return PYRO_CH1_FUNC;
    case EVT_DUAL_DEPLOY_MAIN_CHARGE_LOG:
    case EVT_PYRO_CHANNEL2_LOG:
    case EVT_PYRO_CHANNEL2_OUTPUT_ON:
    case EVT_PYRO_CHANNEL2_OUTPUT_OFF:
      return PYRO_CH2_FUNC;
    case EVT_PYRO_CHANNEL3_LOG:
    case EVT_PYRO_CHANNEL3_OUTPUT_ON:
    case EVT_PYRO_CHANNEL3_OUTPUT_OFF:
      return PYRO_CH3_FUNC;
    case EVT_PYRO_CHANNEL4_LOG:
    case EVT_PYRO_CHANNEL4_OUTPUT_ON:
    case EVT_PYRO_CHANNEL4_OUTPUT_OFF:
      return PYRO_CH4_FUNC;
    case EVT_PYRO_BOOSTER_SEPARATION_LOG: return 'B';
    case EVT_PYRO_SUSTAINER_IGNITION_LOG: return 'I';
    case EVT_PYRO_AIRSTART1_LOG: return '1';
    case EVT_PYRO_AIRSTART2_LOG: return '2';
    default: return 'N';
  }
}

void telemetryNotifyPyroEvent(uint8_t eventType, uint8_t channelIndex, char function) {
  if (pendingPyroEventCount >= PYRO_EVENT_QUEUE_LEN) {
    pendingPyroEventHead = (pendingPyroEventHead + 1) % PYRO_EVENT_QUEUE_LEN;
    pendingPyroEventCount--;
  }
  const uint8_t tail = (pendingPyroEventHead + pendingPyroEventCount) % PYRO_EVENT_QUEUE_LEN;
  PyroEventPacketV1 &pkt = pendingPyroEventPkts[tail];
  pkt = {};
  pkt.version = 1;
  pkt.event_type = eventType;
  pkt.channel_index = channelIndex;
  pkt.function = function ? function : pyroFunctionForEvent(eventType);
  pkt.seq = pyroEventSeq++;
  pkt.ms = millis();
  pkt.state = (uint8_t)flightState;
  pkt.flags = flightFlags;
  pkt.output_enabled = PYRO_OUTPUT_ENABLE != 0 ? 1 : 0;
  pendingPyroEventCount++;
}

static bool sendFlightTelemetry() {
  if (!loraOk || loraTxBusy || !haveAlt) return false;

  FlightPacketV7 pkt = {};
  pkt.version = 7;
  pkt.state = (uint8_t)flightState;
  pkt.flags = flightFlags;
  pkt.seq = flightSeq++;
  pkt.ms = millis();
  pkt.alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
  pkt.vel_cms = (int16_t)lroundf(velZ * 100.0f);
  pkt.ax_cms2 = (int16_t)lroundf(last_ax * 100.0f);
  pkt.ay_cms2 = (int16_t)lroundf(last_ay * 100.0f);
  pkt.az_cms2 = (int16_t)lroundf(last_az * 100.0f);
  pkt.gx_cdeg = (int16_t)lroundf(last_gx * 100.0f);
  pkt.gy_cdeg = (int16_t)lroundf(last_gy * 100.0f);
  pkt.gz_cdeg = (int16_t)lroundf(last_gz * 100.0f);
  pkt.roll_cdeg = (int16_t)lroundf(roll * 5729.57795f);
  pkt.pitch_cdeg = (int16_t)lroundf(pitch * 5729.57795f);

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_FLIGHT_V7, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_FLIGHT_V7);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendNavTelemetry() {
  if (!loraOk || loraTxBusy) return false;

  NavPacketV7 pkt = {};
  pkt.version = 7;
  pkt.gps_fix_type = gpsFixType;
  pkt.gps_sats = gpsSats;
  int hdopX10 = (int)lroundf(gpsHdop * 10.0f);
  if (hdopX10 < 0) hdopX10 = 0;
  if (hdopX10 > 255) hdopX10 = 255;
  pkt.gps_hdop_x10 = (uint8_t)hdopX10;
  pkt.seq = navSeq++;
  pkt.ms = millis();
  pkt.gps_lat_e7 = (int32_t)llround(gpsLatDeg * 1e7);
  pkt.gps_lon_e7 = (int32_t)llround(gpsLonDeg * 1e7);
  pkt.gps_alt_cm = (int32_t)lroundf(gpsAltM * 100.0f);
  pkt.baro_alt_cm = (int32_t)lroundf(filtAlt * 100.0f);
  pkt.last_fix_age_ms = haveGoodFix ? (millis() - lastFixTimeMs) : 0xFFFFFFFFu;

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_NAV_V7, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_NAV_V7);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendStatusTelemetry() {
  if (!loraOk || loraTxBusy) return false;

  StatusPacketV8 pkt = {};
  pkt.version = 8;
  pkt.state = (uint8_t)flightState;
  pkt.health_flags = buildHealthFlags();
  pkt.seq = statusSeq++;
  pkt.ms = millis();
  pkt.batt_mv = (uint16_t)lroundf(rocketBattV * 1000.0f);
  pkt.gps_sats = gpsSats;
  uint16_t launchWaitS = 0;
  pkt.launch_status = currentLaunchStatus(launchWaitS);
  pkt.launch_wait_s = launchWaitS;
  pkt.last_rssi_dbm = 0;

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_STATUS_V8, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_STATUS_V8);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendIdentityTelemetry() {
  if (!loraOk || loraTxBusy) return false;

  IdentityPacketV1 pkt = {};
  pkt.version = 1;
  while (pkt.name_len < sizeof(pkt.name) && rocketName[pkt.name_len] != '\0') {
    pkt.name_len++;
  }
  pkt.seq = identitySeq++;
  pkt.ms = millis();
  strncpy(pkt.name, rocketName, sizeof(pkt.name));

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_IDENTITY_V1, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_IDENTITY_V1);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendPyroConfigTelemetry() {
  if (!loraOk || loraTxBusy) return false;

  PyroConfigPacketV1 pkt = {};
  pkt.version = 1;
  pkt.channel_count = 4;
  pkt.output_enabled = PYRO_OUTPUT_ENABLE != 0 ? 1 : 0;
  pkt.active_high = PYRO_ACTIVE_HIGH != 0 ? 1 : 0;
  pkt.seq = pyroConfigSeq++;
  pkt.ms = millis();
  pkt.fire_ms = PYRO_FIRE_MS > 65535u ? 65535u : (uint16_t)PYRO_FIRE_MS;
  pkt.apogee_delay_ms = PYRO_APOGEE_DELAY_MS > 65535u ? 65535u : (uint16_t)PYRO_APOGEE_DELAY_MS;
  pkt.main_min_after_apogee_ms = PYRO_MAIN_MIN_AFTER_APOGEE_MS > 65535u ? 65535u : (uint16_t)PYRO_MAIN_MIN_AFTER_APOGEE_MS;
  pkt.main_alt_m = DUAL_DEPLOY_MAIN_ALT_M < 0 ? 0 : (uint16_t)lroundf(DUAL_DEPLOY_MAIN_ALT_M);
  pkt.flight_profile = PYRO_FLIGHT_PROFILE;
  pkt.channel_func[0] = PYRO_CH1_FUNC;
  pkt.channel_func[1] = PYRO_CH2_FUNC;
  pkt.channel_func[2] = PYRO_CH3_FUNC;
  pkt.channel_func[3] = PYRO_CH4_FUNC;
  pkt.channel_pin[0] = PYRO_CH1_PIN;
  pkt.channel_pin[1] = PYRO_CH2_PIN;
  pkt.channel_pin[2] = PYRO_CH3_PIN;
  pkt.channel_pin[3] = PYRO_CH4_PIN;
  if (PYRO_CH1_LOG_ENABLE) pkt.channel_log_mask |= 1u << 0;
  if (PYRO_CH2_LOG_ENABLE) pkt.channel_log_mask |= 1u << 1;
  if (PYRO_CH3_LOG_ENABLE) pkt.channel_log_mask |= 1u << 2;
  if (PYRO_CH4_LOG_ENABLE) pkt.channel_log_mask |= 1u << 3;
  if (PYRO_CH1_OUTPUT_ENABLE) pkt.channel_output_mask |= 1u << 0;
  if (PYRO_CH2_OUTPUT_ENABLE) pkt.channel_output_mask |= 1u << 1;
  if (PYRO_CH3_OUTPUT_ENABLE) pkt.channel_output_mask |= 1u << 2;
  if (PYRO_CH4_OUTPUT_ENABLE) pkt.channel_output_mask |= 1u << 3;

  logNandTelemetryBinary(pkt.ms, PKT_TYPE_PYRO_CONFIG_V1, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_PYRO_CONFIG_V1);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

static bool sendPyroEventTelemetry() {
  if (!loraOk || loraTxBusy || pendingPyroEventCount == 0) return false;

  PyroEventPacketV1 pkt = pendingPyroEventPkts[pendingPyroEventHead];
  pendingPyroEventHead = (pendingPyroEventHead + 1) % PYRO_EVENT_QUEUE_LEN;
  pendingPyroEventCount--;
  logNandTelemetryBinary(pkt.ms, PKT_TYPE_PYRO_EVENT_V1, pkt.seq);
  loraTxBusy = true;
  LoRa.beginPacket();
  LoRa.write(PKT_TYPE_PYRO_EVENT_V1);
  LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  return true;
}

void telemetryTask() {
  static uint32_t lastFlightTxMs = 0;
  static uint32_t lastNavTxMs = 0;
  static uint32_t lastStatusTxMs = 0;
  static uint32_t lastIdentityTxMs = 0;
  static uint32_t lastPyroConfigTxMs = 0;
  const uint32_t nowMs = millis();
  const bool recoveryMode = (flightState == FS_LANDED);
  const uint32_t flightTxPeriodMs = recoveryMode ? RECOVERY_FLIGHT_TX_MS : FLIGHT_TX_MS;
  const uint32_t navTxPeriodMs = recoveryMode ? RECOVERY_NAV_TX_MS : NAV_TX_MS;
  const uint32_t statusTxPeriodMs = recoveryMode ? RECOVERY_STATUS_TX_MS : STATUS_TX_MS;

  if (!loraOk || loraTxBusy) return;

  if (pendingPyroEventCount > 0) {
    if (sendPyroEventTelemetry()) return;
  }

  if (lastIdentityTxMs == 0 || (uint32_t)(nowMs - lastIdentityTxMs) >= IDENTITY_TX_MS) {
    if (sendIdentityTelemetry()) lastIdentityTxMs = nowMs;
    return;
  }

  if (lastPyroConfigTxMs == 0 || (uint32_t)(nowMs - lastPyroConfigTxMs) >= PYRO_CONFIG_TX_MS) {
    if (sendPyroConfigTelemetry()) lastPyroConfigTxMs = nowMs;
    return;
  }

  if ((uint32_t)(nowMs - lastStatusTxMs) >= statusTxPeriodMs) {
    if (sendStatusTelemetry()) lastStatusTxMs = nowMs;
    return;
  }

  if ((uint32_t)(nowMs - lastNavTxMs) >= navTxPeriodMs) {
    if (sendNavTelemetry()) lastNavTxMs = nowMs;
    return;
  }

  if ((uint32_t)(nowMs - lastFlightTxMs) >= flightTxPeriodMs) {
    if (sendFlightTelemetry()) lastFlightTxMs = nowMs;
    return;
  }
}
