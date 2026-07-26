#pragma once

#include <Arduino.h>

struct RocketRuntimeSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t loraSf;
  uint8_t loraCr;
  uint8_t txPowerDbm;
  uint8_t reserved0;
  uint32_t loraBandwidthHz;
  uint32_t flightTxMs;
  uint32_t navTxMs;
  uint32_t statusTxMs;
  float launchAccelG;
  uint32_t launchImuConfirmMs;
  float launchImuDeltaVMps;
  float launchBaroAltM;
  float launchBaroVelMps;
  float apogeeMinAltM;
  float mainAltM;
  uint32_t checksum;
};

extern RocketRuntimeSettings rocketSettings;

void rocketSettingsInit();
void rocketSettingsTask();
void applyRocketRadioSettings();
