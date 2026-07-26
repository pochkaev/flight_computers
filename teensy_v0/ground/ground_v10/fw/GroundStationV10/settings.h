#pragma once

#include <Arduino.h>

struct GroundRuntimeSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  int16_t timezoneOffsetMin;
  uint8_t rtcIsUtc;
  uint8_t loraSf;
  uint8_t loraCr;
  uint8_t reserved0;
  uint32_t loraBandwidthHz;
  uint32_t linkLostMs;
  uint32_t checksum;
};

extern GroundRuntimeSettings groundSettings;

void groundSettingsInit();
void groundSettingsTask();
void groundSettingsMarkRtcUtc(bool isUtc, bool persist);
int16_t groundSettingsTimezoneOffsetMin();
bool groundSettingsRtcIsUtc();
uint32_t groundSettingsLinkLostMs();

