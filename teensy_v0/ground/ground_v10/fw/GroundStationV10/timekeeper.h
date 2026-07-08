#pragma once
#include <Arduino.h>

enum GroundTimeSource : uint8_t {
    TIME_SRC_NONE = 0,
    TIME_SRC_RTC = 1,
    TIME_SRC_GPS = 2
};

struct GroundDateTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

void timekeeper_init();
bool timekeeper_update();
bool timekeeper_hasTime();
GroundTimeSource timekeeper_source();
const char *timekeeper_source_name();
bool timekeeper_getDateTime(GroundDateTime &dt);
bool timekeeper_getCentralDateTime(GroundDateTime &dt);
