#include "timekeeper.h"
#include "config.h"
#include "radio.h"
#include "settings.h"
#include <TinyGPSPlus.h>
#include <TimeLib.h>

extern TinyGPSPlus gps;

static GroundTimeSource currentSource = TIME_SRC_NONE;
static bool haveValidTime = false;
static uint32_t lastGpsRtcSetMs = 0;

static time_t teensyRtcProvider() {
#if defined(TEENSYDUINO)
    return Teensy3Clock.get();
#else
    return 0;
#endif
}

static bool saneYear(uint16_t y) {
    return y >= 2024 && y <= 2099;
}

static bool saneTime(time_t t) {
    if (t <= 0) return false;
    return saneYear(year(t));
}

static bool isLeapYear(uint16_t y) {
    return ((y % 4) == 0 && ((y % 100) != 0 || (y % 400) == 0));
}

static uint8_t daysInMonth(uint16_t y, uint8_t m) {
    static const uint8_t days[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
    if (m == 2 && isLeapYear(y)) return 29;
    if (m < 1 || m > 12) return 31;
    return days[m - 1];
}

static bool gpsHasSaneTime() {
    return gps.date.isValid() &&
           gps.time.isValid() &&
           saneYear(gps.date.year()) &&
           gps.date.month() >= 1 &&
           gps.date.month() <= 12 &&
           gps.date.day() >= 1 &&
           gps.date.day() <= 31 &&
           gps.time.hour() <= 23 &&
           gps.time.minute() <= 59 &&
           gps.time.second() <= 59;
}

void timekeeper_init() {
#if defined(TEENSYDUINO)
    setSyncProvider(teensyRtcProvider);
    setSyncInterval(300);
#endif
    haveValidTime = saneTime(now());
    currentSource = haveValidTime ? TIME_SRC_RTC : TIME_SRC_NONE;
}

bool timekeeper_update() {
    if (!gpsHasSaneTime()) {
        if (!haveValidTime && saneTime(now())) {
            haveValidTime = true;
            currentSource = TIME_SRC_RTC;
            return true;
        }
        return false;
    }

    uint32_t nowMs = millis();
    if (currentSource == TIME_SRC_GPS && (nowMs - lastGpsRtcSetMs) < 60000u) {
        return false;
    }

    setTime(gps.time.hour(),
            gps.time.minute(),
            gps.time.second(),
            gps.date.day(),
            gps.date.month(),
            gps.date.year());

#if defined(TEENSYDUINO)
    Teensy3Clock.set(now());
#endif
    groundSettingsMarkRtcUtc(true, true);

    lastGpsRtcSetMs = nowMs;
    bool changed = currentSource != TIME_SRC_GPS || !haveValidTime;
    haveValidTime = true;
    currentSource = TIME_SRC_GPS;
    return changed;
}

bool timekeeper_hasTime() {
    return haveValidTime && saneTime(now());
}

GroundTimeSource timekeeper_source() {
    if (!timekeeper_hasTime()) return TIME_SRC_NONE;
    return currentSource;
}

const char *timekeeper_source_name() {
    switch (timekeeper_source()) {
        case TIME_SRC_RTC: return "RTC";
        case TIME_SRC_GPS: return "GPS";
        default: return "NONE";
    }
}

bool timekeeper_getDateTime(GroundDateTime &dt) {
    if (!timekeeper_hasTime()) return false;
    time_t t = now();
    dt.year = year(t);
    dt.month = month(t);
    dt.day = day(t);
    dt.hour = hour(t);
    dt.minute = minute(t);
    dt.second = second(t);
    return true;
}

bool timekeeper_getCentralDateTime(GroundDateTime &dt) {
    if (!timekeeper_hasTime()) return false;
    time_t t = now();
    // Existing Ground RTCs contain local wall time. Once GPS has synchronized
    // the RTC it contains UTC, so apply the configured offset only then.
    if (currentSource == TIME_SRC_GPS || groundSettingsRtcIsUtc()) {
        t += (int32_t)groundSettingsTimezoneOffsetMin() * 60L;
    }
    dt.year = year(t);
    dt.month = month(t);
    dt.day = day(t);
    dt.hour = hour(t);
    dt.minute = minute(t);
    dt.second = second(t);
    return true;
}

bool timekeeper_setDateTime(uint16_t yearValue, uint8_t monthValue, uint8_t dayValue,
                            uint8_t hourValue, uint8_t minuteValue, uint8_t secondValue,
                            bool utcBasis) {
    if (!saneYear(yearValue) ||
        monthValue < 1 || monthValue > 12 ||
        dayValue < 1 || dayValue > daysInMonth(yearValue, monthValue) ||
        hourValue > 23 || minuteValue > 59 || secondValue > 59) {
        return false;
    }

    setTime(hourValue, minuteValue, secondValue,
            dayValue, monthValue, yearValue);
#if defined(TEENSYDUINO)
    Teensy3Clock.set(now());
#endif
    haveValidTime = true;
    currentSource = TIME_SRC_RTC;
    lastGpsRtcSetMs = 0;
    groundSettingsMarkRtcUtc(utcBasis, true);
    return true;
}
