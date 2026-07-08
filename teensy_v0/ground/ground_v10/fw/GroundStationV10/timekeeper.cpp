#include "timekeeper.h"
#include "config.h"
#include "radio.h"
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

static uint8_t dayOfWeek0Sunday(uint16_t y, uint8_t m, uint8_t d) {
    // Sakamoto algorithm. Returns 0=Sunday, 1=Monday, ... 6=Saturday.
    static const uint8_t offsets[] = { 0,3,2,5,0,3,5,1,4,6,2,4 };
    if (m < 3) y -= 1;
    return (y + y / 4 - y / 100 + y / 400 + offsets[m - 1] + d) % 7;
}

static uint8_t nthSunday(uint16_t y, uint8_t month, uint8_t n) {
    uint8_t firstDow = dayOfWeek0Sunday(y, month, 1);
    uint8_t day = 1 + ((7 - firstDow) % 7) + (n - 1) * 7;
    uint8_t maxDay = daysInMonth(y, month);
    return day > maxDay ? maxDay : day;
}

static time_t utcBoundary(uint16_t y, uint8_t month, uint8_t day, uint8_t hourUtc) {
    tmElements_t tm;
    tm.Year = CalendarYrToTm(y);
    tm.Month = month;
    tm.Day = day;
    tm.Hour = hourUtc;
    tm.Minute = 0;
    tm.Second = 0;
    return makeTime(tm);
}

static bool centralUsesDst(time_t utc) {
    uint16_t y = year(utc);
    if (!saneYear(y)) return false;
    // US Central Time:
    // DST starts second Sunday in March at 02:00 CST = 08:00 UTC.
    // DST ends first Sunday in November at 02:00 CDT = 07:00 UTC.
    time_t dstStart = utcBoundary(y, 3, nthSunday(y, 3, 2), 8);
    time_t dstEnd = utcBoundary(y, 11, nthSunday(y, 11, 1), 7);
    return utc >= dstStart && utc < dstEnd;
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
    time_t utc = now();
    int32_t offsetSeconds = centralUsesDst(utc) ? -5L * 3600L : -6L * 3600L;
    time_t t = utc + offsetSeconds;
    dt.year = year(t);
    dt.month = month(t);
    dt.day = day(t);
    dt.hour = hour(t);
    dt.minute = minute(t);
    dt.second = second(t);
    return true;
}
