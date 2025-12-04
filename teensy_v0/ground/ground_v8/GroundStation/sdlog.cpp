#include "sdlog.h"
#include "config.h"
#include <TinyGPSPlus.h>

bool sdOK = false;
bool sdlog_hasGpsTime = false;

File logFile;
uint32_t logLineCount = 0;
uint32_t nextLogIndex = 1;
uint32_t currentLogIndex = 0;

static uint32_t bootTimeMs = 0;
static bool stampedMode = false;

extern TinyGPSPlus gps;

static int extractIndex(const char *name) {
    const char *p = strstr(name, "flight");
    if (!p) return -1;
    p += 6;
    long v = 0;
    while (*p >= '0' && *p <= '9') {
        v = v*10 + (*p - '0');
        ++p;
    }
    return (v > 0) ? (int)v : -1;
}

static void scanExisting() {
    File root = SD.open("/");
    if (!root) return;
    uint32_t maxIdx = 0;
    while (true) {
        File f = root.openNextFile();
        if (!f) break;
        if (!f.isDirectory()) {
            int idx = extractIndex(f.name());
            if (idx > (int)maxIdx) maxIdx = idx;
        }
        f.close();
    }
    root.close();
    nextLogIndex = maxIdx + 1;
}

static void buildFilename(char *out, size_t n) {
    uint32_t idx = nextLogIndex;
    if (stampedMode && gps.date.isValid() && gps.time.isValid()) {
        snprintf(out, n,
                 "%04d%02d%02d_%02d%02d%02d_flight%lu.log",
                 gps.date.year(),
                 gps.date.month(),
                 gps.date.day(),
                 gps.time.hour(),
                 gps.time.minute(),
                 gps.time.second(),
                 (unsigned long)idx);
    } else {
        snprintf(out, n, "flight%lu.log", (unsigned long)idx);
    }
}

void sdlog_ensureFile() {
    if (!sdOK) return;
    if (logFile) return;

    uint32_t now = millis();
    if (!stampedMode) {
        if (!sdlog_hasGpsTime && (now - bootTimeMs) < GPS_WAIT_MS) {
            return;
        }
    }

    if (sdlog_hasGpsTime) stampedMode = true;

    char filename[64];
    buildFilename(filename, sizeof(filename));

    currentLogIndex = nextLogIndex;
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        DBG1(String("SD open failed: ") + filename);
        sdOK = false;
        currentLogIndex = 0;
        return;
    }
    DBG1(String("SD LOG → ") + filename);

    nextLogIndex++;
    logLineCount = 0;

    logFile.println("#type,fields=csv");
    logLineCount++;
    logFile.flush();
}

void sdlog_write(const char *line) {
    if (!sdOK) return;
    if (!logFile) {
        sdlog_ensureFile();
        if (!logFile) return;
    }
    logFile.println(line);
    logFile.flush();
    logLineCount++;
}

void sdlog_onGpsTimeAvailable() {
    sdlog_hasGpsTime = true;
    if (!sdOK) return;
    if (logFile && !stampedMode) {
        logFile.flush();
        logFile.close();
        logFile = File();
        stampedMode = true;
    }
}

void sdlog_close() {
    if (logFile) {
        logFile.flush();
        logFile.close();
        logFile = File();
    }
    currentLogIndex = 0;
}

void sdlog_init() {
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);

    if (!SD.begin(SD_CS_PIN)) {
        DBG1("SD init FAILED");
        sdOK = false;
        return;
    }
    sdOK = true;
    DBG1("SD init OK");
    scanExisting();
    bootTimeMs = millis();
    stampedMode = false;
    sdlog_hasGpsTime = false;
}
