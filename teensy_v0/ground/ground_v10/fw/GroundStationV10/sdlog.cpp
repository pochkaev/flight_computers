#include "sdlog.h"
#include "config.h"
#include "radio.h"
#include <TinyGPSPlus.h>

bool sdOK = false;
bool sdlog_hasGpsTime = false;

File logFile;
uint32_t logLineCount = 0;
uint32_t nextLogIndex = 1;
uint32_t currentLogIndex = 0;

static uint32_t bootTimeMs = 0;
static bool stampedMode = false;
static bool forceOpenWithoutGpsTime = false;

// Buffered logging to reduce SD flush overhead
static char     logBuf[512];
static uint16_t logBufLen = 0;
static uint32_t lastFlushMs = 0;

extern TinyGPSPlus gps;

static void trimAscii(char *s) {
    if (!s) return;
    char *start = s;
    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') start++;
    if (start != s) memmove(s, start, strlen(start) + 1);
    size_t len = strlen(s);
    while (len > 0) {
        char c = s[len - 1];
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
        s[--len] = '\0';
    }
}

static void applyRocketNameConfig(const char *value) {
    if (!value) return;

    char cleaned[sizeof(rocketName)] = {};
    size_t out = 0;
    for (const char *p = value; *p && out < sizeof(cleaned) - 1; ++p) {
        char c = *p;
        bool allowed =
            (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '_' || c == '-' || c == '.';
        if (allowed) cleaned[out++] = c;
    }

    if (out == 0) return;
    cleaned[out] = '\0';
    strncpy(rocketName, cleaned, sizeof(rocketName) - 1);
    rocketName[sizeof(rocketName) - 1] = '\0';
}

static void loadGroundConfig() {
    File f = SD.open(GROUND_CONFIG_PATH, FILE_READ);
    if (!f) return;

    char line[96];
    uint8_t pos = 0;
    while (f.available()) {
        char c = (char)f.read();
        if (c == '\n' || c == '\r') {
            if (pos > 0) {
                line[pos] = '\0';
                pos = 0;

                char *comment = strchr(line, '#');
                if (comment) *comment = '\0';
                trimAscii(line);
                if (line[0] == '\0') continue;

                char *eq = strchr(line, '=');
                if (!eq) continue;
                *eq = '\0';
                char *key = line;
                char *value = eq + 1;
                trimAscii(key);
                trimAscii(value);

                if (strcmp(key, "rocket_name") == 0 ||
                    strcmp(key, "rocket") == 0 ||
                    strcmp(key, "name") == 0) {
                    applyRocketNameConfig(value);
                }
            }
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        }
    }

    if (pos > 0) {
        line[pos] = '\0';
        char *comment = strchr(line, '#');
        if (comment) *comment = '\0';
        trimAscii(line);
        char *eq = strchr(line, '=');
        if (eq) {
            *eq = '\0';
            char *key = line;
            char *value = eq + 1;
            trimAscii(key);
            trimAscii(value);
            if (strcmp(key, "rocket_name") == 0 ||
                strcmp(key, "rocket") == 0 ||
                strcmp(key, "name") == 0) {
                applyRocketNameConfig(value);
            }
        }
    }

    f.close();
}

static int extractIndexAfter(const char *name, const char *marker) {
    const char *p = strstr(name, marker);
    if (!p) return -1;
    p += strlen(marker);
    long v = 0;
    while (*p >= '0' && *p <= '9') {
        v = v*10 + (*p - '0');
        ++p;
    }
    return (v > 0) ? (int)v : -1;
}

static int extractIndex(const char *name) {
    int idx = extractIndexAfter(name, "ground_log");
    if (idx > 0) return idx;
    idx = extractIndexAfter(name, "_log");
    if (idx > 0) return idx;
    return extractIndexAfter(name, "flight");
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
                 "ground_%04d%02d%02d_%02d%02d%02d_log%04lu.log",
                 gps.date.year(),
                 gps.date.month(),
                 gps.date.day(),
                 gps.time.hour(),
                 gps.time.minute(),
                 gps.time.second(),
                 (unsigned long)idx);
    } else {
        snprintf(out, n, "ground_log%04lu.log", (unsigned long)idx);
    }
}

static void flushBuffer() {
    if (!logFile) return;
    if (logBufLen == 0) return;
    logFile.write((const uint8_t *)logBuf, logBufLen);
    logFile.flush();
    logBufLen = 0;
    lastFlushMs = millis();
}

void sdlog_ensureFile() {
    if (!sdOK) return;
    if (logFile) return;

    uint32_t now = millis();
    if (!stampedMode) {
        if (!forceOpenWithoutGpsTime && !sdlog_hasGpsTime && (now - bootTimeMs) < GPS_WAIT_MS) {
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
    logBufLen = 0;
    lastFlushMs = millis();

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
    size_t len = strlen(line);
    // Ensure there is space; if not, flush first
    if (len + 2 > sizeof(logBuf) - logBufLen) {
        flushBuffer();
        // If a single line is larger than buffer, write directly
        if (len + 2 > sizeof(logBuf)) {
            logFile.println(line);
            logFile.flush();
            logLineCount++;
            lastFlushMs = millis();
            return;
        }
    }
    memcpy(&logBuf[logBufLen], line, len);
    logBufLen += len;
    logBuf[logBufLen++] = '\n';
    logLineCount++;

    uint32_t now = millis();
    // Periodic flush or when buffer is reasonably full
    if ((now - lastFlushMs) > 200 || logBufLen > (sizeof(logBuf) / 2)) {
        flushBuffer();
    }
}

void sdlog_write_now(const char *line) {
    forceOpenWithoutGpsTime = true;
    sdlog_write(line);
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
        flushBuffer();
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
    loadGroundConfig();
    scanExisting();
    bootTimeMs = millis();
    stampedMode = false;
    sdlog_hasGpsTime = false;
}
