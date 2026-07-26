#include "sdlog.h"
#include "config.h"
#include "radio.h"
#include "timekeeper.h"
#include "power.h"
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

// Buffered logging to reduce SD flush overhead. ARM alone keeps normal direct
// SD logging active so a long pad wait cannot fill RAM. During the short
// START/output/PWR_FIRE window, all SD operations are deferred so a slow card
// cannot starve the 150 ms RS-485 command heartbeat. The 64 KiB buffer retains
// the complete fire window plus concurrent ground/rocket rows.
static char   logBuf[64UL * 1024UL];
static size_t logBufLen = 0;
static uint32_t lastFlushMs = 0;
static bool launchCritical = false;
static bool gpsRotateDeferred = false;
static uint32_t launchCriticalDroppedLines = 0;

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
    GroundDateTime dt;
    if (stampedMode && timekeeper_getCentralDateTime(dt)) {
        snprintf(out, n,
                 "ground_%04d%02d%02d_%02d%02d%02d_log%04lu.log",
                 dt.year,
                 dt.month,
                 dt.day,
                 dt.hour,
                 dt.minute,
                 dt.second,
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
        if (!forceOpenWithoutGpsTime && !timekeeper_hasTime() && (now - bootTimeMs) < GPS_WAIT_MS) {
            return;
        }
    }

    if (timekeeper_hasTime()) {
        stampedMode = true;
        sdlog_hasGpsTime = true;
    }

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
    lastFlushMs = millis();

    logFile.println("#type,fields=csv");
    logLineCount++;
    logFile.flush();
}

void sdlog_write(const char *line) {
    if (!sdOK) return;
    if (!logFile) {
        if (!launchCritical) sdlog_ensureFile();
        // While launch-critical, retaining the row in RAM is intentional.
        // The file is opened only after the controls return to SAFE.
        if (!logFile && !launchCritical) return;
    }
    size_t len = strlen(line);
    // Ensure there is space; if not, flush first
    if (len + 2 > sizeof(logBuf) - logBufLen) {
        if (launchCritical) {
            launchCriticalDroppedLines++;
            return;
        }
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
    if (!launchCritical &&
        ((now - lastFlushMs) > 200 || logBufLen > (sizeof(logBuf) / 2))) {
        flushBuffer();
    }
}

void sdlog_write_now(const char *line) {
    forceOpenWithoutGpsTime = true;
    sdlog_write(line);
}

void sdlog_setLaunchCritical(bool active) {
    if (active == launchCritical) return;

    launchCritical = active;
    if (launchCritical) return;

    // We are SAFE again. Complete any deferred timestamp-file transition first,
    // then persist the launch rows accumulated in RAM.
    if (gpsRotateDeferred) {
        if (logFile) {
            flushBuffer();
            logFile.flush();
            logFile.close();
            logFile = File();
        }
        stampedMode = timekeeper_hasTime();
        gpsRotateDeferred = false;
    }

    if (logBufLen > 0) {
        sdlog_ensureFile();
        flushBuffer();
    }

    if (launchCriticalDroppedLines > 0) {
        char line[96];
        snprintf(line, sizeof(line),
                 "LOG_WARN,ms=%lu,launch_buffer_dropped=%lu",
                 (unsigned long)millis(),
                 (unsigned long)launchCriticalDroppedLines);
        launchCriticalDroppedLines = 0;
        sdlog_write(line);
    }
}

void sdlog_onGpsTimeAvailable() {
    sdlog_hasGpsTime = true;
    if (!sdOK) return;
    if (launchCritical) {
        gpsRotateDeferred = true;
        return;
    }
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

bool sdlog_serviceSafe() {
    return !launchCritical &&
           !pwr_anyArmed &&
           digitalRead(PWR_ARM_A_PIN) == HIGH &&
           digitalRead(PWR_ARM_B_PIN) == HIGH &&
           digitalRead(PWR_START_A_PIN) == HIGH &&
           digitalRead(PWR_START_B_PIN) == HIGH;
}

void sdlog_printStatus(Stream &out) {
    out.println("STORAGE STATUS");
    out.print("SD_OK "); out.println(sdOK ? 1 : 0);
    out.print("LOG_OPEN "); out.println(logFile ? 1 : 0);
    out.print("CURRENT_LOG_INDEX "); out.println(currentLogIndex);
    out.print("NEXT_LOG_INDEX "); out.println(nextLogIndex);
    out.print("LOG_LINES "); out.println(logLineCount);
    out.print("LOG_BUFFER_BYTES "); out.println(logBufLen);
    out.print("LAUNCH_CRITICAL "); out.println(launchCritical ? 1 : 0);
    out.print("STORAGE_SERVICE_SAFE "); out.println(sdlog_serviceSafe() ? 1 : 0);
    out.println("STORAGE STATUS END");
}

void sdlog_list(Stream &out) {
    out.println("SD LIST BEGIN");
    if (!sdOK) {
        out.println("ERR SD unavailable");
        out.println("SD LIST END COUNT 0");
        return;
    }
    File root = SD.open("/");
    if (!root) {
        out.println("ERR SD root");
        out.println("SD LIST END COUNT 0");
        return;
    }
    uint32_t count = 0;
    while (true) {
        File f = root.openNextFile();
        if (!f) break;
        out.print(f.isDirectory() ? "SD_DIR NAME " : "SD_FILE NAME ");
        out.print(f.name());
        if (!f.isDirectory()) {
            out.print(" BYTES ");
            out.print((uint32_t)f.size());
        }
        out.println();
        count++;
        f.close();
    }
    root.close();
    out.print("SD LIST END COUNT "); out.println(count);
}

static bool safeStorageName(const char *name) {
    if (!name || !*name || strlen(name) > 80) return false;
    if (strstr(name, "..") || strchr(name, '/') || strchr(name, '\\')) return false;
    for (const char *p = name; *p; ++p) {
        const char c = *p;
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')) {
            return false;
        }
    }
    return true;
}

static bool isGroundLogName(const char *name) {
    if (!safeStorageName(name) || strncmp(name, "ground", 6) != 0) return false;
    const size_t length = strlen(name);
    return length >= 4 && strcmp(name + length - 4, ".log") == 0;
}

struct __attribute__((packed)) SerialTransferFrame {
    char magic[4];
    uint8_t version;
    uint8_t backend;
    uint16_t flags;
    uint32_t sequence;
    uint32_t offset;
    uint16_t length;
    uint16_t reserved;
    uint32_t crc32;
};
static_assert(sizeof(SerialTransferFrame) == 24, "SerialTransferFrame size mismatch");

static uint32_t updateCrc32(uint32_t crc, const uint8_t *data, size_t length) {
    while (length-- > 0) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
        }
    }
    return crc;
}

bool sdlog_info(const char *name, Stream &out) {
    if (!sdOK || !safeStorageName(name)) {
        out.println("ERR SD INFO name");
        return false;
    }
    File f = SD.open(name, FILE_READ);
    if (!f || f.isDirectory()) {
        if (f) f.close();
        out.println("ERR SD INFO not found");
        return false;
    }
    out.print("SD INFO NAME "); out.print(name);
    out.print(" BYTES "); out.println((uint32_t)f.size());
    f.close();
    return true;
}

bool sdlog_read(const char *name, uint32_t offset, uint32_t requestedLength,
                Stream &out) {
    if (!sdOK || !safeStorageName(name)) {
        out.println("ERR SD READ name");
        return false;
    }
    sdlog_close();
    File f = SD.open(name, FILE_READ);
    if (!f || f.isDirectory()) {
        if (f) f.close();
        out.println("ERR SD READ not found");
        return false;
    }
    const uint32_t fileSize = (uint32_t)f.size();
    if (offset > fileSize || !f.seek(offset)) {
        f.close();
        out.println("ERR SD READ offset");
        return false;
    }
    uint32_t transferLength = fileSize - offset;
    if (requestedLength != 0 && requestedLength < transferLength) {
        transferLength = requestedLength;
    }
    out.print("XFER BEGIN BACKEND SD NAME "); out.print(name);
    out.print(" SIZE "); out.print(fileSize);
    out.print(" OFFSET "); out.print(offset);
    out.print(" LENGTH "); out.print(transferLength);
    out.println(" CHUNK 1024");

    uint8_t payload[1024];
    uint32_t sent = 0;
    uint32_t sequence = 0;
    uint32_t runningCrc = 0xFFFFFFFFu;
    while (sent < transferLength) {
        if (!sdlog_serviceSafe()) {
            f.close();
            out.println("\nXFER ERROR LOCKED");
            return false;
        }
        const uint16_t wanted =
            (uint16_t)min((uint32_t)sizeof(payload), transferLength - sent);
        const int got = f.read(payload, wanted);
        if (got <= 0) {
            f.close();
            out.println("\nXFER ERROR READ");
            return false;
        }
        const uint16_t payloadLength = (uint16_t)got;
        const uint32_t payloadCrc =
            updateCrc32(0xFFFFFFFFu, payload, payloadLength) ^ 0xFFFFFFFFu;
        runningCrc = updateCrc32(runningCrc, payload, payloadLength);
        SerialTransferFrame frame = {};
        memcpy(frame.magic, "RVXF", 4);
        frame.version = 1;
        frame.backend = 2;
        frame.flags = (sent + payloadLength == transferLength) ? 1u : 0u;
        frame.sequence = sequence++;
        frame.offset = offset + sent;
        frame.length = payloadLength;
        frame.crc32 = payloadCrc;
        out.write((const uint8_t *)&frame, sizeof(frame));
        out.write(payload, payloadLength);
        sent += payloadLength;
    }
    f.close();
    out.print("\nXFER END BYTES "); out.print(sent);
    out.print(" CRC32 ");
    char crcText[9];
    snprintf(crcText, sizeof(crcText), "%08lX",
             (unsigned long)(runningCrc ^ 0xFFFFFFFFu));
    out.println(crcText);
    return true;
}

bool sdlog_eraseLogs(uint32_t &removedCount) {
    removedCount = 0;
    if (!sdOK) return false;
    sdlog_close();
    File root = SD.open("/");
    if (!root) return false;
    bool allOk = true;
    while (true) {
        File f = root.openNextFile();
        if (!f) break;
        if (!f.isDirectory() && isGroundLogName(f.name())) {
            char name[96];
            strncpy(name, f.name(), sizeof(name) - 1);
            name[sizeof(name) - 1] = '\0';
            f.close();
            if (SD.remove(name)) removedCount++;
            else allOk = false;
            continue;
        }
        f.close();
    }
    root.close();
    nextLogIndex = 1;
    scanExisting();
    return allOk;
}

bool sdlog_mount() {
    sdlog_close();
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    sdOK = SD.begin(SD_CS_PIN);
    if (sdOK) scanExisting();
    return sdOK;
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
