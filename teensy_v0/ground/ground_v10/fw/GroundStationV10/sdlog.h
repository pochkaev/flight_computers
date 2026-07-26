#pragma once
#include <Arduino.h>
#include <SD.h>

extern bool sdOK;
extern File logFile;
extern uint32_t logLineCount;
extern uint32_t nextLogIndex;
extern uint32_t currentLogIndex;

extern bool sdlog_hasGpsTime;

void sdlog_init();
void sdlog_write(const char *line);
void sdlog_write_now(const char *line);
// Prevent synchronous SD file operations while launch controls are active.
// Rows continue to accumulate in RAM and are flushed after both channels are SAFE.
void sdlog_setLaunchCritical(bool active);
void sdlog_close();
void sdlog_ensureFile();
void sdlog_onGpsTimeAvailable();
bool sdlog_serviceSafe();
void sdlog_printStatus(Stream &out);
void sdlog_list(Stream &out);
bool sdlog_mount();
bool sdlog_info(const char *name, Stream &out);
bool sdlog_read(const char *name, uint32_t offset, uint32_t length, Stream &out);
bool sdlog_eraseLogs(uint32_t &removedCount);
