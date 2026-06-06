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
void sdlog_close();
void sdlog_ensureFile();
void sdlog_onGpsTimeAvailable();
