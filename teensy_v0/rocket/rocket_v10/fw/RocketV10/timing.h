#pragma once

#include <Arduino.h>

enum StorageTimingKind : uint8_t {
  STORAGE_TIMING_TASK = 0,
  STORAGE_TIMING_NAND_WRITE,
  STORAGE_TIMING_NAND_FLUSH,
  STORAGE_TIMING_NAND_HEADER,
  STORAGE_TIMING_SD_WRITE,
  STORAGE_TIMING_SD_FLUSH
};

struct RuntimeTimingStats {
  uint32_t loop_count;
  uint32_t loop_gap_last_us;
  uint32_t loop_gap_max_us;
  uint32_t loop_exec_last_us;
  uint32_t loop_exec_max_us;
  uint32_t loop_deadline_misses;
  uint32_t loop_major_stalls;
  uint32_t storage_task_max_us;
  uint32_t nand_write_max_us;
  uint32_t nand_flush_max_us;
  uint32_t nand_header_max_us;
  uint32_t sd_write_max_us;
  uint32_t sd_flush_max_us;
};

extern RuntimeTimingStats runtimeTiming;

void timingLoopBegin(uint32_t nowUs);
void timingLoopEnd(uint32_t nowUs);
void timingRecordStorage(StorageTimingKind kind, uint32_t durationUs);
void timingReset();
void timingPrint(Stream &out);

