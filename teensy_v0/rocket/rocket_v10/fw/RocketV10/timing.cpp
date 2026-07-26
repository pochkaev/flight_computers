#include "timing.h"

#include "config.h"

RuntimeTimingStats runtimeTiming = {};

static uint32_t currentLoopStartUs = 0;
static uint32_t previousLoopStartUs = 0;

void timingLoopBegin(uint32_t nowUs) {
  currentLoopStartUs = nowUs;
  if (previousLoopStartUs != 0) {
    const uint32_t gapUs = nowUs - previousLoopStartUs;
    runtimeTiming.loop_gap_last_us = gapUs;
    if (gapUs > runtimeTiming.loop_gap_max_us) {
      runtimeTiming.loop_gap_max_us = gapUs;
    }
    if (gapUs > FLIGHT_KERNEL_DEADLINE_US) {
      runtimeTiming.loop_deadline_misses++;
    }
    if (gapUs > LOOP_MAJOR_STALL_US) {
      runtimeTiming.loop_major_stalls++;
    }
  }
  previousLoopStartUs = nowUs;
  runtimeTiming.loop_count++;
}

void timingLoopEnd(uint32_t nowUs) {
  const uint32_t execUs = nowUs - currentLoopStartUs;
  runtimeTiming.loop_exec_last_us = execUs;
  if (execUs > runtimeTiming.loop_exec_max_us) {
    runtimeTiming.loop_exec_max_us = execUs;
  }
}

void timingRecordStorage(StorageTimingKind kind, uint32_t durationUs) {
  uint32_t *maximum = nullptr;
  switch (kind) {
    case STORAGE_TIMING_TASK: maximum = &runtimeTiming.storage_task_max_us; break;
    case STORAGE_TIMING_NAND_WRITE: maximum = &runtimeTiming.nand_write_max_us; break;
    case STORAGE_TIMING_NAND_FLUSH: maximum = &runtimeTiming.nand_flush_max_us; break;
    case STORAGE_TIMING_NAND_HEADER: maximum = &runtimeTiming.nand_header_max_us; break;
    case STORAGE_TIMING_SD_WRITE: maximum = &runtimeTiming.sd_write_max_us; break;
    case STORAGE_TIMING_SD_FLUSH: maximum = &runtimeTiming.sd_flush_max_us; break;
    default: return;
  }
  if (durationUs > *maximum) *maximum = durationUs;
}

void timingReset() {
  const uint32_t activeLoopStartUs = currentLoopStartUs;
  runtimeTiming = {};
  // TIMING RESET is handled inside the active loop. Preserve its start so the
  // matching timingLoopEnd() cannot report micros()-0 as a false multi-minute
  // execution time.
  currentLoopStartUs = activeLoopStartUs;
  previousLoopStartUs = activeLoopStartUs;
}

void timingPrint(Stream &out) {
  out.println("TIMING ROCKET");
  out.print("LOOP_COUNT "); out.println(runtimeTiming.loop_count);
  out.print("LOOP_GAP_LAST_US "); out.println(runtimeTiming.loop_gap_last_us);
  out.print("LOOP_GAP_MAX_US "); out.println(runtimeTiming.loop_gap_max_us);
  out.print("LOOP_EXEC_LAST_US "); out.println(runtimeTiming.loop_exec_last_us);
  out.print("LOOP_EXEC_MAX_US "); out.println(runtimeTiming.loop_exec_max_us);
  out.print("LOOP_DEADLINE_MISSES "); out.println(runtimeTiming.loop_deadline_misses);
  out.print("LOOP_MAJOR_STALLS "); out.println(runtimeTiming.loop_major_stalls);
  out.print("STORAGE_TASK_MAX_US "); out.println(runtimeTiming.storage_task_max_us);
  out.print("NAND_WRITE_MAX_US "); out.println(runtimeTiming.nand_write_max_us);
  out.print("NAND_FLUSH_MAX_US "); out.println(runtimeTiming.nand_flush_max_us);
  out.print("NAND_HEADER_MAX_US "); out.println(runtimeTiming.nand_header_max_us);
  out.print("SD_WRITE_MAX_US "); out.println(runtimeTiming.sd_write_max_us);
  out.print("SD_FLUSH_MAX_US "); out.println(runtimeTiming.sd_flush_max_us);
}
