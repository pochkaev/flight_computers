#include <Arduino.h>
#include <LittleFS.h>
#include <SD.h>
#include <SPI.h>

// Standalone RocketV10 storage benchmark.
// This sketch is not part of flight firmware. It writes records with the same
// size and cache shape as the RocketV10 NAND flight logger.

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t RECORD_COUNT = 5000;
static constexpr uint32_t REALTIME_RECORD_COUNT = 1000;
static constexpr uint32_t TARGET_PERIOD_US = 20000;
static constexpr uint8_t CACHE_RECORDS = 32;

struct __attribute__((packed)) BenchRecord {
  uint32_t ms;
  uint16_t health_flags;
  uint16_t flight_flags;
  int32_t alt_cm;
  int32_t rel_alt_cm;
  int16_t vel_cms;
  int16_t temp_centi_c;
  uint32_t pressure_pa_x10;
  int16_t ax_cms2;
  int16_t ay_cms2;
  int16_t az_cms2;
  int16_t gx_cdeg;
  int16_t gy_cdeg;
  int16_t gz_cdeg;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int32_t gps_lat_e7;
  int32_t gps_lon_e7;
  int32_t gps_alt_cm;
  int32_t gps_rel_alt_cm;
  int32_t baro_gps_delta_cm;
  int16_t gps_speed_cms;
  uint16_t batt_mv;
  uint16_t diag_flags;
  int16_t mx_centiuT;
  int16_t my_centiuT;
  int16_t mz_centiuT;
  int16_t yaw_cdeg;
  uint8_t state;
  uint8_t gps_fix_type;
  uint8_t gps_sats;
  uint8_t battery_pack;
};
static_assert(sizeof(BenchRecord) == 78, "BenchRecord must match NandFlightRecordV3 size");

struct BenchResult {
  bool ok = false;
  uint32_t records = 0;
  uint32_t bytes = 0;
  uint32_t total_us = 0;
  uint32_t flush_us = 0;
  uint32_t min_write_us = 0xFFFFFFFFu;
  uint32_t max_write_us = 0;
  uint64_t write_us_sum = 0;
  uint32_t writes = 0;
  uint32_t deadline_misses = 0;
  uint32_t max_lateness_us = 0;
};

LittleFS_QPINAND qspiNand;
BenchRecord cache[CACHE_RECORDS];

static void fillRecord(BenchRecord &rec, uint32_t i) {
  rec = {};
  rec.ms = millis();
  rec.health_flags = 0x7F;
  rec.flight_flags = i & 0x7;
  rec.alt_cm = 22400 + (int32_t)i;
  rec.rel_alt_cm = (int32_t)i;
  rec.vel_cms = (int16_t)(i % 6000);
  rec.temp_centi_c = 2500;
  rec.pressure_pa_x10 = 980000;
  rec.ax_cms2 = 100;
  rec.ay_cms2 = -200;
  rec.az_cms2 = 980;
  rec.gx_cdeg = (int16_t)(i % 32000);
  rec.gy_cdeg = (int16_t)((i * 3) % 32000);
  rec.gz_cdeg = (int16_t)((i * 7) % 32000);
  rec.roll_cdeg = (int16_t)(i % 18000);
  rec.pitch_cdeg = (int16_t)(i % 9000);
  rec.gps_lat_e7 = 423288950;
  rec.gps_lon_e7 = -885225158;
  rec.gps_alt_cm = 26300;
  rec.gps_rel_alt_cm = 0;
  rec.baro_gps_delta_cm = 0;
  rec.gps_speed_cms = 0;
  rec.batt_mv = 3960;
  rec.diag_flags = 0;
  rec.mx_centiuT = 1000;
  rec.my_centiuT = 2000;
  rec.mz_centiuT = -3000;
  rec.yaw_cdeg = (int16_t)(i % 36000);
  rec.state = 2;
  rec.gps_fix_type = 3;
  rec.gps_sats = 10;
  rec.battery_pack = 1;
}

static void recordWriteTiming(BenchResult &r, uint32_t write_us) {
  if (write_us < r.min_write_us) r.min_write_us = write_us;
  if (write_us > r.max_write_us) r.max_write_us = write_us;
  r.write_us_sum += write_us;
  r.writes++;
}

static bool writeChunk(File &file, BenchResult &r, const BenchRecord *records, uint8_t count) {
  const size_t bytes = (size_t)count * sizeof(BenchRecord);
  const uint32_t t0 = micros();
  const size_t written = file.write((const uint8_t *)records, bytes);
  const uint32_t dt = micros() - t0;
  recordWriteTiming(r, dt);
  if (written != bytes) return false;
  r.records += count;
  r.bytes += bytes;
  return true;
}

static File openBenchFile(bool use_nand, const char *path) {
  if (use_nand) {
    qspiNand.remove(path);
    return qspiNand.open(path, FILE_WRITE);
  }
  SD.remove(path);
  return SD.open(path, FILE_WRITE);
}

static BenchResult runBurstWrite(bool use_nand, const char *path, uint32_t records) {
  BenchResult r = {};
  File file = openBenchFile(use_nand, path);
  if (!file) return r;

  uint8_t cache_count = 0;
  const uint32_t t0 = micros();
  for (uint32_t i = 0; i < records; ++i) {
    fillRecord(cache[cache_count++], i);
    if (cache_count >= CACHE_RECORDS) {
      if (!writeChunk(file, r, cache, cache_count)) {
        file.close();
        return r;
      }
      cache_count = 0;
    }
  }
  if (cache_count && !writeChunk(file, r, cache, cache_count)) {
    file.close();
    return r;
  }

  const uint32_t flush0 = micros();
  file.flush();
  r.flush_us = micros() - flush0;
  file.close();
  r.total_us = micros() - t0;
  r.ok = true;
  return r;
}

static BenchResult runRealtimeWrite(bool use_nand, const char *path, uint32_t records) {
  BenchResult r = {};
  File file = openBenchFile(use_nand, path);
  if (!file) return r;

  uint8_t cache_count = 0;
  uint32_t next_due = micros();
  const uint32_t t0 = next_due;
  for (uint32_t i = 0; i < records; ++i) {
    while ((int32_t)(micros() - next_due) < 0) {
      yield();
    }

    const uint32_t now = micros();
    const uint32_t late = now - next_due;
    if (late > 1000) {
      r.deadline_misses++;
      if (late > r.max_lateness_us) r.max_lateness_us = late;
    }

    fillRecord(cache[cache_count++], i);
    if (cache_count >= CACHE_RECORDS) {
      if (!writeChunk(file, r, cache, cache_count)) {
        file.close();
        return r;
      }
      cache_count = 0;
    }
    next_due += TARGET_PERIOD_US;
  }
  if (cache_count && !writeChunk(file, r, cache, cache_count)) {
    file.close();
    return r;
  }

  const uint32_t flush0 = micros();
  file.flush();
  r.flush_us = micros() - flush0;
  file.close();
  r.total_us = micros() - t0;
  r.ok = true;
  return r;
}

static void printResult(const char *label, const char *mode, const BenchResult &r) {
  Serial.print(label);
  Serial.print(' ');
  Serial.print(mode);
  Serial.print(": ok=");
  Serial.print(r.ok ? 1 : 0);
  Serial.print(" records=");
  Serial.print(r.records);
  Serial.print(" bytes=");
  Serial.print(r.bytes);
  Serial.print(" total_ms=");
  Serial.print(r.total_us / 1000.0f, 2);
  Serial.print(" rate_Bps=");
  Serial.print(r.total_us ? (r.bytes * 1000000.0f / r.total_us) : 0.0f, 1);
  Serial.print(" avg_write_us=");
  Serial.print(r.writes ? (float)r.write_us_sum / r.writes : 0.0f, 1);
  Serial.print(" max_write_us=");
  Serial.print(r.max_write_us);
  Serial.print(" flush_us=");
  Serial.print(r.flush_us);
  Serial.print(" deadline_misses=");
  Serial.print(r.deadline_misses);
  Serial.print(" max_late_us=");
  Serial.println(r.max_lateness_us);
}

static void runBenchmarks() {
  Serial.println();
  Serial.println("RocketV10 storage benchmark");
  Serial.print("record_size=");
  Serial.print(sizeof(BenchRecord));
  Serial.print(" cache_records=");
  Serial.print(CACHE_RECORDS);
  Serial.print(" flight_target_Bps=");
  Serial.println(50.0f * sizeof(BenchRecord), 1);

  const bool nand_ok = qspiNand.begin();
  const bool sd_ok = SD.begin(BUILTIN_SDCARD);
  Serial.print("nand_begin=");
  Serial.print(nand_ok ? 1 : 0);
  Serial.print(" sd_begin=");
  Serial.println(sd_ok ? 1 : 0);

  if (nand_ok) {
    printResult("NAND", "burst", runBurstWrite(true, "/bench_nand.bin", RECORD_COUNT));
    printResult("NAND", "50Hz", runRealtimeWrite(true, "/bench_nand_rt.bin", REALTIME_RECORD_COUNT));
  }
  if (sd_ok) {
    printResult("SD", "burst", runBurstWrite(false, "/bench_sd.bin", RECORD_COUNT));
    printResult("SD", "50Hz", runRealtimeWrite(false, "/bench_sd_rt.bin", REALTIME_RECORD_COUNT));
  }

  Serial.println("Benchmark done.");
  Serial.println("For flight logging, rate_Bps must be well above 3900 and 50Hz deadline_misses should be 0.");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t start = millis();
  while (!Serial && (millis() - start) < 6000) {}
  delay(250);
  runBenchmarks();
}

void loop() {
}
