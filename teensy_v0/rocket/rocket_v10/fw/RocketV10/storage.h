#pragma once

#include <Arduino.h>
#include <SD.h>

#include "state.h"

static const uint8_t NAND_RECORD_FULL_STATE_V4 = 1;
static const uint8_t NAND_RECORD_IMU_V4 = 2;
static const uint8_t NAND_RECORD_BARO_V4 = 3;
static const uint8_t NAND_RECORD_GPS_V4 = 4;
static const uint8_t NAND_RECORD_BATT_V4 = 5;
static const uint8_t NAND_RECORD_EVENT_V4 = 6;
static const uint8_t NAND_RECORD_TELEM_V4 = 7;
static const uint8_t NAND_RECORD_ATTITUDE_V4 = 8;
static const uint8_t NAND_RECORD_IMU_WIDE_V4 = 9;
static const uint8_t NAND_RECORD_IMU_QUAT_V4 = 10;
static const uint8_t NAND_RECORD_IMU_CAL_V4 = 11;
static const uint8_t NAND_RECORD_MAG_V4 = 12;
static const uint8_t NAND_RECORD_IMU_ALIGNMENT_V4 = 13;

struct __attribute__((packed)) NandLogHeaderV3 {
  char     magic[8];
  uint16_t version;
  uint16_t header_size;
  uint16_t record_size;
  uint16_t flags;
  uint32_t flight_index;
  uint32_t boot_ms;
  uint32_t open_ms;
  uint32_t close_ms;
  uint32_t record_count;
  uint8_t  final_state;
  uint8_t  close_reason;
  uint16_t reserved0;
  uint32_t reserved1;
  char     firmware_version[16];
  char     rocket_name[16];
  uint16_t imu_hz;
  uint16_t baro_hz;
  uint16_t nand_log_hz;
  uint16_t sd_log_hz;
  uint16_t flight_tx_hz_x10;
  uint16_t nav_tx_hz_x10;
  uint16_t status_tx_hz_x10;
  uint16_t identity_tx_hz_x10;
  uint16_t recovery_flight_tx_hz_x10;
  uint16_t recovery_nav_tx_hz_x10;
  uint16_t recovery_status_tx_hz_x10;
  uint16_t recovery_nand_log_hz_x10;
  uint16_t recovery_sd_log_hz_x10;
  uint16_t gyro_range_dps;
  uint8_t  accel_range_g;
  uint8_t  mag_range_gauss;
  uint8_t  estimator_version;
  uint8_t  record_format;
  uint8_t  reserved2[20];
};
static_assert(sizeof(NandLogHeaderV3) == 128, "NandLogHeaderV3 size mismatch");

struct NandLogMetadata {
  uint16_t header_version = 0;
  uint16_t header_size = 0;
  uint16_t record_size = 0;
  uint16_t flags = 0;
  uint32_t flight_index = 0;
  uint32_t boot_ms = 0;
  uint32_t open_ms = 0;
  uint32_t close_ms = 0;
  uint32_t record_count = 0;
  uint8_t final_state = 0;
  uint8_t close_reason = 0;
  char firmware_version[16] = "";
  char rocket_name[16] = "";
  uint16_t imu_hz = 0;
  uint16_t baro_hz = 0;
  uint16_t nand_log_hz = 0;
  uint16_t sd_log_hz = 0;
  uint16_t flight_tx_hz_x10 = 0;
  uint16_t nav_tx_hz_x10 = 0;
  uint16_t status_tx_hz_x10 = 0;
  uint16_t identity_tx_hz_x10 = 0;
  uint16_t recovery_flight_tx_hz_x10 = 0;
  uint16_t recovery_nav_tx_hz_x10 = 0;
  uint16_t recovery_status_tx_hz_x10 = 0;
  uint16_t recovery_nand_log_hz_x10 = 0;
  uint16_t recovery_sd_log_hz_x10 = 0;
  uint16_t gyro_range_dps = 0;
  uint8_t accel_range_g = 0;
  uint8_t mag_range_gauss = 0;
  uint8_t estimator_version = 0;
  uint8_t record_format = 0;
};

struct __attribute__((packed)) NandFlightRecordV3 {
  uint32_t ms;
  uint16_t health_flags;
  uint16_t flight_flags;
  int32_t  alt_cm;
  int32_t  rel_alt_cm;
  int16_t  vel_cms;
  int16_t  temp_centi_c;
  uint32_t pressure_pa_x10;
  int16_t  ax_cms2;
  int16_t  ay_cms2;
  int16_t  az_cms2;
  int16_t  gx_cdeg;
  int16_t  gy_cdeg;
  int16_t  gz_cdeg;
  int16_t  roll_cdeg;
  int16_t  pitch_cdeg;
  int32_t  gps_lat_e7;
  int32_t  gps_lon_e7;
  int32_t  gps_alt_cm;
  int32_t  gps_rel_alt_cm;
  int32_t  baro_gps_delta_cm;
  int16_t  gps_speed_cms;
  uint16_t batt_mv;
  uint16_t diag_flags;
  int16_t  mx_centiuT;
  int16_t  my_centiuT;
  int16_t  mz_centiuT;
  int16_t  yaw_cdeg;
  uint8_t  state;
  uint8_t  gps_fix_type;
  uint8_t  gps_sats;
  uint8_t  battery_pack;
};
static_assert(sizeof(NandFlightRecordV3) == 78, "NandFlightRecordV3 size mismatch");

struct __attribute__((packed)) NandFullStateRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  NandFlightRecordV3 data;
};
static_assert(sizeof(NandFullStateRecordV4) == 82, "NandFullStateRecordV4 size mismatch");

struct __attribute__((packed)) NandImuRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int16_t ax_cms2;
  int16_t ay_cms2;
  int16_t az_cms2;
  int16_t gx_cdeg;
  int16_t gy_cdeg;
  int16_t gz_cdeg;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandImuRecordV4) == 28, "NandImuRecordV4 size mismatch");

// Wide-gyro IMU record. The original centidegree int16 fields overflow above
// +/-327.67 dps while the configured LSM9DS1 range is +/-2000 dps.
struct __attribute__((packed)) NandImuWideRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int16_t ax_cms2;
  int16_t ay_cms2;
  int16_t az_cms2;
  int32_t gx_mdeg;
  int32_t gy_mdeg;
  int32_t gz_mdeg;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandImuWideRecordV4) == 34, "NandImuWideRecordV4 size mismatch");

// Calibrated quaternion + raw wide-range IMU record. This replaces the
// Euler-only wide record for new logs while preserving the old record decoder.
// dt_us stores the measured accepted-sample interval, not scheduler intent.
struct __attribute__((packed)) NandImuQuatRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint16_t dt_us;
  int16_t ax_cms2;
  int16_t ay_cms2;
  int16_t az_cms2;
  int32_t gx_mdeg;
  int32_t gy_mdeg;
  int32_t gz_mdeg;
  int16_t qw_i16;
  int16_t qx_i16;
  int16_t qy_i16;
  int16_t qz_i16;
  uint16_t quality_flags;
  uint8_t confidence;
  uint8_t state;
};
static_assert(sizeof(NandImuQuatRecordV4) == 40, "NandImuQuatRecordV4 size mismatch");

struct __attribute__((packed)) NandImuCalibrationRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint16_t calibration_version;
  uint16_t valid_flags;
  int32_t gyro_bias_mdps[3];
  int16_t accel_bias_milli_mps2[3];
  int32_t accel_scale_ppm[3];
  int16_t mag_bias_centiuT[3];
  int32_t mag_scale_ppm[3];
  uint32_t calibration_checksum;
};
static_assert(sizeof(NandImuCalibrationRecordV4) == 64,
              "NandImuCalibrationRecordV4 size mismatch");

struct __attribute__((packed)) NandImuAlignmentRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint16_t alignment_version;
  uint16_t valid;
  float sensor_to_airframe[4];
  uint32_t alignment_checksum;
};
static_assert(sizeof(NandImuAlignmentRecordV4) == 32,
              "NandImuAlignmentRecordV4 size mismatch");

struct __attribute__((packed)) NandMagRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint16_t dt_us;
  int16_t mx_centiuT;
  int16_t my_centiuT;
  int16_t mz_centiuT;
  uint16_t quality_flags;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandMagRecordV4) == 20, "NandMagRecordV4 size mismatch");

struct __attribute__((packed)) NandBaroRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int32_t alt_cm;
  int32_t rel_alt_cm;
  int16_t vel_cms;
  int16_t temp_centi_c;
  uint32_t pressure_pa_x10;
  uint16_t diag_flags;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandBaroRecordV4) == 28, "NandBaroRecordV4 size mismatch");

struct __attribute__((packed)) NandGpsRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_cm;
  int32_t rel_alt_cm;
  int32_t baro_gps_delta_cm;
  int16_t speed_cms;
  uint16_t fix_age_ms_x10;
  uint16_t chars_delta;
  uint16_t pass_delta;
  uint16_t fail_delta;
  uint8_t fix_type;
  uint8_t sats;
  uint8_t hdop_x10;
  uint8_t flags;
};
static_assert(sizeof(NandGpsRecordV4) == 42, "NandGpsRecordV4 size mismatch");

struct __attribute__((packed)) NandBatteryRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint16_t batt_mv;
  uint16_t raw_mv;
  uint16_t pin_mv;
  uint8_t pack;
  uint8_t status_flags;
};
static_assert(sizeof(NandBatteryRecordV4) == 16, "NandBatteryRecordV4 size mismatch");

struct __attribute__((packed)) NandEventRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint8_t event_type;
  uint8_t from_state;
  uint8_t to_state;
  uint8_t close_reason;
  uint16_t flight_flags;
  uint16_t diag_flags;
  int32_t rel_alt_cm;
  int16_t vel_cms;
  uint16_t health_flags;
};
static_assert(sizeof(NandEventRecordV4) == 24, "NandEventRecordV4 size mismatch");

struct __attribute__((packed)) NandTelemetryRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  uint8_t packet_type;
  uint8_t state;
  uint16_t health_flags;
  uint32_t packet_seq;
  uint32_t flight_seq;
  uint32_t nav_seq;
  uint32_t status_seq;
  uint32_t identity_seq;
  int16_t last_rssi_dbm;
  uint16_t batt_mv;
};
static_assert(sizeof(NandTelemetryRecordV4) == 36, "NandTelemetryRecordV4 size mismatch");

struct __attribute__((packed)) NandAttitudeRecordV4 {
  uint8_t type;
  uint8_t size;
  uint16_t sequence;
  uint32_t ms;
  int16_t qw_i16;
  int16_t qx_i16;
  int16_t qy_i16;
  int16_t qz_i16;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
  uint16_t diag_flags;
  uint8_t state;
  uint8_t flags;
};
static_assert(sizeof(NandAttitudeRecordV4) == 26, "NandAttitudeRecordV4 size mismatch");

struct ServiceRequest {
  bool valid = false;
  uint32_t operationId = 0;
  bool exportToSd = false;
  bool eraseNandAfterExport = false;
  bool exportLatestOnly = false;
  bool exportImu = true;
  bool requireNandOk = true;
  bool requireSdOk = true;
};

enum NandLogFlags : uint16_t {
  NAND_LOG_FLAG_FINALIZED = 1u << 0
};

enum NandExportResult : uint8_t {
  NAND_EXPORT_EXPORTED = 0,
  NAND_EXPORT_SKIPPED = 1,
  NAND_EXPORT_FAILED = 2
};

extern bool sdOk;
extern bool nandOk;
extern bool logOk;
extern bool sdLogOk;
extern bool nandLogOk;
extern bool logsFinalized;
extern uint32_t nandEventDroppedCount;
extern uint32_t nandFlightRamBytes;
extern uint32_t nandFlightRamDroppedRecords;
extern uint32_t nandFlightCriticalRamBytes;
extern uint32_t nandFlightCriticalDroppedRecords;
extern uint32_t nandFlightNonCriticalDroppedRecords;

int extractPrefixedIndex(const char *name, const char *prefix);
void scanSdLogFiles();
bool verifySdFilesystem();
void trimAscii(char *s);
void copyFixedString(char *dst, size_t dstSize, const char *src);
void loadRocketConfig();
ServiceRequest loadServiceRequest();
void writeServiceResult(const ServiceRequest &req, bool exportDone, uint32_t exportCount,
                        uint32_t exportSkipped, uint32_t exportFailed,
                        bool eraseDone, uint32_t eraseCount, const char *statusText);
void metadataFromHeaderV3(const NandLogHeaderV3 &header, NandLogMetadata &meta);
void writeNandExportMetadata(File &dst, const NandLogMetadata &meta);
void setupStorage();
void processServiceModeIfRequested();
void storageTask();
void storagePrintStatus(Stream &out);
void storageListNand(Stream &out);
void storageListSd(Stream &out);
bool storageInfoNand(uint32_t index, Stream &out);
bool storageInfoSd(const char *name, Stream &out);
bool storageReadNand(uint32_t index, uint32_t offset, uint32_t length, Stream &out);
bool storageReadSd(const char *name, uint32_t offset, uint32_t length, Stream &out);
bool storageMountSd();
bool storageExportNandToSd(int32_t index, bool includeDetail,
                           uint32_t &exportedCount, uint32_t &skippedCount,
                           uint32_t &failedCount);
bool storageEraseLogs(bool eraseNand, bool eraseSd,
                      uint32_t &nandRemoved, uint32_t &sdRemoved);
void finalizeLogFiles(NandCloseReason closeReason);
void logNandImuBinary(uint32_t nowMs);
void logNandMagBinary(uint32_t nowMs);
void logNandBaroBinary(uint32_t nowMs);
void logNandGpsBinary(uint32_t nowMs);
void logNandBatteryBinary(uint32_t nowMs);
void logNandEventBinary(uint32_t nowMs, uint8_t eventType, FlightState fromState,
                        FlightState toState, NandCloseReason closeReason);
void logNandTelemetryBinary(uint32_t nowMs, uint8_t packetType, uint32_t packetSeq);
void logNandAttitudeBinary(uint32_t nowMs);
