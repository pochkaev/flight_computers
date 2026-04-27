# Rocket v10 Improvement Plan

This file tracks the intended `rocket_v10` work, priority order, and what is already underway.

## Goals

- make the rocket controller more reliable in flight
- keep USB serial fully optional
- preserve telemetry compatibility with the ground station
- strengthen data recovery through SD and NAND

## Priority order

1. battery and state-machine robustness
2. NAND workflow validation and log integrity
3. sensor freshness and fault handling
4. telemetry improvements that simplify the ground station
5. cleanup and flight-tuning documentation

## Phase 1: Robustness

Purpose:

- remove threshold flapping
- reduce false state transitions

Planned items:

- battery filtering
- pack-detection hysteresis
- warn/crit hysteresis
- debounced launch/coast/apogee/landed logic
- keep all thresholds in `config.h`

Status:

- implemented; validate with next real flight logs
- first implementation already added to:
  - [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h)
  - [RocketV10.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/RocketV10.ino)
- launch detection has been retuned for the OpenRocket sims:
  - `LAUNCH_REL_ALT_M = 3 m`
  - `LAUNCH_VEL_MPS = 8 m/s`
  - pad-settle, trend, and impossible-velocity guards added to reduce false starts

Pass criteria:

- battery status does not flap near threshold on the bench
- `1S/2S` detection stays stable near the transition point
- flight state does not jump on single-sample noise

## Phase 2: NAND workflow

Purpose:

- make NAND useful as an actual black-box recorder

Planned items:

- test `/nand_ops.txt` export flow on hardware
- test export plus erase
- add stronger per-flight metadata
- add finalization/valid markers if needed

Pass criteria:

- exported files appear on SD correctly
- erase only happens after verified export
- a full flight can be recovered from NAND alone

Status:

- partially implemented
- `rocket_v10` now writes a `v2` NAND header with metadata and finalization state
- export logic is now V3-only for `RV10NLG`
- old `RV8NLOG`, V1, and V2 decode paths were intentionally removed to reduce firmware complexity
- April 26, 2026 fix: header rewrite now records `sizeof(NandFlightRecordV3)`, and export skips old/unsupported/corrupt files as `export_skipped`
- `export_failed` still blocks erase; `export_skipped` does not block erase when `clean_nand=1`
- NAND log rotation is implemented:
  - keeps at least `NAND_MIN_FREE_BYTES = 16 MiB` free
  - keeps fewer than `NAND_MAX_LOG_FILES = 96` NAND log files
  - removes oldest `/fltNNNN.bin` files before opening a new log
- hardware validation is still pending after this fix

## Phase 3: Sensor freshness

Purpose:

- distinguish init success from live data health

Planned items:

- track last successful update time for:
  - barometer
  - IMU
  - GPS
- mark stale data explicitly
- expose stale/fresh state in health logic and debug output

Pass criteria:

- unplugging or freezing a sensor becomes visible as stale, not silently "OK"

Status:

- implemented in firmware for GPS, IMU, and barometer
- health flags now require both init success and fresh data
- verbose serial debug shows `gpsFresh`, `imuFresh`, and `baroFresh`

## Phase 4: Telemetry

Purpose:

- reduce ambiguity on the ground side

Planned items:

- consider adding explicit `relAlt`
- consider adding more log-status detail
- keep packet sizes compact and compatible
- validate rocket identity packet and `/rocket_config.txt` workflow with ground hardware

Pass criteria:

- ground UI needs less inferred state
- no regression in LoRa stability
- ground display shows the rocket name configured on the rocket SD card

Status:

- implemented for recovery mode timing
- `FS_LANDED` now uses lower-rate telemetry and lower-rate logging
- implemented for GPS secondary altitude logging:
  - SD CSV logs now include `gps_rel_alt`, `gps_base_alt`, `baro_gps_delta`, `baro_gps_diverge`, and `diag_flags`
  - new NAND records include GPS relative altitude and baro/GPS delta
  - old NAND record export is no longer supported
- implemented for visualization yaw logging:
  - SD CSV logs now include `mx`, `my`, `mz`, and `yaw`
  - new NAND records use `NandFlightRecordV3`
  - yaw is not flight-control data
- fixed LoRa telemetry starvation:
  - the scheduler now prioritizes `IDENTITY`, then `STATUS`, then `NAV`, then `FLIGHT`
  - transmit timestamps advance only after a packet is queued
- added rocket identity telemetry:
  - rocket reads `/rocket_config.txt` at boot when SD is available
  - `PKT_TYPE_IDENTITY_V1` sends the configured name over LoRa
  - ground can display the rocket-sent name instead of a local fallback

## Phase 5: Documentation and tune-up

Planned items:

- document final thresholds and reasoning
- document field build settings
- document NAND service workflow after hardware validation

Status:

- README now documents current launch thresholds, GPS secondary-altitude logging, yaw/magnetometer logging, LoRa scheduler behavior, and current battery-divider values
- shared build/upload guide added at [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)
- ground README documents current display screens and field meanings

## Immediate next actions

1. charge or replace the rocket battery before any flight
2. perform the next test flight with the current quiet field build
3. inspect rocket SD CSV and ground SD logs together
4. test `/nand_ops.txt` export on hardware with `clean_nand=0`
5. inspect exported CSV from a new `RV10NLG` NAND file
6. only after export is verified, test erase with `clean_nand=1`
7. tune launch/recovery thresholds only from real log evidence
