# Rocket v10 AI Context

Primary files:

- [README.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/README.md)
- [NEXT_STEPS.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/NEXT_STEPS.md)
- [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)
- [RocketV10.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/RocketV10.ino)
- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h)

## Hardware

- MCU: `Teensy 4.1`
- LoRa: `RFM95` at `915 MHz`
- Barometer: `MS5607`
- IMU: `LSM9DS1`
- GPS: `GT-U7`
- Storage:
  - built-in microSD via `BUILTIN_SDCARD`
  - QSPI NAND via `LittleFS_QPINAND`

Important connections:

- LoRa:
  - `CS = 10`
  - `RST = 9`
  - `DIO0 = 2`
  - `MOSI/MISO/SCK = 11/12/13`
- GPS:
  - `Serial1`
  - `RX1 = 0` from GPS `TX`
  - `TX1 = 1` to GPS `RX` optional
- I2C:
  - `SDA = 18`
  - `SCL = 19`
- Battery monitor:
  - `A0`
  - `R1 = 100k`
  - `R2 = 47k`
- External status LED:
  - `pin 3`
  - `pin 3 -> 330R -> LED anode`, LED cathode to `GND`

## Current v10 status

Inherited and working from `rocket_v8`:

- LoRa telemetry
- LoRa identity packet for rocket name
- GPS parsing
- MS5607 barometer
- LSM9DS1 IMU
- SD init plus read/write verification
- NAND init plus read/write verification
- SD CSV logging
- NAND binary logging
- SD-driven NAND service mode
- external status LED state patterns

New in `rocket_v10` already implemented:

- battery voltage filtering
- battery pack detection hysteresis
- battery warn/crit hysteresis
- debounced flight-state transitions for:
  - launch
  - coast
  - apogee/descent
  - landed
- sensor freshness tracking for:
  - GPS
  - IMU
  - barometer
- baro-driven launch detection:
  - launch uses `relAlt` and `velZ`
  - IMU acceleration is no longer a launch trigger
  - current launch thresholds are `relAlt > 3 m` and `velZ > 8 m/s`
  - launch is blocked for `LAUNCH_PAD_SETTLE_MS = 2500 ms` after baro baseline
  - launch requires a recent rising-altitude trend:
    - `LAUNCH_TREND_MIN_M = 2.50 m`
    - over `LAUNCH_TREND_MS = 150 ms`
  - raw baro samples implying more than `BARO_MAX_RAW_VEL_MPS = 120 m/s` are ignored
- GPS is informational only:
  - GPS does not participate in state-machine decisions
- GPS is now used as a secondary altitude reference:
  - GPS altitude is baselined while in `FS_IDLE` / `FS_PAD`
  - `gps_rel_alt` is logged beside baro relative altitude
  - baro-vs-GPS disagreement sets `baro_gps_diverge` / `DIAG_BARO_GPS_DIVERGE`
  - this is diagnostic only and does not trigger flight-state changes
- LSM9DS1 magnetometer is now logged for visualization:
  - SD CSV includes `mx`, `my`, `mz`, and approximate tilt-compensated `yaw`
  - NAND records use `NandFlightRecordV3`
  - NAND export is V3-only; legacy V1/V2 decode paths were intentionally removed
  - yaw is visualization-only and is not used by the state machine
- NAND log header `v2` with:
  - record count
  - open/close times
  - final state
  - close reason
  - finalized flag
- `FS_LANDED` now keeps logging alive at a lower rate for recovery tracking
- LED status refresh now runs in the main loop, so a transient stale-sensor event does not leave the external LED stuck in `ERR_GEN`
- rocket identity is SD-configurable from `/rocket_config.txt`
  - `rocket_name=shadow`
  - aliases: `rocket=...` and `name=...`
  - max 15 chars; allowed chars are letters, numbers, `_`, `-`, and `.`
  - sent periodically in LoRa packet `PKT_TYPE_IDENTITY_V1`

## Battery model

Current battery handling in `v10`:

- raw divider reading is kept as `rocketBattRawV`
- filtered value used by runtime is `rocketBattV`
- pack detection uses hysteresis:
  - switch to `2S` above `5.15 V`
  - switch back to `1S` below `4.85 V`
- warn/crit status also use hysteresis to avoid threshold flapping
- current 1S thresholds are tuned for this hardware, not generic LiPo chemistry:
  - warn: `3.85 V`
  - crit: `3.70 V`

Debug output now shows both:

- `battRawV`
- `batt`
- `gpsFresh`
- `imuFresh`
- `baroFresh`
- `logFinal`

Current build setting:

- `SERIAL_DEBUG_LEVEL = 0` for the quiet flight build

## Flight-state model

Current states:

- `FS_IDLE`
- `FS_PAD`
- `FS_ASCENT`
- `FS_COAST`
- `FS_DESCENT`
- `FS_LANDED`
- `FS_ABORT`

Current hardening added in `v10`:

- launch requires sustained detection for `LAUNCH_CONFIRM_MS`
- coast requires sustained low vertical speed for `COAST_CONFIRM_MS`
- apogee/descent requires sustained negative vertical speed for `APOGEE_CONFIRM_MS`
- landed requires sustained low-speed, low-altitude behavior for `LANDED_CONFIRM_MS`

The thresholds are now centralized in [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h).

## Storage model

SD role:

- readable CSV log for quick inspection
- file names like `rocket_flight0041.csv`

NAND role:

- internal binary recorder
- file names like `/rocket_flt0041.bin`
- service export file names like `rocket_nand_0120_op1004.csv`
- current firmware writes and exports only `NandFlightRecordV3`
- the NAND header is still `v2`, but record payloads are V3 only
- NAND rotation is enabled:
  - `NAND_ROTATE_ENABLE = 1`
  - `NAND_MIN_FREE_BYTES = 16 MiB`
  - `NAND_MAX_LOG_FILES = 96`
  - oldest `/fltNNNN.bin` files are removed before opening a new log if free space or file-count limits require it
- April 26, 2026 fix: header rewrite now stores `sizeof(NandFlightRecordV3)`
- April 26, 2026 recovery behavior: export skips old/unsupported/corrupt files and reports them as `export_skipped`
- hard export failures are reported as `export_failed` and still block erase

Service mode:

- reads `/nand_ops.txt` from SD at boot
- can export NAND logs to SD
- can erase NAND logs only after successful export path
- writes `/nand_ops_result.txt`
- example command file:
  - [nand_ops_example.txt](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/nand_ops_example.txt)

Important:

- NAND service-mode logic exists in firmware
- because storage is now deferred during GPS acquisition, service-mode processing also starts after GPS acquisition completes or times out
- it still needs full hardware validation on the rocket
- result file includes `export_count`, `export_skipped`, and `export_failed`
- logs are no longer finalized on `FS_LANDED`
- logs are still finalized on `FS_ABORT` and before service-mode export
- post-landing telemetry is also reduced to recovery rates
- telemetry scheduling now sends at most one LoRa packet per loop with priority:
  - `IDENTITY`
  - `STATUS`
  - `NAV`
  - `FLIGHT`
  This prevents 5 Hz flight packets from starving lower-rate status/nav packets.

## Current known issues

1. GPS can still be weak inside the device, so it remains diagnostic/secondary and not flight-critical.
2. Rocket battery must be checked before flight; recent bench telemetry showed a critical 1S voltage around `3.59 V`.
3. NAND service-mode export should be re-tested on hardware after the V3-only fix.
4. Current LoRa telemetry is binary and now scheduled correctly, but future field data may still suggest smaller or different packets.

## Recommended next work

1. run a real flight/bench-log review with the current launch thresholds:
   - `LAUNCH_REL_ALT_M = 3 m`
   - `LAUNCH_VEL_MPS = 8 m/s`
   - trend and impossible-velocity guards enabled
2. hardware-debug the rocket GPS separately if it remains weak inside the device
3. hardware-test NAND service mode end-to-end with a no-erase export first
4. inspect SD/NAND logs after the next flight for baro/GPS agreement and yaw visualization quality
5. consider telemetry packet changes only after reviewing real field data

## Build and upload

Use the shared guide first:

- [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)

Rocket target:

- board: `Teensy 4.1`
- FQBN: `teensy:avr:teensy41`
- libraries: `/Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-libs`
- last known rocket serial port: `/dev/cu.usbmodem187564601`
- last known rocket Teensy port: `usb:110000`

Compile:

```bash
mkdir -p /tmp/rocketv10-build
arduino-cli compile --fqbn teensy:avr:teensy41 --libraries /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-libs --build-path /tmp/rocketv10-build /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10
```

Upload:

```bash
arduino-cli upload --fqbn teensy:avr:teensy41 --port usb:110000 --input-dir /tmp/rocketv10-build /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10
```

If only the serial port appears, use `--port /dev/cu.usbmodem187564601`. If Arduino CLI only opens Teensy Loader, run `teensy_post_compile` as shown in the shared guide.
