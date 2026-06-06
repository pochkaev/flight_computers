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
- Finder buzzer:
  - `pin 5`
  - small piezo buzzer to `GND`, or transistor/MOSFET driver for a louder buzzer
  - rising ready melody after successful initialization
  - falling warning melody when required hardware init fails
  - staged landed finder beacon after `FS_LANDED`: frequent for 3 minutes, slower until 10 minutes, then slow single long beeps
- Local service button:
  - `pin 4`
  - momentary switch to `GND`, active-low with `INPUT_PULLUP`
  - short press silences the landed finder beep
  - hold for about 2 seconds to close the current log, re-baseline on the pad, prepare for the next flight attempt, and play a confirmation melody
  - reset is refused during active ascent/coast/descent

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
- staged non-blocking MS5607 reads:
  - pressure/altitude target is `50 Hz`
  - temperature compensation refresh is `5 Hz`
  - barometer samples no longer block the main loop with back-to-back conversion delays
- barometer-derived `velZ` uses `BARO_VEL_WINDOW_MS = 120` to avoid amplifying 50 Hz pressure noise
- launch detection:
  - launch uses current acceleration magnitude, `relAlt`, and `velZ`
  - current launch thresholds are `accMagG >= 2 g`, `relAlt > 3 m`, and `velZ > 8 m/s`
  - launch detection is blocked for `LAUNCH_POWERON_INHIBIT_MS = 60000 ms`
  - after the inhibit, launch detection arms only after `LAUNCH_PAD_STILL_ARM_MS = 30000 ms` of continuous stillness
  - movement before launch clears the launch-ready gate
  - the pad-still and acceleration gates prevent carry-to-pad barometric spikes from starting `ASCENT`
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
  - NAND V4 full-state records include magnetometer and approximate yaw at `50 Hz`
  - NAND V4 compact IMU records include accel/gyro/attitude at `200 Hz`
  - NAND V4 compact attitude records include quaternion, derived Euler attitude, and confidence flags at `200 Hz`
  - NAND V4 now also writes barometer, GPS, battery, flight event, and telemetry snapshot stream records
  - NAND export is current-format only; legacy decode paths are intentionally not kept in production firmware
  - yaw is visualization-only and is not used by the state machine
- Step 3 high-rate IMU logging is complete:
  - `IMU_UPDATE_MS = 5`
  - full-state NAND records remain `50 Hz`
  - compact IMU NAND records are `200 Hz`
  - full export with `_imu.csv` has been verified
  - fast export with `export_latest_only=1` and `export_imu=0` has been verified
- NAND log header version `4` with:
  - record count
  - open/close times
  - final state
  - close reason
  - finalized flag
  - firmware version
  - rocket name
  - configured sensor/log/telemetry rates
  - IMU ranges
  - estimator version
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

Serial debug setting:

- flight builds should use `SERIAL_DEBUG_LEVEL = 0`
- bench/service debug builds may use `SERIAL_DEBUG_LEVEL = 1` or `2`
- the current firmware may be left in a debug setting after export troubleshooting; check [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h) before field use

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
- service export also writes detail files like `rocket_nand_0120_op1004_imu.csv`, `_baro.csv`, `_gps.csv`, `_batt.csv`, `_event.csv`, `_telem.csv`, and `_att.csv`
- detail CSV export is optional with `export_imu=0`
- latest-only service export is available with `export_latest_only=1`
- current firmware writes `RV10NLG` V4 typed records:
  - type `1`: `50 Hz` full-state records
  - type `2`: `200 Hz` compact IMU records
  - type `3`: barometer records
  - type `4`: GPS records
  - type `5`: battery records
  - type `6`: flight state/event records
  - type `7`: telemetry snapshot records
  - type `8`: quaternion attitude records
- current firmware exports only the current V4 header/record combination
- legacy NAND decode paths are intentionally not kept in production firmware; unsupported files are reported as `export_skipped`
- NAND rotation is enabled:
  - `NAND_ROTATE_ENABLE = 1`
  - `NAND_MIN_FREE_BYTES = 16 MiB`
  - `NAND_MAX_LOG_FILES = 96`
  - oldest `/fltNNNN.bin` files are removed before opening a new log if free space or file-count limits require it
- hard export failures are reported as `export_failed` and still block erase

Service mode:

- reads `/nand_ops.txt` from SD at boot
- can export NAND logs to SD
- can erase NAND logs only after successful export path
- writes `/nand_ops_result.txt`
- supports quick export options:
  - `export_latest_only=1`
  - `export_imu=0`
- example command file:
  - [nand_ops_example.txt](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/nand_ops_example.txt)

Important:

- NAND service-mode logic exists in firmware
- because storage is now deferred during GPS acquisition, service-mode processing also starts after GPS acquisition completes or times out
- V4 service export has been hardware-verified on the rocket
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
3. Full multi-log detail CSV export is slow by design because it converts multiple binary streams, especially `200 Hz` IMU records, into decimal CSV on Teensy; use `export_latest_only=1` and `export_imu=0` for quick field checks.
4. Current LoRa telemetry is binary and now scheduled correctly, but future field data may still suggest smaller or different packets.

## Recommended next work

1. run a real flight/bench-log review with the current launch thresholds:
   - `LAUNCH_REL_ALT_M = 3 m`
   - `LAUNCH_VEL_MPS = 8 m/s`
   - trend and impossible-velocity guards enabled
2. hardware-debug the rocket GPS separately if it remains weak inside the device
3. update replay tooling to consume V4 full-state plus optional `_imu.csv` exports
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
