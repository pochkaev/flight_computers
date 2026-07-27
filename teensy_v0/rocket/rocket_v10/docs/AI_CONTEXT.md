# Rocket v10 AI Context

Primary files:

- [README.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/README.md)
- [FLIGHT_LOGGING.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/FLIGHT_LOGGING.md)
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
  - while physical SAFE is recognized, hold for about 2 seconds to stop outputs, commit and close the NAND log, acknowledge, and reset the complete Teensy
  - SAFE long-press reboot is available from every flight state; ARM refuses it without touching logs or flight state
- HPR-style pyro/event channels:
  - channel 1: `pin 6`, default function `A` apogee/drogue
  - channel 2: `pin 7`, default function `M` main
  - channel 3: `pin 8`, default function `N` disabled
  - channel 4: `pin 15`, default function `N` disabled
  - supported function letters: `N` disabled, `A` apogee/drogue, `M` main, `B` booster separation, `I` sustainer ignition, `1` airstart 1, `2` airstart 2
  - each channel has a `PYRO_CH*_LOG_ENABLE` toggle for channel-level event records
  - `PYRO_OUTPUT_ENABLE = 0` by default, so matched channel requests are logged but D6/D7/D8/D15 are not pulsed
  - output pins are initialized to the inactive level; use MOSFET/transistor drivers, pulldowns, external arming, and a pyro battery before enabling output pulses

## Current v10 status

Inherited and working from `rocket_v8`:

- LoRa telemetry
- LoRa identity packet for rocket name
- GPS parsing
- MS5607 barometer
- LSM9DS1 IMU
- SD init plus read/write verification
- NAND init plus read/write verification
- SD initialization and SAFE field-service access; runtime SD CSV logging is
  disabled in the current flight build
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
  - D16 is the removable SAFE/ARM microswitch input: `C -> GND`, `NC -> D16`, inserted pin/pressed lever = HIGH/SAFE
  - after every boot, SAFE must be observed before a later SAFE-to-ARM transition can start pad verification
  - launch detection arms only after the 60 s power-on inhibit and 10 s vertical/still/healthy pad-verification gate
  - current launch detection accepts three paths after `READY`:
    - acceleration path: rising baro trend, `relAlt > LAUNCH_REL_ALT_M`, `velZ > LAUNCH_VEL_MPS`, and calibrated nose-axis acceleration `-az >= LAUNCH_AXIAL_ACCEL_G`
    - baro path: rising baro trend, `relAlt > LAUNCH_BARO_REL_ALT_M`, and `velZ > LAUNCH_BARO_VEL_MPS`
    - obvious-flight fallback: `relAlt > LAUNCH_OBVIOUS_REL_ALT_M` and `velZ > LAUNCH_OBVIOUS_VEL_MPS`
  - `2 g` is no longer mandatory if the barometer clearly shows a launch
  - launch detection is blocked for `LAUNCH_POWERON_INHIBIT_MS = 60000 ms`
  - after the inhibit, launch detection arms only after `LAUNCH_PAD_STILL_ARM_MS = 10000 ms` of continuous vertical stillness
  - telemetry distinguishes `BOOT`, `INSERT PIN`, `SAFE`, `PAD SETTLE`, sensor/log/battery faults, `VERTICAL`, `HOLD STILL`, `ARMING`, `READY`, `LAUNCH CHECK`, and `FLIGHT`; countdown seconds are only attached to timed states
  - READY is latched; IMU/barometer evidence starts a 3 s launch candidate instead of clearing READY after motion
  - a sustained `-Z` IMU trigger can confirm launch without immediate barometer evidence
  - a rejected launch candidate returns to READY
  - the physical SAFE input prevents carry-to-pad motion from starting normal launch detection
  - launch is blocked for `LAUNCH_PAD_SETTLE_MS = 2500 ms` after baro baseline
  - launch requires a recent rising-altitude trend:
    - `LAUNCH_TREND_MIN_M = 2.50 m`
    - over `LAUNCH_TREND_MS = 150 ms`
- HPR-inspired recovery/event classification is log-only:
  - sustained baro climb can recover missed `COAST`
  - high altitude plus low vertical speed logs near-apogee behavior
  - sustained fast negative baro velocity can recover missed `DESCENT_BALLISTIC`
  - sustained slower negative descent logs under-drogue-like behavior
  - a qualified post-apogee impact or sustained low-altitude/low-velocity condition starts the finder before final landing
  - landing no longer depends exclusively on continuous gyro stillness
  - simulated dual-deploy and HPR-style pyro channel events are logged only by default; output pulses require `PYRO_OUTPUT_ENABLE = 1`
  - raw baro samples implying more than `BARO_MAX_RAW_VEL_MPS = 120 m/s` are ignored
- GPS is informational only:
  - GPS does not participate in state-machine decisions
- GPS is now used as a secondary altitude reference:
  - GPS altitude is baselined while in `FS_IDLE` / `FS_PAD`
  - `gps_rel_alt` is logged beside baro relative altitude
  - baro-vs-GPS disagreement sets `baro_gps_diverge` / `DIAG_BARO_GPS_DIVERGE`
  - this is diagnostic only and does not trigger flight-state changes
- LSM9DS1 magnetometer is now logged for visualization:
  - the legacy SD CSV schema includes `mx`, `my`, `mz`, and approximate
    tilt-compensated `yaw`, but runtime SD CSV logging is disabled
  - NAND V4 full-state records include magnetometer and approximate yaw at `10 Hz`
  - accelerometer ODR is explicitly `238 Hz`; coherent accel/gyro data-ready
    samples are accepted at up to the `200 Hz` scheduler rate
  - NAND V4 type-10 IMU records include raw accel/gyro, measured `dt_us`,
    quaternion, confidence, saturation, rejection, and sample-gap flags
  - type-11 records capture the checksummed EEPROM calibration active when the
    log opens
  - the separate quaternion/attitude stream is disabled because type-10 IMU
    records already contain the quaternion
  - NAND V4 now also writes barometer, GPS, battery, flight event, and telemetry snapshot stream records
  - NAND export is current-format only; legacy decode paths are intentionally not kept in production firmware
  - yaw is visualization-only and is not used by the state machine
- Step 3 high-rate IMU logging is complete:
  - `IMU_UPDATE_MS = 5`
  - full-state NAND records are `10 Hz`
  - compact IMU NAND records are `200 Hz` normally and `50 Hz` during descent
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
- `FS_LANDED` commits flight RAM and finalizes the onboard log; recovery
  telemetry continues at reduced rates
- LED status refresh now runs in the main loop, so a transient stale-sensor event does not leave the external LED stuck in `ERR_GEN`
- legacy `/rocket_config.txt` identity parsing remains in the source, but
  automatic SD configuration loading is disabled in the current flight build

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
- `FS_SUBSONIC_COAST`
- `FS_NEAR_APOGEE`
- `FS_DESCENT_BALLISTIC`
- `FS_UNDER_DROGUE`
- `FS_DUAL_DEPLOY_APOGEE_LOGGED`
- `FS_DUAL_DEPLOY_MAIN_LOGGED`
- `FS_POST_FLIGHT_GROUND`
- `FS_LANDED`
- `FS_ABORT`

Current hardening added in `v10`:

- launch requires sustained detection for `LAUNCH_CONFIRM_MS`
- coast requires sustained low vertical speed for `COAST_CONFIRM_MS`
- apogee/descent requires sustained negative vertical speed for `APOGEE_CONFIRM_MS`
- landed requires sustained low-speed, low-altitude behavior for `LANDED_CONFIRM_MS`
- recovery classifications require sustained behavior for `RECOVERY_CLASSIFY_CONFIRM_MS`
- post-flight ground recovery requires sustained behavior for `RECOVERY_POST_FLIGHT_CONFIRM_MS`

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
  - type `1`: `10 Hz` full-state records
  - type `2`: legacy compact IMU records
  - type `3`: barometer records
  - type `4`: GPS records
  - type `5`: battery records
  - type `6`: flight state/event records
  - type `7`: telemetry snapshot records
  - type `8`: quaternion attitude records
  - type `9`: legacy wide-gyro IMU records
  - type `10`: current wide-gyro quaternion IMU records
  - type `11`: IMU calibration snapshot records
  - type `12`: fresh raw magnetometer records at up to `25 Hz`
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
- logs are finalized on `FS_LANDED`, `FS_ABORT`, physical-SAFE reset, and
  before storage service export
- post-landing telemetry is also reduced to recovery rates
- telemetry scheduling now sends at most one LoRa packet per loop with priority:
  - `IDENTITY`
  - `STATUS`
  - `NAV`
  - `FLIGHT`
  This prevents 5 Hz flight packets from starving lower-rate status/nav packets.

## Installed development build

`rv10.20260725f` decouples flight-state scheduling from successful pressure
samples, requires recent ongoing IMU samples for IMU-only launch confirmation,
validates sensor/GPS freshness, keeps LoRa CRC disabled and TX timeout recovery,
saturates narrow telemetry fields, and writes wide-gyro high-rate NAND IMU
records. SAFE/ARM, latched READY, and log-only pyro behavior are unchanged;
physical pyro outputs remain disabled.

This revision also adds EEPROM-backed `SHOW` / `SET` / `SAVE` / `DEFAULTS`
commands for allow-listed RF, telemetry cadence, and flight-detection values.
Mutating commands require physical SAFE plus `IDLE`/`PAD`; no serial pyro
enable, fire, pin, or SAFE-bypass command exists.

The flight-critical recorder now keeps PAD/ARM/flight records in a 576 KiB RAM
buffer and writes them to NAND only after LANDED/ABORT or explicit SAFE service.
PAD uses a rolling 64 KiB pre-launch window. Runtime SD logging and automatic SD
configuration loading are disabled. `TIMING`, `STORAGE STATUS`, `NAND LIST`,
`NAND EXPORT SD ...`, `NAND ERASE ALL CONFIRM`, `SD MOUNT`, `SD LIST`, and
`SD ERASE LOGS CONFIRM` are available over USB serial.

After a single touch produced a false ASCENT/COAST transition in the previous
build, the one-sample 3 g launch path was removed. IMU-only confirmation now
requires continuous fresh threshold evidence plus at least 1.0 m/s integrated
axial delta-V. `FLIGHT RESET CONFIRM` is physical-SAFE-only, commits the RAM
window to NAND, and restores PAD for accidental bench transitions.

Bench timing improved from a `523,918 us` worst loop gap with synchronous NAND
to `16,189 us` over a clean 30-second PAD window, with zero major stalls,
filesystem writes, or RAM record drops. This is still a development build, not
a flight-approved pyro controller.

## Current known issues

1. GPS can still be weak inside the device, so it remains diagnostic/secondary and not flight-critical.
2. Rocket battery must be checked before flight; recent bench telemetry showed a critical 1S voltage around `3.59 V`.
3. Full multi-log detail CSV export is slow by design because it converts multiple binary streams, especially `200 Hz` IMU records, into decimal CSV on Teensy; use `export_latest_only=1` and `export_imu=0` for quick field checks.
4. Current LoRa telemetry is binary and now scheduled correctly, but future field data may still suggest smaller or different packets.
5. RAM-buffered flight data is lost if Rocket power is completely removed
   before the post-flight NAND commit.
6. The SD card was not mounted in the final connected-device check; use
   `SD MOUNT` after inserting/reseating it. NAND was healthy.

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
## 2026-07-25 field storage and recorder update

- Installed Rocket firmware: `rv10.20260725h`.
- LoRa flight profile remains SF7, 125 kHz, CR 4/5, 17 dBm, CRC disabled.
- Physical pyro outputs remain disabled.
- Full-state NAND snapshots are 10 Hz; dedicated IMU is 200 Hz during
  ascent/coast and barometer is 50 Hz.
- Flight RAM is 512 KiB primary plus 64 KiB critical reserve. After primary
  saturation, full-state/barometer/battery/event records are retained while
  noncritical streams are dropped and counted.
- SAFE/PAD serial service implements `STORAGE STATUS`, NAND/SD list and info,
  offset/length reads, NAND-to-SD export, and confirmed log erase.
- Binary reads use 1024-byte `RVXF` frames with sequence, offset, and CRC32.
  Use `tools/flight_storage.py`; `--resume` starts at the existing local size.
- A live partial-plus-resume test downloaded NAND log 15 completely with all
  frame and final segment CRC checks passing.
- The clean live 15-second timing window after installation measured
  `LOOP_GAP_MAX_US=4753`, zero deadline misses, and zero major stalls.
- The Rocket SD card was not detected during the final live check (`SD_OK=0`);
  NAND was healthy with roughly 112 MB free.
## 2026-07-26 Rocket LED state specification

- Rocket external LED D3 only; Ground LEDs are not part of this change.
- SAFE and healthy: one 80 ms heartbeat every 2 seconds.
- ARM requested but not READY: 500 ms on/off blink.
- READY, LAUNCH CHECK, and flight: solid on.
- Critical fault: 100 ms on/off blink.
- Touchdown/LANDED follows the finder buzzer cadence.
- Storage service uses two short flashes.
- SD/GPS absence and battery warning are not critical LED faults.
- Critical means IMU/barometer/NAND/logger/LoRa failure or stale sensors,
  critical battery, ABORT, or critical-recorder-reserve exhaustion.
