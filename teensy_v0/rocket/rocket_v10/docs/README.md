# Rocket v10 Firmware README

Firmware:

- [RocketV10.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/RocketV10.ino)
- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h)
- [FLIGHT_LOGGING.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/FLIGHT_LOGGING.md)
- [PYRO.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/PYRO.md)
- [AI_CONTEXT.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/AI_CONTEXT.md)
- [NEXT_STEPS.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/NEXT_STEPS.md)
- [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)

## Summary

This firmware runs on the rocket flight computer and:

- reads the MS5607 barometer over I2C
- reads the optional LSM9DS1 IMU over I2C
- reads the GT-U7 GPS on `Serial1`
- measures battery voltage on `A0`
- records flight data to timing-safe RAM and commits it to QSPI NAND through
  `LittleFS_QPINAND`
- uses the Teensy 4.1 built-in SD card for optional field export/service
- sends flight, navigation, status, and identity telemetry over LoRa

## Current v10 improvements

Compared with `rocket_v8`, `rocket_v10` currently adds:

- filtered battery voltage for runtime decisions
- hysteresis for `1S/2S` detection
- hysteresis for battery warn/crit thresholds
- debounced flight-state transitions for launch, coast, apogee, and landed
- freshness tracking for GPS, IMU, and barometer
- staged non-blocking MS5607 barometer conversions:
  - pressure/altitude target rate is `50 Hz`
  - temperature compensation refreshes at `5 Hz`
  - the main loop no longer waits through full pressure and temperature conversions in one barometer sample
- barometer vertical velocity is calculated over a short `120 ms` window so the faster pressure stream does not amplify single-sample noise
- launch detection is armed only after pad-ready checks, then accepts several flight signatures:
  - blocked for `LAUNCH_POWERON_INHIBIT_MS = 60000 ms` after power-up
  - then requires `LAUNCH_PAD_STILL_ARM_MS = 30000 ms` of continuous stillness before launch detection is armed
  - movement before launch clears the launch-ready gate after `LAUNCH_ARM_MOTION_GRACE_MS`, so carrying the rocket after it is ready does not remain armed forever
  - acceleration path: rising baro trend, `relAlt > LAUNCH_REL_ALT_M`, `velZ > LAUNCH_VEL_MPS`, and calibrated nose-axis acceleration `-az >= LAUNCH_AXIAL_ACCEL_G`
  - baro path: rising baro trend, `relAlt > LAUNCH_BARO_REL_ALT_M`, and `velZ > LAUNCH_BARO_VEL_MPS`
  - obvious-flight path: `relAlt > LAUNCH_OBVIOUS_REL_ALT_M` and `velZ > LAUNCH_OBVIOUS_VEL_MPS`
  - held for `LAUNCH_CONFIRM_MS`
  - blocked until the barometer has settled for `LAUNCH_PAD_SETTLE_MS`
  - requires recent rising-altitude trend over `LAUNCH_TREND_MS`
  - ignores impossible one-sample barometer velocity spikes
- HPR-inspired recovery/event classification is log-only:
  - recovers missed `COAST` from sustained baro climb
  - records near-apogee behavior from high altitude and low vertical speed
  - recovers missed `DESCENT_BALLISTIC` from sustained fast negative baro velocity
  - records under-drogue-like descent from sustained slower negative baro velocity
  - recovers post-flight ground state from low altitude, low velocity, and stillness
  - logs simulated HPR-style pyro events for configurable channel functions; output pins stay inactive unless `PYRO_OUTPUT_ENABLE=1`
- GPS is telemetry/informational only:
  - GPS is not used for launch, coast, apogee, descent, or landed decisions
- GPS is also logged as a secondary altitude reference:
  - GPS altitude is baselined while sitting on the pad
  - `gps_rel_alt` is logged beside baro `rel_alt_m`
  - `baro_gps_delta` and `baro_gps_diverge` are diagnostic only
- magnetometer data and approximate yaw are logged for post-flight visualization only:
  - `mx`, `my`, `mz`
  - tilt-compensated `yaw`
  - yaw is not used for flight-state decisions
- high-rate onboard IMU logging:
  - Step 3 high-rate IMU logging is complete
  - IMU is sampled at `200 Hz`
  - NAND stores compact IMU records at `200 Hz`
  - LoRa telemetry rate is unchanged
  - V4 service export supports both full `_imu.csv` export and quick latest-only full-state export
- versioned NAND metadata headers with record counts, finalization state, firmware version, rocket name, configured rates, IMU ranges, and estimator version
- current-only NAND export; old/legacy NAND formats are intentionally not decoded by the firmware
- finalizes the onboard flight log immediately on `LANDED` or `ABORT`
- corrected LED-mode refresh in the main loop so transient stale sensor windows do not leave the status LED stuck in error mode
- LoRa telemetry scheduler sends at most one packet per loop, with priority:
  - identity
  - status
  - nav
  - flight
  This prevents the high-rate flight packet stream from starving status/nav telemetry.
- legacy SD configuration support exists, but runtime SD configuration loading
  is disabled in the current flight build
- flight builds should use `SERIAL_DEBUG_LEVEL = 0` so USB serial output is quiet during flight use

These changes are meant to reduce false state changes and battery-status flapping on the bench and in flight.

## Runtime model

The rocket firmware is now structured so USB serial is optional and not part of the flight-critical path.

- GPS parsing runs continuously in the main loop
- IMU sampling runs at `200 Hz` via `IMU_UPDATE_MS = 5`
- barometer/state update targets `50 Hz` pressure samples via `BARO_UPDATE_MS = 20`
- MS5607 temperature compensation refreshes at `5 Hz` via `BARO_TEMP_UPDATE_MS = 200`
- battery update runs at `10 Hz` via `BATT_UPDATE_MS = 100`
- flight telemetry runs at `5 Hz`
- navigation telemetry runs at `1 Hz`
- status telemetry runs at `0.5 Hz`
- identity telemetry runs every `5 s`
- runtime SD CSV logging is disabled
- NAND binary records are buffered in RAM once armed: `10 Hz` full-state,
  `50 Hz` barometer, `200 Hz` compact IMU normally, and `50 Hz` IMU during
  descent states
- flight RAM is committed and the NAND file finalized at `LANDED`, `ABORT`, or
  explicit physical-SAFE service
- serial debug runs only when `SERIAL_DEBUG_LEVEL > 0`

See [FLIGHT_LOGGING.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/FLIGHT_LOGGING.md)
for the exact per-state record flow, buffer behavior, finalization conditions,
SD policy, EEPROM contents, and serial commands.

This is intended for flight use:

- `SERIAL_DEBUG_LEVEL = 0` for flight
- `SERIAL_DEBUG_LEVEL = 1` for quick bench checks
- `SERIAL_DEBUG_LEVEL = 2` for full bench debugging

Serial output is diagnostic only. LoRa telemetry, sensor updates, and storage logging do not depend on a USB serial connection.

For SAFE/PAD field service, use `tools/flight_storage.py`. It lists NAND/SD
files and downloads raw files in checksummed 1024-byte frames. Re-run a
download with `--resume` to continue from the existing local file size. Rocket
rejects these commands unless the physical SAFE switch is present and the
state is IDLE/PAD; it also aborts an active transfer if SAFE is removed.

## Status LED

Dedicated external status LED pin:

- `STATUS_LED_PIN = 3`

Recommended wiring:

```text
Teensy pin 3 -> 330R resistor -> LED anode
LED cathode -> GND
```

Do not use the built-in LED on `pin 13` for this status flow because `D13` is already used as SPI `SCK` for LoRa.

LED patterns:

- short `80 ms` pulse every `2 s`: physical SAFE and flight-critical hardware healthy
- slow `500 ms` on/off blink: ARM requested but pad verification is not READY
- solid on: READY, LAUNCH CHECK, or active flight
- very fast `100 ms` on/off blink: critical fault, do not launch
- finder pattern synchronized with the buzzer: touchdown/LANDED
- 2 short blinks repeating: storage/service operation
- slow blink during initial boot

Critical LED faults are IMU/barometer/NAND/logger/LoRa failure, stale IMU or
barometer data, critical battery, ABORT, or exhaustion of the critical recorder
reserve. Missing SD, missing GPS, battery warning, normal packet loss, and a
short rejected LAUNCH CHECK do not create the critical LED pattern.

## Buzzer and service button

Dedicated rocket finder/service pins:

- `BUZZER_PIN = 5`
- `BUTTON_PIN = 4`

Recommended buzzer wiring for a small piezo buzzer:

```text
Teensy pin 5 -> buzzer +
buzzer - -> GND
```

Use a small active or passive piezo buzzer only if its current draw is safe for a Teensy GPIO pin. For a louder buzzer, connect `D5` to a small NPN transistor or MOSFET driver and power the buzzer from the rocket battery/regulator.

Recommended button wiring:

```text
Teensy pin 4 -> momentary button -> GND
```

The firmware uses `INPUT_PULLUP`, so the button is active-low.

Buzzer behavior:

- boot success: rising ready melody after LoRa, storage, baro, and IMU initialize
- boot failure: falling warning melody if one of the required devices fails initialization
- landed: staged finder beacon
  - first 3 minutes: frequent longer beacon pattern for easy recovery
  - 3 to 10 minutes: slower double-beep beacon
  - after 10 minutes: slow single long beep to save battery
- short button press: silence the landed finder beep
- long button press, about 2 seconds, while physical SAFE is recognized: stop outputs, commit and close the current NAND log, play a short acknowledgement, and reset the complete Teensy
- the SAFE long-press reboot is available from any flight state so a false bench transition can always be recovered without removing power
- long button press is refused while ARM is applied; the refusal plays the low warning pattern and does not change logs or flight state

## Electrical notes

- The firmware should be documented as a `Teensy 4.1` build.
- Teensy GPIO logic is `3.3 V`.
- `A0` must never be driven above the ADC reference used by the code, which is `3.3 V`.
- All modules must share a common ground with the Teensy.
- Any external module connected directly to Teensy GPIO should be `3.3 V` logic compatible, or level shifted.

## Firmware check

I checked the rocket firmware against the ground receiver:

- packet IDs match:
  - `0x01` = flight
  - `0x02` = nav
  - `0x03` = status
  - `0x04` = identity
- packet layouts match the ground-side parser
- both sides use `915 MHz`
- battery status is sent in the status packet as `batt_mv`
- the battery OK flag is pack-aware:
  - `1S`: battery warning below `3.85 V`, critical below `3.70 V`
  - `2S`: battery warning below `6.8 V`, critical below `6.4 V`
- ground currently derives rocket battery `OK/WARN/CRIT` from `batt_mv`, matching these thresholds

## Legacy SD config

The parser for an optional config file remains in the source, but the current
flight build uses `SD_CONFIG_LOAD_ENABLE=0` and does not load it. If that
feature is deliberately re-enabled, the expected file is:

```text
/rocket_config.txt
```

Example:

```ini
rocket_name=shadow
```

Accepted aliases:

```ini
rocket=shadow
name=shadow
```

Rules:

- max transmitted/displayed length is 15 characters
- accepted characters are letters, numbers, `_`, `-`, and `.`
- lines can contain comments after `#`
- if SD is missing or the file is missing/invalid, firmware uses `DEFAULT_ROCKET_NAME`
- the name is sent over LoRa using identity packet `0x04`

## Overall wiring diagram

```text
                    +----------------------+
                    |      Teensy 4.1      |
                    |                      |
    RFM95 CS   <--> | D10              VIN | <--- Rocket power rail
    RFM95 RST  <--> | D9               GND | ----- Common ground
    RFM95 DIO0 <--> | D2                   |
    RFM95 MOSI <--> | D11                  |
    RFM95 MISO <--> | D12                  |
    RFM95 SCK  <--> | D13                  |
                    |                      |
    GPS TX     ---> | D0 / RX1             |
    GPS RX     <--- | D1 / TX1   optional  |
                    |                      |
    MS5607 SDA <--> | D18 / SDA            |
    MS5607 SCL <--> | D19 / SCL            |
    LSM9DS1 SDA<--> | D18 / SDA            |
    LSM9DS1 SCL<--> | D19 / SCL            |
                    |                      |
    VBAT divider -->| A0                   |
    SAFE/ARM NC --->| D16                  |
                    |                      |
    QSPI NAND   <--> | native QSPI pads    |
                    |                      |
    built-in uSD <--| internal SDIO        |
                    +----------------------+
```

## Connection summary

| Function | Teensy pin / bus | External connection | Notes |
|---|---:|---|---|
| LoRa CS | D10 | RFM95 `NSS/CS` | Shared SPI bus |
| LoRa RST | D9 | RFM95 `RST` | Active-low reset |
| LoRa DIO0 | D2 | RFM95 `DIO0` | LoRa interrupt |
| LoRa SPI | D11 / D12 / D13 | RFM95 `MOSI` / `MISO` / `SCK` | Teensy hardware SPI |
| Status LED | D3 | LED + resistor to GND | External status LED |
| Finder buzzer | D5 | Piezo buzzer or transistor driver | Beeps at boot and after landed |
| Service button | D4 | Momentary switch to GND | Short press silences beep; long press resets for next attempt |
| Pyro channel 1 output | D6 | MOSFET/transistor driver input | Default function `A`; output disabled by default |
| Pyro channel 2 output | D7 | MOSFET/transistor driver input | Default function `M`; output disabled by default |
| Pyro channel 3 output | D8 | MOSFET/transistor driver input | Default function `N`; output disabled by default |
| Pyro channel 4 output | D15 | MOSFET/transistor driver input | Default function `N`; output disabled by default |
| GPS RX | D0 (`Serial1 RX`) | GT-U7 `TX` | `9600` baud |
| GPS TX | D1 (`Serial1 TX`) | GT-U7 `RX` | Optional |
| I2C SDA | D18 | MS5607 `SDA`, LSM9DS1 `SDA` | Shared I2C bus |
| I2C SCL | D19 | MS5607 `SCL`, LSM9DS1 `SCL` | Shared I2C bus |
| Battery ADC | A0 | Divider midpoint | See battery divider section |
| SAFE/ARM switch | D16 | Microswitch `NC`; `C` goes to GND | Inserted pin presses lever = SAFE |
| QSPI NAND memory | Native QSPI interface | `LittleFS_QPINAND` device | Optional, see memory section |
| SD card | Built-in SD | Teensy 4.1 on-board microSD | `SD.begin(BUILTIN_SDCARD)` |

## Device-by-device connections

### 1. LoRa radio: RFM95

Firmware references:

- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h)
- [RocketV10.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/RocketV10.ino)

Configured for:

- `915 MHz`
- SPI frequency `8 MHz`

Connections:

| RFM95 pin | Teensy pin | Voltage / note |
|---|---:|---|
| `VCC` | `3.3 V` | Power the radio from `3.3 V` |
| `GND` | `GND` | Common ground |
| `NSS/CS` | `D10` | Chip select |
| `RST` | `D9` | Reset line |
| `DIO0` | `D2` | LoRa IRQ / packet event |
| `MOSI` | `D11` | SPI |
| `MISO` | `D12` | SPI |
| `SCK` | `D13` | SPI |

Diagram:

```text
RFM95                 Teensy 4.1
-----                 ----------
VCC      -----------> 3.3V
GND      -----------> GND
NSS/CS   -----------> D10
RST      -----------> D9
DIO0     -----------> D2
MOSI     -----------> D11
MISO     <----------- D12
SCK      -----------> D13
```

### 2. GPS receiver: GT-U7

Firmware references:

- GPS port is `Serial1`
- baud rate is `9600`

Connections:

| GT-U7 pin | Teensy pin | Voltage / note |
|---|---:|---|
| `TX` | `D0 / RX1` | Required |
| `RX` | `D1 / TX1` | Optional |
| `GND` | `GND` | Common ground |
| `VCC` | module dependent | Check your specific GT-U7 breakout regulator and logic level |

Diagram:

```text
GT-U7                 Teensy 4.1
-----                 ----------
TX       -----------> D0 / RX1
RX       <----------- D1 / TX1   optional
GND      -----------> GND
VCC      -----------> module supply
```

Notes:

- The firmware only requires GPS `TX -> Teensy RX1` to parse NMEA data.
- `TX1 -> GPS RX` is only needed if you want to send commands to the GPS module.
- Do not assume every GT-U7 breakout is identical. Some boards accept `5 V` on `VCC`, some are intended for `3.3 V` systems. Check the actual module you use.

### 3. Barometer: MS5607

Firmware references:

- I2C addresses checked: `0x76`, then `0x77`

Connections:

| MS5607 pin | Teensy pin | Voltage / note |
|---|---:|---|
| `SDA` | `D18 / SDA` | I2C |
| `SCL` | `D19 / SCL` | I2C |
| `GND` | `GND` | Common ground |
| `VCC` | sensor board dependent | Use the voltage required by your MS5607 breakout |

Diagram:

```text
MS5607                Teensy 4.1
------                ----------
SDA      <--------->  D18 / SDA
SCL      <--------->  D19 / SCL
GND      -----------> GND
VCC      -----------> sensor supply
```

Notes:

- The firmware uses `Wire.begin()` and expects a standard I2C device.
- If your breakout does not include pull-ups, you may need external pull-up resistors on `SDA` and `SCL`.

### 4. IMU: LSM9DS1

Firmware references:

- IMU is optional
- the current flight build expects the Adafruit LSM9DS1 library to be available from the repo-local Arduino libraries

Connections:

| LSM9DS1 pin | Teensy pin | Voltage / note |
|---|---:|---|
| `SDA` | `D18 / SDA` | Shared I2C bus |
| `SCL` | `D19 / SCL` | Shared I2C bus |
| `GND` | `GND` | Common ground |
| `VCC` | sensor board dependent | Use the voltage required by your LSM9DS1 breakout |

Diagram:

```text
LSM9DS1               Teensy 4.1
-------               ----------
SDA      <--------->  D18 / SDA
SCL      <--------->  D19 / SCL
GND      -----------> GND
VCC      -----------> sensor supply
```

Notes:

- The firmware configures accel, gyro, and magnetometer after `lsm.begin()`.
- The IMU shares the same I2C bus as the MS5607.

### 5. Memory

The firmware uses two different storage paths:

- built-in SD card via `SD.begin(BUILTIN_SDCARD)`
- optional QSPI NAND via `LittleFS_QPINAND`

Reference implementation used in your working test project:

- [teensy41_nand_sd_test.ino](/Users/k_pochkaev/github/debug_projects/teensy41_nand_sd_test/teensy41_nand_sd_test.ino)
- that sketch documents:
  - Teensy 4.1 QSPI pads
  - `CS1 = pin 51`
  - `LittleFS_QPINAND`
  - `W25N01G`

What the code actually does:

- `sdOk = SD.begin(BUILTIN_SDCARD);`
- SD is only marked healthy after a write/read verification using `"/rocket_sd.txt"`
- `nandOk = qspiNand.begin();` when `LittleFS.h` is available at compile time
- NAND is only marked healthy after a write/read verification using `"/rocket.txt"`
- during flight, SD stores readable CSV logs
- during flight, NAND stores compact binary flight records

Current storage roles:

- SD:
  - primary readable log
  - file names like `rocket_flight0041.csv`
- NAND:
  - resilient onboard recorder
  - file names like `/rocket_flt0041.bin`
  - typed V4 binary records for later export
  - rotates oldest `/fltNNNN.bin` logs before opening a new log if NAND is near full

NAND format status:

- current firmware writes `RV10NLG` header version `4`
- current firmware writes typed V4 payload records:
  - type `1`: `50 Hz` full-state records
  - type `2`: `200 Hz` compact IMU records
  - type `3`: barometer records
  - type `4`: GPS records
  - type `5`: battery records
  - type `6`: flight state/event records, recovery classifications, and log-only dual-deploy events
  - type `7`: telemetry snapshot records
  - type `8`: quaternion attitude records
- current export decodes only the current V4 header/record combination:
  - `header.version = 4`
  - `header.record_format = 4`
- there is no legacy NAND decode path in flight firmware
- old/unsupported/corrupt files are skipped during service export
- exported CSV files include metadata comment lines before the CSV header

NAND full handling:

- before a new NAND log is opened, firmware checks `qspiNand.totalSize()` and `qspiNand.usedSize()`
- if free space is below `NAND_MIN_FREE_BYTES`, oldest `/fltNNNN.bin` logs are deleted until the reserve is restored
- if the number of NAND log files is at or above `NAND_MAX_LOG_FILES`, oldest logs are deleted until a new file can be opened
- current defaults:
  - `NAND_MIN_FREE_BYTES = 16 MiB`
  - `NAND_MAX_LOG_FILES = 96`
  - `NAND_ROTATE_ENABLE = 1`
- V4 writes `10 Hz` full-state, `50 Hz` barometer, and `200 Hz` compact IMU
  records, with IMU reduced to `50 Hz` during descent states
- the 16 MiB reserve protects NAND rotation; armed/in-flight capacity is
  primarily limited by the 576 KiB RAM recorder
- rotation happens before opening a new log, not in the middle of an active flight log

Important detail:

- the firmware does not assign regular SPI pins for the NAND device
- that means this memory is not a generic `MOSI/MISO/SCK/CS` SPI module in the current code
- it is expected to be connected through the native Teensy-supported `QPINAND/QSPI` interface used by `LittleFS_QPINAND`
- based on the working test project, the intended NAND device is `W25N01G` on the Teensy 4.1 native QSPI pads, using `CS1 / pin 51`

So for this firmware:

- LoRa uses the normal SPI pins `D11/D12/D13` plus `D10/D9/D2`
- NAND memory is a separate native memory interface on the Teensy 4.1 QSPI pads
- SD export/service uses the built-in Teensy 4.1 SD hardware

Diagram:

```text
                              Teensy 4.1
                    +--------------------------------+
LoRa RFM95     <---->| SPI: D10/D11/D12/D13 + D2/D9  |
W25N01G NAND   <---->| native QSPI pads, CS1 = D51   |
built-in SD    <---->| built-in SDIO                 |
                    +--------------------------------+
```

## NAND Service Mode

The rocket firmware supports bench/service operations from an SD control file:

- `/nand_ops.txt`

If this file exists on SD at boot, the controller enters service mode, performs the requested NAND operation, writes a result file, and deletes the command file only on success.

Supported keys:

```ini
version=1
operation_id=42
copy_to_sd=1
clean_nand=0
export_latest_only=0
export_imu=1
require_nand_ok=1
require_sd_ok=1
```

Aliases currently accepted:

- `copy_to_sd` or `export_to_sd`
- `clean_nand` or `erase_nand_after_export`
- `export_latest_only` or `latest_only`
- `export_imu` or `copy_imu`

Behavior:

- `copy_to_sd=1` exports NAND binary logs to CSV files on SD
- `clean_nand=1` removes NAND flight log files only after current V4 export completes without hard I/O failures
- `export_latest_only=1` exports only the newest NAND log and skips older logs
- `export_imu=0` skips detail CSVs and exports only the full-state CSV
- full multi-log export with `export_imu=1` can be slow because the Teensy formats multiple binary streams, especially `200 Hz` IMU records, as decimal CSV text
- result file is written to `/nand_ops_result.txt`
- command file `/nand_ops.txt` is removed only if the requested operation succeeds
- test template file:
  - [nand_ops_example.txt](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/nand_ops_example.txt)

Recommended first hardware test:

1. copy [nand_ops_example.txt](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/nand_ops_example.txt) to the SD root as `nand_ops.txt`
2. keep `clean_nand=0`
3. boot the rocket controller
4. after success, inspect:
   - `/nand_ops_result.txt`
   - exported `rocket_nand_*.csv` files on SD

Recommended quick field export:

```ini
version=1
operation_id=1013
copy_to_sd=1
clean_nand=0
export_latest_only=1
export_imu=0
require_nand_ok=1
require_sd_ok=1
```

Important guard:

- NAND erase is not performed unless export was requested and export completed without hard failures

This is intentional to reduce risk of deleting the only copy of flight data.

Current-only NAND export behavior:

- valid current-format `RV10NLG` V4 files are exported
- full-state CSV names look like `rocket_nand_0120_op1004.csv`
- high-rate IMU CSV names look like `rocket_nand_0120_op1004_imu.csv`
- detail CSVs are optional with `export_imu=0`
- detail CSV names include:
  - `_imu.csv`
  - `_baro.csv`
  - `_gps.csv`
  - `_batt.csv`
  - `_event.csv`
  - `_telem.csv`
  - `_att.csv`
- latest-only export is optional with `export_latest_only=1`
- old/unsupported/corrupt files are counted as `export_skipped`
- hard read/write/open failures are counted as `export_failed`
- `export_skipped` does not block `clean_nand=1`
- `export_failed` blocks erase and leaves `/nand_ops.txt` in place

Legacy policy:

- production firmware does not keep legacy NAND decoders
- when NAND format changes, old unsupported files are skipped instead of decoded
- this keeps field firmware smaller and easier to reason about

Post-flight tooling:

- `visualizer/rocket_v10_flight_replay.py` reads exported V4 full-state CSV files, including metadata comment lines
- if the sibling quaternion attitude CSV exists, for example `rocket_nand_0120_op1004_att.csv`, replay loads it automatically and uses quaternion interpolation for rocket attitude motion
- if `_att.csv` is missing but the sibling high-rate IMU CSV exists, replay falls back to `_imu.csv` roll/pitch/yaw
- use `--att-csv path/to/file_att.csv` to select a specific quaternion attitude export
- use `--imu-csv path/to/file_imu.csv` to select a specific IMU export
- use `--no-auto-imu` for full-state-only replay after a quick `export_imu=0` field export
- `visualizer/rocket_v10_flight_report.py` also accepts V4 exports with metadata comment lines

Connection note:

- use the same wiring that works in [teensy41_nand_sd_test.ino](/Users/k_pochkaev/github/debug_projects/teensy41_nand_sd_test/teensy41_nand_sd_test.ino)
- this rocket firmware uses the same `LittleFS_QPINAND` class and initialization style
- if the memory is not wired as native Teensy 4.1 QSPI NAND, `qspiNand.begin()` is expected to fail

If your memory device is not `W25N01G` on the Teensy 4.1 QSPI pads, the current README and firmware assumptions are no longer valid.

## Battery voltage measurement

The battery input is measured through a resistor divider into `A0`.

Firmware values from [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h):

- `VBAT_PIN = A0`
- `VBAT_R1_OHMS = 100000`
- `VBAT_R2_OHMS = 47000`
- `ADC_REF_V = 3.3`
- `ADC_MAX_COUNTS = 4095`

Divider wiring:

- battery `+` -> `100k` -> `A0` -> `47k` -> GND

ASCII diagram:

```text
Battery +
   |
  100k
   |
   +-------> A0 (Teensy ADC input)
   |
   47k
   |
  GND
```

Firmware conversion from [RocketV10.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/RocketV10.ino):

```cpp
int raw = analogRead(VBAT_PIN);
float vPin = (float)raw * ADC_REF_V / ADC_MAX_COUNTS;
return vPin * (VBAT_R1_OHMS + VBAT_R2_OHMS) / VBAT_R2_OHMS;
```

So the firmware computes:

```text
Vbat = ADC_voltage * (100k + 47k) / 47k
Vbat = ADC_voltage * 3.1277
```

Practical implications:

- divider ratio is about `3.13:1`
- maximum measurable battery voltage before `A0` reaches `3.3 V` is about `10.32 V`
- that is fine for the current `1S/2S` rocket battery use
- at `4.2 V` battery input, `A0` sees about `1.34 V`
- at `8.4 V` battery input, `A0` sees about `2.69 V`
- do not use this divider unchanged for a fully charged `3S` LiPo, because `12.6 V` would overdrive `A0`

The ADC is configured to 12-bit in setup, and `A0` is explicitly set as input in [RocketV10.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/RocketV10.ino).

### Battery and power recommendations

- Keep the battery negative tied to Teensy ground.
- Do not connect raw battery voltage directly to `A0`.
- If you change either resistor value, update `VBAT_R1_OHMS` and `VBAT_R2_OHMS` in [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h).
- If your analog reference or board ADC behavior differs, update `ADC_REF_V` and verify the measured voltage with a multimeter.

## Storage

The rocket records through RAM to native QSPI NAND. The built-in Teensy 4.1 SD
slot is an optional export/service destination and is not used by the runtime
flight logger.

Diagram:

```text
RocketV10 firmware
      |
      +----> Teensy 4.1 built-in microSD slot
      |
      +----> optional native QSPI NAND
```

## Sensor notes

- MS5607 is probed at `0x76` and then `0x77`
- LSM9DS1 library must be available at compile time for the current v10 build
- GPS is read continuously and nav packets are sent once per second
- GPS does not control the flight state machine, but it is logged as a secondary altitude reference
- magnetometer and approximate yaw are logged for visualization only
- flight data is logged to CSV on the built-in SD card

## Test-flight build `rv10.20260724g`

This build adds EEPROM-backed USB serial configuration at `115200` baud.
`SHOW` and `HELP` are always available. `SET`, `SAVE`, and `DEFAULTS` are
accepted only while the physical SAFE input is active and the flight state is
`IDLE` or `PAD`.

Supported settings are:

- RF: `LORA_SF` (`6..10`), `LORA_BW` (`62500`, `125000`, `250000`, or
  `500000`), `LORA_CR` (`5..8`), and `TX_POWER_DBM` (`2..17`)
- telemetry: `FLIGHT_TX_MS` (`200..5000`), `NAV_TX_MS` (`500..10000`), and
  `STATUS_TX_MS` (`500..10000`)
- flight detection/logging: `LAUNCH_ACCEL_G`, `LAUNCH_IMU_CONFIRM_MS`,
  `LAUNCH_BARO_ALT_M`, `LAUNCH_BARO_VEL_MPS`, `APOGEE_MIN_ALT_M`, and
  `MAIN_ALT_M`

Example:

```text
SHOW
SET FLIGHT_TX_MS 200
SET LAUNCH_ACCEL_G 1.35
SAVE
```

RF changes apply immediately and must also be made on Ground. Higher spreading
factors enforce a slower minimum flight packet period to prevent an impossible
airtime schedule. No serial command exists to enable/fire pyros, change pyro
pins, or bypass the physical SAFE workflow.

Rocket retries LoRa initialization every two seconds if the radio is not found
at boot. `SHOW` reports radio state and transmit sequence counters.

Rocket log deletion is available over USB serial only while physical SAFE is
active and the state is `IDLE` or `PAD`. The final `CONFIRM` token is required:

```text
ERASE NAND CONFIRM
ERASE SD CONFIRM
ERASE ALL CONFIRM
```

`ERASE SD` removes only recognized `rocket_flight*.csv`, legacy `flight*.csv`,
and `rocket_nand_*.csv` log/export files. It preserves Rocket configuration,
service-request/result, and filesystem-test files. `ERASE NAND` removes only
recognized Rocket binary flight logs. Active logs are closed cleanly and new
logs begin automatically afterward.

This build keeps the physical SAFE/ARM workflow, latched READY behavior, and
log-only pyro configuration unchanged. Reliability/data changes:

- flight-state work now runs independently of successful barometer conversions,
  so fresh IMU acceleration can confirm launch during a barometer delay/failure
- cached IMU acceleration cannot satisfy the 50 ms launch confirmation after
  IMU updates stop
- non-finite or physically impossible IMU/barometer samples do not refresh
  sensor freshness
- GPS validity requires a recent TinyGPS++ location/altitude update
- LoRa payload CRC is disabled and a stuck asynchronous transmission is cleared
  after one second
- the paired flight-link profile restores the original SF7 / 125 kHz /
  CR 4:5 at 17 dBm, with Flight packets at 5 Hz
- telemetry integer fields saturate instead of wrapping
- high-rate NAND IMU data uses a new typed wide-gyro record with 32-bit
  millidegree/s fields, covering the configured +/-2000 dps range
- pyro output-off servicing runs before storage as well as after it

Physical pyro outputs remain disabled.

## Build and upload

Use the shared build/upload guide:

- [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)

Rocket v10 target:

- board: `Teensy 4.1`
- FQBN: `teensy:avr:teensy41`
- repo-local libraries: `/Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-libs`

Verified compile command:

```bash
mkdir -p /tmp/rocketv10-build
arduino-cli compile --fqbn teensy:avr:teensy41 --libraries /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-libs --build-path /tmp/rocketv10-build /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10
```

Verified upload command when the rocket appears as a Teensy port:

```bash
arduino-cli upload --fqbn teensy:avr:teensy41 --port usb:110000 --input-dir /tmp/rocketv10-build /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10
```

Recent rocket serial port:

```text
/dev/cu.usbmodem187564601
```

Prefer `usb:...` Teensy ports from `arduino-cli board list` when available.

## Rocket state and telemetry meanings

Rocket flight states sent to ground:

| State | Meaning |
|---|---|
| `IDLE` | Firmware is running but not yet considered settled on pad. |
| `PAD` | Barometer baseline is established and rocket is waiting for launch detection. |
| `ASCENT` | Launch has been detected and vertical motion is upward. |
| `COAST` | Rocket is still airborne but vertical velocity has dropped below the coast threshold. |
| `SUBSONIC_COAST` | Recovery classifier detected sustained baro climb/coast behavior. |
| `NEAR_APOGEE` | High altitude and low vertical speed indicate near-apogee behavior. |
| `DESCENT_BALLISTIC` | Apogee/descent has been detected, with fast negative vertical velocity. |
| `UNDER_DROGUE` | Descent has slowed into an under-drogue-like profile. |
| `DUAL_DEPLOY_APOGEE_LOGGED` | Simulated/log-only apogee deploy event has been recorded. |
| `DUAL_DEPLOY_MAIN_LOGGED` | Simulated/log-only main deploy event has been recorded. |
| `POST_FLIGHT_GROUND` | Recovery classifier detected post-flight ground behavior. |
| `LANDED` | Sustained low-speed, low-altitude behavior indicates landing. |
| `ABORT` | Firmware entered abort/fault state. |

Main rocket log/telemetry values:

| Value | Meaning |
|---|---|
| `rel_alt_m` | Barometric altitude above the captured pad/base altitude. Used for launch/state logic. |
| `vel_mps` / `velZ` | Barometric vertical velocity estimate in m/s. Used for launch/state logic. |
| `gps_rel_alt` | GPS altitude above GPS baseline. Diagnostic only. |
| `gps_base_alt` | GPS altitude baseline captured on pad. |
| `baro_gps_delta` | Difference between barometric relative altitude and GPS relative altitude. |
| `baro_gps_diverge` | Diagnostic flag when baro/GPS altitude disagreement is large. Not a flight-state trigger. |
| `diag_flags` | Bitmask: bit `0` baro/GPS divergence; bits `1–3` attitude correction modes; bit `4` SAFE input; bit `5` SAFE observed since boot; bit `6` launch candidate; bit `7` touchdown candidate. |
| `mx/my/mz` | Magnetometer values from LSM9DS1. Logged for visualization. |
| `roll/pitch/yaw` | Approximate orientation values derived from the internal quaternion estimator. Yaw is magnetometer-aided when correction is active and remains visualization-only. |
| `batt_mv` | Battery voltage sent to ground in millivolts. |
| `health_flags` | Bitmask for barometer, IMU, GPS, SD, NAND, log, and battery health. |

Launch arming uses a removable SAFE pin and D16 microswitch input. Wire switch `C`
to GND and `NC` to D16. The input uses `INPUT_PULLUP`: inserted pin/pressed
lever is HIGH/SAFE, and removed pin/released lever is LOW/ARM. After every boot,
firmware must observe SAFE for at least the debounce interval before it will
accept a later SAFE-to-ARM transition. Powering on while the pin is temporarily
removed therefore remains inhibited even after the 60 s boot timer.

The telemetry launch-status byte reports the specific readiness reason rather
than one combined inhibit state: boot countdown, SAFE-required, SAFE, pad
settle, sensor/log/battery fault, vertical/stillness hold, 10 s arming
countdown, READY, launch candidate, or active flight. `launch_wait_s` is
meaningful only for boot, pad-settle, and arming countdown states.

After the SAFE pin is removed at the pad, READY requires 10 s of continuous
vertical orientation, stillness, healthy/fresh IMU and barometer data, working
logging, and non-critical battery voltage. READY refreshes the pad baseline and
then remains latched. Launch-like IMU or barometer evidence enters an internal
3 s launch-candidate window. Sustained nose-axis acceleration can confirm a
short D12-class launch without waiting for the barometer; the existing
barometer and obvious-flight paths remain available. A rejected bump returns
to READY instead of disarming the rocket.

Recovery classification is also HPR-inspired. It can recover missed
coast/descent/post-flight states and logs simulated apogee/main charge events
to `_event.csv`. After descent, a qualified impact or sustained low-altitude,
low-velocity condition starts the finder before final LANDED confirmation.
Rolling or dragging no longer has to satisfy the old continuous gyro-stillness
gate before the flight can finish. Inserting the SAFE pin after
`POST_FLIGHT_GROUND` or `LANDED` silences the finder. GPS, yaw, and magnetometer
data are not used for launch, apogee, recovery, or landed decisions.

Pyro channel assignments are HPR-style and configurable in firmware:

| Channel | Teensy pin | Default function | Per-channel log toggle |
|---|---:|---:|---:|
| Pyro 1 | D6 | `A` apogee/drogue | `PYRO_CH1_LOG_ENABLE` |
| Pyro 2 | D7 | `M` main | `PYRO_CH2_LOG_ENABLE` |
| Pyro 3 | D8 | `N` disabled | `PYRO_CH3_LOG_ENABLE` |
| Pyro 4 | D15 | `N` disabled | `PYRO_CH4_LOG_ENABLE` |

Function letters match the HPR idea:

| Function | Meaning |
|---:|---|
| `N` | Disabled/no function |
| `A` | Apogee/drogue deploy request |
| `M` | Main deploy request |
| `B` | Booster/stage separation request |
| `I` | Sustainer ignition request |
| `1` | Airstart motor 1 request |
| `2` | Airstart motor 2 request |

`PYRO_OUTPUT_ENABLE` is `0` by default in [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10/config.h). With this safe default, the firmware initializes D6/D7/D8/D15 to the inactive level but does not pulse them. Matched channels can still create `_event.csv` records if their `PYRO_CH*_LOG_ENABLE` value is `1`. After external drivers and arming hardware are installed, setting `PYRO_OUTPUT_ENABLE=1` will pulse matched channels for `PYRO_FIRE_MS` and add `PYRO_CHANNEL*_OUTPUT_ON/OFF` events to `_event.csv`.

Do not connect an e-match directly to a Teensy GPIO. Use a MOSFET/transistor driver, gate/base pulldown, separate current-limited pyro battery, and a physical arming switch.

## 2026-07-25 development recorder

Installed firmware `rv10.20260725f` keeps the filesystem out of PAD/ARM/flight
timing. A rolling 64 KiB pre-launch window and subsequent flight records are
held in a 576 KiB RAM buffer, then committed to NAND at LANDED/ABORT or during
an explicit SAFE service operation. IMU is logged at 200 Hz through coast and
50 Hz during descent; barometer remains 50 Hz. Runtime SD CSV logging and
automatic SD configuration loading are disabled.

Useful SAFE/PAD serial commands:

```text
SHOW
TIMING
TIMING RESET
STORAGE STATUS
FLIGHT RESET CONFIRM
NAND LIST
NAND EXPORT SD ALL SUMMARY
NAND EXPORT SD <index> FULL
NAND ERASE ALL CONFIRM
SD MOUNT
SD LIST
SD ERASE LOGS CONFIRM
```

The RAM design prevents measured LittleFS pauses from delaying flight-state
evaluation, but uncommitted records are lost on total power failure. Physical
pyro outputs remain disabled.
