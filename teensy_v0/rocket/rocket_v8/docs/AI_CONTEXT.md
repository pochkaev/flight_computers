# Rocket v8 AI Context

This file is a working context/handoff note for continuing work on the rocket flight computer firmware.

Primary docs and firmware:

- [README.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/docs/README.md)
- [RocketV8.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/RocketV8.ino)
- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/config.h)

## Hardware

Board:

- Teensy 4.1

Connected devices:

- LoRa RFM95
- LSM9DS1
- MS5607
- GT-U7 GPS
- built-in SD
- QSPI NAND via `LittleFS_QPINAND`

Important pins:

- LoRa:
  - `CS = 10`
  - `RST = 9`
  - `DIO0 = 2`
  - `SPI = 11/12/13`
- GPS:
  - `Serial1`
  - `RX1 = pin 0` from GPS `TX`
  - `TX1 = pin 1` to GPS `RX` optional
- I2C:
  - `SDA = 18`
  - `SCL = 19`
- Battery divider:
  - `A0`
  - `R1 = 100k`
  - `R2 = 47k`
- External status LED:
  - `pin 3`
  - wiring: `pin 3 -> 330R -> LED anode`, LED cathode to `GND`

## Current firmware status

Working now:

- LoRa init and telemetry
- GPS parsing
- MS5607 altitude/pressure reading
- LSM9DS1 init and readout
- SD init and read/write check
- NAND init and read/write check
- SD CSV logging
- NAND binary logging
- status LED patterns
- SD-driven NAND service mode command parsing

Current runtime debug fields include:

- sensor state
- GPS state
- battery voltage and pack type
- `sd`
- `sdlog`
- `nand`
- `nlog`
- `log`

## Important implementation notes

Serial:

- Serial debug is optional and should not be used for flight-critical behavior.
- For flight builds, set `SERIAL_DEBUG_LEVEL = 0`.
- Current config is still bench-oriented.

Task timing:

- IMU: `10 ms`
- baro/state: `50 ms`
- battery: `100 ms`
- flight telemetry: `200 ms`
- nav telemetry: `1000 ms`
- status telemetry: `2000 ms`
- storage logging: `200 ms`
- flush: `1000 ms`

Battery logic:

- auto-detects `1S` vs `2S`
- `>= 5.0V` means `2S`
- `1S warn = 3.4V`
- `1S crit = 3.2V`
- `2S warn = 6.8V`
- `2S crit = 6.4V`

Note:

- battery warn currently can flap near threshold because hysteresis has not been added yet

## Storage model

SD role:

- human-readable flight log
- files like `flight1.csv`

NAND role:

- onboard binary recorder
- files like `/flt0001.bin`

NAND binary file structure:

- header: `NandLogHeaderV1`
- records: `NandFlightRecordV1`

NAND records store:

- time
- health flags
- flight flags
- altitude
- relative altitude
- velocity
- temperature
- pressure
- accel
- gyro
- roll
- pitch
- GPS fix/sats/lat/lon/alt/speed
- battery voltage
- battery pack type

## Service mode

Trigger:

- place `/nand_ops.txt` on SD before boot

Supported keys:

```ini
version=1
operation_id=42
copy_to_sd=1
clean_nand=0
require_nand_ok=1
require_sd_ok=1
```

Accepted aliases:

- `copy_to_sd` or `export_to_sd`
- `clean_nand` or `erase_nand_after_export`

Service behavior:

- if requested, export NAND logs to CSV files on SD
- if requested, erase NAND logs only after successful export
- write result file `/nand_ops_result.txt`
- remove `/nand_ops.txt` only on success

Important:

- service mode logic is implemented in firmware
- it has not been tested on hardware yet

## LED patterns

- slow blink: boot/init
- heartbeat pulse: ready
- fast blink: service mode active
- solid on: service success
- 2 short blinks: SD error
- 3 short blinks: NAND error
- fast 250 ms blink: general fault

## Verified on hardware

Already verified:

- board boots
- sensors initialize
- LoRa initializes
- SD check passes
- NAND check passes
- both SD and NAND logging paths report active in runtime debug

Observed live debug example:

```text
... sd=1 sdlog=1 nlog=1 log=1 nand=1
```

## Not yet verified

- full `/nand_ops.txt` service mode workflow on hardware
- NAND export-to-SD result files on real flight data
- NAND erase-after-export on hardware
- post-flight end-to-end recovery workflow

## Recommended next tests

1. Create `/nand_ops.txt` on SD with export only:

```ini
version=1
operation_id=1
copy_to_sd=1
clean_nand=0
require_nand_ok=1
require_sd_ok=1
```

2. Boot the controller and verify:

- LED enters service pattern
- `nand_ops_result.txt` is written
- exported CSV files appear on SD
- `nand_ops.txt` is deleted on success

3. Then test erase flow:

```ini
version=1
operation_id=2
copy_to_sd=1
clean_nand=1
require_nand_ok=1
require_sd_ok=1
```

4. Add battery threshold hysteresis after storage workflow is confirmed.

## Build and upload notes

Local CLI config used during development:

- `/Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-cli.yaml`

Typical commands:

```bash
arduino-cli --config-file /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-cli.yaml compile --fqbn teensy:avr:teensy41 /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8
arduino-cli --config-file /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-cli.yaml upload -p /dev/cu.usbmodem187564601 --fqbn teensy:avr:teensy41 /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8
```

Serial port used recently:

- `/dev/cu.usbmodem187564601`

## Caution

- `pin 13` must not be reused for status LED because it is already SPI `SCK` for LoRa.
- NAND service mode is implemented but still unvalidated on hardware.
- Before real flight, switch serial debug to off and re-test a clean flight boot.
