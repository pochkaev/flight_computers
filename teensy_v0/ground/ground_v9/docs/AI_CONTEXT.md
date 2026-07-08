# Ground v9 AI Context

This file is a working handoff/context note for continuing work on the ground controller firmware.

Primary files:

- [README.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/docs/README.md)
- [GroundStationV9.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/GroundStationV9.ino)
- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/config.h)
- [radio.cpp](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/radio.cpp)
- [ui.cpp](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/ui.cpp)
- [power.cpp](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/power.cpp)
- [sdlog.cpp](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/sdlog.cpp)

## Hardware target

Important:

- Ground controller target is `Teensy 4.0`
- Rocket controller target is `Teensy 4.1`

This difference matters for build/upload and likely caused earlier confusion.

## Ground hardware

Main connected parts:

- LoRa `RFM95`
- GPS `GT-U7` on `Serial1`
- barometer `BMP180` on I2C
- QVGA TFT display `ILI9341`
- external SPI SD card
- RS-485 transceiver for launch/power module
- local page button
- ARM / START switches
- two indicator LEDs
- local battery divider on `A0`

## Current pin map from firmware

Ground `config.h` is the source of truth.

LoRa:

- `CS = D10`
- `RST = D9`
- `DIO0 = D2`
- `SPI = D11 / D12 / D13`

Display:

- `TFT_CS_PIN = D3`
- `TFT_DC_PIN = D15`
- `TFT_RST_PIN = D28`
- shared SPI on `D11 / D12 / D13`

SD:

- `SD_CS_PIN = D4`
- shared SPI on `D11 / D12 / D13`

GPS:

- `Serial1`
- `RX1 = D0` from GPS `TX`
- `TX1 = D1` optional to GPS `RX`

BMP180:

- `SDA = D18`
- `SCL = D19`

RS-485:

- `DE/RE = D6`
- `Serial2 RX = D7`
- `Serial2 TX = D8`

Controls:

- page button `D5`
- ARM A `D20`
- ARM B `D21`
- START A `D16`
- START B `D17`
- LED A `D22`
- LED B `D23`

Battery:

- `PWR_VBAT_PIN = A0`
- divider constants:
  - `GND_VBAT_R1_OHMS = 330k`
  - `GND_VBAT_R2_OHMS = 100k`
- calibration factor:
  - `GND_VBAT_CAL_FACTOR = 0.8527132`

## Shared bus warning

These devices share SPI:

- LoRa
- TFT display
- SD card

Current chip selects:

- TFT `CS = D3`
- SD `CS = D4`
- LoRa `CS = D10`

Any bad wiring or electrical contention on one SPI peripheral can destabilize the others.

## Current firmware behavior

The ground firmware:

- receives rocket telemetry over LoRa
- parses:
  - flight packet
  - nav packet
  - status packet
- reads local GPS continuously
- reads BMP180 periodically
- logs to SD
- drives the TFT UI
- optionally controls/monitors the RS-485 launch module

## Current debug style

Ground debug was aligned with rocket style.

Current setting:

- `SERIAL_DEBUG_LEVEL`
  - `0` = disabled for field use
  - `1` = boot + important status
  - `2` = verbose packet/status debug

Compatibility macros still exist:

- `DBG1`
- `DBG2`
- `DBG3`

But they now map onto the rocket-style serial debug model.

## Current UI status area

The rocket status page now shows two lines:

```text
SYS GPS IMU BARO
LOG SD NAND
```

Meaning:

- `SYS` = overall rocket health summary
- `GPS` = rocket GPS health bit
- `IMU` = rocket IMU health bit
- `BARO` = rocket barometer health bit
- `LOG` = aggregate logging status
- `SD` = rocket SD health bit
- `NAND` = rocket NAND health bit

Current color behavior:

- green = OK
- yellow = warning
- red = fault
- labels that are also status indicators are active, not just white labels

`SYS` logic:

- `FAULT` if link is stale
- `WARN` if fix is weak or any main subsystem health is bad
- `OK` otherwise

`LOG` logic:

- green when `rocketLogOk`
- red when `rocketLogOk` is false

## Rocket telemetry compatibility

Ground and rocket are currently aligned on:

- `PKT_TYPE_FLIGHT_V7 = 0x01`
- `PKT_TYPE_NAV_V7 = 0x02`
- `PKT_TYPE_STATUS_V8 = 0x03`

Ground uses the same health bit definitions as the rocket:

- `HEALTH_BARO_OK`
- `HEALTH_IMU_OK`
- `HEALTH_GPS_OK`
- `HEALTH_SD_OK`
- `HEALTH_NAND_OK`
- `HEALTH_LOG_OK`
- `HEALTH_BATT_OK`

## Recent fixes already applied

### 1. Correct MCU target

Ground was mistakenly treated as `Teensy 4.1` earlier.

Now corrected:

- build/upload target is `Teensy 4.0`

### 2. Safer runtime behavior

The assembled ground unit previously showed a severe failure pattern:

- TFT went unstable / white fast blink
- LEDs blinked rapidly
- GPS LED and Teensy LED blinked rapidly
- USB serial disappeared from macOS

Likely causes identified in firmware:

- dynamic `String(...)` debug construction on every received packet
- unbounded `log_flight()` frequency during active flight

Changes made:

- removed packet debug paths that relied on repeated `String` allocations
- replaced them with fixed `snprintf` formatting
- reduced debug verbosity by default
- rate-limited ground logging:
  - `FLIGHT_LOG_MS = 200`
  - `RECOVERY_LOG_MS = 1000`

### 3. Ground battery reading correction

Observed:

- displayed voltage was about `14.19 V`
- real measured voltage was about `12.10 V`

Applied correction:

- `GND_VBAT_CAL_FACTOR = 0.8527132`

This is now baked into the ground voltage read path.

### 4. README corrected

Ground README was updated to match current firmware.

Important pin corrections:

- `START A/B = D16 / D17`
- `LED A/B = D22 / D23`

## Current risks / unknowns

1. Full assembled-device stability is still the main risk.

Important evidence:

- standalone Teensy 4.0 on USB alone could be flashed successfully
- previous violent failure was observed when the MCU was installed in the full ground hardware

That strongly suggests:

- attached hardware interaction
- SPI bus contention
- display/SD/LoRa interaction
- power issue on the assembled board
- or wiring conflict

2. The standalone USB verification was limited.

- Board could be flashed
- It stayed enumerated
- Useful boot serial text was not reliably captured

So the standalone board is more stable than the assembled system, but not deeply characterized yet.

## Recommended next debug path

If the assembled ground unit still misbehaves, isolate by attached subsystem:

1. TFT disconnected
2. SD disconnected
3. GPS disconnected
4. RS-485 disconnected
5. LoRa disconnected

Because TFT, SD, and LoRa share SPI, start there first.

## Current build/upload commands

Compile ground:

```bash
arduino-cli --config-file /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-cli.yaml compile --fqbn teensy:avr:teensy40 /Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9
```

Upload ground:

```bash
arduino-cli --config-file /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-cli.yaml upload -p /dev/cu.usbmodem184901201 --fqbn teensy:avr:teensy40 /Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9
```

Known recent serial port:

- `/dev/cu.usbmodem184901201`

## Good next improvements

1. Improve main operational page density and hierarchy
2. Add per-packet receive stats on ground:
   - last seq
   - missed packets
   - rate per packet type
   - age per packet type
3. If assembled-device instability persists:
   - add temporary boot-time peripheral isolation switches in firmware
4. If battery still needs tuning:
   - refine `GND_VBAT_CAL_FACTOR`

## Current state summary

Ground firmware is in a much better state than before:

- target corrected to `Teensy 4.0`
- packet parsing aligned with rocket
- battery calibration corrected
- debug style aligned with rocket
- status UI improved
- `SYS` and `LOG` are active indicators

But the assembled ground hardware still needs final stability verification with all peripherals attached.
