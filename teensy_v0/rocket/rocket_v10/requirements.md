# Rocket Flight Computer V10 Requirements

## Scope

This is the next rocket flight computer version after `rocket_v7`.

Primary goal:
- create a new Teensy 4.1 based flight computer that stays compatible with the current 915 MHz LoRa ground workflow, while adapting the firmware and storage model to the new rocket hardware.

This document is an initial requirements capture based on:
- existing code in `rocket/rocket_v7`
- existing protocol/docs in `docs/docs_V7` and `docs/docs_V8`
- the confirmed Rocket V10 hardware configuration
- the new hardware list provided for the rocket

## Existing Baseline To Preserve

Current `rocket_v7` behavior to carry forward unless intentionally changed:
- Teensy-based rocket flight computer using LoRa RFM95 at `915 MHz`
- binary telemetry sent over LoRa
- separate high-rate flight packet and lower-rate navigation packet
- flight state machine with at least:
  - `IDLE`
  - `PAD`
  - `ASCENT`
  - `COAST`
  - `DESCENT`
  - `LANDED`
  - `ABORT`
- barometric altitude + derived vertical velocity
- GPS parsing with `TinyGPSPlus`
- periodic navigation reporting for recovery
- operation should continue in degraded mode if some optional peripherals are missing

Known V7 implementation details worth reusing:
- LoRa wiring convention:
  - `CS=10`
  - `RST=9`
  - `DIO0=2`
- GPS on `Serial1`
- `Serial1` may be selected as a `115200 8N1` maintenance console only when
  physical SAFE is active, either by an exact `SERVICE UART CONFIRM` handshake
  at the normal GPS `9600` baud or by holding the service button during
  power-on; the GPS must be disconnected, and normal reboot restores GPS mode
- I2C sensor bus on standard Teensy pins
- telemetry cadence target from prior work:
  - flight: about `5-10 Hz`
  - nav: about `~1 Hz`

## New Rocket Hardware

Confirmed hardware for the new rocket flight computer:
- Teensy 4.1
- on-board SD card slot
- QSPI NAND available on Teensy 4.1 build
  - Winbond `W25N01G`
  - verified on the target hardware
  - use `LittleFS_QPINAND`
- LoRa RFM95 `915 MHz`
- barometer: `MS5607`
- GPS: `GT-U7`
- IMU: `LSM9DS1`

Storage implementation notes:
- SD card should use `SD.begin(BUILTIN_SDCARD)`
- QSPI NAND is available for internal logging/config/cache use
- measured NAND throughput is sufficient for flight logging, but slower on writes than SD

## Functional Requirements

### Core Flight Functions
- sample barometer, IMU, and GPS continuously during operation
- compute rocket altitude, vertical velocity, and basic attitude-related data
- maintain a flight-state machine suitable for hobby rocket launch, ascent, descent, and landing
- transmit live telemetry to the ground station over LoRa
- support recovery-oriented position reporting after landing or loss of high-rate telemetry relevance

### Sensors
- replace V7 altimeter support with `MS5607` as the primary barometric sensor
- replace V7 IMU support with `LSM9DS1`
- keep `GT-U7` GPS support
- detect and report sensor availability/failure at boot and during runtime
- define which sensors are mandatory vs optional:
  - LoRa: strongly preferred for mission use
  - barometer: mission-critical
  - IMU: required for full telemetry, but firmware should still boot if absent
  - GPS: optional for flight operation, required for full recovery telemetry

### Telemetry / Radio
- remain compatible with the current ground-station LoRa approach on `915 MHz`
- start from the existing `rocket_v7` / `ground_v8` packet flow
- evaluate whether to keep two-packet V7 format or move to a unified V8 packet
- if packet format changes, document the exact binary layout before implementation
- include sequence numbers in telemetry
- include signal/health/status fields needed by the new ground station
- include rocket battery voltage in telemetry

### Logging / Storage
- log flight data to the Teensy 4.1 built-in SD slot
- define whether QSPI NAND is used for:
  - fallback logging
  - buffered logging
  - config/settings storage
  - preflight test results
- current implementation decision:
  - SD is the readable/removable CSV log path
  - NAND is an internal redundant binary recorder
  - NAND records are V3-only in current firmware
  - legacy V1/V2 NAND export is not required and should not be reintroduced without a specific recovery need
  - NAND rotates oldest logs when full or over the configured file-count cap
- logging should not block control/telemetry timing
- flight logs should be recoverable even if telemetry is lost

### Configuration
- prepare for persistent settings storage
- settings should eventually include:
  - LoRa frequency / channel config
  - rocket name / callsign
  - telemetry rates
  - debug level
  - logging options
  - sensor calibration values

### Robustness
- boot in degraded mode if optional hardware is missing
- expose clear health flags for:
  - LoRa
  - SD
  - NAND
  - barometer
  - IMU
  - GPS
- do not hard-fail forever just because one non-critical device is missing

## Interfaces

### Ground Station Interface
- radio link must work with the new ground station version created alongside this project
- default RF target is a single fixed `915 MHz` channel
- telemetry must provide enough data for:
  - preflight status
  - in-flight monitoring
  - recovery position display
  - logging on the ground station

### Storage Interface
- SD is the primary removable flight-log medium
- NAND is an internal non-removable storage device and should not replace SD by default without an explicit design decision

## Open Technical Decisions

These are not resolved yet and should be settled before firmware implementation:
- final rocket packet format: keep V7 compatible packets or define a new V8 packet
- exact battery-voltage measurement hardware and ADC pin
- whether QSPI NAND is primary config storage, secondary log storage, or both
- exact `MS5607` library choice and calibration flow
- exact `LSM9DS1` library choice and update rate
- which data should be logged at high rate vs low rate
- whether pyrotechnic event outputs will be part of this version or remain out of scope

## Initial Project Layout

- `docs/` design notes, pinout, protocol, calibration docs
- `fw/` firmware project root
- `fw/src/` source files
- `fw/include/` headers/config structures
- `fw/lib/` local helper modules if needed
- `hw/` wiring notes, pin maps, board-level hardware docs
- `logs/` sample logs, decoded outputs, format examples
- `tools/` host-side parsers, log converters, test scripts

## Starting Point Recommendation

Implementation should start by reusing concepts from `rocket_v7`, but not by assuming the old sensor stack still applies.

Immediate first engineering tasks:
- define the new pinout for Teensy 4.1
- confirm libraries for `MS5607` and `LSM9DS1`
- define SD + NAND logging strategy
- freeze the telemetry packet contract with the new ground station

## Build / Upload Requirement

Keep build and upload instructions current in:

- [docs/BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)
- [rocket_v10 docs README](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/README.md)
- [rocket_v10 AI context](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/docs/AI_CONTEXT.md)

Current rocket target is `Teensy 4.1` / `teensy:avr:teensy41`. Use repo-local libraries from `.arduino-libs`; do not depend on `~/Documents/Arduino/libraries`.
