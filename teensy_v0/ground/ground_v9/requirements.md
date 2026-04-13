# Ground Station V9 Requirements

## Scope

This is the next ground-station version after `ground_v8`.

Primary goal:
- create a new Teensy 4.0 based ground station compatible with the existing `launch_controller` power station, while adapting the user interface and hardware support to the new ground-station hardware set.

This document is an initial requirements capture based on:
- existing code in `ground/ground_v8`
- supporting docs in `docs/docs_V8`
- current rocket baseline in `rocket/rocket_v7`
- the new hardware list provided for the ground station

## Version / Baseline Decision

Repository inspection shows:
- `ground_v8` is the main TFT-based branch and the best match for the current display/UI direction
- `ground_v8_debug` contains newer details in some hardware-facing areas, especially:
  - corrected ARM / START / LED pin mapping
  - simpler external-battery measurement assumptions used in that branch
  - extra profiling / serial diagnostics

For this new work, the baseline should be mixed deliberately:
- use `ground_v8` as the main architecture and TFT/UI reference
- use `ground_v8_debug` as the newer reference for button/control wiring and battery-monitoring assumptions where it is clearly ahead

## Existing Baseline To Preserve

Current `ground_v8` behavior to carry forward unless intentionally changed:
- Teensy-based handheld ground station
- LoRa receive station for rocket telemetry at `915 MHz`
- local GPS for pad location and recovery guidance
- local barometer support on the ground station
- SD logging
- multi-page UI with preflight, flight, recovery, launch, and signal/status views
- RS-485 master link to the power station / launch controller
- same protocol and safety model currently documented for the power module integration

Known V8 interface assumptions worth preserving:
- LoRa pins:
  - `CS=10`
  - `RST=9`
  - `DIO0=2`
- ground GPS on `Serial1`
- RS-485 power-module master on `Serial2`
- active-low operator inputs with internal pull-ups
- logging behavior tied to GPS time when available

Known newer hardware details from `ground_v8_debug` to carry forward:
- launch-control input mapping:
  - `PWR_ARM_A_PIN = 20`
  - `PWR_ARM_B_PIN = 21`
  - `PWR_START_A_PIN = 22`
  - `PWR_START_B_PIN = 23`
  - `PWR_LED_A_PIN = 16`
  - `PWR_LED_B_PIN = 17`
- external battery monitoring in that branch assumes a simple divider and direct status-level measurement rather than the more configurable divider constants used in `ground_v8`

## Compatibility Requirement With Existing Power Station

The existing power-station project in `/Users/k_pochkaev/github/launch_controller` is not to be changed.

Therefore Ground Station V9 must remain compatible with the current launch-controller / power-module behavior already documented in `docs/docs_V8/launch_controller_readme.md`.

Required compatibility points:
- keep the same RS-485 master/slave roles
- keep the same binary command/response framing unless intentionally validated otherwise
- preserve ARM / START semantics for both channels
- preserve safety-key, igniter-presence, fault, and current-status reporting
- preserve master-side timeouts and stale-link handling

Current documented protocol baseline:
- master to power: 6-byte frame
- power to master: 9-byte frame
- command bits for `ARM_A`, `START_A`, `ARM_B`, `START_B`
- returned bits for arm seen, lane on, key present, igniter presence, and fault

## New Ground Hardware

Confirmed hardware for the new ground station:
- Teensy 4.0
- LoRa RFM95 `915 MHz`
- GPS `NEO-M8N`
- RS-485 transceiver `MAX486 3.3V`
- barometer `BMP180`
- external `3.3V` SD card reader + SD card
- `2.2"` QVGA TFT display, `240x320`, ILI9341-style module assumed until verified
- 2 switch buttons
- 2 push buttons for launch
- 1 service button
- regulator to `3.3V`
- `3S` Li-Po battery

## Functional Requirements

### Core Ground Functions
- receive and decode rocket telemetry over LoRa
- maintain a clear operator display for preflight, flight, recovery, and link quality
- act as the RS-485 master for the unchanged power station
- log ground and rocket data to SD
- provide local GPS-based pad position and rocket bearing/distance information

### User Inputs
- support the currently available physical controls:
  - 2 switch buttons
  - 2 launch push buttons
  - 1 service button
- map controls so launch-related inputs remain separate from general UI navigation
- define debouncing and safety handling for all user-operated controls

### Display / UI
- target the current `240x320` TFT hardware path, not the OLED debug branch
- retain the major V8 page concepts:
  - preflight / pad status
  - live flight status
  - recovery / landed
  - launch controller status
  - signal / link status
- ensure the launch page exposes:
  - arm state
  - start state
  - key status
  - igniter presence
  - lane fault state
  - ignition battery / local battery

### Radio / Telemetry
- remain compatible with the current rocket telemetry flow initially
- be prepared to adapt to the new rocket flight computer version once the packet contract is updated
- support at minimum:
  - flight-state display
  - barometric altitude
  - velocity
  - GPS fix / sats / HDOP
  - rocket coordinates
  - rocket battery
  - RSSI / link age

### GPS / Navigation
- replace the previous ground GPS assumption with `NEO-M8N`
- use ground GPS for:
  - pad position
  - GPS time for stamped logs
  - bearing/distance to rocket
- firmware should still run if GPS is absent or does not have fix

### Sensors / Logging
- keep `BMP180` support as the ground barometer
- log at minimum the V8-style event families:
  - `PAD`
  - `FLG`
  - `NAV`
  - `LOST`
  - `PWR`
- add new log record types only after documenting them
- logging must not interfere with launch controls or radio reception

### Power / Battery
- support local battery monitoring for a `3S` Li-Po powered station
- start from the newer `ground_v8_debug` battery-monitoring assumption, then replace it with the real divider ratio for the current `3S` hardware
- confirm actual divider values and ADC scaling before firmware implementation
- display both local battery and ignition battery status

## Interfaces

### Power Station Interface
- unchanged external dependency
- Ground Station V9 must be the only side adapted as needed
- MAX486 should be treated as a 3.3 V RS-485 transceiver replacement for the previous MAX3485/MAX485-style assumptions

### Rocket Interface
- initial compatibility target is the existing rocket telemetry workflow from `rocket_v7`
- design should allow migration to the new rocket flight computer version without another full UI rewrite

## Open Technical Decisions

These are not resolved yet and should be settled before firmware implementation:
- exact Teensy pin map for the new buttons, TFT, SD reader, GPS, LoRa, and MAX486
- whether the TFT module is definitely `ILI9341` compatible and which library should be used
- final button-role mapping across:
  - 2 switches
  - 2 launch buttons
  - 1 service button
- exact ADC divider for local `3S` battery measurement
- whether the ground station should start from a copy of `ground_v8` with selected `ground_v8_debug` hardware updates, or a refactored modular rewrite with the same interfaces
- exact telemetry packet compatibility strategy during transition from `rocket_v7` to the new rocket version

## Initial Project Layout

- `docs/` design notes, protocol notes, UI sketches, pinout docs
- `fw/` firmware project root
- `fw/src/` source files
- `fw/include/` headers/config structures
- `fw/lib/` local helper modules if needed
- `hw/` wiring notes, pin maps, enclosure/control-panel docs
- `logs/` sample logs and decoded examples
- `tools/` support scripts, log parsers, packet decoders

## Starting Point Recommendation

Implementation should begin from the `ground_v8` architecture, while keeping the `launch_controller` protocol exactly compatible.

Immediate first engineering tasks:
- freeze the new ground-station pinout
- verify TFT controller/library
- define control mapping for the 5 physical inputs
- confirm the initial telemetry contract with `rocket_v8`
- document the exact RS-485 compatibility test procedure against the existing power station
