# Ground v10 Firmware README

Firmware:

- [GroundStationV10.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/fw/GroundStationV10/GroundStationV10.ino)
- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/fw/GroundStationV10/config.h)
- [AI_CONTEXT.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/docs/AI_CONTEXT.md)
- [NEXT_STEPS.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/docs/NEXT_STEPS.md)
- [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)

## Summary

This firmware runs on the ground controller and:

- receives rocket telemetry over LoRa
- reads the ground GPS on `Serial1`
- reads a BMP180 barometer over I2C
- renders the UI on a QVGA TFT display based on `ILI9341_t3`
- logs data to an external SPI SD card
- controls the RS-485 launch module through `Serial2`
- reads local switches, LEDs, and ground battery voltage

## Current v10 behavior

- UI refresh interval is `250 ms`
- dynamic UI page selection:
  - no rocket packets yet: `[GND MODULE]`
  - rocket packets present on pad: `[READY]`
  - launch controller armed: `[LAUNCH]`
  - rocket launched and not landed: `[FLIGHT]`
  - rocket landed: `[LANDED]`
  - rocket link stale for `LINK_LOST_MS`: `[LOST]`
- short button press manually cycles pages for `UI_MANUAL_TIMEOUT_MS`
- long button press for `UI_RESET_HOLD_MS` resets remembered rocket state and returns to the ground page
- all pages have a bottom status strip for link, battery, or system warning state
- `[READY]` focuses on launch readiness, system status, battery, GPS, link quality, and rocket health
- while the rocket is still in `PAD`, the large READY-page state shows the rocket launch gate:
  - `BOOT WAIT`: launch detection is inhibited after power-up
  - `SETTLING`: the rocket must stay still on the pad
  - `READY`: launch detection is armed
- `[LOST]` is recovery-focused and shows age, RSSI, battery, distance, bearing, and last known coordinates
- `SYS` ignores rocket GPS state; GPS is shown separately as informational status
- packet miss accounting ignores sequence rollbacks/resets from sender reboot
- SD logging does not write repeated PAD rows before the first rocket packet is ever received
- SD log names identify the module:
  - with GPS time: `ground_YYYYMMDD_HHMMSS_logNNNN.log`
  - without GPS time: `ground_logNNNN.log`
- rocket name shown in the display header can be changed from SD without recompiling:
  - file: `/ground_config.txt`
  - key: `rocket_name`
- received rocket identity packets override the local fallback name:
  - packet: `0x04`
  - source: rocket-side `/rocket_config.txt`
- ground GPS debug is available on serial when debug output is enabled

## SD config

The ground station reads an optional config file from the SD root at boot:

```text
/ground_config.txt
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

- max displayed length is 15 characters
- accepted characters are letters, numbers, `_`, `-`, and `.`
- lines can contain comments after `#`
- if the file is missing or invalid, firmware uses `DEFAULT_ROCKET_NAME`
- this config is only a local fallback; once a rocket identity packet is received, the ground display uses the name sent by the rocket

## MCU

The current ground controller target is:

- `Teensy 4.0`

This matters.

- The ground station is not the same target as the rocket controller.
- Rocket uses `Teensy 4.1`.
- Ground uses `Teensy 4.0`.

## Build and upload

Use the shared build/upload guide:

- [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)

Ground v10 target:

- board: `Teensy 4.0`
- FQBN: `teensy:avr:teensy40`
- repo-local libraries: `/Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-user/libraries`

Verified compile command:

```bash
mkdir -p /tmp/groundv10-build
arduino-cli compile --fqbn teensy:avr:teensy40 --libraries /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-user/libraries --build-path /tmp/groundv10-build /Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/fw/GroundStationV10
```

Upload from the fixed build directory:

```bash
arduino-cli upload --fqbn teensy:avr:teensy40 --port /dev/cu.usbmodem184901201 --input-dir /tmp/groundv10-build /Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/fw/GroundStationV10
```

Use `arduino-cli board list` before upload. If ground appears as a `usb:...` Teensy port, prefer that port over `/dev/cu.*`.

## Firmware check

I verified the ground station against the rocket firmware:

- LoRa packet types match the rocket sender:
  - `0x01` = flight packet
  - `0x02` = navigation packet
  - `0x03` = status packet
- payload layouts match the current rocket packet definitions
- the ground station uses the same health-bit meanings as the rocket status packet
- the launch page uses actual RS-485 power-module state, not stub data

## Electrical notes

- Teensy GPIO is `3.3 V` logic
- LoRa, TFT, SD, GPS, BMP180, and RS-485 interface hardware should all share a common ground
- the display, LoRa radio, and SD card share the same SPI bus
- because SPI is shared, each device must have its own separate `CS`

## Overall wiring diagram

```text
                           +----------------------+
                           |      Teensy 4.0      |
                           |                      |
LoRa DIO0        <-------> | D2                   |
TFT CS           <-------> | D3                   |
SD CS            <-------> | D4                   |
Page Button      <-------  | D5                   |
RS485 DE/RE      <-------> | D6                   |
RS485 RX         ------->  | D7 / RX2             |
RS485 TX         <-------  | D8 / TX2             |
LoRa RST         <-------> | D9                   |
LoRa CS          <-------> | D10                  |
SPI MOSI         <-------> | D11                  |
SPI MISO         <-------> | D12                  |
SPI SCK          <-------> | D13                  |
TFT DC           <-------> | D15                  |
START A          <-------  | D16                  |
START B          <-------  | D17                  |
I2C SDA          <-------> | D18 / SDA            |
I2C SCL          <-------> | D19 / SCL            |
ARM A            <-------  | D20                  |
ARM B            <-------  | D21                  |
LED A            <-------> | D22                  |
LED B            <-------> | D23                  |
TFT RST          <-------> | D28                  |
GPS TX           ------->  | D0 / RX1             |
GPS RX optional  <-------  | D1 / TX1             |
Battery divider  <-------> | A0                   |
                           | 3.3V / VIN / GND     |
                           +----------------------+
```

## Connection summary

| Function | Teensy pin / bus | External connection | Notes |
|---|---:|---|---|
| LoRa CS | D10 | RFM95 `NSS/CS` | Shared SPI bus |
| LoRa RST | D9 | RFM95 `RST` | Active-low reset |
| LoRa DIO0 | D2 | RFM95 `DIO0` | Interrupt / RX done |
| LoRa SPI | D11 / D12 / D13 | RFM95 `MOSI` / `MISO` / `SCK` | Shared with TFT and SD |
| GPS RX | D0 (`Serial1 RX`) | GT-U7 `TX` | Required |
| GPS TX | D1 (`Serial1 TX`) | GT-U7 `RX` | Optional |
| BMP180 SDA | D18 | BMP180 `SDA` | I2C |
| BMP180 SCL | D19 | BMP180 `SCL` | I2C |
| SD card CS | D4 | SD module `CS` | Shared SPI bus |
| Page button | D5 | Momentary button to GND | Active low |
| TFT CS | D3 | ILI9341 `CS` | Shared SPI bus |
| TFT DC | D15 | ILI9341 `DC` | Command/data |
| TFT RST | D28 | ILI9341 `RST` | Display reset |
| RS-485 DE/RE | D6 | MAX3485 `DE` + `RE` | HIGH = TX, LOW = RX |
| RS-485 RX | D7 (`Serial2 RX`) | MAX3485 `RO` | UART receive |
| RS-485 TX | D8 (`Serial2 TX`) | MAX3485 `DI` | UART transmit |
| ARM switch A | D20 | Switch to GND | Active low |
| ARM switch B | D21 | Switch to GND | Active low |
| START switch A | D16 | Switch to GND | Active low |
| START switch B | D17 | Switch to GND | Active low |
| Channel A LED | D22 | LED + resistor | Firmware-driven |
| Channel B LED | D23 | LED + resistor | Firmware-driven |
| Ground battery ADC | A0 | Divider midpoint | `330k / 100k` assumed in firmware |

## Device-by-device connections

### 1. LoRa radio: RFM95

Configured for:

- `915 MHz`
- SPI frequency `8 MHz`

Connections:

| RFM95 pin | Teensy 4.0 pin | Note |
|---|---:|---|
| `VCC` | `3.3 V` | Use `3.3 V`, not raw `5 V` logic |
| `GND` | `GND` | Common ground |
| `NSS/CS` | `D10` | Chip select |
| `RST` | `D9` | Reset |
| `DIO0` | `D2` | RX interrupt |
| `MOSI` | `D11` | SPI |
| `MISO` | `D12` | SPI |
| `SCK` | `D13` | SPI |

Diagram:

```text
RFM95                 Teensy 4.0
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

The firmware reads GPS data from `Serial1` at `9600` baud.

Connections:

| GT-U7 pin | Teensy 4.0 pin | Note |
|---|---:|---|
| `TX` | `D0 / RX1` | Required |
| `RX` | `D1 / TX1` | Optional |
| `GND` | `GND` | Common ground |
| `VCC` | module dependent | Check the actual GT-U7 breakout |

Diagram:

```text
GT-U7                 Teensy 4.0
-----                 ----------
TX       -----------> D0 / RX1
RX       <----------- D1 / TX1   optional
GND      -----------> GND
VCC      -----------> module supply
```

### 3. Ground barometer: BMP180

Connections:

| BMP180 pin | Teensy 4.0 pin | Note |
|---|---:|---|
| `SDA` | `D18 / SDA` | I2C |
| `SCL` | `D19 / SCL` | I2C |
| `GND` | `GND` | Common ground |
| `VCC` | `3.3 V` or breakout supply | Check the specific breakout |

Diagram:

```text
BMP180                Teensy 4.0
------                ----------
SDA      <--------->  D18 / SDA
SCL      <--------->  D19 / SCL
GND      -----------> GND
VCC      -----------> sensor supply
```

### 4. QVGA display: ILI9341 TFT

The UI code uses `ILI9341_t3` and targets a `240x320` QVGA display.

Important:

- this display shares SPI with LoRa and SD
- `CS` must be separate from LoRa and SD chip selects
- the display should be `3.3 V` logic compatible

Required display pins from firmware:

| ILI9341 pin | Teensy 4.0 pin | Note |
|---|---:|---|
| `VCC` | display module supply | Usually `3.3 V`, check your module |
| `GND` | `GND` | Common ground |
| `CS` | `D3` | Display chip select |
| `DC` | `D15` | Data/command |
| `RST` | `D28` | Display reset |
| `MOSI` | `D11` | Shared SPI MOSI |
| `MISO` | `D12` | Shared SPI MISO if module uses it |
| `SCK` | `D13` | Shared SPI clock |
| `LED` / `BL` | display backlight supply | Module-specific |

Recommended wiring:

```text
ILI9341 QVGA          Teensy 4.0
-------------         ----------
VCC      -----------> 3.3V or module supply
GND      -----------> GND
CS       -----------> D3
DC       -----------> D15
RST      -----------> D28
MOSI     -----------> D11
MISO     <----------- D12   if connected on module
SCK      -----------> D13
LED/BL   -----------> backlight supply
```

Notes for the display:

- if your module has `SDO`, connect it to `MISO`
- if your module does not use `MISO`, it may be left unconnected
- if the module has `LED` or `BL`, confirm whether it expects direct `3.3 V` or a resistor/transistor driver
- many ILI9341 boards are physically called `2.2"`, `2.4"`, or `QVGA TFT`; what matters here is the `ILI9341` controller and the current pin mapping

### 5. SD card module

The ground station uses an external SPI SD card module.

Connections:

| SD module pin | Teensy 4.0 pin | Note |
|---|---:|---|
| `CS` | `D4` | Card select |
| `MOSI` | `D11` | Shared SPI bus |
| `MISO` | `D12` | Shared SPI bus |
| `SCK` | `D13` | Shared SPI bus |
| `VCC` | module dependent | Prefer `3.3 V` logic-compatible hardware |
| `GND` | `GND` | Common ground |

Diagram:

```text
SD module             Teensy 4.0
---------             ----------
CS       -----------> D4
MOSI     -----------> D11
MISO     <----------- D12
SCK      -----------> D13
VCC      -----------> module supply
GND      -----------> GND
```

### 6. RS-485 launch controller interface

The ground station talks to the power/launch module through a MAX3485 or similar `3.3 V` RS-485 transceiver.

Connections:

| Ground side | Connects to | Note |
|---|---|---|
| `D8 / Serial2 TX` | MAX3485 `DI` | UART TX |
| `D7 / Serial2 RX` | MAX3485 `RO` | UART RX |
| `D6` | MAX3485 `DE` + `RE` tied together | Direction control |
| MAX3485 `A` / `B` | RS-485 bus `A` / `B` | Differential pair |
| `3.3 V` | MAX3485 `VCC` | Use 3.3 V transceiver |
| `GND` | MAX3485 `GND` | Common ground |

Diagram:

```text
Teensy 4.0            MAX3485 / MAX485
-----------           ----------------
D8 / TX2   ---------> DI
D7 / RX2   <--------- RO
D6         ---------> DE + RE
3.3V       ---------> VCC
GND        ---------> GND
A/B        <-------> RS-485 bus
```

### 7. Local controls and indicators

Switches:

| Control | Teensy pin | Wiring |
|---|---:|---|
| Page button | `D5` | Momentary switch to `GND` |
| ARM A | `D20` | Switch to `GND` |
| ARM B | `D21` | Switch to `GND` |
| START A | `D16` | Switch to `GND` |
| START B | `D17` | Switch to `GND` |

All these inputs are active low and use internal pull-ups in firmware.

LEDs:

| Indicator | Teensy pin | Wiring |
|---|---:|---|
| Channel A LED | `D22` | Pin -> resistor -> LED -> GND |
| Channel B LED | `D23` | Pin -> resistor -> LED -> GND |

### 8. Ground battery measurement

The ground controller reads local battery voltage on `A0`.

Configured divider:

- `R1 = 330k`
- `R2 = 100k`

Wiring:

```text
Battery + -> 330k -> A0 -> 100k -> GND
```

Notes:

- firmware assumes `ADC_REF_V = 3.3`
- intended for a ground-station battery, documented as a `3S Li-Po` path
- current firmware also applies a calibration factor:
  - `GND_VBAT_CAL_FACTOR = 0.8527132`
  - based on measured `12.10 V` real vs `14.19 V` indicated before calibration

## Shared SPI warning

These three devices share the same SPI bus:

- LoRa radio
- QVGA TFT display
- external SD card

That means:

- every device must have a unique `CS`
- all unused `CS` lines should stay inactive
- bad wiring on one SPI device can break the others

Current chip selects:

- TFT `CS = D3`
- SD `CS = D4`
- LoRa `CS = D10`

## Behavior notes

- ARM and START inputs are active low
- the launch page appears automatically when either ARM switch turns on
- firing is blocked if the RS-485 link is stale, the key is missing, or a fault is active
- the master-side fire timeout is `10 s` per lane
- the current safer build reduces debug output and rate-limits flight logging to reduce shared-bus load

## Display screens and field meanings

The display is `320x240` landscape. Pages switch automatically unless a short page-button press has started a temporary manual override. The bottom 20 px are reserved for the status strip.

### Common bottom status strip

This strip appears on every page.

| Text | Meaning |
|---|---|
| `NO ROCKET LINK` | Ground has not received any rocket packet since reset/boot. |
| `LINK OK RSSI n` | Rocket packets are fresh; `n` is the last LoRa RSSI in dBm. Less negative is stronger. |
| `LINK LOST x.xs` | Last rocket packet is older than `LINK_LOST_MS`; `x.xs` is packet age. |
| `BATT WARN v.vvV` | Rocket battery is below the warning threshold. |
| `BATT CRIT v.vvV` | Rocket battery is below the critical threshold. Do not fly. |
| `SYS WARN` | Rocket link is fresh, but IMU, barometer, SD, NAND, or log health is degraded. |

### `[GND MODULE]`

Shown when no rocket packets have been received yet, or when selected manually.

| Field | Meaning |
|---|---|
| `BATT` | Ground-station battery voltage from `A0`. |
| `GPS nSV` | Ground GPS satellite count. |
| `ALT G` | Ground GPS altitude in meters. |
| `B` | Ground BMP180 barometric altitude in meters. |
| `TEMP` | BMP180 temperature in C. |
| `PRES` | BMP180 pressure in hPa. |
| `LINK` | Rocket link state: `---`, `OK`, or `LOST`. |
| `RSSI` | Last received rocket LoRa RSSI in dBm. |
| `RX F/N/S` | Receive rate per second for Flight, Nav, and Status packets. |
| `MISS` | Missed sequence counts for Flight, Nav, and Status packets. |
| `SD` | Current log file index and line count, or SD status. |

### `[READY]`

Shown automatically when rocket packets are present and the rocket has not launched.

| Field | Meaning |
|---|---|
| Large state (`BOOT WAIT`, `SETTLING`, `READY`, etc.) | Rocket launch-readiness gate while the rocket is still in `PAD`; otherwise the current flight state. |
| `SYS` | Overall rocket status: `OK`, `WARN`, `CRIT`, or `FAULT`. GPS is not included in this summary. |
| `BATT` | Rocket battery voltage from the status packet. Color shows OK/WARN/CRIT. |
| `GPS nSV` | Rocket GPS satellite count. |
| `H` | Rocket GPS HDOP. Lower is better; high values such as `25.5` mean weak/unusable GPS geometry. |
| `LINK` | Rocket packet freshness: `OK`, `LOST`, or `---`. |
| `RSSI` | Last received rocket LoRa RSSI in dBm. |
| `AGE` | Seconds since the last rocket packet. |
| `AGL` | Barometric altitude above the captured pad/base altitude. Small sanity check on READY. |
| `VEL` | Rocket vertical velocity estimate in m/s. Small sanity check on READY. |
| `BARO` | Rocket barometric altitude in meters. |
| `RX F/N/S` | Receive rate per second for Flight, Nav, and Status packets. |
| `IMU` | Rocket IMU health from status telemetry. |
| `BARO` health label | Rocket barometer health from status telemetry. |
| `LOG` | Rocket aggregate logging health. |
| `SD` | Rocket SD health. |
| `NAND` | Rocket NAND health. |

### `[LAUNCH]`

Shown automatically when the launch controller is armed.

| Field | Meaning |
|---|---|
| `IGN` | Launch/ignition module battery voltage reported over RS-485. |
| `LNK` | RS-485 power-module link freshness. |
| `A` / `B` | Channel A/B arm state: `ARM`, `SAFE`, or `LOCK`. |
| `[ON]` | Output is commanded on for that channel. |
| `I` | Channel current in amps. |
| `(OK)` / `(--)` | Presence/continuity result for that channel. |
| `FAULT` | Power module reported a fault condition. |
| `RS485 RX` | Power-module packet receive rate per second. |

### `[FLIGHT]`

Shown automatically after rocket launch and before landed state.

| Field | Meaning |
|---|---|
| Large `m AGL` | Rocket barometric altitude above captured pad/base altitude. |
| `V` | Rocket vertical velocity in m/s. |
| `RSSI` | Last received rocket LoRa RSSI in dBm. |
| `D` | Distance from ground GPS position to rocket GPS position in meters. Shows `---` if not available. |
| `BRG` | Bearing from ground station to rocket in degrees. Shows `---` if not available. |
| `AGE F/N/S` | Seconds since last Flight, Nav, and Status packet. |

### `[LANDED]`

Shown automatically after the rocket reports landed.

| Field | Meaning |
|---|---|
| `MAX ALT` | Maximum rocket barometric altitude observed by ground during the flight. |
| `MAX V` | Maximum vertical velocity observed by ground during the flight. |
| `ALT G` | Last rocket GPS altitude in meters. |
| `B` | Last rocket barometric altitude in meters. |
| `LAT` | Last rocket GPS latitude. |
| `LON` | Last rocket GPS longitude. |

### `[LOST]`

Shown automatically when the rocket link becomes stale after at least one packet was received. This page is optimized for recovery.

| Field | Meaning |
|---|---|
| `AGE` | Seconds since the last rocket packet. |
| `RSSI` | RSSI from the last received rocket packet. |
| `BATT` | Last rocket battery voltage/status. |
| `GPS nSV` | Last rocket GPS satellite count. |
| `DIST` | Distance from ground GPS position to last rocket GPS position, if both are valid. |
| `BRG` | Bearing from ground station to last rocket GPS position, if both are valid. |
| `LAT` | Last known rocket GPS latitude. |
| `LON` | Last known rocket GPS longitude. |
| `ALT` | Last rocket GPS altitude in meters. |
| `AGL` | Last barometric altitude above pad/base altitude. |
| `HOLD BTN 3S RESET` | Hold the page button for 3 seconds to clear stale rocket state. |

### `[SIGNAL]`

Manual diagnostic page for link and navigation quality.

| Field | Meaning |
|---|---|
| `RSSI` | Last received rocket LoRa RSSI in dBm. |
| Bar graph | Visual RSSI strength from about `-120` to `-40` dBm. |
| `DIST` | Distance from ground GPS to rocket GPS in meters. |
| `BEAR` | Bearing from ground station to rocket in degrees. |
| `GPS nSV` | Rocket GPS satellite count. |
| `HDOP` | Rocket GPS horizontal dilution of precision. Lower is better. |

## SD logging behavior

Ground logs rich snapshot rows after the first rocket packet has been received.

Current cadence:

- before first rocket packet: no repeated PAD rows
- preflight with rocket link: `PAD` row every `PAD_PRELOG_MS` (`5 s`)
- flight: `FLG` row every `FLIGHT_LOG_MS` (`200 ms`)
- recovery: `NAV` row every `RECOVERY_LOG_MS` (`1 s`)
- lost link after rocket was seen: `LOST` row every `PAD_LOSTLOG_MS` (`30 s`)

Rows include rocket state, altitude, velocity, GPS, ground GPS/baro, distance/bearing, RSSI, packet ages, receive/miss counts, rocket battery, rocket health, and ground GPS parser counters.

Power-module logging is intentionally sparse. Normal RS-485 status frames are used for the screen but are not written repeatedly to SD. A `PWR_START` row is written only on the rising edge of Start A or Start B. It includes the event name, timestamp, ignition voltage, ground-module voltage, channel currents, key/presence/fault state, local arm/start state, power-module arm/on state, and RS-485 link freshness.
