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
  - `BOOT Ns`: the 60 s power-on inhibit is counting down
  - `INSERT PIN`: SAFE has not yet been observed since this boot
  - `SAFE`: the removable SAFE pin is inserted
  - `PAD SETTLE`: the barometric pad baseline is still settling
  - `SENSOR ERR`, `LOG ERR`, or `BATT CRIT`: a required readiness check failed
  - `VERTICAL` or `HOLD STILL`: pad orientation/motion is preventing verification
  - `ARMING Ns`: all checks pass and the continuous 10 s verification is counting down
  - `READY`: launch detection is armed
  - `LAUNCH CHECK`: transient launch evidence is being confirmed
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
- active rocket states received through status/event packets drive the ground
  FLIGHT/RECOVERY phase even when full flight packets are unavailable
- the ground pad-altitude baseline stops updating as soon as an active rocket
  flight state is received

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
D14 / A0         <-------> | Battery divider      |
TFT DC           <-------> | D15                  |
START A          <-------  | D16                  |
START B          <-------  | D17                  |
I2C SDA          <-------> | D18 / SDA            |
I2C SCL          <-------> | D19 / SCL            |
ARM A            <-------  | D20                  |
ARM B            <-------  | D21                  |
LED A            <-------> | D22                  |
LED B            <-------> | D23                  |
Buzzer           <-------> | D26                  |
TFT RST          <-------> | D28                  |
GPS TX           ------->  | D0 / RX1             |
GPS RX optional  <-------  | D1 / TX1             |
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
| ARM reminder buzzer | D26 | Piezo buzzer or transistor driver | Chirps when ARM is on/off; reminds while armed |
| Ground battery ADC | A0 | Divider midpoint | `100k / 22k`, measured A0-fit calibration |

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

Indicators:

| Indicator | Teensy pin | Wiring |
|---|---:|---|
| Channel A LED | `D22` | Pin -> resistor -> LED -> GND |
| Channel B LED | `D23` | Pin -> resistor -> LED -> GND |
| ARM reminder buzzer | `D26` | Small piezo buzzer to GND, or transistor/MOSFET driver for louder buzzer |

Buzzer behavior:

- With no fresh RS-485 Power-module link, raw A/B ARM switches are silent and
  cannot arm either lane.
- Arming while disconnected, or losing the link while armed, latches `REARM`.
  Return that channel to SAFE and ARM it again after the link is healthy.
- A link-qualified ARM edge produces the short rising chirp.
- A valid ARM state produces the heartbeat, faster after 60 seconds.
- START held while validly armed produces the solid high tone.
- A valid ARM-to-SAFE edge produces the lower safe chirp.

### 8. Ground battery measurement

The ground controller reads local battery voltage on `A0`. On Teensy 4.0 this overlaps with digital `D14`, so do not use `D14` for other peripherals in this build.

Configured divider:

- `R1 = 100k`
- `R2 = 22k`

Wiring:

```text
Battery + -> 100k -> A0 -> 22k -> GND
```

Notes:

- firmware assumes `ADC_REF_V = 3.3`
- intended for `1S`, `2S`, and `3S` Li-Po ground-station batteries
- firmware uses the measured A0 correction:
  - `Vbat = A0_V * 5.61783593 - 0.06534455`
- firmware samples A0 every `50 ms` and displays/logs a `40` sample moving average, about `2 s`
- Teensy ADC hardware averaging is also enabled with `32` samples per read
- measured regulator limit: when input falls below about `4.28 V`, the Teensy `3.3 V` rail starts to sag
- for the current power path, `1S` battery status is conservative:
  - warning below `4.45 V`
  - critical below `4.30 V`
- expected ADC voltage with this divider:
  - `4.2 V` battery -> about `0.76 V` on `A0`
  - `12.6 V` battery -> about `2.27 V` on `A0`
- measured fit points:
  - `12.20V -> 2.17V A0`
  - `11.32V -> 2.03V A0`
  - `10.26V -> 1.84V A0`
  - `8.95V -> 1.61V A0`
  - `7.50V -> 1.36V A0`
  - `5.62V -> 1.01V A0`
  - `4.62V -> 0.83V A0`
  - `3.50V -> 0.63V A0`

### 9. RTC backup battery

The Teensy 4.0 RTC backup battery connects to `VBAT` and `GND`. It is only for keeping the internal real-time clock alive when the controller loses main power; it does not power the whole ground station.

Time behavior:

- on boot, firmware reads the Teensy RTC
- if RTC time looks valid, SD logs can use timestamped filenames before GPS is ready
- when GPS date/time becomes valid, firmware updates the Teensy RTC from GPS
- log rows include `time_src=RTC`, `time_src=GPS`, or `time_src=NONE`
- RTC and SD log timestamps are treated as UTC
- the `[GND MODULE]` screen displays Central local time with US daylight-saving rules

After first GPS lock with the backup battery connected, the controller should keep usable time across power removal.

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
- the launch page appears automatically only for a link-qualified ARM state
- a raw ARM transition without a fresh RS-485 status link is invalid and silent
- link loss invalidates both arms; link recovery requires SAFE -> ARM again
- the Launch page displays `REARM` for an invalid raw ARM state
- firing is blocked if the RS-485 link is stale, the key is missing, or a fault is active
- the master-side fire timeout is `10 s` per lane
- ARM alone continues normal direct SD logging, so a long armed pad wait cannot
  fill RAM
- during START, acknowledged output, and the 3-second fire-sampling window, SD
  operations are deferred and rows are retained in a 64 KiB RAM buffer
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
| `HH:MM` | Central local time from RTC/GPS on the right side of the SD line, or `--:--` when no valid time is available. |

### `[READY]`

Shown automatically when rocket packets are present and the rocket has not launched.
The blue header shows `[ROCKET]` on the left and the rocket name on the right.

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

## Test-flight build `gv10.20260724f`

This build preserves standalone Ground launch operation: Rocket presence, link,
and READY are not launch interlocks. It:

- disables LoRa payload CRC to match Rocket `rv10.20260724f`
- restores the original SF7 / 125 kHz / CR 4:5 receiver profile
- validates packet versions, Rocket state, coordinates, altitude, battery, and
  launch-status ranges before using received data
- counts valid status packets as live Rocket link traffic, while periodic pyro
  configuration packets no longer mask missing flight/status/navigation data
- rejects stale TinyGPS++ fixes
- polls LoRa from the main loop instead of performing receive-side SPI work in
  the DIO0 interrupt callback
- services the RS-485 power path repeatedly around sensor, radio, timekeeper,
  logging, and TFT work, with the command period reduced from 150 ms to 100 ms
- records current and maximum RS-485 transmit gaps in `GND` and `PWR_FIRE` rows

The existing launch-critical RAM buffer and genuine-link-loss REARM behavior
remain in place.

### USB serial settings and SD service

Ground supports `STORAGE STATUS`, `SD MOUNT`, `SD LIST`, `SD INFO <filename>`,
`SD READ <filename> <offset> <length>`, and
`SD ERASE LOGS CONFIRM`. SD operations are rejected unless both ARM switches
are SAFE and both START buttons are released. Downloads use the same
checksummed/resumable protocol and
`rocket/rocket_v10/tools/flight_storage.py` client as Rocket:

```text
python3 ../../rocket/rocket_v10/tools/flight_storage.py \
  --port /dev/cu.usbmodem... list sd
python3 ../../rocket/rocket_v10/tools/flight_storage.py \
  --port /dev/cu.usbmodem... download sd ground_log0001.log ground_log0001.log
```

Connect at `115200` baud and terminate each command with Enter. `SHOW` reports
the active values, local display time, Rocket link freshness, packet age/counts,
and last RSSI; `HELP` prints examples. `SET` changes RAM immediately and `SAVE`
writes the current values to EEPROM. `DEFAULTS` restores compiled defaults in
RAM.

Supported settings are `TZ_OFFSET_MIN` (`-720..840`), `LORA_SF` (`6..12`),
`LORA_BW` (`62500`, `125000`, `250000`, or `500000`), `LORA_CR` (`5..8`), and
`LINK_LOST_MS` (`2000..60000`). RF settings must match Rocket. Example:

```text
SHOW
SET TZ_OFFSET_MIN -300
SET LINK_LOST_MS 10000
SAVE
```

The RTC can be set explicitly as local wall time or UTC:

```text
TIME LOCAL 2026-07-24 22:47:00
TIME UTC 2026-07-25 03:47:00
```

Ground remembers whether its RTC contains local or UTC time. GPS synchronization
stores UTC and applies `TZ_OFFSET_MIN` for display and filenames. Existing RTCs
that already contain local wall time are displayed directly, avoiding the old
double-offset error.

If the receiver starts without traffic or an established Rocket link becomes
stale, Ground periodically re-enters LoRa receive mode. A failed LoRa boot
initialization is retried every two seconds. `SHOW` reports `LORA_OK` and
`LORA_REARM_COUNT` so recovery is visible without a debug build.

## SD logging behavior

Ground logs sparse ground-only rows independently of rocket packets, plus richer rocket snapshot rows after the first rocket packet has been received.

Current cadence:

- ground-only: `GND` row every `GROUND_LOG_MS` (`30 s`), including sessions where no rocket computer is present
- before first rocket packet: no repeated `PAD` rows
- preflight with rocket link: `PAD` row every `PAD_PRELOG_MS` (`5 s`)
- flight: `FLG` row every `FLIGHT_LOG_MS` (`200 ms`)
- recovery: `NAV` row every `RECOVERY_LOG_MS` (`1 s`)
- lost link after rocket was seen: `LOST` row every `PAD_LOSTLOG_MS` (`30 s`)

`GND` rows include ground GPS, BMP180 barometric altitude, BMP180 temperature, ground GPS parser counters, ground-module battery voltage, power-module ignition voltage/current/status, RS-485 link status, whether a rocket packet has ever been seen, and `time_src` / `time_valid`.

Rocket snapshot rows include rocket state, altitude, velocity, GPS, ground GPS/baro/temperature, distance/bearing, RSSI, packet ages, receive/miss counts, rocket battery, rocket health, ground GPS parser counters, and `time_src` / `time_valid`.

Power-module logging is intentionally sparse. Normal RS-485 status frames are used for the screen but are not written repeatedly to SD. A `PWR_START` row is written only on the rising edge of Start A or Start B. It includes the event name, timestamp, ignition voltage, ground-module voltage, channel currents, key/presence/fault state, local arm/start state, power-module arm/on state, RS-485 link freshness, and `time_src` / `time_valid`.

After a Start A or Start B press, the ground station also writes `PWR_FIRE` rows every `PWR_FIRE_LOG_MS` (`100 ms`) for `PWR_FIRE_LOG_WINDOW_MS` (`3 s`). These rows capture the latest RS-485 power-module status during ignition and include `ia`, `ib`, the ground-observed `peak_ia` / `peak_ib` over that 3 second window, and `time_src` / `time_valid`.

During START, acknowledged output, and the 3-second fire-sampling window, these
rows and any concurrent ground/rocket rows remain in a 64 KiB RAM buffer. The
firmware performs no SD open, write, flush, close, or GPS-time file rotation
until that bounded launch-critical window ends. ARM alone continues normal SD
logging. If the buffer ever fills, a later `LOG_WARN` row records the number of
dropped rows.

Power-module current note: the existing power-module firmware reports live channel current while a lane is armed normally. If a lane enters overcurrent/short fault, the same current field reports the power module's stored fault peak. The ground station does not change power-module firmware; it logs the values available on the existing RS-485 status protocol.
