# Ground v9 Firmware README

Firmware:

- [GroundStationV9.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/GroundStationV9.ino)
- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v9/fw/GroundStationV9/config.h)

## Summary

This firmware runs on the ground controller and:

- receives rocket telemetry over LoRa
- reads the ground GPS on `Serial1`
- reads a BMP180 barometer over I2C
- renders the UI on a QVGA TFT display based on `ILI9341_t3`
- logs data to an external SPI SD card
- controls the RS-485 launch module through `Serial2`
- reads local switches, LEDs, and ground battery voltage

## MCU

The current ground controller target is:

- `Teensy 4.0`

This matters.

- The ground station is not the same target as the rocket controller.
- Rocket uses `Teensy 4.1`.
- Ground uses `Teensy 4.0`.

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
