# Rocket v8 Firmware README

Firmware:

- [RocketV8.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/RocketV8.ino)
- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/config.h)

## Summary

This firmware runs on the rocket flight computer and:

- reads the MS5607 barometer over I2C
- reads the optional LSM9DS1 IMU over I2C
- reads the GT-U7 GPS on `Serial1`
- measures battery voltage on `A0`
- logs flight data to the Teensy 4.1 built-in SD card
- initializes optional QSPI NAND memory through `LittleFS_QPINAND`
- sends flight, navigation, and status telemetry over LoRa

## Runtime model

The rocket firmware is now structured so USB serial is optional and not part of the flight-critical path.

- GPS parsing runs continuously in the main loop
- IMU sampling runs at `100 Hz` via `IMU_UPDATE_MS = 10`
- barometer/state update runs at `20 Hz` via `BARO_UPDATE_MS = 50`
- battery update runs at `10 Hz` via `BATT_UPDATE_MS = 100`
- flight telemetry runs at `5 Hz`
- navigation telemetry runs at `1 Hz`
- status telemetry runs at `0.5 Hz`
- SD CSV logging runs at `5 Hz`
- NAND binary logging runs at `5 Hz`
- SD and NAND flush run at `1 Hz`
- serial debug runs only when `SERIAL_DEBUG_LEVEL > 0`

This is intended for flight use:

- `SERIAL_DEBUG_LEVEL = 0` for flight
- `SERIAL_DEBUG_LEVEL = 1` for quick bench checks
- `SERIAL_DEBUG_LEVEL = 2` for full bench debugging

Serial output is diagnostic only. LoRa telemetry, sensor updates, and storage logging do not depend on a USB serial connection.

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

- slow blink: boot and initialization
- heartbeat pulse: normal ready state
- fast blink: service mode running from SD command file
- solid on: service mode completed successfully
- 2 short blinks repeating: SD failure
- 3 short blinks repeating: NAND failure
- fast 250 ms blink: general hardware fault

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
- packet layouts match the ground-side parser
- both sides use `915 MHz`
- battery status is sent in the status packet as `batt_mv`
- the battery OK flag is pack-aware:
  - `1S`: battery warning below `3.4 V`, critical below `3.2 V`
  - `2S`: battery warning below `6.8 V`, critical below `6.4 V`

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
| GPS RX | D0 (`Serial1 RX`) | GT-U7 `TX` | `9600` baud |
| GPS TX | D1 (`Serial1 TX`) | GT-U7 `RX` | Optional |
| I2C SDA | D18 | MS5607 `SDA`, LSM9DS1 `SDA` | Shared I2C bus |
| I2C SCL | D19 | MS5607 `SCL`, LSM9DS1 `SCL` | Shared I2C bus |
| Battery ADC | A0 | Divider midpoint | See battery divider section |
| QSPI NAND memory | Native QSPI interface | `LittleFS_QPINAND` device | Optional, see memory section |
| SD card | Built-in SD | Teensy 4.1 on-board microSD | `SD.begin(BUILTIN_SDCARD)` |

## Device-by-device connections

### 1. LoRa radio: RFM95

Firmware references:

- [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/config.h)
- [RocketV8.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/RocketV8.ino)

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
- if the Adafruit LSM9DS1 library is not installed, IMU support is compiled out

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
  - file names like `flight1.csv`
- NAND:
  - resilient onboard recorder
  - file names like `/flt0001.bin`
  - fixed-size binary records for later export

Important detail:

- the firmware does not assign regular SPI pins for the NAND device
- that means this memory is not a generic `MOSI/MISO/SCK/CS` SPI module in the current code
- it is expected to be connected through the native Teensy-supported `QPINAND/QSPI` interface used by `LittleFS_QPINAND`
- based on the working test project, the intended NAND device is `W25N01G` on the Teensy 4.1 native QSPI pads, using `CS1 / pin 51`

So for this firmware:

- LoRa uses the normal SPI pins `D11/D12/D13` plus `D10/D9/D2`
- NAND memory is a separate native memory interface on the Teensy 4.1 QSPI pads
- SD logging uses the built-in Teensy 4.1 SD hardware

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
require_nand_ok=1
require_sd_ok=1
```

Aliases currently accepted:

- `copy_to_sd` or `export_to_sd`
- `clean_nand` or `erase_nand_after_export`

Behavior:

- `copy_to_sd=1` exports NAND binary logs to CSV files on SD
- `clean_nand=1` removes NAND flight log files only after a successful export
- result file is written to `/nand_ops_result.txt`
- command file `/nand_ops.txt` is removed only if the requested operation succeeds

Important guard:

- NAND erase is not performed unless export was requested and export succeeded

This is intentional to reduce risk of deleting the only copy of flight data.

Connection note:

- use the same wiring that works in [teensy41_nand_sd_test.ino](/Users/k_pochkaev/github/debug_projects/teensy41_nand_sd_test/teensy41_nand_sd_test.ino)
- this rocket firmware uses the same `LittleFS_QPINAND` class and initialization style
- if the memory is not wired as native Teensy 4.1 QSPI NAND, `qspiNand.begin()` is expected to fail

If your memory device is not `W25N01G` on the Teensy 4.1 QSPI pads, the current README and firmware assumptions are no longer valid.

## Battery voltage measurement

The battery input is measured through a resistor divider into `A0`.

Firmware values from [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/config.h):

- `VBAT_PIN = A0`
- `VBAT_R1_OHMS = 330000`
- `VBAT_R2_OHMS = 100000`
- `ADC_REF_V = 3.3`
- `ADC_MAX_COUNTS = 4095`

Divider wiring:

- battery `+` -> `330k` -> `A0` -> `100k` -> GND

ASCII diagram:

```text
Battery +
   |
  330k
   |
   +-------> A0 (Teensy ADC input)
   |
  100k
   |
  GND
```

Firmware conversion from [RocketV8.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/RocketV8.ino):

```cpp
int raw = analogRead(VBAT_PIN);
float vPin = (float)raw * ADC_REF_V / ADC_MAX_COUNTS;
return vPin * (VBAT_R1_OHMS + VBAT_R2_OHMS) / VBAT_R2_OHMS;
```

So the firmware computes:

```text
Vbat = ADC_voltage * (330k + 100k) / 100k
Vbat = ADC_voltage * 4.3
```

Practical implications:

- divider ratio is `4.3:1`
- maximum measurable battery voltage before `A0` reaches `3.3 V` is about `14.19 V`
- that is fine for a `2S` or `3S` LiPo input range
- the health flag threshold is `6.8 V`
- at `8.4 V` battery input, `A0` sees about `1.95 V`
- at `12.6 V` battery input, `A0` sees about `2.93 V`

The ADC is configured to 12-bit in setup, and `A0` is explicitly set as input in [RocketV8.ino](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/RocketV8.ino).

### Battery and power recommendations

- Keep the battery negative tied to Teensy ground.
- Do not connect raw battery voltage directly to `A0`.
- If you change either resistor value, update `VBAT_R1_OHMS` and `VBAT_R2_OHMS` in [config.h](/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v8/fw/RocketV8/config.h).
- If your analog reference or board ADC behavior differs, update `ADC_REF_V` and verify the measured voltage with a multimeter.

## Storage

The rocket logs to the built-in Teensy 4.1 SD slot. In addition, the firmware can detect optional QSPI NAND memory through `LittleFS_QPINAND`.

Diagram:

```text
RocketV8 firmware
      |
      +----> Teensy 4.1 built-in microSD slot
      |
      +----> optional native QSPI NAND
```

## Sensor notes

- MS5607 is probed at `0x76` and then `0x77`
- LSM9DS1 is optional at compile time; if the library is missing the firmware still runs
- GPS is read continuously and nav packets are sent once per second
- flight data is logged to CSV on the built-in SD card
