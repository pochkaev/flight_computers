# Wiring And Pinout

## Bus Assignment

This design uses:

- SPI for BMP390, microSD, and ICM-20602
- UART for GT-U7 GPS

## XIAO SAMD21 Pin Map

| XIAO pin | Function in project | Connects to |
| --- | --- | --- |
| `3V3` | Regulated 3.3 V rail | VCC for BMP390, ICM-20602, GPS, SD breakout |
| `GND` | Common ground | GND for all modules |
| `D4` | Spare GPIO | Unused in this version |
| `D5` | Spare GPIO | Unused in this version |
| `D6` | SPI chip select | BMP390 CS |
| `D7 / RX` | UART RX | GT-U7 TX |
| `D8 / SCK` | SPI clock | SD SCK, ICM-20602 SCLK, BMP390 SCK |
| `D9 / MISO` | SPI MISO | SD MISO, ICM-20602 SDO, BMP390 SDO |
| `D10 / MOSI` | SPI MOSI | SD MOSI, ICM-20602 SDA/SDI, BMP390 SDI |
| `D0` | SPI chip select | microSD CS |
| `D1` | SPI chip select | ICM-20602 CS |
| `D2` | User button input | Button to GND using `INPUT_PULLUP` |
| `D3` | Buzzer output | Active buzzer input or transistor base/gate |

The firmware uses the XIAO built-in LED for status, so no external LED wiring is needed.
The XIAO SAMD21 also aliases `D0/D1` as `A0/A1`, but this project uses the physical `0-10` pin labels everywhere.

## Module Wiring

### BMP390

- `VCC` -> `3V3`
- `GND` -> `GND`
- `SCK` -> `D8`
- `SDI` -> `D10`
- `SDO` -> `D9`
- `CS` -> `D6`
- `INT` -> not used in this version

### ICM-20602

- `VCC` -> `3V3`
- `GND` -> `GND`
- `SCLK` -> `D8`
- `SDI` -> `D10`
- `SDO` -> `D9`
- `CS` -> `D1`
- `INT` -> not used in this first version

### GT-U7 GPS

- `VCC` -> `3V3` if your module supports 3.3 V, otherwise use a known-safe regulator path for the module
- `GND` -> `GND`
- `TX` -> `D7`
- `RX` -> not connected in this version

### microSD Breakout

- `VCC` -> `3V3`
- `GND` -> `GND`
- `SCK` -> `D8`
- `MISO` -> `D9`
- `MOSI` -> `D10`
- `CS` -> `D0`

### Button

- One side -> `D2`
- Other side -> `GND`
- Use internal pull-up in firmware

### Buzzer

- Small active 3.3 V buzzer: control input -> `D3`
- Higher current buzzer: drive with transistor, not directly from GPIO

## Power Architecture

Recommended power chain:

```text
LiPo 3.7 V -> switch -> 3.3 V regulator -> XIAO 3V3 + all 3.3 V modules
```

Do not assume every GT-U7 or SD breakout is happy on raw battery voltage. Check the exact breakout you have.

## Physical Integration Notes

- Mount the BMP390 away from direct ejection gases and pressure spikes.
- Rigidly mount the IMU near the rocket centerline.
- Keep GPS antenna facing outward with clear sky view.
- Twist power leads and keep SPI wires short.
- Foam isolate the barometer but do not fully seal it.
