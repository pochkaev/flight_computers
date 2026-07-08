# Pro Mini OLED Receiver

This version uses `AltSoftSerial`, which is usually more reliable than `SoftwareSerial` on ATmega328P boards.

## Wiring

### ATtiny85 to Pro Mini

| ATtiny85 | Pro Mini |
|----------|----------|
| `PB1 / P1 / TX` | `D8` |
| `GND` | `GND` |

The ATtiny85 `baro_no_bl` firmware transmits telemetry at `9600 baud`.

`AltSoftSerial` uses fixed pins on Pro Mini / ATmega328P:

- `RX = D8`
- `TX = D9`

### OLED to Pro Mini

| OLED | Pro Mini |
|------|----------|
| `VCC` | `5V` or `3.3V` depending on your module |
| `GND` | `GND` |
| `SDA` | `A4` |
| `SCL` | `A5` |

## Library

Install the `AltSoftSerial` library in Arduino IDE before compiling.

## Debug

Open Serial Monitor at `115200` for raw byte logs and parser status.
