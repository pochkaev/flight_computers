# Uno OLED Receiver

Receiver sketch for Arduino Uno to read the ATtiny85 barometer stream and show it on an I2C OLED.

This version uses `AltSoftSerial`, which is usually more reliable than `SoftwareSerial` on Uno for marginal UART timing.

## Wiring

### ATtiny85 to Uno

| ATtiny85 | Uno |
|----------|-----|
| `PB1 / P1 / TX` | `D8` |
| `GND` | `GND` |

The ATtiny85 `baro_no_bl` firmware transmits telemetry at `9600 baud`.

`AltSoftSerial` uses fixed Uno pins:

- `RX = D8`
- `TX = D9`

### OLED to Uno

| OLED | Uno |
|------|-----|
| `VCC` | `5V` or `3.3V` depending on your module |
| `GND` | `GND` |
| `SDA` | `A4` |
| `SCL` | `A5` |

## Serial debug

Open Serial Monitor at `115200`.

The sketch prints:

- startup info
- every received byte from the ATtiny85 on `D8`
- completed lines
- parse success/failure
- heartbeat once per second

## Library

Install the `AltSoftSerial` library in Arduino IDE before compiling.

## OLED controller

Default is `SH1106`. If your display stays blank, change:

```cpp
#define OLED_IS_SH1106 1
```

to:

```cpp
#define OLED_IS_SH1106 0
```
