# ATTiny85 Rocket Altimeter (BMP150/BMP180)

This project is a lightweight rocket altimeter based on an **ATTiny85** microcontroller and a **BMP150/BMP180** barometric pressure sensor.
It measures **temperature**, **pressure**, **altitude**, and stores **maximum altitude** in EEPROM.
A reset button allows clearing the stored altitude at power‑on.

---

## Features

- Reads barometric pressure and temperature
- Computes altitude using integer math
- Stores **maximum altitude** in EEPROM
- Reset button to clear max altitude (hold during power-up)
- Powered directly from a **3.7 V LiPo**
- Outputs telemetry via Software Serial @ **9600 baud**

---

## Hardware

### 1. ATTiny85 (Digispark or bare‑chip, ISP flashed)
- Operating voltage: **3.0–5.5 V**
- Internal 16 MHz oscillator
- Compatible with TinyWireM (I²C)

### 2. BMP150 / BMP180 Sensor Module
Most BMP modules contain:
- **3.3 V regulator**
- **I²C level shifters**
- Safe for input voltage **3.3–5 V**

### 3. Power Supply
A single‑cell **3.7 V LiPo battery** powers:
- ATTiny85 directly
- BMP module via its onboard regulator

---

## Pinout / Connections

### ATTiny85 → BMP150/BMP180

| ATTiny Pin | Function | BMP Pin |
|------------|----------|---------|
| P0 (PB0)   | SDA      | SDA     |
| P2 (PB2)   | SCL      | SCL     |
| VCC        | Power    | VIN     |
| GND        | Ground   | GND     |

---

### ATTiny85 → USB‑UART Adapter

| ATTiny Pin | Function | USB‑UART Pin |
|------------|----------|---------------|
| P1 (PB1)   | TX (Software Serial) | RX |
| GND        | Ground | GND |

---

### Power Button (Main Power Switch)

| Connection | Description |
|------------|-------------|
| LiPo + → Button Pin 1 | Battery positive enters switch |
| Button Pin 2 → ATTiny VCC & BMP VIN | Outputs power to entire system |
| LiPo – → GND | Common ground |

---

### Reset Max Altitude Button

| ATTiny Pin | Button Pin | Description |
|------------|-------------|-------------|
| P3 (PB3)   | Pin 1       | Reads LOW at startup → clears EEPROM max altitude |
| GND        | Pin 2       | Ground |

Hold this button while powering on to reset stored max altitude.

---

## Serial Output Format

```
T=23.90C P=99281Pa A=171m MAX=171m
```

Meaning:
- **T** — temperature (°C)
- **P** — pressure in Pascals
- **A** — current altitude (m)
- **MAX** — highest altitude stored in EEPROM

---

## Usage

1. Power device with LiPo battery  
2. Observe live readings via USB‑UART @ **9600 baud**  
3. After flight, check **MAX altitude**  
4. Hold reset button during power-on to clear MAX value  

---

## Notes

- Update `SEA_LEVEL_PRESSURE` in code for best accuracy  
- Ensure BMP module has built‑in regulator (most breakout boards do)  
- Ideal for small hobby rockets due to low weight & power use  
