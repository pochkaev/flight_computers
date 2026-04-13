# ATTiny85 Rocket Altimeter (MS5607)

This project is a lightweight rocket altimeter based on an **ATTiny85** microcontroller and an **MS5607** barometric pressure sensor.
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

### 2. MS5607 Sensor Module
- I2C barometric pressure sensor
- Common module addresses: **0x76** or **0x77**
- Many breakouts are **3.3 V only**: verify your module before powering from 5 V

### 3. Power Supply
A single‑cell **3.7 V LiPo battery** powers:
- ATTiny85 directly
- MS5607 module

---

## Pinout / Connections

### ATTiny85 → MS5607

| ATTiny Pin | Function | MS5607 Pin |
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

### ATTiny85 → Arduino Nano 33 BLE Receiver

Use this wiring with the receiver sketch in [`nano33ble_oled_receiver`](./nano33ble_oled_receiver).

| ATTiny85 | Function | Nano 33 BLE |
|----------|----------|-------------|
| P1 (PB1) | TX telemetry @ 9600 | D0 / RX |
| GND      | Ground | GND |

Important:

- Nano 33 BLE is **3.3 V only**
- Nano 33 BLE input pins are **not 5 V tolerant**
- If the ATTiny85 TX output is above `3.3 V`, use a resistor divider or level shifter before Nano 33 BLE `D0/RX`

### OLED → Arduino Nano 33 BLE

| OLED | Function | Nano 33 BLE |
|------|----------|-------------|
| VCC  | Power    | 3V3         |
| GND  | Ground   | GND         |
| SDA  | I2C data | A4 / SDA    |
| SCL  | I2C clock| A5 / SCL    |

The Nano 33 BLE receiver sketch uses:

- `Serial1` at `9600` for baro telemetry
- `Serial` at `115200` for USB debug

---

### Arduino Nano as ISP → ATTiny85

Use this wiring when flashing the `baro_no_bl` sketch or when burning a bootloader.

| Arduino Nano | ATTiny85 signal | ATTiny85 label | Physical pin |
|--------------|-----------------|----------------|--------------|
| D10          | RESET           | P5 / PB5       | 1            |
| D11          | MOSI            | P0 / PB0       | 5            |
| D12          | MISO            | P1 / PB1       | 6            |
| D13          | SCK             | P2 / PB2       | 7            |
| 5V           | VCC             | VCC            | 8            |
| GND          | GND             | GND            | 4            |

`D10` on the Nano must connect to `PB5` (`P5`) on the ATTiny85 because that is the reset pin used by ISP.

ATTiny85 DIP-8 layout:

```text
        ATtiny85
     +---\/---+
P5 1 |*      | 8 VCC
P3 2 |       | 7 P2 / SCK
P4 3 |       | 6 P1 / MISO
GND4 |       | 5 P0 / MOSI
     +-------+
```

ATtiny85 pin labels used in this project:

| Label | AVR port | ISP role |
|-------|----------|----------|
| P0    | PB0      | MOSI     |
| P1    | PB1      | MISO     |
| P2    | PB2      | SCK      |
| P3    | PB3      | not used |
| P4    | PB4      | not used |
| P5    | PB5      | RESET    |

By default, the hardware SPI pins `MISO`, `MOSI`, and `SCK` are used to communicate with the target. On the Nano, these pins are available both on `D11/D12/D13` and on the ICSP/SPI header:

```text
              MISO  . . 5V
              SCK   . . MOSI
                    . . GND
```

You can use either the `D11/D12/D13` pins or the Nano ICSP header for SPI. They are the same signals.

Recommended Arduino IDE setup:

1. Upload the `ArduinoISP` example sketch to the Nano.
2. Add a `10 uF` capacitor between Nano `RESET` and `GND`.
3. Select `Programmer -> Arduino as ISP`.
4. Use `Sketch -> Upload Using Programmer` to flash a no-bootloader ATTiny85.
5. Use `Tools -> Burn Bootloader` if you want to install Micronucleus first.

---

### Power Button (Main Power Switch)

| Connection | Description |
|------------|-------------|
| LiPo + → Button Pin 1 | Battery positive enters switch |
| Button Pin 2 → ATTiny VCC & MS5607 VIN | Outputs power to entire system |
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
3. Or connect the telemetry line to the Nano 33 BLE receiver and display values on the OLED  
4. After flight, check **MAX altitude**  
5. Hold reset button during power-on to clear MAX value  

---

## Notes

- Update `SEA_LEVEL_PRESSURE` in code for best accuracy  
- Verify whether your MS5607 breakout is 3.3 V only or regulator-equipped before wiring power  
- Ideal for small hobby rockets due to low weight & power use  
