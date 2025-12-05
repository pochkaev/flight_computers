# Ground Station V8 – Pinout Summary

## Teensy Ground Station (Teensy 4.0)

| Function                    | Teensy Pin | Notes                             |
|----------------------------|-----------:|-----------------------------------|
| LoRa CS                    | D10        | `LORA_CS_PIN`                     |
| LoRa RST                   | D9         | `LORA_RST_PIN`                    |
| LoRa DIO0                  | D2         | `LORA_DIO0_PIN`                   |
| SD card CS                 | D4         | `SD_CS_PIN`                       |
| OLED page button           | D5         | Active‑low, `BUTTON_PIN`         |
| I2C SDA                    | D18 (SDA)  | OLED + BMP180                     |
| I2C SCL                    | D19 (SCL)  | OLED + BMP180                     |
| GPS RX (from GT‑U7 TX)     | D0 (RX1)   | `Serial1 RX`                      |
| GPS TX (to GT‑U7 RX)       | D1 (TX1)   | Optional                          |
| RS‑485 TX                  | D8 (TX2)   | `Serial2 TX` → MAX3485 `DI`      |
| RS‑485 RX                  | D7 (RX2)   | `Serial2 RX` ← MAX3485 `RO`      |
| RS‑485 DE/RE               | D6         | `PWR_RS485_DE_RE_PIN` (DE+RE)    |
| ARM switch A               | D20        | Active‑low, `PWR_ARM_A_PIN`      |
| ARM switch B               | D21        | Active‑low, `PWR_ARM_B_PIN`      |
| START button A             | D22        | Active‑low, `PWR_START_A_PIN`    |
| START button B             | D23        | Active‑low, `PWR_START_B_PIN`    |
| Channel A status LED       | D16        | `PWR_LED_A_PIN`                  |
| Channel B status LED       | D17        | `PWR_LED_B_PIN`                  |
| Ground‑station battery ADC | A0         | Divider midpoint, `PWR_VBAT_PIN` |

## MAX3485 (RS‑485 Transceiver)

| MAX3485 Pin | Connects To   | Notes                   |
|-------------|---------------|-------------------------|
| DI          | Teensy D8 (TX2)     | `Serial2 TX`           |
| RO          | Teensy D7 (RX2)    | `Serial2 RX`           |
| DE          | Teensy D6     | Tie DE+RE together     |
| RE          | Teensy D6     | Tie RE to DE           |
| A           | RS‑485 A line | To Power Module A      |
| B           | RS‑485 B line | To Power Module B      |
| VCC         | 3.3 V         |                         |
| GND         | Ground        |                         |

## Power Module (arduino_power_module.ino)

| Function                      | Power Pin | Notes                     |
|------------------------------|----------:|---------------------------|
| RS‑485 DE+RE                 | D2        | `RS_DE_RE`                |
| RS‑485 RO → SoftSerial RX    | D10       | `RS_RX`                   |
| RS‑485 DI ← SoftSerial TX    | D11       | `RS_TX`                   |
| MOSFET gate channel A        | D3        | `MOSFET_A`                |
| MOSFET gate channel B        | D5        | `MOSFET_B`                |
| ACS712 current sensor lane A | A3        | `ACS_A`                   |
| ACS712 current sensor lane B | A7        | `ACS_B`                   |
| Ignition battery divider ADC | A0        | `VBAT_PIN`                |
| Lane A LED                   | D9        | `LED_A`                   |
| Lane B LED                   | D12       | `LED_B`                   |
| Safety key input             | D4        | Active‑low, `KEY_PIN`     |
| Igniter presence lane A      | A1        | `SENSE_A`                 |
| Igniter presence lane B      | A2        | `SENSE_B`                 |
| DIAG button                  | D8        | Active‑low, `DIAG_BTN`    |
