# Arduino Nano as ISP to ATtiny85

Use this wiring when programming the ATtiny85 with an Arduino Nano running the `ArduinoISP` sketch.

## Nano to ATtiny85

| Arduino Nano | ATtiny85 signal | ATtiny85 port | Physical pin |
|--------------|-----------------|---------------|--------------|
| D10          | RESET           | P5 / PB5      | 1            |
| D11          | MOSI            | P0 / PB0      | 5            |
| D12          | MISO            | P1 / PB1      | 6            |
| D13          | SCK             | P2 / PB2      | 7            |
| 5V           | VCC             | VCC           | 8            |
| GND          | GND             | GND           | 4            |

`D10` on the Nano is the reset control line for the target and must go to `PB5` (`P5`) on the ATtiny85.

## ATtiny85 P0..P5 mapping

| Label | AVR port | Function for ISP |
|-------|----------|------------------|
| P0    | PB0      | MOSI             |
| P1    | PB1      | MISO             |
| P2    | PB2      | SCK              |
| P3    | PB3      | Not used for ISP |
| P4    | PB4      | Not used for ISP |
| P5    | PB5      | RESET            |

## ATtiny85 DIP-8 pinout

```text
        ATtiny85
     +---\/---+
P5 1 |*      | 8 VCC
P3 2 |       | 7 P2 / SCK
P4 3 |       | 6 P1 / MISO
GND4 |       | 5 P0 / MOSI
     +-------+
```

## Nano ISP port note

By default, the hardware SPI pins `MISO`, `MOSI`, and `SCK` are used to communicate with the target. On all Arduinos, these pins can be found on the ICSP/SPI header:

```text
              MISO  . . 5V
              SCK   . . MOSI
                    . . GND
```

You can use either:

- Nano pins `D11`, `D12`, `D13`
- or the Nano ICSP header pins for `MOSI`, `MISO`, and `SCK`

They are the same SPI signals.

## Arduino IDE notes

1. Flash the `ArduinoISP` example sketch to the Nano.
2. Add a `10 uF` capacitor between Nano `RESET` and `GND`.
3. In Arduino IDE select:
   - `Programmer -> Arduino as ISP`
   - `Sketch -> Upload Using Programmer` for a no-bootloader target
   - `Tools -> Burn Bootloader` if you want to install Micronucleus first

## Notes for Digispark-style boards

- `P3` and `P4` are often shared with the USB circuitry on Digispark boards, so they are not part of ISP wiring.
- If `PB5` reset was disabled by fuses, normal ISP will not work and high-voltage programming is required.
