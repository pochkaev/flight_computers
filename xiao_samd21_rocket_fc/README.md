# XIAO SAMD21 Rocket Flight Computer

This project is a compact hobby rocket flight computer built around the Seeed Studio XIAO SAMD21. It logs telemetry to a microSD card during flight and includes post-flight tooling in Python and MATLAB for plotting altitude, IMU data, GPS data, and a simple 3D rocket attitude view.

## Hardware

Primary hardware from your list:

- Seeed Studio XIAO SAMD21
- GT-U7 GPS module
- 3.3 V microSD breakout
- BMP390 barometric pressure sensor
- ICM-20602 IMU
- Buzzer
- Button
- 3.7 V 1100 mAh LiPo

Required extra hardware not in the original list:

- 3.3 V regulator or buck-boost regulator sized for GPS current peaks
- LiPo protection / switch / fuse arrangement suitable for your airframe
- Level-safe SD breakout confirmed to be 3.3 V only
- Pull-up resistor for button if you do not use `INPUT_PULLUP`
- Buzzer driver transistor if your buzzer draws more current than a GPIO pin can safely supply

Important power note:

- Do not connect the 3.7 V LiPo directly to the XIAO SAMD21 logic rail.
- Power the whole avionics stack from a regulated 3.3 V rail.
- Tie all module grounds together.

## Project Layout

- [firmware/xiao_samd21_rocket_fc/xiao_samd21_rocket_fc.ino](/Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/firmware/xiao_samd21_rocket_fc/xiao_samd21_rocket_fc.ino): Arduino sketch for the flight computer
- [docs/pinout.md](/Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/docs/pinout.md): wiring and power map
- [docs/build_notes.md](/Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/docs/build_notes.md): assembly, calibration, and flight checklist
- [python/parse_and_visualize.py](/Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/python/parse_and_visualize.py): CSV parser and 3D visualizer
- [matlab/plot_flight.m](/Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/matlab/plot_flight.m): MATLAB plotting entry point
- [matlab/animate_rocket.m](/Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/matlab/animate_rocket.m): MATLAB 3D rocket animation
- [logs/sample_flight.csv](/Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/logs/sample_flight.csv): sample log format

## Firmware Features

- BMP390 barometric altitude logging
- BMP390 over SPI
- ICM-20602 accelerometer and gyroscope logging over SPI
- GT-U7 GPS parsing over UART
- CSV log to microSD
- Automatic launch detection from `IDLE`
- Idle auto-zero for the BMP390 ground reference
- Button-controlled ground-reference refresh and next-flight reset
- Maximum altitude tracking and logging
- Landed locator beep
- Built-in LED and buzzer status indication
- Launch detection from acceleration and altitude growth
- Apogee estimate from filtered barometric altitude

## Flight Status Flow

The rocket passes through these states:

- `BOOT`: power-up, hardware checks, open a new flight log, capture ground pressure
- `IDLE`: pad standby, BMP390 auto-zero active, waiting for launch detection
- `BOOST`: launch detected from acceleration or altitude growth
- `COAST`: boost timer finished, rocket is still climbing
- `DESCENT`: apogee detected from decreasing filtered altitude
- `LANDED`: motion and altitude are quiet for long enough to declare landing
- `ERROR`: one or more required devices failed initialization

Practical flow for a normal flight:

1. Power on and wait for `IDLE`.
2. Let altitude settle near zero.
3. Launch occurs and the firmware switches to `BOOST`.
4. After boost, it switches to `COAST`.
5. When altitude starts falling, it switches to `DESCENT`.
6. After landing is stable long enough, it switches to `LANDED`.
7. In `LANDED`, the buzzer gives periodic locator beeps.
8. After recovery, hold the button about `4 seconds` to reset for the next flight and create a new log.

## Built-In LED Status

The XIAO built-in LED shows the current flight computer state:

- `BOOT`: fast blink, toggles every `100 ms`
- `IDLE`: slow blink, toggles every `1000 ms`
- `ARMED`: medium blink, toggles every `200 ms`, legacy/manual state
- `BOOST`: solid on
- `COAST`: solid on
- `DESCENT`: moderate blink, toggles every `300 ms`
- `LANDED`: very slow blink, toggles every `1200 ms`
- `ERROR`: very fast blink, toggles every `80 ms`

## Current Bench Behavior

Current working bench configuration:

- `BMP390`: working over SPI
- `ICM-20602`: working over SPI
- `microSD`: working and creating `FLIGHTxx.CSV` log files
- `GT-U7`: receiving NMEA over UART

Normal startup sequence:

1. Power on the XIAO.
2. The firmware checks BMP390, ICM-20602, and SD.
3. A new log file is created.
4. Ground pressure is captured.
5. The system enters `IDLE`.
6. While in `IDLE`, the barometer slowly auto-zeroes toward `0 m`.

The button actions are:

- Hold about `1 second` in `IDLE`: refresh ground pressure reference
- Hold about `4 seconds` in any non-error state: flush and close the current log, open a new log file, recapture ground pressure, and return to `IDLE`

The reset action is intended to return the rocket to the same usable pre-flight state as startup:

- close current log
- create a new `FLIGHTxx.CSV`
- clear boost/coast/descent/landed timing state
- clear stored flight max altitude for the new flight
- recapture ground pressure
- return to `IDLE`

## Arduino Libraries

Install these in Arduino IDE before compiling:

- `Adafruit BMP3XX`
- `TinyGPSPlus`

The sketch uses the standard Arduino `SD` library plus a direct-register ICM-20602 driver, so no separate IMU library is required.

Important hardware note:

- This project now assumes a BMP390 breakout with pins `INT`, `CS`, `SDO`, `SDI`, `SCK`, `VCC`, `GND`
- That means the BMP390 is wired over SPI, not I2C
- `D6` is used as the BMP390 chip-select pin
- GPS is used one-way in this version: GPS `TX` to XIAO `D7`
- ICM-20602 is on the shared SPI bus with chip select on `D1`
- microSD is on the shared SPI bus with chip select on `D0`

For the Python tools:

```bash
cd /Users/k_pochkaev/github/flight_computers/xiao_samd21_rocket_fc/python
python3 -m pip install -r requirements.txt
```

## Log Format

The firmware writes one CSV row per sample:

```text
ms,phase,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,baro_alt_m,max_alt_m,baro_temp_c,pressure_pa,gps_fix,gps_sats,lat_deg,lon_deg,gps_alt_m,gps_speed_mps,gps_course_deg
```

`max_alt_m` is the maximum filtered barometric altitude reached so far in that flight log.

## Constraints

- XIAO SAMD21 has limited RAM, so this project keeps buffers small.
- Without a magnetometer, yaw in the 3D replay is gyro-integrated and will drift.
- GT-U7 update rate and lock quality strongly affect trajectory quality.
- BMP390 altitude may need a few seconds in `IDLE` to settle near zero on the pad.
- The landed locator beep is intended for recovery assistance and should be matched to the buzzer current capability of your hardware.

## Recommended Next Hardware Revision

For a more flight-ready board later, add:

- Pyro channel outputs with proper MOSFETs and continuity sensing
- Battery voltage divider to log supply voltage
- Flash storage or FRAM for redundancy
- Magnetometer or dual-antenna GNSS if you want better heading reconstruction
