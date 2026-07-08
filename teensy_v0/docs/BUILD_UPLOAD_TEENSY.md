# Teensy Build and Upload Guide

This file is the short path for future humans and AI agents. Use these commands before spending time rediscovering Arduino library paths, board targets, or Teensy upload behavior.

## Project Root

Run commands from:

```bash
cd /Users/k_pochkaev/github/flight_computers/teensy_v0
```

Important paths:

| Purpose | Path |
|---|---|
| Arduino CLI config | `/Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-cli.yaml` |
| Ground libraries | `/Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-user/libraries` |
| Rocket libraries | `/Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-libs` |
| Rocket sketch | `/Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10` |
| Ground sketch | `/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/fw/GroundStationV10` |

Do not rely on `~/Documents/Arduino/libraries` on this Mac. It can be blocked by macOS privacy permissions and caused failed library discovery during the April 26, 2026 field debug.

## Board Targets

| Module | Board | FQBN |
|---|---|---|
| Rocket v10 | Teensy 4.1 | `teensy:avr:teensy41` |
| Ground v10 | Teensy 4.0 | `teensy:avr:teensy40` |

Do not swap these targets. Rocket and ground are different Teensy boards.

## Required Libraries

Rocket v10 uses the repo-local libraries in `.arduino-libs`:

- `LoRa`
- `TinyGPSPlus`
- `Adafruit_LSM9DS1`
- `Adafruit_Sensor`
- `Adafruit_BusIO`
- `Adafruit_LIS3MDL`

Ground v10 uses `.arduino-user/libraries`:

- `LoRa`
- `TinyGPSPlus`
- `Adafruit_BMP085_Library`
- `Adafruit_Unified_Sensor`
- `Adafruit_BusIO`
- `ILI9341_t3` comes from the Teensy core package

If `.arduino-libs` is missing, recreate the rocket libraries with:

```bash
git clone --depth 1 https://github.com/sandeepmistry/arduino-LoRa.git .arduino-libs/LoRa
git clone --depth 1 https://github.com/mikalhart/TinyGPSPlus.git .arduino-libs/TinyGPSPlus
git clone --depth 1 https://github.com/adafruit/Adafruit_LSM9DS1.git .arduino-libs/Adafruit_LSM9DS1
git clone --depth 1 https://github.com/adafruit/Adafruit_Sensor.git .arduino-libs/Adafruit_Sensor
git clone --depth 1 https://github.com/adafruit/Adafruit_BusIO.git .arduino-libs/Adafruit_BusIO
git clone --depth 1 https://github.com/adafruit/Adafruit_LIS3MDL.git .arduino-libs/Adafruit_LIS3MDL
```

## Rocket Build

Compile into a stable build directory:

```bash
mkdir -p /tmp/rocketv10-build
arduino-cli compile \
  --fqbn teensy:avr:teensy41 \
  --libraries /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-libs \
  --build-path /tmp/rocketv10-build \
  /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10
```

Expected output includes a Teensy 4.1 memory summary. LoRa may emit `IRQ_NUMBER_t` conversion warnings; those warnings are known and did not block the April 26, 2026 build.

## Rocket Upload

First list boards:

```bash
arduino-cli board list
```

The rocket may appear both as a serial port and as a Teensy port:

```text
/dev/cu.usbmodem187564601       serial   Serial Port (USB) Unknown
usb:110000                      teensy   Teensy Ports      Teensy 4.1 teensy:avr:teensy41
```

Prefer the `teensy` port when available:

```bash
arduino-cli upload \
  --fqbn teensy:avr:teensy41 \
  --port usb:110000 \
  --input-dir /tmp/rocketv10-build \
  /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10
```

If only the serial port is known, this can still open Teensy Loader:

```bash
arduino-cli upload \
  --fqbn teensy:avr:teensy41 \
  --port /dev/cu.usbmodem187564601 \
  --input-dir /tmp/rocketv10-build \
  /Users/k_pochkaev/github/flight_computers/teensy_v0/rocket/rocket_v10/fw/RocketV10
```

For an explicit Teensyduino handoff and reboot result, use:

```bash
/Users/k_pochkaev/Library/Arduino15/packages/teensy/tools/teensy-tools/1.60.0/teensy_post_compile \
  -v \
  -file=RocketV10.ino \
  -path=/tmp/rocketv10-build \
  -tools=/Users/k_pochkaev/Library/Arduino15/packages/teensy/tools/teensy-tools/1.60.0 \
  -board=TEENSY41 \
  -port=usb:110000 \
  -portlabel=usb:110000 \
  -reboot
```

On April 26, 2026 this returned `Success` for the rocket module.

## Ground Build

Compile into a stable build directory:

```bash
mkdir -p /tmp/groundv10-build
arduino-cli compile \
  --fqbn teensy:avr:teensy40 \
  --libraries /Users/k_pochkaev/github/flight_computers/teensy_v0/.arduino-user/libraries \
  --build-path /tmp/groundv10-build \
  /Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/fw/GroundStationV10
```

Expected output includes a Teensy 4.0 memory summary. Known warnings can appear from `LoRa` and `TinyGPSPlus`; they did not block the verified build.

## Ground Upload

List ports:

```bash
arduino-cli board list
```

Use the Teensy port if listed. If only the serial port is known, the recent ground serial port was:

```text
/dev/cu.usbmodem184901201
```

Upload from the fixed build directory:

```bash
arduino-cli upload \
  --fqbn teensy:avr:teensy40 \
  --port /dev/cu.usbmodem184901201 \
  --input-dir /tmp/groundv10-build \
  /Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/fw/GroundStationV10
```

If Arduino CLI reports a `usb:...` Teensy port for ground, prefer that port and keep `--fqbn teensy:avr:teensy40`.

## Verification Commands

Board discovery:

```bash
arduino-cli board list
```

Confirm the rocket build artifact:

```bash
ls -la /tmp/rocketv10-build/RocketV10.ino.hex
```

Confirm the ground build artifact:

```bash
ls -la /tmp/groundv10-build/GroundStationV10.ino.hex
```

