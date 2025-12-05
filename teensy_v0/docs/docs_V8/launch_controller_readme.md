# Ground Station V8 – Launch Controller Integration

This document describes how the Teensy-based Ground Station integrates the two-channel ignition (Power Module) previously driven by the stand-alone Remote Module.

## Overview

- The existing `arduino_power_module` remains unchanged and still performs all high-current and safety-critical functions (igniter drive, overcurrent/short protection, key interlock, presence sensing, 10 s cutoff).
- The Teensy Ground Station now acts as the RS‑485 master, replacing `arduino_remote_module`:
  - Sends ARM/START commands and periodic polls over RS‑485.
  - Receives ignition battery voltage, per-lane current/peak, presence, key status, and fault status.
  - Implements additional master-side safety (START latch logic and 10 s fire timeout).
  - Presents a `[LAUNCH]` page on the OLED plus ARM/START LEDs and switches.

## Protocol

The Ground Station uses the same binary protocol as the original Remote Module:

- Master → Power (6 bytes): `AA 55 [Type] [Seq] [Bits] [CRC8]`
  - `Type`: `0x01` = state only, `0x02` = state + poll.
  - `Bits`: `b0=ARM_A`, `b1=START_A`, `b2=ARM_B`, `b3=START_B`.
- Power → Master (9 bytes): `55 AA 81 [SeqEcho] [Bits] [VBATx10] [IAx10] [IBx10] [CRC8]`
  - `Bits`:
    - `b0=ARM_A_seen`, `b1=ON_A`, `b2=ARM_B_seen`, `b3=ON_B`.
    - `b4=KEY_OK` (safety key present).
    - `b5=PRES_A`, `b6=PRES_B`.
    - `b7=FAULT_ANY` (any lane in fault/short).

The Ground Station code mirrors the Remote Module timing:

- TX period ≈ 50 ms, poll every ≈ 300 ms.
- Link freshness timeout ≈ 500 ms.
- Master fire timeout per lane = 10 s (in addition to Power Module 10 s cutoff).

## Teensy Ground Station Pinout (Launch Controller)

All pin numbers are defined in `GroundStation/config.h`. Adjust if needed to match your board/layout.

### RS‑485 / MAX3485

- `PWR_RS485_SERIAL` = `Serial2` (Teensy hardware UART):
  - `Serial2 TX` → MAX3485 `DI`.
  - `Serial2 RX` → MAX3485 `RO`.
- `PWR_RS485_DE_RE_PIN` (default: pin 6):
  - Connect to MAX3485 `DE` and `RE` tied together.
  - HIGH = transmit, LOW = receive.
- MAX3485 power:
  - `VCC` → 3.3 V.
  - `GND` → ground.
  - `A/B` differential pair → Power Module A/B lines.

### ARM / START Inputs and LEDs

Active‑low switches and buttons with internal pull‑ups enabled:

- `PWR_ARM_A_PIN` (default: 20) → ARM switch for lane A (to GND when armed).
- `PWR_ARM_B_PIN` (21) → ARM switch for lane B.
- `PWR_START_A_PIN` (22) → START button for lane A.
- `PWR_START_B_PIN` (23) → START button for lane B.

Indicator LEDs (with series resistors):

- `PWR_LED_A_PIN` (16) → channel A status LED.
- `PWR_LED_B_PIN` (17) → channel B status LED.

LED behaviour:

- Link/key/fault not OK → LEDs off.
- Armed, idle → solid ON.
- Firing (START pressed / ON) → blink.

### Local Ground Battery Measurement

- `PWR_VBAT_PIN` (default: `A0`):
  - Connect via a 2:1 divider (e.g. 100k/100k) from the ground station’s battery to GND.
  - Code assumes 2:1; adjust `readLocalVbat()` if you use a different ratio or reference voltage.

## UI Integration

The launch controller UI is implemented in `GroundStation/ui.cpp`:

- New page: `PAGE_LAUNCH` (labelled `[LAUNCH]`):
  - Top line: `Ign:xx.xV Bat:yy.yV` and a link marker:
    - `*` – link OK, no fault.
    - `F` – link OK, `FAULT_ANY` active.
    - `!` – link stale.
  - Lane A line: `A:ARM/SAFE/LOCK [ON]/[  ] I:x.x(OK/--)`.
  - Lane B line: same for B.
- Page is included in the normal page cycle and respects the existing manual/auto page logic.

Data used by the UI (from `power.cpp` / `power.h`):

- `pwr_vbat_x10` – ignition battery (x10).
- `pwr_localVbat` – local Teensy battery voltage (from `PWR_VBAT_PIN`).
- `pwr_armA_seen`, `pwr_onA`, `pwr_armB_seen`, `pwr_onB`.
- `pwr_key_ok`, `pwr_presA`, `pwr_presB`, `pwr_faultAny`.
- `pwr_ia_x10`, `pwr_ib_x10` – current or peak current per lane.

## Safety Notes

- The Power Module remains the single point of high‑current switching and overcurrent/short detection.
- The Ground Station adds:
  - START must be released after arming before it is accepted.
  - Master 10 s fire timeout per lane.
  - UI clearly showing KEY/LOCK, presence, fault, and current per lane.
- Always test with dummy loads (e.g., lamps or power resistors) and verify:
  - ARM/START logic.
  - Key interlock.
  - Overcurrent fault behaviour and reset.
  - Loss‑of‑link behaviour.
