# Internal README — Rocket Altimeter (ATTiny85 + BMP180)

This document explains how the device works and defines two parameter profiles
for **Indoor Testing** and **Rocket Flight** modes.  
It is meant for personal reference while maintaining the firmware.

---

## How the Device Works

### Overview
The rocket altimeter is built using:

- **ATtiny85** microcontroller  
- **BMP180/BMP150** barometric pressure sensor  
- **3.7 V LiPo battery** (through power switch)

The device measures barometric pressure, computes altitude, detects launch/apogee,
and stores flight data in EEPROM:

- Pad altitude  
- Apogee altitude  
- Delta altitude (apogee − pad)  
- Lifetime record max altitude  

Altitude is calculated using Bosch's fixed‑point compensation formulas and an
integer approximation of the barometric formula:

alt ≈ (SEA_LEVEL_PRESSURE − pressure) * 0.0833

### Startup / Calibration
1. If the reset button is held during power‑on, EEPROM flight data is cleared.  
2. The device measures and averages 16 altitude samples to determine **padAlt**.

### Flight Detection Logic

#### Launch Detection
Launch is detected when current altitude rises above pad altitude by:

LAUNCH_ALT_THRESH  (in meters)

#### Apogee Detection
While launched:
- maxAlt tracks the highest filtered altitude.
- The device counts consecutive samples where altitude is falling.
- When fallingCnt > APOGEE_FALL_SAMPLES,
  apogee is confirmed, and flight data is written to EEPROM.

### Filtering
Altitude is low‑pass smoothed:

altFiltered += (altRaw - altFiltered) >> ALT_FILTER_SHIFT

### Telemetry Output
Example output (Software Serial @ 9600 baud):

T=19.20C P=98487Pa A=236m PAD=235m MAX=238m dA=3m REC=241m

Meaning:

- T — temperature  
- P — pressure  
- A — current filtered altitude  
- PAD — pad altitude  
- MAX — max altitude this session  
- dA — current flight height (MAX − PAD)  
- REC — lifetime record apogee  

---

## Profiles (Adjustable Constant Sets)

Use these when editing the firmware constants—no code modification required.

---

# Profile 1 — Indoor Test Mode  
(For debugging on stairs, walking around, very small altitude changes.)

#define ALT_FILTER_SHIFT      0     // no smoothing
#define LAUNCH_ALT_THRESH     1     // detect launch at +1 m
#define APOGEE_FALL_SAMPLES   3     // fast apogee detection
#define EEPROM_RECORD_DELTA   1     // update record easily

### Behavior
- Altitude responds immediately  
- Good for testing near the ground  
- More jitter (expected)  
- Launch + apogee work at small height differences  

---

# Profile 2 — Rocket Flight Mode  
(For real launches with strong vibration and large altitude changes.)

#define ALT_FILTER_SHIFT      3     // strong smoothing
#define LAUNCH_ALT_THRESH     10    // ignore small bumps
#define APOGEE_FALL_SAMPLES   10    // stable apogee detection
#define EEPROM_RECORD_DELTA   5     // update record only for real improvements

### Behavior
- Smooth altitude even with vibration  
- Launch detection triggers only on real ascent  
- Apogee not falsely triggered  
- EEPROM wear minimized  
- Reliable for high‑speed rocket flight  

---

## Notes
- Switching between profiles only requires editing the four constants at the top of the firmware.  
- No formula changes or structural code changes are needed.  
- For flight, always use Profile 2.

