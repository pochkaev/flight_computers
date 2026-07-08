# Rocket v10 Pyro Configuration

This document describes the current Rocket v10 pyro logic.

Current important safety state:

- real pyro outputs are disabled by default
- events are still logged to NAND/SD and sent to the ground station
- ground shows channel state on the `[PYRO CFG]` page
- ground flashes red three times when it receives a pyro event packet

The current firmware is safe/log-only unless both the global output enable and the matching channel output enable are changed.

## Current Defaults

Current defaults in `fw/RocketV10/config.h`:

```cpp
#define PYRO_OUTPUT_ENABLE       0
#define PYRO_ACTIVE_HIGH         1
#define PYRO_FIRE_MS             1000u
#define PYRO_FLIGHT_PROFILE      'S'
#define PYRO_APOGEE_DELAY_MS     1000u
#define PYRO_MAIN_MIN_AFTER_APOGEE_MS 1000u

#define PYRO_CH1_PIN         6
#define PYRO_CH1_FUNC        'A'
#define PYRO_CH1_LOG_ENABLE  1
#define PYRO_CH1_OUTPUT_ENABLE 0

#define PYRO_CH2_PIN         7
#define PYRO_CH2_FUNC        'M'
#define PYRO_CH2_LOG_ENABLE  1
#define PYRO_CH2_OUTPUT_ENABLE 0

#define PYRO_CH3_PIN         8
#define PYRO_CH3_FUNC        'N'
#define PYRO_CH3_LOG_ENABLE  1
#define PYRO_CH3_OUTPUT_ENABLE 0

#define PYRO_CH4_PIN         15
#define PYRO_CH4_FUNC        'N'
#define PYRO_CH4_LOG_ENABLE  1
#define PYRO_CH4_OUTPUT_ENABLE 0
```

With these values:

- channel 1 logs apogee/drogue events
- channel 2 logs main events
- channels 3 and 4 are function-disabled but still configured to log if their function is later changed
- no Teensy pyro pin produces a real output pulse

## Output Enable Model

Physical firing requires both levels:

```text
PYRO_OUTPUT_ENABLE == 1
and
PYRO_CHx_OUTPUT_ENABLE == 1
```

Logging is independent:

```text
PYRO_CHx_LOG_ENABLE == 1
```

This means a channel can be log-only while another channel is physically enabled.

Example:

```cpp
// Global physical outputs enabled.
#define PYRO_OUTPUT_ENABLE       1

// Apogee is log-only.
#define PYRO_CH1_FUNC        'A'
#define PYRO_CH1_LOG_ENABLE  1
#define PYRO_CH1_OUTPUT_ENABLE 0

// Main can physically fire.
#define PYRO_CH2_FUNC        'M'
#define PYRO_CH2_LOG_ENABLE  1
#define PYRO_CH2_OUTPUT_ENABLE 1
```

Ground display:

```text
1 D6  APOGEE   LOG
2 D7  MAIN     FIRE
```

## PYRO_ACTIVE_HIGH

`PYRO_ACTIVE_HIGH` controls electrical polarity.

```cpp
#define PYRO_ACTIVE_HIGH 1
```

Means:

- inactive pin level is `LOW`
- fire pulse pin level is `HIGH`

This is the normal setting for a low-side MOSFET driver where Teensy GPIO drives the MOSFET gate through suitable protection/resistance.

```cpp
#define PYRO_ACTIVE_HIGH 0
```

Means:

- inactive pin level is `HIGH`
- fire pulse pin level is `LOW`

Use active-low only if the external driver circuit is designed that way.

Hardware safety notes:

- do not drive an e-match directly from a Teensy GPIO
- use a MOSFET/transistor driver
- add gate/base pulldowns so outputs stay inactive during boot/reset
- use an external arming switch
- use current-limited pyro power
- test with LEDs or dummy loads before any real charges

## Channel Functions

Each channel has a function letter:

```text
N = disabled
A = apogee/drogue
M = main
B = booster separation
I = sustainer ignition
1 = airstart 1
2 = airstart 2
```

The firmware calls `requestPyroFunction()` for a function event. Every channel with matching function can log and, if enabled, fire.

Example:

```cpp
#define PYRO_CH1_FUNC 'A'
#define PYRO_CH2_FUNC 'M'
#define PYRO_CH3_FUNC 'N'
#define PYRO_CH4_FUNC 'N'
```

Means:

- apogee/drogue event uses channel 1
- main event uses channel 2
- channels 3 and 4 are disabled

## Normal Dual Deploy Logic

Normal apogee/main dual-deploy logic is active regardless of `PYRO_FLIGHT_PROFILE`.

Apogee/drogue:

- rocket detects apogee from barometer vertical velocity and minimum altitude
- `tApogeeMs` is saved
- `FLAG_APOGEE` is set
- apogee pyro event waits for `PYRO_APOGEE_DELAY_MS`
- function `A` is requested

Current delay:

```cpp
#define PYRO_APOGEE_DELAY_MS 1000u
```

Main:

- main event requires apogee already detected
- main waits at least `PYRO_MAIN_MIN_AFTER_APOGEE_MS`
- rocket must be descending
- relative altitude must be at or below `DUAL_DEPLOY_MAIN_ALT_M`
- rocket must have reached safely above main altitude by `DUAL_DEPLOY_MAIN_MIN_APOGEE_MARGIN_M`

Current main altitude:

```cpp
#define DUAL_DEPLOY_MAIN_ALT_M 153.0f
```

`153 m` is about `502 ft`.

Current safety margin:

```cpp
#define DUAL_DEPLOY_MAIN_MIN_APOGEE_MARGIN_M 20.0f
```

This means main is allowed only if max altitude reached at least:

```text
main altitude + 20 m
```

Example:

- main altitude: `153 m` / about `500 ft`
- required max altitude: `173 m` / about `568 ft`
- if flight only reaches `400 ft`, main is inhibited

This prevents a low flight from firing/logging main immediately after apogee.

## PYRO_FLIGHT_PROFILE

`PYRO_FLIGHT_PROFILE` is a configured mode, not a detected value.

Rocket sends it to ground in the pyro config LoRa packet. Ground currently displays it as a short type label.

```cpp
#define PYRO_FLIGHT_PROFILE 'S'
```

Available values:

```text
S = single-stage
2 = two-stage
A = airstart
```

### `S`: Single-Stage

This is the current normal mode.

Behavior:

- normal apogee/main logic works
- no booster separation event
- no sustainer ignition event
- no airstart events

Use this for current low-power and L1-style single-stage flights.

### `2`: Two-Stage

This adds future staging events.

After booster burnout/coast is marked:

1. wait `PYRO_BOOSTER_SEP_DELAY_MS`
2. request function `B` for booster separation
3. wait `PYRO_SUSTAINER_FIRE_DELAY_MS`
4. request function `I` for sustainer ignition

Sustainer ignition uses staging safety checks:

- relative altitude must be at least `PYRO_STAGING_MIN_REL_ALT_M`
- tilt must be at or below `PYRO_STAGING_MAX_TILT_DEG`
- rocket must not be landed or aborted

Current values:

```cpp
#define PYRO_STAGING_MIN_REL_ALT_M    20.0f
#define PYRO_STAGING_MAX_TILT_DEG     45.0f
#define PYRO_BOOSTER_SEP_DELAY_MS     1000u
#define PYRO_SUSTAINER_FIRE_DELAY_MS  1000u
```

Example two-stage channel config:

```cpp
#define PYRO_FLIGHT_PROFILE '2'

#define PYRO_CH1_FUNC 'A'  // apogee
#define PYRO_CH2_FUNC 'M'  // main
#define PYRO_CH3_FUNC 'B'  // booster separation
#define PYRO_CH4_FUNC 'I'  // sustainer ignition
```

Do not use this profile for current single-stage flights.

### `A`: Airstart

This adds future airstart events.

Airstart 1 timing base:

```cpp
#define PYRO_AIRSTART1_EVENT 'I'
```

Meanings:

- `I`: start timer at liftoff
- `B`: start timer at booster burnout/coast

Airstart 2 timing base:

```cpp
#define PYRO_AIRSTART2_EVENT '1'
```

Meanings:

- `1`: start timer after airstart 1 event
- `B`: start timer at booster burnout/coast

Current delays:

```cpp
#define PYRO_AIRSTART1_DELAY_MS 1000u
#define PYRO_AIRSTART2_DELAY_MS 1000u
```

Example airstart channel config:

```cpp
#define PYRO_FLIGHT_PROFILE 'A'

#define PYRO_CH1_FUNC 'A'  // apogee
#define PYRO_CH2_FUNC 'M'  // main
#define PYRO_CH3_FUNC '1'  // airstart 1
#define PYRO_CH4_FUNC '2'  // airstart 2
```

Airstart events also use staging safety checks before requesting functions.

Do not use this profile for current single-stage flights.

## Ground `[PYRO CFG]` Screen

The ground station receives a low-rate pyro config packet over LoRa.

Current fields:

```text
MODE: LOG ONLY   TYPE: 1ST
PULSE: 1.0s      MAIN: 153m
APOGEE DELAY: 1.0s
MAIN WAIT: 1.0s

1 D6  APOGEE              LOG
2 D7  MAIN                LOG
3 D8  DISABLED            LOG
4 D15 DISABLED            LOG
```

Meaning:

- `MODE` is the global output mode:
  - `LOG ONLY`: no physical pyro output can fire
  - `ARMED`: global physical output is enabled
- `TYPE` is the configured flight profile:
  - `1ST`: `PYRO_FLIGHT_PROFILE 'S'`
  - `2ST`: `PYRO_FLIGHT_PROFILE '2'`
  - `AIR`: `PYRO_FLIGHT_PROFILE 'A'`
- `PULSE` is the output pulse length if a physical channel fires
- `MAIN` is the main deployment altitude
- `APOGEE DELAY` is the delay after apogee before requesting function `A`
- `MAIN WAIT` is the minimum time after apogee before main can happen
- channel row mode:
  - `LOG`: event is logged, but physical output is not enabled for that channel
  - `FIRE`: physical output is enabled for that channel
  - `NOLOG`: channel event logging is disabled

Pyro events are also sent over LoRa. When ground receives a pyro event packet, it flashes the whole screen red three times.

## LoRa Pyro Packets

Rocket sends:

- `0x05`: pyro config packet
- `0x06`: pyro event packet

Config packet behavior:

- sent every `PYRO_CONFIG_TX_MS`
- currently every `10000 ms`
- includes global output mode, active polarity, profile, channel functions, pins, log mask, and output mask

Event packet behavior:

- sent when a pyro function/channel event is logged or fired
- events are queued so quick bursts are not overwritten before LoRa sends them
- ground uses these packets for the red flash alert

## Recommended Test Progression

For current flights:

```cpp
#define PYRO_OUTPUT_ENABLE 0
#define PYRO_CH1_OUTPUT_ENABLE 0
#define PYRO_CH2_OUTPUT_ENABLE 0
#define PYRO_CH3_OUTPUT_ENABLE 0
#define PYRO_CH4_OUTPUT_ENABLE 0
```

Use this to validate:

- apogee event is logged
- main decision is logged only when conditions are valid
- ground `[PYRO CFG]` receives config
- ground flashes red on pyro events
- post-flight logs match the expected state timeline

For first real main-only test:

```cpp
#define PYRO_OUTPUT_ENABLE 1

#define PYRO_CH1_FUNC 'A'
#define PYRO_CH1_LOG_ENABLE 1
#define PYRO_CH1_OUTPUT_ENABLE 0

#define PYRO_CH2_FUNC 'M'
#define PYRO_CH2_LOG_ENABLE 1
#define PYRO_CH2_OUTPUT_ENABLE 1
```

This keeps apogee log-only and enables physical main output only.

Before any real charge:

- test with LED or resistor dummy load
- verify output polarity
- verify output pulse duration
- verify no output pulse happens on boot/reset
- verify ground shows `FIRE` only on the intended channel
- verify physical arming switch behavior

## Future Dedicated Pyro Module

A separate pyro module may be useful later if it is autonomous, not just a MOSFET expander.

Good future architecture:

- main Teensy handles navigation, LoRa, logging, replay data, and ground telemetry
- dedicated pyro module has its own controller, barometer, 2S pyro battery, MOSFETs, continuity checks, and state machine
- Teensy sends initial config before flight
- pyro module acknowledges config back to Teensy
- pyro module can still make safe apogee/main decisions if Teensy resets or disappears

Prototype approach:

1. build XIAO SAMD21 module in log-only or LED-load mode
2. send config from Teensy
3. report acknowledged config and local status
4. compare pyro-module decisions with Teensy NAND logs
5. enable real outputs only after repeated successful dummy-load tests

Prefer UART with CRC, sequence numbers, and acknowledgements for the inter-board link. Avoid I2C for flight-critical inter-board communication.
