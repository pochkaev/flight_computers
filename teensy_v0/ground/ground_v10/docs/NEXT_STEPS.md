# Ground v10 Next Steps

This file defines the practical next work sequence for the ground controller.

Related context:

- [AI_CONTEXT.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/docs/AI_CONTEXT.md)
- [README.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/docs/README.md)

## Goal

Bring the ground controller to a stable, field-usable state with:

- correct `Teensy 4.0` firmware target
- stable operation in the fully assembled device
- correct battery measurement
- reliable LoRa receive path
- readable and operational UI
- controlled logging behavior

## Current state

Already done:

- ground target corrected to `Teensy 4.0`
- safer firmware runtime changes applied
- battery scaling corrected with calibration factor
- ground README corrected
- AI handoff context written
- shared Teensy build/upload guide added at [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md)
- dynamic UI page switching implemented:
  - ground, ready, launch, flight, landed, lost
- readiness UI improved:
  - state, system status, battery, GPS, link, sanity-check altitude/velocity, packet rates, rocket health
- recovery/lost UI improved:
  - last packet age, RSSI, battery, distance, bearing, coordinates, altitude, reset hint
- common bottom status strip implemented for link/battery/system warnings
- `SYS` is now an active overall status indicator
- `LOG` is now an active aggregate logging indicator
- rocket display name is configurable from SD using `/ground_config.txt`
- rocket display name updates from rocket LoRa identity telemetry when received
- rocket and ground packet definitions are aligned
- SD logging no longer writes repeated PAD rows before first rocket packet

Still not fully closed:

- final stability of the fully assembled ground hardware
- root cause confirmation if the assembled unit still crashes or resets
- final validation of display, SD, LoRa, GPS, and RS-485 together

## Priority order

1. Confirm assembled-device stability
2. Validate displayed values and UI behavior
3. Validate telemetry quality and receive behavior
4. Validate logging and SD behavior
5. Improve operator workflow and main live page

## Execution plan

### Phase 1: Stability check in the assembled device

Purpose:

- determine whether the updated safer firmware removed the fast-blink/reset failure

Steps:

1. Install the standalone-flashed `Teensy 4.0` back into the ground controller
2. Power the ground controller normally
3. Observe for at least 1 to 3 minutes
4. Watch for:
   - TFT stability
   - GPS LED behavior
   - RS-485 LEDs
   - Teensy USB enumeration stability
   - page transitions / UI freezes

Pass criteria:

- no rapid white TFT flashing
- no rapid LED reset-loop behavior
- device remains powered and responsive
- USB serial stays present when connected

Fail criteria:

- TFT starts flashing or corrupting
- device disappears from USB
- repeated reset pattern appears

If Phase 1 fails:

- move immediately to Phase 1A isolation

### Phase 1A: Peripheral isolation if instability remains

Purpose:

- identify which attached hardware block is destabilizing the controller

Isolation order:

1. disconnect TFT
2. disconnect SD
3. disconnect LoRa
4. disconnect GPS
5. disconnect RS-485 module

Reason for this order:

- TFT, SD, and LoRa share SPI and are the most likely common bus-conflict source

Pass criteria:

- one configuration becomes stable

Output of this phase:

- name the subsystem or combination that triggers instability

### Phase 2: UI validation

Purpose:

- verify the current ground UI is correct and readable in actual use

Checks:

1. battery display matches measured battery voltage
2. `SYS` color matches overall health expectation
3. `LOG` color matches aggregate logging state
4. `GPS`, `IMU`, `BARO`, `SD`, `NAND` status colors make sense
5. QVGA display content fits the screen cleanly
6. no clipped or corrupted status text remains

Pass criteria:

- all values are readable
- no obvious status misrepresentation
- no clipped health row text

### Phase 3: Telemetry receive validation

Purpose:

- confirm the ground station is receiving rocket data correctly and continuously

Checks:

1. flight packet reception
2. nav packet reception
3. status packet reception
4. rocket state changes appear correctly
5. RSSI and packet age appear reasonable
6. relative altitude and velocity update as expected

Pass criteria:

- all packet classes arrive
- UI updates are coherent
- no stale/frozen rocket data during active reception

### Phase 4: Logging validation

Purpose:

- confirm the safer rate-limited logging behavior works without destabilizing the system

Checks:

1. SD initializes reliably
2. a log file is created
3. line counts increase during operation
4. flight logging rate is bounded
5. recovery logging rate is bounded
6. logging no longer causes visible UI or link instability

Pass criteria:

- SD logging works
- no crashes or resets during active logging

### Phase 5: Operator workflow improvement

Purpose:

- improve field usability after stability is confirmed

Status:

- mostly implemented for the core launch/recovery flow

Implemented:

1. redesign the main live page for actual field use
2. prioritize:
   - rocket state
   - system status
   - link age
   - RSSI
   - battery
   - GPS
   - distance / bearing
   - health
3. reduce need for manual paging during launch and recovery

Remaining tuning should be driven by actual field use and screen readability.

## Specific technical tasks to keep in mind

### Task 0: Build and upload hygiene

- keep ground target as `teensy:avr:teensy40`
- keep rocket target as `teensy:avr:teensy41`
- compile ground with repo-local libraries from `.arduino-user/libraries`
- do not rely on `~/Documents/Arduino/libraries`
- use [BUILD_UPLOAD_TEENSY.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/docs/BUILD_UPLOAD_TEENSY.md) before changing build commands

### Task A: Receive statistics

- track packet counts by type
- track missed sequences
- track last-received age by packet class

Why:

- this gives much better link diagnostics than raw RSSI alone

Status:

- implemented enough for current UI:
  - RX rates by Flight/Nav/Status
  - missed sequence counts
  - ages by packet class on `[FLIGHT]`
  - last RSSI on major screens

### Task B: Remove remaining ambiguity in status semantics

- make sure every displayed health item has a clearly documented meaning
- keep label vs status behavior explicit

Status:

- display field meanings are now documented in [README.md](/Users/k_pochkaev/github/flight_computers/teensy_v0/ground/ground_v10/docs/README.md)

### Task C: Fine battery calibration if needed

Current calibration is based on:

- real `12.10 V`
- old indicated `14.19 V`

If the new displayed value is still off after flashing:

- refine only the calibration factor
- do not change divider constants unless hardware changed

## What not to do yet

Do not start these until the assembled system is stable:

- large UI redesign
- telemetry format expansion
- new packet types
- advanced logging features
- RS-485 feature expansion

Reason:

- stability first
- then observability
- then feature growth

## Recommended immediate next action

The next action should be a physical UI and logging validation with the actual ground unit:

1. power ground station with rocket transmitting nearby
2. confirm `[READY]` readability and bottom strip spacing
3. turn rocket off or block LoRa and confirm `[LOST]` recovery page
4. hold page button for 3 seconds and confirm return to `[GND MODULE]`
5. confirm SD creates no repeated idle PAD spam before first rocket link
6. inspect a ground SD log after a bench run

If assembled hardware instability returns, fall back to peripheral isolation.

## Success definition

Ground controller is considered ready for the next development stage when:

- it is stable in the assembled hardware
- battery reading is acceptably accurate
- UI is readable and coherent
- LoRa receive path is stable
- SD logging works without destabilizing the board
- rocket subsystem health is displayed correctly

After that, the next major work item is field-test validation and any readability adjustments from real use.
