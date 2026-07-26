# Rocket v10 Flight Recorder and Storage Flow

This document describes the recorder implemented in Rocket firmware
`rv10.20260725l`. The firmware source remains authoritative:

- [config.h](../fw/RocketV10/config.h)
- [storage.cpp](../fw/RocketV10/storage.cpp)
- [settings.cpp](../fw/RocketV10/settings.cpp)

## Design goal

The flight-state kernel must run on fresh sensor data without waiting for an SD
card or NAND filesystem transaction. The current data path is:

```text
sensors -> flight-state kernel -> recorder records -> flight RAM -> QSPI NAND
                                                  \-> LoRa telemetry

                                                    QSPI NAND -> serial download
                                                              -> SD export
```

Sensor acquisition and state/output decisions run before telemetry, storage,
button service, and serial diagnostics. Once the rocket is physically armed,
flight records are held in RAM and written to NAND only after a terminal state
or an explicit SAFE service action.

## Storage responsibilities

| Storage | Purpose | Flight samples written directly? |
|---|---|---:|
| EEPROM | Persistent user settings only | No |
| RAM | Timing-safe prelaunch and flight recorder buffer | Yes |
| QSPI NAND | Authoritative onboard black-box file | Not while armed/in flight |
| Built-in microSD | Field export and file transfer | No |
| Ground microSD | Independent copy of received telemetry and ground power data | N/A |

The rocket can fly and record without an SD card. NAND is the primary onboard
flight store. Runtime SD CSV logging and SD configuration loading are disabled
with `SD_RUNTIME_LOG_ENABLE=0` and `SD_CONFIG_LOAD_ENABLE=0`.

## EEPROM

EEPROM contains one checksummed `RocketRuntimeSettings` structure:

- LoRa spreading factor, bandwidth, coding rate, and transmit power
- flight, navigation, and status telemetry periods
- launch acceleration threshold
- launch IMU confirmation time and minimum integrated delta-v
- launch barometric altitude and velocity thresholds
- minimum apogee altitude
- main-deployment altitude

EEPROM does not contain sensor samples, events, or flight logs. A serial `SET`
changes the live setting; `SAVE` is the only normal command that persists the
current values to EEPROM. `DEFAULTS` loads defaults into RAM and requires
`SAVE` if they should survive reboot. The stored structure has a magic value,
version, size, and checksum; invalid EEPROM data is replaced by defaults at
boot.

## NAND record streams

The NAND file uses typed binary V4 records. All rates below are nominal and
depend on the corresponding sensor producing a new sample.

| Record | Nominal rate | Main contents |
|---|---:|---|
| Full state | 10 Hz | state, altitude, velocity, acceleration, attitude, GPS summary, battery, health and diagnostic flags |
| Wide IMU | 200 Hz normally; 50 Hz during descent states | acceleration, 32-bit gyro, roll, pitch, yaw, state |
| Barometer | 50 Hz | absolute/relative altitude, vertical velocity, pressure, temperature, diagnostics, state |
| Battery | 10 Hz | filtered/raw voltage, ADC pin voltage, pack classification, warning/critical flags |
| GPS | 1 Hz by default | position, absolute/relative altitude, speed, fix quality/age, parser counters, baro-GPS difference |
| Telemetry audit | When each packet is transmitted | packet type and sequence counters, state, health, RSSI, battery |
| Event | Immediately through a deferred event queue | state changes, recovery classifications, pyro requests/output changes, close reason and flight evidence |

The separate 200 Hz quaternion/attitude stream is disabled with
`NAND_ATTITUDE_LOG_ENABLE=0`. The wide IMU stream already records Euler
attitude, and disabling the duplicate stream preserves flight-buffer capacity.

Default telemetry audit cadence is:

- flight packet: 5 Hz
- navigation packet: 1 Hz
- status packet: 0.5 Hz
- identity packet: once per 5 seconds
- pyro configuration packet: once per 10 seconds
- pyro event packet: immediately when queued

After landing, recovery telemetry slows to flight 1 Hz, navigation 0.5 Hz, and
status 0.2 Hz. The onboard log is finalized on entry to `LANDED`, so those
later recovery transmissions are not appended to the closed flight file.

## Records by flight state

`READY`, `LAUNCH CHECK`, `PAD SETTLE`, and similar messages are launch-status
details inside `FS_PAD`; they are not separate flight states.

| Flight state | Recorder destination | Full state | IMU | Barometer | Battery/GPS/telemetry/events |
|---|---|---:|---:|---:|---|
| `IDLE` / physical SAFE | NAND cache/file | 10 Hz | 200 Hz | 50 Hz | Normal rates |
| `PAD` | Bounded 64 KiB prelaunch RAM window | 10 Hz | 200 Hz | 50 Hz | Normal rates |
| `ASCENT_POWERED` | Linear flight RAM | 10 Hz | 200 Hz | 50 Hz | Normal rates |
| `COAST` | Linear flight RAM | 10 Hz | 200 Hz | 50 Hz | Normal rates |
| `SUBSONIC` | Linear flight RAM | 10 Hz | 200 Hz | 50 Hz | Normal rates |
| `NEAR_APOGEE` | Linear flight RAM | 10 Hz | 200 Hz | 50 Hz | Normal rates |
| `DESCENT_BALLISTIC` | Linear flight RAM | 10 Hz | 50 Hz | 50 Hz | Normal rates |
| `UNDER_DROGUE` | Linear flight RAM | 10 Hz | 50 Hz | 50 Hz | Normal rates |
| `DUAL_DEPLOY_APOGEE_LOGGED` | Linear flight RAM | 10 Hz | 50 Hz | 50 Hz | Normal rates |
| `DUAL_DEPLOY_MAIN_LOGGED` | Linear flight RAM | 10 Hz | 50 Hz | 50 Hz | Normal rates |
| `POST_FLIGHT_GROUND` | Linear flight RAM | 10 Hz | 200 Hz | 50 Hz | Normal rates |
| `LANDED` | Final state/event, then commit and close | Stops | Stops | Stops | Flight file stops |
| `ABORT` | Final state/event, then commit and close | Stops | Stops | Stops | Flight file stops |

The 50 Hz descent IMU policy applies from `DESCENT_BALLISTIC` through the
states before `POST_FLIGHT_GROUND`. `POST_FLIGHT_GROUND` currently returns to
the normal 200 Hz IMU rate.

In SAFE `IDLE`, recorder traffic may reach the NAND file cache because flight
RAM capture is inactive. Once the log file has been prepared in `PAD`, records
use the bounded prelaunch RAM window. At launch this changes to a linear flight
buffer so the history is preserved in order.

## RAM capacity and overflow behavior

Flight RAM is split across:

- 160 KiB normal RAM
- 416 KiB DMA RAM
- 576 KiB total
- 64 KiB reserved for critical records
- 512 KiB primary capacity
- 64 KiB maximum bounded prelaunch window

The critical reserve accepts:

- full-state records
- barometer records
- battery records
- event records

IMU, GPS, and telemetry-audit records are noncritical. When the 512 KiB primary
area fills, noncritical records are dropped while critical records continue
into the 64 KiB reserve. If the reserve also fills, critical records are
dropped. `STORAGE STATUS` reports total, critical, and noncritical drop
counters. Recorder saturation does not stop the flight-state kernel or pyro
output timing.

The prelaunch area is bounded rather than an infinite armed-time log. When its
64 KiB limit is reached, the accumulated prelaunch block is discarded and a
new recent block begins. This prevents a long armed wait from consuming the
flight buffer.

Because in-flight data is intentionally held in volatile RAM, complete power
loss before finalization loses the buffered flight portion. The NAND header and
any earlier SAFE cache may remain, but they are not a substitute for the
uncommitted records.

## NAND commit and finalization

RAM records are flushed to NAND and the V4 header is rewritten as finalized
when any of these occurs:

- the state enters `LANDED`
- the state enters `ABORT`
- the local button is held for about two seconds while physical SAFE is
  recognized; outputs are stopped, the log is finalized, and the Teensy fully
  reboots
- `FLIGHT RESET CONFIRM` is accepted whenever physical SAFE is recognized; it
  finalizes the file and resets the flight state without rebooting
- a SAFE storage export or erase service finalizes the active file first

An ARM long-button press is rejected and does not modify the log or state.
`NAND READ` also refuses to read the active log; finalize it first or use the
export command, which performs the service finalization.

The finalized header contains firmware and rocket identity, format/rate
metadata, record count, close time/reason, IMU ranges, and estimator version.
Current files use `/fltNNNN.bin`. Before opening a new log, rotation removes
the oldest logs as needed to retain at least 16 MiB free and fewer than 96 log
files.

## Rocket SD flow

The microSD card is mounted and can be serviced, but it is not in the
flight-critical write path:

- runtime CSV logging is disabled
- SD-based runtime configuration is disabled
- NAND logs can be exported to SD as summary or full CSV sets
- exported SD files can be listed and downloaded over USB serial
- SD logs can be erased with an explicit confirmation command

Legacy boot service through `/nand_ops.txt` still exists for compatibility.
For field use, prefer the explicit serial commands below because they report
the result immediately and do not require editing a command file.

## Rocket serial commands

Storage and settings mutations require the physical switch to be SAFE and the
flight state to be `IDLE` or `PAD`. `FLIGHT RESET CONFIRM` is the deliberate
exception: it requires physical SAFE but can recover a falsely latched flight
state.

Settings and diagnostics:

```text
HELP
SHOW
TIMING
TIMING RESET
SET LORA_SF <6..10>
SET LORA_BW <62500|125000|250000|500000>
SET LORA_CR <5..8>
SET TX_POWER_DBM <2..17>
SET FLIGHT_TX_MS <milliseconds>
SET NAV_TX_MS <milliseconds>
SET STATUS_TX_MS <milliseconds>
SET LAUNCH_ACCEL_G <g>
SET LAUNCH_IMU_CONFIRM_MS <milliseconds>
SET LAUNCH_IMU_DELTA_V_MPS <m/s>
SET LAUNCH_BARO_ALT_M <metres>
SET LAUNCH_BARO_VEL_MPS <m/s>
SET APOGEE_MIN_ALT_M <metres>
SET MAIN_ALT_M <metres>
SAVE
DEFAULTS
FLIGHT RESET CONFIRM
```

NAND and SD service:

```text
STORAGE STATUS
NAND LIST
NAND INFO <index>
NAND READ <index> <offset> <length|0-to-end>
NAND EXPORT SD ALL SUMMARY
NAND EXPORT SD ALL FULL
NAND EXPORT SD <index> SUMMARY
NAND EXPORT SD <index> FULL
NAND ERASE ALL CONFIRM
SD MOUNT
SD LIST
SD INFO <filename>
SD READ <filename> <offset> <length|0-to-end>
SD ERASE LOGS CONFIRM
ERASE NAND CONFIRM
ERASE SD CONFIRM
ERASE ALL CONFIRM
```

Reads use checksummed transfer frames and support offset/resume. The helper
[flight_storage.py](../tools/flight_storage.py) automates listing, downloading,
and resuming files, which is preferable to manually copying a large serial
transfer.

## Ground station logging

Ground firmware `gv10.20260725i` has no NAND. It independently writes received
rocket telemetry and ground measurements to its SD card:

- PAD telemetry snapshot every 5 seconds
- in-flight snapshot every 200 ms
- recovery snapshot every 1 second
- signal-lost snapshot every 30 seconds
- independent ground-only snapshot every 30 seconds
- launch power samples every 100 ms for 3 seconds after START

While an ARM/START launch-control sequence is active, ground log lines are held
in a 16 KiB RAM buffer and SD file operations are deferred so a slow SD card
cannot starve radio and launch control. The buffer is flushed when the controls
return to SAFE. Ground SD serial commands require both ARM switches SAFE and
START released:

```text
SD MOUNT
SD LIST
SD INFO <filename>
SD READ <filename> <offset> <length|0-to-end>
SD ERASE LOGS CONFIRM
```

Ground logs are valuable corroboration, but they only contain packets that
reached the ground station. The rocket NAND log is authoritative for onboard
sensor evidence and state transitions.

## Field checks

Before flight:

1. Put the rocket in physical SAFE.
2. Run `SHOW`, `TIMING RESET`, and `STORAGE STATUS`.
3. Confirm NAND is healthy, RAM drop counters are zero, and runtime SD logging
   is reported as disabled.
4. An SD card is optional for flight; use one only if a field export is wanted.
5. Arm only after the normal pad-settle and readiness sequence.

After a flight or bench state transition:

1. Prefer allowing `LANDED` to finalize automatically.
2. If the state is stuck, return the switch to physical SAFE and hold the local
   button for about two seconds. This finalizes the log and fully reboots.
3. Run `NAND LIST`, inspect the newest file with `NAND INFO <index>`, and check
   `STORAGE STATUS` for dropped records.
4. Download directly over serial or export to SD. Do not remove power while a
   commit/export is in progress.

## Known recorder limits

- Sudden power removal before finalization loses buffered flight RAM.
- A long or falsely latched nonterminal state can fill the primary buffer and
  cause noncritical record loss.
- `POST_FLIGHT_GROUND` currently records IMU at 200 Hz and can consume remaining
  capacity quickly.
- The bounded PAD window is block-reset when full, not a byte-perfect circular
  ring.
- Firmware exports only the current V4 format; unsupported legacy files are
  skipped.
