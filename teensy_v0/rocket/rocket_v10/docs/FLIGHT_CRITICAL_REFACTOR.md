# Rocket v10 Flight-Critical Refactor

Status: first development implementation installed as `rv10.20260725f`;
physical pyro outputs remain disabled.

## Implemented development recorder

- Sensor acquisition and flight-state evaluation run before telemetry/storage.
- SD runtime logging and periodic NAND header rewrites are disabled.
- PAD/ARM/flight records use a 576 KiB RAM buffer; PAD retains a rolling
  64 KiB pre-launch window.
- NAND filesystem writes are deferred until LANDED/ABORT or an explicit SAFE
  service operation.
- IMU remains 200 Hz through ascent/coast and becomes 50 Hz after ballistic
  descent begins, extending RAM capture duration.
- Runtime counters expose loop gaps, deadline misses, storage latency, RAM
  usage, and dropped records through `TIMING` and `STORAGE STATUS`.
- IMU-only launch confirmation requires continuous fresh acceleration and at
  least 1.0 m/s integrated axial delta-V. A single shock sample cannot launch.
- `FLIGHT RESET CONFIRM` is physical-SAFE-only and commits buffered records
  before returning an accidental bench flight to PAD.

The RAM buffer deliberately trades power-loss durability for flight timing:
total power loss before the post-flight commit loses the uncommitted records.
This is safer for state detection than allowing an unbounded filesystem pause,
but it is not the final recorder architecture.

## Non-negotiable architecture rules

1. Sensor acquisition, flight-state evaluation, watchdog service, and pyro
   output shutoff must never wait for NAND, SD, LoRa, GPS, USB serial, or
   human-readable formatting.
2. NAND is the primary in-flight binary black box.
3. SD is used for boot-time configuration and post-flight export. No SD write
   is allowed while the Rocket is READY or in an active flight state unless a
   future bounded-latency implementation passes fault-injection testing.
4. No CSV formatting, filesystem scan, erase, directory operation, header
   rewrite, seek-to-start, or periodic filesystem flush is allowed in the
   flight-critical path.
5. Physical pyro outputs remain disabled until replay, timing, watchdog,
   hardware-in-loop, and dummy-load acceptance tests pass.

## 25Jul timing evidence

The exported V4 NAND full-state summaries show 20 ms recording bursts
separated by large synchronous stalls:

| NAND log | Duration | Effective full-state rate | Gaps over 500 ms | Worst gap |
|---|---:|---:|---:|---:|
| 0001 | 82.2 s | 23.7 Hz | 26 | 8.628 s |
| 0002 | 35.1 s | 37.6 Hz | 0 | 0.452 s |
| 0003 | 131.4 s | 20.6 Hz | 91 | 1.217 s |
| 0004 | 450.2 s | 7.2 Hz | 402 | 1.604 s |
| 0005 | 2.9 s | 48.7 Hz | 0 | 0.050 s |
| 0006 | 1630.3 s | 2.6 Hz | 1412 | 1.928 s |
| 0007 | 730.8 s | 4.8 Hz | 664 | 1.774 s |

Acceptance requirement: storage failure or an injected multi-second storage
stall must not change sensor sampling, flight-state timing, output shutoff, or
watchdog timing.

The first 30-second PAD measurement after enabling RAM capture reported:

- worst loop gap: `16,189 us`;
- major stalls: `0`;
- NAND/SD writes during the window: `0`;
- RAM record drops: `0`.

The previous synchronous build measured a `523,918 us` worst loop gap in a
comparable window.

## Launch replay evidence

The 25Jul successful-flight summary (`rocket_nand_0004_op20260725.csv`)
contains a continuous axial impulse of approximately `3.25 m/s` within
`63 ms` at launch. The `rv10.20260725f` IMU-only threshold is `1.0 m/s` plus
`50 ms` continuous fresh evidence, so that recorded launch passes with margin.
The controlled bench touch entered LAUNCH CHECK and returned to READY.

The detailed 200 Hz IMU export was not retained for this flight
(`export_imu=0`), so this result uses the available 50 Hz full-state stream.

## RAM capacity limit

At current record rates the 576 KiB buffer holds approximately:

- `46 s` if 200 Hz IMU and 50 Hz full-state logging continue throughout;
- about `70 s` for a representative 10-second ascent/coast followed by 50 Hz
  descent IMU logging.

The current flight build reduces the redundant full-state snapshot to `10 Hz`
while retaining dedicated `200 Hz` IMU and `50 Hz` barometer streams. The
576 KiB flight RAM is split into a 512 KiB primary area and a 64 KiB critical
reserve. After the primary area fills, full-state, barometer, battery, and
event records continue into the reserve while IMU/GPS/telemetry records are
dropped and counted. The deterministic capacity check estimates roughly
108 seconds for a typical 10-second ascent plus descent, followed by about
28 seconds of critical-only retention.

## Serial storage management

The normal service interface will be USB serial. The `/nand_ops.txt` SD
workflow may remain only as a legacy recovery mechanism.

All storage commands are rejected unless the physical switch is SAFE and the
state is IDLE/PAD. A storage operation must abort if the state changes.

Implemented read-only commands:

```text
STORAGE STATUS
NAND LIST
NAND INFO <index>
NAND READ <index> <offset> <length>
SD LIST
SD INFO <filename>
SD READ <filename> <offset> <length>
```

Copy/export commands:

```text
NAND EXPORT SD <index> SUMMARY
NAND EXPORT SD <index> FULL
NAND EXPORT SD ALL SUMMARY
NAND EXPORT SD ALL FULL
```

Implemented destructive commands:

```text
NAND ERASE ALL CONFIRM
SD ERASE LOGS CONFIRM
```

Requirements:

- exact confirmation token for every destructive command;
- path and filename allow-listing;
- configuration and service files protected from `SD ERASE LOGS`;
- active logs finalized before a SAFE-only operation and logging restarted
  afterward;
- raw serial download uses framed binary chunks with index, offset, length,
  sequence, and integrity value;
- downloads support resume by offset and the host-side
  `tools/flight_storage.py` verifies every 1024-byte frame CRC plus the final
  segment byte count and CRC;
- serial backpressure is allowed only in SAFE/PAD and never executes in the
  flight-critical path.

## Planned in-flight records

| Stream | Rate while READY/flight |
|---|---:|
| Raw accelerometer and gyro | 200 Hz |
| Magnetometer | 50 Hz |
| Raw pressure and temperature | 50 Hz |
| Filtered altitude and vertical velocity | 50 Hz |
| Flight state and predicate snapshot | 100 Hz |
| Battery | 20 Hz |

The current implemented rates differ from the older planning table above:
full-state is `10 Hz`, battery is `10 Hz`, and the dedicated barometer stream
contains filtered altitude and velocity at `50 Hz`.

Field examples:

```text
python3 tools/flight_storage.py --port /dev/cu.usbmodem... list nand
python3 tools/flight_storage.py --port /dev/cu.usbmodem... download nand 15 flight15.bin
python3 tools/flight_storage.py --port /dev/cu.usbmodem... download nand 15 flight15.bin --resume
python3 tools/flight_storage.py --port /dev/cu.usbmodem... list sd
python3 tools/flight_storage.py --port /dev/cu.usbmodem... download sd filename.csv filename.csv
```
| GPS | Every new fix, up to 10 Hz |
| Scheduler and deadline health | 10 Hz |
| State, fault, and output events | Immediate |

Records are fixed-size binary data with monotonic timestamps and sequence
numbers. Human-readable CSV is produced only after flight.
