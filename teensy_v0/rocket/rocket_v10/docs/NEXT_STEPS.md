# Rocket v10 Next Steps

This project is now moving toward the architecture style of `HPR-Rocket-Flight-Computer`: higher-rate sensor capture, richer flight logs, better attitude estimation, and post-flight analysis that is driven by high-quality onboard data.

The goal is not to copy that project directly. The goal is to evolve RocketV10 step by step while preserving the parts that already work for our hardware: LoRa ground link, SD/NAND recovery workflow, and the current RocketV10 packet protocol.

## Direction

- keep LoRa bandwidth low and stable for live ground display
- make onboard logs the source of truth for post-flight analysis
- increase local sensor capture rates, especially IMU and barometer
- improve attitude estimation enough for useful 3D replay
- keep production firmware understandable and field-serviceable
- keep temporary hardware experiments outside the production repository
- preserve physical SAFE/ARM behavior and independent recovery during validation
- keep quaternion attitude out of flight-critical decisions until multiple
  flights agree with video, barometer, and GPS evidence

## Current baseline — 2026-07-27

The current installed Rocket V10 firmware is `rv10.20260727b`.

Confirmed on the assembled flight computer:

- IMU calibration is checksummed in EEPROM with `CAL_FLAGS 7`
- sensor-to-airframe alignment version `2` is valid
- a real all-power-off reboot returned to the same reference pose within about
  `3.1 degrees`
- the current calibration values are archived in
  [`../calibration/rocket_v10_imu_20260727.json`](../calibration/rocket_v10_imu_20260727.json)
- quaternion attitude is logged and visualized but does not affect launch,
  apogee, recovery state, landed detection, or pyro decisions
- physical pyro control is currently `NONE`; deployment decisions are logged
  only
- Rocket NAND is the authoritative onboard flight record; Ground SD is
  independent radio/link corroboration

The project is now in flight-validation rather than feature-expansion mode.
Freeze and identify this baseline before making another major firmware change.

## Step 1: Log Metadata - Completed

Purpose:

- make every exported flight log self-describing
- make future analysis scripts know exactly how data was captured

Implemented state:

- NAND V4 header stores firmware version
- NAND V4 header stores rocket name loaded from SD config, falling back to the firmware default
- NAND V4 header stores configured sensor rates:
  - IMU rate
  - barometer rate
  - NAND log rate
  - SD log rate
  - LoRa packet rates
- NAND V4 header stores IMU range settings:
  - accelerometer range
  - gyro range
  - magnetometer gain
- NAND V4 header stores attitude estimator version
- NAND V4 header stores record format version and log flags
- exported CSV files print this metadata as `# key=value` lines before sample rows

Pass criteria:

- exported NAND CSV files show what firmware/settings produced them
- current V4 logs can be compared without guessing which firmware was used

## Step 2: Faster Barometer Driver - Completed

Purpose:

- improve altitude and vertical velocity timing
- reduce blocking time in the main loop
- move closer to the HPR-style `30-100 Hz` barometer data path

Implemented state:

- RocketV10 barometer scheduler is now `20 ms`, targeting `50 Hz`
- MS5607 reads use staged conversions instead of two blocking conversion waits in one sample
- pressure/altitude are the fast path
- temperature compensation refreshes every `200 ms`, about `5 Hz`
- vertical velocity is derived over a `120 ms` barometer window to keep bench noise from looking like real climb rate

Future tuning:

- compare altitude/velocity noise against more flight data
- later evaluate `75-100 Hz` if noise and loop timing allow

Pass criteria:

- barometer sampling no longer blocks the loop for the full pressure plus temperature conversion time
- exported metadata reports `baro_hz=50`
- altitude records update faster than the old `20 Hz` path
- vertical velocity uses the `120 ms` barometer window instead of the old slower path

## Step 3: High-Rate IMU Logging - Completed

Purpose:

- capture boost and tumble dynamics with much more detail
- move toward the HPR-style high-rate inertial logging path

Implemented state:

- accelerometer and gyro are coherently sampled at up to `200 Hz`
- fresh magnetometer samples are recorded at up to `25 Hz`
- NAND full-state records are written at `10 Hz`
- current type-10 quaternion IMU records are written at up to `200 Hz`,
  reducing to `50 Hz` during descent states
- LoRa packet rates are unchanged
- runtime SD logging is disabled
- SAFE-only service export generates a main CSV and all available detail CSVs

Implemented items:

- added 40-byte high-rate quaternion IMU records
- included measured `dt_us`, wide gyro values, quaternion, estimator
  confidence, and quality flags
- added independent fresh-magnetometer records
- verified full export with `_imu.csv`
- verified NAND erase after successful export
- documented that full multi-log IMU CSV export is slow because Teensy formats large text CSV streams

Pass criteria:

- exported logs contain a high-rate `_imu.csv` with accel/gyro data: complete
- replay uses the recorded quaternion for rocket orientation: complete
- no LoRa rate increase is required: complete

## Step 4: NAND Log Format V4 - Completed

Purpose:

- support different sensors at different frequencies without wasting space
- avoid forcing every record to contain every field

Implemented items:

- define `RV10NLG` V4 record format
- do not keep legacy NAND decoders in production firmware
- export supports the current production format only
- add per-record type byte
- add per-record timestamp for IMU records
- add sequence counters
- add stream-specific records:
  - full state
  - quaternion IMU
  - magnetometer
  - barometer
  - GPS
  - battery
  - flight state/event
  - telemetry snapshot
  - calibration metadata
  - sensor-to-airframe alignment metadata
- service export writes full-state CSV plus optional detail CSVs:
  - `_imu.csv`
  - `_baro.csv`
  - `_gps.csv`
  - `_batt.csv`
  - `_event.csv`
  - `_telem.csv`
  - `_att.csv`
- a 64 KiB rolling PAD window prevents a long armed wait from consuming the
  flight buffer
- armed/in-flight records use 512 KiB primary RAM plus a 64 KiB critical
  reserve and are committed to NAND only after a terminal state
- optional future export optimization remains possible if full detail CSV
  export is too slow for field workflow

Pass criteria:

- high-rate IMU logging does not bloat every barometer/GPS record
- export remains deterministic and easy to analyze
- visualizer can consume V4 logs directly

## Step 5: Quaternion Attitude Estimator - Completed

Purpose:

- replace Euler-angle integration with a more stable representation
- improve 3D replay and future control-readiness

Implemented state:

- estimator version `4` stores and corrects attitude directly as a quaternion
- gyro integration uses calibrated samples and measured microsecond intervals
- accelerometer correction requires magnitude and quaternion-innovation checks
- magnetometer correction requires fresh data, field magnitude, and yaw-innovation checks
- boost is gyro-only and magnetometer correction is restricted to preflight/recovery ground
- SAFE-only gyro, six-face accelerometer, and magnetometer calibration is checksummed in EEPROM
- existing roll/pitch/yaw telemetry and CSV fields are still derived for compatibility
- exported metadata reports attitude estimator version `4`
- diagnostic flags expose:
  - accel correction active
  - mag correction active
  - gyro-only mode
- type-10 high-rate IMU records log raw accel/gyro, measured `dt_us`,
  quaternion, confidence, saturation, rejection, and sample-gap flags
- type-11 records preserve the calibration snapshot used by the flight
- type-12 records preserve fresh magnetometer samples
- type-13 records preserve the sensor-to-airframe alignment used by the flight
- service export writes quaternion and quality fields directly into `_imu.csv`
- replay uses logged quaternion interpolation and shows confidence state
- deterministic gravity-plus-horizontal-magnetic startup avoids the installed
  nose-up gravity singularity
- aligned output uses airframe `+Z` through the rocket nose

Current limitations:

- this is better than the Euler-state estimator but still not control-grade
- aggressive motion or thrust correctly causes temporary low confidence and
  gyro-only propagation
- magnetometer evidence is susceptible to nearby steel, current, and motors

Future tuning:

- current LSM9DS1 still has limited margin at `±16 g` and `±2000 dps`
- data-ready polling is used; FIFO/interrupt wiring is not yet available
- tune only from real flight evidence, not from a single bench movement

Pass criteria:

- no artificial roll/pitch flips during boost due to thrust acceleration
- replay uses quaternion attitude when available: complete
- logs make it clear when attitude is gyro-only and may drift: complete
- installed calibration, alignment, and cold-reboot repeatability: complete

## Step 6: GPS Data Logging - Completed

Purpose:

- preserve GPS timing and quality evidence independently from the barometer
- improve mapped ground-track and recovery analysis

Implemented state:

- type-4 records contain position, absolute/relative altitude, speed, fix age,
  HDOP, satellites, validity flags, parser counters, and barometer/GPS
  difference
- GPS remains informational and independent from launch, apogee, recovery,
  deployment, and landed decisions
- the analyzer maps valid GPS fixes and separates GPS ground track from
  barometric altitude

Pass criteria:

- exported logs show GPS timing and quality clearly: complete
- replay uses only validated fixes: complete
- GPS cannot block flight-state logic: complete

## Step 7: Replay and Analysis Upgrade - Completed

Purpose:

- make visualization reflect the new multi-rate logs
- separate real measured data from estimated/interpolated data

Implemented items:

- `visualizer/rocket_v10_log_analyzer.py` reads V4 multi-rate exports
- use high-rate IMU stream for rocket body motion
- use barometer stream for altitude
- use GPS stream for ground track
- show sensor-rate summary:
  - full-state samples/sec
  - IMU samples/sec when `_imu.csv` is available
- allow summary-only analysis when detail CSV streams are not exported
- update flight report parser to skip exported V4 metadata comment lines
- show estimator confidence on the replay with `_att.csv`
- show richer sensor-rate summary:
  - baro samples/sec
  - GPS samples/sec
  - long stream gaps
- visually mark periods where attitude is gyro-only in the scene, charts, and timeline
- `visualizer/rocket_v10_imu_demo.py` creates a self-contained offline bench
  orientation replay from `IMU STREAM` captures

Pass criteria:

- replay is smooth without inventing fake data
- user can tell which parts of attitude/position are trustworthy

## Step 8: Flight Validation Campaign — Current priority

Purpose:

- prove the complete recorder and state machine with real flights
- compare onboard attitude against independent evidence
- change thresholds only when flight data demonstrates a need

Flight rollout:

1. Freeze and identify the firmware, settings, and calibration used.
2. Keep motor or another independent system responsible for parachute
   deployment; keep Rocket V10 pyro output disabled.
3. Record high-frame-rate ground video when practical.
4. After every flight, preserve Rocket NAND and Ground SD logs before erasing
   either device.
5. Generate the analyzer report and review:
   - launch evidence and launch timestamp
   - all flight-state transitions
   - barometer altitude/velocity quality and apogee decision delay
   - IMU rate, sample gaps, saturation, confidence, and quaternion norm
   - `airframe_aligned`, accelerometer correction, magnetometer correction,
     and gyro-only coverage
   - GPS fix quality and ground track
   - LoRa RSSI, packet age, and missed packets from Ground logs
   - RAM usage, dropped records, finalization, and close reason
6. Compare orientation and recovery events with video and trajectory.

Acceptance target:

- several successful low-power flights and at least one representative
  higher-power flight
- zero critical recorder drops
- no unexplained state transitions or loop stalls
- attitude behavior consistent with video within the limits of the current IMU
- complete finalized onboard logs after landing

Do not make quaternion attitude flight-critical merely because one flight
looks good.

## Step 9: Power-loss-resilient flight recording

Purpose:

- preserve useful flight evidence if power is lost before `LANDED` or `ABORT`
- retain the current rule that storage must not delay the flight-state kernel

Current limitation:

- armed/in-flight records remain in volatile RAM
- sudden power removal before finalization loses the buffered flight portion
- direct NAND filesystem operations previously caused pauses near `80 ms`

Investigation order:

1. Measure commit time and worst-case loop effects with the current NAND.
2. Prototype bounded, asynchronous checkpoints outside the time-critical path.
3. Evaluate dedicated SPI/QSPI FRAM or MRAM if NAND checkpoints cannot meet
   timing requirements.
4. Add a recoverable segment format with sequence numbers, CRC, and explicit
   finalization state.
5. Fault-test resets and power removal at PAD, boost, coast, descent, and
   post-flight states.

Pass criteria:

- no flight-kernel deadline regression
- after injected power loss, the latest valid segment can be recovered
- a corrupt/incomplete segment cannot hide earlier valid records

## Step 10: GPS configuration and validation

Implement Step 6 after the current flight baseline is preserved:

- identify the exact module and supported binary configuration protocol
- configure a known airborne/dynamic model
- target `5 Hz` and reduce unused NMEA output
- verify actual update rate, fix age, parser errors, and serial bandwidth
- store commanded and observed GPS configuration in log metadata
- compare GPS track with the existing barometer and mapped replay

GPS remains informational and must not become a launch, apogee, deployment, or
landed dependency.

## Step 11: Deployment qualification

Purpose:

- progress from log-only decisions to deployment outputs without making the
  rocket dependent on unproven software

Required order:

1. Keep `PYRO_CONTROL NONE` during the flight-validation campaign.
2. Replay real flight logs through the deployment state machine.
3. Bench-test with LEDs and resistive dummy loads.
4. Verify boot/reset output-off behavior, gate pulldowns, physical SAFE,
   continuity, current sensing, battery limits, pulse timing, and watchdogs.
5. Perform inert e-match tests in a controlled setup.
6. Enable one output/function at a time while retaining an independent
   recovery system.

An autonomous dedicated pyro module remains a possible future architecture.
It is valuable only if it has its own power, physical arming, barometer,
validated local state machine, continuity sensing, watchdog, and event log.
A simple remote MOSFET expander adds complexity without meaningful
independence. UART with CRC, sequence numbers, and acknowledgements is the
preferred first inter-board transport; do not use a shared I2C bus for a
flight-critical module.

Pass criteria:

- independent recovery remains available
- no output can energize during boot, reset, SAFE, corrupt configuration, or
  missing communication
- logged and physical output events agree across repeated tests

## Step 12: Hardware and system robustness

Evaluate after the current flight campaign identifies real limits:

- lower-drift IMU with suitable gyro range
- dedicated high-g accelerometer
- redundant barometer
- vibration isolation validated against flight data
- robust locking connectors and power-loss protection
- thermal testing for both Rocket and Ground modules
- Ground display watchdog/reinitialization while keeping launch control
  independent of display health

Active stabilization is a separate, long-term project. Keep actuator/control
code out until the estimator, sensors, timing, power, redundancy, and flight
evidence are control-grade.

## Field workflow

Before flight:

1. Cold-boot in the final pad orientation without UART service mode.
2. Confirm expected firmware and settings.
3. Confirm barometer, IMU, NAND, LoRa, and battery health.
4. Confirm `CAL_FLAGS 7`, valid alignment, zero recorder drops, and intended
   pyro mode.
5. Wait for the normal pad-settle and `READY` sequence before launch.

After flight:

1. Keep the rocket powered until the log is finalized.
2. Return to physical SAFE.
3. Inspect `NAND LIST`, `NAND INFO`, `STORAGE STATUS`, and timing counters.
4. Download and archive Rocket NAND and Ground SD logs.
5. Generate the visual report and record findings before changing firmware.

## Immediate implementation order

1. Freeze, commit, and tag the current tested Rocket/Ground baseline.
2. Run the controlled flight-validation campaign and review every flight.
3. Fix only issues supported by recorded evidence.
4. Design and test power-loss-resilient recording.
5. Configure and validate GPS at `5 Hz`.
6. Qualify deployment hardware and logic with dummy loads and independent
   recovery.
7. Consider hardware sensor upgrades from measured flight limitations.
