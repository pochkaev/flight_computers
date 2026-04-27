# Rocket v10 Next Steps

This project is now moving toward the architecture style of `HPR-Rocket-Flight-Computer`: higher-rate sensor capture, richer flight logs, better attitude estimation, and post-flight analysis that is driven by high-quality onboard data.

The goal is not to copy that project directly. The goal is to evolve RocketV10 step by step while preserving the parts that already work for our hardware: LoRa ground link, SD/NAND recovery workflow, and the current RocketV10 packet protocol.

## Direction

- keep LoRa bandwidth low and stable for live ground display
- make onboard logs the source of truth for post-flight analysis
- increase local sensor capture rates, especially IMU and barometer
- improve attitude estimation enough for useful 3D replay
- keep production firmware understandable and field-serviceable
- use separate benchmark/test sketches for risky hardware experiments

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

- IMU is sampled at `200 Hz`
- NAND full-state records are still written at `50 Hz`
- NAND compact IMU records are written at `200 Hz`
- LoRa and SD CSV rates are unchanged
- service export can generate full-state CSV and optional high-rate `_imu.csv`
- quick field export is supported with `export_latest_only=1` and `export_imu=0`

Implemented items:

- added high-rate IMU-only NAND records
- started with `200 Hz`
- kept the existing low-rate SD CSV path readable
- V4 typed NAND records are now the current production format:
  - type `1`: full state
  - type `2`: IMU
- verified full export with `_imu.csv`
- verified fast latest-only export without `_imu.csv`
- verified NAND erase after successful export
- documented that full multi-log IMU CSV export is slow because Teensy formats large text CSV streams

Pass criteria:

- exported logs contain a high-rate `_imu.csv` with accel/gyro data: complete
- replay can use high-rate IMU data for the rocket model: data is available; replay parser update remains in Step 7
- no LoRa rate increase is required: complete

## Step 4: NAND Log Format V4

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
  - IMU

Remaining items:

- add stream-specific records:
  - barometer
  - GPS
  - battery
  - flight state/event
  - telemetry snapshot
- optional future export optimization if full IMU CSV export remains too slow for field workflow

Pass criteria:

- high-rate IMU logging does not bloat every barometer/GPS record
- export remains deterministic and easy to analyze
- visualizer can consume V4 logs directly

## Step 5: Quaternion Attitude Estimator

Purpose:

- replace Euler-angle integration with a more stable representation
- improve 3D replay and future control-readiness

Current state:

- estimator uses gyro integration with gated accel/mag correction
- accelerometer correction is disabled when acceleration is far from `1g`
- this is better than the old estimator but still not control-grade

Planned items:

- store attitude as a quaternion internally
- integrate gyro rates in quaternion space
- correct tilt from accelerometer only when acceleration magnitude is sane
- correct yaw from magnetometer only when magnetic field magnitude/direction is sane
- expose estimator confidence flags:
  - accel correction active
  - mag correction active
  - gyro-only mode
  - possible clipping/saturation
- log quaternion or compact attitude record to NAND

Pass criteria:

- no artificial roll/pitch flips during boost due to thrust acceleration
- replay uses quaternion attitude when available
- logs make it clear when attitude is gyro-only and may drift

## Step 6: GPS Configuration

Purpose:

- make GPS behavior intentional instead of passive
- improve ground track and recovery data

Planned items:

- configure GPS update rate if supported by the module
- configure airborne/dynamic model if supported
- reduce unused NMEA messages
- log GPS fix age, HDOP, satellites, and validity flags clearly
- target `5 Hz` first
- evaluate `10 Hz` only if the module and serial bandwidth support it

Pass criteria:

- GPS update rate is known from configuration/log metadata
- exported logs show GPS timing and quality clearly
- replay ground track is smoother and easier to trust

## Step 7: Replay and Analysis Upgrade

Purpose:

- make visualization reflect the new multi-rate logs
- separate real measured data from estimated/interpolated data

Planned items:

- update replay generator to read V4 logs
- use high-rate IMU stream for rocket body motion
- use barometer stream for altitude
- use GPS stream for ground track
- show estimator confidence on the replay
- show sensor-rate summary:
  - IMU samples/sec
  - baro samples/sec
  - GPS samples/sec
  - dropped records
- visually mark periods where attitude is gyro-only

Pass criteria:

- replay is smooth without inventing fake data
- user can tell which parts of attitude/position are trustworthy

## Step 8: Bench Test Sketches

Purpose:

- test hardware limits without adding temporary code to flight firmware

Planned sketches:

- NAND/SD storage benchmark
- IMU maximum sustainable sample-rate test
- barometer non-blocking timing test
- GPS configuration/rate test
- full loop timing profiler
- NAND V4 export validator

Pass criteria:

- each risky subsystem can be tested alone
- production firmware stays clean
- benchmark results are documented before field use

## Step 9: Long-Term Control Readiness

Purpose:

- keep future active stabilization possible without committing to it now

Planned items:

- log control-relevant state at high rate
- estimate body attitude and angular rates robustly
- add sensor saturation flags
- add loop timing and latency measurements
- keep actuator/control code out until estimator and logs are trustworthy

Possible future hardware:

- better IMU with lower gyro drift
- dedicated high-G accelerometer
- faster barometer or better pressure sensor
- GPS module with known high-rate airborne mode

## Immediate Implementation Order

1. Completed: NAND log metadata.
2. Completed: staged `50 Hz` MS5607 path.
3. Completed: NAND V4 typed records.
4. Completed: high-rate `200 Hz` IMU-only NAND stream.
5. Completed: V4 export tooling, including latest-only and optional IMU export.
6. Update replay to consume multi-rate logs.
7. Replace Euler attitude internals with quaternion estimator.
8. Add GPS configuration and metadata.
9. Add optional barometer timing benchmark if sample spacing or noise looks suspicious.
