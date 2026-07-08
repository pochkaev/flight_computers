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
- replay can use high-rate IMU data for the rocket model: complete for exported V4 CSV plus sibling `_imu.csv`
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
  - IMU
  - barometer
  - GPS
  - battery
  - flight state/event
  - telemetry snapshot
- service export writes full-state CSV plus optional detail CSVs:
  - `_imu.csv`
  - `_baro.csv`
  - `_gps.csv`
  - `_batt.csv`
  - `_event.csv`
  - `_telem.csv`
  - `_att.csv`
- quick field export can still skip detail CSVs with `export_imu=0`
- optional future export optimization remains possible if full detail CSV export is too slow for field workflow

Pass criteria:

- high-rate IMU logging does not bloat every barometer/GPS record
- export remains deterministic and easy to analyze
- visualizer can consume V4 logs directly

## Step 5: Quaternion Attitude Estimator - Completed

Purpose:

- replace Euler-angle integration with a more stable representation
- improve 3D replay and future control-readiness

Implemented state:

- estimator now stores attitude internally as a quaternion
- gyro integration now runs in quaternion space
- accelerometer correction is disabled when acceleration is far from `1g`
- magnetometer yaw correction is gated by acceleration sanity and magnetic-field magnitude
- existing roll/pitch/yaw telemetry and CSV fields are still derived for compatibility
- exported metadata reports attitude estimator version `3`
- diagnostic flags expose:
  - accel correction active
  - mag correction active
  - gyro-only mode
- compact high-rate attitude records log:
  - quaternion
  - derived roll/pitch/yaw
  - estimator confidence flags
- service export writes `_att.csv`
- replay auto-loads sibling `_att.csv`, uses quaternion interpolation when available, and shows confidence state

Current limitations:

- this is better than the Euler-state estimator but still not control-grade

Future tuning:

- possible clipping/saturation

Pass criteria:

- no artificial roll/pitch flips during boost due to thrust acceleration
- replay uses quaternion attitude when available: complete
- logs make it clear when attitude is gyro-only and may drift: complete

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

## Step 7: Replay and Analysis Upgrade - Completed

Purpose:

- make visualization reflect the new multi-rate logs
- separate real measured data from estimated/interpolated data

Implemented items:

- update replay generator to read V4 logs
- use high-rate IMU stream for rocket body motion
- use barometer stream for altitude
- use GPS stream for ground track
- show sensor-rate summary:
  - full-state samples/sec
  - IMU samples/sec when `_imu.csv` is available
- allow full-state-only replay when quick field export used `export_imu=0`
- update flight report parser to skip exported V4 metadata comment lines
- show estimator confidence on the replay with `_att.csv`
- show richer sensor-rate summary:
  - baro samples/sec
  - GPS samples/sec
  - long stream gaps
- visually mark periods where attitude is gyro-only in the scene, charts, and timeline

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

## Step 10: Dedicated Pyro Module Architecture

Purpose:

- decide whether to move real pyro outputs out of the main Teensy flight computer
- make deployment more robust if the main flight computer resets, brownouts, or software gets stuck
- keep early dual-deploy work safe by testing a separate module in log-only or dummy-load mode before real charges

Candidate architecture:

- main Teensy remains responsible for navigation, LoRa, SD/NAND logging, replay data, and ground telemetry
- dedicated pyro module handles deployment outputs with its own controller, battery, barometer, and local state machine
- possible prototype controller: XIAO SAMD21
- possible prototype sensor: BMP180 for bench redundancy only; prefer a newer barometer such as BMP388/BMP390, MS5607/MS5611, or LPS22/LPS28 for serious deployment work
- pyro module has its own 2S pyro battery, 4 MOSFET outputs, physical arm/safe input, and per-channel enable/configuration
- main Teensy connects to pyro module only with data wires, not by sharing pyro output current paths

Expected advantage:

- if the module is only a MOSFET expander that fires when Teensy sends a command, the gain is small and complexity goes up
- the real value comes when the pyro module is autonomous enough to deploy safely if the main Teensy disappears
- Teensy can still send deployment commands, but the pyro module should verify them against its own barometer, timers, and arming state before firing

Startup/config flow:

- before flight, Teensy sends initial pyro configuration to the module
- configuration should include:
  - channel function: apogee/drogue, main, disabled, future staging/airstart
  - per-channel output enable
  - per-channel log enable
  - apogee delay
  - main deployment altitude
  - minimum time after apogee before main
  - minimum apogee margin above main altitude
  - safety mode/checksum/version
- pyro module stores the active config in RAM and reports it back to Teensy
- ground should eventually show both Teensy config and pyro-module acknowledged config

Flight behavior:

- Teensy may send events such as `APOGEE_DETECTED` or `MAIN_ALLOWED`
- pyro module should not blindly trust a fire command
- pyro module should fire only when:
  - physical arm/safe input is armed
  - channel output is enabled
  - continuity is acceptable, when continuity hardware is available
  - local deployment logic says the event is valid
  - local battery voltage is acceptable
- if Teensy link is lost during flight, pyro module should continue with its own fallback logic
- fallback main logic should be altitude/descent based, not just "main altitude minus 50 m"
- main should deploy on descent when altitude is at or below the configured main altitude, but only if the flight reached safely above that altitude by a configured margin
- if main is configured for `500 ft` and the flight only reaches `400 ft`, the main channel should stay inhibited instead of firing immediately after apogee

Recommended communication:

- use UART with CRC, sequence numbers, and acknowledgements as the first implementation
- avoid I2C between boards for flight-critical inter-board communication because a noisy or reset device can hang the bus
- consider RS485 if wiring gets longer or noisy
- consider CAN only if the project grows into a larger avionics bus

Required module safety features:

- outputs forced inactive on boot/reset
- MOSFET gate pulldowns
- independent pyro battery voltage measurement
- physical arm/safe input
- per-channel output enable
- per-channel continuity sensing
- command timeout/watchdog
- local event log if possible
- status report back to Teensy: armed state, battery, barometer state, continuity, active config, fired/logged events

Prototype plan:

1. Build a log-only or LED/dummy-load XIAO pyro module.
2. Send config from Teensy before flight.
3. Make the module report acknowledged config and local status.
4. Let the module detect apogee/main from its own barometer and log what it would do.
5. Compare module decisions with Teensy/NAND logs after multiple flights.
6. Only after confidence is high, enable real MOSFET outputs for one channel at a time.

Open design questions:

- exact controller and barometer choice
- whether the pyro module needs its own nonvolatile log storage
- continuity sensing circuit and current limit
- connector/pinout between Teensy and pyro module
- packet format and failsafe behavior if config is missing or corrupt
- whether to keep main Teensy pyro outputs as a backup or remove real outputs from Teensy entirely

## Immediate Implementation Order

1. Completed: NAND log metadata.
2. Completed: staged `50 Hz` MS5607 path.
3. Completed: NAND V4 typed records.
4. Completed: high-rate `200 Hz` IMU-only NAND stream.
5. Completed: V4 export tooling, including latest-only and optional IMU export.
6. Completed: finish V4 stream-specific records.
7. Completed: update replay to consume multi-rate logs.
8. Completed: replace Euler attitude internals with quaternion estimator.
9. Add GPS configuration and metadata.
10. Add optional barometer timing benchmark if sample spacing or noise looks suspicious.
11. Draft dedicated pyro module packet format, pinout, and safety state machine before building hardware.
