# Rocket V10 IMU Calibration and Quaternion Acceptance

This document describes the production IMU service and the bench acceptance
procedure for firmware `rv10.20260727b`.

The attitude estimator is diagnostic only. Launch detection, recovery,
apogee, landed detection, and all deployment-event logic continue to use the
existing raw body-axis IMU and barometer evidence. Do not make attitude
flight-critical until real flight logs pass review.

## Coordinate frames

The LSM9DS1 does not expose all nine channels in one common native frame.
According to ST's LSM9DS1 datasheet Figure 1, magnetometer `X` and `Y` point
opposite accelerometer/gyro `X` and `Y`; `Z` is shared. The firmware maps
calibrated magnetometer samples as:

- estimator `mag X = -calibrated mag X`
- estimator `mag Y = -calibrated mag Y`
- estimator `mag Z = calibrated mag Z`

This correction is applied after hard/soft-iron calibration, so calibration
values made by earlier firmware remain valid.

The estimator runs in the accelerometer/gyro sensor frame. A separate stored
mounting quaternion converts estimator output into the rocket airframe:

- airframe `+Z` = rocket nose/forward direction
- airframe `X/Y` = transverse axes defined by the alignment template
- rotation around airframe `Z` = rocket roll

This separates sensor calibration from the fixed mechanical installation
angle. Do not assume a fin is exactly aligned with a raw sensor axis.

## What changed

- accelerometer output rate is explicitly `238 Hz`; the flight kernel accepts
  coherent accelerometer/gyro data-ready samples at up to the `200 Hz`
  scheduler rate
- magnetometer freshness is checked independently
- accepted samples use measured microsecond intervals
- missed samples, invalid samples, saturation, maximum interval, and
  magnetometer freshness are counted
- gyro, six-face accelerometer, and assembled-rocket magnetometer calibration
  are stored in a separate versioned/checksummed EEPROM structure
- the estimator propagates a normalized quaternion from calibrated gyro data
- gravity and magnetic corrections use vectors directly, avoiding Euler
  singularities when the rocket is vertical
- accelerometer and magnetometer corrections require magnitude and innovation
  checks
- boost is gyro-only; magnetometer correction is limited to preflight and
  recovery-ground phases
- NAND IMU records contain raw acceleration/gyro, measured `dt_us`, quaternion,
  confidence, and quality flags
- fresh raw magnetometer samples are logged separately at up to `25 Hz`
- the calibration active when a log opens is stored inside that NAND log
- LSM9DS1 magnetometer samples are mapped into the accel/gyro frame before
  fusion
- last evaluated magnetometer health and innovation persist between fresh
  40 Hz magnetic samples instead of falsely reverting to `180 degrees`
- startup attitude uses gravity plus the horizontal magnetic vector, removing
  the undefined-yaw singularity when installed nose-up gravity is almost
  exactly opposite the sensor axis
- a separately checksummed EEPROM record stores the fixed sensor-to-airframe
  mounting quaternion
- logged/telemetry quaternion and Euler angles use the aligned airframe frame
  when alignment is valid

## Safety lock

All `IMU` service commands require:

- physical SAFE
- flight state `IDLE` or `PAD`

Streaming and calibration stop automatically if SAFE/preflight authorization
is lost.

## Serial commands

```text
IMU STATUS
IMU CAL GYRO
IMU CAL ACCEL START
IMU CAL ACCEL ADD
IMU CAL MAG START
IMU CAL MAG STOP
IMU CAL SAVE
IMU CAL RESET CONFIRM
IMU ALIGN STATUS
IMU ALIGN CAPTURE CONFIRM
IMU ALIGN RESET CONFIRM
IMU STREAM START
IMU STREAM STOP
IMU BENCH ZERO
IMU BENCH CHECK X 90
IMU BENCH CHECK Y -90
IMU BENCH CHECK Z 180
```

`IMU STREAM START` emits `IMU_DATA` rows at `20 Hz`; it does not change the
sensor or NAND rate.

## GPS connector maintenance console

The GPS `Serial1` connector can be used for calibration and every other
command without opening the flight computer:

1. Keep the rocket physically SAFE and powered off.
2. Disconnect the GPS from D0/D1. The GPS TX and adapter TX must never drive
   D0 simultaneously.
3. Connect a 3.3 V TTL USB-UART adapter:
   - adapter TX to Teensy D0 / RX1
   - adapter RX to Teensy D1 / TX1
   - adapter GND to rocket GND
   - do not connect adapter 5 V/VCC when the rocket has its own power
4. Open the adapter at `9600 8N1`, no flow control, and send:

   ```text
   SERVICE UART CONFIRM
   ```

5. Wait for `SERVICE SWITCH 115200`.
6. Reopen the adapter at `115200 8N1`. The console reports
   `UART SERVICE ACTIVE GPS DISABLED UNTIL REBOOT`.

Holding the rocket service button during SAFE power-on remains an alternative
activation method.

Maintenance mode requires physical SAFE at boot, disables GPS until the next
reboot, and retains the normal SAFE plus IDLE/PAD command lock. A normal reboot
without holding the button restores the GPS at `9600` baud.

This UART console supports the complete IMU calibration, bench tests, NAND/SD
management, settings, and log reads. It does not implement the Teensy
bootloader; firmware upload still requires native USB/program access.

## Calibration procedure

Disconnect or inhibit every energetic output before handling the rocket.
Keep the physical SAFE switch applied throughout calibration.

### 1. Gyroscope

Place the fully assembled rocket on a rigid, motionless support:

```text
IMU CAL GYRO
```

Do not touch it until `OK IMU CAL GYRO` appears, normally after about
10 seconds of accepted samples. Motion samples are rejected rather than
silently averaged.

### 2. Six-face accelerometer

Start a new six-face set:

```text
IMU CAL ACCEL START
```

Place the flight computer/rocket in each of these six orientations:

- `+X` up
- `-X` up
- `+Y` up
- `-Y` up
- `+Z` up
- `-Z` up

For each orientation, hold it rigidly and run:

```text
IMU CAL ACCEL ADD
```

Wait for the accepted face and `faces=N/6` result before moving it. Repeating
one face replaces that face; it does not falsely complete the set.

### 3. Magnetometer

Calibrate with the computer in the final assembled rocket, with normal
battery, wiring, LoRa antenna, buzzer, fasteners, and recovery hardware:

```text
IMU CAL MAG START
```

Slowly rotate the rocket through as many headings and inclinations as
practical for at least 20–30 seconds. Keep magnets, steel tools, large
currents, speakers, and energized pyro wiring away. Then run:

```text
IMU CAL MAG STOP
```

The firmware rejects insufficient sample count or axis coverage.

### 4. Persist and verify

```text
IMU CAL SAVE
IMU STATUS
```

Power-cycle while SAFE and run `IMU STATUS` again. `CAL_FLAGS 7` means gyro,
accelerometer, and magnetometer calibration all loaded successfully.

## Airframe alignment

Alignment is a separate operation from sensor calibration. Perform it with the
IMU rigidly installed in its final position:

1. Place the rocket nose-up in the center of the printed rotation template.
2. Point the selected reference fin exactly at the template's `0` / airframe
   `+X` direction.
3. Keep the rocket still and wait until `IMU STATUS` reports
   `MAG_HEALTHY 1`, confidence at least `0.90`, and no saturation.
4. Run:

   ```text
   IMU ALIGN CAPTURE CONFIRM
   IMU ALIGN STATUS
   ```

The command stores a normalized `sensor-to-airframe` quaternion immediately
in its own EEPROM record. Alignment record version 2 requires the deterministic
gravity-plus-magnetic startup used by `rv10.20260727b`; older alignment is
invalidated automatically. `ALIGN_VALID 1` and quality flag bit 13 mean
quaternion, Euler, telemetry, and NAND IMU output are in the rocket airframe.
`IMU ALIGN RESET CONFIRM` deletes only mounting alignment; it does not delete
gyro, accelerometer, or magnetometer calibration.

Repeat alignment whenever the IMU board's installed position changes.

## Quaternion bench acceptance

Use a cradle or square blocks so physical angles are known. Place visible
axis/rotation marks on the airframe. Keep the computer inside the assembled
rocket so wiring and magnetic effects are realistic.

1. Hold the rocket in the reference pose for several seconds.
2. Confirm `ALIGN_VALID 1`, then run `IMU BENCH ZERO`.
3. Rotate by a known body-axis angle using the right-hand rule.
4. Hold still for 2–3 seconds.
5. Run the matching check, for example:

   ```text
   IMU BENCH CHECK Z 90
   ```

6. Return exactly to the reference and check:

   ```text
   IMU BENCH CHECK Z 0
   ```

Repeat for:

- `X +90`, `X -90`
- `Y +90`, `Y -90`
- `Z +90`, `Z -90`, `Z 180`, and `Z 360`
- return-to-zero after every move

Bench commands use the aligned airframe quaternion. The firmware calculates
quaternion angular distance, which treats `q` and
`-q` as the same orientation. A check passes when:

- angular error is at most `8 degrees`
- quaternion norm error is at most `0.005`
- estimator confidence is at least `0.60`
- no accel/gyro saturation flag is active

If the physical direction was opposite the right-hand rule, check the
negative angle instead. Do not change axis signs merely to make a test pass;
confirm the actual board/airframe orientation.

## Disturbance tests

After the known-angle tests pass:

1. Capture `IMU BENCH ZERO`.
2. Briefly move a steel object near the rocket without touching it.
3. Confirm `MAG_REJECTED`/reduced confidence appears and orientation does not
   jump materially.
4. Remove the object, let the estimator settle, and check the zero pose.
5. While SAFE, apply short translational movements without intentional
   rotation. Acceleration correction should be rejected during the movement.
6. Check the zero pose again.

These tests prove correction gating, not high-g flight accuracy.

## Stationary drift acceptance

Leave the rocket motionless for 10 minutes with `IMU STREAM START`, then stop
the stream and run `IMU STATUS`.

Acceptance targets:

- quaternion norm remains `0.995–1.005`
- roll/pitch return error is at most `5 degrees`
- yaw drift is at most `15 degrees` in a magnetically clean location
- `MAX_DT_US` is normally below `10000`
- missed accepted samples are below `1%`
- no saturation samples occur while stationary

## Laptop mathematical acceptance

The host test compiles the exact estimator source used by the firmware:

```bash
build_dir=$(mktemp -d /tmp/rocket-imu-test.XXXXXX)
c++ -std=c++17 -Wall -Wextra -Werror \
  rocket/rocket_v10/fw/RocketV10/attitude_estimator.cpp \
  rocket/rocket_v10/tests/attitude_estimator_test.cpp \
  -o "$build_dir/attitude_estimator_test"
"$build_dir/attitude_estimator_test"
```

It proves normalization, a known 90-degree rotation, return-to-start,
boost/magnetic rejection, and stationary gyro-bias convergence. It does not
replace the physical axis, vibration, saturation, or flight tests.

## Flight rollout

For the first flights:

- attitude remains log/visualization-only
- physical pyro outputs remain under their existing independent configuration
- review quaternion confidence, saturation, sample gaps, and calibration
  metadata after every flight
- compare the replay against high-frame-rate ground video

Only consider attitude in flight-critical decisions after multiple flights
show consistent results.

## Live acceptance record — 2026-07-27

Firmware `rv10.20260727b` was uploaded and tested through the external GPS
UART service console with the IMU rigidly installed:

- stored sensor calibration survived upload and cold starts:
  `CAL_FLAGS 7`
- LSM9DS1 magnetometer frame correction reduced stationary magnetic
  innovation from the misleading/rejected behavior to approximately
  `0.15–1.25 degrees`
- `MAG_HEALTHY 1`, confidence `1.000`, quaternion norm `1.000000`, and no
  stationary saturation were confirmed
- version-2 sensor-to-airframe alignment was captured nose-up with the
  reference fin at template zero
- immediate aligned reference quaternion was approximately
  `(1, 0, 0, 0)` and the zero bench check passed with `0.000 degrees` error
- a hand-operated transverse move measured `95.0 degrees`; return-to-reference
  passed with `5.33 degrees` total error while gravity and magnetometer
  corrections remained healthy
- the old gravity-only startup was proven unstable at the installed
  near-180-degree gravity pose, producing about `60 degrees` yaw difference
  after reboot
- gravity-plus-horizontal-magnetic initialization replaced it; after a real
  all-power-off cold start, the saved aligned pose returned within about
  `3.1 degrees`, with gravity innovation `0.23 degrees`, magnetic innovation
  `0.94 degrees`, confidence `1.000`, and no saturation

This accepts the current quaternion path for flight logging and visualization.
It remains excluded from launch, apogee, recovery-state, and dual-deploy
decisions pending real-flight comparison with video and trajectory data.
