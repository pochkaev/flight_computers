# Build Notes And Flight Checklist

## Mechanical

- Put the flight computer on vibration-resistant standoffs or foam tape.
- Keep the IMU axes aligned with the rocket body axes.
- Mark the forward direction on the PCB and in the airframe.
- Vent the avionics bay so the BMP390 sees ambient pressure without direct airflow blast.

## Sensor Orientation Convention

This project assumes:

- `+X`: rocket nose direction
- `+Y`: right side when looking from tail to nose
- `+Z`: up from the board plane based on your IMU mounting

If your IMU is mounted differently, change the axis sign mapping in the sketch before flight.

## Firmware States

- `BOOT`: startup and self-test
- `IDLE`: auto-zero on the pad, waiting for automatic launch detection
- `ARMED`: optional legacy/manual state, not required in the current bench flow
- `BOOST`: acceleration above threshold
- `COAST`: motor burnout to apogee search
- `DESCENT`: falling after apogee
- `LANDED`: low dynamics for a sustained period
- `ERROR`: fatal initialization fault

## Ground Procedure

1. Wire everything with the power off.
2. Insert a formatted FAT32 microSD card.
3. Power the system from the regulated 3.3 V rail.
4. Wait for self-test beeps.
5. Confirm GPS fix and sensor data on serial monitor if available.
6. Let the board sit still for a few seconds so the BMP390 zero reference can settle in `IDLE`.
7. Optional: hold the button for about 1 second to force a fresh ground-reference capture.
8. Optional: hold the button for about 4 seconds in any non-error state to reset for the next flight, close the current log, open a new log, and return to `IDLE`.
9. Install into the rocket only after confirming the log file is being created.

## Flight Detection Logic

- Launch: acceleration magnitude above threshold or altitude rising quickly
- Apogee: filtered barometric altitude begins decreasing for several samples
- Landed: low angular rate and low vertical motion for several seconds

## Button Actions

- Hold about `1 second` in `IDLE`: refresh the ground pressure reference
- Hold about `4 seconds` in any non-error state: reset for next flight, rotate the log file, and return to `IDLE`

This is intended for logging and replay. It is not a certified deployment computer.

## Calibration Recommendations

- Keep the rocket stationary for a few seconds at boot.
- Let `IDLE` auto-zero the BMP390 before launch.
- Check ICM-20602 zero-rate bias on the bench and update offsets if needed.
- Validate GPS baud rate from your specific GT-U7 board if serial output looks corrupted.

## SD Card Notes

- Use a small, good-quality card, typically 4 GB to 16 GB.
- Format FAT32.
- Avoid very long filenames.
- Bench test write reliability before any launch.

## Safety

- Do not use this first version as the only recovery deployment computer.
- Bench test all wiring before installing in an airframe.
- Vibration, wiring faults, and brownouts are common failure modes in amateur rocketry.
