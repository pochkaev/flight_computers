# Rocket V10 calibration snapshots

Calibration files in this directory are hardware-specific audit and recovery
references. The installed flight computer's checksummed EEPROM records remain
authoritative at runtime.

Current installed-unit snapshot:

- [`rocket_v10_imu_20260727.json`](rocket_v10_imu_20260727.json)
- firmware: `rv10.20260727b`
- sensor calibration record version: `1`
- airframe alignment record version: `2`
- all calibration groups valid: `CAL_FLAGS 7`
- airframe alignment valid: `ALIGN_VALID 1`

The JSON values are captured at the precision printed by `IMU STATUS`.
Changing the IMU, flight computer, wiring orientation, or physical mounting
requires a new calibration. Moving the IMU relative to the airframe requires a
new airframe alignment.

Do not copy this snapshot to another Rocket V10 unit as a generic default.
