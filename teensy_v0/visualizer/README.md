# Rocket Flight Log Analyzer

Run these commands from:

```bash
cd /Users/k_pochkaev/github/flight_computers/teensy_v0
```

## `rocket_v10_log_analyzer.py`

Interactive post-flight analyzer for the CSV set created by:

```text
NAND EXPORT SD <index> FULL
```

Pass the main `rocket_nand_<index>_op<id>.csv` file. The analyzer automatically
finds the matching `_imu`, `_mag`, `_baro`, `_gps`, `_batt`, `_event`,
`_telem`, and `_att` files, then generates one browser report:

```bash
python3 visualizer/rocket_v10_log_analyzer.py \
  /path/to/rocket_nand_0045_op141342.csv \
  --open
```

The report includes:

- satellite and street basemaps with pad, apogee, landing, and GPS flight track
- interactive 3D satellite reconstruction with a moving rocket model
- selectable logged-IMU attitude or trajectory-direction orientation
- an illustrative parachute canopy while the recorded state is `UNDER DROGUE`
- synchronized altitude, velocity, acceleration, gyro, battery, and radio charts
- an unwrapped roll/pitch/yaw change chart relative to the stable pad pose
- direct quaternion interpolation for new type-10 Rocket V10 IMU exports
- estimator-confidence trace plus saturation and sample-quality markers
- fresh magnetometer field-strength diagnostics when `_mag.csv` is present
- clickable state/deployment event timeline
- animated time scrubber that moves the rocket on both the 2D map and 3D scene
- automatic log-only/physical-pyro event indication

The 3D horizontal track uses validated GPS fixes and its vertical axis uses
barometric altitude AGL. By default the model orientation uses the logged
quaternion when present and falls back to the estimator's roll/pitch/yaw for
older logs. It is shown relative to the stable pad pose near `T-1 s`. The
report displays estimator confidence and marks gyro-only, sample-gap, and
sensor-saturation evidence. `Path direction` remains available as a comparison
and fallback.
The canopy is an icon for the recorded `UNDER DROGUE` classification; its shape
and pose were not measured. The rocket model is visually enlarged so it remains
visible at full-flight scale.

Leaflet, Plotly, Three.js, and map tiles are loaded when the report opens, so
the satellite maps and interactive charts require an internet connection. If
3D satellite tiles cannot load, the scene keeps working with a schematic ground
grid. The exported CSV files remain local; the report only requests public
JavaScript and map-tile assets.

## `rocket_v10_imu_demo.py`

Builds an offline, self-contained 3D orientation replay from a serial
`IMU STREAM START` terminal transcript:

```bash
python3 visualizer/rocket_v10_imu_demo.py \
  rocket/rocket_v10/demo_sessions/imu_demo_20260727_01.typescript \
  --open
```

It extracts valid `IMU_DATA` rows, writes a normalized CSV, detects the
movement interval, and generates an interactive HTML report. The replay uses
the recorded airframe quaternion directly. It intentionally keeps position
fixed because bench translation cannot be reconstructed reliably by
double-integrating accelerometer data.
