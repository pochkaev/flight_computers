# Rocket Visualizer Scripts

Run these commands from:

```bash
cd /Users/k_pochkaev/github/flight_computers/teensy_v0
```

## `rocket_v10_live_visualizer.py`

Live browser visualizer for a connected RocketV10 or ground-module serial port.

It reads direct rocket `dbg ...` lines, or ground-module `FLIGHT ...`, `NAV ...`, and `STATUS ...` lines, then serves a browser page.

```bash
python3 visualizer/rocket_v10_live_visualizer.py \
  --serial /dev/cu.usbmodem187564601 \
  --http-port 8765
```

Open:

```text
http://127.0.0.1:8765
```

Ground module example:

```bash
python3 visualizer/rocket_v10_live_visualizer.py \
  --serial /dev/cu.usbmodem184901201 \
  --http-port 8765
```

Quick serial test without browser:

```bash
python3 visualizer/rocket_v10_live_visualizer.py \
  --serial /dev/cu.usbmodem187564601 \
  --probe 5
```

## `rocket_v10_trajectory_replay.py`

Recorded-flight browser animation. It reads a RocketV10 `rocket_flight*.csv` file and shows the rocket moving upward from `0 m` using normalized barometric relative altitude.

```bash
python3 visualizer/rocket_v10_trajectory_replay.py \
  /Users/k_pochkaev/github/flight_logs/9May/rocket_v10/rocket_flight0013.csv
```

This creates:

```text
rocket_flight0013_trajectory.html
```

## `rocket_v10_flight_replay.py`

Recorded-flight browser replay with rocket attitude, altitude chart, velocity chart, and optional high-rate `_imu.csv` or `_att.csv` detail exports.

```bash
python3 visualizer/rocket_v10_flight_replay.py \
  /Users/k_pochkaev/github/flight_logs/9May/rocket_v10/rocket_flight0013.csv
```

Optional explicit attitude file:

```bash
python3 visualizer/rocket_v10_flight_replay.py \
  /path/to/rocket_nand_0001.csv \
  --att-csv /path/to/rocket_nand_0001_att.csv
```

## `rocket_v10_flight_report.py`

Static HTML report for one RocketV10 CSV. Useful for charts and summary numbers.

```bash
python3 visualizer/rocket_v10_flight_report.py \
  /Users/k_pochkaev/github/flight_logs/9May/rocket_v10/rocket_flight0013.csv
```

## `analyze_9may_flight.py`

Special report generator for the May 9 log folder. It summarizes rocket, ground, and Centurion logs into one HTML report.

```bash
python3 visualizer/analyze_9may_flight.py \
  /Users/k_pochkaev/github/flight_logs/9May \
  --output /Users/k_pochkaev/github/flight_logs/9May/flight_report.html
```

## `visualizer.py`

Older live Python/matplotlib visualizer. It uses Python packages such as `pyserial` and `matplotlib`, unlike the newer browser-based scripts.

```bash
python3 visualizer/visualizer.py
```
