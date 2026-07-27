#!/usr/bin/env python3
"""Generate an interactive RocketV10 flight analyzer from NAND CSV exports.

The script has no third-party Python dependencies. It auto-discovers the
detail files produced by ``NAND EXPORT SD <index> FULL`` and writes one HTML
report. The report uses Leaflet and Plotly from public CDNs; satellite and
street-map tiles therefore require an internet connection when it is opened.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path
import statistics
import webbrowser
from typing import Any, Iterable


STATE_NAMES = {
    0: "IDLE",
    1: "PAD",
    2: "ASCENT",
    3: "COAST",
    4: "SUBSONIC COAST",
    5: "NEAR APOGEE",
    6: "DESCENT BALLISTIC",
    7: "UNDER DROGUE",
    8: "APOGEE CHARGE LOGGED",
    9: "MAIN CHARGE LOGGED",
    10: "POST-FLIGHT",
    11: "LANDED",
    12: "ABORT",
}

STATE_COLORS = {
    0: "#8290a3",
    1: "#8996a8",
    2: "#ff5c35",
    3: "#ffb020",
    4: "#f5d547",
    5: "#e8cb45",
    6: "#b274ff",
    7: "#48a7ff",
    8: "#ff8f3d",
    9: "#e959ff",
    10: "#4dd6a7",
    11: "#36d17c",
    12: "#ef476f",
}

DETAIL_SUFFIXES = ("_imu", "_mag", "_baro", "_gps", "_batt", "_event", "_telem", "_att")


def finite_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def finite_int(value: str | None) -> int | None:
    value_float = finite_float(value)
    return int(value_float) if value_float is not None else None


def read_csv_with_metadata(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    metadata: dict[str, str] = {}
    csv_lines: list[str] = []
    with path.open(newline="", encoding="utf-8-sig") as source:
        for line in source:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("#"):
                item = stripped[1:].strip()
                if "=" in item:
                    key, value = item.split("=", 1)
                    metadata[key.strip()] = value.strip()
                continue
            csv_lines.append(line)
    if not csv_lines:
        return metadata, []
    return metadata, list(csv.DictReader(csv_lines))


def median(values: Iterable[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.median(finite) if finite else None


def nearest_state(ms: int, state_times: list[int], states: list[int]) -> int | None:
    if not state_times:
        return None
    index = bisect.bisect_left(state_times, ms)
    if index == 0:
        return states[0]
    if index >= len(state_times):
        return states[-1]
    before = index - 1
    return states[before] if ms - state_times[before] <= state_times[index] - ms else states[index]


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    radius_m = 6_371_000.0
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    term = math.sin(dlat / 2.0) ** 2
    term += math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * radius_m * math.asin(math.sqrt(term))


def decimate(samples: list[dict[str, Any]], maximum: int) -> list[dict[str, Any]]:
    if len(samples) <= maximum:
        return samples
    stride = (len(samples) - 1) / float(maximum - 1)
    indexes = {0, len(samples) - 1}
    for i in range(1, maximum - 1):
        indexes.add(round(i * stride))
    return [samples[index] for index in sorted(indexes)]


def interpolate_value(
    samples: list[dict[str, Any]],
    sample_times: list[int],
    ms: int,
    key: str,
) -> float | None:
    """Linearly interpolate one numeric field from time-ordered samples."""
    if not samples:
        return None
    index = bisect.bisect_left(sample_times, ms)
    if index <= 0:
        return samples[0].get(key)
    if index >= len(samples):
        return samples[-1].get(key)
    before = samples[index - 1]
    after = samples[index]
    before_value = before.get(key)
    after_value = after.get(key)
    if before_value is None:
        return after_value
    if after_value is None:
        return before_value
    span = after["ms"] - before["ms"]
    if span <= 0:
        return before_value
    fraction = (ms - before["ms"]) / span
    return before_value + (after_value - before_value) * fraction


def local_east_north_m(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Convert a nearby WGS84 point to a local east/north approximation."""
    radius_m = 6_371_000.0
    east = math.radians(lon - origin_lon) * radius_m * math.cos(math.radians(origin_lat))
    north = math.radians(lat - origin_lat) * radius_m
    return east, north


def event_time(
    events: list[dict[str, Any]],
    *,
    event_name: str | None = None,
    to_state: int | None = None,
) -> int | None:
    for event in events:
        if event_name is not None and event.get("name") != event_name:
            continue
        if to_state is not None and event.get("to") != to_state:
            continue
        return event["ms"]
    return None


def load_exports(main_path: Path) -> tuple[dict[str, str], dict[str, list[dict[str, str]]]]:
    metadata, main_rows = read_csv_with_metadata(main_path)
    exports: dict[str, list[dict[str, str]]] = {"main": main_rows}
    stem = main_path.with_suffix("")
    for suffix in DETAIL_SUFFIXES:
        detail_path = Path(f"{stem}{suffix}.csv")
        if detail_path.exists():
            detail_metadata, exports[suffix[1:]] = read_csv_with_metadata(detail_path)
            if suffix == "_imu":
                metadata.update(detail_metadata)
        else:
            exports[suffix[1:]] = []
    return metadata, exports


def build_payload(main_path: Path) -> dict[str, Any]:
    metadata, exports = load_exports(main_path)

    full: list[dict[str, Any]] = []
    for source in exports["main"]:
        ms = finite_int(source.get("ms"))
        state = finite_int(source.get("state"))
        if ms is None:
            continue
        full.append(
            {
                "ms": ms,
                "state": state,
                "alt": finite_float(source.get("rel_alt_m")),
                "vel": finite_float(source.get("vel_mps")),
                "batt": finite_float(source.get("batt_v")),
                "health": finite_int(source.get("health")),
                "diag": finite_int(source.get("diag_flags")),
                "roll": finite_float(source.get("roll")),
                "pitch": finite_float(source.get("pitch")),
                "yaw": finite_float(source.get("yaw")),
            }
        )
    full.sort(key=lambda row: row["ms"])
    if not full:
        raise ValueError(f"{main_path} has no usable flight rows")

    events: list[dict[str, Any]] = []
    for source in exports["event"]:
        ms = finite_int(source.get("ms"))
        if ms is None:
            continue
        event = {
            "ms": ms,
            "seq": finite_int(source.get("seq")),
            "type": finite_int(source.get("event_type")),
            "name": source.get("event_name") or "EVENT",
            "from": finite_int(source.get("from_state")),
            "to": finite_int(source.get("to_state")),
            "alt": finite_float(source.get("rel_alt_m")),
            "vel": finite_float(source.get("vel_mps")),
            "health": finite_int(source.get("health")),
        }
        events.append(event)
    events.sort(key=lambda event: event["ms"])

    launch_ms = event_time(events, event_name="STATE_CHANGE", to_state=2)
    if launch_ms is None:
        launch_ms = next((row["ms"] for row in full if row["state"] == 2), full[0]["ms"])
    landed_ms = event_time(events, event_name="STATE_CHANGE", to_state=11)
    if landed_ms is None:
        landed_ms = full[-1]["ms"]

    baro: list[dict[str, Any]] = []
    for source in exports["baro"]:
        ms = finite_int(source.get("ms"))
        altitude = finite_float(source.get("rel_alt_m"))
        if ms is None or altitude is None:
            continue
        baro.append(
            {
                "ms": ms,
                "alt": altitude,
                "vel": finite_float(source.get("vel_mps")),
                "state": finite_int(source.get("state")),
                "temp": finite_float(source.get("temp_c")),
                "pressure": finite_float(source.get("pres_pa")),
            }
        )
    if not baro:
        baro = [
            {
                "ms": row["ms"],
                "alt": row["alt"],
                "vel": row["vel"],
                "state": row["state"],
                "temp": None,
                "pressure": None,
            }
            for row in full
            if row["alt"] is not None
        ]
    baro.sort(key=lambda row: row["ms"])

    imu_all: list[dict[str, Any]] = []
    for source in exports["imu"]:
        ms = finite_int(source.get("ms"))
        ax = finite_float(source.get("ax"))
        ay = finite_float(source.get("ay"))
        az = finite_float(source.get("az"))
        if ms is None or ax is None or ay is None or az is None:
            continue
        gx = finite_float(source.get("gx"))
        gy = finite_float(source.get("gy"))
        gz = finite_float(source.get("gz"))
        accel_g = math.sqrt(ax * ax + ay * ay + az * az) / 9.80665
        gyro_dps = None
        if gx is not None and gy is not None and gz is not None:
            gyro_dps = math.sqrt(gx * gx + gy * gy + gz * gz)
        imu_all.append(
            {
                "ms": ms,
                "state": finite_int(source.get("state")),
                "g": accel_g,
                "gyro": gyro_dps,
                "roll": finite_float(source.get("roll")),
                "pitch": finite_float(source.get("pitch")),
                "yaw": finite_float(source.get("yaw")),
                "dtUs": finite_int(source.get("dt_us")),
                "qw": finite_float(source.get("qw")),
                "qx": finite_float(source.get("qx")),
                "qy": finite_float(source.get("qy")),
                "qz": finite_float(source.get("qz")),
                "confidence": finite_float(source.get("confidence")),
                "qualityFlags": finite_int(source.get("quality_flags")),
                "accelCorr": finite_int(source.get("accel_corr")),
                "magCorr": finite_int(source.get("mag_corr")),
                "gyroOnly": finite_int(source.get("gyro_only")),
                "accelSat": finite_int(source.get("accel_sat")),
                "gyroSat": finite_int(source.get("gyro_sat")),
                "sampleGap": finite_int(source.get("sample_gap")),
                "accelRejected": finite_int(source.get("accel_rejected")),
                "magRejected": finite_int(source.get("mag_rejected")),
                "stationary": finite_int(source.get("stationary")),
                "magFresh": finite_int(source.get("mag_fresh")),
            }
        )
    imu_all.sort(key=lambda row: row["ms"])
    imu = decimate(imu_all, 15_000)

    magnetometer: list[dict[str, Any]] = []
    for source in exports["mag"]:
        ms = finite_int(source.get("ms"))
        field = finite_float(source.get("field_ut"))
        if ms is None:
            continue
        if field is None:
            mx = finite_float(source.get("mx"))
            my = finite_float(source.get("my"))
            mz = finite_float(source.get("mz"))
            if mx is not None and my is not None and mz is not None:
                field = math.sqrt(mx * mx + my * my + mz * mz)
        magnetometer.append(
            {
                "ms": ms,
                "field": field,
                "dtUs": finite_int(source.get("dt_us")),
                "fresh": finite_int(source.get("mag_fresh")),
                "rejected": finite_int(source.get("mag_rejected")),
            }
        )
    magnetometer.sort(key=lambda row: row["ms"])

    battery: list[dict[str, Any]] = []
    for source in exports["batt"]:
        ms = finite_int(source.get("ms"))
        voltage = finite_float(source.get("batt_v"))
        if ms is not None and voltage is not None:
            battery.append({"ms": ms, "v": voltage})
    if not battery:
        battery = [{"ms": row["ms"], "v": row["batt"]} for row in full if row["batt"] is not None]
    battery.sort(key=lambda row: row["ms"])

    telemetry: list[dict[str, Any]] = []
    for source in exports["telem"]:
        ms = finite_int(source.get("ms"))
        if ms is None:
            continue
        telemetry.append(
            {
                "ms": ms,
                "state": finite_int(source.get("state")),
                "rssi": finite_float(source.get("last_rssi_dbm")),
                "batt": finite_float(source.get("batt_v")),
                "packet": finite_int(source.get("packet_type")),
            }
        )
    telemetry.sort(key=lambda row: row["ms"])

    state_times = [row["ms"] for row in full]
    states = [row["state"] if row["state"] is not None else 0 for row in full]
    gps: list[dict[str, Any]] = []
    for source in exports["gps"]:
        ms = finite_int(source.get("ms"))
        lat = finite_float(source.get("lat"))
        lon = finite_float(source.get("lon"))
        fix = finite_int(source.get("fix") or source.get("gps_fix"))
        age = finite_int(source.get("fix_age_ms") or source.get("last_fix_age_ms"))
        sats = finite_int(source.get("sats"))
        if (
            ms is None
            or lat is None
            or lon is None
            or abs(lat) > 90
            or abs(lon) > 180
            or lat == 0
            or lon == 0
            or (fix is not None and fix < 2)
            or (age is not None and age > 5_000)
        ):
            continue
        gps.append(
            {
                "ms": ms,
                "lat": lat,
                "lon": lon,
                "alt": finite_float(source.get("gps_rel_alt_m")),
                "speed": finite_float(source.get("gps_speed_mps")),
                "sats": sats,
                "hdop": finite_float(source.get("hdop")),
                "state": nearest_state(ms, state_times, states),
            }
        )
    if not gps:
        for source in exports["main"]:
            ms = finite_int(source.get("ms"))
            lat = finite_float(source.get("lat"))
            lon = finite_float(source.get("lon"))
            fix = finite_int(source.get("gps_fix"))
            if ms is None or lat in (None, 0.0) or lon in (None, 0.0) or (fix is not None and fix < 2):
                continue
            gps.append(
                {
                    "ms": ms,
                    "lat": lat,
                    "lon": lon,
                    "alt": finite_float(source.get("gps_rel_alt_m")),
                    "speed": finite_float(source.get("gps_speed_mps")),
                    "sats": finite_int(source.get("sats")),
                    "hdop": None,
                    "state": nearest_state(ms, state_times, states),
                }
            )
    gps.sort(key=lambda row: row["ms"])

    pad_source = [row for row in gps if row["ms"] <= launch_ms]
    if not pad_source:
        pad_source = gps[: min(3, len(gps))]
    pad = None
    if pad_source:
        pad = {
            "lat": median(row["lat"] for row in pad_source),
            "lon": median(row["lon"] for row in pad_source),
        }
    landing_source = [row for row in gps if row["ms"] >= landed_ms - 5_000]
    if not landing_source:
        landing_source = gps[-min(3, len(gps)) :]
    landing = None
    if landing_source:
        landing = {
            "lat": median(row["lat"] for row in landing_source),
            "lon": median(row["lon"] for row in landing_source),
        }

    trajectory_3d: list[dict[str, Any]] = []
    if pad and gps and baro:
        gps_times = [row["ms"] for row in gps]
        trajectory_source = decimate(baro, 2_400)
        for row in trajectory_source:
            lat = interpolate_value(gps, gps_times, row["ms"], "lat")
            lon = interpolate_value(gps, gps_times, row["ms"], "lon")
            if lat is None or lon is None:
                continue
            east, north = local_east_north_m(lat, lon, pad["lat"], pad["lon"])
            trajectory_3d.append(
                {
                    "ms": row["ms"],
                    "t": (row["ms"] - launch_ms) / 1000.0,
                    "e": east,
                    "n": north,
                    "u": max(0.0, row["alt"]),
                    "state": row["state"],
                }
            )

    peak_altitude_row = max(baro, key=lambda row: row["alt"]) if baro else None
    peak_velocity_candidates = [row for row in baro if row["vel"] is not None]
    peak_velocity_row = (
        max(peak_velocity_candidates, key=lambda row: row["vel"]) if peak_velocity_candidates else None
    )
    peak_acceleration_row = max(imu_all, key=lambda row: row["g"]) if imu_all else None
    apogee_decision_ms = event_time(events, event_name="STATE_CHANGE", to_state=6)
    apogee_log_ms = next(
        (
            event["ms"]
            for event in events
            if "APOGEE_CHARGE" in event["name"] or event["to"] == 8
        ),
        None,
    )

    for collection in (full, baro, imu, magnetometer, battery, telemetry, gps, events):
        for row in collection:
            row["t"] = (row["ms"] - launch_ms) / 1000.0

    pad_to_landing_m = None
    if pad and landing and None not in (pad["lat"], pad["lon"], landing["lat"], landing["lon"]):
        pad_to_landing_m = haversine_m(
            (pad["lat"], pad["lon"]),
            (landing["lat"], landing["lon"]),
        )

    health_values = [row["health"] for row in full if row["health"] is not None]
    battery_values = [row["v"] for row in battery]
    source_files = [main_path.name]
    for suffix in DETAIL_SUFFIXES:
        candidate = Path(f"{main_path.with_suffix('')}{suffix}.csv")
        if candidate.exists():
            source_files.append(candidate.name)

    metrics = {
        "flightIndex": finite_int(metadata.get("flight_index")),
        "rocketName": metadata.get("rocket_name", "Rocket"),
        "firmware": metadata.get("firmware_version", "unknown"),
        "durationS": (landed_ms - launch_ms) / 1000.0,
        "maxAltM": peak_altitude_row["alt"] if peak_altitude_row else None,
        "maxVelMps": peak_velocity_row["vel"] if peak_velocity_row else None,
        "peakG": peak_acceleration_row["g"] if peak_acceleration_row else None,
        "minBattV": min(battery_values) if battery_values else None,
        "maxBattV": max(battery_values) if battery_values else None,
        "gpsDriftM": pad_to_landing_m,
        "gpsPoints": len(gps),
        "recordCount": finite_int(metadata.get("record_count")),
        "healthAll": min(health_values) if health_values else None,
        "apogeeDecisionDelayS": (
            (apogee_decision_ms - peak_altitude_row["ms"]) / 1000.0
            if apogee_decision_ms is not None and peak_altitude_row is not None
            else None
        ),
        "apogeeLogDelayS": (
            (apogee_log_ms - peak_altitude_row["ms"]) / 1000.0
            if apogee_log_ms is not None and peak_altitude_row is not None
            else None
        ),
        "physicalPyroObserved": any("OUTPUT_ON" in event["name"] for event in events),
    }

    return {
        "metadata": metadata,
        "metrics": metrics,
        "states": STATE_NAMES,
        "stateColors": STATE_COLORS,
        "launchMs": launch_ms,
        "landedMs": landed_ms,
        "pad": pad,
        "landing": landing,
        "full": decimate(full, 8_000),
        "baro": decimate(baro, 12_000),
        "imu": imu,
        "mag": decimate(magnetometer, 8_000),
        "battery": decimate(battery, 5_000),
        "telemetry": decimate(telemetry, 5_000),
        "gps": gps,
        "trajectory3d": trajectory_3d,
        "trajectory3dMeta": {
            "position": "GPS horizontal + barometric altitude AGL",
            "orientation": "IMU attitude estimator relative to stable pad attitude",
            "imuAttitudeRows": sum(
                1
                for row in imu_all
                if row["roll"] is not None and row["pitch"] is not None and row["yaw"] is not None
            ),
            "dedicatedQuaternionRows": sum(
                1
                for row in imu_all
                if all(row.get(key) is not None for key in ("qw", "qx", "qy", "qz"))
            ) + len(exports["att"]),
            "calibrationFlags": finite_int(metadata.get("imu_cal_flags")),
            "calibrationVersion": finite_int(metadata.get("imu_cal_version")),
        },
        "events": events,
        "sourceFiles": source_files,
    }


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E%3Ccircle cx='32' cy='32' r='30' fill='%230b1724'/%3E%3Cpath d='M39 8c8-2 14-1 17 0 1 5 1 12-1 18-3 10-12 18-20 23L19 33C24 24 30 12 39 8Z' fill='%2355d8ff'/%3E%3Ccircle cx='41' cy='22' r='6' fill='%23070b12'/%3E%3C/svg%3E">
<link rel="preconnect" href="https://unpkg.com">
<link rel="preconnect" href="https://server.arcgisonline.com">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
 integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
 integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<script type="importmap">
{"imports":{"three":"https://unpkg.com/three@0.180.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.180.0/examples/jsm/"}}
</script>
<style>
:root {
  color-scheme: dark;
  --bg: #070b12;
  --panel: rgba(17, 24, 37, .92);
  --panel-2: #101927;
  --line: rgba(149, 168, 195, .16);
  --ink: #f5f8ff;
  --muted: #8e9caf;
  --cyan: #55d8ff;
  --orange: #ff875c;
  --green: #4dd6a7;
  --purple: #b274ff;
  --danger: #ff5d7a;
  --shadow: 0 22px 70px rgba(0, 0, 0, .35);
}
* { box-sizing: border-box; }
html { scroll-behavior: smooth; }
body {
  margin: 0;
  min-width: 320px;
  background:
    radial-gradient(circle at 15% -10%, rgba(44, 112, 187, .28), transparent 30rem),
    radial-gradient(circle at 90% 0%, rgba(122, 67, 191, .16), transparent 28rem),
    var(--bg);
  color: var(--ink);
  font: 14px/1.5 Inter, ui-sans-serif, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
button, input { font: inherit; }
.shell { width: min(1680px, 100%); margin: 0 auto; padding: 22px; }
.topbar {
  display: flex; align-items: center; justify-content: space-between; gap: 20px;
  margin-bottom: 22px;
}
.brand { display: flex; align-items: center; gap: 14px; }
.mission-mark {
  width: 48px; height: 48px; display: grid; place-items: center; border-radius: 15px;
  background: linear-gradient(145deg, #173149, #0c1724);
  border: 1px solid rgba(85,216,255,.28); box-shadow: 0 0 34px rgba(85,216,255,.12);
}
.mission-mark svg { width: 25px; fill: var(--cyan); transform: rotate(42deg); }
.eyebrow {
  color: var(--cyan); letter-spacing: .16em; text-transform: uppercase;
  font-size: 11px; font-weight: 800;
}
h1 { margin: 2px 0 0; font-size: clamp(22px, 3vw, 34px); letter-spacing: -.035em; line-height: 1.05; }
.top-meta { color: var(--muted); text-align: right; }
.status-pill {
  display: inline-flex; gap: 8px; align-items: center; margin-bottom: 4px;
  padding: 6px 10px; border-radius: 999px; color: #bfffe9;
  border: 1px solid rgba(77,214,167,.26); background: rgba(77,214,167,.08);
  font-weight: 700; font-size: 12px;
}
.status-pill::before { content:""; width: 7px; height: 7px; border-radius: 50%; background: var(--green); box-shadow: 0 0 12px var(--green); }
.metrics { display: grid; grid-template-columns: repeat(6, minmax(120px, 1fr)); gap: 12px; margin-bottom: 14px; }
.metric, .panel {
  background: linear-gradient(160deg, rgba(20,30,45,.96), rgba(12,18,29,.96));
  border: 1px solid var(--line); border-radius: 18px; box-shadow: var(--shadow);
}
.metric { min-height: 112px; padding: 16px; position: relative; overflow: hidden; }
.metric::after {
  content:""; position:absolute; width: 90px; height: 90px; border-radius:50%; right:-40px; top:-42px;
  background: var(--metric-glow, rgba(85,216,255,.08));
}
.metric-label { color: var(--muted); text-transform: uppercase; letter-spacing: .1em; font-size: 10px; font-weight: 800; }
.metric-value { font-size: clamp(24px, 2.5vw, 34px); line-height: 1.15; font-weight: 780; letter-spacing: -.04em; margin-top: 12px; }
.metric-unit { color: var(--muted); font-size: 12px; margin-left: 3px; }
.metric-sub { color: var(--muted); font-size: 11px; margin-top: 4px; }
.hero-grid { display: grid; grid-template-columns: minmax(0, 1.45fr) minmax(340px, .75fr); gap: 14px; }
.panel { overflow: hidden; }
.panel-head { display: flex; align-items: center; justify-content: space-between; padding: 16px 18px; border-bottom: 1px solid var(--line); }
.panel-title { font-size: 13px; font-weight: 800; letter-spacing: .08em; text-transform: uppercase; }
.panel-note { color: var(--muted); font-size: 11px; }
#map { height: 660px; background: #0b111a; }
.map-overlay {
  position: absolute; z-index: 500; left: 14px; bottom: 26px;
  display: flex; gap: 8px; pointer-events: none;
}
.map-chip {
  padding: 7px 10px; border-radius: 9px; color: #e9f7ff;
  background: rgba(7,11,18,.82); border: 1px solid rgba(255,255,255,.14);
  backdrop-filter: blur(10px); font-size: 11px;
}
.map-wrap { position: relative; }
.flight-3d-panel { margin-top: 14px; }
.flight-3d-head { gap: 14px; }
.view-actions { display:flex; align-items:center; gap:8px; flex-wrap:wrap; justify-content:flex-end; }
.view-divider { width:1px; height:24px; background:var(--line); }
.view-button {
  color:var(--ink); background:#152234; border:1px solid var(--line); border-radius:9px;
  padding:7px 10px; cursor:pointer; font-size:11px; font-weight:750;
}
.view-button:hover, .view-button.active { color:#06111a; background:var(--cyan); border-color:var(--cyan); }
.view-button:disabled { opacity:.42; cursor:not-allowed; color:var(--muted); background:#152234; border-color:var(--line); }
.flight-3d-wrap { position:relative; height:min(68vh,720px); min-height:500px; overflow:hidden; background:#08101a; }
#flight-3d { width:100%; height:100%; touch-action:none; }
#flight-3d canvas { width:100%; height:100%; display:block; }
.flight-3d-overlay {
  position:absolute; z-index:4; pointer-events:none; display:flex; gap:8px; flex-wrap:wrap;
}
.flight-3d-overlay.top { left:14px; top:14px; right:14px; }
.flight-3d-overlay.bottom { left:14px; right:14px; bottom:14px; align-items:flex-end; justify-content:space-between; }
.scene-chip {
  padding:7px 10px; border-radius:9px; color:#e9f7ff; font-size:11px;
  background:rgba(7,11,18,.82); border:1px solid rgba(255,255,255,.14);
  backdrop-filter:blur(10px);
}
.scene-chip strong { color:var(--cyan); }
.compass {
  width:58px; height:58px; display:grid; place-items:center; border-radius:50%;
  color:var(--ink); background:rgba(7,11,18,.78); border:1px solid rgba(255,255,255,.14);
  font-size:10px; font-weight:900; position:relative;
}
.compass::before { content:"N"; position:absolute; top:5px; color:var(--cyan); }
.compass::after { content:"▲"; position:absolute; top:16px; color:var(--cyan); font-size:15px; }
.leaflet-container { font-family: inherit; }
.leaflet-control-layers, .leaflet-bar a { background: #121b29; color: white; border-color: rgba(255,255,255,.14); }
.leaflet-control-layers-expanded { color: #dbe7f5; }
.leaflet-control-attribution { background: rgba(7,11,18,.78) !important; color: #b8c7d9; }
.leaflet-control-attribution a { color: var(--cyan); }
.pad-icon, .apogee-icon, .land-icon, .rocket-icon { background: none; border: 0; }
.marker-core { width: 18px; height: 18px; border-radius: 50%; border: 3px solid white; box-shadow: 0 0 0 5px rgba(85,216,255,.25), 0 3px 14px rgba(0,0,0,.65); }
.pad-icon .marker-core { background: var(--cyan); }
.apogee-icon .marker-core { background: var(--orange); box-shadow: 0 0 0 5px rgba(255,135,92,.25), 0 3px 14px rgba(0,0,0,.65); }
.land-icon .marker-core { background: var(--green); box-shadow: 0 0 0 5px rgba(77,214,167,.25), 0 3px 14px rgba(0,0,0,.65); }
.rocket-arrow {
  width: 22px; height: 22px; display:grid; place-items:center; border-radius: 7px;
  background: #f7fbff; color:#09111c; font-size:14px; transform: rotate(45deg);
  border: 2px solid var(--cyan); box-shadow: 0 0 18px rgba(85,216,255,.55);
}
.side-stack { display: flex; flex-direction: column; min-height: 0; }
.phase-readout { padding: 18px; border-bottom: 1px solid var(--line); }
.phase-name { font-size: 27px; font-weight: 800; letter-spacing: -.03em; }
.phase-time { color: var(--cyan); font-variant-numeric: tabular-nums; margin-top: 3px; }
.live-values { display:grid; grid-template-columns: repeat(2, 1fr); gap: 9px; margin-top: 15px; }
.live-value { padding: 11px; background: rgba(255,255,255,.035); border: 1px solid var(--line); border-radius: 12px; }
.live-value span { display:block; color:var(--muted); font-size:10px; text-transform:uppercase; letter-spacing:.08em; }
.live-value strong { display:block; margin-top:3px; font-size:18px; font-variant-numeric: tabular-nums; }
.events { padding: 8px 10px 12px; overflow: auto; max-height: 413px; }
.event {
  width:100%; display:grid; grid-template-columns: 58px 10px 1fr; gap:10px; align-items:start;
  color:inherit; text-align:left; border:0; background:transparent; padding:9px 8px; border-radius:11px; cursor:pointer;
}
.event:hover, .event.active { background: rgba(85,216,255,.08); }
.event-time { color:var(--muted); font-size:11px; font-variant-numeric:tabular-nums; padding-top:2px; }
.event-dot { width:8px; height:8px; border-radius:50%; background:var(--event-color, var(--cyan)); margin-top:5px; box-shadow:0 0 10px var(--event-color, var(--cyan)); }
.event-name { font-size:12px; font-weight:750; }
.event-detail { color:var(--muted); font-size:10px; margin-top:2px; }
.playback {
  position: relative; z-index: 800; margin: 14px 0;
  display:grid; grid-template-columns: auto 120px 1fr auto; gap:12px; align-items:center;
  padding:12px 14px; background:rgba(13,20,31,.93); border:1px solid rgba(85,216,255,.18);
  border-radius:16px; box-shadow:var(--shadow); backdrop-filter:blur(18px);
}
.play {
  width:42px; height:42px; border-radius:12px; border:1px solid rgba(85,216,255,.3);
  color:#06111a; background:var(--cyan); cursor:pointer; font-weight:900;
}
.timecode { font-size:18px; font-weight:800; font-variant-numeric:tabular-nums; }
.timecode small { display:block; color:var(--muted); font-size:9px; letter-spacing:.08em; text-transform:uppercase; }
input[type=range] { width:100%; accent-color:var(--cyan); }
.speed {
  color:var(--ink); background:#152234; border:1px solid var(--line); border-radius:9px;
  padding:7px 9px; cursor:pointer;
}
.charts { display:grid; grid-template-columns: 1fr 1fr; gap:14px; }
.chart { height:340px; }
.events-wide { margin-top:14px; }
.phase-strip { display:flex; overflow:hidden; height:9px; border-radius:999px; background:#101826; margin:18px; }
.phase-segment { min-width:2px; }
.footer {
  display:flex; justify-content:space-between; gap:20px; margin-top:16px; padding:16px 4px;
  color:var(--muted); font-size:11px;
}
.warning { color:#ffd6c8; }
@media (max-width: 1100px) {
  .metrics { grid-template-columns: repeat(3, 1fr); }
  .hero-grid { grid-template-columns:1fr; }
  #map { height:500px; }
  .flight-3d-wrap { height:560px; min-height:460px; }
}
@media (max-width: 720px) {
  .shell { padding:13px; }
  .topbar { align-items:flex-start; }
  .top-meta { display:none; }
  .metrics { grid-template-columns:repeat(2,1fr); }
  .charts { grid-template-columns:1fr; }
  .playback { grid-template-columns:auto 1fr; }
  .playback input[type=range] { grid-column:1 / -1; }
  .speed { display:none; }
  #map { height:430px; }
  .flight-3d-head { align-items:flex-start; }
  .view-actions { justify-content:flex-start; }
  .flight-3d-wrap { height:500px; min-height:420px; }
  .flight-3d-overlay.bottom .scene-chip:last-child { max-width:78%; }
}
</style>
</head>
<body>
<main class="shell">
  <header class="topbar">
    <div class="brand">
      <div class="mission-mark" aria-hidden="true">
        <svg viewBox="0 0 24 24"><path d="M14.6 2.4c2.8-.7 5.5-.5 7-.1.4 1.5.6 4.2-.1 7-1 4-4.4 7.4-7.5 9.3l-3.2-3.2-3.2-3.2C9.5 9 10.6 6.3 14.6 2.4ZM15.8 7A2.2 2.2 0 1 0 19 10.1 2.2 2.2 0 0 0 15.8 7ZM7 13.4l3.6 3.6-2.3 2.3-2.1-.7-.8 2.2-2.2-2.2 2.2-.8-.7-2.1L7 13.4Z"/></svg>
      </div>
      <div><div class="eyebrow">Rocket V10 · Flight intelligence</div><h1 id="mission-title">Flight report</h1></div>
    </div>
    <div class="top-meta">
      <div class="status-pill">Finalized flight data</div>
      <div id="source-line"></div>
    </div>
  </header>

  <section class="metrics" id="metrics"></section>

  <section class="hero-grid">
    <article class="panel">
      <div class="panel-head">
        <div><div class="panel-title">Recovery map</div><div class="panel-note">Satellite imagery · GPS track colored by flight phase</div></div>
        <div class="panel-note" id="map-quality"></div>
      </div>
      <div class="map-wrap">
        <div id="map"></div>
        <div class="map-overlay"><div class="map-chip">PAD</div><div class="map-chip">APOGEE</div><div class="map-chip">LANDING</div></div>
      </div>
    </article>

    <article class="panel side-stack">
      <div class="panel-head"><div class="panel-title">Mission timeline</div><div class="panel-note" id="event-count"></div></div>
      <div class="phase-readout">
        <div class="phase-name" id="phase-name">PAD</div>
        <div class="phase-time" id="phase-time">T−00:00.00</div>
        <div class="live-values">
          <div class="live-value"><span>Altitude AGL</span><strong id="live-alt">—</strong></div>
          <div class="live-value"><span>Vertical speed</span><strong id="live-vel">—</strong></div>
          <div class="live-value"><span>Acceleration</span><strong id="live-g">—</strong></div>
          <div class="live-value"><span>Battery</span><strong id="live-batt">—</strong></div>
        </div>
      </div>
      <div class="events" id="events"></div>
    </article>
  </section>

  <article class="panel flight-3d-panel">
    <div class="panel-head flight-3d-head">
      <div>
        <div class="panel-title">3D flight reconstruction</div>
        <div class="panel-note">Measured GPS + barometric AGL · logged IMU attitude relative to the stable pad pose</div>
      </div>
      <div class="view-actions">
        <button class="view-button active" id="attitude-logged" type="button">Logged attitude</button>
        <button class="view-button" id="attitude-path" type="button">Path direction</button>
        <span class="view-divider" aria-hidden="true"></span>
        <button class="view-button active" id="view-orbit" type="button">Orbit view</button>
        <button class="view-button" id="view-follow" type="button">Follow rocket</button>
        <button class="view-button" id="view-fit" type="button">Fit full track</button>
      </div>
    </div>
    <div class="flight-3d-wrap">
      <div id="flight-3d" aria-label="Interactive 3D rocket flight reconstruction"></div>
      <div class="flight-3d-overlay top">
        <div class="scene-chip" id="scene-position"><strong>Position</strong> —</div>
        <div class="scene-chip" id="scene-state"><strong>Phase</strong> —</div>
        <div class="scene-chip" id="scene-attitude"><strong>Attitude</strong> —</div>
        <div class="scene-chip" id="scene-recovery" hidden><strong>Recovery</strong> canopy pose is illustrative</div>
      </div>
      <div class="flight-3d-overlay bottom">
        <div class="compass" aria-label="North indicator"></div>
        <div class="scene-chip" id="scene-quality">Loading 3D scene…</div>
      </div>
    </div>
  </article>

  <section class="playback">
    <button class="play" id="play" aria-label="Play flight">▶</button>
    <div class="timecode"><small>Mission time</small><span id="timecode">T+00:00.00</span></div>
    <input id="scrubber" type="range" min="0" max="1" value="0" step=".02" aria-label="Flight time">
    <select class="speed" id="speed" aria-label="Playback speed"><option value=".1">0.1×</option><option value=".25">0.25×</option><option value=".5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="5">5×</option><option value="10">10×</option></select>
  </section>

  <section class="charts">
    <article class="panel"><div class="panel-head"><div class="panel-title">Altitude & velocity</div><div class="panel-note">Barometric flight solution</div></div><div class="chart" id="flight-chart"></div></article>
    <article class="panel"><div class="panel-head"><div class="panel-title">Dynamic loads</div><div class="panel-note">IMU acceleration and rotation</div></div><div class="chart" id="imu-chart"></div></article>
    <article class="panel"><div class="panel-head"><div class="panel-title">Rocket attitude</div><div class="panel-note">Onboard quaternion estimator · relative to pad attitude</div></div><div class="chart" id="attitude-chart"></div></article>
    <article class="panel"><div class="panel-head"><div class="panel-title">Power system</div><div class="panel-note">Battery voltage throughout flight</div></div><div class="chart" id="battery-chart"></div></article>
    <article class="panel"><div class="panel-head"><div class="panel-title">Radio diagnostic</div><div class="panel-note">Rocket-side field · Ground log is authoritative for downlink RSSI</div></div><div class="chart" id="radio-chart"></div></article>
  </section>

  <article class="panel events-wide">
    <div class="panel-head"><div><div class="panel-title">Flight phases</div><div class="panel-note">Recorded state coverage from the saved high-rate window</div></div><div class="panel-note" id="pyro-mode"></div></div>
    <div class="phase-strip" id="phase-strip"></div>
  </article>

  <footer class="footer">
    <div id="file-list"></div>
    <div>Satellite layer © Esri and imagery providers · Street layer © OpenStreetMap contributors</div>
  </footer>
</main>

<script>
const DATA = __PAYLOAD__;
window.FLIGHT_DATA = DATA;
const states = Object.fromEntries(Object.entries(DATA.states).map(([k,v]) => [Number(k),v]));
const colors = Object.fromEntries(Object.entries(DATA.stateColors).map(([k,v]) => [Number(k),v]));
const metrics = DATA.metrics;
const finite = value => Number.isFinite(value);
const fmt = (value, digits=1) => finite(value) ? value.toFixed(digits) : "—";
const stateName = state => states[state] || `STATE ${state}`;
const stateColor = state => colors[state] || "#55d8ff";
const duration = metrics.durationS || Math.max(...DATA.baro.map(p => p.t), 0);
const minTime = Math.min(...DATA.baro.map(p => p.t), 0);
const maxTime = Math.max(duration, ...DATA.baro.map(p => p.t));

document.getElementById("mission-title").textContent = `${metrics.rocketName || "Rocket"} · Flight ${metrics.flightIndex ?? "—"}`;
document.getElementById("source-line").textContent = `${metrics.firmware} · ${metrics.recordCount ?? "—"} recorder entries`;
document.getElementById("event-count").textContent = `${DATA.events.length} events`;
document.getElementById("map-quality").textContent = `${metrics.gpsPoints} validated GPS fixes`;
document.getElementById("pyro-mode").innerHTML = metrics.physicalPyroObserved
  ? '<span class="warning">Physical output event present</span>'
  : 'Deployment decisions are <strong>LOG ONLY</strong>';
document.getElementById("file-list").textContent = `Sources: ${DATA.sourceFiles.join(", ")}`;

const metricCards = [
  ["Peak altitude", metrics.maxAltM, "m AGL", "Barometric", "#ff875c"],
  ["Peak velocity", metrics.maxVelMps, "m/s", "Vertical", "#55d8ff"],
  ["Peak load", metrics.peakG, "g", "Vector magnitude", "#b274ff"],
  ["Flight duration", metrics.durationS, "s", "Launch to landed", "#4dd6a7"],
  ["Landing drift", metrics.gpsDriftM, "m", "Pad to final GPS", "#f5d547"],
  ["Battery minimum", metrics.minBattV, "V", `${fmt(metrics.minBattV,3)}–${fmt(metrics.maxBattV,3)} V`, "#48a7ff"],
];
document.getElementById("metrics").innerHTML = metricCards.map(([label,value,unit,sub,color]) => `
  <article class="metric" style="--metric-glow:${color}22">
    <div class="metric-label">${label}</div>
    <div class="metric-value">${fmt(value, unit === "V" ? 3 : 1)}<span class="metric-unit">${unit}</span></div>
    <div class="metric-sub">${sub}</div>
  </article>`).join("");

function nearest(points, t) {
  if (!points.length) return null;
  let lo=0, hi=points.length-1;
  while (lo<hi) { const mid=Math.floor((lo+hi)/2); if (points[mid].t<t) lo=mid+1; else hi=mid; }
  if (lo>0 && Math.abs(points[lo-1].t-t)<Math.abs(points[lo].t-t)) return points[lo-1];
  return points[lo];
}
function timecode(t) {
  const sign=t<0 ? "−" : "+";
  const value=Math.abs(t);
  const minutes=Math.floor(value/60);
  const seconds=value-minutes*60;
  return `T${sign}${String(minutes).padStart(2,"0")}:${seconds.toFixed(2).padStart(5,"0")}`;
}

// Map
let map=null, rocketMarker=null;
const gps = DATA.gps;
if (gps.length && window.L) {
  map=L.map("map",{zoomControl:true,preferCanvas:true});
  const satellite=L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",{
    maxZoom:20, attribution:"Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics, and contributors"
  }).addTo(map);
  const streets=L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png",{
    maxZoom:20, attribution:"© OpenStreetMap contributors"
  });
  L.control.layers({"Satellite":satellite,"Street":streets},null,{position:"topright"}).addTo(map);
  const icon = kind => L.divIcon({className:`${kind}-icon`,html:'<div class="marker-core"></div>',iconSize:[18,18],iconAnchor:[9,9]});
  const rocketIcon=L.divIcon({className:"rocket-icon",html:'<div class="rocket-arrow">➤</div>',iconSize:[24,24],iconAnchor:[12,12]});
  for(let i=1;i<gps.length;i++){
    L.polyline([[gps[i-1].lat,gps[i-1].lon],[gps[i].lat,gps[i].lon]],{
      color:stateColor(gps[i].state),weight:5,opacity:.92,lineCap:"round"
    }).addTo(map);
  }
  if(DATA.pad) L.marker([DATA.pad.lat,DATA.pad.lon],{icon:icon("pad")}).addTo(map).bindPopup("<strong>Launch pad</strong>");
  if(DATA.landing) L.marker([DATA.landing.lat,DATA.landing.lon],{icon:icon("land")}).addTo(map).bindPopup(`<strong>Landing</strong><br>${fmt(metrics.gpsDriftM,1)} m from pad`);
  const peakBaro=DATA.baro.reduce((best,p)=>!best||p.alt>best.alt?p:best,null);
  const peakGps=peakBaro ? nearest(gps,peakBaro.t) : null;
  if(peakGps) L.marker([peakGps.lat,peakGps.lon],{icon:icon("apogee")}).addTo(map).bindPopup(`<strong>Apogee</strong><br>${fmt(metrics.maxAltM,1)} m AGL`);
  rocketMarker=L.marker([gps[0].lat,gps[0].lon],{icon:rocketIcon,zIndexOffset:1000}).addTo(map);
  const bounds=L.latLngBounds(gps.map(p=>[p.lat,p.lon]));
  if(DATA.pad) bounds.extend([DATA.pad.lat,DATA.pad.lon]);
  if(DATA.landing) bounds.extend([DATA.landing.lat,DATA.landing.lon]);
  map.fitBounds(bounds,{paddingTopLeft:[70,70],paddingBottomRight:[70,70],maxZoom:17});
} else {
  document.getElementById("map").innerHTML='<div style="height:100%;display:grid;place-items:center;color:#8e9caf">No valid GPS export found</div>';
}

// Plotly charts
const plotConfig={responsive:true,displaylogo:false,displayModeBar:false};
const chartLayout=(yTitle,extra={})=>({
  margin:{l:58,r:48,t:18,b:46},paper_bgcolor:"transparent",plot_bgcolor:"transparent",
  font:{color:"#aebbd0",family:"Inter, ui-sans-serif, sans-serif",size:11},
  xaxis:{title:"Mission time (s)",gridcolor:"rgba(149,168,195,.10)",zerolinecolor:"rgba(85,216,255,.25)",rangeslider:{visible:false}},
  yaxis:{title:yTitle,gridcolor:"rgba(149,168,195,.10)",zerolinecolor:"rgba(149,168,195,.14)"},
  hovermode:"x unified",showlegend:true,legend:{orientation:"h",x:0,y:1.12},
  ...extra
});
const eventShapes=DATA.events.filter(e=>e.name==="STATE_CHANGE").map(e=>({
  type:"line",x0:e.t,x1:e.t,y0:0,y1:1,yref:"paper",line:{color:stateColor(e.to),width:1,dash:"dot"}
}));
Plotly.newPlot("flight-chart",[
  {x:DATA.baro.map(p=>p.t),y:DATA.baro.map(p=>p.alt),name:"Altitude",mode:"lines",line:{color:"#ff875c",width:2.4}},
  {x:DATA.baro.map(p=>p.t),y:DATA.baro.map(p=>p.vel),name:"Velocity",mode:"lines",yaxis:"y2",line:{color:"#55d8ff",width:1.8}}
],chartLayout("Altitude (m)",{yaxis2:{title:"Velocity (m/s)",overlaying:"y",side:"right",gridcolor:"transparent"},shapes:eventShapes}),plotConfig);
Plotly.newPlot("imu-chart",[
  {x:DATA.imu.map(p=>p.t),y:DATA.imu.map(p=>p.g),name:"Acceleration",mode:"lines",line:{color:"#b274ff",width:1.4}},
  {x:DATA.imu.map(p=>p.t),y:DATA.imu.map(p=>p.gyro),name:"Rotation",mode:"lines",yaxis:"y2",line:{color:"#f5d547",width:1,opacity:.65}},
  {x:(DATA.mag||[]).map(p=>p.t),y:(DATA.mag||[]).map(p=>p.field),name:"Mag field",mode:"lines",yaxis:"y3",line:{color:"#4dd6a7",width:1}},
  {x:DATA.imu.filter(p=>p.accelSat||p.gyroSat).map(p=>p.t),y:DATA.imu.filter(p=>p.accelSat||p.gyroSat).map(p=>p.g),
   name:"Saturation",mode:"markers",marker:{color:"#ef476f",size:7,symbol:"x"}}
],chartLayout("Acceleration (g)",{
  margin:{l:58,r:94,t:18,b:46},
  yaxis2:{title:"Gyro (°/s)",overlaying:"y",side:"right",gridcolor:"transparent"},
  yaxis3:{title:"Mag (µT)",overlaying:"y",side:"right",position:.94,gridcolor:"transparent",anchor:"free"}
}),plotConfig);
const attitudeRows=DATA.imu.filter(p=>finite(p.roll)&&finite(p.pitch)&&finite(p.yaw));
function relativeUnwrapped(rows,key){
  if(!rows.length)return [];
  const values=[]; let offset=0,previous=rows[0][key];
  for(const row of rows){
    const value=row[key],delta=value-previous;
    if(delta>180)offset-=360;
    else if(delta<-180)offset+=360;
    values.push(value+offset);
    previous=value;
  }
  let referenceIndex=0;
  for(let i=1;i<rows.length;i++)if(Math.abs(rows[i].t+1)<Math.abs(rows[referenceIndex].t+1))referenceIndex=i;
  const reference=values[referenceIndex];
  return values.map(value=>value-reference);
}
Plotly.newPlot("attitude-chart",[
  {x:attitudeRows.map(p=>p.t),y:relativeUnwrapped(attitudeRows,"roll"),name:"Roll Δ",mode:"lines",line:{color:"#ff875c",width:1.4}},
  {x:attitudeRows.map(p=>p.t),y:relativeUnwrapped(attitudeRows,"pitch"),name:"Pitch Δ",mode:"lines",line:{color:"#55d8ff",width:1.4}},
  {x:attitudeRows.map(p=>p.t),y:relativeUnwrapped(attitudeRows,"yaw"),name:"Yaw Δ",mode:"lines",line:{color:"#b274ff",width:1.2}},
  {x:attitudeRows.filter(p=>finite(p.confidence)).map(p=>p.t),
   y:attitudeRows.filter(p=>finite(p.confidence)).map(p=>p.confidence*100),
   name:"Confidence",mode:"lines",yaxis:"y2",line:{color:"#4dd6a7",width:1,dash:"dot"}}
],chartLayout("Angle change from pad (°)",{
  shapes:eventShapes,
  yaxis2:{title:"Confidence (%)",overlaying:"y",side:"right",range:[0,100],gridcolor:"transparent"}
}),plotConfig);
Plotly.newPlot("battery-chart",[
  {x:DATA.battery.map(p=>p.t),y:DATA.battery.map(p=>p.v),name:"Battery",mode:"lines",fill:"tozeroy",fillcolor:"rgba(77,214,167,.08)",line:{color:"#4dd6a7",width:2}}
],chartLayout("Battery (V)"),plotConfig);
const radioRows=DATA.telemetry.filter(p=>finite(p.rssi));
Plotly.newPlot("radio-chart",[
  {x:radioRows.map(p=>p.t),y:radioRows.map(p=>p.rssi),name:"Onboard RSSI field",mode:"markers",marker:{color:"#48a7ff",size:4,opacity:.75}}
],chartLayout("RSSI (dBm)"),plotConfig);

const phaseStrip=document.getElementById("phase-strip");
const ordered=DATA.full;
if(ordered.length){
  let start=0;
  while(start<ordered.length){
    let end=start+1;
    while(end<ordered.length && ordered[end].state===ordered[start].state) end++;
    const width=(end-start)/ordered.length*100;
    phaseStrip.insertAdjacentHTML("beforeend",`<div class="phase-segment" title="${stateName(ordered[start].state)}" style="width:${width}%;background:${stateColor(ordered[start].state)}"></div>`);
    start=end;
  }
}

const eventContainer=document.getElementById("events");
eventContainer.innerHTML=DATA.events.map((e,index)=>`
  <button class="event" data-event="${index}" style="--event-color:${stateColor(e.to)}">
    <span class="event-time">${timecode(e.t)}</span><span class="event-dot"></span>
    <span><span class="event-name">${e.name.replaceAll("_"," ")}</span>
    <span class="event-detail">${stateName(e.from)} → ${stateName(e.to)} · ${fmt(e.alt,1)} m</span></span>
  </button>`).join("");
eventContainer.addEventListener("click",event=>{
  const button=event.target.closest("[data-event]"); if(!button)return;
  setTime(DATA.events[Number(button.dataset.event)].t,true);
});

const scrubber=document.getElementById("scrubber");
scrubber.min=minTime; scrubber.max=maxTime; scrubber.value=minTime;
const playButton=document.getElementById("play");
let currentTime=minTime, playing=false, lastFrame=0;
function setTime(t,pan=false){
  currentTime=Math.max(minTime,Math.min(maxTime,t));
  scrubber.value=currentTime;
  const baro=nearest(DATA.baro,currentTime);
  const imu=nearest(DATA.imu,currentTime);
  const batt=nearest(DATA.battery,currentTime);
  const full=nearest(DATA.full,currentTime);
  const point=nearest(gps,currentTime);
  const state=full?.state ?? baro?.state ?? point?.state ?? 0;
  document.getElementById("phase-name").textContent=stateName(state);
  document.getElementById("phase-name").style.color=stateColor(state);
  document.getElementById("phase-time").textContent=timecode(currentTime);
  document.getElementById("timecode").textContent=timecode(currentTime);
  document.getElementById("live-alt").textContent=`${fmt(baro?.alt,1)} m`;
  document.getElementById("live-vel").textContent=`${fmt(baro?.vel,1)} m/s`;
  document.getElementById("live-g").textContent=`${fmt(imu?.g,2)} g`;
  document.getElementById("live-batt").textContent=`${fmt(batt?.v,3)} V`;
  if(rocketMarker && point){
    rocketMarker.setLatLng([point.lat,point.lon]);
    if(pan) map.panTo([point.lat,point.lon]);
  }
  if(window.flight3D) window.flight3D.setTime(currentTime,state);
  const eventIndex=DATA.events.reduce((best,e,i)=>e.t<=currentTime?i:best,-1);
  document.querySelectorAll(".event").forEach((el,i)=>el.classList.toggle("active",i===eventIndex));
}
function frame(timestamp){
  if(!playing)return;
  if(!lastFrame)lastFrame=timestamp;
  const dt=(timestamp-lastFrame)/1000*Number(document.getElementById("speed").value);
  lastFrame=timestamp;
  if(currentTime+dt>=maxTime){ setTime(maxTime); playing=false; playButton.textContent="▶"; return; }
  setTime(currentTime+dt);
  requestAnimationFrame(frame);
}
playButton.addEventListener("click",()=>{
  playing=!playing; playButton.textContent=playing?"❚❚":"▶"; lastFrame=0;
  if(playing){ if(currentTime>=maxTime)setTime(minTime); requestAnimationFrame(frame); }
});
scrubber.addEventListener("input",()=>{ playing=false; playButton.textContent="▶"; setTime(Number(scrubber.value)); });
for(const id of ["flight-chart","imu-chart","attitude-chart","battery-chart","radio-chart"]){
  document.getElementById(id).on("plotly_hover",event=>{ if(event.points?.length)setTime(event.points[0].x); });
}
setTime(minTime);
</script>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const D = window.FLIGHT_DATA;
const host = document.getElementById("flight-3d");
const quality = document.getElementById("scene-quality");
const positionLabel = document.getElementById("scene-position");
const stateLabel = document.getElementById("scene-state");
const attitudeLabel = document.getElementById("scene-attitude");
const recoveryLabel = document.getElementById("scene-recovery");
const trajectory = D.trajectory3d || [];
const attitude = (D.imu || []).filter(p=>Number.isFinite(p.roll)&&Number.isFinite(p.pitch)&&Number.isFinite(p.yaw));

if (!trajectory.length) {
  quality.textContent = "3D unavailable: no valid GPS + barometer trajectory";
  host.innerHTML = '<div style="height:100%;display:grid;place-items:center;color:#8e9caf">No reconstructable 3D trajectory</div>';
} else {
  try {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x07101a);
    scene.fog = new THREE.FogExp2(0x07101a, 0.0015);

    const camera = new THREE.PerspectiveCamera(46, 1, 0.1, 5000);
    const renderer = new THREE.WebGLRenderer({antialias:true,alpha:false,powerPreference:"high-performance"});
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    host.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = .07;
    controls.maxPolarAngle = Math.PI * .495;
    controls.minDistance = 8;
    controls.maxDistance = 1800;

    scene.add(new THREE.HemisphereLight(0xc9e8ff, 0x18231a, 2.25));
    const sun = new THREE.DirectionalLight(0xffffff, 2.4);
    sun.position.set(-120,220,90);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024,1024);
    scene.add(sun);

    const points = trajectory.map(p => new THREE.Vector3(p.e, p.u, -p.n));
    const xs = points.map(p=>p.x), ys = points.map(p=>p.y), zs = points.map(p=>p.z);
    const bounds = {
      minX:Math.min(...xs), maxX:Math.max(...xs),
      minY:Math.min(...ys), maxY:Math.max(...ys),
      minZ:Math.min(...zs), maxZ:Math.max(...zs)
    };
    const horizontalSpan = Math.max(bounds.maxX-bounds.minX,bounds.maxZ-bounds.minZ,80);
    const sceneSpan = Math.max(horizontalSpan,bounds.maxY-bounds.minY,80);

    const groundSize = Math.max(240,horizontalSpan*1.8);
    const groundFallback = new THREE.Mesh(
      new THREE.PlaneGeometry(groundSize,groundSize),
      new THREE.MeshStandardMaterial({color:0x19271f,roughness:1,metalness:0})
    );
    groundFallback.rotation.x = -Math.PI/2;
    groundFallback.position.y = -.4;
    groundFallback.receiveShadow = true;
    scene.add(groundFallback);

    const grid = new THREE.GridHelper(groundSize,24,0x55d8ff,0x728095);
    grid.position.y = -.18;
    grid.material.opacity = .28;
    grid.material.transparent = true;
    scene.add(grid);

    const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);
    const vertexColors = [];
    for (const p of trajectory) {
      const color = new THREE.Color(D.stateColors[String(p.state)] || "#55d8ff");
      vertexColors.push(color.r,color.g,color.b);
    }
    lineGeometry.setAttribute("color",new THREE.Float32BufferAttribute(vertexColors,3));
    const flightLine = new THREE.Line(
      lineGeometry,
      new THREE.LineBasicMaterial({vertexColors:true,transparent:true,opacity:.96})
    );
    scene.add(flightLine);

    const shadowLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(p.x,.08,p.z))),
      new THREE.LineBasicMaterial({color:0x55d8ff,transparent:true,opacity:.36})
    );
    scene.add(shadowLine);

    const padGroup = new THREE.Group();
    const padDisc = new THREE.Mesh(
      new THREE.CylinderGeometry(3.1,3.1,.3,48),
      new THREE.MeshStandardMaterial({color:0x173647,metalness:.5,roughness:.4,emissive:0x09202b})
    );
    padDisc.position.y = -.05;
    padDisc.receiveShadow = true;
    padGroup.add(padDisc);
    const padRing = new THREE.Mesh(
      new THREE.TorusGeometry(4.1,.18,10,64),
      new THREE.MeshBasicMaterial({color:0x55d8ff})
    );
    padRing.rotation.x = Math.PI/2;
    padRing.position.y = .12;
    padGroup.add(padRing);
    scene.add(padGroup);

    function buildRocket() {
      const group = new THREE.Group();
      const white = new THREE.MeshStandardMaterial({color:0xf4f7fb,metalness:.22,roughness:.3});
      const orange = new THREE.MeshStandardMaterial({color:0xff633e,metalness:.12,roughness:.38});
      const dark = new THREE.MeshStandardMaterial({color:0x182331,metalness:.7,roughness:.24});
      const body = new THREE.Mesh(new THREE.CylinderGeometry(.43,.43,2.8,28),white);
      body.position.y = 1.55; body.castShadow = true; group.add(body);
      const stripe = new THREE.Mesh(new THREE.CylinderGeometry(.445,.445,.48,28),orange);
      stripe.position.y = 1.25; stripe.castShadow = true; group.add(stripe);
      const nose = new THREE.Mesh(new THREE.ConeGeometry(.46,1.18,28),orange);
      nose.position.y = 3.54; nose.castShadow = true; group.add(nose);
      const bell = new THREE.Mesh(new THREE.CylinderGeometry(.28,.39,.42,20),dark);
      bell.position.y = -.06; bell.castShadow = true; group.add(bell);
      const finGeometry = new THREE.BufferGeometry();
      finGeometry.setAttribute("position",new THREE.Float32BufferAttribute([
        0,0,0, 1.05,.18,0, 0,1.28,0,
        0,0,.05, 0,1.28,.05, 1.05,.18,.05
      ],3));
      finGeometry.computeVertexNormals();
      for(let i=0;i<4;i++){
        const fin = new THREE.Mesh(finGeometry,orange);
        fin.rotation.y=i*Math.PI/2; fin.castShadow=true; group.add(fin);
      }
      const flame = new THREE.Mesh(
        new THREE.ConeGeometry(.34,1.8,20),
        new THREE.MeshBasicMaterial({color:0xffb020,transparent:true,opacity:.88})
      );
      flame.position.y=-1.15; flame.rotation.x=Math.PI; flame.name="flame"; group.add(flame);
      return group;
    }
    const rocket = buildRocket();
    rocket.scale.setScalar(1.45);
    scene.add(rocket);

    function buildParachute() {
      const group = new THREE.Group();
      const canopy = new THREE.Mesh(
        new THREE.SphereGeometry(3.8,32,14,0,Math.PI*2,0,Math.PI/2),
        new THREE.MeshStandardMaterial({
          color:0xffd7c7,emissive:0x321008,emissiveIntensity:.25,
          side:THREE.DoubleSide,transparent:true,opacity:.92,roughness:.72
        })
      );
      canopy.position.y=10;
      canopy.castShadow=true;
      group.add(canopy);
      const linePoints=[];
      for(let i=0;i<8;i++){
        const angle=i*Math.PI/4;
        linePoints.push(
          new THREE.Vector3(Math.cos(angle)*3.7,10,Math.sin(angle)*3.7),
          new THREE.Vector3(0,1.2,0)
        );
      }
      const lines=new THREE.LineSegments(
        new THREE.BufferGeometry().setFromPoints(linePoints),
        new THREE.LineBasicMaterial({color:0xf3f6ff,transparent:true,opacity:.78})
      );
      group.add(lines);
      group.visible=false;
      return group;
    }
    const parachute=buildParachute();
    scene.add(parachute);

    const landingMarker = new THREE.Mesh(
      new THREE.TorusGeometry(2.4,.25,12,48),
      new THREE.MeshBasicMaterial({color:0x4dd6a7})
    );
    landingMarker.rotation.x=Math.PI/2;
    landingMarker.position.copy(points[points.length-1]);
    landingMarker.position.y=.15;
    scene.add(landingMarker);

    function interpolateTrack(t) {
      let lo=0,hi=trajectory.length-1;
      while(lo<hi){const mid=Math.floor((lo+hi)/2);if(trajectory[mid].t<t)lo=mid+1;else hi=mid;}
      if(lo===0)return {...trajectory[0],index:0};
      if(lo>=trajectory.length)return {...trajectory[trajectory.length-1],index:trajectory.length-1};
      const a=trajectory[lo-1],b=trajectory[lo];
      const f=b.t===a.t?0:Math.max(0,Math.min(1,(t-a.t)/(b.t-a.t)));
      return {
        t,e:a.e+(b.e-a.e)*f,n:a.n+(b.n-a.n)*f,u:a.u+(b.u-a.u)*f,
        state:f<.5?a.state:b.state,index:lo
      };
    }
    function nearestSample(rows,t) {
      if(!rows.length)return null;
      let lo=0,hi=rows.length-1;
      while(lo<hi){const mid=Math.floor((lo+hi)/2);if(rows[mid].t<t)lo=mid+1;else hi=mid;}
      if(lo>0&&Math.abs(rows[lo-1].t-t)<Math.abs(rows[lo].t-t))return rows[lo-1];
      return rows[lo];
    }
    function quaternionFromEulerDeg(sample) {
      const roll=sample.roll*Math.PI/180,pitch=sample.pitch*Math.PI/180,yaw=sample.yaw*Math.PI/180;
      const cr=Math.cos(roll/2),sr=Math.sin(roll/2);
      const cp=Math.cos(pitch/2),sp=Math.sin(pitch/2);
      const cy=Math.cos(yaw/2),sy=Math.sin(yaw/2);
      return new THREE.Quaternion(
        sr*cp*cy-cr*sp*sy,
        cr*sp*cy+sr*cp*sy,
        cr*cp*sy-sr*sp*cy,
        cr*cp*cy+sr*sp*sy
      ).normalize();
    }
    function quaternionFromSample(sample) {
      if(Number.isFinite(sample?.qw)&&Number.isFinite(sample?.qx)&&
         Number.isFinite(sample?.qy)&&Number.isFinite(sample?.qz)){
        return new THREE.Quaternion(sample.qx,sample.qy,sample.qz,sample.qw).normalize();
      }
      return quaternionFromEulerDeg(sample);
    }
    const padAttitude=nearestSample(attitude,-1);
    const padAttitudeInverse=padAttitude
      ? quaternionFromSample(padAttitude).invert()
      : new THREE.Quaternion();
    const estimatorToScene=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);
    const sceneToEstimator=estimatorToScene.clone().invert();
    function interpolatedAttitude(t) {
      if(!attitude.length)return null;
      let lo=0,hi=attitude.length-1;
      while(lo<hi){const mid=Math.floor((lo+hi)/2);if(attitude[mid].t<t)lo=mid+1;else hi=mid;}
      const b=attitude[lo],a=lo>0?attitude[lo-1]:b;
      const fraction=b.t===a.t?0:Math.max(0,Math.min(1,(t-a.t)/(b.t-a.t)));
      const q=new THREE.Quaternion().slerpQuaternions(
        quaternionFromSample(a),quaternionFromSample(b),fraction
      );
      const relative=q.multiply(padAttitudeInverse);
      const display=estimatorToScene.clone().multiply(relative).multiply(sceneToEstimator);
      const raw=fraction<.5?a:b;
      return {quaternion:display,raw};
    }
    function confidenceAt(t) {
      const imu=nearestSample(attitude,t);
      if(Number.isFinite(imu?.confidence)){
        const percent=(imu.confidence*100).toFixed(0);
        const flags=imu.qualityFlags||0;
        if(flags&24)return `${percent}% · SENSOR SATURATION`;
        if(flags&32)return `${percent}% · sample gap`;
        if(flags&4)return `${percent}% · gyro-only`;
        if((flags&1)&&(flags&2))return `${percent}% · accel + mag corrected`;
        if(flags&1)return `${percent}% · accelerometer corrected`;
        return `${percent}% · gyro propagation`;
      }
      const full=nearestSample(D.full||[],t);
      const diag=full?.diag;
      if(!Number.isFinite(diag))return "confidence flag unavailable";
      if(diag&8)return "gyro-only · lower confidence";
      if((diag&2)&&(diag&4))return "accelerometer + magnetometer corrected";
      if(diag&2)return "accelerometer corrected";
      if(diag&4)return "magnetometer corrected";
      return "gyro integration";
    }
    const upAxis = new THREE.Vector3(0,1,0);
    let orientationMode=attitude.length?"logged":"path";
    let follow = false;
    let lastRocketPosition = new THREE.Vector3();
    let lastSceneTime=trajectory[0].t,lastSceneState=trajectory[0].state??0;
    function updateRocket(t,state) {
      lastSceneTime=t;
      lastSceneState=state;
      const p=interpolateTrack(t);
      const position=new THREE.Vector3(p.e,p.u,-p.n);
      rocket.position.copy(position);
      lastRocketPosition.copy(position);
      const before=interpolateTrack(t-.45),after=interpolateTrack(t+.45);
      const direction=new THREE.Vector3(after.e-before.e,after.u-before.u,-(after.n-before.n));
      if(direction.lengthSq()<.04) direction.set(0,1,0);
      direction.normalize();
      const pathQuaternion=new THREE.Quaternion().setFromUnitVectors(upAxis,direction);
      const logged=interpolatedAttitude(t);
      rocket.quaternion.copy(orientationMode==="logged"&&logged?logged.quaternion:pathQuaternion);
      const flame=rocket.getObjectByName("flame");
      if(flame) flame.visible=state===2 && (p.u>1 || t>0);
      parachute.position.copy(position);
      parachute.visible=state===7;
      positionLabel.innerHTML=`<strong>Position</strong> E ${p.e.toFixed(1)} m · N ${p.n.toFixed(1)} m · ${p.u.toFixed(1)} m AGL`;
      const stateText=D.states[String(state)] || `STATE ${state}`;
      stateLabel.innerHTML=`<strong>Phase</strong> ${stateText}`;
      if(orientationMode==="logged"&&logged){
        attitudeLabel.innerHTML=`<strong>Attitude</strong> logged · R ${logged.raw.roll.toFixed(1)}° · P ${logged.raw.pitch.toFixed(1)}° · Y ${logged.raw.yaw.toFixed(1)}° · ${confidenceAt(t)}`;
      }else{
        attitudeLabel.innerHTML="<strong>Attitude</strong> path direction · not body orientation";
      }
      recoveryLabel.hidden=state!==7&&state!==8;
      if(state===8)recoveryLabel.innerHTML="<strong>Recovery</strong> apogee decision logged only · no physical output";
      else if(state===7)recoveryLabel.innerHTML="<strong>Recovery</strong> UNDER DROGUE detected · canopy pose is illustrative";
    }

    function fitTrack() {
      const center=new THREE.Vector3(
        (bounds.minX+bounds.maxX)/2,
        (bounds.minY+bounds.maxY)/2,
        (bounds.minZ+bounds.maxZ)/2
      );
      controls.target.copy(center);
      camera.position.set(center.x+sceneSpan*.9,center.y+sceneSpan*.66,center.z+sceneSpan*.9);
      camera.near=Math.max(.1,sceneSpan/2000);
      camera.far=Math.max(2500,sceneSpan*12);
      camera.updateProjectionMatrix();
      controls.update();
    }
    fitTrack();

    async function satelliteGround() {
      if(!D.pad) throw new Error("Pad coordinates unavailable");
      const lat=D.pad.lat,lon=D.pad.lon;
      const desired=groundSize*1.35;
      let zoom=12;
      for(let z=12;z<=19;z++){
        const tileWidth=156543.03392*Math.cos(lat*Math.PI/180)/Math.pow(2,z)*256;
        if(tileWidth*3>=desired) zoom=z; else break;
      }
      const count=Math.pow(2,zoom);
      const tileX=(lon+180)/360*count;
      const tileY=(1-Math.asinh(Math.tan(lat*Math.PI/180))/Math.PI)/2*count;
      const startX=Math.floor(tileX)-1,startY=Math.floor(tileY)-1;
      const canvas=document.createElement("canvas"); canvas.width=canvas.height=768;
      const ctx=canvas.getContext("2d");
      const loads=[];
      for(let row=0;row<3;row++)for(let col=0;col<3;col++){
        loads.push(new Promise((resolve,reject)=>{
          const image=new Image(); image.crossOrigin="anonymous";
          image.onload=()=>{ctx.drawImage(image,col*256,row*256,256,256);resolve();};
          image.onerror=reject;
          image.src=`https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${zoom}/${startY+row}/${startX+col}`;
        }));
      }
      await Promise.all(loads);
      const tileLon=x=>x/count*360-180;
      const tileLat=y=>Math.atan(Math.sinh(Math.PI*(1-2*y/count)))*180/Math.PI;
      const west=tileLon(startX),east=tileLon(startX+3);
      const north=tileLat(startY),south=tileLat(startY+3);
      const earth=6371000,cosLat=Math.cos(lat*Math.PI/180);
      const westM=(west-lon)*Math.PI/180*earth*cosLat;
      const eastM=(east-lon)*Math.PI/180*earth*cosLat;
      const northM=(north-lat)*Math.PI/180*earth;
      const southM=(south-lat)*Math.PI/180*earth;
      const texture=new THREE.CanvasTexture(canvas);
      texture.colorSpace=THREE.SRGBColorSpace;
      texture.anisotropy=Math.min(8,renderer.capabilities.getMaxAnisotropy());
      const plane=new THREE.Mesh(
        new THREE.PlaneGeometry(eastM-westM,northM-southM),
        new THREE.MeshBasicMaterial({map:texture,color:0xffffff})
      );
      plane.rotation.x=-Math.PI/2;
      plane.position.set((westM+eastM)/2,-.32,-(northM+southM)/2);
      scene.add(plane);
      groundFallback.visible=false;
      quality.textContent=`Satellite ground · 3 × 3 Esri tiles at zoom ${zoom} · model enlarged for visibility`;
    }
    satelliteGround().catch(()=>{
      quality.textContent="Map tiles unavailable · schematic ground shown · model enlarged for visibility";
    });

    const loggedButton=document.getElementById("attitude-logged");
    const pathButton=document.getElementById("attitude-path");
    if(!attitude.length){
      loggedButton.disabled=true;
      loggedButton.classList.remove("active");
      pathButton.classList.add("active");
      orientationMode="path";
    }
    loggedButton.addEventListener("click",()=>{
      if(!attitude.length)return;
      orientationMode="logged";
      loggedButton.classList.add("active");
      pathButton.classList.remove("active");
      updateRocket(lastSceneTime,lastSceneState);
    });
    pathButton.addEventListener("click",()=>{
      orientationMode="path";
      pathButton.classList.add("active");
      loggedButton.classList.remove("active");
      updateRocket(lastSceneTime,lastSceneState);
    });
    document.getElementById("view-orbit").addEventListener("click",()=>{
      follow=false;
      document.getElementById("view-orbit").classList.add("active");
      document.getElementById("view-follow").classList.remove("active");
    });
    document.getElementById("view-follow").addEventListener("click",()=>{
      follow=true;
      controls.target.copy(lastRocketPosition);
      document.getElementById("view-follow").classList.add("active");
      document.getElementById("view-orbit").classList.remove("active");
    });
    document.getElementById("view-fit").addEventListener("click",()=>{follow=false;fitTrack();});

    function render() {
      const width=Math.max(1,host.clientWidth),height=Math.max(1,host.clientHeight);
      if(renderer.domElement.width!==Math.floor(width*Math.min(window.devicePixelRatio||1,2)) ||
         renderer.domElement.height!==Math.floor(height*Math.min(window.devicePixelRatio||1,2))){
        renderer.setSize(width,height,false);
        camera.aspect=width/height;
        camera.updateProjectionMatrix();
      }
      if(follow){
        const focus=parachute.visible
          ? lastRocketPosition.clone().add(new THREE.Vector3(0,6,0))
          : lastRocketPosition.clone().add(
              new THREE.Vector3(0,2.4*1.45,0).applyQuaternion(rocket.quaternion)
            );
        const desired=focus.clone().add(
          parachute.visible ? new THREE.Vector3(24,14,24) : new THREE.Vector3(15,9,15)
        );
        camera.position.lerp(desired,.12);
        controls.target.lerp(focus,.18);
      }
      controls.update();
      renderer.render(scene,camera);
      requestAnimationFrame(render);
    }
    window.flight3D={
      setTime:updateRocket,
      fit:fitTrack
    };
    const initialState=trajectory[0].state ?? 0;
    updateRocket(trajectory[0].t,initialState);
    render();
  } catch(error) {
    quality.textContent=`3D unavailable: ${error.message}`;
    host.innerHTML='<div style="height:100%;display:grid;place-items:center;color:#8e9caf">This browser could not start WebGL</div>';
  }
}
</script>
</body>
</html>
"""


def render_html(payload: dict[str, Any]) -> str:
    metrics = payload["metrics"]
    title = f"{metrics.get('rocketName', 'Rocket')} flight {metrics.get('flightIndex', '')} analyzer"
    payload_json = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    payload_json = payload_json.replace("</", "<\\/")
    return HTML_TEMPLATE.replace("__TITLE__", title).replace("__PAYLOAD__", payload_json)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("main_csv", type=Path, help="Main rocket_nand_<index>_op<id>.csv export")
    parser.add_argument("-o", "--output", type=Path, help="Output HTML path")
    parser.add_argument("--open", action="store_true", help="Open the generated report in the default browser")
    args = parser.parse_args()

    main_path = args.main_csv.expanduser().resolve()
    if not main_path.is_file():
        parser.error(f"{main_path} does not exist")
    output_path = (
        args.output.expanduser().resolve()
        if args.output
        else main_path.with_name(f"{main_path.stem}_analysis.html")
    )

    payload = build_payload(main_path)
    output_path.write_text(render_html(payload), encoding="utf-8")
    print(f"Generated {output_path}")
    print(
        f"Flight {payload['metrics']['flightIndex']}: "
        f"{payload['metrics']['maxAltM']:.1f} m AGL, "
        f"{payload['metrics']['durationS']:.1f} s, "
        f"{payload['metrics']['gpsPoints']} GPS fixes"
    )
    if args.open:
        webbrowser.open(output_path.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
