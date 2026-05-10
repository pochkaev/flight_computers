#!/usr/bin/env python3
"""Generate an offline RocketV10 flight replay from a CSV log."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import os
from typing import Any


STATE_NAMES = {
    0: "IDLE",
    1: "PAD",
    2: "ASCENT",
    3: "COAST",
    4: "DESCENT",
    5: "LANDED",
    6: "ABORT",
}


def parse_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def parse_int(value: str | None) -> int | None:
    parsed = parse_float(value)
    return int(parsed) if parsed is not None else None


def read_csv_with_metadata(path: str) -> tuple[dict[str, str], list[dict[str, str]]]:
    metadata: dict[str, str] = {}
    csv_lines: list[str] = []
    with open(path, newline="") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("#"):
                item = stripped[1:].strip()
                if "=" in item:
                    key, value = item.split("=", 1)
                    metadata[key.strip()] = value.strip()
                continue
            if stripped:
                csv_lines.append(line)
    if not csv_lines:
        return metadata, []
    return metadata, list(csv.DictReader(csv_lines))


def load_points(path: str) -> tuple[list[dict[str, Any]], dict[str, str]]:
    points: list[dict[str, Any]] = []
    metadata, rows = read_csv_with_metadata(path)
    for row in rows:
        ms = parse_float(row.get("ms"))
        rel_alt = parse_float(row.get("rel_alt_m"))
        if ms is None or rel_alt is None:
            continue
        state = parse_int(row.get("state"))
        point = {
            "ms": ms,
            "state": state,
            "stateName": STATE_NAMES.get(state, str(state)),
            "flags": parse_int(row.get("flags")),
            "relAlt": rel_alt,
            "alt": parse_float(row.get("alt_m")),
            "vel": parse_float(row.get("vel_mps")),
            "batt": parse_float(row.get("batt_v")),
            "gpsFix": parse_int(row.get("gps_fix")),
            "sats": parse_int(row.get("sats")),
            "lat": parse_float(row.get("lat")),
            "lon": parse_float(row.get("lon")),
            "gpsAlt": parse_float(row.get("gps_alt_m")),
            "gpsRelAlt": parse_float(row.get("gps_rel_alt_m")),
            "rollDeg": parse_float(row.get("roll")),
            "pitchDeg": parse_float(row.get("pitch")),
            "yawDeg": parse_float(row.get("yaw")),
            "attitudeSource": "full",
        }
        points.append(point)
    if not points:
        return [], metadata
    t0 = points[0]["ms"]
    for p in points:
        p["t"] = (p["ms"] - t0) / 1000.0
    return points, metadata


def load_imu_points(path: str, t0_ms: float) -> tuple[list[dict[str, Any]], dict[str, str]]:
    samples: list[dict[str, Any]] = []
    metadata, rows = read_csv_with_metadata(path)
    for row in rows:
        ms = parse_float(row.get("ms"))
        if ms is None:
            continue
        sample = {
            "ms": ms,
            "t": (ms - t0_ms) / 1000.0,
            "seq": parse_int(row.get("seq")),
            "state": parse_int(row.get("state")),
            "ax": parse_float(row.get("ax")),
            "ay": parse_float(row.get("ay")),
            "az": parse_float(row.get("az")),
            "gx": parse_float(row.get("gx")),
            "gy": parse_float(row.get("gy")),
            "gz": parse_float(row.get("gz")),
            "rollDeg": parse_float(row.get("roll")),
            "pitchDeg": parse_float(row.get("pitch")),
            "yawDeg": parse_float(row.get("yaw")),
            "attitudeSource": "imu",
        }
        if sample["rollDeg"] is None and sample["pitchDeg"] is None and sample["yawDeg"] is None:
            continue
        samples.append(sample)
    samples.sort(key=lambda p: p["t"])
    return samples, metadata


def load_attitude_points(path: str, t0_ms: float) -> tuple[list[dict[str, Any]], dict[str, str]]:
    samples: list[dict[str, Any]] = []
    metadata, rows = read_csv_with_metadata(path)
    for row in rows:
        ms = parse_float(row.get("ms"))
        if ms is None:
            continue
        sample = {
            "ms": ms,
            "t": (ms - t0_ms) / 1000.0,
            "seq": parse_int(row.get("seq")),
            "state": parse_int(row.get("state")),
            "qw": parse_float(row.get("qw")),
            "qx": parse_float(row.get("qx")),
            "qy": parse_float(row.get("qy")),
            "qz": parse_float(row.get("qz")),
            "rollDeg": parse_float(row.get("roll")),
            "pitchDeg": parse_float(row.get("pitch")),
            "yawDeg": parse_float(row.get("yaw")),
            "diagFlags": parse_int(row.get("diag_flags")),
            "flags": parse_int(row.get("flags")),
            "accelCorr": parse_int(row.get("accel_corr")),
            "magCorr": parse_int(row.get("mag_corr")),
            "gyroOnly": parse_int(row.get("gyro_only")),
            "attitudeSource": "quat",
        }
        if sample["qw"] is None or sample["qx"] is None or sample["qy"] is None or sample["qz"] is None:
            continue
        samples.append(sample)
    samples.sort(key=lambda p: p["t"])
    return samples, metadata


def load_stream_points(path: str, t0_ms: float, fields: tuple[str, ...]) -> tuple[list[dict[str, Any]], dict[str, str]]:
    samples: list[dict[str, Any]] = []
    metadata, rows = read_csv_with_metadata(path)
    for row in rows:
        ms = parse_float(row.get("ms"))
        if ms is None:
            continue
        sample: dict[str, Any] = {
            "ms": ms,
            "t": (ms - t0_ms) / 1000.0,
            "seq": parse_int(row.get("seq")),
            "state": parse_int(row.get("state")),
        }
        for field in fields:
            if field not in sample:
                sample[field] = parse_float(row.get(field))
        samples.append(sample)
    samples.sort(key=lambda p: p["t"])
    return samples, metadata


def minmax(points: list[dict[str, Any]], key: str) -> tuple[float, float]:
    vals = [p[key] for p in points if isinstance(p.get(key), (int, float)) and math.isfinite(p[key])]
    if not vals:
        return 0.0, 1.0
    lo, hi = min(vals), max(vals)
    if lo == hi:
        lo -= 1.0
        hi += 1.0
    return lo, hi


def estimate_hz(samples: list[dict[str, Any]]) -> float | None:
    if len(samples) < 2:
        return None
    duration = samples[-1]["t"] - samples[0]["t"]
    if duration <= 0:
        return None
    return (len(samples) - 1) / duration


def estimate_sample_gaps(samples: list[dict[str, Any]]) -> int:
    if len(samples) < 4:
        return 0
    dts = [
        samples[i]["t"] - samples[i - 1]["t"]
        for i in range(1, len(samples))
        if isinstance(samples[i].get("t"), (int, float))
        and isinstance(samples[i - 1].get("t"), (int, float))
        and samples[i]["t"] > samples[i - 1]["t"]
    ]
    if len(dts) < 3:
        return 0
    median_dt = sorted(dts)[len(dts) // 2]
    if median_dt <= 0:
        return 0
    gaps = 0
    threshold = max(0.5, median_dt * 5.0)
    for dt in dts:
        if dt > threshold:
            gaps += 1
    return gaps


def stream_summary(samples: list[dict[str, Any]], valid_key: str | None = None) -> dict[str, Any]:
    valid = None
    if valid_key:
        valid = sum(1 for sample in samples if sample.get(valid_key) not in (None, 0.0))
    return {
        "rows": len(samples),
        "hz": estimate_hz(samples),
        "dropped": estimate_sample_gaps(samples),
        "valid": valid,
    }


def attitude_trust_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if not samples:
        return {"gyroOnlyRows": 0, "gyroOnlyPercent": None}
    gyro_only = sum(1 for sample in samples if sample.get("gyroOnly"))
    return {
        "gyroOnlyRows": gyro_only,
        "gyroOnlyPercent": gyro_only * 100.0 / len(samples),
    }


def build_payload(
    points: list[dict[str, Any]],
    source: str,
    metadata: dict[str, str],
    imu_points: list[dict[str, Any]] | None = None,
    imu_source: str | None = None,
    imu_metadata: dict[str, str] | None = None,
    baro_points: list[dict[str, Any]] | None = None,
    baro_source: str | None = None,
    gps_points: list[dict[str, Any]] | None = None,
    gps_source: str | None = None,
) -> dict[str, Any]:
    max_alt = max(points, key=lambda p: p["relAlt"])
    max_vel = max((p for p in points if isinstance(p.get("vel"), (int, float))), key=lambda p: p["vel"])
    batt_lo, batt_hi = minmax(points, "batt")
    imu_points = imu_points or []
    baro_points = baro_points or []
    gps_points = gps_points or []
    return {
        "source": source,
        "imuSource": imu_source,
        "baroSource": baro_source,
        "gpsSource": gps_source,
        "attitudeSource": "quaternion" if imu_points and imu_points[0].get("attitudeSource") == "quat" else ("IMU" if imu_points else None),
        "metadata": metadata,
        "imuMetadata": imu_metadata or {},
        "points": points,
        "attitudePoints": imu_points,
        "baroPoints": baro_points,
        "gpsPoints": gps_points,
        "summary": {
            "duration": points[-1]["t"],
            "rows": len(points),
            "imuRows": len(imu_points),
            "fullHz": estimate_hz(points),
            "imuHz": estimate_hz(imu_points),
            "fullDropped": estimate_sample_gaps(points),
            "attitudeTrust": attitude_trust_summary(imu_points),
            "baro": stream_summary(baro_points),
            "gps": stream_summary(gps_points, "lat"),
            "maxAlt": max_alt,
            "maxVel": max_vel,
            "battLo": batt_lo,
            "battHi": batt_hi,
            "altDomain": minmax(points, "relAlt"),
            "velDomain": minmax(points, "vel"),
        },
    }


HTML_TEMPLATE = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RocketV10 Flight Replay - __TITLE__</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #101418;
      --panel: #171d22;
      --grid: #31404a;
      --text: #e8edf2;
      --muted: #93a4b2;
      --ok: #4cc36f;
      --warn: #e6bb4a;
      --bad: #ef6461;
      --accent: #73b7ff;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font: 14px/1.35 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    .layout {{
      display: grid;
      grid-template-columns: minmax(520px, 1fr) 390px;
      min-height: 100vh;
    }}
    .stage {{
      position: relative;
      min-height: 720px;
      background: #101418;
    }}
    canvas {{
      width: 100%;
      display: block;
    }}
    #rocket {{
      height: calc(100vh - 72px);
      min-height: 620px;
    }}
    .controls {{
      height: 72px;
      display: grid;
      grid-template-columns: auto 1fr 112px 120px;
      gap: 10px;
      align-items: center;
      padding: 12px 16px;
      border-top: 1px solid #26313a;
      background: #0d1216;
    }}
    button, select {{
      border: 1px solid #34434f;
      border-radius: 6px;
      background: #11171c;
      color: var(--text);
      padding: 8px 10px;
      font-weight: 650;
    }}
    input[type="range"] {{ width: 100%; }}
    .side {{
      background: var(--panel);
      border-left: 1px solid #26313a;
      padding: 16px;
      overflow: auto;
    }}
    h1 {{ font-size: 18px; margin: 0 0 4px; font-weight: 650; }}
    .source {{ color: var(--muted); font-size: 12px; overflow-wrap: anywhere; margin-bottom: 14px; }}
    .grid {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-bottom: 14px;
    }}
    .metric {{
      background: #11171c;
      border: 1px solid #26313a;
      border-radius: 6px;
      padding: 8px;
      min-height: 46px;
    }}
    .wide {{ grid-column: 1 / -1; }}
    .label {{
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: .04em;
    }}
    .value {{ font-size: 18px; margin-top: 3px; white-space: nowrap; }}
    .chart {{
      height: 150px;
      margin: 10px 0 14px;
      background: #10161b;
      border: 1px solid #26313a;
      border-radius: 6px;
    }}
    .timeline {{
      color: var(--muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 12px;
      background: #0d1216;
      border: 1px solid #26313a;
      border-radius: 6px;
      padding: 8px;
      white-space: pre-wrap;
    }}
    @media (max-width: 920px) {{
      .layout {{ grid-template-columns: 1fr; }}
      .side {{ border-left: 0; border-top: 1px solid #26313a; }}
      #rocket {{ height: 560px; min-height: 560px; }}
      .controls {{ grid-template-columns: auto 1fr; height: auto; }}
    }}
  </style>
</head>
<body>
  <div class="layout">
    <div class="stage">
      <canvas id="rocket"></canvas>
      <div class="controls">
        <button id="play">Play</button>
        <input id="scrub" type="range" min="0" value="0" step="1">
        <select id="speed">
          <option value="0.1">0.1x</option>
          <option value="0.25" selected>0.25x</option>
          <option value="0.5">0.5x</option>
          <option value="1">1x</option>
          <option value="3">3x</option>
          <option value="8">8x</option>
          <option value="20">20x</option>
        </select>
        <div id="timeReadout">0.0 s</div>
      </div>
    </div>
    <aside class="side">
      <h1>RocketV10 Replay</h1>
      <div class="source">__SOURCE__</div>
      <div class="grid">
        <div class="metric"><div class="label">State</div><div id="state" class="value">-</div></div>
        <div class="metric"><div class="label">Battery</div><div id="batt" class="value">-</div></div>
        <div class="metric"><div class="label">Rel Alt</div><div id="relAlt" class="value">-</div></div>
        <div class="metric"><div class="label">Velocity</div><div id="vel" class="value">-</div></div>
        <div class="metric"><div class="label">Roll</div><div id="roll" class="value">-</div></div>
        <div class="metric"><div class="label">Pitch</div><div id="pitch" class="value">-</div></div>
        <div class="metric"><div class="label">Yaw</div><div id="yaw" class="value">-</div></div>
        <div class="metric"><div class="label">GPS</div><div id="gps" class="value">-</div></div>
        <div class="metric wide"><div class="label">Position</div><div id="pos" class="value">-</div></div>
        <div class="metric wide"><div class="label">Samples</div><div id="samples" class="value">-</div></div>
        <div class="metric wide"><div class="label">Detail Streams</div><div id="streams" class="value">-</div></div>
        <div class="metric wide"><div class="label">Attitude Trust</div><div id="attTrust" class="value">-</div></div>
        <div class="metric"><div class="label">Max Alt</div><div id="maxAlt" class="value">-</div></div>
        <div class="metric"><div class="label">Max Vel</div><div id="maxVel" class="value">-</div></div>
      </div>
      <canvas id="altChart" class="chart"></canvas>
      <canvas id="velChart" class="chart"></canvas>
      <div id="timeline" class="timeline"></div>
    </aside>
  </div>

<script>
const DATA = __PAYLOAD__;
const points = DATA.points;
const attitudePoints = (DATA.attitudePoints && DATA.attitudePoints.length) ? DATA.attitudePoints : points;
const baroPoints = DATA.baroPoints || [];
const gpsPoints = DATA.gpsPoints || [];
const altitudeTrace = (baroPoints.length ? baroPoints.map(p => ({{t:p.t, relAlt:p.rel_alt_m, vel:p.vel_mps}})) : points)
  .filter(p => Number.isFinite(p.t) && Number.isFinite(p.relAlt));
const gpsTrace = (gpsPoints.length ? gpsPoints.map(p => ({{t:p.t, lat:p.lat, lon:p.lon}})) : points)
  .filter(p => Number.isFinite(p.t));
const summary = DATA.summary;
const stateColors = {{1:'#93a4b2',2:'#4cc36f',3:'#e6bb4a',4:'#73b7ff',5:'#b58cff',6:'#ef6461'}};
const rocketCanvas = document.getElementById('rocket');
const rocketCtx = rocketCanvas.getContext('2d');
const altCanvas = document.getElementById('altChart');
const velCanvas = document.getElementById('velChart');
const altCtx = altCanvas.getContext('2d');
const velCtx = velCanvas.getContext('2d');
const scrub = document.getElementById('scrub');
let idx = 0;
let playing = false;
let lastTs = 0;
let playT = 0;

scrub.max = String(points.length - 1);
document.getElementById('play').onclick = () => {{
  playing = !playing;
  lastTs = 0;
  playT = points[idx].t;
  document.getElementById('play').textContent = playing ? 'Pause' : 'Play';
  requestAnimationFrame(frame);
}};
scrub.oninput = () => {{ idx = Number(scrub.value); playT = points[idx].t; lastTs = 0; update(); }};

function fitCanvas(canvas) {{
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}}
function fmt(v, d=1) {{ return Number.isFinite(v) ? v.toFixed(d) : '-'; }}
function deg(v) {{ return Number.isFinite(v) ? `${{v.toFixed(1)}} deg` : '-'; }}
function meters(v) {{ return Number.isFinite(v) ? `${{v.toFixed(2)}} m` : '-'; }}
function mps(v) {{ return Number.isFinite(v) ? `${{v.toFixed(2)}} m/s` : '-'; }}
function setText(id, text) {{ document.getElementById(id).textContent = text; }}
function clamp(v, lo, hi) {{ return Math.max(lo, Math.min(hi, v)); }}
function pct(v) {{ return Number.isFinite(v) ? `${{v.toFixed(1)}}%` : '-'; }}
function streamText(s, label) {{
  if (!s || !s.rows) return `${{label}} none`;
  const hz = Number.isFinite(s.hz) ? `${{s.hz.toFixed(1)}} Hz` : 'rate ?';
  const dropped = s.dropped ? `, long gaps ${{s.dropped}}` : '';
  const valid = Number.isFinite(s.valid) ? `, valid ${{s.valid}}` : '';
  return `${{label}} ${{s.rows}} (${{hz}}${{dropped}}${{valid}})`;
}}
function wrapDeg(v) {{
  if (!Number.isFinite(v)) return 0;
  let out = ((v + 180) % 360 + 360) % 360 - 180;
  return out === -180 ? 180 : out;
}}
function lerpAngleDeg(a, b, f) {{
  if (!Number.isFinite(a) && !Number.isFinite(b)) return 0;
  if (!Number.isFinite(a)) return b;
  if (!Number.isFinite(b)) return a;
  let delta = ((b - a + 180) % 360 + 360) % 360 - 180;
  return wrapDeg(a + delta * f);
}}
function normalizeQuat(q) {{
  const n = Math.hypot(q.qw || 0, q.qx || 0, q.qy || 0, q.qz || 0);
  if (!Number.isFinite(n) || n <= 1e-9) return {{qw:1, qx:0, qy:0, qz:0}};
  return {{qw:q.qw/n, qx:q.qx/n, qy:q.qy/n, qz:q.qz/n}};
}}
function slerpQuat(a, b, f) {{
  a = normalizeQuat(a); b = normalizeQuat(b);
  let dot = a.qw*b.qw + a.qx*b.qx + a.qy*b.qy + a.qz*b.qz;
  if (dot < 0) {{ b = {{qw:-b.qw, qx:-b.qx, qy:-b.qy, qz:-b.qz}}; dot = -dot; }}
  if (dot > 0.9995) {{
    return normalizeQuat({{
      qw:a.qw + (b.qw-a.qw)*f,
      qx:a.qx + (b.qx-a.qx)*f,
      qy:a.qy + (b.qy-a.qy)*f,
      qz:a.qz + (b.qz-a.qz)*f
    }});
  }}
  const theta0 = Math.acos(Math.max(-1, Math.min(1, dot)));
  const theta = theta0 * f;
  const sinTheta = Math.sin(theta), sinTheta0 = Math.sin(theta0);
  const s0 = Math.cos(theta) - dot * sinTheta / sinTheta0;
  const s1 = sinTheta / sinTheta0;
  return {{
    qw:a.qw*s0 + b.qw*s1,
    qx:a.qx*s0 + b.qx*s1,
    qy:a.qy*s0 + b.qy*s1,
    qz:a.qz*s0 + b.qz*s1
  }};
}}
function eulerFromQuat(q) {{
  q = normalizeQuat(q);
  const sinr = 2 * (q.qw*q.qx + q.qy*q.qz);
  const cosr = 1 - 2 * (q.qx*q.qx + q.qy*q.qy);
  const roll = Math.atan2(sinr, cosr);
  const sinp = 2 * (q.qw*q.qy - q.qz*q.qx);
  const pitch = Math.asin(Math.max(-1, Math.min(1, sinp)));
  const siny = 2 * (q.qw*q.qz + q.qx*q.qy);
  const cosy = 1 - 2 * (q.qy*q.qy + q.qz*q.qz);
  const yaw = Math.atan2(siny, cosy);
  return {{rollDeg: roll*180/Math.PI, pitchDeg: pitch*180/Math.PI, yawDeg: yaw*180/Math.PI}};
}}
function attitudeForTime(t) {{
  if (!attitudePoints.length) return {{}};
  let lo = 0, hi = attitudePoints.length - 1;
  if (t <= attitudePoints[0].t) return attitudePoints[0];
  if (t >= attitudePoints[hi].t) return attitudePoints[hi];
  while (lo + 1 < hi) {{
    const mid = (lo + hi) >> 1;
    if (attitudePoints[mid].t <= t) lo = mid; else hi = mid;
  }}
  const a = attitudePoints[lo], b = attitudePoints[hi];
  const f = (t - a.t) / Math.max(0.001, b.t - a.t);
  if (Number.isFinite(a.qw) && Number.isFinite(b.qw)) {{
    const q = slerpQuat(a, b, f);
    const e = eulerFromQuat(q);
    return {{
      t,
      ...q,
      ...e,
      accelCorr: f < 0.5 ? a.accelCorr : b.accelCorr,
      magCorr: f < 0.5 ? a.magCorr : b.magCorr,
      gyroOnly: f < 0.5 ? a.gyroOnly : b.gyroOnly,
      attitudeSource: 'quat'
    }};
  }}
  return {{
    t,
    rollDeg: lerpAngleDeg(a.rollDeg, b.rollDeg, f),
    pitchDeg: lerpAngleDeg(a.pitchDeg, b.pitchDeg, f),
    yawDeg: lerpAngleDeg(a.yawDeg, b.yawDeg, f),
    attitudeSource: a.attitudeSource || b.attitudeSource || 'full'
  }};
}}
function correctedAttitude(d) {{
  const source = attitudeForTime(d.t);
  return {{
    roll: wrapDeg((source.rollDeg || 0) - 180),
    pitch: wrapDeg(source.pitchDeg || 0),
    yaw: wrapDeg(source.yawDeg || 0),
    source: source.attitudeSource || 'full',
    gyroOnly: source.gyroOnly,
    accelCorr: source.accelCorr,
    magCorr: source.magCorr,
  }};
}}
function gpsOrigin() {{
  return gpsTrace.find(p => Number.isFinite(p.lat) && Number.isFinite(p.lon) && Math.abs(p.lat) > 0.001 && Math.abs(p.lon) > 0.001);
}}
const origin = gpsOrigin();
const originLat = origin ? origin.lat * Math.PI / 180 : 0;
const originLon = origin ? origin.lon * Math.PI / 180 : 0;
const originCos = Math.cos(originLat);
function eastNorth(d) {{
  if (!origin || !Number.isFinite(d.lat) || !Number.isFinite(d.lon) || d.lat === 0 || d.lon === 0) return null;
  return {{
    east: (d.lon * Math.PI / 180 - originLon) * originCos * 6371000,
    north: (d.lat * Math.PI / 180 - originLat) * 6371000
  }};
}}
const enPoints = gpsTrace.map(eastNorth).filter(Boolean);
const eastVals = enPoints.map(p => p.east), northVals = enPoints.map(p => p.north);
const eastMin = Math.min(...eastVals, -20), eastMax = Math.max(...eastVals, 20);
const northMin = Math.min(...northVals, -20), northMax = Math.max(...northVals, 20);

function rotPoint(p, rollDeg, pitchDeg, yawDeg) {{
  const r = rollDeg * Math.PI / 180, pch = pitchDeg * Math.PI / 180, y = yawDeg * Math.PI / 180;
  let [x, yy, z] = p;
  let c = Math.cos(r), s = Math.sin(r);
  [yy, z] = [yy * c - z * s, yy * s + z * c];
  c = Math.cos(pch); s = Math.sin(pch);
  [x, z] = [x * c + z * s, -x * s + z * c];
  c = Math.cos(y); s = Math.sin(y);
  [x, yy] = [x * c - yy * s, x * s + yy * c];
  return [x, yy, z];
}}
function project(p, cx, cy, scale) {{
  const dist = 4.2;
  const f = scale / (dist - p[1]);
  return [cx + p[0] * f, cy - p[2] * f];
}}
function drawLine(ctx, pts, color, width, closed=false) {{
  if (!pts.length) return;
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (let i=1; i<pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
  if (closed) ctx.closePath();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
}}
function drawPoly(ctx, pts, stroke, fill) {{
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (let i=1; i<pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 1.5;
  ctx.stroke();
}}
function drawRocketModel(ctx, d, cx, cy, scale) {{
  const att = correctedAttitude(d);
  const roll = att.roll, pitch = att.pitch, yaw = att.yaw;
  const sides = 12, radius = 0.16, tail = -0.9, shoulder = 0.58, nose = 0.95;
  const rings = [tail, shoulder].map(z => Array.from({length:sides}, (_, i) => {{
    const a = i * Math.PI * 2 / sides;
    return [Math.cos(a)*radius, Math.sin(a)*radius, z];
  }}));
  const toScreen = p => project(rotPoint(p, roll, pitch, yaw), cx, cy, scale);
  const r0 = rings[0].map(toScreen), r1 = rings[1].map(toScreen), np = toScreen([0,0,nose]);
  for (let i=0; i<sides; i++) {{
    drawLine(ctx, [r0[i], r1[i]], '#6fb6ff', 1.8);
    drawLine(ctx, [r1[i], np], '#dce9f5', 1.2);
  }}
  drawLine(ctx, r0, '#6fb6ff', 2, true);
  drawLine(ctx, r1, '#6fb6ff', 2, true);
  for (let i=0; i<4; i++) {{
    const a = i * Math.PI / 2 + Math.PI / 4;
    const b = [Math.cos(a)*radius, Math.sin(a)*radius, tail + 0.18];
    const c = [Math.cos(a)*0.44, Math.sin(a)*0.44, tail - 0.04];
    const e = [Math.cos(a)*radius, Math.sin(a)*radius, tail + 0.48];
    drawPoly(ctx, [toScreen(b), toScreen(c), toScreen(e)], '#ffcf6f', 'rgba(230,187,74,.22)');
  }}
  drawLine(ctx, [toScreen([0,0,tail]), toScreen([0,0,nose])], '#ffffff', 4);
}}

function drawScene() {{
  fitCanvas(rocketCanvas);
  const rect = rocketCanvas.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  const d = points[idx];
  const att = correctedAttitude(d);
  rocketCtx.clearRect(0,0,w,h);
  rocketCtx.fillStyle = '#101418';
  rocketCtx.fillRect(0,0,w,h);
  if (att.gyroOnly) {{
    rocketCtx.fillStyle = 'rgba(239,100,97,.14)';
    rocketCtx.fillRect(0, 0, w, h);
  }}
  rocketCtx.strokeStyle = '#25323b';
  rocketCtx.lineWidth = 1;
  for (let x=0; x<w; x+=40) {{ rocketCtx.beginPath(); rocketCtx.moveTo(x,0); rocketCtx.lineTo(x,h); rocketCtx.stroke(); }}
  for (let y=0; y<h; y+=40) {{ rocketCtx.beginPath(); rocketCtx.moveTo(0,y); rocketCtx.lineTo(w,y); rocketCtx.stroke(); }}

  const sideX = 70, sideY = 82, sideW = Math.max(240, w * 0.28), sideH = Math.max(360, h - 150);
  const traceAltMax = Math.max(...altitudeTrace.map(p => p.relAlt).filter(Number.isFinite), summary.altDomain[1]);
  const altMax = Math.max(20, traceAltMax);
  rocketCtx.strokeStyle = '#31404a';
  rocketCtx.strokeRect(sideX, sideY, sideW, sideH);
  rocketCtx.fillStyle = '#93a4b2';
  rocketCtx.font = '12px system-ui';
  rocketCtx.fillText('ALTITUDE', sideX, sideY - 12);
  rocketCtx.fillText(`${{fmt(altMax,0)}} m`, sideX + 8, sideY + 16);
  rocketCtx.fillText('0 m', sideX + 8, sideY + sideH - 8);
  rocketCtx.strokeStyle = '#73b7ff';
  rocketCtx.lineWidth = 2;
  rocketCtx.beginPath();
  altitudeTrace.filter(p => p.t <= d.t).forEach((p, i) => {{
    const x = sideX + (p.t / summary.duration) * sideW;
    const y = sideY + sideH - clamp(p.relAlt / altMax, 0, 1) * sideH;
    if (i === 0) rocketCtx.moveTo(x,y); else rocketCtx.lineTo(x,y);
  }});
  rocketCtx.stroke();
  const rx = sideX + (d.t / summary.duration) * sideW;
  const ry = sideY + sideH - clamp(d.relAlt / altMax, 0, 1) * sideH;
  rocketCtx.fillStyle = stateColors[d.state] || '#73b7ff';
  rocketCtx.beginPath(); rocketCtx.arc(rx, ry, 6, 0, Math.PI*2); rocketCtx.fill();

  const mapX = w - sideW - 70, mapY = 82, mapW = sideW, mapH = sideH;
  rocketCtx.strokeStyle = '#31404a';
  rocketCtx.strokeRect(mapX, mapY, mapW, mapH);
  rocketCtx.fillStyle = '#93a4b2';
  rocketCtx.fillText('GPS GROUND TRACK', mapX, mapY - 12);
  const en = eastNorth(d);
  const ex = e => mapX + (e - eastMin) / Math.max(1, eastMax - eastMin) * mapW;
  const ny = n => mapY + mapH - (n - northMin) / Math.max(1, northMax - northMin) * mapH;
  rocketCtx.strokeStyle = '#4cc36f';
  rocketCtx.beginPath();
  let open = false;
  gpsTrace.forEach(sample => {{
    if (sample.t > d.t) return;
    const p = eastNorth(sample);
    if (!p) continue;
    const x = ex(p.east), y = ny(p.north);
    if (!open) {{ rocketCtx.moveTo(x,y); open = true; }} else rocketCtx.lineTo(x,y);
  }});
  rocketCtx.stroke();
  if (en) {{
    rocketCtx.fillStyle = stateColors[d.state] || '#4cc36f';
    rocketCtx.beginPath(); rocketCtx.arc(ex(en.east), ny(en.north), 6, 0, Math.PI*2); rocketCtx.fill();
  }}
  rocketCtx.fillStyle = '#93a4b2';
  rocketCtx.fillText('N', mapX + mapW - 18, mapY + 18);

  drawRocketModel(rocketCtx, d, w * 0.5, h * 0.52, Math.min(w, h) * 0.34);
  rocketCtx.fillStyle = '#e8edf2';
  rocketCtx.font = '15px system-ui';
  rocketCtx.fillText(`state ${{d.stateName}}   t ${{fmt(d.t,1)}}s   relAlt ${{fmt(d.relAlt,1)}} m   vel ${{fmt(d.vel,1)}} m/s`, 18, 28);
  rocketCtx.fillStyle = '#93a4b2';
  const conf = att.gyroOnly ? 'gyro-only' : `accel ${{att.accelCorr ? 'on' : 'off'}} mag ${{att.magCorr ? 'on' : 'off'}}`;
  rocketCtx.fillText(`roll ${{fmt(att.roll,1)}}   pitch ${{fmt(att.pitch,1)}}   yaw ${{fmt(att.yaw,1)}}   attitude ${{att.source}} ${{conf}}`, 18, 52);
  if (att.gyroOnly) {{
    rocketCtx.fillStyle = '#ef6461';
    rocketCtx.fillRect(18, 66, 188, 24);
    rocketCtx.fillStyle = '#101418';
    rocketCtx.font = '13px system-ui';
    rocketCtx.fillText('GYRO-ONLY ATTITUDE', 28, 83);
  }}
}}

function drawChart(canvas, ctx, key, label, color) {{
  fitCanvas(canvas);
  const rect = canvas.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle = '#10161b';
  ctx.fillRect(0,0,w,h);
  if (attitudePoints.length) {{
    const xForTime = t => 10 + (t / Math.max(0.001, summary.duration)) * (w - 20);
    ctx.fillStyle = 'rgba(239,100,97,.16)';
    let openT = null;
    attitudePoints.forEach((p, i) => {{
      const gyroOnly = !!p.gyroOnly;
      if (gyroOnly && openT === null) openT = p.t;
      const last = i === attitudePoints.length - 1;
      if ((!gyroOnly || last) && openT !== null) {{
        const endT = gyroOnly && last ? p.t : attitudePoints[Math.max(0, i - 1)].t;
        ctx.fillRect(xForTime(openT), 0, Math.max(2, xForTime(endT) - xForTime(openT)), h);
        openT = null;
      }}
    }});
  }}
  ctx.strokeStyle = '#31404a';
  for (let i=1; i<4; i++) {{ const y=h*i/4; ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke(); }}
  const vals = points.map(p => p[key]).filter(Number.isFinite);
  let lo = Math.min(...vals), hi = Math.max(...vals);
  if (lo === hi) {{ lo -= 1; hi += 1; }}
  const yFor = v => h - 18 - ((v - lo) / (hi - lo)) * (h - 38);
  const xFor = i => 10 + (i / Math.max(1, points.length - 1)) * (w - 20);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  points.forEach((p, i) => {{
    if (!Number.isFinite(p[key])) return;
    const x = xFor(i), y = yFor(p[key]);
    if (i === 0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }});
  ctx.stroke();
  ctx.strokeStyle = '#ffffff';
  ctx.beginPath(); ctx.moveTo(xFor(idx), 0); ctx.lineTo(xFor(idx), h); ctx.stroke();
  ctx.fillStyle = '#93a4b2';
  ctx.font = '12px system-ui';
  ctx.fillText(label, 10, 18);
  ctx.fillStyle = '#e8edf2';
  ctx.fillText(`${{fmt(points[idx][key],2)}}  min ${{fmt(lo,2)}} max ${{fmt(hi,2)}}`, 10, h - 8);
}}

function updateMetrics() {{
  const d = points[idx];
  const att = correctedAttitude(d);
  setText('state', d.stateName || '-');
  setText('batt', Number.isFinite(d.batt) ? `${{d.batt.toFixed(3)}} V` : '-');
  setText('relAlt', meters(d.relAlt));
  setText('vel', mps(d.vel));
  setText('roll', deg(att.roll));
  setText('pitch', deg(att.pitch));
  setText('yaw', deg(att.yaw));
  setText('gps', `${{d.gpsFix ?? '-'}}D ${{d.sats ?? '-'}} sats`);
  setText('pos', (Number.isFinite(d.lat) && Number.isFinite(d.lon) && d.lat !== 0) ? `${{d.lat.toFixed(7)}}, ${{d.lon.toFixed(7)}}` : '-');
  const fullRate = Number.isFinite(summary.fullHz) ? `${{summary.fullHz.toFixed(1)}} Hz full` : `${{summary.rows}} full`;
  const attName = DATA.attitudeSource || (summary.imuRows ? 'IMU' : 'full');
  const imuRate = summary.imuRows && Number.isFinite(summary.imuHz) ? `${{summary.imuHz.toFixed(1)}} Hz ${{attName}}` : 'no attitude CSV';
  const fullDrop = summary.fullDropped ? `, long gaps ${{summary.fullDropped}}` : '';
  setText('samples', `${{summary.rows}} full rows (${{fullRate}}${{fullDrop}}), ${{summary.imuRows}} attitude rows (${{imuRate}})`);
  setText('streams', `${{streamText(summary.baro, 'baro')}}; ${{streamText(summary.gps, 'gps')}}`);
  const trust = summary.attitudeTrust || {{}};
  setText('attTrust', summary.imuRows ? `${{trust.gyroOnlyRows || 0}} gyro-only rows (${{pct(trust.gyroOnlyPercent)}})` : 'full-state Euler only');
  setText('maxAlt', `${{fmt(summary.maxAlt.relAlt,1)}} m`);
  setText('maxVel', `${{fmt(summary.maxVel.vel,1)}} m/s`);
  setText('timeReadout', `${{fmt(d.t,1)}} s`);
  scrub.value = String(idx);
}}
function setTimeline() {{
  const changes = [];
  let prev = null;
  points.forEach(p => {{
    if (p.state !== prev) {{
      changes.push(`${{fmt(p.t,1)}}s  ${{p.stateName}}  alt=${{fmt(p.relAlt,1)}}m  vel=${{fmt(p.vel,1)}}m/s`);
      prev = p.state;
    }}
  }});
  if (attitudePoints.length) {{
    let open = null;
    attitudePoints.forEach((p, i) => {{
      const gyroOnly = !!p.gyroOnly;
      if (gyroOnly && open === null) open = p.t;
      const last = i === attitudePoints.length - 1;
      if ((!gyroOnly || last) && open !== null) {{
        const end = gyroOnly && last ? p.t : attitudePoints[Math.max(0, i - 1)].t;
        changes.push(`${{fmt(open,1)}}-${{fmt(end,1)}}s  GYRO-ONLY attitude`);
        open = null;
      }}
    }});
  }}
  document.getElementById('timeline').textContent = changes.join('\\n');
}}
function update() {{
  updateMetrics();
  drawScene();
  drawChart(altCanvas, altCtx, 'relAlt', 'Relative altitude (m)', '#73b7ff');
  drawChart(velCanvas, velCtx, 'vel', 'Vertical velocity (m/s)', '#e6bb4a');
}}
function indexForTime(t) {{
  while (idx < points.length - 1 && points[idx + 1].t <= t) idx++;
  while (idx > 0 && points[idx].t > t) idx--;
  return idx;
}}
function frame(ts) {{
  if (!playing) return;
  const speed = Number(document.getElementById('speed').value || 0.25);
  if (!lastTs) {{
    lastTs = ts;
    requestAnimationFrame(frame);
    return;
  }}
  const elapsed = (ts - lastTs) / 1000 * speed;
  lastTs = ts;
  playT = Math.min(summary.duration, playT + elapsed);
  idx = indexForTime(playT);
  if (playT >= summary.duration || idx >= points.length - 1) {{
    playing = false;
    document.getElementById('play').textContent = 'Play';
  }}
  update();
  requestAnimationFrame(frame);
}}
window.addEventListener('resize', update);
setTimeline();
update();
</script>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv")
    parser.add_argument("--imu-csv", help="Optional high-rate RocketV10 _imu.csv export. Defaults to sibling *_imu.csv when present.")
    parser.add_argument("--att-csv", help="Optional high-rate RocketV10 _att.csv quaternion attitude export. Defaults to sibling *_att.csv when present.")
    parser.add_argument("--baro-csv", help="Optional RocketV10 _baro.csv export. Defaults to sibling *_baro.csv when present.")
    parser.add_argument("--gps-csv", help="Optional RocketV10 _gps.csv export. Defaults to sibling *_gps.csv when present.")
    parser.add_argument("--no-auto-imu", action="store_true", help="Do not auto-load a sibling *_imu.csv file.")
    parser.add_argument("--no-auto-detail", action="store_true", help="Do not auto-load sibling *_baro.csv or *_gps.csv files.")
    parser.add_argument("-o", "--output")
    args = parser.parse_args()

    input_path = os.path.abspath(args.input_csv)
    points, metadata = load_points(input_path)
    if not points:
        raise SystemExit(f"No usable rows in {input_path}")
    attitude_path = os.path.abspath(args.att_csv) if args.att_csv else None
    if attitude_path is None and not args.no_auto_imu:
        root, ext = os.path.splitext(input_path)
        candidate = root + "_att" + ext
        if os.path.exists(candidate):
            attitude_path = candidate
    imu_path = os.path.abspath(args.imu_csv) if args.imu_csv else None
    if attitude_path is None and imu_path is None and not args.no_auto_imu:
        root, ext = os.path.splitext(input_path)
        candidate = root + "_imu" + ext
        if os.path.exists(candidate):
            imu_path = candidate
    baro_path = os.path.abspath(args.baro_csv) if args.baro_csv else None
    gps_path = os.path.abspath(args.gps_csv) if args.gps_csv else None
    if not args.no_auto_detail:
        root, ext = os.path.splitext(input_path)
        if baro_path is None:
            candidate = root + "_baro" + ext
            if os.path.exists(candidate):
                baro_path = candidate
        if gps_path is None:
            candidate = root + "_gps" + ext
            if os.path.exists(candidate):
                gps_path = candidate

    imu_points: list[dict[str, Any]] = []
    imu_metadata: dict[str, str] = {}
    if attitude_path:
        if not os.path.exists(attitude_path):
            raise SystemExit(f"Attitude CSV not found: {attitude_path}")
        imu_points, imu_metadata = load_attitude_points(attitude_path, points[0]["ms"])
        if not imu_points:
            raise SystemExit(f"No usable attitude rows in {attitude_path}")
    elif imu_path:
        if not os.path.exists(imu_path):
            raise SystemExit(f"IMU CSV not found: {imu_path}")
        imu_points, imu_metadata = load_imu_points(imu_path, points[0]["ms"])
        if not imu_points:
            raise SystemExit(f"No usable IMU rows in {imu_path}")
    baro_points: list[dict[str, Any]] = []
    gps_points: list[dict[str, Any]] = []
    if baro_path:
        if not os.path.exists(baro_path):
            raise SystemExit(f"Barometer CSV not found: {baro_path}")
        baro_points, _ = load_stream_points(
            baro_path,
            points[0]["ms"],
            ("alt_m", "rel_alt_m", "vel_mps", "temp_c", "pres_pa", "diag_flags"),
        )
    if gps_path:
        if not os.path.exists(gps_path):
            raise SystemExit(f"GPS CSV not found: {gps_path}")
        gps_points, _ = load_stream_points(
            gps_path,
            points[0]["ms"],
            ("fix", "sats", "hdop", "lat", "lon", "gps_alt_m", "gps_rel_alt_m", "baro_gps_delta_m", "gps_speed_mps", "fix_age_ms", "flags"),
        )

    payload = json.dumps(
        build_payload(
            points,
            os.path.basename(input_path),
            metadata,
            imu_points,
            os.path.basename(attitude_path or imu_path) if (attitude_path or imu_path) else None,
            imu_metadata,
            baro_points,
            os.path.basename(baro_path) if baro_path else None,
            gps_points,
            os.path.basename(gps_path) if gps_path else None,
        ),
        separators=(",", ":"),
    )
    output = args.output
    if not output:
        root, _ = os.path.splitext(input_path)
        output = root + "_replay.html"
    template = HTML_TEMPLATE.replace("{{", "{").replace("}}", "}")
    html_text = (
        template
        .replace("__TITLE__", html.escape(os.path.basename(input_path)))
        .replace("__SOURCE__", html.escape(input_path))
        .replace("__PAYLOAD__", payload)
    )
    with open(output, "w", encoding="utf-8") as f:
        f.write(html_text)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
