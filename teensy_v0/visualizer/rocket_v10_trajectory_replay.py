#!/usr/bin/env python3
"""Generate a browser animation of a recorded RocketV10 flight trajectory."""

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


def read_rows(path: str) -> list[dict[str, str]]:
    csv_lines: list[str] = []
    with open(path, newline="") as src:
        for line in src:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            csv_lines.append(line)
    return list(csv.DictReader(csv_lines))


def load_points(path: str) -> list[dict[str, Any]]:
    rows = read_rows(path)
    points: list[dict[str, Any]] = []
    first_ms: float | None = None
    first_rel: float | None = None

    for row in rows:
        ms = parse_float(row.get("ms"))
        rel = parse_float(row.get("rel_alt_m"))
        if ms is None or rel is None:
            continue
        if first_ms is None:
            first_ms = ms
            first_rel = rel
        state = parse_int(row.get("state"))
        ax = parse_float(row.get("ax"))
        ay = parse_float(row.get("ay"))
        az = parse_float(row.get("az"))
        accel = None
        if ax is not None and ay is not None and az is not None:
            accel = math.sqrt(ax * ax + ay * ay + az * az)
        normalized_rel = rel - (first_rel or 0.0)
        point = {
            "t": (ms - first_ms) / 1000.0,
            "ms": ms,
            "state": state,
            "stateName": STATE_NAMES.get(state, str(state)),
            "relAltRaw": rel,
            "relAlt": normalized_rel,
            "displayAlt": max(0.0, normalized_rel),
            "vel": parse_float(row.get("vel_mps")),
            "accel": accel,
            "roll": parse_float(row.get("roll")),
            "pitch": parse_float(row.get("pitch")),
            "yaw": parse_float(row.get("yaw")),
            "batt": parse_float(row.get("batt_v")),
            "lat": parse_float(row.get("lat")),
            "lon": parse_float(row.get("lon")),
            "gpsFix": parse_int(row.get("gps_fix")),
            "sats": parse_int(row.get("sats")),
            "health": parse_int(row.get("health")),
            "diagFlags": parse_int(row.get("diag_flags")),
        }
        points.append(point)
    return points


def minmax(points: list[dict[str, Any]], key: str) -> tuple[float, float]:
    vals = [p[key] for p in points if isinstance(p.get(key), (int, float)) and math.isfinite(p[key])]
    if not vals:
        return 0.0, 1.0
    lo, hi = min(vals), max(vals)
    if lo == hi:
        lo -= 1.0
        hi += 1.0
    return lo, hi


def build_payload(points: list[dict[str, Any]], source: str) -> dict[str, Any]:
    if not points:
        raise SystemExit("No usable RocketV10 rows found")
    max_alt = max(points, key=lambda p: p["displayAlt"])
    max_vel = max((p for p in points if isinstance(p.get("vel"), (int, float))), key=lambda p: p["vel"], default=points[0])
    max_accel = max((p for p in points if isinstance(p.get("accel"), (int, float))), key=lambda p: p["accel"], default=points[0])
    return {
        "source": source,
        "points": points,
        "summary": {
            "duration": points[-1]["t"],
            "rows": len(points),
            "maxAlt": max_alt,
            "maxVel": max_vel,
            "maxAccel": max_accel,
            "altDomain": minmax(points, "displayAlt"),
            "velDomain": minmax(points, "vel"),
            "accelDomain": minmax(points, "accel"),
        },
    }


HTML_TEMPLATE = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>RocketV10 Trajectory Replay</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #101418;
      --panel: #171d22;
      --grid: #31404a;
      --text: #e8edf2;
      --muted: #93a4b2;
      --accent: #73b7ff;
      --green: #4cc36f;
      --yellow: #e6bb4a;
      --red: #ef6461;
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
      grid-template-columns: minmax(680px, 1fr) 390px;
      min-height: 100vh;
    }}
    .stage {{
      min-height: 720px;
      background: #101418;
    }}
    canvas {{
      width: 100%;
      display: block;
    }}
    #scene {{ height: calc(100vh - 178px); min-height: 540px; }}
    .charts {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      padding: 10px;
      border-top: 1px solid #26313a;
    }}
    .chart {{
      height: 150px;
      background: #10161b;
      border: 1px solid #26313a;
      border-radius: 6px;
    }}
    .side {{
      background: var(--panel);
      border-left: 1px solid #26313a;
      padding: 16px;
      overflow: auto;
    }}
    h1 {{
      font-size: 18px;
      margin: 0 0 10px;
    }}
    .source {{
      color: var(--muted);
      font: 12px ui-monospace, SFMono-Regular, Menlo, monospace;
      overflow-wrap: anywhere;
      margin-bottom: 14px;
    }}
    .controls {{
      display: grid;
      grid-template-columns: 84px 1fr;
      gap: 10px;
      align-items: center;
      margin-bottom: 14px;
    }}
    button, select {{
      height: 34px;
      border: 1px solid #31404a;
      border-radius: 6px;
      background: #11171c;
      color: var(--text);
    }}
    input[type=range] {{ width: 100%; }}
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
      min-height: 48px;
    }}
    .label {{
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
    }}
    .value {{
      font-size: 18px;
      margin-top: 3px;
      white-space: nowrap;
    }}
    .timeline {{
      background: #0d1216;
      border: 1px solid #26313a;
      border-radius: 6px;
      padding: 10px;
      color: var(--muted);
      white-space: pre-wrap;
      font: 12px ui-monospace, SFMono-Regular, Menlo, monospace;
    }}
    @media (max-width: 960px) {{
      .layout {{ grid-template-columns: 1fr; }}
      .side {{ border-left: 0; border-top: 1px solid #26313a; }}
      #scene {{ height: 560px; }}
    }}
  </style>
</head>
<body>
<div class="layout">
  <main class="stage">
    <canvas id="scene"></canvas>
    <div class="charts">
      <canvas id="altChart" class="chart"></canvas>
      <canvas id="velChart" class="chart"></canvas>
    </div>
  </main>
  <aside class="side">
    <h1>RocketV10 Trajectory Replay</h1>
    <div class="source">__SOURCE__</div>
    <div class="controls">
      <button id="play">Play</button>
      <input id="scrub" type="range" min="0" max="0" value="0">
      <label for="speed">Speed</label>
      <select id="speed">
        <option value="0.25">0.25x</option>
        <option value="0.5">0.5x</option>
        <option value="1" selected>1x</option>
        <option value="2">2x</option>
        <option value="5">5x</option>
        <option value="10">10x</option>
      </select>
    </div>
    <div class="grid">
      <div class="metric"><div class="label">State</div><div id="state" class="value">-</div></div>
      <div class="metric"><div class="label">Time</div><div id="time" class="value">-</div></div>
      <div class="metric"><div class="label">Altitude From Pad</div><div id="alt" class="value">-</div></div>
      <div class="metric"><div class="label">Velocity</div><div id="vel" class="value">-</div></div>
      <div class="metric"><div class="label">Acceleration</div><div id="accel" class="value">-</div></div>
      <div class="metric"><div class="label">Battery</div><div id="batt" class="value">-</div></div>
      <div class="metric"><div class="label">Max Altitude</div><div id="maxAlt" class="value">-</div></div>
      <div class="metric"><div class="label">Max Velocity</div><div id="maxVel" class="value">-</div></div>
    </div>
    <div id="timeline" class="timeline"></div>
  </aside>
</div>
<script>
const DATA = __PAYLOAD__;
const points = DATA.points;
const summary = DATA.summary;
const scene = document.getElementById('scene');
const sceneCtx = scene.getContext('2d');
const altChart = document.getElementById('altChart');
const velChart = document.getElementById('velChart');
const altCtx = altChart.getContext('2d');
const velCtx = velChart.getContext('2d');
const scrub = document.getElementById('scrub');
let idx = 0;
let playing = false;
let playT = 0;
let lastTs = 0;

const stateColors = {{
  0: '#93a4b2',
  1: '#73b7ff',
  2: '#4cc36f',
  3: '#e6bb4a',
  4: '#ef6461',
  5: '#b48cff',
  6: '#ff5c7c',
}};

function fitCanvas(canvas) {{
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}}
function fmt(v, n=1) {{ return Number.isFinite(v) ? v.toFixed(n) : '-'; }}
function clamp(v, lo, hi) {{ return Math.max(lo, Math.min(hi, v)); }}
function setText(id, text) {{ document.getElementById(id).textContent = text; }}
function wrapDeg(v) {{
  if (!Number.isFinite(v)) return 0;
  return ((v + 180) % 360 + 360) % 360 - 180;
}}
function correctedAttitude(d) {{
  return {{
    roll: wrapDeg((d.roll ?? 0) - 180),
    pitch: wrapDeg(d.pitch ?? 0),
    yaw: wrapDeg(d.yaw ?? 0),
  }};
}}
function projectWorld(x, z, w, h, altMax) {{
  const groundY = h - 70;
  const topY = 70;
  const usableH = groundY - topY;
  const y = groundY - clamp(z / altMax, -0.05, 1.05) * usableH;
  const sway = Math.sin(z * 0.035) * 18;
  return [w * 0.48 + x + sway, y];
}}
function drawRocket(ctx, x, y, d, scale) {{
  const att = correctedAttitude(d);
  const angle = -att.pitch * Math.PI / 180;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(angle);
  ctx.scale(scale, scale);
  ctx.fillStyle = '#dce9f5';
  ctx.strokeStyle = '#6fb6ff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, -42);
  ctx.lineTo(13, -20);
  ctx.lineTo(13, 34);
  ctx.lineTo(-13, 34);
  ctx.lineTo(-13, -20);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = '#ef6461';
  ctx.beginPath();
  ctx.moveTo(0, -42);
  ctx.lineTo(13, -20);
  ctx.lineTo(-13, -20);
  ctx.closePath();
  ctx.fill();
  ctx.fillStyle = '#ffcf6f';
  ctx.beginPath();
  ctx.moveTo(-13, 18);
  ctx.lineTo(-32, 42);
  ctx.lineTo(-13, 34);
  ctx.closePath();
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(13, 18);
  ctx.lineTo(32, 42);
  ctx.lineTo(13, 34);
  ctx.closePath();
  ctx.fill();
  if (d.state === 2) {{
    const flame = 22 + Math.sin(performance.now() / 70) * 8;
    ctx.fillStyle = '#ff9f1a';
    ctx.beginPath();
    ctx.moveTo(-7, 34);
    ctx.lineTo(0, 34 + flame);
    ctx.lineTo(7, 34);
    ctx.closePath();
    ctx.fill();
  }}
  ctx.restore();
}}
function drawScene() {{
  fitCanvas(scene);
  const rect = scene.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  const d = points[idx];
  const altMax = Math.max(20, summary.altDomain[1] * 1.08);
  sceneCtx.clearRect(0, 0, w, h);
  sceneCtx.fillStyle = '#101418';
  sceneCtx.fillRect(0, 0, w, h);
  sceneCtx.strokeStyle = '#25323b';
  sceneCtx.lineWidth = 1;
  for (let x = 0; x < w; x += 48) {{ sceneCtx.beginPath(); sceneCtx.moveTo(x, 0); sceneCtx.lineTo(x, h); sceneCtx.stroke(); }}
  for (let y = 0; y < h; y += 48) {{ sceneCtx.beginPath(); sceneCtx.moveTo(0, y); sceneCtx.lineTo(w, y); sceneCtx.stroke(); }}

  const groundY = h - 70;
  sceneCtx.strokeStyle = '#526070';
  sceneCtx.lineWidth = 2;
  sceneCtx.beginPath();
  sceneCtx.moveTo(50, groundY);
  sceneCtx.lineTo(w - 50, groundY);
  sceneCtx.stroke();
  sceneCtx.fillStyle = '#93a4b2';
  sceneCtx.font = '12px system-ui';
  sceneCtx.fillText('pad 0 m', 58, groundY - 8);
  sceneCtx.fillText(`${fmt(altMax, 0)} m`, 58, 76);

  sceneCtx.strokeStyle = '#73b7ff';
  sceneCtx.lineWidth = 3;
  sceneCtx.beginPath();
  for (let i = 0; i <= idx; i++) {{
    const p = points[i];
    const xDrift = (i / Math.max(1, points.length - 1) - 0.5) * Math.min(260, w * 0.25);
    const [x, y] = projectWorld(xDrift, p.displayAlt, w, h, altMax);
    if (i === 0) sceneCtx.moveTo(x, y); else sceneCtx.lineTo(x, y);
  }}
  sceneCtx.stroke();

  const xDrift = (idx / Math.max(1, points.length - 1) - 0.5) * Math.min(260, w * 0.25);
  const [rx, ry] = projectWorld(xDrift, d.displayAlt, w, h, altMax);
  drawRocket(sceneCtx, rx, ry, d, Math.max(0.7, Math.min(1.25, w / 1000)));

  sceneCtx.fillStyle = '#e8edf2';
  sceneCtx.font = '15px system-ui';
  sceneCtx.fillText(`state ${{d.stateName}}   t ${{fmt(d.t,1)}}s   altitude ${{fmt(d.displayAlt,1)}} m   velocity ${{fmt(d.vel,1)}} m/s`, 18, 28);
  sceneCtx.fillStyle = '#93a4b2';
  const att = correctedAttitude(d);
  sceneCtx.fillText(`roll ${{fmt(att.roll,1)}}   pitch ${{fmt(att.pitch,1)}}   yaw ${{fmt(att.yaw,1)}}   source: baro relative altitude normalized to pad`, 18, 52);
}}
function drawChart(canvas, ctx, key, label, color) {{
  fitCanvas(canvas);
  const rect = canvas.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle = '#10161b';
  ctx.fillRect(0,0,w,h);
  ctx.strokeStyle = '#31404a';
  for (let i=1; i<4; i++) {{ const y=h*i/4; ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke(); }}
  const vals = points.map(p => p[key]).filter(Number.isFinite);
  if (vals.length < 2) return;
  let lo = Math.min(...vals), hi = Math.max(...vals);
  if (Math.abs(hi - lo) < 0.01) {{ lo -= 1; hi += 1; }}
  const xFor = i => 10 + (i / Math.max(1, points.length - 1)) * (w - 20);
  const yFor = v => h - 18 - ((v - lo) / (hi - lo)) * (h - 38);
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
  ctx.fillText(`${fmt(points[idx][key],2)}  min ${fmt(lo,2)} max ${fmt(hi,2)}`, 10, h - 8);
}}
function updateMetrics() {{
  const d = points[idx];
  setText('state', d.stateName || '-');
  setText('time', `${fmt(d.t,1)} s`);
  setText('alt', `${fmt(d.displayAlt,2)} m`);
  setText('vel', Number.isFinite(d.vel) ? `${fmt(d.vel,2)} m/s` : '-');
  setText('accel', Number.isFinite(d.accel) ? `${fmt(d.accel,2)} m/s^2` : '-');
  setText('batt', Number.isFinite(d.batt) ? `${fmt(d.batt,2)} V` : '-');
  setText('maxAlt', `${fmt(summary.maxAlt.displayAlt,1)} m`);
  setText('maxVel', `${fmt(summary.maxVel.vel,1)} m/s`);
  scrub.value = String(idx);
}}
function setTimeline() {{
  const rows = [];
  let prev = null;
  for (const p of points) {{
    if (p.state !== prev) {{
      rows.push(`${fmt(p.t,1)}s  ${p.stateName}  alt=${fmt(p.displayAlt,1)}m  vel=${fmt(p.vel,1)}m/s`);
      prev = p.state;
    }}
  }}
  rows.push('');
  rows.push(`max alt ${fmt(summary.maxAlt.displayAlt,1)}m at ${fmt(summary.maxAlt.t,1)}s`);
  rows.push(`max vel ${fmt(summary.maxVel.vel,1)}m/s at ${fmt(summary.maxVel.t,1)}s`);
  rows.push(`max accel ${fmt(summary.maxAccel.accel,1)}m/s^2 at ${fmt(summary.maxAccel.t,1)}s`);
  document.getElementById('timeline').textContent = rows.join('\\n');
}}
function update() {{
  updateMetrics();
  drawScene();
  drawChart(altChart, altCtx, 'displayAlt', 'Altitude from pad (m)', '#73b7ff');
  drawChart(velChart, velCtx, 'vel', 'Vertical velocity (m/s)', '#e6bb4a');
}}
function indexForTime(t) {{
  while (idx < points.length - 1 && points[idx + 1].t <= t) idx++;
  while (idx > 0 && points[idx].t > t) idx--;
}}
function frame(ts) {{
  if (!playing) return;
  if (!lastTs) {{
    lastTs = ts;
    requestAnimationFrame(frame);
    return;
  }}
  const speed = Number(document.getElementById('speed').value || 1);
  playT = Math.min(summary.duration, playT + (ts - lastTs) / 1000 * speed);
  lastTs = ts;
  indexForTime(playT);
  if (playT >= summary.duration || idx >= points.length - 1) {{
    playing = false;
    document.getElementById('play').textContent = 'Play';
  }}
  update();
  requestAnimationFrame(frame);
}}
document.getElementById('play').addEventListener('click', () => {{
  playing = !playing;
  document.getElementById('play').textContent = playing ? 'Pause' : 'Play';
  lastTs = 0;
  if (playing) requestAnimationFrame(frame);
}});
scrub.max = String(points.length - 1);
scrub.addEventListener('input', () => {{
  idx = Number(scrub.value);
  playT = points[idx].t;
  update();
}});
window.addEventListener('resize', update);
setTimeline();
update();
</script>
</body>
</html>
"""


def write_html(payload: dict[str, Any], output: str) -> None:
    source = html.escape(payload["source"])
    data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    template = HTML_TEMPLATE.replace("{{", "{").replace("}}", "}")
    with open(output, "w") as dst:
        dst.write(template.replace("__SOURCE__", source).replace("__PAYLOAD__", data))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="RocketV10 rocket_flight*.csv file")
    parser.add_argument("--output", help="Output HTML path")
    args = parser.parse_args()

    output = args.output
    if not output:
        root, _ = os.path.splitext(args.csv)
        output = root + "_trajectory.html"

    points = load_points(args.csv)
    payload = build_payload(points, args.csv)
    write_html(payload, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
