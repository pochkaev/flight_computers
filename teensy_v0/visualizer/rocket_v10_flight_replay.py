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


def load_points(path: str) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
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
            }
            points.append(point)
    if not points:
        return []
    t0 = points[0]["ms"]
    for p in points:
        p["t"] = (p["ms"] - t0) / 1000.0
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
    max_alt = max(points, key=lambda p: p["relAlt"])
    max_vel = max((p for p in points if isinstance(p.get("vel"), (int, float))), key=lambda p: p["vel"])
    batt_lo, batt_hi = minmax(points, "batt")
    return {
        "source": source,
        "points": points,
        "summary": {
            "duration": points[-1]["t"],
            "rows": len(points),
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
function wrapDeg(v) {{
  if (!Number.isFinite(v)) return 0;
  let out = ((v + 180) % 360 + 360) % 360 - 180;
  return out === -180 ? 180 : out;
}}
function correctedAttitude(d) {{
  return {{
    roll: wrapDeg((d.rollDeg || 0) - 180),
    pitch: wrapDeg(d.pitchDeg || 0),
    yaw: wrapDeg(d.yawDeg || 0),
  }};
}}
function gpsOrigin() {{
  return points.find(p => Number.isFinite(p.lat) && Number.isFinite(p.lon) && Math.abs(p.lat) > 0.001 && Math.abs(p.lon) > 0.001);
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
const enPoints = points.map(eastNorth).filter(Boolean);
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
  rocketCtx.strokeStyle = '#25323b';
  rocketCtx.lineWidth = 1;
  for (let x=0; x<w; x+=40) {{ rocketCtx.beginPath(); rocketCtx.moveTo(x,0); rocketCtx.lineTo(x,h); rocketCtx.stroke(); }}
  for (let y=0; y<h; y+=40) {{ rocketCtx.beginPath(); rocketCtx.moveTo(0,y); rocketCtx.lineTo(w,y); rocketCtx.stroke(); }}

  const sideX = 70, sideY = 82, sideW = Math.max(240, w * 0.28), sideH = Math.max(360, h - 150);
  const altMax = Math.max(20, summary.altDomain[1]);
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
  points.slice(0, idx + 1).forEach((p, i) => {{
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
  for (let i=0; i<=idx; i++) {{
    const p = eastNorth(points[i]);
    if (!p) continue;
    const x = ex(p.east), y = ny(p.north);
    if (!open) {{ rocketCtx.moveTo(x,y); open = true; }} else rocketCtx.lineTo(x,y);
  }}
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
  rocketCtx.fillText(`roll ${{fmt(att.roll,1)}}   pitch ${{fmt(att.pitch,1)}}   yaw ${{fmt(att.yaw,1)}}`, 18, 52);
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
    parser.add_argument("-o", "--output")
    args = parser.parse_args()

    input_path = os.path.abspath(args.input_csv)
    points = load_points(input_path)
    if not points:
        raise SystemExit(f"No usable rows in {input_path}")
    payload = json.dumps(build_payload(points, os.path.basename(input_path)), separators=(",", ":"))
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
