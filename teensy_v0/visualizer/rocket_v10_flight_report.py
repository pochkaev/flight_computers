#!/usr/bin/env python3
"""
Generate a self-contained RocketV10 flight report from a NAND/SD CSV export.

The output is plain HTML with embedded SVG charts and does not require Python
plotting libraries or an internet connection.
"""

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


FIELDS = [
    "ms",
    "state",
    "flags",
    "alt_m",
    "rel_alt_m",
    "vel_mps",
    "batt_v",
    "gps_fix",
    "sats",
    "lat",
    "lon",
    "gps_alt_m",
    "gps_rel_alt_m",
    "baro_gps_delta_m",
    "gps_speed_mps",
    "roll",
    "pitch",
    "yaw",
]


def finite_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def finite_int(value: str | None) -> int | None:
    parsed = finite_float(value)
    return int(parsed) if parsed is not None else None


def load_rows(path: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for source in reader:
            row: dict[str, Any] = {}
            for field in FIELDS:
                if field in ("state", "flags", "gps_fix", "sats"):
                    row[field] = finite_int(source.get(field))
                else:
                    row[field] = finite_float(source.get(field))
            if row.get("ms") is None or row.get("rel_alt_m") is None:
                continue
            rows.append(row)
    if not rows:
        return rows

    t0 = rows[0]["ms"]
    for row in rows:
        row["t_s"] = (row["ms"] - t0) / 1000.0
        state = row.get("state")
        row["state_name"] = STATE_NAMES.get(state, str(state))
    return rows


def transitions(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    previous = object()
    for idx, row in enumerate(rows, start=2):
        state = row.get("state")
        if state != previous:
            out.append(
                {
                    "line": idx,
                    "t_s": row["t_s"],
                    "ms": row["ms"],
                    "state": state,
                    "state_name": row["state_name"],
                    "rel_alt_m": row.get("rel_alt_m"),
                    "vel_mps": row.get("vel_mps"),
                    "batt_v": row.get("batt_v"),
                    "gps_fix": row.get("gps_fix"),
                    "sats": row.get("sats"),
                }
            )
            previous = state
    return out


def minmax(rows: list[dict[str, Any]], key: str) -> tuple[float | None, float | None]:
    vals = [r[key] for r in rows if isinstance(r.get(key), (int, float)) and math.isfinite(r[key])]
    if not vals:
        return None, None
    return min(vals), max(vals)


def best_row(rows: list[dict[str, Any]], key: str) -> dict[str, Any] | None:
    valid = [r for r in rows if isinstance(r.get(key), (int, float)) and math.isfinite(r[key])]
    if not valid:
        return None
    return max(valid, key=lambda r: r[key])


def render_number(value: Any, digits: int = 2) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return "-"
    return f"{value:.{digits}f}"


def build_report(input_path: str, rows: list[dict[str, Any]]) -> str:
    trans = transitions(rows)
    max_alt = best_row(rows, "rel_alt_m")
    max_vel = best_row(rows, "vel_mps")
    min_batt, max_batt = minmax(rows, "batt_v")
    _, max_sats = minmax(rows, "sats")
    duration_s = rows[-1]["t_s"] if rows else 0.0
    gps_rows = [r for r in rows if r.get("lat") not in (None, 0.0) and r.get("lon") not in (None, 0.0)]

    summary = {
        "source": os.path.basename(input_path),
        "rows": len(rows),
        "duration_s": duration_s,
        "max_alt_m": max_alt,
        "max_vel_mps": max_vel,
        "batt_min_v": min_batt,
        "batt_max_v": max_batt,
        "max_sats": max_sats,
        "gps_rows": len(gps_rows),
        "transitions": trans,
    }

    points = [
        {
            "t": r["t_s"],
            "state": r.get("state"),
            "stateName": r.get("state_name"),
            "relAlt": r.get("rel_alt_m"),
            "vel": r.get("vel_mps"),
            "batt": r.get("batt_v"),
            "gpsFix": r.get("gps_fix"),
            "sats": r.get("sats"),
            "lat": r.get("lat"),
            "lon": r.get("lon"),
            "gpsRelAlt": r.get("gps_rel_alt_m"),
            "baroGpsDelta": r.get("baro_gps_delta_m"),
        }
        for r in rows
    ]

    payload = json.dumps({"summary": summary, "points": points}, separators=(",", ":"))

    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RocketV10 Flight Report - {html.escape(os.path.basename(input_path))}</title>
<style>
:root {{
  color-scheme: light;
  --bg: #f4f6f8;
  --panel: #ffffff;
  --ink: #17212b;
  --muted: #687789;
  --grid: #d7dee6;
  --accent: #1267b1;
  --green: #198754;
  --orange: #b85c00;
  --red: #b42318;
}}
* {{ box-sizing: border-box; }}
body {{
  margin: 0;
  background: var(--bg);
  color: var(--ink);
  font: 14px/1.45 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}}
header {{
  padding: 18px 22px 8px;
  border-bottom: 1px solid #dfe5eb;
  background: #fff;
}}
h1 {{ margin: 0 0 4px; font-size: 22px; }}
.source {{ color: var(--muted); }}
main {{ padding: 18px 22px 28px; max-width: 1320px; margin: 0 auto; }}
.metrics {{
  display: grid;
  grid-template-columns: repeat(6, minmax(120px, 1fr));
  gap: 10px;
  margin-bottom: 16px;
}}
.metric, .panel {{
  background: var(--panel);
  border: 1px solid #dfe5eb;
  border-radius: 8px;
  box-shadow: 0 1px 2px rgba(16,24,40,.04);
}}
.metric {{ padding: 12px; min-height: 74px; }}
.label {{ color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .03em; }}
.value {{ font-size: 23px; font-weight: 700; margin-top: 4px; }}
.unit {{ font-size: 13px; color: var(--muted); font-weight: 500; }}
.grid {{
  display: grid;
  grid-template-columns: minmax(0, 1.3fr) minmax(320px, .7fr);
  gap: 14px;
}}
.panel {{ padding: 14px; margin-bottom: 14px; }}
.panel h2 {{ margin: 0 0 10px; font-size: 16px; }}
svg {{ width: 100%; height: 330px; display: block; }}
.small svg {{ height: 250px; }}
canvas {{
  width: 100%;
  height: 440px;
  display: block;
  background: #f8fafc;
  border: 1px solid #dfe5eb;
  border-radius: 8px;
}}
.controls {{
  display: grid;
  grid-template-columns: auto 1fr auto;
  gap: 10px;
  align-items: center;
  margin-top: 10px;
}}
button {{
  border: 1px solid #b8c3cf;
  background: #fff;
  color: var(--ink);
  border-radius: 6px;
  padding: 7px 11px;
  font-weight: 650;
  cursor: pointer;
}}
input[type="range"] {{ width: 100%; }}
.readout {{
  font-variant-numeric: tabular-nums;
  color: var(--muted);
  min-width: 190px;
  text-align: right;
}}
table {{ width: 100%; border-collapse: collapse; }}
th, td {{ padding: 6px 7px; border-bottom: 1px solid #edf1f5; text-align: right; white-space: nowrap; }}
th:first-child, td:first-child {{ text-align: left; }}
th {{ color: var(--muted); font-weight: 600; font-size: 12px; }}
.badge {{ display: inline-block; padding: 2px 7px; border-radius: 999px; background: #eef4fb; color: #164c7e; font-weight: 650; }}
.note {{ color: var(--muted); margin: 8px 0 0; }}
.tooltip {{
  position: fixed;
  pointer-events: none;
  background: #17212b;
  color: #fff;
  padding: 8px 9px;
  border-radius: 6px;
  font-size: 12px;
  opacity: 0;
  transform: translate(10px, 10px);
  white-space: pre;
}}
@media (max-width: 900px) {{
  .metrics {{ grid-template-columns: repeat(2, 1fr); }}
  .grid {{ grid-template-columns: 1fr; }}
}}
</style>
</head>
<body>
<header>
  <h1>RocketV10 Flight Report</h1>
  <div class="source">{html.escape(input_path)}</div>
</header>
<main>
  <section class="metrics" id="metrics"></section>
  <section class="panel">
    <h2>Rocket Position And Attitude</h2>
    <canvas id="rocketScene" width="1160" height="440"></canvas>
    <div class="controls">
      <button id="playBtn" type="button">Play</button>
      <input id="timeSlider" type="range" min="0" max="0" value="0" step="1">
      <div class="readout" id="sceneReadout"></div>
    </div>
    <p class="note">The side view uses barometric relative altitude. The top view uses GPS when available. The rocket drawing uses logged roll, pitch, and yaw, so it is an attitude visualization, not a physics simulation.</p>
  </section>
  <section class="grid">
    <div>
      <div class="panel">
        <h2>Altitude And Velocity</h2>
        <svg id="altVel"></svg>
        <p class="note">Barometric relative altitude and vertical velocity from the clean NAND flight export.</p>
      </div>
      <div class="panel small">
        <h2>Battery And GPS Satellites</h2>
        <svg id="health"></svg>
      </div>
    </div>
    <div>
      <div class="panel small">
        <h2>GPS Track</h2>
        <svg id="track"></svg>
        <p class="note" id="trackNote"></p>
      </div>
      <div class="panel">
        <h2>State Transitions</h2>
        <table id="transitions"></table>
      </div>
    </div>
  </section>
</main>
<div class="tooltip" id="tooltip"></div>
<script>
const DATA = {payload};
const stateColors = {{
  1: "#667085",
  2: "#198754",
  3: "#b85c00",
  4: "#1267b1",
  5: "#6f42c1",
  6: "#b42318"
}};
const fmt = (v, d=1) => Number.isFinite(v) ? v.toFixed(d) : "-";
const points = DATA.points;
const summary = DATA.summary;
let sceneSamples = [];
let sceneIndex = 0;
let playing = false;
let lastFrameTs = 0;

function setMetrics() {{
  const maxAlt = summary.max_alt_m || {{}};
  const maxVel = summary.max_vel_mps || {{}};
  const items = [
    ["Max altitude", fmt(maxAlt.rel_alt_m, 1), "m AGL"],
    ["Max velocity", fmt(maxVel.vel_mps, 1), "m/s"],
    ["Duration", fmt(summary.duration_s, 1), "s"],
    ["Rows", String(summary.rows), ""],
    ["Battery", `${{fmt(summary.batt_min_v, 3)}}-${{fmt(summary.batt_max_v, 3)}}`, "V"],
    ["GPS sats", fmt(summary.max_sats, 0), "max"]
  ];
  document.getElementById("metrics").innerHTML = items.map(([label, value, unit]) => `
    <div class="metric"><div class="label">${{label}}</div><div class="value">${{value}} <span class="unit">${{unit}}</span></div></div>
  `).join("");
}}

function domain(vals, pad=0.08) {{
  const finite = vals.filter(Number.isFinite);
  let lo = Math.min(...finite), hi = Math.max(...finite);
  if (lo === hi) {{ lo -= 1; hi += 1; }}
  const p = (hi - lo) * pad;
  return [lo - p, hi + p];
}}

function linePath(data, x, y, key) {{
  let d = "";
  let open = false;
  data.forEach(p => {{
    const v = p[key];
    if (!Number.isFinite(v)) {{ open = false; return; }}
    d += `${{open ? "L" : "M"}}${{x(p.t).toFixed(2)}},${{y(v).toFixed(2)}}`;
    open = true;
  }});
  return d;
}}

function drawChart(svgId, series) {{
  const svg = document.getElementById(svgId);
  const w = svg.clientWidth || 800;
  const h = svg.clientHeight || 320;
  const m = {{l: 52, r: 18, t: 14, b: 34}};
  const innerW = w - m.l - m.r;
  const innerH = h - m.t - m.b;
  const xDom = domain(points.map(p => p.t), 0.02);
  const yDom = domain(series.flatMap(s => points.map(p => p[s.key])), 0.1);
  const x = v => m.l + (v - xDom[0]) / (xDom[1] - xDom[0]) * innerW;
  const y = v => m.t + innerH - (v - yDom[0]) / (yDom[1] - yDom[0]) * innerH;
  const grid = [];
  for (let i=0; i<=5; i++) {{
    const yy = m.t + i * innerH / 5;
    const val = yDom[1] - i * (yDom[1] - yDom[0]) / 5;
    grid.push(`<line x1="${{m.l}}" y1="${{yy}}" x2="${{w-m.r}}" y2="${{yy}}" stroke="var(--grid)" />`);
    grid.push(`<text x="${{m.l-8}}" y="${{yy+4}}" text-anchor="end" fill="var(--muted)" font-size="11">${{fmt(val, 1)}}</text>`);
  }}
  for (let i=0; i<=5; i++) {{
    const xx = m.l + i * innerW / 5;
    const val = xDom[0] + i * (xDom[1] - xDom[0]) / 5;
    grid.push(`<line x1="${{xx}}" y1="${{m.t}}" x2="${{xx}}" y2="${{h-m.b}}" stroke="#eef2f6" />`);
    grid.push(`<text x="${{xx}}" y="${{h-10}}" text-anchor="middle" fill="var(--muted)" font-size="11">${{fmt(val, 0)}}s</text>`);
  }}
  const paths = series.map(s => `<path d="${{linePath(points, x, y, s.key)}}" fill="none" stroke="${{s.color}}" stroke-width="2.2" />`).join("");
  const trans = summary.transitions.map(t => {{
    const xx = x(t.t_s);
    const color = stateColors[t.state] || "#444";
    return `<line x1="${{xx}}" y1="${{m.t}}" x2="${{xx}}" y2="${{h-m.b}}" stroke="${{color}}" stroke-dasharray="4 4" opacity=".7" />
            <text x="${{xx+4}}" y="${{m.t+14}}" fill="${{color}}" font-size="11">${{t.state_name}}</text>`;
  }}).join("");
  const legend = series.map((s,i) => `<g transform="translate(${{m.l + i*120}},${{m.t+10}})"><line x1="0" y1="0" x2="20" y2="0" stroke="${{s.color}}" stroke-width="3"/><text x="26" y="4" font-size="12" fill="var(--ink)">${{s.label}}</text></g>`).join("");
  svg.setAttribute("viewBox", `0 0 ${{w}} ${{h}}`);
  svg.innerHTML = `<rect width="${{w}}" height="${{h}}" fill="#fff"/>${{grid.join("")}}${{trans}}${{paths}}${{legend}}`;
}}

function drawTrack() {{
  const svg = document.getElementById("track");
  const gps = points.filter(p => p.lat && p.lon);
  const note = document.getElementById("trackNote");
  if (gps.length < 2) {{
    svg.innerHTML = "";
    note.textContent = "Not enough GPS coordinates for a track.";
    return;
  }}
  const w = svg.clientWidth || 420, h = svg.clientHeight || 250;
  const m = {{l: 22, r: 22, t: 18, b: 24}};
  const latDom = domain(gps.map(p => p.lat), 0.15);
  const lonDom = domain(gps.map(p => p.lon), 0.15);
  const x = lon => m.l + (lon - lonDom[0]) / (lonDom[1] - lonDom[0]) * (w-m.l-m.r);
  const y = lat => m.t + (latDom[1] - lat) / (latDom[1] - latDom[0]) * (h-m.t-m.b);
  const d = gps.map((p,i) => `${{i ? "L" : "M"}}${{x(p.lon).toFixed(2)}},${{y(p.lat).toFixed(2)}}`).join("");
  const start = gps[0], end = gps[gps.length-1];
  svg.setAttribute("viewBox", `0 0 ${{w}} ${{h}}`);
  svg.innerHTML = `<rect width="${{w}}" height="${{h}}" fill="#fff"/>
    <path d="${{d}}" fill="none" stroke="#1267b1" stroke-width="2"/>
    <circle cx="${{x(start.lon)}}" cy="${{y(start.lat)}}" r="5" fill="#198754"/>
    <circle cx="${{x(end.lon)}}" cy="${{y(end.lat)}}" r="5" fill="#b42318"/>
    <text x="${{m.l}}" y="${{h-7}}" fill="var(--muted)" font-size="11">green=start, red=end</text>`;
  note.textContent = `${{gps.length}} GPS rows. Lat ${{fmt(latDom[0], 6)}}..${{fmt(latDom[1], 6)}}, lon ${{fmt(lonDom[0], 6)}}..${{fmt(lonDom[1], 6)}}.`;
}}

function setTransitions() {{
  const rows = summary.transitions.map(t => `
    <tr>
      <td><span class="badge">${{t.state_name}}</span></td>
      <td>${{fmt(t.t_s, 1)}}s</td>
      <td>${{fmt(t.rel_alt_m, 1)}}m</td>
      <td>${{fmt(t.vel_mps, 1)}}m/s</td>
      <td>${{fmt(t.batt_v, 3)}}V</td>
      <td>${{t.gps_fix ?? "-"}}/${{t.sats ?? "-"}}</td>
    </tr>`).join("");
  document.getElementById("transitions").innerHTML = `<thead><tr><th>State</th><th>Time</th><th>Alt</th><th>Vel</th><th>Batt</th><th>GPS</th></tr></thead><tbody>${{rows}}</tbody>`;
}}

function metersFromGps(data) {{
  const gps = data.filter(p => Number.isFinite(p.lat) && Number.isFinite(p.lon) && p.lat !== 0 && p.lon !== 0);
  if (gps.length < 2) return data.map(p => ({{...p, east: 0, north: 0}}));
  const lat0 = gps[0].lat * Math.PI / 180;
  const lon0 = gps[0].lon * Math.PI / 180;
  const cosLat = Math.cos(lat0);
  return data.map(p => {{
    if (!Number.isFinite(p.lat) || !Number.isFinite(p.lon) || p.lat === 0 || p.lon === 0) {{
      return {{...p, east: null, north: null}};
    }}
    const lat = p.lat * Math.PI / 180;
    const lon = p.lon * Math.PI / 180;
    return {{
      ...p,
      east: (lon - lon0) * cosLat * 6371000,
      north: (lat - lat0) * 6371000
    }};
  }});
}}

function prepareSceneSamples() {{
  const withGps = metersFromGps(points);
  sceneSamples = withGps.filter((p, i) => i % 2 === 0 || p.state !== points[Math.max(0, i-1)].state);
  if (!sceneSamples.length) sceneSamples = withGps;
  const slider = document.getElementById("timeSlider");
  slider.max = String(Math.max(0, sceneSamples.length - 1));
  slider.addEventListener("input", () => {{
    sceneIndex = Number(slider.value);
    drawRocketScene();
  }});
  document.getElementById("playBtn").addEventListener("click", () => {{
    playing = !playing;
    document.getElementById("playBtn").textContent = playing ? "Pause" : "Play";
    requestAnimationFrame(animateScene);
  }});
}}

function scaleFor(vals, fallback=1) {{
  const finite = vals.filter(Number.isFinite);
  if (!finite.length) return [-fallback, fallback];
  let lo = Math.min(...finite), hi = Math.max(...finite);
  if (lo === hi) {{ lo -= fallback; hi += fallback; }}
  const pad = (hi - lo) * 0.12;
  return [lo - pad, hi + pad];
}}

function drawRocketIcon(ctx, cx, cy, len, angle, roll, color) {{
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(angle);
  const bodyW = len * 0.17;
  ctx.fillStyle = "#f8fafc";
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(len * 0.48, 0);
  ctx.lineTo(len * 0.25, -bodyW * 0.9);
  ctx.lineTo(-len * 0.35, -bodyW * 0.9);
  ctx.lineTo(-len * 0.48, -bodyW * 1.55);
  ctx.lineTo(-len * 0.32, 0);
  ctx.lineTo(-len * 0.48, bodyW * 1.55);
  ctx.lineTo(-len * 0.35, bodyW * 0.9);
  ctx.lineTo(len * 0.25, bodyW * 0.9);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.strokeStyle = "#b42318";
  ctx.beginPath();
  const fin = Math.sin((roll || 0) * Math.PI / 180) * bodyW * 0.8;
  ctx.moveTo(-len * 0.15, -bodyW * 0.9);
  ctx.lineTo(-len * 0.15, bodyW * 0.9);
  ctx.moveTo(-len * 0.30, fin);
  ctx.lineTo(-len * 0.44, fin);
  ctx.stroke();
  ctx.restore();
}}

function drawRocketScene() {{
  const canvas = document.getElementById("rocketScene");
  const ctx = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (!sceneSamples.length) return;

  const p = sceneSamples[Math.max(0, Math.min(sceneIndex, sceneSamples.length - 1))];
  const relDom = scaleFor(sceneSamples.map(x => x.relAlt), 10);
  const eastDom = scaleFor(sceneSamples.map(x => x.east), 20);
  const northDom = scaleFor(sceneSamples.map(x => x.north), 20);
  const side = {{x: 38, y: 36, w: 520, h: 350}};
  const top = {{x: 610, y: 36, w: 500, h: 350}};

  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, w, h);
  ctx.fillStyle = "#17212b";
  ctx.font = "600 15px system-ui";
  ctx.fillText("Side view: range/time vs altitude", side.x, 24);
  ctx.fillText("Top view: GPS ground track", top.x, 24);
  ctx.strokeStyle = "#d7dee6";
  ctx.strokeRect(side.x, side.y, side.w, side.h);
  ctx.strokeRect(top.x, top.y, top.w, top.h);

  const sx = t => side.x + (t / summary.duration_s) * side.w;
  const sy = alt => side.y + side.h - (alt - relDom[0]) / (relDom[1] - relDom[0]) * side.h;
  const tx = east => top.x + (east - eastDom[0]) / (eastDom[1] - eastDom[0]) * top.w;
  const ty = north => top.y + top.h - (north - northDom[0]) / (northDom[1] - northDom[0]) * top.h;

  ctx.strokeStyle = "#edf1f5";
  ctx.lineWidth = 1;
  for (let i=1; i<5; i++) {{
    ctx.beginPath();
    ctx.moveTo(side.x, side.y + i*side.h/5);
    ctx.lineTo(side.x + side.w, side.y + i*side.h/5);
    ctx.moveTo(top.x, top.y + i*top.h/5);
    ctx.lineTo(top.x + top.w, top.y + i*top.h/5);
    ctx.stroke();
  }}

  ctx.strokeStyle = "#1267b1";
  ctx.lineWidth = 2;
  ctx.beginPath();
  sceneSamples.forEach((s, i) => {{
    const x = sx(s.t), y = sy(s.relAlt || 0);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }});
  ctx.stroke();

  const gps = sceneSamples.filter(s => Number.isFinite(s.east) && Number.isFinite(s.north));
  ctx.strokeStyle = "#198754";
  ctx.beginPath();
  gps.forEach((s, i) => {{
    const x = tx(s.east), y = ty(s.north);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }});
  ctx.stroke();

  const sideX = sx(p.t);
  const sideY = sy(p.relAlt || 0);
  const topX = Number.isFinite(p.east) ? tx(p.east) : top.x + top.w / 2;
  const topY = Number.isFinite(p.north) ? ty(p.north) : top.y + top.h / 2;
  const color = stateColors[p.state] || "#1267b1";
  const pitchAngle = -((p.pitch || 0) * Math.PI / 180) - Math.PI / 2;
  const yawAngle = ((p.yaw || 0) * Math.PI / 180) - Math.PI / 2;
  drawRocketIcon(ctx, sideX, sideY, 56, pitchAngle, p.roll || 0, color);
  drawRocketIcon(ctx, topX, topY, 54, yawAngle, p.roll || 0, color);

  ctx.fillStyle = "#17212b";
  ctx.font = "12px system-ui";
  ctx.fillText(`${{fmt(relDom[1],0)}} m`, side.x + 6, side.y + 16);
  ctx.fillText(`${{fmt(relDom[0],0)}} m`, side.x + 6, side.y + side.h - 8);
  ctx.fillText("N", top.x + top.w - 18, top.y + 16);

  document.getElementById("timeSlider").value = String(sceneIndex);
  document.getElementById("sceneReadout").textContent =
    `${{fmt(p.t,1)}}s  ${{p.stateName}}  alt ${{fmt(p.relAlt,1)}}m  roll/pitch/yaw ${{fmt(p.roll,0)}}/${{fmt(p.pitch,0)}}/${{fmt(p.yaw,0)}}`;
}}

function animateScene(ts) {{
  if (!playing) return;
  if (!lastFrameTs || ts - lastFrameTs > 45) {{
    sceneIndex = (sceneIndex + 1) % sceneSamples.length;
    drawRocketScene();
    lastFrameTs = ts;
  }}
  requestAnimationFrame(animateScene);
}}

setMetrics();
prepareSceneSamples();
drawRocketScene();
drawChart("altVel", [
  {{key: "relAlt", label: "rel alt m", color: "#1267b1"}},
  {{key: "vel", label: "vel m/s", color: "#b85c00"}}
]);
drawChart("health", [
  {{key: "batt", label: "battery V", color: "#198754"}},
  {{key: "sats", label: "GPS sats", color: "#6f42c1"}}
]);
drawTrack();
setTransitions();
</script>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv")
    parser.add_argument("-o", "--output", help="Output HTML path")
    args = parser.parse_args()

    input_path = os.path.abspath(args.input_csv)
    output_path = args.output
    if not output_path:
        root, _ = os.path.splitext(input_path)
        output_path = root + "_report.html"

    rows = load_rows(input_path)
    if not rows:
        raise SystemExit(f"No usable rows in {input_path}")

    report = build_report(input_path, rows)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(report)

    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
