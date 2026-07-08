#!/usr/bin/env python3
"""
Live RocketV10 serial visualizer.

Uses only the Python standard library. It reads current RocketV10 `dbg ...`
serial lines, serves a small browser UI, and streams parsed telemetry by SSE.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import re
import select
import socketserver
import sys
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


DEFAULT_PORT = "/dev/cu.usbmodem184901201"
DEFAULT_BAUD = 115200
DEFAULT_HTTP_PORT = 8765

FLOAT_KEYS = {
    "alt", "relAlt", "maxAlt", "apogeeAlt", "vel", "tempC", "presPa",
    "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz",
    "rollDeg", "pitchDeg", "yawDeg", "hdop", "lat", "lon", "gpsAlt",
    "gpsRelAlt", "gpsBaseAlt", "baroGpsDelta", "gpsSpd", "battRawV",
    "batt", "battPin", "rssi",
}

INT_KEYS = {
    "ms", "state", "flags", "gpsFix", "sats", "gpsChars", "gpsPass",
    "gpsFail", "gpsLocValid", "gpsAltValid", "gpsDateValid", "gpsTimeValid",
    "gpsFixAgeMs", "battRaw", "health", "fix",
}

BOOL_KEYS = {
    "baroGpsDiv", "bwarn", "bcrit", "lora", "baro", "imu", "gps",
    "gpsFresh", "sd", "sdlog", "nlog", "log", "nand", "imuFresh",
    "baroFresh", "logFinal",
}

STATE_NAMES = {
    0: "IDLE",
    1: "PAD",
    2: "ASCENT",
    3: "COAST",
    4: "DESCENT",
    5: "LANDED",
    6: "ABORT",
}

STATE_IDS = {name: value for value, name in STATE_NAMES.items()}
GROUND_PREFIXES = {"FLIGHT", "NAV", "STATUS"}


def parse_dbg_line(line: str) -> dict | None:
    line = line.strip()
    if not line.startswith("dbg "):
        return None

    data: dict[str, object] = {"source": "rocket", "packetType": "dbg", "raw": line, "hostTime": time.time()}
    for key, value in re.findall(r"([A-Za-z][A-Za-z0-9_]*)=([^ ]+)", line):
        if value == "nan":
            parsed: object = None
        elif key in BOOL_KEYS:
            parsed = value == "1"
        elif key in INT_KEYS:
            try:
                parsed = int(float(value))
            except ValueError:
                parsed = value
        elif key in FLOAT_KEYS:
            try:
                parsed = float(value)
            except ValueError:
                parsed = value
        else:
            parsed = value
        data[key] = parsed

    state = data.get("state")
    if isinstance(state, int):
        data["stateName"] = STATE_NAMES.get(state, str(state))
    return data


def parse_ground_line(line: str) -> dict | None:
    line = line.strip()
    if " " not in line:
        return None
    prefix, payload = line.split(" ", 1)
    if prefix not in GROUND_PREFIXES:
        return None

    host_time = time.time()
    data: dict[str, object] = {
        "source": "ground",
        "packetType": prefix.lower(),
        "raw": line,
        "hostTime": host_time,
        "ms": int(host_time * 1000),
    }
    for key, value in re.findall(r"([A-Za-z][A-Za-z0-9_]*)=([^ ]+)", payload):
        if value == "nan":
            parsed: object = None
        elif key == "state" and not re.fullmatch(r"-?\d+(\.\d+)?", value):
            parsed = value
        elif key in BOOL_KEYS:
            parsed = value == "1"
        elif key in INT_KEYS:
            try:
                parsed = int(float(value))
            except ValueError:
                parsed = value
        elif key in FLOAT_KEYS:
            try:
                parsed = float(value)
            except ValueError:
                parsed = value
        else:
            parsed = value
        data[key] = parsed

    if prefix == "FLIGHT":
        state = data.get("state")
        if isinstance(state, str):
            data["stateName"] = state
            data["state"] = STATE_IDS.get(state, state)
        elif isinstance(state, int):
            data["stateName"] = STATE_NAMES.get(state, str(state))
        if "alt" in data and "relAlt" not in data:
            data["relAlt"] = data["alt"]
    elif prefix == "NAV":
        if "fix" in data:
            data["gpsFix"] = data["fix"]
        data["gpsFresh"] = bool(data.get("gpsFix"))
        data["gps"] = bool(data.get("gpsFix"))
    elif prefix == "STATUS":
        state = data.get("state")
        if isinstance(state, str):
            data["stateName"] = state
            data["state"] = STATE_IDS.get(state, state)
        elif isinstance(state, int):
            data["stateName"] = STATE_NAMES.get(state, str(state))
        if "baro" in data:
            data["baroFresh"] = data["baro"]
        if "imu" in data:
            data["imuFresh"] = data["imu"]
        if "gps" in data:
            data["gpsFresh"] = data["gps"]
    return data


def parse_serial_line(line: str) -> dict | None:
    return parse_dbg_line(line) or parse_ground_line(line)


class Broadcaster:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clients: list[queue.Queue[dict]] = []
        self.latest: dict | None = None

    def add_client(self) -> queue.Queue[dict]:
        q: queue.Queue[dict] = queue.Queue(maxsize=20)
        with self._lock:
            self._clients.append(q)
            if self.latest is not None:
                q.put_nowait(self.latest)
        return q

    def remove_client(self, q: queue.Queue[dict]) -> None:
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)

    def publish(self, data: dict) -> None:
        with self._lock:
            self.latest = data
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(data)
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait(data)
                except queue.Empty:
                    pass


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, broadcaster: Broadcaster) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.broadcaster = broadcaster
        self.stop_event = threading.Event()
        self.lines_seen = 0
        self.packets_seen = 0

    def _open_port(self) -> int:
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        baud_const = getattr(termios, f"B{self.baud}")
        attrs[4] = baud_const
        attrs[5] = baud_const
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~termios.PARENB
        attrs[2] &= ~termios.CSTOPB
        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8
        attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                fd = self._open_port()
                print(f"Opened serial {self.port} at {self.baud}", flush=True)
                self._read_loop(fd)
            except FileNotFoundError:
                print(f"Serial port not found: {self.port}; retrying", flush=True)
                time.sleep(1.0)
            except Exception as exc:
                print(f"Serial reader error: {exc}; retrying", flush=True)
                time.sleep(1.0)
            finally:
                try:
                    os.close(fd)  # type: ignore[name-defined]
                except Exception:
                    pass

    def _read_loop(self, fd: int) -> None:
        buf = b""
        while not self.stop_event.is_set():
            ready, _, _ = select.select([fd], [], [], 0.25)
            if not ready:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw_line, buf = buf.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="ignore").strip()
                self.lines_seen += 1
                data = parse_serial_line(line)
                if data is not None:
                    self.packets_seen += 1
                    self.broadcaster.publish(data)


HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>RocketV10 Live Visualizer</title>
  <style>
    :root {
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
    }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font: 14px/1.35 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    .layout {
      display: grid;
      grid-template-columns: minmax(420px, 1fr) 390px;
      min-height: 100vh;
    }
    .stage {
      position: relative;
      min-height: 620px;
      background: linear-gradient(#121920, #0f1419);
    }
    canvas {
      width: 100%;
      height: 100%;
      display: block;
    }
    .side {
      background: var(--panel);
      border-left: 1px solid #26313a;
      padding: 16px;
      overflow: auto;
    }
    h1 {
      font-size: 18px;
      margin: 0 0 14px;
      font-weight: 650;
    }
    .status {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 14px;
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: var(--bad);
      box-shadow: 0 0 12px currentColor;
    }
    .dot.live { background: var(--ok); color: var(--ok); }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-bottom: 14px;
    }
    .metric {
      background: #11171c;
      border: 1px solid #26313a;
      border-radius: 6px;
      padding: 8px;
      min-height: 46px;
    }
    .label {
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: .04em;
    }
    .value {
      font-size: 18px;
      margin-top: 3px;
      white-space: nowrap;
    }
    .wide { grid-column: 1 / -1; }
    .chart {
      height: 150px;
      margin: 10px 0 14px;
      background: #10161b;
      border: 1px solid #26313a;
      border-radius: 6px;
    }
    .flags {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 6px;
      margin-bottom: 14px;
    }
    .flag {
      text-align: center;
      padding: 6px 4px;
      background: #10161b;
      color: var(--muted);
      border: 1px solid #26313a;
      border-radius: 5px;
      font-size: 12px;
    }
    .flag.on { color: var(--ok); border-color: #316943; }
    .raw {
      color: var(--muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 11px;
      overflow-wrap: anywhere;
      background: #0d1216;
      border: 1px solid #26313a;
      border-radius: 6px;
      padding: 8px;
    }
    @media (max-width: 860px) {
      .layout { grid-template-columns: 1fr; }
      .side { border-left: 0; border-top: 1px solid #26313a; }
      .stage { min-height: 460px; }
    }
  </style>
</head>
<body>
  <div class="layout">
    <div class="stage">
      <canvas id="rocket"></canvas>
    </div>
    <aside class="side">
      <h1>RocketV10 Live</h1>
      <div class="status"><span id="dot" class="dot"></span><span id="link">waiting for serial data</span></div>
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
      </div>
      <div class="flags">
        <div id="fGps" class="flag">GPS</div>
        <div id="fBaro" class="flag">BARO</div>
        <div id="fImu" class="flag">IMU</div>
        <div id="fLog" class="flag">LOG</div>
      </div>
      <canvas id="altChart" class="chart"></canvas>
      <canvas id="velChart" class="chart"></canvas>
      <div id="raw" class="raw">No data yet.</div>
    </aside>
  </div>

<script>
const rocketCanvas = document.getElementById('rocket');
const rocketCtx = rocketCanvas.getContext('2d');
const altCanvas = document.getElementById('altChart');
const velCanvas = document.getElementById('velChart');
const altCtx = altCanvas.getContext('2d');
const velCtx = velCanvas.getContext('2d');
const MOUNT_ROLL_OFFSET_DEG = -180;
const MOUNT_PITCH_OFFSET_DEG = 0;
const MOUNT_YAW_OFFSET_DEG = 0;
let latest = {};
let history = [];
let lastPacketAt = 0;

function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function deg(v) { return Number.isFinite(v) ? `${v.toFixed(1)} deg` : '-'; }
function meters(v) { return Number.isFinite(v) ? `${v.toFixed(2)} m` : '-'; }
function mps(v) { return Number.isFinite(v) ? `${v.toFixed(2)} m/s` : '-'; }
function wrapDeg(v) {
  if (!Number.isFinite(v)) return v;
  let out = ((v + 180) % 360 + 360) % 360 - 180;
  return out === -180 ? 180 : out;
}
function correctedAttitude(d) {
  return {
    roll: wrapDeg((d.rollDeg || 0) + MOUNT_ROLL_OFFSET_DEG),
    pitch: wrapDeg((d.pitchDeg || 0) + MOUNT_PITCH_OFFSET_DEG),
    yaw: wrapDeg((d.yawDeg || 0) + MOUNT_YAW_OFFSET_DEG),
  };
}

function setText(id, text) { document.getElementById(id).textContent = text; }
function setFlag(id, on) { document.getElementById(id).classList.toggle('on', !!on); }

function updateMetrics(d) {
  const att = correctedAttitude(d);
  setText('state', d.stateName ?? '-');
  setText('batt', Number.isFinite(d.batt) ? `${d.batt.toFixed(2)} V${d.bwarn ? ' WARN' : ''}` : '-');
  setText('relAlt', meters(d.relAlt));
  setText('vel', mps(d.vel));
  setText('roll', deg(att.roll));
  setText('pitch', deg(att.pitch));
  setText('yaw', deg(att.yaw));
  setText('gps', `${d.gpsFix ?? '-'}D ${d.sats ?? '-'} sats`);
  const hasPos = Number.isFinite(d.lat) && Number.isFinite(d.lon) && Math.abs(d.lat) > 0.001;
  setText('pos', hasPos ? `${d.lat.toFixed(7)}, ${d.lon.toFixed(7)}` : '-');
  setFlag('fGps', d.gpsFresh && d.gps);
  setFlag('fBaro', d.baroFresh && d.baro);
  setFlag('fImu', d.imuFresh && d.imu);
  setFlag('fLog', d.log);
  document.getElementById('raw').textContent = d.raw || '';
}

function rotPoint(p, rollDeg, pitchDeg, yawDeg) {
  const r = (rollDeg || 0) * Math.PI / 180;
  const pch = (pitchDeg || 0) * Math.PI / 180;
  const y = (yawDeg || 0) * Math.PI / 180;
  let [x, yy, z] = p;
  let c = Math.cos(r), s = Math.sin(r);
  [yy, z] = [yy * c - z * s, yy * s + z * c];
  c = Math.cos(pch); s = Math.sin(pch);
  [x, z] = [x * c + z * s, -x * s + z * c];
  c = Math.cos(y); s = Math.sin(y);
  [x, yy] = [x * c - yy * s, x * s + yy * c];
  return [x, yy, z];
}

function project(p, w, h) {
  const dist = 4.2;
  const scale = Math.min(w, h) * 0.34;
  const f = scale / (dist - p[1]);
  return [w * 0.5 + p[0] * f, h * 0.53 - p[2] * f];
}

function drawLine(ctx, pts, color, width, closed=false) {
  if (!pts.length) return;
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
  if (closed) ctx.closePath();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
}

function drawPoly(ctx, pts, stroke, fill) {
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

function drawRocket() {
  fitCanvas(rocketCanvas);
  const rect = rocketCanvas.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  rocketCtx.clearRect(0, 0, w, h);
  rocketCtx.fillStyle = '#101418';
  rocketCtx.fillRect(0, 0, w, h);
  rocketCtx.strokeStyle = '#25323b';
  rocketCtx.lineWidth = 1;
  for (let x = 0; x < w; x += 40) { rocketCtx.beginPath(); rocketCtx.moveTo(x,0); rocketCtx.lineTo(x,h); rocketCtx.stroke(); }
  for (let y = 0; y < h; y += 40) { rocketCtx.beginPath(); rocketCtx.moveTo(0,y); rocketCtx.lineTo(w,y); rocketCtx.stroke(); }

  const att = correctedAttitude(latest);
  const roll = att.roll, pitch = att.pitch, yaw = att.yaw;
  const sides = 12, radius = 0.16, tail = -0.9, shoulder = 0.58, nose = 0.95;
  const rings = [tail, shoulder].map(z => Array.from({length:sides}, (_, i) => {
    const a = i * Math.PI * 2 / sides;
    return [Math.cos(a)*radius, Math.sin(a)*radius, z];
  }));
  const nosePt = [0, 0, nose];
  const toScreen = p => project(rotPoint(p, roll, pitch, yaw), w, h);
  const r0 = rings[0].map(toScreen), r1 = rings[1].map(toScreen), np = toScreen(nosePt);

  for (let i = 0; i < sides; i++) {
    drawLine(rocketCtx, [r0[i], r1[i]], '#6fb6ff', 1.6);
    drawLine(rocketCtx, [r1[i], np], '#dce9f5', 1.2);
  }
  drawLine(rocketCtx, r0, '#6fb6ff', 2, true);
  drawLine(rocketCtx, r1, '#6fb6ff', 2, true);

  for (let i = 0; i < 4; i++) {
    const a = i * Math.PI / 2 + Math.PI / 4;
    const b = [Math.cos(a)*radius, Math.sin(a)*radius, tail + 0.18];
    const c = [Math.cos(a)*0.44, Math.sin(a)*0.44, tail - 0.04];
    const d = [Math.cos(a)*radius, Math.sin(a)*radius, tail + 0.48];
    drawPoly(rocketCtx, [toScreen(b), toScreen(c), toScreen(d)], '#ffcf6f', 'rgba(230,187,74,.22)');
  }
  drawLine(rocketCtx, [toScreen([0,0,tail]), toScreen([0,0,nose])], '#ffffff', 3.5);

  rocketCtx.fillStyle = '#e8edf2';
  rocketCtx.font = '14px system-ui';
  rocketCtx.fillText(`roll ${roll.toFixed(1)}  pitch ${pitch.toFixed(1)}  yaw ${yaw.toFixed(1)}`, 18, 28);
  rocketCtx.fillStyle = '#93a4b2';
  rocketCtx.fillText(`state ${latest.stateName || '-'}  relAlt ${Number.isFinite(latest.relAlt) ? latest.relAlt.toFixed(2) : '-'} m`, 18, 50);
}

function drawChart(canvas, ctx, key, label, color) {
  fitCanvas(canvas);
  const rect = canvas.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#10161b';
  ctx.fillRect(0,0,w,h);
  ctx.strokeStyle = '#31404a';
  ctx.lineWidth = 1;
  for (let i=1; i<4; i++) { const y=h*i/4; ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke(); }
  const vals = history.map(d => d[key]).filter(Number.isFinite);
  ctx.fillStyle = '#93a4b2';
  ctx.font = '12px system-ui';
  ctx.fillText(label, 10, 18);
  if (vals.length < 2) return;
  let min = Math.min(...vals), max = Math.max(...vals);
  if (Math.abs(max-min) < 0.01) { min -= 1; max += 1; }
  const firstMs = history[0].ms || 0;
  const lastMs = history[history.length - 1].ms || firstMs + 1;
  ctx.beginPath();
  history.forEach((d, i) => {
    if (!Number.isFinite(d[key])) return;
    const x = ((d.ms - firstMs) / Math.max(1, lastMs - firstMs)) * (w - 22) + 10;
    const y = h - 18 - ((d[key] - min) / (max - min)) * (h - 36);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.fillStyle = '#e8edf2';
  ctx.fillText(`${vals[vals.length-1].toFixed(2)}  min ${min.toFixed(2)} max ${max.toFixed(2)}`, 10, h - 8);
}

function frame() {
  const live = Date.now() - lastPacketAt < 2500;
  document.getElementById('dot').classList.toggle('live', live);
  document.getElementById('link').textContent = live ? `serial live (${latest.source || 'unknown'})` : 'waiting for serial data';
  drawRocket();
  drawChart(altCanvas, altCtx, 'relAlt', 'Relative altitude (m)', '#73b7ff');
  drawChart(velCanvas, velCtx, 'vel', 'Vertical velocity (m/s)', '#e6bb4a');
  requestAnimationFrame(frame);
}

const es = new EventSource('/events');
es.onmessage = (ev) => {
  const incoming = JSON.parse(ev.data);
  latest = {...latest, ...incoming};
  lastPacketAt = Date.now();
  history.push({...latest});
  if (history.length > 500) history.shift();
  updateMetrics(latest);
};
es.onerror = () => { document.getElementById('link').textContent = 'event stream disconnected'; };
window.addEventListener('resize', () => { fitCanvas(rocketCanvas); fitCanvas(altCanvas); fitCanvas(velCanvas); });
requestAnimationFrame(frame);
</script>
</body>
</html>
"""


def make_handler(broadcaster: Broadcaster):
    class Handler(BaseHTTPRequestHandler):
        def handle(self) -> None:
            try:
                super().handle()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def log_message(self, fmt: str, *args: object) -> None:
            sys.stdout.write(f"{self.address_string()} - {fmt % args}\n")

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                body = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                q = broadcaster.add_client()
                try:
                    while True:
                        try:
                            data = q.get(timeout=10.0)
                            payload = json.dumps(data, separators=(",", ":"))
                            self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        except queue.Empty:
                            self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    broadcaster.remove_client(q)
                return
            self.send_error(404)
    return Handler


def probe_serial(port: str, baud: int, seconds: float) -> int:
    broadcaster = Broadcaster()
    reader = SerialReader(port, baud, broadcaster)
    reader.start()
    deadline = time.time() + seconds
    last_count = 0
    try:
        while time.time() < deadline:
            if broadcaster.latest is not None and reader.packets_seen != last_count:
                last_count = reader.packets_seen
                d = broadcaster.latest
                print(
                    f"parsed state={d.get('stateName')} relAlt={d.get('relAlt')} "
                    f"roll={d.get('rollDeg')} pitch={d.get('pitchDeg')} yaw={d.get('yawDeg')}"
                )
            time.sleep(0.2)
    finally:
        reader.stop_event.set()
    print(f"lines_seen={reader.lines_seen} packets_seen={reader.packets_seen}")
    return 0 if reader.packets_seen > 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default=DEFAULT_PORT)
    parser.add_argument("--baud", default=DEFAULT_BAUD, type=int)
    parser.add_argument("--http-port", default=DEFAULT_HTTP_PORT, type=int)
    parser.add_argument("--probe", type=float, default=0.0, help="Read serial for N seconds and exit")
    args = parser.parse_args()

    if args.probe > 0:
        return probe_serial(args.serial, args.baud, args.probe)

    broadcaster = Broadcaster()
    reader = SerialReader(args.serial, args.baud, broadcaster)
    reader.start()

    socketserver.TCPServer.allow_reuse_address = True
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(broadcaster))
    print(f"Open http://127.0.0.1:{args.http_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop_event.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
