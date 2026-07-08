#!/usr/bin/env python3
"""Build a dependency-free HTML report for the May 9 RocketV10 flight logs."""

from __future__ import annotations

import argparse
import csv
import html
import math
from collections import Counter
from pathlib import Path
from typing import Iterable


STATE_NAMES = {
    0: "IDLE",
    1: "PAD",
    2: "ASCENT",
    3: "COAST",
    4: "DESCENT",
    5: "LANDED",
    6: "ABORT",
}


def fnum(value: object) -> float:
    try:
        parsed = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def safe_min(values: Iterable[float]) -> float:
    vals = [v for v in values if math.isfinite(v)]
    return min(vals) if vals else math.nan


def safe_max(values: Iterable[float]) -> float:
    vals = [v for v in values if math.isfinite(v)]
    return max(vals) if vals else math.nan


def fmt(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}" if math.isfinite(value) else "n/a"


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as src:
        return list(csv.DictReader(src))


def parse_kv_log(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    with path.open(errors="replace") as src:
        for line in src:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            row = {"type": parts[0]}
            for part in parts[1:]:
                if "=" in part:
                    key, value = part.split("=", 1)
                    row[key] = value
            rows.append(row)
    return rows


def rocket_summary(path: Path) -> dict[str, object]:
    rows = read_csv_rows(path)
    ms = [fnum(r.get("ms")) for r in rows]
    rel = [fnum(r.get("rel_alt_m")) for r in rows]
    alt = [fnum(r.get("alt_m")) for r in rows]
    vel = [fnum(r.get("vel_mps")) for r in rows]
    batt = [fnum(r.get("batt_v")) for r in rows]
    acc = [
        math.sqrt(fnum(r.get("ax")) ** 2 + fnum(r.get("ay")) ** 2 + fnum(r.get("az")) ** 2)
        for r in rows
    ]
    states = Counter(int(fnum(r.get("state"))) for r in rows if math.isfinite(fnum(r.get("state"))))
    gps_rows = sum(1 for r in rows if abs(fnum(r.get("lat"))) > 0.01 and abs(fnum(r.get("lon"))) > 0.01)
    duration = (ms[-1] - ms[0]) / 1000.0 if len(ms) >= 2 else 0.0
    return {
        "path": path,
        "rows": rows,
        "count": len(rows),
        "duration": duration,
        "max_rel": safe_max(rel),
        "max_alt": safe_max(alt),
        "max_vel": safe_max(vel),
        "min_vel": safe_min(vel),
        "max_acc": safe_max(acc),
        "min_batt": safe_min(batt),
        "max_batt": safe_max(batt),
        "gps_rows": gps_rows,
        "states": states,
    }


def state_transitions(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    if not rows:
        return []
    t0 = fnum(rows[0].get("ms"))
    out: list[dict[str, object]] = []
    last: int | None = None
    for row in rows:
        state = int(fnum(row.get("state"))) if math.isfinite(fnum(row.get("state"))) else None
        if state != last:
            out.append({
                "t": (fnum(row.get("ms")) - t0) / 1000.0,
                "state": state,
                "rel": fnum(row.get("rel_alt_m")),
                "vel": fnum(row.get("vel_mps")),
            })
            last = state
    return out


def ground_summary(path: Path) -> dict[str, object]:
    rows = parse_kv_log(path)
    ms_rows = [r for r in rows if "ms" in r]
    rel = [fnum(r.get("rkt_rel_alt")) for r in ms_rows]
    vel = [fnum(r.get("rkt_vel")) for r in ms_rows]
    rssi = [fnum(r.get("rssi_last")) for r in ms_rows]
    miss = [fnum(r.get("miss_f")) for r in ms_rows]
    states = Counter(r.get("rocket_state", r.get("type", "?")) for r in ms_rows)
    types = Counter(r.get("type", "?") for r in rows)
    return {
        "path": path,
        "rows": rows,
        "ms_rows": ms_rows,
        "types": types,
        "states": states,
        "max_rel": safe_max(rel),
        "min_rel": safe_min(rel),
        "max_vel": safe_max(vel),
        "min_rssi": safe_min(rssi),
        "max_miss": safe_max(miss),
    }


def points(rows: list[dict[str, str]], x_col: str, y_col: str, t0: float | None = None) -> list[tuple[float, float]]:
    out = []
    if t0 is None and rows:
        t0 = fnum(rows[0].get(x_col))
    for row in rows:
        x = fnum(row.get(x_col))
        y = fnum(row.get(y_col))
        if math.isfinite(x) and math.isfinite(y):
            out.append(((x - (t0 or 0.0)) / 1000.0, y))
    return out


def accel_points(rows: list[dict[str, str]]) -> list[tuple[float, float]]:
    if not rows:
        return []
    t0 = fnum(rows[0].get("ms"))
    out = []
    for row in rows:
        ax, ay, az = fnum(row.get("ax")), fnum(row.get("ay")), fnum(row.get("az"))
        ms = fnum(row.get("ms"))
        if all(math.isfinite(v) for v in [ax, ay, az, ms]):
            out.append(((ms - t0) / 1000.0, math.sqrt(ax * ax + ay * ay + az * az)))
    return out


def svg_plot(title: str, series: list[tuple[str, list[tuple[float, float]], str]], width: int = 980, height: int = 260) -> str:
    margin = 44
    xs = [x for _, pts, _ in series for x, _ in pts]
    ys = [y for _, pts, _ in series for _, y in pts]
    if not xs or not ys:
        return f"<h3>{html.escape(title)}</h3><p>No finite points.</p>"
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if xmin == xmax:
        xmax = xmin + 1.0
    if ymin == ymax:
        ymax = ymin + 1.0
    ypad = (ymax - ymin) * 0.08
    ymin -= ypad
    ymax += ypad

    def sx(x: float) -> float:
        return margin + (x - xmin) / (xmax - xmin) * (width - margin * 1.5)

    def sy(y: float) -> float:
        return height - margin - (y - ymin) / (ymax - ymin) * (height - margin * 1.5)

    paths = []
    legend = []
    for name, pts, color in series:
        if not pts:
            continue
        d = " ".join(("M" if i == 0 else "L") + f"{sx(x):.1f},{sy(y):.1f}" for i, (x, y) in enumerate(pts))
        paths.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2"/>')
        legend.append(f'<span><b style="color:{color}">■</b> {html.escape(name)}</span>')
    return f"""
<section class="plot">
  <h3>{html.escape(title)}</h3>
  <div class="legend">{' '.join(legend)}</div>
  <svg viewBox="0 0 {width} {height}" role="img">
    <rect x="0" y="0" width="{width}" height="{height}" fill="#fff"/>
    <line x1="{margin}" y1="{height-margin}" x2="{width-margin/2}" y2="{height-margin}" stroke="#999"/>
    <line x1="{margin}" y1="{margin/2}" x2="{margin}" y2="{height-margin}" stroke="#999"/>
    <text x="{margin}" y="{height-10}" font-size="12">t {fmt(xmin, 1)}s</text>
    <text x="{width-110}" y="{height-10}" font-size="12">t {fmt(xmax, 1)}s</text>
    <text x="6" y="{sy(ymax):.1f}" font-size="12">{fmt(ymax, 1)}</text>
    <text x="6" y="{sy(ymin):.1f}" font-size="12">{fmt(ymin, 1)}</text>
    {''.join(paths)}
  </svg>
</section>"""


def table(headers: list[str], rows: list[list[object]]) -> str:
    head = "".join(f"<th>{html.escape(h)}</th>" for h in headers)
    body = "\n".join("<tr>" + "".join(f"<td>{html.escape(str(c))}</td>" for c in row) + "</tr>" for row in rows)
    return f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>"


def build_report(root: Path, output: Path) -> None:
    rocket = [rocket_summary(p) for p in sorted((root / "rocket_v10").glob("rocket_flight*.csv"))]
    primary = max(rocket, key=lambda s: (float(s["max_rel"]), float(s["max_acc"])))
    ground_logs = [ground_summary(p) for p in sorted((root / "ground_v10").glob("ground_*.log"))]
    primary_ground = max(ground_logs, key=lambda s: (float(s["max_rel"]) if math.isfinite(float(s["max_rel"])) else -1.0, len(s["ms_rows"])))  # type: ignore[arg-type]
    centurion_files = sorted((root / "centurion").glob("*.CSV"))
    centurion = [(p, read_csv_rows(p)) for p in centurion_files]
    primary_centurion = max(
        centurion,
        key=lambda item: safe_max(fnum(r.get("alt_m")) for r in item[1]) if item[1] else -9999.0,
        default=None,
    )

    primary_rows = primary["rows"]  # type: ignore[assignment]
    assert isinstance(primary_rows, list)
    t0 = fnum(primary_rows[0].get("ms")) if primary_rows else 0.0
    ground_rows = primary_ground["ms_rows"]  # type: ignore[assignment]
    assert isinstance(ground_rows, list)
    gt0 = fnum(ground_rows[0].get("ms")) if ground_rows else 0.0

    summary_rows = []
    for item in rocket:
        states = item["states"]
        assert isinstance(states, Counter)
        summary_rows.append([
            Path(str(item["path"])).name,
            item["count"],
            fmt(float(item["duration"]), 1),
            fmt(float(item["max_rel"]), 2),
            fmt(float(item["max_acc"]), 2),
            fmt(float(item["min_batt"]), 2),
            item["gps_rows"],
            " ".join(f"{STATE_NAMES.get(k, k)}:{v}" for k, v in sorted(states.items())),
        ])

    transition_rows = [
        [fmt(float(t["t"]), 2), STATE_NAMES.get(t["state"], t["state"]), fmt(float(t["rel"]), 2), fmt(float(t["vel"]), 2)]
        for t in state_transitions(primary_rows)
    ]

    ground_state_rows = []
    states = primary_ground["states"]
    assert isinstance(states, Counter)
    for name, count in states.most_common():
        ground_state_rows.append([name, count])

    centurion_summary_rows = []
    for path, rows in centurion:
        ms = [fnum(r.get("ms")) for r in rows]
        duration = (ms[-1] - ms[0]) / 1000.0 if len(ms) >= 2 else 0.0
        states = Counter(r.get("state", "?") for r in rows)
        centurion_summary_rows.append([
            path.name,
            len(rows),
            fmt(duration, 1),
            fmt(safe_min(fnum(r.get("alt_m")) for r in rows), 2),
            fmt(safe_max(fnum(r.get("alt_m")) for r in rows), 2),
            fmt(safe_max(fnum(r.get("accelMag_g")) for r in rows), 2),
            " ".join(f"{k or 'blank'}:{v}" for k, v in sorted(states.items())),
        ])

    centurion_section = ""
    if primary_centurion is not None:
        centurion_path, centurion_rows = primary_centurion
        ct0 = fnum(centurion_rows[0].get("ms")) if centurion_rows else 0.0
        centurion_section = f"""
<h2>Centurion Auxiliary Logger</h2>
{table(["file", "rows", "duration_s", "min_alt_m", "max_alt_m", "max_accel_g", "states"], centurion_summary_rows)}
<p class="note">Primary Centurion file by max altitude: <code>{html.escape(centurion_path.name)}</code>.</p>
{svg_plot("Centurion altitude and acceleration", [("alt_m", points(centurion_rows, "ms", "alt_m", ct0), "#165DFF"), ("accelMag_g", points(centurion_rows, "ms", "accelMag_g", ct0), "#00875A")])}
{svg_plot("Centurion pitch and roll", [("pitch_deg", points(centurion_rows, "ms", "pitch_deg", ct0), "#B7791F"), ("roll_deg", points(centurion_rows, "ms", "roll_deg", ct0), "#7A4FD8")])}
"""

    html_text = f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>May 9 RocketV10 Flight Report</title>
<style>
body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 28px; color: #18202a; }}
h1, h2, h3 {{ margin-bottom: 0.35rem; }}
.note {{ color: #526070; }}
table {{ border-collapse: collapse; width: 100%; margin: 12px 0 24px; font-size: 13px; }}
th, td {{ border-bottom: 1px solid #d8dee6; padding: 7px 8px; text-align: left; vertical-align: top; }}
th {{ background: #f4f6f8; }}
.grid {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; margin: 16px 0 24px; }}
.metric {{ border: 1px solid #d8dee6; border-radius: 6px; padding: 12px; }}
.metric b {{ display: block; font-size: 22px; }}
.legend span {{ margin-right: 16px; font-size: 13px; }}
.plot {{ margin: 22px 0; }}
svg {{ width: 100%; height: auto; border: 1px solid #d8dee6; border-radius: 6px; }}
</style>
</head>
<body>
<h1>May 9 RocketV10 Flight Report</h1>
<p class="note">Generated from <code>{html.escape(str(root))}</code>. Primary rocket file selected by largest relative altitude: <code>{html.escape(Path(str(primary["path"])).name)}</code>.</p>
<div class="grid">
  <div class="metric">Max relative altitude<b>{fmt(float(primary["max_rel"]), 2)} m</b></div>
  <div class="metric">Max acceleration magnitude<b>{fmt(float(primary["max_acc"]), 2)} m/s²</b></div>
  <div class="metric">Max vertical velocity<b>{fmt(float(primary["max_vel"]), 2)} m/s</b></div>
  <div class="metric">Battery range<b>{fmt(float(primary["min_batt"]), 2)}-{fmt(float(primary["max_batt"]), 2)} V</b></div>
</div>
<h2>Rocket CSV Files</h2>
{table(["file", "rows", "duration_s", "max_rel_m", "max_acc_mps2", "min_batt_v", "gps_rows", "states"], summary_rows)}
<h2>Primary Rocket Flight</h2>
{table(["t_s", "state", "rel_alt_m", "vel_mps"], transition_rows)}
{svg_plot("Rocket relative altitude and vertical velocity", [("rel_alt_m", points(primary_rows, "ms", "rel_alt_m", t0), "#165DFF"), ("vel_mps", points(primary_rows, "ms", "vel_mps", t0), "#D64545")])}
{svg_plot("Rocket acceleration magnitude", [("accel_mag_mps2", accel_points(primary_rows), "#00875A")])}
{svg_plot("Rocket attitude", [("roll_deg", points(primary_rows, "ms", "roll", t0), "#7A4FD8"), ("pitch_deg", points(primary_rows, "ms", "pitch", t0), "#B7791F"), ("yaw_deg", points(primary_rows, "ms", "yaw", t0), "#0F766E")])}
{svg_plot("Rocket battery and health mask", [("batt_v", points(primary_rows, "ms", "batt_v", t0), "#444"), ("health", points(primary_rows, "ms", "health", t0), "#C2410C")])}
<h2>Ground Log</h2>
<p class="note">Primary ground log: <code>{html.escape(Path(str(primary_ground["path"])).name)}</code>. Max received rocket relative altitude {fmt(float(primary_ground["max_rel"]), 2)} m, min RSSI {fmt(float(primary_ground["min_rssi"]), 1)} dBm, max flight missed counter {fmt(float(primary_ground["max_miss"]), 0)}.</p>
{table(["rocket_state", "rows"], ground_state_rows)}
{svg_plot("Ground received rocket altitude and velocity", [("rkt_rel_alt", points(ground_rows, "ms", "rkt_rel_alt", gt0), "#165DFF"), ("rkt_vel", points(ground_rows, "ms", "rkt_vel", gt0), "#D64545")])}
{svg_plot("Ground radio RSSI and missed flight packets", [("rssi_last", points(ground_rows, "ms", "rssi_last", gt0), "#444"), ("miss_f", points(ground_rows, "ms", "miss_f", gt0), "#C2410C")])}
{centurion_section}
</body>
</html>
"""
    output.write_text(html_text)
    print(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", nargs="?", default="/Users/k_pochkaev/github/flight_logs/9May")
    parser.add_argument("--output", default="/Users/k_pochkaev/github/flight_logs/9May/flight_report.html")
    args = parser.parse_args()
    build_report(Path(args.root), Path(args.output))


if __name__ == "__main__":
    main()
