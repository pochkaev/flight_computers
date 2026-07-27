#!/usr/bin/env python3
"""Build a self-contained Rocket V10 bench-orientation replay.

The input is the terminal transcript produced while `IMU STREAM START` is
active.  The output HTML has no network dependencies and replays the recorded
airframe quaternion.  Translation is intentionally not reconstructed from a
bench accelerometer because the result would be dominated by integration drift.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import webbrowser
from pathlib import Path


FIELDS = (
    "sample_us",
    "dt_us",
    "ax_mps2",
    "ay_mps2",
    "az_mps2",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "qw",
    "qx",
    "qy",
    "qz",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "confidence",
    "quality_flags",
)


def parse_transcript(path: Path) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []
    for raw_line in path.read_text(errors="replace").splitlines():
        line = raw_line.strip().replace("\r", "")
        if not line.startswith("IMU_DATA,"):
            continue
        values = line.split(",")[1:]
        if len(values) != len(FIELDS):
            continue
        parsed: dict[str, float | int] = {
            name: float(value) for name, value in zip(FIELDS, values)
        }
        parsed["sample_us"] = int(parsed["sample_us"])
        parsed["dt_us"] = int(parsed["dt_us"])
        parsed["quality_flags"] = int(parsed["quality_flags"])
        rows.append(parsed)
    if len(rows) < 2:
        raise ValueError(f"{path} contains fewer than two valid IMU_DATA rows")

    first_us = int(rows[0]["sample_us"])
    previous_us = first_us
    for row in rows:
        sample_us = int(row["sample_us"])
        if sample_us < previous_us:
            raise ValueError("sample timestamps are not monotonic")
        previous_us = sample_us
        row["time_s"] = (sample_us - first_us) / 1_000_000.0
        row["gyro_dps"] = math.sqrt(
            sum(float(row[name]) ** 2 for name in ("gx_dps", "gy_dps", "gz_dps"))
        )
        row["accel_mps2"] = math.sqrt(
            sum(float(row[name]) ** 2 for name in ("ax_mps2", "ay_mps2", "az_mps2"))
        )
        row["q_norm"] = math.sqrt(
            sum(float(row[name]) ** 2 for name in ("qw", "qx", "qy", "qz"))
        )
    return rows


def detect_motion(rows: list[dict[str, float | int]]) -> tuple[int, int]:
    moving = [
        float(row["gyro_dps"]) > 8.0
        or abs(float(row["accel_mps2"]) - 9.80665) > 1.5
        for row in rows
    ]
    # Fill very short quiet holes, then require three consecutive active samples.
    for index in range(1, len(moving) - 1):
        if not moving[index] and moving[index - 1] and moving[index + 1]:
            moving[index] = True
    start = next(
        (
            index
            for index in range(len(moving) - 2)
            if moving[index] and moving[index + 1] and moving[index + 2]
        ),
        0,
    )
    end = len(rows) - 1
    for index in range(len(moving) - 1, start, -1):
        if moving[index]:
            end = index
            break
    return start, end


def write_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    columns = ("time_s",) + FIELDS + ("gyro_dps", "accel_mps2", "q_norm")
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def make_payload(rows: list[dict[str, float | int]]) -> list[list[float | int]]:
    keys = (
        "time_s",
        "ax_mps2",
        "ay_mps2",
        "az_mps2",
        "gx_dps",
        "gy_dps",
        "gz_dps",
        "qw",
        "qx",
        "qy",
        "qz",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "confidence",
        "quality_flags",
        "gyro_dps",
        "accel_mps2",
    )
    return [[row[key] for key in keys] for row in rows]


def build_html(
    path: Path,
    rows: list[dict[str, float | int]],
    motion_start: int,
    motion_end: int,
    source_name: str,
) -> None:
    duration = float(rows[-1]["time_s"])
    motion_start_s = float(rows[motion_start]["time_s"])
    motion_end_s = float(rows[motion_end]["time_s"])
    sample_intervals = [
        float(rows[index]["time_s"]) - float(rows[index - 1]["time_s"])
        for index in range(1, len(rows))
    ]
    sample_hz = 1.0 / statistics.median(sample_intervals)
    max_gyro = max(float(row["gyro_dps"]) for row in rows)
    max_q_error = max(abs(float(row["q_norm"]) - 1.0) for row in rows)
    low_confidence = sum(float(row["confidence"]) < 0.5 for row in rows)

    substitutions = {
        "__DATA__": json.dumps(make_payload(rows), separators=(",", ":")),
        "__SOURCE__": json.dumps(source_name),
        "__DURATION__": f"{duration:.2f}",
        "__MOTION_START__": f"{motion_start_s:.6f}",
        "__MOTION_END__": f"{motion_end_s:.6f}",
        "__MOTION_DURATION__": f"{motion_end_s - motion_start_s:.2f}",
        "__SAMPLE_COUNT__": str(len(rows)),
        "__SAMPLE_HZ__": f"{sample_hz:.1f}",
        "__MAX_GYRO__": f"{max_gyro:.1f}",
        "__MAX_Q_ERROR__": f"{max_q_error:.6f}",
        "__LOW_CONFIDENCE__": f"{100.0 * low_confidence / len(rows):.1f}",
    }
    html = HTML_TEMPLATE
    for token, value in substitutions.items():
        html = html.replace(token, value)
    path.write_text(html)


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rocket V10 · IMU motion replay</title>
<style>
:root{color-scheme:dark;--bg:#070a0f;--panel:#101722;--line:#253246;--muted:#8391a7;
--text:#eef4ff;--cyan:#56d9ff;--orange:#ff9b54;--green:#4fe1a5;--red:#ff637d}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 25% -10%,#17263b 0,#070a0f 44%);
color:var(--text);font:14px/1.4 Inter,ui-sans-serif,system-ui,-apple-system,sans-serif;min-height:100vh}
main{max-width:1500px;margin:auto;padding:24px}.top{display:flex;align-items:flex-end;justify-content:space-between;gap:20px;margin-bottom:18px}
h1{font-size:clamp(24px,3vw,42px);letter-spacing:-.04em;margin:0}.eyebrow{color:var(--cyan);font-size:11px;
font-weight:800;letter-spacing:.18em;text-transform:uppercase;margin-bottom:6px}.sub{color:var(--muted);margin-top:4px}
.badge{border:1px solid #284059;border-radius:999px;padding:8px 12px;color:#b7c7da;background:#0b111b;white-space:nowrap}
.grid{display:grid;grid-template-columns:minmax(0,1.65fr) minmax(320px,.75fr);gap:16px}
.panel{background:linear-gradient(145deg,rgba(17,25,38,.94),rgba(9,14,22,.96));border:1px solid #202c3d;
border-radius:18px;box-shadow:0 20px 60px #0007;overflow:hidden}.stage{position:relative;min-height:610px}
#scene{width:100%;height:610px;display:block;cursor:grab}.overlay{position:absolute;left:18px;top:16px;pointer-events:none}
.time{font-variant-numeric:tabular-nums;font-size:34px;font-weight:750;letter-spacing:-.04em}.state{color:var(--green);font-size:12px;
font-weight:800;letter-spacing:.12em;text-transform:uppercase}.hint{position:absolute;right:16px;bottom:14px;color:#708099;font-size:11px}
.side{padding:18px;display:flex;flex-direction:column;gap:16px}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.metric{padding:12px;border-radius:12px;background:#0a1019;border:1px solid #202b3a}.metric b{display:block;font-size:21px;
font-variant-numeric:tabular-nums}.metric span{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.08em}
.angles{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}.angle{text-align:center;padding:10px 4px;background:#0a1019;border-radius:10px}
.angle b{font-size:18px;font-variant-numeric:tabular-nums}.angle span{display:block;color:var(--muted);font-size:10px}
.quality{padding:12px;border-radius:12px;border:1px solid #24354a}.quality-head{display:flex;justify-content:space-between}
.bar{height:7px;border-radius:9px;background:#202b39;overflow:hidden;margin-top:9px}.bar i{display:block;height:100%;background:var(--green)}
.note{color:#91a1b8;font-size:12px;border-left:2px solid var(--cyan);padding-left:10px}
.controls{grid-column:1/-1;padding:16px 18px}.row{display:flex;align-items:center;gap:12px}
button,select{border:1px solid #32435b;background:#121c2a;color:var(--text);border-radius:10px;padding:9px 13px;font:inherit}
button:hover{border-color:var(--cyan)}button.primary{background:var(--cyan);border-color:var(--cyan);color:#041019;font-weight:800}
input[type=range]{flex:1;accent-color:var(--cyan)}.range-label{font-variant-numeric:tabular-nums;color:#a6b5c8;min-width:72px;text-align:right}
#chart{height:190px;width:100%;display:block;margin-top:12px}.legend{display:flex;gap:14px;color:var(--muted);font-size:11px}
.dot{display:inline-block;width:8px;height:8px;border-radius:99px;margin-right:5px}.summary{display:grid;grid-template-columns:repeat(5,1fr);
gap:10px;margin-top:16px}.summary div{background:#0c121c;border:1px solid #1d2938;border-radius:12px;padding:12px}
.summary b{display:block;font-size:19px}.summary span{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}
@media(max-width:900px){main{padding:12px}.grid{grid-template-columns:1fr}.stage,#scene{min-height:440px;height:440px}
.summary{grid-template-columns:repeat(2,1fr)}.top{align-items:flex-start;flex-direction:column}}
</style>
</head>
<body><main>
<div class="top"><div><div class="eyebrow">Rocket V10 · measured orientation</div><h1>Bench motion replay</h1>
<div class="sub">Quaternion attitude from <span id="source"></span></div></div><div class="badge">Offline · self-contained report</div></div>
<div class="grid">
  <section class="panel stage"><canvas id="scene"></canvas><div class="overlay"><div class="state" id="state">MOTION</div>
  <div class="time" id="clock">00.00 s</div></div><div class="hint">drag to orbit camera · scroll to zoom</div></section>
  <aside class="panel side">
    <div><div class="eyebrow">Airframe attitude</div><div class="angles">
      <div class="angle"><b id="roll">—</b><span>ROLL</span></div><div class="angle"><b id="pitch">—</b><span>PITCH</span></div>
      <div class="angle"><b id="yaw">—</b><span>YAW</span></div></div></div>
    <div class="metrics"><div class="metric"><b id="gyro">—</b><span>Angular rate °/s</span></div>
      <div class="metric"><b id="accel">—</b><span>Acceleration m/s²</span></div></div>
    <div class="quality"><div class="quality-head"><span>Estimator confidence</span><b id="confidence">—</b></div>
      <div class="bar"><i id="confidenceBar"></i></div></div>
    <div class="metric"><b id="flags">—</b><span>Quality flags</span></div>
    <div class="note">The rocket is fixed at the center because bench translation cannot be reconstructed reliably from acceleration alone. Rotation is measured, not guessed.</div>
    <div class="note">Airframe +Z points through the nose. Cyan, orange and green vectors are the rocket X, Y and Z axes.</div>
  </aside>
  <section class="panel controls">
    <div class="row"><button class="primary" id="play">Pause</button><button id="restart">Restart motion</button>
      <select id="speed"><option value=".25">0.25×</option><option value=".5">0.5×</option><option value="1" selected>1×</option>
      <option value="2">2×</option><option value="4">4×</option></select>
      <input id="scrub" type="range" min="0" max="1000" value="0"><span class="range-label" id="duration">/ __DURATION__ s</span></div>
    <canvas id="chart"></canvas><div class="legend"><span><i class="dot" style="background:#56d9ff"></i>roll</span>
      <span><i class="dot" style="background:#ff9b54"></i>pitch</span><span><i class="dot" style="background:#4fe1a5"></i>yaw</span>
      <span><i class="dot" style="background:#ff637d"></i>angular rate</span></div>
  </section>
</div>
<div class="summary"><div><b>__MOTION_DURATION__ s</b><span>Detected motion</span></div><div><b>__SAMPLE_COUNT__</b><span>Samples</span></div>
<div><b>__SAMPLE_HZ__ Hz</b><span>Stream rate</span></div><div><b>__MAX_GYRO__ °/s</b><span>Peak rotation</span></div>
<div><b>__MAX_Q_ERROR__</b><span>Max quaternion norm error</span></div></div>
</main>
<script>
"use strict";
const rows=__DATA__, source=__SOURCE__, motionStart=__MOTION_START__, motionEnd=__MOTION_END__;
document.getElementById("source").textContent=source;
const $=id=>document.getElementById(id), scene=$("scene"), ctx=scene.getContext("2d"), chart=$("chart"), cctx=chart.getContext("2d");
let play=true, playTime=motionStart, last=performance.now(), speed=1, camYaw=-.72, camPitch=.42, zoom=1, dragging=false, dragX=0, dragY=0;
const idx={t:0,ax:1,ay:2,az:3,gx:4,gy:5,gz:6,qw:7,qx:8,qy:9,qz:10,roll:11,pitch:12,yaw:13,conf:14,flags:15,gyro:16,accel:17};
function resize(canvas,context){const r=canvas.getBoundingClientRect(),d=devicePixelRatio||1;if(canvas.width!==r.width*d||canvas.height!==r.height*d){
 canvas.width=r.width*d;canvas.height=r.height*d;context.setTransform(d,0,0,d,0,0)}}
function sample(t){let lo=0,hi=rows.length-1;while(lo<hi){const m=(lo+hi)>>1;if(rows[m][0]<t)lo=m+1;else hi=m}
 const b=Math.max(1,lo),a=b-1,u=Math.max(0,Math.min(1,(t-rows[a][0])/(rows[b][0]-rows[a][0]||1)));
 const out=rows[a].map((v,k)=>v+(rows[b][k]-v)*u),qa=rows[a].slice(7,11),qb=rows[b].slice(7,11);out.splice(7,4,...slerp(qa,qb,u));return out}
function slerp(a,b,t){let d=a.reduce((s,v,i)=>s+v*b[i],0),bb=b;if(d<0){d=-d;bb=b.map(v=>-v)}if(d>.9995){
 const q=a.map((v,i)=>v+t*(bb[i]-v)),n=Math.hypot(...q);return q.map(v=>v/n)}const th=Math.acos(Math.min(1,d)),s=Math.sin(th);
 return a.map((v,i)=>(Math.sin((1-t)*th)*v+Math.sin(t*th)*bb[i])/s)}
function qrot(q,v){const [w,x,y,z]=q,[vx,vy,vz]=v,ix=w*vx+y*vz-z*vy,iy=w*vy+z*vx-x*vz,iz=w*vz+x*vy-y*vx,iw=-x*vx-y*vy-z*vz;
 return [ix*w+iw*-x+iy*-z-iz*-y,iy*w+iw*-y+iz*-x-ix*-z,iz*w+iw*-z+ix*-y-iy*-x]}
function camera(v){let [x,y,z]=v,cy=Math.cos(camYaw),sy=Math.sin(camYaw),cp=Math.cos(camPitch),sp=Math.sin(camPitch);
 let x1=cy*x-sy*y,y1=sy*x+cy*y;return [x1,cp*z-sp*y1,sp*z+cp*y1]}
function project(v,w,h){let [x,y,z]=camera(v),f=230*zoom/(4-z*.18);return [w/2+x*f,h/2-y*f,z]}
function line3(a,b,color,width=1,alpha=1){const w=scene.clientWidth,h=scene.clientHeight,A=project(a,w,h),B=project(b,w,h);ctx.globalAlpha=alpha;ctx.strokeStyle=color;
 ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(A[0],A[1]);ctx.lineTo(B[0],B[1]);ctx.stroke();ctx.globalAlpha=1}
function poly3(points,fill,stroke="#fff2"){const w=scene.clientWidth,h=scene.clientHeight,p=points.map(v=>project(v,w,h));ctx.beginPath();p.forEach((v,i)=>i?ctx.lineTo(v[0],v[1]):ctx.moveTo(v[0],v[1]));
 ctx.closePath();ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.stroke()}
function drawScene(r){
 resize(scene,ctx);const w=scene.clientWidth,h=scene.clientHeight;ctx.clearRect(0,0,w,h);
 const grad=ctx.createRadialGradient(w*.5,h*.48,20,w*.5,h*.5,w*.65);grad.addColorStop(0,"#162337");grad.addColorStop(1,"#070b12");ctx.fillStyle=grad;ctx.fillRect(0,0,w,h);
 for(let n=-5;n<=5;n++){line3([-5,n,-1.25],[5,n,-1.25],"#29405a",1,n===0?.7:.28);line3([n,-5,-1.25],[n,5,-1.25],"#29405a",1,n===0?.7:.28)}
 const q=r.slice(7,11),R=v=>qrot(q,v), ring=(z,rad,n=18)=>Array.from({length:n},(_,i)=>R([rad*Math.cos(i*2*Math.PI/n),rad*Math.sin(i*2*Math.PI/n),z]));
 const back=ring(-1.35,.23),front=ring(.9,.23),nose=R([0,0,1.65]);
 for(let i=0;i<18;i++)poly3([back[i],back[(i+1)%18],front[(i+1)%18],front[i]],i%2?"#d7dde6":"#f6f8fb","#596474");
 for(let i=0;i<18;i++)poly3([front[i],front[(i+1)%18],nose],"#d84c4f","#ff8990");
 [[1,0],[-1,0],[0,1],[0,-1]].forEach(([sx,sy])=>{let a=R([sx*.18,sy*.18,-.75]),b=R([sx*.92,sy*.92,-1.3]),c=R([sx*.2,sy*.2,-1.33]);poly3([a,b,c],"#c92f38","#ff7a82")});
 line3(R([0,0,-1.65]),R([0,0,1.95]),"#fff",2,.55);line3([0,0,0],R([.9,0,0]),"#56d9ff",3);line3([0,0,0],R([0,.9,0]),"#ff9b54",3);line3([0,0,0],R([0,0,2.05]),"#4fe1a5",4);
 ctx.fillStyle="#72849c";ctx.font="11px system-ui";ctx.fillText("+Z NOSE",project(R([0,0,2.2]),w,h)[0]+5,project(R([0,0,2.2]),w,h)[1]);
}
function drawChart(){
 resize(chart,cctx);const w=chart.clientWidth,h=chart.clientHeight;cctx.clearRect(0,0,w,h);cctx.fillStyle="#080e16";cctx.fillRect(0,0,w,h);
 const pad={l:38,r:12,t:10,b:22},x=t=>pad.l+t/rows.at(-1)[0]*(w-pad.l-pad.r), yA=v=>pad.t+(180-v)/360*(h-pad.t-pad.b);
 for(let v=-180;v<=180;v+=90){cctx.strokeStyle="#263447";cctx.beginPath();cctx.moveTo(pad.l,yA(v));cctx.lineTo(w-pad.r,yA(v));cctx.stroke()}
 function trace(k,color,map=yA){cctx.strokeStyle=color;cctx.lineWidth=1.35;cctx.beginPath();rows.forEach((r,i)=>{let X=x(r[0]),Y=map(r[k]);i?cctx.lineTo(X,Y):cctx.moveTo(X,Y)});cctx.stroke()}
 trace(idx.roll,"#56d9ff");trace(idx.pitch,"#ff9b54");trace(idx.yaw,"#4fe1a5");
 const maxG=Math.max(...rows.map(r=>r[idx.gyro])),yG=v=>h-pad.b-v/maxG*(h-pad.t-pad.b);trace(idx.gyro,"#ff637d",yG);
 cctx.fillStyle="#ffffff10";cctx.fillRect(x(motionStart),pad.t,x(motionEnd)-x(motionStart),h-pad.t-pad.b);
 cctx.strokeStyle="#fff";cctx.globalAlpha=.75;cctx.beginPath();cctx.moveTo(x(playTime),pad.t);cctx.lineTo(x(playTime),h-pad.b);cctx.stroke();cctx.globalAlpha=1;
 cctx.fillStyle="#7f8da2";cctx.font="10px system-ui";cctx.fillText("−180°",2,yA(-180)+3);cctx.fillText("0°",14,yA(0)+3);cctx.fillText("180°",5,yA(180)+3)
}
function update(now){
 const dt=Math.min(.1,(now-last)/1000);last=now;if(play){playTime+=dt*speed;if(playTime>motionEnd)playTime=motionStart}
 const r=sample(playTime);drawScene(r);drawChart();$("clock").textContent=playTime.toFixed(2)+" s";$("state").textContent=playTime>=motionStart&&playTime<=motionEnd?"MOTION":"STILL";
 $("roll").textContent=r[idx.roll].toFixed(1)+"°";$("pitch").textContent=r[idx.pitch].toFixed(1)+"°";$("yaw").textContent=r[idx.yaw].toFixed(1)+"°";
 $("gyro").textContent=r[idx.gyro].toFixed(1);$("accel").textContent=r[idx.accel].toFixed(2);$("confidence").textContent=(r[idx.conf]*100).toFixed(0)+"%";
 $("confidenceBar").style.width=(r[idx.conf]*100)+"%";$("confidenceBar").style.background=r[idx.conf]>.8?"#4fe1a5":r[idx.conf]>.5?"#ff9b54":"#ff637d";
 $("flags").textContent="0x"+Math.round(r[idx.flags]).toString(16).toUpperCase().padStart(4,"0");$("scrub").value=playTime/rows.at(-1)[0]*1000;requestAnimationFrame(update)}
$("play").onclick=()=>{play=!play;$("play").textContent=play?"Pause":"Play"};$("restart").onclick=()=>{playTime=motionStart;play=true;$("play").textContent="Pause"};
$("speed").onchange=e=>speed=+e.target.value;$("scrub").oninput=e=>{playTime=+e.target.value/1000*rows.at(-1)[0];play=false;$("play").textContent="Play"};
scene.onpointerdown=e=>{dragging=true;dragX=e.clientX;dragY=e.clientY;scene.setPointerCapture(e.pointerId);scene.style.cursor="grabbing"};
scene.onpointermove=e=>{if(!dragging)return;camYaw+=(e.clientX-dragX)*.008;camPitch=Math.max(-1.3,Math.min(1.3,camPitch+(e.clientY-dragY)*.006));dragX=e.clientX;dragY=e.clientY};
scene.onpointerup=()=>{dragging=false;scene.style.cursor="grab"};scene.onwheel=e=>{e.preventDefault();zoom=Math.max(.55,Math.min(2.1,zoom*Math.exp(-e.deltaY*.001)))};
requestAnimationFrame(update);
</script></body></html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("transcript", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()

    transcript = args.transcript.resolve()
    output = (args.output or transcript.with_suffix(".html")).resolve()
    csv_path = (args.csv or transcript.with_suffix(".csv")).resolve()
    rows = parse_transcript(transcript)
    motion_start, motion_end = detect_motion(rows)
    write_csv(csv_path, rows)
    build_html(output, rows, motion_start, motion_end, transcript.name)
    print(
        f"Wrote {output}\nWrote {csv_path}\n"
        f"Samples: {len(rows)}; capture: {float(rows[-1]['time_s']):.2f} s; "
        f"motion: {float(rows[motion_start]['time_s']):.2f}-"
        f"{float(rows[motion_end]['time_s']):.2f} s"
    )
    if args.open:
        webbrowser.open(output.as_uri())


if __name__ == "__main__":
    main()
