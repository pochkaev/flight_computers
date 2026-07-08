import serial
import time
import re
import csv
from collections import deque
import math

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# -----------------------
# CONFIG
# -----------------------

SERIAL_PORT = "/dev/cu.usbmodem184901201"
BAUD_RATE = 115200
MAX_POINTS = 1000
CSV_FILENAME = "telemetry_log.csv"

# -----------------------
# Parsing
# -----------------------

line_re = re.compile(
    r"V=(?P<version>\d+),\s*"
    r"SEQ=(?P<seq>\d+),\s*"
    r"ms=(?P<ms>\d+),\s*"
    r"alt=(?P<alt>[-+]?\d*\.?\d+),\s*"
    r"temp=(?P<temp>[-+]?\d*\.?\d+),\s*"
    r"ax=(?P<ax>[-+]?\d*\.?\d+),\s*"
    r"ay=(?P<ay>[-+]?\d*\.?\d+),\s*"
    r"az=(?P<az>[-+]?\d*\.?\d+),\s*"
    r"gx=(?P<gx>[-+]?\d*\.?\d+),\s*"
    r"gy=(?P<gy>[-+]?\d*\.?\d+),\s*"
    r"gz=(?P<gz>[-+]?\d*\.?\d+),\s*"
    r"RSSI=(?P<rssi>[-+]?\d+)"
)

def parse_line(line):
    m = line_re.match(line.strip())
    if not m:
        return None
    d = m.groupdict()
    return {
        "version": int(d["version"]),
        "seq": int(d["seq"]),
        "ms": int(d["ms"]),
        "alt": float(d["alt"]),
        "temp": float(d["temp"]),
        "ax": float(d["ax"]),
        "ay": float(d["ay"]),
        "az": float(d["az"]),
        "gx": float(d["gx"]),
        "gy": float(d["gy"]),
        "gz": float(d["gz"]),
        "rssi": int(d["rssi"]),
    }

# -----------------------
# Serial + logging setup
# -----------------------

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
print(f"Opened {SERIAL_PORT} at {BAUD_RATE} baud")

csv_file = open(CSV_FILENAME, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "t_s",
    "version",
    "seq",
    "ms",
    "alt_m",
    "temp_C",
    "ax_m_s2",
    "ay_m_s2",
    "az_m_s2",
    "gx_deg_s",
    "gy_deg_s",
    "gz_deg_s",
    "rssi_dBm",
])

start_time = time.time()

# Buffers for plots
t_data    = deque(maxlen=MAX_POINTS)
alt_data  = deque(maxlen=MAX_POINTS)
temp_data = deque(maxlen=MAX_POINTS)
rssi_data = deque(maxlen=MAX_POINTS)

# Latest IMU values for orientation
last_ax = 0.0
last_ay = 0.0
last_az = 9.81

# -----------------------
# Matplotlib setup
# -----------------------

plt.style.use("ggplot")

fig = plt.figure(figsize=(10, 8))
gs = fig.add_gridspec(4, 1, height_ratios=[2, 1, 1, 2])

ax_alt  = fig.add_subplot(gs[0, 0])
ax_temp = fig.add_subplot(gs[1, 0])
ax_rssi = fig.add_subplot(gs[2, 0])
ax_3d   = fig.add_subplot(gs[3, 0], projection="3d")

line_alt,  = ax_alt.plot([], [], label="Altitude (m)")
line_temp, = ax_temp.plot([], [], label="Temp (°C)")
line_rssi, = ax_rssi.plot([], [], label="RSSI (dBm)")

ax_alt.set_ylabel("Altitude (m)")
ax_temp.set_ylabel("Temp (°C)")
ax_rssi.set_ylabel("RSSI (dBm)")
ax_rssi.set_xlabel("Time (s) since start")

ax_alt.legend(loc="upper left")
ax_temp.legend(loc="upper left")
ax_rssi.legend(loc="upper left")

ax_alt.grid(True)
ax_temp.grid(True)
ax_rssi.grid(True)

# 3D rocket axis
rocket_line, = ax_3d.plot([], [], [], linewidth=3)

ax_3d.set_xlabel("X")
ax_3d.set_ylabel("Y")
ax_3d.set_zlabel("Z")
ax_3d.set_title("Rocket orientation (body axis)")
ax_3d.set_xlim(-1, 1)
ax_3d.set_ylim(-1, 1)
ax_3d.set_zlim(-1, 1)

# -----------------------
# Helper: update rocket orientation from accel
# -----------------------

def update_rocket(ax_val, ay_val, az_val):
    # Use accelerometer to estimate tilt (assumes only gravity)
    g_mag = math.sqrt(ax_val**2 + ay_val**2 + az_val**2)
    if g_mag == 0:
        return

    ax_n = ax_val / g_mag
    ay_n = ay_val / g_mag
    az_n = az_val / g_mag

    # Roll and pitch from accel
    roll = math.atan2(ay_n, az_n)
    pitch = math.atan2(-ax_n, math.sqrt(ay_n**2 + az_n**2))

    # Direction of rocket body axis (z_body) in world frame
    # Assuming yaw = 0 for now
    # x = -sin(pitch)
    # y = sin(roll)*cos(pitch)
    # z = cos(roll)*cos(pitch)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)

    x_dir = -sp
    y_dir = sr * cp
    z_dir = cr * cp

    length = 1.0
    x2 = x_dir * length
    y2 = y_dir * length
    z2 = z_dir * length

    rocket_line.set_data([0, x2], [0, y2])
    rocket_line.set_3d_properties([0, z2])

    # Keep cube-ish bounds
    ax_3d.set_xlim(-1, 1)
    ax_3d.set_ylim(-1, 1)
    ax_3d.set_zlim(-1, 1)

# -----------------------
# Animation / update loop
# -----------------------

def update(frame):
    global last_ax, last_ay, last_az

    # Read any available lines from serial
    while True:
        try:
            line_bytes = ser.readline()
        except serial.SerialException as e:
            print("Serial error:", e)
            return line_alt, line_temp, line_rssi, rocket_line

        if not line_bytes:
            break

        try:
            line = line_bytes.decode("utf-8", errors="ignore").strip()
        except UnicodeDecodeError:
            continue

        if not line:
            continue

        data = parse_line(line)
        if not data:
            continue

        now = time.time()
        t_s = now - start_time

        t_data.append(t_s)
        alt_data.append(data["alt"])
        temp_data.append(data["temp"])
        rssi_data.append(data["rssi"])

        last_ax = data["ax"]
        last_ay = data["ay"]
        last_az = data["az"]

        # Log everything
        csv_writer.writerow([
            f"{t_s:.3f}",
            data["version"],
            data["seq"],
            data["ms"],
            f"{data['alt']:.3f}",
            f"{data['temp']:.3f}",
            f"{data['ax']:.3f}",
            f"{data['ay']:.3f}",
            f"{data['az']:.3f}",
            f"{data['gx']:.3f}",
            f"{data['gy']:.3f}",
            f"{data['gz']:.3f}",
            data["rssi"],
        ])
        csv_file.flush()

    # Update 2D plots
    if t_data:
        line_alt.set_data(t_data, alt_data)
        line_temp.set_data(t_data, temp_data)
        line_rssi.set_data(t_data, rssi_data)

        ax_alt.relim();  ax_alt.autoscale_view()
        ax_temp.relim(); ax_temp.autoscale_view()
        ax_rssi.relim(); ax_rssi.autoscale_view()

    # Update 3D rocket orientation
    update_rocket(last_ax, last_ay, last_az)

    return line_alt, line_temp, line_rssi, rocket_line

ani = animation.FuncAnimation(fig, update, interval=200, blit=False)

try:
    plt.tight_layout()
    plt.show()
finally:
    print("Closing serial and log file")
    ser.close()
    csv_file.close()
