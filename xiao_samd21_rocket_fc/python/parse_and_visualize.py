from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_flight_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["t_s"] = df["ms"] / 1000.0
    return df


def estimate_attitude(df: pd.DataFrame) -> pd.DataFrame:
    roll = np.zeros(len(df))
    pitch = np.zeros(len(df))
    yaw = np.zeros(len(df))

    alpha = 0.98

    for i in range(1, len(df)):
        dt = max(df["t_s"].iloc[i] - df["t_s"].iloc[i - 1], 1e-3)

        gx = math.radians(df["gx_dps"].iloc[i])
        gy = math.radians(df["gy_dps"].iloc[i])
        gz = math.radians(df["gz_dps"].iloc[i])

        roll_gyro = roll[i - 1] + gx * dt
        pitch_gyro = pitch[i - 1] + gy * dt
        yaw[i] = yaw[i - 1] + gz * dt

        ax = df["ax_g"].iloc[i]
        ay = df["ay_g"].iloc[i]
        az = df["az_g"].iloc[i]
        g = math.sqrt(ax * ax + ay * ay + az * az)
        if g < 1e-6:
            roll[i] = roll_gyro
            pitch[i] = pitch_gyro
            continue

        ax /= g
        ay /= g
        az /= g

        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        roll[i] = alpha * roll_gyro + (1.0 - alpha) * roll_acc
        pitch[i] = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc

    df["roll_rad"] = roll
    df["pitch_rad"] = pitch
    df["yaw_rad"] = yaw
    df["roll_deg"] = np.degrees(roll)
    df["pitch_deg"] = np.degrees(pitch)
    df["yaw_deg"] = np.degrees(yaw)
    return df


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def build_rocket_model() -> tuple[np.ndarray, list[tuple[int, int]]]:
    body = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    ]
    nose = [
        [1.0, 0.0, 0.0],
        [1.25, 0.0, 0.0],
    ]
    fin_span = 0.18
    fin_back = 0.18
    fin_root = 0.36
    fins = [
        [fin_back, fin_span, 0.0],
        [0.0, fin_span * 0.6, 0.0],
        [fin_root, 0.0, 0.0],
        [fin_back, -fin_span, 0.0],
        [0.0, -fin_span * 0.6, 0.0],
        [fin_root, 0.0, 0.0],
        [fin_back, 0.0, fin_span],
        [0.0, 0.0, fin_span * 0.6],
        [fin_root, 0.0, 0.0],
        [fin_back, 0.0, -fin_span],
        [0.0, 0.0, -fin_span * 0.6],
        [fin_root, 0.0, 0.0],
    ]
    points = np.array(body + nose + fins, dtype=float)
    segments = [
        (0, 1),
        (2, 3),
        (4, 5), (5, 6), (6, 4),
        (7, 8), (8, 9), (9, 7),
        (10, 11), (11, 12), (12, 10),
        (13, 14), (14, 15), (15, 13),
    ]
    return points, segments


def plot_timeseries(df: pd.DataFrame) -> tuple[plt.Figure, list[plt.Axes]]:
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(df["t_s"], df["baro_alt_m"], label="Baro altitude")
    axes[0].plot(df["t_s"], df["gps_alt_m"], label="GPS altitude", alpha=0.7)
    axes[0].set_ylabel("Altitude (m)")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(df["t_s"], df["ax_g"], label="ax")
    axes[1].plot(df["t_s"], df["ay_g"], label="ay")
    axes[1].plot(df["t_s"], df["az_g"], label="az")
    axes[1].set_ylabel("Accel (g)")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(df["t_s"], df["gx_dps"], label="gx")
    axes[2].plot(df["t_s"], df["gy_dps"], label="gy")
    axes[2].plot(df["t_s"], df["gz_dps"], label="gz")
    axes[2].set_ylabel("Gyro (dps)")
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(df["t_s"], df["roll_deg"], label="roll")
    axes[3].plot(df["t_s"], df["pitch_deg"], label="pitch")
    axes[3].plot(df["t_s"], df["yaw_deg"], label="yaw")
    axes[3].set_ylabel("Attitude (deg)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()
    axes[3].grid(True)

    fig.tight_layout()
    return fig, list(axes)


def animate_rocket(df: pd.DataFrame, step: int = 2) -> animation.FuncAnimation:
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    points, segments = build_rocket_model()
    lines = [ax.plot([], [], [], lw=2)[0] for _ in segments]
    trail, = ax.plot([], [], [], color="tab:orange", alpha=0.6, lw=1.5)

    gps_xyz = np.column_stack(
        [
            (df["lon_deg"] - df["lon_deg"].iloc[0]).fillna(0.0) * 85000.0,
            (df["lat_deg"] - df["lat_deg"].iloc[0]).fillna(0.0) * 111000.0,
            df["baro_alt_m"].ffill().fillna(0.0),
        ]
    )

    span = max(10.0, np.nanmax(np.ptp(gps_xyz, axis=0)) * 0.6)
    center = np.nanmean(gps_xyz, axis=0)
    ax.set_xlim(center[0] - span, center[0] + span)
    ax.set_ylim(center[1] - span, center[1] + span)
    ax.set_zlim(max(0.0, center[2] - span), center[2] + span)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("Rocket replay")

    def update(frame: int):
        idx = min(frame * step, len(df) - 1)
        row = df.iloc[idx]
        rot = rotation_matrix(row["roll_rad"], row["pitch_rad"], row["yaw_rad"])
        origin = gps_xyz[idx]
        rotated = (rot @ points.T).T + origin

        for line, (i0, i1) in zip(lines, segments):
            xyz = rotated[[i0, i1], :]
            line.set_data(xyz[:, 0], xyz[:, 1])
            line.set_3d_properties(xyz[:, 2])

        trail_xyz = gps_xyz[: idx + 1]
        trail.set_data(trail_xyz[:, 0], trail_xyz[:, 1])
        trail.set_3d_properties(trail_xyz[:, 2])
        ax.set_title(f"Rocket replay  t={row['t_s']:.2f}s  phase={row['phase']}")
        return lines + [trail]

    frames = max(1, math.ceil(len(df) / step))
    ani = animation.FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
    return ani


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse and visualize rocket flight log")
    parser.add_argument("csv", type=Path, help="Path to telemetry CSV")
    parser.add_argument("--no-animation", action="store_true", help="Only show timeseries plots")
    args = parser.parse_args()

    df = load_flight_csv(args.csv)
    df = estimate_attitude(df)
    plot_timeseries(df)

    if not args.no_animation:
        animate_rocket(df)

    plt.show()


if __name__ == "__main__":
    main()
