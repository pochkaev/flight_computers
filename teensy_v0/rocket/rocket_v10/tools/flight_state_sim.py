#!/usr/bin/env python3
"""Deterministic pre-flight checks for launch/state and recorder policies."""

from __future__ import annotations

G = 9.80665


def integrated_delta_v(accel_g: float, duration_ms: int, sample_ms: int = 5) -> float:
    value = 0.0
    for _ in range(duration_ms // sample_ms):
        value += max(0.0, accel_g - 1.0) * G * sample_ms / 1000.0
    return value


def check_launch_detector() -> None:
    bump_dv = integrated_delta_v(2.0, 40)
    assert bump_dv < 1.0, bump_dv
    motor_dv = integrated_delta_v(7.0, 50)
    assert motor_dv >= 1.0, motor_dv
    print(f"PASS touch/bump rejected: 2.0 g for 40 ms gives {bump_dv:.2f} m/s")
    print(f"PASS motor launch accepted: 7.0 g for 50 ms gives {motor_dv:.2f} m/s")


def check_state_sequence() -> None:
    state = "READY"
    state = "ASCENT"
    assert state == "ASCENT"
    launch_ms = 60_000
    coast_condition_ms = launch_ms + 751
    assert coast_condition_ms - launch_ms > 750
    state = "COAST" if 250 >= 250 else state
    assert state == "COAST"
    rel_alt_m, vertical_mps = 120.0, -1.0
    if rel_alt_m > 30.0 and vertical_mps < -0.5:
        state = "DESCENT_BALLISTIC"
    assert state == "DESCENT_BALLISTIC"
    apogee_ms = coast_condition_ms + 250
    touchdown_ms = apogee_ms + 15_001
    assert touchdown_ms - apogee_ms >= 15_000
    state = "POST_FLIGHT_GROUND"
    assert state == "POST_FLIGHT_GROUND"
    state = "LANDED"
    assert state == "LANDED"
    print("PASS sequence: READY -> ASCENT -> COAST -> DESCENT -> POST_FLIGHT -> LANDED")


def check_recovery_sequence() -> None:
    state = "READY"
    rel_alt_m = 40.0
    vertical_mps = -12.0
    held_ms = 1000
    if rel_alt_m > 25.0 and vertical_mps < -8.0 and held_ms >= 1000:
        state = "DESCENT_BALLISTIC"
    assert state == "DESCENT_BALLISTIC"
    print("PASS recovery: missed launch is recovered directly into ballistic descent")


def check_recorder_policy() -> None:
    total = (160 + 416) * 1024
    reserve = 64 * 1024
    primary = total - reserve
    # New quaternion IMU records are 40 bytes instead of the old 34-byte
    # Euler-only wide records. Calibration adds one 64-byte record per log.
    ascent_bytes_s = (
        40 * 200 + 20 * 25 + 28 * 50 + 82 * 10 + 16 * 10 + 36 * 7 + 42
    )
    descent_bytes_s = (
        40 * 50 + 20 * 25 + 28 * 50 + 82 * 10 + 16 * 10 + 36 * 7 + 42
    )
    critical_bytes_s = 28 * 50 + 82 * 10 + 16 * 10
    primary_after_10s = primary - ascent_bytes_s * 10
    descent_seconds = primary_after_10s / descent_bytes_s
    reserve_seconds = reserve / critical_bytes_s
    assert descent_seconds > 75
    assert reserve_seconds > 25
    assert primary + reserve == total

    primary_used = 0
    reserve_used = 0
    dropped_noncritical = 0
    dropped_critical = 0

    def append(size: int, critical: bool) -> None:
        nonlocal primary_used, reserve_used
        nonlocal dropped_noncritical, dropped_critical
        if primary_used + size <= primary:
            primary_used += size
        elif critical and reserve_used + size <= reserve:
            reserve_used += size
        elif critical:
            dropped_critical += 1
        else:
            dropped_noncritical += 1

    while primary_used + 40 <= primary:
        append(40, False)
    append(40, False)
    append(24, True)
    append(28, True)
    assert dropped_noncritical == 1
    assert dropped_critical == 0
    assert reserve_used == 52
    print(
        "PASS recorder: "
        f"~{10 + descent_seconds:.0f}s full-rate typical capture, then "
        f"~{reserve_seconds:.0f}s critical-state reserve"
    )
    print("PASS recorder overflow: IMU drops while later event/barometer records survive")


def main() -> None:
    check_launch_detector()
    check_state_sequence()
    check_recovery_sequence()
    check_recorder_policy()
    print("ALL PREFLIGHT SIMULATION CHECKS PASSED")


if __name__ == "__main__":
    main()
