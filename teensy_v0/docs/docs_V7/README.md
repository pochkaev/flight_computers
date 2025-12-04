
# Rocket Telemetry Protocol Specification (V7)

This document defines the binary telemetry packets transmitted by the rocket flight computer (Teensy 4.0) and decoded by the ground station.

Two packet types are transmitted:

1. Flight Telemetry Packet (FLIGHT_V7) — high-rate dynamic flight data
2. GPS Snapshot Packet (GPS_V7) — low-rate position fix for recovery

Both packet types are binary and transmitted over LoRa in raw byte form.

## 1) FLIGHT TELEMETRY PACKET — FlightPacketV7

### Purpose
Provides high-rate (5–10 Hz) flight dynamics data required for analysis, visualization, and event detection.

### Size: 38 bytes
### Sent: ~5–10 Hz, always active

### 1.1 Binary Structure (Layout)

Offset | Field | Type | Size | Units | Description
---: | --- | --- | --- | --- | ---
0 | version | uint8 | 1 | — | Always 7
1 | state | uint8 | 1 | enum | Flight state machine
2 | flags | uint16 | 2 | bitfield | Launch, Apogee, Landed bits
4 | seq | uint32 | 4 | count | Sequential packet index
8 | ms | uint32 | 4 | ms | System uptime
12 | alt_m | float | 4 | m | Filtered barometric altitude
16 | vel_mps | float | 4 | m/s | Vertical velocity
20 | ax | float | 4 | m/s² | Accel X
24 | ay | float | 4 | m/s² | Accel Y
28 | az | float | 4 | m/s² | Accel Z
32 | roll_deg | float | 4 | deg | Roll angle
36 | pitch_deg | float | 4 | deg | Pitch angle

Total = 38 bytes

## 2) GPS SNAPSHOT PACKET — GPSPacketV7

### Purpose
Low-rate GPS-only packet used only for rocket recovery location.

### Size: 20 bytes
### Sent: Only when GPS has fix (~1 Hz)

### 2.1 Binary Structure

Offset | Field | Type | Size | Units | Description
---: | --- | --- | --- | --- | ---
0 | version | uint8 | 1 | — | Always 7
1 | fix_type | uint8 | 1 | 0/2/3 | GPS fix dimension
2 | sats | uint8 | 1 | count | Satellites tracked
3 | reserved | uint8 | 1 | — | padding
4 | lat_deg | float | 4 | deg | Latitude
8 | lon_deg | float | 4 | deg | Longitude
12 | gps_alt_m | float | 4 | m | GPS altitude
16 | speed_mps | float | 4 | m/s | Ground speed

Total = 20 bytes

## Summary Table

Packet | Size | Rate | Purpose
--- | --- | --- | ---
FlightPacketV7 | 38 bytes | 5–10 Hz | Dynamics + IMU + altitude
GPSPacketV7 | 20 bytes | 0.5–1 Hz | Rocket recovery position
