# TODO – Ground v8 and Rocket v7 Alignment

## Rocket firmware (rocket_v7)

- [ ] Increase IMU + baro internal update rate to 50–200 Hz, keep LoRa telemetry at 5–10 Hz.
- [ ] Add rocket battery voltage measurement and include it in flight/nav telemetry packets.
- [ ] Extend flight/event flags (e.g. separate pyro events, more detailed ascent/coast/descent states).
- [ ] Optionally add magnetometer support and simple roll/pitch/yaw estimation.

## Telemetry / LoRa protocol

- [ ] Define a single unified “v8” telemetry format (binary) that carries: time, state, flags, baro alt, velocity, accel, roll/pitch, rocket battery, GPS alt/lat/lon, RSSI.
- [ ] Add a packet sequence number and simple payload checksum/CRC in addition to LoRa’s built‑in CRC.
- [ ] Keep a single fixed 915 MHz channel (no FHSS) but reserve fields for future channel ID or hop information.

## GPS (rocket and ground)

- [ ] Implement a small UBX configuration helper for GT‑U7 (u‑blox) to set: dynamic model = airborne, update rate (e.g. 5 Hz rocket, 1 Hz ground), and reduced NMEA sentences.
- [ ] Optionally add a landing power‑save mode that slows GPS updates after `FS_LANDED`.

## Logging and analysis

- [ ] Finalize log line formats on ground (`PAD`, `FLG`, `NAV`, `LOST`, `PWR`, `PROF`) and document them.
- [ ] Add a parser script that reads SD logs and produces CSV/plots for altitude, velocity, IMU, GPS, and power (similar to the HPR “Parser” folder).
- [ ] Consider logging rocket battery and key power‑module events on the ground station.

## Configuration and robustness

- [ ] Introduce a small settings struct in EEPROM for both rocket and ground (frequency, rocket name/callsign, debug level, logging options), including a timestamp and version field.
- [ ] Add support for loading settings from an SD-card config file on boot; compare timestamps so EEPROM always holds the latest config (whether last changed from SD or from PC over USB-serial).
- [ ] Ensure all peripherals are optional (LoRa, GPS, SD, RS-485, OLED): system should still boot and run in a degraded mode if any device is missing.
- [ ] Add a simple USB-serial configuration protocol on the rocket (GETCFG / SET key=value / SAVECFG) so a future PC/app can edit and store settings into EEPROM.

## Ground station UI (ground_v8)

- [ ] Treat Flight, Landed, and Lost pages as the primary rocket status views; keep Launch and GND MODULE focused on pad/ignition.
- [ ] Optionally add a compact “link” overlay or page (packet rate, RSSI history, last packet age).
- [ ] Re‑tune UI update intervals after any major telemetry‑rate changes to balance smoothness vs CPU time.

## RF / Antennas

- [ ] For now, keep simple whips on rocket and ground; verify mounting and keep clear of metal/carbon.
- [ ] Later, consider a small handheld Yagi for the ground station receive antenna for longer range and direction finding.
