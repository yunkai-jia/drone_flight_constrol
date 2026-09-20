# Changelog

## 0.2.0 — 2026-09-20

Internal engineering preview. Not a vendor-accepted multi-airframe SDK.

- Direct vehicle URI (`udp` / `tcp` / `serial`): `connect()` waits for a PX4 or ArduPilot HEARTBEAT.
- Link diagnostics: `TransportOpenError`, `HeartbeatTimeoutError` (packet counts), `LinkLostError`, caller-driven `reconnect()`.
- `reconnect(uri=...)` can retarget after USB re-enumeration (e.g. `ttyACM0` → `ttyACM1`).
- pymavlink `autoreconnect` is off so reconnect is only `reconnect()`.
- `wait_telemetry` raises `TelemetryTimeoutError` if the predicate is still false; `spin` still pumps recv for a duration.
- Offboard `set_position` / `set_velocity`: prime setpoint → mode switch → setpoint again when not already GUIDED/OFFBOARD.
- `SET_MESSAGE_INTERVAL` send failures are kept in `telemetry_rate_errors` (logging warning); not swallowed silently.
- Mode tables and capability profile for PX4/ArduPilot × copter/plane/VTOL (unit-tested). Plane `land()` is rejected.
- Mission / Offboard / gimbal methods exist on the interface; continuous streaming and real vehicles are not protocol-accepted in this drop (see `docs/已知限制.md`).
- Device-side checklist (USB / TELEM2 / UART / UDP, identity, bus, unplug) in `docs/已知限制.md` section 5.
- `examples/sitl_takeoff_land.py` defaults to `skip_gps_check=False`; pass `--skip-gps-check` for SITL/mock only.
- Runtime dependencies: `pymavlink` and `pyserial`. Plot extra: `pip install -e ".[plot]"`.
- CI: GitHub Actions runs unit tests on Python 3.10 and 3.12.
