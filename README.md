# Civil flight-controller Python API (MAVLink)

One Python API for **PX4** and **ArduPilot**, across **multicopter, fixed-wing, and VTOL**. Applications do not branch on firmware or airframe.

**Vendors / internal review:** start with [docs/供应商集成说明.md](docs/供应商集成说明.md) and the acceptance scope in [docs/已知限制.md](docs/已知限制.md) (section 5: device-side checklist by USB / TELEM2 / UART / UDP). Product notes: [docs/需求文档.md](docs/需求文档.md). Changelog: [CHANGELOG.md](CHANGELOG.md).

This tree is **0.2 engineering preview**. Link open, HEARTBEAT, drop, and `reconnect` are tested against a **PX4 copter protocol mock**. Plane/VTOL mode tables exist; Mission / Offboard / gimbal are exposed APIs, not protocol-accepted. No weapons or mission-strike logic.

## Direct vehicle connection

`connect()` opens the port **and** waits for the autopilot HEARTBEAT. You do not `sleep` afterwards.

```python
from drone_fc import ConnectionConfig, MavlinkFlightController

# USB to the flight controller
fc = MavlinkFlightController(ConnectionConfig("serial:/dev/ttyACM0:57600"))
# Jetson TELEM2:  ConnectionConfig("serial:/dev/ttyTHS1:921600")
# Raspberry Pi:   ConnectionConfig("serial:/dev/serial0:921600")
# Local router:   ConnectionConfig("udp:127.0.0.1:14540")

fc.connect()   # linked when this returns
fc.takeoff(5.0)
fc.wait_telemetry(
    lambda s: s.position is not None and s.position.altitude_m is not None and s.position.altitude_m >= 3.5,
    20.0,
)
fc.land()
fc.disconnect()
```

Keep `skip_gps_check=False` on a real aircraft (the default). SITL/mock may set it True.

```bash
python examples/companion_check_link.py --uri serial:/dev/ttyACM0:57600
python examples/vendor_same_control.py --uri serial:/dev/ttyACM0:57600 --dry-run
```

Need `dialout` (or equivalent) for serial, matching baud, and TELEM2 configured on the FC. A missing vehicle shows up as `HeartbeatTimeoutError` (with `packets_seen` / `other_heartbeats` so you can tell silence from a GCS-only bus). After a drop, call `reconnect()`; a stale FC heartbeat raises `LinkLostError`. MAVLink is unauthenticated: anyone who can reach the URI can send commands. This library does not replace RC; if the companion process dies, behaviour is whatever you set on the autopilot (RC loss / onboard loss). Use one `MavlinkFlightController` per control thread.

| URI | Typical use |
|-----|-------------|
| `serial:/dev/ttyACM0:57600` | FC USB |
| `serial:/dev/ttyTHS1:921600` | Jetson UART (TELEM2) |
| `serial:/dev/serial0:921600` | Raspberry Pi UART |
| `udp:127.0.0.1:14540` | mavlink-router or PX4 SITL on this host |
| `tcp:127.0.0.1:5760` | ArduPilot SITL |
| `udpin:` / `udpout:` | Explicit UDP direction |

IPv6 address literals are not supported; use IPv4 or a hostname.

Default identity is **onboard computer**: `source_system=1`, `source_component=191`, heartbeat `MAV_TYPE_ONBOARD_CONTROLLER`, about 1 Hz while you call `telemetry` / `wait_telemetry` / `spin`. Laptop GCS: `source_system=255, source_component=190`.

`takeoff` / `goto` / `land` return when the vehicle ACKs the command, not when the manoeuvre finishes. Use `wait_telemetry` (blocks on recv, not `sleep`). If the predicate is still false when the timeout expires, that is `TelemetryTimeoutError` — do not continue as success. Arm denial: check pre-arm in QGroundControl; this library never force-arms. After a drop, call `reconnect()`; this library does not enable pymavlink `autoreconnect`.

## Architecture

```
app / examples
    └── FlightController
            └── MavlinkFlightController (pymavlink)
                    ├── URI: udp / tcp / serial
                    └── PX4 / ArduPilot × Copter / Plane / VTOL mode maps
```

`HEARTBEAT.autopilot`: 12 = PX4, 3 = ArduPilot. `HEARTBEAT.type` selects `COPTER` / `PLANE` / `VTOL`.

Semantic modes: `MANUAL`, `HOLD`, `MISSION`, `RTL`, `LAND`, `TAKEOFF`, `GUIDED`.

PX4: `custom_mode = (sub_mode << 24) | (main_mode << 16)` (same packing for all airframes). ArduPilot uses integer mode ids per vehicle class (ArduCopter GUIDED=4; ArduPlane GUIDED=15).

## Layout

```
src/drone_fc/     API (English docstrings) + implementation comments in Chinese
examples/         customer-facing scripts (English)
tests/            unit tests (no hardware)
CHANGELOG.md            0.2 preview notes
docs/需求文档.md        internal product notes (Chinese)
docs/供应商集成说明.md  vendor handoff (Chinese)
docs/已知限制.md        verified matrix + device-side checklist (USB / TELEM2 / UART / UDP)
LICENSE                 proprietary; pymavlink is LGPLv3
```

## Environment

Python 3.10+ (3.13 verified).

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[plot]"
```

After `pip install -e .` you do not need `PYTHONPATH`. `pyserial` is a core dependency (USB/UART). Plotting is optional: omit `[plot]` if you do not run `live_telemetry_plot.py`.

`pip install -r requirements.txt` installs the same core deps (not matplotlib).

## Protocol mock (no vehicle)

```bash
python examples/live_telemetry_plot.py --no-show
# other terminal:
python examples/mock_px4_mavlink.py
python examples/check_connection.py
python examples/sitl_takeoff_land.py --hover-s 2 --skip-gps-check
```

## Units

- Lat/lon: WGS84 degrees
- Altitude: metres above home (`GLOBAL_POSITION_INT.relative_alt`)
- Attitude: degrees (`rad * 180/π`)
- Battery: volts / amperes

Plot offsets from home:

```
north_m = (lat - home_lat) × 111320
east_m  = (lon - home_lon) × 111320 × cos(home_lat)
```

## Tests

Three layers. Layer 1 needs no pymavlink. Layer 2 needs pymavlink and starts [`examples/mock_px4_mavlink.py`](examples/mock_px4_mavlink.py); it is skipped if pymavlink is missing. Layer 3 is a real vehicle (props off).

```bash
# Layer 1 — FakeTransport state machine (no sockets)
python -m unittest tests.test_link -v

# Layer 2 — real UDP: connect, empty-port timeout, GCS-only HEARTBEAT,
# missing serial, kill mock → LinkLostError → reconnect
python -m unittest tests.test_link_udp -v

# All unit tests (layer 2 included when pymavlink is installed)
python -m unittest discover -s tests -v
```

Manual layer 2 (expected English diagnostics):

```bash
python examples/mock_px4_mavlink.py --port 14550
python examples/check_connection.py --uri udp:127.0.0.1:14550 --timeout 5
# Ctrl+C the mock, then:
python examples/check_connection.py --uri udp:127.0.0.1:14550 --timeout 2
# expected: link failed, packets_seen printed, fc_heartbeats=0
```

Layer 3 (optional, props off): run [docs/已知限制.md](docs/已知限制.md) section 5 per cable (USB / TELEM2 / UART / UDP). `python examples/companion_check_link.py --uri serial:/dev/ttyACM0:57600` proves HEARTBEAT on that URI. Unplug while a script is still in `wait_telemetry` / `spin` / `telemetry` → `LinkLostError`; plug back on the same node → `reconnect()`, or `reconnect(uri=...)` if the device path changed. That stock link-check script exits after one snapshot, so it will not sit there waiting for an unplug.

## API

| Method | Role |
|--------|------|
| `connect(timeout_s)` | Open transport **and** wait for autopilot HEARTBEAT |
| `reconnect(timeout_s, uri=None)` | Close, open again, wait for HEARTBEAT; optional new URI after USB re-enum |
| `wait_heartbeat(timeout_s)` | Heartbeat only (already used by `connect`) |
| `telemetry()` | Non-blocking snapshot (firmware, airframe, capabilities, pose, battery, GPS) |
| `wait_telemetry(predicate, timeout_s)` | Block on recv until predicate; unmet → `TelemetryTimeoutError` |
| `spin(duration_s)` | Pump recv for a duration (hover timing); does not use wait_telemetry timeout |
| `arm()` / `disarm()` | Arm / disarm |
| `set_mode(mode)` | Semantic mode |
| `takeoff(altitude_m)` | Takeoff command (copter/VTOL ≤100 m, plane ≤500 m) |
| `goto(lat, lon, alt_m)` | Go to a point |
| `land()` / `rtl()` | Land / RTL (plane has no `land`) |
| `upload_mission` / `start_mission` / `pause_mission` / `clear_mission` | Waypoint list — **interface only in 0.2** |
| `set_position` / `set_velocity` | Guided/Offboard setpoints — **interface only in 0.2** |
| `set_gimbal` / `center_gimbal` | Gimbal pointing, not video — **interface only in 0.2** |
| `disconnect()` | Close |

Failures raise `drone_fc` errors (`TransportOpenError`, `HeartbeatTimeoutError`, `LinkLostError`, `TelemetryTimeoutError`, `ArmDeniedError`, `GpsNotFixedError`, …).
