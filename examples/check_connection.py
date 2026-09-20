#!/usr/bin/env python3
"""Link check: connect() waits for HEARTBEAT, then print telemetry.

Direct vehicle (USB)::

    python examples/check_connection.py --uri serial:/dev/ttyACM0:57600

Protocol mock (optional, no hardware)::

    python examples/mock_px4_mavlink.py
    python examples/check_connection.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drone_fc import (  # noqa: E402
    ConnectionConfig,
    DroneFcError,
    HeartbeatTimeoutError,
    LinkLostError,
    MavlinkFlightController,
    TelemetryTimeoutError,
    TransportOpenError,
)


def _print_link_failure(exc: BaseException) -> None:
    """Print English diagnostics for connect / heartbeat / open failures."""

    print(f"link failed: {exc}", flush=True)
    if isinstance(exc, HeartbeatTimeoutError):
        print(
            f"  packets_seen={exc.packets_seen} fc_heartbeats={exc.fc_heartbeats} "
            f"other_heartbeats={exc.other_heartbeats} last_msg_type={exc.last_msg_type}",
            flush=True,
        )
    elif isinstance(exc, LinkLostError):
        print(f"  age_s={exc.age_s}", flush=True)
    elif isinstance(exc, TelemetryTimeoutError):
        print(f"  timeout_s={exc.timeout_s}", flush=True)
    elif isinstance(exc, TransportOpenError):
        print(f"  uri={exc.uri} device={exc.device}", flush=True)


def parse_args() -> argparse.Namespace:
    """Parse CLI. timeout must be > 0."""
    parser = argparse.ArgumentParser(description="Check MAVLink link to the flight controller")
    parser.add_argument(
        "--uri",
        default="udp:127.0.0.1:14540",
        help="Connection URI, e.g. serial:/dev/ttyACM0:57600 or udp:127.0.0.1:14540",
    )
    parser.add_argument("--timeout", type=float, default=8.0, help="Seconds to wait for HEARTBEAT, must be > 0")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be > 0")
    fc = MavlinkFlightController(ConnectionConfig(uri=args.uri, skip_gps_check=False))
    try:
        autopilot = fc.connect(timeout_s=args.timeout)
        fc.spin(min(2.0, args.timeout))
        snap = fc.telemetry()
        print("Link OK")
        print(f"  firmware: {autopilot.value}")
        print(f"  airframe: {snap.vehicle_class.value if snap.vehicle_class else None}")
        if snap.capabilities is not None:
            print("  capabilities: " + ", ".join(sorted(c.value for c in snap.capabilities.supported)))
        print(f"  mode: {snap.mode.value}")
        print(f"  armed: {snap.armed}")
        print(f"  GPS: {snap.gps_fix}")
        if snap.position is not None:
            print(
                f"  position: {snap.position.latitude_deg:.6f}, "
                f"{snap.position.longitude_deg:.6f}, "
                f"alt={snap.position.altitude_m:.2f} m"
            )
        if snap.battery is not None:
            print(f"  voltage: {snap.battery.voltage_v} V  remaining: {snap.battery.remaining_percent}%")
        return 0
    except DroneFcError as exc:
        _print_link_failure(exc)
        return 1
    finally:
        fc.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
