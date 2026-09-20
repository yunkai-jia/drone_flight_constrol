#!/usr/bin/env python3
"""Vendor skeleton: one script for every PX4/ArduPilot airframe.

Change only the URI. Inspect capabilities before takeoff or land.
Does not force-arm. Remove propellers until you intend to fly.

    python examples/vendor_same_control.py --uri serial:/dev/ttyACM0:57600 --dry-run
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
    Capability,
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
    """URI, heartbeat timeout, and whether to command takeoff."""

    parser = argparse.ArgumentParser(description="Same control path for vendor airframes")
    parser.add_argument(
        "--uri",
        default="udp:127.0.0.1:14540",
        help="serial:/dev/ttyACM0:57600 or udp:127.0.0.1:14540",
    )
    parser.add_argument("--timeout", type=float, default=12.0, help="HEARTBEAT timeout, must be > 0")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Identify firmware, airframe, and capabilities; do not arm or takeoff",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be > 0")

    fc = MavlinkFlightController(ConnectionConfig(uri=args.uri, skip_gps_check=False))
    try:
        fc.connect(timeout_s=args.timeout)
        snap = fc.telemetry()
        caps = snap.capabilities
        print(f"firmware: {snap.autopilot.value if snap.autopilot else None}")
        print(f"airframe: {snap.vehicle_class.value if snap.vehicle_class else None}")
        if caps is None:
            print("capabilities: none (no heartbeat)")
            return 1
        print("capabilities:", ", ".join(sorted(c.value for c in caps.supported)))

        if args.dry_run:
            print("dry-run: not arming")
            return 0

        if not caps.supports(Capability.TAKEOFF):
            print("this airframe cannot takeoff through this SDK")
            return 1
        fc.takeoff(5.0)
        fc.wait_telemetry(
            lambda s: s.position is not None
            and s.position.altitude_m is not None
            and s.position.altitude_m >= 3.5,
            25.0,
        )
        if caps.supports(Capability.LAND):
            fc.land()
        else:
            fc.rtl()
        return 0
    except DroneFcError as exc:
        _print_link_failure(exc)
        return 1
    finally:
        fc.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
