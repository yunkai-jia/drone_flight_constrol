#!/usr/bin/env python3
"""Companion link check: serial (or UDP) → HEARTBEAT → telemetry. Does not arm.

Remove propellers first. Direct URIs::

    serial:/dev/ttyTHS1:921600    Jetson UART
    serial:/dev/serial0:921600    Raspberry Pi UART
    serial:/dev/ttyACM0:57600     FC USB
    udp:127.0.0.1:14540           local mavlink-router

    python examples/companion_check_link.py --uri serial:/dev/ttyACM0:57600

Failsafe after this process dies is entirely the autopilot RC / onboard-loss
parameters. This library does not replace a radio.
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
    GpsFixType,
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
    """Parse serial URI and heartbeat timeout. timeout must be > 0."""

    parser = argparse.ArgumentParser(
        description="Companion link check (no arm). Remove propellers first."
    )
    parser.add_argument(
        "--uri",
        default="serial:/dev/ttyTHS1:921600",
        help="Connection URI, default Jetson TELEM2 at 921600",
    )
    parser.add_argument("--timeout", type=float, default=12.0, help="Seconds to wait for HEARTBEAT, must be > 0")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be > 0")

    print("Props off. This script will not arm.", flush=True)
    config = ConnectionConfig(uri=args.uri, skip_gps_check=False)
    fc = MavlinkFlightController(config)
    try:
        autopilot = fc.connect(timeout_s=args.timeout)
        fc.spin(2.0)
        snap = fc.telemetry()
        print("Link OK (disarmed)", flush=True)
        print(f"  firmware: {autopilot.value}", flush=True)
        print(
            f"  airframe: {snap.vehicle_class.value if snap.vehicle_class else None}",
            flush=True,
        )
        if snap.capabilities is not None:
            print(
                "  capabilities: "
                + ", ".join(sorted(c.value for c in snap.capabilities.supported)),
                flush=True,
            )
        print(
            f"  local: sys={config.source_system} comp={config.source_component} (onboard)",
            flush=True,
        )
        print(f"  mode: {snap.mode.value}", flush=True)
        print(f"  armed: {snap.armed}", flush=True)
        print(f"  GPS: {snap.gps_fix}  sats: {snap.satellites_visible}", flush=True)
        if snap.position is not None:
            print(
                f"  position: {snap.position.latitude_deg:.6f}, "
                f"{snap.position.longitude_deg:.6f}, "
                f"alt={snap.position.altitude_m:.2f} m",
                flush=True,
            )
        if snap.battery is not None:
            print(
                f"  voltage: {snap.battery.voltage_v} V  remaining: {snap.battery.remaining_percent}%",
                flush=True,
            )
        if snap.gps_fix is None or snap.gps_fix.value < GpsFixType.FIX_3D.value:
            print("  note: no 3D GPS yet; do not set skip_gps_check for outdoor takeoff.", flush=True)
        return 0
    except DroneFcError as exc:
        _print_link_failure(exc)
        return 1
    finally:
        fc.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
