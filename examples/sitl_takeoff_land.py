#!/usr/bin/env python3
"""Takeoff → wait for altitude on telemetry → hover (recv pump) → land.

Waits on MAVLink recv, not time.sleep. For SITL or the protocol mock only.
Do **not** use this script to take off a real vehicle.

PX4 SITL::

    make px4_sitl gz_x500
    python examples/sitl_takeoff_land.py --skip-gps-check

Protocol mock::

    python examples/mock_px4_mavlink.py
    python examples/sitl_takeoff_land.py --hover-s 2 --skip-gps-check

ArduPilot SITL::

    sim_vehicle.py -v ArduCopter --console --map
    python examples/sitl_takeoff_land.py --uri tcp:127.0.0.1:5760 --skip-gps-check
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
from drone_fc.types import Telemetry  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Parse altitude and hover duration. altitude ∈ (0, 100], hover-s > 0."""
    parser = argparse.ArgumentParser(description="SITL takeoff/land (wait on telemetry)")
    parser.add_argument(
        "--uri",
        default="udp:127.0.0.1:14540",
        help="Connection URI, default PX4 SITL UDP 14540",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=5.0,
        help="Takeoff relative altitude in metres, (0, 100], default 5",
    )
    parser.add_argument(
        "--hover-s",
        type=float,
        default=8.0,
        help="Extra hover seconds after reaching altitude, must be > 0, default 8",
    )
    parser.add_argument(
        "--skip-gps-check",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Skip 3D GPS before takeoff. Required for typical SITL/mock. "
            "Default False — never use --skip-gps-check on a real aircraft"
        ),
    )
    return parser.parse_args()


def _print_link_failure(exc: BaseException) -> None:
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


def main() -> int:
    args = parse_args()
    if args.altitude <= 0 or args.altitude > 100:
        raise SystemExit("--altitude must be in (0, 100]")
    hover = args.hover_s
    if hover <= 0:
        raise SystemExit("--hover-s must be > 0")

    config = ConnectionConfig(uri=args.uri, skip_gps_check=args.skip_gps_check)
    fc = MavlinkFlightController(config)
    try:
        autopilot = fc.connect(timeout_s=15.0)
        snap = fc.telemetry()
        print(f"connected: {autopilot.value}")
        print(f"mode={snap.mode.value} armed={snap.armed} GPS={snap.gps_fix}")
        print(f"takeoff to {args.altitude} m …")
        fc.takeoff(args.altitude)

        # 示例用 70% 宽松判断爬升完成（mock/SITL 仿真速度考虑）；真机应等到
        # 100% 目标高度且检查下降趋势再判定到位。
        def climbed(s: Telemetry) -> bool:
            return (
                s.position is not None
                and s.position.altitude_m is not None
                and s.position.altitude_m >= args.altitude * 0.7
            )

        snap = fc.wait_telemetry(climbed, 20.0)
        print("climb done", flush=True)
        snap = fc.spin(hover)
        if snap.position is not None:
            print(
                f"position: lat={snap.position.latitude_deg:.6f} "
                f"lon={snap.position.longitude_deg:.6f} "
                f"alt={snap.position.altitude_m:.2f} m"
            )
        print("landing …")
        fc.land()

        def on_ground(s: Telemetry) -> bool:
            return (
                s.position is not None
                and s.position.altitude_m is not None
                and s.position.altitude_m <= 0.4
            )

        snap = fc.wait_telemetry(on_ground, 25.0)
        print("land done", flush=True)
        print("done")
        return 0
    except DroneFcError as exc:
        _print_link_failure(exc)
        return 1
    finally:
        fc.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
