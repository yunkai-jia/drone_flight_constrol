#!/usr/bin/env python3
"""2D telemetry plot: east/north path (m) plus altitude vs time.

By default starts the protocol mock, then takeoff → translate → land.
Use ``--no-mock`` to talk to a real SITL or vehicle on ``--uri``.
Use ``--no-show`` to save PNG without a window.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import matplotlib
from matplotlib import rcParams

from drone_fc import (  # noqa: E402
    ConnectionConfig,
    DroneFcError,
    HeartbeatTimeoutError,
    LinkLostError,
    MavlinkFlightController,
    TelemetryTimeoutError,
    TransportOpenError,
)
from drone_fc.types import Telemetry

_METERS_PER_DEG_LAT = 111320.0


def _setup_fonts() -> None:
    """Prefer a system sans-serif so ASCII titles render; keep the minus sign visible."""

    rcParams["font.sans-serif"] = [
        "DejaVu Sans",
        "Arial",
        "Helvetica",
        "sans-serif",
    ]
    rcParams["axes.unicode_minus"] = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2D telemetry path and altitude plot")
    parser.add_argument("--uri", default="udp:127.0.0.1:14540", help="Vehicle URI")
    parser.add_argument("--altitude", type=float, default=8.0, help="Takeoff relative altitude in metres, (0, 100]")
    parser.add_argument(
        "--save",
        default=str(ROOT / "examples" / "last_telemetry_plot.png"),
        help="PNG path written at the end",
    )
    parser.add_argument("--no-show", action="store_true", help="No window (headless)")
    parser.add_argument("--no-mock", action="store_true", help="Do not spawn the protocol mock; use --uri as-is")
    return parser.parse_args()


def _east_north_m(lat: float, lon: float, home_lat: float, home_lon: float) -> tuple[float, float]:
    """把经纬度换成相对 Home 的东向/北向米。

    北向: ``(lat - home_lat) * 111320``
    东向: ``(lon - home_lon) * 111320 * cos(home_lat)``
    """

    north = (lat - home_lat) * _METERS_PER_DEG_LAT
    east = (lon - home_lon) * _METERS_PER_DEG_LAT * math.cos(math.radians(home_lat))
    return east, north


def _offset_lon(home_lat: float, home_lon: float, east_m: float) -> float:
    """Home 点向东 east_m 米后的经度。"""

    return home_lon + east_m / (_METERS_PER_DEG_LAT * math.cos(math.radians(home_lat)))


def _offset_lat(home_lat: float, north_m: float) -> float:
    """Home 点向北 north_m 米后的纬度。"""

    return home_lat + north_m / _METERS_PER_DEG_LAT


def main() -> int:
    args = parse_args()
    if args.altitude <= 0 or args.altitude > 100:
        raise SystemExit("--altitude must be in (0, 100]")

    if args.no_show:
        matplotlib.use("Agg")
    _setup_fonts()
    import matplotlib.pyplot as plt

    mock_proc: subprocess.Popen[str] | None = None
    if not args.no_mock:
        mock_script = ROOT / "examples" / "mock_px4_mavlink.py"
        env = os.environ.copy()
        env["PYTHONPATH"] = str(SRC) + os.pathsep + env.get("PYTHONPATH", "")
        mock_proc = subprocess.Popen(
            [sys.executable, str(mock_script)],
            cwd=str(ROOT),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # mock 进程随后发心跳；connect() 会阻塞等到 HEARTBEAT，不必 sleep

    fc = MavlinkFlightController(ConnectionConfig(uri=args.uri, skip_gps_check=True))
    t0 = time.monotonic()
    times: list[float] = []
    easts: list[float] = []
    norths: list[float] = []
    alts: list[float] = []
    home_lat: float | None = None
    home_lon: float | None = None

    fig, (ax_map, ax_alt) = plt.subplots(1, 2, figsize=(11, 4.8))
    fig.suptitle("Flight telemetry")
    (line_path,) = ax_map.plot([], [], "-", color="C0", lw=1.5, label="path")
    (dot_now,) = ax_map.plot([], [], "o", color="C3", ms=8, label="now")
    ax_map.set_xlabel("East (m)")
    ax_map.set_ylabel("North (m)")
    ax_map.set_title("Horizontal path")
    ax_map.grid(True, alpha=0.3)
    ax_map.legend(loc="upper right")
    (line_alt,) = ax_alt.plot([], [], "-", color="C1", lw=1.5)
    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("Relative altitude (m)")
    ax_alt.set_title("Altitude")
    ax_alt.grid(True, alpha=0.3)
    fig.tight_layout()
    if not args.no_show:
        plt.ion()
        plt.show(block=False)

    def redraw() -> None:
        """用当前采样点更新折线；无显示器时只更新 figure 对象。"""
        line_path.set_data(easts, norths)
        if easts:
            dot_now.set_data([easts[-1]], [norths[-1]])
        ax_map.relim()
        ax_map.autoscale_view()
        ax_map.set_aspect("equal", adjustable="datalim")
        line_alt.set_data(times, alts)
        ax_alt.relim()
        ax_alt.autoscale_view()
        fig.canvas.draw_idle()
        if not args.no_show:
            fig.canvas.flush_events()
            plt.pause(0.05)

    def sample(snap: Telemetry) -> None:
        """把一次遥测追加到轨迹数组。第一帧位置记为 Home。"""
        nonlocal home_lat, home_lon
        if snap.position is None or snap.position.altitude_m is None:
            return
        if home_lat is None:
            home_lat = snap.position.latitude_deg
            home_lon = snap.position.longitude_deg
        east, north = _east_north_m(
            snap.position.latitude_deg,
            snap.position.longitude_deg,
            home_lat,
            home_lon,
        )
        times.append(time.monotonic() - t0)
        easts.append(east)
        norths.append(north)
        alts.append(snap.position.altitude_m)

    def poll_until(predicate, timeout_s: float, label: str) -> None:
        """Block on recv via wait_telemetry and redraw the plot."""

        def wrapped(snap: Telemetry) -> bool:
            sample(snap)
            redraw()
            return predicate(snap)

        fc.wait_telemetry(wrapped, timeout_s)
        print(label, "done", flush=True)

    try:
        autopilot = fc.connect(timeout_s=8.0)
        print("firmware:", autopilot.value, flush=True)
        start = fc.wait_telemetry(lambda s: s.position is not None, 3.0)
        sample(start)
        redraw()
        if home_lat is None or home_lon is None:
            raise SystemExit("no position yet; cannot plot")

        print(f"takeoff to {args.altitude} m …", flush=True)
        fc.takeoff(args.altitude)
        poll_until(
            lambda s: s.position is not None
            and s.position.altitude_m is not None
            and s.position.altitude_m >= args.altitude * 0.7,
            12.0,
            "climb",
        )

        east_lon = _offset_lon(home_lat, home_lon, 25.0)
        north_lat = _offset_lat(home_lat, 15.0)
        print("goto east 25 m …", flush=True)
        fc.goto(home_lat, east_lon, args.altitude)
        poll_until(
            lambda s: s.position is not None
            and _east_north_m(s.position.latitude_deg, s.position.longitude_deg, home_lat, home_lon)[0]
            >= 18.0,
            12.0,
            "east",
        )
        print("goto north 15 m …", flush=True)
        fc.goto(north_lat, east_lon, args.altitude)
        poll_until(
            lambda s: s.position is not None
            and _east_north_m(s.position.latitude_deg, s.position.longitude_deg, home_lat, home_lon)[1]
            >= 10.0,
            12.0,
            "north",
        )

        print("landing …", flush=True)
        fc.land()
        poll_until(
            lambda s: s.position is not None
            and s.position.altitude_m is not None
            and s.position.altitude_m <= 0.4,
            15.0,
            "land",
        )
        save_path = Path(args.save)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=120)
        print(f"saved {save_path}", flush=True)
        if not args.no_show:
            plt.ioff()
            plt.show()
        return 0
    except DroneFcError as exc:
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
        return 1
    finally:
        fc.disconnect()
        if mock_proc is not None:
            mock_proc.terminate()
            try:
                mock_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                mock_proc.kill()
        plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
