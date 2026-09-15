#!/usr/bin/env python3
"""简易遥测可视化：水平轨迹（东/北，米）+ 高度曲线。

默认会拉起协议级模拟端，再执行 起飞 → 平移 → 降落，并用 matplotlib 实时刷新。
本机有显示器时弹出窗口；无界面时加 ``--no-show`` 只存 PNG。

运行::

    pip install -r requirements.txt
    PYTHONPATH=src python examples/live_telemetry_plot.py

已有模拟端在跑时::

    PYTHONPATH=src python examples/live_telemetry_plot.py --no-mock
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

from drone_fc import ConnectionConfig, MavlinkFlightController
from drone_fc.types import Telemetry

_METERS_PER_DEG_LAT = 111320.0


def _setup_fonts() -> None:
    """优先用系统中文字体，避免标题变成方框。"""

    rcParams["font.sans-serif"] = [
        "PingFang SC",
        "Hiragino Sans GB",
        "Songti SC",
        "Heiti SC",
        "Arial Unicode MS",
        "DejaVu Sans",
    ]
    rcParams["axes.unicode_minus"] = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="遥测 2D 轨迹与高度图")
    parser.add_argument("--uri", default="udp:127.0.0.1:14540", help="飞控连接串")
    parser.add_argument("--altitude", type=float, default=8.0, help="起飞相对高（米），(0, 100]")
    parser.add_argument(
        "--save",
        default=str(ROOT / "examples" / "last_telemetry_plot.png"),
        help="结束时保存 PNG 的路径",
    )
    parser.add_argument("--no-show", action="store_true", help="不弹窗，适合无显示器环境")
    parser.add_argument("--no-mock", action="store_true", help="不启动内置模拟端，使用已有 SITL")
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
        raise SystemExit("--altitude 必须在 (0, 100]")

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
        time.sleep(0.5)

    fc = MavlinkFlightController(ConnectionConfig(uri=args.uri, skip_gps_check=True))
    t0 = time.monotonic()
    times: list[float] = []
    easts: list[float] = []
    norths: list[float] = []
    alts: list[float] = []
    home_lat: float | None = None
    home_lon: float | None = None

    fig, (ax_map, ax_alt) = plt.subplots(1, 2, figsize=(11, 4.8))
    fig.suptitle("民用飞控遥测（协议模拟）")
    (line_path,) = ax_map.plot([], [], "-", color="C0", lw=1.5, label="轨迹")
    (dot_now,) = ax_map.plot([], [], "o", color="C3", ms=8, label="当前")
    ax_map.set_xlabel("东向 (m)")
    ax_map.set_ylabel("北向 (m)")
    ax_map.set_title("水平轨迹")
    ax_map.grid(True, alpha=0.3)
    ax_map.legend(loc="upper right")
    (line_alt,) = ax_alt.plot([], [], "-", color="C1", lw=1.5)
    ax_alt.set_xlabel("时间 (s)")
    ax_alt.set_ylabel("相对高度 (m)")
    ax_alt.set_title("高度")
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
        """循环读遥测、刷新图，直到条件成立或超时。

        输入:
            predicate: 接收 Telemetry，为 True 则结束。
            timeout_s: 最长等待秒数，必须 > 0。
            label: 完成/超时时打印的阶段名。
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            snap = fc.telemetry()
            sample(snap)
            redraw()
            if predicate(snap):
                print(label, "完成", flush=True)
                return
            time.sleep(0.08)
        print(label, "超时，继续", flush=True)

    try:
        fc.connect(timeout_s=5.0)
        print("固件:", fc.wait_heartbeat(timeout_s=8.0).value, flush=True)
        time.sleep(0.3)
        start = fc.telemetry()
        sample(start)
        redraw()
        if home_lat is None or home_lon is None:
            raise SystemExit("未读到位置，无法绘图")

        print(f"起飞到 {args.altitude} m …", flush=True)
        fc.takeoff(args.altitude)
        poll_until(
            lambda s: s.position is not None
            and s.position.altitude_m is not None
            and s.position.altitude_m >= args.altitude * 0.7,
            12.0,
            "爬升",
        )

        east_lon = _offset_lon(home_lat, home_lon, 25.0)
        north_lat = _offset_lat(home_lat, 15.0)
        print("前往东 25 m …", flush=True)
        fc.goto(home_lat, east_lon, args.altitude)
        poll_until(
            lambda s: s.position is not None
            and _east_north_m(s.position.latitude_deg, s.position.longitude_deg, home_lat, home_lon)[0]
            >= 18.0,
            12.0,
            "东移",
        )
        print("前往北 15 m …", flush=True)
        fc.goto(north_lat, east_lon, args.altitude)
        poll_until(
            lambda s: s.position is not None
            and _east_north_m(s.position.latitude_deg, s.position.longitude_deg, home_lat, home_lon)[1]
            >= 10.0,
            12.0,
            "北移",
        )

        print("降落 …", flush=True)
        fc.land()
        poll_until(
            lambda s: s.position is not None
            and s.position.altitude_m is not None
            and s.position.altitude_m <= 0.4,
            15.0,
            "降落",
        )
        save_path = Path(args.save)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=120)
        print(f"已保存 {save_path}", flush=True)
        if not args.no_show:
            plt.ioff()
            plt.show()
        return 0
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
