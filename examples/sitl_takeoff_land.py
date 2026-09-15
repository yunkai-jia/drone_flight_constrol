#!/usr/bin/env python3
"""PX4 / ArduPilot SITL 示例：连接 → 心跳 → 起飞 → 短悬停 → 降落。

本脚本只演示民用飞控接口，不涉及任务分配或打击。

PX4 SITL（默认本脚本 URI）::

    在 PX4-Autopilot 源码目录: make px4_sitl gz_x500
    或使用已有仿真，确保 MAVLink 出现在 UDP 14540。

ArduPilot SITL 示例::

    sim_vehicle.py -v ArduCopter --console --map
    PYTHONPATH=src python examples/sitl_takeoff_land.py --uri tcp:127.0.0.1:5760

运行::

    pip install -r requirements.txt
    PYTHONPATH=src python examples/sitl_takeoff_land.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drone_fc import ConnectionConfig, MavlinkFlightController  # noqa: E402


def parse_args() -> argparse.Namespace:
    """解析起飞高度与悬停时间。altitude ∈ (0, 100]，hover-s > 0。"""
    parser = argparse.ArgumentParser(description="SITL 起飞降落演示")
    parser.add_argument(
        "--uri",
        default="udp:127.0.0.1:14540",
        help="连接串，默认 PX4 SITL UDP 14540",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=5.0,
        help="起飞相对高度（米），范围 (0, 100]，默认 5",
    )
    parser.add_argument(
        "--hover-s",
        type=float,
        default=8.0,
        help="起飞后悬停秒数，必须 > 0，默认 8",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = ConnectionConfig(uri=args.uri, skip_gps_check=True)
    fc = MavlinkFlightController(config)
    fc.connect(timeout_s=10.0)
    try:
        autopilot = fc.wait_heartbeat(timeout_s=15.0)
        snap = fc.telemetry()
        print(f"已连接固件: {autopilot.value}")
        print(f"模式={snap.mode.value} 解锁={snap.armed} GPS={snap.gps_fix}")
        print(f"起飞到 {args.altitude} m …")
        fc.takeoff(args.altitude)
        hover = args.hover_s
        if hover <= 0:
            raise SystemExit("--hover-s 必须 > 0")
        time.sleep(hover)
        snap = fc.telemetry()
        if snap.position is not None:
            print(
                f"当前位置: lat={snap.position.latitude_deg:.6f} "
                f"lon={snap.position.longitude_deg:.6f} "
                f"alt={snap.position.altitude_m:.2f} m"
            )
        print("降落 …")
        fc.land()
        time.sleep(5.0)
        print("完成")
        return 0
    finally:
        fc.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
