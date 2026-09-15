#!/usr/bin/env python3
"""连通性检查：连接 → 等待心跳 → 打印遥测后断开。

需要先启动模拟端或官方 PX4 SITL::

    PYTHONPATH=src python examples/mock_px4_mavlink.py
    PYTHONPATH=src python examples/check_connection.py
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
    """解析命令行。timeout 必须 > 0，默认 8 秒。"""
    parser = argparse.ArgumentParser(description="检查飞控 MAVLink 是否连通")
    parser.add_argument("--uri", default="udp:127.0.0.1:14540", help="连接串")
    parser.add_argument("--timeout", type=float, default=8.0, help="等待心跳秒数，必须 > 0")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout 必须 > 0")
    fc = MavlinkFlightController(ConnectionConfig(uri=args.uri, skip_gps_check=True))
    fc.connect(timeout_s=5.0)
    try:
        autopilot = fc.wait_heartbeat(timeout_s=args.timeout)
        # 等一小段让 GPS / 位置包进缓冲，再读遥测
        time.sleep(0.4)
        snap = fc.telemetry()
        print("连通成功")
        print(f"  固件: {autopilot.value}")
        print(f"  模式: {snap.mode.value}")
        print(f"  解锁: {snap.armed}")
        print(f"  GPS: {snap.gps_fix}")
        if snap.position is not None:
            print(
                f"  位置: {snap.position.latitude_deg:.6f}, "
                f"{snap.position.longitude_deg:.6f}, "
                f"alt={snap.position.altitude_m:.2f} m"
            )
        if snap.battery is not None:
            print(f"  电压: {snap.battery.voltage_v} V  电量: {snap.battery.remaining_percent}%")
        return 0
    finally:
        fc.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
