#!/usr/bin/env python3
"""协议级 PX4 模拟端：在本机没有官方 SITL 时，验证客户端能否连通。

本进程扮演飞控：向 GCS 端口发送 HEARTBEAT / 位置 / GPS，并对 SET_MODE、
COMMAND_LONG 回 COMMAND_ACK。它不是物理仿真，也不包含任务或武器逻辑。

默认对端与客户端一致：客户端 ``udp:127.0.0.1:14540``（监听），本端
``udpout:127.0.0.1:14540``（发送）。

水平位置按球面近似移动（1 度纬度约 111320 m），供 2D 轨迹图使用。

运行::

    PYTHONPATH=src python examples/mock_px4_mavlink.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pymavlink import mavutil  # noqa: E402

from drone_fc.mavlink.modes import (  # noqa: E402
    PX4_MAIN_AUTO,
    PX4_SUB_AUTO_LAND,
    PX4_SUB_AUTO_LOITER,
    PX4_SUB_AUTO_RTL,
    decode_px4_custom_mode,
    encode_px4_custom_mode,
)

_HOME_LAT = 47.397742  # 与 PX4 SITL 默认 Home（苏黎世附近）一致，便于对照
_HOME_LON = 8.545594
_METERS_PER_DEG_LAT = 111320.0  # 纬度 1 度约等于的地面距离（米）
_CLIMB_MPS = 2.0
_DESCEND_MPS = 1.5
_CRUISE_MPS = 8.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="协议级 PX4 MAVLink 模拟端")
    parser.add_argument("--host", default="127.0.0.1", help="客户端监听地址")
    parser.add_argument("--port", type=int, default=14540, help="客户端监听端口")
    return parser.parse_args()


def _lon_meters_per_deg(lat_deg: float) -> float:
    """当前纬度下，经度 1 度对应的东向米数：111320 * cos(lat)。"""

    return _METERS_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def _step_horizontal(
    lat: float,
    lon: float,
    target_lat: float,
    target_lon: float,
    dt: float,
) -> tuple[float, float, float]:
    """向目标点平移，返回 (lat, lon, yaw_rad)。

    yaw 为北偏东弧度：0 朝北，正值朝东。速度 _CRUISE_MPS 米/秒。
    """

    north_m = (target_lat - lat) * _METERS_PER_DEG_LAT
    east_m = (target_lon - lon) * _lon_meters_per_deg(lat)
    dist = math.hypot(north_m, east_m)
    yaw = math.atan2(east_m, north_m) if dist > 0.05 else 0.0
    if dist < 0.5:
        return target_lat, target_lon, yaw
    step = min(_CRUISE_MPS * max(dt, 0.0), dist)
    lat += (north_m / dist) * step / _METERS_PER_DEG_LAT
    lon += (east_m / dist) * step / _lon_meters_per_deg(lat)
    return lat, lon, yaw


def main() -> int:
    args = parse_args()
    if args.port < 1 or args.port > 65535:
        raise SystemExit("port 必须在 [1, 65535]")

    device = f"udpout:{args.host}:{args.port}"
    conn = mavutil.mavlink_connection(device, source_system=1, source_component=1)
    custom_mode = encode_px4_custom_mode(PX4_MAIN_AUTO, PX4_SUB_AUTO_LOITER)
    armed = False
    relative_alt_m = 0.0
    target_alt_m = 0.0
    lat = _HOME_LAT
    lon = _HOME_LON
    target_lat = _HOME_LAT
    target_lon = _HOME_LON
    yaw_rad = 0.0
    landing = False

    print(f"模拟端已启动，正在向 {device} 发送心跳（Ctrl+C 结束）", flush=True)
    last_telem = 0.0
    last_motion = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            dt = now - last_motion  # 上一圈到现在的秒数，用于匀速积分
            last_motion = now
            if landing:
                relative_alt_m = max(0.0, relative_alt_m - _DESCEND_MPS * dt)
                lat, lon, yaw_rad = _step_horizontal(lat, lon, target_lat, target_lon, dt)
                if relative_alt_m <= 0.05:
                    relative_alt_m = 0.0
                    landing = False
                    armed = False
            elif armed and target_alt_m > relative_alt_m:
                relative_alt_m = min(target_alt_m, relative_alt_m + _CLIMB_MPS * dt)
            if armed and not landing:
                lat, lon, yaw_rad = _step_horizontal(lat, lon, target_lat, target_lon, dt)

            if now - last_telem >= 0.2:
                last_telem = now
                _send_telemetry(
                    conn,
                    custom_mode,
                    armed,
                    lat,
                    lon,
                    relative_alt_m,
                    yaw_rad,
                    int(time.monotonic() * 1000) & 0xFFFFFFFF,
                )

            msg = conn.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                continue
            if mtype == "SET_MODE":
                custom_mode = int(msg.custom_mode)
                main_mode, sub_mode = decode_px4_custom_mode(custom_mode)
                if main_mode == PX4_MAIN_AUTO and sub_mode in (PX4_SUB_AUTO_LAND, PX4_SUB_AUTO_RTL):
                    landing = True
                    target_alt_m = 0.0
                    target_lat = _HOME_LAT
                    target_lon = _HOME_LON
                    print("SET_MODE LAND/RTL", flush=True)
                else:
                    print(f"SET_MODE custom_mode={custom_mode}", flush=True)
                continue
            if mtype == "SET_POSITION_TARGET_GLOBAL_INT":
                target_lat = msg.lat / 1e7
                target_lon = msg.lon / 1e7
                if math.isfinite(msg.alt) and msg.alt > 0:
                    target_alt_m = float(msg.alt)
                landing = False
                print(f"GOTO {target_lat:.6f},{target_lon:.6f} alt={target_alt_m:.1f}", flush=True)
                continue
            if mtype == "COMMAND_LONG":
                command = int(msg.command)
                result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    armed = float(msg.param1) >= 0.5
                    print(f"ARM={armed}", flush=True)
                elif command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    target_alt_m = float(msg.param7) if msg.param7 else 5.0
                    landing = False
                    print(f"TAKEOFF target={target_alt_m:.1f}m", flush=True)
                elif command in (
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                ):
                    landing = True
                    target_alt_m = 0.0
                    target_lat = _HOME_LAT
                    target_lon = _HOME_LON
                    print("LAND/RTL", flush=True)
                elif command == mavutil.mavlink.MAV_CMD_DO_REPOSITION:
                    if math.isfinite(msg.param5) and math.isfinite(msg.param6):
                        target_lat = float(msg.param5)
                        target_lon = float(msg.param6)
                    if math.isfinite(msg.param7) and msg.param7 > 0:
                        target_alt_m = float(msg.param7)
                    landing = False
                    print(f"DO_REPOSITION {target_lat:.6f},{target_lon:.6f}", flush=True)
                conn.mav.command_ack_send(command, result)
    except KeyboardInterrupt:
        print("模拟端结束", flush=True)
        return 0
    finally:
        conn.close()


def _send_telemetry(
    conn: mavutil.mavfile,
    custom_mode: int,
    armed: bool,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    yaw_rad: float,
    time_boot_ms: int,
) -> None:
    """按约 5 Hz 向外发一组飞控常用消息。

    输入:
        conn: pymavlink 连接（本端 sysid=1）。
        custom_mode: PX4 HEARTBEAT.custom_mode 位域。
        armed: True 时 HEARTBEAT 带 SAFETY_ARMED。
        lat_deg / lon_deg: WGS84 度。
        alt_m: 相对 Home 高度，米；内部再乘 1000 变成毫米。
        yaw_rad: 偏航弧度，北偏东为正。
        time_boot_ms: 启动后毫秒，写入 GLOBAL_POSITION_INT / ATTITUDE。
    """
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    state = mavutil.mavlink.MAV_STATE_ACTIVE if armed else mavutil.mavlink.MAV_STATE_STANDBY
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_PX4,
        base_mode,
        custom_mode,
        state,
    )
    lat = int(lat_deg * 1e7)  # GLOBAL_POSITION_INT / GPS_RAW_INT 使用 degE7
    lon = int(lon_deg * 1e7)
    alt_mm = int(alt_m * 1000)
    conn.mav.gps_raw_int_send(
        0,
        mavutil.mavlink.GPS_FIX_TYPE_3D_FIX,
        lat,
        lon,
        alt_mm,
        65535,
        65535,
        0,
        0,
        12,
    )
    conn.mav.global_position_int_send(
        time_boot_ms,
        lat,
        lon,
        alt_mm,
        alt_mm,
        0,
        0,
        0,
        0,
    )
    conn.mav.attitude_send(time_boot_ms, 0.0, 0.0, yaw_rad, 0.0, 0.0, 0.0)
    conn.mav.sys_status_send(0, 0, 0, 500, 12600, 10, 80, 0, 0, 0, 0, 0, 0)


if __name__ == "__main__":
    raise SystemExit(main())
