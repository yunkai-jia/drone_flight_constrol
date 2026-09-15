"""民用无人机飞控统一 Python 接口（MAVLink）。

应用通过 ``FlightController`` 操作 PX4 或 ArduPilot，二者均走 MAVLink。
当前实现为 ``MavlinkFlightController``（pymavlink）。本包只提供连接、
解锁、起飞、前往、降落、返航与遥测，不包含任务打击或武器相关能力。

典型用法::

    from drone_fc import ConnectionConfig, FlightMode, MavlinkFlightController

    fc = MavlinkFlightController(ConnectionConfig("udp:127.0.0.1:14540", skip_gps_check=True))
    fc.connect()
    print(fc.wait_heartbeat())
    fc.takeoff(5.0)
    fc.land()
    fc.disconnect()
"""

from __future__ import annotations

from typing import Any

from drone_fc.exceptions import (
    ArmDeniedError,
    CommandFailedError,
    DroneFcError,
    GpsNotFixedError,
    HeartbeatTimeoutError,
    InvalidArgumentError,
    NotConnectedError,
    UnsupportedModeError,
)
from drone_fc.interface import FlightController
from drone_fc.types import (
    Attitude,
    AutopilotType,
    Battery,
    ConnectionConfig,
    FlightMode,
    GpsFixType,
    Position,
    Telemetry,
)

__all__ = [
    "ArmDeniedError",
    "Attitude",
    "AutopilotType",
    "Battery",
    "CommandFailedError",
    "ConnectionConfig",
    "DroneFcError",
    "FlightController",
    "FlightMode",
    "GpsFixType",
    "GpsNotFixedError",
    "HeartbeatTimeoutError",
    "InvalidArgumentError",
    "MavlinkFlightController",
    "NotConnectedError",
    "Position",
    "Telemetry",
    "UnsupportedModeError",
]


def __getattr__(name: str) -> Any:
    """延迟加载 pymavlink 实现，使类型与模式映射可在未安装 pymavlink 时导入。"""

    if name == "MavlinkFlightController":
        from drone_fc.mavlink.controller import MavlinkFlightController

        return MavlinkFlightController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
