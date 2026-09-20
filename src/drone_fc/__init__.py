"""Civil flight-controller API over MAVLink (PX4 and ArduPilot).

Applications use ``FlightController``. Same verbs for multicopter,
fixed-wing, and VTOL; the backend maps firmware and airframe. No weapons
or strike logic.

Typical usage — connect the vehicle directly (USB / UART / UDP)::

    from drone_fc import ConnectionConfig, MavlinkFlightController

    fc = MavlinkFlightController(ConnectionConfig("serial:/dev/ttyACM0:57600"))
    fc.connect()                 # opens the port and waits for HEARTBEAT
    fc.takeoff(5.0)
    fc.wait_telemetry(lambda s: s.position is not None and s.position.altitude_m >= 3.5, 20.0)
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
    LinkLostError,
    NotConnectedError,
    TelemetryTimeoutError,
    TransportOpenError,
    UnsupportedCapabilityError,
    UnsupportedModeError,
)
from drone_fc.interface import FlightController
from drone_fc.types import (
    Attitude,
    AutopilotType,
    Battery,
    Capability,
    CapabilityProfile,
    ConnectionConfig,
    FlightMode,
    GpsFixType,
    MissionItem,
    Position,
    Telemetry,
    VehicleClass,
)

__all__ = [
    "ArmDeniedError",
    "Attitude",
    "AutopilotType",
    "Battery",
    "Capability",
    "CapabilityProfile",
    "CommandFailedError",
    "ConnectionConfig",
    "DroneFcError",
    "FlightController",
    "FlightMode",
    "GpsFixType",
    "GpsNotFixedError",
    "HeartbeatTimeoutError",
    "InvalidArgumentError",
    "LinkLostError",
    "MavlinkFlightController",
    "MissionItem",
    "NotConnectedError",
    "Position",
    "Telemetry",
    "TelemetryTimeoutError",
    "TransportOpenError",
    "UnsupportedCapabilityError",
    "UnsupportedModeError",
    "VehicleClass",
]


def __getattr__(name: str) -> Any:
    """延迟加载 pymavlink 实现，未安装 pymavlink 时仍可导入类型。"""

    if name == "MavlinkFlightController":
        from drone_fc.mavlink.controller import MavlinkFlightController

        return MavlinkFlightController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
