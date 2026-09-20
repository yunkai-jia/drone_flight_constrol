"""Public data types for the unified flight-controller API.

Coordinates are WGS84. Altitude is metres above home
(``GLOBAL_POSITION_INT.relative_alt``), not ellipsoid height.
Attitude is aerospace Euler angles in degrees.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class AutopilotType(Enum):
    """Firmware family from HEARTBEAT.autopilot.

    - PX4: ``MAV_AUTOPILOT_PX4`` (12)
    - ARDUPILOT: ``MAV_AUTOPILOT_ARDUPILOTMEGA`` (3)
    - UNKNOWN: heartbeat seen but firmware is neither; mode mapping unavailable
    """

    PX4 = "px4"
    ARDUPILOT = "ardupilot"
    UNKNOWN = "unknown"


class VehicleClass(Enum):
    """Airframe family from HEARTBEAT.type (MAV_TYPE).

    Applications use the same verbs for every class. The backend picks the
    mode table and capability profile. Rovers, boats, and other non-aircraft
    types are ``UNKNOWN``.
    """

    COPTER = "copter"
    PLANE = "plane"
    VTOL = "vtol"
    UNKNOWN = "unknown"


class Capability(Enum):
    """One action the unified API may expose.

    Presence in ``CapabilityProfile`` means the library will send the command.
    Absence means the call fails immediately with ``UnsupportedCapabilityError``.
    """

    ARM = "arm"
    TAKEOFF = "takeoff"
    GOTO = "goto"
    HOLD = "hold"
    LAND = "land"
    RTL = "rtl"
    SET_MODE = "set_mode"
    MISSION = "mission"
    OFFBOARD = "offboard"
    GIMBAL = "gimbal"


@dataclass(frozen=True)
class CapabilityProfile:
    """Read-only actions this vehicle can perform through this SDK.

    Attributes:
        vehicle_class: Airframe used to build the set.
        supported: Capabilities that are implemented and valid for the airframe.
    """

    vehicle_class: VehicleClass
    supported: frozenset[Capability]

    def supports(self, capability: Capability) -> bool:
        """Return True if ``capability`` is in ``supported``.

        Args:
            capability: One Capability enum member.
        Returns:
            True if the call is allowed after connect.
        """

        return capability in self.supported


class FlightMode(Enum):
    """Firmware-agnostic flight mode.

    Applications use these values only. The backend maps them to PX4
    ``custom_mode`` or ArduPilot mode numbers for the current vehicle
    class. ``UNKNOWN`` is telemetry-only and must not be passed to
    ``set_mode``.
    """

    MANUAL = "manual"
    HOLD = "hold"
    MISSION = "mission"
    RTL = "rtl"
    LAND = "land"
    TAKEOFF = "takeoff"
    GUIDED = "guided"
    UNKNOWN = "unknown"


class GpsFixType(Enum):
    """GPS quality from ``GPS_RAW_INT.fix_type`` (MAV_GPS_FIX_TYPE).

    Takeoff requires ``FIX_3D`` or better (value >= 3).
    """

    NO_GPS = 0
    NO_FIX = 1
    FIX_2D = 2
    FIX_3D = 3
    DGPS = 4
    RTK_FLOAT = 5
    RTK_FIXED = 6


@dataclass(frozen=True)
class Position:
    """Geographic position.

    Attributes:
        latitude_deg: Latitude in degrees, [-90, 90]. North is positive.
        longitude_deg: Longitude in degrees, [-180, 180]. East is positive.
        altitude_m: Metres above home. Usually > 0 after takeoff; None if unknown.
    """

    latitude_deg: float
    longitude_deg: float
    altitude_m: float | None


@dataclass(frozen=True)
class Attitude:
    """Body attitude (ZYX Euler, aerospace convention).

    MAVLink ``ATTITUDE`` radians are converted with ``deg = rad * 180 / pi``.
    Yaw 0 deg is north.

    Attributes:
        roll_deg: Roll in degrees, typically [-180, 180]. Right-wing down positive.
        pitch_deg: Pitch in degrees, typically [-90, 90]. Nose up positive.
        yaw_deg: Yaw in degrees, (-180, 180]. Nose east of north positive.
    """

    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class Battery:
    """Power state from ``SYS_STATUS`` or ``BATTERY_STATUS``.

    Attributes:
        voltage_v: Pack voltage in volts. None if unknown. Typical (0, 60].
        remaining_percent: Remaining charge percent, [0, 100]. None if unknown.
        current_a: Current in amperes, discharge positive. None if unknown.
    """

    voltage_v: float | None
    remaining_percent: float | None
    current_a: float | None


@dataclass(frozen=True)
class Telemetry:
    """One telemetry snapshot. Fields are None until the matching MAVLink message arrives.

    Attributes:
        autopilot: Firmware type; None before heartbeat.
        vehicle_class: Airframe class; None before heartbeat.
        capabilities: Actions allowed on this link; None before heartbeat.
        mode: Semantic flight mode.
        armed: True if motors may spin.
        position: Global position; None without ``GLOBAL_POSITION_INT``.
        attitude: Attitude; None without ``ATTITUDE``.
        battery: Battery; None without power messages.
        gps_fix: GPS fix type; None without ``GPS_RAW_INT``.
        satellites_visible: Visible satellites, roughly [0, 64]; None if unknown.
    """

    autopilot: AutopilotType | None
    vehicle_class: VehicleClass | None
    capabilities: CapabilityProfile | None
    mode: FlightMode
    armed: bool
    position: Position | None
    attitude: Attitude | None
    battery: Battery | None
    gps_fix: GpsFixType | None
    satellites_visible: int | None


@dataclass(frozen=True)
class MissionItem:
    """One mission waypoint (WGS84, relative-home altitude).

    Attributes:
        latitude_deg: Latitude in degrees, [-90, 90].
        longitude_deg: Longitude in degrees, [-180, 180].
        altitude_m: Metres above home, (0, 500].
        command: MAV_CMD id. Default 16 (NAV_WAYPOINT).
        hold_s: Loiter time at the point in seconds, >= 0. Default 0.
        accept_radius_m: Acceptance radius in metres, >= 0. Default 2.
    """

    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    command: int = 16
    hold_s: float = 0.0
    accept_radius_m: float = 2.0


@dataclass(frozen=True)
class ConnectionConfig:
    """How to reach the flight controller.

    Direct vehicle URIs (no mock required)::

        serial:/dev/ttyACM0:57600      USB to the FC
        serial:/dev/ttyTHS1:921600     Jetson UART to TELEM2
        serial:/dev/serial0:921600     Raspberry Pi UART
        udp:127.0.0.1:14540            mavlink-router or PX4 SITL on this host
        tcp:127.0.0.1:5760             ArduPilot SITL

    MAVLink is unauthenticated by default. ``connect()`` opens a socket or
    serial port and waits for HEARTBEAT; it is not a login. Anyone who can
    reach that URI (UDP on the same network, or the serial device) can send
    commands. Multiple GCS clients on one link are normal. Isolate the
    network or serial path; MAVLink2 signing is not implemented in this
    release.

    Attributes:
        uri: Connection string as above; also ``udpin:`` / ``udpout:``.
        source_system: Local MAVLink system id, [1, 255], default 1 (same vehicle).
            Laptop GCS may use 255.
        source_component: Local component id, [1, 255], default 191
            (``MAV_COMP_ID_ONBOARD_COMPUTER``). GCS often uses 190. comp=191 sends
            an onboard-controller HEARTBEAT; any other value sends a GCS HEARTBEAT.
            Do not use comp=1 (the autopilot's own component id) for a companion —
            it collides with the FC and ``_is_own_source`` would drop FC heartbeats.
        skip_gps_check: If True, takeoff does not require 3D GPS. SITL/mock only.
            Keep False on a real vehicle. Default False.
    """

    uri: str
    source_system: int = 1
    source_component: int = 191
    skip_gps_check: bool = False
