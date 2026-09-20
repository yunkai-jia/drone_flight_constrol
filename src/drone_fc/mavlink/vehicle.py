"""从 HEARTBEAT.type 归类机型，并给出该机型当前 SDK 的能力画像。

MAV_TYPE 数值来自 MAVLink common.xml，本模块不依赖 pymavlink。
"""

from __future__ import annotations

from drone_fc.types import Capability, CapabilityProfile, VehicleClass

# MAV_TYPE（common.xml）
MAV_TYPE_FIXED_WING = 1
MAV_TYPE_QUADROTOR = 2
MAV_TYPE_COAXIAL = 3
MAV_TYPE_HELICOPTER = 4
MAV_TYPE_HEXAROTOR = 13
MAV_TYPE_OCTOROTOR = 14
MAV_TYPE_TRICOPTER = 15
MAV_TYPE_FLAPPING_WING = 16
MAV_TYPE_VTOL_DUOROTOR = 19
MAV_TYPE_VTOL_QUADROTOR = 20
MAV_TYPE_VTOL_TILTROTOR = 21
MAV_TYPE_VTOL_RESERVED2 = 22
MAV_TYPE_VTOL_RESERVED3 = 23
MAV_TYPE_VTOL_RESERVED4 = 24
MAV_TYPE_VTOL_RESERVED5 = 25

_COPTER_TYPES = frozenset(
    {
        MAV_TYPE_QUADROTOR,
        MAV_TYPE_COAXIAL,
        MAV_TYPE_HELICOPTER,
        MAV_TYPE_HEXAROTOR,
        MAV_TYPE_OCTOROTOR,
        MAV_TYPE_TRICOPTER,
    }
)
_PLANE_TYPES = frozenset({MAV_TYPE_FIXED_WING, MAV_TYPE_FLAPPING_WING})
_VTOL_TYPES = frozenset(
    {
        MAV_TYPE_VTOL_DUOROTOR,
        MAV_TYPE_VTOL_QUADROTOR,
        MAV_TYPE_VTOL_TILTROTOR,
        MAV_TYPE_VTOL_RESERVED2,
        MAV_TYPE_VTOL_RESERVED3,
        MAV_TYPE_VTOL_RESERVED4,
        MAV_TYPE_VTOL_RESERVED5,
    }
)

_AIR_BASE = frozenset(
    {
        Capability.ARM,
        Capability.TAKEOFF,
        Capability.GOTO,
        Capability.HOLD,
        Capability.RTL,
        Capability.SET_MODE,
    }
)
_FLIGHT_PLUS = frozenset({Capability.MISSION, Capability.OFFBOARD})
_COPTER_CAPS = _AIR_BASE | {Capability.LAND} | _FLIGHT_PLUS
_VTOL_CAPS = _COPTER_CAPS
_PLANE_CAPS = _AIR_BASE | _FLIGHT_PLUS  # 纯固定翼无独立 LAND；无云台直到检测到设备


def vehicle_class_from_mav_type(mav_type: int) -> VehicleClass:
    """把 HEARTBEAT.type 归为 COPTER / PLANE / VTOL / UNKNOWN。

    输入:
        mav_type: MAV_TYPE 整型，合法范围由 MAVLink 枚举定义，通常 [0, 40]。
    输出:
        VehicleClass。无人车、船、云台等返回 UNKNOWN。
    """

    value = int(mav_type)
    if value in _COPTER_TYPES:
        return VehicleClass.COPTER
    if value in _PLANE_TYPES:
        return VehicleClass.PLANE
    if value in _VTOL_TYPES:
        return VehicleClass.VTOL
    return VehicleClass.UNKNOWN


def default_capabilities(vehicle_class: VehicleClass) -> CapabilityProfile:
    """当前 SDK 已实现、且该机型允许的动作集合。

    输入:
        vehicle_class: 已识别的机型。
    输出:
        CapabilityProfile。GIMBAL 仅在总线上见到云台后由控制器追加。
        UNKNOWN 为空集（仍可读遥测，但拒绝飞行命令）。
    """

    mapping = {
        VehicleClass.COPTER: _COPTER_CAPS,
        VehicleClass.PLANE: _PLANE_CAPS,
        VehicleClass.VTOL: _VTOL_CAPS,
        VehicleClass.UNKNOWN: frozenset(),
    }
    return CapabilityProfile(vehicle_class=vehicle_class, supported=mapping[vehicle_class])


def with_capability(profile: CapabilityProfile, capability: Capability) -> CapabilityProfile:
    """返回带上额外能力的新画像（原对象不变）。

    输入:
        profile: 已有画像。
        capability: 要打开的能力，例如检测到云台后的 GIMBAL。
    输出:
        新的 CapabilityProfile，vehicle_class 不变。
    """

    return CapabilityProfile(
        vehicle_class=profile.vehicle_class,
        supported=profile.supported | {capability},
    )


def takeoff_altitude_max_m(vehicle_class: VehicleClass) -> float:
    """该机型 takeoff() 允许的最大相对高（米）。

    输入:
        vehicle_class: 已识别机型。UNKNOWN 按多旋翼收紧。
    输出:
        Copter / VTOL: 100。Plane: 500。UNKNOWN: 100。
    """

    if vehicle_class is VehicleClass.PLANE:
        return 500.0
    return 100.0
