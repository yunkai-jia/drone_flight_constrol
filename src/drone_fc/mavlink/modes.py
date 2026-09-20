"""PX4 custom_mode 位域与 ArduPilot 各机型模式号的双向映射。

PX4 把主模式、子模式打包进 HEARTBEAT.custom_mode（小端）：

    custom_mode = (sub_mode << 24) | (main_mode << 16)

即 bit16-23 为主模式，bit24-31 为子模式，低 16 位保留为 0。PX4 各机型共用这套位域。

ArduPilot 的 custom_mode 等于机型相关的整数模式号：
多旋翼见 copter/defines.h，固定翼/垂见 ArduPlane/mode.h（QuadPlane 走 Plane 表）。
"""

from __future__ import annotations

from drone_fc.exceptions import UnsupportedModeError
from drone_fc.types import AutopilotType, FlightMode, VehicleClass

# PX4 主模式（px4_custom_mode.main_mode），见 PX4 src/modules/commander/px4_custom_mode.h
PX4_MAIN_MANUAL = 1
PX4_MAIN_ALTCTL = 2
PX4_MAIN_POSCTL = 3
PX4_MAIN_AUTO = 4
PX4_MAIN_ACRO = 5
PX4_MAIN_OFFBOARD = 6
PX4_MAIN_STABILIZED = 7

# PX4 AUTO 子模式
PX4_SUB_AUTO_TAKEOFF = 2
PX4_SUB_AUTO_LOITER = 3
PX4_SUB_AUTO_MISSION = 4
PX4_SUB_AUTO_RTL = 5
PX4_SUB_AUTO_LAND = 6

# ArduCopter 模式号（copter/defines.h）
AP_STABILIZE = 0
AP_AUTO = 3
AP_GUIDED = 4
AP_LOITER = 5
AP_RTL = 6
AP_LAND = 9

# ArduPlane / QuadPlane 模式号（ArduPlane/mode.h）
PLANE_MANUAL = 0
PLANE_FBWA = 5
PLANE_AUTO = 10
PLANE_RTL = 11
PLANE_LOITER = 12
PLANE_TAKEOFF = 13
PLANE_GUIDED = 15
PLANE_QSTABILIZE = 17
PLANE_QLOITER = 19
PLANE_QLAND = 20
PLANE_QRTL = 21


def encode_px4_custom_mode(main_mode: int, sub_mode: int = 0) -> int:
    """把 PX4 主/子模式编码为 HEARTBEAT.custom_mode 用的 32 位整数。

    输入:
        main_mode: 主模式，整数 [1, 10]。
        sub_mode: 子模式，整数 [0, 15]，非 AUTO 时通常为 0。
    输出:
        32 位无符号含义的 Python int：``(sub_mode << 24) | (main_mode << 16)``。
    """

    return (int(sub_mode) << 24) | (int(main_mode) << 16)


def decode_px4_custom_mode(custom_mode: int) -> tuple[int, int]:
    """从 custom_mode 解出 (main_mode, sub_mode)。

    输入:
        custom_mode: HEARTBEAT.custom_mode，按无符号 32 位解释。
    输出:
        (main_mode, sub_mode)，范围分别约 [0, 255]。
    """

    value = int(custom_mode) & 0xFFFFFFFF
    main_mode = (value >> 16) & 0xFF
    sub_mode = (value >> 24) & 0xFF
    return main_mode, sub_mode


def flight_mode_from_heartbeat(
    autopilot: AutopilotType,
    custom_mode: int,
    vehicle_class: VehicleClass = VehicleClass.COPTER,
) -> FlightMode:
    """根据固件与机型把 HEARTBEAT.custom_mode 译为语义模式。

    输入:
        autopilot: PX4 或 ARDUPILOT；UNKNOWN 时直接返回 UNKNOWN。
        custom_mode: 心跳中的 custom_mode 字段。
        vehicle_class: 机型。默认 COPTER，与 V1 多旋翼表兼容。PX4 忽略此参数。
    输出:
        能识别则返回对应 FlightMode，否则 UNKNOWN。
    """

    if autopilot is AutopilotType.PX4:
        return _from_px4(custom_mode)
    if autopilot is AutopilotType.ARDUPILOT:
        if vehicle_class is VehicleClass.PLANE:
            return _from_arduplane(custom_mode)
        if vehicle_class is VehicleClass.VTOL:
            return _from_ardupilot_vtol(custom_mode)
        return _from_arducopter(custom_mode)
    return FlightMode.UNKNOWN


def custom_mode_for(
    autopilot: AutopilotType,
    mode: FlightMode,
    vehicle_class: VehicleClass = VehicleClass.COPTER,
) -> int:
    """把语义模式译为 SET_MODE 所需的 custom_mode。

    输入:
        autopilot: 必须是 PX4 或 ARDUPILOT。
        mode: 不可为 UNKNOWN。
        vehicle_class: 机型。默认 COPTER。PX4 各机型共用位域。
    输出:
        可写入 SET_MODE.custom_mode 的整数。
    失败:
        UnsupportedModeError: 固件未知，机型无该模式，或 mode 为 UNKNOWN。
    """

    if mode is FlightMode.UNKNOWN:
        raise UnsupportedModeError("UNKNOWN cannot be used with set_mode")
    if autopilot is AutopilotType.PX4:
        return _to_px4(mode)
    if autopilot is AutopilotType.ARDUPILOT:
        if vehicle_class is VehicleClass.PLANE:
            return _to_arduplane(mode)
        if vehicle_class is VehicleClass.VTOL:
            return _to_ardupilot_vtol(mode)
        if vehicle_class is VehicleClass.UNKNOWN:
            raise UnsupportedModeError("UNKNOWN airframe has no ArduPilot mode map")
        return _to_arducopter(mode)
    raise UnsupportedModeError(f"firmware {autopilot} has no mode map")


def _from_px4(custom_mode: int) -> FlightMode:
    """PX4 custom_mode → 语义模式。无法识别时返回 UNKNOWN。"""

    main_mode, sub_mode = decode_px4_custom_mode(custom_mode)
    if main_mode in (PX4_MAIN_MANUAL, PX4_MAIN_ACRO, PX4_MAIN_STABILIZED, PX4_MAIN_ALTCTL):
        return FlightMode.MANUAL
    if main_mode == PX4_MAIN_POSCTL:
        return FlightMode.HOLD
    if main_mode == PX4_MAIN_OFFBOARD:
        return FlightMode.GUIDED
    if main_mode == PX4_MAIN_AUTO:
        return {
            PX4_SUB_AUTO_TAKEOFF: FlightMode.TAKEOFF,
            PX4_SUB_AUTO_LOITER: FlightMode.HOLD,
            PX4_SUB_AUTO_MISSION: FlightMode.MISSION,
            PX4_SUB_AUTO_RTL: FlightMode.RTL,
            PX4_SUB_AUTO_LAND: FlightMode.LAND,
        }.get(sub_mode, FlightMode.UNKNOWN)
    return FlightMode.UNKNOWN


def _to_px4(mode: FlightMode) -> int:
    """语义模式 → PX4 custom_mode。HOLD 使用 AUTO.LOITER，GUIDED 使用 OFFBOARD。"""

    mapping = {
        FlightMode.MANUAL: encode_px4_custom_mode(PX4_MAIN_MANUAL),
        FlightMode.HOLD: encode_px4_custom_mode(PX4_MAIN_AUTO, PX4_SUB_AUTO_LOITER),
        FlightMode.MISSION: encode_px4_custom_mode(PX4_MAIN_AUTO, PX4_SUB_AUTO_MISSION),
        FlightMode.RTL: encode_px4_custom_mode(PX4_MAIN_AUTO, PX4_SUB_AUTO_RTL),
        FlightMode.LAND: encode_px4_custom_mode(PX4_MAIN_AUTO, PX4_SUB_AUTO_LAND),
        FlightMode.TAKEOFF: encode_px4_custom_mode(PX4_MAIN_AUTO, PX4_SUB_AUTO_TAKEOFF),
        FlightMode.GUIDED: encode_px4_custom_mode(PX4_MAIN_OFFBOARD),
    }
    if mode not in mapping:
        raise UnsupportedModeError(f"PX4 has no mapping for {mode}")
    return mapping[mode]


def _from_arducopter(custom_mode: int) -> FlightMode:
    """ArduCopter 模式号 → 语义模式。未列入映射表则 UNKNOWN。"""

    return {
        AP_STABILIZE: FlightMode.MANUAL,
        AP_AUTO: FlightMode.MISSION,
        AP_GUIDED: FlightMode.GUIDED,
        AP_LOITER: FlightMode.HOLD,
        AP_RTL: FlightMode.RTL,
        AP_LAND: FlightMode.LAND,
    }.get(int(custom_mode), FlightMode.UNKNOWN)


def _to_arducopter(mode: FlightMode) -> int:
    """语义模式 → ArduCopter 模式号。

    ArduCopter 没有独立 TAKEOFF 模式：起飞在 GUIDED 下发 NAV_TAKEOFF，
    因此 TAKEOFF 与 GUIDED 都映射为 4。
    """

    mapping = {
        FlightMode.MANUAL: AP_STABILIZE,
        FlightMode.HOLD: AP_LOITER,
        FlightMode.MISSION: AP_AUTO,
        FlightMode.RTL: AP_RTL,
        FlightMode.LAND: AP_LAND,
        FlightMode.TAKEOFF: AP_GUIDED,
        FlightMode.GUIDED: AP_GUIDED,
    }
    if mode not in mapping:
        raise UnsupportedModeError(f"ArduCopter has no mapping for {mode}")
    return mapping[mode]


def _from_arduplane(custom_mode: int) -> FlightMode:
    """ArduPlane 模式号 → 语义模式。"""

    return {
        PLANE_MANUAL: FlightMode.MANUAL,
        PLANE_FBWA: FlightMode.MANUAL,
        PLANE_AUTO: FlightMode.MISSION,
        PLANE_RTL: FlightMode.RTL,
        PLANE_LOITER: FlightMode.HOLD,
        PLANE_TAKEOFF: FlightMode.TAKEOFF,
        PLANE_GUIDED: FlightMode.GUIDED,
    }.get(int(custom_mode), FlightMode.UNKNOWN)


def _to_arduplane(mode: FlightMode) -> int:
    """语义模式 → ArduPlane 模式号。纯固定翼无 LAND 模式。"""

    mapping = {
        FlightMode.MANUAL: PLANE_MANUAL,
        FlightMode.HOLD: PLANE_LOITER,
        FlightMode.MISSION: PLANE_AUTO,
        FlightMode.RTL: PLANE_RTL,
        FlightMode.TAKEOFF: PLANE_TAKEOFF,
        FlightMode.GUIDED: PLANE_GUIDED,
    }
    if mode not in mapping:
        raise UnsupportedModeError(f"ArduPlane has no mapping for {mode}")
    return mapping[mode]


def _from_ardupilot_vtol(custom_mode: int) -> FlightMode:
    """QuadPlane 模式号 → 语义模式。多旋翼起降模式优先于固定翼同名语义。"""

    return {
        PLANE_MANUAL: FlightMode.MANUAL,
        PLANE_FBWA: FlightMode.MANUAL,
        PLANE_QSTABILIZE: FlightMode.MANUAL,
        PLANE_AUTO: FlightMode.MISSION,
        PLANE_RTL: FlightMode.RTL,
        PLANE_QRTL: FlightMode.RTL,
        PLANE_LOITER: FlightMode.HOLD,
        PLANE_QLOITER: FlightMode.HOLD,
        PLANE_TAKEOFF: FlightMode.TAKEOFF,
        PLANE_GUIDED: FlightMode.GUIDED,
        PLANE_QLAND: FlightMode.LAND,
    }.get(int(custom_mode), FlightMode.UNKNOWN)


def _to_ardupilot_vtol(mode: FlightMode) -> int:
    """语义模式 → QuadPlane 模式号。起降与悬停走 Q* 模式。"""

    mapping = {
        FlightMode.MANUAL: PLANE_QSTABILIZE,
        FlightMode.HOLD: PLANE_QLOITER,
        FlightMode.MISSION: PLANE_AUTO,
        FlightMode.RTL: PLANE_QRTL,
        FlightMode.LAND: PLANE_QLAND,
        FlightMode.TAKEOFF: PLANE_TAKEOFF,
        FlightMode.GUIDED: PLANE_GUIDED,
    }
    if mode not in mapping:
        raise UnsupportedModeError(f"ArduPilot VTOL has no mapping for {mode}")
    return mapping[mode]
