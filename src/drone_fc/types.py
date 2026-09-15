"""统一飞控接口的数据类型。

经纬度采用 WGS84 大地坐标；高度默认是相对 Home 点的米制高度（相对高），
与 MAVLink ``GLOBAL_POSITION_INT.relative_alt`` 一致，而不是海拔椭球高。
姿态角采用航空常用的欧拉角：滚转 / 俯仰 / 偏航，单位为度。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class AutopilotType(Enum):
    """由 HEARTBEAT.autopilot 识别出的飞控固件族。

    取值对应 MAVLink ``MAV_AUTOPILOT`` 枚举中的常用项：
    - PX4: ``MAV_AUTOPILOT_PX4`` (12)
    - ARDUPILOT: ``MAV_AUTOPILOT_ARDUPILOTMEGA`` (3)
    - UNKNOWN: 已收到心跳但固件不在上述两类，模式映射不可用
    """

    PX4 = "px4"
    ARDUPILOT = "ardupilot"
    UNKNOWN = "unknown"


class FlightMode(Enum):
    """与具体固件无关的语义飞行模式。

    应用层只使用这些取值；底层再翻译成 PX4 ``custom_mode`` 位域
    或 ArduPilot 多旋翼整数模式号。``UNKNOWN`` 仅用于遥测回读，
    禁止传入 ``set_mode``。
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
    """GPS 定位质量，对应 ``GPS_RAW_INT.fix_type``（MAV_GPS_FIX_TYPE）。

    起飞安全检查要求 ``FIX_3D`` 及以上（取值 >= 3）。
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
    """地理位置。

    Attributes:
        latitude_deg: 纬度，单位度，闭区间 [-90, 90]。正值北纬。
        longitude_deg: 经度，单位度，闭区间 [-180, 180]。正值东经。
        altitude_m: 相对 Home 的高度，单位米。起飞后通常 > 0；未知时为 None。
    """

    latitude_deg: float
    longitude_deg: float
    altitude_m: float | None


@dataclass(frozen=True)
class Attitude:
    """机体姿态（ZYX 欧拉角，航空约定）。

    滚转/俯仰/偏航由 MAVLink ``ATTITUDE`` 的弧度值换算为度：
    ``deg = rad * 180 / pi``。偏航 0 度表示北，顺时针增大至 360（若来源
    为 ``yaw`` 弧度则先规范到 (-pi, pi]，再转度）。

    Attributes:
        roll_deg: 滚转，单位度，典型范围 [-180, 180]。右翼下沉为正。
        pitch_deg: 俯仰，单位度，典型范围 [-90, 90]。机头上仰为正。
        yaw_deg: 偏航，单位度，范围 (-180, 180]。机头北偏东为正。
    """

    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class Battery:
    """电源状态，来自 ``SYS_STATUS`` 或 ``BATTERY_STATUS``。

    Attributes:
        voltage_v: 电池电压，单位伏特。未知时为 None。合法范围通常 (0, 60]。
        remaining_percent: 剩余电量百分比，闭区间 [0, 100]。未知时为 None。
        current_a: 电流，单位安培，放电为正。未知时为 None。
    """

    voltage_v: float | None
    remaining_percent: float | None
    current_a: float | None


@dataclass(frozen=True)
class Telemetry:
    """一次遥测快照。字段在对应 MAVLink 消息尚未到达时可为 None。

    Attributes:
        autopilot: 已识别的固件类型；尚未心跳时为 None。
        mode: 语义飞行模式。
        armed: 是否已解锁。True 表示螺旋桨可能转动。
        position: 全球位置；无 ``GLOBAL_POSITION_INT`` 时为 None。
        attitude: 姿态；无 ``ATTITUDE`` 时为 None。
        battery: 电池；无电源消息时为 None。
        gps_fix: GPS 定位类型；无 ``GPS_RAW_INT`` 时为 None。
        satellites_visible: 可见卫星数，范围 [0, 64] 量级；未知为 None。
    """

    autopilot: AutopilotType | None
    mode: FlightMode
    armed: bool
    position: Position | None
    attitude: Attitude | None
    battery: Battery | None
    gps_fix: GpsFixType | None
    satellites_visible: int | None


@dataclass(frozen=True)
class ConnectionConfig:
    """连接到飞控的参数。

    Attributes:
        uri: 连接串，支持：
            - ``udp:host:port`` — UDP 监听（PX4 SITL 默认 ``udp:127.0.0.1:14540``）
            - ``udpin:host:port`` / ``udpout:host:port`` — 显式方向
            - ``tcp:host:port`` — TCP 客户端（ArduPilot SITL 常见 ``tcp:127.0.0.1:5760``）
            - ``serial:device:baud`` — 串口，例如 ``serial:/dev/ttyACM0:57600``
        source_system: 本端 MAVLink 系统 ID，整数 [1, 255]，默认 255（地面站惯例）。
        source_component: 本端组件 ID，整数 [1, 255]，默认 190。
        skip_gps_check: True 时起飞不要求 3D 定位，仅建议用于 SITL。默认 False。
    """

    uri: str
    source_system: int = 255
    source_component: int = 190
    skip_gps_check: bool = False
