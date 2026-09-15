"""基于 pymavlink 的统一飞控实现，同时覆盖 PX4 与 ArduPilot。

本类把「语义 API」翻译成 MAVLink：HEARTBEAT 识别固件，SET_MODE / COMMAND_LONG
发控制，GLOBAL_POSITION_INT 等消息填遥测缓存。单线程使用，内部用最近值快照，
不另开读线程。
"""

from __future__ import annotations

import math
import time
from typing import Any

from pymavlink import mavutil

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
from drone_fc.mavlink.connection import parse_connection_uri
from drone_fc.mavlink.modes import custom_mode_for, flight_mode_from_heartbeat
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

_MAV_AUTOPILOT_PX4 = 12  # MAV_AUTOPILOT_PX4
_MAV_AUTOPILOT_ARDUPILOT = 3  # MAV_AUTOPILOT_ARDUPILOTMEGA
_MAV_RESULT_ACCEPTED = 0
_MAV_RESULT_IN_PROGRESS = 5  # 继续等后续 ACK
_MAV_RESULT_UNSUPPORTED = 3
_COMMAND_TIMEOUT_S = 5.0
_MODE_WAIT_S = 5.0
_TAKEOFF_ALT_MAX_M = 100.0
_GOTO_ALT_MAX_M = 500.0
# SET_POSITION_TARGET_GLOBAL_INT 的 type_mask：位置有效，忽略速度/加速度/力/偏航。
# bit0-2 = x/y/z 使用；bit3-11 = vx,vy,vz,ax,ay,az,force,yaw,yaw_rate 忽略。
_POS_TYPEMASK_POSITION_ONLY = 0b0000_1111_1111_1000
_RAD2DEG = 180.0 / math.pi  # 姿态：弧度 → 度


class MavlinkFlightController(FlightController):
    """通过 MAVLink 连接一台 PX4 或 ArduPilot 飞控。

    输入（构造）:
        config: ConnectionConfig。uri 见该类型说明；skip_gps_check 仅建议 SITL 使用。
    """

    def __init__(self, config: ConnectionConfig) -> None:
        self._config = config
        self._master: Any = None  # pymavlink 连接，未 connect 时为 None
        self._autopilot: AutopilotType | None = None
        self._got_heartbeat = False
        self._armed = False
        self._custom_mode = 0  # 最近一次 HEARTBEAT.custom_mode
        self._position: Position | None = None
        self._attitude: Attitude | None = None
        self._battery: Battery | None = None
        self._gps_fix: GpsFixType | None = None
        self._satellites: int | None = None

    def connect(self, timeout_s: float = 10.0) -> None:
        """打开传输层。详见 ``FlightController.connect``。"""

        self._require_timeout(timeout_s)
        self._validate_ids()
        if self._master is not None:
            self.disconnect()
        parsed = parse_connection_uri(self._config.uri)
        try:
            kwargs: dict[str, Any] = {
                "autoreconnect": True,
                "source_system": self._config.source_system,
                "source_component": self._config.source_component,
                # pymavlink 的 timeout 是默认 recv 超时，不是 TCP 三次握手专用时限
                "timeout": timeout_s,
            }
            if parsed.baud is not None:
                kwargs["baud"] = parsed.baud
            self._master = mavutil.mavlink_connection(parsed.device, **kwargs)
        except Exception as exc:
            self._master = None
            raise DroneFcError(f"无法连接 {self._config.uri!r}: {exc}") from exc

    def disconnect(self) -> None:
        """关闭连接。详见 ``FlightController.disconnect``。"""

        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
        self._autopilot = None
        self._got_heartbeat = False
        self._clear_cache()

    def wait_heartbeat(self, timeout_s: float = 15.0) -> AutopilotType:
        """等待心跳并识别固件。详见 ``FlightController.wait_heartbeat``。"""

        self._require_timeout(timeout_s)
        master = self._require_master()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            # 不按类型过滤：recv_match(type=HEARTBEAT) 会丢弃同批次 GPS/位置包。
            msg = master.recv_match(blocking=True, timeout=min(1.0, remaining))
            if msg is None:
                continue
            self._update_cache(msg)
            if msg.get_type() != "HEARTBEAT":
                continue
            if msg.get_srcSystem() == self._config.source_system:
                continue  # 忽略本端发出的地面站心跳
            # 后续 COMMAND_LONG 必须写到这对 sys/comp id
            master.target_system = msg.get_srcSystem()
            master.target_component = msg.get_srcComponent()
            self._got_heartbeat = True
            self._send_gcs_heartbeat()
            self._request_telemetry_rates()
            self._drain()
            assert self._autopilot is not None
            return self._autopilot
        raise HeartbeatTimeoutError(f"{timeout_s}s 内未收到 HEARTBEAT，请检查 URI 与 SITL/飞控是否在线")

    def telemetry(self) -> Telemetry:
        """非阻塞读取遥测。详见 ``FlightController.telemetry``。"""

        self._require_master()
        self._drain()
        mode = FlightMode.UNKNOWN
        if self._got_heartbeat and self._autopilot is not None:
            mode = flight_mode_from_heartbeat(self._autopilot, self._custom_mode)
        return Telemetry(
            autopilot=self._autopilot,
            mode=mode,
            armed=self._armed,
            position=self._position,
            attitude=self._attitude,
            battery=self._battery,
            gps_fix=self._gps_fix,
            satellites_visible=self._satellites,
        )

    def arm(self) -> None:
        """解锁。详见 ``FlightController.arm``。"""

        self._require_heartbeat()
        self._send_gcs_heartbeat()
        try:
            self._command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                1.0,  # param1=1 解锁；0 上锁
                0.0,  # param2=0 不强制；21196 为强制解锁（本库不使用）
            )
        except CommandFailedError as exc:
            raise ArmDeniedError(f"解锁被拒绝: {exc}") from exc

    def disarm(self) -> None:
        """上锁。详见 ``FlightController.disarm``。"""

        self._require_heartbeat()
        self._command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0.0,
            0.0,
        )

    def set_mode(self, mode: FlightMode) -> None:
        """切换语义模式。详见 ``FlightController.set_mode``。"""

        self._require_heartbeat()
        if mode is FlightMode.UNKNOWN:
            raise UnsupportedModeError("UNKNOWN 不能用于 set_mode")
        assert self._autopilot is not None
        custom = custom_mode_for(self._autopilot, mode)
        master = self._require_master()
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom,
        )
        deadline = time.monotonic() + _MODE_WAIT_S
        while time.monotonic() < deadline:
            self._pump(0.2)
            current = flight_mode_from_heartbeat(self._autopilot, self._custom_mode)
            if current is mode:
                return
            # ArduPilot TAKEOFF 映射为 GUIDED，回读为 GUIDED 即视为成功
            if mode is FlightMode.TAKEOFF and current is FlightMode.GUIDED:
                return
        raise CommandFailedError(
            f"切换模式到 {mode.value} 超时（当前 custom_mode={self._custom_mode}）"
        )

    def takeoff(self, altitude_m: float) -> None:
        """起飞到相对高。详见 ``FlightController.takeoff``。"""

        self._require_heartbeat()
        alt = self._require_finite("altitude_m", altitude_m)
        if alt <= 0 or alt > _TAKEOFF_ALT_MAX_M:
            raise InvalidArgumentError(f"altitude_m 必须在 (0, {_TAKEOFF_ALT_MAX_M}] 米，收到 {alt}")
        self._drain()
        if not self._config.skip_gps_check:
            self._require_3d_fix()
        if self._autopilot is AutopilotType.PX4:
            self.set_mode(FlightMode.TAKEOFF)
        else:
            # ArduCopter：GUIDED + NAV_TAKEOFF
            self.set_mode(FlightMode.GUIDED)
        if not self._armed:
            self.arm()
        # param7 = 相对 Home 高度（米）
        self._command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            alt,
        )

    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """前往经纬高。详见 ``FlightController.goto``。"""

        self._require_heartbeat()
        latitude = self._require_finite("lat", lat)
        longitude = self._require_finite("lon", lon)
        alt = self._require_finite("alt_m", alt_m)
        if latitude < -90.0 or latitude > 90.0:
            raise InvalidArgumentError(f"lat 必须在 [-90, 90]，收到 {latitude}")
        if longitude < -180.0 or longitude > 180.0:
            raise InvalidArgumentError(f"lon 必须在 [-180, 180]，收到 {longitude}")
        if alt <= 0 or alt > _GOTO_ALT_MAX_M:
            raise InvalidArgumentError(f"alt_m 必须在 (0, {_GOTO_ALT_MAX_M}] 米，收到 {alt}")

        if self._autopilot is AutopilotType.PX4:
            try:
                self._command_long(
                    mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                    -1.0,  # 地速：-1 表示由飞控决定
                    0.0,
                    0.0,
                    float("nan"),  # 偏航不指定
                    latitude,
                    longitude,
                    alt,
                )
                return
            except CommandFailedError as exc:
                if exc.result != _MAV_RESULT_UNSUPPORTED:
                    raise

        self.set_mode(FlightMode.GUIDED)
        master = self._require_master()
        master.mav.set_position_target_global_int_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            _POS_TYPEMASK_POSITION_ONLY,
            int(round(latitude * 1e7)),  # degE7
            int(round(longitude * 1e7)),
            alt,  # 相对高，米
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def land(self) -> None:
        """降落。详见 ``FlightController.land``。"""

        self._require_heartbeat()
        try:
            self.set_mode(FlightMode.LAND)
            return
        except CommandFailedError:
            self._command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)

    def rtl(self) -> None:
        """返航。详见 ``FlightController.rtl``。"""

        self._require_heartbeat()
        try:
            self.set_mode(FlightMode.RTL)
            return
        except CommandFailedError:
            self._command_long(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def __enter__(self) -> MavlinkFlightController:
        self.connect()
        return self

    def __exit__(self, *args: object) -> None:
        self.disconnect()

    def _command_long(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        timeout_s: float = _COMMAND_TIMEOUT_S,
    ) -> None:
        """发送 COMMAND_LONG 并等待 COMMAND_ACK。

        输入:
            command: MAVLink 命令枚举整数值。
            param1..param7: 命令参数，单位与取值由具体 MAV_CMD 定义。
            timeout_s: 等待 ACK 秒数，必须 > 0。
        输出:
            None。ACK.result 为 ACCEPTED 或最终非 IN_PROGRESS 的成功码才返回。
        失败:
            CommandFailedError: 超时，或 result 不是 ACCEPTED。
        """

        master = self._require_master()
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            command,
            0,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self._pump(0.2)
            if msg is None or msg.get_type() != "COMMAND_ACK":
                continue
            if int(msg.command) != int(command):
                continue
            result = int(msg.result)
            if result == _MAV_RESULT_IN_PROGRESS:
                continue
            if result == _MAV_RESULT_ACCEPTED:
                return
            raise CommandFailedError(
                f"命令 {command} 失败，MAV_RESULT={result}",
                command=command,
                result=result,
            )
        raise CommandFailedError(f"命令 {command} 等待 ACK 超时", command=command)

    def _request_telemetry_rates(self) -> None:
        """请求常用消息约 4 Hz（间隔 250000 微秒）。失败忽略，依赖飞控默认流。

        消息 ID：1 SYS_STATUS，24 GPS_RAW_INT，30 ATTITUDE，
        33 GLOBAL_POSITION_INT，147 BATTERY_STATUS。
        """

        master = self._require_master()
        try:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,
                1,
            )
        except Exception:
            pass
        interval_us = 250_000.0
        for msg_id in (1, 24, 30, 33, 147):
            try:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    float(msg_id),
                    interval_us,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception:
                pass

    def _send_gcs_heartbeat(self) -> None:
        """以地面站身份发一次 HEARTBEAT，让飞控知道对端是 GCS。"""
        master = self._require_master()
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

    def _require_3d_fix(self) -> None:
        """起飞前检查 GPS_RAW_INT.fix_type >= 3。"""
        if self._gps_fix is None or self._gps_fix.value < GpsFixType.FIX_3D.value:
            raise GpsNotFixedError(
                f"起飞需要 3D GPS（fix_type>=3），当前为 {self._gps_fix}"
            )

    def _require_heartbeat(self) -> None:
        """尚未识别飞控时禁止解锁/改模式等控制命令。"""
        self._require_master()
        if not self._got_heartbeat or self._autopilot is None:
            raise NotConnectedError("尚未收到 HEARTBEAT，请先调用 wait_heartbeat()")

    def _require_master(self) -> Any:
        """返回已打开的 pymavlink 连接；未 connect 则抛 NotConnectedError。"""
        if self._master is None:
            raise NotConnectedError("尚未 connect()")
        return self._master

    def _validate_ids(self) -> None:
        """source_system / source_component 必须是 [1, 255] 的整数。"""
        for name, value in (
            ("source_system", self._config.source_system),
            ("source_component", self._config.source_component),
        ):
            if not isinstance(value, int) or value < 1 or value > 255:
                raise InvalidArgumentError(f"{name} 必须是 [1, 255] 的整数")

    def _clear_cache(self) -> None:
        """断开后丢掉遥测，避免下次 connect 读到旧数据。"""
        self._armed = False
        self._custom_mode = 0
        self._position = None
        self._attitude = None
        self._battery = None
        self._gps_fix = None
        self._satellites = None

    def _drain(self) -> None:
        """非阻塞读完当前套接字缓冲，并更新缓存。"""
        while True:
            msg = self._pump(0.0)
            if msg is None:
                return

    def _pump(self, timeout_s: float) -> Any | None:
        """收一条 MAVLink 并写入缓存。

        输入:
            timeout_s: <=0 表示非阻塞；>0 表示最多阻塞这么多秒。
        输出:
            消息对象或 None。
        注意:
            pymavlink 在 timeout=0 时会在 recv_msg 之前就判定超时，因此非阻塞
            路径必须用 blocking=False 且不传 timeout=0。
        """
        master = self._require_master()
        # timeout=0 时 pymavlink 会在 recv_msg 之前就因超时返回 None。
        if timeout_s <= 0:
            msg = master.recv_match(blocking=False)
        else:
            msg = master.recv_match(blocking=True, timeout=timeout_s)
        if msg is not None:
            self._update_cache(msg)
        return msg

    def _update_cache(self, msg: Any) -> None:
        """按消息类型刷新遥测快照。单位换算见各分支注释。"""
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            if msg.get_srcSystem() == self._config.source_system:
                return
            self._autopilot = _autopilot_from_heartbeat(int(msg.autopilot))
            self._custom_mode = int(msg.custom_mode)
            armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            self._armed = bool(int(msg.base_mode) & armed_flag)
        elif mtype == "GLOBAL_POSITION_INT":
            # lat/lon 为 degE7（度×1e7）；relative_alt 为毫米
            self._position = Position(
                latitude_deg=msg.lat / 1e7,
                longitude_deg=msg.lon / 1e7,
                altitude_m=msg.relative_alt / 1000.0,
            )
        elif mtype == "ATTITUDE":
            # MAVLink ATTITUDE 为弧度
            self._attitude = Attitude(
                roll_deg=msg.roll * _RAD2DEG,
                pitch_deg=msg.pitch * _RAD2DEG,
                yaw_deg=msg.yaw * _RAD2DEG,
            )
        elif mtype == "GPS_RAW_INT":
            fix = int(msg.fix_type)
            try:
                self._gps_fix = GpsFixType(fix)
            except ValueError:
                self._gps_fix = GpsFixType.NO_GPS if fix <= 0 else GpsFixType.FIX_3D
            self._satellites = int(msg.satellites_visible)
        elif mtype == "SYS_STATUS":
            # voltage_battery: mV，UINT16_MAX(65535) 表示未知
            # current_battery: cA（0.01 A），负值表示未知
            voltage = None if msg.voltage_battery == 65535 else msg.voltage_battery / 1000.0
            percent = None if msg.battery_remaining < 0 else float(msg.battery_remaining)
            current = None if msg.current_battery < 0 else msg.current_battery / 100.0
            self._battery = Battery(voltage_v=voltage, remaining_percent=percent, current_a=current)
        elif mtype == "BATTERY_STATUS":
            remaining = None if msg.battery_remaining < 0 else float(msg.battery_remaining)
            current = None if msg.current_battery < 0 else msg.current_battery / 100.0
            voltages = list(msg.voltages)
            cells = [v / 1000.0 for v in voltages if 0 < v < 65535]  # 每电芯 mV，65535 为无效
            voltage = sum(cells) if cells else None
            prev = self._battery
            self._battery = Battery(
                voltage_v=voltage if voltage is not None else (prev.voltage_v if prev else None),
                remaining_percent=remaining if remaining is not None else (prev.remaining_percent if prev else None),
                current_a=current if current is not None else (prev.current_a if prev else None),
            )

    @staticmethod
    def _require_timeout(timeout_s: float) -> None:
        """timeout_s 必须是 > 0 的有限数。"""
        if not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise InvalidArgumentError("timeout_s 必须是 > 0 的有限数")

    @staticmethod
    def _require_finite(name: str, value: float) -> float:
        """把调用参数规范成有限 float；NaN/Inf 非法。"""
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise InvalidArgumentError(f"{name} 必须是有限数字")
        return float(value)


def _autopilot_from_heartbeat(autopilot: int) -> AutopilotType:
    """HEARTBEAT.autopilot 整型 → AutopilotType。"""
    if autopilot == _MAV_AUTOPILOT_PX4:
        return AutopilotType.PX4
    if autopilot == _MAV_AUTOPILOT_ARDUPILOT:
        return AutopilotType.ARDUPILOT
    return AutopilotType.UNKNOWN
