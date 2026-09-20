"""统一飞控实现：HEARTBEAT 识别固件与机型，经 Transport 发 MAVLink。

语义 API 译成 SET_MODE / COMMAND_LONG；GLOBAL_POSITION_INT 等填遥测缓存。
单线程使用，不另开读线程。开发期 Transport 为 pymavlink。
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque
from collections.abc import Callable, Sequence
from typing import Any

_LOG = logging.getLogger("drone_fc")

from drone_fc.exceptions import (
    ArmDeniedError,
    CommandFailedError,
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
from drone_fc.mavlink import constants as mav
from drone_fc.mavlink.connection import parse_connection_uri
from drone_fc.mavlink.mission import deg_to_degE7, validate_mission_items
from drone_fc.mavlink.modes import custom_mode_for, flight_mode_from_heartbeat
from drone_fc.mavlink.transport import MavlinkTransport, PymavlinkTransport
from drone_fc.mavlink.vehicle import (
    default_capabilities,
    takeoff_altitude_max_m,
    vehicle_class_from_mav_type,
    with_capability,
)
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

_HEARTBEAT_PERIOD_S = 1.0  # 机载/地面站保活，秒
_LINK_LOST_S = 3.0  # 飞控 HEARTBEAT 超过此时长视为掉线；不自动重连
_STATUS_TEXT_MAX = 8  # 最近 STATUSTEXT 条数，供解锁失败诊断
_TELEMETRY_INTERVAL_US = 250_000.0  # 常用遥测约 4 Hz
_COMMAND_TIMEOUT_S = 5.0
_MODE_WAIT_S = 5.0
_GOTO_ALT_MAX_M = 500.0
_VELOCITY_MAX_MPS = 20.0
_MISSION_TIMEOUT_S = 15.0
# SET_POSITION_TARGET_GLOBAL_INT 的 type_mask：位置有效，忽略速度/加速度/力/偏航。
# bit0-2 = x/y/z 使用；bit3-11 = vx,vy,vz,ax,ay,az,force,yaw,yaw_rate 忽略。
_POS_TYPEMASK_POSITION_ONLY = 0b0000_1111_1111_1000
# 速度有效（NED），忽略位置与加速度/偏航。
_POS_TYPEMASK_VELOCITY_NED = 0b0000_1111_1100_0111
_RAD2DEG = 180.0 / math.pi  # 姿态：弧度 → 度


class MavlinkFlightController(FlightController):
    """通过 MAVLink 连接一台 PX4 或 ArduPilot 飞控。

    输入（构造）:
        config: ConnectionConfig。uri 见该类型说明；skip_gps_check 仅建议 SITL / mock。
            默认以机载计算机身份发心跳（sysid=1、comp=191）。
    """

    def __init__(self, config: ConnectionConfig) -> None:
        self._config = config
        self._transport: MavlinkTransport | None = None
        self._autopilot: AutopilotType | None = None
        self._vehicle_class: VehicleClass | None = None
        self._capabilities: CapabilityProfile | None = None
        self._got_heartbeat = False
        self._armed = False
        self._custom_mode = 0  # 最近一次飞控 HEARTBEAT.custom_mode
        self._position: Position | None = None
        self._attitude: Attitude | None = None
        self._battery: Battery | None = None
        self._gps_fix: GpsFixType | None = None
        self._satellites: int | None = None
        self._last_heartbeat_sent = 0.0  # monotonic 秒；0 表示尚未发过本端心跳
        self._last_fc_heartbeat_mono = 0.0  # 对端飞控最近一次 HEARTBEAT；0 表示尚未收到
        self._status_texts: deque[str] = deque(maxlen=_STATUS_TEXT_MAX)
        # SET_MESSAGE_INTERVAL 发送失败记录（msg_id → 原因），供排障；不阻塞 connect
        self._telemetry_rate_errors: dict[int, str] = {}

    def connect(self, timeout_s: float = 15.0) -> AutopilotType:
        """打开传输并等待飞控心跳。详见 ``FlightController.connect``。"""

        self._open_transport(timeout_s)
        return self.wait_heartbeat(timeout_s)

    def reconnect(self, timeout_s: float = 15.0, uri: str | None = None) -> AutopilotType:
        """关闭后再打开并等待心跳。详见 ``FlightController.reconnect``。

        uri 非空时替换 ConnectionConfig.uri（USB 重枚举换节点），其它字段不变。
        """

        if uri is not None:
            text = uri.strip() if isinstance(uri, str) else ""
            if not text:
                raise InvalidArgumentError("uri must be a non-empty string")
            parse_connection_uri(text)  # 先校验，避免 disconnect 后才发现非法
            self._config = ConnectionConfig(
                uri=text,
                source_system=self._config.source_system,
                source_component=self._config.source_component,
                skip_gps_check=self._config.skip_gps_check,
            )
        self.disconnect()
        return self.connect(timeout_s)

    @property
    def telemetry_rate_errors(self) -> dict[int, str]:
        """最近一次 ``connect``/心跳后，SET_MESSAGE_INTERVAL 发送失败的 msg_id → 原因。

        空字典表示请求都发出去了（飞控是否 ACK 另说）。真机若位置一直为 None，
        先看这里是否非空，再查飞控默认流与波特率。
        """

        return dict(self._telemetry_rate_errors)

    def _create_transport(self) -> MavlinkTransport:
        """开发期默认 pymavlink；单测可覆盖此方法注入假传输。"""

        return PymavlinkTransport()

    def _open_transport(self, timeout_s: float) -> None:
        """只打开套接字/串口，不等心跳。"""

        self._require_timeout(timeout_s)
        self._validate_ids()
        if self._transport is not None:
            self.disconnect()
        parsed = parse_connection_uri(self._config.uri)
        transport = self._create_transport()
        try:
            transport.open(
                parsed,
                self._config.source_system,
                self._config.source_component,
                timeout_s,
            )
        except TransportOpenError as exc:
            raise TransportOpenError(
                f"cannot open {self._config.uri!r}: {exc}",
                uri=self._config.uri,
                device=exc.device or parsed.device,
            ) from exc
        self._transport = transport

    def disconnect(self) -> None:
        """关闭连接。详见 ``FlightController.disconnect``。"""

        if self._transport is not None:
            self._transport.close()
            self._transport = None
        self._autopilot = None
        self._vehicle_class = None
        self._capabilities = None
        self._got_heartbeat = False
        self._last_heartbeat_sent = 0.0
        self._last_fc_heartbeat_mono = 0.0
        self._status_texts.clear()
        self._telemetry_rate_errors.clear()
        self._clear_cache()

    def wait_heartbeat(self, timeout_s: float = 15.0) -> AutopilotType:
        """等待心跳并识别固件。详见 ``FlightController.wait_heartbeat``。"""

        self._require_timeout(timeout_s)
        transport = self._require_transport()
        deadline = time.monotonic() + timeout_s
        packets_seen = 0
        fc_heartbeats = 0
        other_heartbeats = 0
        last_msg_type: str | None = None
        while time.monotonic() < deadline:
            self._maybe_send_heartbeat()
            remaining = max(0.05, deadline - time.monotonic())
            # 不按类型过滤：只收 HEARTBEAT 会丢弃同批次 GPS/位置包。
            msg = transport.recv(blocking=True, timeout_s=min(1.0, remaining))
            if msg is None:
                continue
            packets_seen += 1
            last_msg_type = msg.get_type()
            self._update_cache(msg)
            if msg.get_type() != "HEARTBEAT":
                continue
            if _is_own_source(
                int(msg.get_srcSystem()),
                int(msg.get_srcComponent()),
                self._config.source_system,
                self._config.source_component,
            ):
                continue
            if not is_flight_controller_autopilot(int(msg.autopilot)):
                other_heartbeats += 1
                continue  # 相机 / OSD / GCS，不能当作飞控目标
            fc_heartbeats += 1
            # 后续 COMMAND_LONG 必须写到飞控的 sys/comp id
            transport.target_system = int(msg.get_srcSystem())
            transport.target_component = int(msg.get_srcComponent())
            self._got_heartbeat = True
            self._maybe_send_heartbeat()
            self._request_telemetry_rates()
            self._drain()
            if self._autopilot is None:
                raise NotConnectedError("HEARTBEAT did not identify autopilot")
            return self._autopilot
        raise self._heartbeat_timeout_error(
            timeout_s, packets_seen, fc_heartbeats, other_heartbeats, last_msg_type
        )

    def telemetry(self) -> Telemetry:
        """非阻塞读取遥测。详见 ``FlightController.telemetry``。"""

        self._require_transport()
        self._drain()
        self._check_link_alive()
        return self._snapshot()

    def wait_telemetry(
        self,
        predicate: Callable[[Telemetry], bool],
        timeout_s: float,
    ) -> Telemetry:
        """阻塞收包直到条件成立。超时抛 ``TelemetryTimeoutError``。"""

        self._require_timeout(timeout_s)
        self._require_transport()
        self._check_link_alive()
        deadline = time.monotonic() + timeout_s
        snap = self.telemetry()
        while True:
            if predicate(snap):
                return snap
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._check_link_alive()
                hint = ""
                if snap.position is None and self._telemetry_rate_errors:
                    failed = ", ".join(
                        f"{mid}:{reason}" for mid, reason in sorted(self._telemetry_rate_errors.items())
                    )
                    hint = f"; telemetry rate request errors: {failed}"
                raise TelemetryTimeoutError(
                    f"wait_telemetry timed out after {timeout_s}s{hint}",
                    timeout_s=timeout_s,
                    telemetry=snap,
                )
            # 阻塞在 recv 上，不用 sleep；再排空缓冲以免漏掉同批次位置包
            self._pump(min(0.2, remaining))
            self._drain()
            self._check_link_alive()
            snap = self._snapshot()

    def spin(self, duration_s: float) -> Telemetry:
        """收包 duration_s 秒。详见 ``FlightController.spin``。"""

        self._require_timeout(duration_s)
        self._require_transport()
        self._check_link_alive()
        deadline = time.monotonic() + duration_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._check_link_alive()
                return self._snapshot()
            self._pump(min(0.2, remaining))
            self._drain()
            self._check_link_alive()

    def _snapshot(self) -> Telemetry:
        """从缓存组装快照，不再读套接字。"""

        mode = FlightMode.UNKNOWN
        if self._got_heartbeat and self._autopilot is not None:
            vehicle = self._vehicle_class or VehicleClass.COPTER
            mode = flight_mode_from_heartbeat(self._autopilot, self._custom_mode, vehicle)
        return Telemetry(
            autopilot=self._autopilot,
            vehicle_class=self._vehicle_class,
            capabilities=self._capabilities,
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

        self._require_capability(Capability.ARM)
        self._maybe_send_heartbeat()
        try:
            self._command_long(
                mav.MAV_CMD_COMPONENT_ARM_DISARM,
                1.0,  # param1=1 解锁；0 上锁
                0.0,  # param2=0 不强制；21196 为强制解锁（本库不使用）
            )
        except CommandFailedError as exc:
            summary = self._status_text_summary()
            message = (
                f"arm denied: {exc}. Check pre-arm in the GCS (sensors, GPS, safety "
                "switch, battery, fence). This library does not force-arm."
            )
            if summary:
                message = f"{message} STATUSTEXT: {summary}"
            raise ArmDeniedError(message, status_text=summary) from exc

    def disarm(self) -> None:
        """上锁。详见 ``FlightController.disarm``。"""

        self._require_capability(Capability.ARM)
        self._command_long(
            mav.MAV_CMD_COMPONENT_ARM_DISARM,
            0.0,
            0.0,
        )

    def set_mode(self, mode: FlightMode) -> None:
        """切换语义模式。详见 ``FlightController.set_mode``。"""

        self._require_capability(Capability.SET_MODE)
        if mode is FlightMode.UNKNOWN:
            raise UnsupportedModeError("UNKNOWN cannot be used with set_mode")
        # _require_capability 已保证 _got_heartbeat 且 _autopilot 非 None。
        if self._autopilot is None:
            raise NotConnectedError("no HEARTBEAT yet; call connect() first")
        vehicle = self._vehicle_class or VehicleClass.COPTER
        custom = custom_mode_for(self._autopilot, mode, vehicle)
        transport = self._require_transport()
        transport.send_set_mode(mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom)
        deadline = time.monotonic() + _MODE_WAIT_S
        while time.monotonic() < deadline:
            self._pump(0.2)
            current = flight_mode_from_heartbeat(self._autopilot, self._custom_mode, vehicle)
            if current is mode:
                return
            # ArduCopter TAKEOFF 映射为 GUIDED，回读为 GUIDED 即视为成功
            if (
                mode is FlightMode.TAKEOFF
                and current is FlightMode.GUIDED
                and vehicle is VehicleClass.COPTER
            ):
                return
        raise CommandFailedError(
            f"timed out switching to {mode.value} (custom_mode={self._custom_mode})"
        )

    def takeoff(self, altitude_m: float) -> None:
        """起飞到相对高。详见 ``FlightController.takeoff``。"""

        self._require_capability(Capability.TAKEOFF)
        alt = self._require_finite("altitude_m", altitude_m)
        vehicle = self._vehicle_class or VehicleClass.COPTER
        max_alt = takeoff_altitude_max_m(vehicle)
        if alt <= 0 or alt > max_alt:
            raise InvalidArgumentError(
                f"altitude_m must be in (0, {max_alt}] m for {vehicle.value}, got {alt}"
            )
        self._drain()
        if not self._config.skip_gps_check:
            self._require_3d_fix()
        if self._autopilot is AutopilotType.PX4:
            # PX4：切到 AUTO.TAKEOFF 后飞控在 arm 时自动起飞，不再重复发
            # MAV_CMD_NAV_TAKEOFF（该命令在 TAKEOFF 模式下多余，部分固件会
            # 返回 IN_PROGRESS 或忽略）。
            self.set_mode(FlightMode.TAKEOFF)
        elif vehicle is VehicleClass.COPTER:
            # ArduCopter：GUIDED + NAV_TAKEOFF（param7=目标高）
            self.set_mode(FlightMode.GUIDED)
        else:
            # ArduPlane / QuadPlane：TAKEOFF 模式 + NAV_TAKEOFF 设定目标高
            self.set_mode(FlightMode.TAKEOFF)
        if not self._armed:
            self.arm()
        if self._autopilot is AutopilotType.PX4:
            return
        # param7 = 相对 Home 高度（米）
        self._command_long(
            mav.MAV_CMD_NAV_TAKEOFF,
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

        self._require_capability(Capability.GOTO)
        latitude, longitude, alt = self._require_global_point(lat, lon, alt_m)

        if self._autopilot is AutopilotType.PX4:
            try:
                self._command_long(
                    mav.MAV_CMD_DO_REPOSITION,
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
                if exc.result != mav.MAV_RESULT_UNSUPPORTED:
                    raise

        self.set_mode(FlightMode.GUIDED)
        self._send_position_target(
            _POS_TYPEMASK_POSITION_ONLY,
            deg_to_degE7(latitude),
            deg_to_degE7(longitude),
            alt,
            0.0,
            0.0,
            0.0,
        )

    def land(self) -> None:
        """降落。详见 ``FlightController.land``。"""

        self._require_capability(Capability.LAND)
        try:
            self.set_mode(FlightMode.LAND)
            return
        except UnsupportedModeError:
            # 该固件/机型没有 LAND 语义模式时，退而发 NAV_LAND 命令。
            # set_mode 超时（CommandFailedError）不 fallback：模式可能已生效，
            # 再发命令多余甚至冲突。
            self._command_long(mav.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)

    def rtl(self) -> None:
        """返航。详见 ``FlightController.rtl``。"""

        self._require_capability(Capability.RTL)
        try:
            self.set_mode(FlightMode.RTL)
            return
        except UnsupportedModeError:
            self._command_long(mav.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def upload_mission(self, items: Sequence[MissionItem], timeout_s: float = _MISSION_TIMEOUT_S) -> None:
        """上传航线。详见 ``FlightController.upload_mission``。"""

        self._require_capability(Capability.MISSION)
        self._require_timeout(timeout_s)
        validated = validate_mission_items(items)
        transport = self._require_transport()
        transport.send_mission_clear_all(mav.MAV_MISSION_TYPE_MISSION)
        self._wait_mission_ack(min(5.0, timeout_s))
        transport.send_mission_count(len(validated), mav.MAV_MISSION_TYPE_MISSION)
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CommandFailedError("mission upload timed out")
            msg = self._pump(min(0.5, remaining))
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                seq = int(msg.seq)
                if seq < 0 or seq >= len(validated):
                    raise CommandFailedError(f"mission request seq {seq} out of range")
                item = validated[seq]
                transport.send_mission_item_int(
                    seq,
                    mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    item.command,
                    1 if seq == 0 else 0,
                    1,
                    item.hold_s,
                    item.accept_radius_m,
                    0.0,
                    float("nan"),
                    deg_to_degE7(item.latitude_deg),
                    deg_to_degE7(item.longitude_deg),
                    item.altitude_m,
                    mav.MAV_MISSION_TYPE_MISSION,
                )
            elif mtype == "MISSION_ACK":
                result = int(msg.type)
                if result != mav.MAV_MISSION_ACCEPTED:
                    raise CommandFailedError(
                        f"mission upload rejected, MAV_MISSION_RESULT={result}"
                    )
                # 飞控有权在未请求完所有航点时发 ACCEPTED（例如只取了部分点就
                # 判定可执行）。收到 ACCEPTED 即视为上传完成，不再卡到超时。
                return

    def start_mission(self) -> None:
        """开始航线。详见 ``FlightController.start_mission``。"""

        self._require_capability(Capability.MISSION)
        self.set_mode(FlightMode.MISSION)
        try:
            self._command_long(mav.MAV_CMD_MISSION_START)
        except CommandFailedError as exc:
            # ArduPilot 常不实现 MISSION_START；切到 MISSION 模式即足够。
            if exc.result is not None and exc.result != mav.MAV_RESULT_UNSUPPORTED:
                raise

    def pause_mission(self) -> None:
        """暂停航线。详见 ``FlightController.pause_mission``。"""

        self._require_capability(Capability.MISSION)
        try:
            self._command_long(mav.MAV_CMD_DO_PAUSE_CONTINUE, 0.0)
        except CommandFailedError:
            self.set_mode(FlightMode.HOLD)

    def clear_mission(self, timeout_s: float = 5.0) -> None:
        """清除航线。详见 ``FlightController.clear_mission``。"""

        self._require_capability(Capability.MISSION)
        self._require_timeout(timeout_s)
        self._require_transport().send_mission_clear_all(mav.MAV_MISSION_TYPE_MISSION)
        self._wait_mission_ack(timeout_s)

    def set_position(self, lat: float, lon: float, alt_m: float) -> None:
        """发送位置设定点。详见 ``FlightController.set_position``。"""

        self._require_capability(Capability.OFFBOARD)
        latitude, longitude, alt = self._require_global_point(lat, lon, alt_m)
        lat_i = deg_to_degE7(latitude)
        lon_i = deg_to_degE7(longitude)

        def _send() -> None:
            self._send_position_target(
                _POS_TYPEMASK_POSITION_ONLY,
                lat_i,
                lon_i,
                alt,
                0.0,
                0.0,
                0.0,
            )

        # PX4：切 OFFBOARD 前总线上至少要有设定点；切完再发一帧供流控。
        if not self._in_guided():
            _send()
            self._ensure_guided()
        _send()

    def set_velocity(self, north_mps: float, east_mps: float, down_mps: float) -> None:
        """发送 NED 速度设定点。详见 ``FlightController.set_velocity``。"""

        self._require_capability(Capability.OFFBOARD)
        vn = self._require_finite("north_mps", north_mps)
        ve = self._require_finite("east_mps", east_mps)
        vd = self._require_finite("down_mps", down_mps)
        for name, value in (("north_mps", vn), ("east_mps", ve), ("down_mps", vd)):
            if abs(value) > _VELOCITY_MAX_MPS:
                raise InvalidArgumentError(
                    f"{name} must be in [-{_VELOCITY_MAX_MPS}, {_VELOCITY_MAX_MPS}] m/s, got {value}"
                )

        def _send() -> None:
            self._send_position_target(_POS_TYPEMASK_VELOCITY_NED, 0, 0, 0.0, vn, ve, vd)

        if not self._in_guided():
            _send()
            self._ensure_guided()
        _send()

    def set_gimbal(self, pitch_deg: float, yaw_deg: float) -> None:
        """云台指向。详见 ``FlightController.set_gimbal``。"""

        self._require_capability(Capability.GIMBAL)
        pitch = self._require_finite("pitch_deg", pitch_deg)
        yaw = self._require_finite("yaw_deg", yaw_deg)
        if pitch < -90.0 or pitch > 90.0:
            raise InvalidArgumentError(f"pitch_deg must be in [-90, 90], got {pitch}")
        if yaw < -180.0 or yaw > 180.0:
            raise InvalidArgumentError(f"yaw_deg must be in [-180, 180], got {yaw}")
        try:
            self._command_long(
                mav.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                pitch,
                yaw,
                float("nan"),
                float("nan"),
                0.0,
                0.0,
                0.0,
            )
        except CommandFailedError as exc:
            if exc.result != mav.MAV_RESULT_UNSUPPORTED:
                raise
            self._command_long(
                mav.MAV_CMD_DO_MOUNT_CONTROL,
                pitch,
                0.0,
                yaw,
                0.0,
                0.0,
                0.0,
                float(mav.MAV_MOUNT_MODE_MAVLINK_TARGETING),
            )

    def center_gimbal(self) -> None:
        """云台回中。详见 ``FlightController.center_gimbal``。"""

        self.set_gimbal(0.0, 0.0)

    def _in_guided(self) -> bool:
        """当前语义模式是否已是 GUIDED/OFFBOARD。"""

        if self._autopilot is None:
            return False
        vehicle = self._vehicle_class or VehicleClass.COPTER
        current = flight_mode_from_heartbeat(self._autopilot, self._custom_mode, vehicle)
        return current is FlightMode.GUIDED

    def _ensure_guided(self) -> None:
        """已在 GUIDED/OFFBOARD 则不再切模式。"""

        if not self._in_guided():
            self.set_mode(FlightMode.GUIDED)

    def _require_global_point(self, lat: float, lon: float, alt_m: float) -> tuple[float, float, float]:
        """校验 goto / setpoint 的经纬高。"""

        latitude = self._require_finite("lat", lat)
        longitude = self._require_finite("lon", lon)
        alt = self._require_finite("alt_m", alt_m)
        if latitude < -90.0 or latitude > 90.0:
            raise InvalidArgumentError(f"lat must be in [-90, 90], got {latitude}")
        if longitude < -180.0 or longitude > 180.0:
            raise InvalidArgumentError(f"lon must be in [-180, 180], got {longitude}")
        if alt <= 0 or alt > _GOTO_ALT_MAX_M:
            raise InvalidArgumentError(f"alt_m must be in (0, {_GOTO_ALT_MAX_M}] m, got {alt}")
        return latitude, longitude, alt

    def _send_position_target(
        self,
        type_mask: int,
        lat_int: int,
        lon_int: int,
        alt: float,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        """发一条 SET_POSITION_TARGET_GLOBAL_INT。"""

        self._require_transport().send_position_target_global_int(
            0,
            mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            lat_int,
            lon_int,
            alt,
            vx,
            vy,
            vz,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _wait_mission_ack(self, timeout_s: float) -> None:
        """等待 MISSION_ACK.type == ACCEPTED。"""

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            msg = self._pump(min(0.2, remaining))
            if msg is None or msg.get_type() != "MISSION_ACK":
                continue
            result = int(msg.type)
            if result == mav.MAV_MISSION_ACCEPTED:
                return
            raise CommandFailedError(f"mission rejected, MAV_MISSION_RESULT={result}")
        raise CommandFailedError("mission ACK timed out")

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

        transport = self._require_transport()
        transport.send_command_long(
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
            self._check_link_alive()
            if msg is None or msg.get_type() != "COMMAND_ACK":
                continue
            if int(msg.command) != int(command):
                continue
            result = int(msg.result)
            if result == mav.MAV_RESULT_IN_PROGRESS:
                continue
            if result == mav.MAV_RESULT_ACCEPTED:
                return
            raise CommandFailedError(
                f"command {command} failed, MAV_RESULT={result}",
                command=command,
                result=result,
            )
        raise CommandFailedError(f"command {command} ACK timed out", command=command)

    def _request_telemetry_rates(self) -> None:
        """只订位置 / GPS / 姿态 / 电源，约 4 Hz。

        不发送 MAV_DATA_STREAM_ALL，避免在 57600 UART 上拉高 IMU 堵口。
        消息 ID：1 SYS_STATUS，24 GPS_RAW_INT，30 ATTITUDE，
        33 GLOBAL_POSITION_INT，147 BATTERY_STATUS，280 GIMBAL_MANAGER_INFORMATION。

        发送失败写入 ``telemetry_rate_errors`` 并打 warning，不抛异常（仍依赖飞控默认流）。
        不在此等待 COMMAND_ACK，以免拖慢 connect。

        msg_id=280（GIMBAL_MANAGER_INFORMATION）在总线上无云台时飞控常返回
        FAILED/UNSUPPORTED，属正常现象；云台能力以 HEARTBEAT(MAV_TYPE_GIMBAL)
        或 GIMBAL_MANAGER_INFORMATION 实际出现为准，不依赖该订阅是否成功。
        排障时若 ``telemetry_rate_errors`` 只含 280 一项，可忽略。
        """

        self._telemetry_rate_errors.clear()
        transport = self._require_transport()
        for msg_id in (1, 24, 30, 33, 147, 280):
            try:
                transport.send_command_long(
                    mav.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    float(msg_id),
                    _TELEMETRY_INTERVAL_US,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception as exc:
                reason = f"{type(exc).__name__}: {exc}"
                self._telemetry_rate_errors[msg_id] = reason
                _LOG.warning(
                    "SET_MESSAGE_INTERVAL msg_id=%s failed: %s",
                    msg_id,
                    reason,
                )

    def _maybe_send_heartbeat(self) -> None:
        """距上次发送满 1 秒则发本端 HEARTBEAT，供飞控判断链路仍在。

        输入: 无（使用 ConnectionConfig 的 sys/comp 与组件角色）。
        输出: None。未 connect 时直接返回。
        阻塞: 否。
        """

        if self._transport is None:
            return
        now = time.monotonic()
        if self._last_heartbeat_sent > 0.0 and now - self._last_heartbeat_sent < _HEARTBEAT_PERIOD_S:
            return
        self._send_local_heartbeat()
        self._last_heartbeat_sent = now

    def _send_local_heartbeat(self) -> None:
        """按本端组件 ID 选择 GCS 或机载计算机心跳类型，发送一帧。

        输入: 无。comp=191（MAV_COMP_ID_ONBOARD_COMPUTER，机载计算机默认）发
        MAV_TYPE_ONBOARD_CONTROLLER；其余（含 comp=190 地面站）发 MAV_TYPE_GCS。
        注意 comp=1 是飞控自身组件号，机载计算机不应使用，否则会被
        ``_is_own_source`` 当作飞控回环丢掉飞控心跳。
        输出: None。报文头 sys/comp 由 pymavlink 连接的 source_* 填写。
        """

        transport = self._require_transport()
        if self._config.source_component == mav.MAV_COMP_ID_ONBOARD_COMPUTER:
            mav_type = mav.MAV_TYPE_ONBOARD_CONTROLLER
        else:
            mav_type = mav.MAV_TYPE_GCS
        transport.send_heartbeat(
            mav_type,
            mav.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

    def _require_3d_fix(self) -> None:
        """起飞前检查 GPS_RAW_INT.fix_type >= 3。"""
        if self._gps_fix is None or self._gps_fix.value < GpsFixType.FIX_3D.value:
            raise GpsNotFixedError(
                f"takeoff needs 3D GPS (fix_type>=3), current={self._gps_fix}"
            )

    def _require_capability(self, capability: Capability) -> None:
        """未心跳或该机型/SDK 无此能力时拒绝发命令。"""

        self._require_heartbeat()
        profile = self._capabilities
        if profile is None or not profile.supports(capability):
            airframe = self._vehicle_class.value if self._vehicle_class else "unknown"
            raise UnsupportedCapabilityError(
                f"{capability.value} is not available on {airframe}"
            )

    def _enable_capability(self, capability: Capability) -> None:
        """在现有画像上打开一项能力（例如检测到云台）。"""

        if self._capabilities is None:
            return
        if self._capabilities.supports(capability):
            return
        self._capabilities = with_capability(self._capabilities, capability)

    def _require_heartbeat(self) -> None:
        """尚未识别飞控时禁止解锁/改模式等控制命令。"""
        self._require_transport()
        if not self._got_heartbeat or self._autopilot is None:
            raise NotConnectedError("no HEARTBEAT yet; call connect() first")
        self._check_link_alive()

    def _check_link_alive(self) -> None:
        """已识别飞控后，心跳过期则抛 LinkLostError。wait_heartbeat 期间不调用。"""

        if not self._got_heartbeat:
            return
        if self._last_fc_heartbeat_mono <= 0.0:
            raise LinkLostError(
                "no flight-controller HEARTBEAT age recorded; call reconnect()",
                age_s=None,
            )
        age = time.monotonic() - self._last_fc_heartbeat_mono
        if age > _LINK_LOST_S:
            raise LinkLostError(
                f"no flight-controller HEARTBEAT for {age:.1f}s; call reconnect()",
                age_s=age,
            )

    def _record_status_text(self, raw: object) -> None:
        """把 STATUSTEXT 文本记入环形缓冲，供解锁失败时附在英文异常里。"""

        if isinstance(raw, bytes):
            text = raw.decode("utf-8", errors="replace")
        else:
            text = str(raw)
        text = text.replace("\x00", "").strip()
        if text:
            self._status_texts.append(text)

    def _status_text_summary(self) -> str | None:
        """最近若干条 STATUSTEXT，用 `` | `` 拼接；无则 None。"""

        if not self._status_texts:
            return None
        return " | ".join(self._status_texts)

    def _heartbeat_timeout_error(
        self,
        timeout_s: float,
        packets_seen: int,
        fc_heartbeats: int,
        other_heartbeats: int,
        last_msg_type: str | None,
    ) -> HeartbeatTimeoutError:
        """按收包计数生成带排障提示的 HeartbeatTimeoutError。"""

        if packets_seen == 0:
            hint = (
                "no MAVLink frames received; check URI, serial permissions, baud, "
                "and that the vehicle is powered"
            )
        elif other_heartbeats > 0 and fc_heartbeats == 0:
            hint = "HEARTBEAT seen from GCS/camera/OSD, not PX4/ArduPilot"
        else:
            hint = (
                "MAVLink frames seen but no PX4/ArduPilot HEARTBEAT "
                "(loopback or non-FC traffic)"
            )
        message = (
            f"no flight-controller HEARTBEAT within {timeout_s}s; "
            f"packets_seen={packets_seen} fc_heartbeats={fc_heartbeats} "
            f"other_heartbeats={other_heartbeats} last_msg_type={last_msg_type}; {hint}"
        )
        return HeartbeatTimeoutError(
            message,
            packets_seen=packets_seen,
            fc_heartbeats=fc_heartbeats,
            other_heartbeats=other_heartbeats,
            last_msg_type=last_msg_type,
        )

    def _require_transport(self) -> MavlinkTransport:
        """返回已打开的 Transport；未 connect 则抛 NotConnectedError。"""
        if self._transport is None:
            raise NotConnectedError("not connected; call connect() first")
        return self._transport

    def _validate_ids(self) -> None:
        """source_system / source_component 必须是 [1, 255] 的整数。"""
        for name, value in (
            ("source_system", self._config.source_system),
            ("source_component", self._config.source_component),
        ):
            if not isinstance(value, int) or value < 1 or value > 255:
                raise InvalidArgumentError(f"{name} must be an integer in [1, 255]")

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
        transport = self._require_transport()
        self._maybe_send_heartbeat()
        # timeout=0 时 pymavlink 会在 recv_msg 之前就因超时返回 None。
        if timeout_s <= 0:
            msg = transport.recv(blocking=False, timeout_s=None)
        else:
            msg = transport.recv(blocking=True, timeout_s=timeout_s)
        if msg is not None:
            self._update_cache(msg)
        return msg

    def _update_cache(self, msg: Any) -> None:
        """按消息类型刷新遥测快照。单位换算见各分支注释。

        忽略本端 (sys,comp) 回环。云台 HEARTBEAT / GIMBAL_MANAGER_INFORMATION
        在 target_system 过滤之前识别（云台可能有自己的 sysid）。锁定飞控后，
        只接受该 sysid 的其它遥测，避免机上总线上的相机/OSD 覆盖位置或解锁状态。
        HEARTBEAT 仅在 autopilot 为 PX4 / ArduPilot 时更新固件与解锁标志。
        """
        src_system = int(msg.get_srcSystem())
        src_component = int(msg.get_srcComponent())
        if _is_own_source(
            src_system,
            src_component,
            self._config.source_system,
            self._config.source_component,
        ):
            return
        mtype = msg.get_type()
        # 云台消息可能来自云台自己的 sysid（与飞控不同），必须在 target_system
        # 过滤之前识别，否则锁定飞控后云台 HEARTBEAT / GIMBAL_MANAGER_INFORMATION
        # 会被丢弃，GIMBAL 能力永远无法打开。
        if mtype == "HEARTBEAT" and int(getattr(msg, "type", 0)) == mav.MAV_TYPE_GIMBAL:
            self._enable_capability(Capability.GIMBAL)
            return
        if mtype == "GIMBAL_MANAGER_INFORMATION":
            self._enable_capability(Capability.GIMBAL)
            return
        # 锁定飞控后只接受该 sysid 的遥测，避免相机/OSD 覆盖位置或解锁状态。
        if self._got_heartbeat and self._transport is not None:
            if src_system != int(self._transport.target_system):
                return
        if mtype == "HEARTBEAT":
            if not is_flight_controller_autopilot(int(msg.autopilot)):
                return
            self._last_fc_heartbeat_mono = time.monotonic()
            had_gimbal = self._capabilities is not None and self._capabilities.supports(
                Capability.GIMBAL
            )
            self._autopilot = _autopilot_from_heartbeat(int(msg.autopilot))
            self._custom_mode = int(msg.custom_mode)
            self._vehicle_class = vehicle_class_from_mav_type(int(msg.type))
            self._capabilities = default_capabilities(self._vehicle_class)
            if had_gimbal:
                self._enable_capability(Capability.GIMBAL)
            armed_flag = mav.MAV_MODE_FLAG_SAFETY_ARMED
            self._armed = bool(int(msg.base_mode) & armed_flag)
        elif mtype == "STATUSTEXT":
            self._record_status_text(getattr(msg, "text", ""))
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
            raise InvalidArgumentError("timeout_s must be a finite number > 0")

    @staticmethod
    def _require_finite(name: str, value: float) -> float:
        """把调用参数规范成有限 float；NaN/Inf 非法。"""
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise InvalidArgumentError(f"{name} must be a finite number")
        return float(value)


def is_flight_controller_autopilot(autopilot: int) -> bool:
    """判断 HEARTBEAT.autopilot 是否为 PX4 或 ArduPilot 飞控。

    输入:
        autopilot: MAV_AUTOPILOT 整型，合法范围由 MAVLink 枚举定义。
    输出:
        True 表示应按飞控处理（可锁定为命令目标）；False 表示相机/OSD/GCS 等。
    """

    return int(autopilot) in (mav.MAV_AUTOPILOT_PX4, mav.MAV_AUTOPILOT_ARDUPILOTMEGA)


def _is_own_source(
    src_system: int,
    src_component: int,
    own_system: int,
    own_component: int,
) -> bool:
    """本端发出的报文（含回环）用 sys+comp 同时匹配，不能只比 sysid。

    机载与飞控常共用 sysid=1，只比系统 ID 会把飞控心跳丢掉。
    """

    return int(src_system) == int(own_system) and int(src_component) == int(own_component)


def _autopilot_from_heartbeat(autopilot: int) -> AutopilotType:
    """HEARTBEAT.autopilot 整型 → AutopilotType。"""
    if autopilot == mav.MAV_AUTOPILOT_PX4:
        return AutopilotType.PX4
    if autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA:
        return AutopilotType.ARDUPILOT
    return AutopilotType.UNKNOWN
