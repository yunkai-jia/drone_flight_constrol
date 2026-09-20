"""MAVLink 收发抽象。实现类可换 pymavlink 或 MIT 方言，业务命令不直接碰套接字。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from drone_fc.exceptions import DroneFcError, TransportOpenError
from drone_fc.mavlink.connection import ParsedConnection


class MavlinkTransport(ABC):
    """打开链路、收一帧、发命令。target_system / target_component 由心跳锁定。

    输入/输出见各方法。实现不必线程安全。
    """

    target_system: int = 0
    target_component: int = 0

    @abstractmethod
    def open(
        self,
        parsed: ParsedConnection,
        source_system: int,
        source_component: int,
        timeout_s: float,
    ) -> None:
        """打开 URI 对应的套接字或串口。

        输入:
            parsed: parse_connection_uri 的结果。
            source_system: 本端 sysid，[1, 255]。
            source_component: 本端 compid，[1, 255]。
            timeout_s: 底层 recv 默认超时秒数，必须 > 0。
        输出:
            None。成功后 recv / send_* 可用。
        失败:
            TransportOpenError: 打不开设备（路径、权限、网络）。
        """

    @abstractmethod
    def close(self) -> None:
        """关闭链路。可重复调用。"""

    @abstractmethod
    def recv(self, blocking: bool, timeout_s: float | None) -> Any | None:
        """读取一条已解码的 MAVLink 消息。

        输入:
            blocking: False 时立即返回，可能为 None。
            timeout_s: blocking True 时的最长等待秒数；None 表示实现默认。
        输出:
            带 get_type / get_srcSystem / get_srcComponent 的消息，或 None。
        """

    @abstractmethod
    def send_heartbeat(
        self,
        mav_type: int,
        autopilot: int,
        base_mode: int,
        custom_mode: int,
        system_status: int,
    ) -> None:
        """发送本端 HEARTBEAT。sys/comp 由 open 时的 source_* 填写。"""

    @abstractmethod
    def send_set_mode(self, base_mode: int, custom_mode: int) -> None:
        """向已锁定的飞控发 SET_MODE。"""

    @abstractmethod
    def send_command_long(
        self,
        command: int,
        confirmation: int,
        param1: float,
        param2: float,
        param3: float,
        param4: float,
        param5: float,
        param6: float,
        param7: float,
    ) -> None:
        """向已锁定的飞控发 COMMAND_LONG。"""

    @abstractmethod
    def send_position_target_global_int(
        self,
        time_boot_ms: int,
        coordinate_frame: int,
        type_mask: int,
        lat_int: int,
        lon_int: int,
        alt: float,
        vx: float,
        vy: float,
        vz: float,
        afx: float,
        afy: float,
        afz: float,
        yaw: float,
        yaw_rate: float,
    ) -> None:
        """向已锁定的飞控发 SET_POSITION_TARGET_GLOBAL_INT。"""

    @abstractmethod
    def send_mission_count(self, count: int, mission_type: int) -> None:
        """发送 MISSION_COUNT。count 为航点数，mission_type 一般为 0。"""

    @abstractmethod
    def send_mission_clear_all(self, mission_type: int) -> None:
        """发送 MISSION_CLEAR_ALL。"""

    @abstractmethod
    def send_mission_item_int(
        self,
        seq: int,
        frame: int,
        command: int,
        current: int,
        autocontinue: int,
        param1: float,
        param2: float,
        param3: float,
        param4: float,
        x: int,
        y: int,
        z: float,
        mission_type: int,
    ) -> None:
        """发送 MISSION_ITEM_INT。x/y 为 degE7，z 为相对高米。"""


class PymavlinkTransport(MavlinkTransport):
    """开发期适配器：用 pymavlink 打开连接。商用发行应换成 MIT 方言实现。"""

    def __init__(self) -> None:
        self._master: Any = None

    def open(
        self,
        parsed: ParsedConnection,
        source_system: int,
        source_component: int,
        timeout_s: float,
    ) -> None:
        from pymavlink import mavutil

        kwargs: dict[str, Any] = {
            # Caller-driven reconnect() only; do not let pymavlink reopen the port.
            "autoreconnect": False,
            "source_system": source_system,
            "source_component": source_component,
            "timeout": timeout_s,
        }
        if parsed.baud is not None:
            kwargs["baud"] = parsed.baud
        try:
            self._master = mavutil.mavlink_connection(parsed.device, **kwargs)
        except Exception as exc:
            self._master = None
            raise TransportOpenError(
                f"cannot open {parsed.device!r}: {exc}",
                device=parsed.device,
            ) from exc

    def close(self) -> None:
        if self._master is None:
            return
        try:
            self._master.close()
        except Exception:
            pass
        self._master = None

    def recv(self, blocking: bool, timeout_s: float | None) -> Any | None:
        master = self._require()
        if not blocking:
            return master.recv_match(blocking=False)
        timeout = 1.0 if timeout_s is None else timeout_s
        return master.recv_match(blocking=True, timeout=timeout)

    def send_heartbeat(
        self,
        mav_type: int,
        autopilot: int,
        base_mode: int,
        custom_mode: int,
        system_status: int,
    ) -> None:
        self._require().mav.heartbeat_send(
            mav_type, autopilot, base_mode, custom_mode, system_status
        )

    def send_set_mode(self, base_mode: int, custom_mode: int) -> None:
        master = self._require()
        master.mav.set_mode_send(master.target_system, base_mode, custom_mode)

    def send_command_long(
        self,
        command: int,
        confirmation: int,
        param1: float,
        param2: float,
        param3: float,
        param4: float,
        param5: float,
        param6: float,
        param7: float,
    ) -> None:
        master = self._require()
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            command,
            confirmation,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
        )

    def send_position_target_global_int(
        self,
        time_boot_ms: int,
        coordinate_frame: int,
        type_mask: int,
        lat_int: int,
        lon_int: int,
        alt: float,
        vx: float,
        vy: float,
        vz: float,
        afx: float,
        afy: float,
        afz: float,
        yaw: float,
        yaw_rate: float,
    ) -> None:
        master = self._require()
        master.mav.set_position_target_global_int_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            coordinate_frame,
            type_mask,
            lat_int,
            lon_int,
            alt,
            vx,
            vy,
            vz,
            afx,
            afy,
            afz,
            yaw,
            yaw_rate,
        )

    def send_mission_count(self, count: int, mission_type: int) -> None:
        master = self._require()
        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            int(count),
            int(mission_type),
        )

    def send_mission_clear_all(self, mission_type: int) -> None:
        master = self._require()
        master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component,
            int(mission_type),
        )

    def send_mission_item_int(
        self,
        seq: int,
        frame: int,
        command: int,
        current: int,
        autocontinue: int,
        param1: float,
        param2: float,
        param3: float,
        param4: float,
        x: int,
        y: int,
        z: float,
        mission_type: int,
    ) -> None:
        master = self._require()
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            int(seq),
            int(frame),
            int(command),
            int(current),
            int(autocontinue),
            param1,
            param2,
            param3,
            param4,
            int(x),
            int(y),
            z,
            int(mission_type),
        )

    def _require(self) -> Any:
        if self._master is None:
            raise DroneFcError("transport is not open")
        return self._master

    @property
    def target_system(self) -> int:
        master = self._master
        return 0 if master is None else int(master.target_system)

    @target_system.setter
    def target_system(self, value: int) -> None:
        self._require().target_system = int(value)

    @property
    def target_component(self) -> int:
        master = self._master
        return 0 if master is None else int(master.target_component)

    @target_component.setter
    def target_component(self, value: int) -> None:
        self._require().target_component = int(value)
