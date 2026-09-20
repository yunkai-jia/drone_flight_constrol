"""连接诊断、掉线与重连，用假 Transport，不依赖真机或 pymavlink。"""

from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from drone_fc.exceptions import (
    ArmDeniedError,
    HeartbeatTimeoutError,
    InvalidArgumentError,
    LinkLostError,
    TelemetryTimeoutError,
    TransportOpenError,
    UnsupportedCapabilityError,
)
from drone_fc.mavlink import constants as mav
from drone_fc.mavlink.controller import MavlinkFlightController, _LINK_LOST_S
from drone_fc.mavlink.connection import ParsedConnection
from drone_fc.mavlink.transport import MavlinkTransport
from drone_fc.mavlink.vehicle import MAV_TYPE_FIXED_WING, MAV_TYPE_QUADROTOR
from drone_fc.mavlink.modes import (
    encode_px4_custom_mode,
    PX4_MAIN_AUTO,
    PX4_MAIN_OFFBOARD,
    PX4_SUB_AUTO_LAND,
    PX4_SUB_AUTO_TAKEOFF,
)
from drone_fc.types import AutopilotType, Capability, ConnectionConfig, MissionItem, VehicleClass


class FakeMsg:
    """最小 MAVLink 消息：get_type / src sys-comp，以及测试需要的字段。"""

    def __init__(
        self,
        mtype: str,
        src_system: int = 1,
        src_component: int = 1,
        **fields: Any,
    ) -> None:
        self._mtype = mtype
        self._src_system = src_system
        self._src_component = src_component
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self) -> str:
        return self._mtype

    def get_srcSystem(self) -> int:
        return self._src_system

    def get_srcComponent(self) -> int:
        return self._src_component


def _fc_heartbeat() -> FakeMsg:
    """PX4 四旋翼飞控心跳（sys=1, comp=1）。"""

    return FakeMsg(
        "HEARTBEAT",
        src_system=1,
        src_component=1,
        type=MAV_TYPE_QUADROTOR,
        autopilot=mav.MAV_AUTOPILOT_PX4,
        base_mode=mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode=0,
        system_status=0,
    )


def _gcs_heartbeat() -> FakeMsg:
    """地面站心跳，不得当成飞控。"""

    return FakeMsg(
        "HEARTBEAT",
        src_system=255,
        src_component=190,
        type=mav.MAV_TYPE_GCS,
        autopilot=mav.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=0,
    )


def _plane_heartbeat() -> FakeMsg:
    """PX4 固定翼心跳：能力画像无独立 LAND。"""

    return FakeMsg(
        "HEARTBEAT",
        src_system=1,
        src_component=1,
        type=MAV_TYPE_FIXED_WING,
        autopilot=mav.MAV_AUTOPILOT_PX4,
        base_mode=mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode=0,
        system_status=0,
    )


class FakeTransport(MavlinkTransport):
    """脚本化收包；open/close 计数供 reconnect 断言。"""

    def __init__(self, messages: list[FakeMsg] | None = None) -> None:
        self.messages: list[FakeMsg] = list(messages or [])
        self.opened = 0
        self.closed = 0
        self.target_system = 0
        self.target_component = 0
        self.sent: list[tuple[str, tuple[Any, ...]]] = []
        self.echo_guided_on_set_mode = False
        # 设为非 None 时，send_set_mode 会回显一条带该 custom_mode 的心跳，
        # 供 set_mode 的回读循环确认；None 表示不回显。
        self.echo_custom_mode_on_set_mode: int | None = None
        # 回显心跳的 autopilot 字段，默认 PX4；ArduPilot 测试设为 ARDUPILOTMEGA。
        self.echo_autopilot: int = mav.MAV_AUTOPILOT_PX4
        # 设为 True 时，send_command_long 会回一条 COMMAND_ACK(ACCEPTED)，
        # 供 _command_long 的等待循环走完（arm / NAV_TAKEOFF 等）。
        self.ack_arm = False
        self.last_parsed: ParsedConnection | None = None

    def open(
        self,
        parsed: ParsedConnection,
        source_system: int,
        source_component: int,
        timeout_s: float,
    ) -> None:
        self.opened += 1
        self.last_parsed = parsed

    def close(self) -> None:
        self.closed += 1

    def recv(self, blocking: bool, timeout_s: float | None) -> Any | None:
        if self.messages:
            return self.messages.pop(0)
        return None

    def send_heartbeat(
        self,
        mav_type: int,
        autopilot: int,
        base_mode: int,
        custom_mode: int,
        system_status: int,
    ) -> None:
        self.sent.append(("heartbeat", (mav_type, autopilot, base_mode, custom_mode, system_status)))

    def send_set_mode(self, base_mode: int, custom_mode: int) -> None:
        self.sent.append(("set_mode", (base_mode, custom_mode)))
        if self.echo_guided_on_set_mode:
            self.messages.append(
                FakeMsg(
                    "HEARTBEAT",
                    src_system=1,
                    src_component=1,
                    type=MAV_TYPE_QUADROTOR,
                    autopilot=mav.MAV_AUTOPILOT_PX4,
                    base_mode=mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode=encode_px4_custom_mode(PX4_MAIN_OFFBOARD),
                    system_status=0,
                )
            )
        elif self.echo_custom_mode_on_set_mode is not None:
            self.messages.append(
                FakeMsg(
                    "HEARTBEAT",
                    src_system=1,
                    src_component=1,
                    type=MAV_TYPE_QUADROTOR,
                    autopilot=self.echo_autopilot,
                    base_mode=mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode=self.echo_custom_mode_on_set_mode,
                    system_status=0,
                )
            )

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
        self.sent.append(
            (
                "command_long",
                (command, confirmation, param1, param2, param3, param4, param5, param6, param7),
            )
        )
        if self.ack_arm and command == mav.MAV_CMD_COMPONENT_ARM_DISARM:
            self.messages.append(
                FakeMsg(
                    "COMMAND_ACK",
                    src_system=1,
                    src_component=1,
                    command=command,
                    result=mav.MAV_RESULT_ACCEPTED,
                )
            )
        elif self.ack_arm:
            # 其它命令（如 NAV_TAKEOFF）也回 ACCEPTED，让 _command_long 走完。
            self.messages.append(
                FakeMsg(
                    "COMMAND_ACK",
                    src_system=1,
                    src_component=1,
                    command=command,
                    result=mav.MAV_RESULT_ACCEPTED,
                )
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
        self.sent.append(
            (
                "position_target",
                (type_mask, lat_int, lon_int, alt, vx, vy, vz),
            )
        )

    def send_mission_count(self, count: int, mission_type: int) -> None:
        self.sent.append(("mission_count", (count, mission_type)))

    def send_mission_clear_all(self, mission_type: int) -> None:
        self.sent.append(("mission_clear_all", (mission_type,)))
        # 默认回一个 ACCEPTED ACK，让 _wait_mission_ack 通过。
        self.messages.append(
            FakeMsg("MISSION_ACK", src_system=1, src_component=1, type=0)
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
        self.sent.append(
            ("mission_item_int", (seq, command, x, y, z))
        )


class BoomTransport(FakeTransport):
    """open 失败，用于 TransportOpenError。"""

    def open(
        self,
        parsed: ParsedConnection,
        source_system: int,
        source_component: int,
        timeout_s: float,
    ) -> None:
        raise TransportOpenError("permission denied", device=parsed.device)


class ControllerWithFake(MavlinkFlightController):
    """把同一 FakeTransport 实例注入 connect / reconnect。"""

    def __init__(self, config: ConnectionConfig, transport: FakeTransport) -> None:
        super().__init__(config)
        self._fake_transport = transport

    def _create_transport(self) -> MavlinkTransport:
        return self._fake_transport


def _controller(transport: FakeTransport) -> ControllerWithFake:
    return ControllerWithFake(
        ConnectionConfig(uri="udp:127.0.0.1:14540"),
        transport,
    )


class TransportOpenTests(unittest.TestCase):
    def test_open_failure_is_transport_open_error(self) -> None:
        fc = _controller(BoomTransport())
        with self.assertRaises(TransportOpenError) as ctx:
            fc.connect(timeout_s=1.0)
        self.assertEqual(ctx.exception.uri, "udp:127.0.0.1:14540")
        self.assertIn("permission denied", str(ctx.exception))


class HeartbeatDiagnosticsTests(unittest.TestCase):
    def test_timeout_counts_non_fc_traffic(self) -> None:
        fake = FakeTransport(
            [
                FakeMsg("GPS_RAW_INT", fix_type=3, satellites_visible=8),
                _gcs_heartbeat(),
            ]
        )
        fc = _controller(fake)
        with self.assertRaises(HeartbeatTimeoutError) as ctx:
            fc.connect(timeout_s=0.15)
        err = ctx.exception
        self.assertGreaterEqual(err.packets_seen, 2)
        self.assertEqual(err.fc_heartbeats, 0)
        self.assertGreaterEqual(err.other_heartbeats, 1)
        self.assertIsNotNone(err.last_msg_type)
        self.assertIn("GCS/camera/OSD", str(err))

    def test_timeout_with_no_frames(self) -> None:
        fc = _controller(FakeTransport())
        with self.assertRaises(HeartbeatTimeoutError) as ctx:
            fc.connect(timeout_s=0.12)
        err = ctx.exception
        self.assertEqual(err.packets_seen, 0)
        self.assertIsNone(err.last_msg_type)
        self.assertIn("no MAVLink frames received", str(err))

    def test_still_rejects_non_positive_timeout(self) -> None:
        fc = _controller(FakeTransport([_fc_heartbeat()]))
        with self.assertRaises(InvalidArgumentError):
            fc.connect(timeout_s=0)
        with self.assertRaises(InvalidArgumentError):
            fc.connect(timeout_s=float("nan"))


class LinkLostAndReconnectTests(unittest.TestCase):
    def test_wait_telemetry_raises_when_heartbeat_stale(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        self.assertEqual(fc.connect(timeout_s=1.0), AutopilotType.PX4)
        fc._last_fc_heartbeat_mono = time.monotonic() - (_LINK_LOST_S + 1.0)
        with self.assertRaises(LinkLostError) as ctx:
            fc.wait_telemetry(lambda _snap: False, 0.5)
        self.assertIsNotNone(ctx.exception.age_s)
        self.assertGreater(ctx.exception.age_s or 0.0, _LINK_LOST_S)

    def test_telemetry_raises_when_heartbeat_stale(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fc._last_fc_heartbeat_mono = time.monotonic() - (_LINK_LOST_S + 1.0)
        with self.assertRaises(LinkLostError):
            fc.telemetry()

    def test_reconnect_opens_again_and_waits_for_heartbeat(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        self.assertEqual(fake.opened, 1)
        fake.messages.append(_fc_heartbeat())
        autopilot = fc.reconnect(timeout_s=1.0)
        self.assertEqual(autopilot, AutopilotType.PX4)
        self.assertEqual(fake.opened, 2)
        self.assertGreaterEqual(fake.closed, 1)

    def test_reconnect_can_switch_uri(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.messages.append(_fc_heartbeat())
        fc.reconnect(timeout_s=1.0, uri="serial:/dev/ttyACM1:57600")
        self.assertEqual(fc._config.uri, "serial:/dev/ttyACM1:57600")
        self.assertIsNotNone(fake.last_parsed)
        assert fake.last_parsed is not None
        self.assertEqual(fake.last_parsed.device, "/dev/ttyACM1")
        self.assertEqual(fake.last_parsed.baud, 57600)


class ArmStatusTextTests(unittest.TestCase):
    def test_arm_denied_includes_statustext(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.messages.extend(
            [
                FakeMsg("STATUSTEXT", text="PreArm: Accel not calibrated"),
                FakeMsg(
                    "COMMAND_ACK",
                    command=mav.MAV_CMD_COMPONENT_ARM_DISARM,
                    result=mav.MAV_RESULT_DENIED,
                ),
            ]
        )
        with self.assertRaises(ArmDeniedError) as ctx:
            fc.arm()
        self.assertEqual(ctx.exception.status_text, "PreArm: Accel not calibrated")
        self.assertIn("PreArm: Accel not calibrated", str(ctx.exception))


class WaitTelemetryTimeoutTests(unittest.TestCase):
    def test_unmet_predicate_raises(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        with self.assertRaises(TelemetryTimeoutError) as ctx:
            fc.wait_telemetry(lambda _snap: False, 0.15)
        self.assertEqual(ctx.exception.timeout_s, 0.15)
        self.assertIsNotNone(ctx.exception.telemetry)

    def test_met_predicate_returns(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        snap = fc.wait_telemetry(lambda s: s.autopilot is AutopilotType.PX4, 0.5)
        self.assertEqual(snap.autopilot, AutopilotType.PX4)

    def test_spin_returns_without_raising(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        snap = fc.spin(0.12)
        self.assertEqual(snap.autopilot, AutopilotType.PX4)


class PlaneCapabilityTests(unittest.TestCase):
    def test_plane_land_is_unsupported(self) -> None:
        fake = FakeTransport([_plane_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        snap = fc.telemetry()
        self.assertEqual(snap.vehicle_class, VehicleClass.PLANE)
        with self.assertRaises(UnsupportedCapabilityError) as ctx:
            fc.land()
        self.assertIn("land", str(ctx.exception))


class OffboardOrderTests(unittest.TestCase):
    def test_set_position_primes_before_mode_then_sends_again(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fake.echo_guided_on_set_mode = True
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        fc.set_position(47.4, 8.5, 10.0)
        kinds = [name for name, _args in fake.sent if name in ("position_target", "set_mode")]
        self.assertEqual(kinds[:3], ["position_target", "set_mode", "position_target"])

    def test_set_position_when_already_guided_sends_once(self) -> None:
        guided = FakeMsg(
            "HEARTBEAT",
            src_system=1,
            src_component=1,
            type=MAV_TYPE_QUADROTOR,
            autopilot=mav.MAV_AUTOPILOT_PX4,
            base_mode=mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=encode_px4_custom_mode(PX4_MAIN_OFFBOARD),
            system_status=0,
        )
        fake = FakeTransport([guided])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        fc.set_position(47.4, 8.5, 10.0)
        kinds = [name for name, _args in fake.sent if name in ("position_target", "set_mode")]
        self.assertEqual(kinds, ["position_target"])


class TelemetryRateErrorTests(unittest.TestCase):
    def test_rate_request_failure_is_recorded(self) -> None:
        class FlakyRates(FakeTransport):
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
                if command == mav.MAV_CMD_SET_MESSAGE_INTERVAL:
                    raise RuntimeError("bus busy")
                super().send_command_long(
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

        fake = FlakyRates([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        self.assertIn(33, fc.telemetry_rate_errors)
        self.assertIn("bus busy", fc.telemetry_rate_errors[33])


class GimbalCapabilityTests(unittest.TestCase):
    def test_gimbal_heartbeat_from_other_sysid_enables_capability(self) -> None:
        # 锁定飞控后云台心跳来自云台自己的 sysid（这里用 2/220），
        # 不得被 target_system 过滤丢掉，否则 GIMBAL 能力永远打不开。
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.messages.append(
            FakeMsg(
                "HEARTBEAT",
                src_system=2,
                src_component=220,
                type=mav.MAV_TYPE_GIMBAL,
                autopilot=mav.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=0,
            )
        )
        fc.spin(0.1)
        snap = fc.telemetry()
        assert snap.capabilities is not None
        self.assertTrue(snap.capabilities.supports(Capability.GIMBAL))

    def test_gimbal_manager_info_from_other_sysid_enables_capability(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.messages.append(
            FakeMsg(
                "GIMBAL_MANAGER_INFORMATION",
                src_system=2,
                src_component=220,
            )
        )
        fc.spin(0.1)
        snap = fc.telemetry()
        assert snap.capabilities is not None
        self.assertTrue(snap.capabilities.supports(Capability.GIMBAL))


class UploadMissionTests(unittest.TestCase):
    def test_accepted_before_all_items_requested_returns(self) -> None:
        # 飞控只请求了第 0 个航点就发 ACCEPTED：旧逻辑会卡到超时，
        # 现在应直接返回成功。
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.messages.extend(
            [
                FakeMsg("MISSION_REQUEST", src_system=1, src_component=1, seq=0),
                FakeMsg("MISSION_ACK", src_system=1, src_component=1, type=0),
            ]
        )
        items = [
            MissionItem(latitude_deg=47.4, longitude_deg=8.5, altitude_m=10.0),
            MissionItem(latitude_deg=47.41, longitude_deg=8.51, altitude_m=12.0),
        ]
        # 不应抛 CommandFailedError（超时）
        fc.upload_mission(items, timeout_s=2.0)

    def test_rejected_ack_raises(self) -> None:
        fake = FakeTransport([_fc_heartbeat()])
        fc = _controller(fake)
        fc.connect(timeout_s=1.0)
        fake.messages.append(
            FakeMsg("MISSION_ACK", src_system=1, src_component=1, type=1)  # ERROR
        )
        from drone_fc.exceptions import CommandFailedError

        with self.assertRaises(CommandFailedError):
            fc.upload_mission(
                [MissionItem(latitude_deg=47.4, longitude_deg=8.5, altitude_m=10.0)],
                timeout_s=2.0,
            )


class TakeoffTests(unittest.TestCase):
    def _gps_skipping_controller(self, transport: FakeTransport) -> ControllerWithFake:
        return ControllerWithFake(
            ConnectionConfig(uri="udp:127.0.0.1:14540", skip_gps_check=True),
            transport,
        )

    def test_px4_takeoff_does_not_send_nav_takeoff_command(self) -> None:
        # PX4：set_mode(TAKEOFF) + arm 即可，不应再发 MAV_CMD_NAV_TAKEOFF。
        fake = FakeTransport([_fc_heartbeat()])
        fake.echo_custom_mode_on_set_mode = encode_px4_custom_mode(
            PX4_MAIN_AUTO, PX4_SUB_AUTO_TAKEOFF
        )
        fake.ack_arm = True
        fc = self._gps_skipping_controller(fake)
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        fc.takeoff(5.0)
        commands = [
            args[0]
            for name, args in fake.sent
            if name == "command_long"
        ]
        self.assertNotIn(mav.MAV_CMD_NAV_TAKEOFF, commands)
        # 仍应发 arm
        self.assertIn(mav.MAV_CMD_COMPONENT_ARM_DISARM, commands)

    def test_arducopter_takeoff_sends_nav_takeoff_command(self) -> None:
        # ArduCopter：GUIDED + NAV_TAKEOFF（param7=alt）应保留。
        from drone_fc.mavlink import constants as mav_const

        ardu_heartbeat = FakeMsg(
            "HEARTBEAT",
            src_system=1,
            src_component=1,
            type=MAV_TYPE_QUADROTOR,
            autopilot=mav_const.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mav_const.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=4,  # AP_GUIDED
            system_status=0,
        )
        fake = FakeTransport([ardu_heartbeat])
        fake.echo_custom_mode_on_set_mode = 4  # AP_GUIDED 回显
        fake.echo_autopilot = mav_const.MAV_AUTOPILOT_ARDUPILOTMEGA
        fake.ack_arm = True
        fc = self._gps_skipping_controller(fake)
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        fc.takeoff(5.0)
        commands = [
            (args[0], args[8])
            for name, args in fake.sent
            if name == "command_long"
        ]
        self.assertIn((mav.MAV_CMD_NAV_TAKEOFF, 5.0), commands)


class LandRtlFallbackTests(unittest.TestCase):
    def _gps_skipping_controller(self, transport: FakeTransport) -> ControllerWithFake:
        return ControllerWithFake(
            ConnectionConfig(uri="udp:127.0.0.1:14540", skip_gps_check=True),
            transport,
        )

    def test_land_set_mode_success_does_not_send_nav_land(self) -> None:
        # set_mode(LAND) 成功时不应再发 MAV_CMD_NAV_LAND。
        fake = FakeTransport([_fc_heartbeat()])
        fake.echo_custom_mode_on_set_mode = encode_px4_custom_mode(
            PX4_MAIN_AUTO, PX4_SUB_AUTO_LAND
        )
        fake.ack_arm = True
        fc = self._gps_skipping_controller(fake)
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        fc.land()
        commands = [
            args[0] for name, args in fake.sent if name == "command_long"
        ]
        self.assertNotIn(mav.MAV_CMD_NAV_LAND, commands)

    def test_land_set_mode_timeout_does_not_fallback(self) -> None:
        # set_mode 超时（CommandFailedError）不应 fallback 到 NAV_LAND：
        # 模式切换可能已生效，再发命令多余。应直接抛 CommandFailedError。
        fake = FakeTransport([_fc_heartbeat()])
        # 不设 echo_custom_mode_on_set_mode → set_mode 收不到回读 → 超时
        fc = self._gps_skipping_controller(fake)
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        from drone_fc.exceptions import CommandFailedError

        with self.assertRaises(CommandFailedError):
            fc.land()
        commands = [
            args[0] for name, args in fake.sent if name == "command_long"
        ]
        self.assertNotIn(mav.MAV_CMD_NAV_LAND, commands)


class HeartbeatRoleTests(unittest.TestCase):
    def test_comp_191_sends_onboard_controller_heartbeat(self) -> None:
        # 默认 comp=191（机载计算机）应发 MAV_TYPE_ONBOARD_CONTROLLER。
        fake = FakeTransport([_fc_heartbeat()])
        fc = ControllerWithFake(
            ConnectionConfig(
                uri="udp:127.0.0.1:14540",
                source_system=1,
                source_component=191,
            ),
            fake,
        )
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        fc._last_heartbeat_sent = 0.0  # 强制下次 _maybe_send_heartbeat 真正发送
        fc._maybe_send_heartbeat()
        heartbeats = [args for name, args in fake.sent if name == "heartbeat"]
        self.assertTrue(heartbeats)
        self.assertEqual(heartbeats[-1][0], mav.MAV_TYPE_ONBOARD_CONTROLLER)

    def test_comp_190_sends_gcs_heartbeat(self) -> None:
        # comp=190（地面站）应发 MAV_TYPE_GCS。
        fake = FakeTransport([_fc_heartbeat()])
        fc = ControllerWithFake(
            ConnectionConfig(
                uri="udp:127.0.0.1:14540",
                source_system=255,
                source_component=190,
            ),
            fake,
        )
        fc.connect(timeout_s=1.0)
        fake.sent.clear()
        fc._last_heartbeat_sent = 0.0
        fc._maybe_send_heartbeat()
        heartbeats = [args for name, args in fake.sent if name == "heartbeat"]
        self.assertTrue(heartbeats)
        self.assertEqual(heartbeats[-1][0], mav.MAV_TYPE_GCS)


if __name__ == "__main__":
    unittest.main()
