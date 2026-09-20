"""UDP 仿真：真 pymavlink + 协议 mock 启停，不改内部心跳时间戳。

无 pymavlink 时整模块跳过。不依赖真机或串口。
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

try:
    from pymavlink import mavutil

    _HAS_PYMAVLINK = True
except ImportError:
    mavutil = None  # type: ignore[assignment]
    _HAS_PYMAVLINK = False

from drone_fc.exceptions import HeartbeatTimeoutError, LinkLostError, TransportOpenError
from drone_fc.mavlink.controller import MavlinkFlightController, _LINK_LOST_S
from drone_fc.types import AutopilotType, ConnectionConfig, VehicleClass

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
MOCK_SCRIPT = ROOT / "examples" / "mock_px4_mavlink.py"
_MISSING_SERIAL = "serial:/no/such/drone_fc_port:57600"


def _free_udp_port() -> int:
    """临时 bind 再关闭，拿到本机空闲 UDP 端口。"""

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _start_mock(port: int) -> subprocess.Popen[str]:
    """子进程启动协议 mock，向 ``udp:127.0.0.1:port`` 发飞控心跳。"""

    env = os.environ.copy()
    env["PYTHONPATH"] = str(SRC) + os.pathsep + env.get("PYTHONPATH", "")
    return subprocess.Popen(
        [sys.executable, str(MOCK_SCRIPT), "--host", "127.0.0.1", "--port", str(port)],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )


def _stop_proc(proc: subprocess.Popen[str] | None) -> None:
    """terminate，必要时 kill。可重复调用。"""

    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)


def _controller(port: int) -> MavlinkFlightController:
    return MavlinkFlightController(ConnectionConfig(uri=f"udp:127.0.0.1:{port}"))


def _print_diag(exc: BaseException) -> None:
    """Print English diagnostics the same way examples do."""

    print(f"link failed: {exc}", flush=True)
    if isinstance(exc, HeartbeatTimeoutError):
        print(
            f"  packets_seen={exc.packets_seen} fc_heartbeats={exc.fc_heartbeats} "
            f"other_heartbeats={exc.other_heartbeats} last_msg_type={exc.last_msg_type}",
            flush=True,
        )
    elif isinstance(exc, LinkLostError):
        print(f"  age_s={exc.age_s}", flush=True)
    elif isinstance(exc, TransportOpenError):
        print(f"  uri={exc.uri} device={exc.device}", flush=True)


def _send_gcs_heartbeats(port: int, stop: threading.Event) -> None:
    """向客户端监听口发 GCS HEARTBEAT（不是 PX4/ArduPilot）。"""

    assert mavutil is not None
    conn = mavutil.mavlink_connection(
        f"udpout:127.0.0.1:{port}",
        source_system=255,
        source_component=190,
    )
    try:
        while not stop.is_set():
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            stop.wait(0.2)
    finally:
        conn.close()


@unittest.skipUnless(_HAS_PYMAVLINK, "pymavlink required for UDP link simulation")
class UdpLinkSimulationTests(unittest.TestCase):
    """审查人分类：打开失败 / 无飞控心跳 / 连上后心跳停 / 再重连。"""

    def setUp(self) -> None:
        self._procs: list[subprocess.Popen[str]] = []
        self._stop_events: list[threading.Event] = []

    def tearDown(self) -> None:
        for event in self._stop_events:
            event.set()
        for proc in self._procs:
            _stop_proc(proc)

    def _track(self, proc: subprocess.Popen[str]) -> subprocess.Popen[str]:
        self._procs.append(proc)
        return proc

    def test_connect_succeeds_with_mock(self) -> None:
        port = _free_udp_port()
        self._track(_start_mock(port))
        fc = _controller(port)
        try:
            autopilot = fc.connect(timeout_s=5.0)
            self.assertEqual(autopilot, AutopilotType.PX4)
            snap = fc.telemetry()
            self.assertEqual(snap.vehicle_class, VehicleClass.COPTER)
            print(
                f"Link OK firmware={autopilot.value} "
                f"airframe={snap.vehicle_class.value}",
                flush=True,
            )
        finally:
            fc.disconnect()

    def test_timeout_with_no_peer(self) -> None:
        port = _free_udp_port()
        fc = _controller(port)
        try:
            with self.assertRaises(HeartbeatTimeoutError) as ctx:
                fc.connect(timeout_s=1.5)
            err = ctx.exception
            _print_diag(err)
            self.assertEqual(err.fc_heartbeats, 0)
            self.assertEqual(err.other_heartbeats, 0)
            # 本端心跳若 UDP 回环，packets_seen 可能 > 0；无对端时不得看成飞控。
            if err.packets_seen == 0:
                self.assertIsNone(err.last_msg_type)
            else:
                self.assertEqual(err.last_msg_type, "HEARTBEAT")
        finally:
            fc.disconnect()

    def test_timeout_counts_gcs_only_heartbeat(self) -> None:
        port = _free_udp_port()
        stop = threading.Event()
        self._stop_events.append(stop)
        thread = threading.Thread(
            target=_send_gcs_heartbeats,
            args=(port, stop),
            daemon=True,
        )
        thread.start()
        fc = _controller(port)
        try:
            with self.assertRaises(HeartbeatTimeoutError) as ctx:
                fc.connect(timeout_s=2.0)
            err = ctx.exception
            _print_diag(err)
            self.assertEqual(err.fc_heartbeats, 0)
            self.assertGreaterEqual(err.other_heartbeats, 1)
            self.assertGreaterEqual(err.packets_seen, 1)
            self.assertIn("GCS/camera/OSD", str(err))
        finally:
            stop.set()
            fc.disconnect()

    def test_missing_serial_is_transport_open_error(self) -> None:
        fc = MavlinkFlightController(ConnectionConfig(uri=_MISSING_SERIAL))
        try:
            with self.assertRaises(TransportOpenError) as ctx:
                fc.connect(timeout_s=2.0)
            _print_diag(ctx.exception)
            self.assertEqual(ctx.exception.uri, _MISSING_SERIAL)
            self.assertIn(_MISSING_SERIAL, str(ctx.exception))
        finally:
            fc.disconnect()

    def test_kill_mock_raises_link_lost_then_reconnect(self) -> None:
        port = _free_udp_port()
        mock = self._track(_start_mock(port))
        fc = _controller(port)
        try:
            self.assertEqual(fc.connect(timeout_s=5.0), AutopilotType.PX4)
            _stop_proc(mock)
            self.assertIsNotNone(mock.poll())

            with self.assertRaises(LinkLostError) as ctx:
                fc.wait_telemetry(lambda _snap: False, timeout_s=8.0)
            _print_diag(ctx.exception)
            age = ctx.exception.age_s
            self.assertIsNotNone(age)
            self.assertGreater(age or 0.0, _LINK_LOST_S)

            self._track(_start_mock(port))
            self.assertEqual(fc.reconnect(timeout_s=5.0), AutopilotType.PX4)
            snap = fc.telemetry()
            self.assertEqual(snap.autopilot, AutopilotType.PX4)
            print("reconnect OK firmware=px4", flush=True)
        finally:
            fc.disconnect()


if __name__ == "__main__":
    unittest.main()
