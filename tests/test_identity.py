"""本端身份与飞控心跳识别，不依赖真机。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from drone_fc.mavlink.controller import (
    _is_own_source,
    is_flight_controller_autopilot,
)
from drone_fc.types import ConnectionConfig


class OwnSourceTests(unittest.TestCase):
    def test_same_sysid_different_component_is_not_own(self) -> None:
        # 机载 sys=1/comp=191，飞控 sys=1/comp=1，不得当回环丢掉
        self.assertFalse(_is_own_source(1, 1, 1, 191))

    def test_own_sys_and_comp_match(self) -> None:
        self.assertTrue(_is_own_source(1, 191, 1, 191))

    def test_gcs_defaults_not_confused_with_fc(self) -> None:
        self.assertFalse(_is_own_source(1, 1, 255, 190))


class FlightControllerAutopilotTests(unittest.TestCase):
    def test_px4_and_ardupilot(self) -> None:
        self.assertTrue(is_flight_controller_autopilot(12))
        self.assertTrue(is_flight_controller_autopilot(3))

    def test_camera_or_gcs_not_fc(self) -> None:
        self.assertFalse(is_flight_controller_autopilot(8))  # MAV_AUTOPILOT_INVALID
        self.assertFalse(is_flight_controller_autopilot(0))


class ConnectionConfigCompanionDefaultsTests(unittest.TestCase):
    def test_onboard_ids(self) -> None:
        cfg = ConnectionConfig(uri="serial:/dev/ttyTHS1:921600")
        self.assertEqual(cfg.source_system, 1)
        self.assertEqual(cfg.source_component, 191)
        self.assertFalse(cfg.skip_gps_check)


if __name__ == "__main__":
    unittest.main()
