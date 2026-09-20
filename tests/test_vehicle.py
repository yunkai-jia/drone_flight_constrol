"""机型归类与能力画像，不依赖真机。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from drone_fc.mavlink.vehicle import (
    MAV_TYPE_FIXED_WING,
    MAV_TYPE_QUADROTOR,
    MAV_TYPE_VTOL_QUADROTOR,
    default_capabilities,
    takeoff_altitude_max_m,
    vehicle_class_from_mav_type,
    with_capability,
)
from drone_fc.types import Capability, VehicleClass


class VehicleClassTests(unittest.TestCase):
    def test_copter_types(self) -> None:
        self.assertEqual(vehicle_class_from_mav_type(MAV_TYPE_QUADROTOR), VehicleClass.COPTER)
        self.assertEqual(vehicle_class_from_mav_type(13), VehicleClass.COPTER)  # hexarotor

    def test_plane_and_vtol(self) -> None:
        self.assertEqual(vehicle_class_from_mav_type(MAV_TYPE_FIXED_WING), VehicleClass.PLANE)
        self.assertEqual(vehicle_class_from_mav_type(MAV_TYPE_VTOL_QUADROTOR), VehicleClass.VTOL)

    def test_rover_is_unknown(self) -> None:
        self.assertEqual(vehicle_class_from_mav_type(10), VehicleClass.UNKNOWN)


class CapabilityTests(unittest.TestCase):
    def test_copter_can_land_and_takeoff(self) -> None:
        profile = default_capabilities(VehicleClass.COPTER)
        self.assertTrue(profile.supports(Capability.TAKEOFF))
        self.assertTrue(profile.supports(Capability.LAND))
        self.assertTrue(profile.supports(Capability.GOTO))
        self.assertTrue(profile.supports(Capability.MISSION))
        self.assertTrue(profile.supports(Capability.OFFBOARD))
        self.assertFalse(profile.supports(Capability.GIMBAL))

    def test_plane_has_no_independent_land(self) -> None:
        profile = default_capabilities(VehicleClass.PLANE)
        self.assertTrue(profile.supports(Capability.TAKEOFF))
        self.assertTrue(profile.supports(Capability.RTL))
        self.assertFalse(profile.supports(Capability.LAND))

    def test_unknown_rejects_flight_commands(self) -> None:
        profile = default_capabilities(VehicleClass.UNKNOWN)
        self.assertFalse(profile.supports(Capability.ARM))
        self.assertFalse(profile.supports(Capability.TAKEOFF))
        self.assertEqual(profile.supported, frozenset())


class TakeoffLimitTests(unittest.TestCase):
    def test_plane_allows_higher_than_copter(self) -> None:
        self.assertEqual(takeoff_altitude_max_m(VehicleClass.COPTER), 100.0)
        self.assertEqual(takeoff_altitude_max_m(VehicleClass.VTOL), 100.0)
        self.assertEqual(takeoff_altitude_max_m(VehicleClass.PLANE), 500.0)

    def test_gimbal_appended(self) -> None:
        base = default_capabilities(VehicleClass.COPTER)
        self.assertFalse(base.supports(Capability.GIMBAL))
        updated = with_capability(base, Capability.GIMBAL)
        self.assertTrue(updated.supports(Capability.GIMBAL))
        self.assertFalse(base.supports(Capability.GIMBAL))


if __name__ == "__main__":
    unittest.main()
