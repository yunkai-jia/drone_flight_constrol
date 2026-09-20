"""模式映射单元测试，不依赖真机或 SITL。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from drone_fc.exceptions import UnsupportedModeError
from drone_fc.mavlink.modes import (
    AP_AUTO,
    AP_GUIDED,
    AP_LAND,
    AP_LOITER,
    AP_RTL,
    AP_STABILIZE,
    PLANE_AUTO,
    PLANE_GUIDED,
    PLANE_LOITER,
    PLANE_QLAND,
    PLANE_QLOITER,
    PLANE_QRTL,
    PLANE_TAKEOFF,
    PX4_MAIN_ACRO,
    PX4_MAIN_ALTCTL,
    PX4_MAIN_AUTO,
    PX4_MAIN_MANUAL,
    PX4_MAIN_OFFBOARD,
    PX4_MAIN_POSCTL,
    PX4_MAIN_STABILIZED,
    PX4_SUB_AUTO_LAND,
    PX4_SUB_AUTO_LOITER,
    PX4_SUB_AUTO_MISSION,
    PX4_SUB_AUTO_RTL,
    PX4_SUB_AUTO_TAKEOFF,
    custom_mode_for,
    decode_px4_custom_mode,
    encode_px4_custom_mode,
    flight_mode_from_heartbeat,
)
from drone_fc.types import AutopilotType, FlightMode, VehicleClass


class Px4CodecTests(unittest.TestCase):
    def test_encode_layout(self) -> None:
        value = encode_px4_custom_mode(PX4_MAIN_AUTO, PX4_SUB_AUTO_TAKEOFF)
        self.assertEqual(value, (PX4_SUB_AUTO_TAKEOFF << 24) | (PX4_MAIN_AUTO << 16))
        main_mode, sub_mode = decode_px4_custom_mode(value)
        self.assertEqual(main_mode, PX4_MAIN_AUTO)
        self.assertEqual(sub_mode, PX4_SUB_AUTO_TAKEOFF)

    def test_decode_masks_to_32_bit(self) -> None:
        packed = encode_px4_custom_mode(3, 7) | (0xABC << 32)
        main_mode, sub_mode = decode_px4_custom_mode(packed)
        self.assertEqual(main_mode, 3)
        self.assertEqual(sub_mode, 7)


class Px4MappingTests(unittest.TestCase):
    def test_from_heartbeat_auto_submodes(self) -> None:
        cases = {
            PX4_SUB_AUTO_TAKEOFF: FlightMode.TAKEOFF,
            PX4_SUB_AUTO_LOITER: FlightMode.HOLD,
            PX4_SUB_AUTO_MISSION: FlightMode.MISSION,
            PX4_SUB_AUTO_RTL: FlightMode.RTL,
            PX4_SUB_AUTO_LAND: FlightMode.LAND,
        }
        for sub, expected in cases.items():
            custom = encode_px4_custom_mode(PX4_MAIN_AUTO, sub)
            actual = flight_mode_from_heartbeat(AutopilotType.PX4, custom)
            self.assertEqual(actual, expected, msg=f"sub={sub}")

    def test_from_heartbeat_main_modes(self) -> None:
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.PX4, encode_px4_custom_mode(PX4_MAIN_MANUAL)),
            FlightMode.MANUAL,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.PX4, encode_px4_custom_mode(PX4_MAIN_ACRO)),
            FlightMode.MANUAL,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.PX4, encode_px4_custom_mode(PX4_MAIN_STABILIZED)),
            FlightMode.MANUAL,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.PX4, encode_px4_custom_mode(PX4_MAIN_ALTCTL)),
            FlightMode.MANUAL,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.PX4, encode_px4_custom_mode(PX4_MAIN_POSCTL)),
            FlightMode.HOLD,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.PX4, encode_px4_custom_mode(PX4_MAIN_OFFBOARD)),
            FlightMode.GUIDED,
        )

    def test_unknown_auto_submode(self) -> None:
        custom = encode_px4_custom_mode(PX4_MAIN_AUTO, 15)
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.PX4, custom),
            FlightMode.UNKNOWN,
        )

    def test_to_custom_mode_roundtrip(self) -> None:
        for mode in (
            FlightMode.MANUAL,
            FlightMode.HOLD,
            FlightMode.MISSION,
            FlightMode.RTL,
            FlightMode.LAND,
            FlightMode.TAKEOFF,
            FlightMode.GUIDED,
        ):
            custom = custom_mode_for(AutopilotType.PX4, mode)
            decoded = flight_mode_from_heartbeat(AutopilotType.PX4, custom)
            self.assertEqual(decoded, mode, msg=mode.value)


class ArduPilotMappingTests(unittest.TestCase):
    def test_from_heartbeat(self) -> None:
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.ARDUPILOT, AP_STABILIZE),
            FlightMode.MANUAL,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.ARDUPILOT, AP_AUTO),
            FlightMode.MISSION,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.ARDUPILOT, AP_GUIDED),
            FlightMode.GUIDED,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.ARDUPILOT, AP_LOITER),
            FlightMode.HOLD,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.ARDUPILOT, AP_RTL),
            FlightMode.RTL,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.ARDUPILOT, AP_LAND),
            FlightMode.LAND,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.ARDUPILOT, 99),
            FlightMode.UNKNOWN,
        )

    def test_takeoff_maps_to_guided(self) -> None:
        self.assertEqual(custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.TAKEOFF), AP_GUIDED)
        self.assertEqual(
            flight_mode_from_heartbeat(
                AutopilotType.ARDUPILOT,
                custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.TAKEOFF),
            ),
            FlightMode.GUIDED,
        )

    def test_to_custom_mode(self) -> None:
        expected = {
            FlightMode.MANUAL: AP_STABILIZE,
            FlightMode.HOLD: AP_LOITER,
            FlightMode.MISSION: AP_AUTO,
            FlightMode.RTL: AP_RTL,
            FlightMode.LAND: AP_LAND,
            FlightMode.GUIDED: AP_GUIDED,
        }
        for mode, number in expected.items():
            self.assertEqual(custom_mode_for(AutopilotType.ARDUPILOT, mode), number)


class ArduPlaneMappingTests(unittest.TestCase):
    def test_from_heartbeat(self) -> None:
        self.assertEqual(
            flight_mode_from_heartbeat(
                AutopilotType.ARDUPILOT, PLANE_LOITER, VehicleClass.PLANE
            ),
            FlightMode.HOLD,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(
                AutopilotType.ARDUPILOT, PLANE_GUIDED, VehicleClass.PLANE
            ),
            FlightMode.GUIDED,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(
                AutopilotType.ARDUPILOT, PLANE_TAKEOFF, VehicleClass.PLANE
            ),
            FlightMode.TAKEOFF,
        )
        self.assertEqual(
            flight_mode_from_heartbeat(
                AutopilotType.ARDUPILOT, PLANE_AUTO, VehicleClass.PLANE
            ),
            FlightMode.MISSION,
        )

    def test_to_custom_mode(self) -> None:
        self.assertEqual(
            custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.GUIDED, VehicleClass.PLANE),
            PLANE_GUIDED,
        )
        self.assertEqual(
            custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.TAKEOFF, VehicleClass.PLANE),
            PLANE_TAKEOFF,
        )

    def test_land_unsupported_on_plane(self) -> None:
        with self.assertRaises(UnsupportedModeError):
            custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.LAND, VehicleClass.PLANE)


class ArduVtolMappingTests(unittest.TestCase):
    def test_q_modes(self) -> None:
        self.assertEqual(
            flight_mode_from_heartbeat(
                AutopilotType.ARDUPILOT, PLANE_QLAND, VehicleClass.VTOL
            ),
            FlightMode.LAND,
        )
        self.assertEqual(
            custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.LAND, VehicleClass.VTOL),
            PLANE_QLAND,
        )
        self.assertEqual(
            custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.HOLD, VehicleClass.VTOL),
            PLANE_QLOITER,
        )
        self.assertEqual(
            custom_mode_for(AutopilotType.ARDUPILOT, FlightMode.RTL, VehicleClass.VTOL),
            PLANE_QRTL,
        )


class GuardTests(unittest.TestCase):
    def test_unknown_firmware_reads_as_unknown_mode(self) -> None:
        self.assertEqual(
            flight_mode_from_heartbeat(AutopilotType.UNKNOWN, 4),
            FlightMode.UNKNOWN,
        )

    def test_cannot_set_unknown_mode(self) -> None:
        with self.assertRaises(UnsupportedModeError):
            custom_mode_for(AutopilotType.PX4, FlightMode.UNKNOWN)

    def test_cannot_set_mode_on_unknown_airframe(self) -> None:
        with self.assertRaises(UnsupportedModeError):
            custom_mode_for(
                AutopilotType.ARDUPILOT, FlightMode.HOLD, VehicleClass.UNKNOWN
            )


if __name__ == "__main__":
    unittest.main()
