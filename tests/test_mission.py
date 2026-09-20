"""航线点校验，不依赖真机。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from drone_fc.exceptions import InvalidArgumentError
from drone_fc.mavlink.mission import deg_to_degE7, validate_mission_items
from drone_fc.types import MissionItem


class DegE7Tests(unittest.TestCase):
    def test_round_trip_scale(self) -> None:
        self.assertEqual(deg_to_degE7(47.397742), 473977420)
        self.assertEqual(deg_to_degE7(-122.0), -1220000000)


class ValidateMissionTests(unittest.TestCase):
    def test_one_valid_item(self) -> None:
        items = validate_mission_items(
            [MissionItem(latitude_deg=47.4, longitude_deg=8.5, altitude_m=20.0)]
        )
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0].command, 16)

    def test_rejects_empty(self) -> None:
        with self.assertRaises(InvalidArgumentError):
            validate_mission_items([])

    def test_rejects_bad_lat(self) -> None:
        with self.assertRaises(InvalidArgumentError):
            validate_mission_items(
                [MissionItem(latitude_deg=91.0, longitude_deg=0.0, altitude_m=10.0)]
            )

    def test_rejects_non_item(self) -> None:
        with self.assertRaises(InvalidArgumentError):
            validate_mission_items([{"lat": 1}])  # type: ignore[list-item]


if __name__ == "__main__":
    unittest.main()
