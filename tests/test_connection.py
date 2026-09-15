"""连接 URI 解析测试，不依赖 pymavlink。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from drone_fc.exceptions import InvalidArgumentError
from drone_fc.mavlink.connection import parse_connection_uri


class ParseConnectionUriTests(unittest.TestCase):
    def test_udp_defaults_to_listen(self) -> None:
        parsed = parse_connection_uri("udp:127.0.0.1:14540")
        self.assertEqual(parsed.device, "udpin:127.0.0.1:14540")
        self.assertIsNone(parsed.baud)

    def test_udpin_and_udpout_passthrough(self) -> None:
        self.assertEqual(parse_connection_uri("udpin:0.0.0.0:14540").device, "udpin:0.0.0.0:14540")
        self.assertEqual(
            parse_connection_uri("udpout:127.0.0.1:14580").device,
            "udpout:127.0.0.1:14580",
        )

    def test_tcp(self) -> None:
        parsed = parse_connection_uri("tcp:127.0.0.1:5760")
        self.assertEqual(parsed.device, "tcp:127.0.0.1:5760")

    def test_serial(self) -> None:
        parsed = parse_connection_uri("serial:/dev/ttyACM0:57600")
        self.assertEqual(parsed.device, "/dev/ttyACM0")
        self.assertEqual(parsed.baud, 57600)

    def test_rejects_bad_values(self) -> None:
        with self.assertRaises(InvalidArgumentError):
            parse_connection_uri("")
        with self.assertRaises(InvalidArgumentError):
            parse_connection_uri("foo:bar")
        with self.assertRaises(InvalidArgumentError):
            parse_connection_uri("udp:localhost:99999")
        with self.assertRaises(InvalidArgumentError):
            parse_connection_uri("serial:/dev/ttyUSB0:0")


if __name__ == "__main__":
    unittest.main()
