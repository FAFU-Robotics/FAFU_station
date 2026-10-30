from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_station.adapters.fafu_arm import NO_DEBUG_BOARD, resolve_live_serial_port


class ResolveLiveSerialPortTests(unittest.TestCase):
    def test_auto_refuses_ambiguous_boards(self) -> None:
        with self.assertRaisesRegex(RuntimeError,"多个"):
            resolve_live_serial_port("auto", ["COM3", "COM7"])
        self.assertEqual(
            resolve_live_serial_port("", [SimpleNamespace(port="COM7")]),
            "COM7",
        )

    def test_exact_com_match(self) -> None:
        self.assertEqual(resolve_live_serial_port("COM7", ["COM3", "COM7"]), "COM7")

    def test_no_board_raises(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "no USB debug board"):
            resolve_live_serial_port("auto", [])
        self.assertIn("debug board", NO_DEBUG_BOARD)

    def test_udev_symlink_is_passed_to_driver_verbatim(self) -> None:
        self.assertEqual(resolve_live_serial_port("/dev/fafu_debug_board", ["/dev/ttyUSB0"]),
                         "/dev/fafu_debug_board")

    def test_posix_realpath_match_without_real_dev(self) -> None:
        def fake_real(path: str) -> str:
            if "fafu_debug_board" in path or "ttyUSB0" in path:
                return "/dev/ttyUSB0"
            return path

        with patch("robot_station.adapters.fafu_arm.os.name", "posix"):
            with patch("robot_station.adapters.fafu_arm.os.path.realpath", side_effect=fake_real):
                with patch("robot_station.adapters.fafu_arm.os.path.exists", return_value=False):
                    self.assertEqual(
                        resolve_live_serial_port("/dev/fafu_debug_board", ["/dev/ttyUSB0"]),
                        "/dev/fafu_debug_board",
                    )

    def test_preferred_exists_but_not_enumerated(self) -> None:
        with patch("robot_station.adapters.fafu_arm.os.name", "posix"):
            with patch("robot_station.adapters.fafu_arm.os.path.realpath", side_effect=lambda p: p):
                with patch("robot_station.adapters.fafu_arm.os.path.exists", return_value=True):
                    self.assertEqual(
                        resolve_live_serial_port("/dev/fafu_debug_board", ["/dev/ttyUSB1"]),
                        "/dev/fafu_debug_board",
                    )

    def test_unknown_preferred_never_falls_back(self) -> None:
        with patch("robot_station.adapters.fafu_arm.os.name", "nt"):
            self.assertEqual(
                resolve_live_serial_port("COM9", ["COM3", "COM7"]),
                "COM9",
            )


if __name__ == "__main__":
    unittest.main()
