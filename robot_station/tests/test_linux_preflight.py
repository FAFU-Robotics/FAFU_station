from __future__ import annotations

import json
import os
import sys
import tempfile
import types
import unittest
from io import StringIO
from pathlib import Path
from unittest.mock import patch

from robot_station.linux_preflight import (
    FUSE_HINT,
    LinuxPreflight,
    inspect,
    main,
    preflight_report_path,
    probe_webview,
    udev_rules_path,
    write_preflight_report,
)


class LinuxPreflightTests(unittest.TestCase):
    def test_inspect_returns_dataclass(self) -> None:
        result = inspect()
        self.assertIsInstance(result, LinuxPreflight)
        self.assertIsInstance(result.serial_ports, list)
        self.assertIsInstance(result.warnings, list)
        self.assertIsInstance(result.blockers, list)
        self.assertIsInstance(result.fixes, list)

    def test_inspect_json_roundtrip(self) -> None:
        payload = inspect().as_dict()
        self.assertIn("webview_ok", payload)
        self.assertIn("serial_ports", payload)
        self.assertIn("fixes", payload)
        self.assertIn("motor_ok", payload)
        json.dumps(payload)

    def test_bundled_udev_rules_are_found(self) -> None:
        path = udev_rules_path()
        self.assertIsNotNone(path)
        self.assertTrue(path.is_file())
        text = path.read_text(encoding="utf-8")
        self.assertIn("fafu_debug_board", text)

    def test_inspect_cli_writes_cache_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(os.environ, {"XDG_CACHE_HOME": tmp}, clear=False):
                with patch("sys.stdout", StringIO()):
                    with patch("sys.stderr", StringIO()):
                        self.assertEqual(main(["--inspect"]), 0)
            report = Path(tmp) / "fafu-station-preflight.json"
            self.assertTrue(report.is_file())
            payload = json.loads(report.read_text(encoding="utf-8"))
            self.assertIn("fixes", payload)
            self.assertIn("warnings", payload)

    def test_write_preflight_report_roundtrip(self) -> None:
        result = LinuxPreflight(webview_ok=True, webview_detail="ok", fixes=["sudo apt install libfuse2"])
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "out.json"
            written = write_preflight_report(result, dest)
            self.assertEqual(written, dest)
            payload = json.loads(dest.read_text(encoding="utf-8"))
        self.assertEqual(payload["fixes"], ["sudo apt install libfuse2"])

    def test_preflight_report_path_honors_xdg(self) -> None:
        with patch.dict(os.environ, {"XDG_CACHE_HOME": r"C:\cache-preflight"}, clear=False):
            path = preflight_report_path()
        self.assertEqual(path, Path(r"C:\cache-preflight") / "fafu-station-preflight.json")

    def test_empty_serial_is_warning_not_blocker(self) -> None:
        with patch("robot_station.linux_preflight.serial_candidates", return_value=[]):
            with patch("robot_station.linux_preflight.probe_webview", return_value=(True, "ok")):
                result = inspect()
        self.assertEqual(result.serial_ports, [])
        self.assertTrue(any("串口" in w for w in result.warnings))
        self.assertFalse(any("串口" in b for b in result.blockers))
        self.assertTrue(any("udev" in f for f in result.fixes))

    def test_missing_webview_is_blocker_with_fuse_hint(self) -> None:
        with patch("robot_station.linux_preflight.probe_webview", return_value=(False, "no gtk")):
            result = inspect()
        self.assertFalse(result.webview_ok)
        self.assertTrue(result.blockers)
        self.assertTrue(any("libfuse2" in b or "--appimage-extract-and-run" in b for b in result.blockers))
        self.assertTrue(any("libfuse2" in f or "--appimage-extract-and-run" in f for f in result.fixes))

    def test_fuse_hint_on_posix_inspect(self) -> None:
        with patch("robot_station.linux_preflight.os.name", "posix"):
            with patch("robot_station.linux_preflight.probe_webview", return_value=(True, "ok")):
                with patch("robot_station.linux_preflight.serial_candidates", return_value=[]):
                    result = inspect()
        self.assertTrue(any("--appimage-extract-and-run" in w for w in result.warnings))
        self.assertIn(FUSE_HINT, result.warnings)
        self.assertTrue(any("libfuse2" in f for f in result.fixes))

    def test_no_display_gi_missing_is_not_blocker(self) -> None:
        fake_webview = types.ModuleType("webview")
        env = {k: v for k, v in os.environ.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY")}
        with patch.dict(sys.modules, {"webview": fake_webview}):
            with patch.dict(os.environ, env, clear=True):
                ok, detail = probe_webview()
        self.assertTrue(ok)
        self.assertIn("DISPLAY", detail)

    def test_headless_inspect_warns_not_blocks(self) -> None:
        env = {k: v for k, v in os.environ.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY")}
        with patch.dict(os.environ, env, clear=True):
            with patch("robot_station.linux_preflight.probe_webview", return_value=(True, "headless")):
                with patch("robot_station.linux_preflight.serial_candidates", return_value=[]):
                    result = inspect()
        self.assertTrue(result.webview_ok)
        self.assertFalse(result.blockers)
        self.assertTrue(any("DISPLAY" in w for w in result.warnings))

    def test_portable_motor_import_failure_is_blocker(self) -> None:
        with patch.dict(os.environ, {"STATION_PORTABLE": "1"}, clear=False):
            with patch("robot_station.linux_preflight.probe_webview", return_value=(True, "ok")):
                with patch("robot_station.linux_preflight.serial_candidates", return_value=[]):
                    with patch(
                        "robot_station.linux_preflight.probe_fafu_motor",
                        return_value=(False, "missing .so"),
                    ):
                        result = inspect()
        self.assertFalse(result.motor_ok)
        self.assertTrue(any("fafu_motor" in b for b in result.blockers))
        self.assertFalse(any("fafu_motor" in w for w in result.warnings))

    def test_source_motor_import_failure_is_warning(self) -> None:
        env = {k: v for k, v in os.environ.items() if k != "STATION_PORTABLE"}
        env["FAFU_ARM_SDK"] = "/tmp/fake-sdk"
        with patch.dict(os.environ, env, clear=True):
            with patch("robot_station.linux_preflight.probe_webview", return_value=(True, "ok")):
                with patch("robot_station.linux_preflight.serial_candidates", return_value=[]):
                    with patch(
                        "robot_station.linux_preflight.probe_fafu_motor",
                        return_value=(False, "missing .so"),
                    ):
                        result = inspect()
        self.assertFalse(result.motor_ok)
        self.assertFalse(any("fafu_motor" in b for b in result.blockers))
        self.assertTrue(any("fafu_motor" in w for w in result.warnings))

    def test_portable_motor_ok_does_not_block_without_board(self) -> None:
        with patch.dict(os.environ, {"STATION_PORTABLE": "1"}, clear=False):
            with patch("robot_station.linux_preflight.probe_webview", return_value=(True, "ok")):
                with patch("robot_station.linux_preflight.serial_candidates", return_value=[]):
                    with patch(
                        "robot_station.linux_preflight.probe_fafu_motor",
                        return_value=(True, "已加载 fafu_motor；未枚举到调试板（可先用仿真臂）"),
                    ):
                        result = inspect()
        self.assertTrue(result.motor_ok)
        self.assertFalse(any("fafu_motor" in b for b in result.blockers))
        self.assertTrue(any("串口" in w for w in result.warnings))


if __name__ == "__main__":
    unittest.main()
