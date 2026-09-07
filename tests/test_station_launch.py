from __future__ import annotations

import unittest
from pathlib import Path
from unittest.mock import patch

from robot_station.config import StationConfig
from robot_station.runtime import RuntimeStatus
from robot_station.service import main, motion_argv

ROOT = Path(__file__).resolve().parents[1]


class StationLaunchFilesTests(unittest.TestCase):
    def test_pc_host_kit_exists(self) -> None:
        self.assertFalse((ROOT / "station_client_laptop").exists())
        self.assertFalse((ROOT / "launch").exists())
        self.assertFalse((ROOT / "packaging/systemd").exists())
        self.assertFalse((ROOT / "packaging/systemd/robot-station.service").is_file())
        self.assertFalse((ROOT / "scripts/start_delivery_wifi.sh").is_file())
        self.assertFalse((ROOT / "scripts/install_station_service.sh").is_file())
        self.assertFalse((ROOT / "scripts/start_station_gui.sh").is_file())
        self.assertFalse((ROOT / "scripts/start_station_app.sh").is_file())
        self.assertFalse((ROOT / "scripts/start_station.sh").is_file())
        self.assertFalse((ROOT / "scripts/open_status.sh").is_file())
        self.assertFalse((ROOT / "scripts/install_desktop_shortcut.sh").is_file())
        self.assertFalse((ROOT / "scripts/print_lab_urls.py").is_file())
        self.assertFalse((ROOT / "robot_station/discover.py").is_file())
        self.assertFalse((ROOT / "robot_station/adapters/chassis.py").is_file())
        self.assertFalse((ROOT / "robot_station/adapters/bunker.py").is_file())
        self.assertFalse((ROOT / "启动站控.bat").is_file())
        self.assertFalse((ROOT / "打开界面.bat").is_file())
        self.assertTrue((ROOT / "启动真机.bat").is_file())
        self.assertTrue((ROOT / "station_desktop.py").is_file())
        self.assertTrue((ROOT / "scripts/verify_sim_arm.py").is_file())
        self.assertTrue((ROOT / "scripts/rebuild_fafu_motor.bat").is_file())
        live_bat = (ROOT / "启动真机.bat").read_text(encoding="ascii")
        self.assertIn("station_desktop.py", live_bat)
        live = (ROOT / "启动真机.bat").read_bytes()
        self.assertTrue(live.isascii(), "cmd.exe misparses UTF-8 Chinese in .bat files")
        self.assertIn(b"STATION_ALLOW_LIVE_ARM", live)
        self.assertIn(b"Python310", live)
        self.assertIn(b"STATION_PORTABLE", live)
        self.assertIn(b"runtime\\python310", live)
        self.assertIn(b"install_home", live)
        self.assertIn(b"LOCALAPPDATA", live)
        self.assertIn(b"FAFUArmStation.exe", live)
        self.assertNotIn(b"setx", live.lower())
        self.assertEqual(live, (ROOT / "start_live_arm.bat").read_bytes())
        self.assertTrue((ROOT / "packaging/windows/build_portable.ps1").is_file())
        self.assertTrue((ROOT / "packaging/windows/PortableLauncher.cs").is_file())
        desktop = (ROOT / "station_desktop.py").read_text(encoding="utf-8")
        self.assertIn("--arm", desktop)
        self.assertIn("_live_station_ready", desktop)
        self.assertIn("_existing_live_station_ok", desktop)
        self.assertIn("--camera", desktop)
        self.assertIn("camera auto", desktop)
        self.assertIn("existing :", desktop)
        self.assertIn("host exited", desktop)
        self.assertNotIn("--discover", desktop)
        self.assertNotIn("FAFULabPortal", desktop)
        self.assertNotIn("工控机", desktop)
        spec = (ROOT / "docs/SPEC.md").read_text(encoding="utf-8")
        self.assertIn("客户 PC", spec)
        self.assertIn("本机宿主", spec)
        self.assertNotIn("工控机", spec)
        self.assertNotIn("Jetson", spec)
        self.assertNotIn("FAFU-ARM", spec)
        self.assertNotIn("bunker-local", spec)
        example = (ROOT / "configs/station.example.yaml").read_text(encoding="utf-8")
        self.assertNotIn("password:", example)
        self.assertNotIn("lease_s:", example)

    def test_desktop_url_helpers(self) -> None:
        import sys
        import tempfile

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        self.assertEqual(sd._make_url("127.0.0.1", 9400, "lab"), "http://127.0.0.1:9400/lab")
        self.assertEqual(sd._norm_path(""), "/")
        with tempfile.NamedTemporaryFile(
            "w", suffix=".conf", delete=False, encoding="utf-8"
        ) as fh:
            fh.write("HOST=127.0.0.1\nPORT=9400\nPATH=/\n")
            conf_path = Path(fh.name)
        try:
            data = sd._load_conf(conf_path)
        finally:
            conf_path.unlink(missing_ok=True)
        self.assertEqual(data.get("PORT"), "9400")
        self.assertEqual(data.get("PATH"), "/")
        self.assertEqual(data.get("HOST"), "127.0.0.1")
        self.assertFalse(sd._existing_live_station_ok({"arm": "fafu", "arm_allow_motion": True, "camera": "mock"}))
        self.assertTrue(sd._existing_live_station_ok({"arm": "fafu", "arm_allow_motion": True, "camera": "auto"}))
        argv = sd._app_shell_argv("http://127.0.0.1:9400/")
        if argv is not None:
            self.assertTrue(any(str(a).startswith("--app=") for a in argv))
            self.assertIn("--window-size=1280,860", argv)
        try:
            import webview
        except ImportError:
            self.skipTest("pywebview not installed")
        self.assertTrue(hasattr(webview, "create_window"))
        self.assertTrue(hasattr(webview, "start"))

    def test_wait_ready_gives_up_when_child_exits(self) -> None:
        import subprocess
        import sys
        import time

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(0.05)"])
        t0 = time.monotonic()
        ok = sd._wait_ready("127.0.0.1", 59999, 8.0, proc)
        self.assertFalse(ok)
        self.assertLess(time.monotonic() - t0, 3.0)
        proc.wait(timeout=2)

    def test_motion_child_keeps_arm_sim_override(self) -> None:
        cmd = motion_argv(StationConfig(arm="sim"), None)
        self.assertIn("--motion-only", cmd)
        self.assertIn("--arm", cmd)
        self.assertEqual(cmd[cmd.index("--arm") + 1], "sim")
        self.assertNotIn("fafu", cmd)


class StationServiceFlagTests(unittest.TestCase):
    def test_service_refuses_attach(self) -> None:
        status = RuntimeStatus(http_up=True, motion_up=True, lock_held=True)
        with patch("robot_station.service.inspect_runtime", return_value=status):
            self.assertEqual(main(["--service"]), 1)

    def test_manual_start_still_attaches(self) -> None:
        status = RuntimeStatus(http_up=True, motion_up=True, lock_held=True)
        with patch("robot_station.service.inspect_runtime", return_value=status), patch(
            "robot_station.service._maybe_open_browser"
        ):
            self.assertEqual(main(["--no-browser"]), 0)


if __name__ == "__main__":
    unittest.main()
