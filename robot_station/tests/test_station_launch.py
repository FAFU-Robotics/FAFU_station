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
        self.assertTrue((ROOT / "启动真机.sh").is_file())
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
        self.assertNotIn(b"install_home", live)
        self.assertIn(b"goto portable", live)
        self.assertIn(b"STATION_URDF_DIR", live)
        self.assertNotIn(b"setx", live.lower())
        self.assertEqual(live, (ROOT / "start_live_arm.bat").read_bytes())
        live_sh = (ROOT / "启动真机.sh").read_text(encoding="utf-8")
        self.assertIn("STATION_ALLOW_LIVE_ARM=1", live_sh)
        self.assertIn("station_desktop.py", live_sh)
        self.assertIn("STATION_PORTABLE", live_sh)
        self.assertIn("runtime/python310", live_sh)
        self.assertIn("linux_preflight", live_sh)
        self.assertNotIn("setx", live_sh.lower())
        self.assertTrue((ROOT / "packaging/windows/build_portable.ps1").is_file())
        self.assertTrue((ROOT / "packaging/windows/PortableLauncher.cs").is_file())
        desktop = (ROOT / "station_desktop.py").read_text(encoding="utf-8")
        self.assertIn("--arm", desktop)
        self.assertIn("_live_station_ready", desktop)
        self.assertIn("_existing_live_station_ok", desktop)
        self.assertIn("--camera", desktop)
        self.assertIn("--web-only", desktop)
        self.assertIn("do not kill arm", desktop)
        self.assertIn("existing :", desktop)
        self.assertIn("host exited", desktop)
        self.assertIn("_guard_web", desktop)
        self.assertIn("_station_should_outlive_window", desktop)
        self.assertIn("never replace while motion", desktop)
        self.assertIn("_running_code_matches", desktop)
        self.assertIn("boot_rev", desktop)
        self.assertIn("_has_pinocchio", desktop)
        self.assertIn("dyn_ready=false", desktop)
        self.assertIn("_kickoff_local_station", desktop)
        self.assertIn("_LOADING_HTML", desktop)
        self.assertIn('start_kwargs["gui"] = "gtk"', desktop)
        self.assertIn("edgechromium", desktop)
        self.assertIn("正在启动机械臂站控", desktop)
        self.assertIn("_wait_ready(host, port, WAIT_S", desktop)
        self.assertIn("HOST_LOG", desktop)
        self.assertIn("fafu-station-host.log", desktop)
        self.assertIn(b"pick_py310", live)
        self.assertIn(b"pinocchio", live)
        self.assertIn(b"PINOCCHIO_WINDOWS_DLL_PATH", live)
        self.assertIn(b"Library\\bin", live)
        self.assertIn("code_rev", desktop)
        server = (ROOT / "robot_station" / "web" / "server.py").read_text(encoding="utf-8")
        self.assertIn("boot_rev", server)
        self.assertIn('snap.get("code_rev")', server)
        self.assertIn("web lost, motion still up", desktop)
        self.assertIn("replace_web", desktop)
        self.assertIn("0x01000000", desktop)
        service = (ROOT / "robot_station" / "service.py").read_text(encoding="utf-8")
        self.assertIn("_serve_http_loop", service)
        self.assertIn("replace_web_then_start", service)
        spec = (ROOT / "docs/SPEC.md").read_text(encoding="utf-8")
        self.assertIn("客户 PC", spec)
        self.assertIn("本机宿主", spec)
        self.assertIn("不得** `--replace` 整站", spec)
        self.assertIn("code_rev", spec)
        self.assertIn("只补 `--web-only`", spec)
        self.assertNotIn("工控机", spec)
        self.assertIn("Jetson", spec)
        self.assertIn("不要在 Jetson", spec)
        self.assertNotIn("FAFU-ARM", spec)
        self.assertNotIn("bunker-local", spec)
        self.assertNotIn("--discover", desktop)
        self.assertNotIn("FAFULabPortal", desktop)
        self.assertNotIn("工控机", desktop)
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
        self.assertTrue(sd._existing_live_station_ok({"arm": "fafu", "arm_allow_motion": True, "camera": "mock"}))
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

    def test_ensure_attaches_when_motion_up_even_if_info_is_wrong(self) -> None:
        import sys
        from unittest.mock import patch

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        def ports(_host: str, port: int, *_a: object, **_k: object) -> bool:
            return int(port) in (9400, 9470)

        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_port_open", side_effect=ports),
            patch.object(sd, "_running_code_matches", return_value=True),
            patch.object(sd, "_has_pinocchio", return_value=False),
            patch.object(sd, "_station_info", return_value={"arm": "mock", "camera": "mock"}),
            patch.object(sd, "_spawn_local_station") as spawn,
        ):
            ok, proc = sd._ensure_local_station(9400)
        self.assertTrue(ok)
        self.assertIsNone(proc)
        spawn.assert_not_called()

    def test_ensure_replaces_motion_when_code_rev_stale(self) -> None:
        import sys
        from unittest.mock import MagicMock, patch

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        def ports(_host: str, port: int, *_a: object, **_k: object) -> bool:
            return int(port) in (9400, 9470)

        child = MagicMock()
        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_port_open", side_effect=ports),
            patch.object(sd, "_running_code_matches", return_value=False),
            patch.object(sd, "_spawn_and_wait", return_value=(True, child)) as spawn,
        ):
            ok, proc = sd._ensure_local_station(9400)
        self.assertTrue(ok)
        self.assertIs(proc, child)
        spawn.assert_called_once_with(9400, web_only=False)

    def test_ensure_replaces_when_pinocchio_ready_but_station_not(self) -> None:
        import sys
        from unittest.mock import MagicMock, patch

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        def ports(_host: str, port: int, *_a: object, **_k: object) -> bool:
            return int(port) in (9400, 9470)

        child = MagicMock()
        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_port_open", side_effect=ports),
            patch.object(sd, "_running_code_matches", return_value=True),
            patch.object(sd, "_has_pinocchio", return_value=True),
            patch.object(
                sd,
                "_station_info",
                return_value={"arm": "fafu", "arm_allow_motion": True, "dyn_ready": False},
            ),
            patch.object(sd, "_spawn_and_wait", return_value=(True, child)) as spawn,
        ):
            ok, proc = sd._ensure_local_station(9400)
        self.assertTrue(ok)
        self.assertIs(proc, child)
        spawn.assert_called_once_with(9400, web_only=False)

    def test_running_code_matches_needs_frozen_boot_rev(self) -> None:
        import sys
        from unittest.mock import patch

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        with patch.object(sd, "_station_info", return_value={"code_rev": "deadbeef1234"}):
            self.assertFalse(sd._running_code_matches(9400))
        with patch.object(
            sd,
            "_station_info",
            return_value={"boot_rev": "deadbeef1234", "code_rev": "deadbeef1234"},
        ), patch("robot_station.runtime.station_code_rev", return_value="deadbeef1234"):
            self.assertTrue(sd._running_code_matches(9400))
        with patch.object(
            sd,
            "_station_info",
            return_value={"boot_rev": "oldoldoldold", "code_rev": "oldoldoldold"},
        ), patch("robot_station.runtime.station_code_rev", return_value="newnewnewnew"):
            self.assertFalse(sd._running_code_matches(9400))

    def test_live_orphan_web_is_replaced_despite_cached_info(self) -> None:
        import station_desktop as sd
        from unittest.mock import MagicMock

        child = MagicMock()
        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_port_open", return_value=True),
            patch.object(sd, "_motion_up", return_value=False),
            patch.object(sd, "_station_info", return_value={
                "arm": "fafu", "arm_allow_motion": True, "dyn_ready": True,
            }),
            patch.object(sd, "_spawn_and_wait", return_value=(True, child)) as spawn,
        ):
            self.assertEqual(sd._ensure_local_station(9400), (True, child))
        spawn.assert_called_once_with(9400, web_only=False)

    def test_failed_live_child_cannot_reuse_orphan_web(self) -> None:
        import station_desktop as sd
        from unittest.mock import MagicMock

        child = MagicMock()
        child.poll.return_value = 1
        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_spawn_local_station", return_value=child),
            patch.object(sd, "_port_open", return_value=True),
        ):
            self.assertEqual(sd._spawn_and_wait(9400, web_only=False), (False, None))

    def test_live_start_waits_for_motion_and_current_version(self) -> None:
        import station_desktop as sd
        from unittest.mock import MagicMock

        child = MagicMock()
        child.poll.return_value = None
        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_spawn_local_station", return_value=child),
            patch.object(sd, "_port_open", return_value=True),
            patch.object(sd, "_motion_up", side_effect=[False, True, True]),
            patch.object(sd, "_running_code_matches", side_effect=[False, True]),
            patch.object(sd.time, "sleep") as sleep,
        ):
            self.assertEqual(sd._spawn_and_wait(9400, web_only=False), (True, child))
        self.assertEqual(sleep.call_count, 2)

    def test_ensure_web_only_when_motion_up_and_http_down(self) -> None:
        import sys
        from unittest.mock import MagicMock, patch

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        def ports(_host: str, port: int, *_a: object, **_k: object) -> bool:
            return int(port) == 9470

        child = MagicMock()
        with (
            patch.object(sd, "_port_open", side_effect=ports),
            patch.object(sd, "_spawn_and_wait", return_value=(True, child)) as spawn,
        ):
            ok, proc = sd._ensure_local_station(9400)
        self.assertTrue(ok)
        self.assertIs(proc, child)
        spawn.assert_called_once_with(9400, web_only=True)

    def test_kickoff_attaches_when_motion_up(self) -> None:
        import station_desktop as sd

        def ports(_host: str, port: int, *_a: object, **_k: object) -> bool:
            return int(port) in (9400, 9470)

        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_port_open", side_effect=ports),
            patch.object(sd, "_running_code_matches", return_value=True),
            patch.object(sd, "_has_pinocchio", return_value=False),
            patch.object(sd, "_spawn_local_station") as spawn,
            patch.object(sd, "_spawn_and_wait") as wait,
        ):
            ok, proc = sd._kickoff_local_station(9400)
        self.assertTrue(ok)
        self.assertIsNone(proc)
        spawn.assert_not_called()
        wait.assert_not_called()

    def test_kickoff_spawns_without_waiting(self) -> None:
        import station_desktop as sd
        from unittest.mock import MagicMock

        child = MagicMock()
        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_port_open", return_value=False),
            patch.object(sd, "_motion_up", return_value=False),
            patch.object(sd, "_spawn_local_station", return_value=child) as spawn,
            patch.object(sd, "_spawn_and_wait") as wait,
        ):
            ok, proc = sd._kickoff_local_station(9400)
        self.assertFalse(ok)
        self.assertIs(proc, child)
        spawn.assert_called_once_with(web_only=False)
        wait.assert_not_called()

    def test_kickoff_replaces_stale_code_without_waiting(self) -> None:
        import station_desktop as sd
        from unittest.mock import MagicMock

        child = MagicMock()

        def ports(_host: str, port: int, *_a: object, **_k: object) -> bool:
            return int(port) in (9400, 9470)

        with (
            patch.object(sd, "_live_arm_wanted", return_value=True),
            patch.object(sd, "_port_open", side_effect=ports),
            patch.object(sd, "_running_code_matches", return_value=False),
            patch.object(sd, "_spawn_local_station", return_value=child) as spawn,
            patch.object(sd, "_spawn_and_wait") as wait,
        ):
            ok, proc = sd._kickoff_local_station(9400)
        self.assertFalse(ok)
        self.assertIs(proc, child)
        spawn.assert_called_once_with(web_only=False)
        wait.assert_not_called()

    def test_loading_html_formats(self) -> None:
        import station_desktop as sd

        html = sd._LOADING_HTML.format(url="http://127.0.0.1:9400/", host="127.0.0.1", port=9400)
        self.assertIn("正在启动机械臂站控", html)
        self.assertIn("http://127.0.0.1:9400/", html)
        offline = sd._OFFLINE_HTML.format(url="http://127.0.0.1:9400/", host="127.0.0.1", port=9400)
        self.assertIn("本机站控未启动", offline)

    def test_station_outlives_window_when_live_or_motion_up(self) -> None:
        import sys
        from unittest.mock import patch

        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        import station_desktop as sd

        with patch.object(sd, "_live_arm_wanted", return_value=True), patch.object(
            sd, "_motion_up", return_value=False
        ):
            self.assertTrue(sd._station_should_outlive_window())
        with patch.object(sd, "_live_arm_wanted", return_value=False), patch.object(
            sd, "_motion_up", return_value=True
        ):
            self.assertTrue(sd._station_should_outlive_window())
        with patch.object(sd, "_live_arm_wanted", return_value=False), patch.object(
            sd, "_motion_up", return_value=False
        ):
            self.assertFalse(sd._station_should_outlive_window())

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
