from __future__ import annotations

import os
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
PACK = ROOT / "packaging" / "linux"
WORKFLOW = ROOT.parent / ".github" / "workflows" / "packaging-smoke.yml"


class LinuxPackagingFilesTests(unittest.TestCase):
    def test_packaging_kit_exists(self) -> None:
        self.assertTrue((PACK / "build_appimage.sh").is_file())
        self.assertTrue((PACK / "AppRun").is_file())
        self.assertTrue((PACK / "verify_appimage.sh").is_file())
        self.assertTrue((PACK / "ACCEPTANCE.md").is_file())
        self.assertTrue((PACK / "README.md").is_file())
        self.assertTrue((PACK / "Dockerfile").is_file())
        self.assertTrue((PACK / "docker-entrypoint.sh").is_file())
        self.assertTrue((PACK / "fafu-arm-station.desktop").is_file())
        self.assertTrue((PACK / "gtk_probe.c").is_file())
        self.assertTrue((PACK / "99-fafu-debug-board.rules").is_file())
        self.assertTrue((PACK / "annotate_gha_log.py").is_file())
        self.assertTrue((ROOT / "robot_station" / "linux_preflight.py").is_file())
        self.assertFalse(
            (ROOT / "robot_station" / "preflight.py").is_file(),
            "Windows preflight stays in PortableLauncher.cs; Linux uses linux_preflight.py",
        )

    def test_apprun_sets_portable_env(self) -> None:
        apprun = (PACK / "AppRun").read_text(encoding="utf-8")
        for needle in (
            "STATION_PORTABLE=1",
            "STATION_INSTALL_ROOT",
            "STATION_ALLOW_LIVE_ARM=1",
            "STATION_ARM_WATCH=1",
            "STATION_CAMERA_WATCH=1",
            "PYTHONNOUSERSITE",
            "GDK_BACKEND",
            "linux_preflight",
            "station_desktop.py",
            "FAFU_ARM_SDK",
            "STATION_URDF_DIR",
            "LD_LIBRARY_PATH",
        ):
            self.assertIn(needle, apprun)
        self.assertNotIn("setx", apprun.lower())
        self.assertNotIn("pyinstaller", apprun.lower())

    def test_build_refuses_customer_image_without_sdk(self) -> None:
        script = (PACK / "build_appimage.sh").read_text(encoding="utf-8")
        self.assertIn("PUT_SDK_HERE", script)
        self.assertIn("allow-no-sdk", script)
        self.assertIn("fafu_motor.cpython-310", script)
        self.assertIn("not wrapping a customer AppImage", script)
        self.assertNotIn("pyinstaller", script.lower())
        self.assertIn("appimagetool", script)
        self.assertIn("appimagetool/releases/download/1.9.1/", script)
        self.assertIn("linuxdeploy", script)
        self.assertIn("gtk_probe", script)
        self.assertIn("APPIMAGE_EXTRACT_AND_RUN=1", script)
        self.assertNotIn("releases/download/continuous/", script)
        self.assertIn("linuxdeploy failed", script)
        self.assertIn("BUILD_MANIFEST.json", script)
        self.assertIn("FAFUAPP1", script)
        self.assertIn("--plugin gtk", script)
        self.assertIn("cc and gtk+-3.0 are required", script)
        self.assertIn("curl -fL", script)
        self.assertIn("fetch_url", script)
        self.assertIn("PyGObject>=3.42,<3.52", script)
        self.assertIn("pybind11>=2.12,<3", script)
        docker = (PACK / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("APPIMAGE_EXTRACT_AND_RUN=1", docker)
        entry = (PACK / "docker-entrypoint.sh").read_text(encoding="utf-8")
        self.assertIn("APPIMAGE_EXTRACT_AND_RUN=1", entry)
        self.assertIn("FAFU_ARM_SDK", entry)
        self.assertIn("/src/fafu_arm_sdk", entry)
        self.assertIn("appimage-build.log", entry)

    def test_build_appimage_bat_fails_without_docker(self) -> None:
        bat = (PACK / "build_appimage.bat").read_text(encoding="utf-8")
        self.assertIn("where docker", bat)
        self.assertIn("GitHub Actions", bat)
        self.assertIn("Ubuntu 22.04", bat)
        self.assertIn("docker build", bat)
        self.assertIn("exit /b 1", bat)
        self.assertIn("packaging-smoke.yml", bat)

    def test_verify_requires_fafu_motor_and_manifest(self) -> None:
        verify = (PACK / "verify_appimage.sh").read_text(encoding="utf-8")
        self.assertIn("import fafu_motor", verify)
        self.assertIn("BUILD_MANIFEST.json", verify)
        self.assertIn("FAFUAPP1", verify)
        self.assertIn("image_sha256", verify)

    def test_ci_builds_real_appimage(self) -> None:
        self.assertTrue(WORKFLOW.is_file())
        text = WORKFLOW.read_text(encoding="utf-8")
        self.assertIn("linux-appimage:", text)
        self.assertIn("ubuntu-22.04", text)
        self.assertIn("build_appimage.sh", text)
        self.assertIn("FAFU_ARM_SDK", text)
        self.assertIn("refusing customer AppImage", text)
        self.assertIn("upload-artifact", text)
        self.assertIn("BUILD_MANIFEST.json", text)
        self.assertIn("retention-days: 7", text)
        self.assertIn("workflow_dispatch", text)
        self.assertNotIn("Full AppImage build needs fafu_arm_sdk and is not run", text)

    def test_verify_checks_so_and_imports(self) -> None:
        verify = (PACK / "verify_appimage.sh").read_text(encoding="utf-8")
        self.assertIn("fafu_motor.cpython-310", verify)
        self.assertIn("PUT_SDK_HERE", verify)
        self.assertIn("import websockets, yaml, numpy, webview", verify)
        self.assertIn("STATION_PORTABLE", verify)

    def test_desktop_and_udev(self) -> None:
        desktop = (PACK / "fafu-arm-station.desktop").read_text(encoding="utf-8")
        self.assertIn("Exec=AppRun", desktop)
        self.assertIn("Icon=FAFUArmStation", desktop)
        rules = (PACK / "99-fafu-debug-board.rules").read_text(encoding="utf-8")
        self.assertIn("fafu_debug_board", rules)
        self.assertIn("1a86", rules)
        self.assertIn("10c4", rules)

    def test_linux_customer_readme(self) -> None:
        readme = ROOT / "docs" / "客户使用说明-linux.md"
        self.assertTrue(readme.is_file())
        text = readme.read_text(encoding="utf-8")
        self.assertIn("station-customer-readme-linux", text)
        self.assertIn("AppImage", text)
        self.assertIn("recordings", text)
        build = (PACK / "build_appimage.sh").read_text(encoding="utf-8")
        self.assertIn("station-customer-readme-linux", build)
        docker = (PACK / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("ubuntu:22.04", docker)
        self.assertIn("libwebkit2gtk-4.0", docker)
        self.assertIn("patchelf", docker)
        entry = (PACK / "docker-entrypoint.sh").read_text(encoding="utf-8")
        self.assertIn("build_appimage.sh", entry)
        self.assertIn("FAFU_ARM_SDK", entry)
        spec = (ROOT / "docs" / "SPEC.md").read_text(encoding="utf-8")
        self.assertIn("FAFUArmStation-x86_64.AppImage", spec)
        self.assertIn("linux_preflight", spec)
        self.assertIn("SCHED_FIFO", spec)
        self.assertIn("linux-appimage", spec)
        pack_readme = (PACK / "README.md").read_text(encoding="utf-8")
        self.assertIn("GitHub Actions", pack_readme)
        self.assertIn("workflow_dispatch", pack_readme)
        self.assertIn("Jetson", pack_readme)
        accept = (PACK / "ACCEPTANCE.md").read_text(encoding="utf-8")
        self.assertIn("GHA 绿 ≠ 客户验收过", accept)
        self.assertIn("Jetson", accept)
        usage = (ROOT / "docs" / "使用说明.md").read_text(encoding="utf-8")
        self.assertIn("linux-appimage", usage)
        self.assertIn("Jetson", usage)


class LinuxRtDegradeTests(unittest.TestCase):
    def test_linux_rt_begin_false_on_windows(self) -> None:
        from robot_station.motion import _linux_rt_begin

        if os.name == "nt":
            self.assertFalse(_linux_rt_begin())

    def test_linux_rt_begin_permission_error_is_false(self) -> None:
        from robot_station import motion

        class FakeOs:
            name = "posix"
            SCHED_FIFO = 1

            @staticmethod
            def sched_param(prio: int) -> int:
                return prio

            @staticmethod
            def sched_setscheduler(_pid: int, _policy: int, _param: object) -> None:
                raise PermissionError("no cap_sys_nice")

        with patch.object(motion, "os", FakeOs()):
            self.assertFalse(motion._linux_rt_begin())

    def test_linux_rt_begin_missing_fifo_is_false(self) -> None:
        from robot_station import motion

        class FakeOs:
            name = "posix"

        with patch.object(motion, "os", FakeOs()):
            self.assertFalse(motion._linux_rt_begin())

    def test_motion_start_succeeds_when_rt_unavailable(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.motion import MotionCore

        with patch("robot_station.motion._linux_rt_begin", return_value=False):
            core = MotionCore(StationConfig(arm="sim", open_browser=False))
            try:
                core.start()
                self.assertFalse(core._rt_armed)
            finally:
                core.stop()


if __name__ == "__main__":
    unittest.main()
