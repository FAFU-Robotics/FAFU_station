from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_station.portable import (
    app_root,
    apply_bundled_runtime,
    child_env,
    is_portable,
    looks_like_bundled_python,
)
from robot_station.service import spawn_motion
from robot_station.config import StationConfig

ROOT = Path(__file__).resolve().parents[1]


def _fake_install(tmp: str) -> Path:
    inst = Path(tmp)
    py = inst / "runtime" / "python310"
    py.mkdir(parents=True)
    (py / "python.exe").write_bytes(b"")
    (inst / "app").mkdir()
    (inst / "app" / "run_station.py").write_text("#", encoding="utf-8")
    return inst


class PortableDetectTests(unittest.TestCase):
    def test_bundled_python_path_shape(self) -> None:
        self.assertTrue(
            looks_like_bundled_python(r"C:\Users\x\FAFUArmStation\runtime\python310\python.exe")
        )
        self.assertFalse(looks_like_bundled_python(r"C:\Users\x\AppData\Local\Programs\Python\Python310\python.exe"))
        self.assertFalse(looks_like_bundled_python(r"C:\Python310\python.exe"))

    def test_leftover_flag_does_not_hijack_system_python(self) -> None:
        with patch.dict(os.environ, {"STATION_PORTABLE": "1", "STATION_INSTALL_ROOT": ""}, clear=False):
            with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                self.assertFalse(is_portable())

    def test_flag_with_install_root_is_portable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = _fake_install(tmp)
            with patch.dict(
                os.environ,
                {"STATION_PORTABLE": "1", "STATION_INSTALL_ROOT": str(inst)},
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    self.assertTrue(is_portable())


class PortableEnvTests(unittest.TestCase):
    def test_child_env_replaces_pythonpath_when_portable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = _fake_install(tmp)
            with patch.dict(
                os.environ,
                {
                    "STATION_PORTABLE": "1",
                    "STATION_INSTALL_ROOT": str(inst),
                    "PYTHONPATH": r"C:\other\site-packages",
                    "PYTHONSTARTUP": "x.py",
                    "PYTHONHOME": r"C:\Python310",
                },
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    env = child_env(ROOT)
            self.assertEqual(env["PYTHONPATH"], str(ROOT))
            self.assertEqual(env["PYTHONNOUSERSITE"], "1")
            self.assertEqual(env["PIP_USER"], "0")
            self.assertNotIn("PYTHONSTARTUP", env)
            self.assertNotIn("PYTHONHOME", env)
            self.assertNotIn(r"C:\other\site-packages", env["PYTHONPATH"])

    def test_leftover_flag_keeps_developer_pythonpath(self) -> None:
        with patch.dict(
            os.environ,
            {"STATION_PORTABLE": "1", "STATION_INSTALL_ROOT": "", "PYTHONPATH": "legacy"},
            clear=False,
        ):
            with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                env = child_env(ROOT)
        self.assertTrue(env["PYTHONPATH"].startswith(str(ROOT)))
        self.assertIn("legacy", env["PYTHONPATH"])

    def test_child_env_appends_when_not_portable(self) -> None:
        with patch.dict(os.environ, {"STATION_PORTABLE": "", "PYTHONPATH": "legacy"}, clear=False):
            with patch("robot_station.portable.is_portable", return_value=False):
                env = child_env(ROOT)
        self.assertTrue(env["PYTHONPATH"].startswith(str(ROOT)))
        self.assertIn("legacy", env["PYTHONPATH"])

    def test_apply_sets_private_flags(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = _fake_install(tmp)
            with patch.dict(
                os.environ,
                {
                    "STATION_PORTABLE": "1",
                    "STATION_INSTALL_ROOT": str(inst),
                    "PYTHONSTARTUP": "x.py",
                    "PYTHONHOME": r"C:\Python310",
                    "PYTHONPATH": "hijack",
                    "FAFU_ARM_SDK": r"C:\not-this-app\fafu_arm_sdk",
                },
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    self.assertTrue(apply_bundled_runtime())
                    self.assertEqual(os.environ.get("PYTHONNOUSERSITE"), "1")
                    self.assertEqual(os.environ.get("PYTHONPATH"), str(inst / "app"))
                    self.assertNotIn("PYTHONSTARTUP", os.environ)
                    self.assertNotIn("PYTHONHOME", os.environ)
                    self.assertNotEqual(os.environ.get("FAFU_ARM_SDK"), r"C:\not-this-app\fafu_arm_sdk")

    def test_apply_bundled_runtime_exposes_pinocchio_dll_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = _fake_install(tmp)
            libbin = inst / "runtime" / "python310" / "Library" / "bin"
            libbin.mkdir(parents=True)
            (libbin / "pinocchio_default.dll").write_bytes(b"")
            with patch.dict(
                os.environ,
                {
                    "STATION_PORTABLE": "1",
                    "STATION_INSTALL_ROOT": str(inst),
                    "PATH": r"C:\Windows\System32",
                },
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    self.assertTrue(apply_bundled_runtime())
                    self.assertEqual(os.environ.get("PINOCCHIO_WINDOWS_DLL_PATH"), str(libbin))
                    self.assertTrue(os.environ["PATH"].startswith(str(libbin)))

    def test_spawn_motion_does_not_keep_foreign_path_when_portable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = _fake_install(tmp)
            with patch.dict(
                os.environ,
                {
                    "STATION_PORTABLE": "1",
                    "STATION_INSTALL_ROOT": str(inst),
                    "PYTHONPATH": r"C:\hijack",
                },
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    with patch("robot_station.service.subprocess.Popen") as popen:
                        popen.return_value = object()
                        spawn_motion(StationConfig(arm="sim"), None)
            kwargs = popen.call_args.kwargs
            self.assertNotIn(r"C:\hijack", kwargs["env"]["PYTHONPATH"])
            self.assertEqual(kwargs["env"]["PYTHONPATH"], str(ROOT))


class PortablePackagingFilesTests(unittest.TestCase):
    def test_packaging_kit_exists(self) -> None:
        pack = ROOT / "packaging" / "windows"
        self.assertTrue((pack / "build_portable.ps1").is_file())
        self.assertTrue((pack / "build_portable.bat").is_file())
        self.assertTrue((pack / "PortableLauncher.cs").is_file())
        self.assertTrue((pack / "python310._pth").is_file())
        self.assertTrue((pack / "install_user.bat").is_file())
        self.assertTrue((pack / "uninstall_user.bat").is_file())
        self.assertTrue((pack / "install_shortcut.vbs").is_file())
        self.assertTrue((pack / "FAFUArmStation.ico").is_file())
        shortcut = (pack / "install_shortcut.vbs").read_text(encoding="ascii", errors="ignore")
        self.assertIn("FAFUArmStation.ico", shortcut)
        self.assertTrue((pack / "bundle_pinocchio.py").is_file())
        pth = (pack / "python310._pth").read_text(encoding="utf-8")
        self.assertIn("..\\..\\app", pth.replace("/", "\\"))
        self.assertIn("import site", pth)
        cs = (pack / "PortableLauncher.cs").read_text(encoding="utf-8")
        self.assertIn("STATION_PORTABLE", cs)
        self.assertIn("PYTHONNOUSERSITE", cs)
        self.assertIn("PYTHONHOME", cs)
        self.assertIn("Remove", cs)
        self.assertIn("LocalApplicationData", cs)
        self.assertIn("InstallTo", cs)
        self.assertIn("robocopy", cs)
        self.assertIn("python310", cs)
        self.assertIn("PINOCCHIO_WINDOWS_DLL_PATH", cs)
        self.assertIn("Library", cs)
        self.assertNotIn("FindPython", cs)
        self.assertNotIn("setx", cs.lower())
        self.assertNotIn("py -3", cs)
        ps1 = (pack / "build_portable.ps1").read_text(encoding="utf-8")
        self.assertIn("bootstrap.pypa.io/get-pip.py", ps1)
        self.assertIn("embed-amd64", ps1)
        self.assertIn("station.yaml", ps1)
        self.assertIn("envBackup", ps1)
        self.assertIn("启动真机.bat", ps1)
        self.assertIn("urdf", ps1)
        self.assertIn("bundle_pinocchio.py", ps1)
        self.assertIn("SkipPinocchio", ps1)
        self.assertIn("Find-PinocchioEnv", ps1)
        self.assertIn("FAFUArmStation.ico", ps1)
        self.assertIn("客户使用说明.md", ps1)
        self.assertIn("README.md", ps1)
        self.assertTrue((ROOT / "docs" / "客户使用说明.md").is_file())
        customer = (ROOT / "docs" / "客户使用说明.md").read_text(encoding="utf-8")
        self.assertIn("客户使用说明", customer)
        self.assertIn("急停", customer)
        self.assertIn("示教轨迹", customer)
        self.assertIn("重力 URDF", customer)
        self.assertNotIn("setx ", ps1.lower())
        self.assertNotIn("setenvironmentvariable", ps1.lower())
        install = (pack / "install_user.bat").read_text(encoding="utf-8")
        self.assertIn(r"%LOCALAPPDATA%\FAFUArmStation", install)
        self.assertIn("FAFUArmStation.exe", install)
        self.assertIn("start", install.lower())
        self.assertNotIn("setx ", install.lower())
        live = (ROOT / "启动真机.bat").read_text(encoding="ascii")
        self.assertIn("install_home", live)
        self.assertIn("LOCALAPPDATA", live)
        self.assertIn("robocopy", live)
        self.assertIn("STATION_PORTABLE", live)
        self.assertIn("PYTHONNOUSERSITE", live)
        self.assertIn("PYTHONHOME", live)
        self.assertIn("Python310", live)
        self.assertNotIn("setx", live.lower())

    def test_packaging_does_not_write_machine_path(self) -> None:
        pack = ROOT / "packaging" / "windows"
        for path in pack.glob("*"):
            if not path.is_file():
                continue
            raw = path.read_bytes().lower()
            self.assertNotIn(b"setx ", raw)
            self.assertNotIn(b"setx.exe", raw)
            self.assertNotIn(b"setenvironmentvariable", raw)
            self.assertNotIn(b"hklm\\system\\currentcontrolset\\control\\session manager\\environment", raw)


class PortableSdkIsolationTests(unittest.TestCase):
    def test_portable_ignores_foreign_and_home_sdk(self) -> None:
        from robot_station.adapters.fafu_arm import resolve_sdk_root

        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            inst = _fake_install(str(base / "pack"))
            home = base / "home"
            foreign = home / "fafu_arm_sdk"
            (foreign / "fafu_robot_python").mkdir(parents=True)
            with patch.dict(
                os.environ,
                {
                    "STATION_PORTABLE": "1",
                    "STATION_INSTALL_ROOT": str(inst),
                    "FAFU_ARM_SDK": str(foreign),
                },
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    with patch("robot_station.adapters.fafu_arm.Path.home", return_value=home):
                        try:
                            found = resolve_sdk_root()
                        except FileNotFoundError:
                            return
                        self.assertFalse(str(found.resolve()).startswith(str(home.resolve())))


class PortableLayoutSmokeTests(unittest.TestCase):
    def test_fake_install_root_from_env(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = Path(tmp)
            py = inst / "runtime" / "python310"
            py.mkdir(parents=True)
            (py / "python.exe").write_bytes(b"")
            (inst / "app").mkdir()
            (inst / "app" / "run_station.py").write_text("#", encoding="utf-8")
            with patch.dict(
                os.environ,
                {"STATION_PORTABLE": "1", "STATION_INSTALL_ROOT": str(inst)},
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    self.assertEqual(app_root().resolve(), (inst / "app").resolve())


if __name__ == "__main__":
    unittest.main()
