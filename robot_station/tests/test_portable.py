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


def _fake_linux_install(tmp: str) -> Path:
    inst = Path(tmp)
    bindir = inst / "runtime" / "python310" / "lib"
    bindir.mkdir(parents=True)
    pybin = inst / "runtime" / "python310" / "bin"
    pybin.mkdir(parents=True)
    (pybin / "python3").write_bytes(b"")
    (inst / "app").mkdir()
    (inst / "app" / "run_station.py").write_text("#", encoding="utf-8")
    return inst


class PortableDetectTests(unittest.TestCase):
    def test_bundled_python_path_shape(self) -> None:
        # pathlib on POSIX cannot parse Windows drive paths; skip those there.
        if os.name == "nt":
            self.assertTrue(
                looks_like_bundled_python(r"C:\Users\x\FAFUArmStation\runtime\python310\python.exe")
            )
            self.assertFalse(looks_like_bundled_python(r"C:\Users\x\AppData\Local\Programs\Python\Python310\python.exe"))
            self.assertFalse(looks_like_bundled_python(r"C:\Python310\python.exe"))
        self.assertTrue(
            looks_like_bundled_python("/opt/FAFUArmStation/runtime/python310/bin/python3")
        )
        self.assertFalse(looks_like_bundled_python("/usr/bin/python3"))
        self.assertFalse(looks_like_bundled_python("/usr/lib/python3.10/bin/python3"))

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

    def test_appdir_fallback_is_portable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = _fake_linux_install(tmp)
            with patch.dict(
                os.environ,
                {"STATION_PORTABLE": "1", "STATION_INSTALL_ROOT": "", "APPDIR": str(inst)},
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    self.assertTrue(is_portable())
                    self.assertEqual(app_root().resolve(), (inst / "app").resolve())


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

    def test_apply_bundled_runtime_sets_ld_library_path_on_posix(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            inst = _fake_linux_install(tmp)
            lib = inst / "runtime" / "python310" / "lib"
            sdk_py = inst / "app" / "vendor" / "fafu_arm_sdk" / "fafu_robot_python"
            sdk_py.mkdir(parents=True)
            (sdk_py / "libserial_cmake.so").write_bytes(b"")
            with patch.dict(
                os.environ,
                {
                    "STATION_PORTABLE": "1",
                    "STATION_INSTALL_ROOT": str(inst),
                    "LD_LIBRARY_PATH": "/usr/lib",
                },
                clear=False,
            ):
                with patch("robot_station.portable.looks_like_bundled_python", return_value=False):
                    self.assertTrue(apply_bundled_runtime())
                    ld = os.environ.get("LD_LIBRARY_PATH") or ""
                    self.assertIn(str(lib), ld)
                    self.assertIn(str(sdk_py), ld)

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
        self.assertTrue((pack / "SingleFileLauncher.cs").is_file())
        self.assertTrue((pack / "pack_single_exe.ps1").is_file())
        self.assertTrue((pack / "verify_portable.ps1").is_file())
        verify = (pack / "verify_portable.ps1").read_text(encoding="utf-8")
        self.assertGreater(len(verify.strip()), 200)
        self.assertIn("FAFUEXE1", verify)
        self.assertIn("BUILD_MANIFEST.json", verify)
        self.assertIn("PUT_SDK_HERE", verify)
        self.assertFalse(
            (ROOT / "robot_station" / "preflight.py").is_file(),
            "empty Python preflight leftover; checks live in PortableLauncher.cs",
        )
        self.assertFalse(
            (ROOT / "tests" / "test_preflight.py").is_file(),
            "empty test_preflight.py leftover",
        )
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
        self.assertIn("STATION_INSTALL_ROOT", cs)
        self.assertIn("EnsureShortcut", cs)
        self.assertIn("STATION_URDF_DIR", cs)
        self.assertNotIn("InstallTo", cs)
        self.assertNotIn("LocalApplicationData", cs)
        self.assertNotIn("robocopy", cs)
        self.assertIn("python310", cs)
        self.assertIn("PINOCCHIO_WINDOWS_DLL_PATH", cs)
        self.assertIn("Library", cs)
        self.assertNotIn("FindPython", cs)
        self.assertNotIn("setx", cs.lower())
        self.assertNotIn("py -3", cs)
        single = (pack / "SingleFileLauncher.cs").read_text(encoding="utf-8")
        self.assertIn("FAFUEXE1", single)
        self.assertIn("LocalApplicationData", single)
        self.assertIn("ZipFile", single)
        self.assertIn(".bundle_sha256", single)
        self.assertIn("ReplaceInstall", single)
        self.assertIn("recordings", single)
        self.assertNotIn("setx", single.lower())
        self.assertNotIn("FindPython", single)
        portable_cs = (pack / "PortableLauncher.cs").read_text(encoding="utf-8")
        self.assertIn("RunPreflight", portable_cs)
        self.assertIn("HasWebView2", portable_cs)
        self.assertIn("HasVcRuntime", portable_cs)
        self.assertIn("ComPorts", portable_cs)
        self.assertIn("vcruntime140", portable_cs)
        self.assertIn("CH340", portable_cs)
        self.assertTrue((pack / "ACCEPTANCE.md").is_file())
        pack_single = (pack / "pack_single_exe.ps1").read_text(encoding="utf-8")
        self.assertIn("FAFUEXE1", pack_single)
        self.assertIn("SingleFileLauncher.cs", pack_single)
        self.assertIn("CreateFromDirectory", pack_single)
        self.assertIn("AllowNoSdk", pack_single)
        self.assertIn("BUILD_MANIFEST.json", pack_single)
        self.assertIn("exe_sha256", pack_single)
        self.assertIn("payload_sha256", pack_single)
        self.assertIn("verify_portable.ps1", pack_single)
        self.assertNotIn("setx ", pack_single.lower())
        ps1 = (pack / "build_portable.ps1").read_text(encoding="utf-8")
        self.assertIn("bootstrap.pypa.io/get-pip.py", ps1)
        self.assertIn("embed-amd64", ps1)
        self.assertIn("station.yaml", ps1)
        self.assertIn("envBackup", ps1)
        self.assertIn("启动真机.bat", ps1)
        self.assertIn("urdf", ps1)
        self.assertIn("bundle_pinocchio.py", ps1)
        self.assertIn("SkipPinocchio", ps1)
        self.assertIn("WithPinocchio", ps1)
        self.assertIn("SkipSingleExe", ps1)
        self.assertIn("pack_single_exe.ps1", ps1)
        self.assertIn("win32icon", ps1)
        self.assertIn("Find-PinocchioEnv", ps1)
        self.assertIn("FAFUArmStation.ico", ps1)
        self.assertIn("station-customer-readme", ps1)
        self.assertIn("README.md", ps1)
        self.assertTrue((ROOT / "docs" / "客户使用说明.md").is_file())
        customer = (ROOT / "docs" / "客户使用说明.md").read_text(encoding="utf-8")
        self.assertIn("客户使用说明", customer)
        self.assertIn("station-customer-readme", customer)
        self.assertIn("急停", customer)
        self.assertIn("示教轨迹", customer)
        self.assertIn("重力 URDF", customer)
        self.assertIn("一个文件", customer)
        self.assertIn("LOCALAPPDATA", customer)
        self.assertNotIn("请把整个", customer)
        self.assertNotIn("setx ", ps1.lower())
        self.assertNotIn("setenvironmentvariable", ps1.lower())
        install = (pack / "install_user.bat").read_text(encoding="utf-8")
        self.assertIn(r"%LOCALAPPDATA%\FAFUArmStation", install)
        self.assertIn("FAFUArmStation.exe", install)
        self.assertIn("start", install.lower())
        self.assertNotIn("setx ", install.lower())
        live = (ROOT / "启动真机.bat").read_text(encoding="ascii")
        self.assertNotIn("install_home", live)
        self.assertIn("goto portable", live)
        self.assertIn("STATION_URDF_DIR", live)
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
