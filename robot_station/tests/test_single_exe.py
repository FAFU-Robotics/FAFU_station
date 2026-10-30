"""Single-file customer exe: stub + zip + 48-byte FAFUEXE1 footer."""
from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACK = ROOT / "packaging" / "windows"
MAGIC = b"FAFUEXE1"
FOOTER = 48


def bundle_bytes(stub: bytes, payload: bytes) -> bytes:
    digest = hashlib.sha256(payload).digest()
    return stub + payload + digest + len(payload).to_bytes(8, "little") + MAGIC


def split_bundle(blob: bytes) -> tuple[bytes, bytes, bytes]:
    if len(blob) < FOOTER + 1 or blob[-8:] != MAGIC:
        raise ValueError("missing FAFUEXE1 footer")
    zip_len = int.from_bytes(blob[-16:-8], "little")
    digest = blob[-48:-16]
    zip_off = len(blob) - FOOTER - zip_len
    if zip_off < 0 or zip_len < 0:
        raise ValueError("bad payload length")
    payload = blob[zip_off : zip_off + zip_len]
    stub = blob[:zip_off]
    return stub, payload, digest


class FooterLayoutTests(unittest.TestCase):
    def test_round_trip(self) -> None:
        stub = b"MZSTUB"
        payload = b"PK\x03\x04payload-bytes"
        blob = bundle_bytes(stub, payload)
        got_stub, got_payload, digest = split_bundle(blob)
        self.assertEqual(got_stub, stub)
        self.assertEqual(got_payload, payload)
        self.assertEqual(digest, hashlib.sha256(payload).digest())
        self.assertEqual(len(blob), len(stub) + len(payload) + FOOTER)

    def test_corrupt_magic_is_rejected(self) -> None:
        blob = bundle_bytes(b"MZ", b"abc")[:-1] + b"X"
        with self.assertRaises(ValueError):
            split_bundle(blob)

    def test_zip_payload_extracts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "tree"
            (src / "runtime" / "python310").mkdir(parents=True)
            (src / "runtime" / "python310" / "python.exe").write_bytes(b"")
            (src / "FAFUArmStation.exe").write_bytes(b"thin")
            (src / "app").mkdir()
            (src / "app" / "station_desktop.py").write_text("#", encoding="utf-8")
            zpath = Path(tmp) / "payload.zip"
            with zipfile.ZipFile(zpath, "w") as zf:
                for path in src.rglob("*"):
                    if path.is_file():
                        zf.write(path, path.relative_to(src).as_posix())
            payload = zpath.read_bytes()
            blob = bundle_bytes(b"MZSTUB", payload)
            _, got, digest = split_bundle(blob)
            self.assertEqual(digest, hashlib.sha256(payload).digest())
            dest = Path(tmp) / "out"
            dest.mkdir()
            with zipfile.ZipFile(__import__("io").BytesIO(got)) as zf:
                zf.extractall(dest)
            self.assertTrue((dest / "FAFUArmStation.exe").is_file())
            self.assertTrue((dest / "app" / "station_desktop.py").is_file())


class SingleFileSourcesTests(unittest.TestCase):
    def test_sources_exist(self) -> None:
        self.assertTrue((PACK / "SingleFileLauncher.cs").is_file())
        self.assertTrue((PACK / "pack_single_exe.ps1").is_file())
        cs = (PACK / "SingleFileLauncher.cs").read_text(encoding="utf-8")
        self.assertIn(MAGIC.decode("ascii"), cs)
        self.assertIn("FooterSize = 48", cs)
        self.assertIn("ParseFooter", cs)
        self.assertIn("VerifyPayload", cs)
        self.assertIn("ReplaceInstall", cs)
        self.assertIn("recordings", cs)
        ps1 = (PACK / "pack_single_exe.ps1").read_text(encoding="utf-8")
        self.assertIn(MAGIC.decode("ascii"), ps1)
        self.assertIn("BUILD_MANIFEST.json", ps1)
        self.assertIn("exe_sha256", ps1)
        self.assertTrue((PACK / "verify_portable.ps1").is_file())
        verify = (PACK / "verify_portable.ps1").read_text(encoding="utf-8")
        self.assertIn("FAFUEXE1", verify)
        self.assertGreater(len(verify.strip()), 200)
        spec = (ROOT / "docs" / "SPEC.md").read_text(encoding="utf-8")
        self.assertIn("FAFUEXE1", spec)
        self.assertIn("pack_single_exe.ps1", spec)
        self.assertIn("单个", spec)
        self.assertIn("recordings", spec)
        self.assertTrue((PACK / "ACCEPTANCE.md").is_file())


@unittest.skipUnless(sys.platform == "win32", "Windows csc")
class SingleFileCompileTests(unittest.TestCase):
    def test_stub_compiles(self) -> None:
        windir = os.environ.get("WINDIR") or r"C:\Windows"
        csc = Path(windir) / "Microsoft.NET" / "Framework64" / "v4.0.30319" / "csc.exe"
        if not csc.is_file():
            csc = Path(windir) / "Microsoft.NET" / "Framework" / "v4.0.30319" / "csc.exe"
        if not csc.is_file():
            self.skipTest("csc.exe not found")
        net = csc.parent
        cs = PACK / "SingleFileLauncher.cs"
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "stub.exe"
            cmd = [
                str(csc),
                "/nologo",
                "/target:winexe",
                "/platform:anycpu",
                "/utf8output",
                f"/out:{out}",
                "/r:System.dll",
                "/r:System.Windows.Forms.dll",
                "/r:System.Drawing.dll",
                f"/r:{net / 'System.IO.Compression.dll'}",
                f"/r:{net / 'System.IO.Compression.FileSystem.dll'}",
                str(cs),
            ]
            import subprocess

            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            self.assertTrue(out.is_file())
            self.assertGreater(out.stat().st_size, 1024)

    def test_thin_launcher_compiles(self) -> None:
        windir = os.environ.get("WINDIR") or r"C:\Windows"
        csc = Path(windir) / "Microsoft.NET" / "Framework64" / "v4.0.30319" / "csc.exe"
        if not csc.is_file():
            csc = Path(windir) / "Microsoft.NET" / "Framework" / "v4.0.30319" / "csc.exe"
        if not csc.is_file():
            self.skipTest("csc.exe not found")
        cs = PACK / "PortableLauncher.cs"
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "thin.exe"
            cmd = [
                str(csc),
                "/nologo",
                "/target:winexe",
                "/platform:anycpu",
                "/utf8output",
                f"/out:{out}",
                "/r:System.dll",
                "/r:System.Windows.Forms.dll",
                "/r:System.Drawing.dll",
                str(cs),
            ]
            import subprocess

            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            self.assertTrue(out.is_file())


@unittest.skipUnless(sys.platform == "win32", "Windows file locking")
class InstallReplacementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        windir = Path(os.environ.get("WINDIR", r"C:\Windows"))
        csc = windir / "Microsoft.NET/Framework64/v4.0.30319/csc.exe"
        if not csc.is_file():
            raise unittest.SkipTest("csc.exe not found")
        cls.build = tempfile.TemporaryDirectory(prefix="fafu-installer-tests-")
        cls.addClassCleanup(cls.build.cleanup)
        cls.probe = Path(cls.build.name) / "InstallerProbe.exe"
        source = Path(cls.build.name) / "InstallerProbe.cs"
        source.write_text(r'''
using System;
using System.IO;
using System.Reflection;
internal static class InstallerProbe {
    static int Main(string[] args) {
        if (args[0] == "hold") {
            Console.WriteLine("READY"); Console.ReadLine(); return 0;
        }
        FileStream locked = null;
        try {
            if (args.Length > 3)
                locked = new FileStream(args[3], FileMode.Open, FileAccess.Read, FileShare.None);
            MethodInfo replace = typeof(SingleFileLauncher).GetMethod("ReplaceInstall", BindingFlags.Static | BindingFlags.NonPublic);
            replace.Invoke(null, new object[] {args[1], args[2]});
            Console.WriteLine("OK"); return 0;
        } catch (TargetInvocationException ex) {
            Console.WriteLine("ERROR " + ex.InnerException.Message); return 2;
        } finally { if (locked != null) locked.Dispose(); }
    }
}
''', encoding="utf-8")
        args = [str(csc), "/nologo", "/target:exe", "/main:InstallerProbe", "/utf8output",
                f"/out:{cls.probe}", "/r:System.dll", "/r:System.Windows.Forms.dll", "/r:System.Drawing.dll",
                f"/r:{csc.parent / 'System.IO.Compression.dll'}",
                f"/r:{csc.parent / 'System.IO.Compression.FileSystem.dll'}",
                str(PACK / "SingleFileLauncher.cs"), str(source)]
        result = subprocess.run(args, capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(prefix="fafu-replace-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.install, self.staging = self.root / "install", self.root / "staging"
        self.native = self.install / "app/vendor/sdk/fafu_motor.cp310-win_amd64.pyd"
        for relative, body in {
            "FAFUArmStation.exe": b"old-launcher", ".bundle_sha256": b"old-stamp",
            "app/obsolete.py": b"old-code", "runtime/python310/python.exe": b"old-python",
            "app/vendor/sdk/fafu_motor.cp310-win_amd64.pyd": b"native-extension",
            "recordings/traj.jsonl": b"user-trajectory",
            "recordings/workspace/programs/pick.json": b"user-program",
        }.items():
            file = self.install / relative
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(body)
        for relative, body in {
            "FAFUArmStation.exe": b"new-launcher", ".bundle_sha256": b"new-stamp",
            "app/new.py": b"new-code", "runtime/python310/python.exe": b"new-python",
        }.items():
            file = self.staging / relative
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(body)

    def snapshot(self):
        return {file.relative_to(self.install).as_posix(): file.read_bytes()
                for file in self.install.rglob("*") if file.is_file()}

    def replace(self, locked=None):
        args = [str(self.probe), "replace", str(self.staging), str(self.install)]
        if locked is not None:
            args.append(str(locked))
        return subprocess.run(args, capture_output=True, text=True, timeout=30,
                              creationflags=subprocess.CREATE_NO_WINDOW)

    def test_locked_native_extension_leaves_old_install_intact(self):
        before = self.snapshot()
        result = self.replace(self.native)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.snapshot(), before, "locked DLL must be found before any old file is removed")

    def test_failed_copy_rolls_back_old_install(self):
        before = self.snapshot()
        result = self.replace(self.staging / "app/new.py")
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.snapshot(), before, "a failed update must restore the old launcher, code, runtime and stamp")

    def test_success_replaces_code_and_preserves_user_data(self):
        result = self.replace()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        files = self.snapshot()
        self.assertNotIn("app/obsolete.py", files)
        self.assertEqual(files["app/new.py"], b"new-code")
        self.assertEqual(files[".bundle_sha256"], b"new-stamp")
        self.assertEqual(files["recordings/traj.jsonl"], b"user-trajectory")
        self.assertEqual(files["recordings/workspace/programs/pick.json"], b"user-program")

    def test_running_private_runtime_blocks_update_before_mutation(self):
        runtime = self.install / "runtime/python310/python.exe"
        shutil.copyfile(self.probe, runtime)
        child = subprocess.Popen([str(runtime), "hold"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 text=True, creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            self.assertEqual(child.stdout.readline().strip(), "READY")
            before = self.snapshot()
            result = self.replace()
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(str(child.pid), result.stdout)
            self.assertEqual(self.snapshot(), before)
        finally:
            child.communicate("\n", timeout=10)

    def test_payload_cannot_overwrite_recordings(self):
        source = self.staging / "recordings/traj.jsonl"
        source.parent.mkdir()
        source.write_bytes(b"must-not-overwrite")
        before = self.snapshot()
        result = self.replace()
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.snapshot(), before)


if __name__ == "__main__":
    unittest.main()
