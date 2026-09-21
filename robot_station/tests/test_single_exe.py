"""Single-file customer exe: stub + zip + 48-byte FAFUEXE1 footer."""
from __future__ import annotations

import hashlib
import os
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


if __name__ == "__main__":
    unittest.main()
