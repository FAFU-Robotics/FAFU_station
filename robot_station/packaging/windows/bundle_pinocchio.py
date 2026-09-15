"""Copy conda-forge pinocchio into an embeddable CPython 3.10 tree.

Windows ``pip install pinocchio`` is not the real library. Gravity on the
portable package therefore copies a working conda env (py310 + pinocchio)
into ``runtime/python310``.

Skips the casadi wrapper so we do not pull casadi.dll.
"""
from __future__ import annotations

import argparse
import shutil
import struct
import sys
from pathlib import Path

PACKAGES = ("pinocchio", "eigenpy", "coal", "hppfcl")
SKIP_FILES = frozenset(
    {
        "pinocchio_pywrap_casadi.cp310-win_amd64.pyd",
        "pinocchio_casadi.dll",
    }
)
SYSTEM_DLLS = frozenset(
    {
        "kernel32.dll",
        "user32.dll",
        "gdi32.dll",
        "advapi32.dll",
        "shell32.dll",
        "ole32.dll",
        "oleaut32.dll",
        "ws2_32.dll",
        "wsock32.dll",
        "ntdll.dll",
        "rpcrt4.dll",
        "combase.dll",
        "shlwapi.dll",
        "comdlg32.dll",
        "winmm.dll",
        "imm32.dll",
        "version.dll",
        "bcrypt.dll",
        "crypt32.dll",
        "secur32.dll",
        "iphlpapi.dll",
        "dbghelp.dll",
        "psapi.dll",
        "setupapi.dll",
        "wintrust.dll",
        "wldap32.dll",
        "normaliz.dll",
        "dnsapi.dll",
        "mswsock.dll",
        "ucrtbase.dll",
        "vcruntime140.dll",
        "vcruntime140_1.dll",
        "msvcp140.dll",
        "concrt140.dll",
        "msvcrt.dll",
        "python310.dll",
        "python3.dll",
    }
)


def _is_system_dll(name: str) -> bool:
    low = name.lower()
    return low in SYSTEM_DLLS or low.startswith("api-ms-win-") or low.startswith("ext-ms-")


def pe_imports(path: Path) -> list[str]:
    data = path.read_bytes()
    if data[:2] != b"MZ":
        return []
    e_lfanew = struct.unpack_from("<I", data, 0x3C)[0]
    if data[e_lfanew : e_lfanew + 4] != b"PE\x00\x00":
        return []
    magic = struct.unpack_from("<H", data, e_lfanew + 24)[0]
    if magic == 0x20B:
        dd_off = e_lfanew + 24 + 112
    elif magic == 0x10B:
        dd_off = e_lfanew + 24 + 96
    else:
        return []
    imp_rva = struct.unpack_from("<I", data, dd_off + 8)[0]
    if not imp_rva:
        return []
    num_sections = struct.unpack_from("<H", data, e_lfanew + 6)[0]
    opt_size = struct.unpack_from("<H", data, e_lfanew + 20)[0]
    sec0 = e_lfanew + 24 + opt_size
    sections: list[tuple[int, int, int]] = []
    for i in range(num_sections):
        off = sec0 + i * 40
        va, rawsz, rawptr = struct.unpack_from("<III", data, off + 12)
        sections.append((va, rawsz, rawptr))

    def rva_to_off(rva: int) -> int | None:
        for va, rawsz, rawptr in sections:
            if va <= rva < va + max(rawsz, 1):
                return rawptr + (rva - va)
        return None

    names: list[str] = []
    desc = rva_to_off(imp_rva)
    if desc is None:
        return []
    while True:
        lookup, _t, _f, name_rva, _ft = struct.unpack_from("<IIIII", data, desc)
        if lookup == 0 and name_rva == 0:
            break
        no = rva_to_off(name_rva)
        if no is not None:
            end = data.find(b"\x00", no)
            names.append(data[no:end].decode("ascii", "replace"))
        desc += 20
    return names


def env_root(src: Path) -> Path:
    if src.is_file():
        src = src.parent
    if (src / "python.exe").is_file() and (src / "Lib" / "site-packages").is_dir():
        return src
    raise SystemExit(f"not a Python 3.10 env root: {src}")


def _copy_tree(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(
        src,
        dst,
        ignore=shutil.ignore_patterns(*SKIP_FILES, "__pycache__", "*.pyc", "*.pyo"),
    )


def collect_dlls(src_root: Path, starts: list[Path]) -> list[Path]:
    search = [
        src_root / "Library" / "bin",
        src_root / "Library" / "mingw-w64" / "bin",
        src_root / "DLLs",
    ]
    needed: dict[str, Path] = {}
    queue = list(starts)
    seen: set[str] = set()
    missing: list[str] = []
    while queue:
        cur = queue.pop(0)
        key = cur.name.lower()
        if key in seen:
            continue
        seen.add(key)
        if cur.suffix.lower() == ".dll" and cur.name not in SKIP_FILES:
            needed[cur.name] = cur
        try:
            deps = pe_imports(cur)
        except OSError:
            continue
        for dep in deps:
            if _is_system_dll(dep) or dep in SKIP_FILES:
                continue
            found = None
            for folder in search:
                cand = folder / dep
                if cand.is_file():
                    found = cand
                    break
            if found is not None:
                queue.append(found)
            else:
                missing.append(dep)
    if missing:
        uniq = ", ".join(sorted(set(missing)))
        raise SystemExit(f"pinocchio native deps missing from conda env: {uniq}")
    return sorted(needed.values(), key=lambda p: p.name.lower())


def bundle(src_root: Path, dest_py: Path) -> None:
    site = src_root / "Lib" / "site-packages"
    dest_site = dest_py / "Lib" / "site-packages"
    dest_site.mkdir(parents=True, exist_ok=True)
    starts: list[Path] = []
    for name in PACKAGES:
        src = site / name
        if not src.is_dir():
            if name == "hppfcl":
                continue
            raise SystemExit(f"conda env missing package {name}: {src}")
        _copy_tree(src, dest_site / name)
        info = next(site.glob(f"{name}-*.dist-info"), None)
        if info is not None and info.is_dir():
            _copy_tree(info, dest_site / info.name)
        starts.extend(
            p
            for p in (dest_site / name).rglob("*.pyd")
            if p.name not in SKIP_FILES
        )
        starts.extend(
            p
            for p in src.rglob("*.pyd")
            if p.name not in SKIP_FILES
        )
    dlls = collect_dlls(src_root, starts)
    dest_bin = dest_py / "Library" / "bin"
    dest_bin.mkdir(parents=True, exist_ok=True)
    for dll in dlls:
        shutil.copy2(dll, dest_bin / dll.name)
    print(f"pinocchio packages: {', '.join(PACKAGES)}")
    print(f"native DLLs: {len(dlls)} -> {dest_bin}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--from", dest="src", required=True)
    parser.add_argument("--to", dest="dest", required=True)
    args = parser.parse_args()
    bundle(env_root(Path(args.src)), Path(args.dest))
    return 0


if __name__ == "__main__":
    sys.exit(main())
