#!/usr/bin/env python3
"""Print a GitHub Actions annotation from an AppImage build log."""
from __future__ import annotations

import sys
from pathlib import Path

KEYS = (
    "error:",
    "ERROR",
    "fatal error",
    "CMake Error",
    "FAILED",
    "undefined reference",
    "multiple definition",
    "cannot find",
    "ninja: build stopped",
    "collect2:",
    "Refusing",
    "not gzip",
    "download empty",
    "customer readme missing",
    "fafu_motor.so was not produced",
    "python3 missing",
    "OS Support seems wrong",
    "linuxdeploy failed",
)


def main(argv: list[str]) -> int:
    log = Path(argv[1]) if len(argv) > 1 else Path("appimage-build.log")
    rc_path = Path(argv[2]) if len(argv) > 2 else Path("appimage-build.status")
    text = log.read_text(errors="replace") if log.is_file() else ""
    rc = rc_path.read_text(encoding="utf-8").strip() if rc_path.is_file() else "1"
    hits = [ln.strip() for ln in text.splitlines() if any(k in ln for k in KEYS)]
    tail = [ln.strip() for ln in text.splitlines()[-15:] if ln.strip()]
    body = " | ".join((hits[-12:] + ["//"] + tail) if hits else tail)
    print(body[:1200])
    if rc != "0":
        print(f"::error title=AppImage build failed::{body[:700]}")
        return 1
    print(f"::notice title=AppImage build ok::{body[:400]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
