#!/usr/bin/env python3
"""FAFU 机械臂站控：启动服务。"""
from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
os.chdir(ROOT)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_station.portable import apply_bundled_runtime
from robot_station.service import main

if __name__ == "__main__":
    apply_bundled_runtime()
    raise SystemExit(main())
