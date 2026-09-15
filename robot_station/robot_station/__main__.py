"""python3 -m robot_station"""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_station.service import main

if __name__ == "__main__":
    raise SystemExit(main())
