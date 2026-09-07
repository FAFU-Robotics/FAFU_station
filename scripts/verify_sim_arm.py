#!/usr/bin/env python3
"""Exercise the sim arm stack in-process. Never opens USB serial or CAN.

Run without opening the real USB serial:

    python scripts/verify_sim_arm.py
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.environ.pop("STATION_ALLOW_LIVE_ARM", None)


def main() -> int:
    from robot_station.adapters.fafu_sim import FakeFafuController
    from robot_station.config import StationConfig
    from robot_station.motion import MotionCore
    from robot_station.serial_guard import live_serial_allowed

    if live_serial_allowed():
        print("STATION_ALLOW_LIVE_ARM 已打开，本脚本拒绝继续，以免误开真串口。")
        return 2

    FakeFafuController.last = None
    core = MotionCore(
        StationConfig(
            arm="sim",
            camera="mock",
            open_browser=False,
            control_hz=80,
            watchdog_s=2.0,
            camera_width=16,
            camera_height=16,
            video_hz=5,
        )
    )
    core.start()
    try:
        backend = core.snapshot()["arm"]["backend"]
        if backend != "sim":
            print("backend 不是 sim:", backend)
            return 1
        if core.on_power("v", True):
            print("enable 失败")
            return 1
        if core.on_arm_targets("v", [8.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80.0):
            print("move_j 失败")
            return 1
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and core.snapshot()["arm"]["q_deg"][0] < 4.0:
            time.sleep(0.04)
        if core.snapshot()["arm"]["q_deg"][0] < 4.0:
            print("关节未动")
            return 1
        x0 = core.snapshot()["arm"]["ee_m"][0]
        if core.on_cartesian("v", [0.02, 0.0, 0.0], [0.0, 0.0, 0.0]):
            print("笛卡尔失败")
            return 1
        deadline = time.monotonic() + 1.5
        x1 = x0
        while time.monotonic() < deadline:
            x1 = core.snapshot()["arm"]["ee_m"][0]
            if x1 > x0 + 0.005:
                break
            time.sleep(0.04)
        if x1 <= x0:
            print("末端 X 未增加")
            return 1
        core.on_estop("verify")
        fake = FakeFafuController.last
        assert fake is not None
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            if "hold_position" in fake.calls:
                break
            time.sleep(0.02)
        if "hold_position" not in fake.calls:
            print("急停未保持姿态:", fake.calls[-16:])
            return 1
        if "move_j" not in fake.calls and "servo_j" not in fake.calls:
            print("Fake 未收到运动方法:", fake.calls[-16:])
            return 1
        if "emergency_stop" in fake.calls:
            print("急停不应再调用 SDK emergency_stop（MODE_STOP 会卸力）")
            return 1
        if fake.kwargs.get("auto_enable") is not False:
            print("auto_enable 不是 False")
            return 1
        print("sim 通路 OK：enable / move_j / cartesian / estop；未开串口")
        return 0
    finally:
        core.stop()


if __name__ == "__main__":
    raise SystemExit(main())
