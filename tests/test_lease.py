from __future__ import annotations

import time
import unittest

from robot_station.config import StationConfig
from robot_station.service import Station


def _cfg(**kwargs) -> StationConfig:
    data = dict(
        watchdog_s=0.5,
        video_hz=5,
        camera_width=16,
        camera_height=16,
        control_hz=50,
        telemetry_hz=50,
    )
    data.update(kwargs)
    return StationConfig(**data)


class LocalControlTests(unittest.TestCase):
    def test_any_client_can_move_without_take(self) -> None:
        st = Station(_cfg())
        st.start()
        try:
            st.add_client("a")
            self.assertIsNone(st.on_arm_targets("a", [40.0, 0, 0, 0, 0, 0], 80))
            time.sleep(0.10)
            self.assertGreater(st.snapshot()["arm"]["q_deg"][0], 2)
        finally:
            st.stop()

    def test_second_client_can_also_command(self) -> None:
        st = Station(_cfg())
        st.start()
        try:
            st.add_client("a")
            st.add_client("b")
            self.assertIsNone(st.on_arm_targets("b", [40.0, 0, 0, 0, 0, 0], 80))
            time.sleep(0.10)
            self.assertGreater(st.snapshot()["arm"]["q_deg"][0], 2)
        finally:
            st.stop()

    def test_last_client_drop_holds_arm(self) -> None:
        st = Station(_cfg())
        st.start()
        try:
            st.add_client("a")
            st.on_arm_targets("a", [40.0, 0, 0, 0, 0, 0], 80)
            time.sleep(0.08)
            st.drop_client("a")
            time.sleep(0.08)
            snap = st.snapshot()
            self.assertAlmostEqual(snap["arm"]["q_deg"][0], snap["arm"]["target_deg"][0], places=1)
        finally:
            st.stop()

    def test_touch_keeps_watchdog_and_move_j(self) -> None:
        st = Station(_cfg(watchdog_s=0.8, arm="sim"))
        st.start()
        try:
            st.add_client("a")
            deadline = time.monotonic() + 0.9
            while time.monotonic() < deadline:
                st.on_touch("a")
                time.sleep(0.05)
            self.assertIsNone(st.on_arm_targets("a", [20.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80))
            deadline = time.monotonic() + 1.2
            q0 = 0.0
            while time.monotonic() < deadline:
                st.on_touch("a")
                q0 = st.snapshot()["arm"]["q_deg"][0]
                if q0 > 8.0:
                    break
                time.sleep(0.04)
            self.assertGreater(q0, 8.0)
        finally:
            st.stop()


if __name__ == "__main__":
    unittest.main()
