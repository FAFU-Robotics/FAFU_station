from __future__ import annotations

import math
import time
import unittest

from robot_station.adapters.arm import MockArm
from robot_station.adapters.camera import MockCameraBank, rgb_png
from robot_station.adapters.fafu_kin import forward
from robot_station.config import StationConfig
from robot_station.service import Station


class MockLoopTests(unittest.TestCase):
    def test_png_header(self) -> None:
        blob = rgb_png(b"\x00\x80\xff" * 4, 2, 2)
        self.assertTrue(blob.startswith(b"\x89PNG"))

    def test_cartesian_pose_uses_speed_not_instant(self) -> None:
        arm = MockArm()
        xyz0, rpy0 = forward([math.radians(x) for x in arm.poll().q_deg])
        rpy_deg = [math.degrees(x) for x in rpy0]
        err = arm.apply_cartesian_pose(
            [xyz0[0] + 0.04, xyz0[1], xyz0[2]], rpy_deg, speed_deg_s=20.0
        )
        self.assertIsNone(err, err)
        xyz1, _ = forward([math.radians(x) for x in arm.poll().q_deg])
        self.assertLess(xyz1[0], xyz0[0] + 0.008)
        for _ in range(120):
            arm.servo(0.05)
        xyz2, _ = forward([math.radians(x) for x in arm.poll().q_deg])
        self.assertGreater(xyz2[0], xyz0[0] + 0.02)

    def test_cartesian_moves_x(self) -> None:
        arm = MockArm()
        xyz0, _ = forward([math.radians(x) for x in arm.poll().q_deg])
        err = arm.apply_cartesian([0.03, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.assertIsNone(err)
        xyz1, _ = forward([math.radians(x) for x in arm.poll().q_deg])
        self.assertGreater(xyz1[0], xyz0[0] + 0.008)

    def test_stream_lands_without_ui_speed(self) -> None:
        arm = MockArm()
        arm.apply_targets([40.0, 40.0, 40.0, 0.0, 0.0, 0.0], speed_deg_s=40.0, stream=True)
        self.assertAlmostEqual(arm.poll().q_deg[0], 40.0, places=4)

    def test_cartesian_uses_commanded_pose(self) -> None:
        arm = MockArm()
        with arm._lock:
            arm._tgt = [0.0, 40.0, 40.0, 0.0, 0.0, 0.0]
            arm._q = [30.0, 40.0, 40.0, 0.0, 0.0, 0.0]
        err = arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.assertIsNone(err)
        self.assertLess(abs(arm.poll().q_deg[0]), 8.0)

    def test_arm_moves_toward_target(self) -> None:
        arm = MockArm()
        arm.apply_targets([10, 0, 0, 0, 0, 0], speed_deg_s=50)
        for _ in range(20):
            arm.servo(0.05)
        snap = arm.poll()
        self.assertGreater(snap.q_deg[0], 4)
        arm.hold()
        held = arm.poll().q_deg[0]
        arm.servo(0.05)
        self.assertAlmostEqual(arm.poll().q_deg[0], held, places=3)

    def test_arm_holds_on_estop(self) -> None:
        st = Station(StationConfig(watchdog_s=0.5, video_hz=5, camera_width=16, camera_height=16))
        st.start()
        try:
            st.on_arm_targets("a", [40.0, 0, 0, 0, 0, 0], 80)
            time.sleep(0.08)
            st.on_estop("test")
            time.sleep(0.05)
            snap = st.snapshot()
            self.assertEqual(snap["safety"], "ESTOP_LATCHED")
            held = snap["arm"]["q_deg"][0]
            time.sleep(0.08)
            self.assertAlmostEqual(st.snapshot()["arm"]["q_deg"][0], held, places=1)
            self.assertIsNone(st.on_clear_estop("a"))
            self.assertEqual(st.snapshot()["safety"], "IDLE")
        finally:
            st.stop()

    def test_mock_camera_emits(self) -> None:
        bank = MockCameraBank(1, 32, 18, 20)
        bank.start()
        try:
            deadline = time.time() + 1.0
            got = None
            while time.time() < deadline:
                got = bank.latest_png(0)
                if got:
                    break
                time.sleep(0.05)
            self.assertIsNotNone(got)
            self.assertTrue(got[2].startswith(b"\x89PNG"))
            self.assertEqual(bank.stream_ids(), [0])
            self.assertEqual(bank.poll().roles, ["作业相机"])
        finally:
            bank.stop()


if __name__ == "__main__":
    unittest.main()
