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
    def test_tick_continues_after_poll_error(self) -> None:
        from robot_station.motion import MotionCore

        core = MotionCore(StationConfig(arm="mock", open_browser=False, control_hz=50))
        core.start()
        try:
            orig = core.arm.poll
            state = {"n": 0}

            def flaky():
                state["n"] += 1
                if state["n"] <= 3:
                    raise RuntimeError("io")
                return orig()

            core.arm.poll = flaky
            time.sleep(0.2)
            self.assertTrue(core._tick_th is not None and core._tick_th.is_alive())
            self.assertGreater(state["n"], 3)
            # The error is transient: the next healthy tick clears it.
            self.assertNotIn("运动拍异常", core.snapshot().get("cmd_err") or "")
        finally:
            core.stop()

    def test_tick_error_is_published_and_cleared(self) -> None:
        from robot_station.motion import MotionCore, TICK_ERR_TEXT

        core = MotionCore(StationConfig(arm="mock", open_browser=False, control_hz=50))
        core.start()
        try:
            orig = core._step
            seq0 = core.snapshot()["seq"]

            def boom(now: float, dt: float) -> None:
                raise RuntimeError("tick exploded")

            core._step = boom
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                snap = core.snapshot()
                if snap["seq"] > seq0 and TICK_ERR_TEXT in (snap.get("cmd_err") or ""):
                    break
                time.sleep(0.02)
            snap = core.snapshot()
            self.assertGreater(snap["seq"], seq0)
            self.assertIn(TICK_ERR_TEXT, snap.get("cmd_err") or "")

            core._step = orig
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if not (core.snapshot().get("cmd_err") or ""):
                    break
                time.sleep(0.02)
            self.assertNotIn(TICK_ERR_TEXT, core.snapshot().get("cmd_err") or "")
        finally:
            core.stop()

    def test_mock_cartesian_rejects_small_orientation_noop(self) -> None:
        arm = MockArm()
        arm.teach_pose([0.0] * 6)
        # At the zero pose the 24-iteration fallback solver returns the seed;
        # before the adaptive tolerance that looked like success.
        err = arm.apply_cartesian([0.0, 0.0, 0.0], [0.0, 0.0, 2.9])
        self.assertIsNotNone(err)
        self.assertIn("IK 无解", err or "")
        # Away from the singularity a 1.7 deg step is a real solution and must
        # still be accepted (the tolerance is 0.5 deg for this request).
        arm.teach_pose([0.0, 40.0, 40.0, 0.0, 0.0, 0.0])
        before = list(arm._tgt)
        err = arm.apply_cartesian([0.0, 0.0, 0.0], [0.0, 0.0, 1.7])
        self.assertIsNone(err, err)
        self.assertNotEqual(before, list(arm._tgt))

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
