from __future__ import annotations

import math
import os
import time
import unittest
from unittest.mock import patch

from robot_station.adapters.arm import build_arm
from robot_station.adapters.fafu_arm import FafuArm
from robot_station.adapters.fafu_kin import forward_pose
from robot_station.adapters.fafu_sim import FakeFafuController
from robot_station.config import StationConfig
from robot_station.motion import MotionCore


class SimMotionTests(unittest.TestCase):
    """arm: sim uses FakeFafuController. No USB serial."""

    def test_build_sim(self) -> None:
        arm = build_arm("sim", 6, 40.0, True)
        self.assertIsInstance(arm, FafuArm)
        self.assertEqual(arm.backend, "sim")
        self.assertTrue(arm.allow_motion)

    def test_send_position_uses_servo_path(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
        )
        core.start()
        try:
            err = core.on_arm_targets("a", [12.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80.0)
            self.assertIsNone(err)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                if core.snapshot()["arm"]["q_deg"][0] > 6.0:
                    break
                time.sleep(0.04)
            snap = core.snapshot()["arm"]
            self.assertEqual(snap["backend"], "sim")
            self.assertTrue(snap["enabled"])
            self.assertGreater(snap["q_deg"][0], 6.0)
            fake = FakeFafuController.last
            assert fake is not None
            self.assertIn("enable", fake.calls)
            self.assertIn("servo_j", fake.calls)
            self.assertNotIn("move_j", fake.calls)
            self.assertTrue(fake.allow_motion)
        finally:
            core.stop()

    def test_cartesian_from_home_then_joint_home(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
        )
        core.start()
        try:
            self.assertIsNone(core.on_home("a", 80.0))
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                snap = core.snapshot()["arm"]
                q = snap["q_deg"]
                if max(abs(x) for x in q[:3]) < 2.0 and not snap.get("moving"):
                    break
                time.sleep(0.04)
            x0 = core.snapshot()["arm"]["ee_m"][0]
            for _ in range(8):
                err = core.on_cartesian("a", [0.008, 0.0, 0.0], [0.0, 0.0, 0.0])
                self.assertIsNone(err, err)
                time.sleep(0.04)
            x1 = core.snapshot()["arm"]["ee_m"][0]
            self.assertGreater(x1, x0 + 0.008)
            self.assertIsNone(core.on_home("a", 80.0))
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                q = core.snapshot()["arm"]["q_deg"]
                if abs(q[1]) < 8.0:
                    break
                time.sleep(0.04)
            self.assertLess(abs(core.snapshot()["arm"]["q_deg"][0]), 8.0)
        finally:
            core.stop()

    def _home_path_z_stats(self, start: list[float], wps: list[list[float]]) -> tuple[float, float]:
        chain = [start] + list(wps)
        zmin, zmax = 9.0, -9.0
        for a, b in zip(chain, chain[1:]):
            for k in range(9):
                t = k / 8.0
                q = [a[i] + (b[i] - a[i]) * t for i in range(6)]
                xyz, _rot, _rpy = forward_pose([math.radians(x) for x in q])
                zmin = min(zmin, xyz[2])
                zmax = max(zmax, xyz[2])
        return zmin, zmax

    def test_home_waypoints_tuck_before_shoulder_drop(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        start = [0.0, 100.0, 10.0, 30.0, 10.0, 5.0]
        wps = arm._home_waypoints_deg(start)
        self.assertGreaterEqual(len(wps), 2)
        self.assertLessEqual(len(wps), 3)
        self.assertGreater(wps[0][1], 85.0)
        self.assertGreater(wps[0][2], start[2] + 5.0)
        self.assertLess(wps[0][2], 75.0)
        self.assertLess(max(abs(x) for x in wps[-1]), 0.5)
        self.assertFalse(any(p[1] < 20.0 and p[2] > 70.0 for p in wps[:-1]))
        zmin, zmax = self._home_path_z_stats(start, wps)
        self.assertGreaterEqual(zmin, -0.09)
        peak_i = max(range(len(wps)), key=lambda i: wps[i][2])
        z_after, z_peak = self._home_path_z_stats(wps[peak_i], wps[peak_i:])
        self.assertGreaterEqual(z_after, 0.13)
        self.assertLess(zmax, 0.40)
        self.assertLess(z_peak, 0.40)

    def test_home_waypoints_low_stretch_folds_elbow(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        start = [0.0, 20.0, 10.0, 0.0, 0.0, 0.0]
        wps = arm._home_waypoints_deg(start)
        self.assertLess(max(p[2] for p in wps), 55.0)
        self.assertLess(max(abs(x) for x in wps[-1]), 0.5)
        _zmin, zmax = self._home_path_z_stats(start, wps)
        self.assertLess(zmax, 0.28)

    def test_home_waypoints_follow_start_pose(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        compact = arm._home_waypoints_deg([0.0, 40.0, 40.0, 0.0, 0.0, 0.0])
        stretched = arm._home_waypoints_deg([0.0, 100.0, 10.0, 30.0, 0.0, 0.0])
        self.assertLessEqual(max(p[2] for p in compact), 42.0)
        self.assertGreater(max(p[2] for p in stretched), 40.0)
        self.assertNotEqual(
            [round(x, 1) for x in compact[0]],
            [round(x, 1) for x in stretched[0]],
        )

    def test_estop_holds_without_sdk_free_spin(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            arm.emergency_stop()
            self.assertNotIn("emergency_stop", fake.calls)
            self.assertNotIn("move_j", fake.calls)
            self.assertIn("hold_position", fake.calls)
            self.assertTrue(fake.is_enabled)
            self.assertTrue(all(abs(v) < 1e-9 for v in fake.v_rad_s))
        finally:
            arm.stop()

    def test_end_servo_latches_zero_velocity(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            fake.servo_start()
            arm._servoing = True
            arm._servo_cmd_rad = list(fake.q_rad)
            self.assertTrue(arm._end_servo())
            self.assertIn("servo_j", fake.calls)
            self.assertIn("servo_end", fake.calls)
            self.assertFalse(arm._servoing)
            self.assertTrue(all(abs(v) < 1e-9 for v in fake.v_rad_s))
        finally:
            arm.stop()

    def test_stream_idle_parks_without_touch(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            self.assertIsNone(arm.apply_targets([12.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True))
            arm.servo(0.01)
            self.assertTrue(arm._servoing or arm._servo_intent)
            arm._stream_rx_mono = time.monotonic() - 1.0
            arm.servo(0.01)
            self.assertTrue(arm._parked)
            self.assertTrue(arm._servo_intent)
            self.assertTrue(arm._servoing)
            self.assertNotIn("servo_end", fake.calls)
        finally:
            arm.stop()

    def test_live_send_position_uses_servo_path(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            before = fake.calls.count("move_j")
            self.assertIsNone(arm.apply_targets([18.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=False))
            self.assertEqual(fake.calls.count("move_j"), before)
            self.assertTrue(arm._home_path)
            self.assertTrue(arm._servo_intent)
        finally:
            arm.stop()

    def test_lowering_shoulder_tucks_before_drop(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        wps = arm._shoulder_safe_waypoints(
            [0.0, 100.0, 10.0, 0.0, 0.0, 0.0],
            [0.0, 20.0, 20.0, 0.0, 0.0, 0.0],
        )
        self.assertGreater(len(wps), 1)
        self.assertLessEqual(len(wps), 3)
        self.assertGreater(wps[0][1], 85.0)
        self.assertGreater(wps[0][2], 10.0)
        self.assertLess(wps[0][2], 80.0)
        self.assertLess(wps[-1][1], 25.0)

    def test_home_path_does_not_dwell_at_waypoints(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        start = [0.0, 100.0, 10.0, 30.0, 10.0, 5.0]
        wps = arm._home_waypoints_deg(start)
        arm._teleop_deg_s = 40.0
        arm._servo_dt = 0.01
        arm._servo_cmd_rad = arm._q_rad(start)
        arm._home_path = [arm._q_rad(p) for p in wps]
        step_deg = 40.0 * 0.01
        steps: list[float] = []
        while arm._home_path and len(steps) < 4000:
            prev = list(arm._servo_cmd_rad)
            arm._advance_home()
            steps.append(math.degrees(arm._max_abs_delta(arm._servo_cmd_rad, prev)))
        self.assertTrue(arm._parked)
        self.assertGreater(len(steps), 80)
        self.assertLess(max(steps), step_deg * 1.35)
        interior = steps[:-1]
        self.assertTrue(interior)
        self.assertGreater(min(interior), step_deg * 0.55)

    def test_home_path_keeps_joints_synchronized(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        start = [0.0, 80.0, 40.0, 20.0, 0.0, 0.0]
        arm._teleop_deg_s = 40.0
        arm._servo_dt = 0.01
        arm._servo_cmd_rad = arm._q_rad(start)
        arm._home_path = [arm._q_rad([0.0] * 6)]
        seen = False
        for _ in range(800):
            arm._advance_home()
            q = [math.degrees(x) for x in arm._servo_cmd_rad]
            if 15.0 < q[1] < 65.0:
                seen = True
                self.assertAlmostEqual(q[2] / q[1], 0.5, delta=0.08)
                self.assertAlmostEqual(q[3] / q[1], 0.25, delta=0.08)
            if not arm._home_path:
                break
        self.assertTrue(seen)
        self.assertTrue(arm._parked)

    def test_park_does_not_abort_home_path(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            fake.sim_teach = False
            arm._home_path = [arm._q_rad([0.0, 40.0, 40.0, 0.0, 0.0, 0.0])]
            arm._servo_intent = True
            arm.park_stream()
            self.assertTrue(arm._home_path)
            self.assertTrue(arm._servo_intent)
        finally:
            arm.stop()

    def test_parked_ignores_follow_at_current_pose(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            arm._parked = True
            q = [math.degrees(x) for x in fake.q_rad]
            self.assertIsNone(arm.apply_targets(q, 40.0, stream=True))
            self.assertFalse(arm._servo_intent)
            self.assertTrue(arm._parked)
        finally:
            arm.stop()

    def test_parked_follow_unparks_on_small_slider_move(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            arm._parked = True
            q = [math.degrees(x) for x in fake.q_rad]
            q[0] += 1.0
            self.assertIsNone(arm.apply_targets(q, 40.0, stream=True))
            self.assertFalse(arm._parked)
            self.assertTrue(arm._servo_intent)
        finally:
            arm.stop()

    def test_hold_does_not_drop_home_path(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            goal = arm._q_rad([0.0, 40.0, 40.0, 0.0, 0.0, 0.0])
            arm._home_path = [goal]
            arm._servo_intent = True
            arm.hold()
            self.assertEqual(arm._home_path, [goal])
            arm.hold(abort_path=True)
            self.assertFalse(arm._home_path)
        finally:
            arm.stop()

    def test_live_home_keeps_servo_session(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            fake.q_rad = arm._q_rad([20.0, 80.0, 40.0, 10.0, 0.0, 0.0])
            arm._servo_cmd_rad = list(fake.q_rad)
            arm._stream_q_rad = list(fake.q_rad)
            n_end = fake.calls.count("servo_end")
            self.assertIsNone(arm.home(40.0))
            self.assertEqual(fake.calls.count("servo_end"), n_end)
            self.assertTrue(arm._home_path)
            self.assertTrue(arm._servo_intent)
            for _ in range(8):
                arm.servo(0.01)
            self.assertTrue(arm._home_path or arm._parked)
            self.assertGreater(fake.calls.count("servo_j"), 0)
            self.assertNotIn("go_home", fake.calls)
        finally:
            arm.stop()

    def test_cartesian_x_moves_ee(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
        )
        core.start()
        try:
            core.on_arm_targets("a", [0.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80.0)
            time.sleep(0.25)
            x0 = core.snapshot()["arm"]["ee_m"][0]
            err = core.on_cartesian("a", [0.03, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.assertIsNone(err, err)
            deadline = time.monotonic() + 1.5
            x1 = x0
            while time.monotonic() < deadline:
                x1 = core.snapshot()["arm"]["ee_m"][0]
                if x1 > x0 + 0.008:
                    break
                time.sleep(0.04)
            self.assertGreater(x1, x0 + 0.008)
            fake = FakeFafuController.last
            assert fake is not None
            self.assertIn("servo_j", fake.calls)
        finally:
            core.stop()

    def test_cartesian_pose_uses_servo_path(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
        )
        core.start()
        try:
            self.assertIsNone(core.on_arm_targets("a", [0.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80.0))
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                q = core.snapshot()["arm"]["q_deg"]
                if abs(q[1] - 40.0) < 2.0:
                    break
                time.sleep(0.04)
            snap = core.snapshot()["arm"]
            xyz = list(snap["ee_m"])
            rpy = list(snap["ee_rpy_deg"])
            x0 = xyz[0]
            xyz[0] = x0 + 0.03
            fake = FakeFafuController.last
            assert fake is not None
            n_move = fake.calls.count("move_j")
            err = core.on_cartesian_pose("a", xyz, rpy, 40.0)
            self.assertIsNone(err, err)
            self.assertEqual(fake.calls.count("move_j"), n_move)
            ack = core.handle(
                {"t": "cart_go", "cid": "a", "xyz": xyz, "rpy": rpy, "speed": 40.0}
            )
            self.assertTrue(ack.get("ok"), ack)
            deadline = time.monotonic() + 3.5
            got = list(snap["ee_m"])
            while time.monotonic() < deadline:
                got = core.snapshot()["arm"]["ee_m"]
                if (
                    abs(got[0] - xyz[0]) < 0.006
                    and abs(got[1] - xyz[1]) < 0.006
                    and abs(got[2] - xyz[2]) < 0.006
                ):
                    break
                time.sleep(0.04)
            self.assertIn("servo_j", fake.calls)
            self.assertLess(abs(got[0] - xyz[0]), 0.008)
            self.assertLess(abs(got[1] - xyz[1]), 0.008)
            self.assertLess(abs(got[2] - xyz[2]), 0.008)
        finally:
            core.stop()

    def test_cartesian_without_pinocchio_uses_station_ik(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            arm._dyn_ready = False

            def boom(*_a: object, **_k: object) -> None:
                raise RuntimeError("dynamics model not loaded")

            fake.get_pose = boom  # type: ignore[method-assign]
            fake.inverse_kinematics = boom  # type: ignore[method-assign]
            fake.forward_kinematics = boom  # type: ignore[method-assign]
            self.assertIsNone(arm._ensure_enabled())
            err = arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.assertIsNone(err, err)
            self.assertTrue(arm._servo_intent)
            self.assertIsNotNone(arm._stream_q_rad)
            pos, _ori, rpy = forward_pose(list(arm._stream_q_rad))
            goal = [pos[0] + 0.02, pos[1], pos[2]]
            err = arm.apply_cartesian_pose(
                goal, [math.degrees(x) for x in rpy], 30.0
            )
            self.assertIsNone(err, err)
            self.assertTrue(arm._home_path or arm._servo_intent)
        finally:
            arm.stop()

    def test_ensure_enabled_surfaces_sdk_error(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None

            def boom(*_a: object, **_k: object) -> None:
                raise RuntimeError("usb busy")

            fake.is_enabled = False
            fake.enable = boom  # type: ignore[method-assign]
            err = arm._ensure_enabled()
            self.assertIsNotNone(err)
            self.assertIn("使能失败", err or "")
        finally:
            arm.stop()

    def test_live_servo_caps_match_ui_speed(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        vel, step = arm._live_servo_caps()
        self.assertLess(vel, 3.0)
        self.assertGreater(vel, 2.0)
        self.assertLess(step, 0.04)
        self.assertGreater(step, 0.02)

    def test_live_like_stream_obeys_ui_speed(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            err = arm.apply_targets([90.0, 0.0, 0.0, 0.0, 0.0, 0.0], 40.0, stream=True)
            self.assertIsNone(err, err)
            arm.servo(0.01)
            moved = abs(math.degrees(fake.q_rad[0]))
            self.assertGreater(moved, 0.2)
            self.assertLess(moved, 1.2)
        finally:
            arm.stop()

    def test_path_and_estop(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
        )
        core.start()
        try:
            start = [0.0, 40.0, 40.0, 0.0, 0.0, 0.0]
            goal = [18.0, 40.0, 40.0, 0.0, 0.0, 0.0]
            self.assertIsNone(core.on_path("a", [start, goal], 80.0))
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if core.snapshot()["arm"]["q_deg"][0] > 8.0:
                    break
                time.sleep(0.04)
            self.assertGreater(core.snapshot()["arm"]["q_deg"][0], 8.0)
            core.on_estop("sim")
            held = core.snapshot()["arm"]["q_deg"][0]
            time.sleep(0.12)
            self.assertAlmostEqual(core.snapshot()["arm"]["q_deg"][0], held, places=1)
            fake = FakeFafuController.last
            assert fake is not None
            self.assertNotIn("emergency_stop", fake.calls)
            self.assertIn("move_jntspace_path", fake.calls)
            self.assertIsNotNone(core.on_arm_targets("a", goal, 40.0))
            self.assertIsNone(core.on_home("a", 40.0))
            self.assertIsNone(core.on_clear_estop("a"))
            self.assertIsNone(core.on_arm_targets("a", [5.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0))
        finally:
            core.stop()

    def test_gravity_teach_snaps(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=50, watchdog_s=2.0)
        )
        core.start()
        try:
            self.assertIsNone(core.on_mode("a", "Gravity"))
            self.assertIsNone(core.on_teach("a", [25.0, 40.0, 40.0, 0.0, 0.0, 0.0]))
            time.sleep(0.05)
            self.assertAlmostEqual(core.snapshot()["arm"]["q_deg"][0], 25.0, places=0)
            blocked = core.on_arm_targets("a", [0, 40, 40, 0, 0, 0], 40)
            self.assertIn("Position", blocked or "")
        finally:
            core.stop()

    def test_stream_uses_servo_opts(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=50, watchdog_s=2.0)
        )
        core.start()
        try:
            err = core.on_arm_targets("a", [5.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True)
            self.assertIsNone(err, err)
            fake = FakeFafuController.last
            assert fake is not None
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                if "servo_start" in fake.calls:
                    break
                time.sleep(0.02)
            self.assertIn("servo_start", fake.calls)
            self.assertIsNotNone(fake.servo_opts)
        finally:
            core.stop()

    def test_stream_then_move_j_not_dropped(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
        )
        core.start()
        try:
            self.assertIsNone(core.on_arm_targets("a", [5.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True))
            time.sleep(0.12)
            err = core.on_arm_targets("a", [22.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80.0)
            self.assertIsNone(err, err)
            deadline = time.monotonic() + 1.5
            q0 = 0.0
            while time.monotonic() < deadline:
                q0 = core.snapshot()["arm"]["q_deg"][0]
                if q0 > 12.0:
                    break
                time.sleep(0.04)
            self.assertGreater(q0, 12.0)
        finally:
            core.stop()

    def test_float_modes_follow_dynamics(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            snap = arm.poll()
            self.assertTrue(snap.float_ok)
            self.assertEqual(snap.float_reason, "")
            fake.sim_teach = False
            arm._dyn_ready = False
            snap = arm.poll()
            self.assertFalse(snap.float_ok)
            self.assertIn("pinocchio", snap.float_reason)
            self.assertIn("pinocchio", arm.set_ctrl_mode("Gravity") or "")
            self.assertEqual(arm._ctrl_mode, "Position")
            arm._dyn_ready = True
            snap = arm.poll()
            self.assertTrue(snap.float_ok)
        finally:
            arm.stop()

    def test_sdk_gravity_loop_then_position(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            arm._dyn_ready = True
            self.assertIsNone(arm.set_ctrl_mode("Gravity"))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if "start_gravity_compensation" in fake.calls:
                    break
                time.sleep(0.02)
            self.assertIn("start_gravity_compensation", fake.calls)
            self.assertTrue(arm.poll().grav_active)
            blocked = arm.set_gripper_open(False)
            self.assertIsNotNone(blocked)
            self.assertIn("Position", blocked or "")
            self.assertIsNone(arm.set_ctrl_mode("Position"))
            self.assertEqual(arm._ctrl_mode, "Position")
        finally:
            arm.stop()
        fake = FakeFafuController.last
        assert fake is not None
        self.assertTrue(fake.closed)

    def test_arm_src_sim_refuses_live_without_serial_flag(self) -> None:
        os.environ.pop("STATION_ALLOW_LIVE_ARM", None)
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=50, watchdog_s=2.0)
        )
        core.start()
        try:
            self.assertEqual(core.arm_kind(), "sim")
            self.assertIsNone(core.on_arm_src("a", "sim"))
            err = core.on_arm_src("a", "fafu")
            self.assertIsNotNone(err)
            self.assertIn("真机", err or "")
            self.assertEqual(core.arm_kind(), "sim")
        finally:
            core.stop()

    def test_live_boot_auto_enables_after_connect(self) -> None:
        os.environ["STATION_ALLOW_LIVE_ARM"] = "1"
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.backend = "fafu"
        core = MotionCore(
            StationConfig(arm="fafu", arm_allow_motion=True, open_browser=False, control_hz=40)
        )
        core.arm = arm
        try:
            core.start()
            fake = FakeFafuController.last
            assert fake is not None
            self.assertTrue(fake.is_enabled)
            self.assertIn("enable", fake.calls)
        finally:
            core.stop()
            os.environ.pop("STATION_ALLOW_LIVE_ARM", None)

    def test_live_boot_skips_enable_when_readonly(self) -> None:
        os.environ["STATION_ALLOW_LIVE_ARM"] = "1"
        arm = FafuArm(allow_motion=False, controller_cls=FakeFafuController, required=True)
        self.assertEqual(arm.backend, "fafu")
        core = MotionCore(
            StationConfig(arm="fafu", arm_allow_motion=False, open_browser=False, control_hz=40)
        )
        core.arm = arm
        try:
            core.start()
            fake = FakeFafuController.last
            assert fake is not None
            self.assertFalse(fake.is_enabled)
            self.assertNotIn("enable", fake.calls)
        finally:
            core.stop()
            os.environ.pop("STATION_ALLOW_LIVE_ARM", None)

    def test_arm_src_switch_to_fafu_does_not_auto_enable(self) -> None:
        os.environ["STATION_ALLOW_LIVE_ARM"] = "1"
        core = MotionCore(
            StationConfig(arm="sim", arm_allow_motion=True, open_browser=False, control_hz=40)
        )
        orig = build_arm

        def fake_build(name, *args, **kwargs):
            if name == "fafu":
                nxt = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
                nxt.backend = "fafu"
                return nxt
            return orig(name, *args, **kwargs)

        core.start()
        try:
            with patch("robot_station.motion.build_arm", side_effect=fake_build):
                err = core.on_arm_src("a", "fafu")
            self.assertIsNone(err)
            self.assertEqual(core.arm_kind(), "fafu")
            fake = FakeFafuController.last
            assert fake is not None
            self.assertFalse(fake.is_enabled)
            self.assertNotIn("enable", fake.calls)
        finally:
            core.stop()
            os.environ.pop("STATION_ALLOW_LIVE_ARM", None)

    def test_fafu_start_failure_falls_back_to_sim(self) -> None:
        os.environ["STATION_ALLOW_LIVE_ARM"] = "1"
        try:
            core = MotionCore(
                StationConfig(arm="fafu", open_browser=False, control_hz=40, watchdog_s=2.0)
            )
            self.assertEqual(core.arm.backend, "fafu")

            def boom() -> None:
                core.arm._robot = None
                core.arm._link_error = "motors did not respond within 500ms"

            core.arm.start = boom  # type: ignore[method-assign]
            core.start()
            try:
                self.assertEqual(core.arm_kind(), "sim")
                self.assertIn("仿真", core._cmd_err)
                self.assertIn("motors", core._cmd_err)
            finally:
                core.stop()
        finally:
            os.environ.pop("STATION_ALLOW_LIVE_ARM", None)

    def test_park_keeps_servo_hold_stream(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            self.assertIsNone(arm.apply_targets([12.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True))
            arm.servo(0.01)
            n_end = fake.calls.count("servo_end")
            arm.park_stream()
            arm.servo(0.01)
            self.assertTrue(arm._parked)
            self.assertTrue(arm._servo_intent)
            self.assertTrue(arm._servoing)
            self.assertEqual(fake.calls.count("servo_end"), n_end)
        finally:
            arm.stop()

    def test_keyboard_after_park_skips_enable_while_servoing(self) -> None:
        """Live SDK rejects enable() in SERVOING; idle WASD must still servo_j."""
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            self.assertIsNone(arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0]))
            arm.servo(0.01)
            self.assertTrue(arm._servoing)
            arm.park_stream()
            arm._stream_rx_mono = time.monotonic() - 1.0
            arm.servo(0.01)
            self.assertTrue(arm._parked)
            self.assertTrue(arm._servoing)
            n_enable = {"n": 0}

            def boom(*_a: object, **_k: object) -> None:
                n_enable["n"] += 1
                raise RuntimeError("enable 被拒绝: 机械臂正忙 (state=servoing)。")

            fake.is_enabled = False
            fake.enable = boom  # type: ignore[method-assign]
            self.assertIsNone(arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0]))
            n_j = fake.calls.count("servo_j")
            arm.servo(0.01)
            self.assertEqual(n_enable["n"], 0)
            self.assertGreater(fake.calls.count("servo_j"), n_j)
            self.assertFalse(arm._parked)
            self.assertTrue(arm._servo_intent)
            self.assertTrue(arm._servoing)
        finally:
            arm.stop()

    def test_keyboard_preempts_stuck_home_path(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            arm._home_path = [arm._q_rad([0.0, 40.0, 40.0, 0.0, 0.0, 0.0])]
            arm._parked = True
            err = arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.assertIsNone(err, err)
            self.assertFalse(arm._home_path)
            self.assertFalse(arm._parked)
            self.assertTrue(arm._servo_intent)
        finally:
            arm.stop()

    def test_enable_reports_motor_online(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=50, watchdog_s=2.0)
        )
        core.start()
        try:
            self.assertIsNone(core.on_power("a", True))
            deadline = time.monotonic() + 1.0
            motors: list = []
            while time.monotonic() < deadline:
                snap = core.snapshot()["arm"]
                motors = list(snap.get("motors") or [])
                if snap["enabled"] and len(motors) >= 6 and all(m.get("online") for m in motors[:6]):
                    break
                time.sleep(0.04)
            self.assertTrue(core.snapshot()["arm"]["enabled"])
            self.assertGreaterEqual(len(motors), 6)
            self.assertTrue(all(m.get("online") for m in motors[:6]))
            self.assertTrue(all(m.get("mode") == "position" for m in motors[:6]))
        finally:
            core.stop()

    def test_script_map(self) -> None:
        from robot_station.adapters.fafu_arm import map_script

        self.assertEqual(map_script("go_home.py"), "home")
        self.assertEqual(map_script("03_gripper.py"), "grip_open")
        self.assertEqual(map_script("06_emergency_stop.py"), "estop")
        self.assertIsNone(map_script("visible_motion.py"))
        self.assertIsNone(map_script("04_enable_disable.py"))
        self.assertIsNone(map_script("02_move_joint.py"))

    def test_gripper_open_close_uses_effort_cap(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            self.assertEqual(arm.poll().gripper_effort, 300)
            self.assertIsNone(arm.set_gripper_effort(95))
            self.assertEqual(arm.poll().gripper_effort, 95)
            self.assertIsNone(arm.set_gripper_open(False))
            arm.servo(0.01)
            self.assertEqual(fake.last_gripper_effort, 95)
            self.assertIn("close_gripper", fake.calls)
            hi = float(fake.gripper_limits_deg[1])
            self.assertGreater(fake.gripper_deg, hi - 5.0)
            self.assertLess(fake.gripper_deg, hi)
            self.assertIsNone(arm.set_gripper_open(True))
            arm.servo(0.01)
            self.assertEqual(fake.last_gripper_effort, 95)
            self.assertIn("open_gripper", fake.calls)
            self.assertIsNone(arm.set_gripper_deg(40.0))
            arm.servo(0.01)
            self.assertEqual(fake.last_gripper_effort, 95)
            self.assertIn("gripper_control", fake.calls)
            self.assertIsNone(arm.set_gripper_open(False))
            arm.servo(0.01)
            self.assertFalse(arm.poll().gripper_open)
            fake.sim_teach = False
            fake.gripper_deg = 0.0
            fake.gripper_open = True
            snap = arm.poll()
            self.assertFalse(snap.gripper_open)
            self.assertLess(snap.gripper_deg, 8.0)
        finally:
            arm.stop()

    def test_grip_effort_only_does_not_open(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=50, watchdog_s=2.0)
        )
        core.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None

            def grip_calls() -> list[str]:
                return [n for n in fake.calls if n in ("open_gripper", "close_gripper", "gripper_control")]

            self.assertEqual(grip_calls(), [])
            err = core.on_gripper("a", None, None, 120)
            self.assertIsNone(err)
            self.assertEqual(getattr(core.arm, "_gripper_effort"), 120)
            self.assertEqual(grip_calls(), [])
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if core.snapshot()["arm"].get("gripper_effort") == 120:
                    break
                time.sleep(0.02)
            self.assertEqual(core.snapshot()["arm"]["gripper_effort"], 120)
            self.assertIsNone(core.on_gripper("a", False))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if fake.last_gripper_effort == 120:
                    break
                time.sleep(0.02)
            self.assertEqual(fake.last_gripper_effort, 120)
            self.assertIn("close_gripper", fake.calls)
        finally:
            core.stop()

    def test_yaml_still_mock_and_fafu_motion_blocked(self) -> None:
        from robot_station.config import load_config

        cfg = load_config()
        self.assertEqual(cfg.arm, "mock")
        self.assertFalse(cfg.arm_allow_motion)
        os.environ.pop("STATION_ALLOW_LIVE_ARM", None)
        arm = build_arm("fafu", 6, 40.0, True, allow_motion=True)
        self.assertFalse(arm.allow_motion)


class SdkContractHelpersTests(unittest.TestCase):
    def test_motor_position_turns_to_deg(self) -> None:
        from robot_station.adapters.fafu_arm import motor_position_to_deg

        self.assertAlmostEqual(motor_position_to_deg(75.0 / 360.0), 75.0, places=5)

    def test_rotation_matrix_to_rpy(self) -> None:
        from robot_station.adapters.fafu_arm import rotation_to_rpy

        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.assertEqual(rotation_to_rpy(identity), [0.0, 0.0, 0.0])
        yaw90 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        rpy = rotation_to_rpy(yaw90)
        assert rpy is not None
        self.assertAlmostEqual(rpy[2], math.pi / 2.0, places=5)


if __name__ == "__main__":
    unittest.main()
