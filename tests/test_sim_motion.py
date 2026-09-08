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

    def test_ensure_enabled_ignores_vendor_enable_error(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None

            def boom(*_a: object, **_k: object) -> None:
                raise RuntimeError(
                    "enable failed even after motor_reset; check the diagnostic above. "
                    "Likely causes: (a) motor controller in latched FAULT state -> hard power-cycle; "
                    "(b) USB-CAN bus disconnected / wrong COM port; "
                    "(c) mechanical jam holding the joint outside soft limits."
                )

            fake.is_enabled = False
            fake.enable = boom  # type: ignore[method-assign]
            err = arm._ensure_enabled()
            self.assertIsNone(err, err)
            self.assertTrue(fake.is_enabled)
        finally:
            arm.stop()

    def test_ensure_enabled_surfaces_present_motor_error(self) -> None:
        from unittest.mock import patch

        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.is_enabled = False

            def boom(*_a: object, **_k: object) -> None:
                raise RuntimeError("usb busy")

            with patch("robot_station.adapters.fafu_arm._enable_present_motors", side_effect=boom):
                err = arm._ensure_enabled()
            self.assertIsNotNone(err)
            self.assertIn("使能失败", err or "")
            self.assertIn("usb busy", err or "")
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
            self.assertTrue(snap.float_ok)
            self.assertEqual(snap.float_reason, "")
            self.assertIsNone(arm.set_ctrl_mode("Gravity"))
            self.assertEqual(arm._ctrl_mode, "Gravity")
            self.assertIsNone(arm.set_ctrl_mode("Position"))
            send = fake.apply_compensation_torque
            fake.apply_compensation_torque = None  # type: ignore[method-assign]
            snap = arm.poll()
            self.assertFalse(snap.float_ok)
            self.assertIn("力矩环", snap.float_reason)
            self.assertIsNotNone(arm.set_ctrl_mode("Gravity"))
            self.assertEqual(arm._ctrl_mode, "Position")
            fake.apply_compensation_torque = send
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

    def test_station_gravity_loop_without_pinocchio(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            arm._dyn_ready = False
            self.assertIsNone(arm.set_ctrl_mode("Gravity"))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if "apply_compensation_torque" in fake.calls:
                    break
                time.sleep(0.02)
            self.assertIn("apply_compensation_torque", fake.calls)
            self.assertNotIn("start_gravity_compensation", fake.calls)
            self.assertTrue(arm.poll().grav_active)
            self.assertGreater(max(abs(x) for x in fake.last_tau_nm), 0.05)
            blocked = arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.assertIsNotNone(blocked)
            self.assertIn("Position", blocked or "")
            self.assertIsNone(arm.set_ctrl_mode("Position"))
            self.assertEqual(arm._ctrl_mode, "Position")
            err = arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.assertIsNone(err, err)
            self.assertTrue(arm._servo_intent)
        finally:
            arm.stop()

    def test_gravity_refused_when_arm_joint_offline(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            fake._missing_motors = [3]
            arm._dyn_ready = False
            err = arm.set_ctrl_mode("Gravity")
            self.assertIsNotNone(err)
            self.assertIn("J1", err or "")
            self.assertEqual(arm._ctrl_mode, "Position")
            self.assertFalse(arm.poll().float_ok)
        finally:
            arm.stop()

    def test_station_gravity_ok_without_gripper(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake._missing_motors = [7]
            fake.enable()
            arm._dyn_ready = False
            self.assertIsNone(arm.set_ctrl_mode("Gravity"))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if "apply_compensation_torque" in fake.calls:
                    break
                time.sleep(0.02)
            self.assertIn("apply_compensation_torque", fake.calls)
            self.assertEqual(len(fake.last_tau_nm), 6)
            self.assertGreater(max(abs(x) for x in fake.last_tau_nm), 0.05)
            self.assertIsNone(arm.set_ctrl_mode("Position"))
        finally:
            arm.stop()

    def test_dead_gravity_thread_holds_position_not_servo_j(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.sim_teach = False
            fake.enable()
            arm._dyn_ready = False
            n = {"n": 0}

            def boom(tau, **_kwargs):
                n["n"] += 1
                fake.calls.append("apply_compensation_torque")
                if n["n"] > 40:
                    raise ValueError("tau must have 5 elements, got (6,)")
                fake.last_tau_nm = [float(x) for x in tau][:6]

            fake.apply_compensation_torque = boom  # type: ignore[method-assign]
            self.assertIsNone(arm.set_ctrl_mode("Gravity"))
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                th = arm._grav_th
                if th is None or not th.is_alive():
                    break
                time.sleep(0.02)
            self.assertTrue(arm._grav_err)
            before = list(fake.calls)
            for _ in range(10):
                arm.servo(0.01)
                time.sleep(0.005)
            self.assertEqual(arm._ctrl_mode, "Position")
            self.assertFalse(arm._float_writer_active())
            self.assertNotIn("servo_j", fake.calls[len(before) :])
            self.assertIsNone(arm.apply_targets([12.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True))
        finally:
            arm.stop()

    def test_tau_maps_onto_live_joint_ids(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.joint_motor_ids = [1, 2, 4, 5, 6]
            fake._joint_motor_ids = [1, 2, 4, 5, 6]
            fake.num_joints = 5
            tau = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
            mapped = arm._vec_to_sdk(tau, fake)
            self.assertEqual(mapped, [0.1, 0.2, 0.4, 0.5, 0.6])
        finally:
            arm.stop()

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


class TrajMotionTests(unittest.TestCase):
    """Continuous record / timestamp playback. Waypoint panel path is unchanged."""

    def test_record_position_then_timestamp_replay(self) -> None:
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir() as tmp:
            core = MotionCore(
                StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
            )
            core.start()
            try:
                self.assertIsNone(core.on_rec_start("a"))
                deadline = time.monotonic() + 0.8
                while time.monotonic() < deadline:
                    if core.snapshot()["arm"]["recording"]:
                        break
                    time.sleep(0.02)
                self.assertTrue(core.snapshot()["arm"]["recording"])
                err = core.on_arm_targets("a", [14.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80.0, stream=True)
                self.assertIsNone(err, err)
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    if core.snapshot()["arm"]["rec_frames"] >= 12:
                        break
                    time.sleep(0.03)
                self.assertGreaterEqual(core.snapshot()["arm"]["rec_frames"], 12)
                self.assertIsNone(core.on_rec_stop("a"))
                deadline = time.monotonic() + 0.8
                snap = core.snapshot()["arm"]
                while time.monotonic() < deadline:
                    snap = core.snapshot()["arm"]
                    if not snap["recording"] and snap.get("traj_file"):
                        break
                    time.sleep(0.02)
                self.assertFalse(snap["recording"])
                name = snap["traj_file"]
                self.assertTrue(name)
                self.assertTrue((tmp / name).is_file())
                self.assertIsNone(core.on_replay("a", name, 1.0, 80.0))
                deadline = time.monotonic() + 3.0
                saw_replay = False
                while time.monotonic() < deadline:
                    arm = core.snapshot()["arm"]
                    if arm.get("replay_active"):
                        saw_replay = True
                    if saw_replay and not arm.get("replay_active") and arm["q_deg"][0] > 6.0:
                        break
                    time.sleep(0.04)
                arm = core.snapshot()["arm"]
                self.assertGreater(arm["q_deg"][0], 6.0)
                fake = FakeFafuController.last
                assert fake is not None
                self.assertIn("servo_j", fake.calls)
                self.assertNotIn("move_MIT", fake.calls)
            finally:
                core.stop()

    def test_record_gravity_teach_and_path_still_refused(self) -> None:
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir():
            core = MotionCore(
                StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
            )
            core.start()
            try:
                self.assertIsNone(core.on_mode("a", "Gravity"))
                self.assertIsNone(core.on_rec_start("a"))
                self.assertIsNone(core.on_teach("a", [22.0, 40.0, 40.0, 0.0, 0.0, 0.0]))
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    if core.snapshot()["arm"]["rec_frames"] >= 8:
                        break
                    time.sleep(0.03)
                self.assertGreaterEqual(core.snapshot()["arm"]["rec_frames"], 8)
                self.assertEqual(core.snapshot()["arm"]["rec_teach"], "drag")
                blocked = core.on_path("a", [[0.0, 40.0, 40.0, 0.0, 0.0, 0.0]], 80.0)
                self.assertIsNotNone(blocked)
                self.assertIn("Position", blocked or "")
                self.assertIsNone(core.on_rec_stop("a"))
            finally:
                core.stop()

    def test_station_gravity_records_without_pinocchio(self) -> None:
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir() as tmp:
            arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
            arm.start()
            try:
                fake = FakeFafuController.last
                assert fake is not None
                fake.sim_teach = False
                fake.enable()
                arm._dyn_ready = False
                self.assertIsNone(arm.set_ctrl_mode("Gravity"))
                self.assertIsNone(arm.record_start())
                for _ in range(30):
                    arm.servo(0.01)
                    time.sleep(0.005)
                self.assertGreater(arm.poll().rec_frames, 8)
                self.assertIsNone(arm.record_stop())
                self.assertTrue(list(tmp.glob("*.jsonl")))
                self.assertIn("apply_compensation_torque", fake.calls)
                self.assertNotIn("start_gravity_compensation", fake.calls)
                blocked = arm.apply_cartesian([0.01, 0.0, 0.0], [0.0, 0.0, 0.0])
                self.assertIsNotNone(blocked)
                self.assertIn("Position", blocked or "")
            finally:
                arm.stop()

    def test_estop_stops_recording_and_keeps_file(self) -> None:
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir() as tmp:
            core = MotionCore(
                StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
            )
            core.start()
            try:
                self.assertIsNone(core.on_rec_start("a", "keepme"))
                core.on_arm_targets("a", [8.0, 40.0, 40.0, 0.0, 0.0, 0.0], 80.0, stream=True)
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    if core.snapshot()["arm"]["rec_frames"] >= 6:
                        break
                    time.sleep(0.03)
                self.assertGreaterEqual(core.snapshot()["arm"]["rec_frames"], 6)
                core.on_estop("sim")
                deadline = time.monotonic() + 0.8
                while time.monotonic() < deadline:
                    if not core.snapshot()["arm"]["recording"]:
                        break
                    time.sleep(0.02)
                self.assertFalse(core.snapshot()["arm"]["recording"])
                files = list(tmp.glob("*.jsonl"))
                self.assertTrue(files)
                from robot_station.traj import load_traj

                _header, frames = load_traj(files[0])
                self.assertGreaterEqual(len(frames), 6)
            finally:
                core.stop()

    def test_replay_keeps_session_when_servo_j_false(self) -> None:
        from robot_station.traj import TrajectoryWriter
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir() as tmp:
            arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
            arm.start()
            try:
                fake = FakeFafuController.last
                assert fake is not None
                fake.enable()
                path = tmp / "replay_false.jsonl"
                writer = TrajectoryWriter(path, teach="soft", mode="Position", n=6)
                for i in range(25):
                    writer.log(
                        [0.15 + 0.01 * i, 0.70, 0.70, 0.0, 0.0, 0.0],
                        [0.0] * 6,
                        0.5,
                        0.0,
                    )
                    time.sleep(0.012)
                writer.close()
                n = {"n": 0}
                orig = fake.servo_j

                def flaky(target_angles):
                    n["n"] += 1
                    if n["n"] == 4:
                        fake._servo_aborted_reason = "gripper hold"
                        return False
                    return orig(target_angles)

                fake.servo_j = flaky  # type: ignore[method-assign]
                self.assertIsNone(arm.replay_start(str(path), 1.0, 80.0))
                for _ in range(50):
                    arm.servo(0.01)
                    time.sleep(0.008)
                self.assertGreaterEqual(n["n"], 5)
                self.assertTrue(fake.is_servoing)
            finally:
                arm.stop()

    def test_replay_offline_gripper_does_not_fault_writer(self) -> None:
        from robot_station.traj import TrajectoryWriter
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir() as tmp:
            arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
            arm.start()
            try:
                fake = FakeFafuController.last
                assert fake is not None
                fake._missing_motors = [7]
                fake.enable()
                path = tmp / "replay_grip.jsonl"
                writer = TrajectoryWriter(path, teach="soft", mode="Position", n=6)
                for i in range(20):
                    writer.log(
                        [0.12 + 0.008 * i, 0.70, 0.70, 0.0, 0.0, 0.0],
                        [0.0] * 6,
                        0.2 if i < 8 else 1.4,
                        0.0,
                    )
                    time.sleep(0.012)
                writer.close()
                self.assertIsNone(arm.replay_start(str(path), 1.0, 80.0))
                for _ in range(40):
                    arm.servo(0.01)
                    time.sleep(0.008)
                err = str(arm._writer_err or "")
                self.assertNotIn("夹爪", err)
                self.assertTrue(fake.is_servoing or not arm._replay_frames)
            finally:
                arm.stop()

    def test_drag_record_enters_gravity(self) -> None:
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir():
            core = MotionCore(
                StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
            )
            core.start()
            try:
                self.assertEqual(core.arm_mode(), "Position")
                self.assertIsNone(core.on_rec_start("a", teach="drag"))
                self.assertIn(core.arm_mode(), ("Gravity", "Gra+Fri"))
                deadline = time.monotonic() + 0.8
                while time.monotonic() < deadline:
                    if core.snapshot()["arm"]["recording"]:
                        break
                    time.sleep(0.02)
                self.assertTrue(core.snapshot()["arm"]["recording"])
                self.assertEqual(core.snapshot()["arm"]["rec_teach"], "drag")
                blocked = core.on_arm_targets("a", [0, 40, 40, 0, 0, 0], 40)
                self.assertIn("Position", blocked or "")
                self.assertIsNone(core.on_rec_stop("a"))
                self.assertEqual(core.arm_mode(), "Position")
            finally:
                core.stop()

    def test_delete_traj_removes_file(self) -> None:
        from robot_station.traj import TrajectoryWriter, list_recordings
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir() as tmp:
            path = tmp / "dropme.jsonl"
            TrajectoryWriter(path, teach="soft", mode="Position", n=6).close()
            core = MotionCore(
                StationConfig(arm="sim", open_browser=False, control_hz=50, watchdog_s=2.0)
            )
            core.start()
            try:
                self.assertTrue(path.is_file())
                self.assertIsNone(core.on_rec_delete("a", path.name))
                self.assertFalse(path.is_file())
                self.assertFalse(any(it["name"] == path.name for it in list_recordings()))
            finally:
                core.stop()

    def test_replay_holds_start_before_timestamp(self) -> None:
        from robot_station.traj import TrajectoryWriter
        from tests.test_traj import recordings_tmpdir

        with recordings_tmpdir() as tmp:
            arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
            arm.start()
            try:
                fake = FakeFafuController.last
                assert fake is not None
                fake.enable()
                path = tmp / "settle.jsonl"
                writer = TrajectoryWriter(path, teach="soft", mode="Position", n=6)
                for i in range(30):
                    writer.log(
                        [0.25 + 0.01 * i, 0.70, 0.70, 0.0, 0.0, 0.0],
                        [0.0] * 6,
                        0.5,
                        0.0,
                    )
                    time.sleep(0.012)
                writer.close()
                self.assertIsNone(arm.replay_start(str(path), 1.0, 80.0))
                self.assertIsNone(arm._replay_t0)
                for _ in range(8):
                    arm.servo(0.01)
                self.assertIsNone(arm._replay_t0)
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    arm.servo(0.01)
                    time.sleep(0.01)
                    if arm._replay_t0 is not None:
                        break
                self.assertIsNotNone(arm._replay_t0)
            finally:
                arm.stop()

    def test_replay_approach_keeps_wrists(self) -> None:
        arm = FafuArm(allow_motion=True, controller_cls=FakeFafuController, required=True)
        now = [10.0, 80.0, 90.0, 0.0, 35.0, 20.0]
        goal = [12.0, 40.0, 40.0, 0.0, 25.0, 15.0]
        wps = arm._replay_approach_waypoints(now, goal)
        for wp in wps[:-1]:
            self.assertGreater(abs(wp[4]), 1.0)
            self.assertGreater(abs(wp[5]), 1.0)
        self.assertAlmostEqual(wps[-1][4], 25.0, places=1)
        self.assertAlmostEqual(wps[-1][5], 15.0, places=1)
        arm.stop()

    def test_waypoint_path_still_uses_existing_channel(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=80, watchdog_s=2.0)
        )
        core.start()
        try:
            start = [0.0, 40.0, 40.0, 0.0, 0.0, 0.0]
            goal = [16.0, 40.0, 40.0, 0.0, 0.0, 0.0]
            self.assertIsNone(core.on_path("a", [start, goal], 80.0))
            fake = FakeFafuController.last
            assert fake is not None
            self.assertIn("move_jntspace_path", fake.calls)
        finally:
            core.stop()


if __name__ == "__main__":
    unittest.main()
