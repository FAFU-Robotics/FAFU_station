from __future__ import annotations

import math
import threading
import unittest
from unittest.mock import patch

from robot_station.adapters.fafu_arm import FafuArm
from robot_station.adapters.fafu_sim import FakeFafuController
from robot_station.config import StationConfig
from robot_station.motion import MotionCore


class PowerControlTests(unittest.TestCase):
    def arm(self, controller=FakeFafuController):
        arm = FafuArm(allow_motion=True, controller_cls=controller)
        arm.start()
        self.addCleanup(arm.stop)
        self.assertIsNone(arm.set_powered(True))
        return arm

    def test_rpc_stages_disable_and_servo_owner_finishes_it(self):
        class ThreadBound(FakeFafuController):
            def servo_start(self, opts=None):
                self.owner = threading.get_ident()
                super().servo_start(opts)

            def servo_end(self, *args, **kwargs):
                if threading.get_ident() != self.owner:
                    raise AssertionError("servo_end must run on its owner thread")
                super().servo_end(*args, **kwargs)

            def disable(self):
                self.disable_thread = threading.get_ident()
                super().disable()

        arm = self.arm(ThreadBound)
        robot = arm._robot
        self.assertIsNone(arm.apply_targets([15,40,40,0,0,0], 20, stream=True))
        arm.servo(0.01)
        self.assertTrue(arm._servoing)
        result = []
        caller = threading.Thread(target=lambda: result.append(arm.set_powered(False)))
        caller.start(); caller.join(timeout=1)
        self.assertFalse(caller.is_alive(), "RPC must not wait while holding the motion lock")
        self.assertEqual(result, [None])
        self.assertNotIn("disable", robot.calls)
        arm.servo(0.01)
        self.assertEqual(robot.disable_thread, robot.owner)
        self.assertFalse(arm.poll().enabled)
        self.assertEqual(arm._writer_err, "")
        self.assertFalse(arm._power_off_requested)

    def test_disable_remains_off_through_hold_resume_and_tick(self):
        arm = self.arm()
        robot = arm._robot
        self.assertIsNone(arm.set_powered(False))
        arm.servo(0.01)
        calls = list(robot.calls)
        for _ in range(100):
            arm.hold(abort_path=True)
            arm.resume_motion()
            arm.servo(0.01)
        self.assertFalse(arm.poll().enabled)
        self.assertEqual(robot.calls.count("enable"), calls.count("enable"))
        self.assertEqual(robot.calls.count("servo_j"), calls.count("servo_j"))
        self.assertIn("去使能", arm.apply_targets([15,40,40,0,0,0], 20))
        self.assertIn("去使能", arm._ensure_enabled())
        self.assertIn("去使能", arm.record_start())

    def test_enable_adopts_pose_measured_after_free_motion(self):
        arm = self.arm()
        arm.set_powered(False)
        arm.servo(0.01)
        moved = [math.radians(v) for v in [25,40,40,0,0,0]]
        arm._robot.teleport(moved)
        self.assertIsNone(arm.set_powered(True))
        self.assertFalse(arm._power_off_latched)
        self.assertAlmostEqual(math.degrees(arm._servo_cmd_rad[0]), 25)
        self.assertTrue(arm.poll().enabled)

    def test_native_path_also_calls_disable_and_checks_stop_mode(self):
        arm = self.arm()
        with patch.object(arm, "_is_fake", return_value=False):
            arm.set_powered(False)
            arm.servo(0.01)
        self.assertIn("disable", arm._robot.calls)
        self.assertFalse(arm.poll().enabled)
        self.assertEqual(arm._writer_err, "")

    def test_failed_mode_switch_is_reported_and_never_reenables(self):
        class NoStop(FakeFafuController):
            def disable(self):
                self._note("disable")

        arm = self.arm(NoStop)
        arm.set_powered(False)
        arm.servo(0.01)
        self.assertIn("STOP", arm._writer_err)
        self.assertTrue(arm.poll().enabled, "failed hardware command must not be reported as disabled")
        self.assertTrue(arm._power_off_latched)
        count = arm._robot.calls.count("disable")
        for _ in range(10):
            arm.hold(); arm.servo(0.01)
        self.assertEqual(arm._robot.calls.count("disable"), count, "no automatic retry")

    def test_shutdown_waits_for_gravity_writer_to_exit(self):
        arm = self.arm()
        arm.set_powered(False)
        with patch.object(arm, "_stop_gravity", return_value=False):
            arm.servo(0.01)
        self.assertNotIn("disable", arm._robot.calls)
        self.assertIn("重力", arm._writer_err)

    def test_explicit_disable_survives_host_close(self):
        arm = self.arm()
        robot = arm._robot
        arm.set_powered(False)
        arm.servo(0.01)
        with patch.object(robot, "close_connection") as close:
            arm.stop()
        close.assert_called_once_with(joint_release="stop", gripper_release="stop")

    def test_read_only_controller_cannot_disable(self):
        arm = FafuArm(allow_motion=False, controller_cls=FakeFafuController)
        arm.start()
        self.addCleanup(arm.stop)
        self.assertIsNotNone(arm.set_powered(False))
        self.assertNotIn("disable", arm._robot.calls)

    def test_motion_rpc_can_enable_again_after_explicit_disable(self):
        import time
        core = MotionCore(StationConfig(arm="sim"))
        core.arm.start()
        self.addCleanup(core.arm.stop)
        core.add_client("operator")
        self.assertIsNone(core.on_power("operator", True))
        self.assertIsNone(core.on_power("operator", False))
        core._step(time.monotonic(), 0.01)
        self.assertFalse(core.arm.poll().enabled)
        self.assertIsNone(core.on_power("operator", True))
        self.assertTrue(core.arm.poll().enabled)
