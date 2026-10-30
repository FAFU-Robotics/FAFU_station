from __future__ import annotations

import math
import unittest
from unittest.mock import patch

from robot_station.config import StationConfig
from robot_station.motion import MotionCore
from tests.test_traj import recordings_tmpdir


class TeachControlTests(unittest.TestCase):
    def setUp(self):
        self.tmp = recordings_tmpdir()
        self.tmp.__enter__()
        self.core = MotionCore(StationConfig(arm="sim"))
        self.core.arm.start()
        self.core.arm.set_powered(True)

    def tearDown(self):
        self.core.arm.stop()
        self.tmp.__exit__(None, None, None)

    def advance(self, count):
        for _ in range(count):
            self.core.arm.servo(0.01)

    def test_home_then_park_and_send_never_restores_old_target(self):
        arm = self.core.arm
        self.assertIsNone(arm.apply_targets([30,40,40,0,0,0], 40, safe_shoulder=False))
        self.advance(200)
        self.assertIsNone(arm.home(40))
        self.advance(600)
        before = arm.poll().q_deg
        self.assertLess(max(abs(q) for q in before), 0.1)
        arm.park_stream()  # Sent by the browser immediately before "发送位置".
        self.advance(1)
        after = arm.poll().q_deg
        self.assertLess(max(abs(a-b) for a,b in zip(before, after)), 0.5)
        self.assertIsNone(arm.apply_targets([30,40,40,0,0,0],10,safe_shoulder=False))
        self.advance(1)
        self.assertLess(max(abs(a-b) for a,b in zip(after,arm.poll().q_deg)),0.11)
        self.assertTrue(arm.poll().moving)
        self.advance(50)
        self.assertLess(arm.poll().q_deg[1],6)

    def test_recording_locks_mode_until_stop_and_drag_park_does_not_start_position_writer(self):
        arm=self.core.arm
        self.assertIsNone(self.core.on_rec_start("ui", teach="drag"))
        self.assertIn("录制",self.core.on_mode("ui","Position") or "")
        self.assertEqual(self.core.arm_mode(),"Gravity")
        self.core.on_park("ui")
        self.assertIsNone(self.core.on_teach("ui",[25,40,40,0,0,0]))
        self.advance(4)
        self.assertAlmostEqual(arm.poll().q_deg[0],25)
        self.assertFalse(arm._servo_intent)
        self.assertTrue(arm.poll().recording)
        self.assertIsNone(self.core.on_rec_stop("ui"))
        self.assertEqual(self.core.arm_mode(),"Position")
        self.assertIsNone(self.core.on_rec_start("ui",teach="soft"))
        self.assertIn("录制",self.core.on_mode("ui","Gravity") or "")
        self.assertEqual(self.core.arm_mode(),"Position")

    def test_new_manual_target_takes_over_without_finishing_the_old_goal(self):
        arm = self.core.arm
        for simulated in (True, False):
            with self.subTest(simulated=simulated):
                arm._robot.sim_teach = simulated  # Fake controller also exercises the live branch.
                self.assertIsNone(self.core.on_arm_targets("ui", [70,40,40,0,0,0], 10))
                self.advance(20)
                before = arm.poll().feedback_deg
                self.assertLess(before[0], 70)
                self.assertIsNone(self.core.on_arm_targets("ui", [-20,40,40,0,0,0], 10))
                self.advance(1)
                after = arm.poll().feedback_deg
                self.assertLess(after[0], before[0], "the very next tick must head towards the new target")
                self.assertLessEqual(max(abs(a-b) for a,b in zip(before, after)), 0.101)
                self.advance(1200)
                self.assertAlmostEqual(arm.poll().feedback_deg[0], -20, delta=0.1)

    def test_manual_target_replaces_home_and_discards_pending_keyboard_motion(self):
        arm = self.core.arm
        self.assertIsNone(self.core.on_home("ui", 10))
        self.advance(20)
        self.assertIsNone(self.core.on_cartesian("ui", [0.005,0,0], [0,0,0]))
        self.assertIsNone(self.core.on_arm_targets("ui", [25,40,40,0,0,0], 10))
        self.assertFalse(self.core._cart_pending)
        self.core._flush_cart()
        self.assertEqual(arm.poll().target_deg, [25,40,40,0,0,0])
        before = arm.poll().feedback_deg
        self.advance(1)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(before, arm.poll().feedback_deg)), 0.101)
        self.advance(1500)
        self.assertEqual([round(q, 1) for q in arm.poll().feedback_deg], [25,40,40,0,0,0])

    def test_park_never_restores_a_stale_servo_pose(self):
        arm = self.core.arm
        for simulated in (True, False):
            for servoing in (True, False):
                with self.subTest(simulated=simulated, servoing=servoing):
                    arm._robot.sim_teach = simulated
                    arm._servoing = servoing
                    current = arm._q_rad([0,0,0,0,0,0])
                    arm._robot.teleport(current)
                    arm._servo_cmd_rad = arm._q_rad([30,40,40,0,0,0])
                    arm._stream_q_rad = list(arm._servo_cmd_rad)
                    arm.park_stream()
                    self.advance(1)
                    self.assertLess(max(abs(q) for q in arm.poll().feedback_deg), 0.1)

    def test_real_controller_cannot_accept_simulated_drag_even_without_dynamics(self):
        arm=self.core.arm
        arm._robot.sim_teach=False  # Exercise the real-controller branch with no USB.
        arm._ctrl_mode="Gravity"
        original=list(arm._target_deg)
        error=self.core.on_teach("ui",[50,60,60,0,0,0])
        self.assertIn("仿真",error or "")
        self.assertEqual(arm._target_deg,original)

    def test_cartesian_follow_parks_only_the_owners_follow_motion(self):
        arm = self.core.arm
        with patch.object(arm, "apply_cartesian_pose", side_effect=lambda *args: arm.apply_targets([30,40,40,0,0,0],10,safe_shoulder=False)):
            self.assertIsNone(self.core.on_cartesian_pose("ui", [0.3,0,0.3], [0,0,0],10,stream=True))
            self.advance(10)
            self.core.on_park("another-window")
            self.assertTrue(arm._home_path)
            self.core.on_park("ui")
            self.assertFalse(arm._home_path)
            before = arm.poll().feedback_deg
            self.advance(20)
            self.assertEqual(arm.poll().feedback_deg, before)
            self.assertIsNone(self.core.on_cartesian_pose("ui", [0.3,0,0.3], [0,0,0],10,stream=True))
            self.assertIsNone(self.core.on_arm_targets("ui", [-20,40,40,0,0,0],10))
            self.core.on_park("ui")
            self.assertTrue(arm._home_path, "turning off old follow must not cancel a new manual move")

    def test_failed_exit_from_teach_does_not_claim_position_mode(self):
        self.assertIsNone(self.core.on_rec_start("ui",teach="drag"))
        with patch.object(self.core.arm,"_reenter_position",return_value="测试恢复失败"):
            self.assertIn("恢复失败",self.core.on_rec_stop("ui") or "")
        self.assertFalse(self.core.arm.poll().recording)
        self.assertEqual(self.core.arm_mode(),"Gravity")
