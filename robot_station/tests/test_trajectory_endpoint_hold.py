from __future__ import annotations

import math
import time
import unittest

from robot_station.config import StationConfig
from robot_station.motion import MotionCore, ENDPOINT_HOLD_TIMEOUT_S
from robot_station.traj import TrajectoryWriter, make_traj_path
from tests.test_traj import recordings_tmpdir


class EndpointHoldTests(unittest.TestCase):
    def setUp(self):
        self.storage = recordings_tmpdir()
        self.storage.__enter__()
        self.core = MotionCore(StationConfig(arm="sim"))
        self.core.arm.start()
        self.core.arm.set_powered(True)
        self.core.add_client("operator")
        self.core.add_client("observer")
        snap = self.core.arm.poll()
        target = list(snap.q_deg)
        target[0] += 10
        self.target = target
        self.path = make_traj_path("endpoint")
        rows = [{"t":i/100,"pos":[math.radians(q) for q in target],
                 "gpos":math.radians(snap.gripper_deg),"vel":[0]*6} for i in range(101)]
        writer = TrajectoryWriter(self.path, teach="soft", mode="Position", n=6, prefix=rows)
        writer.close()
        self.original = self.path.read_bytes()

    def tearDown(self):
        self.core.arm.stop()
        self.storage.__exit__(None, None, None)

    def command(self, action, owner="operator", token="press-1", **values):
        return self.core.on_workbench(owner, {"action":action,"file":self.path.name,"hold_id":token, **values})

    def begin(self, token="press-1"):
        reply = self.command("traj_endpoint", token=token)
        self.assertTrue(reply["ok"], reply)
        self.assertTrue(reply["data"]["active"])

    def test_release_stops_partial_motion_and_later_press_resumes(self):
        initial = self.core.arm.poll().q_deg[0]
        self.begin()
        self.core._step(time.monotonic(), 0.1)
        partial = self.core.arm.poll().q_deg[0]
        self.assertGreater(partial, initial)
        self.assertLess(partial, self.target[0]-1)
        self.command("traj_endpoint_stop")
        for _ in range(20): self.core._step(time.monotonic(), 0.02)
        self.assertAlmostEqual(self.core.arm.poll().q_deg[0], partial, delta=0.03)
        self.assertIsNone(self.core._endpoint_hold)
        self.begin("press-2")
        for _ in range(150):
            self.command("traj_endpoint_touch", token="press-2")
            self.core._step(time.monotonic(), 0.01)
        self.assertIsNone(self.core._endpoint_hold)
        self.assertAlmostEqual(self.core.arm.poll().q_deg[0], self.target[0], delta=0.26)
        reached = self.command("traj_endpoint", token="press-3")
        self.assertTrue(reached["data"]["reached"])
        self.assertFalse(reached["data"]["active"])
        self.assertEqual(self.path.read_bytes(), self.original)

    def test_generic_touch_and_other_clients_cannot_keep_a_hold_alive(self):
        self.begin()
        for _ in range(5): self.core.on_touch("operator")
        other = self.command("traj_endpoint_touch", owner="observer")
        self.assertFalse(other["data"]["active"])
        self.core._step(time.monotonic()+ENDPOINT_HOLD_TIMEOUT_S+0.01, 0.01)
        self.assertIsNone(self.core._endpoint_hold)
        self.assertFalse(self.core.arm.is_homing)
        self.assertFalse(self.command("traj_endpoint_touch")["data"]["active"])

    def test_disconnect_stops_owner_even_with_an_observer_connected(self):
        self.begin()
        self.core.drop_client("operator")
        self.assertIn("observer", self.core.clients)
        self.assertIsNone(self.core._endpoint_hold)
        self.assertFalse(self.core.arm.is_homing)

    def test_stale_release_cannot_stop_a_new_press_or_other_manual_motion(self):
        self.begin()
        self.command("traj_endpoint_stop", token="stale")
        self.assertIsNotNone(self.core._endpoint_hold)
        self.command("traj_endpoint_stop")
        self.begin("press-2")
        self.command("traj_endpoint_stop")
        self.assertEqual(self.core._endpoint_hold["id"], "press-2")
        new_target = list(self.target); new_target[0] += 5
        self.assertIsNone(self.core.on_arm_targets("observer", new_target, 10))
        self.core._step(time.monotonic(), 0.01)
        before = self.core.arm.poll().q_deg[0]
        self.command("traj_endpoint_stop", token="press-2")
        for _ in range(5): self.core._step(time.monotonic(), 0.01)
        self.assertGreater(self.core.arm.poll().q_deg[0], before+0.1)

    def test_old_click_only_request_and_disabled_arm_cannot_start_motion(self):
        self.assertFalse(self.command("traj_endpoint", token="")["ok"])
        self.core.arm.set_powered(False)
        self.assertFalse(self.command("traj_endpoint")["ok"])
        self.assertIsNone(self.core._endpoint_hold)


if __name__ == "__main__":
    unittest.main()
