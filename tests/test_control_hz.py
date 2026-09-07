from __future__ import annotations

import time
import unittest
from types import SimpleNamespace

from robot_station.adapters.fafu_sim import FakeFafuController
from robot_station.config import StationConfig
from robot_station.motion import MotionCore
from robot_station.service import WebFront


class ServoOptsSimTests(unittest.TestCase):
    def test_servo_j_lands_this_tick(self) -> None:
        FakeFafuController.last = None
        fake = FakeFafuController("sim://fake", allow_motion=True, auto_enable=False)
        fake.enable()
        fake.servo_start(SimpleNamespace(max_vel=1.5, max_step_rad=0.03, rate_hz=100.0))
        q = list(fake.q_rad)
        q[0] = q[0] + 1.0
        fake.servo_j(q)
        self.assertAlmostEqual(fake.q_rad[0], q[0], places=6)

    def test_impedance_scale_does_not_snap(self) -> None:
        fake = FakeFafuController("sim://fake", allow_motion=True, auto_enable=False)
        fake.enable()
        fake.set_speed_scale(0.35)
        fake.servo_start(SimpleNamespace(max_vel=10.0, max_step_rad=0.20, rate_hz=100.0))
        q = list(fake.q_rad)
        q[0] = q[0] + 1.0
        fake.servo_j(q)
        q0 = fake.q_rad[0]
        fake.step(0.01)
        dq = abs(fake.q_rad[0] - q0)
        self.assertGreater(dq, 0.01)
        self.assertLess(dq, 0.08)

    def test_move_j_still_interpolates(self) -> None:
        fake = FakeFafuController("sim://fake", allow_motion=True, auto_enable=False)
        fake.enable()
        goal = list(fake.q_rad)
        goal[0] = goal[0] + 1.0
        fake.move_j(goal, is_radians=True, speed=50, block=False)
        q0 = fake.q_rad[0]
        fake.step(0.01)
        self.assertGreater(abs(fake.q_rad[0] - q0), 1e-6)
        self.assertLess(abs(fake.q_rad[0] - goal[0]), 1.0)


class WaitSnapTests(unittest.TestCase):
    def test_wait_snap_advances_with_tick(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=50, telemetry_hz=50)
        )
        core.start()
        try:
            seq0, _ = core.wait_snap(-1, 0.4)
            seq1, snap = core.wait_snap(seq0, 0.4)
            self.assertGreater(seq1, seq0)
            self.assertEqual(snap["arm"]["backend"], "sim")
        finally:
            core.stop()

    def test_stream_error_lands_in_cmd_err(self) -> None:
        core = MotionCore(StationConfig(arm="sim", open_browser=False, control_hz=40))
        core.start()
        try:
            core.on_estop("test")
            ack = core.handle({"t": "cart", "cid": "a", "dxyz": [0.01, 0, 0], "drpy": [0, 0, 0]})
            self.assertFalse(ack.get("ok"))
            self.assertIn("急停", core.snapshot().get("cmd_err") or "")
        finally:
            core.stop()


class StreamSendTests(unittest.TestCase):
    def test_webfront_stream_does_not_wait_ack(self) -> None:
        front = WebFront(
            StationConfig(camera="mock", video_hz=1, camera_width=8, camera_height=8)
        )

        class FakeMotion:
            def __init__(self) -> None:
                self.sent: list[dict] = []
                self.called: list[dict] = []

            def send(self, msg: dict) -> None:
                self.sent.append(msg)

            def call(self, msg: dict, timeout_s: float = 1.0) -> dict:
                self.called.append(msg)
                return {"t": "ack", "ok": True}

        front.motion = FakeMotion()  # type: ignore[assignment]
        self.assertIsNone(front.on_cartesian("c", [0.001, 0.0, 0.0], [0.0, 0.0, 0.0], t0=12.5))
        self.assertEqual(front.motion.called, [])
        self.assertEqual(front.motion.sent[0]["t"], "cart")
        self.assertEqual(front.motion.sent[0]["t0"], 12.5)
        self.assertIsNone(front.on_arm_targets("c", [0.0] * 6, 40.0, stream=True))
        self.assertTrue(front.motion.sent[-1].get("stream"))
        self.assertIsNone(front.on_teach("c", [0.0] * 6))
        self.assertEqual(front.motion.sent[-1]["t"], "teach")
        self.assertIsNone(front.on_arm_targets("c", [0.0] * 6, 40.0, stream=False))
        self.assertEqual(len(front.motion.called), 1)
        self.assertTrue(front.teleop_hot())
        n_sent = len(front.motion.sent)
        self.assertIsNone(
            front.on_cartesian_pose("c", [0.25, 0.0, 0.17], [0.0, 0.0, 0.0], 40.0)
        )
        self.assertEqual(len(front.motion.sent), n_sent)
        self.assertEqual(front.motion.called[-1]["t"], "cart_go")
        self.assertEqual(front.motion.called[-1]["speed"], 40.0)


class TickRateTests(unittest.TestCase):
    def test_control_tick_near_configured_hz(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=100, telemetry_hz=100)
        )
        core.start()
        try:
            time.sleep(0.05)
            seq0 = core.snapshot()["seq"]
            time.sleep(0.35)
            seq1 = core.snapshot()["seq"]
            hz = (seq1 - seq0) / 0.35
            self.assertGreater(hz, 55)
            self.assertLess(hz, 150)
        finally:
            core.stop()

    def test_mock_stream_lands_within_one_tick(self) -> None:
        core = MotionCore(
            StationConfig(arm="mock", open_browser=False, control_hz=100, telemetry_hz=100)
        )
        core.start()
        try:
            err = core.on_arm_targets("a", [40.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True)
            self.assertIsNone(err, err)
            time.sleep(0.03)
            self.assertAlmostEqual(core.snapshot()["arm"]["q_deg"][0], 40.0, places=1)
        finally:
            core.stop()

    def test_sim_stream_lands_within_one_tick(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=100, telemetry_hz=100)
        )
        core.start()
        try:
            err = core.on_arm_targets("a", [20.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True)
            self.assertIsNone(err, err)
            time.sleep(0.05)
            self.assertAlmostEqual(core.snapshot()["arm"]["q_deg"][0], 20.0, places=1)
        finally:
            core.stop()

    def test_stream_echo_t0_lands_in_snapshot(self) -> None:
        core = MotionCore(
            StationConfig(arm="mock", open_browser=False, control_hz=100, telemetry_hz=100)
        )
        core.start()
        try:
            err = core.on_arm_targets(
                "a", [5.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0, stream=True, t0=1234.5
            )
            self.assertIsNone(err, err)
            self.assertAlmostEqual(core.snapshot()["echo_t0"], 1234.5, places=2)
        finally:
            core.stop()


if __name__ == "__main__":
    unittest.main()
