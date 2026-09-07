from __future__ import annotations

import inspect
import os
import time
import unittest

from robot_station.adapters.fafu_arm import FafuArm, import_fafu_sdk, probe_link, resolve_sdk_root
from robot_station.adapters.fafu_sim import MOTION_METHODS, FakeFafuController
from robot_station.config import StationConfig, _as_bool
from robot_station.motion import MotionCore


def _attach_fake(core: MotionCore) -> None:
    FakeFafuController.last = None
    core.arm._controller_cls = FakeFafuController


def _motion_calls(fake: FakeFafuController) -> list[str]:
    return [name for name in fake.calls if name in MOTION_METHODS]


class FafuSimTests(unittest.TestCase):
    """No USB, no CAN. Fake controller proves read-only wiring of steps 1–2."""

    def test_probe_sim_never_enables(self) -> None:
        FakeFafuController.last = None
        self.assertEqual(probe_link(sim=True), 0)
        fake = FakeFafuController.last
        self.assertIsNotNone(fake)
        assert fake is not None
        self.assertIs(fake.kwargs.get("auto_enable"), False)
        self.assertTrue(fake.kwargs.get("has_gripper"))
        self.assertEqual(fake.kwargs.get("gripper_motor_id"), 7)
        self.assertTrue(fake.closed)
        self.assertEqual(_motion_calls(fake), [])
        self.assertIn("get_joint_values", fake.calls)
        self.assertIn("close_connection", fake.calls)

    def test_rad_to_deg_and_fault_bits(self) -> None:
        arm = FafuArm(required=True, allow_motion=False, controller_cls=FakeFafuController)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            snap = arm.poll()
            self.assertTrue(snap.online)
            self.assertFalse(snap.enabled)
            self.assertEqual(snap.backend, "fafu")
            self.assertAlmostEqual(snap.q_deg[0], 0.0, places=4)
            self.assertAlmostEqual(snap.q_deg[1], 0.0, places=4)
            self.assertAlmostEqual(snap.q_deg[2], 0.0, places=4)
            self.assertTrue(all(snap.ok))
            self.assertFalse(snap.moving)
            fake.v_rad_s[0] = 0.5
            fake.faults[2] = 1
            snap = arm.poll()
            self.assertTrue(snap.moving)
            self.assertFalse(snap.ok[2])
            self.assertTrue(snap.ok[0])
        finally:
            arm.stop()

    def test_ui_commands_refused_while_connected(self) -> None:
        core = MotionCore(StationConfig(arm="fafu", open_browser=False))
        _attach_fake(core)
        core.start()
        try:
            self.assertIn("只读", core.on_arm_targets("a", [1, 0, 0, 0, 0, 0], 10.0) or "")
            self.assertIn("只读", core.on_home("a", 10.0) or "")
            self.assertIn("只读", core.on_gripper("a", False) or "")
            fake = FakeFafuController.last
            assert fake is not None
            self.assertEqual(_motion_calls(fake), [])
            snap = core.snapshot()["arm"]
            self.assertTrue(snap["online"])
            self.assertEqual(snap["backend"], "fafu")
            self.assertFalse(snap["enabled"])
        finally:
            core.stop()

    def test_tick_estop_lease_do_not_command_motors(self) -> None:
        core = MotionCore(
            StationConfig(arm="fafu", open_browser=False, control_hz=50)
        )
        _attach_fake(core)
        core.start()
        try:
            time.sleep(0.12)
            core.on_estop("sim")
            fake = FakeFafuController.last
            assert fake is not None
            self.assertGreater(fake.calls.count("get_joint_values"), 3)
            self.assertEqual(_motion_calls(fake), [])
            self.assertNotIn("enable", fake.calls)
            self.assertNotIn("brake", fake.calls)
        finally:
            core.stop()
        fake = FakeFafuController.last
        assert fake is not None
        self.assertTrue(fake.closed)
        self.assertEqual(_motion_calls(fake), [])

    def test_auto_stays_offline_when_connect_fails(self) -> None:
        class Boom:
            def __init__(self, *args, **kwargs):
                raise RuntimeError("no USB debug board detected")

        core = MotionCore(StationConfig(arm="auto", open_browser=False))
        core.arm._controller_cls = Boom
        core.start()
        try:
            snap = core.snapshot()["arm"]
            self.assertEqual(snap["backend"], "fafu")
            self.assertFalse(snap["online"])
            self.assertIn("未连接", core.arm.refuse_motion() or "")
        finally:
            core.stop()

    def test_fafu_start_failure_falls_back_to_sim(self) -> None:
        class Boom:
            def __init__(self, *args, **kwargs):
                raise RuntimeError("motors did not respond within 500ms")

        core = MotionCore(StationConfig(arm="fafu", open_browser=False))
        core.arm._controller_cls = Boom
        core.arm.required = True
        core.start()
        try:
            self.assertIsNotNone(core._tick_th)
            self.assertEqual(core.arm_kind(), "sim")
            self.assertIn("仿真", core._cmd_err)
            self.assertIn("motors", core._cmd_err)
        finally:
            core.stop()

    def test_sdk_still_defaults_auto_enable_true(self) -> None:
        """SDK 默认仍会 enable；适配器必须继续显式传 False。"""
        try:
            root = resolve_sdk_root()
        except FileNotFoundError:
            self.skipTest("本机未安装 fafu_arm_sdk")
        try:
            _pm, cls = import_fafu_sdk(root)
        except RuntimeError as exc:
            self.skipTest(str(exc))
        default = inspect.signature(cls.__init__).parameters["auto_enable"].default
        self.assertIs(default, True)

    def test_allow_motion_string_false_is_false(self) -> None:
        self.assertFalse(_as_bool("false"))
        self.assertFalse(_as_bool("False"))
        cfg = StationConfig.from_mapping({"arm_allow_motion": "false"})
        self.assertIs(cfg.arm_allow_motion, False)
        from robot_station.adapters.arm import build_arm

        os.environ.pop("STATION_ALLOW_LIVE_ARM", None)
        arm = build_arm("fafu", 6, 40.0, True, allow_motion=True)
        self.assertFalse(arm.allow_motion)

    def test_yaml_station_stays_mock(self) -> None:
        from robot_station.config import load_config

        cfg = load_config()
        self.assertEqual(cfg.arm, "mock")
        self.assertFalse(cfg.arm_allow_motion)


if __name__ == "__main__":
    unittest.main()
