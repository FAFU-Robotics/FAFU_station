from __future__ import annotations

import unittest

from robot_station.adapters.arm import MockArm, build_arm
from robot_station.adapters.fafu_arm import FafuArm, import_fafu_sdk, resolve_sdk_root
from robot_station.config import StationConfig
from robot_station.motion import MotionCore


class FafuArmTests(unittest.TestCase):
    def test_mock_still_default(self) -> None:
        arm = build_arm("mock", 6, 40.0, True)
        self.assertIsInstance(arm, MockArm)
        self.assertIsNone(arm.refuse_motion())

    def test_sim_kind(self) -> None:
        arm = build_arm("sim", 6, 40.0, True)
        self.assertIsInstance(arm, FafuArm)
        self.assertEqual(arm.backend, "sim")
        self.assertTrue(arm.allow_motion)

    def test_unknown_kind_rejected(self) -> None:
        with self.assertRaises(ValueError):
            build_arm("real", 6, 40.0, True)

    def test_allow_motion_without_live_env_stays_readonly(self) -> None:
        import os

        os.environ.pop("STATION_ALLOW_LIVE_ARM", None)
        arm = build_arm("fafu", 6, 40.0, True, allow_motion=True)
        self.assertIsInstance(arm, FafuArm)
        self.assertFalse(arm.allow_motion)

    def test_allow_motion_with_live_env(self) -> None:
        import os
        from unittest.mock import patch

        with patch.dict(os.environ, {"STATION_ALLOW_LIVE_ARM": "1"}):
            arm = build_arm("fafu", 6, 40.0, True, allow_motion=True)
        self.assertIsInstance(arm, FafuArm)
        self.assertTrue(arm.allow_motion)
        self.assertIsNone(arm._robot)

    def test_fafu_and_auto_do_not_open_serial_in_ctor(self) -> None:
        fafu = build_arm("fafu", 6, 40.0, True)
        auto = build_arm("auto", 6, 40.0, True)
        self.assertIsInstance(fafu, FafuArm)
        self.assertIsInstance(auto, FafuArm)
        self.assertFalse(fafu.required)
        self.assertFalse(auto.required)
        self.assertIsNone(fafu._robot)
        self.assertEqual(fafu.poll().backend, "fafu")
        self.assertFalse(fafu.poll().online)

    def test_read_only_gates(self) -> None:
        arm = FafuArm(required=False, allow_motion=False)
        self.assertIsNotNone(arm.refuse_motion())
        arm.hold()
        arm.servo(0.01)
        arm.apply_targets([1, 0, 0, 0, 0, 0], 10.0)
        arm.home(10.0)
        arm.set_gripper_open(False)
        snap = arm.poll()
        self.assertEqual(snap.backend, "fafu")
        self.assertFalse(snap.online)
        self.assertFalse(snap.enabled)

    def test_native_module_loads_without_serial(self) -> None:
        try:
            root = resolve_sdk_root()
        except FileNotFoundError:
            self.skipTest("本机未安装 fafu_arm_sdk")
        try:
            pm, controller = import_fafu_sdk(root)
        except RuntimeError as exc:
            self.skipTest(str(exc))
        self.assertTrue(callable(controller))
        self.assertTrue(hasattr(pm, "HightorqueSerial"))
        self.assertTrue(
            hasattr(pm, "TORQUE_COEFF")
            or int(getattr(pm, "CORE_ABI_VERSION", 0) or 0) == 4
        )

    def test_motion_core_refuses_fafu_commands_before_start(self) -> None:
        core = MotionCore(StationConfig(arm="fafu", open_browser=False))
        blocked = core.on_arm_targets("a", [0, 0, 0, 0, 0, 0], 10.0)
        self.assertIsNotNone(blocked)
        self.assertIn("未连接", blocked or "")


if __name__ == "__main__":
    unittest.main()
