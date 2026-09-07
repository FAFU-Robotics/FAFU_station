from __future__ import annotations

import os
import unittest
from unittest.mock import patch

from robot_station.adapters.fafu_arm import FafuArm
from robot_station.serial_guard import apply_live_arm_policy, live_serial_allowed


class SerialGuardTests(unittest.TestCase):
    def test_default_denies_live_serial(self) -> None:
        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop("STATION_ALLOW_LIVE_ARM", None)
            self.assertFalse(live_serial_allowed())
            self.assertEqual(apply_live_arm_policy("fafu"), "sim")
            self.assertEqual(apply_live_arm_policy("auto"), "sim")
            self.assertEqual(apply_live_arm_policy("mock"), "mock")
            self.assertEqual(apply_live_arm_policy("sim"), "sim")

    def test_opt_in_keeps_fafu(self) -> None:
        with patch.dict(os.environ, {"STATION_ALLOW_LIVE_ARM": "1"}):
            self.assertTrue(live_serial_allowed())
            self.assertEqual(apply_live_arm_policy("fafu"), "fafu")

    def test_real_adapter_start_blocked_without_opt_in(self) -> None:
        os.environ.pop("STATION_ALLOW_LIVE_ARM", None)
        arm = FafuArm(required=True, allow_motion=False)
        with self.assertRaises(RuntimeError) as ctx:
            arm.start()
        self.assertIn("STATION_ALLOW_LIVE_ARM", str(ctx.exception))
        self.assertIsNone(arm._robot)


if __name__ == "__main__":
    unittest.main()
