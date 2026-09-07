from __future__ import annotations

import unittest

from robot_station.adapters.camera import (
    AutoCameraBank,
    CamInfo,
    MockCameraBank,
    build_camera,
    pick_work_camera,
    score_camera,
)
from robot_station.config import StationConfig


class CameraDetectTests(unittest.TestCase):
    def test_prefers_realsense_over_laptop_webcam(self) -> None:
        lid = CamInfo(
            name="Integrated Camera",
            device_id=r"USB\VID_5986&PID_2175&MI_00\1",
            vid="5986",
            pid="2175",
            kind="uvc",
        )
        work = CamInfo(
            name="Intel(R) RealSense(TM) Depth Camera 405  Depth",
            device_id=r"USB\VID_8086&PID_0B5B&MI_00\1",
            serial="255323073306",
            vid="8086",
            pid="0B5B",
            kind="realsense",
        )
        self.assertLessEqual(score_camera(lid), 0)
        self.assertGreater(score_camera(work), 0)
        picked = pick_work_camera([lid, work])
        self.assertIsNotNone(picked)
        self.assertEqual(picked.pid, "0B5B")
        self.assertEqual(picked.kind, "realsense")

    def test_skips_only_builtin_webcam(self) -> None:
        lid = CamInfo(name="Integrated Camera", vid="5986", pid="2175", kind="uvc")
        self.assertIsNone(pick_work_camera([lid]))

    def test_build_mock_explicit(self) -> None:
        bank = build_camera("mock", 1, 16, 8, 5)
        self.assertIsInstance(bank, MockCameraBank)

    def test_build_auto_wrapper(self) -> None:
        bank = build_camera("auto", 1, 16, 8, 5)
        self.assertIsInstance(bank, AutoCameraBank)

    def test_default_config_stays_mock_for_tests(self) -> None:
        self.assertEqual(StationConfig().camera, "mock")


if __name__ == "__main__":
    unittest.main()
