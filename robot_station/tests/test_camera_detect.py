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

    def test_rescan_swaps_when_choose_changes(self) -> None:
        bank = AutoCameraBank(1, 16, 8, 5)
        first = MockCameraBank(1, 16, 8, 5, reason="a")
        second = MockCameraBank(1, 16, 8, 5, reason="b")
        second.device = "realsense:1"
        bank._inner = first
        picks = [second]

        def _choose() -> MockCameraBank:
            return picks[0]

        bank._choose = _choose  # type: ignore[method-assign]
        out = bank.rescan()
        self.assertIs(out, second)
        self.assertIs(bank._inner, second)
        bank.stop()

    def test_list_linux_cameras_from_sys_root(self) -> None:
        import tempfile
        from pathlib import Path

        from robot_station.adapters.camera import list_linux_cameras

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            video = root / "video0"
            video.mkdir()
            (video / "name").write_text("Intel RealSense D405\n", encoding="utf-8")
            (video / "idVendor").write_text("8086\n", encoding="ascii")
            (video / "idProduct").write_text("0b5b\n", encoding="ascii")
            meta = root / "video1"
            meta.mkdir()
            (meta / "name").write_text("Intel RealSense D405 Metadata\n", encoding="utf-8")
            found = list_linux_cameras(sys_root=root)
            self.assertEqual(len(found), 1)
            self.assertEqual(found[0].kind, "realsense")
            self.assertEqual(found[0].vid, "8086")
            self.assertEqual(found[0].pid, "0B5B")
            self.assertGreater(score_camera(found[0]), 0)
            self.assertEqual(pick_work_camera(found).pid, "0B5B")

    def test_list_linux_cameras_skips_laptop_webcam(self) -> None:
        import tempfile
        from pathlib import Path

        from robot_station.adapters.camera import list_linux_cameras

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            video = root / "video0"
            video.mkdir()
            (video / "name").write_text("Integrated Camera\n", encoding="utf-8")
            (video / "idVendor").write_text("5986\n", encoding="ascii")
            (video / "idProduct").write_text("2175\n", encoding="ascii")
            found = list_linux_cameras(sys_root=root)
            self.assertEqual(len(found), 1)
            self.assertIsNone(pick_work_camera(found))

    def test_rescan_keeps_backend_when_same_device(self) -> None:
        bank = AutoCameraBank(1, 16, 8, 5)
        inner = MockCameraBank(1, 16, 8, 5, reason="same")
        inner.device = "cam-a"
        bank._inner = inner
        clone = MockCameraBank(1, 16, 8, 5, reason="same")
        clone.device = "cam-a"

        bank._choose = lambda: clone  # type: ignore[method-assign]
        out = bank.rescan()
        self.assertIs(out, inner)


if __name__ == "__main__":
    unittest.main()
