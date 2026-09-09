from __future__ import annotations

import unittest

from robot_station.adapters.camera import (
    AutoCameraBank,
    CamInfo,
    CameraBank,
    MockCameraBank,
    build_camera,
    clarify_camera_error,
    describe_camera_choice,
    pick_work_camera,
    score_camera,
)
from robot_station.config import StationConfig
from robot_station.world import CamSnap


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

    def test_idle_reason_skips_lid_and_waits_for_usb(self) -> None:
        lid = CamInfo(name="Integrated Camera", vid="5986", pid="2175", kind="uvc")
        picked, reason = describe_camera_choice([lid], rs_ok=True)
        self.assertIsNone(picked)
        self.assertIn("未接入作业相机 USB", reason)
        self.assertIn("已忽略笔记本内置摄像头", reason)
        self.assertIn("接入后将自动出画", reason)
        none, empty = describe_camera_choice([], rs_ok=True)
        self.assertIsNone(none)
        self.assertIn("未接入作业相机 USB", empty)

    def test_reason_when_pyrealsense_missing(self) -> None:
        work = CamInfo(
            name="Intel RealSense D405",
            serial="255323073306",
            vid="8086",
            pid="0B5B",
            kind="realsense",
        )
        picked, reason = describe_camera_choice([work], rs_ok=False)
        self.assertIsNotNone(picked)
        self.assertIn("未安装 pyrealsense2", reason)

    def test_clarify_camera_error(self) -> None:
        self.assertIn("隐私", clarify_camera_error(RuntimeError("Access denied")))
        self.assertIn("占用", clarify_camera_error(RuntimeError("device is busy")))
        self.assertIn("拔出", clarify_camera_error(RuntimeError("No device connected")))
        self.assertIn("打开作业相机失败", clarify_camera_error(RuntimeError("weird firmware")))

    def test_hotplug_opens_live_without_restarting_process(self) -> None:
        work = CamInfo(
            name="Intel RealSense D405",
            serial="S1",
            vid="8086",
            pid="0B5B",
            kind="realsense",
        )
        opened: list[CamInfo] = []

        class DummyLive(CameraBank):
            backend = "realsense"

            def __init__(self, info: CamInfo) -> None:
                self.info = info
                self.device = info.name
                self.started = 0
                self.stopped = 0
                self._alive = True
                self._online = True

            def start(self) -> None:
                self.started += 1

            def stop(self) -> None:
                self.stopped += 1
                self._alive = False

            def capture_alive(self) -> bool:
                return self._alive

            def poll(self) -> CamSnap:
                return CamSnap(
                    online=[self._online],
                    fps=[15.0],
                    backend="realsense",
                    device=self.device,
                    reason="" if self._online else "作业相机已拔出，等待重新接入",
                )

            def latest_png(self, stream_id: int) -> tuple[int, int, bytes] | None:
                return None

        found: list[CamInfo] = []
        bank = AutoCameraBank(1, 16, 8, 5)
        bank.discover_fn = lambda: list(found)
        bank.rs_ok_fn = lambda: True
        bank.live_factory = lambda info: opened.append(info) or DummyLive(info)
        bank._tick()
        self.assertEqual(bank.poll().backend, "mock")
        self.assertIn("未接入作业相机 USB", bank.poll().reason)
        found.append(work)
        bank._tick()
        self.assertEqual(len(opened), 1)
        self.assertEqual(bank.poll().backend, "realsense")
        self.assertTrue(bank.poll().online[0])
        live = bank._inner
        assert isinstance(live, DummyLive)
        self.assertTrue(bank._same_device(live, work))
        found.clear()
        bank._tick()
        self.assertIs(bank._inner, live)
        self.assertEqual(live.stopped, 0)
        live._online = False
        live._alive = False
        bank._tick()
        self.assertEqual(bank.poll().backend, "mock")
        self.assertIn("拔出", bank.poll().reason)
        bank._inner.stop()

    def test_camera_mock_kind_has_no_hotplug_watch(self) -> None:
        bank = build_camera("mock", 1, 16, 8, 5)
        self.assertIsInstance(bank, MockCameraBank)
        self.assertFalse(hasattr(bank, "_tick"))


if __name__ == "__main__":
    unittest.main()
