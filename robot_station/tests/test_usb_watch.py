"""USB change watcher: only a settled device change may trigger a rescan."""
from __future__ import annotations

import unittest

from robot_station import usb_watch
from robot_station.usb_watch import UsbWatcher


def one(devices: list[str]) -> tuple:
    """A one-group signature, the shape ``serial_signature`` / ``camera_signature`` return."""
    return (tuple(sorted(set(devices))),)


class Script:
    """Feed a probe a scripted sequence; repeats the last value when exhausted."""

    def __init__(self, *values: tuple) -> None:
        self.values = list(values)
        self.calls = 0
        self.last = values[-1] if values else one([])

    def __call__(self) -> tuple:
        self.calls += 1
        if len(self.values) > 1:
            return self.values.pop(0)
        if self.values:
            self.last = self.values[0]
        return self.last


class UsbWatcherTests(unittest.TestCase):
    def _watch(self, serial: Script, camera: Script | None = None, **kwargs) -> tuple:
        arms: list[bool] = []
        cams: list[bool] = []
        opts = {"settle_rounds": 1, "camera_every": 1}
        opts.update(kwargs)
        watcher = UsbWatcher(
            serial_fn=serial,
            camera_fn=camera,
            on_arm=arms.append,
            on_camera=cams.append if camera is not None else None,
            **opts,
        )
        return watcher, arms, cams

    # ---- arm side ------------------------------------------------------

    def test_no_change_never_fires(self) -> None:
        serial = Script(one(["COM3"]))
        watcher, arms, _cams = self._watch(serial)
        for _ in range(5):
            watcher.rescan()
        self.assertEqual(arms, [])
        self.assertEqual(watcher.checks, 5)
        self.assertEqual(watcher.changes, 0)

    def test_first_reading_is_not_a_change(self) -> None:
        """A board already present at startup is the adapter's job, not ours."""
        serial = Script(one(["COM3"]))
        watcher, arms, _cams = self._watch(serial)
        watcher.rescan()
        self.assertEqual(arms, [])
        self.assertEqual(watcher.changes, 0)

    def test_arm_appears_then_disappears(self) -> None:
        serial = Script(one([]), one(["COM3"]), one([]))
        watcher, arms, _cams = self._watch(serial)
        watcher.rescan()
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(arms, [True, False])
        self.assertEqual(watcher.changes, 2)

    def test_linux_tty_appears_then_disappears(self) -> None:
        serial = Script(one([]), one(["/dev/ttyUSB0"]), one([]))
        watcher, arms, _cams = self._watch(serial)
        watcher.rescan()
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(arms, [True, False])
        self.assertEqual(watcher.changes, 2)

    def test_linux_udev_symlink_rename_reports_attached(self) -> None:
        serial = Script(one(["/dev/ttyUSB0"]), one(["/dev/fafu_debug_board"]))
        watcher, arms, _cams = self._watch(serial)
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(arms, [True])

    def test_port_rename_reports_attached(self) -> None:
        """Re-plugging on another port keeps attached=True so the arm can retry."""
        serial = Script(one(["COM3"]), one(["COM7"]))
        watcher, arms, _cams = self._watch(serial)
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(arms, [True])

    def test_fast_replug_without_empty_reading_still_reports(self) -> None:
        """An unplug/re-plug that lands between two polls must not be missed."""
        serial = Script(one(["COM3"]), one(["COM3", "COM4"]))
        watcher, arms, _cams = self._watch(serial)
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(arms, [True])

    # ---- camera side ---------------------------------------------------

    def test_camera_appears_and_disappears(self) -> None:
        serial = Script(one([]))
        camera = Script(one([]), one(["cam-A"]), one(["cam-A", "cam-B"]), one([]))
        watcher, _arms, cams = self._watch(serial, camera)
        for _ in range(4):
            watcher.rescan()
        self.assertEqual(cams, [True, True, False])

    def test_arm_callback_not_fired_by_camera_change(self) -> None:
        serial = Script(one(["COM3"]))
        camera = Script(one([]), one(["cam-A"]))
        watcher, arms, cams = self._watch(serial, camera)
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(arms, [])
        self.assertEqual(cams, [True])

    def test_camera_probe_runs_on_divisor_only(self) -> None:
        serial = Script(one(["COM3"]))
        camera = Script(one(["cam-A"]))
        watcher = UsbWatcher(
            serial_fn=serial,
            camera_fn=camera,
            on_camera=lambda _a: None,
            camera_every=4,
            settle_rounds=1,
        )
        for _ in range(8):
            watcher.rescan()
        # ticks 0..7 -> camera probed on 0 and 4 only
        self.assertEqual(camera.calls, 2)
        self.assertEqual(watcher.camera_checks, 2)

    def test_on_camera_defaults_camera_fn(self) -> None:
        watcher = UsbWatcher(
            serial_fn=lambda: one([]),
            on_camera=lambda _a: None,
            settle_rounds=1,
            camera_every=99,
        )
        self.assertIsNotNone(watcher._camera_fn)
        serial = Script(one(["COM3"]))
        camera = Script(one(["cam-A"]))
        watcher = UsbWatcher(serial_fn=serial, camera_fn=camera, settle_rounds=1, camera_every=1)
        for _ in range(3):
            watcher.rescan()
        self.assertEqual(camera.calls, 0)

    # ---- debounce ------------------------------------------------------

    def test_debounce_holds_unstable_change(self) -> None:
        """A board that re-enumerates must not be reported twice."""
        serial = Script(one([]), one(["COM3"]), one(["COM4"]), one(["COM4"]))
        watcher, arms, _cams = self._watch(serial, settle_rounds=2, settle_s=0.0)
        watcher.rescan()  # baseline
        watcher.rescan()  # change seen; re-read says COM4 -> adopt, no emit
        self.assertEqual(arms, [])
        watcher.rescan()  # COM4 == last -> no change
        self.assertEqual(arms, [])
        self.assertEqual(watcher.changes, 1)

    def test_settled_change_is_reported_once(self) -> None:
        serial = Script(one([]), one(["COM3"]), one(["COM3"]))
        watcher, arms, _cams = self._watch(serial, settle_rounds=2, settle_s=0.0)
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(arms, [True])
        watcher.rescan()
        self.assertEqual(arms, [True])

    # ---- robustness ----------------------------------------------------

    def test_callback_exception_does_not_escape(self) -> None:
        def boom(_attached: bool) -> None:
            raise RuntimeError("callback exploded")

        serial = Script(one([]), one(["COM3"]))
        watcher = UsbWatcher(serial_fn=serial, on_arm=boom, settle_rounds=1, camera_every=1)
        watcher.rescan()
        watcher.rescan()  # must not raise
        self.assertEqual(watcher.changes, 1)

    def test_serial_fn_exception_keeps_previous_reading(self) -> None:
        state = {"boom": False}

        def flaky() -> tuple:
            if state["boom"]:
                raise OSError("enum failed")
            return one(["COM3"])

        watcher = UsbWatcher(serial_fn=flaky, on_arm=lambda _a: None, settle_rounds=1)
        watcher.rescan()
        state["boom"] = True
        watcher.rescan()  # must not raise and must not clear the baseline
        self.assertEqual(watcher.checks, 2)
        state["boom"] = False
        watcher.rescan()
        self.assertEqual(watcher.checks, 3)

    def test_camera_fn_exception_is_survived(self) -> None:
        def flaky() -> tuple:
            raise OSError("enum failed")

        serial = Script(one(["COM3"]))
        watcher = UsbWatcher(
            serial_fn=serial, camera_fn=flaky, on_camera=lambda _a: None, settle_rounds=1, camera_every=1
        )
        watcher.rescan()
        watcher.rescan()
        self.assertEqual(watcher.camera_checks, 0)

    def test_start_is_idempotent_and_stop_is_safe(self) -> None:
        serial = Script(one([]))
        watcher, _arms, _cams = self._watch(serial, interval_s=0.05)
        watcher.start()
        watcher.start()
        watcher.stop()
        watcher.stop()
        self.assertIsNone(watcher._thread)

    def test_thread_runs_and_stops(self) -> None:
        serial = Script(one([]))
        watcher, _arms, _cams = self._watch(serial, interval_s=0.05)
        watcher.start()
        watcher.stop(timeout=2.0)
        self.assertGreaterEqual(watcher.checks, 1)


class SignatureTests(unittest.TestCase):
    def test_signature_shape_is_stable(self) -> None:
        self.assertEqual(usb_watch._signature([["b", "a"], []]), (("a", "b"), ()))

    def test_signature_dedupes_and_drops_blanks(self) -> None:
        self.assertEqual(usb_watch._signature([["COM3", "COM3", "  ", ""]]), (("COM3",),))

    def test_serial_ports_never_raises(self) -> None:
        self.assertIsInstance(usb_watch.serial_ports(), list)

    def test_serial_ports_are_openable_by_the_adapter(self) -> None:
        """Report only ports the adapter itself would pick.

        Guards against falling back to the full port list: Windows always has
        Bluetooth/virtual COM ports, and treating those as a debug board would
        switch the station to a live backend that cannot open.
        """
        ports = set(usb_watch.serial_ports())
        try:
            from robot_station.adapters.fafu_arm import import_fafu_sdk, resolve_sdk_root

            import_fafu_sdk(resolve_sdk_root())
            import fafu_motor as pm
        except Exception:
            self.skipTest("fafu_motor unavailable")
        boards = {str(b.port) for b in pm.find_likely_debug_boards()}
        self.assertTrue(
            ports <= boards,
            f"watcher reports ports the adapter would not open: {sorted(ports - boards)}",
        )

    def test_camera_devices_never_raises(self) -> None:
        self.assertIsInstance(usb_watch.camera_devices(), list)

    def test_serial_ports_sorted_unique(self) -> None:
        ports = usb_watch.serial_ports()
        self.assertEqual(ports, sorted(set(ports)))

    def test_signatures_are_one_group(self) -> None:
        self.assertEqual(len(usb_watch.serial_signature()), 1)
        self.assertEqual(len(usb_watch.camera_signature()), 1)
        self.assertEqual(len(usb_watch.device_signature()), 2)

    def test_watch_enabled_defaults_false(self) -> None:
        class Cfg:
            pass

        self.assertEqual(usb_watch.watch_enabled(Cfg()), (False, False))

    def test_watch_enabled_reads_config(self) -> None:
        class Cfg:
            arm_watch = True
            camera_watch = "yes"

        self.assertEqual(usb_watch.watch_enabled(Cfg()), (True, True))

    def test_describe_devices_formats(self) -> None:
        text = usb_watch.describe_devices(ports=[], cameras=[])
        self.assertIn("串口=无", text)
        self.assertIn("相机=无", text)


class ConfigDefaultsTests(unittest.TestCase):
    def test_station_config_watch_flags_default_off(self) -> None:
        from robot_station.config import StationConfig, load_config

        cfg = StationConfig()
        self.assertFalse(cfg.arm_watch)
        self.assertFalse(cfg.camera_watch)
        self.assertFalse(getattr(load_config(), "arm_watch"))

    def test_yaml_flag_is_read_as_bool(self) -> None:
        from robot_station.config import StationConfig

        cfg = StationConfig.from_mapping({"arm_watch": "true", "camera_watch": "0"})
        self.assertTrue(cfg.arm_watch)
        self.assertFalse(cfg.camera_watch)


class MotionWatchWiringTests(unittest.TestCase):
    """The watcher must stay dormant unless cfg turns it on."""

    def _core(self, **flags):
        from robot_station.config import StationConfig
        from robot_station.motion import MotionCore

        return MotionCore(StationConfig(arm="sim", open_browser=False, **flags))

    def test_default_core_has_no_watcher(self) -> None:
        core = self._core()
        self.assertIsNone(core._watch)
        core.start()
        try:
            self.assertIsNone(core._watch)
            core.note_device_change()  # must be a no-op, must not raise
        finally:
            core.stop()
        self.assertIsNone(core._watch)

    def test_flag_starts_and_stops_watcher(self) -> None:
        core = self._core(arm_watch=True, camera_watch=True)
        core.start()
        try:
            self.assertIsNotNone(core._watch)
            core.note_device_change()
        finally:
            core.stop()
        self.assertIsNone(core._watch)

    def test_stop_is_safe_without_start(self) -> None:
        core = self._core(arm_watch=True)
        core.stop()

    def test_watch_arm_keeps_sim_when_no_board(self) -> None:
        core = self._core(arm_watch=True)
        core._watch_arm(False)
        self.assertEqual(core.arm_kind(), "sim")

    def test_watch_arm_refuses_while_estop_latched(self) -> None:
        core = self._core(arm_watch=True, arm_allow_motion=True)
        core.on_estop("test")
        try:
            core._watch_arm(True)
            self.assertEqual(core.arm_kind(), "sim")
        finally:
            core.on_clear_estop("a")

    def test_watch_arm_leaves_connected_adapter_alone(self) -> None:
        """A board re-enumerating mid-session must not disturb the live arm."""
        core = self._core(arm_watch=True, arm_allow_motion=True)
        core.arm.backend = "fafu"
        try:
            core._watch_arm(True)
            self.assertEqual(core.arm_kind(), "fafu")
            core._watch_arm(False)
            self.assertEqual(core.arm_kind(), "fafu")
        finally:
            core.arm.backend = "sim"

    def test_watch_camera_is_safe_without_rescan(self) -> None:
        core = self._core(camera_watch=True)
        core.camera = None
        core._watch_camera(True)  # must not raise

    def test_camera_watch_alone_does_not_start_on_motion_without_camera(self) -> None:
        """Production motion process has camera=None; camera watch belongs to WebFront."""
        core = self._core(camera_watch=True)
        core.start()
        try:
            self.assertIsNone(core._watch)
        finally:
            core.stop()

    def test_in_process_camera_watch_passes_camera_fn(self) -> None:
        from robot_station.adapters.camera import MockCameraBank
        from robot_station.config import StationConfig
        from robot_station.motion import MotionCore

        cam = MockCameraBank(1, 16, 8, 5)
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, camera_watch=True),
            camera=cam,
        )
        core.start()
        try:
            self.assertIsNotNone(core._watch)
            self.assertIsNotNone(core._watch._camera_fn)
            self.assertIsNotNone(core._watch.on_camera)
        finally:
            core.stop()


class CliAndLauncherWiringTests(unittest.TestCase):
    def test_motion_argv_carries_watch_flags(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.service import motion_argv

        cmd = motion_argv(
            StationConfig(arm="fafu", arm_allow_motion=True, arm_watch=True, camera_watch=True),
            None,
        )
        self.assertIn("--arm-watch", cmd)
        self.assertIn("--camera-watch", cmd)

    def test_motion_argv_omits_watch_flags_by_default(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.service import motion_argv

        cmd = motion_argv(StationConfig(arm="sim"), None)
        self.assertNotIn("--arm-watch", cmd)
        self.assertNotIn("--camera-watch", cmd)

    def test_service_flags_reach_the_config(self) -> None:
        """``--arm-watch`` / the env vars must land on the StationConfig."""
        import os
        from unittest import mock

        from robot_station import service
        from robot_station.runtime import StartupPlan

        seen: list = []

        def _capture(cfg, stop):
            seen.append(cfg)

        clean = StartupPlan("start", "test")
        with mock.patch.object(service, "plan_startup", return_value=clean):
            with mock.patch.object(service, "run_motion_process", _capture):
                rc = service.main(["--motion-only", "--arm-watch", "--camera-watch"])
        self.assertEqual(rc, 0)
        self.assertTrue(seen and seen[0].arm_watch and seen[0].camera_watch)

        seen.clear()
        os.environ["STATION_ARM_WATCH"] = "1"
        try:
            with mock.patch.object(service, "plan_startup", return_value=clean):
                with mock.patch.object(service, "run_motion_process", _capture):
                    rc = service.main(["--motion-only"])
        finally:
            os.environ.pop("STATION_ARM_WATCH", None)
        self.assertEqual(rc, 0)
        self.assertTrue(seen and seen[0].arm_watch)

    def test_launcher_sets_watch_env(self) -> None:
        import station_desktop as sd

        source = open(sd.__file__, encoding="utf-8").read()
        self.assertIn('env["STATION_ARM_WATCH"] = "1"', source)
        self.assertIn('env["STATION_CAMERA_WATCH"] = "1"', source)


class WebFrontCameraWatchTests(unittest.TestCase):
    def test_webfront_starts_and_stops_camera_watch(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.service import WebFront

        front = WebFront(StationConfig(camera="mock", camera_watch=True, open_browser=False))
        try:
            front._start_camera_watch()
            self.assertIsNotNone(front._watch)
            self.assertIsNotNone(front._watch.on_camera)
            self.assertIsNotNone(front._watch._camera_fn)
        finally:
            front._stop_camera_watch()
        self.assertIsNone(front._watch)

    def test_webfront_watch_off_by_default(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.service import WebFront

        front = WebFront(StationConfig(camera="mock", open_browser=False))
        front._start_camera_watch()
        self.assertIsNone(front._watch)

    def test_webfront_watch_camera_safe_without_rescan(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.service import WebFront

        front = WebFront(StationConfig(camera="mock", camera_watch=True, open_browser=False))
        front.camera = object()
        front._watch_camera(True)


if __name__ == "__main__":
    unittest.main()
