from __future__ import annotations

import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_station.lock import (
    ExclusiveLock,
    _prepare_lock_file,
    _try_exclusive,
    lock_held,
    port_open,
    wait_lock_free,
)
from robot_station.runtime import RuntimeStatus, plan_startup, should_open_browser
from robot_station.config import StationConfig


def _st(**kwargs) -> RuntimeStatus:
    data = dict(http_up=False, motion_up=False, lock_held=False)
    data.update(kwargs)
    return RuntimeStatus(**data)


class StartupPlanTests(unittest.TestCase):
    def test_idle_starts(self) -> None:
        plan = plan_startup(_st())
        self.assertEqual(plan.action, "start")

    def test_live_station_attaches(self) -> None:
        plan = plan_startup(_st(http_up=True, motion_up=True, lock_held=True))
        self.assertEqual(plan.action, "attach")
        self.assertIn("复用", plan.reason)

    def test_replace_kills_live(self) -> None:
        plan = plan_startup(_st(http_up=True, motion_up=True, lock_held=True), replace=True)
        self.assertEqual(plan.action, "replace_then_start")

    def test_motion_alive_repairs_web(self) -> None:
        plan = plan_startup(_st(motion_up=True, lock_held=True))
        self.assertEqual(plan.action, "web_only")

    def test_web_only_replace_skips_motion_port(self) -> None:
        plan = plan_startup(
            _st(http_up=True, motion_up=True, lock_held=True), replace=True, web_only=True
        )
        self.assertEqual(plan.action, "replace_web_then_start")
        self.assertIn("不杀运动", plan.reason)

    def test_web_only_attaches_if_http_already_up(self) -> None:
        plan = plan_startup(_st(http_up=True, motion_up=True, lock_held=True), web_only=True)
        self.assertEqual(plan.action, "attach")

    def test_wait_port_closed_imports_port_open(self) -> None:
        from robot_station.service import wait_port_closed

        self.assertTrue(wait_port_closed(59998, 0.2))

    def test_http_orphan_refuses(self) -> None:
        plan = plan_startup(_st(http_up=True))
        self.assertEqual(plan.action, "refuse")

    def test_lock_only_refuses(self) -> None:
        plan = plan_startup(_st(lock_held=True))
        self.assertEqual(plan.action, "refuse")

    def test_browser_follows_config(self) -> None:
        cfg = StationConfig(open_browser=True)
        self.assertTrue(should_open_browser(cfg, _st()))
        cfg.open_browser = False
        self.assertFalse(should_open_browser(cfg, _st()))


class LockTests(unittest.TestCase):
    def test_held_and_free(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "t.lock"
            self.assertFalse(lock_held(path))
            with ExclusiveLock(path):
                self.assertTrue(lock_held(path))
            self.assertTrue(wait_lock_free(path, 1.0))
            self.assertFalse(lock_held(path))

    def test_prepare_permission_denied_does_not_raise(self) -> None:
        class Boom:
            def seek(self, *_a: object) -> None:
                return None

            def read(self, *_a: object) -> bytes:
                raise PermissionError("[Errno 13] Permission denied")

            def write(self, *_a: object) -> None:
                raise AssertionError("must not write after read fail")

            def flush(self) -> None:
                return None

            def fileno(self) -> int:
                return 3

        self.assertFalse(_prepare_lock_file(Boom()))
        self.assertFalse(_try_exclusive(Boom()))

    def test_motion_listens_before_core_start_finishes(self) -> None:
        from robot_station.config import StationConfig
        from robot_station.motion import MotionCore, run_motion_process

        gate = threading.Event()
        port = 19471

        def slow_start(self: MotionCore) -> None:
            gate.wait(timeout=4.0)

        stop = threading.Event()
        cfg = StationConfig(arm="mock", motion_port=port, open_browser=False)
        with tempfile.TemporaryDirectory() as raw:
            lock_path = Path(raw) / "motion.lock"
            with patch("robot_station.lock.MOTION_LOCK", lock_path):
                with patch.object(MotionCore, "start", slow_start):
                    th = threading.Thread(
                        target=run_motion_process,
                        args=(cfg, stop),
                        name="test-motion",
                        daemon=True,
                    )
                    th.start()
                    try:
                        deadline = time.monotonic() + 2.0
                        up = False
                        while time.monotonic() < deadline:
                            if port_open(port):
                                up = True
                                break
                            time.sleep(0.05)
                        self.assertTrue(up)
                    finally:
                        gate.set()
                        stop.set()
                        th.join(timeout=3.0)


if __name__ == "__main__":
    unittest.main()
