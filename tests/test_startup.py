from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from robot_station.lock import ExclusiveLock, lock_held, wait_lock_free
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


if __name__ == "__main__":
    unittest.main()
