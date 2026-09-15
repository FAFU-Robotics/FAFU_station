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

    def test_native_openable_path_keeps_ascii(self) -> None:
        import tempfile
        from pathlib import Path

        from robot_station.adapters.fafu_arm import native_openable_path

        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "robot.cfg"
            src.write_text("port = auto\n", encoding="utf-8")
            if not str(src).isascii():
                self.skipTest("临时目录本身含非 ASCII 字符")
            self.assertEqual(native_openable_path(src), str(src.resolve()))

    def test_native_openable_path_copies_non_ascii(self) -> None:
        import tempfile
        from pathlib import Path

        from robot_station.adapters.fafu_arm import native_openable_path

        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "配置目录" / "robot.cfg"
            src.parent.mkdir(parents=True, exist_ok=True)
            src.write_text("port = auto\nmotor_ids = 1,2,3\n", encoding="utf-8")
            out = native_openable_path(src)
            self.assertTrue(out.isascii(), out)
            self.assertTrue(Path(out).is_file())
            self.assertEqual(Path(out).read_text(encoding="utf-8"), src.read_text(encoding="utf-8"))

    def test_robot_config_load_via_native_path(self) -> None:
        from robot_station.adapters.fafu_arm import default_cfg_path, native_openable_path

        try:
            root = resolve_sdk_root()
        except FileNotFoundError:
            self.skipTest("本机未安装 fafu_arm_sdk")
        try:
            pm, _controller = import_fafu_sdk(root)
        except RuntimeError as exc:
            self.skipTest(str(exc))
        cfg = default_cfg_path(root)
        loaded = pm.RobotConfig.load(native_openable_path(cfg))
        self.assertTrue(list(loaded.motor_ids))

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

    def test_hardware_link_down_when_rx_stale(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _hardware_link_up, _station_stream_link_ok

        class Stats:
            last_rx_age_ms = 5000.0

        ht = SimpleNamespace(
            is_open=lambda: True,
            is_async_rx=lambda: True,
            get_stats=lambda: Stats(),
        )
        stale = SimpleNamespace(
            state=SimpleNamespace(value="IDLE"),
            _ht=ht,
            _dead_rx_timeout_ms=500.0,
            sim_teach=False,
            _station_had_rx=True,
        )
        self.assertFalse(_hardware_link_up(stale))
        self.assertFalse(_station_stream_link_ok(stale))

        class Never:
            last_rx_age_ms = -1.0

        ht_never = SimpleNamespace(
            is_open=lambda: True,
            is_async_rx=lambda: True,
            get_stats=lambda: Never(),
            get_cached_state=lambda _mid: None,
        )
        waking = SimpleNamespace(
            state=SimpleNamespace(value="IDLE"),
            _ht=ht_never,
            _dead_rx_timeout_ms=500.0,
            sim_teach=False,
            _joint_motor_ids=[1, 2, 3, 4, 5, 6],
        )
        self.assertTrue(_hardware_link_up(waking))
        self.assertTrue(_station_stream_link_ok(waking))
        self.assertFalse(bool(getattr(waking, "_station_had_rx", False)))

        class Fresh:
            last_rx_age_ms = 40.0

        ht_ok = SimpleNamespace(
            is_open=lambda: True,
            is_async_rx=lambda: True,
            get_stats=lambda: Fresh(),
        )
        live = SimpleNamespace(
            state=SimpleNamespace(value="IDLE"),
            _ht=ht_ok,
            _dead_rx_timeout_ms=500.0,
            sim_teach=False,
            _stream_link_ok=lambda: True,
        )
        self.assertTrue(_hardware_link_up(live))
        self.assertTrue(_station_stream_link_ok(live))

        fake = SimpleNamespace(state=SimpleNamespace(value="IDLE"), _ht=None, sim_teach=True)
        self.assertTrue(_hardware_link_up(fake))
        no_ht = SimpleNamespace(state=SimpleNamespace(value="IDLE"), _ht=None, sim_teach=False)
        self.assertTrue(_hardware_link_up(no_ht))

    def test_poll_offline_when_usb_rx_stale(self) -> None:
        from types import SimpleNamespace

        class Motor:
            online = True
            fault = 0
            mode = 0x0A
            position = 0.0
            torque = 0

        class Stats:
            last_rx_age_ms = 5000.0

        class Ht:
            def is_open(self) -> bool:
                return True

            def is_async_rx(self) -> bool:
                return True

            def get_stats(self) -> Stats:
                return Stats()

            def get_cached_state(self, _mid: int) -> Motor:
                return Motor()

        class Dummy:
            joint_motor_ids = [1, 2, 3, 4, 5, 6]
            sim_teach = False
            _dead_rx_timeout_ms = 500.0
            _tgt_rad = None
            closed = False
            _station_had_rx = True

            def __init__(self) -> None:
                self._ht = Ht()
                self.state = SimpleNamespace(value="IDLE")

            def get_motor_states(self, prefer_cache: bool = True) -> dict:
                return {i: Motor() for i in range(1, 7)}

            def get_joint_values(self, prefer_cache: bool = True) -> list[float]:
                return [0.0] * 6

            def get_joint_velocities(self, prefer_cache: bool = True) -> list[float]:
                return [0.0] * 6

            def close_connection(self, **_kwargs: object) -> None:
                self.closed = True

        arm = FafuArm(required=False, allow_motion=False, has_gripper=True)
        dummy = Dummy()
        arm._robot = dummy
        snap = arm.poll()
        self.assertFalse(snap.online)
        self.assertTrue(all(not m.get("online") for m in snap.motors))
        self.assertIsNone(arm._robot)
        self.assertTrue(dummy.closed)

    def test_poll_online_when_usb_rx_fresh(self) -> None:
        from types import SimpleNamespace

        class Motor:
            online = True
            fault = 0
            mode = 0x0A
            position = 0.0
            torque = 0

        class Stats:
            last_rx_age_ms = 40.0

        class Ht:
            def is_open(self) -> bool:
                return True

            def is_async_rx(self) -> bool:
                return True

            def get_stats(self) -> Stats:
                return Stats()

            def get_cached_state(self, _mid: int) -> Motor:
                return Motor()

        class Dummy:
            joint_motor_ids = [1, 2, 3, 4, 5, 6]
            sim_teach = False
            _dead_rx_timeout_ms = 500.0
            _tgt_rad = None
            is_enabled = False

            def __init__(self) -> None:
                self._ht = Ht()
                self.state = SimpleNamespace(value="IDLE")

            def _stream_link_ok(self) -> bool:
                return True

            def get_motor_states(self, prefer_cache: bool = True) -> dict:
                return {i: Motor() for i in range(1, 7)}

            def get_joint_values(self, prefer_cache: bool = True) -> list[float]:
                return [0.0] * 6

            def get_joint_velocities(self, prefer_cache: bool = True) -> list[float]:
                return [0.0] * 6

            def close_connection(self, **_kwargs: object) -> None:
                raise AssertionError("fresh USB must not drop the link")

        arm = FafuArm(required=False, allow_motion=False, has_gripper=True)
        dummy = Dummy()
        arm._robot = dummy
        snap = arm.poll()
        self.assertTrue(snap.online)
        self.assertIs(arm._robot, dummy)
        self.assertEqual(sum(1 for m in snap.motors if m.get("name") != "夹爪" and m.get("online")), 6)

    def test_poll_keeps_link_when_bus_never_rx(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _live_rx_age_ms

        class Stats:
            last_rx_age_ms = -1.0

        class Ht:
            def is_open(self) -> bool:
                return True

            def is_async_rx(self) -> bool:
                return True

            def get_stats(self) -> Stats:
                return Stats()

            def get_cached_state(self, _mid: int) -> None:
                return None

        class Dummy:
            joint_motor_ids = [1, 2, 3, 4, 5, 6]
            sim_teach = False
            _dead_rx_timeout_ms = 500.0
            _tgt_rad = None
            closed = False
            _joint_motor_ids = [1, 2, 3, 4, 5, 6]

            def __init__(self) -> None:
                self._ht = Ht()
                self.state = SimpleNamespace(value="IDLE")

            def get_motor_states(self, prefer_cache: bool = True) -> dict:
                raise AssertionError("live poll must not blocking-read an empty cache")

            def get_joint_values(self, prefer_cache: bool = True) -> list[float]:
                raise AssertionError("live poll must not blocking-read joints")

            def get_joint_velocities(self, prefer_cache: bool = True) -> list[float]:
                raise AssertionError("live poll must not blocking-read velocities")

            def close_connection(self, **_kwargs: object) -> None:
                self.closed = True

        dummy = Dummy()
        self.assertIsNone(_live_rx_age_ms(dummy))
        arm = FafuArm(required=False, allow_motion=False, has_gripper=True)
        arm._robot = dummy
        snap = arm.poll()
        self.assertTrue(snap.online)
        self.assertIs(arm._robot, dummy)
        self.assertFalse(dummy.closed)
        self.assertTrue(all(not m.get("online") for m in snap.motors if m.get("name") != "夹爪"))


if __name__ == "__main__":
    unittest.main()
