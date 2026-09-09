from __future__ import annotations

import inspect
import os
import time
import unittest

from pathlib import Path

from robot_station.adapters.fafu_arm import FafuArm, import_fafu_sdk, probe_link, resolve_sdk_root
from robot_station.adapters.fafu_sim import MOTION_METHODS, FakeFafuController
from robot_station.config import StationConfig, _as_bool
from robot_station.motion import MotionCore

ROOT = Path(__file__).resolve().parents[1]


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

    def test_missing_motor_does_not_mark_arm_offline(self) -> None:
        arm = FafuArm(required=True, allow_motion=True, controller_cls=FakeFafuController)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake.enable()
            first = arm.poll()
            self.assertTrue(first.online)
            orig_states = fake.get_motor_states

            def no_j3(**_k: object) -> dict:
                states = orig_states()
                states.pop(3, None)
                return states

            def boom(*_a: object, **_k: object) -> list[float]:
                raise RuntimeError("no feedback from motor 3")

            fake.get_joint_values = boom  # type: ignore[method-assign]
            fake.get_joint_velocities = boom  # type: ignore[method-assign]
            fake.get_motor_states = no_j3  # type: ignore[method-assign]
            snap = arm.poll()
            self.assertTrue(snap.online)
            motors = {int(m["id"]): m for m in (snap.motors or [])}
            self.assertFalse(motors[3]["online"])
            self.assertTrue(motors[1]["online"])
            self.assertTrue(motors[7]["online"])
            self.assertTrue(snap.enabled)
        finally:
            arm.stop()

    def test_missing_gripper_keeps_link(self) -> None:
        arm = FafuArm(required=True, allow_motion=True, controller_cls=FakeFafuController)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            orig_states = fake.get_motor_states

            def no_grip(**_k: object) -> dict:
                states = orig_states()
                states.pop(7, None)
                return states

            fake.get_motor_states = no_grip  # type: ignore[method-assign]
            snap = arm.poll()
            self.assertTrue(snap.online)
            grip = [m for m in snap.motors if m.get("name") == "夹爪"]
            self.assertTrue(grip)
            self.assertFalse(grip[0]["online"])
            self.assertFalse((snap.link_err or "").startswith("离线："))
        finally:
            arm.stop()

    def test_enable_succeeds_without_gripper(self) -> None:
        arm = FafuArm(required=True, allow_motion=True, controller_cls=FakeFafuController)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake._missing_motors = [7]
            self.assertIsNone(arm.set_powered(True))
            fake.enable()
            snap = arm.poll()
            self.assertTrue(snap.enabled)
            self.assertEqual(list(fake._cfg.motor_ids), [1, 2, 3, 4, 5, 6])
            self.assertIsNone(arm.apply_targets([12.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0))
            err = arm.set_gripper_open(True)
            self.assertIsNotNone(err)
            self.assertIn("夹爪", err or "")
        finally:
            arm.stop()

    def test_vendor_enable_noise_cleared_when_joints_enabled(self) -> None:
        core = MotionCore(
            StationConfig(arm="sim", open_browser=False, control_hz=50, watchdog_s=2.0)
        )
        core.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake._missing_motors = [7]
            self.assertIsNone(core.arm.set_powered(True))
            core._cmd_err = (
                "使能失败: enable failed even after motor_reset; check the diagnostic above."
            )
            core.arm._writer_err = core._cmd_err
            deadline = time.monotonic() + 1.2
            while time.monotonic() < deadline:
                err = str(core.snapshot().get("cmd_err") or "")
                if "motor_reset" not in err.lower() and "使能失败" not in err:
                    break
                time.sleep(0.04)
            err = str(core.snapshot().get("cmd_err") or "")
            self.assertNotIn("motor_reset", err.lower(), err)
            self.assertNotIn("使能失败", err)
            self.assertTrue(core.snapshot()["arm"]["enabled"])
        finally:
            core.stop()

    def test_enable_succeeds_without_one_joint(self) -> None:
        arm = FafuArm(required=True, allow_motion=True, controller_cls=FakeFafuController)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake._missing_motors = [3]
            self.assertIsNone(arm.set_powered(True))
            snap = arm.poll()
            self.assertTrue(snap.enabled)
            self.assertNotIn(3, [int(m) for m in fake._cfg.motor_ids])
            motors = {int(m["id"]): m for m in (snap.motors or [])}
            self.assertFalse(motors[3]["online"])
            self.assertTrue(motors[1]["online"])
            self.assertIsNone(arm.apply_targets([12.0, 40.0, 40.0, 0.0, 0.0, 0.0], 40.0))
        finally:
            arm.stop()

    def test_enable_fails_when_no_joints_live(self) -> None:
        arm = FafuArm(required=True, allow_motion=True, controller_cls=FakeFafuController)
        arm.start()
        try:
            fake = FakeFafuController.last
            assert fake is not None
            fake._missing_motors = [1, 2, 3, 4, 5, 6, 7]
            err = arm.set_powered(True)
            self.assertIsNotNone(err)
            self.assertIn("没有在线", err or "")
        finally:
            arm.stop()

    def test_soft_precheck_warns_instead_of_raising(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _with_soft_precheck

        class Dummy:
            def _precheck_communication(self) -> None:
                raise RuntimeError("motors [7] did not respond within 500ms")

        wrapped = _with_soft_precheck(Dummy)
        obj = wrapped.__new__(wrapped)
        obj._ht = SimpleNamespace(read_motor_state=lambda mid, _t: None if int(mid) == 7 else object())
        obj._cfg = SimpleNamespace(motor_ids=[1, 2, 3, 4, 5, 6, 7])
        obj._precheck_communication()
        self.assertEqual(obj._missing_motors, [7])
        self.assertEqual(list(obj._cfg.motor_ids), [1, 2, 3, 4, 5, 6])

    def test_wrapped_enable_skips_offline_gripper(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _with_soft_precheck

        switched: list[int] = []

        class Motor:
            def __init__(self, mode: int = 0) -> None:
                self.mode = mode
                self.online = True

        cache = {i: Motor(0) for i in range(1, 7)}

        class Ht:
            def read_motor_state(self, mid: int, _t: float) -> Motor | None:
                if int(mid) == 7:
                    return None
                return cache.get(int(mid))

            def get_cached_state(self, mid: int) -> Motor | None:
                return cache.get(int(mid))

            def set_motor_mode(self, mid: int, mode: int) -> Motor:
                if int(mid) == 7:
                    raise AssertionError("must not enable gripper")
                switched.append(int(mid))
                cache[int(mid)] = Motor(mode)
                return cache[int(mid)]

            def motor_reset(self, mid: int) -> None:
                raise AssertionError(f"must not reset motor {mid}")

        class Dummy:
            MODE_POSITION = 0x0A

            def _precheck_communication(self) -> None:
                raise RuntimeError("motors [7] did not respond within 500ms")

            def _set_state(self, new: object) -> None:
                self._state = new

            def get_motor_states(self, prefer_cache: bool = True) -> dict[int, Motor]:
                return dict(cache)

            def enable(self, **_k: object) -> None:
                raise AssertionError("vendor enable must not run")

            def servo_start(self, opts: object = None) -> None:
                self.servo_slots = int(max(self._cfg.motor_ids))
                self._servo_max_motor_id = self.servo_slots
                self.started = True

        class St:
            value = "disabled"
            IDLE = "idle"

        wrapped = _with_soft_precheck(Dummy)
        obj = wrapped.__new__(wrapped)
        obj._ht = Ht()
        obj._cfg = SimpleNamespace(motor_ids=[1, 2, 3, 4, 5, 6, 7])
        obj._joint_motor_ids = [1, 2, 3, 4, 5, 6]
        obj._gripper_motor_id = 7
        obj._state = St()
        obj._precheck_communication()
        obj.enable()
        self.assertNotIn(7, switched)
        self.assertEqual(sorted(set(switched)), [1, 2, 3, 4, 5, 6])
        self.assertTrue(obj.is_enabled)
        obj.servo_start()
        self.assertTrue(obj.started)
        self.assertEqual(obj.servo_slots, 6)
        self.assertEqual(obj._servo_max_motor_id, 6)
        self.assertNotIn(7, list(obj._cfg.motor_ids))

    def test_silent_gripper_dropped_before_vendor_enable(self) -> None:
        from robot_station.adapters.fafu_arm import _install_gripper_offline_gate

        class Ht:
            def get_cached_state(self, mid: int) -> object | None:
                return None if int(mid) == 7 else object()

            def start_state_polling(self, ids: list[int], _hz: float) -> None:
                self.polled = [int(x) for x in ids]

        class Dummy:
            def enable(self, **_k: object) -> None:
                ids = list(self._joint_motor_ids)
                if self._has_gripper and self._gripper_online:
                    ids.append(int(self._gripper_motor_id))
                if 7 in ids:
                    raise RuntimeError(
                        "enable failed even after motor_reset; check the diagnostic above."
                    )
                self.ok = True

            def servo_start(self, opts: object = None) -> None:
                self.enable()
                self.started = True

        gated = _install_gripper_offline_gate(Dummy)
        obj = gated.__new__(Dummy)
        obj._ht = Ht()
        obj._has_gripper = True
        obj._gripper_online = True
        obj._gripper_motor_id = 7
        obj._joint_motor_ids = [1, 2, 3, 4, 5, 6]
        obj._missing_motors = []
        obj.enable()
        self.assertFalse(obj._gripper_online)
        self.assertTrue(obj.ok)
        obj.servo_start()
        self.assertTrue(obj.started)
        self.assertIn(7, obj._missing_motors)

    def test_faulted_gripper_dropped_before_vendor_enable(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _install_gripper_offline_gate

        class Ht:
            def get_cached_state(self, mid: int) -> object | None:
                if int(mid) == 7:
                    return SimpleNamespace(online=False, fault=1, mode=0)
                return SimpleNamespace(online=True, fault=0, mode=0x0A)

            def start_state_polling(self, ids: list[int], _hz: float) -> None:
                self.polled = [int(x) for x in ids]

        class Dummy:
            def enable(self, **_k: object) -> None:
                ids = list(self._joint_motor_ids)
                if self._has_gripper and self._gripper_online:
                    ids.append(int(self._gripper_motor_id))
                if 7 in ids:
                    raise RuntimeError(
                        "enable failed even after motor_reset; check the diagnostic above."
                    )
                self.ok = True

            def servo_start(self, opts: object = None) -> None:
                if not self._gripper_online:
                    self.ok = True
                    self.started = True
                    self._servo_max_motor_id = max(self._joint_motor_ids)
                    return
                self.enable()
                self.started = True

        gated = _install_gripper_offline_gate(Dummy)
        obj = gated.__new__(Dummy)
        obj._ht = Ht()
        obj._has_gripper = True
        obj._gripper_online = True
        obj._gripper_motor_id = 7
        obj._joint_motor_ids = [1, 2, 3, 4, 5, 6]
        obj._missing_motors = []
        obj._station_offline_ids = [7]
        obj.servo_start()
        self.assertFalse(obj._gripper_online)
        self.assertTrue(obj.ok)
        self.assertTrue(obj.started)
        self.assertEqual(obj._servo_max_motor_id, 6)

    def test_stream_link_ok_ignores_silent_gripper(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _with_soft_precheck

        class Motor:
            def __init__(self, mode: int = 0x0A) -> None:
                self.mode = mode

        cache = {i: Motor() for i in range(1, 7)}

        class Stats:
            last_rx_age_ms = 50.0

        class Ht:
            def is_async_rx(self) -> bool:
                return True

            def get_stats(self) -> Stats:
                return Stats()

            def get_cached_state(self, mid: int) -> Motor | None:
                return cache.get(int(mid))

        class Dummy:
            MODE_POSITION = 0x0A
            _dead_rx_timeout_ms = 500.0

            def _enter_dead(self, reason: str) -> None:
                raise AssertionError(f"must not latch DEAD: {reason}")

            def _stream_link_ok(self) -> bool:
                self._enter_dead("no CAN RX for 5000 ms")
                return False

        wrapped = _with_soft_precheck(Dummy)
        obj = wrapped.__new__(wrapped)
        obj._ht = Ht()
        obj._cfg = SimpleNamespace(motor_ids=[1, 2, 3, 4, 5, 6, 7])
        obj._joint_motor_ids = [1, 2, 3, 4, 5, 6]
        obj._gripper_motor_id = 7
        obj._missing_motors = [7]
        self.assertTrue(obj._stream_link_ok())

    def test_stream_link_ok_false_when_usb_silent(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _with_soft_precheck

        class Motor:
            def __init__(self, mode: int = 0x0A) -> None:
                self.mode = mode

        cache = {i: Motor() for i in range(1, 7)}
        dead: list[str] = []

        class Stats:
            last_rx_age_ms = 5000.0

        class Ht:
            def is_async_rx(self) -> bool:
                return True

            def get_stats(self) -> Stats:
                return Stats()

            def get_cached_state(self, mid: int) -> Motor | None:
                return cache.get(int(mid))

        class Dummy:
            MODE_POSITION = 0x0A
            _dead_rx_timeout_ms = 500.0

            def _enter_dead(self, reason: str) -> None:
                dead.append(reason)

            def _stream_link_ok(self) -> bool:
                self._enter_dead("vendor would latch DEAD")
                return False

        wrapped = _with_soft_precheck(Dummy)
        obj = wrapped.__new__(wrapped)
        obj._ht = Ht()
        obj._cfg = SimpleNamespace(motor_ids=[1, 2, 3, 4, 5, 6, 7])
        obj._joint_motor_ids = [1, 2, 3, 4, 5, 6]
        obj._gripper_motor_id = 7
        obj._missing_motors = [7]
        self.assertFalse(obj._stream_link_ok())
        self.assertTrue(dead)

    def test_vendor_enable_impl_does_not_reset_offline_gripper(self) -> None:
        """SDK enable() calls _enable_impl on every cfg.motor_ids, including M7."""
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _with_soft_precheck

        switched: list[int] = []

        class Motor:
            def __init__(self, mode: int = 0) -> None:
                self.mode = mode
                self.online = True

        cache = {i: Motor(0) for i in range(1, 7)}

        class Ht:
            def read_motor_state(self, mid: int, _t: float) -> Motor | None:
                if int(mid) == 7:
                    return None
                return cache.get(int(mid))

            def get_cached_state(self, mid: int) -> Motor | None:
                return cache.get(int(mid))

            def set_motor_mode(self, mid: int, mode: int) -> Motor:
                if int(mid) == 7:
                    raise AssertionError("must not enable gripper")
                switched.append(int(mid))
                cache[int(mid)] = Motor(mode)
                return cache[int(mid)]

            def motor_reset(self, mid: int) -> None:
                raise AssertionError(f"must not reset motor {mid}")

        class Dummy:
            MODE_POSITION = 0x0A

            def _precheck_communication(self) -> None:
                return

            def _set_state(self, new: object) -> None:
                self._state = new

            def get_motor_states(self, prefer_cache: bool = True) -> dict[int, Motor]:
                return dict(cache)

            def enable(self, *, allow_motor_reset: bool = True) -> None:
                self._enable_impl(allow_motor_reset=allow_motor_reset)

            def _enable_impl(self, *, allow_motor_reset: bool = True) -> None:
                raise RuntimeError(
                    "enable failed even after motor_reset; check the diagnostic above. "
                    "Likely causes: (a) motor controller in latched FAULT state -> hard power-cycle; "
                    "(b) USB-CAN bus disconnected / wrong COM port; "
                    "(c) mechanical jam holding the joint outside soft limits."
                )

        class St:
            value = "disabled"
            IDLE = "idle"

        wrapped = _with_soft_precheck(Dummy)
        obj = wrapped.__new__(wrapped)
        obj._ht = Ht()
        obj._cfg = SimpleNamespace(motor_ids=[1, 2, 3, 4, 5, 6, 7])
        obj._joint_motor_ids = [1, 2, 3, 4, 5, 6]
        obj._gripper_motor_id = 7
        obj._state = St()
        obj._missing_motors = [7]
        # Unbound parent enable() is what SDK servo_start uses when
        # is_enabled is False: it must dispatch to our _enable_impl.
        Dummy.enable(obj)
        self.assertNotIn(7, list(obj._cfg.motor_ids))
        self.assertNotIn(7, switched)
        self.assertEqual(sorted(set(switched)), [1, 2, 3, 4, 5, 6])
        self.assertTrue(obj.is_enabled)

    def test_cfg_motor_ids_proxy_when_assignment_ignored(self) -> None:
        from types import SimpleNamespace

        from robot_station.adapters.fafu_arm import _CfgMotorIds, _set_cfg_motor_ids

        class Frozen:
            def __init__(self) -> None:
                self._ids = [1, 2, 3, 4, 5, 6, 7]

            @property
            def motor_ids(self) -> list[int]:
                return list(self._ids)

            @motor_ids.setter
            def motor_ids(self, _v: list[int]) -> None:
                return

        robot = SimpleNamespace()
        robot._cfg = Frozen()
        _set_cfg_motor_ids(robot, [1, 2, 3, 4, 5, 6])
        self.assertIsInstance(robot._cfg, _CfgMotorIds)
        self.assertEqual(list(robot._cfg.motor_ids), [1, 2, 3, 4, 5, 6])

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

        self.assertTrue(callable(getattr(cls, "_controlled_motor_ids", None)))
        src = inspect.getsource(cls._precheck_communication)
        self.assertIn("gripper", src.lower())
        self.assertIn("_mark_gripper_offline", src)
        src_servo = inspect.getsource(cls.servo_start)
        self.assertIn("frame_ids", src_servo)
        self.assertIn("_joint_motor_ids", src_servo)
        src_ids = inspect.getsource(cls._controlled_motor_ids)
        self.assertIn("_gripper_online", src_ids)
        from types import SimpleNamespace

        obj = SimpleNamespace(
            _joint_motor_ids=[1, 2, 3, 4, 5, 6],
            _has_gripper=True,
            _gripper_online=False,
            _gripper_motor_id=7,
        )
        self.assertEqual(cls._controlled_motor_ids(obj), [1, 2, 3, 4, 5, 6])
        obj._gripper_online = True
        self.assertEqual(cls._controlled_motor_ids(obj), [1, 2, 3, 4, 5, 6, 7])
        obj._cfg = SimpleNamespace(motor_ids=[1, 2, 3, 4, 5, 6])
        obj._ht = SimpleNamespace(get_cached_state=lambda _mid: None)
        self.assertEqual(cls._controlled_motor_ids(obj), [1, 2, 3, 4, 5, 6])

    def test_allow_motion_string_false_is_false(self) -> None:
        self.assertFalse(_as_bool("false"))
        self.assertFalse(_as_bool("False"))
        cfg = StationConfig.from_mapping({"arm_allow_motion": "false"})
        self.assertIs(cfg.arm_allow_motion, False)
        from robot_station.adapters.arm import build_arm

        os.environ.pop("STATION_ALLOW_LIVE_ARM", None)
        arm = build_arm("fafu", 6, 40.0, True, allow_motion=True)
        self.assertFalse(arm.allow_motion)

    def test_station_code_rev_is_stable_hash(self) -> None:
        from robot_station.runtime import station_code_rev

        a = station_code_rev(ROOT)
        b = station_code_rev(ROOT)
        self.assertEqual(len(a), 12)
        self.assertEqual(a, b)

    def test_connect_reports_failure_when_controller_missing(self) -> None:
        class Boom:
            def __init__(self, *args: object, **kwargs: object) -> None:
                raise RuntimeError("motors [7] did not respond within 500ms")

        core = MotionCore(StationConfig(arm="fafu", open_browser=False))
        core.arm = FafuArm(required=False, allow_motion=True, controller_cls=Boom)
        err = core.on_link("c", "connect")
        self.assertIsNotNone(err)
        self.assertIn("motors", err or "")
        self.assertIsNone(core.arm._robot)

    def test_yaml_station_stays_mock(self) -> None:
        from robot_station.config import load_config

        cfg = load_config()
        self.assertEqual(cfg.arm, "mock")
        self.assertFalse(cfg.arm_allow_motion)


if __name__ == "__main__":
    unittest.main()
