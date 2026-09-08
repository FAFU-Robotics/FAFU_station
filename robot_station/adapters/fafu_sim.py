"""In-process fake FafuRobotController. No serial, no USB, no motor commands.

Read-only mode (allow_motion=False): any enable/move/gripper/servo call is an
error. Used by ``python3 -m robot_station.adapters.fafu_arm --sim``.

Motion mode (allow_motion=True): implements the SDK method names the station
adapter will call on the real arm, with 100 Hz interpolation and the same
URDF FK/IK chain as ``fafu_follower.urdf``.
"""
from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace
from typing import Any, Iterable

from robot_station.adapters.fafu_kin import (
    DEFAULT_GRIPPER_LIMITS,
    DEFAULT_LIMITS,
    clip_rad,
    forward_pose,
    inverse,
    inverse_with_fallback,
)

MOTION_METHODS = frozenset(
    {
        "enable",
        "disable",
        "brake",
        "move_j",
        "go_home",
        "open_gripper",
        "close_gripper",
        "gripper_control",
        "servo_j",
        "emergency_stop",
        "set_pos_vel_acc",
        "start_gravity_compensation",
        "apply_compensation_torque",
        "grasp",
        "recover",
    }
)

# Same as SDK go_home: all zeros. J2 lower limit is 0°, so a negative seed is illegal.
_DEFAULT_Q_RAD = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_READY_Q_RAD = [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0]
# 100 Hz servo_j is the next setpoint. Position teleop lands this tick so the
# 3D (q_deg) matches the command; Impedance still uses _speed_scale lag.
# UI "Speed" still applies to move_j / 发送位置, not to the servo stream.
# Live ServoOpts in fafu_arm.py cap real hardware at the UI speed envelope.
# Fake Position teleop still lands this tick (no mechanical hazard).
_SERVO_MAX_VEL = 10.0
_SERVO_MAX_STEP = 0.20
# Between SDK open (~0.3 turns/s) and close (~0.15 turns/s).
_GRIP_DEG_S = 90.0


class FakeMotorState:
    def __init__(self, fault: int = 0, mode: int = 0, torque: int = 0, position: float = 0.0) -> None:
        self.fault = int(fault)
        self.mode = int(mode)
        self.position = float(position)
        self.velocity = 0.0
        self.torque = int(torque)
        self.online = True


class FakeFafuController:
    """Stand-in for FafuRobotController. Records calls; motion is opt-in."""

    last: FakeFafuController | None = None

    def __init__(self, cfg_path: str, **kwargs: Any) -> None:
        self.cfg_path = cfg_path
        self.allow_motion = bool(kwargs.pop("allow_motion", False))
        # True: Gravity is UI teach (teleport). False: blocking compensation loop like the SDK.
        self.sim_teach = True
        self.kwargs = dict(kwargs)
        self.calls: list[str] = []
        self.dyn_ready = False
        self.servo_opts: Any = None
        self.closed = False
        self.state = SimpleNamespace(value="CONNECTED")
        self.joint_motor_ids = [1, 2, 3, 4, 5, 6]
        self.num_joints = 6
        self.is_enabled = False
        self.is_servoing = False
        self.is_gravity_compensating = False
        self.q_rad = list(_DEFAULT_Q_RAD)
        self._tgt_rad = list(_DEFAULT_Q_RAD)
        self._spd_rad_s = math.radians(40.0)
        self._max_step_rad = _SERVO_MAX_STEP
        self._speed_scale = 1.0
        self._path: list[list[float]] = []
        self._estopped = False
        self._mlock = threading.Lock()
        self.v_rad_s = [0.0] * 6
        self.faults = [0] * 6
        self.gripper_open = True
        self.gripper_deg = float(DEFAULT_GRIPPER_LIMITS[1])
        self._grip_tgt_deg = float(self.gripper_deg)
        self.last_gripper_effort = None
        self.tau_raw = [0] * 6
        self.last_tau_nm: list[float] = [0.0] * 6
        self._dyn_motor_models: list[str] | None = None
        self.limits_deg = [DEFAULT_LIMITS[i] for i in range(6)]
        self.gripper_limits_deg = DEFAULT_GRIPPER_LIMITS
        self._cfg = SimpleNamespace(motor_ids=[1, 2, 3, 4, 5, 6, 7], max_torque_raw=300)
        self._missing_motors: list[int] = []
        self._has_gripper = bool(kwargs.get("has_gripper", True))
        self._gripper_motor_id = int(kwargs.get("gripper_motor_id") or 7)
        self._joint_motor_ids = [1, 2, 3, 4, 5, 6]
        if self.allow_motion:
            self.q_rad = list(_READY_Q_RAD)
            self._tgt_rad = list(self.q_rad)
        FakeFafuController.last = self
        if kwargs.get("auto_enable", True):
            raise AssertionError(
                f"模拟调试板要求 auto_enable=False，收到 {kwargs.get('auto_enable')!r}"
            )

    def _note(self, name: str) -> None:
        self.calls.append(name)
        if len(self.calls) > 64:
            del self.calls[:-32]
        if name in MOTION_METHODS and not self.allow_motion:
            raise AssertionError(f"只读模拟禁止调用 {name}")

    def _require_motion(self, name: str) -> None:
        self._note(name)
        if self._estopped and name not in ("emergency_stop", "resume", "enable"):
            raise RuntimeError("仿真急停锁存中")

    def get_joint_values(self, *, prefer_cache: bool = True) -> list[float]:
        self._note("get_joint_values")
        with self._mlock:
            return list(self.q_rad)

    def get_joint_velocities(self, *, prefer_cache: bool = True) -> list[float]:
        with self._mlock:
            return list(self.v_rad_s)

    def get_motor_states(self, *, prefer_cache: bool = True) -> dict[int, FakeMotorState]:
        try:
            enabled = bool(vars(self).get("is_enabled", False))
        except Exception:
            enabled = bool(self.is_enabled)
        mode = 0x0A if enabled else 0x00
        states = {
            mid: FakeMotorState(
                fault=self.faults[i] if i < len(self.faults) else 0,
                torque=self.tau_raw[i] if i < len(self.tau_raw) else 0,
                position=self.q_rad[i] if i < len(self.q_rad) else 0.0,
                mode=mode,
            )
            for i, mid in enumerate(self.joint_motor_ids)
        }
        states[7] = FakeMotorState(position=self.gripper_deg / 360.0, torque=0, mode=mode)
        missing = {int(x) for x in (getattr(self, "_missing_motors", None) or [])}
        for mid in list(states):
            if int(mid) in missing:
                states.pop(mid, None)
        return states

    def close_connection(self, **kwargs: Any) -> None:
        self._note("close_connection")
        self.closed = True

    def enable(self, *, allow_motor_reset: bool = True) -> None:
        self._require_motion("enable")
        if self.is_servoing:
            raise RuntimeError("enable 被拒绝: 机械臂正忙 (state=servoing)。")
        ids = [int(m) for m in list(getattr(getattr(self, "_cfg", None), "motor_ids", []) or [1, 2, 3, 4, 5, 6, 7])]
        missing = {int(x) for x in (getattr(self, "_missing_motors", None) or [])}
        extra = {int(x) for x in (getattr(self, "_station_offline_ids", None) or [])}
        skip = missing | extra
        live = [mid for mid in ids if mid not in skip]
        grip = int(getattr(self, "_gripper_motor_id", 7) or 7)
        if self._has_gripper:
            live = [mid for mid in live if mid != grip]
        if not live:
            raise RuntimeError("没有在线关节电机，无法使能")
        self._estopped = False
        self.is_enabled = True
        with self._mlock:
            self.q_rad = clip_rad(self.q_rad)
            self._tgt_rad = list(self.q_rad)

    def disable(self) -> None:
        self._require_motion("disable")
        self.is_enabled = False
        self.is_servoing = False

    def brake(self) -> None:
        self._require_motion("brake")
        self.hold_position()

    def emergency_stop(self) -> None:
        self._require_motion("emergency_stop")
        self._estopped = True
        self.is_servoing = False
        self.is_enabled = False
        self._path.clear()
        self.hold_position()

    def resume(self) -> None:
        self._note("resume")
        self._estopped = False
        self.is_enabled = True

    def recover(self, *, confirm: bool = False) -> bool:
        self._note("recover")
        if self.state.value == "dead" and not confirm:
            raise RuntimeError("recover(confirm=True) required")
        self.state = SimpleNamespace(value="braked")
        self._estopped = False
        self.is_enabled = False
        return True

    def hold_position(self) -> None:
        self._note("hold_position")
        with self._mlock:
            self._tgt_rad = list(self.q_rad)
            self._path.clear()
            self.v_rad_s = [0.0] * 6

    def teleport(self, joint_angles: Iterable[float], *, is_radians: bool = True) -> None:
        """Gravity teach: snap measured pose. Not a real SDK method."""
        self._note("teleport")
        q = [float(x) for x in joint_angles]
        if not is_radians:
            q = [math.radians(x) for x in q]
        with self._mlock:
            self.q_rad = clip_rad(q)
            self._tgt_rad = list(self.q_rad)
            self.v_rad_s = [0.0] * 6

    def set_speed_scale(self, scale: float) -> None:
        self._speed_scale = max(0.15, min(1.0, float(scale)))
        if self.is_servoing:
            self._apply_servo_opts()

    def move_j(
        self,
        joint_angles: Iterable[float],
        *,
        is_radians: bool = True,
        speed: int = 50,
        block: bool = True,
        tolerance: float = 0.01,
        settle_timeout: float = 1.0,
    ) -> None:
        self._require_motion("move_j")
        if not self.is_enabled:
            raise RuntimeError("仿真未使能")
        q = [float(x) for x in joint_angles]
        if not is_radians:
            q = [math.radians(x) for x in q]
        with self._mlock:
            self._tgt_rad = clip_rad(q)
            self._spd_rad_s = max(0.05, (int(speed) / 100.0) * math.pi) * self._speed_scale
        if block:
            for _ in range(20000):
                if self._at_target(tolerance):
                    return
                self.step(0.01)

    def go_home(self, *, speed: int = 20, block: bool = True) -> None:
        self._require_motion("go_home")
        self.move_j([0.0] * 6, is_radians=True, speed=speed, block=block)

    def setup_dynamics(self, *args: Any, **kwargs: Any) -> None:
        self._note("setup_dynamics")
        self.dyn_ready = True

    def servo_start(self, opts: Any = None) -> None:
        self._note("servo_start")
        if not self.allow_motion:
            raise AssertionError("只读模拟禁止调用 servo_start")
        if self._estopped:
            raise RuntimeError("仿真急停锁存中")
        if not self.is_enabled:
            raise RuntimeError("仿真未使能")
        self.servo_opts = opts
        self.is_servoing = True
        with self._mlock:
            self._path.clear()
        self._apply_servo_opts()

    def _apply_servo_opts(self) -> None:
        opts = self.servo_opts
        vel = float(getattr(opts, "max_vel", _SERVO_MAX_VEL) or _SERVO_MAX_VEL) if opts else _SERVO_MAX_VEL
        step = float(getattr(opts, "max_step_rad", _SERVO_MAX_STEP) or _SERVO_MAX_STEP) if opts else _SERVO_MAX_STEP
        self._spd_rad_s = max(0.05, vel) * self._speed_scale
        self._max_step_rad = max(1e-4, step)

    def servo_j(self, target_angles: Iterable[float]) -> bool:
        self._require_motion("servo_j")
        if not self.is_servoing:
            raise RuntimeError("仿真未开始 servo 会话")
        q = clip_rad([float(x) for x in target_angles])
        with self._mlock:
            self._tgt_rad = q
            self._apply_servo_opts()
            # Position teleop: this tick's command is plant state. Extra
            # max_vel crawl on top of 100 Hz setpoints is software delay.
            if self._speed_scale >= 0.99:
                self.q_rad = list(q)
                self.v_rad_s = [0.0] * 6
        return True

    def servo_end(self, finish_mode: str = "hold") -> None:
        self._note("servo_end")
        self.is_servoing = False
        if finish_mode == "hold":
            self.hold_position()

    def _capture_effort(self, args: tuple[Any, ...], kwargs: dict[str, Any], index: int) -> None:
        if kwargs.get("effort") is not None:
            self.last_gripper_effort = int(kwargs["effort"])
            return
        if len(args) > index and args[index] is not None:
            try:
                self.last_gripper_effort = int(args[index])
            except (TypeError, ValueError):
                pass

    def open_gripper(self, *args: Any, **kwargs: Any) -> None:
        self._require_motion("open_gripper")
        self._capture_effort(args, kwargs, 1)
        with self._mlock:
            self.gripper_open = True
            self._grip_tgt_deg = float(self.gripper_limits_deg[1])

    def close_gripper(self, *args: Any, **kwargs: Any) -> None:
        self._require_motion("close_gripper")
        self._capture_effort(args, kwargs, 1)
        with self._mlock:
            self.gripper_open = False
            self._grip_tgt_deg = float(self.gripper_limits_deg[0])

    def gripper_control(self, angle: float, *args: Any, **kwargs: Any) -> None:
        self._require_motion("gripper_control")
        self._capture_effort(args, kwargs, 0)
        is_radians = bool(kwargs.get("is_radians", True))
        deg = math.degrees(float(angle)) if is_radians else float(angle)
        lo, hi = self.gripper_limits_deg
        mid = 0.5 * (lo + hi)
        with self._mlock:
            self._grip_tgt_deg = max(lo, min(hi, deg))
            self.gripper_open = self._grip_tgt_deg >= mid

    def _step_gripper(self, dt: float) -> None:
        tgt = float(self._grip_tgt_deg)
        cur = float(self.gripper_deg)
        step = _GRIP_DEG_S * dt
        if abs(tgt - cur) <= step:
            self.gripper_deg = tgt
        else:
            self.gripper_deg = cur + (step if tgt > cur else -step)

    def start_gravity_compensation(self, *args: Any, **kwargs: Any) -> None:
        self._require_motion("start_gravity_compensation")
        if not self.is_enabled:
            raise RuntimeError("仿真未使能")
        abort = kwargs.get("abort_check")
        duration = kwargs.get("duration")
        t0 = time.monotonic()
        self.is_gravity_compensating = True
        try:
            while not self._estopped:
                if callable(abort) and abort():
                    break
                if duration is not None and (time.monotonic() - t0) >= float(duration):
                    break
                time.sleep(0.02)
        finally:
            self.is_enabled = False
            self.state = SimpleNamespace(value="braked")
            self.is_gravity_compensating = False

    def set_torque_scale(self, scale: float | Iterable[float] = 1.0) -> None:
        self._note("set_torque_scale")
        self._torque_scale = scale

    def apply_compensation_torque(self, tau: Iterable[float], *, damping_kd: float = 0.0) -> None:
        self._require_motion("apply_compensation_torque")
        vals = [float(x) for x in tau]
        n = int(getattr(self, "num_joints", 6) or 6)
        if len(vals) != n:
            raise ValueError(f"tau must have {n} elements, got ({len(vals)},)")
        self.last_tau_nm = vals[:6] + [0.0] * max(0, 6 - len(vals))
        self.tau_raw = [int(round(x * 100.0)) for x in self.last_tau_nm]

    def get_limit(self, motor_id: int, *, is_radians: bool = True) -> tuple[float, float] | None:
        self._note("get_limit")
        mid = int(motor_id)
        if 1 <= mid <= 6:
            lo, hi = self.limits_deg[mid - 1]
        elif mid == 7:
            lo, hi = self.gripper_limits_deg
        else:
            return None
        if is_radians:
            return math.radians(lo), math.radians(hi)
        return lo, hi

    def move_jntspace_path(
        self,
        path: Any,
        *,
        is_radians: bool = True,
        speed: int = 50,
        **kwargs: Any,
    ) -> None:
        self._note("move_jntspace_path")
        if not self.allow_motion:
            raise AssertionError("只读模拟禁止调用 move_jntspace_path")
        if self._estopped:
            raise RuntimeError("仿真急停锁存中")
        rows = [list(row) for row in path]
        out: list[list[float]] = []
        for row in rows:
            q = [float(x) for x in row]
            if not is_radians:
                q = [math.radians(x) for x in q]
            out.append(clip_rad(q))
        if not out:
            raise ValueError("path must not be empty")
        with self._mlock:
            self._spd_rad_s = max(0.05, (int(speed) / 100.0) * math.pi) * self._speed_scale
            self._path = out[1:]
            self._tgt_rad = out[0]

    def get_pose(self, *, prefer_cache: bool = True) -> tuple[list[float], list[list[float]]]:
        """Match FafuRobotController.get_pose: (xyz m, 3×3 rotation)."""
        with self._mlock:
            q = list(self.q_rad)
        xyz, rot, _rpy = forward_pose(q)
        return xyz, rot

    def forward_kinematics(
        self,
        q: Iterable[float] | None = None,
        *,
        is_radians: bool = True,
    ) -> dict[str, object]:
        raw = list(self.q_rad if q is None else q)
        if not is_radians:
            raw = [math.radians(x) for x in raw]
        xyz, rot, rpy = forward_pose(raw)
        return {"position": xyz, "rotation": rot, "rpy": rpy, "q": raw}

    def inverse_kinematics(
        self,
        target_position: Iterable[float],
        target_rotation: Any = None,
        *,
        is_euler: bool = False,
        is_radians: bool = True,
        init_q: Iterable[float] | None = None,
        **kwargs: Any,
    ) -> list[float] | None:
        self._note("inverse_kinematics")
        xyz = [float(x) for x in target_position]
        seed = list(init_q) if init_q is not None else list(self.q_rad)
        if not is_radians and init_q is not None:
            seed = [math.radians(x) for x in seed]
        extra = dict(kwargs)
        if "ori_weight" in extra:
            return inverse(
                xyz,
                seed_rad=seed,
                rotation=target_rotation,
                is_euler=is_euler,
                is_radians=is_radians,
                ori_weight=float(extra["ori_weight"]),
            )
        return inverse_with_fallback(
            xyz,
            seed_rad=seed,
            rotation=target_rotation,
            is_euler=is_euler,
            is_radians=is_radians,
        )

    def step(self, dt: float) -> None:
        """Advance interpolation. Called from FafuArm.servo at control_hz."""
        with self._mlock:
            dt = max(0.0, min(0.05, float(dt)))
            if not self.allow_motion or self._estopped or not self.is_enabled:
                self.v_rad_s = [0.0] * 6
                return
            self._step_gripper(dt)
            if self.is_servoing and self._speed_scale >= 0.99:
                # Position teleop / path vertices already landed in servo_j.
                self.v_rad_s = [0.0] * 6
                return
            spd = self._spd_rad_s * dt
            if self.is_servoing:
                spd = min(spd, self._max_step_rad)
            moved = False
            for i in range(6):
                err = self._tgt_rad[i] - self.q_rad[i]
                if abs(err) <= spd:
                    self.q_rad[i] = self._tgt_rad[i]
                    self.v_rad_s[i] = 0.0
                else:
                    self.q_rad[i] += spd if err > 0 else -spd
                    self.v_rad_s[i] = self._spd_rad_s if err > 0 else -self._spd_rad_s
                    moved = True
            if not moved and self._path:
                self._tgt_rad = self._path.pop(0)

    def _at_target(self, tolerance: float = 0.01) -> bool:
        with self._mlock:
            return all(abs(self.q_rad[i] - self._tgt_rad[i]) <= tolerance for i in range(6))

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)

        def _forbidden(*_a: Any, **_k: Any) -> None:
            self._note(name)
            raise AssertionError(f"只读模拟禁止调用 {name}")

        return _forbidden
