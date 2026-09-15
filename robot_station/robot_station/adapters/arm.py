from __future__ import annotations

import math
import threading

from robot_station.adapters.fafu_kin import (
    DEFAULT_LIMITS,
    apply_delta_pose,
    clip_cart_rpy_deg,
    clip_cart_xyz,
    forward,
    forward_pose,
    inverse_with_fallback,
    rpy_to_matrix,
)
from robot_station.grav_urdf import list_gravity_urdfs, persist_selected_urdf, resolve_gravity_urdf, selected_urdf_name
from robot_station.world import ArmSnap


class ArmAdapter:
    backend = "mock"

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def hold(self, *, abort_path: bool = False) -> None:
        raise NotImplementedError

    def apply_targets(self, q_deg: list[float], speed_deg_s: float, stream: bool = False) -> str | None:
        raise NotImplementedError

    def set_gripper_open(self, open_: bool) -> None:
        raise NotImplementedError

    def home(self, speed_deg_s: float) -> str | None:
        raise NotImplementedError

    def servo(self, dt: float) -> None:
        raise NotImplementedError

    def poll(self) -> ArmSnap:
        raise NotImplementedError

    def refuse_motion(self) -> str | None:
        """If non-None, MotionCore must not treat UI arm commands as success."""
        return None

    def set_ctrl_mode(self, mode: str) -> str | None:
        return None

    def set_grav_urdf(self, name: str) -> str | None:
        return None

    def teach_pose(self, q_deg: list[float]) -> None:
        self.apply_targets(q_deg, 999.0)

    def apply_cartesian(
        self, dxyz: list[float], drpy_deg: list[float]
    ) -> str | None:
        return "此后端不支持笛卡尔键盘（配置 arm: sim）"

    def apply_cartesian_pose(
        self, xyz_m: list[float], rpy_deg: list[float], speed_deg_s: float
    ) -> str | None:
        return "此后端不支持笛卡尔滑条（配置 arm: sim）"

    def run_path(
        self, waypoints_deg: list[list[float]], speed_deg_s: float, durations_s: list[float] | None = None
    ) -> str | None:
        return "此后端不支持路点路径（配置 arm: sim）"

    def record_start(self, name: str | None = None, teach: str | None = None) -> str | None:
        return "此后端不支持连续录制"

    def delete_traj(self, path: str) -> str | None:
        return "此后端不支持删除轨迹"

    def rename_traj(self, path: str, dest: str) -> str | None:
        return "此后端不支持重命名轨迹"

    def record_stop(self) -> str | None:
        return None

    def replay_start(
        self, path: str, rate: float = 1.0, speed_deg_s: float = 40.0
    ) -> str | None:
        return "此后端不支持轨迹回放"

    def emergency_stop(self) -> None:
        self.hold()

    def resume_motion(self) -> None:
        return None

    def set_powered(self, on: bool) -> str | None:
        return None


class MockArm(ArmAdapter):
    def __init__(self, n: int = 6, default_speed: float = 40.0, has_gripper: bool = True) -> None:
        self.n = int(n)
        self.default_speed = float(default_speed)
        self.has_gripper = bool(has_gripper)
        self._q = [0.0, 40.0, 40.0, 0.0, 0.0, 0.0]
        self._tgt = list(self._q)
        self._spd = [self.default_speed] * self.n
        self._grip_open = True
        self._lock = threading.Lock()
        self._path: list[list[float]] = []
        self._ctrl_mode = "Position"
        self._grav_urdf_name = selected_urdf_name()
        self.limits = [DEFAULT_LIMITS[i] if i < len(DEFAULT_LIMITS) else (-180.0, 180.0) for i in range(self.n)]

    def start(self) -> None:
        return None

    def stop(self) -> None:
        self.hold()

    def hold(self, *, abort_path: bool = False) -> None:
        with self._lock:
            self._tgt = list(self._q)
            if abort_path:
                self._path.clear()

    def _clip(self, i: int, value: float) -> float:
        lo, hi = self.limits[i]
        return max(lo, min(hi, value))

    def apply_targets(self, q_deg: list[float], speed_deg_s: float, stream: bool = False) -> str | None:
        speed = max(1.0, float(speed_deg_s))
        with self._lock:
            for i, raw in enumerate(q_deg[: self.n]):
                val = self._clip(i, float(raw))
                self._tgt[i] = val
                self._spd[i] = speed
                if stream:
                    # 100 Hz follow / cartesian: land this tick. UI speed is for move_j.
                    self._q[i] = val
            if not stream:
                self._path.clear()
        return None

    def set_gripper_open(self, open_: bool) -> None:
        with self._lock:
            self._grip_open = bool(open_)

    def set_gripper_deg(self, deg: float) -> str | None:
        self.set_gripper_open(float(deg) >= 40.0)
        return None

    def home(self, speed_deg_s: float) -> str | None:
        zeros = []
        for i in range(self.n):
            lo, hi = self.limits[i]
            zeros.append(0.0 if lo <= 0.0 <= hi else 0.5 * (lo + hi))
        return self.apply_targets(zeros, speed_deg_s)

    def servo(self, dt: float) -> None:
        dt = max(0.0, min(0.05, float(dt)))
        with self._lock:
            arrived = True
            for i in range(self.n):
                err = self._tgt[i] - self._q[i]
                step = self._spd[i] * dt
                if abs(err) <= step:
                    self._q[i] = self._tgt[i]
                else:
                    self._q[i] += step if err > 0 else -step
                    arrived = False
            if arrived and self._path:
                nxt = self._path.pop(0)
                for i, raw in enumerate(nxt[: self.n]):
                    self._tgt[i] = self._clip(i, float(raw))

    def set_ctrl_mode(self, mode: str) -> str | None:
        name = (mode or "Position").strip()
        self._ctrl_mode = name
        if name in ("Gravity", "Gra+Fri"):
            self.set_gripper_open(True)
        return None

    def set_grav_urdf(self, name: str) -> str | None:
        raw = (name or "").strip()
        if not raw:
            return "请选择 urdf 目录里的文件"
        try:
            path = resolve_gravity_urdf(raw)
        except FileNotFoundError as exc:
            return str(exc)
        persist_selected_urdf(path.name)
        self._grav_urdf_name = path.name
        return None

    def teach_pose(self, q_deg: list[float]) -> None:
        with self._lock:
            for i, raw in enumerate(q_deg[: self.n]):
                val = self._clip(i, float(raw))
                self._q[i] = val
                self._tgt[i] = val
            self._path.clear()

    def apply_cartesian(self, dxyz: list[float], drpy_deg: list[float]) -> str | None:
        with self._lock:
            # Increment the commanded pose. IK from lagged measured q + 40°/s
            # catch-up is what made the default mock arm feel seconds late.
            q_rad = [math.radians(x) for x in self._tgt]
        xyz, rot, _ = forward_pose(q_rad)
        xyz, rot = apply_delta_pose(
            xyz,
            rot,
            [float(dxyz[i] if i < len(dxyz) else 0.0) for i in range(3)],
            [math.radians(float(drpy_deg[i] if i < len(drpy_deg) else 0.0)) for i in range(3)],
        )
        solved = inverse_with_fallback(xyz, seed_rad=q_rad, rotation=rot, is_euler=False)
        if solved is None:
            return "仿真 IK 无解（末端超出工作空间或靠近奇异位）"
        self.apply_targets([math.degrees(x) for x in solved], self.default_speed, stream=True)
        return None

    def apply_cartesian_pose(
        self, xyz_m: list[float], rpy_deg: list[float], speed_deg_s: float
    ) -> str | None:
        xyz = clip_cart_xyz(xyz_m)
        rpy = [math.radians(x) for x in clip_cart_rpy_deg(rpy_deg)]
        with self._lock:
            q_rad = [math.radians(x) for x in self._tgt]
        solved = inverse_with_fallback(
            xyz, seed_rad=q_rad, rotation=rpy_to_matrix(rpy), is_euler=False
        )
        if solved is None:
            return "仿真 IK 无解（末端超出工作空间或靠近奇异位）"
        return self.apply_targets([math.degrees(x) for x in solved], speed_deg_s, stream=False)

    def run_path(
        self, waypoints_deg: list[list[float]], speed_deg_s: float, durations_s: list[float] | None = None
    ) -> str | None:
        if not waypoints_deg:
            return "没有路点"
        first, rest = waypoints_deg[0], waypoints_deg[1:]
        with self._lock:
            self._path = [[self._clip(i, float(p[i])) for i in range(self.n)] for p in rest]
        self.apply_targets(first, speed_deg_s)
        return None

    def poll(self) -> ArmSnap:
        with self._lock:
            moving = any(abs(self._q[i] - self._tgt[i]) > 0.15 for i in range(self.n))
            q_rad = [math.radians(x) for x in self._q]
            xyz, rpy = forward(q_rad)
            return ArmSnap(
                online=True,
                backend="mock",
                enabled=True,
                n=self.n,
                q_deg=list(self._q),
                target_deg=list(self._tgt),
                ok=[True] * self.n,
                gripper_open=self._grip_open,
                has_gripper=self.has_gripper,
                moving=moving,
                ee_m=list(xyz),
                ee_rpy_deg=[math.degrees(x) for x in rpy],
                ctrl_mode=self._ctrl_mode,
                limits=[list(p) for p in self.limits],
                gripper_deg=0.0 if not self._grip_open else 75.0,
                dyn_ready=True,
                float_ok=True,
                float_reason="",
                urdf=self._grav_urdf_name,
                urdf_files=list_gravity_urdfs(),
            )


def build_arm(
    kind: str,
    n: int,
    speed: float,
    has_gripper: bool,
    *,
    sdk: str = "",
    cfg_path: str = "",
    port: str = "",
    allow_motion: bool = False,
    gripper_id: int = 7,
) -> ArmAdapter:
    """Build an arm adapter.

    ``mock`` — Demo 假臂（进程内插补，不走 SDK 方法名）。
    ``sim`` — FafuArm + 可运动 FakeFafuController，方法名与真机相同，不开串口。
    ``fafu`` — 官方 SDK；``start()`` 失败时站控仍起来并切到仿真，避免没电/没应答时打不开页面。
    ``auto`` — 尽量连真臂；失败保持 ``backend=fafu`` 且 offline，不换成假臂
    （避免界面显示假关节却像已连上）。

    ``allow_motion`` on ``fafu``/``auto`` is honoured only when live serial is
    allowed. Without ``STATION_ALLOW_LIVE_ARM=1`` it is forced false (read-only
    if the kind is not remapped earlier). ``arm: sim`` always allows motion.
    """
    name = (kind or "mock").strip().lower()
    if name in ("", "mock"):
        return MockArm(n=n, default_speed=speed, has_gripper=has_gripper)
    if name == "sim":
        from robot_station.adapters.fafu_arm import FafuArm
        from robot_station.adapters.fafu_sim import FakeFafuController

        return FafuArm(
            n=n,
            default_speed=speed,
            has_gripper=has_gripper,
            sdk=sdk,
            cfg_path=cfg_path,
            port=port,
            allow_motion=True,
            gripper_id=gripper_id,
            required=True,
            controller_cls=FakeFafuController,
        )
    if name in ("fafu", "auto"):
        from robot_station.adapters.fafu_arm import FafuArm
        from robot_station.serial_guard import live_serial_allowed

        motion = bool(allow_motion)
        if motion and not live_serial_allowed():
            import logging

            logging.getLogger("station.arm").warning(
                "arm_allow_motion 已忽略：未设置 STATION_ALLOW_LIVE_ARM=1"
            )
            motion = False
        return FafuArm(
            n=n,
            default_speed=speed,
            has_gripper=has_gripper,
            sdk=sdk,
            cfg_path=cfg_path,
            port=port,
            allow_motion=motion,
            gripper_id=gripper_id,
            required=False,
        )
    raise ValueError(f"未知 arm={kind!r}，支持 mock / sim / fafu / auto。")
