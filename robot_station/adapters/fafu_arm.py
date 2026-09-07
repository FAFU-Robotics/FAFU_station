"""Official FAFU arm adapter.

Construction always uses ``auto_enable=False``. ``allow_motion=False`` (live
``arm: fafu``) is read-only: no enable / move_j / servo_j. ``arm: sim`` injects
FakeFafuController with ``allow_motion=True`` and uses the same method names
the real controller will later receive.

Live motion requires both ``STATION_ALLOW_LIVE_ARM=1`` (serial) and
``allow_motion=True`` (commands). Gravity on the real SDK is a background
``start_gravity_compensation`` loop; Fake keeps slider teach via ``teleport``.
"""
from __future__ import annotations

import logging
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

from robot_station.adapters.arm import ArmAdapter
from robot_station.adapters.fafu_kin import (
    DEFAULT_GRIPPER_EFFORT,
    DEFAULT_GRIPPER_LIMITS,
    DEFAULT_LIMITS,
    apply_delta_pose,
    as_rotation,
    clip_cart_rpy_deg,
    clip_cart_xyz,
    clip_gripper_effort,
    forward_pose,
    inverse_with_fallback,
    load_cfg_limits,
    load_gripper_effort,
    rpy_to_matrix,
    step_cartesian,
    xyz_err,
)
from robot_station.world import ArmSnap

logger = logging.getLogger("station.arm.fafu")

REQUIRED_CORE_ABI = 4
FLOAT_MODES = frozenset({"Gravity", "Gra+Fri", "Impedance"})
POSITION_MODES = frozenset({"Position", "Impedance"})

# Official Fafu 6-DoF table used by fafu_arm_sdk gravity tests. Empty models make
# tau_to_raw raise "unknown motor model" and the gravity loop dies on the first tick.
FAFU_JOINT_MOTOR_MODELS = (
    "M5036_02",
    "M6036_02",
    "M6036_02",
    "M5036_02",
    "M4438_30",
    "M4438_30",
)
FAFU_TAU_LIMIT_NM = (15.0, 30.0, 30.0, 15.0, 5.0, 5.0)

# Live teleop envelope. UI Speed is the command rate; ServoOpts are a hardware
# cap. The old 10 rad/s / 0.20 rad-per-tick values let the arm sprint ~570–1100
# deg/s on a slider jump or IK step — that is not "tracking", it is unsafe.
LIVE_TELEOP_MIN_DEG_S = 5.0
LIVE_TELEOP_MAX_DEG_S = 120.0
LIVE_SERVO_RATE_HZ = 100.0
HOME_MAX_DEG_S = 55.0
HOME_ARRIVE_RAD = 0.04
# Tool height at URDF zero is ≈ 0.17 m. Joint-lerp below this scrapes the table.
HOME_Z_FLOOR_M = 0.14
# Stay below this unless the start pose is already higher. The old 95° tuck +
# J2→0 pose pointed the upper arm at the ceiling (tool z ≈ 0.60–0.74 m).
HOME_Z_CEIL_M = 0.40
HOME_J3_SEARCH_MAX_DEG = 130.0
# Ignore leftover follow only when the slider has not actually moved.
# 3° was large enough that a slow drag sat still for ~1 s, then jumped.
STREAM_RESTART_DEG = 0.35
# UI sends touch every 100 ms, which feeds SafetyGate and would keep a
# leftover servo_j session alive forever after WASD / 跟随 stop. This
# timeout is counted from the last *stream target*, not from touch.
STREAM_IDLE_S = 0.35
# Firmware brakes (no holding torque) if no servo_j arrives within
# watchdog_ms. 150 ms was shorter than keyboard/IK hitches, so a stutter
# at a raised J2 dropped the arm. Keep this well above one slow tick.
SERVO_WATCHDOG_MS = 400
CART_POSE_TOL_M = 0.004


def _motor_mode_name(mode: int) -> str:
    names = {0x00: "stop", 0x0A: "position", 0x0B: "mit", 0x0F: "brake"}
    code = int(mode)
    return names.get(code, f"0x{code:02X}")


def motor_position_to_deg(raw: float) -> float:
    """SDK MotorState.position is protocol-native turns, not radians."""
    return float(raw) * 360.0


def rotation_to_rpy(ori: Any) -> list[float] | None:
    """Return RPY radians from a 3-vector or 3x3 matrix (pinocchio matrixToRpy)."""
    try:
        seq = list(ori)
    except TypeError:
        return None
    if len(seq) == 3:
        scalars = True
        out: list[float] = []
        for x in seq:
            if isinstance(x, (list, tuple)):
                scalars = False
                break
            if hasattr(x, "shape") and getattr(x, "ndim", 0) > 0:
                scalars = False
                break
            try:
                out.append(float(x))
            except (TypeError, ValueError):
                scalars = False
                break
        if scalars:
            return out
    if len(seq) != 3:
        return None
    rows: list[list[float]] = []
    for row in seq:
        try:
            cells = list(row)
        except TypeError:
            return None
        if len(cells) != 3:
            return None
        rows.append([float(x) for x in cells])
    r02 = rows[2][0]
    sy = math.hypot(rows[2][1], rows[2][2])
    if sy > 1e-6:
        roll = math.atan2(rows[2][1], rows[2][2])
        pitch = math.atan2(-r02, sy)
        yaw = math.atan2(rows[1][0], rows[0][0])
    else:
        roll = math.atan2(-rows[1][2], rows[1][1])
        pitch = math.atan2(-r02, sy)
        yaw = 0.0
    return [roll, pitch, yaw]


def resolve_sdk_root(explicit: str = "") -> Path:
    from robot_station.portable import is_portable, sdk_path_allowed

    raw = (explicit or os.environ.get("FAFU_ARM_SDK") or "").strip()
    if raw:
        path = Path(raw).expanduser().resolve()
        if is_portable() and not sdk_path_allowed(path):
            raw = ""
        elif not path.is_dir():
            raise FileNotFoundError(f"arm_sdk 目录不存在: {path}")
        else:
            return path
    repo = Path(__file__).resolve().parents[2]
    if is_portable():
        candidates = (
            repo / "vendor" / "fafu_arm_sdk",
            repo / "vendor" / "fafu_arm_sdk-main",
        )
    else:
        home = Path.home()
        candidates = (
            repo / "vendor" / "fafu_arm_sdk",
            repo / "vendor" / "fafu_arm_sdk-main",
            repo.parent / "fafu_arm_sdk",
            repo.parent / "fafu_arm_sdk-main",
            home / "fafu_arm_sdk",
            home / "fafu_arm_sdk-main",
            home / "zx" / "fafu_arm_sdk-main",
        )
    for path in candidates:
        if path.is_dir() and (path / "fafu_robot_python").is_dir():
            return path
    raise FileNotFoundError(
        "找不到 fafu_arm_sdk。把 SDK 放到仓库旁，或设置 arm_sdk / 环境变量 FAFU_ARM_SDK。"
    )


def import_fafu_sdk(sdk_root: Path | None = None) -> tuple[Any, Any]:
    """Load fafu_motor + FafuRobotController for this interpreter. No serial I/O."""
    root = sdk_root if sdk_root is not None else resolve_sdk_root()
    py_dir = root / "fafu_robot_python"
    if not py_dir.is_dir():
        raise FileNotFoundError(f"SDK Python 目录不存在: {py_dir}")
    inserted = str(py_dir)
    if inserted not in sys.path:
        sys.path.insert(0, inserted)
    try:
        import fafu_motor as pm  # type: ignore
    except ImportError as exc:
        tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
        native = f"fafu_motor.{tag}-win_amd64.pyd" if os.name == "nt" else f"fafu_motor.{tag}-*-linux-gnu.so"
        have = sorted(p.name for p in py_dir.glob("fafu_motor.*"))
        raise RuntimeError(
            f"当前 Python {sys.version.split()[0]} 载不入 fafu_motor "
            f"（需要 {native}）。目录里现有: {have or '无'}。"
            " 官方预编译通常是 cp310；本机 3.11+ 须用 Python 3.10 启动站控，"
            "或在 fafu_robot_cpp 里对当前解释器重编 .pyd。"
            f" 原始错误: {exc}"
        ) from exc
    abi = int(getattr(pm, "CORE_ABI_VERSION", 0) or 0)
    # GitHub 当前 SDK 已不再导出 CORE_ABI_VERSION；用 TORQUE_COEFF 判断是否与
    # fafu_robot_controller.py 配套。旧站控时代的模块则仍认 ABI 4。
    if not hasattr(pm, "HightorqueSerial"):
        raise RuntimeError("fafu_motor 不是官方调试板模块（缺少 HightorqueSerial）")
    if not hasattr(pm, "TORQUE_COEFF") and abi != REQUIRED_CORE_ABI:
        raise RuntimeError(
            f"fafu_motor 过旧（无 TORQUE_COEFF，ABI={abi}）。"
            "请在 fafu_robot_cpp 对当前 Python 重编后覆盖 fafu_robot_python 里的 .pyd。"
        )
    from fafu_robot_controller import FafuRobotController  # type: ignore

    return pm, FafuRobotController


def default_cfg_path(sdk_root: Path | None = None) -> Path:
    root = sdk_root if sdk_root is not None else resolve_sdk_root()
    return root / "fafu_robot_python" / "robot.cfg"


def default_urdf_path(sdk_root: Path | None = None) -> Path | None:
    """Official SDK now vendors URDFs under fafu_robot_description/urdf/."""
    try:
        root = sdk_root if sdk_root is not None else resolve_sdk_root()
    except FileNotFoundError:
        return None
    desc = root / "fafu_robot_python" / "fafu_robot_description"
    preferred = (
        desc / "urdf" / "fafu_baseV1.urdf",
        desc / "fafu_follower.urdf",
        desc / "urdf" / "fafu_follower.urdf",
    )
    for path in preferred:
        if path.is_file():
            return path
    folder = desc / "urdf"
    if folder.is_dir():
        found = sorted(folder.glob("*.urdf"))
        if found:
            return found[0]
    return None


def list_sdk_scripts(sdk_root: Path | str | None = None) -> list[str]:
    """One-level .py files under examples/ and tests/, Host-style listing."""
    try:
        raw = str(sdk_root).strip() if sdk_root is not None else ""
        root = resolve_sdk_root(raw)
    except FileNotFoundError:
        return ["go_home.py"]
    names: list[str] = []
    for folder in ("examples", "tests"):
        directory = root / "fafu_robot_python" / folder
        if not directory.is_dir():
            continue
        for path in sorted(directory.glob("*.py")):
            if path.name.startswith("_"):
                continue
            names.append(path.name)
    seen: set[str] = set()
    out: list[str] = []
    for name in names:
        if name not in seen:
            seen.add(name)
            out.append(name)
    return out or ["go_home.py"]


def map_script(name: str) -> str | None:
    """Map a listed script to a station op. None = refuse (would steal the serial)."""
    key = Path(name or "").name.lower()
    if key in {"go_home.py", "home.py"}:
        return "home"
    if key in {"03_gripper.py", "gripper.py"}:
        return "grip_open"
    if key in {"06_emergency_stop.py", "emergency_stop.py"}:
        return "estop"
    return None


def _robot_state_name(robot: Any) -> str:
    state = getattr(robot, "state", None)
    if state is None:
        return ""
    value = getattr(state, "value", state)
    return str(value).lower()


class FafuArm(ArmAdapter):
    """ArmAdapter wrapping FafuRobotController or FakeFafuController."""

    backend = "fafu"

    def __init__(
        self,
        n: int = 6,
        default_speed: float = 40.0,
        has_gripper: bool = True,
        *,
        sdk: str = "",
        cfg_path: str = "",
        port: str = "",
        allow_motion: bool = False,
        gripper_id: int = 7,
        required: bool = True,
        controller_cls: Any | None = None,
    ) -> None:
        self.n = int(n)
        self.default_speed = float(default_speed)
        self.has_gripper = bool(has_gripper)
        self.allow_motion = bool(allow_motion)
        self.gripper_id = int(gripper_id)
        self.required = bool(required)
        self._sdk_arg = sdk
        self._cfg_arg = (cfg_path or "").strip()
        self.port = (port or "").strip()
        self._controller_cls = controller_cls
        self.backend = "sim" if (controller_cls is not None and allow_motion) else "fafu"
        self._sdk_root: Path | None = None
        self._robot: Any = None
        self._lock = threading.Lock()
        self._link_error = ""
        self._servoing = False
        self._ctrl_mode = "Position"
        self._grip_open = True
        self._gripper_deg = float(DEFAULT_GRIPPER_LIMITS[1])
        self._target_deg = [0.0] * self.n
        self._hold_latched = False
        self._dyn_ready = False
        self._writer_err = ""
        self._grav_stop = threading.Event()
        self._grav_th: threading.Thread | None = None
        self._grav_err = ""
        # Native servo_start/servo_j/servo_end must share one OS thread (SDK BusyError
        # otherwise). The 100 Hz MotionCore.servo() tick is that thread.
        self._servo_intent = False
        self._stream_q_rad: list[float] | None = None
        self._servo_cmd_rad: list[float] | None = None
        self._teleop_deg_s = max(LIVE_TELEOP_MIN_DEG_S, float(self.default_speed) or 40.0)
        self._servo_dt = 1.0 / LIVE_SERVO_RATE_HZ
        self._freeze_needed = False
        self._servo_idle = threading.Event()
        self._servo_idle.set()
        self._pending_grip: tuple[str, Any] | None = None
        self._gripper_effort = DEFAULT_GRIPPER_EFFORT
        self._writer_busy = False
        self._servo_opts: Any = None
        self._servo_opts_ready = False
        self._parked = False
        self._home_path: list[list[float]] = []
        self._stream_rx_mono = 0.0
        self.limits = [DEFAULT_LIMITS[i] if i < len(DEFAULT_LIMITS) else (-180.0, 180.0) for i in range(self.n)]
        self.gripper_limits = DEFAULT_GRIPPER_LIMITS
        cfg_file = self._cfg_arg or ""
        if not cfg_file:
            try:
                cfg_file = str(default_cfg_path(resolve_sdk_root(self._sdk_arg) if self._sdk_arg else None))
            except FileNotFoundError:
                cfg_file = ""
        if cfg_file:
            self.limits, self.gripper_limits = load_cfg_limits(cfg_file, self.n)
            self._gripper_effort = load_gripper_effort(cfg_file)
        self._last = ArmSnap(
            online=False,
            backend=self.backend,
            enabled=False,
            n=self.n,
            q_deg=[0.0] * self.n,
            target_deg=[0.0] * self.n,
            ok=[False] * self.n,
            gripper_open=True,
            has_gripper=self.has_gripper,
            moving=False,
            ctrl_mode=self._ctrl_mode,
            limits=[list(p) for p in self.limits],
            gripper_limits=list(self.gripper_limits),
            gripper_deg=self._gripper_deg,
            gripper_effort=self._gripper_effort,
        )

    def _is_fake(self) -> bool:
        robot = self._robot
        return robot is not None and bool(getattr(robot, "sim_teach", False))

    def _use_sdk_float(self) -> bool:
        """Real gravity/impedance loops. Fake keeps slider teach."""
        robot = self._robot
        return (
            self.allow_motion
            and robot is not None
            and not bool(getattr(robot, "sim_teach", False))
            and callable(getattr(robot, "start_gravity_compensation", None))
        )

    def _float_modes_available(self) -> bool:
        """Gravity / Gra+Fri / Impedance: sim teach, or live SDK dynamics."""
        if self._use_sdk_float():
            return bool(self._dyn_ready)
        return bool(self._is_fake() or self.backend == "sim")

    def _float_modes_reason(self) -> str:
        if self._float_modes_available():
            return ""
        return "需 pinocchio"

    def _float_writer_active(self) -> bool:
        th = self._grav_th
        if th is not None and th.is_alive():
            return True
        robot = self._robot
        if robot is None:
            return False
        flag = getattr(type(robot), "is_gravity_compensating", None)
        if not isinstance(flag, property):
            return False
        try:
            return bool(flag.__get__(robot, type(robot)))
        except Exception:
            return False

    def refuse_motion(self) -> str | None:
        if self._robot is None:
            if self._link_error:
                return f"机械臂未连接（{self._link_error}）"
            return "机械臂未连接"
        if not self.allow_motion:
            return "真臂当前只读（arm_allow_motion=false），不会下发运动或使能"
        return None

    def start(self) -> None:
        if self._robot is not None:
            return
        try:
            if self._controller_cls is not None:
                controller_cls = self._controller_cls
                cfg_path = self._cfg_arg or "sim://fake"
                sim = True
            else:
                from robot_station.serial_guard import live_serial_allowed

                if not live_serial_allowed():
                    raise RuntimeError(
                        "未允许真机串口（未设置 STATION_ALLOW_LIVE_ARM=1）。"
                        "请用 arm: sim，或设置环境变量后用 arm: fafu。"
                    )
                self._sdk_root = resolve_sdk_root(self._sdk_arg)
                _pm, controller_cls = import_fafu_sdk(self._sdk_root)
                cfg_path = self._cfg_arg or str(default_cfg_path(self._sdk_root))
                sim = False
            if cfg_path and not str(cfg_path).startswith("sim:"):
                self.limits, self.gripper_limits = load_cfg_limits(cfg_path, self.n)
                self._gripper_effort = load_gripper_effort(cfg_path)
            kwargs: dict[str, Any] = {
                "cfg_path": cfg_path,
                "has_gripper": self.has_gripper,
                "auto_enable": False,
                "auto_polling": True,
            }
            if sim:
                kwargs["allow_motion"] = self.allow_motion
            if self.has_gripper:
                kwargs["gripper_motor_id"] = self.gripper_id
            if self.port:
                kwargs["port"] = self.port
            logger.info(
                "连接 FAFU 臂（auto_enable=False%s%s）cfg=%s port=%s",
                "，模拟" if sim else "",
                "，可运动" if self.allow_motion else "，只读",
                cfg_path,
                self.port or "cfg/auto",
            )
            robot = controller_cls(**kwargs)
            self._apply_limits_from_robot(robot)
            self._setup_dynamics(robot, sim=sim)
            with self._lock:
                self._robot = robot
                self._link_error = ""
                self._hold_latched = False
            self._make_servo_opts()
            logger.info(
                "FAFU 臂%s已开 state=%s joints=%s enabled=%s allow_motion=%s dyn=%s",
                "模拟 " if sim else "串口",
                getattr(getattr(robot, "state", None), "value", "?"),
                list(getattr(robot, "joint_motor_ids", [])),
                self._flag_enabled(robot),
                self.allow_motion,
                self._dyn_ready,
            )
        except Exception as exc:
            with self._lock:
                self._robot = None
                self._link_error = str(exc)
            logger.error("FAFU 臂连接失败（未使能、未下发运动）: %s", exc)
            if self.required:
                raise RuntimeError(f"arm=fafu 连接失败: {exc}") from exc

    def _apply_limits_from_robot(self, robot: Any) -> None:
        ids = list(getattr(robot, "joint_motor_ids", []))[: self.n]
        if not ids or not hasattr(robot, "get_limit"):
            return
        for i, mid in enumerate(ids):
            try:
                lim = robot.get_limit(int(mid), is_radians=True)
            except Exception:
                continue
            if not lim:
                continue
            self.limits[i] = (math.degrees(float(lim[0])), math.degrees(float(lim[1])))
        if self.has_gripper:
            try:
                lim = robot.get_limit(int(self.gripper_id), is_radians=True)
            except Exception:
                lim = None
            if lim:
                self.gripper_limits = (math.degrees(float(lim[0])), math.degrees(float(lim[1])))

    def _setup_dynamics(self, robot: Any, *, sim: bool) -> None:
        self._dyn_ready = False
        setup = getattr(robot, "setup_dynamics", None)
        if not callable(setup):
            self._dyn_ready = bool(sim)
            return
        try:
            kwargs: dict[str, Any] = {}
            urdf = default_urdf_path(self._sdk_root)
            if urdf is not None:
                kwargs["urdf_path"] = str(urdf)
            if not sim and self.n == len(FAFU_JOINT_MOTOR_MODELS):
                kwargs["motor_models"] = list(FAFU_JOINT_MOTOR_MODELS)
                kwargs["tau_limit"] = list(FAFU_TAU_LIMIT_NM)
                kwargs["torque_scale"] = 1.0
            setup(**kwargs)
            self._dyn_ready = True
        except Exception as exc:
            self._dyn_ready = bool(sim)
            logger.warning("setup_dynamics 失败（重力/阻抗需 pinocchio+URDF；笛卡尔走站控 IK）: %s", exc)

    def stop(self) -> None:
        self._servo_intent = False
        self._pending_grip = None
        self._stop_gravity(join=True)
        self._wait_servo_idle(1.0)
        self._end_servo()
        with self._lock:
            robot = self._robot
            self._robot = None
        if robot is None:
            return
        try:
            robot.close_connection(joint_release="hold", gripper_release="hold")
        except TypeError:
            try:
                robot.close_connection()
            except Exception:
                logger.exception("关闭 FAFU 臂串口失败")
        except Exception:
            logger.exception("关闭 FAFU 臂串口失败")

    def _use_live_servo_envelope(self) -> bool:
        """Real USB arm, not FakeFafuController."""
        return self.backend != "sim" and not self._is_fake()

    def _speed_pct(self, speed_deg_s: float) -> int:
        return max(1, min(100, int(round(float(speed_deg_s) / 180.0 * 100.0))))

    def set_teleop_speed(self, speed_deg_s: float) -> None:
        self._teleop_deg_s = max(
            LIVE_TELEOP_MIN_DEG_S, min(LIVE_TELEOP_MAX_DEG_S, float(speed_deg_s))
        )

    def _live_servo_caps(self) -> tuple[float, float]:
        """Hardware envelope at the UI slider maximum, not the latency-sprint values."""
        cap = math.radians(LIVE_TELEOP_MAX_DEG_S)
        max_vel = cap * 1.15
        max_step = cap / LIVE_SERVO_RATE_HZ * 1.25
        return max_vel, max_step

    def _slew_live_cmd(self, goal: list[float], dt: float) -> list[float]:
        """Advance the sent pose toward ``goal`` at UI speed. Live only."""
        dt = max(1e-4, min(0.05, float(dt)))
        max_step = math.radians(self._teleop_deg_s) * dt
        cur = self._servo_cmd_rad
        robot = self._robot
        if cur is None and robot is not None:
            try:
                cur = [float(x) for x in list(robot.get_joint_values(prefer_cache=True))[: self.n]]
            except Exception:
                cur = None
        if cur is None:
            cur = list(goal)
        while len(cur) < self.n:
            cur.append(0.0)
        out: list[float] = []
        for i in range(self.n):
            g = float(goal[i]) if i < len(goal) else 0.0
            c = float(cur[i])
            delta = g - c
            if abs(delta) > max_step:
                delta = math.copysign(max_step, delta)
            out.append(c + delta)
        self._servo_cmd_rad = self._q_rad(self._clip_deg([math.degrees(x) for x in out]))
        return self._servo_cmd_rad

    def _near_q(self, a: list[float] | None, b: list[float] | None, tol: float = HOME_ARRIVE_RAD) -> bool:
        if not a or not b:
            return False
        n = min(self.n, len(a), len(b))
        return all(abs(float(a[i]) - float(b[i])) <= tol for i in range(n))

    def _note_stream(self) -> None:
        self._stream_rx_mono = time.monotonic()

    def _hold_stream_pose(self) -> None:
        """Keep 100 Hz servo_j at the last commanded pose. Do not servo_end.

        Official firmware brakes (no holding torque) if a servo session goes
        silent for watchdog_ms. Ending the session on key-up / 发送位置到位
        was dropping a raised arm. Same-pose servo_j writes feedforward vel 0.
        """
        hold = self._servo_cmd_rad or self._stream_q_rad
        if hold is None:
            robot = self._robot
            if robot is not None:
                try:
                    hold = [float(x) for x in list(robot.get_joint_values(prefer_cache=True))[: self.n]]
                except Exception:
                    hold = None
        if not hold:
            self._servo_intent = False
            self._parked = True
            self._freeze_needed = True
            return
        self._stream_q_rad = list(hold)
        self._target_deg = [math.degrees(x) for x in hold[: self.n]]
        self._servo_intent = True
        self._parked = True
        self._freeze_needed = False
        self._note_stream()

    def _dedupe_waypoints(self, pts: list[list[float]]) -> list[list[float]]:
        out: list[list[float]] = []
        for pose in pts:
            pose = self._clip_deg(pose)
            if out and max(abs(pose[i] - out[-1][i]) for i in range(self.n)) < 6.0:
                out[-1] = pose
            else:
                out.append(pose)
        return out or [[0.0] * self.n]

    def _tool_z_m(self, q_deg: list[float]) -> float:
        xyz, _rot, _rpy = forward_pose(self._q_rad(q_deg))
        return float(xyz[2])

    def _joint_lerp_z_range(self, a_deg: list[float], b_deg: list[float], steps: int = 14) -> tuple[float, float]:
        a = self._clip_deg(a_deg)
        b = self._clip_deg(b_deg)
        zmin, zmax = 9.0, -9.0
        n = max(1, int(steps))
        for k in range(n + 1):
            t = k / n
            q = [a[i] + (b[i] - a[i]) * t for i in range(self.n)]
            z = self._tool_z_m(q)
            zmin = min(zmin, z)
            zmax = max(zmax, z)
        return zmin, zmax

    def _lerp_clears_table(self, a_deg: list[float], b_deg: list[float]) -> bool:
        zmin, _zmax = self._joint_lerp_z_range(a_deg, b_deg)
        return zmin >= HOME_Z_FLOOR_M

    def _min_j3_for_table_clear(self, q_deg: list[float], goal_deg: list[float]) -> float:
        """Smallest J3 ≥ current that keeps the joint-space lerp above the table."""
        q = self._clip_deg(q_deg)
        lo = float(q[2])
        hi_lim = self.limits[2][1] if len(self.limits) > 2 else HOME_J3_SEARCH_MAX_DEG
        hi = min(float(hi_lim), max(lo, HOME_J3_SEARCH_MAX_DEG))
        if self._lerp_clears_table(q, goal_deg):
            return lo
        best = hi
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            cand = list(q)
            cand[2] = mid
            cand[4] = 0.0
            cand[5] = 0.0
            if self._lerp_clears_table(cand, goal_deg):
                best = mid
                hi = mid
            else:
                lo = mid
        return best

    def _max_abs_delta(self, a: list[float], b: list[float]) -> float:
        n = min(self.n, len(a), len(b))
        if n <= 0:
            return 0.0
        return max(abs(float(a[i]) - float(b[i])) for i in range(n))

    def _interp_q(self, a: list[float], b: list[float], t: float) -> list[float]:
        t = max(0.0, min(1.0, float(t)))
        out: list[float] = []
        for i in range(self.n):
            ai = float(a[i]) if i < len(a) else 0.0
            bi = float(b[i]) if i < len(b) else 0.0
            out.append(ai + (bi - ai) * t)
        return out

    def _shoulder_safe_waypoints(self, q_now_deg: list[float], q_goal_deg: list[float]) -> list[list[float]]:
        """If lowering a raised J2, raise J3 only as far as the table check needs."""
        now = self._clip_deg(q_now_deg)
        goal = self._clip_deg(q_goal_deg)
        pts: list[list[float]] = []
        lowering = float(now[1]) > 55.0 and float(goal[1]) < min(float(now[1]) - 8.0, 52.0)
        if lowering and not self._lerp_clears_table(now, goal):
            via = list(now)
            via[2] = self._min_j3_for_table_clear(now, goal)
            via[4] = 0.0
            via[5] = 0.0
            pts.append(via)
        pts.append(goal)
        return self._dedupe_waypoints(pts)

    def _home_waypoints_deg(self, q_deg: list[float]) -> list[list[float]]:
        """Pose-dependent retract to zero. Do not send all joints to 0 in one shot.

        If a straight joint-space lerp already stays above the table, use it.
        Otherwise raise J3 only as far as that lerp needs, then go to zero along
        that line. The 100 Hz writer interpolates each segment with synchronized
        joints, so the elbow cannot arrive at 0 while the shoulder is still high
        (gripper into desk). Never pass through the vertical 95° / J2=0 pose
        (gripper into ceiling).
        """
        q = self._clip_deg(q_deg)
        home = [0.0] * self.n
        chain: list[list[float]] = []
        if not self._lerp_clears_table(q, home):
            lift = list(q)
            lift[2] = self._min_j3_for_table_clear(q, home)
            lift[4] = 0.0
            lift[5] = 0.0
            chain.append(lift)
        chain.append(home)
        out = self._dedupe_waypoints(chain)
        if not out or max(abs(x) for x in out[-1]) > 0.5:
            out.append(self._clip_deg(home))
        return out

    def _measured_deg(self) -> list[float]:
        robot = self._robot
        if robot is not None:
            try:
                return [math.degrees(float(x)) for x in robot.get_joint_values(prefer_cache=True)[: self.n]]
            except Exception:
                pass
        if self._servo_cmd_rad:
            return [math.degrees(x) for x in self._servo_cmd_rad]
        return list(self._target_deg)

    def _start_joint_path(self, waypoints_rad: list[list[float]], speed_deg_s: float) -> None:
        wps = [list(p) for p in waypoints_rad if p]
        if not wps:
            wps = [self._q_rad(self._measured_deg())]
        self.set_teleop_speed(speed_deg_s)
        self._home_path = wps
        self._parked = False
        self._hold_latched = False
        self._freeze_needed = False
        self._servo_intent = True
        # Keep the current command as the stream pose. Pointing servo_j at the
        # first vertex immediately makes later independent-axis slew twitch.
        if self._servo_cmd_rad is not None:
            self._stream_q_rad = list(self._servo_cmd_rad)
        self._target_deg = [math.degrees(x) for x in wps[-1]]
        self._note_stream()

    @property
    def is_homing(self) -> bool:
        return bool(self._home_path)

    def _q_rad(self, q_deg: list[float]) -> list[float]:
        vals = [math.radians(float(x)) for x in q_deg[: self.n]]
        while len(vals) < self.n:
            vals.append(0.0)
        return vals

    def _clip_deg(self, q_deg: list[float]) -> list[float]:
        out: list[float] = []
        for i in range(self.n):
            raw = float(q_deg[i]) if i < len(q_deg) else 0.0
            lo, hi = self.limits[i] if i < len(self.limits) else (-180.0, 180.0)
            out.append(max(lo, min(hi, raw)))
        return out

    def _make_servo_opts(self) -> Any:
        if self._servo_opts_ready:
            return self._servo_opts
        try:
            root = self._sdk_root
            if root is None:
                try:
                    root = resolve_sdk_root(self._sdk_arg)
                except FileNotFoundError:
                    root = None
            if root is not None:
                import_fafu_sdk(root)
            from fafu_robot_controller import ServoOpts  # type: ignore

            if self._use_live_servo_envelope():
                max_vel, max_step = self._live_servo_caps()
            else:
                max_vel, max_step = 10.0, 0.20
            self._servo_opts = ServoOpts(
                watchdog_ms=SERVO_WATCHDOG_MS,
                max_vel=max_vel,
                max_step_rad=max_step,
                max_lag_rad=0.50,
                rate_hz=LIVE_SERVO_RATE_HZ,
                lag_abort_consecutive=0,
                lookahead_time=0.0,
                feedforward_vel=True,
                use_mit=False,
            )
        except Exception:
            from types import SimpleNamespace

            if self._use_live_servo_envelope():
                max_vel, max_step = self._live_servo_caps()
            else:
                max_vel, max_step = 10.0, 0.20
            self._servo_opts = SimpleNamespace(
                watchdog_ms=SERVO_WATCHDOG_MS,
                max_vel=max_vel,
                max_step_rad=max_step,
                max_lag_rad=0.50,
                rate_hz=LIVE_SERVO_RATE_HZ,
                lag_abort_consecutive=0,
                lookahead_time=0.0,
                feedforward_vel=True,
                use_mit=False,
            )
        self._servo_opts_ready = True
        return self._servo_opts

    @staticmethod
    def _flag_enabled(robot: Any) -> bool:
        flag = getattr(robot, "is_enabled", False)
        if callable(flag):
            try:
                flag = flag()
            except Exception:
                return False
        return bool(flag)

    @staticmethod
    def _sdk_servoing(robot: Any) -> bool:
        flag = getattr(robot, "is_servoing", None)
        if callable(flag):
            try:
                return bool(flag())
            except Exception:
                return False
        if isinstance(flag, bool):
            return flag
        return False

    def _clear_joint_watchdogs(self, robot: Any) -> None:
        """SDK servo_end set_timeout(0) is fire-and-forget; leftover watchdog
        brakes joints and then 0x8090 frames look dead. Repeat the clear."""
        ht = getattr(robot, "_ht", None)
        ids = list(getattr(robot, "joint_motor_ids", []) or [])
        if ht is None or not ids or not hasattr(ht, "set_timeout"):
            return
        for mid in ids:
            try:
                ht.set_timeout(int(mid), 0)
            except Exception:
                pass

    def _ensure_enabled(self) -> str | None:
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        if self._float_writer_active():
            return "重力/阻抗环占用写者，不能 enable"
        name = _robot_state_name(robot)
        if name in ("dead", "disconnected"):
            self._servo_intent = False
            return "通信丢失（DEAD）。请 Disconnect 后 Connect"
        # Official SDK: enable() raises RobotStateError while SERVOING.
        # test_fafu_keyboard_cartesian never calls enable in the 100 Hz loop;
        # it only servo_j. is_enabled() may also block on cache-miss reads
        # long enough to trip the firmware watchdog.
        if self._servoing or self._sdk_servoing(robot):
            return None
        if self._flag_enabled(robot):
            return None
        try:
            robot.enable(allow_motor_reset=True)
        except TypeError:
            try:
                robot.enable()
            except Exception as exc:
                return f"使能失败: {exc}"
        except Exception as exc:
            return f"使能失败: {exc}"
        return None

    def _servo_writer_idle(self) -> bool:
        return not self._servoing and not self._servo_intent and not self._writer_busy

    def _wait_servo_idle(self, timeout: float = 0.8) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._servo_writer_idle():
                return True
            time.sleep(0.01)
        return self._servo_writer_idle()

    def _end_servo(self) -> bool:
        robot = self._robot
        if robot is None or not self._servoing:
            self._servoing = False
            self._servo_idle.set()
            return False
        last = self._servo_cmd_rad or self._stream_q_rad
        if last is not None:
            try:
                # Same pose again ⇒ feedforward vel ≈ 0, so hold does not
                # latch a non-zero 0x8090 velocity (motors otherwise keep running).
                robot.servo_j(list(last))
            except Exception:
                logger.exception("结束 servo 前零速度刷新失败")
        try:
            robot.servo_end("hold")
            ended = True
        except Exception:
            logger.exception("结束 servo 会话失败")
            ended = False
        self._servoing = False
        self._clear_joint_watchdogs(robot)
        self._servo_idle.set()
        if last is not None:
            self._send_zero_vel_hold(list(last))
        return ended

    def _stop_gravity(self, join: bool = True) -> bool:
        self._grav_stop.set()
        th = self._grav_th
        if join and th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(timeout=2.5)
        if th is not None and th.is_alive():
            if join:
                logger.error("重力环未能在超时内退出，保持 abort 标志")
                return False
            return True
        self._grav_th = None
        self._grav_stop.clear()
        if self._grav_err:
            logger.warning("重力环结束时有错误: %s", self._grav_err)
            self._grav_err = ""
        return True

    def _send_zero_vel_hold(self, q_rad: list[float] | None = None) -> None:
        """Latch MODE_POSITION at ``q_rad`` with velocity 0.

        ``move_j(..., block=False)`` writes a non-zero vel field even when the
        target equals the current pose, so motors keep humming/hunting. SDK
        ``emergency_stop`` / ``disable`` are MODE_STOP (PWM off) and drop the arm.
        """
        robot = self._robot
        if robot is None or not self.allow_motion:
            return
        if hasattr(robot, "hold_position"):
            try:
                robot.hold_position()
                return
            except Exception:
                logger.exception("hold_position 失败")
        if q_rad is None:
            try:
                q_rad = [float(x) for x in list(robot.get_joint_values(prefer_cache=True))[: self.n]]
            except Exception:
                q_rad = list(self._servo_cmd_rad or [])
        if not q_rad:
            return
        build = getattr(robot, "_build_many_cmds_holding_others", None)
        ht = getattr(robot, "_ht", None)
        cfg = getattr(robot, "_cfg", None)
        if not callable(build) or ht is None or cfg is None:
            logger.warning("无法下发零速度保持帧（缺少 SDK 内部接口）")
            return
        try:
            import fafu_motor as pm  # type: ignore

            ids = list(cfg.motor_ids)
            targets: dict[int, float] = {}
            n = min(self.n, len(q_rad), len(ids))
            for i in range(n):
                targets[int(ids[i])] = float(q_rad[i]) / (2.0 * math.pi)
            cmds = build(targets, vel_rps=0.0)
            ht.set_many_pos_vel_tqe(cmds, pm.PosUnit.Turns, int(max(ids)), 0.05)
            self._target_deg = [math.degrees(float(x)) for x in q_rad[: self.n]]
        except Exception:
            logger.exception("零速度保持帧发送失败")

    def _freeze_measured_pose(self) -> None:
        robot = self._robot
        if robot is None or not self.allow_motion:
            return
        if bool(getattr(robot, "_estopped", False)):
            self._send_zero_vel_hold()
            return
        name = _robot_state_name(robot)
        if name in ("estop", "dead", "disconnected"):
            if name == "estop":
                logger.warning("SDK 仍为 ESTOP，尝试保持姿态前先 enable")
            else:
                return
        err = self._ensure_enabled()
        if err:
            logger.warning("冻结当前角失败: %s", err)
            return
        self._send_zero_vel_hold()

    def _start_gravity_loop(self, *, friction: bool, impedance: bool) -> str | None:
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        if not self._dyn_ready:
            return "需 pinocchio（真机力矩环未加载）"
        self._servo_intent = False
        if not self._wait_servo_idle(1.0):
            return "servo 未结束，无法进入重力/阻抗"
        if not self._stop_gravity(join=True):
            return "上一轮重力环未退出"
        err = self._ensure_enabled()
        if err:
            return err
        self._grav_stop.clear()
        self._grav_err = ""
        n = self.n
        kwargs: dict[str, Any] = {
            "friction": friction,
            "rate_hz": 200.0,
            "abort_check": lambda: self._grav_stop.is_set(),
            "home_on_exit": False,
            "hold_on_release": not impedance,
        }
        if impedance:
            pass
        else:
            kwargs["k_soft"] = [0.0] * n
            kwargs["b_soft"] = [0.0] * n
            kwargs["i_soft"] = [0.0] * n

        def run() -> None:
            try:
                robot.start_gravity_compensation(**kwargs)
            except Exception as exc:
                self._grav_err = str(exc)
                logger.exception("重力补偿环退出")

        self._grav_th = threading.Thread(target=run, name="fafu-gravity", daemon=True)
        self._grav_th.start()
        time.sleep(0.08)
        if self._grav_err:
            err = self._grav_err
            self._grav_th = None
            return f"重力/阻抗环启动失败: {err}"
        if self._grav_th is None or not self._grav_th.is_alive():
            return "重力/阻抗环未能启动"
        return None

    def _reenter_position(self) -> str | None:
        """Stop float/impedance, enable, and hold the measured pose (no jump)."""
        self._servo_intent = False
        if not self._stop_gravity(join=True):
            return "重力环未退出，拒绝 Position 接管"
        if not self._wait_servo_idle(1.0):
            return "servo 未结束，拒绝 Position 接管"
        robot = self._robot
        if robot is None or not self.allow_motion:
            return None
        self._freeze_measured_pose()
        self._hold_latched = True
        return None

    def hold(self, *, abort_path: bool = False) -> None:
        if not self.allow_motion or self._robot is None:
            return
        if abort_path:
            self._home_path = []
        elif self._home_path:
            return
        if (
            self._hold_latched
            and not self._servoing
            and not self._servo_intent
            and not self._freeze_needed
            and not self._float_writer_active()
        ):
            return
        was_float = self._ctrl_mode in FLOAT_MODES
        self._pending_grip = None
        self._stop_gravity(join=False)
        # Keep the 100 Hz servo_j hold. Ending the session here is what made
        # keyboard/slider/home look like a twitch then a dead arm.
        self._hold_stream_pose()
        if was_float:
            self._ctrl_mode = "Position"

    def servo(self, dt: float) -> None:
        robot = self._robot
        if robot is None:
            return
        self._servo_dt = max(0.0, min(0.05, float(dt)))
        if self.allow_motion:
            self._flush_writer()
        step = getattr(robot, "step", None)
        if callable(step):
            step(dt)

    def _flush_writer(self) -> None:
        robot = self._robot
        if robot is None or not self.allow_motion:
            return
        self._writer_busy = True
        try:
            self._flush_writer_locked()
        finally:
            self._writer_busy = False

    def _flush_writer_locked(self) -> None:
        robot = self._robot
        if robot is None or not self.allow_motion:
            return
        if self._float_writer_active():
            return
        following_path = bool(self._home_path)
        if following_path:
            self._advance_home()
        elif (
            self._servo_intent
            and self._stream_q_rad is not None
            and not self._parked
            and (time.monotonic() - self._stream_rx_mono) > STREAM_IDLE_S
        ):
            self._hold_stream_pose()
        if self._grav_th is not None and not self._grav_th.is_alive():
            self._grav_th = None
        if self._servoing and not self._sdk_servoing(robot):
            self._servoing = False
        if self._servo_intent and self._stream_q_rad is not None:
            self._freeze_needed = False
            err = self._ensure_enabled()
            if err:
                logger.warning("servo 使能失败: %s", err)
                self._writer_err = err
                name = _robot_state_name(robot)
                if name in ("dead", "disconnected"):
                    self._servo_intent = False
                    self._home_path = []
                    self._freeze_needed = True
            else:
                if not self._servoing:
                    opts = self._make_servo_opts()
                    try:
                        robot.servo_start(opts)
                        self._servoing = True
                        self._servo_idle.clear()
                    except Exception as exc:
                        logger.exception("servo_start 失败")
                        self._writer_err = f"servo_start 失败: {exc}"
                        if not self._home_path:
                            self._servo_intent = False
                            self._freeze_needed = True
                if self._servoing and self._servo_intent:
                    try:
                        q_send = list(self._stream_q_rad)
                        # Path following already speed-limits with synchronized
                        # joints. Independent per-axis slew toward a far vertex
                        # is what made 复位 stop-start twitch.
                        if not self._is_fake() and not following_path:
                            q_send = self._slew_live_cmd(q_send, self._servo_dt)
                        sent = robot.servo_j(q_send)
                    except Exception as exc:
                        logger.exception("servo_j 失败")
                        self._writer_err = f"servo_j 失败: {exc}"
                        ended = self._end_servo()
                        self._hold_stream_pose()
                        if not ended:
                            self._freeze_needed = True
                    else:
                        if sent is False:
                            reason = getattr(robot, "_servo_aborted_reason", None) or "servo_j 未发送"
                            logger.warning("servo_j 拒绝: %s", reason)
                            self._writer_err = str(reason)
                            if self._home_path:
                                pass
                            else:
                                ended = self._end_servo()
                                self._hold_stream_pose()
                                if not ended:
                                    self._freeze_needed = True
                        else:
                            self._writer_err = ""
                            self._servo_cmd_rad = list(q_send)
        elif self._servoing:
            self._end_servo()
        if self._freeze_needed:
            if not self._servoing:
                self._freeze_measured_pose()
            self._freeze_needed = False
            self._hold_latched = True
        grip = self._pending_grip
        self._pending_grip = None
        if grip is not None:
            self._apply_pending_grip(grip)

    def _grip_kwargs(self) -> dict[str, Any]:
        return {"block": False, "effort": int(clip_gripper_effort(self._gripper_effort))}

    def _apply_pending_grip(self, grip: tuple[str, Any]) -> None:
        robot = self._robot
        if robot is None:
            return
        kind, value = grip
        try:
            err = self._ensure_enabled()
            if err:
                logger.warning("夹爪使能失败: %s", err)
                self._writer_err = f"夹爪: {err}"
                return
            cmd = self._grip_kwargs()
            if kind == "open":
                if value:
                    robot.open_gripper(**cmd)
                    self._gripper_deg = float(self.gripper_limits[1])
                else:
                    robot.close_gripper(**cmd)
                    self._gripper_deg = float(self.gripper_limits[0])
                self._grip_open = bool(value)
                self._writer_err = ""
                return
            lo, hi = self.gripper_limits
            deg = max(lo, min(hi, float(value)))
            control = getattr(robot, "gripper_control", None)
            if callable(control):
                control(math.radians(deg), is_radians=True, **cmd)
            elif deg >= 0.5 * (lo + hi):
                robot.open_gripper(**cmd)
            else:
                robot.close_gripper(**cmd)
            self._gripper_deg = deg
            self._grip_open = deg >= 0.5 * (lo + hi)
            self._writer_err = ""
        except Exception as exc:
            logger.exception("夹爪指令失败")
            self._writer_err = f"夹爪指令失败: {exc}"

    def _end_stream(self, timeout: float | None = None) -> str | None:
        """Ask the 100 Hz tick to close servo, then allow move_j / home / path.

        ``servo_start`` / ``servo_j`` / ``servo_end`` must stay on that writer
        thread (live SDK raises BusyError otherwise). Fake has no such rule, so
        if the tick missed the close we end the session here as a last resort.
        Errors are returned to the UI — never swallowed.
        """
        if timeout is None:
            timeout = 0.12 if self._is_fake() else 0.5
        self._servo_intent = False
        if self._wait_servo_idle(timeout):
            return None
        if self._is_fake() or self._robot is None:
            self._end_servo()
        if not self._servo_writer_idle():
            return "servo 未结束，请稍候再发关节/复位/路点"
        return None

    def apply_targets(
        self,
        q_deg: list[float],
        speed_deg_s: float,
        stream: bool = False,
        *,
        safe_shoulder: bool = True,
    ) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            logger.warning("忽略关节目标: %s", blocked)
            return blocked
        if self._float_writer_active():
            return "重力/阻抗环运行中请先切回 Position"
        if stream and self._home_path:
            return None
        robot = self._robot
        q_deg = self._clip_deg(q_deg)
        if stream and self._parked:
            meas = [math.degrees(x) for x in (self._servo_cmd_rad or [])]
            if not meas and robot is not None:
                try:
                    meas = [math.degrees(float(x)) for x in robot.get_joint_values(prefer_cache=True)[: self.n]]
                except Exception:
                    meas = list(self._target_deg)
            if self._near_q(self._q_rad(q_deg), self._q_rad(meas), math.radians(STREAM_RESTART_DEG)):
                return None
            self._parked = False
        if not stream:
            self._parked = False
            self._home_path = []
        self._target_deg = list(q_deg)
        q_rad = self._q_rad(q_deg)
        self._hold_latched = False
        self._freeze_needed = False
        if stream:
            self.set_teleop_speed(speed_deg_s)
            self._stream_q_rad = q_rad
            self._servo_intent = True
            self._note_stream()
            return None
        # Sim used to servo_end (~120 ms) then move_j. That extra wait, plus
        # independent-axis crawl, made the twin lag the live arm. Same 100 Hz
        # synchronized servo path as hardware.
        err = self._ensure_enabled()
        if err:
            return err
        now = self._measured_deg()
        if safe_shoulder:
            wps = [self._q_rad(p) for p in self._shoulder_safe_waypoints(now, q_deg)]
        else:
            wps = [self._q_rad(self._clip_deg(q_deg))]
        self._start_joint_path(wps, speed_deg_s)
        return None

    def park_stream(self) -> None:
        """Freeze leftover follow/keyboard servo at the last commanded pose."""
        if self._home_path:
            return
        self._hold_stream_pose()

    def set_gripper_effort(self, effort: int) -> str | None:
        self._gripper_effort = clip_gripper_effort(effort)
        return None

    def set_gripper_open(self, open_: bool, effort: int | None = None) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            logger.warning("忽略夹爪: %s", blocked)
            return blocked
        if self._float_writer_active():
            return "重力/阻抗环运行中请先切回 Position 再动夹爪"
        if effort is not None:
            self._gripper_effort = clip_gripper_effort(effort)
        self._hold_latched = False
        self._pending_grip = ("open", bool(open_))
        return None

    def set_gripper_deg(self, deg: float, effort: int | None = None) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        if self._float_writer_active():
            return "重力/阻抗环运行中请先切回 Position 再动夹爪"
        if effort is not None:
            self._gripper_effort = clip_gripper_effort(effort)
        lo, hi = self.gripper_limits
        value = max(lo, min(hi, float(deg)))
        self._hold_latched = False
        self._pending_grip = ("deg", value)
        return None

    def _advance_home(self) -> None:
        """Follow remaining vertices at UI speed with synchronized joints.

        Intermediate corners are fly-by: leftover step continues into the next
        segment in the same tick. Only the final pose parks (same-pose servo_j).
        """
        if not self._home_path:
            return
        dt = max(1e-4, min(0.05, float(self._servo_dt or 0.0)))
        step = math.radians(self._teleop_deg_s) * dt
        cur = self._servo_cmd_rad
        if cur is None:
            robot = self._robot
            try:
                cur = (
                    [float(x) for x in list(robot.get_joint_values(prefer_cache=True))[: self.n]]
                    if robot
                    else None
                )
            except Exception:
                cur = None
            if cur is None:
                cur = list(self._home_path[0])
        remaining = step
        while self._home_path and remaining > 1e-12:
            goal = self._home_path[0]
            dist = self._max_abs_delta(cur, goal)
            if dist <= 1e-6:
                self._home_path.pop(0)
                continue
            if dist <= remaining:
                cur = list(goal) if len(goal) >= self.n else self._interp_q(cur, goal, 1.0)
                remaining -= dist
                self._home_path.pop(0)
            else:
                cur = self._interp_q(cur, goal, remaining / dist)
                remaining = 0.0
        self._servo_cmd_rad = list(cur)
        self._stream_q_rad = list(cur)
        self._servo_intent = True
        self._parked = False
        self._note_stream()
        if self._home_path:
            self._target_deg = [math.degrees(x) for x in self._home_path[-1]]
            return
        self._parked = True
        self._hold_latched = True
        self._target_deg = [math.degrees(x) for x in cur]

    def home(self, speed_deg_s: float) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            logger.warning("忽略复位: %s", blocked)
            return blocked
        if self._float_writer_active():
            return "重力/阻抗环运行中请先切回 Position 再复位"
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        self._hold_latched = False
        self._parked = False
        err = self._ensure_enabled()
        if err:
            return err
        spd = min(float(speed_deg_s), HOME_MAX_DEG_S)
        self.set_teleop_speed(spd)
        try:
            q_now = [math.degrees(float(x)) for x in robot.get_joint_values(prefer_cache=True)[: self.n]]
        except Exception as exc:
            return f"读关节失败: {exc}"
        if self._servo_cmd_rad is not None:
            q_now = [math.degrees(x) for x in self._servo_cmd_rad]
        wps = [self._q_rad(p) for p in self._home_waypoints_deg(q_now)]
        if self._is_fake():
            err = self._end_stream()
            if err:
                return err
            robot.go_home(speed=self._speed_pct(spd), block=False)
            self._home_path = []
            self._target_deg = [0.0] * self.n
            self._parked = True
            self._hold_latched = True
            return None
        self._home_path = wps
        if self._servo_cmd_rad is not None:
            self._stream_q_rad = list(self._servo_cmd_rad)
        else:
            self._stream_q_rad = self._q_rad(q_now)
        self._servo_intent = True
        self._target_deg = [math.degrees(x) for x in wps[-1]]
        self._note_stream()
        return None

    def set_ctrl_mode(self, mode: str) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        name = (mode or "Position").strip()
        if name not in ("Position", "Gravity", "Gra+Fri", "Impedance"):
            return f"未知控制模式 {mode!r}"
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        prev = self._ctrl_mode
        if name == prev:
            return None
        if name in FLOAT_MODES and not self._float_modes_available():
            return "需 pinocchio（真机力矩环未加载）"
        self._servo_intent = False
        if prev in FLOAT_MODES or name == "Position":
            err = self._reenter_position()
            if err:
                return err
        else:
            err = self._end_stream(1.0)
            if err:
                return err
        self._ctrl_mode = name
        self._hold_latched = False
        if name in ("Gravity", "Gra+Fri"):
            if self._use_sdk_float():
                try:
                    err = self._ensure_enabled()
                    if err:
                        self._ctrl_mode = "Position"
                        return err
                    robot.open_gripper(**self._grip_kwargs())
                    self._gripper_deg = float(self.gripper_limits[1])
                    self._grip_open = True
                except Exception as exc:
                    logger.warning("切 Gravity 时开夹爪失败: %s", exc)
                err = self._start_gravity_loop(friction=(name == "Gra+Fri"), impedance=False)
                if err:
                    self._ctrl_mode = "Position"
                return err
            opened = self.set_gripper_open(True)
            if opened:
                logger.warning("切 Gravity 时开夹爪: %s", opened)
            return None
        if name == "Impedance":
            if self._use_sdk_float():
                err = self._start_gravity_loop(friction=True, impedance=True)
                if err:
                    self._ctrl_mode = "Position"
                return err
            if hasattr(robot, "set_speed_scale"):
                robot.set_speed_scale(0.35)
            return None
        if hasattr(robot, "set_speed_scale"):
            robot.set_speed_scale(1.0)
        return None

    def teach_pose(self, q_deg: list[float]) -> None:
        blocked = self.refuse_motion()
        if blocked:
            logger.warning("忽略示教: %s", blocked)
            return
        robot = self._robot
        if robot is None:
            return
        if self._use_sdk_float():
            return
        if hasattr(robot, "teleport"):
            robot.teleport(self._q_rad(self._clip_deg(q_deg)), is_radians=True)
        self._target_deg = self._clip_deg(q_deg)

    def _fk_xyz_rot(self, q_rad: list[float], robot: Any) -> tuple[list[float], Any]:
        """End-effector pose. SDK FK needs pinocchio; station URDF IK does not."""
        if self._dyn_ready:
            fk = getattr(robot, "forward_kinematics", None)
            if callable(fk):
                try:
                    pack = fk(q_rad, is_radians=True)
                    pos = [float(x) for x in list(pack.get("position") or [])[:3]]
                    ori = pack.get("rotation")
                    if len(pos) == 3 and ori is not None:
                        return pos, ori
                except Exception:
                    pass
        pos, ori, _rpy = forward_pose(q_rad)
        return [float(x) for x in pos], ori

    def apply_cartesian(self, dxyz: list[float], drpy_deg: list[float]) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        if self._float_writer_active() or self._ctrl_mode in ("Gravity", "Gra+Fri"):
            return "重力/阻抗环运行中请先切回 Position 再用键盘"
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        # Key-up park must not abort home, but a new WASD press should.
        # A leftover _home_path after 复位 also silently ate later keys.
        self._home_path = []
        self._parked = False
        seed: list[float] | None = None
        pos: list[float] | None = None
        ori: Any = None
        # Teleop must increment the *commanded* pose. IK from lagged measured q
        # fights the interpolator and feels like seconds of delay.
        # Lead with the stream target while the key is down so slew can catch
        # up; park still freezes at the last sent pose.
        if self._servo_intent and self._stream_q_rad is not None:
            seed = list(self._stream_q_rad)
            pos, ori = self._fk_xyz_rot(seed, robot)
        elif not self._is_fake() and self._servo_cmd_rad is not None:
            seed = list(self._servo_cmd_rad)
            pos, ori = self._fk_xyz_rot(seed, robot)
        if pos is None:
            try:
                seed = list(robot.get_joint_values(prefer_cache=True))
            except Exception as exc:
                return f"读关节失败: {exc}"
            pos, ori = self._fk_xyz_rot(seed, robot)
        dpos = [float(dxyz[i] if i < len(dxyz) else 0.0) for i in range(3)]
        drpy = [math.radians(float(drpy_deg[i] if i < len(drpy_deg) else 0.0)) for i in range(3)]
        q = step_cartesian(seed or [], dpos, drpy)
        if q is None:
            xyz, rot = apply_delta_pose(
                pos,
                as_rotation(ori, is_euler=False),
                dpos,
                drpy,
            )
            q = self._solve_pose(xyz, rot, seed)
        if q is None:
            # Keep holding the last pose. Ending servo here is what dropped
            # the arm at a raised shoulder when IK failed mid-keypress.
            self._hold_stream_pose()
            return "IK 无解（末端超出工作空间或靠近奇异位，可先把 J2/J3 抬离零位）"
        self._hold_latched = False
        self._freeze_needed = False
        self._stream_q_rad = q
        self._servo_intent = True
        self._parked = False
        self._target_deg = [math.degrees(x) for x in q]
        self._note_stream()
        return None

    def _solve_pose(self, xyz: list[float], rot: Any, seed: list[float] | None) -> list[float] | None:
        candidates: list[list[float]] = []
        robot = self._robot
        if robot is not None and self._dyn_ready:
            try:
                solved = robot.inverse_kinematics(
                    xyz,
                    rot,
                    is_euler=False,
                    is_radians=True,
                    init_q=seed,
                    multi_init=False,
                )
                if solved is not None:
                    candidates.append([float(x) for x in list(solved)[: self.n]])
            except Exception as exc:
                logger.warning("SDK IK 失败，改用站控 IK: %s", exc)
        station = inverse_with_fallback(
            xyz,
            seed_rad=seed,
            rotation=rot,
            is_euler=False,
            is_radians=True,
            max_iter=80,
        )
        if station is not None:
            candidates.append([float(x) for x in list(station)[: self.n]])
        best: list[float] | None = None
        best_err = 1e9
        for q in candidates:
            pos, _ori = self._fk_xyz_rot(q, robot)
            err = xyz_err(pos, xyz)
            if err < best_err:
                best_err = err
                best = q
        if best is None or best_err > CART_POSE_TOL_M * 2.5:
            return None
        return best

    def _cartesian_joint_path(
        self, xyz: list[float], rot: Any, seed: list[float]
    ) -> list[list[float]] | None:
        pos0, _ori0 = self._fk_xyz_rot(seed, self._robot)
        dist = xyz_err(pos0, xyz)
        n = max(1, min(48, int(math.ceil(dist / 0.012))))
        out: list[list[float]] = []
        q = list(seed)
        for i in range(1, n + 1):
            t = i / float(n)
            xyz_i = [pos0[j] + (xyz[j] - pos0[j]) * t for j in range(3)]
            if i < n:
                _p, r_keep, _rpy = forward_pose(q)
                step = inverse_with_fallback(
                    xyz_i,
                    seed_rad=q,
                    rotation=r_keep,
                    is_euler=False,
                    is_radians=True,
                    max_iter=40,
                )
                if step is None:
                    continue
                q = [float(x) for x in step[: self.n]]
            else:
                step = self._solve_pose(xyz_i, rot, q)
                if step is None:
                    return None
                q = step
            out.append(list(q))
        return out or None

    def apply_cartesian_pose(
        self, xyz_m: list[float], rpy_deg: list[float], speed_deg_s: float
    ) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        if self._float_writer_active() or self._ctrl_mode in ("Gravity", "Gra+Fri"):
            return "重力/阻抗环运行中请先切回 Position 再发笛卡尔目标"
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        self._parked = False
        self._home_path = []
        xyz = clip_cart_xyz(xyz_m)
        rpy = clip_cart_rpy_deg(rpy_deg)
        rot = rpy_to_matrix([math.radians(x) for x in rpy])
        seed: list[float] | None = None
        try:
            if not self._is_fake() and self._servo_cmd_rad is not None:
                seed = list(self._servo_cmd_rad)
            elif self._servo_intent and self._stream_q_rad is not None:
                seed = list(self._stream_q_rad)
            else:
                seed = list(robot.get_joint_values(prefer_cache=True))[: self.n]
        except Exception as exc:
            return f"读关节失败: {exc}"
        if seed is None:
            return "读关节失败"
        while len(seed) < self.n:
            seed.append(0.0)
        path = self._cartesian_joint_path(xyz, rot, seed)
        if not path:
            # RPC thread (ACK). Never run Trac-IK on the 100 Hz WASD/servo tick.
            try:
                from robot_station.adapters.fafu_tracik import inverse_tracik

                q_trac = inverse_tracik(xyz, rot, seed)
            except Exception:
                q_trac = None
            if q_trac is None:
                return "IK 无解（末端超出工作空间或靠近奇异位，可先把 J2/J3 抬离零位）"
            path = [list(q_trac)]
        q_goal = path[-1]
        pos, _ori = self._fk_xyz_rot(q_goal, robot)
        if xyz_err(pos, xyz) > CART_POSE_TOL_M * 2.0:
            return (
                f"IK 未到目标（误差 {xyz_err(pos, xyz)*1000:.0f} mm）。"
                "可先抬 J2/J3 再发，或减小步距。"
            )
        self._start_joint_path(path, speed_deg_s)
        return None

    def _rpy_from_pose(self, ori: Any, robot: Any) -> list[float]:
        parsed = rotation_to_rpy(ori)
        if parsed is not None:
            return parsed
        try:
            fk = robot.forward_kinematics()
            rpy = fk.get("rpy")
            if rpy is not None:
                return [float(x) for x in rpy]
        except Exception:
            pass
        logger.warning("末端姿态不是 RPY 或 3x3，笛卡尔增量按零姿态处理")
        return [0.0, 0.0, 0.0]

    def run_path(
        self,
        waypoints_deg: list[list[float]],
        speed_deg_s: float,
        durations_s: list[float] | None = None,
    ) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        if not waypoints_deg:
            return "没有路点"
        robot = self._robot
        if self._float_writer_active():
            return "重力/阻抗环运行中请先切回 Position 再跑路点"
        err = self._ensure_enabled()
        if err:
            return err
        clipped = [self._clip_deg(q) for q in waypoints_deg]
        speed = speed_deg_s
        if durations_s:
            max_delta = 0.0
            prev = clipped[0]
            for i, cur in enumerate(clipped[1:], start=1):
                dt = float(durations_s[i] if i < len(durations_s) else durations_s[-1])
                dt = max(0.15, dt)
                delta = max(abs(cur[j] - prev[j]) for j in range(self.n))
                max_delta = max(max_delta, delta / dt)
                prev = cur
            if max_delta > 0:
                speed = max(5.0, min(80.0, max_delta))
        if self._is_fake():
            err = self._end_stream()
            if err:
                return err
            self._hold_latched = False
            path = [self._q_rad(q) for q in clipped]
            robot.move_jntspace_path(path, is_radians=True, speed=self._speed_pct(speed))
            self._target_deg = list(clipped[-1])
            return None
        now = self._measured_deg()
        expanded: list[list[float]] = []
        prev = now
        for cur in clipped:
            for pose in self._shoulder_safe_waypoints(prev, cur):
                expanded.append(self._q_rad(pose))
            prev = cur
        if not expanded:
            return "没有路点"
        self._start_joint_path(expanded, speed)
        return None

    def emergency_stop(self) -> None:
        """Hold the current pose. Do not call SDK emergency_stop (MODE_STOP / free-spin)."""
        self._home_path = []
        self._grav_stop.set()
        self._servo_intent = False
        self._pending_grip = None
        self._parked = True
        self._freeze_needed = True
        self._hold_latched = True
        robot = self._robot
        if robot is None or not self.allow_motion:
            return
        if self._servo_writer_idle():
            try:
                self._freeze_measured_pose()
            except Exception:
                logger.exception("急停保持姿态失败")

    def resume_motion(self) -> None:
        robot = self._robot
        if robot is None or not self.allow_motion:
            return
        self._stop_gravity(join=True)
        self._servo_intent = False
        self._home_path = []
        self._ctrl_mode = "Position"
        name = _robot_state_name(robot)
        if name == "dead" and hasattr(robot, "recover"):
            try:
                robot.recover(confirm=True)
            except Exception:
                logger.exception("SDK recover 失败")
        if name == "estop" and hasattr(robot, "resume"):
            try:
                robot.resume()
            except Exception:
                logger.exception("SDK resume 失败")
        try:
            self._ensure_enabled()
        except Exception:
            logger.exception("急停解除后使能失败")
        self._hold_latched = True
        self._parked = True
        self._freeze_needed = True

    def set_powered(self, on: bool) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        if on:
            self._hold_latched = False
            return self._ensure_enabled()
        self._servo_intent = False
        self._home_path = []
        self._stop_gravity(join=True)
        if not self._wait_servo_idle(0.8):
            return "servo 未结束，无法去使能"
        self._send_zero_vel_hold()
        self._parked = True
        self._hold_latched = True
        if self._is_fake():
            try:
                robot.disable()
            except Exception as exc:
                return f"去使能失败: {exc}"
            return None
        # Live disable() is MODE_STOP (PWM off). Keep position hold instead.
        return None

    def run_mapped_script(self, name: str) -> str | None:
        op = map_script(name)
        if op is None:
            return (
                f"不启动 {name}：站控已占用调试板串口（单写者）。"
                "请用本页关节、键盘或路点。"
            )
        if op == "home":
            return self.home(self.default_speed)
        if op == "grip_open":
            self.set_gripper_open(True)
            return None
        if op == "estop":
            self.emergency_stop()
            return None
        return f"未实现脚本映射 {op}"

    def _motor_rows(self, states: dict, ids: list[int], ok: list[bool]) -> list[dict]:
        rows: list[dict] = []
        mids = list(ids) if ids else list(range(1, self.n + 1))
        for i in range(self.n):
            mid = int(mids[i]) if i < len(mids) else i + 1
            st = states.get(mid)
            online = st is not None and bool(getattr(st, "online", True))
            fault = int(getattr(st, "fault", 0) or 0) if st is not None else 1
            mode = int(getattr(st, "mode", 0) or 0) if st is not None else 0
            rows.append(
                {
                    "id": mid,
                    "name": f"J{i + 1}",
                    "online": online,
                    "ok": bool(ok[i]) if i < len(ok) else False,
                    "fault": fault,
                    "mode": _motor_mode_name(mode),
                }
            )
        if self.has_gripper:
            gst = states.get(self.gripper_id)
            rows.append(
                {
                    "id": int(self.gripper_id),
                    "name": "夹爪",
                    "online": gst is not None and bool(getattr(gst, "online", True)),
                    "ok": gst is not None and not bool(getattr(gst, "fault", 0)),
                    "fault": int(getattr(gst, "fault", 0) or 0) if gst is not None else 1,
                    "mode": _motor_mode_name(int(getattr(gst, "mode", 0) or 0) if gst is not None else 0),
                }
            )
        return rows

    def poll(self) -> ArmSnap:
        with self._lock:
            robot = self._robot
            extra = {
                "limits": [list(p) for p in self.limits],
                "gripper_limits": list(self.gripper_limits),
                "dyn_ready": self._dyn_ready,
                "grav_active": self._grav_th is not None and self._grav_th.is_alive(),
                "float_ok": self._float_modes_available(),
                "float_reason": self._float_modes_reason(),
                "gripper_effort": int(clip_gripper_effort(self._gripper_effort)),
            }
        if robot is None:
            snap = ArmSnap(
                online=False,
                backend=self.backend,
                enabled=False,
                n=self.n,
                q_deg=list(self._last.q_deg),
                target_deg=list(self._last.target_deg),
                ok=[False] * self.n,
                gripper_open=self._last.gripper_open,
                has_gripper=self.has_gripper,
                moving=False,
                ee_m=self._last.ee_m,
                ee_rpy_deg=self._last.ee_rpy_deg,
                ctrl_mode=self._ctrl_mode,
                gripper_deg=self._last.gripper_deg,
                tau_raw=list(self._last.tau_raw),
                **extra,
            )
            self._last = snap
            return snap
        try:
            q_rad = list(robot.get_joint_values(prefer_cache=True))
            # Live USB reads can lag the motor command by a cache tick. The
            # twin should track the last servo_j we sent while a session is
            # open; otherwise the 3D arm trails the metal arm.
            if (
                not self._is_fake()
                and self._servo_cmd_rad is not None
                and (self._servoing or self._home_path)
            ):
                q_rad = list(self._servo_cmd_rad[: self.n])
            q_deg = [math.degrees(float(x)) for x in q_rad[: self.n]]
            while len(q_deg) < self.n:
                q_deg.append(0.0)
            ok = [True] * self.n
            moving = False
            tau_raw = [0] * self.n
            states: dict = {}
            ids: list[int] = []
            try:
                vels = list(robot.get_joint_velocities(prefer_cache=True))
                moving = any(abs(float(v)) > 0.02 for v in vels[: self.n])
            except Exception:
                moving = False
            try:
                raw_states = robot.get_motor_states(prefer_cache=True)
                states = raw_states if isinstance(raw_states, dict) else {}
                ids = list(getattr(robot, "joint_motor_ids", []))[: self.n]
                for i, mid in enumerate(ids):
                    st = states.get(mid)
                    if st is None:
                        ok[i] = False
                        continue
                    fault = getattr(st, "fault", 0)
                    ok[i] = not bool(fault)
                    tau_raw[i] = int(getattr(st, "torque", 0) or 0)
                if self.has_gripper:
                    gst = states.get(self.gripper_id)
                    if gst is not None:
                        pos = getattr(gst, "position", None)
                        if pos is not None:
                            self._gripper_deg = motor_position_to_deg(float(pos))
            except Exception:
                pass
            motors = self._motor_rows(states, ids, ok)
            # Prefer cached joint modes. SDK is_enabled() may block-read every
            # motor (including gripper) and stall the 100 Hz servo watchdog.
            if ids and states:
                enabled = True
                for mid in ids:
                    st = states.get(mid)
                    if st is None or int(getattr(st, "mode", 0) or 0) != 0x0A:
                        enabled = False
                        break
            else:
                enabled = self._flag_enabled(robot)
            tgt = list(self._target_deg) if self.allow_motion else list(q_deg)
            extra_tgt = getattr(robot, "_tgt_rad", None)
            if extra_tgt is not None:
                tgt = [math.degrees(float(x)) for x in extra_tgt[: self.n]]
            lo_g = float(self.gripper_limits[0])
            hi_g = float(self.gripper_limits[1])
            if self._is_fake() and hasattr(robot, "gripper_deg"):
                self._gripper_deg = float(robot.gripper_deg)
                grip = bool(getattr(robot, "gripper_open", self._grip_open))
            else:
                span = max(1.0, hi_g - lo_g)
                # Live SDK has no gripper_open. 0° (cfg lower limit) is closed;
                # do not treat a sub-limit reading as "invalid / open".
                grip = float(self._gripper_deg) >= (lo_g + 0.35 * span)
            self._grip_open = grip
            ee_m = None
            ee_rpy = None
            try:
                if self._dyn_ready and not self._is_fake():
                    pos, ori = robot.get_pose()
                    ee_m = [float(x) for x in pos[:3]]
                    ee_rpy = [math.degrees(x) for x in self._rpy_from_pose(ori, robot)]
                else:
                    pos, _ori, rpy = forward_pose(q_rad)
                    ee_m = [float(x) for x in pos]
                    ee_rpy = [math.degrees(x) for x in rpy]
            except Exception:
                try:
                    pos, _ori, rpy = forward_pose(q_rad)
                    ee_m = [float(x) for x in pos]
                    ee_rpy = [math.degrees(x) for x in rpy]
                except Exception:
                    pass
            snap = ArmSnap(
                online=True,
                backend=self.backend,
                enabled=enabled,
                n=self.n,
                q_deg=q_deg,
                target_deg=tgt,
                ok=ok,
                gripper_open=grip,
                has_gripper=self.has_gripper,
                moving=moving,
                ee_m=ee_m,
                ee_rpy_deg=ee_rpy,
                ctrl_mode=self._ctrl_mode,
                gripper_deg=self._gripper_deg,
                tau_raw=tau_raw,
                motors=motors,
                **extra,
            )
            self._last = snap
            return snap
        except Exception as exc:
            logger.warning("读取臂状态失败: %s", exc)
            snap = ArmSnap(
                online=False,
                backend=self.backend,
                enabled=False,
                n=self.n,
                q_deg=list(self._last.q_deg),
                target_deg=list(self._last.target_deg),
                ok=[False] * self.n,
                gripper_open=self._last.gripper_open,
                has_gripper=self.has_gripper,
                moving=False,
                ctrl_mode=self._ctrl_mode,
                **extra,
            )
            self._last = snap
            return snap


def probe_link(
    *,
    sdk: str = "",
    cfg_path: str = "",
    port: str = "",
    has_gripper: bool = True,
    gripper_id: int = 7,
    sim: bool = False,
) -> int:
    """Connect, print joints, disconnect. Never enable or command motion."""
    arm = FafuArm(
        has_gripper=has_gripper,
        sdk=sdk,
        cfg_path=cfg_path,
        port=port,
        allow_motion=False,
        gripper_id=gripper_id,
        required=True,
    )
    if sim:
        from robot_station.adapters.fafu_sim import FakeFafuController

        arm._controller_cls = FakeFafuController
        print("模拟调试板：不开 USB 串口、不使能、不下发运动")
    else:
        print("SDK:", resolve_sdk_root(sdk))
        pm, _cls = import_fafu_sdk(resolve_sdk_root(sdk))
        print("fafu_motor:", getattr(pm, "__file__", "?"))
        print("fafu_motor: 已导入，auto_enable 将为 False")
        try:
            boards = list(pm.find_likely_debug_boards())
        except Exception as exc:
            boards = [f"<enum failed: {exc}>"]
        print("debug_boards:", boards)
    try:
        try:
            arm.start()
        except RuntimeError as exc:
            print("连接失败（未使能、未下发运动）:", exc)
            return 1
        snap = arm.poll()
        print("online:", snap.online)
        print("enabled:", snap.enabled, "(应为 False)")
        print("q_deg:", [round(x, 2) for x in snap.q_deg])
        print("ok:", snap.ok)
        print("moving:", snap.moving)
        print("limits:", snap.limits)
        if snap.enabled:
            print("警告: 电机已处于位置模式，本次探测没有调用 enable()。", file=sys.stderr)
            return 2
        return 0 if snap.online else 1
    finally:
        arm.stop()


if __name__ == "__main__":
    port = ""
    argv = [a for a in sys.argv[1:] if a != "--sim"]
    if "--port" in argv:
        i = argv.index("--port")
        if i + 1 < len(argv):
            port = argv[i + 1]
    raise SystemExit(probe_link(sim="--sim" in sys.argv, port=port))
