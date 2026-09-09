"""Official FAFU arm adapter.

Construction always uses ``auto_enable=False``. ``allow_motion=False`` (live
``arm: fafu``) is read-only: no enable / move_j / servo_j. ``arm: sim`` injects
FakeFafuController with ``allow_motion=True`` and uses the same method names
the real controller will later receive.

Live motion requires both ``STATION_ALLOW_LIVE_ARM=1`` (serial) and
``allow_motion=True`` (commands). Gravity on the real SDK prefers
``start_gravity_compensation`` when pinocchio loaded; otherwise the station
owns a MIT torque loop using ``fafu_dyn``. Fake keeps slider teach via
``teleport``.
"""
from __future__ import annotations

import logging
import math
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from robot_station.adapters.arm import ArmAdapter
from robot_station.adapters.fafu_dyn import (
    IMPEDANCE_B,
    IMPEDANCE_I,
    IMPEDANCE_K,
    clip_tau,
    compensation_torque,
)
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
from robot_station.traj import (
    TrajectoryWriter,
    clip_replay_rate,
    delete_recording,
    load_traj,
    make_traj_path,
    prepare_playback,
    resolve_traj_path,
    sample_at,
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
REPLAY_APPROACH_RATIO = 0.35
REPLAY_APPROACH_MIN_DEG_S = 8.0
REPLAY_SETTLE_S = 0.25
REPLAY_START_TOL_RAD = 0.07
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


def _station_fafu_controller_path() -> Path:
    return Path(__file__).resolve().parent.parent / "vendor_fafu" / "fafu_robot_controller.py"


def _uses_controlled_motors(obj: Any) -> bool:
    """True when the controller excludes an offline gripper from enable/servo."""
    return callable(getattr(obj, "_controlled_motor_ids", None))


def _gripper_state_unusable(st: Any) -> bool:
    """True when a cached gripper sample must not block J1–J6 motion."""
    if st is None:
        return True
    if hasattr(st, "online") and not bool(getattr(st, "online")):
        return True
    if int(getattr(st, "fault", 0) or 0):
        return True
    return False


def _force_silent_gripper_offline(robot: Any) -> None:
    """Drop a silent or faulted gripper from enable / servo / polling.

    Official ``is_enabled`` / ``enable`` / ``servo_start`` still treat a
    configured M7 as required until ``_gripper_online`` is False. A gripper
    that answers in FAULT/STOP still has a cache, so absence-only checks
    leave it in the enable set; then ``servo_start`` calls vendor enable,
    ``motor_reset`` stalls the 100 Hz writer, and J1–J6 never get servo_j.
    Do not blocking-read M7 on the writer. Fake has no ``_ht``.
    """
    grip = getattr(robot, "_gripper_motor_id", None)
    if grip is None or not bool(getattr(robot, "_has_gripper", False)):
        return
    ht = getattr(robot, "_ht", None)
    if ht is None:
        return
    gid = int(grip)
    extra = {int(x) for x in (getattr(robot, "_station_offline_ids", None) or [])}
    extra.update(int(x) for x in (getattr(robot, "_missing_motors", None) or []))
    cached = None
    if hasattr(ht, "get_cached_state"):
        try:
            cached = ht.get_cached_state(gid)
        except Exception:
            cached = None
    if gid not in extra and not _gripper_state_unusable(cached):
        return
    was_online = bool(getattr(robot, "_gripper_online", True))
    try:
        robot._gripper_online = False
    except Exception:
        pass
    missing = [int(x) for x in (getattr(robot, "_missing_motors", None) or [])]
    if gid not in missing:
        missing.append(gid)
        try:
            robot._missing_motors = missing
        except Exception:
            pass
    if not was_online:
        return
    logger.warning("夹爪 M%s 离线/故障，仅警告；关节 1–6 继续使能和控制", gid)
    joints = [int(m) for m in list(getattr(robot, "_joint_motor_ids", []) or [])]
    if joints:
        _restart_live_polling(robot, joints)


def _install_gripper_offline_gate(controller_cls: type) -> type:
    """Keep official enable/servo_start, but never require a silent gripper."""
    if getattr(controller_cls, "_station_gripper_gate", False):
        return controller_cls
    orig_enable = getattr(controller_cls, "enable", None)
    orig_start = getattr(controller_cls, "servo_start", None)

    def enable(self, *args: Any, **kwargs: Any) -> Any:
        _force_silent_gripper_offline(self)
        if callable(orig_enable):
            return orig_enable(self, *args, **kwargs)
        return None

    def servo_start(self, opts: Any = None) -> None:
        _force_silent_gripper_offline(self)
        if callable(orig_start):
            orig_start(self, opts)

    if callable(orig_enable):
        controller_cls.enable = enable
    if callable(orig_start):
        controller_cls.servo_start = servo_start
    controller_cls._station_gripper_gate = True
    return controller_cls


def _load_fafu_controller() -> Any:
    """Prefer the station overlay: gripper offline is a warning; joints 1–6 still run.

    Official SDK ``enable`` / ``is_enabled`` / polling iterate every
    ``cfg.motor_ids``. The overlay keeps that same code path but skips M7
    after the startup ping. Do not wrap/replace ``enable`` on this class.
    """
    overlay = _station_fafu_controller_path()
    if overlay.is_file():
        import importlib.util

        sys.modules.pop("fafu_robot_controller", None)
        spec = importlib.util.spec_from_file_location("fafu_robot_controller", overlay)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"无法加载 {overlay}")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["fafu_robot_controller"] = mod
        spec.loader.exec_module(mod)
        logger.info("已加载站控控制器：夹爪离线只警告，关节 1–6 在线即可使能和控制")
        return _install_gripper_offline_gate(mod.FafuRobotController)
    from fafu_robot_controller import FafuRobotController  # type: ignore

    _patch_controller_partial_axis(FafuRobotController)
    return _install_gripper_offline_gate(FafuRobotController)


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
    return pm, _load_fafu_controller()


def _unique_motor_ids(ids: list[int]) -> list[int]:
    out: list[int] = []
    seen: set[int] = set()
    for mid in ids:
        value = int(mid)
        if value in seen:
            continue
        seen.add(value)
        out.append(value)
    return out


def _cfg_motor_ids(robot: Any) -> list[int]:
    cfg = getattr(robot, "_cfg", None)
    return [int(m) for m in list(getattr(cfg, "motor_ids", []) or [])]


def _absent_from_cached_states(robot: Any, motor_ids: list[int]) -> list[int]:
    """IDs with no cached state, but only when at least one other motor answered.

    An empty cache must not look like every motor is missing.
    """
    try:
        raw = robot.get_motor_states(prefer_cache=True)
    except Exception:
        return []
    states = raw if isinstance(raw, dict) else {}
    present: list[int] = []
    absent: list[int] = []
    for mid in motor_ids:
        st = states.get(int(mid))
        if st is None or not bool(getattr(st, "online", True)):
            absent.append(int(mid))
        else:
            present.append(int(mid))
    if not present:
        return []
    return absent


class _CfgMotorIds:
    """Python view over SDK RobotConfig with a truncated ``motor_ids``.

    Assigning ``cfg.motor_ids`` on the C++ ``RobotConfig`` may not stick;
    ``FafuRobotController`` still iterates ``self._cfg.motor_ids`` in
    Python (enable, servo_start max id, hold list). Shadow it here.
    """

    def __init__(self, inner: Any, motor_ids: list[int]) -> None:
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "motor_ids", list(motor_ids))

    def __getattr__(self, name: str) -> Any:
        return getattr(object.__getattribute__(self, "_inner"), name)

    def __setattr__(self, name: str, value: Any) -> None:
        if name in {"motor_ids", "_inner"}:
            object.__setattr__(self, name, list(value) if name == "motor_ids" else value)
            return
        setattr(object.__getattribute__(self, "_inner"), name, value)


def _set_cfg_motor_ids(robot: Any, ids: list[int]) -> None:
    cfg = getattr(robot, "_cfg", None)
    if cfg is None:
        robot._cfg = SimpleNamespace(motor_ids=list(ids))
        return
    if isinstance(cfg, _CfgMotorIds):
        cfg.motor_ids = list(ids)
        return
    # Always wrap. pybind RobotConfig.motor_ids assignment can look like it
    # stuck in Python while enable/_switch_mode_all still iterate the C++ vector.
    robot._cfg = _CfgMotorIds(cfg, ids)


def _state_is_absent(st: Any) -> bool:
    if st is None:
        return True
    if hasattr(st, "online") and not bool(getattr(st, "online")):
        return True
    return False


def _probe_absent_motors(robot: Any, motor_ids: list[int]) -> list[int]:
    """IDs that do not answer. Prefer cache; one short live read if needed.

    A motor that answers in STOP/FAULT is still present (do not drop it here).
    Completely silent IDs must not fail the rest of the arm.
    """
    ht = getattr(robot, "_ht", None)
    skip = _skip_motor_ids(robot)
    from_cache = _absent_from_cached_states(robot, motor_ids)
    if ht is None or not hasattr(ht, "read_motor_state"):
        return from_cache
    absent: list[int] = []
    present: list[int] = []
    for mid in motor_ids:
        if int(mid) in skip:
            absent.append(int(mid))
            continue
        st = None
        try:
            if hasattr(ht, "get_cached_state"):
                st = ht.get_cached_state(int(mid))
        except Exception:
            st = None
        if _state_is_absent(st):
            try:
                st = ht.read_motor_state(int(mid), 0.12)
            except Exception:
                st = None
        if _state_is_absent(st):
            absent.append(int(mid))
        else:
            present.append(int(mid))
    if not present:
        return []
    return absent


def _skip_motor_ids(robot: Any, extra: list[int] | None = None) -> set[int]:
    skip = {int(x) for x in (getattr(robot, "_missing_motors", None) or [])}
    skip.update(int(x) for x in (getattr(robot, "_station_offline_ids", None) or []))
    skip.update(int(x) for x in (extra or []))
    return skip


def _command_joint_ids(
    robot: Any,
    *,
    gripper_id: int | None = None,
    extra: list[int] | None = None,
) -> list[int]:
    """Joints that may be powered. The gripper is never required to enable the arm."""
    drop_absent_motors(robot, gripper_id=gripper_id, extra=extra)
    skip = _skip_motor_ids(robot, extra)
    joints = [
        int(m)
        for m in list(
            getattr(robot, "_joint_motor_ids", None)
            or getattr(robot, "joint_motor_ids", [])
            or []
        )
    ]
    if gripper_id is not None and int(gripper_id) not in set(joints):
        skip.add(int(gripper_id))
    live = [m for m in joints if m not in skip]
    if live:
        return live
    live = [m for m in _cfg_motor_ids(robot) if m not in skip]
    if gripper_id is not None:
        live = [m for m in live if m != int(gripper_id)]
    return live


def _vendor_partial_axis_text(msg: str) -> bool:
    low = (msg or "").lower()
    return "motor_reset" in low or "did not respond" in low or "enable failed" in low


def _vendor_partial_axis_error(exc: BaseException) -> bool:
    return _vendor_partial_axis_text(str(exc))


def _bind_partial_axis_enable(robot: Any, gripper_id: int | None) -> None:
    """Shadow instance enable/_enable_impl so SDK internals cannot motor_reset-all."""

    def _run(*, allow_motor_reset: bool = True) -> None:
        extra = list(getattr(robot, "_station_offline_ids", None) or [])
        _enable_present_motors(
            robot,
            gripper_id=gripper_id,
            extra=extra,
            allow_motor_reset=allow_motor_reset,
        )

    try:
        robot.enable = _run
        robot._enable_impl = _run
    except Exception:
        logger.exception("无法把 enable 绑到在线轴")


def _live_joints_have_rx(robot: Any) -> bool:
    ht = getattr(robot, "_ht", None)
    ids = [int(m) for m in list(getattr(robot, "_joint_motor_ids", []) or [])]
    skip = _skip_motor_ids(robot)
    ids = [m for m in ids if m not in skip]
    if ht is None or not ids:
        return False
    for mid in ids:
        try:
            s = ht.get_cached_state(int(mid)) if hasattr(ht, "get_cached_state") else None
        except Exception:
            s = None
        if s is not None:
            return True
    return False


def _restart_live_polling(robot: Any, ids: list[int]) -> None:
    ht = getattr(robot, "_ht", None)
    if ht is None or not ids or not hasattr(ht, "start_state_polling"):
        return
    try:
        cfg = getattr(robot, "_cfg", None)
        hz = float(getattr(cfg, "control_rate_hz", 50.0) or 50.0)
        ht.start_state_polling([int(m) for m in ids], max(10.0, hz))
    except Exception:
        logger.warning("按在线轴重开状态轮询失败", exc_info=True)


def _prepare_live_servo_session(
    robot: Any,
    *,
    gripper_id: int | None = None,
    extra: list[int] | None = None,
) -> list[int]:
    """Shrink the SDK servo set to answering joints before servo_start.

    Official servo_start uses max(cfg.motor_ids) as the CAN slot count.
    A missing gripper (id 7) makes every 100 Hz frame a 7-slot payload
    and can re-enter vendor enable(). Live joints only.
    """
    live = _command_joint_ids(robot, gripper_id=gripper_id, extra=extra)
    if not live:
        raise RuntimeError("没有在线关节电机，无法开始伺服")
    if getattr(robot, "_ht", None) is not None:
        robot._joint_motor_ids = list(live)
        _set_cfg_motor_ids(robot, list(live))
        _restart_live_polling(robot, live)
    _lift_false_dead(robot)
    try:
        _enable_present_motors(
            robot,
            gripper_id=gripper_id,
            extra=extra,
        )
    except RuntimeError as exc:
        if "正忙" not in str(exc) and not _vendor_partial_axis_error(exc):
            raise
    return live


def _lift_false_dead(robot: Any) -> bool:
    """Undo a DEAD latch caused by a silent gripper while joints still reply."""
    value = str(
        getattr(getattr(robot, "_state", None), "value", getattr(robot, "_state", "")) or ""
    ).lower()
    if "dead" not in value:
        return False
    if not _live_joints_have_rx(robot):
        return False
    recover = getattr(robot, "recover", None)
    if callable(recover):
        try:
            recover(confirm=True)
            return True
        except Exception:
            logger.warning("误判 DEAD，recover 失败", exc_info=True)
    setter = getattr(robot, "_set_state", None)
    state = getattr(robot, "_state", None)
    idle = getattr(type(state), "IDLE", None) if state is not None else None
    disabled = getattr(type(state), "DISABLED", None) if state is not None else None
    nxt = idle or disabled
    if callable(setter) and nxt is not None:
        try:
            robot._dead_reason = None
            setter(nxt)
            return True
        except Exception:
            return False
    return False


def _live_rx_age_ms(robot: Any) -> float | None:
    ht = getattr(robot, "_ht", None)
    if ht is None or not hasattr(ht, "get_stats"):
        return None
    try:
        return float(getattr(ht.get_stats(), "last_rx_age_ms", 0.0) or 0.0)
    except Exception:
        return None


def _hardware_link_up(robot: Any) -> bool:
    """True while the USB-CAN link is actually exchanging frames.

    A leftover MotorState cache after unplug must not keep the arm "connected".
    """
    if robot is None:
        return False
    name = _robot_state_name(robot)
    if name in ("dead", "disconnected"):
        return False
    ht = getattr(robot, "_ht", None)
    if ht is None:
        # Fake plant and live test doubles have no serial. Unplug is detected
        # only when a real _ht still exists but RX has gone stale.
        return True
    try:
        opened = getattr(ht, "is_open", None)
        if callable(opened) and not bool(opened()):
            return False
        if isinstance(opened, bool) and not opened:
            return False
    except Exception:
        return False
    stream = getattr(robot, "_stream_link_ok", None)
    if callable(stream):
        try:
            if not bool(stream()):
                return False
        except Exception:
            return False
    timeout = float(getattr(robot, "_dead_rx_timeout_ms", 500.0) or 500.0)
    age = _live_rx_age_ms(robot)
    if age is not None and age > timeout:
        return False
    return True


def _station_stream_link_ok(robot: Any, orig: Any = None) -> bool:
    """Keep the link when joints still RX. USB unplug / bus silence is a real drop.

    A silent gripper must not latch DEAD (joints still produce RX, age stays low).
    Stale cached joint states after the cable is gone must not keep the link up.
    """
    ht = getattr(robot, "_ht", None)
    try:
        if ht is None or not hasattr(ht, "is_async_rx") or not ht.is_async_rx():
            return True
        age = float(getattr(ht.get_stats(), "last_rx_age_ms", 0.0) or 0.0)
    except Exception:
        return False
    timeout = float(getattr(robot, "_dead_rx_timeout_ms", 500.0) or 500.0)
    if age <= timeout:
        return True
    enter = getattr(robot, "_enter_dead", None)
    if callable(enter):
        try:
            enter(
                f"no CAN RX for {age:.0f} ms (motor power / USB / bus lost mid-stream?)"
            )
        except Exception:
            pass
    return False


def _bind_stream_link_ok(robot: Any) -> None:
    orig = getattr(robot, "_stream_link_ok", None)
    if not callable(orig) or getattr(robot, "_station_stream_link", False):
        return

    def _ok() -> bool:
        return _station_stream_link_ok(robot, orig)

    try:
        robot._stream_link_ok = _ok
        robot._station_stream_link = True
    except Exception:
        logger.exception("无法接管链路检测")


def drop_absent_motors(robot: Any, *, gripper_id: int | None = None, extra: list[int] | None = None) -> list[int]:
    """Drop unresponsive IDs from the SDK enable set so remaining axes can run.

    Official ``enable()`` / ``is_enabled()`` iterate every ``cfg.motor_ids``.
    Delivery kits often omit a gripper or a joint; those IDs must not fail
    the whole arm. Do not edit the vendor SDK. C++ ``RobotConfig.motor_ids``
    may ignore Python assignment, so callers must still skip missing IDs
    instead of relying on this mutation alone.
    """
    configured = _cfg_motor_ids(robot)
    if not configured:
        joints = [int(m) for m in list(getattr(robot, "joint_motor_ids", []) or [])]
        configured = list(joints)
        if gripper_id is not None and int(gripper_id) not in configured:
            configured.append(int(gripper_id))
        if configured:
            _set_cfg_motor_ids(robot, configured)
    probe_ids = list(configured)
    known = [int(m) for m in (getattr(robot, "_missing_motors", None) or [])]
    extra_ids = [int(m) for m in (extra or [])]
    if gripper_id is not None and int(gripper_id) not in probe_ids:
        if int(gripper_id) not in set(known + extra_ids):
            probe_ids.append(int(gripper_id))
    from_states = _probe_absent_motors(robot, probe_ids)
    absent = _unique_motor_ids(known + extra_ids + from_states)
    robot._missing_motors = list(absent)
    if not absent:
        return []
    live = [m for m in configured if m not in set(absent)]
    if not live:
        logger.warning("配置的电机均无应答：%s", absent)
        return absent
    if live != configured:
        _set_cfg_motor_ids(robot, live)
        logger.warning("交付缺轴 %s，仅使能在线电机 %s", absent, live)
    # Keep SDK ``_has_gripper`` True so servo_start still *excludes* M7 from
    # the per-tick hold list. Clearing it while M7 remains in motor_ids
    # makes every servo_j try to hold the missing gripper.
    joint_ids = [int(m) for m in list(getattr(robot, "_joint_motor_ids", None) or getattr(robot, "joint_motor_ids", []) or [])]
    live_joints = [m for m in joint_ids if m not in set(absent)]
    # Live servo_start reads every joint_motor_ids starting pose. Fake has no
    # ``_ht`` and keeps a 6-vector plant; only shrink the live SDK joint list.
    if live_joints and live_joints != joint_ids and getattr(robot, "_ht", None) is not None:
        robot._joint_motor_ids = live_joints
    return absent


def switch_motors_to_position(
    robot: Any,
    motor_ids: list[int],
    *,
    allow_motor_reset: bool = True,
) -> None:
    """Put only ``motor_ids`` into position mode. Never touches omitted axes."""
    ids = [int(m) for m in motor_ids]
    ht = getattr(robot, "_ht", None)
    if ht is None or not ids:
        return
    mode_pos = int(getattr(robot, "MODE_POSITION", 0x0A) or 0x0A)

    def current_mode(mid: int) -> int | None:
        try:
            s = ht.read_motor_state(int(mid), 0.2)
        except Exception:
            return None
        if s is None:
            return None
        return int(getattr(s, "mode", 0) or 0)

    pending = [mid for mid in ids if current_mode(mid) != mode_pos]
    if not pending:
        return
    failed: list[int] = []
    for mid in pending:
        s = None
        try:
            if hasattr(ht, "set_motor_mode"):
                s = ht.set_motor_mode(int(mid), mode_pos)
        except Exception:
            s = None
        got = int(getattr(s, "mode", 0) or 0) if s is not None else current_mode(mid)
        if got != mode_pos:
            failed.append(mid)
    if failed and allow_motor_reset and hasattr(ht, "motor_reset"):
        for mid in failed:
            try:
                ht.motor_reset(int(mid))
            except Exception:
                logger.warning("motor_reset(%s) 失败", mid)
        time.sleep(1.0)
        still: list[int] = []
        for mid in failed:
            try:
                if hasattr(ht, "set_motor_mode"):
                    ht.set_motor_mode(int(mid), mode_pos)
            except Exception:
                pass
            if current_mode(mid) != mode_pos:
                still.append(mid)
        failed = still
    ok = [mid for mid in ids if current_mode(mid) == mode_pos]
    if not ok:
        raise RuntimeError(f"在线轴 {failed or ids} 未能切入位置环（已跳过离线电机）")
    if failed:
        logger.warning("部分关节未能切入位置环 %s，已使能 %s", failed, ok)


def _instance_enabled_flag(robot: Any) -> bool:
    """Read the Fake/instance ``is_enabled`` bool, bypassing any property."""
    try:
        return bool(vars(robot).get("is_enabled", False))
    except Exception:
        return False


def _enable_present_motors(
    robot: Any,
    *,
    gripper_id: int | None = None,
    extra: list[int] | None = None,
    allow_motor_reset: bool = True,
) -> None:
    """Enable responding joints only. Do not call vendor ``_enable_impl``."""
    value = str(getattr(getattr(robot, "_state", None), "value", getattr(robot, "_state", "")) or "").lower()
    if value == "disconnected":
        raise RuntimeError("enable 失败: 连接已关闭 (state=DISCONNECTED)。")
    if value == "dead":
        _lift_false_dead(robot)
        value = str(getattr(getattr(robot, "_state", None), "value", getattr(robot, "_state", "")) or "").lower()
    if value == "dead":
        raise RuntimeError("enable 被拒绝: 掉电/通信丢失锁定 (DEAD)。请先 recover(confirm=True)。")
    if value in ("servoing", "moving", "grasping", "gravity_comp"):
        raise RuntimeError(f"enable 被拒绝: 机械臂正忙 (state={value})。")
    live = _command_joint_ids(robot, gripper_id=gripper_id, extra=extra)
    if not live:
        raise RuntimeError("没有在线关节电机，无法使能")
    if getattr(robot, "_ht", None) is None:
        require = getattr(robot, "_require_motion", None)
        if callable(require):
            require("enable")
        _mark_controller_enabled(robot)
        return
    logger.info("只使能在线关节 %s（已跳过离线电机）", live)
    switch_motors_to_position(robot, live, allow_motor_reset=allow_motor_reset)
    _mark_controller_enabled(robot)


def _mark_controller_enabled(robot: Any) -> None:
    """Fake uses an instance bool; live SDK uses RobotState.IDLE + mode bits."""
    try:
        robot.is_enabled = True
    except Exception:
        try:
            vars(robot)["is_enabled"] = True
        except Exception:
            pass
    setter = getattr(robot, "_set_state", None)
    state = getattr(robot, "_state", None)
    idle = getattr(type(state), "IDLE", None) if state is not None else None
    if callable(setter) and idle is not None:
        try:
            setter(idle)
        except Exception:
            logger.warning("在线轴已在位置环，但未能把 SDK 状态标成 IDLE")


def switch_motors_mode(
    robot: Any,
    motor_ids: list[int],
    mode: int,
    *,
    max_retry: int = 3,
) -> bool:
    """Switch only ``motor_ids``. Silent / omitted IDs are not a failure."""
    ids = [int(m) for m in motor_ids]
    ht = getattr(robot, "_ht", None)
    if ht is None or not ids or not hasattr(ht, "set_motor_mode"):
        return True
    target = int(mode)
    for attempt in range(1, max_retry + 1):
        failed: list[int] = []
        for mid in ids:
            s = None
            try:
                s = ht.set_motor_mode(int(mid), target)
            except Exception:
                s = None
            if s is None or int(getattr(s, "mode", 0) or 0) != target:
                failed.append(int(mid))
        if failed:
            time.sleep(0.03)
            really: list[int] = []
            for mid in failed:
                s = None
                try:
                    if hasattr(ht, "read_motor_state"):
                        s = ht.read_motor_state(int(mid), 0.3)
                except Exception:
                    s = None
                if s is None or int(getattr(s, "mode", 0) or 0) != target:
                    really.append(mid)
            failed = really
        if not failed:
            return True
        if attempt < max_retry:
            time.sleep(0.1)
    return False


def _soft_precheck_communication(robot: Any) -> None:
    ht = getattr(robot, "_ht", None)
    cfg = getattr(robot, "_cfg", None)
    ids = list(getattr(cfg, "motor_ids", []) or [])
    grip = getattr(robot, "_gripper_motor_id", None)
    if ht is None or not ids or not hasattr(ht, "read_motor_state"):
        drop_absent_motors(robot, gripper_id=grip if grip is not None else None)
        return
    skip = _skip_motor_ids(robot)
    bad: list[int] = []
    for mid in ids:
        if int(mid) in skip:
            bad.append(int(mid))
            continue
        try:
            s = ht.read_motor_state(int(mid), 0.5)
        except Exception:
            s = None
        if s is None:
            bad.append(int(mid))
    robot._missing_motors = bad
    if bad:
        logger.warning(
            "电机 %s 未在 500ms 内应答，仍保持调试板连接；缺轴在电机表标离线，不把整臂标成未连接",
            bad,
        )
    drop_absent_motors(robot, gripper_id=grip if grip is not None else None)


def _station_is_enabled(robot: Any) -> bool:
    skip = _skip_motor_ids(robot)
    grip = getattr(robot, "_gripper_motor_id", None)
    ids = [int(m) for m in list(getattr(robot, "_joint_motor_ids", []) or [])]
    if grip is not None and int(grip) not in set(ids):
        skip.add(int(grip))
    ids = [m for m in ids if m not in skip]
    if not ids:
        return False
    ht = getattr(robot, "_ht", None)
    if ht is None:
        return _instance_enabled_flag(robot)
    mode_pos = int(getattr(robot, "MODE_POSITION", 0x0A) or 0x0A)
    for mid in ids:
        s = None
        try:
            if hasattr(ht, "get_cached_state"):
                s = ht.get_cached_state(int(mid))
        except Exception:
            s = None
        if s is None and hasattr(ht, "read_motor_state"):
            try:
                s = ht.read_motor_state(int(mid), 0.05)
            except Exception:
                s = None
        if s is None or int(getattr(s, "mode", 0) or 0) != mode_pos:
            return False
    return True


def _patch_controller_partial_axis(controller_cls: type) -> type:
    """Replace vendor all-motor enable so a silent ID cannot fail the arm.

    Subclass wrapping is not enough: ``FafuRobotController.enable`` looks up
    ``_enable_impl`` on the instance, but construction / servo_start may still
    hit the original class methods if wrapping is skipped. Patch the imported
    class in-place (once).
    """
    if getattr(controller_cls, "_station_partial_axis", False):
        return controller_cls
    if _uses_controlled_motors(controller_cls):
        return controller_cls
    if not hasattr(controller_cls, "_enable_impl"):
        return controller_cls

    def _precheck(self) -> None:
        _soft_precheck_communication(self)

    def _enable_impl(self, *, allow_motor_reset: bool = True) -> None:
        grip = getattr(self, "_gripper_motor_id", None)
        extra = list(getattr(self, "_station_offline_ids", None) or [])
        _enable_present_motors(
            self,
            gripper_id=grip if grip is not None else None,
            extra=extra,
            allow_motor_reset=allow_motor_reset,
        )

    def _switch_mode_all(self, mode: int, *, label: str = "", max_retry: int = 3) -> bool:
        grip = getattr(self, "_gripper_motor_id", None)
        extra = list(getattr(self, "_station_offline_ids", None) or [])
        drop_absent_motors(self, gripper_id=grip if grip is not None else None, extra=extra)
        skip = _skip_motor_ids(self, extra)
        ids = [m for m in _cfg_motor_ids(self) if m not in skip]
        if not ids:
            ids = _command_joint_ids(self, gripper_id=grip, extra=extra)
        if not ids:
            return False
        return switch_motors_mode(self, ids, int(mode), max_retry=max_retry)

    def is_enabled(self) -> bool:
        return _station_is_enabled(self)

    def enable(self, *, allow_motor_reset: bool = True) -> None:
        value = str(getattr(getattr(self, "_state", None), "value", getattr(self, "_state", "")) or "").lower()
        if value == "disconnected":
            raise RuntimeError("enable 失败: 连接已关闭 (state=DISCONNECTED)。")
        if value == "dead":
            _lift_false_dead(self)
            value = str(getattr(getattr(self, "_state", None), "value", getattr(self, "_state", "")) or "").lower()
        if value == "dead":
            raise RuntimeError("enable 被拒绝: 掉电/通信丢失锁定 (DEAD)。请先 recover(confirm=True)。")
        if value in ("servoing", "moving", "grasping", "gravity_comp"):
            raise RuntimeError(f"enable 被拒绝: 机械臂正忙 (state={value})。")
        _enable_impl(self, allow_motor_reset=allow_motor_reset)
        setter = getattr(self, "_set_state", None)
        state = getattr(self, "_state", None)
        idle = getattr(type(state), "IDLE", None) if state is not None else None
        if callable(setter) and idle is not None:
            try:
                setter(idle)
            except Exception:
                pass

    _orig_stream_link_ok = getattr(controller_cls, "_stream_link_ok", None)
    _orig_servo_start = getattr(controller_cls, "servo_start", None)

    def _stream_link_ok(self) -> bool:
        def _orig() -> bool:
            if callable(_orig_stream_link_ok):
                return bool(_orig_stream_link_ok(self))
            return False

        return _station_stream_link_ok(self, _orig)

    def servo_start(self, opts: Any = None) -> None:
        grip = getattr(self, "_gripper_motor_id", None)
        extra = list(getattr(self, "_station_offline_ids", None) or [])
        live = _prepare_live_servo_session(
            self,
            gripper_id=grip if grip is not None else None,
            extra=extra,
        )
        if callable(_orig_servo_start):
            _orig_servo_start(self, opts)
        if live:
            self._servo_max_motor_id = int(max(live))
            self._servo_mit_max_motor_id = int(max(live))

    controller_cls._precheck_communication = _precheck
    controller_cls._enable_impl = _enable_impl
    controller_cls._switch_mode_all = _switch_mode_all
    controller_cls.enable = enable
    controller_cls.is_enabled = property(is_enabled)
    controller_cls._stream_link_ok = _stream_link_ok
    if callable(_orig_servo_start):
        controller_cls.servo_start = servo_start
    controller_cls._station_partial_axis = True
    logger.info("已接管 SDK 使能：离线电机不再阻止在线轴")
    return controller_cls


def _with_soft_precheck(controller_cls: type) -> type:
    """Keep the debug-board link when some motors (often the gripper) are missing.

    Official ``enable()`` / ``_enable_impl`` / ``is_enabled`` iterate every
    ``cfg.motor_ids``. Delivery kits omit axes; those IDs must not ``motor_reset``
    the rest of the arm. Always wrap — not only when ``_precheck_communication``
    exists — because ``servo_start`` still calls ``enable()`` → ``_enable_impl``.
    """

    if _uses_controlled_motors(controller_cls):
        return controller_cls

    class SoftPrecheckController(controller_cls):  # type: ignore[misc,valid-type]
        def _precheck_communication(self) -> None:  # type: ignore[override]
            _soft_precheck_communication(self)

        @property
        def is_enabled(self) -> bool:  # type: ignore[override]
            return _station_is_enabled(self)

        @is_enabled.setter
        def is_enabled(self, value: bool) -> None:
            vars(self)["is_enabled"] = bool(value)

        def _enable_impl(self, *, allow_motor_reset: bool = True) -> None:  # type: ignore[override]
            grip = getattr(self, "_gripper_motor_id", None)
            extra = list(getattr(self, "_station_offline_ids", None) or [])
            _enable_present_motors(
                self,
                gripper_id=grip if grip is not None else None,
                extra=extra,
                allow_motor_reset=allow_motor_reset,
            )

        def enable(self, *, allow_motor_reset: bool = True) -> None:  # type: ignore[override]
            grip = getattr(self, "_gripper_motor_id", None)
            extra = list(getattr(self, "_station_offline_ids", None) or [])
            _enable_present_motors(
                self,
                gripper_id=grip if grip is not None else None,
                extra=extra,
                allow_motor_reset=allow_motor_reset,
            )

        def _switch_mode_all(self, mode: int, *, label: str = "", max_retry: int = 3) -> bool:  # type: ignore[override]
            grip = getattr(self, "_gripper_motor_id", None)
            extra = list(getattr(self, "_station_offline_ids", None) or [])
            drop_absent_motors(self, gripper_id=grip if grip is not None else None, extra=extra)
            skip = _skip_motor_ids(self, extra)
            ids = [m for m in _cfg_motor_ids(self) if m not in skip]
            if not ids:
                ids = _command_joint_ids(self, gripper_id=grip, extra=extra)
            if not ids:
                return False
            return switch_motors_mode(self, ids, int(mode), max_retry=max_retry)

        def _stream_link_ok(self) -> bool:  # type: ignore[override]
            def _orig() -> bool:
                parent = getattr(super(SoftPrecheckController, self), "_stream_link_ok", None)
                if callable(parent):
                    return bool(parent())
                return False

            return _station_stream_link_ok(self, _orig)

        def servo_start(self, opts: Any = None) -> None:  # type: ignore[override]
            grip = getattr(self, "_gripper_motor_id", None)
            extra = list(getattr(self, "_station_offline_ids", None) or [])
            live = _prepare_live_servo_session(
                self,
                gripper_id=grip if grip is not None else None,
                extra=extra,
            )
            super().servo_start(opts)
            if live:
                self._servo_max_motor_id = int(max(live))
                self._servo_mit_max_motor_id = int(max(live))

    SoftPrecheckController.__name__ = str(getattr(controller_cls, "__name__", "Controller"))
    SoftPrecheckController.__qualname__ = SoftPrecheckController.__name__
    return SoftPrecheckController


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
        self._rec: TrajectoryWriter | None = None
        self._rec_teach = ""
        self._rec_prev_q: list[float] | None = None
        self._rec_prev_mono = 0.0
        self._grip_cmd_deg = float(self._gripper_deg)
        self._replay_frames: list[dict[str, Any]] = []
        self._replay_t0: float | None = None
        self._replay_rate = 1.0
        self._replay_settle_left = 0.0
        self._rec_entered_float = False
        self._traj_name = ""
        self._stream_rx_mono = 0.0
        self._q_fb_err = ""
        self._full_joint_ids: list[int] = list(range(1, self.n + 1))
        self._offline_ids: list[int] = []
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

    def _station_grav_ok(self) -> bool:
        """Station ``G(q)`` + MIT torque send, no pinocchio."""
        robot = self._robot
        if robot is None or not self.allow_motion or self.n != 6:
            return False
        if bool(getattr(robot, "sim_teach", False)):
            return False
        return callable(getattr(robot, "apply_compensation_torque", None))

    def _float_modes_available(self) -> bool:
        """Gravity / Gra+Fri / Impedance: sim teach, pinocchio SDK, or station G(q)."""
        if self._use_sdk_float():
            robot = self._robot
            if robot is not None and not self._arm_joints_complete(robot):
                return False
            return bool(self._dyn_ready or self._station_grav_ok())
        return bool(self._is_fake() or self.backend == "sim")

    def _float_modes_reason(self) -> str:
        if self._float_modes_available():
            return ""
        robot = self._robot
        if robot is not None and self._use_sdk_float() and not self._arm_joints_complete(robot):
            missing = self._missing_arm_joint_ids(robot)
            return f"J1–J6 必须全部在线才能重力补偿（缺 {missing}）"
        if self._use_sdk_float() or (self.allow_motion and self._robot is not None):
            return "真机力矩环未加载"
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
            robot = controller_cls(**kwargs) if _uses_controlled_motors(controller_cls) else _with_soft_precheck(controller_cls)(**kwargs)
            _force_silent_gripper_offline(robot)
            _install_gripper_offline_gate(type(robot))
            joints = [int(m) for m in list(getattr(robot, "joint_motor_ids", []) or [])]
            self._full_joint_ids = joints[: self.n] or list(range(1, self.n + 1))
            missing = drop_absent_motors(
                robot, gripper_id=self.gripper_id, extra=self._offline_ids
            )
            if self.has_gripper and not bool(getattr(robot, "_gripper_online", True)):
                gid = int(self.gripper_id)
                if gid not in missing:
                    missing = list(missing) + [gid]
                    try:
                        robot._missing_motors = list(missing)
                    except Exception:
                        pass
            if missing:
                logger.warning("已连接调试板，但电机 %s 未启动；在线轴仍可控制", missing)
            # Overlay still has official enable(). Bind the station proxy so
            # servo_start cannot motor_reset the whole arm for a dead gripper.
            _bind_partial_axis_enable(robot, self.gripper_id)
            _bind_stream_link_ok(robot)
            live = [m for m in (joints[: self.n] or []) if m not in set(missing)]
            if live and getattr(robot, "_ht", None) is not None:
                _restart_live_polling(robot, live)
            self._apply_limits_from_robot(robot)
            self._setup_dynamics(robot, sim=sim)
            with self._lock:
                self._robot = robot
                self._link_error = ""
                self._hold_latched = False
                self._offline_ids = list(missing)
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

    def _prepare_torque_send(self, robot: Any) -> None:
        """Nm→raw tables for MIT feed-forward. Does not load pinocchio."""
        ids = self._sdk_joint_ids(robot)
        if not ids or self.n != len(FAFU_JOINT_MOTOR_MODELS):
            return
        orig = list(self._full_joint_ids) or list(range(1, self.n + 1))
        models_all = list(FAFU_JOINT_MOTOR_MODELS)
        lim_all = list(FAFU_TAU_LIMIT_NM)
        model_by = {int(orig[i]): models_all[i] for i in range(min(len(orig), len(models_all)))}
        lim_by = {int(orig[i]): float(lim_all[i]) for i in range(min(len(orig), len(lim_all)))}
        models = [model_by.get(int(mid), models_all[0]) for mid in ids]
        limits = [lim_by.get(int(mid), 15.0) for mid in ids]
        try:
            current = getattr(robot, "_dyn_motor_models", None)
            if not current or len(list(current)) != len(ids):
                robot._dyn_motor_models = models
        except Exception:
            logger.warning("无法写入 motor_models，重力力矩可能未按电机系数换算")
        setter = getattr(robot, "set_torque_scale", None)
        if callable(setter):
            try:
                setter(1.0)
            except Exception:
                logger.warning("set_torque_scale 失败，沿用控制器默认增益")
        try:
            current_lim = getattr(robot, "_dyn_tau_limit", None)
            if current_lim is None or len(list(current_lim)) != len(ids):
                robot._dyn_tau_limit = limits
        except Exception:
            pass

    def _setup_dynamics(self, robot: Any, *, sim: bool) -> None:
        self._dyn_ready = False
        if not sim:
            self._prepare_torque_send(robot)
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
            logger.warning(
                "setup_dynamics 失败（笛卡尔走站控 IK；重力改用站控 G(q)+MIT）: %s",
                exc,
            )

    def stop(self) -> None:
        self._servo_intent = False
        self._pending_grip = None
        self._stop_recording()
        self._rec_entered_float = False
        self._abort_replay(hold=False)
        self._stop_gravity(join=True)
        self._wait_servo_idle(1.0)
        self._end_servo()
        with self._lock:
            robot = self._robot
            self._robot = None
            self._offline_ids = []
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

    def _drop_lost_live_link(self) -> None:
        """USB/bus gone: mark disconnected and free the serial for Connect.

        Do not call stop() from poll() — that waits for the 100 Hz writer and
        can stall the tick that invoked poll.
        """
        if self._is_fake():
            return
        robot = self._robot
        if robot is None:
            return
        logger.warning("真机 USB/总线已断开，标为未连接")
        if not self._link_error:
            self._link_error = "USB/总线已断开"
        self._servo_intent = False
        self._servoing = False
        self._writer_busy = False
        try:
            self._grav_stop.set()
        except Exception:
            pass
        try:
            self._servo_idle.set()
        except Exception:
            pass
        with self._lock:
            if self._robot is robot:
                self._robot = None
                self._offline_ids = []
        try:
            robot.close_connection(joint_release="hold", gripper_release="hold")
        except TypeError:
            try:
                robot.close_connection()
            except Exception:
                logger.exception("拔线后关闭串口失败")
        except Exception:
            logger.exception("拔线后关闭串口失败")

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
                cur = self._read_joint_q_rad(robot)
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
                    hold = self._read_joint_q_rad(robot)
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

    def _replay_approach_waypoints(self, q_now_deg: list[float], q_goal_deg: list[float]) -> list[list[float]]:
        """Move to the taught start. Only lift J3 if a straight lerp hits the table.

        Do not zero J5/J6 (the home/reset detour). That wrist snap plus an
        early timestamp playback is what made replay-approach chatter.
        """
        now = self._clip_deg(q_now_deg)
        goal = self._clip_deg(q_goal_deg)
        pts: list[list[float]] = []
        if not self._lerp_clears_table(now, goal):
            via = list(now)
            via[2] = self._min_j3_for_table_clear(now, goal)
            pts.append(via)
        pts.append(goal)
        return self._dedupe_waypoints(pts)

    @staticmethod
    def _replay_approach_speed(speed_deg_s: float) -> float:
        return max(REPLAY_APPROACH_MIN_DEG_S, float(speed_deg_s) * REPLAY_APPROACH_RATIO)

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
                return [math.degrees(x) for x in self._read_joint_q_rad(robot)]
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
        return bool(self._home_path) or bool(self._replay_frames)

    @property
    def is_replaying(self) -> bool:
        return bool(self._replay_frames)

    def _traj_snap(self) -> dict[str, Any]:
        rec = self._rec
        rec_s = 0.0
        if rec is not None:
            rec_s = max(0.0, time.monotonic() - rec.t0)
        return {
            "recording": rec is not None,
            "rec_frames": int(rec.count) if rec is not None else 0,
            "rec_s": rec_s,
            "rec_teach": self._rec_teach,
            "replay_active": bool(self._replay_frames),
            "traj_file": self._traj_name,
        }

    def _abort_replay(self, *, hold: bool = True) -> None:
        was = bool(self._replay_frames) or self._replay_t0 is not None
        self._replay_frames = []
        self._replay_t0 = None
        self._replay_rate = 1.0
        self._replay_settle_left = 0.0
        if was:
            self._home_path = []
            if hold:
                self._hold_stream_pose()

    def _stop_recording(self) -> str | None:
        rec = self._rec
        self._rec = None
        self._rec_prev_q = None
        self._rec_prev_mono = 0.0
        if rec is None:
            return None
        rec.close()
        self._traj_name = rec.path.name
        self._rec_teach = rec.teach
        return None

    def record_start(self, name: str | None = None, teach: str | None = None) -> str | None:
        if self._robot is None:
            return "机械臂未连接"
        if not self.allow_motion:
            return "真臂当前只读，不能录制"
        if self._replay_frames:
            return "回放中请先停止再录制"
        if self._rec is not None:
            return "已在录制"
        kind = (teach or "").strip().lower()
        self._rec_entered_float = False
        if kind == "drag":
            if self._ctrl_mode not in ("Gravity", "Gra+Fri"):
                err = self.set_ctrl_mode("Gra+Fri")
                if err:
                    err = self.set_ctrl_mode("Gravity")
                if err:
                    return err
                self._rec_entered_float = True
        elif kind == "soft":
            if self._ctrl_mode in FLOAT_MODES or self._float_writer_active():
                err = self._reenter_position()
                if err:
                    return err
                self._ctrl_mode = "Position"
        mode_teach = "drag" if self._ctrl_mode in ("Gravity", "Gra+Fri") else "soft"
        path = make_traj_path(name)
        self._rec_prev_q = None
        self._rec_prev_mono = time.monotonic()
        self._rec_teach = mode_teach
        self._traj_name = path.name
        self._rec = TrajectoryWriter(path, teach=mode_teach, mode=self._ctrl_mode, n=self.n)
        return None

    def record_stop(self) -> str | None:
        entered = self._rec_entered_float
        self._rec_entered_float = False
        if self._rec is None:
            return None
        err = self._stop_recording()
        if entered and self._ctrl_mode in FLOAT_MODES:
            self._reenter_position()
            self._ctrl_mode = "Position"
        return err

    def delete_traj(self, path: str) -> str | None:
        if self._rec is not None:
            return "请先停止录制再删除"
        if self._replay_frames:
            return "回放中请先停止再删除"
        try:
            gone = delete_recording(path)
        except FileNotFoundError as exc:
            return str(exc)
        except OSError as exc:
            return f"删除失败: {exc}"
        if self._traj_name == gone.name:
            self._traj_name = ""
        return None

    def replay_start(
        self, path: str, rate: float = 1.0, speed_deg_s: float = 40.0
    ) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        if self._rec is not None:
            return "请先停止录制"
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        if self._float_writer_active() or self._ctrl_mode in FLOAT_MODES:
            err = self._reenter_position()
            if err:
                return err
            self._ctrl_mode = "Position"
        if self._ctrl_mode != "Position":
            return "请先切回 Position 再回放"
        try:
            file_path = resolve_traj_path(path)
        except FileNotFoundError as exc:
            return str(exc)
        _header, raw = load_traj(file_path)
        frames = prepare_playback(raw)
        if not frames:
            return "轨迹为空"
        start = frames[0].get("pos") or []
        if len(start) < self.n:
            return "轨迹关节数不足"
        self._abort_replay(hold=False)
        err = self._ensure_enabled()
        if err:
            return err
        self._replay_frames = frames
        self._replay_t0 = None
        self._replay_rate = clip_replay_rate(rate)
        self._replay_settle_left = REPLAY_SETTLE_S
        self._traj_name = file_path.name
        now = self._measured_deg()
        start_deg = [math.degrees(float(x)) for x in start[: self.n]]
        wps = [self._q_rad(p) for p in self._replay_approach_waypoints(now, start_deg)]
        self._start_joint_path(wps, self._replay_approach_speed(speed_deg_s))
        return None

    def _record_tick(self) -> None:
        rec = self._rec
        robot = self._robot
        if rec is None or robot is None:
            return
        drag = self._ctrl_mode in ("Gravity", "Gra+Fri") or self._float_writer_active()
        self._rec_teach = "drag" if drag else "soft"
        if drag or self._servo_cmd_rad is None:
            q = self._read_joint_q_rad(robot)
            v = self._read_joint_v_rad(robot)
        else:
            q = list(self._servo_cmd_rad[: self.n])
            while len(q) < self.n:
                q.append(0.0)
            now = time.monotonic()
            prev_q = self._rec_prev_q
            dt = max(1e-4, now - float(self._rec_prev_mono or now))
            if prev_q and len(prev_q) >= self.n:
                v = [(q[i] - float(prev_q[i])) / dt for i in range(self.n)]
            else:
                v = [0.0] * self.n
            self._rec_prev_q = list(q)
            self._rec_prev_mono = now
        gpos = math.radians(float(self._grip_cmd_deg))
        rec.log(q[: self.n], v[: self.n], gpos, 0.0)

    def _advance_replay(self) -> None:
        frames = self._replay_frames
        if not frames:
            return
        if self._home_path:
            return
        start = [float(x) for x in (frames[0].get("pos") or [])[: self.n]]
        while len(start) < self.n:
            start.append(0.0)
        if self._replay_t0 is None:
            cur = self._servo_cmd_rad
            meas: list[float] | None = None
            robot = self._robot
            if robot is not None:
                try:
                    meas = self._read_joint_q_rad(robot)
                except Exception:
                    meas = None
            # Do not start the timestamp clock from the software pose alone.
            # That is the chatter: file t=0… plays while the metal arm is
            # still on the approach.
            at_start = self._near_q(meas, start, REPLAY_START_TOL_RAD)
            if cur is not None:
                at_start = at_start and self._near_q(cur, start, REPLAY_START_TOL_RAD)
            if not at_start:
                self._stream_q_rad = list(start)
                self._target_deg = [math.degrees(x) for x in start]
                self._servo_intent = True
                self._parked = False
                self._hold_latched = False
                self._note_stream()
                return
            dt = max(0.008, float(self._servo_dt or 0.01))
            if self._replay_settle_left > 0.0:
                self._replay_settle_left = max(0.0, self._replay_settle_left - dt)
                self._stream_q_rad = list(start)
                self._target_deg = [math.degrees(x) for x in start]
                self._servo_intent = True
                self._parked = False
                self._hold_latched = False
                self._note_stream()
                return
            self._replay_t0 = time.monotonic()
        elapsed = (time.monotonic() - self._replay_t0) * self._replay_rate
        last_t = float(frames[-1].get("t") or 0.0)
        sample = sample_at(frames, elapsed)
        q = [float(x) for x in (sample.get("pos") or [])[: self.n]]
        while len(q) < self.n:
            q.append(0.0)
        self._stream_q_rad = q
        self._target_deg = [math.degrees(x) for x in q]
        self._servo_intent = True
        self._parked = False
        self._hold_latched = False
        self._note_stream()
        gpos = sample.get("gpos")
        if gpos is not None:
            deg = math.degrees(float(gpos))
            if abs(deg - float(self._grip_cmd_deg)) > 2.0:
                robot = self._robot
                offline = robot is not None and self._gripper_offline_reason(robot)
                if not offline:
                    self._pending_grip = ("deg", deg)
                self._grip_cmd_deg = deg
        if elapsed >= last_t:
            self._replay_frames = []
            self._replay_t0 = None
            self._parked = True
            self._hold_latched = True

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

    def _absent_motor_ids(self, robot: Any) -> list[int]:
        return _unique_motor_ids(
            [int(m) for m in (getattr(robot, "_missing_motors", None) or [])]
            + list(self._offline_ids)
        )

    def _missing_arm_joint_ids(self, robot: Any) -> list[int]:
        missing = set(self._absent_motor_ids(robot))
        orig = list(self._full_joint_ids) or list(range(1, self.n + 1))
        return [int(mid) for mid in orig[: self.n] if int(mid) in missing]

    def _arm_joints_complete(self, robot: Any) -> bool:
        return not self._missing_arm_joint_ids(robot)

    def _sdk_joint_ids(self, robot: Any) -> list[int]:
        ids = [int(m) for m in list(
            getattr(robot, "_joint_motor_ids", None)
            or getattr(robot, "joint_motor_ids", [])
            or []
        )]
        return ids[: self.n] or list(self._full_joint_ids)

    def _vec_to_sdk(self, values: list[float], robot: Any) -> list[float]:
        ids = self._sdk_joint_ids(robot)
        orig = list(self._full_joint_ids) or list(range(1, self.n + 1))
        if len(ids) == self.n or not values:
            return list(values[: self.n])
        by_id = {int(orig[i]): float(values[i]) for i in range(min(len(orig), len(values)))}
        return [by_id.get(int(mid), 0.0) for mid in ids]

    def _q_to_sdk(self, q_rad: list[float], robot: Any) -> list[float]:
        return self._vec_to_sdk(q_rad, robot)

    def _q_from_sdk(self, q_sdk: list[float], robot: Any, fallback: list[float] | None = None) -> list[float]:
        out = list(fallback if fallback is not None else ([0.0] * self.n))
        while len(out) < self.n:
            out.append(0.0)
        ids = self._sdk_joint_ids(robot)
        orig = list(self._full_joint_ids) or list(range(1, self.n + 1))
        if len(ids) == self.n and len(q_sdk) >= self.n:
            return [float(x) for x in q_sdk[: self.n]]
        idx = {int(mid): i for i, mid in enumerate(orig)}
        for i, mid in enumerate(ids):
            j = idx.get(int(mid))
            if j is not None and i < len(q_sdk):
                out[j] = float(q_sdk[i])
        return out[: self.n]

    def _read_joint_q_rad(self, robot: Any) -> list[float]:
        fallback = list(self._servo_cmd_rad or self._q_rad(self._last.q_deg))
        try:
            raw = list(robot.get_joint_values(prefer_cache=True))
        except Exception:
            return fallback[: self.n]
        return self._q_from_sdk(raw, robot, fallback)

    def _read_joint_v_rad(self, robot: Any) -> list[float]:
        fallback = [0.0] * self.n
        try:
            raw = list(robot.get_joint_velocities(prefer_cache=True))
        except Exception:
            return fallback
        return self._q_from_sdk(raw, robot, fallback)

    def _present_joints_enabled(self, robot: Any) -> bool:
        try:
            raw = robot.get_motor_states(prefer_cache=True)
        except Exception:
            raw = {}
        states = raw if isinstance(raw, dict) else {}
        ids = self._sdk_joint_ids(robot)
        missing = set(self._absent_motor_ids(robot))
        ids = [m for m in ids if m not in missing]
        if ids and states:
            return self._joints_enabled(states, ids, robot)
        return self._flag_enabled(robot)

    def _gripper_offline_reason(self, robot: Any) -> str | None:
        if self.has_gripper and not bool(getattr(robot, "_gripper_online", True)):
            return "夹爪离线，无法控制；在线轴仍可运动"
        if int(self.gripper_id) in set(self._absent_motor_ids(robot)):
            return "夹爪离线，无法控制；在线轴仍可运动"
        return None

    def _adopt_present_motors(self, robot: Any) -> list[int]:
        return drop_absent_motors(robot, gripper_id=self.gripper_id, extra=self._offline_ids)

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
        _force_silent_gripper_offline(robot)
        extra = list(self._offline_ids)
        try:
            robot._station_offline_ids = extra
        except Exception:
            pass
        if self.has_gripper and int(self.gripper_id) in set(extra):
            try:
                robot._gripper_online = False
            except Exception:
                pass
        absent = drop_absent_motors(robot, gripper_id=self.gripper_id, extra=extra)
        live_joints = [m for m in self._sdk_joint_ids(robot) if m not in set(absent)]
        if not live_joints and getattr(robot, "_ht", None) is not None:
            return "没有在线关节电机，无法使能"
        # Delivery kits omit parts (often the gripper). If remaining joints
        # are already in position mode, do not call SDK enable() — that still
        # iterates missing IDs, latches FAULT, and blocks the whole arm.
        if self._present_joints_enabled(robot) or self._flag_enabled(robot):
            return None
        try:
            _enable_present_motors(
                robot,
                gripper_id=self.gripper_id,
                extra=extra,
            )
        except Exception as exc:
            return self._enable_failure(robot, exc)
        return None

    def _enable_failure(self, robot: Any, exc: BaseException) -> str | None:
        if self._present_joints_enabled(robot):
            logger.warning("SDK enable 因缺轴失败，在线轴已在位置环，继续控制: %s", exc)
            return None
        grip_offline = self.has_gripper and (
            not bool(getattr(robot, "_gripper_online", True))
            or int(self.gripper_id) in set(self._absent_motor_ids(robot))
        )
        live = [m for m in self._sdk_joint_ids(robot) if m not in set(self._absent_motor_ids(robot))]
        if _vendor_partial_axis_error(exc) and (live or grip_offline):
            logger.warning("SDK 因离线夹爪/缺轴报错，已忽略；在线轴继续控制: %s", exc)
            return None
        return f"使能失败: {exc}"

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
                robot.servo_j(self._q_to_sdk(list(last), robot))
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
                q_rad = self._read_joint_q_rad(robot)
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

            orig = list(self._full_joint_ids) or list(range(1, self.n + 1))
            live = set(int(m) for m in self._sdk_joint_ids(robot))
            targets: dict[int, float] = {}
            for i, mid in enumerate(orig):
                if i >= len(q_rad):
                    break
                if live and int(mid) not in live:
                    continue
                targets[int(mid)] = float(q_rad[i]) / (2.0 * math.pi)
            if not targets:
                return
            cmds = build(targets, vel_rps=0.0)
            ids = list(cfg.motor_ids) or list(targets)
            ht.set_many_pos_vel_tqe(cmds, pm.PosUnit.Turns, int(max(int(m) for m in ids)), 0.05)
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

    def _run_station_gravity(self, robot: Any, *, friction: bool, impedance: bool) -> None:
        """200 Hz MIT feed-forward using station ``G(q)``. Does not call pinocchio.

        Gravity / Gra+Fri keep K/B/I at zero (same as the pinocchio SDK path).
        Impedance uses the SDK 6-DoF net around the pose at entry.
        """
        send = getattr(robot, "apply_compensation_torque", None)
        if not callable(send):
            raise RuntimeError("控制器没有 apply_compensation_torque，无法下发重力力矩")
        self._prepare_torque_send(robot)
        n = self.n
        period = 1.0 / 200.0
        alpha = 0.4
        max_dtau = 40.0 * period
        vel_abort = 4.0
        v_alpha = 0.3
        i_clamp = 3.0
        rest_thresh = 0.05
        k_soft = list(IMPEDANCE_K[:n]) if impedance else [0.0] * n
        b_soft = list(IMPEDANCE_B[:n]) if impedance else [0.0] * n
        i_soft = list(IMPEDANCE_I[:n]) if impedance else [0.0] * n
        use_net = impedance
        q_des = [0.0] * n
        if use_net:
            try:
                q_des = list(self._read_joint_q_rad(robot)[:n])
            except Exception:
                q_des = list(self._q_rad(self._last.q_deg))
            while len(q_des) < n:
                q_des.append(0.0)
        integ = [0.0] * n
        v_filt = [0.0] * n
        tau_prev: list[float] | None = None
        last_t = time.monotonic()
        try:
            while not self._grav_stop.is_set():
                tick = time.monotonic()
                link_ok = getattr(robot, "_stream_link_ok", None)
                if callable(link_ok):
                    try:
                        if not bool(link_ok()):
                            logger.warning("重力环：链路断开，停止下发")
                            break
                    except Exception:
                        pass
                try:
                    raw_q = list(robot.get_joint_values(prefer_cache=True))
                    raw_v = list(robot.get_joint_velocities(prefer_cache=True))
                except Exception as exc:
                    self._grav_err = str(exc)
                    logger.exception("重力环读关节失败")
                    break
                q = self._q_from_sdk(raw_q, robot, self._q_rad(self._last.q_deg))
                v = self._q_from_sdk(raw_v, robot, [0.0] * n)
                while len(q) < n:
                    q.append(0.0)
                while len(v) < n:
                    v.append(0.0)
                if vel_abort > 0.0 and max(abs(x) for x in v) > vel_abort:
                    logger.warning("重力环：速度跑飞保护 |v|=%.2f rad/s", max(abs(x) for x in v))
                    break
                dt = max(0.0, tick - last_t)
                last_t = tick
                for i in range(n):
                    v_filt[i] = v_alpha * v[i] + (1.0 - v_alpha) * v_filt[i]
                tau = compensation_torque(q, v, friction=friction, tau_limit=FAFU_TAU_LIMIT_NM)
                if use_net:
                    for i in range(n):
                        tau[i] += k_soft[i] * (q_des[i] - q[i]) + b_soft[i] * (-v_filt[i])
                        if dt > 0.0 and i_soft[i] > 0.0:
                            if abs(v[i]) < rest_thresh:
                                integ[i] += (q_des[i] - q[i]) * dt
                            else:
                                integ[i] *= 0.95
                            cap = i_clamp / max(i_soft[i], 1e-9)
                            integ[i] = max(-cap, min(cap, integ[i]))
                            tau[i] += i_soft[i] * integ[i]
                    tau = clip_tau(tau, FAFU_TAU_LIMIT_NM)
                if tau_prev is not None:
                    if alpha < 1.0:
                        tau = [alpha * tau[i] + (1.0 - alpha) * tau_prev[i] for i in range(n)]
                    tau = [
                        max(tau_prev[i] - max_dtau, min(tau_prev[i] + max_dtau, tau[i]))
                        for i in range(n)
                    ]
                tau_prev = list(tau)
                try:
                    send(self._vec_to_sdk(tau, robot))
                except Exception as exc:
                    self._grav_err = str(exc)
                    logger.exception("重力力矩下发失败")
                    break
                sleep_s = period - (time.monotonic() - tick)
                if sleep_s > 0:
                    time.sleep(sleep_s)
        finally:
            # Leave MIT. Position hold, never PWM-off / limp.
            try:
                _enable_present_motors(
                    robot,
                    gripper_id=self.gripper_id,
                    extra=list(self._offline_ids),
                    allow_motor_reset=False,
                )
            except Exception:
                logger.warning("重力环退出后切位置模式失败", exc_info=True)
            try:
                self._send_zero_vel_hold()
            except Exception:
                logger.exception("重力环退出后姿态保持失败")

    def _start_gravity_loop(self, *, friction: bool, impedance: bool) -> str | None:
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        missing = self._missing_arm_joint_ids(robot)
        if missing:
            return f"J1–J6 必须全部在线才能重力补偿（缺 {missing}）；夹爪可离线"
        use_sdk = bool(self._dyn_ready)
        if not use_sdk and not self._station_grav_ok():
            return "真机力矩环未加载"
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

        def run_sdk() -> None:
            try:
                robot.start_gravity_compensation(**kwargs)
            except Exception as exc:
                self._grav_err = str(exc)
                logger.exception("重力补偿环退出")

        def run_station() -> None:
            try:
                self._run_station_gravity(robot, friction=friction, impedance=impedance)
            except Exception as exc:
                self._grav_err = str(exc)
                logger.exception("站控重力补偿环退出")

        target = run_sdk if use_sdk else run_station
        name = "fafu-gravity" if use_sdk else "station-gravity"
        if not use_sdk:
            logger.info("重力环使用站控 G(q)（未加载 pinocchio）friction=%s impedance=%s", friction, impedance)
        self._grav_th = threading.Thread(target=target, name=name, daemon=True)
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

    def _recover_dead_gravity(self) -> None:
        """Gravity thread exited while ctrl_mode still float. Hold; do not servo_j into MIT."""
        err = self._grav_err or "重力/阻抗环已退出"
        logger.warning("重力环线程已结束，切回位置保持: %s", err)
        self._grav_th = None
        if self._ctrl_mode in FLOAT_MODES:
            self._ctrl_mode = "Position"
        self._servo_intent = False
        self._hold_latched = True
        try:
            self._freeze_measured_pose()
        except Exception:
            logger.exception("重力环异常退出后冻结姿态失败")
        self._writer_err = f"重力环结束: {err}"

    def hold(self, *, abort_path: bool = False) -> None:
        if not self.allow_motion or self._robot is None:
            return
        if abort_path:
            self._abort_replay(hold=False)
            self._home_path = []
        elif self._home_path or self._replay_frames:
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
            self._record_tick()
            self._flush_writer()
        step = getattr(robot, "step", None)
        if callable(step):
            step(dt)

    def _drop_gripper_from_servo(self, robot: Any) -> None:
        _force_silent_gripper_offline(robot)
        if self.has_gripper and int(self.gripper_id) in set(self._offline_ids):
            try:
                robot._gripper_online = False
            except Exception:
                pass

    def _start_servo_session(self, robot: Any, opts: Any) -> str | None:
        """Open SDK servo. A bad gripper must not leave _servoing false."""
        self._drop_gripper_from_servo(robot)
        extra = list(self._offline_ids)
        if getattr(robot, "_ht", None) is not None:
            try:
                _prepare_live_servo_session(
                    robot,
                    gripper_id=self.gripper_id,
                    extra=extra,
                )
            except Exception as exc:
                logger.warning("准备伺服会话失败，仍尝试 servo_start: %s", exc)
        try:
            robot.servo_start(opts)
            self._servoing = True
            self._servo_idle.clear()
            self._writer_err = ""
            return None
        except Exception as exc:
            if self._sdk_servoing(robot):
                self._servoing = True
                self._servo_idle.clear()
                self._writer_err = ""
                return None
            logger.warning("servo_start 首次失败，去掉夹爪后重试: %s", exc)
            try:
                robot._gripper_online = False
            except Exception:
                pass
            self._drop_gripper_from_servo(robot)
            if getattr(robot, "_ht", None) is not None:
                try:
                    _prepare_live_servo_session(
                        robot,
                        gripper_id=self.gripper_id,
                        extra=list(self._offline_ids) + (
                            [int(self.gripper_id)] if self.has_gripper else []
                        ),
                    )
                except Exception:
                    pass
            try:
                robot.servo_start(opts)
                self._servoing = True
                self._servo_idle.clear()
                self._writer_err = ""
                return None
            except Exception as exc2:
                if self._sdk_servoing(robot):
                    self._servoing = True
                    self._servo_idle.clear()
                    self._writer_err = ""
                    return None
                logger.exception("servo_start 失败")
                return f"servo_start 失败: {exc2}"

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
        if self._grav_th is not None and not self._grav_th.is_alive():
            self._grav_th = None
            if self._ctrl_mode in FLOAT_MODES:
                self._recover_dead_gravity()
                return
        following_path = bool(self._home_path)
        replaying = bool(self._replay_frames)
        if following_path:
            self._advance_home()
        elif replaying:
            self._advance_replay()
        elif (
            self._servo_intent
            and self._stream_q_rad is not None
            and not self._parked
            and (time.monotonic() - self._stream_rx_mono) > STREAM_IDLE_S
        ):
            self._hold_stream_pose()
        if self._servoing and not self._sdk_servoing(robot):
            self._servoing = False
        if self._servo_intent and self._stream_q_rad is not None:
            self._freeze_needed = False
            err = self._ensure_enabled()
            if err and (
                self._present_joints_enabled(robot) or _vendor_partial_axis_text(err)
            ):
                logger.warning("忽略使能报错，在线轴继续: %s", err)
                self._writer_err = ""
                err = None
            if err:
                logger.warning("servo 使能失败: %s", err)
                self._writer_err = err
                name = _robot_state_name(robot)
                if name in ("dead", "disconnected"):
                    self._servo_intent = False
                    self._home_path = []
                    self._abort_replay(hold=False)
                    self._freeze_needed = True
            else:
                if not self._servoing:
                    opts = self._make_servo_opts()
                    err_start = self._start_servo_session(robot, opts)
                    if err_start:
                        logger.warning("servo_start 失败: %s", err_start)
                        self._writer_err = err_start
                        if not self._home_path and not replaying:
                            self._servo_intent = False
                            self._freeze_needed = True
                if self._servoing and self._servo_intent:
                    try:
                        q_send = list(self._stream_q_rad)
                        # Path following already speed-limits with synchronized
                        # joints. Independent per-axis slew toward a far vertex
                        # is what made 复位 stop-start twitch.
                        if not self._is_fake() and not following_path:
                            if not replaying:
                                q_send = self._slew_live_cmd(q_send, self._servo_dt)
                            elif self._replay_t0 is None or (
                                self._servo_cmd_rad is not None
                                and self._max_abs_delta(self._servo_cmd_rad, q_send) > math.radians(3.5)
                            ):
                                q_send = self._slew_live_cmd(q_send, self._servo_dt)
                        sent = robot.servo_j(self._q_to_sdk(q_send, robot))
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
                            if self._home_path or replaying:
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
            offline = self._gripper_offline_reason(robot)
            if offline:
                if not (self._replay_frames or self._replay_t0 is not None):
                    self._writer_err = offline
                return
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
                self._grip_cmd_deg = float(self.gripper_limits[1 if value else 0])
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
            self._grip_cmd_deg = deg
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
        if self._float_writer_active() or self._ctrl_mode in ("Gravity", "Gra+Fri"):
            return "重力/阻抗环运行中请先切回 Position"
        if self._replay_frames:
            self._abort_replay(hold=False)
            self._home_path = []
        if stream and self._home_path:
            return None
        robot = self._robot
        q_deg = self._clip_deg(q_deg)
        if stream and self._parked:
            meas = [math.degrees(x) for x in (self._servo_cmd_rad or [])]
            if not meas and robot is not None:
                try:
                    meas = [math.degrees(x) for x in self._read_joint_q_rad(robot)]
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
        if self._home_path or self._replay_frames:
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
        robot = self._robot
        if robot is not None:
            blocked_g = self._gripper_offline_reason(robot)
            if blocked_g:
                return blocked_g
        self._hold_latched = False
        self._pending_grip = ("open", bool(open_))
        self._grip_cmd_deg = float(self.gripper_limits[1 if open_ else 0])
        return None

    def set_gripper_deg(self, deg: float, effort: int | None = None) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            return blocked
        if self._float_writer_active():
            return "重力/阻抗环运行中请先切回 Position 再动夹爪"
        if effort is not None:
            self._gripper_effort = clip_gripper_effort(effort)
        robot = self._robot
        if robot is not None:
            blocked_g = self._gripper_offline_reason(robot)
            if blocked_g:
                return blocked_g
        lo, hi = self.gripper_limits
        value = max(lo, min(hi, float(deg)))
        self._hold_latched = False
        self._pending_grip = ("deg", value)
        self._grip_cmd_deg = value
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
                cur = self._read_joint_q_rad(robot) if robot else None
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
        if self._replay_frames and self._replay_t0 is None:
            self._parked = False
            self._hold_latched = False
            self._servo_intent = True
            self._note_stream()
            return
        self._parked = True
        self._hold_latched = True
        self._target_deg = [math.degrees(x) for x in cur]

    def home(self, speed_deg_s: float) -> str | None:
        blocked = self.refuse_motion()
        if blocked:
            logger.warning("忽略复位: %s", blocked)
            return blocked
        if self._float_writer_active() or self._ctrl_mode in ("Gravity", "Gra+Fri"):
            return "重力/阻抗环运行中请先切回 Position 再复位"
        robot = self._robot
        if robot is None:
            return "机械臂未连接"
        self._abort_replay(hold=False)
        self._hold_latched = False
        self._parked = False
        err = self._ensure_enabled()
        if err:
            return err
        spd = min(float(speed_deg_s), HOME_MAX_DEG_S)
        self.set_teleop_speed(spd)
        try:
            q_now = [math.degrees(x) for x in self._read_joint_q_rad(robot)]
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
        self._abort_replay(hold=False)
        prev = self._ctrl_mode
        if name == prev:
            return None
        if name in FLOAT_MODES:
            if self._use_sdk_float():
                missing = self._missing_arm_joint_ids(robot)
                if missing:
                    return f"J1–J6 必须全部在线才能重力补偿（缺 {missing}）；夹爪可离线"
            if not self._float_modes_available():
                return "真机力矩环未加载"
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
                    if not self._gripper_offline_reason(robot):
                        robot.open_gripper(**self._grip_kwargs())
                        self._gripper_deg = float(self.gripper_limits[1])
                        self._grip_open = True
                        self._grip_cmd_deg = self._gripper_deg
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
        self._abort_replay(hold=False)
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
                seed = self._read_joint_q_rad(robot)
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
        self._abort_replay(hold=False)
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
                seed = self._read_joint_q_rad(robot)
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
        if self._float_writer_active() or self._ctrl_mode in ("Gravity", "Gra+Fri"):
            return "重力/阻抗环运行中请先切回 Position 再跑路点"
        self._abort_replay(hold=False)
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
        self._stop_recording()
        self._rec_entered_float = False
        self._abort_replay(hold=False)
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
        self._abort_replay(hold=False)
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
            err = self._ensure_enabled()
            if err and self._present_joints_enabled(robot):
                logger.warning("使能报错已忽略（在线轴已在位置环）: %s", err)
                err = None
            if err and _vendor_partial_axis_text(err):
                try:
                    _enable_present_motors(
                        robot,
                        gripper_id=self.gripper_id,
                        extra=list(self._offline_ids),
                    )
                except Exception:
                    logger.exception("在线轴补使能失败")
                if self._present_joints_enabled(robot):
                    logger.warning("SDK 全员使能失败已忽略，在线轴已切入位置环")
                    err = None
            if err:
                return err
            self._writer_err = ""
            # Cold enable used to leave no servo session. 发送位置 / follow
            # then hit SDK servo_start → vendor enable() which still required
            # the gripper. Start a parked hold so remaining joints are already
            # in the 100 Hz writer.
            if not self._is_fake():
                self._hold_stream_pose()
            return None
        self._servo_intent = False
        self._home_path = []
        self._abort_replay(hold=False)
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

    def _q_rad_from_states(self, states: dict, ids: list[int]) -> list[float]:
        last_q = list(self._last.q_deg)
        fake = self._is_fake()
        out: list[float] = []
        for i in range(self.n):
            mid = int(ids[i]) if i < len(ids) else i + 1
            fallback = math.radians(float(last_q[i] if i < len(last_q) else 0.0))
            st = states.get(mid)
            pos = getattr(st, "position", None) if st is not None else None
            if pos is None:
                out.append(fallback)
                continue
            if fake:
                out.append(float(pos))
            else:
                out.append(math.radians(motor_position_to_deg(float(pos))))
        return out

    def _joints_enabled(self, states: dict, ids: list[int], robot: Any) -> bool:
        seen = 0
        for mid in ids:
            st = states.get(int(mid))
            if st is None:
                continue
            seen += 1
            if int(getattr(st, "mode", 0) or 0) != 0x0A:
                return False
        if seen:
            return True
        return self._flag_enabled(robot)

    def _motor_rows(self, states: dict, ids: list[int], ok: list[bool], *, link_up: bool = True) -> list[dict]:
        rows: list[dict] = []
        mids = list(ids) if ids else list(range(1, self.n + 1))
        for i in range(self.n):
            mid = int(mids[i]) if i < len(mids) else i + 1
            st = states.get(mid) if link_up else None
            online = bool(link_up) and st is not None and bool(getattr(st, "online", True))
            fault = int(getattr(st, "fault", 0) or 0) if st is not None else (0 if not link_up else 1)
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
            gst = states.get(self.gripper_id) if link_up else None
            rows.append(
                {
                    "id": int(self.gripper_id),
                    "name": "夹爪",
                    "online": bool(link_up) and gst is not None and bool(getattr(gst, "online", True)),
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
                "link_err": self._link_error if robot is None else "",
            }
            extra.update(self._traj_snap())
        drop_live = False
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
            ids = list(self._full_joint_ids)[: self.n] or list(getattr(robot, "joint_motor_ids", []))[: self.n]
            states: dict = {}
            try:
                raw_states = robot.get_motor_states(prefer_cache=True)
                states = raw_states if isinstance(raw_states, dict) else {}
            except Exception:
                states = {}
            try:
                raw_q = list(robot.get_joint_values(prefer_cache=True))
                self._q_fb_err = ""
                q_rad = self._q_from_sdk(raw_q, robot, self._q_rad(self._last.q_deg))
            except Exception as exc:
                msg = str(exc)
                if self._q_fb_err != msg:
                    self._q_fb_err = msg
                    logger.warning("部分关节无反馈，整臂仍视为已连接: %s", exc)
                q_rad = self._q_rad_from_states(states, self._full_joint_ids or ids)
            # Live USB reads can lag the motor command by a cache tick. The
            # twin should track the last servo_j we sent while a session is
            # open; otherwise the 3D arm trails the metal arm.
            if (
                not self._is_fake()
                and self._servo_cmd_rad is not None
                and (self._servoing or self._home_path or self._replay_frames)
            ):
                q_rad = list(self._servo_cmd_rad[: self.n])
            q_deg = [math.degrees(float(x)) for x in q_rad[: self.n]]
            while len(q_deg) < self.n:
                q_deg.append(0.0)
            ok = [True] * self.n
            moving = False
            tau_raw = [0] * self.n
            try:
                vels = list(robot.get_joint_velocities(prefer_cache=True))
                moving = any(abs(float(v)) > 0.02 for v in vels[: self.n])
            except Exception:
                moving = False
            try:
                if not ids:
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
            link_up = bool(self._is_fake() or _hardware_link_up(robot))
            motors = self._motor_rows(states, ids, ok, link_up=link_up)
            online_ids = [int(m["id"]) for m in motors if m.get("online")]
            if online_ids:
                self._offline_ids = [int(m["id"]) for m in motors if not m.get("online")]
                try:
                    robot._station_offline_ids = list(self._offline_ids)
                except Exception:
                    pass
            if (
                self.has_gripper
                and not self._is_fake()
                and int(self.gripper_id) in set(self._offline_ids)
            ):
                try:
                    robot._gripper_online = False
                except Exception:
                    pass
            if not link_up:
                extra["link_err"] = extra.get("link_err") or "USB/总线已断开"
                ok = [False] * self.n
                enabled = False
                moving = False
            else:
                extra["link_err"] = ""
                if self._q_fb_err:
                    extra["link_err"] = self._q_fb_err
                # Prefer cached joint modes. SDK is_enabled() may block-read every
                # motor (including gripper) and stall the 100 Hz servo watchdog.
                # Missing axes must not force enabled=false for the ones that are up.
                if ids and states:
                    enabled = self._joints_enabled(states, ids, robot)
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
            drop_live = not self._is_fake() and not link_up
            snap = ArmSnap(
                online=link_up,
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
        except Exception as exc:
            logger.warning("读取臂状态失败: %s", exc)
            link_up = bool(self._is_fake() or _hardware_link_up(robot))
            drop_live = not self._is_fake() and not link_up
            extra["link_err"] = extra.get("link_err") or (
                str(exc) if link_up else "USB/总线已断开"
            )
            motors = (
                list(self._last.motors)
                if link_up
                else self._motor_rows({}, [], [False] * self.n, link_up=False)
            )
            snap = ArmSnap(
                online=link_up,
                backend=self.backend,
                enabled=False,
                n=self.n,
                q_deg=list(self._last.q_deg),
                target_deg=list(self._last.target_deg),
                ok=list(self._last.ok) if self._last.ok else [False] * self.n,
                gripper_open=self._last.gripper_open,
                has_gripper=self.has_gripper,
                moving=False,
                ee_m=self._last.ee_m,
                ee_rpy_deg=self._last.ee_rpy_deg,
                ctrl_mode=self._ctrl_mode,
                gripper_deg=self._last.gripper_deg,
                tau_raw=list(self._last.tau_raw),
                motors=motors,
                **extra,
            )
            self._last = snap
        if drop_live:
            self._drop_lost_live_link()
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
