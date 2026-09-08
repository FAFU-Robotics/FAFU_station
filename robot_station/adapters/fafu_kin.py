"""FAFU arm geometry: same kinematic chain as the official SDK URDF.

Joint origins and axes copy ``fafu_robot_python/fafu_robot_description/fafu_follower.urdf``
from https://github.com/FAFU-Robotics/fafu_arm_sdk (all joint RPY are zero).
``forward`` / ``inverse`` are this chain — not a planar 2-link sketch — so sim
cartesian and the End Effector readout match live pinocchio FK at the same q.

Live hardware uses SDK ``setup_dynamics`` (pinocchio) for cartesian when it is
installed. If pinocchio is missing (typical Windows), WASD / 笛卡尔 use this
module on the 100 Hz writer. Optional ``pytracik`` is only a ``cart_go``
fallback, never inside the servo tick. Gravity / impedance use ``fafu_dyn``
(``G(q)`` from URDF masses/CoMs), not Trac-IK and not Pinocchio.
"""
from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any, Sequence

# Soft limits in degrees, aligned with fafu_robot_python/robot.cfg joints 1–6.
DEFAULT_LIMITS = (
    (-140.688, 148.140),
    (0.0, 180.0),
    (-0.036, 205.0),
    (-124.956, 90.972),
    (-87.228, 72.972),
    (-95.004, 90.540),
)

DEFAULT_GRIPPER_LIMITS = (8.0, 75.0)
# robot.cfg gripper_max_torque_raw. M4438_30: 1 raw ≈ 0.0053 Nm; 300 ≈ 1.6 Nm.
DEFAULT_GRIPPER_EFFORT = 300
GRIPPER_EFFORT_MIN = 50
GRIPPER_EFFORT_MAX = 800
_EFFORT_RE = re.compile(r"^gripper_max_torque_raw\s*=\s*(-?\d+)", re.IGNORECASE)

# Conservative slider box around the follower workspace (home tool ≈ 0.25, 0, 0.17 m).
CART_XYZ_LIMITS = ((-0.30, 0.75), (-0.55, 0.55), (-0.20, 0.70))
CART_RPY_LIMITS_DEG = ((-180.0, 180.0), (-180.0, 180.0), (-180.0, 180.0))


def clip_cart_xyz(xyz: Sequence[float]) -> list[float]:
    out: list[float] = []
    for i in range(3):
        raw = float(xyz[i]) if i < len(xyz) else 0.0
        lo, hi = CART_XYZ_LIMITS[i]
        out.append(max(lo, min(hi, raw)))
    return out


def clip_cart_rpy_deg(rpy_deg: Sequence[float]) -> list[float]:
    out: list[float] = []
    for i in range(3):
        raw = float(rpy_deg[i]) if i < len(rpy_deg) else 0.0
        lo, hi = CART_RPY_LIMITS_DEG[i]
        out.append(max(lo, min(hi, raw)))
    return out

# (origin xyz metres, axis) for revolute joints 1–6. Keep in sync with arm3d.js.
URDF_JOINTS: tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.0584), (0.0, 0.0, 1.0)),
    ((0.018199, 0.0, 0.053), (0.0, 1.0, 0.0)),
    ((-0.26, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ((0.23, 0.0, 0.06), (0.0, -1.0, 0.0)),
    ((0.07, 0.0, 0.036319), (0.0, 0.0, -1.0)),
    ((0.02345, 0.0, -0.039), (1.0, 0.0, 0.0)),
)
URDF_TOOL_XYZ = (0.165, 0.0, 0.0)

_LIMIT_RE = re.compile(r"^limits\.(\d+)\s*=\s*([^#;]+)", re.IGNORECASE)

Mat3 = list[list[float]]
Vec3 = list[float]


def _clip_deg(i: int, deg: float, limits: tuple[tuple[float, float], ...] = DEFAULT_LIMITS) -> float:
    lo, hi = limits[i] if i < len(limits) else (-180.0, 180.0)
    return max(lo, min(hi, deg))


def clip_rad(
    q_rad: list[float],
    limits: tuple[tuple[float, float], ...] | list[tuple[float, float]] = DEFAULT_LIMITS,
) -> list[float]:
    out: list[float] = []
    for i, raw in enumerate(q_rad[:6]):
        out.append(math.radians(_clip_deg(i, math.degrees(float(raw)), tuple(limits))))
    while len(out) < 6:
        out.append(0.0)
    return out


def _eye() -> Mat3:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _mmul(a: Mat3, b: Mat3) -> Mat3:
    return [
        [
            a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            for j in range(3)
        ]
        for i in range(3)
    ]


def _mv(r: Mat3, v: Sequence[float]) -> Vec3:
    return [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]


def _add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2])]


def _rot_axis(axis: Sequence[float], ang: float) -> Mat3:
    x, y, z = float(axis[0]), float(axis[1]), float(axis[2])
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c = math.cos(ang)
    s = math.sin(ang)
    C = 1.0 - c
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def rpy_to_matrix(rpy: Sequence[float]) -> Mat3:
    """R = Rz(yaw) Ry(pitch) Rx(roll), same convention as pinocchio rpyToMatrix."""
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx: Mat3 = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    ry: Mat3 = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rz: Mat3 = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    return _mmul(rz, _mmul(ry, rx))


def matrix_to_rpy(r: Mat3) -> Vec3:
    """Match robot_station.adapters.fafu_arm.rotation_to_rpy / pinocchio matrixToRpy."""
    r02 = r[2][0]
    sy = math.hypot(r[2][1], r[2][2])
    if sy > 1e-6:
        roll = math.atan2(r[2][1], r[2][2])
        pitch = math.atan2(-r02, sy)
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = math.atan2(-r[1][2], r[1][1])
        pitch = math.atan2(-r02, sy)
        yaw = 0.0
    return [roll, pitch, yaw]


def as_rotation(ori: Any, *, is_euler: bool = False, is_radians: bool = True) -> Mat3:
    """Accept None, RPY triple, or 3×3 matrix (list / tuple / ndarray)."""
    if ori is None:
        return _eye()
    if is_euler:
        seq = [float(x) for x in list(ori)[:3]]
        if not is_radians:
            seq = [math.radians(x) for x in seq]
        return rpy_to_matrix(seq)
    try:
        rows = list(ori)
    except TypeError:
        return _eye()
    if len(rows) == 3:
        first = rows[0]
        nested = False
        if isinstance(first, (list, tuple)):
            nested = True
        elif hasattr(first, "shape") and getattr(first, "ndim", 0) > 0:
            nested = True
        if nested:
            mat: Mat3 = []
            for row in rows:
                cells = [float(x) for x in list(row)[:3]]
                if len(cells) != 3:
                    return _eye()
                mat.append(cells)
            return mat
        seq = [float(x) for x in rows]
        if not is_radians:
            seq = [math.radians(x) for x in seq]
        return rpy_to_matrix(seq)
    return _eye()


def forward_pose(q_rad: Sequence[float]) -> tuple[Vec3, Mat3, Vec3]:
    """Return (xyz metres, 3×3 rotation, rpy radians) of tool_link in base_link."""
    p, r = _fk_raw(q_rad)[:2]
    return p, r, matrix_to_rpy(r)


def _fk_raw(q_rad: Sequence[float]) -> tuple[Vec3, Mat3, list[Vec3], list[Vec3]]:
    q = [float(q_rad[i]) if i < len(q_rad) else 0.0 for i in range(6)]
    r = _eye()
    p: Vec3 = [0.0, 0.0, 0.0]
    origins: list[Vec3] = []
    axes: list[Vec3] = []
    for i, (xyz, axis) in enumerate(URDF_JOINTS):
        p = _add(p, _mv(r, xyz))
        origins.append(p)
        axes.append(_mv(r, axis))
        r = _mmul(r, _rot_axis(axis, q[i]))
    p = _add(p, _mv(r, URDF_TOOL_XYZ))
    return p, r, origins, axes


def geometric_jacobian(q_rad: Sequence[float]) -> tuple[list[list[float]], Vec3, Mat3]:
    """World-frame geometric Jacobian of tool_link (6×6)."""
    p_ee, r_ee, origins, axes = _fk_raw(q_rad)
    jac = [[0.0] * 6 for _ in range(6)]
    for i in range(6):
        vx, vy, vz = _cross(axes[i], [p_ee[0] - origins[i][0], p_ee[1] - origins[i][1], p_ee[2] - origins[i][2]])
        jac[0][i], jac[1][i], jac[2][i] = vx, vy, vz
        jac[3][i], jac[4][i], jac[5][i] = axes[i]
    return jac, p_ee, r_ee


def forward(q_rad: list[float]) -> tuple[list[float], list[float]]:
    """Return (xyz metres, rpy radians) from 6 joint radians (URDF tool_link)."""
    xyz, _rot, rpy = forward_pose(q_rad)
    return xyz, rpy


def apply_delta_pose(
    xyz: Sequence[float],
    rot: Any,
    dxyz: Sequence[float],
    drpy_rad: Sequence[float],
) -> tuple[Vec3, Mat3]:
    """Translate in base frame; right-multiply Rx, Ry, Rz (SDK keyboard convention)."""
    r = as_rotation(rot, is_euler=False, is_radians=True)
    out = [
        float(xyz[0]) + float(dxyz[0] if len(dxyz) > 0 else 0.0),
        float(xyz[1]) + float(dxyz[1] if len(dxyz) > 1 else 0.0),
        float(xyz[2]) + float(dxyz[2] if len(dxyz) > 2 else 0.0),
    ]
    for i, axis in enumerate(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))):
        ang = float(drpy_rad[i]) if i < len(drpy_rad) else 0.0
        if abs(ang) > 1e-12:
            r = _mmul(r, _rot_axis(axis, ang))
    return out, r


def xyz_err(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def step_cartesian(
    q_rad: Sequence[float],
    dxyz: Sequence[float],
    drpy_rad: Sequence[float],
    *,
    damping: float = 0.05,
    max_dq: float = 0.12,
) -> list[float] | None:
    """One damped Jacobian step for keyboard teleop. Fast enough for the 100 Hz writer."""
    q = clip_rad(list(q_rad))
    jac, p, r = geometric_jacobian(q)
    xyz_g, r_g = apply_delta_pose(p, r, dxyz, drpy_rad)
    err = _task_error(p, r, xyz_g, r_g, 1.0)
    dq = _dls_step(jac, err, damping)
    if dq is None:
        return None
    nrm = math.sqrt(sum(v * v for v in dq))
    if nrm > max_dq and nrm > 1e-12:
        dq = [v * (max_dq / nrm) for v in dq]
    nxt = clip_rad([q[i] + dq[i] for i in range(6)])
    p2, _r2, _ = forward_pose(nxt)
    if xyz_err(p2, xyz_g) > xyz_err(p, xyz_g) + 1e-4:
        return None
    return nxt


def _cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _rot_err_world(r_cur: Mat3, r_goal: Mat3) -> Vec3:
    rt = [[r_cur[j][i] for j in range(3)] for i in range(3)]
    re = _mmul(r_goal, rt)
    return [
        0.5 * (re[2][1] - re[1][2]),
        0.5 * (re[0][2] - re[2][0]),
        0.5 * (re[1][0] - re[0][1]),
    ]


def _task_error(p: Sequence[float], r: Mat3, p_goal: Sequence[float], r_goal: Mat3, ori_weight: float) -> list[float]:
    err = [float(p_goal[i]) - float(p[i]) for i in range(3)]
    w = _rot_err_world(r, r_goal)
    return err + [ori_weight * w[0], ori_weight * w[1], ori_weight * w[2]]


def _solve6(a: list[list[float]], b: Sequence[float]) -> list[float] | None:
    n = 6
    m = [row[:] + [float(b[i])] for i, row in enumerate(a)]
    for i in range(n):
        piv = i
        best = abs(m[i][i])
        for r in range(i + 1, n):
            val = abs(m[r][i])
            if val > best:
                best = val
                piv = r
        if best < 1e-14:
            return None
        m[i], m[piv] = m[piv], m[i]
        div = m[i][i]
        for c in range(i, n + 1):
            m[i][c] /= div
        for r in range(n):
            if r == i:
                continue
            fac = m[r][i]
            if fac == 0.0:
                continue
            for c in range(i, n + 1):
                m[r][c] -= fac * m[i][c]
    return [m[i][n] for i in range(n)]


def _dls_step(j: list[list[float]], err: Sequence[float], damping: float) -> list[float] | None:
    # dq = J^T (J J^T + λ² I)^{-1} err  (world-frame J; caller adds dq)
    jjt = [[0.0] * 6 for _ in range(6)]
    for r in range(6):
        for c in range(6):
            s = 0.0
            for k in range(6):
                s += j[r][k] * j[c][k]
            jjt[r][c] = s
            if r == c:
                jjt[r][c] += damping * damping
    alpha = _solve6(jjt, err)
    if alpha is None:
        return None
    dq = [0.0] * 6
    for i in range(6):
        s = 0.0
        for r in range(6):
            s += j[r][i] * alpha[r]
        dq[i] = s
    return dq


def inverse(
    xyz: list[float],
    rpy_rad: list[float] | Sequence[Sequence[float]] | None = None,
    seed_rad: list[float] | None = None,
    *,
    rotation: Any = None,
    is_euler: bool = True,
    is_radians: bool = True,
    max_iter: int = 80,
    eps: float = 1e-4,
    damping: float = 1e-2,
    ori_weight: float = 1.0,
) -> list[float] | None:
    """Damped least-squares IK on the URDF chain. None if it does not converge.

    ``ori_weight=0`` solves position only (keyboard translations near a
    singularity). Geometric Jacobian is world-frame, matching ``apply_delta_pose``.
    """
    x, y, z = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    if rotation is not None:
        r_goal = as_rotation(rotation, is_euler=is_euler, is_radians=is_radians)
    else:
        r_goal = as_rotation(rpy_rad, is_euler=True, is_radians=True)
    q = clip_rad(list(seed_rad or [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0]))
    p_goal = [x, y, z]
    ow = max(0.0, float(ori_weight))
    best_q = list(q)
    best_pos = 1e9
    for _ in range(max_iter):
        jac, p, r = geometric_jacobian(q)
        err = _task_error(p, r, p_goal, r_goal, ow)
        pos_n = math.sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2])
        nrm = math.sqrt(sum(v * v for v in err))
        if pos_n < best_pos:
            best_pos = pos_n
            best_q = list(q)
        if nrm < eps:
            return clip_rad(q)
        if ow < 1e-9:
            for row in range(3, 6):
                for col in range(6):
                    jac[row][col] = 0.0
        else:
            for row in range(3, 6):
                for col in range(6):
                    jac[row][col] *= ow
        lam = damping * (1.0 + 1.0 / (nrm + 0.1))
        dq = _dls_step(jac, err, lam)
        if dq is None:
            break
        step = math.sqrt(sum(v * v for v in dq))
        if step > 0.35:
            dq = [v * (0.35 / step) for v in dq]
        q = clip_rad([q[i] + dq[i] for i in range(6)])
    if best_pos <= 0.001:
        return clip_rad(best_q)
    return None


def inverse_with_fallback(
    xyz: list[float],
    rpy_rad: list[float] | Sequence[Sequence[float]] | None = None,
    seed_rad: list[float] | None = None,
    **kwargs: Any,
) -> list[float] | None:
    """6D IK, then relax orientation so small translations still work at home.

    URDF zero (SDK ``go_home``) is near a singularity; full-pose DLS often
    returns None for a 5 mm +X. Keyboard translations must still succeed.

    Do not call Trac-IK here. WASD / mock teleop run on the 100 Hz writer;
    a native 20 ms solve (or a hang) skips ``servo_j`` and the arm looks dead.
    Trac-IK is only for non-realtime ``cart_go`` in ``fafu_arm``.
    """
    kwargs.pop("ori_weight", None)
    kwargs.pop("use_tracik", None)
    kwargs.setdefault("max_iter", 24)
    for weight in (1.0, 0.0):
        solved = inverse(xyz, rpy_rad, seed_rad, ori_weight=weight, **kwargs)
        if solved is not None:
            return solved
    return None


def load_cfg_limits(
    cfg_path: str | Path | None,
    n: int = 6,
) -> tuple[list[tuple[float, float]], tuple[float, float]]:
    """Parse ``limits.N`` from robot.cfg. Values are degrees (pos_unit=degrees)."""
    joints = [DEFAULT_LIMITS[i] if i < len(DEFAULT_LIMITS) else (-180.0, 180.0) for i in range(n)]
    gripper = DEFAULT_GRIPPER_LIMITS
    path = Path(cfg_path) if cfg_path else None
    if path is None or not path.is_file():
        return joints, gripper
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return joints, gripper
    found: dict[int, tuple[float, float]] = {}
    for line in text.splitlines():
        raw = line.strip()
        if not raw or raw.startswith("#") or raw.startswith(";"):
            continue
        match = _LIMIT_RE.match(raw)
        if not match:
            continue
        motor_id = int(match.group(1))
        parts = [p.strip() for p in match.group(2).replace(";", ",").split(",") if p.strip()]
        if len(parts) < 2:
            continue
        try:
            lo, hi = float(parts[0]), float(parts[1])
        except ValueError:
            continue
        if lo > hi:
            lo, hi = hi, lo
        found[motor_id] = (lo, hi)
    for i in range(n):
        if (i + 1) in found:
            joints[i] = found[i + 1]
    if 7 in found:
        gripper = found[7]
    elif n + 1 in found:
        gripper = found[n + 1]
    return joints, gripper


def clip_gripper_effort(raw: Any, default: int = DEFAULT_GRIPPER_EFFORT) -> int:
    try:
        value = int(raw)
    except (TypeError, ValueError):
        value = int(default)
    return max(GRIPPER_EFFORT_MIN, min(GRIPPER_EFFORT_MAX, value))


def load_gripper_effort(
    cfg_path: str | Path | None,
    default: int = DEFAULT_GRIPPER_EFFORT,
) -> int:
    """Parse ``gripper_max_torque_raw`` from robot.cfg (SDK interactive default)."""
    path = Path(cfg_path) if cfg_path else None
    if path is None or not path.is_file():
        return clip_gripper_effort(default)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return clip_gripper_effort(default)
    found = default
    for line in text.splitlines():
        raw = line.strip()
        if not raw or raw.startswith("#") or raw.startswith(";"):
            continue
        match = _EFFORT_RE.match(raw)
        if not match:
            continue
        try:
            found = int(match.group(1))
        except ValueError:
            continue
    return clip_gripper_effort(found, default)
