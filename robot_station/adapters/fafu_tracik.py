"""Optional Trac-IK for non-realtime cartesian ``cart_go`` only.

Do not call this from the 100 Hz WASD / servo writer. A native solve that
takes tens of milliseconds (or hangs) skips ``servo_j`` and the arm looks
unresponsive. Import the ``trac_ik`` wrapper first so Windows finds ``nlopt.dll``.
"""
from __future__ import annotations

import logging
import math
import os
import sys
import threading
from pathlib import Path
from typing import Any

from robot_station.adapters.fafu_kin import (
    DEFAULT_LIMITS,
    as_rotation,
    clip_rad,
    forward_pose,
    xyz_err,
)

logger = logging.getLogger("station.arm.tracik")

_HERE = Path(__file__).resolve().parent
_BUNDLED_URDF = _HERE / "fafu_baseV1_kin.urdf"
_LOCK = threading.Lock()
_SOLVER: Any = None
_FAILED = False
_POS_TOL_M = 0.008
_ORI_DOT_MIN = 0.995

# cart_go waits for ACK; 20 ms is fine. WASD only hits this after Jacobian fail.
_TIMEOUT_S = 0.02


def _prepare_import() -> None:
    """Prefer pip ``trac_ik``; else the cloned pytracik tree beside the repo."""
    extra: list[Path] = []
    env = (os.environ.get("PYTRACIK_ROOT") or "").strip()
    if env:
        extra.append(Path(env))
    extra.extend(
        (
            Path(r"D:\pytracik\pytracik-main"),
            _HERE.parents[1].parent / "pytracik" / "pytracik-main",
            Path.home() / "pytracik" / "pytracik-main",
        )
    )
    for root in extra:
        init = root / "trac_ik" / "__init__.py"
        if not init.is_file():
            continue
        inserted = str(root)
        if inserted not in sys.path:
            sys.path.append(inserted)
        break


def _urdf_path() -> Path | None:
    candidates: list[Path] = [_BUNDLED_URDF]
    env = (os.environ.get("FAFU_ARM_SDK") or "").strip()
    if env:
        root = Path(env).expanduser()
        candidates.append(root / "fafu_robot_python" / "fafu_robot_description" / "urdf" / "fafu_baseV1.urdf")
    repo = _HERE.parents[1]
    home = Path.home()
    for sdk in (
        repo / "vendor" / "fafu_arm_sdk",
        repo.parent / "fafu_arm_sdk",
        repo.parent / "fafu_arm_sdk-main" / "fafu_arm_sdk-main",
        repo.parent / "fafu_arm_sdk-main",
        home / "fafu_arm_sdk",
        Path(r"D:\fafu_arm_sdk"),
        Path(r"D:\fafu_arm_sdk-main\fafu_arm_sdk-main"),
    ):
        candidates.append(sdk / "fafu_robot_python" / "fafu_robot_description" / "urdf" / "fafu_baseV1.urdf")
    for path in candidates:
        if path.is_file():
            return path
    return None


def _station_limits_rad() -> tuple[list[float], list[float]]:
    lo = [math.radians(p[0]) for p in DEFAULT_LIMITS]
    hi = [math.radians(p[1]) for p in DEFAULT_LIMITS]
    return lo, hi


def _load_solver() -> Any | None:
    global _SOLVER, _FAILED
    if _FAILED:
        return None
    if _SOLVER is not None:
        return _SOLVER
    with _LOCK:
        if _FAILED:
            return None
        if _SOLVER is not None:
            return _SOLVER
        urdf = _urdf_path()
        if urdf is None:
            _FAILED = True
            logger.info("Trac-IK unused: no FAFU URDF")
            return None
        _prepare_import()
        try:
            from trac_ik import TracIK  # type: ignore
        except Exception as exc:
            _FAILED = True
            logger.info("Trac-IK unused: %s", exc)
            return None
        try:
            solver = TracIK(
                "base_link",
                "tool_link",
                str(urdf),
                timeout=_TIMEOUT_S,
                epsilon=1e-5,
                solver_type="Speed",
            )
            if int(solver.dof) != 6:
                raise RuntimeError(f"Trac-IK dof={solver.dof}, expected 6")
            lo, hi = _station_limits_rad()
            try:
                solver.joint_limits = (lo, hi)
            except Exception:
                logger.warning("Trac-IK joint_limits 未套用站控限位，事后仍 clip")
            _SOLVER = solver
            logger.info("Trac-IK ready (%s)", urdf.name)
            return _SOLVER
        except Exception as exc:
            _FAILED = True
            logger.warning("Trac-IK load failed: %s", exc)
            return None


def available() -> bool:
    return _load_solver() is not None


def _as_np(rot: Any):
    import numpy as np

    mat = as_rotation(rot, is_euler=False, is_radians=True)
    out = np.eye(4, dtype=float)
    for i in range(3):
        for j in range(3):
            out[i, j] = float(mat[i][j])
    return out


def _accept(q: list[float], xyz: list[float], rot: Any) -> list[float] | None:
    clipped = clip_rad(q)
    pos, r_got, _rpy = forward_pose(clipped)
    if xyz_err(pos, xyz) > _POS_TOL_M:
        return None
    r_goal = as_rotation(rot, is_euler=False, is_radians=True)
    # trace(R_goal^T R_got) = 1 + 2 cos θ
    inner = 0.0
    for i in range(3):
        for j in range(3):
            inner += r_goal[i][j] * r_got[i][j]
    c = max(-1.0, min(1.0, (inner - 1.0) * 0.5))
    if c < _ORI_DOT_MIN:
        return None
    return clipped


def inverse_tracik(
    xyz: list[float],
    rot: Any,
    seed_rad: list[float] | None,
) -> list[float] | None:
    """Full-pose IK. None if pytracik is missing or the solver fails."""
    solver = _load_solver()
    if solver is None:
        return None
    try:
        import numpy as np
    except Exception:
        return None
    seed = clip_rad(list(seed_rad or [0.0, math.radians(40.0), math.radians(40.0), 0.0, 0.0, 0.0]))
    pos = np.array([float(xyz[0]), float(xyz[1]), float(xyz[2])], dtype=float)
    rmat = _as_np(rot)
    seed_np = np.array(seed, dtype=float)
    try:
        with _LOCK:
            raw = solver.ik(pos, rmat[:3, :3], seed_np)
    except Exception as exc:
        logger.debug("Trac-IK ik failed: %s", exc)
        return None
    if raw is None:
        return None
    q = [float(x) for x in list(raw)[:6]]
    if len(q) < 6:
        return None
    return _accept(q, [float(x) for x in xyz[:3]], rot)
