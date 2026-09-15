"""Gravity / friction feed-forward for the FAFU 6-DoF chain, without Pinocchio.

``G(q)`` is virtual work on link masses and CoMs from the official
``fafu_baseV1.urdf`` (same joint origins/axes as ``fafu_kin``). Inertia
tensors are not used: the SDK float loop is ``tau = clip(G(q)+friction)``,
not full RNEA.

Keep this off the 100 Hz ``servo_j`` writer. Teach float (Gravity / Gra+Fri)
is ~100 Hz, matching SDK ``test_fafu_motion_interactive.teach_record``.
"""
from __future__ import annotations

import math
from typing import Sequence

from robot_station.adapters.fafu_kin import (
    URDF_JOINTS,
    _add,
    _cross,
    _eye,
    _mmul,
    _mv,
    _rot_axis,
)

# Child-link inertial of joint1..joint6 in each link frame (fafu_baseV1.urdf).
# tool_link has no inertial; extra payload belongs on link6.
LINK_MASS: tuple[float, ...] = (
    0.28101551,
    1.5165092,
    0.66917784,
    0.30780516,
    0.23711139,
    0.40,
)
LINK_COM: tuple[tuple[float, float, float], ...] = (
    (0.00278873, -0.00094387, 0.02491842),
    (-0.12936212, -0.00213339, 7.183e-05),
    (0.16343103, -0.00164646, 0.05337427),
    (0.05240131, -0.00055249, 0.04094177),
    (0.00303007, 8.509e-05, -0.03624079),
    (0.06704625, 0.00051731, 1.674e-05),
)

GRAVITY_VEC: tuple[float, float, float] = (0.0, 0.0, -9.81)
TAU_LIMIT_NM: tuple[float, ...] = (15.0, 30.0, 30.0, 15.0, 5.0, 5.0)

# SDK test_fafu_motion_interactive teach Fc / Fv (official 5_record_trajectory).
FRICTION_FC: tuple[float, ...] = (0.15, 0.12, 0.12, 0.12, 0.04, 0.04)
FRICTION_FV: tuple[float, ...] = (0.05, 0.05, 0.05, 0.03, 0.02, 0.02)
FRICTION_VEL_EPS = 0.02
TEACH_RATE_HZ = 100.0
# Brief software Kd at Gravity / Gra+Fri entry. Official apply_compensation_torque
# damping_kd has no firmware effect; subtract kd*v here, then fade to 0.
GRAV_START_DAMP_KD = 1.5
GRAV_START_DAMP_S = 0.50
GRAV_START_DAMP_HOLD_S = 0.12
GRAV_START_DAMP_SETTLE = 0.08

# SDK start_gravity_compensation 6-DoF impedance net (Nm/rad, Nm·s/rad, Nm/(rad·s)).
IMPEDANCE_K: tuple[float, ...] = (1.5, 8.0, 10.0, 10.0, 1.5, 1.5)
IMPEDANCE_B: tuple[float, ...] = (0.4, 0.4, 0.6, 0.8, 0.2, 0.2)
IMPEDANCE_I: tuple[float, ...] = (0.0, 2.5, 3.5, 6.0, 0.0, 0.0)
I_CLAMP_NM = 3.0
# SDK docs say 0.15 rev/s; get_joint_velocities() is rad/s. 0.15 rad/s (~9°/s)
# treats gravity sag as a hand-drag, zeros K/I, and the arm walks down.
MOVE_VEL_THRESH_REV_S = 0.15
MOVE_VEL_THRESH = MOVE_VEL_THRESH_REV_S * math.tau
DRAG_ENTER_S = 0.08
DRAG_SETTLE_S = 0.25
REST_VEL_THRESH = 0.05


def _pad6(q: Sequence[float]) -> list[float]:
    out = [float(q[i]) if i < len(q) else 0.0 for i in range(6)]
    return out


def gravity_torque(
    q_rad: Sequence[float],
    *,
    gravity: Sequence[float] = GRAVITY_VEC,
    masses: Sequence[float] | None = None,
    coms: Sequence[Sequence[float]] | None = None,
    joints: Sequence[tuple[Sequence[float], Sequence[float]]] | None = None,
) -> list[float]:
    """Generalized gravity ``G(q)`` in Nm, one value per revolute joint.

    Joint 1 is vertical, so ``G[0]`` is ~0 for this URDF (g along -Z).
    Optional ``masses`` / ``coms`` / ``joints`` override the bundled
    ``fafu_baseV1.urdf`` table (used when the page picks another file).
    """
    q = _pad6(q_rad)
    mass = _pad6(masses) if masses is not None else list(LINK_MASS)
    com_tbl = list(coms) if coms is not None else list(LINK_COM)
    chain = list(joints) if joints is not None else list(URDF_JOINTS)
    gx, gy, gz = float(gravity[0]), float(gravity[1]), float(gravity[2])
    r = _eye()
    p = [0.0, 0.0, 0.0]
    origins: list[list[float]] = []
    axes: list[list[float]] = []
    world_coms: list[list[float]] = []
    for i, (xyz, axis) in enumerate(chain[:6]):
        p = _add(p, _mv(r, [float(xyz[0]), float(xyz[1]), float(xyz[2])]))
        origins.append(p)
        axes.append(_mv(r, [float(axis[0]), float(axis[1]), float(axis[2])]))
        r = _mmul(r, _rot_axis(axis, q[i]))
        ci = com_tbl[i] if i < len(com_tbl) else (0.0, 0.0, 0.0)
        world_coms.append(_add(p, _mv(r, [float(ci[0]), float(ci[1]), float(ci[2])])))
    tau = [0.0] * 6
    for j in range(6):
        mj = mass[j]
        fx, fy, fz = mj * gx, mj * gy, mj * gz
        cx, cy, cz = world_coms[j]
        for i in range(j + 1):
            ox, oy, oz = origins[i]
            rx, ry, rz = _cross(axes[i], [cx - ox, cy - oy, cz - oz])
            tau[i] += fx * rx + fy * ry + fz * rz
    return tau


def friction_torque(
    vel_rad_s: Sequence[float],
    *,
    fc: Sequence[float] = FRICTION_FC,
    fv: Sequence[float] = FRICTION_FV,
    vel_threshold: float = FRICTION_VEL_EPS,
) -> list[float]:
    """Coulomb + viscous, with a low-speed dead-band on the sign term."""
    v = _pad6(vel_rad_s)
    eps = max(0.0, float(vel_threshold))
    out = [0.0] * 6
    for i in range(6):
        vi = v[i]
        visc = float(fv[i] if i < len(fv) else 0.0) * vi
        if abs(vi) < eps:
            out[i] = visc
        else:
            coul = float(fc[i] if i < len(fc) else 0.0) * (1.0 if vi > 0.0 else -1.0)
            out[i] = coul + visc
    return out


def startup_damping_kd(elapsed_s: float, max_abs_vel: float = 0.0) -> float:
    """Scalar Kd (Nm·s/rad). Full for a short hold, then fade; 0 after ``GRAV_START_DAMP_S``.

    If motion has already settled after the hold, drop to 0 so the loop
    returns to pure ``G(q)`` without waiting out the whole window.
    """
    t = max(0.0, float(elapsed_s))
    if t >= GRAV_START_DAMP_S:
        return 0.0
    if t >= GRAV_START_DAMP_HOLD_S and abs(float(max_abs_vel)) < GRAV_START_DAMP_SETTLE:
        return 0.0
    return GRAV_START_DAMP_KD * (1.0 - t / GRAV_START_DAMP_S)


def apply_vel_damping(
    tau: Sequence[float],
    vel_rad_s: Sequence[float],
    kd: float,
) -> list[float]:
    """``tau - kd * v``. ``kd <= 0`` returns ``tau`` unchanged."""
    gain = float(kd)
    n = max(len(tau), len(vel_rad_s))
    out = [float(tau[i]) if i < len(tau) else 0.0 for i in range(n)]
    if gain <= 1e-9:
        return out
    for i in range(n):
        vi = float(vel_rad_s[i]) if i < len(vel_rad_s) else 0.0
        out[i] -= gain * vi
    return out


def clip_tau(tau: Sequence[float], limit: Sequence[float] = TAU_LIMIT_NM) -> list[float]:
    out: list[float] = []
    for i in range(6):
        raw = float(tau[i]) if i < len(tau) else 0.0
        lim = abs(float(limit[i])) if i < len(limit) else 5.0
        out.append(max(-lim, min(lim, raw)))
    return out


def lead_through_step(
    q: Sequence[float],
    vel: Sequence[float],
    dt: float,
    q_des: list[float],
    integ: list[float],
    dragging: list[bool],
    fast_time: list[float],
    slow_time: list[float],
    *,
    hold_on_release: bool = True,
    move_vel_thresh: float = MOVE_VEL_THRESH,
    enter_time: float = DRAG_ENTER_S,
    settle_time: float = DRAG_SETTLE_S,
    rest_thresh: float = REST_VEL_THRESH,
) -> list[bool]:
    """SDK ``hold_on_release`` debounce: drag follows ``q``, then lock.

    Returns per-joint hold mask (``True`` = integrate / spring toward ``q_des``).
    """
    n = min(len(q_des), 6)
    hold = [True] * n
    step = max(0.0, float(dt))
    if hold_on_release:
        for i in range(n):
            vi = abs(float(vel[i])) if i < len(vel) else 0.0
            if vi > move_vel_thresh:
                fast_time[i] += step
                slow_time[i] = 0.0
            else:
                slow_time[i] += step
                fast_time[i] = 0.0
            if fast_time[i] >= enter_time:
                dragging[i] = True
            if slow_time[i] >= settle_time:
                dragging[i] = False
            if dragging[i]:
                q_des[i] = float(q[i]) if i < len(q) else q_des[i]
                integ[i] = 0.0
            hold[i] = not dragging[i]
        return hold
    for i in range(n):
        vi = abs(float(vel[i])) if i < len(vel) else 0.0
        hold[i] = vi < rest_thresh
    return hold


def compensation_torque(
    q_rad: Sequence[float],
    vel_rad_s: Sequence[float] | None = None,
    *,
    friction: bool = False,
    tau_limit: Sequence[float] = TAU_LIMIT_NM,
    masses: Sequence[float] | None = None,
    coms: Sequence[Sequence[float]] | None = None,
    joints: Sequence[tuple[Sequence[float], Sequence[float]]] | None = None,
) -> list[float]:
    """``clip(G(q) + optional friction(v), ±tau_limit)`` in Nm."""
    tau = gravity_torque(q_rad, masses=masses, coms=coms, joints=joints)
    if friction:
        tau = [tau[i] + x for i, x in enumerate(friction_torque(vel_rad_s or [0.0] * 6))]
    return clip_tau(tau, tau_limit)
