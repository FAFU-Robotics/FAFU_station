"""Gravity / friction feed-forward for the FAFU 6-DoF chain, without Pinocchio.

``G(q)`` is virtual work on link masses and CoMs from the official
``fafu_baseV1.urdf`` (same joint origins/axes as ``fafu_kin``). Inertia
tensors are not used: the SDK float loop is ``tau = clip(G(q)+friction)``,
not full RNEA.

Keep this off the 100 Hz ``servo_j`` writer. The gravity thread (≈200 Hz)
owns the serial MIT frames.
"""
from __future__ import annotations

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

# SDK FrictionParams.reference_6dof (Nm, Nm·s/rad).
FRICTION_FC: tuple[float, ...] = (0.20, 0.15, 0.15, 0.15, 0.04, 0.04)
FRICTION_FV: tuple[float, ...] = (0.06, 0.06, 0.06, 0.03, 0.02, 0.02)
FRICTION_VEL_EPS = 0.02

# SDK start_gravity_compensation 6-DoF impedance net (Nm/rad, Nm·s/rad, Nm/(rad·s)).
IMPEDANCE_K: tuple[float, ...] = (1.5, 8.0, 10.0, 10.0, 1.5, 1.5)
IMPEDANCE_B: tuple[float, ...] = (0.4, 0.4, 0.6, 0.8, 0.2, 0.2)
IMPEDANCE_I: tuple[float, ...] = (0.0, 2.5, 3.5, 6.0, 0.0, 0.0)


def _pad6(q: Sequence[float]) -> list[float]:
    out = [float(q[i]) if i < len(q) else 0.0 for i in range(6)]
    return out


def gravity_torque(
    q_rad: Sequence[float],
    *,
    gravity: Sequence[float] = GRAVITY_VEC,
) -> list[float]:
    """Generalized gravity ``G(q)`` in Nm, one value per revolute joint.

    Joint 1 is vertical, so ``G[0]`` is ~0 for this URDF (g along -Z).
    """
    q = _pad6(q_rad)
    gx, gy, gz = float(gravity[0]), float(gravity[1]), float(gravity[2])
    r = _eye()
    p = [0.0, 0.0, 0.0]
    origins: list[list[float]] = []
    axes: list[list[float]] = []
    coms: list[list[float]] = []
    for i, (xyz, axis) in enumerate(URDF_JOINTS):
        p = _add(p, _mv(r, xyz))
        origins.append(p)
        axes.append(_mv(r, axis))
        r = _mmul(r, _rot_axis(axis, q[i]))
        coms.append(_add(p, _mv(r, LINK_COM[i])))
    tau = [0.0] * 6
    for j in range(6):
        mj = LINK_MASS[j]
        fx, fy, fz = mj * gx, mj * gy, mj * gz
        cx, cy, cz = coms[j]
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


def clip_tau(tau: Sequence[float], limit: Sequence[float] = TAU_LIMIT_NM) -> list[float]:
    out: list[float] = []
    for i in range(6):
        raw = float(tau[i]) if i < len(tau) else 0.0
        lim = abs(float(limit[i])) if i < len(limit) else 5.0
        out.append(max(-lim, min(lim, raw)))
    return out


def compensation_torque(
    q_rad: Sequence[float],
    vel_rad_s: Sequence[float] | None = None,
    *,
    friction: bool = False,
    tau_limit: Sequence[float] = TAU_LIMIT_NM,
) -> list[float]:
    """``clip(G(q) + optional friction(v), ±tau_limit)`` in Nm."""
    tau = gravity_torque(q_rad)
    if friction:
        tau = [tau[i] + x for i, x in enumerate(friction_torque(vel_rad_s or [0.0] * 6))]
    return clip_tau(tau, tau_limit)
