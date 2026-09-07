from __future__ import annotations

from dataclasses import dataclass, field
import time


@dataclass
class ArmSnap:
    online: bool = True
    backend: str = "mock"
    enabled: bool = True
    n: int = 6
    q_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    target_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    ok: list[bool] = field(default_factory=lambda: [True] * 6)
    gripper_open: bool = True
    has_gripper: bool = True
    moving: bool = False
    ee_m: list[float] | None = None
    ee_rpy_deg: list[float] | None = None
    ctrl_mode: str = "Position"
    limits: list[list[float]] | None = None
    gripper_limits: list[float] | None = None
    gripper_deg: float = 75.0
    gripper_effort: int = 300
    tau_raw: list[int] = field(default_factory=lambda: [0] * 6)
    dyn_ready: bool = False
    grav_active: bool = False
    float_ok: bool = False
    float_reason: str = ""
    ik_err: str = ""
    motors: list[dict] = field(default_factory=list)


@dataclass
class CamSnap:
    online: list[bool] = field(default_factory=lambda: [True])
    fps: list[float] = field(default_factory=lambda: [0.0])
    roles: list[str] = field(default_factory=lambda: ["作业相机"])
    backend: str = "mock"
    device: str = ""
    reason: str = ""


@dataclass
class World:
    seq: int = 0
    t_mono: float = 0.0
    safety: str = "IDLE"
    safety_reason: str = ""
    clients: int = 0
    cmd_hz: float = 0.0
    last_rtt_ms: float | None = None
    echo_t0: float | None = None
    arm: ArmSnap = field(default_factory=ArmSnap)
    camera: CamSnap = field(default_factory=CamSnap)

    def as_dict(self) -> dict:
        a, cam = self.arm, self.camera
        return {
            "seq": self.seq,
            "t_mono": self.t_mono,
            "now": time.time(),
            "safety": self.safety,
            "safety_reason": self.safety_reason,
            "clients": self.clients,
            "cmd_hz": round(self.cmd_hz, 1),
            "rtt_ms": None if self.last_rtt_ms is None else round(self.last_rtt_ms, 2),
            "echo_t0": self.echo_t0,
            "arm": {
                "online": a.online,
                "backend": a.backend,
                "enabled": a.enabled,
                "n": a.n,
                "q_deg": [round(x, 2) for x in a.q_deg],
                "target_deg": [round(x, 2) for x in a.target_deg],
                "ok": list(a.ok),
                "gripper_open": a.gripper_open,
                "has_gripper": a.has_gripper,
                "moving": a.moving,
                "ee_m": None if a.ee_m is None else [round(x, 4) for x in a.ee_m],
                "ee_rpy_deg": None
                if a.ee_rpy_deg is None
                else [round(x, 2) for x in a.ee_rpy_deg],
                "ctrl_mode": a.ctrl_mode,
                "limits": a.limits,
                "gripper_limits": a.gripper_limits,
                "gripper_deg": None if a.gripper_deg is None else round(float(a.gripper_deg), 1),
                "gripper_effort": int(a.gripper_effort),
                "tau_raw": list(a.tau_raw),
                "dyn_ready": bool(a.dyn_ready),
                "grav_active": bool(a.grav_active),
                "float_ok": bool(a.float_ok),
                "float_reason": a.float_reason or "",
                "ik_err": a.ik_err or "",
                "motors": list(a.motors),
            },
            "camera": {
                "online": list(cam.online),
                "fps": [round(x, 1) for x in cam.fps],
                "roles": list(cam.roles),
                "backend": cam.backend,
                "device": cam.device,
                "reason": cam.reason,
            },
        }
