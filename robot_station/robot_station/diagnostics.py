"""Read-only connection details. Enumeration never opens or probes a serial port."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from robot_station.adapters.fafu_arm import import_fafu_sdk
from robot_station.config import StationConfig
from robot_station.serial_guard import live_serial_allowed


def connection_diagnostics(cfg: StationConfig, snapshot: dict, *,
                           in_process: bool = False, motion_connected: bool = False) -> dict:
    arm = snapshot.get("arm") or {}
    serial: dict[str, Any] = {"ports": [], "error": "", "sdk_loaded": False}
    try:
        motor, _controller = import_fafu_sdk(cfg.arm_sdk or None)
        serial["sdk_loaded"] = True
        candidates = {str(getattr(p, "port", p)) for p in motor.find_likely_debug_boards()}
        ports = motor.list_serial_ports()
        serial["ports"] = sorted([
            {"port": str(p.port), "description": str(getattr(p, "description", "")),
             "hardware_id": str(getattr(p, "hardware_id", "")),
             "vid": str(getattr(p, "vid", "")), "pid": str(getattr(p, "pid", "")),
             "candidate": str(p.port) in candidates}
            for p in ports
        ], key=lambda p: (not p["candidate"], p["port"]))
    except Exception as exc:
        serial["error"] = f"串口枚举不可用：{exc}"
    return {
        "ok": True, "checked_at": datetime.now(timezone.utc).isoformat(),
        "http": {"host": cfg.listen_host, "port": cfg.http_port},
        "motion": {"mode": "in_process" if in_process else "tcp",
                   "host": cfg.motion_host, "port": cfg.motion_port,
                   "connected": bool(in_process or motion_connected)},
        "permissions": {"serial":live_serial_allowed(), "motion":bool(cfg.arm_allow_motion)},
        "serial": {**serial, "configured_port": (snapshot.get("connection") or {}).get("port") or cfg.arm_port or "",
                   "active_port": str(arm.get("serial_port") or "") if arm.get("online") else ""},
    }
