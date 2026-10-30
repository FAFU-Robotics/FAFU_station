"""Explicit endpoints and command fencing for one robot connection at a time."""
from __future__ import annotations

import re
from typing import Any


def connection_kind(backend: str) -> str:
    return "serial" if backend in ("fafu", "auto") else "simulation"


def target_backend(kind: str) -> str:
    aliases = {"serial": "fafu", "fafu": "fafu", "simulation": "sim", "sim": "sim"}
    try:
        return aliases[str(kind).strip().lower()]
    except KeyError:
        raise ValueError("请选择串口机械臂或本机仿真") from None


def serial_endpoint(port: Any) -> str:
    if not isinstance(port, str):
        raise ValueError("请选择要连接的串口")
    port = port.strip()
    if not port or port.lower() in ("auto", "cfg/auto"):
        raise ValueError("请选择具体串口；不会自动连接其它设备")
    if len(port) > 256 or any(ord(char) < 32 for char in port):
        raise ValueError("串口名称无效")
    if re.fullmatch(r"COM[1-9][0-9]*", port, re.IGNORECASE):
        return port.upper()
    if port.startswith("/dev/") or re.fullmatch(r"\\\\\.\\COM[1-9][0-9]*", port, re.IGNORECASE):
        return port
    raise ValueError("串口应为 COM3 等端口，或 /dev/ttyUSB0 等设备路径")


def needs_session(msg: dict) -> bool:
    kind = msg.get("t")
    if kind == "workbench":
        return msg.get("action") in {
            "run", "step", "resume", "pause", "stop", "traj_record", "traj_record_stop",
            "traj_execute", "traj_endpoint", "traj_endpoint_touch", "traj_endpoint_stop",
        }
    return kind in {"link", "arm_src", "arm", "home", "grip", "mode", "grav_urdf", "teach",
                    "cart", "cart_go", "path", "rec_start", "rec_stop", "replay", "script", "power", "clear", "touch", "park"}


def rejected(msg: dict, error: str) -> dict:
    reply = {"t":"ack", "ok":False, "op":str(msg.get("t") or ""), "error":error}
    if msg.get("t") == "workbench":
        reply.update(request_id=msg.get("request_id"), action=msg.get("action"))
    return reply
