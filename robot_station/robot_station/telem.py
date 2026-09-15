"""Compact binary joint telemetry for the 100 Hz pose loop.

Fat JSON snapshots stay on a ~10 Hz HUD channel. The pose loop is an
88-byte little-endian frame so USB-side control is not queued behind
json.dumps / JSON.parse / 3D / fake video.

The same frame is used on the motion TCP (prefixed by MAGIC, no newline)
and on the browser `/ws` socket.
"""
from __future__ import annotations

import json
import math
import struct
from typing import Any

MAGIC = 0x314E5453
MAGIC_LE = struct.pack("<I", MAGIC)
TELEM = struct.Struct("<IIHBxd6f6ff3ff")
TELEM_SIZE = TELEM.size

FLAG_ONLINE = 1
FLAG_ENABLED = 2
FLAG_MOVING = 4
FLAG_GRIP = 8
FLAG_IK = 16

_SAFETY = {"IDLE": 0, "OPERATING": 1, "ESTOP_LATCHED": 2, "WATCHDOG": 3}
_SAFETY_NAME = {code: name for name, code in _SAFETY.items()}


def _six(raw: Any) -> list[float]:
    vals = [float(x) for x in (raw or [])[:6]]
    while len(vals) < 6:
        vals.append(0.0)
    return vals


def _three(raw: Any) -> list[float]:
    vals = [float(x) for x in (raw or [])[:3]]
    while len(vals) < 3:
        vals.append(0.0)
    return vals


def pack_telem(data: dict[str, Any]) -> bytes:
    arm = data.get("arm") or {}
    q = _six(arm.get("q_deg"))
    tgt = _six(arm.get("target_deg") or q)
    ee = _three(arm.get("ee_m"))
    echo = data.get("echo_t0")
    echo_f = float(echo) if echo is not None else math.nan
    flags = 0
    if arm.get("online"):
        flags |= FLAG_ONLINE
    if arm.get("enabled"):
        flags |= FLAG_ENABLED
    if arm.get("moving"):
        flags |= FLAG_MOVING
    if arm.get("gripper_open"):
        flags |= FLAG_GRIP
    if arm.get("ik_err"):
        flags |= FLAG_IK
    safety = _SAFETY.get(str(data.get("safety") or "IDLE"), 255)
    raw_g = arm.get("gripper_deg")
    grip = float(raw_g) if raw_g is not None else 0.0
    cmd_hz = float(data.get("cmd_hz") or 0.0)
    seq = int(data.get("seq") or 0) & 0xFFFFFFFF
    return TELEM.pack(MAGIC, seq, flags, safety, echo_f, *q, *tgt, grip, *ee, cmd_hz)


def unpack_telem(blob: bytes) -> dict[str, Any] | None:
    if not blob or len(blob) != TELEM_SIZE:
        return None
    magic, seq, flags, safety, echo_f, *rest = TELEM.unpack(blob)
    if magic != MAGIC:
        return None
    q = rest[0:6]
    tgt = rest[6:12]
    grip = rest[12]
    ee = rest[13:16]
    cmd_hz = rest[16]
    echo = None if isinstance(echo_f, float) and math.isnan(echo_f) else float(echo_f)
    return {
        "seq": int(seq),
        "echo_t0": echo,
        "cmd_hz": float(cmd_hz),
        "safety": _SAFETY_NAME.get(int(safety), "IDLE"),
        "arm": {
            "online": bool(flags & FLAG_ONLINE),
            "enabled": bool(flags & FLAG_ENABLED),
            "moving": bool(flags & FLAG_MOVING),
            "gripper_open": bool(flags & FLAG_GRIP),
            "ik_err": "IK" if flags & FLAG_IK else "",
            "q_deg": [float(x) for x in q],
            "target_deg": [float(x) for x in tgt],
            "gripper_deg": float(grip),
            "ee_m": [float(x) for x in ee],
        },
    }


def overlay_telem(dst: dict[str, Any], pose: dict[str, Any]) -> dict[str, Any]:
    """Fold a compact pose into a full snapshot, keeping HUD-only fields."""
    cur = dict(dst or {})
    arm = dict(cur.get("arm") or {})
    arm.update(pose.get("arm") or {})
    cur.update(pose)
    cur["arm"] = arm
    return cur


def take_motion_frames(buf: bytes) -> tuple[bytes, list[tuple[str, dict[str, Any]]]]:
    """Split a motion-TCP buffer into JSON lines and 88-byte telem frames.

    JSON objects start with ``{`` and end with a newline. Binary pose frames
    start with MAGIC (``STN1``). Incomplete tails stay in the returned remainder.
    """
    events: list[tuple[str, dict[str, Any]]] = []
    while buf:
        if buf[0] == 0x7B:
            nl = buf.find(b"\n")
            if nl < 0:
                break
            raw, buf = buf[:nl], buf[nl + 1 :]
            if not raw.strip():
                continue
            try:
                msg = json.loads(raw.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if isinstance(msg, dict):
                events.append(("json", msg))
            continue
        if len(buf) < 4:
            break
        if buf[:4] != MAGIC_LE:
            pos_j = buf.find(b"{")
            pos_m = buf.find(MAGIC_LE)
            cands = [p for p in (pos_j, pos_m) if p >= 0]
            if not cands:
                return b"", events
            buf = buf[min(cands) :]
            continue
        if len(buf) < TELEM_SIZE:
            break
        pose = unpack_telem(buf[:TELEM_SIZE])
        buf = buf[TELEM_SIZE:]
        if pose is not None:
            events.append(("telem", pose))
    return buf, events
