"""Web process client for the motion TCP (127.0.0.1).

Commands are JSON lines. Pose telemetry is 88-byte binary frames; a fat JSON
snapshot still arrives about 10 Hz for HUD fields. Stream arm/teach commands
are coalesced latest-wins. Keyboard cart deltas are summed so a slow USB tick
cannot drop the motion that already happened.
"""

from __future__ import annotations

import json
import logging
import socket
import threading
import time
from typing import Any

from robot_station.telem import overlay_telem, take_motion_frames

logger = logging.getLogger("station.bridge")

_STREAM_KINDS = {"cart", "teach"}
_CART_FLUSH_MAX_M = 0.025
_CART_FLUSH_MAX_DEG = 10.0


def coalesce_stream(prev: dict[str, Any] | None, msg: dict[str, Any]) -> dict[str, Any]:
    """Latest-wins for arm/teach; sum keyboard cart increments."""
    out = dict(msg)
    if str(msg.get("t") or "") != "cart" or not prev:
        return out
    dxyz = []
    prev_xyz = list(prev.get("dxyz") or [0.0, 0.0, 0.0])
    cur_xyz = list(msg.get("dxyz") or [0.0, 0.0, 0.0])
    for i in range(3):
        v = float(prev_xyz[i] if i < len(prev_xyz) else 0.0) + float(cur_xyz[i] if i < len(cur_xyz) else 0.0)
        dxyz.append(max(-_CART_FLUSH_MAX_M, min(_CART_FLUSH_MAX_M, v)))
    drpy = []
    prev_rpy = list(prev.get("drpy") or [0.0, 0.0, 0.0])
    cur_rpy = list(msg.get("drpy") or [0.0, 0.0, 0.0])
    for i in range(3):
        v = float(prev_rpy[i] if i < len(prev_rpy) else 0.0) + float(cur_rpy[i] if i < len(cur_rpy) else 0.0)
        drpy.append(max(-_CART_FLUSH_MAX_DEG, min(_CART_FLUSH_MAX_DEG, v)))
    out["dxyz"] = dxyz
    out["drpy"] = drpy
    return out


class MotionClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = int(port)
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._snap: dict[str, Any] = {}
        self._ready: dict[str, Any] = {}
        self._stop = threading.Event()
        self._rx: threading.Thread | None = None
        self._tx: threading.Thread | None = None
        self._pending: dict[int, tuple[threading.Event, dict | None]] = {}
        self._req = 0
        self._snap_cv = threading.Condition()
        self._snap_seq = 0
        self._stream_lock = threading.Lock()
        self._stream_latest: dict[str, dict[str, Any]] = {}
        self._stream_ev = threading.Event()
        self.connected = False

    def connect(self, timeout_s: float = 8.0) -> None:
        deadline = time.monotonic() + timeout_s
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            try:
                sock.connect((self.host, self.port))
                sock.settimeout(0.5)
                try:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                self._sock = sock
                self.connected = True
                self._stop.clear()
                self._rx = threading.Thread(target=self._read_loop, name="motion-rx", daemon=True)
                self._rx.start()
                self._tx = threading.Thread(target=self._stream_loop, name="motion-tx", daemon=True)
                self._tx.start()
                logger.info("已连运动进程 %s:%s", self.host, self.port)
                return
            except OSError as exc:
                last_err = exc
                try:
                    sock.close()
                except OSError:
                    pass
                time.sleep(0.15)
        raise ConnectionError(f"连不上运动进程 {self.host}:{self.port}: {last_err}")

    def close(self) -> None:
        self._stop.set()
        self._stream_ev.set()
        with self._snap_cv:
            self._snap_cv.notify_all()
        self.connected = False
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        if self._rx is not None:
            self._rx.join(timeout=1.0)
        if self._tx is not None:
            self._tx.join(timeout=1.0)

    def latest(self) -> dict[str, Any]:
        with self._snap_cv:
            return dict(self._snap)

    def wait_snap(self, last_seq: int, timeout_s: float = 0.25) -> tuple[int, dict[str, Any]]:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._snap_cv:
            while self._snap_seq == last_seq and not self._stop.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._snap_cv.wait(remaining)
            seq = self._snap_seq
            data = dict(self._snap)
        return seq, data

    def backend(self) -> str:
        with self._snap_cv:
            snap = dict(self._snap)
        with self._lock:
            ready = dict(self._ready)
        return str((snap.get("arm") or {}).get("backend") or ready.get("arm") or "?")

    def send(self, msg: dict[str, Any]) -> None:
        kind = str(msg.get("t") or "")
        if kind in _STREAM_KINDS or (kind == "arm" and msg.get("stream")):
            with self._stream_lock:
                prev = self._stream_latest.get(kind)
                self._stream_latest[kind] = coalesce_stream(prev, msg)
                self._stream_ev.set()
            return
        self._send_now(msg)

    def _send_now(self, msg: dict[str, Any]) -> None:
        data = (json.dumps(msg, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
        with self._send_lock:
            with self._lock:
                sock = self._sock
            if sock is None:
                return
            try:
                sock.sendall(data)
            except OSError:
                self.connected = False

    def _stream_loop(self) -> None:
        while not self._stop.is_set():
            self._stream_ev.wait(0.008)
            if self._stop.is_set():
                return
            with self._stream_lock:
                batch = self._stream_latest
                self._stream_latest = {}
                self._stream_ev.clear()
            for payload in batch.values():
                self._send_now(payload)

    def call(self, msg: dict[str, Any], timeout_s: float = 1.0) -> dict[str, Any]:
        with self._lock:
            self._req += 1
            req = self._req
        ev = threading.Event()
        with self._lock:
            self._pending[req] = (ev, None)
        payload = dict(msg)
        payload["req"] = req
        self._send_now(payload)
        if not ev.wait(timeout_s):
            with self._lock:
                self._pending.pop(req, None)
            return {"t": "ack", "ok": False, "error": "运动进程无应答", "op": msg.get("t")}
        with self._lock:
            pair = self._pending.pop(req, None)
        if not pair or pair[1] is None:
            return {"t": "ack", "ok": False, "error": "运动进程无应答", "op": msg.get("t")}
        return pair[1]

    def _apply_telem(self, pose: dict[str, Any]) -> None:
        with self._snap_cv:
            self._snap = overlay_telem(self._snap, pose)
            self._snap_seq = int(pose.get("seq") or self._snap_seq + 1)
            self._snap_cv.notify_all()

    def _apply_json(self, msg: dict[str, Any]) -> None:
        kind = msg.get("t")
        if kind == "snap":
            data = msg.get("d")
            if isinstance(data, dict):
                with self._snap_cv:
                    self._snap = data
                    seq = int(data.get("seq") or self._snap_seq)
                    if seq != self._snap_seq:
                        self._snap_seq = seq
                        self._snap_cv.notify_all()
        elif kind == "ready":
            with self._lock:
                self._ready = msg
        elif kind == "ack":
            req = msg.get("req")
            with self._lock:
                pair = self._pending.get(int(req)) if req is not None else None
                if pair is not None:
                    self._pending[int(req)] = (pair[0], msg)
                    pair[0].set()

    def _read_loop(self) -> None:
        buf = b""
        sock = self._sock
        if sock is None:
            return
        while not self._stop.is_set():
            try:
                chunk = sock.recv(16384)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            buf, events = take_motion_frames(buf)
            for kind, payload in events:
                if kind == "telem":
                    self._apply_telem(payload)
                else:
                    self._apply_json(payload)
        self.connected = False
