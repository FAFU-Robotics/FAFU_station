"""Motion lock and occupancy.

Lock files live in the OS temp directory so the same code runs on Windows PC
and Linux PC.
"""

from __future__ import annotations

import logging
import os
import socket
import time
from pathlib import Path

logger = logging.getLogger("station.lock")

if os.name == "nt":
    import msvcrt
else:
    import fcntl


def _lock_dir() -> Path:
    if os.name == "nt":
        raw = os.environ.get("TEMP") or os.environ.get("TMP") or str(Path.home() / "AppData" / "Local" / "Temp")
        return Path(raw)
    return Path("/tmp")


MOTION_LOCK = _lock_dir() / "fafu_station_motion.lock"
_HELD_PATHS: set[str] = set()


def _lock_key(path: Path) -> str:
    try:
        resolved = str(Path(path).resolve())
    except OSError:
        resolved = str(path)
    return resolved.lower() if os.name == "nt" else resolved


def _prepare_lock_file(fh) -> bool:
    try:
        fh.seek(0)
        if not fh.read(1):
            fh.write(b"0")
            fh.flush()
        fh.seek(0)
        return True
    except OSError:
        try:
            fh.seek(0)
        except OSError:
            pass
        return False


def _try_exclusive(fh) -> bool:
    if not _prepare_lock_file(fh):
        return False
    if os.name == "nt":
        try:
            msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
            return True
        except OSError:
            return False
    try:
        fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return True
    except BlockingIOError:
        return False


def _unlock(fh) -> None:
    try:
        fh.seek(0)
        if os.name == "nt":
            msvcrt.locking(fh.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
    except OSError:
        pass


def lock_held(path: Path) -> bool:
    """True if this or another process currently holds the lock."""
    target = Path(path)
    if _lock_key(target) in _HELD_PATHS:
        return True
    if not target.is_file():
        return False
    try:
        with open(target, "a+b") as fh:
            if not _try_exclusive(fh):
                return True
            _unlock(fh)
            return False
    except OSError:
        return False


def lock_holder_pid(path: Path) -> int | None:
    target = Path(path)
    if not target.is_file():
        return None
    try:
        raw = target.read_text(encoding="utf-8").strip().splitlines()
        if not raw:
            return None
        pid = int(raw[0].strip())
        return pid if pid > 0 else None
    except (OSError, ValueError):
        return None


class ExclusiveLock:
    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._fh = None

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "a+b")
        if not _try_exclusive(self._fh):
            who = lock_holder_pid(self.path)
            extra = f"，持有进程 pid={who}" if who else ""
            raise RuntimeError(f"锁已被占用：{self.path}{extra}（已有一份站控在跑？不要再开一份。）")
        _HELD_PATHS.add(_lock_key(self.path))
        try:
            self._fh.seek(0)
            self._fh.write(f"{os.getpid()}\n".encode("ascii"))
            self._fh.flush()
        except OSError:
            pass

    def release(self) -> None:
        if self._fh is None:
            return
        _HELD_PATHS.discard(_lock_key(self.path))
        _unlock(self._fh)
        try:
            self._fh.close()
        except OSError:
            pass
        self._fh = None

    def __enter__(self) -> "ExclusiveLock":
        self.acquire()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.release()


def list_ipv4_addrs() -> list[str]:
    """Non-loopback, non-docker IPv4 (used only when listen_host is all-interfaces)."""
    from robot_station.netinfo import list_lan_ips

    return list_lan_ips()


def port_open(port: int, host: str = "127.0.0.1") -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.15)
    try:
        return sock.connect_ex((host, int(port))) == 0
    finally:
        sock.close()


def wait_lock_free(path: Path, timeout_s: float = 3.0) -> bool:
    deadline = time.monotonic() + max(0.05, float(timeout_s))
    while time.monotonic() < deadline:
        if not lock_held(path):
            return True
        time.sleep(0.05)
    return not lock_held(path)
