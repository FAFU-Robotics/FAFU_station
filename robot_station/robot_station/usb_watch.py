"""USB change watcher: monitor device presence, detect only when it changed.

The station already knows how to *probe* hardware — ``fafu_motor``
``find_likely_debug_boards()`` for the arm debug board and
``camera.discover_cameras()`` for the work camera. What it never had is a
trigger: both probes run exactly once at startup and are never repeated, so a
cable plugged in later is not noticed.

This module adds the missing half, without opening any device:

* :func:`serial_signature` / :func:`camera_signature` are cheap **enumerations**
  (COM port list / camera list). They never open the serial port and never
  start a RealSense pipeline.
* :class:`UsbWatcher` polls those signatures on its own thread. When a
  signature is unchanged it does nothing at all; only a *change* leads to a
  :meth:`UsbWatcher.rescan` call, and only a change that survives the debounce
  window is reported to the arm / camera callbacks.

Nothing here is started by default. ``StationConfig.arm_watch`` /
``camera_watch`` gate it, so a plain ``MotionCore`` (unit tests, ``arm: mock``
developer runs) behaves exactly as before.
"""
from __future__ import annotations

import importlib
import logging
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from typing import Any

logger = logging.getLogger("station.usb")

# Cheap enumeration. The serial probe is a Windows SetupAPI walk (a few ms);
# the camera probe is one PowerShell PnP query or a RealSense device query and
# is an order of magnitude more expensive, so it runs on a slower divisor.
DEFAULT_INTERVAL_S = 1.5
DEFAULT_CAMERA_EVERY = 4
DEFAULT_SETTLE_S = 0.4
DEFAULT_SETTLE_ROUNDS = 2
# 当 fafu_motor 还不在 sys.modules 里（站控启动时真机没连上、SDK 未曾加载）
# 时，最多每隔这么久借站控自己的 SDK 解析去加载一次，好让监视看得见调试板。
SDL_LOAD_RETRY_S = 60.0

Signature = tuple[tuple[str, ...], ...]

_motor_load_lock = threading.Lock()
_motor_load_last = 0.0


def _as_text(value: Any) -> str:
    try:
        text = str(value)
    except Exception:
        return ""
    return text.strip()


def serial_ports() -> list[str]:
    """Ports the arm adapter itself would consider a debug board.

    Uses the SDK enum (``find_likely_debug_boards``, a USB-serial VID
    whitelist) — the same call ``_pick_serial_port`` uses, so "the watcher saw
    a board" and "the adapter can open a board" can never disagree.

    Deliberately **no** fallback to the full port list: Windows always exposes
    Bluetooth/virtual ports (this machine reports COM3/COM4 with empty
    VID/PID while the whitelist returns nothing), and treating those as a
    debug board would make the watcher switch to a live backend that then
    fails to open. Reporting nothing is the honest answer.

    Never opens a port, never raises.
    """
    try:
        pm = _load_motor_module()
        if pm is None:
            return []
        try:
            boards = list(pm.find_likely_debug_boards())
        except Exception:
            logger.debug("调试板枚举失败", exc_info=True)
            return []
        return sorted({p for p in (_as_text(getattr(b, "port", b)) for b in boards) if p})
    except Exception:
        logger.debug("USB 串口枚举失败", exc_info=True)
        return []


def _load_motor_module() -> Any | None:
    """Return ``fafu_motor`` if it can be reached, else ``None``.

    The watcher must never *open* a device, but it does need the module to
    enumerate. Two ways in, in order:

    1. ``sys.modules`` / a plain import — the normal case, because the arm
       adapter calls ``import_fafu_sdk()`` before it tries to open the board,
       so the module is already loaded even when the connection then fails.
    2. A throttled fallback that borrows the station's own SDK resolution
       (``fafu_arm.import_fafu_sdk``) at most once per ``SDL_LOAD_RETRY_S``.
       Needed when the very first ``arm.start()`` never got as far as loading
       the SDK, e.g. the SDK directory was found but something else failed.

    Never adds a search path of its own and never raises.
    """
    global _motor_load_last
    try:
        return importlib.import_module("fafu_motor")
    except Exception:
        pass
    now = time.monotonic()
    if now - _motor_load_last < SDL_LOAD_RETRY_S:
        return None
    if not _motor_load_lock.acquire(blocking=False):
        return None
    try:
        _motor_load_last = now
        try:
            from robot_station.adapters.fafu_arm import import_fafu_sdk, resolve_sdk_root

            import_fafu_sdk(resolve_sdk_root())
        except Exception:
            logger.debug("借站控路径加载 fafu_motor 失败", exc_info=True)
        try:
            return importlib.import_module("fafu_motor")
        except Exception:
            return None
    finally:
        _motor_load_lock.release()


def camera_devices() -> list[str]:
    """Camera identifiers present right now. Never opens a camera, never raises.

    RealSense first (``pyrealsense2`` device query, no pipeline), then the
    OS camera list (Windows PnP or Linux v4l2). Enumeration only — the
    laptop lid webcam is included here on purpose, because the watcher only
    cares whether the *set of devices* changed.
    """
    out: list[str] = []
    try:
        from robot_station.adapters.camera import list_realsense_devices

        for info in list_realsense_devices():
            out.append(_as_text(getattr(info, "serial", "") or getattr(info, "device_id", "")))
    except Exception:
        logger.debug("RealSense 枚举失败", exc_info=True)
    if not out:
        try:
            from robot_station.adapters.camera import list_linux_cameras, list_windows_cameras

            for info in list_windows_cameras():
                out.append(_as_text(getattr(info, "device_id", "") or getattr(info, "name", "")))
            for info in list_linux_cameras():
                out.append(_as_text(getattr(info, "device_id", "") or getattr(info, "name", "")))
        except Exception:
            logger.debug("系统相机枚举失败", exc_info=True)
    return sorted({x for x in out if x})


def _signature(groups: Iterable[Iterable[str]]) -> Signature:
    return tuple(tuple(sorted({_as_text(v) for v in group if _as_text(v)})) for group in groups)


def serial_signature() -> Signature:
    return _signature([serial_ports()])


def camera_signature() -> Signature:
    return _signature([camera_devices()])


def device_signature() -> Signature:
    """Both probes in one call. Handy for logs; the watcher uses them separately."""
    return _signature([serial_ports(), camera_devices()])


class UsbWatcher:
    """Poll the device signatures; report only *settled changes*.

    ``on_arm(attached)`` fires when the debug-board port set changed and stayed
    changed for :attr:`settle_rounds` probes (``attached=True`` = a board
    appeared). ``on_camera(attached)`` fires the same way for the camera set.

    The callbacks run on the watcher thread and are wrapped in ``try`` — a
    raising callback is logged and never kills the loop.
    """

    def __init__(
        self,
        *,
        interval_s: float = DEFAULT_INTERVAL_S,
        camera_every: int = DEFAULT_CAMERA_EVERY,
        settle_s: float = DEFAULT_SETTLE_S,
        settle_rounds: int = DEFAULT_SETTLE_ROUNDS,
        serial_fn: Callable[[], Signature] | None = None,
        camera_fn: Callable[[], Signature] | None = None,
        on_arm: Callable[[bool], None] | None = None,
        on_camera: Callable[[bool], None] | None = None,
    ) -> None:
        self.interval_s = max(0.05, float(interval_s))
        self.camera_every = max(1, int(camera_every))
        self.settle_s = max(0.0, float(settle_s))
        self.settle_rounds = max(1, int(settle_rounds))
        self._serial_fn = serial_fn or serial_signature
        self.on_arm = on_arm
        self.on_camera = on_camera
        self._camera_fn = camera_fn
        if self._camera_fn is None and on_camera is not None:
            self._camera_fn = camera_signature
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._last: Signature | None = None
        self._last_camera: tuple[str, ...] | None = None
        self.checks = 0
        self.changes = 0
        self.rescans = 0
        self.camera_checks = 0

    # ---- lifecycle -----------------------------------------------------

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._last = None
        self._last_camera = None
        self._thread = threading.Thread(target=self._loop, name="usb-watch", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        th = self._thread
        self._thread = None
        if th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(timeout=max(0.0, float(timeout)))

    # ---- one round (also the unit-test entry point) ---------------------

    def rescan(self) -> None:
        """Take one signature reading without waiting for the interval.

        Called after a manual arm-source switch so the watcher adopts the new
        reality instead of immediately undoing it.
        """
        self._round()

    def _round(self) -> None:
        with self._lock:
            tick = self.checks
            self.checks += 1
        try:
            serial = self._serial_fn()
        except Exception:
            logger.debug("串口签名读取失败", exc_info=True)
            return

        prev_serial = self._last
        if prev_serial is not None and serial != prev_serial:
            self.changes += 1
            self.rescans += 1
            logger.info("USB 变化：%s → %s", _fmt(prev_serial), _fmt(serial))
            if self.settle_rounds > 1:
                time.sleep(self.settle_s)
                try:
                    again = self._serial_fn()
                except Exception:
                    again = serial
                if again != serial:
                    # Still moving (a board that resets on power-up enumerates
                    # twice). Adopt the newer reading and wait for the next tick.
                    self._last = again
                    self._note_camera(tick)
                    return
            self._emit_serial(prev_serial, serial)
        self._last = serial
        self._note_camera(tick)

    def _note_camera(self, tick: int) -> None:
        if self.on_camera is None or self._camera_fn is None:
            return
        if tick % self.camera_every != 0:
            return
        try:
            current = self._camera_fn()[0]
        except Exception:
            logger.debug("相机签名读取失败", exc_info=True)
            return
        self.camera_checks += 1
        prev = self._last_camera
        self._last_camera = current
        if prev is None or current == prev:
            return
        self._emit(self.on_camera, bool(current), "相机")

    def _emit_serial(self, prev: Signature, current: Signature) -> None:
        """Report any port-set change while a board is present.

        Covers three cases with one signal: appeared, disappeared, and
        "still present but on another port". The last one matters because a
        quick unplug/re-plug can happen entirely between two polls — the
        watcher would see COM3 → COM7 with no empty reading in between, and if
        that were treated as "already attached, nothing to do" the arm would
        stay on the sim backend.
        """
        if self.on_arm is None:
            return
        now = bool(current[0]) if current else False
        self._emit(self.on_arm, now, "机械臂")

    def _emit(self, fn: Callable[[bool], None], attached: bool, what: str) -> None:
        logger.info("%s %s", what, "已接上" if attached else "已断开")
        try:
            fn(attached)
        except Exception:
            logger.warning("%s 变化回调失败", what, exc_info=True)

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._round()
            except Exception:
                logger.warning("USB 监视轮次异常", exc_info=True)
            self._stop.wait(self.interval_s)


def _fmt(sig: Signature) -> str:
    parts = ["、".join(group) or "-" for group in sig]
    return " | ".join(parts)


def watch_enabled(cfg: Any) -> tuple[bool, bool]:
    """``(arm_watch, camera_watch)`` from a config object, tolerant of absence."""
    return bool(getattr(cfg, "arm_watch", False)), bool(getattr(cfg, "camera_watch", False))


def describe_devices(ports: Sequence[str] | None = None, cameras: Sequence[str] | None = None) -> str:
    p = list(ports) if ports is not None else serial_ports()
    c = list(cameras) if cameras is not None else camera_devices()
    return f"串口={p or '无'} 相机={c or '无'}"
