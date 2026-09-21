from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import logging
import re
import struct
import subprocess
import sys
import threading
import time
import zlib

from robot_station.world import CamSnap

logger = logging.getLogger(__name__)

_VIDPID_RE = re.compile(r"VID_([0-9A-F]{4})&PID_([0-9A-F]{4})", re.I)
_BUILTIN_NAME_RE = re.compile(
    r"integrated|internal|built[\s-]*in|ir\s*camera|rgb\s*camera|"
    r"windows\s*hello|face\s*camera|privacy|mip[i]|sunplus|"
    r"front\s*camera|user\s*facing|world\s*facing",
    re.I,
)
_REALSENSE_NAME_RE = re.compile(r"realsense|d405|d415|d435|d455|depth camera\s*405", re.I)
# Laptop webcam vendors. 8086 here is Intel RealSense, not a builtin webcam.
_BUILTIN_VIDS = frozenset(
    {
        "5986",  # SunplusIT (this delivery laptop)
        "13D3",  # Azurewave
        "04F2",  # Chicony
        "1BCF",  # Bison
        "174F",  # Syntek
        "0BDA",  # Realtek laptop camera
        "04CA",  # Lite-On
        "1E4E",  # Dell / Realtek variants
        "0C45",  # Sonix (often lid camera)
    }
)
_REALSENSE_PIDS = frozenset(
    {
        "0AD3",  # D415
        "0B07",  # D435
        "0B3A",  # D435i
        "0B5B",  # D405
        "0B5C",  # D455
        "0B64",  # D457
        "0B6B",
        "0B51",
        "0B52",
    }
)


def rgb_png(rgb: bytes, width: int, height: int) -> bytes:
    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    raw = bytearray()
    row = width * 3
    for y in range(height):
        raw.append(0)
        raw.extend(rgb[y * row : (y + 1) * row])
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(bytes(raw), 1)) + chunk(b"IEND", b"")


@dataclass(frozen=True)
class CamInfo:
    name: str
    device_id: str = ""
    serial: str = ""
    vid: str = ""
    pid: str = ""
    kind: str = "uvc"  # realsense | uvc


def _vid_pid(device_id: str) -> tuple[str, str]:
    m = _VIDPID_RE.search(device_id or "")
    if not m:
        return "", ""
    return m.group(1).upper(), m.group(2).upper()


def _kind_for(name: str, vid: str, pid: str) -> str:
    if _REALSENSE_NAME_RE.search(name or "") or (vid == "8086" and pid in _REALSENSE_PIDS):
        return "realsense"
    return "uvc"


def score_camera(info: CamInfo) -> int:
    """Higher is more likely the delivered work camera. <=0 means skip (laptop webcam)."""
    name = info.name or ""
    vid = (info.vid or "").upper()
    pid = (info.pid or "").upper()
    score = 0
    if info.kind == "realsense" or _REALSENSE_NAME_RE.search(name) or (vid == "8086" and pid in _REALSENSE_PIDS):
        score += 100
        if pid == "0B5B" or "d405" in name.lower() or "405" in name:
            score += 40
    if _BUILTIN_NAME_RE.search(name):
        score -= 120
    if vid in _BUILTIN_VIDS:
        score -= 80
    if vid == "046D":  # Logitech
        score += 30
    if info.serial:
        score += 2
    return score


def pick_work_camera(candidates: list[CamInfo]) -> CamInfo | None:
    """Pick the USB work camera. Never auto-select the laptop lid webcam."""
    ranked: list[tuple[int, CamInfo]] = []
    for info in candidates:
        s = score_camera(info)
        if s > 0:
            ranked.append((s, info))
    if not ranked:
        return None
    ranked.sort(key=lambda row: row[0], reverse=True)
    return ranked[0][1]


def list_realsense_devices() -> list[CamInfo]:
    try:
        import pyrealsense2 as rs  # type: ignore
    except Exception:
        return []
    out: list[CamInfo] = []
    try:
        ctx = rs.context()
        for dev in ctx.query_devices():
            name = str(dev.get_info(rs.camera_info.name) or "RealSense")
            serial = str(dev.get_info(rs.camera_info.serial_number) or "")
            pid = str(dev.get_info(rs.camera_info.product_id) or "").replace("0x", "").upper()
            vid = "8086"
            out.append(
                CamInfo(
                    name=name,
                    device_id=f"realsense:{serial or pid}",
                    serial=serial,
                    vid=vid,
                    pid=pid,
                    kind="realsense",
                )
            )
    except Exception:
        logger.exception("enumerate RealSense")
    return out


def list_windows_cameras() -> list[CamInfo]:
    if sys.platform != "win32":
        return []
    script = (
        "[Console]::OutputEncoding = [System.Text.Encoding]::UTF8; "
        "Get-PnpDevice -Class Camera -Status OK -ErrorAction SilentlyContinue | "
        "Select-Object FriendlyName, InstanceId | ConvertTo-Json -Compress"
    )
    exe = r"C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe"
    try:
        proc = subprocess.run(
            [exe, "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", script],
            capture_output=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        logger.warning("无法列举 Windows 相机")
        return []
    raw = (proc.stdout or b"").decode("utf-8", "replace").strip()
    if not raw:
        return []
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        logger.warning("相机 PnP JSON 无法解析")
        return []
    if isinstance(data, dict):
        data = [data]
    out: list[CamInfo] = []
    for row in data:
        name = str(row.get("FriendlyName") or row.get("Name") or "").strip()
        device_id = str(row.get("InstanceId") or row.get("DeviceID") or "").strip()
        if not name:
            continue
        vid, pid = _vid_pid(device_id)
        out.append(
            CamInfo(
                name=name,
                device_id=device_id,
                vid=vid,
                pid=pid,
                kind=_kind_for(name, vid, pid),
            )
        )
    return out


def _linux_usb_ids(node: Path) -> tuple[str, str]:
    cur = node
    for _ in range(10):
        vid_f = cur / "idVendor"
        pid_f = cur / "idProduct"
        try:
            if vid_f.is_file() and pid_f.is_file():
                vid = vid_f.read_text(encoding="ascii", errors="ignore").strip().upper()
                pid = pid_f.read_text(encoding="ascii", errors="ignore").strip().upper()
                return vid, pid
        except OSError:
            return "", ""
        parent = cur.parent
        if parent == cur:
            break
        cur = parent
    return "", ""


def list_linux_cameras(sys_root: str | Path | None = None) -> list[CamInfo]:
    """Enumerate ``/sys/class/video4linux``. Never opens a capture device."""
    if sys.platform.startswith("linux") is False and sys_root is None:
        return []
    root = Path(sys_root) if sys_root is not None else Path("/sys/class/video4linux")
    try:
        if not root.is_dir():
            return []
    except OSError:
        return []
    out: list[CamInfo] = []
    try:
        entries = sorted(root.iterdir(), key=lambda p: p.name)
    except OSError:
        return []
    for node in entries:
        name_f = node / "name"
        try:
            name = name_f.read_text(encoding="utf-8", errors="replace").strip() if name_f.is_file() else node.name
        except OSError:
            name = node.name
        if not name or "metadata" in name.lower():
            continue
        device_link = node / "device"
        try:
            resolved = device_link.resolve() if device_link.exists() else node
        except OSError:
            resolved = node
        vid, pid = _linux_usb_ids(resolved)
        device_id = f"/dev/{node.name}"
        out.append(
            CamInfo(
                name=name,
                device_id=device_id,
                vid=vid,
                pid=pid,
                kind=_kind_for(name, vid, pid),
            )
        )
    return out


def discover_cameras() -> list[CamInfo]:
    """RealSense SDK first. OS camera class is a fallback if SDK sees nothing."""
    found: list[CamInfo] = []
    seen_id: set[str] = set()
    seen_vp: set[str] = set()

    def _add(info: CamInfo) -> None:
        ident = info.serial or info.device_id or info.name.lower()
        vp = f"{info.vid}:{info.pid}" if info.vid and info.pid else ""
        if ident in seen_id or (vp and vp in seen_vp):
            return
        seen_id.add(ident)
        if vp:
            seen_vp.add(vp)
        found.append(info)

    rs = list_realsense_devices()
    for info in rs:
        _add(info)
    if not any(score_camera(c) > 0 for c in found):
        for info in list_windows_cameras():
            _add(info)
        for info in list_linux_cameras():
            _add(info)
    return found


def _bgr_frame_to_png(bgr, out_w: int, out_h: int) -> bytes:
    import numpy as np

    arr = np.ascontiguousarray(bgr)
    if arr.ndim != 3 or arr.shape[2] < 3:
        raise ValueError("not a color frame")
    h, w = int(arr.shape[0]), int(arr.shape[1])
    if (w, h) != (out_w, out_h):
        ys = (np.linspace(0, h - 1, out_h)).astype(np.intp)
        xs = (np.linspace(0, w - 1, out_w)).astype(np.intp)
        arr = arr[ys][:, xs]
    rgb = np.ascontiguousarray(arr[:, :, :3][:, :, ::-1])
    return rgb_png(rgb.tobytes(), out_w, out_h)


class CameraBank:
    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def latest_png(self, stream_id: int) -> tuple[int, int, bytes] | None:
        raise NotImplementedError

    def poll(self) -> CamSnap:
        raise NotImplementedError

    def stream_ids(self) -> list[int]:
        return [0]

    def set_paused(self, paused: bool) -> None:
        return None


class MockCameraBank(CameraBank):
    PALETTES = (
        ((40, 90, 70), (30, 170, 110), (20, 40, 50)),
        ((30, 70, 110), (80, 180, 220), (16, 28, 40)),
    )

    def __init__(self, count: int, width: int, height: int, hz: float, reason: str = "") -> None:
        self.count = max(1, min(8, int(count)))
        self.width = int(width)
        self.height = int(height)
        self.period = 1.0 / max(1.0, float(hz))
        self.roles = ["作业相机"] if self.count == 1 else [f"作业相机 {i + 1}" for i in range(self.count)]
        self.backend = "mock"
        self.device = ""
        self.pick_reason = reason
        self._png = [b""] * self.count
        self._seq = [0] * self.count
        self._fps = [0.0] * self.count
        self._hits = [0] * self.count
        self._hit_t = [time.monotonic()] * self.count
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._lock = threading.Lock()
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        self._stop.clear()
        for i in range(self.count):
            th = threading.Thread(target=self._run, args=(i,), name=f"cam-mock-{i}", daemon=True)
            th.start()
            self._threads.append(th)

    def stop(self) -> None:
        self._stop.set()
        for th in self._threads:
            th.join(timeout=1.0)
        self._threads.clear()

    def set_paused(self, paused: bool) -> None:
        if paused:
            self._paused.set()
        else:
            self._paused.clear()

    def stream_ids(self) -> list[int]:
        return list(range(self.count))

    def latest_png(self, stream_id: int) -> tuple[int, int, bytes] | None:
        if stream_id < 0 or stream_id >= self.count:
            return None
        with self._lock:
            blob = self._png[stream_id]
        if not blob:
            return None
        return self.width, self.height, blob

    def poll(self) -> CamSnap:
        with self._lock:
            return CamSnap(
                online=[True] * self.count,
                fps=list(self._fps),
                roles=list(self.roles),
                backend="mock",
                device="",
                reason=self.pick_reason,
            )

    def _run(self, idx: int) -> None:
        bg, accent, base = self.PALETTES[idx % len(self.PALETTES)]
        w, h = self.width, self.height
        next_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if self._paused.is_set():
                next_t = now + self.period
                time.sleep(0.05)
                continue
            if now < next_t:
                time.sleep(min(0.01, next_t - now))
                continue
            next_t += self.period
            if next_t < now:
                next_t = now + self.period
            shift = int((now * 80) % w)
            rgb = bytearray(w * h * 3)
            bar_w = max(8, w // 8)
            for y in range(h):
                for x in range(w):
                    o = (y * w + x) * 3
                    band = ((x + shift) // bar_w) & 1
                    col = accent if band else bg
                    if y < 18:
                        col = base
                    if abs(x - shift) < 3:
                        col = (230, 240, 255)
                    rgb[o : o + 3] = bytes(col)
            png = rgb_png(bytes(rgb), w, h)
            with self._lock:
                self._png[idx] = png
                self._seq[idx] += 1
                self._hits[idx] += 1
                elapsed = now - self._hit_t[idx]
                if elapsed >= 1.0:
                    self._fps[idx] = self._hits[idx] / elapsed
                    self._hits[idx] = 0
                    self._hit_t[idx] = now


class RealSenseCameraBank(CameraBank):
    def __init__(self, info: CamInfo, width: int, height: int, hz: float) -> None:
        self.info = info
        self.width = int(width)
        self.height = int(height)
        self.hz = max(5.0, float(hz))
        self.roles = ["作业相机"]
        self.backend = "realsense"
        self.device = info.name
        self._png = b""
        self._fps = 0.0
        self._hits = 0
        self._hit_t = time.monotonic()
        self._online = False
        self._err = ""
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._pipeline = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="cam-realsense", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.5)
        self._thread = None
        self._close_pipeline()

    def set_paused(self, paused: bool) -> None:
        if paused:
            self._paused.set()
        else:
            self._paused.clear()

    def stream_ids(self) -> list[int]:
        return [0]

    def latest_png(self, stream_id: int) -> tuple[int, int, bytes] | None:
        if stream_id != 0:
            return None
        with self._lock:
            blob = self._png
        if not blob:
            return None
        return self.width, self.height, blob

    def poll(self) -> CamSnap:
        with self._lock:
            online = self._online
            fps = self._fps
            err = self._err
        return CamSnap(
            online=[online],
            fps=[fps],
            roles=list(self.roles),
            backend="realsense",
            device=self.device,
            reason=err,
        )

    def _close_pipeline(self) -> None:
        pipe = self._pipeline
        self._pipeline = None
        if pipe is None:
            return
        try:
            pipe.stop()
        except Exception:
            pass

    def _open_pipeline(self):
        import pyrealsense2 as rs  # type: ignore

        fps_try = [15, 30, 6]
        want = int(round(self.hz))
        if want in (6, 15, 30) and want not in fps_try:
            fps_try.insert(0, want)
        sizes = ((640, 480), (848, 480), (424, 240), (1280, 720))
        last_err: Exception | None = None
        for fps in fps_try:
            for w, h in sizes:
                for with_depth in (False, True):
                    cfg = rs.config()
                    pipe = None
                    if self.info.serial:
                        cfg.enable_device(self.info.serial)
                    try:
                        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
                        if with_depth:
                            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
                        pipe = rs.pipeline()
                        pipe.start(cfg)
                        logger.info(
                            "RealSense 作业相机 %s serial=%s %sx%s@%s depth=%s",
                            self.info.name,
                            self.info.serial,
                            w,
                            h,
                            fps,
                            with_depth,
                        )
                        return pipe
                    except Exception as exc:
                        last_err = exc
                        if pipe is not None:
                            try:
                                pipe.stop()
                            except Exception:
                                pass
        if last_err is not None:
            raise last_err
        raise RuntimeError("RealSense 无法打开彩色流")

    def _run(self) -> None:
        try:
            self._pipeline = self._open_pipeline()
        except Exception as exc:
            logger.exception("打开 RealSense 失败")
            with self._lock:
                self._online = False
                self._err = str(exc)
            return
        period = 1.0 / min(30.0, max(5.0, self.hz))
        next_enc = time.monotonic()
        while not self._stop.is_set():
            pipe = self._pipeline
            if pipe is None:
                break
            try:
                frames = pipe.wait_for_frames(timeout_ms=1000)
            except Exception:
                with self._lock:
                    self._online = False
                continue
            color = frames.get_color_frame()
            if not color:
                continue
            now = time.monotonic()
            with self._lock:
                self._online = True
                self._err = ""
                self._hits += 1
                elapsed = now - self._hit_t
                if elapsed >= 1.0:
                    self._fps = self._hits / elapsed
                    self._hits = 0
                    self._hit_t = now
            if self._paused.is_set() or now < next_enc:
                continue
            next_enc = now + period
            try:
                import numpy as np

                bgr = np.ascontiguousarray(color.get_data())
                png = _bgr_frame_to_png(bgr, self.width, self.height)
            except Exception:
                logger.exception("编码作业相机帧")
                continue
            with self._lock:
                self._png = png
        self._close_pipeline()


class AutoCameraBank(CameraBank):
    """Pick the delivered USB work camera at start; skip the laptop webcam."""

    def __init__(self, count: int, width: int, height: int, hz: float) -> None:
        self.count = max(1, int(count))
        self.width = int(width)
        self.height = int(height)
        self.hz = float(hz)
        self._inner: CameraBank = MockCameraBank(self.count, self.width, self.height, self.hz)
        self._swap_lock = threading.Lock()
        self._paused = False

    def start(self) -> None:
        try:
            picked = self._choose()
        except Exception:
            logger.exception("识别作业相机失败")
            picked = MockCameraBank(
                self.count, self.width, self.height, self.hz, reason="识别作业相机失败"
            )
        old = self._inner
        self._inner = picked
        picked.start()
        picked.set_paused(self._paused)
        if old is not picked:
            try:
                old.stop()
            except Exception:
                pass

    def rescan(self) -> CameraBank:
        """Re-run the pick (USB work camera plugged/unplugged) and swap the backend.

        Called only when the device signature actually changed
        (``robot_station.usb_watch``). Keeps the current pause state, so a swap
        during teleop does not resume encoding. Never raises: on failure the
        previous backend stays in place.
        """
        with self._swap_lock:
            was_paused = bool(self._paused)
            try:
                picked = self._choose()
            except Exception:
                logger.exception("重新识别作业相机失败，沿用当前后端")
                return self._inner
            old = self._inner
            same = type(picked) is type(old) and (
                getattr(picked, "device", None) == getattr(old, "device", None)
            )
            if same:
                try:
                    picked.stop()
                except Exception:
                    pass
                return old
            self._inner = picked
            try:
                picked.start()
            except Exception:
                logger.exception("新相机后端启动失败，回退旧后端")
                self._inner = old
                try:
                    picked.stop()
                except Exception:
                    pass
                return old
            picked.set_paused(was_paused)
            try:
                old.stop()
            except Exception:
                logger.exception("停旧相机后端失败")
            logger.info("作业相机已切换为 %s", getattr(picked, "device", "") or type(picked).__name__)
            return picked

    def stop(self) -> None:
        self._inner.stop()

    def set_paused(self, paused: bool) -> None:
        self._paused = bool(paused)
        self._inner.set_paused(paused)

    def stream_ids(self) -> list[int]:
        return self._inner.stream_ids()

    def latest_png(self, stream_id: int) -> tuple[int, int, bytes] | None:
        return self._inner.latest_png(stream_id)

    def poll(self) -> CamSnap:
        return self._inner.poll()

    def _choose(self) -> CameraBank:
        found = discover_cameras()
        for info in found:
            logger.info(
                "相机候选 score=%s kind=%s %s vid=%s pid=%s serial=%s",
                score_camera(info),
                info.kind,
                info.name,
                info.vid,
                info.pid,
                info.serial,
            )
        picked = pick_work_camera(found)
        skipped = [c.name for c in found if score_camera(c) <= 0]
        if picked is None:
            if skipped:
                reason = "未找到作业相机（已忽略笔记本内置摄像头：" + "、".join(skipped) + "）"
            else:
                reason = "未找到 USB 作业相机"
            logger.warning("%s", reason)
            return MockCameraBank(self.count, self.width, self.height, self.hz, reason=reason)
        if picked.kind == "realsense":
            try:
                import pyrealsense2 as rs  # noqa: F401
            except Exception:
                reason = f"已识别 {picked.name}，但未安装 pyrealsense2"
                logger.warning("%s", reason)
                return MockCameraBank(self.count, self.width, self.height, self.hz, reason=reason)
            logger.info("选用作业相机 %s", picked.name)
            return RealSenseCameraBank(picked, self.width, self.height, self.hz)
        reason = f"已识别 {picked.name}，当前只支持 RealSense 作业相机"
        logger.warning("%s", reason)
        return MockCameraBank(self.count, self.width, self.height, self.hz, reason=reason)


def build_camera(kind: str, count: int, width: int, height: int, hz: float) -> CameraBank:
    name = (kind or "auto").strip().lower()
    if name in ("mock", "fake", "off"):
        return MockCameraBank(count=count, width=width, height=height, hz=hz)
    if name in ("auto", "live", "usb", "realsense"):
        return AutoCameraBank(count=count, width=width, height=height, hz=hz)
    raise ValueError(f"不支持 camera={kind!r}，可用 mock / auto")


def _probe() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    found = discover_cameras()
    if not found:
        print("未枚举到相机")
        return 1
    print("检测到的相机：")
    for info in found:
        mark = "SKIP" if score_camera(info) <= 0 else "ok"
        print(
            f"  [{mark}] score={score_camera(info):4d}  {info.kind:10s}  "
            f"{info.name}  VID_{info.vid}&PID_{info.pid}  serial={info.serial or '-'}"
        )
    picked = pick_work_camera(found)
    if picked is None:
        print("作业相机：无（不会使用笔记本内置摄像头）")
        return 2
    print(f"作业相机：{picked.name}  ({picked.kind} {picked.serial or picked.pid})")
    return 0


if __name__ == "__main__":
    raise SystemExit(_probe())
