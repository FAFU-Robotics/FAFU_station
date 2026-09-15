"""Continuous teach trajectories: jsonl on disk, timed servo_j playback.

Drag (Gravity, pure G(q)) and Position software teleop share one sampler.
Playback is Position-only timestamped ``servo_j``, not MIT and not the sparse
waypoint panel.
"""
from __future__ import annotations

import json
import os
import re
import threading
import time
from pathlib import Path
from typing import Any

PLAYBACK_DT = 0.01
SMOOTH_WINDOW = 7
REPLAY_RATE_MIN = 0.5
REPLAY_RATE_MAX = 1.0
_UNSAFE_NAME = re.compile(r'[\\/:*?"<>|\x00-\x1f]+')


def recordings_dir() -> Path:
    override = (os.environ.get("STATION_RECORDINGS_DIR") or "").strip()
    if override:
        return Path(override)
    local = (os.environ.get("LOCALAPPDATA") or "").strip()
    if local:
        return Path(local) / "FAFUArmStation" / "recordings"
    xdg = (os.environ.get("XDG_DATA_HOME") or "").strip()
    if xdg:
        return Path(xdg) / "FAFUArmStation" / "recordings"
    return Path.home() / ".local" / "share" / "FAFUArmStation" / "recordings"


def sanitize_traj_name(name: str | None) -> str:
    raw = (name or "").strip()
    if not raw:
        return ""
    base = Path(raw).name
    if base.lower().endswith(".jsonl"):
        stem = base[:-6]
        suffix = ".jsonl"
    else:
        stem, suffix = base, ".jsonl"
    stem = _UNSAFE_NAME.sub("", stem).strip(" ._")
    if not stem or stem in {".", ".."}:
        return ""
    return stem + suffix


def default_traj_name() -> str:
    return time.strftime("traj_%Y%m%d_%H%M%S.jsonl")


def make_traj_path(name: str | None = None) -> Path:
    root = recordings_dir()
    root.mkdir(parents=True, exist_ok=True)
    raw = sanitize_traj_name(name)
    if not raw:
        raw = default_traj_name()
    path = root / raw
    if not path.exists():
        return path
    stem = path.stem
    i = 2
    while True:
        cand = root / f"{stem}_{i}.jsonl"
        if not cand.exists():
            return cand
        i += 1


def resolve_traj_path(name: str) -> Path:
    raw = (name or "").strip()
    if not raw:
        raise FileNotFoundError("没有轨迹文件")
    root = recordings_dir().resolve()
    cand = Path(raw)
    if not cand.is_absolute():
        cand = root / Path(raw).name
    if cand.suffix.lower() != ".jsonl":
        cand = cand.with_suffix(".jsonl")
    resolved = cand.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise FileNotFoundError("轨迹文件不在录制目录内") from exc
    if not resolved.is_file():
        raise FileNotFoundError(f"找不到轨迹 {resolved.name}")
    return resolved


def delete_recording(name: str) -> Path:
    """Remove a jsonl under the recordings directory. Path traversal is refused."""
    path = resolve_traj_path(name)
    path.unlink()
    return path


def rename_recording(src: str, dest: str) -> Path:
    """Rename a jsonl inside the recordings directory. Path traversal is refused."""
    old = resolve_traj_path(src)
    new_name = sanitize_traj_name(dest)
    if not new_name:
        raise ValueError("请输入有效的文件名")
    root = recordings_dir().resolve()
    new = (root / new_name).resolve()
    try:
        new.relative_to(root)
    except ValueError as exc:
        raise FileNotFoundError("轨迹文件不在录制目录内") from exc
    if new == old:
        return old
    if new.exists():
        raise FileExistsError(f"已有同名轨迹 {new.name}")
    old.rename(new)
    return new


def clip_replay_rate(rate: float) -> float:
    try:
        value = float(rate)
    except (TypeError, ValueError):
        value = 1.0
    if value <= 0:
        value = 1.0
    return max(REPLAY_RATE_MIN, min(REPLAY_RATE_MAX, value))


def is_header(obj: dict[str, Any]) -> bool:
    return obj.get("kind") == "traj" or "pos" not in obj


class TrajectoryWriter:
    def __init__(self, path: Path, *, teach: str, mode: str, n: int) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.teach = teach
        self.mode = mode
        self.n = int(n)
        self.t0 = time.monotonic()
        self.count = 0
        self._lock = threading.Lock()
        self._closed = False
        self._fh = path.open("w", encoding="utf-8")
        header = {"v": 1, "kind": "traj", "teach": teach, "mode": mode, "n": int(n)}
        self._fh.write(json.dumps(header, ensure_ascii=False) + "\n")
        self._fh.flush()

    def log(
        self,
        pos: list[float],
        vel: list[float],
        gripper_pos: float | None = None,
        gripper_vel: float | None = None,
    ) -> None:
        rec = {
            "t": time.monotonic() - self.t0,
            "pos": [float(x) for x in pos],
            "vel": [float(x) for x in vel],
            "gpos": None if gripper_pos is None else float(gripper_pos),
            "gvel": 0.0 if gripper_vel is None else float(gripper_vel),
        }
        line = json.dumps(rec, ensure_ascii=False) + "\n"
        with self._lock:
            if self._closed:
                return
            self._fh.write(line)
            self.count += 1
            if self.count % 10 == 0:
                self._fh.flush()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass


def load_traj(path: str | Path) -> tuple[dict[str, Any] | None, list[dict[str, Any]]]:
    header: dict[str, Any] | None = None
    frames: list[dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            obj = json.loads(line)
            if not isinstance(obj, dict):
                continue
            if is_header(obj):
                if obj.get("kind") == "traj":
                    header = obj
                continue
            frames.append(obj)
    return header, frames


def _moving_average(rows: list[list[float]], window: int) -> list[list[float]]:
    if window <= 1 or not rows:
        return rows
    if window % 2 == 0:
        window += 1
    pad = window // 2
    n = len(rows)
    cols = len(rows[0])
    out: list[list[float]] = []
    inv = 1.0 / float(window)
    for i in range(n):
        acc = [0.0] * cols
        for k in range(-pad, pad + 1):
            src = rows[min(n - 1, max(0, i + k))]
            for c in range(cols):
                acc[c] += float(src[c]) if c < len(src) else 0.0
        out.append([x * inv for x in acc])
    return out


def _moving_average_1d(values: list[float], window: int) -> list[float]:
    if not values:
        return values
    smoothed = _moving_average([[float(x)] for x in values], window)
    return [row[0] for row in smoothed]


def _interp_series(t: list[float], y: list[float], tq: list[float]) -> list[float]:
    n = len(t)
    if n == 0:
        return [0.0] * len(tq)
    if n == 1:
        return [float(y[0])] * len(tq)
    out: list[float] = []
    j = 0
    last = n - 1
    for tq_i in tq:
        if tq_i <= t[0]:
            out.append(float(y[0]))
            continue
        if tq_i >= t[-1]:
            out.append(float(y[-1]))
            continue
        while j < last - 1 and t[j + 1] < tq_i:
            j += 1
        t0 = t[j]
        t1 = t[j + 1]
        span = t1 - t0
        u = 0.0 if span <= 1e-12 else (tq_i - t0) / span
        out.append(float(y[j]) + (float(y[j + 1]) - float(y[j])) * u)
    return out


def _gradient(y: list[float], t: list[float]) -> list[float]:
    n = len(y)
    if n == 0:
        return []
    if n == 1:
        return [0.0]
    out = [0.0] * n
    dt0 = t[1] - t[0]
    out[0] = (y[1] - y[0]) / dt0 if abs(dt0) > 1e-12 else 0.0
    dtn = t[-1] - t[-2]
    out[-1] = (y[-1] - y[-2]) / dtn if abs(dtn) > 1e-12 else 0.0
    for i in range(1, n - 1):
        dt = t[i + 1] - t[i - 1]
        out[i] = (y[i + 1] - y[i - 1]) / dt if abs(dt) > 1e-12 else 0.0
    return out


def prepare_playback(
    samples: list[dict[str, Any]],
    playback_dt: float = PLAYBACK_DT,
    smooth_window: int = SMOOTH_WINDOW,
) -> list[dict[str, Any]]:
    """Resample to ``playback_dt``, smooth pos, recompute vel from gradient."""
    frames = [s for s in samples if isinstance(s, dict) and "pos" in s and "t" in s]
    if playback_dt <= 0:
        raise ValueError("playback_dt 必须大于 0")
    if len(frames) < 2:
        return list(frames)

    kept: list[dict[str, Any]] = []
    last_t = None
    for sample in frames:
        ts = float(sample["t"])
        if last_t is not None and ts - last_t <= 1e-6:
            continue
        kept.append(sample)
        last_t = ts
    if len(kept) < 2:
        return kept

    t0 = float(kept[0]["t"])
    t = [float(s["t"]) - t0 for s in kept]
    n_j = max(len(s.get("pos") or []) for s in kept)
    cols = [
        [float((s.get("pos") or [0.0] * n_j)[i]) if i < len(s.get("pos") or []) else 0.0 for s in kept]
        for i in range(n_j)
    ]
    end = t[-1]
    new_t: list[float] = []
    ts = 0.0
    while ts < end + playback_dt * 0.5:
        new_t.append(ts)
        ts += playback_dt
    if not new_t:
        new_t = [0.0]
    col_series = [_interp_series(t, col, new_t) for col in cols]
    rows = [list(row) for row in zip(*col_series)]
    rows = _moving_average(rows, smooth_window)
    col_series = [list(col) for col in zip(*rows)] if rows else col_series
    vel_series = [_gradient(col, new_t) for col in col_series]

    has_g = any(s.get("gpos") is not None for s in kept)
    new_g: list[float] | None = None
    new_gvel: list[float] | None = None
    if has_g:
        g = [float(s["gpos"]) if s.get("gpos") is not None else 0.0 for s in kept]
        new_g = _moving_average_1d(_interp_series(t, g, new_t), smooth_window)
        new_gvel = _gradient(new_g, new_t)

    out: list[dict[str, Any]] = []
    for i, tsi in enumerate(new_t):
        item: dict[str, Any] = {
            "t": float(tsi),
            "pos": [float(col_series[j][i]) for j in range(n_j)],
            "vel": [float(vel_series[j][i]) for j in range(n_j)],
        }
        if new_g is not None and new_gvel is not None:
            item["gpos"] = float(new_g[i] if i < len(new_g) else new_g[-1])
            item["gvel"] = float(new_gvel[i] if i < len(new_gvel) else 0.0)
        out.append(item)
    return out


def sample_at(frames: list[dict[str, Any]], t: float) -> dict[str, Any]:
    if not frames:
        return {"t": 0.0, "pos": [0.0] * 6, "vel": [0.0] * 6}
    if t <= float(frames[0]["t"]):
        return frames[0]
    if t >= float(frames[-1]["t"]):
        return frames[-1]
    lo = 0
    hi = len(frames) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if float(frames[mid]["t"]) <= t:
            lo = mid
        else:
            hi = mid
    a = frames[lo]
    b = frames[hi]
    span = float(b["t"]) - float(a["t"])
    u = 0.0 if span <= 1e-12 else (t - float(a["t"])) / span
    pos_a = list(a.get("pos") or [])
    pos_b = list(b.get("pos") or [])
    n = max(len(pos_a), len(pos_b))
    pos = []
    vel = []
    vel_a = list(a.get("vel") or [])
    vel_b = list(b.get("vel") or [])
    for i in range(n):
        pa = float(pos_a[i]) if i < len(pos_a) else 0.0
        pb = float(pos_b[i]) if i < len(pos_b) else 0.0
        pos.append(pa + (pb - pa) * u)
        va = float(vel_a[i]) if i < len(vel_a) else 0.0
        vb = float(vel_b[i]) if i < len(vel_b) else 0.0
        vel.append(va + (vb - va) * u)
    item: dict[str, Any] = {"t": float(t), "pos": pos, "vel": vel}
    ga = a.get("gpos")
    gb = b.get("gpos")
    if ga is not None or gb is not None:
        ga_f = float(ga or 0.0)
        gb_f = float(gb if gb is not None else ga_f)
        item["gpos"] = ga_f + (gb_f - ga_f) * u
        item["gvel"] = 0.0
    return item


def list_recordings(directory: str | Path | None = None) -> list[dict[str, Any]]:
    root = Path(directory) if directory is not None else recordings_dir()
    if not root.is_dir():
        return []
    out: list[dict[str, Any]] = []
    for path in sorted(root.glob("*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            header, n, dur = _peek_traj(path)
        except Exception:
            continue
        teach = ""
        if header:
            teach = str(header.get("teach") or "")
        out.append(
            {
                "name": path.name,
                "frames": n,
                "s": round(dur, 2),
                "teach": teach,
                "mtime": path.stat().st_mtime,
            }
        )
    return out


def _peek_traj(path: Path) -> tuple[dict[str, Any] | None, int, float]:
    header: dict[str, Any] | None = None
    n = 0
    last_t = 0.0
    with path.open("r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(obj, dict):
                continue
            if is_header(obj):
                if obj.get("kind") == "traj":
                    header = obj
                continue
            n += 1
            try:
                last_t = float(obj.get("t") or last_t)
            except (TypeError, ValueError):
                pass
    return header, n, last_t
