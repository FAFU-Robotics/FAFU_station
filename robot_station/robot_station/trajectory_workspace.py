"""Non-destructive trajectory editing. Operations are replayed against the source.

The browser receives a bounded preview; saves always retain every edited sample.
No editing operation commands the robot.
"""
from __future__ import annotations

import math
import re
import uuid
from typing import Any

from robot_station.traj import (
    PLAYBACK_DT, TrajectoryWriter, _gradient, _moving_average, load_traj,
    make_traj_path, prepare_playback, recordings_dir, resolve_traj_path, sample_at,
)

MAX_FRAMES = 360000
MAX_OPERATIONS = 200
EDITOR_VERSION = 3


def snapshot_path(key: str):
    if not isinstance(key, str) or not re.fullmatch(r"[0-9a-f]{32}", key):
        raise ValueError("编辑历史快照标识无效")
    root = recordings_dir().resolve()
    path = (root / "workspace" / "trajectory_history" / (key + ".jsonl")).resolve()
    if not path.is_relative_to(root):
        raise ValueError("编辑历史快照不在录制目录内")
    return path


def store_snapshot(frames: list[dict]) -> str:
    key = uuid.uuid4().hex
    writer = TrajectoryWriter(snapshot_path(key), teach="soft", mode="Position", n=6, prefix=frames)
    writer.close()
    return key


def read_snapshot(key: str) -> list[dict]:
    _, raw = load_traj(snapshot_path(key))
    return validate_frames(raw)


def finite(value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("轨迹参数必须是有限数值")
    return result


def validate_frames(frames: list[dict]) -> list[dict]:
    if not 2 <= len(frames) <= MAX_FRAMES:
        raise ValueError("轨迹需要 2–360000 帧")
    out = []
    previous = -1.0
    n = len(frames[0].get("pos") or [])
    if n != 6:
        raise ValueError("轨迹必须包含 6 个关节")
    origin = finite(frames[0]["t"])
    for item in frames:
        t = finite(item["t"]) - origin
        pos = [finite(x) for x in item["pos"]]
        if len(pos) != n or t < previous:
            raise ValueError("轨迹时间戳必须递增，关节数必须一致")
        row = {"t": t, "pos": pos, "vel": [0.0] * n}
        row["gpos"] = None if item.get("gpos") is None else finite(item["gpos"])
        row["gvel"] = 0.0
        # Older Windows recordings used a coarse monotonic clock. Keep the
        # last measurement for equal timestamps, as the player does.
        if t == previous:
            out[-1] = row
        else:
            out.append(row)
        previous = t
    if len(out) < 2:
        raise ValueError("轨迹需要至少两个不同时间的采样点")
    return out


def velocities(frames: list[dict]) -> list[dict]:
    times = [x["t"] for x in frames]
    cols = [_gradient([x["pos"][j] for x in frames], times) for j in range(6)]
    g = _gradient([x.get("gpos") or 0.0 for x in frames], times)
    for i, frame in enumerate(frames):
        frame["vel"] = [col[i] for col in cols]
        frame["gvel"] = g[i]
    return frames


def join_frames(left: list[dict], right: list[dict], seconds: float) -> list[dict]:
    """A minimum-jerk bridge has zero first/second derivatives at its ends."""
    seconds = finite(seconds)
    if not 0.2 <= seconds <= 30:
        raise ValueError("过渡时长应在 0.2–30 秒之间")
    a, b = left[-1], right[0]
    steps = max(2, math.ceil(seconds / PLAYBACK_DT))
    if len(left) + len(right) + steps > MAX_FRAMES:
        raise ValueError("合并后的轨迹超过帧数上限")
    bridge = []
    for i in range(1, steps):
        u = i / steps
        blend = 10 * u**3 - 15 * u**4 + 6 * u**5
        g0, g1 = a.get("gpos"), b.get("gpos")
        bridge.append({"t": a["t"] + u * seconds,
                       "pos": [x + (y-x)*blend for x, y in zip(a["pos"], b["pos"])],
                       "gpos": g0 + (g1-g0)*blend if g0 is not None and g1 is not None else g0})
    offset = a["t"] + seconds - b["t"]
    return left + bridge + [{**row, "t": row["t"] + offset} for row in right]


RANGE_TOOLS = {"smooth", "idle", "speed", "reverse", "resample"}


def selection_bounds(frames: list[dict], op: dict) -> tuple[float, float]:
    duration = frames[-1]["t"]
    start = finite(op.get("start", 0))
    end = finite(op.get("end", duration))
    if not 0 <= start < end <= duration + 1e-6:
        raise ValueError("请选择有效的起止时间")
    return start, min(end, duration)


def clip_frames(frames: list[dict], start: float, end: float) -> list[dict]:
    rows = [sample_at(frames, start)] + [x for x in frames if start < x["t"] < end] + [sample_at(frames, end)]
    return validate_frames(rows)


def edit_selection(frames: list[dict], op: dict) -> list[dict]:
    """Edit only the selected clip; ripple later times when its length changes.

    Outside joint/gripper positions are retained. Reversing a middle clip
    adds explicit transitions rather than jumping between swapped endpoints.
    """
    start, end = selection_bounds(frames, op)
    clip = clip_frames(frames, start, end)
    local = {key: value for key, value in op.items() if key not in ("scope", "start", "end")}
    edited = edit_frames(clip, local)
    prefix = [x for x in frames if x["t"] < start]
    suffix = [x for x in frames if x["t"] > end]
    if op["type"] == "reverse" and prefix:
        boundary = {**sample_at(frames, start), "t": start}
        result = join_frames(prefix + [boundary], edited, op.get("transition", 1))
    else:
        result = prefix + [{**x, "t": x["t"] + start} for x in edited]
    if op["type"] == "reverse" and suffix:
        boundary = {**sample_at(frames, end), "t": end}
        result = join_frames(result, [boundary] + suffix, op.get("transition", 1))
    else:
        delta = result[-1]["t"] - end
        result += [{**x, "t": x["t"] + delta} for x in suffix]
    return validate_frames(result)


def edit_frames(frames: list[dict], op: dict) -> list[dict]:
    kind = op.get("type")
    if kind == "select":
        selection_bounds(frames, op)
        return frames
    scope = op.get("scope", "all")  # Old saved histories retain their original global behavior.
    if scope not in ("all", "selection"):
        raise ValueError("处理范围必须是选区或整条轨迹")
    if scope == "selection" and kind in RANGE_TOOLS:
        return edit_selection(frames, op)
    if scope == "selection" and kind == "merge":
        raise ValueError("合并将追加到整条轨迹末尾，请使用整条轨迹范围")
    duration = frames[-1]["t"]
    if kind in ("trim", "delete"):
        start, end = selection_bounds(frames, op)
        if kind == "trim":
            frames = clip_frames(frames, start, end)
        else:
            left = [x for x in frames if x["t"] < start]
            right = [x for x in frames if x["t"] > end]
            if left and right:
                frames = join_frames(left, right, op.get("transition", 1))
            else:
                frames = left or right
    elif kind == "smooth":
        window = int(op.get("window", 9))
        if not 3 <= window <= 101:
            raise ValueError("平滑窗口应在 3–101 帧之间")
        # Preserve endpoints, including the gripper. Convex averaging cannot
        # introduce a joint position outside the source position range.
        rows = _moving_average([x["pos"] for x in frames], window)
        for i in range(1, len(frames)-1):
            frames[i] = {**frames[i], "pos": rows[i]}
    elif kind == "idle":
        threshold = math.radians(finite(op.get("threshold", 0.5)))
        minimum = finite(op.get("minimum", 0.5))
        if not 0 < threshold <= math.radians(10) or not 0.1 <= minimum <= 10:
            raise ValueError("静止阈值应在 0–10°，最短静止时长应在 0.1–10 秒")
        removed = 0.0
        result = [dict(frames[0])]
        i = 1
        while i < len(frames):
            anchor = frames[i-1]
            j = i
            while j < len(frames):
                row = frames[j]
                # A moving gripper is an action, never a stationary interval.
                if max(abs(a-b) for a, b in zip(anchor["pos"], row["pos"])) > threshold:
                    break
                ga, gb = anchor.get("gpos"), row.get("gpos")
                if (ga is None) != (gb is None) or (ga is not None and abs(ga-gb) > threshold):
                    break
                j += 1
            if j > i and frames[j-1]["t"] - anchor["t"] >= minimum:
                tail = frames[j-1]
                removed += max(0.0, tail["t"] - anchor["t"] - PLAYBACK_DT)
                result.append({**tail, "t": tail["t"] - removed})
                i = j
            else:
                result.append({**frames[i], "t": frames[i]["t"] - removed})
                i += 1
        frames = result
    elif kind == "speed":
        factor = finite(op.get("factor", 1))
        if not 0.1 <= factor <= 2:
            raise ValueError("编辑倍率应在 0.1–2 之间")
        frames = [{**x, "t": x["t"] / factor} for x in frames]
    elif kind == "reverse":
        frames = [{**x, "t": duration-x["t"]} for x in reversed(frames)]
    elif kind == "resample":
        hz = finite(op.get("hz", 100))
        if not 10 <= hz <= 100 or duration * hz > MAX_FRAMES:
            raise ValueError("重采样频率应在 10–100 Hz，且不超过帧数上限")
        # Keep the endpoint exactly; prepare_playback rounds the time grid.
        result = prepare_playback(frames, 1/hz, 1)
        result = [x for x in result if x["t"] < duration - 1e-6]
        frames = result + [dict(frames[-1])]
    elif kind == "merge":
        if op.get("merge_source"):
            other = read_snapshot(op["merge_source"])
        else:
            _, other = load_traj(resolve_traj_path(str(op.get("file") or "")))
            other = validate_frames(other)
        frames = join_frames(frames, other, op.get("transition", 1))
    else:
        raise ValueError("未知轨迹编辑操作")
    return validate_frames(frames)


def process_trajectory(name: str, operations: list[dict] | None = None, *, save: bool = False, dest: str = "",
                       future: list[dict] | None = None) -> dict:
    header, raw = load_traj(resolve_traj_path(name))
    frames = validate_frames(raw)
    history = (header or {}).get("edit_history")
    source = ""
    warning = ""
    if isinstance(history, dict):
        try:
            frames = read_snapshot(history.get("source"))
            source = history["source"]
        except FileNotFoundError:
            # Exported JSONL is a complete playable trajectory. Its history
            # archive may be absent on a different PC; keep the file usable.
            warning = "历史源快照缺失；当前轨迹可正常使用，此前的编辑历史无法恢复。"
            history = None
    if operations is None:
        timeline = history.get("operations", []) if history else []
        cursor = history.get("cursor", len(timeline)) if history else 0
        if not isinstance(timeline, list) or not isinstance(cursor, int) or not 0 <= cursor <= len(timeline):
            raise ValueError("编辑历史损坏")
        operations = timeline[:cursor]
        future = timeline[cursor:]
    future = future or []
    if not isinstance(operations, list) or not isinstance(future, list) or len(operations) + len(future) > MAX_OPERATIONS:
        raise ValueError(f"每份轨迹最多支持 {MAX_OPERATIONS} 步编辑历史")
    operations = [dict(op) if isinstance(op, dict) else op for op in operations]
    future = [dict(op) if isinstance(op, dict) else op for op in future]
    for op in operations + future:
        if not isinstance(op, dict):
            raise ValueError("轨迹编辑步骤必须是有效的操作对象")
        if save and op.get("type") == "merge" and not op.get("merge_source"):
            _, other = load_traj(resolve_traj_path(str(op.get("file") or "")))
            op["merge_source"] = store_snapshot(validate_frames(other))
    if save and not source:
        source = store_snapshot(frames)
    selection = {"start": 0.0, "end": frames[-1]["t"]}
    for op in operations:
        before = frames[-1]["t"]
        frames = edit_frames(frames, op)
        after = frames[-1]["t"]
        if op["type"] == "select":
            start, end = selection_bounds(frames, op)
            selection = {"start": start, "end": end}
        elif op.get("scope") == "selection" and op["type"] in RANGE_TOOLS:
            selection = {"start": float(op.get("start", 0)),
                         "end": min(after, float(op.get("end", before)) + after - before)}
        elif op["type"] == "delete":
            start = min(float(op.get("start", 0)), max(0.0, after - PLAYBACK_DT))
            selection = {"start": start, "end": min(after, start + float(op.get("transition", 1)))}
        else:
            selection = {"start": 0.0, "end": after}
    velocities(frames)
    saved = ""
    if save:
        path = make_traj_path(dest or name.removesuffix(".jsonl") + "_编辑")
        saved_history = {"v": 1, "source": source, "operations": operations + future, "cursor": len(operations)}
        writer = TrajectoryWriter(path, teach=(header or {}).get("teach", "soft"), mode="Position", n=6,
                                  prefix=frames, edit_history=saved_history)
        writer.close()
        saved = path.name
    count, duration = len(frames), frames[-1]["t"]
    stride = max(1, math.ceil((count-1)/2999))
    preview = frames[::stride]
    if preview[-1] is not frames[-1]:
        preview.append(frames[-1])
    return {"name": saved or name, "frames": preview, "count": count, "duration": duration,
            "editor_version": EDITOR_VERSION,
            "endpoint_hold": True,
            "operations": operations, "future": future, "history_warning": warning, "selection": selection,
            "teach": (header or {}).get("teach", ""), "preview_stride": stride,
            "max_speed_deg_s": max(abs(math.degrees(v)) for x in frames for v in x["vel"])}
