"""FAFU program trees, a small non-executable script language, and local assets.

Scripts are parsed as data, never evaluated as Python. The motion process owns
the runner; neither the editor nor a second SDK process can write the serial port.
"""
from __future__ import annotations

import ast
import copy
import json
import math
import re
import threading
from pathlib import Path
from typing import Any, Callable

from robot_station.traj import recordings_dir, resolve_traj_path

MAX_NODES = 256
MAX_DEPTH = 8
CONDITIONS = ("gripper_open", "gripper_closed", "enabled", "online")
_STORE_LOCK = threading.RLock()


def number(value: Any, low: float, high: float, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label}必须是数值")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label}必须是数值") from None
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"{label}须在 {low:g}–{high:g} 之间")
    return result


def asset_name(value: Any) -> str:
    if not isinstance(value, str):
        raise ValueError("请输入名称")
    value = value.strip()
    if not value or len(value) > 64 or re.search(r'[\\/:*?"<>|\x00-\x1f]', value) or value in {".", ".."}:
        raise ValueError("名称须为 1–64 个字符，不能包含路径或特殊文件字符")
    return value


def joint_values(value: Any) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError("关节位置必须包含六个角度（单位 °）")
    return [number(x, -360, 360, "关节角度") for x in value]


def validate_points(raw: Any) -> list[dict]:
    if not isinstance(raw, list) or len(raw) > 500:
        raise ValueError("最多保存 500 个路点")
    result, names = [], set()
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError("路点格式无效")
        name = asset_name(item.get("name") or f"路点_{index + 1}")
        if name in names:
            raise ValueError(f"路点名称重复：{name}")
        names.add(name)
        result.append({"name": name, "q": joint_values(item.get("q")),
                       "t": number(item.get("t", 2), 0.2, 600, "路点时长")})
    return result


def validate_tree(raw: Any) -> list[dict]:
    count = 0
    ids: set[str] = set()

    def visit(items: Any, depth: int) -> list[dict]:
        nonlocal count
        if not isinstance(items, list) or depth > MAX_DEPTH:
            raise ValueError("程序结构无效或嵌套超过 8 层")
        result = []
        for item in items:
            count += 1
            if count > MAX_NODES or not isinstance(item, dict):
                raise ValueError("程序最多包含 256 个节点")
            kind = item.get("type")
            ident = str(item.get("id") or f"n{count}")
            if not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", ident) or ident in ids:
                raise ValueError("程序节点 ID 无效或重复")
            ids.add(ident)
            node = {"id": ident, "type": kind}
            if kind == "movej":
                if item.get("point"):
                    node["point"] = asset_name(item["point"])
                else:
                    node["q"] = joint_values(item.get("q"))
                node["speed"] = number(item.get("speed", 20), 5, 80, "速度 °/s")
            elif kind == "gripper":
                if not isinstance(item.get("open"), bool):
                    raise ValueError("请选择夹爪打开或闭合")
                node["open"] = item["open"]
                node["effort"] = int(number(item.get("effort", 300), 50, 800, "夹爪力矩 raw"))
            elif kind == "wait":
                node["seconds"] = number(item.get("seconds", 0.5), 0, 3600, "等待秒数")
            elif kind == "replay":
                name = str(item.get("file") or "")
                if not name.endswith(".jsonl") or "/" in name or "\\" in name:
                    raise ValueError("请选择本机示教轨迹文件")
                node.update(file=name, rate=number(item.get("rate", 1), 0.5, 1, "回放倍率"),
                            speed=number(item.get("speed", 20), 5, 80, "接近速度 °/s"))
            elif kind == "loop":
                times = number(item.get("count", 2), 1, 1000, "重复次数")
                if times != int(times):
                    raise ValueError("重复次数必须是整数")
                node.update(count=int(times), children=visit(item.get("children", []), depth + 1))
            elif kind == "if":
                condition = item.get("condition", "gripper_open")
                if condition not in CONDITIONS:
                    raise ValueError("不支持此判断条件")
                node.update(condition=condition, children=visit(item.get("children", []), depth + 1),
                            otherwise=visit(item.get("otherwise", []), depth + 1))
            elif kind == "comment":
                node["text"] = str(item.get("text", "注释"))[:500]
            else:
                raise ValueError(f"不支持的程序节点：{kind}")
            result.append(node)
        return result

    return visit(raw, 0)


def parse_script(source: str) -> list[dict]:
    if not isinstance(source, str) or len(source) > 100_000:
        raise ValueError("脚本超过 100 KB")
    try:
        module = ast.parse(source)
    except SyntaxError as exc:
        raise ValueError(f"第 {exc.lineno} 行：{exc.msg}") from None

    def literal(node: ast.AST) -> Any:
        try:
            return ast.literal_eval(node)
        except (ValueError, TypeError, SyntaxError, RecursionError):
            raise ValueError("参数只允许数值、字符串或关节角度列表") from None

    def block(items: list[ast.stmt], depth: int = 0) -> list[dict]:
        if depth > MAX_DEPTH:
            raise ValueError("程序嵌套超过 8 层")
        result = []
        for statement in items:
            try:
                if isinstance(statement, ast.Pass):
                    continue
                if isinstance(statement, ast.For):
                    call = statement.iter
                    if (not isinstance(statement.target, ast.Name) or not isinstance(call, ast.Call)
                            or not isinstance(call.func, ast.Name) or call.func.id != "range"
                            or len(call.args) != 1 or call.keywords or statement.orelse):
                        raise ValueError("循环格式：for _ in range(次数):")
                    node = {"type": "loop", "count": literal(call.args[0]),
                            "children": block(statement.body, depth + 1)}
                elif isinstance(statement, ast.If):
                    condition = statement.test
                    conditions = {"gripper_is_open": "gripper_open", "gripper_is_closed": "gripper_closed",
                                  "robot_is_enabled": "enabled", "robot_is_connected": "online"}
                    if (not isinstance(condition, ast.Call) or not isinstance(condition.func, ast.Name)
                            or condition.func.id not in conditions or condition.args or condition.keywords):
                        raise ValueError("判断支持 gripper_is_open/closed、robot_is_enabled/connected()")
                    node = {"type": "if", "condition": conditions[condition.func.id],
                            "children": block(statement.body, depth + 1),
                            "otherwise": block(statement.orelse, depth + 1)}
                elif isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Call):
                    call = statement.value
                    if not isinstance(call.func, ast.Name) or any(k.arg is None for k in call.keywords):
                        raise ValueError("只允许调用 FAFU 程序指令")
                    name = call.func.id
                    args = [literal(x) for x in call.args]
                    kw = {k.arg: literal(k.value) for k in call.keywords}
                    if len(kw) != len(call.keywords):
                        raise ValueError("参数重复")
                    if name == "movej" and len(args) == 1 and set(kw) <= {"speed"}:
                        node = {"type": "movej", "point" if isinstance(args[0], str) else "q": args[0], **kw}
                    elif name in ("gripper_open", "gripper_close") and not args and set(kw) <= {"effort"}:
                        node = {"type": "gripper", "open": name == "gripper_open", **kw}
                    elif name in ("sleep", "wait") and len(args) == 1 and not kw:
                        node = {"type": "wait", "seconds": args[0]}
                    elif name == "replay" and len(args) == 1 and set(kw) <= {"rate", "speed"}:
                        node = {"type": "replay", "file": args[0], **kw}
                    elif name == "comment" and len(args) == 1 and isinstance(args[0], str) and not kw:
                        node = {"type": "comment", "text": args[0]}
                    else:
                        raise ValueError(f"未知指令或参数不匹配：{name}")
                else:
                    raise ValueError("仅支持运动、夹爪、等待、回放、for 循环和 if 判断")
                result.append(node)
            except ValueError as exc:
                raise ValueError(f"第 {statement.lineno} 行：{exc}") from None
        return result

    return validate_tree(block(module.body))


def format_script(nodes: list[dict], depth: int = 0) -> str:
    lines = []
    quote = lambda x: json.dumps(x, ensure_ascii=False)
    for node in nodes:
        kind = node["type"]
        if kind == "movej":
            text = f"movej({quote(node.get('point', node.get('q')))}, speed={node['speed']:g})"
        elif kind == "gripper":
            text = f"gripper_{'open' if node['open'] else 'close'}(effort={node['effort']})"
        elif kind == "wait":
            text = f"sleep({node['seconds']:g})"
        elif kind == "replay":
            text = f"replay({quote(node['file'])}, rate={node['rate']:g}, speed={node['speed']:g})"
        elif kind == "comment":
            text = f"comment({quote(node['text'])})"
        else:
            condition = {"gripper_open": "gripper_is_open", "gripper_closed": "gripper_is_closed",
                         "enabled": "robot_is_enabled", "online": "robot_is_connected"}
            text = (f"for _ in range({node['count']}):" if kind == "loop"
                    else f"if {condition[node['condition']]}():")
            lines.append("    " * depth + text)
            lines.append(format_script(node["children"], depth + 1) or "    " * (depth + 1) + "pass")
            if kind == "if" and node["otherwise"]:
                lines.append("    " * depth + "else:")
                lines.append(format_script(node["otherwise"], depth + 1))
            continue
        lines.append("    " * depth + text)
    return "\n".join(lines)


class WorkbenchStore:
    def __init__(self, root: Path | None = None) -> None:
        self.root = Path(root) if root is not None else recordings_dir() / "workspace"

    def _read(self, path: Path, default: Any) -> Any:
        if not path.exists():
            return copy.deepcopy(default)
        return json.loads(path.read_text(encoding="utf-8"))

    def _write(self, path: Path, data: Any) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)

    def points(self) -> list[dict]:
        with _STORE_LOCK:
            return validate_points(self._read(self.root / "waypoints.json", []))

    def save_points(self, points: Any) -> list[dict]:
        result = validate_points(points)
        with _STORE_LOCK:
            self._write(self.root / "waypoints.json", result)
        return result

    def catalog(self) -> dict:
        with _STORE_LOCK:
            directory = self.root / "programs"
            files = sorted(directory.glob("*.json")) if directory.exists() else []
            return {"waypoints": self.points(), "programs": [p.stem for p in files]}

    def load(self, name: str) -> dict:
        with _STORE_LOCK:
            path = self.root / "programs" / (asset_name(name) + ".json")
            if not path.is_file():
                raise ValueError("程序不存在")
            data = self._read(path, {})
            nodes = validate_tree(data.get("nodes"))
            return {"name": path.stem, "nodes": nodes, "script": format_script(nodes)}

    def save(self, name: str, nodes: Any) -> dict:
        name, nodes = asset_name(name), validate_tree(nodes)
        with _STORE_LOCK:
            self._write(self.root / "programs" / (name + ".json"), {"version": 1, "nodes": nodes})
        return {"name": name, "nodes": nodes, "script": format_script(nodes)}

    def delete(self, name: str) -> None:
        with _STORE_LOCK:
            (self.root / "programs" / (asset_name(name) + ".json")).unlink()


class ProgramRunner:
    """Cooperatively advance one program on the existing motion tick."""
    def __init__(self) -> None:
        self.status = "idle"
        self.name = ""
        self.current: dict | None = None
        self.error = ""
        self.completed = 0
        self.stack: list[dict] = []
        self.pending = False
        self.started_at = 0.0
        self.deadline = 0.0
        self.stable_at: float | None = None
        self.single = False
        self.dispatched = False

    @property
    def active(self) -> bool:
        return self.status in ("running", "paused")

    def snapshot(self) -> dict:
        return {"status": self.status, "name": self.name, "current_id": (self.current or {}).get("id", ""),
                "current_type": (self.current or {}).get("type", ""), "completed": self.completed,
                "error": self.error}

    def start(self, name: str, nodes: list[dict], points: list[dict], limits: list | None,
              *, single: bool = False) -> None:
        if self.active:
            raise ValueError("请先停止当前程序")
        nodes = copy.deepcopy(validate_tree(nodes))
        lookup = {p["name"]: p["q"] for p in validate_points(points)}
        budget = 0

        def resolve(items: list[dict], multiplier: int = 1) -> None:
            nonlocal budget
            for node in items:
                budget += multiplier
                if budget > 10000:
                    raise ValueError("程序展开后超过 10000 步，请减少循环次数")
                if node["type"] == "movej":
                    if "point" in node:
                        if node["point"] not in lookup:
                            raise ValueError(f"找不到路点：{node['point']}")
                        node["q"] = list(lookup[node["point"]])
                    if limits and any(not limits[i][0] <= q <= limits[i][1] for i, q in enumerate(node["q"])):
                        raise ValueError(f"路点 {node.get('point', '')} 超出关节限位")
                elif node["type"] == "replay":
                    resolve_traj_path(node["file"])
                elif node["type"] == "loop":
                    resolve(node["children"], multiplier * node["count"])
                elif node["type"] == "if":
                    resolve(node["children"], multiplier)
                    resolve(node["otherwise"], multiplier)

        resolve(nodes)
        self.__init__()
        self.name, self.single, self.status = name, single, "running"
        self.stack = [{"nodes": nodes, "index": 0, "remaining": 1}]

    def stop(self, reason: str = "") -> None:
        self.status = "error" if reason else "stopped"
        self.error = reason
        self.stack = []
        self.pending = False

    def pause(self) -> None:
        if self.status == "running":
            self.status = "paused"
            # Resume re-enters the current motion from the measured pose.
            self.dispatched = False

    def resume(self, single: bool = False) -> None:
        if self.status != "paused":
            raise ValueError("没有暂停的程序")
        self.status, self.single = "running", single

    def tick(self, now: float, arm: Any, dispatch: Callable[[dict], str | None], scale: float = 1) -> None:
        if self.status != "running":
            return
        try:
            if self.pending:
                node = self.current
                assert node is not None
                if not self.dispatched:
                    error = dispatch(node)
                    if error:
                        raise ValueError(error)
                    self.dispatched = True
                    self.started_at, self.stable_at = now, None
                    timeout = (max(60, 360 / max(1, node.get("speed", 20) * scale) + 30)
                               if node["type"] == "movej" else node.get("seconds", 3600) + 10)
                    self.deadline = now + timeout
                elapsed = now - self.started_at
                kind = node["type"]
                if kind == "wait":
                    done = elapsed >= node["seconds"]
                elif kind == "movej":
                    feedback = getattr(arm, "feedback_deg", None) or arm.q_deg
                    done = (not arm.moving and len(feedback) == 6 and
                            max(abs(a - b) for a, b in zip(feedback, node["q"])) <= 0.75)
                elif kind == "replay":
                    done = elapsed >= 0.05 and not arm.replay_active and not arm.moving
                else:
                    limits = getattr(arm, "gripper_limits", None)
                    if limits and len(limits) == 2 and getattr(arm, "gripper_moving", None) is not None:
                        goal = limits[1] if node["open"] else limits[0]
                        at_goal = abs(float(arm.gripper_deg) - goal) <= 2
                        torque = getattr(arm, "gripper_tau_raw", None)
                        contact = not node["open"] and torque is not None and abs(torque) >= node["effort"] * 0.8
                        done = elapsed >= 0.2 and not arm.gripper_moving and (at_goal or contact)
                    else:
                        done = bool(arm.gripper_open) == node["open"]
                    if elapsed > 15:
                        raise ValueError("夹爪动作超时")
                if not done:
                    self.stable_at = None
                elif self.stable_at is None:
                    self.stable_at = now
                if now > self.deadline:
                    raise ValueError("动作超时，程序已停止")
                if not done or now - self.stable_at < 0.05:
                    return
                self.pending, self.dispatched = False, False
                self.completed += 1
                if self.single:
                    remaining = any(f["index"] < len(f["nodes"]) or f["remaining"] > 1 for f in self.stack)
                    self.status = "paused" if remaining else "completed"
                    return
            # Limit structural work per tick, even for empty nested loops.
            for _ in range(32):
                if not self.stack:
                    self.status = "completed"
                    return
                frame = self.stack[-1]
                if frame["index"] >= len(frame["nodes"]):
                    frame["remaining"] -= 1
                    if frame["remaining"] <= 0:
                        self.stack.pop()
                    else:
                        frame["index"] = 0
                    continue
                node = frame["nodes"][frame["index"]]
                frame["index"] += 1
                self.current = node
                kind = node["type"]
                if kind in ("loop", "if"):
                    if kind == "loop":
                        children, count = node["children"], node["count"]
                    else:
                        yes = {"gripper_open": arm.gripper_open, "gripper_closed": not arm.gripper_open,
                               "online": arm.online, "enabled": arm.enabled}[node["condition"]]
                        children, count = node["children"] if yes else node["otherwise"], 1
                    self.stack.append({"nodes": children, "index": 0, "remaining": count})
                elif kind == "comment":
                    continue
                else:
                    self.pending, self.dispatched = True, False
                    return
        except Exception as exc:
            self.stop(str(exc))
