from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "configs" / "station.example.yaml"
DEFAULT_PATH = ROOT / "configs" / "station.yaml"


def _as_bool(value: Any, default: bool = False) -> bool:
    """YAML 里写成 ``"false"`` 时不能当 Python 真值。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        s = value.strip().lower()
        if s in ("1", "true", "yes", "on"):
            return True
        if s in ("0", "false", "no", "off", ""):
            return False
    return bool(default)


@dataclass
class StationConfig:
    robot_id: str = "FAFU-STATION-001"
    listen_host: str = "127.0.0.1"
    http_port: int = 9400
    control_hz: int = 100
    telemetry_hz: int = 100
    video_hz: int = 30
    watchdog_s: float = 0.20
    arm: str = "mock"  # mock | sim | fafu | auto
    camera: str = "mock"  # mock | auto（交付 yaml 用 auto：自动认 RealSense，跳过内置摄像头）
    motion_host: str = "127.0.0.1"
    motion_port: int = 9470
    open_browser: bool = True
    camera_count: int = 1
    camera_width: int = 320
    camera_height: int = 180
    arm_joints: int = 6
    arm_speed_deg_s: float = 40.0
    arm_has_gripper: bool = True
    arm_sdk: str = ""
    arm_cfg: str = ""
    arm_port: str = ""
    arm_allow_motion: bool = False
    arm_gripper_id: int = 7

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> "StationConfig":
        allowed = {f.name for f in fields(cls)}
        data = {k: v for k, v in raw.items() if k in allowed}
        if "arm_allow_motion" in data:
            data["arm_allow_motion"] = _as_bool(data["arm_allow_motion"], False)
        cfg = cls(**data)
        cfg.camera_count = 1  # 本机笔记本只接一路作业相机
        return cfg


def load_config(path: Path | None = None) -> StationConfig:
    target = path or DEFAULT_PATH
    if not target.is_file():
        if not EXAMPLE.is_file():
            return StationConfig()
        target = EXAMPLE
    with target.open(encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"配置不是键值表: {target}")
    return StationConfig.from_mapping(raw)
