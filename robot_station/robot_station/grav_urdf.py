"""Station-local URDF folder for gravity compensation.

Put 6-DoF ``.urdf`` files in ``<station>/urdf/`` (or ``STATION_URDF_DIR``).
Gravity / Gra+Fri / Impedance and drag-teach use the file selected on the page.
"""
from __future__ import annotations

import os
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from robot_station.config import ROOT

_SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]+\.urdf$", re.IGNORECASE)
_LINK_INERTIAL = re.compile(
    r'<link name="link(\d)">\s*<inertial>\s*<origin xyz="([^"]+)"[^/]*/>\s*<mass value="([^"]+)"',
    flags=re.S,
)
_JOINT_BLOCK = re.compile(
    r'<joint name="joint(\d)"[^>]*>\s*<origin xyz="([^"]+)"[^/]*/>.*?<axis xyz="([^"]+)"',
    flags=re.S,
)
PREFERRED = (
    "fafu_baseV1.urdf",
    "fafu_baseV1_teach.urdf",
    "fafu_baseV1_d405.urdf",
    "fafu_baseV2.urdf",
)
_cache: dict[str, object] = {"key": None, "names": []}


@dataclass(frozen=True)
class UrdfDynamics:
    name: str
    path: Path
    masses: tuple[float, ...]
    coms: tuple[tuple[float, float, float], ...]
    joints: tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...]


def urdf_dir() -> Path:
    override = (os.environ.get("STATION_URDF_DIR") or "").strip()
    if override:
        return Path(override)
    return ROOT / "urdf"


def _sdk_urdf_dirs() -> list[Path]:
    raw = (os.environ.get("FAFU_ARM_SDK") or "").strip()
    home = Path.home()
    roots = [
        Path(raw) if raw else None,
        ROOT / "vendor" / "fafu_arm_sdk",
        ROOT.parent / "fafu_arm_sdk",
        home / "fafu_arm_sdk",
        Path(r"D:\fafu_arm_sdk"),
    ]
    out: list[Path] = []
    for root in roots:
        if root is None:
            continue
        folder = root / "fafu_robot_python" / "fafu_robot_description" / "urdf"
        if folder.is_dir():
            out.append(folder)
    return out


def seed_urdf_dir(directory: Path | None = None) -> Path:
    dest = directory if directory is not None else urdf_dir()
    dest.mkdir(parents=True, exist_ok=True)
    have = {p.name.lower() for p in dest.glob("*.urdf") if p.is_file()}
    if have:
        return dest
    for folder in _sdk_urdf_dirs():
        for name in PREFERRED:
            src = folder / name
            if src.is_file() and name.lower() not in have:
                shutil.copy2(src, dest / name)
                have.add(name.lower())
        if have:
            break
    return dest


def list_gravity_urdfs(directory: str | Path | None = None) -> list[str]:
    root = Path(directory) if directory is not None else urdf_dir()
    seed_urdf_dir(root)
    try:
        key = (str(root.resolve()), root.stat().st_mtime)
    except OSError:
        return []
    if _cache.get("key") == key:
        return list(_cache["names"])  # type: ignore[arg-type]
    names = sorted(p.name for p in root.glob("*.urdf") if p.is_file() and _SAFE_NAME.match(p.name))
    _cache["key"] = key
    _cache["names"] = names
    return list(names)


def resolve_gravity_urdf(name: str, directory: str | Path | None = None) -> Path:
    raw = Path((name or "").strip()).name
    if not _SAFE_NAME.match(raw):
        raise FileNotFoundError("URDF 文件名不合法")
    root = (Path(directory) if directory is not None else urdf_dir()).resolve()
    seed_urdf_dir(root)
    path = (root / raw).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise FileNotFoundError("URDF 不在站控 urdf 目录内") from exc
    if not path.is_file():
        raise FileNotFoundError(f"找不到 URDF：{raw}")
    return path


def selected_urdf_name(directory: str | Path | None = None) -> str:
    root = Path(directory) if directory is not None else urdf_dir()
    names = list_gravity_urdfs(root)
    if not names:
        return ""
    marker = root / ".selected"
    if marker.is_file():
        try:
            choice = Path(marker.read_text(encoding="utf-8").strip()).name
        except OSError:
            choice = ""
        if choice in names:
            return choice
    for pref in PREFERRED:
        if pref in names:
            return pref
    return names[0]


def persist_selected_urdf(name: str, directory: str | Path | None = None) -> None:
    root = Path(directory) if directory is not None else urdf_dir()
    seed_urdf_dir(root)
    (root / ".selected").write_text(Path(name).name + "\n", encoding="utf-8")


def _vec3(text: str) -> tuple[float, float, float]:
    parts = [float(x) for x in text.split()]
    if len(parts) != 3:
        raise ValueError(f"expected 3 numbers, got {text!r}")
    return parts[0], parts[1], parts[2]


def parse_urdf_dynamics(text: str) -> tuple[
    tuple[float, ...],
    tuple[tuple[float, float, float], ...],
    tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...],
]:
    inert: dict[int, tuple[float, tuple[float, float, float]]] = {}
    for idx, xyz, mass in _LINK_INERTIAL.findall(text):
        inert[int(idx)] = (float(mass), _vec3(xyz))
    joints: dict[int, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}
    for idx, xyz, axis in _JOINT_BLOCK.findall(text):
        joints[int(idx)] = (_vec3(xyz), _vec3(axis))
    missing_m = [i for i in range(1, 7) if i not in inert]
    missing_j = [i for i in range(1, 7) if i not in joints]
    if missing_m:
        raise ValueError(f"URDF 缺少 link{missing_m} 的质量/质心")
    if missing_j:
        raise ValueError(f"URDF 缺少 joint{missing_j} 的原点和轴")
    masses = tuple(inert[i][0] for i in range(1, 7))
    coms = tuple(inert[i][1] for i in range(1, 7))
    chain = tuple(joints[i] for i in range(1, 7))
    return masses, coms, chain


def load_urdf_dynamics(path: str | Path) -> UrdfDynamics:
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    masses, coms, joints = parse_urdf_dynamics(text)
    return UrdfDynamics(name=target.name, path=target, masses=masses, coms=coms, joints=joints)


def load_selected_dynamics(name: str | None = None, directory: str | Path | None = None) -> UrdfDynamics | None:
    choice = (name or "").strip() or selected_urdf_name(directory)
    if not choice:
        return None
    return load_urdf_dynamics(resolve_gravity_urdf(choice, directory))


def table_or_default(
    dyn: UrdfDynamics | None,
    default_masses: Sequence[float],
    default_coms: Sequence[Sequence[float]],
    default_joints: Sequence[tuple[Sequence[float], Sequence[float]]],
) -> tuple[
    tuple[float, ...],
    tuple[tuple[float, float, float], ...],
    tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...],
]:
    if dyn is None:
        masses = tuple(float(x) for x in default_masses)
        coms = tuple((float(c[0]), float(c[1]), float(c[2])) for c in default_coms)
        joints = tuple(
            ((float(o[0]), float(o[1]), float(o[2])), (float(a[0]), float(a[1]), float(a[2])))
            for o, a in default_joints
        )
        return masses, coms, joints
    return dyn.masses, dyn.coms, dyn.joints
