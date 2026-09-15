"""Downsample SDK SolidWorks STL into compact triangle bins for the web 3D view.

Stride-sampling punched holes in large faces. Keep the biggest triangles
(after a light vertex weld) so the silhouette stays closed.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAMES = ("base_link", "link1", "link2", "link3", "link4", "link5", "link6")
MAGIC = b"FARM"
VERSION = 1
DEFAULT_MAX_TRIS = 16000
WELD_CELL_M = 0.0006
_LINK_RE = re.compile(
    r'<link name="([^"]+)">.*?<color rgba="([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)"',
    re.DOTALL,
)


def _urdf_path(mesh_dir: Path) -> Path | None:
    for rel in (
        Path("..") / "urdf" / "fafu_baseV1.urdf",
        Path("..") / ".." / "urdf" / "fafu_baseV1.urdf",
    ):
        cand = (mesh_dir / rel).resolve()
        if cand.is_file():
            return cand
    root = ROOT / "dist" / "FAFUArmStation" / "app" / "vendor" / "fafu_arm_sdk"
    cand = root / "fafu_robot_python" / "fafu_robot_description" / "urdf" / "fafu_baseV1.urdf"
    return cand if cand.is_file() else None


def parse_urdf_colors(urdf: Path) -> dict[str, list[float]]:
    text = urdf.read_text(encoding="utf-8")
    out: dict[str, list[float]] = {}
    for match in _LINK_RE.finditer(text):
        out[match.group(1)] = [float(match.group(i)) for i in range(2, 5)]
    return out


def write_visual_json(path: Path, colors: dict[str, list[float]], urdf_name: str) -> None:
    payload = {"source": urdf_name, "links": {name: colors.get(name, [0.752941176470588] * 3) for name in NAMES}}
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _sdk_mesh_dir() -> Path | None:
    env = (ROOT / ".." / "fafu_arm_sdk").resolve()
    candidates = [
        Path(r"D:\fafu_arm_sdk\fafu_robot_python\fafu_robot_description\meshes"),
        env / "fafu_robot_python" / "fafu_robot_description" / "meshes",
        ROOT / "vendor" / "fafu_arm_sdk" / "fafu_robot_python" / "fafu_robot_description" / "meshes",
        ROOT / "dist" / "FAFUArmStation" / "app" / "vendor" / "fafu_arm_sdk"
        / "fafu_robot_python" / "fafu_robot_description" / "meshes",
    ]
    for path in candidates:
        if (path / "base_link.STL").is_file():
            return path
    return None


def load_stl_triangles(path: Path) -> list[tuple[float, ...]]:
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:200].lower():
        return _load_ascii(data.decode("latin-1", errors="ignore"))
    n = struct.unpack_from("<I", data, 80)[0]
    need = 84 + n * 50
    if len(data) < need:
        raise ValueError(f"{path.name}: truncated binary STL ({len(data)} < {need})")
    tris: list[tuple[float, ...]] = []
    off = 84
    for _ in range(n):
        tris.append(struct.unpack_from("<9f", data, off + 12))
        off += 50
    return tris


def _load_ascii(text: str) -> list[tuple[float, ...]]:
    verts: list[float] = []
    tris: list[tuple[float, ...]] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line.lower().startswith("vertex"):
            continue
        parts = line.split()
        verts.extend(float(parts[i]) for i in range(1, 4))
        if len(verts) >= 9:
            tris.append(tuple(verts[:9]))
            verts = verts[9:]
    return tris


def _area2(tri: tuple[float, ...]) -> float:
    ax, ay, az, bx, by, bz, cx, cy, cz = tri
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    return nx * nx + ny * ny + nz * nz


def weld(tris: list[tuple[float, ...]], cell: float = WELD_CELL_M) -> list[tuple[float, ...]]:
    keys: dict[tuple[int, int, int], int] = {}
    verts: list[tuple[float, float, float]] = []

    def vid(x: float, y: float, z: float) -> int:
        k = (round(x / cell), round(y / cell), round(z / cell))
        i = keys.get(k)
        if i is None:
            i = len(verts)
            keys[k] = i
            verts.append((x, y, z))
        return i

    out: list[tuple[float, ...]] = []
    for tri in tris:
        i0 = vid(tri[0], tri[1], tri[2])
        i1 = vid(tri[3], tri[4], tri[5])
        i2 = vid(tri[6], tri[7], tri[8])
        if i0 == i1 or i1 == i2 or i2 == i0:
            continue
        a, b, c = verts[i0], verts[i1], verts[i2]
        out.append(a + b + c)
    return out


def downsample(tris: list[tuple[float, ...]], max_tris: int) -> list[tuple[float, ...]]:
    welded = weld(tris)
    if len(welded) <= max_tris:
        return welded
    ranked = sorted(welded, key=_area2, reverse=True)
    return ranked[:max_tris]


def write_bin(path: Path, tris: list[tuple[float, ...]]) -> None:
    blob = bytearray(MAGIC)
    blob += struct.pack("<II", VERSION, len(tris))
    for tri in tris:
        blob += struct.pack("<9f", *tri)
    path.write_bytes(blob)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", type=Path, default=None)
    parser.add_argument("--dst", type=Path, default=ROOT / "robot_station" / "web" / "static" / "meshes")
    parser.add_argument("--max-tris", type=int, default=DEFAULT_MAX_TRIS)
    parser.add_argument("--colors-only", action="store_true")
    args = parser.parse_args()
    src = args.src or _sdk_mesh_dir()
    if src is None:
        print("找不到 SDK meshes（base_link.STL）", file=sys.stderr)
        return 1
    args.dst.mkdir(parents=True, exist_ok=True)
    urdf = _urdf_path(src)
    colors = parse_urdf_colors(urdf) if urdf else {}
    if urdf:
        write_visual_json(args.dst / "urdf_visual.json", colors, urdf.name)
        print("urdf colors from", urdf.name, {k: [round(x, 3) for x in v] for k, v in colors.items()})
    if args.colors_only:
        return 0 if colors else 1
    for name in NAMES:
        stl = src / f"{name}.STL"
        if not stl.is_file():
            stl = src / f"{name}.stl"
        if not stl.is_file():
            print(f"缺 {name}.STL", file=sys.stderr)
            return 1
        tris = downsample(load_stl_triangles(stl), args.max_tris)
        out = args.dst / f"{name}.bin"
        write_bin(out, tris)
        span = 0.0
        if tris:
            xs = [t[i] for t in tris for i in (0, 3, 6)]
            ys = [t[i] for t in tris for i in (1, 4, 7)]
            zs = [t[i] for t in tris for i in (2, 5, 8)]
            span = math.sqrt((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2 + (max(zs) - min(zs)) ** 2)
        print(f"{name}: {out.stat().st_size} bytes, {len(tris)} tris, span {span:.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
