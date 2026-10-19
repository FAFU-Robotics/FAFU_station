"""Launch a Panda3D / WRS window that follows the station arm.

The station process only checks paths and spawns this module. WRS and
Panda3D are imported inside :func:`run_viewer`, so the web server can
import :class:`WrsSession` without those packages.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

from robot_station.config import ROOT
from robot_station.grav_urdf import resolve_gravity_urdf, selected_urdf_name

_LOCK = threading.Lock()
_PROC: subprocess.Popen | None = None
_LOG: Path | None = None
_LOG_FH = None
_ERROR = ""


def wrs_python() -> str:
    override = (os.environ.get("WRS_PYTHON") or "").strip()
    return override or sys.executable


def wrs_root() -> Path | None:
    raw = (os.environ.get("WRS_ROOT") or "").strip()
    if not raw:
        return None
    path = Path(raw).expanduser()
    if (path / "wrs").is_dir():
        return path
    if path.name == "wrs" and (path / "__init__.py").is_file():
        return path.parent
    return None


def _mesh_dirs(urdf_path: Path) -> list[Path]:
    dirs = [
        urdf_path.parent / "meshes",
        urdf_path.parent.parent / "meshes",
        ROOT / "meshes",
        ROOT / "urdf" / "meshes",
    ]
    sdk = (os.environ.get("FAFU_ARM_SDK") or "").strip()
    roots = [
        Path(sdk) if sdk else None,
        ROOT / "vendor" / "fafu_arm_sdk",
        ROOT.parent / "fafu_arm_sdk",
        Path.home() / "fafu_arm_sdk",
    ]
    for root in roots:
        if root is None:
            continue
        desc = root / "fafu_robot_python" / "fafu_robot_description"
        dirs.append(desc / "meshes")
        dirs.append(desc / "urdf" / "meshes")
    out: list[Path] = []
    seen: set[str] = set()
    for folder in dirs:
        key = str(folder)
        if key in seen:
            continue
        seen.add(key)
        out.append(folder)
    return out


def _resolve_mesh(filename: str, urdf_path: Path) -> Path | None:
    name = Path(filename.replace("\\", "/")).name
    if not name:
        return None
    stems = [name]
    lower = name.lower()
    if lower.endswith(".stl"):
        stems.append(name[:-4] + ".STL")
        stems.append(name[:-4] + ".stl")
    direct = Path(filename)
    if direct.is_file():
        return direct.resolve()
    for folder in _mesh_dirs(urdf_path):
        for candidate in stems:
            path = folder / candidate
            if path.is_file():
                return path.resolve()
    return None


def rewrite_urdf_meshes(urdf_path: Path, dest: Path | None = None) -> tuple[Path, dict[str, Path]]:
    """Copy ``urdf_path`` with ``package://`` meshes rewritten to local STL files."""
    tree = ET.parse(urdf_path)
    missing: list[str] = []
    by_link: dict[str, Path] = {}
    for link in tree.iter("link"):
        link_name = link.get("name") or ""
        for mesh in link.iter("mesh"):
            filename = mesh.get("filename") or ""
            if not filename:
                continue
            resolved = _resolve_mesh(filename, urdf_path)
            if resolved is None:
                missing.append(Path(filename).name or filename)
                continue
            mesh.set("filename", str(resolved))
            by_link.setdefault(link_name, resolved)
    if missing:
        names = ", ".join(sorted(set(missing)))
        raise RuntimeError(
            f"找不到网格 {names}。请把 STL 放到 URDF 旁的 meshes/，或设置 FAFU_ARM_SDK。"
        )
    if dest is None:
        dest = Path(tempfile.gettempdir()) / "fafu-wrs-urdf" / urdf_path.name
    dest.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dest, encoding="utf-8", xml_declaration=True)
    return dest, by_link


def _selected_urdf() -> Path:
    name = selected_urdf_name() or "fafu_baseV1.urdf"
    try:
        return resolve_gravity_urdf(name)
    except FileNotFoundError:
        return resolve_gravity_urdf("fafu_baseV1.urdf")


def status() -> dict:
    with _LOCK:
        proc = _PROC
        running = proc is not None and proc.poll() is None
        if proc is not None and not running and not _ERROR:
            tail = _log_tail()
            err = tail or f"仿真进程已退出（code {proc.returncode}）"
        else:
            err = _ERROR
        pid = proc.pid if proc is not None and running else None
    return {"ok": True, "running": running, "pid": pid, "error": err}


def _log_tail() -> str:
    if _LOG is None or not _LOG.is_file():
        return ""
    try:
        text = _LOG.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    return "\n".join(lines[-8:])


def _set_error(message: str) -> None:
    global _ERROR
    _ERROR = message


def _close_log() -> None:
    global _LOG_FH
    fh = _LOG_FH
    _LOG_FH = None
    if fh is not None:
        try:
            fh.close()
        except OSError:
            pass


def stop() -> None:
    global _PROC
    with _LOCK:
        proc = _PROC
        _PROC = None
        _set_error("")
        _close_log()
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()


def start(info_url: str) -> str | None:
    """Start the viewer. Returns an error string, or None on success."""
    global _PROC, _LOG, _LOG_FH
    root = wrs_root()
    if root is None:
        raw = (os.environ.get("WRS_ROOT") or "").strip()
        if not raw:
            msg = "未设置 WRS_ROOT。请指向已 clone 的 wrs_tiaozhanbei（目录下要有 wrs/）。"
        else:
            msg = f"WRS_ROOT={raw} 下找不到 wrs 包。"
        _set_error(msg)
        return msg
    try:
        urdf = _selected_urdf()
        rewritten, _meshes = rewrite_urdf_meshes(urdf)
    except (FileNotFoundError, RuntimeError, OSError) as exc:
        _set_error(str(exc))
        return str(exc)

    with _LOCK:
        if _PROC is not None and _PROC.poll() is None:
            _set_error("")
            return None
        _close_log()
        log_path = Path(tempfile.gettempdir()) / "fafu-wrs-view.log"
        log_fh = log_path.open("w", encoding="utf-8")
        _LOG_FH = log_fh
        env = os.environ.copy()
        env["WRS_ROOT"] = str(root)
        prev = env.get("PYTHONPATH") or ""
        env["PYTHONPATH"] = os.pathsep.join(
            [str(ROOT), str(root)] + ([prev] if prev else [])
        )
        try:
            proc = subprocess.Popen(
                [
                    wrs_python(),
                    "-m",
                    "tools.wrs_legacy_preview.wrs_view",
                    "--info-url",
                    info_url,
                    "--urdf",
                    str(rewritten),
                ],
                cwd=str(ROOT),
                env=env,
                stdout=log_fh,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            _set_error(str(exc))
            _close_log()
            return str(exc)
        _PROC = proc
        _LOG = log_path
        _set_error("")

    time.sleep(0.6)
    with _LOCK:
        if _PROC is proc and proc.poll() is not None:
            msg = _log_tail() or f"仿真进程启动失败（code {proc.returncode}）"
            _set_error(msg)
            return msg
    return None


def _fetch_q(info_url: str) -> list[float] | None:
    try:
        with urllib.request.urlopen(info_url, timeout=0.4) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
        return None
    raw = payload.get("q_deg") or []
    if not isinstance(raw, list) or len(raw) < 6:
        return None
    try:
        return [float(x) for x in raw[:6]]
    except (TypeError, ValueError):
        return None


def _as_vec3(value) -> list[float]:
    import numpy as np

    if value is None:
        return [0.0, 0.0, 1.0]
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size < 3:
        return [0.0, 0.0, 1.0]
    return [float(arr[0]), float(arr[1]), float(arr[2])]


def _origin_pos_rot(joint):
    import numpy as np

    origin = getattr(joint, "origin", None)
    if origin is None:
        return np.zeros(3), np.eye(3)
    mat = np.asarray(origin, dtype=float)
    if mat.shape == (4, 4):
        return mat[:3, 3].copy(), mat[:3, :3].copy()
    return np.zeros(3), np.eye(3)


def _link_named(robot, name: str):
    link_map = getattr(robot, "link_map", {}) or {}
    if name in link_map:
        return link_map[name]
    for link in getattr(robot, "links", []) or []:
        if getattr(link, "name", "") == name:
            return link
    return None


def _joint_named(robot, name: str):
    joint_map = getattr(robot, "joint_map", {}) or {}
    if name in joint_map:
        return joint_map[name]
    for joint in getattr(robot, "joints", []) or []:
        if getattr(joint, "name", "") == name:
            return joint
    return None


def _collision_model(mcm, link, stl: Path | None, name: str):
    mesh = getattr(link, "collision_mesh", None) if link is not None else None
    if mesh is not None:
        try:
            return mcm.CollisionModel(initor=mesh, name=name)
        except TypeError:
            return mcm.CollisionModel(mesh)
        except Exception:
            pass
    if stl is not None and stl.is_file():
        try:
            return mcm.CollisionModel(initor=str(stl), name=name)
        except TypeError:
            return mcm.CollisionModel(str(stl))
    return None


def _build_chain(urdf_path: Path, meshes: dict[str, Path]):
    import numpy as np
    import wrs.modeling.collision_model as mcm
    import wrs.robot_sim._kinematics.jlchain as rkjlc
    from wrs.robot_sim.urdf.urdf_parser import URDF

    robot = URDF.load(str(urdf_path))
    joints = []
    for index in range(1, 7):
        joint = _joint_named(robot, f"joint{index}")
        if joint is None:
            raise RuntimeError(f"URDF 缺少 joint{index}")
        joints.append(joint)
    chain = rkjlc.JLChain(name="fafu", n_dof=6)
    base_link = _link_named(robot, "base_link")
    base_model = _collision_model(mcm, base_link, meshes.get("base_link"), "base_link")
    if base_model is not None:
        try:
            chain.anchor.lnk_list[0].cmodel = base_model
        except Exception:
            pass
    for index, joint in enumerate(joints):
        pos, rot = _origin_pos_rot(joint)
        jnt = chain.jnts[index]
        jnt.loc_pos = np.asarray(pos, dtype=float)
        jnt.loc_rotmat = np.asarray(rot, dtype=float)
        jnt.loc_motion_ax = np.asarray(_as_vec3(getattr(joint, "axis", [0, 0, 1])), dtype=float)
        limit = getattr(joint, "limit", None)
        lower = getattr(limit, "lower", None) if limit is not None else None
        upper = getattr(limit, "upper", None) if limit is not None else None
        if lower is None or upper is None:
            lower, upper = -np.pi, np.pi
        jnt.motion_range = np.array([float(lower), float(upper)], dtype=float)
        child = getattr(joint, "child", "")
        child_name = child if isinstance(child, str) else getattr(child, "name", "")
        link = _link_named(robot, child_name)
        model = _collision_model(mcm, link, meshes.get(child_name), child_name or f"link{index+1}")
        if model is not None:
            jnt.lnk.cmodel = model
    chain.finalize(ik_solver=None)
    return chain


def _add_ground(base, mgm) -> None:
    import numpy as np

    try:
        ground = mgm.gen_box(
            xyz_lengths=np.array([1.2, 1.2, 0.02]),
            pos=np.array([0.2, 0.0, -0.02]),
            rgba=np.array([0.22, 0.26, 0.30, 1.0]),
        )
        ground.attach_to(base)
    except Exception as exc:
        print(f"ground skipped: {exc}")


def run_viewer(info_url: str, urdf_path: Path) -> None:
    root = wrs_root()
    if root is None:
        raise RuntimeError("WRS_ROOT 未设置或无效")
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import numpy as np
    import wrs.modeling.geometric_model as mgm
    import wrs.visualization.panda.world as wd
    from direct.task.TaskManagerGlobal import taskMgr

    _rewritten, meshes = rewrite_urdf_meshes(urdf_path, urdf_path)
    chain = _build_chain(urdf_path, meshes)
    base = wd.World(cam_pos=[1.4, 1.2, 0.9], lookat_pos=[0.15, 0.0, 0.25])
    mgm.gen_frame().attach_to(base)
    _add_ground(base, mgm)
    holder = {"mesh": None, "q": None}

    def _show(q_deg: list[float]) -> None:
        chain.goto_given_conf(np.radians(np.asarray(q_deg, dtype=float)))
        old = holder["mesh"]
        if old is not None:
            try:
                old.detach()
            except Exception:
                pass
        mesh = chain.gen_meshmodel(alpha=1.0)
        mesh.attach_to(base)
        holder["mesh"] = mesh
        holder["q"] = list(q_deg)

    _show([0.0] * 6)

    def update(task):
        q = _fetch_q(info_url)
        if q is not None and q != holder["q"]:
            _show(q)
        return task.again

    taskMgr.doMethodLater(0.05, update, "fafu-wrs-sync")
    base.run()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="WRS / Panda3D view of the Fafu arm")
    parser.add_argument("--info-url", required=True)
    parser.add_argument("--urdf", required=True)
    args = parser.parse_args(argv)
    try:
        run_viewer(args.info_url, Path(args.urdf))
    except Exception as exc:
        print(f"WRS 仿真启动失败: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
