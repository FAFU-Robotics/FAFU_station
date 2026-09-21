"""Private runtime: keep the bundled Python inside this app.

The portable layout is::

    FAFUArmStation/                  # Windows tree or AppImage AppDir
      runtime/python310/             # private CPython, not on PATH
        python.exe                   # Windows
        bin/python3                  # Linux / AppImage
      app/                           # station sources (this package lives here)

Launchers set process-local environment only. They must not write
user/system PATH, install a global Python, or pip into the customer's
interpreter.

``STATION_PORTABLE=1`` alone is not enough: a leftover flag in a developer
shell must not hijack the system interpreter. Portable mode requires the
bundled ``runtime/python310`` layout (or ``STATION_INSTALL_ROOT`` / ``APPDIR``
pointing at one).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

_TRUE = frozenset({"1", "true", "yes", "on"})
_STRIP_VARS = ("PYTHONSTARTUP", "PYTHONHOME", "PYTHONUSERBASE")


def _flag(name: str) -> bool:
    return (os.environ.get(name) or "").strip().lower() in _TRUE


def _is_under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except (OSError, ValueError):
        return False


def _exe_name(path: Path) -> str:
    return path.name.lower()


def looks_like_bundled_python(executable: str | None = None) -> bool:
    """True when the interpreter lives under ``runtime/python310``.

    Windows: ``.../runtime/python310/python.exe``
    POSIX / AppImage: ``.../runtime/python310/bin/python3``
    """
    exe = Path(executable or sys.executable)
    try:
        exe = exe.resolve()
    except OSError:
        pass
    name = _exe_name(exe)
    if not name.startswith("python"):
        return False
    parent = exe.parent
    if parent.name.lower() == "python310" and parent.parent.name.lower() == "runtime":
        return True
    if (
        parent.name.lower() == "bin"
        and parent.parent.name.lower() == "python310"
        and parent.parent.parent.name.lower() == "runtime"
    ):
        return True
    return False


def _install_root_from_exe(exe: Path) -> Path | None:
    parent = exe.parent
    if parent.name.lower() == "python310" and parent.parent.name.lower() == "runtime":
        return parent.parent.parent
    if (
        parent.name.lower() == "bin"
        and parent.parent.name.lower() == "python310"
        and parent.parent.parent.name.lower() == "runtime"
    ):
        return parent.parent.parent.parent
    return None


def bundled_python_exe(root: Path) -> Path | None:
    """Private interpreter under ``root/runtime/python310``, if present."""
    py_dir = root / "runtime" / "python310"
    win = py_dir / "python.exe"
    if win.is_file():
        return win
    for name in ("python3", "python"):
        posix = py_dir / "bin" / name
        if posix.is_file() or posix.is_symlink():
            return posix
    return None


def _has_bundled_runtime(root: Path) -> bool:
    return bundled_python_exe(root) is not None


def install_root() -> Path | None:
    """Install directory that contains ``runtime`` and ``app``, if any."""
    if looks_like_bundled_python():
        exe = Path(sys.executable)
        try:
            exe = exe.resolve()
        except OSError:
            pass
        root = _install_root_from_exe(exe)
        if root is not None:
            if (root / "app" / "run_station.py").is_file() or _has_bundled_runtime(root):
                return root
    for key in ("STATION_INSTALL_ROOT", "APPDIR"):
        raw = (os.environ.get(key) or "").strip()
        if not raw:
            continue
        root = Path(raw).expanduser()
        try:
            root = root.resolve()
        except OSError:
            pass
        if _has_bundled_runtime(root):
            return root
    return None


def is_portable() -> bool:
    """True only when a private runtime is actually present.

    A leftover ``STATION_PORTABLE=1`` on a developer machine must not
    treat the system Python as the app runtime.
    """
    if looks_like_bundled_python():
        return True
    return _flag("STATION_PORTABLE") and install_root() is not None


def app_root(source_root: Path | None = None) -> Path:
    """Directory that contains ``run_station.py``.

    Portable: ``<install>/app``. Source checkout: repository root.
    """
    inst = install_root()
    if inst is not None:
        bundled = inst / "app"
        if (bundled / "run_station.py").is_file():
            return bundled
    if source_root is not None:
        return source_root
    return Path(__file__).resolve().parents[1]


def bundled_sdk_dir() -> Path | None:
    app = app_root()
    for candidate in (
        app / "vendor" / "fafu_arm_sdk",
        (install_root() or app) / "vendor" / "fafu_arm_sdk",
    ):
        if (candidate / "fafu_robot_python").is_dir():
            return candidate
    return None


def sdk_search_roots() -> tuple[Path, ...]:
    """Directories portable mode may load an SDK from. Empty when not portable."""
    if not is_portable():
        return ()
    roots: list[Path] = [app_root()]
    inst = install_root()
    if inst is not None:
        roots.append(inst)
    # Preserve order, drop duplicates.
    out: list[Path] = []
    seen: set[str] = set()
    for root in roots:
        key = str(root.resolve())
        if key not in seen:
            seen.add(key)
            out.append(root)
    return tuple(out)


def sdk_path_allowed(path: Path) -> bool:
    """Whether an FAFU_ARM_SDK path is inside this app (portable) or unrestricted."""
    if not is_portable():
        return True
    return any(_is_under(path, root) for root in sdk_search_roots())


def _scrub_python_env(env: dict[str, str]) -> None:
    env["PYTHONNOUSERSITE"] = "1"
    env["PIP_USER"] = "0"
    for name in _STRIP_VARS:
        env.pop(name, None)


def _private_python_dir() -> Path:
    inst = install_root()
    if inst is not None:
        bundled = inst / "runtime" / "python310"
        if bundled_python_exe(inst) is not None or bundled.is_dir():
            return bundled
    exe = Path(sys.executable).resolve()
    if exe.parent.name.lower() == "bin":
        return exe.parent.parent
    return exe.parent


def _native_bin_dirs(py_dir: Path) -> list[Path]:
    """Private python.exe, Scripts, and conda-style Library/bin (pinocchio DLLs)."""
    return [py_dir / "Library" / "bin", py_dir / "bin", py_dir, py_dir / "Scripts"]


def _native_lib_dirs(py_dir: Path) -> list[Path]:
    """Directories that must be on ``LD_LIBRARY_PATH`` for ``fafu_motor`` / GTK."""
    dirs: list[Path] = [
        py_dir / "lib",
        py_dir / "lib64",
        py_dir / "Library" / "lib",
    ]
    inst = install_root()
    if inst is not None:
        dirs.extend(
            (
                inst / "usr" / "lib",
                inst / "usr" / "lib" / "x86_64-linux-gnu",
                inst / "usr" / "lib64",
            )
        )
    sdk = bundled_sdk_dir()
    if sdk is not None:
        dirs.append(sdk / "fafu_robot_python")
    return dirs


def _prepend_env_path(env: dict[str, str], name: str, extras: list[Path]) -> None:
    existing = env.get(name) or ""
    parts = [p for p in existing.split(os.pathsep) if p]
    resolved: set[str] = set()
    for part in parts:
        try:
            resolved.add(str(Path(part).resolve()))
        except (OSError, ValueError):
            resolved.add(part)
    prefix: list[str] = []
    for folder in extras:
        try:
            key = str(folder.resolve())
        except (OSError, ValueError):
            key = str(folder)
        if key not in resolved:
            prefix.append(str(folder))
            resolved.add(key)
    if prefix:
        env[name] = os.pathsep.join(prefix + parts)


def _prepend_native_path(env: dict[str, str], py_dir: Path) -> None:
    extras = [d for d in _native_bin_dirs(py_dir) if d.is_dir()]
    if not extras:
        extras = [py_dir]
    _prepend_env_path(env, "PATH", extras)
    libbin = py_dir / "Library" / "bin"
    if libbin.is_dir():
        env["PINOCCHIO_WINDOWS_DLL_PATH"] = str(libbin)
    lib_dirs = [d for d in _native_lib_dirs(py_dir) if d.is_dir()]
    if lib_dirs:
        _prepend_env_path(env, "LD_LIBRARY_PATH", lib_dirs)


def apply_bundled_runtime() -> bool:
    """Lock this process (and children) onto the private runtime.

    Returns True when the portable layout is active.
    """
    if not is_portable():
        return False
    os.environ["STATION_PORTABLE"] = "1"
    _scrub_python_env(os.environ)

    root = app_root()
    os.environ["PYTHONPATH"] = str(root)
    inst = install_root()
    if inst is not None:
        os.environ["STATION_INSTALL_ROOT"] = str(inst)

    raw_sdk = (os.environ.get("FAFU_ARM_SDK") or "").strip()
    if raw_sdk and not sdk_path_allowed(Path(raw_sdk)):
        os.environ.pop("FAFU_ARM_SDK", None)
    sdk = bundled_sdk_dir()
    if sdk is not None:
        os.environ["FAFU_ARM_SDK"] = str(sdk)

    _prepend_native_path(os.environ, _private_python_dir())
    return True


def child_env(pythonpath_root: Path) -> dict[str, str]:
    """Environment for a station subprocess. Portable mode does not inherit a foreign PYTHONPATH."""
    env = os.environ.copy()
    root = str(pythonpath_root)
    if is_portable():
        env["STATION_PORTABLE"] = "1"
        _scrub_python_env(env)
        env["PYTHONPATH"] = root
        raw_sdk = (env.get("FAFU_ARM_SDK") or "").strip()
        if raw_sdk and not sdk_path_allowed(Path(raw_sdk)):
            env.pop("FAFU_ARM_SDK", None)
        sdk = bundled_sdk_dir()
        if sdk is not None:
            env["FAFU_ARM_SDK"] = str(sdk)
        _prepend_native_path(env, _private_python_dir())
        return env
    inherited = env.get("PYTHONPATH") or ""
    env["PYTHONPATH"] = root + (os.pathsep + inherited if inherited else "")
    return env
