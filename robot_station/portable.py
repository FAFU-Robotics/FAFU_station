"""Private runtime: keep the bundled Python inside this app.

The portable layout is::

    FAFUArmStation/
      FAFUArmStation.exe
      runtime/python310/     # embeddable CPython, not on PATH
      app/                   # station sources (this package lives here)

Launchers set process-local environment only. They must not write
user/system PATH, install a global Python, or pip into the customer's
interpreter.

``STATION_PORTABLE=1`` alone is not enough: a leftover flag in a developer
shell must not hijack the system interpreter. Portable mode requires the
bundled ``runtime/python310`` layout (or ``STATION_INSTALL_ROOT`` pointing
at one).
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


def looks_like_bundled_python(executable: str | None = None) -> bool:
    """True when the interpreter path is ``.../runtime/python310/python.exe``."""
    exe = Path(executable or sys.executable).resolve()
    parent = exe.parent
    return parent.name.lower() == "python310" and parent.parent.name.lower() == "runtime"


def install_root() -> Path | None:
    """Install directory that contains ``runtime`` and ``app``, if any."""
    if looks_like_bundled_python():
        root = Path(sys.executable).resolve().parent.parent.parent
        if (root / "app" / "run_station.py").is_file():
            return root
        if (root / "runtime" / "python310" / "python.exe").is_file():
            return root
    env_root = (os.environ.get("STATION_INSTALL_ROOT") or "").strip()
    if not env_root:
        return None
    root = Path(env_root).expanduser().resolve()
    if (root / "runtime" / "python310" / "python.exe").is_file():
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
        if (bundled / "python.exe").is_file() or bundled.is_dir():
            return bundled
    return Path(sys.executable).resolve().parent


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

    py_dir = _private_python_dir()
    path = os.environ.get("PATH") or ""
    prefix = str(py_dir) + os.pathsep + str(py_dir / "Scripts")
    parts = [p for p in path.split(os.pathsep) if p]
    already = False
    for part in parts:
        try:
            if Path(part).resolve() == py_dir.resolve():
                already = True
                break
        except (OSError, ValueError):
            continue
    if not already:
        os.environ["PATH"] = prefix + os.pathsep + path
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
        return env
    inherited = env.get("PYTHONPATH") or ""
    env["PYTHONPATH"] = root + (os.pathsep + inherited if inherited else "")
    return env
