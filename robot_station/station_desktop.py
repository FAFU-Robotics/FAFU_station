#!/usr/bin/env python3
"""PC 宿主窗口：拉起本机站控，再用独立应用窗口打开，不走系统浏览器。"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONF = HERE / "station_client.conf"
DEFAULT_PORT = 9400
MOTION_PORT = 9470
DEFAULT_PATH = "/"
APP_TITLE = "FAFU 机械臂站控"
WAIT_S = 25.0
LOG_DIR = Path(
    os.environ.get("TEMP")
    or os.environ.get("TMP")
    or os.environ.get("XDG_CACHE_HOME")
    or (str(Path.home() / ".cache") if os.name != "nt" else str(HERE))
)
LOG = LOG_DIR / "fafu-station-app.log"
HOST_LOG = LOG_DIR / "fafu-station-host.log"

_PAGE_CSS = """
  html, body {{
    margin: 0; min-height: 100%;
    background: #0a1016; color: #e8eef5;
    font: 15px/1.5 "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
  }}
  .box {{
    max-width: 520px; margin: 12vh auto; padding: 28px 26px;
    background: #141d27; border: 1px solid #243140; border-radius: 16px;
  }}
  h1 {{ margin: 0 0 8px; font-size: 20px; }}
  p {{ color: #8b9aab; }}
  button {{
    margin-top: 16px; width: 100%; padding: 10px 12px; border-radius: 8px;
    border: 1px solid #2d6f8f; background: #123346; color: #d7f3ff;
    font: inherit; cursor: pointer;
  }}
"""

_LOADING_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>FAFU 机械臂站控</title>
<style>""" + _PAGE_CSS + """</style>
</head>
<body>
  <div class="box">
    <h1>正在启动机械臂站控</h1>
    <p>目标：<code>{url}</code></p>
    <p>控制页马上打开。真机 USB 在后台连接，不会挡住这个窗口。</p>
    <p>日志：%TEMP%\\fafu-station-app.log</p>
    <!-- host={host} port={port} -->
  </div>
</body>
</html>
"""

_OFFLINE_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>FAFU 机械臂站控</title>
<style>""" + _PAGE_CSS + """</style>
</head>
<body>
  <div class="box">
    <h1>本机站控未启动</h1>
    <p>目标：<code>{url}</code></p>
    <p>日志：%TEMP%\\fafu-station-app.log</p>
    <button onclick="retry()">重新连接</button>
    <!-- host={host} -->
  </div>
  <script>
    function retry() {{
      if (window.pywebview && window.pywebview.api) {{
        window.pywebview.api.reconnect('127.0.0.1', '{port}');
      }}
    }}
  </script>
</body>
</html>
"""


def _log(msg: str) -> None:
    line = time.strftime("%H:%M:%S ") + msg
    try:
        with LOG.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except OSError:
        pass


def _alert(text: str, title: str = APP_TITLE) -> None:
    _log(text)
    if os.name == "nt":
        try:
            import ctypes

            ctypes.windll.user32.MessageBoxW(0, text, title, 0x10)
            return
        except Exception:
            pass
    zenity = shutil.which("zenity")
    if zenity:
        try:
            subprocess.run(
                [zenity, "--error", f"--title={title}", "--width=480", f"--text={text}"],
                timeout=180,
                check=False,
            )
            return
        except (OSError, subprocess.TimeoutExpired):
            pass
    print(text, file=sys.stderr)


def _repo_root() -> Path:
    here = Path(__file__).resolve().parent
    if (here / "run_station.py").is_file():
        return here
    parent = here.parent
    if (parent / "run_station.py").is_file():
        return parent
    return here


def _load_conf(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not path.is_file():
        return data
    with path.open("r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            data[key.strip()] = value.strip()
    return data


def _save_host(host: str, port: int) -> None:
    if not CONF.parent.is_dir():
        return
    lines: list[str] = []
    if CONF.is_file():
        lines = CONF.read_text(encoding="utf-8").splitlines()
    written = {"HOST": False, "PORT": False}
    out: list[str] = []
    for line in lines:
        raw = line.strip()
        if raw.startswith("HOST="):
            out.append(f"HOST={host}")
            written["HOST"] = True
        elif raw.startswith("PORT="):
            out.append(f"PORT={port}")
            written["PORT"] = True
        else:
            out.append(line)
    if not written["HOST"]:
        out.append(f"HOST={host}")
    if not written["PORT"]:
        out.append(f"PORT={port}")
    CONF.write_text("\n".join(out) + "\n", encoding="utf-8")


def _norm_path(path: str) -> str:
    raw = (path or DEFAULT_PATH).strip() or DEFAULT_PATH
    if not raw.startswith("/"):
        raw = "/" + raw
    return raw


def _make_url(host: str, port: int, path: str) -> str:
    return f"http://{host}:{int(port)}{_norm_path(path)}"


def _port_open(host: str, port: int, timeout: float = 0.35) -> bool:
    if not host:
        return False
    sock = socket.socket()
    sock.settimeout(timeout)
    try:
        sock.connect((host, int(port)))
    except OSError:
        return False
    finally:
        sock.close()
    return True


def _looks_like_station(host: str, port: int) -> bool:
    url = f"http://{host}:{int(port)}/api/lab"
    try:
        with urllib.request.urlopen(url, timeout=0.7) as resp:
            raw = resp.read(800)
    except (urllib.error.URLError, TimeoutError, OSError, ValueError):
        return False
    try:
        data = json.loads(raw.decode("utf-8", errors="replace"))
    except json.JSONDecodeError:
        return False
    return bool(isinstance(data, dict) and data.get("ok") and data.get("station_port"))


def _wait_ready(host: str, port: int, timeout_s: float, proc: subprocess.Popen | None = None) -> bool:
    if not host:
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            _log(f"host exited {proc.returncode} before :{port} listened")
            return False
        if _port_open(host, port):
            return True
        time.sleep(0.3)
    return False


def _python_exe() -> str:
    exe = Path(sys.executable)
    if exe.name.lower() == "pythonw.exe":
        alt = exe.with_name("python.exe")
        if alt.is_file():
            return str(alt)
    return str(exe)


def _has_pinocchio() -> bool:
    try:
        import pinocchio  # noqa: F401
    except Exception:
        return False
    return True


def _live_arm_wanted() -> bool:
    return (os.environ.get("STATION_ALLOW_LIVE_ARM") or "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _station_info(port: int) -> dict:
    url = f"http://127.0.0.1:{int(port)}/api/info"
    try:
        with urllib.request.urlopen(url, timeout=0.8) as resp:
            data = json.loads(resp.read().decode("utf-8", errors="replace"))
    except (urllib.error.URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _running_code_matches(port: int | None = None) -> bool:
    """True when the motion process was started from this tree's control files.

    ``boot_rev`` is frozen at motion start. Do not use a live disk hash from
    ``/api/info`` — an old host recomputing ``station_code_rev()`` would look
    current and never ``--replace``.
    """
    try:
        from robot_station.runtime import station_code_rev
    except Exception:
        return False
    info = _station_info(port or DEFAULT_PORT)
    boot = str(info.get("boot_rev") or "").strip()
    if not boot:
        return False
    try:
        disk = station_code_rev(_repo_root())
    except Exception:
        return False
    return boot == disk


def _station_http_ok(port: int) -> bool:
    info = _station_info(port)
    if isinstance(info, dict) and info.get("ok"):
        return True
    return _looks_like_station("127.0.0.1", port)


def _motion_up() -> bool:
    return _port_open("127.0.0.1", MOTION_PORT)


def _live_station_ready(port: int) -> bool:
    return _existing_live_station_ok(_station_info(port))


def _existing_live_station_ok(info: dict | None) -> bool:
    """Reuse a live arm host. Camera mock must not kill a working serial session."""
    if not info:
        return False
    if str(info.get("arm") or "") != "fafu":
        return False
    return bool(info.get("arm_allow_motion"))


def _station_should_outlive_window() -> bool:
    """Live motion must keep 100 Hz hold; the window is only a viewer."""
    return _live_arm_wanted() or _motion_up()


def _spawn_local_station(web_only: bool = False, *, replace_web: bool = False) -> subprocess.Popen | None:
    root = _repo_root()
    run_py = root / "run_station.py"
    if not run_py.is_file():
        return None
    cmd = [_python_exe(), str(run_py), "--no-browser"]
    if web_only:
        cmd.extend(["--web-only", "--camera", "auto"])
        if replace_web:
            cmd.append("--replace")
            _log("9400 hung, motion still up: replace web-only (do not kill arm)")
        else:
            _log("9400 down, motion still up: spawn web-only + camera auto")
    elif _live_arm_wanted():
        cmd.extend(["--replace", "--arm", "fafu", "--allow-motion", "--camera", "auto"])
        _log("live arm: STATION_ALLOW_LIVE_ARM set, spawning fafu + allow-motion + camera auto")
    try:
        from robot_station.portable import child_env

        env = child_env(root)
    except Exception:
        env = os.environ.copy()
    if _live_arm_wanted():
        # USB 热插拔：调试板插上时自动把臂源切到真机（不自动使能），
        # 作业相机插拔时自动重认。见 robot_station/usb_watch.py。
        env["STATION_ARM_WATCH"] = "1"
        env["STATION_CAMERA_WATCH"] = "1"
    log_fh = None
    try:
        log_fh = HOST_LOG.open("a", encoding="utf-8")
    except OSError:
        log_fh = subprocess.DEVNULL
    kwargs: dict = {
        "cwd": str(root),
        "env": env,
        "stdout": log_fh,
        "stderr": subprocess.STDOUT,
    }
    if os.name == "nt":
        # CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW | CREATE_BREAKAWAY_FROM_JOB
        # so closing the GUI / console job cannot take 9400/9470 with it.
        kwargs["creationflags"] = (
            getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0x00000200)
            | 0x08000000
            | 0x01000000
        )
    else:
        kwargs["start_new_session"] = True
    _log("spawn " + " ".join(cmd))
    return subprocess.Popen(cmd, **kwargs)


def _spawn_and_wait(port: int, *, web_only: bool, replace_web: bool = False) -> tuple[bool, subprocess.Popen | None]:
    proc = _spawn_local_station(web_only=web_only, replace_web=replace_web)
    if proc is None:
        return False, None
    need_motion = not web_only and _live_arm_wanted()
    deadline = time.monotonic() + WAIT_S
    saw_gap = not replace_web
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            if (not need_motion and _port_open("127.0.0.1", port)
                    and (not replace_web or _station_http_ok(port))):
                return True, proc
            _log(f"host exited {proc.returncode} before :{port} listened")
            return False, None
        up = _port_open("127.0.0.1", port)
        ok = up and (not replace_web or _station_http_ok(port))
        if need_motion:
            ok = ok and _motion_up() and _running_code_matches(port)
        if replace_web and not saw_gap:
            if not up or not _station_http_ok(port):
                saw_gap = True
            time.sleep(0.3)
            continue
        if ok:
            return True, proc
        time.sleep(0.3)
    _stop_proc(proc)
    return False, None


def _stop_proc(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    if os.name == "nt":
        try:
            subprocess.run(
                ["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=8,
                check=False,
            )
            proc.wait(timeout=3.0)
            return
        except Exception:
            pass
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()


def _ensure_local_station(port: int) -> tuple[bool, subprocess.Popen | None]:
    http_up = _port_open("127.0.0.1", port)
    motion_up = _motion_up()
    if http_up and motion_up:
        if _live_arm_wanted() and not _running_code_matches(port):
            _log(
                f"motion :{MOTION_PORT} is up but code_rev != disk; "
                "--replace so enable/servo patches actually load"
            )
            return _spawn_and_wait(port, web_only=False)
        if _live_arm_wanted() and _has_pinocchio():
            info = _station_info(port)
            if info and not bool(info.get("dyn_ready")):
                _log(
                    "this Python has pinocchio but live station dyn_ready=false; "
                    "--replace so SDK gravity_compensation_step can load"
                )
                return _spawn_and_wait(port, web_only=False)
        _log(f"attach existing :{port} (motion :{MOTION_PORT} up, never replace while motion)")
        return True, None
    if http_up and _live_arm_wanted():
        info = _station_info(port)
        arm = str((info or {}).get("arm") or "")
        # /api/info may still describe the last snapshot after 9470 exits.
        # An orphan web process cannot control the arm, even if it says fafu.
        _log(
            f"live arm requested; existing :{port} is arm={arm or 'unknown'} "
            f"camera={(info or {}).get('camera') or 'unknown'} without motion, replacing"
        )
        return _spawn_and_wait(port, web_only=False)
    if http_up:
        return True, None
    if motion_up:
        _log(f"port {port} down, motion :{MOTION_PORT} still up — web-only (do not kill arm)")
        return _spawn_and_wait(port, web_only=True)
        _log(f"port {port} down, starting host")
    return _spawn_and_wait(port, web_only=False)


def _kickoff_local_station(port: int) -> tuple[bool, subprocess.Popen | None]:
    """Start or attach the host without waiting on USB / 9470 boot.

    The window must open first. Return ``(page_ready, child)``.
    ``page_ready`` means :9400 is already serving this tree.
    """
    http_up = _port_open("127.0.0.1", port)
    motion_up = _motion_up()
    if http_up and motion_up:
        if _live_arm_wanted() and not _running_code_matches(port):
            _log(
                f"motion :{MOTION_PORT} is up but code_rev != disk; "
                "spawn --replace in background so the window can open"
            )
            return False, _spawn_local_station(web_only=False)
        if _live_arm_wanted() and _has_pinocchio():
            info = _station_info(port)
            if info and not bool(info.get("dyn_ready")):
                _log(
                    "this Python has pinocchio but live station dyn_ready=false; "
                    "spawn --replace in background so SDK gravity_compensation_step can load"
                )
                return False, _spawn_local_station(web_only=False)
        _log(f"attach existing :{port} (motion :{MOTION_PORT} up, never replace while motion)")
        return True, None
    if http_up and _live_arm_wanted():
        info = _station_info(port)
        arm = str((info or {}).get("arm") or "")
        _log(
            f"live arm requested; existing :{port} is arm={arm or 'unknown'} "
            f"camera={(info or {}).get('camera') or 'unknown'} without motion, replacing"
        )
        return False, _spawn_local_station(web_only=False)
    if http_up:
        return True, None
    if motion_up:
        _log(f"port {port} down, motion :{MOTION_PORT} still up — web-only (do not kill arm)")
        return False, _spawn_local_station(web_only=True)
    _log(f"port {port} down, starting host (window first)")
    return False, _spawn_local_station(web_only=False)


def _guard_web(
    window,
    host: str,
    port: int,
    path: str,
    halt: threading.Event,
    state: dict,
) -> None:
    """While the window is open: if 9400 dies and 9470 lives, only respawn the web process."""
    misses = 0
    while not halt.wait(1.5):
        listening = _port_open(host, port)
        if listening:
            misses = 0
            if state.get("offline"):
                try:
                    window.load_url(_make_url(host, port, path))
                    state["offline"] = False
                    _log("web restored, reloaded window")
                except Exception as exc:
                    _log(f"reload after web restore failed: {exc}")
            continue
        if not _motion_up():
            misses = 0
            continue
        misses += 1
        if misses < 2:
            continue
        _log("web lost, motion still up; respawn web-only")
        ok, _proc = _spawn_and_wait(port, web_only=True, replace_web=False)
        if not ok and _port_open(host, port):
            ok, _proc = _spawn_and_wait(port, web_only=True, replace_web=True)
        if not ok:
            continue
        misses = 0
        if state.get("offline"):
            try:
                window.load_url(_make_url(host, port, path))
                state["offline"] = False
            except Exception as exc:
                _log(f"reload after web restore failed: {exc}")


class _Api:
    def __init__(self, window_holder: dict, path: str, port: int) -> None:
        self._holder = window_holder
        self._path = path
        self._port = port

    def reconnect(self, host: str, port: str) -> None:
        host = (host or "").strip() or "127.0.0.1"
        try:
            port_i = int(port)
        except (TypeError, ValueError):
            port_i = DEFAULT_PORT
        ready, _proc = _ensure_local_station(port_i)
        _save_host(host, port_i)
        window = self._holder.get("window")
        if window is None:
            return
        url = _make_url(host, port_i, self._path)
        if ready or _wait_ready(host, port_i, 2.0):
            window.load_url(url)
        else:
            window.load_html(_OFFLINE_HTML.format(url=url, host=host, port=port_i))


def _app_profile_dir() -> Path:
    if os.name == "nt":
        base = os.environ.get("LOCALAPPDATA") or str(Path.home() / "AppData" / "Local")
        return Path(base) / "FAFUArmStationWeb"
    return Path.home() / ".cache" / "fafu-station-app"


def _windows_browser_exes() -> list[Path]:
    pf = os.environ.get("PROGRAMFILES", r"C:\Program Files")
    pf86 = os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)")
    local = os.environ.get("LOCALAPPDATA", "")
    return [
        Path(pf86) / "Microsoft/Edge/Application/msedge.exe",
        Path(pf) / "Microsoft/Edge/Application/msedge.exe",
        Path(local) / "Microsoft/Edge/Application/msedge.exe",
        Path(pf) / "Google/Chrome/Application/chrome.exe",
        Path(pf86) / "Google/Chrome/Application/chrome.exe",
        Path(local) / "Google/Chrome/Application/chrome.exe",
    ]


def _app_shell_argv(url: str) -> list[str] | None:
    """Kept for tests. The product window is pywebview, not a browser."""
    profile = str(_app_profile_dir())
    args_tail = [
        f"--app={url}",
        "--window-size=1280,860",
        f"--user-data-dir={profile}",
        "--no-first-run",
        "--no-default-browser-check",
        "--new-window",
        "--class=fafu-station",
    ]
    if os.name == "nt":
        for exe in _windows_browser_exes():
            if exe.is_file():
                return [str(exe), *args_tail]
        for name in ("msedge", "chrome"):
            found = shutil.which(name)
            if found:
                return [found, *args_tail]
        return None
    for name in ("chromium-browser", "chromium", "google-chrome", "microsoft-edge"):
        found = shutil.which(name)
        if found:
            return [found, *args_tail]
    return None


def _load_webview():
    try:
        import webview

        return webview
    except ImportError:
        try:
            from robot_station.portable import is_portable

            portable = is_portable()
        except Exception:
            portable = False
        if portable:
            _alert(
                "便携包缺少独立窗口组件 pywebview。\n"
                "Windows：请用 packaging\\windows\\build_portable.ps1 重新打包。\n"
                "Ubuntu：请用 packaging/linux/build_appimage.sh 重新打包。\n"
                "不要对系统 Python 执行 pip。"
            )
            return None
        _log("pywebview missing, pip install")
        try:
            subprocess.check_call(
                [_python_exe(), "-m", "pip", "install", "pywebview"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            import webview

            return webview
        except Exception as exc:
            _alert(
                "无法加载独立窗口组件 pywebview。\n"
                f"{exc}\n\n请执行：python -m pip install pywebview"
            )
            return None


def _start_webview(
    url: str,
    ready: bool,
    host: str,
    port: int,
    path: str,
    child: subprocess.Popen | None = None,
) -> int:
    webview = _load_webview()
    if webview is None:
        return 1
    holder: dict = {}
    profile = _app_profile_dir()
    try:
        profile.mkdir(parents=True, exist_ok=True)
    except OSError:
        pass
    window_kwargs = {
        "title": APP_TITLE,
        "width": 1280,
        "height": 860,
        "min_size": (900, 640),
        "text_select": True,
        "confirm_close": False,
        "js_api": _Api(holder, path, port),
        "background_color": "#0A1016",
    }
    if ready:
        window_kwargs["url"] = url
    else:
        window_kwargs["html"] = _LOADING_HTML.format(url=url, host=host, port=port)
    window = webview.create_window(**window_kwargs)
    holder["window"] = window
    halt = threading.Event()
    state = {"offline": not ready}

    def _after_start() -> None:
        threading.Thread(
            target=_guard_web,
            args=(window, host, port, path, halt, state),
            name="web-guard",
            daemon=True,
        ).start()
        if ready:
            return
        if _wait_ready(host, port, WAIT_S, child):
            try:
                window.load_url(url)
                state["offline"] = False
                _log("control page loaded")
            except Exception as exc:
                _log(f"late load failed: {exc}")
            return
        if not state.get("offline"):
            return
        try:
            window.load_html(_OFFLINE_HTML.format(url=url, host=host, port=port))
            _log("host did not listen; showing offline page")
        except Exception as exc:
            _log(f"offline page failed: {exc}")

    _log(f"window {url} ready={ready}")
    start_kwargs = {
        "private_mode": False,
        "storage_path": str(profile),
    }
    if os.name == "nt":
        start_kwargs["gui"] = "edgechromium"
    else:
        start_kwargs["gui"] = "gtk"
    try:
        try:
            webview.start(_after_start, **start_kwargs)
            return 0
        except Exception as exc:
            _log(f"primary gui failed: {exc}")
            if os.name == "nt":
                try:
                    start_kwargs.pop("gui", None)
                    webview.start(_after_start, **start_kwargs)
                    return 0
                except Exception as exc2:
                    _alert(f"无法打开站控窗口：{exc2}")
                    return 1
            _alert(
                "无法打开独立控制窗口（需要 WebKitGTK / pywebview GTK 后端）。\n"
                f"{exc}\n\n请使用官方 AppImage，或安装：\n"
                "sudo apt install gir1.2-webkit2-4.0 gir1.2-gtk-3.0"
            )
            return 1
    finally:
        halt.set()


def main() -> int:
    try:
        from robot_station.portable import apply_bundled_runtime

        apply_bundled_runtime()
    except Exception:
        pass
    conf = _load_conf(CONF)
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--host", default=conf.get("HOST") or "127.0.0.1")
    parser.add_argument("--port", type=int, default=int(conf.get("PORT") or DEFAULT_PORT))
    parser.add_argument("--path", default=conf.get("PATH") or DEFAULT_PATH)
    args = parser.parse_args()

    host = (args.host or "127.0.0.1").strip() or "127.0.0.1"

    _log(f"launch desktop exe={sys.executable}")
    ready, child = _kickoff_local_station(args.port)
    if host not in ("127.0.0.1", "localhost") and not _port_open(host, args.port):
        host = "127.0.0.1"
    path = _norm_path(args.path)
    url = _make_url(host, args.port, path)
    try:
        return _start_webview(url, ready, host, args.port, path, child)
    finally:
        if _station_should_outlive_window():
            _log("window closed; leave station running (do not kill arm / web)")
        else:
            _stop_proc(child)


if __name__ == "__main__":
    raise SystemExit(main())
