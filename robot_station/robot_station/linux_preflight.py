"""Linux launch checks: GTK WebKit, serial ports, udev, ModemManager.

Windows keeps preflight in ``packaging/windows/PortableLauncher.cs``.
This module is the AppImage / ``启动真机.sh`` equivalent. It never opens
the arm serial port and never starts a camera pipeline.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

TITLE = "FAFU 机械臂站控"
UDEV_NAME = "99-fafu-debug-board.rules"
UDEV_DEST = Path("/etc/udev/rules.d") / UDEV_NAME


@dataclass
class LinuxPreflight:
    webview_ok: bool = False
    webview_detail: str = ""
    motor_ok: bool = False
    motor_detail: str = ""
    serial_ports: list[str] = field(default_factory=list)
    serial_writable: bool = False
    udev_installed: bool = False
    modemmanager: bool = False
    warnings: list[str] = field(default_factory=list)
    blockers: list[str] = field(default_factory=list)
    fixes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return asdict(self)


FUSE_HINT = (
    "若双击 AppImage 没有窗口：先 chmod +x，再试 sudo apt install libfuse2；"
    "仍不行则：./FAFUArmStation-x86_64.AppImage --appimage-extract-and-run"
)


def preflight_report_path() -> Path:
    xdg = (os.environ.get("XDG_CACHE_HOME") or "").strip()
    if xdg:
        return Path(xdg) / "fafu-station-preflight.json"
    return Path.home() / ".cache" / "fafu-station-preflight.json"


def write_preflight_report(result: LinuxPreflight, path: Path | None = None) -> Path:
    dest = path or preflight_report_path()
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(result.as_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return dest


def serial_candidates() -> list[Path]:
    """Debug-board-like tty nodes. Does not open them."""
    found: list[Path] = []
    seen: set[str] = set()
    root = Path("/dev")
    names = ["fafu_debug_board"]
    if root.is_dir():
        names.extend(sorted(p.name for p in root.glob("ttyUSB*")))
        names.extend(sorted(p.name for p in root.glob("ttyACM*")))
    for name in names:
        path = root / name
        try:
            if not (path.exists() or path.is_symlink()):
                continue
        except OSError:
            continue
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        found.append(path)
    return found


def _port_writable(path: Path) -> bool:
    try:
        return os.access(path, os.W_OK)
    except OSError:
        return False


def udev_rules_path() -> Path | None:
    """Bundled udev rules shipped with the AppImage or source tree."""
    candidates: list[Path] = []
    try:
        from robot_station.portable import app_root, install_root

        app = app_root()
        inst = install_root()
        candidates.append(
            app / "vendor" / "fafu_arm_sdk" / "fafu_robot_cpp" / "linux" / UDEV_NAME
        )
        if inst is not None:
            candidates.append(inst / "usr" / "share" / "fafu" / UDEV_NAME)
            candidates.append(inst / "app" / "vendor" / "fafu_arm_sdk" / "fafu_robot_cpp" / "linux" / UDEV_NAME)
    except Exception:
        pass
    here = Path(__file__).resolve()
    repo = here.parents[1]
    candidates.append(repo / "packaging" / "linux" / UDEV_NAME)
    sdk_env = (os.environ.get("FAFU_ARM_SDK") or "").strip()
    if sdk_env:
        candidates.append(Path(sdk_env) / "fafu_robot_cpp" / "linux" / UDEV_NAME)
    for cand in candidates:
        try:
            if cand.is_file():
                return cand
        except OSError:
            continue
    return None


def udev_installed() -> bool:
    return UDEV_DEST.is_file()


def modemmanager_active() -> bool:
    systemctl = shutil.which("systemctl")
    if not systemctl:
        return False
    try:
        proc = subprocess.run(
            [systemctl, "is-active", "--quiet", "ModemManager"],
            timeout=2,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return proc.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def probe_webview() -> tuple[bool, str]:
    try:
        import webview  # noqa: F401
    except Exception as exc:
        return False, f"无法加载 pywebview：{exc}"
    display = (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or "").strip()
    try:
        import gi  # type: ignore
    except Exception as exc:
        if not display:
            return True, f"已 import webview（无 DISPLAY，未探测 GTK：{exc}）"
        return False, f"无法加载 PyGObject：{exc}"
    errors: list[str] = []
    for webkit in ("4.0", "4.1"):
        try:
            gi.require_version("Gtk", "3.0")
            gi.require_version("WebKit2", webkit)
            from gi.repository import Gtk, WebKit2  # type: ignore  # noqa: F401

            return True, f"GTK3 + WebKit2 {webkit} 可用"
        except Exception as exc:
            errors.append(f"WebKit2 {webkit}: {exc}")
    if not display:
        return True, "已 import webview（无 DISPLAY，GTK 探测失败，源码/SSH 模式可继续测 9470）"
    return False, "；".join(errors) or "WebKitGTK 不可用"


def _truthy_env(name: str) -> bool:
    return (os.environ.get(name) or "").strip().lower() in {"1", "true", "yes", "on"}


def probe_fafu_motor() -> tuple[bool, str]:
    """Load fafu_motor without opening the serial port."""
    try:
        from robot_station.adapters.fafu_arm import import_fafu_sdk

        pm, _ctrl = import_fafu_sdk()
    except Exception as exc:
        return False, str(exc)
    boards: list[str] = []
    try:
        for item in list(pm.find_likely_debug_boards()):
            port = str(getattr(item, "port", item) or "").strip()
            if port:
                boards.append(port)
    except Exception as exc:
        return True, f"已加载 fafu_motor（枚举调试板失败：{exc}）"
    if boards:
        return True, f"已加载 fafu_motor；调试板 {', '.join(boards)}"
    return True, "已加载 fafu_motor；未枚举到调试板（可先用仿真臂）"


def inspect() -> LinuxPreflight:
    result = LinuxPreflight()
    result.webview_ok, result.webview_detail = probe_webview()
    display = (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or "").strip()
    if not result.webview_ok:
        result.blockers.append(
            "未找到独立窗口组件（pywebview + WebKitGTK）。"
            "请安装 WebKitGTK / PyGObject，或使用官方 AppImage。"
            + " " + FUSE_HINT
        )
        result.fixes.append("sudo apt install gir1.2-webkit2-4.0 gir1.2-gtk-3.0 python3-gi")
        result.fixes.append(FUSE_HINT)
    elif not display:
        result.warnings.append(
            "当前没有图形 DISPLAY。源码/SSH 仍可测运动进程；独立窗口需桌面会话或官方 AppImage。"
        )
    if os.name != "nt":
        result.warnings.append(FUSE_HINT)
        result.fixes.append("sudo apt install libfuse2")
        result.fixes.append("./FAFUArmStation-x86_64.AppImage --appimage-extract-and-run")
    ports = serial_candidates()
    result.serial_ports = [str(p) for p in ports]
    result.serial_writable = any(_port_writable(p) for p in ports)
    result.udev_installed = udev_installed()
    result.modemmanager = modemmanager_active()
    if not ports:
        result.warnings.append(
            "未检测到串口（/dev/ttyUSB*、/dev/ttyACM*、/dev/fafu_debug_board）。"
            "若已插机械臂 USB，请确认调试板驱动（CH340 / CP210x / FTDI）并安装 udev 规则。"
            "软件仍会打开，可先用仿真臂。"
        )
        result.fixes.append("python3 -m robot_station.linux_preflight --install-udev")
    elif not result.serial_writable:
        result.warnings.append(
            "检测到串口但当前用户无法写入。请安装 udev 规则，或把用户加入 dialout 组后重新登录。"
        )
        result.fixes.append("python3 -m robot_station.linux_preflight --install-udev")
        result.fixes.append("sudo usermod -aG dialout $USER  # 然后重新登录")
    if result.modemmanager:
        result.warnings.append(
            "ModemManager 正在运行，可能占用 USB 串口。若连不上调试板，可执行："
            "sudo systemctl stop ModemManager"
        )
        result.fixes.append("sudo systemctl stop ModemManager")
    if not result.udev_installed and ports:
        result.warnings.append("尚未安装 FAFU 调试板 udev 规则（/dev/fafu_debug_board）。")
        result.fixes.append("python3 -m robot_station.linux_preflight --install-udev")
    should_probe = _truthy_env("STATION_PORTABLE") or bool((os.environ.get("FAFU_ARM_SDK") or "").strip())
    if should_probe:
        result.motor_ok, result.motor_detail = probe_fafu_motor()
        if not result.motor_ok:
            msg = "无法加载官方 fafu_motor（Linux 需要捆绑的 .so）。 " + result.motor_detail
            if _truthy_env("STATION_PORTABLE"):
                result.blockers.append(msg)
                result.fixes.append("请使用官方 AppImage；源码模式请用 Python 3.10 编译 fafu_motor.so")
            else:
                result.warnings.append(msg + " 源码模式仍可开窗，先用仿真臂。")
                result.fixes.append("bash fafu_arm_sdk/fafu_robot_cpp/linux/build.sh --module-only")
    return result


def _zenity(args: list[str], timeout: float = 180) -> int:
    exe = shutil.which("zenity")
    if not exe:
        return 127
    try:
        return int(
            subprocess.run(
                [exe, *args],
                timeout=timeout,
                check=False,
            ).returncode
        )
    except (OSError, subprocess.TimeoutExpired):
        return 1


def _dialog_error(text: str) -> None:
    if _zenity(["--error", f"--title={TITLE}", "--width=480", f"--text={text}"]) == 127:
        print(text, file=sys.stderr)


def _dialog_info(text: str) -> None:
    if _zenity(["--info", f"--title={TITLE}", "--width=480", f"--text={text}"]) == 127:
        print(text, file=sys.stderr)


def _dialog_question(text: str) -> bool:
    code = _zenity(
        ["--question", f"--title={TITLE}", "--width=480", "--ok-label=安装", "--cancel-label=跳过", f"--text={text}"]
    )
    if code == 127:
        print(text, file=sys.stderr)
        return False
    return code == 0


def install_udev_rules(rules: Path | None = None) -> str | None:
    """Copy bundled rules into /etc/udev/rules.d via pkexec. Returns error text or None."""
    src = rules or udev_rules_path()
    if src is None or not src.is_file():
        return "找不到 udev 规则文件"
    pkexec = shutil.which("pkexec")
    if not pkexec:
        return "未找到 pkexec，请手动：sudo cp 规则到 /etc/udev/rules.d/"
    dest = str(UDEV_DEST)
    cmd = (
        f"cp {shlex_quote(str(src))} {shlex_quote(dest)} && "
        "udevadm control --reload-rules && udevadm trigger"
    )
    try:
        proc = subprocess.run(
            [pkexec, "sh", "-c", cmd],
            timeout=120,
            check=False,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    if proc.returncode != 0:
        err = (proc.stderr or proc.stdout or "").strip() or f"exit {proc.returncode}"
        return err
    return None


def shlex_quote(value: str) -> str:
    import shlex

    return shlex.quote(value)


def run_interactive() -> int:
    """Show zenity dialogs. Return 0 to continue launching, 1 to abort."""
    result = inspect()
    if result.blockers:
        _dialog_error("\n\n".join(result.blockers) + f"\n\n{result.webview_detail}")
        return 1
    if not result.serial_ports:
        _dialog_info(result.warnings[0] if result.warnings else "未检测到串口，将进入仿真。")
    elif not result.serial_writable:
        if _dialog_question(
            "当前用户无法访问 USB 串口。\n"
            "可以现在安装调试板 udev 规则（需要管理员密码），"
            "安装后一般不必加入 dialout 组。\n\n仍要安装吗？"
        ):
            err = install_udev_rules()
            if err:
                _dialog_error(f"安装 udev 失败：{err}\n仍可先用仿真臂。")
    if result.modemmanager:
        _dialog_info(
            "检测到 ModemManager。它有时会占用调试板串口。\n"
            "若稍后无法 Connect，请执行：\n"
            "sudo systemctl stop ModemManager"
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="FAFU Linux 启动预检")
    parser.add_argument("--inspect", action="store_true", help="打印 JSON，不弹窗")
    parser.add_argument("--interactive", action="store_true", help="zenity 预检（AppRun 用）")
    parser.add_argument("--install-udev", action="store_true", help="请求 pkexec 安装 udev 规则")
    args = parser.parse_args(argv)
    if args.install_udev:
        err = install_udev_rules()
        if err:
            print(err, file=sys.stderr)
            return 1
        print("udev 已安装", UDEV_DEST)
        return 0
    if args.inspect or not args.interactive:
        result = inspect()
        dest = write_preflight_report(result)
        print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
        print("report", dest, file=sys.stderr)
        return 0
    result = inspect()
    write_preflight_report(result)
    return run_interactive()


if __name__ == "__main__":
    raise SystemExit(main())
