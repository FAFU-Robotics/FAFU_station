"""Launcher: motion process + web process. Tests use Station in-process.

Arm and cameras on the customer PC; USB serial to the arm debug board.
"""

from __future__ import annotations

import argparse
import errno
import logging
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

from robot_station.adapters.camera import CameraBank, build_camera
from robot_station.bridge import MotionClient
from robot_station.config import ROOT, StationConfig, load_config
from robot_station.lock import MOTION_LOCK, wait_lock_free
from robot_station.motion import MotionCore, run_motion_process
from robot_station.netinfo import guess_lan_ip
from robot_station.runtime import RuntimeStatus, inspect_runtime, plan_startup, should_open_browser

logger = logging.getLogger("station")

TELEOP_HOLD_S = 0.25


def _mark_teleop(owner: object) -> None:
    setattr(owner, "_teleop_mono", time.monotonic())


def _teleop_hot(owner: object) -> bool:
    last = float(getattr(owner, "_teleop_mono", 0.0) or 0.0)
    hot = (time.monotonic() - last) < TELEOP_HOLD_S
    cam = getattr(owner, "camera", None)
    setter = getattr(cam, "set_paused", None)
    if callable(setter):
        setter(hot)
    return hot


class Station:
    """In-process motion + cameras (unit tests / --in-process)."""

    def __init__(self, cfg: StationConfig) -> None:
        self.cfg = cfg
        self.camera: CameraBank = build_camera(
            cfg.camera, cfg.camera_count, cfg.camera_width, cfg.camera_height, cfg.video_hz
        )
        self.core = MotionCore(cfg, camera=self.camera)
        self._teleop_mono = 0.0

    @property
    def host_ip(self) -> str:
        return _guess_ip(self.cfg)

    @property
    def arm(self):
        return self.core.arm

    def start(self) -> None:
        self.core.start()

    def stop(self) -> None:
        self.core.stop()

    def add_client(self, cid: str) -> None:
        self.core.add_client(cid)

    def drop_client(self, cid: str) -> None:
        self.core.drop_client(cid)

    def on_touch(self, cid: str) -> None:
        self.core.on_touch(cid)

    def on_park(self, cid: str) -> None:
        self.core.on_park(cid)

    def on_arm_src(self, cid: str, kind: str) -> str | None:
        return self.core.on_arm_src(cid, kind)

    def on_estop(self, reason: str = "operator") -> None:
        self.core.on_estop(reason)

    def on_clear_estop(self, cid: str) -> str | None:
        return self.core.on_clear_estop(cid)

    def on_arm_targets(
        self, cid: str, q_deg: list[float], speed: float, stream: bool = False, t0: float | None = None
    ) -> str | None:
        if stream:
            _mark_teleop(self)
        return self.core.on_arm_targets(cid, q_deg, speed, stream=stream, t0=t0)

    def on_home(self, cid: str, speed: float) -> str | None:
        return self.core.on_home(cid, speed)

    def on_gripper(
        self, cid: str, open_: bool | None = None, deg: float | None = None, effort: int | None = None
    ) -> str | None:
        return self.core.on_gripper(cid, open_, deg, effort)

    def on_mode(self, cid: str, mode: str) -> str | None:
        return self.core.on_mode(cid, mode)

    def on_teach(self, cid: str, q_deg: list[float], t0: float | None = None) -> str | None:
        _mark_teleop(self)
        return self.core.on_teach(cid, q_deg, t0=t0)

    def on_cartesian(
        self, cid: str, dxyz: list[float], drpy_deg: list[float], t0: float | None = None, speed: float | None = None
    ) -> str | None:
        if speed is not None:
            setter = getattr(self.core.arm, "set_teleop_speed", None)
            if callable(setter):
                setter(float(speed))
        _mark_teleop(self)
        return self.core.on_cartesian(cid, dxyz, drpy_deg, t0=t0)

    def on_cartesian_pose(
        self, cid: str, xyz_m: list[float], rpy_deg: list[float], speed: float, t0: float | None = None
    ) -> str | None:
        return self.core.on_cartesian_pose(cid, xyz_m, rpy_deg, speed, t0=t0)

    def on_path(
        self, cid: str, waypoints: list[list[float]], speed: float, durations: list[float] | None = None
    ) -> str | None:
        return self.core.on_path(cid, waypoints, speed, durations=durations)

    def on_link(self, cid: str, action: str) -> str | None:
        return self.core.on_link(cid, action)

    def on_script(self, cid: str, name: str) -> str | None:
        return self.core.on_script(cid, name)

    def on_power(self, cid: str, on: bool) -> str | None:
        return self.core.on_power(cid, on)

    def note_rtt_ms(self, ms: float) -> None:
        self.core.note_rtt_ms(ms)

    def snapshot(self) -> dict:
        return self.core.snapshot()

    def wait_snap(self, last_seq: int, timeout_s: float = 0.25, decorate: bool = True) -> tuple[int, dict]:
        return self.core.wait_snap(last_seq, timeout_s)

    def teleop_hot(self) -> bool:
        return _teleop_hot(self)


class WebFront:
    """Web process: cameras + TCP to motion. Same methods as Station for the HTTP server."""

    def __init__(self, cfg: StationConfig) -> None:
        self.cfg = cfg
        self.camera: CameraBank = build_camera(
            cfg.camera, cfg.camera_count, cfg.camera_width, cfg.camera_height, cfg.video_hz
        )
        self.motion = MotionClient(cfg.motion_host, cfg.motion_port)
        self.clients: set[str] = set()
        self._rtt: float | None = None
        self._lock = threading.Lock()
        self._teleop_mono = 0.0

    @property
    def host_ip(self) -> str:
        return _guess_ip(self.cfg)

    def start(self) -> None:
        self.camera.start()
        self.motion.connect(timeout_s=15.0)

    def stop(self) -> None:
        try:
            self.camera.stop()
        except Exception:
            logger.exception("停相机")
        self.motion.close()

    def add_client(self, cid: str) -> None:
        with self._lock:
            self.clients.add(cid)
        self.motion.send({"t": "add", "cid": cid})

    def drop_client(self, cid: str) -> None:
        with self._lock:
            self.clients.discard(cid)
        self.motion.send({"t": "drop", "cid": cid})

    def on_touch(self, cid: str) -> None:
        self.motion.send({"t": "touch", "cid": cid})

    def on_park(self, cid: str) -> None:
        self.motion.send({"t": "park", "cid": cid})

    def on_arm_src(self, cid: str, kind: str) -> str | None:
        ack = self.motion.call({"t": "arm_src", "cid": cid, "mode": kind})
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "切换臂源失败")

    def on_estop(self, reason: str = "operator") -> None:
        self.motion.send({"t": "e", "reason": reason})

    def on_clear_estop(self, cid: str) -> str | None:
        ack = self.motion.call({"t": "clear", "cid": cid})
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "解除失败")

    def on_arm_targets(
        self, cid: str, q_deg: list[float], speed: float, stream: bool = False, t0: float | None = None
    ) -> str | None:
        msg = {"t": "arm", "cid": cid, "q": q_deg, "speed": speed, "stream": stream}
        if t0 is not None:
            msg["t0"] = float(t0)
        if stream:
            _mark_teleop(self)
            self.motion.send(msg)
            return None
        ack = self.motion.call(msg)
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "下发失败")

    def on_home(self, cid: str, speed: float) -> str | None:
        ack = self.motion.call({"t": "home", "cid": cid, "speed": speed}, timeout_s=2.5)
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "复位失败")

    def on_gripper(
        self, cid: str, open_: bool | None = None, deg: float | None = None, effort: int | None = None
    ) -> str | None:
        msg: dict = {"t": "grip", "cid": cid}
        if open_ is not None:
            msg["open"] = bool(open_)
        if deg is not None:
            msg["deg"] = float(deg)
        if effort is not None:
            msg["effort"] = int(effort)
        ack = self.motion.call(msg)
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "夹爪失败")

    def on_mode(self, cid: str, mode: str) -> str | None:
        ack = self.motion.call({"t": "mode", "cid": cid, "mode": mode})
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "切模式失败")

    def on_teach(self, cid: str, q_deg: list[float], t0: float | None = None) -> str | None:
        msg = {"t": "teach", "cid": cid, "q": q_deg}
        if t0 is not None:
            msg["t0"] = float(t0)
        _mark_teleop(self)
        self.motion.send(msg)
        return None

    def on_cartesian(
        self, cid: str, dxyz: list[float], drpy_deg: list[float], t0: float | None = None, speed: float | None = None
    ) -> str | None:
        msg = {"t": "cart", "cid": cid, "dxyz": dxyz, "drpy": drpy_deg}
        if t0 is not None:
            msg["t0"] = float(t0)
        if speed is not None:
            msg["speed"] = float(speed)
        _mark_teleop(self)
        self.motion.send(msg)
        return None

    def on_cartesian_pose(
        self, cid: str, xyz_m: list[float], rpy_deg: list[float], speed: float, t0: float | None = None
    ) -> str | None:
        msg: dict = {"t": "cart_go", "cid": cid, "xyz": xyz_m, "rpy": rpy_deg, "speed": speed}
        if t0 is not None:
            msg["t0"] = float(t0)
        ack = self.motion.call(msg)
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "笛卡尔目标失败")

    def on_path(
        self, cid: str, waypoints: list[list[float]], speed: float, durations: list[float] | None = None
    ) -> str | None:
        ack = self.motion.call({"t": "path", "cid": cid, "q": waypoints, "speed": speed, "dt": durations})
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "路点失败")

    def on_link(self, cid: str, action: str) -> str | None:
        ack = self.motion.call({"t": "link", "cid": cid, "action": action})
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "连接失败")

    def on_script(self, cid: str, name: str) -> str | None:
        ack = self.motion.call({"t": "script", "cid": cid, "name": name})
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "脚本失败")

    def on_power(self, cid: str, on: bool) -> str | None:
        ack = self.motion.call({"t": "power", "cid": cid, "on": on})
        if ack.get("ok"):
            return None
        return str(ack.get("error") or "使能失败")

    def note_rtt_ms(self, ms: float) -> None:
        self._rtt = float(ms)
        self.motion.send({"t": "rtt", "ms": ms})

    def snapshot(self) -> dict:
        return self._decorate(self.motion.latest())

    def wait_snap(self, last_seq: int, timeout_s: float = 0.25, decorate: bool = True) -> tuple[int, dict]:
        seq, data = self.motion.wait_snap(last_seq, timeout_s)
        if decorate:
            return seq, self._decorate(data)
        return seq, data

    def _decorate(self, data: dict) -> dict:
        if not data:
            data = {}
        else:
            data = dict(data)
        cam = self.camera.poll()
        data["camera"] = {
            "online": list(cam.online),
            "fps": [round(x, 1) for x in cam.fps],
            "roles": list(cam.roles),
            "backend": cam.backend,
            "device": cam.device,
            "reason": cam.reason,
        }
        with self._lock:
            n = len(self.clients)
            data["clients"] = n
            if self._rtt is not None:
                data["rtt_ms"] = round(self._rtt, 2)
        return data

    def teleop_hot(self) -> bool:
        return _teleop_hot(self)


def _guess_ip(cfg: StationConfig | None = None) -> str:
    """Address printed in logs. Default delivery is localhost."""
    listen = (cfg.listen_host if cfg is not None else "") or ""
    if listen and listen not in ("0.0.0.0", "::", "[::]"):
        return listen
    ip = guess_lan_ip()
    if ip and not ip.startswith("127."):
        return ip
    return "127.0.0.1"


def _setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
        datefmt="%H:%M:%S",
    )


def _pids_on_port(port: int) -> list[int]:
    found: list[int] = []

    def _add(pid: int) -> None:
        if pid > 1 and pid not in found:
            found.append(pid)

    if os.name == "nt":
        try:
            out = subprocess.check_output(["netstat", "-ano", "-p", "tcp"], text=True, timeout=3)
        except Exception:
            return found
        needle = f":{int(port)}"
        for line in out.splitlines():
            upper = line.upper()
            if "LISTENING" not in upper and "侦听" not in line:
                continue
            if needle not in line:
                continue
            parts = line.split()
            if not parts:
                continue
            try:
                _add(int(parts[-1]))
            except ValueError:
                continue
        return found

    try:
        out = subprocess.check_output(["ss", "-ltnp"], text=True, timeout=2)
    except Exception:
        out = ""
    needle = f":{int(port)} "
    for line in out.splitlines():
        if needle not in line and not line.rstrip().endswith(f":{int(port)}"):
            continue
        for match in re.finditer(r"pid=(\d+)", line):
            _add(int(match.group(1)))
    if found:
        return found
    try:
        proc = subprocess.run(
            ["fuser", f"{int(port)}/tcp"],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
        blob = (proc.stdout or "") + " " + (proc.stderr or "")
        for part in re.findall(r"\d+", blob):
            pid = int(part)
            if pid == int(port):
                continue
            _add(pid)
    except Exception:
        pass
    return found


def _kill_pid(pid: int, force: bool = False) -> None:
    if os.name == "nt":
        cmd = ["taskkill", "/PID", str(pid)]
        if force:
            cmd.append("/F")
        try:
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
                check=False,
            )
        except Exception:
            pass
        return
    try:
        os.kill(pid, signal.SIGKILL if force else signal.SIGTERM)
    except OSError:
        pass


def wait_port_closed(port: int, timeout_s: float = 3.0) -> bool:
    deadline = time.monotonic() + max(0.05, float(timeout_s))
    while time.monotonic() < deadline:
        if not port_open(port, "127.0.0.1"):
            return True
        time.sleep(0.05)
    return not port_open(port, "127.0.0.1")


def replace_listener(port: int) -> None:
    me = os.getpid()
    parent = os.getppid()
    for pid in _pids_on_port(port):
        if pid in (me, parent, 1):
            continue
        logger.warning("端口 %s 已被 pid %s 占用，按 --replace 结束它", port, pid)
        _kill_pid(pid, force=False)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and _pids_on_port(port):
        time.sleep(0.05)
    for pid in _pids_on_port(port):
        if pid in (me, parent, 1):
            continue
        _kill_pid(pid, force=True)
    if os.name != "nt":
        try:
            subprocess.run(
                ["fuser", "-k", f"{int(port)}/tcp"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2,
                check=False,
            )
        except Exception:
            pass
    wait_port_closed(port, 2.0)


def wait_port(host: str, port: int, timeout_s: float, proc: subprocess.Popen | None = None) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            return False
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.2)
        try:
            if sock.connect_ex((host, int(port))) == 0:
                return True
        finally:
            sock.close()
        time.sleep(0.1)
    return False


def motion_argv(cfg: StationConfig, config_path: Path | None) -> list[str]:
    """Child must get ``--arm``: yaml stays ``mock``, parent override would otherwise be lost."""
    cmd = [sys.executable, "-m", "robot_station.service", "--motion-only", "--no-browser"]
    if config_path is not None:
        cmd += ["--config", str(config_path)]
    kind = (cfg.arm or "mock").strip().lower() or "mock"
    cmd += ["--arm", kind]
    if cfg.arm_allow_motion:
        cmd.append("--allow-motion")
    return cmd


def spawn_motion(cfg: StationConfig, config_path: Path | None) -> subprocess.Popen:
    cmd = motion_argv(cfg, config_path)
    from robot_station.portable import child_env

    env = child_env(ROOT)
    logger.info("拉起运动进程 arm=%s", cfg.arm)
    return subprocess.Popen(cmd, cwd=str(ROOT), env=env)


def _stop_proc(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=1.0)


def _announce(cfg: StationConfig, host_ip: str) -> tuple[str, str]:
    local = f"http://127.0.0.1:{cfg.http_port}"
    lan = f"http://{host_ip}:{cfg.http_port}"
    return local, lan


def _maybe_open_browser(cfg: StationConfig, url: str, status: RuntimeStatus) -> None:
    if not should_open_browser(cfg, status):
        return
    from robot_station.web.server import _open_browser

    _open_browser(url, wait=True)


def _replace_old(cfg: StationConfig) -> None:
    logger.warning("按 --replace 结束旧站控。")
    replace_listener(cfg.http_port)
    replace_listener(cfg.motion_port)
    if not wait_lock_free(MOTION_LOCK, 3.0):
        logger.warning("运动锁仍被占用，仍尝试启动")
    wait_port_closed(cfg.http_port, 2.0)
    wait_port_closed(cfg.motion_port, 2.0)
    time.sleep(0.2)


def main(argv: list[str] | None = None) -> int:
    from robot_station.portable import apply_bundled_runtime

    apply_bundled_runtime()
    _setup_logging()
    parser = argparse.ArgumentParser(description="FAFU 机械臂站控")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument(
        "--replace",
        action="store_true",
        default=False,
        help="结束占用 9400/9470 的旧站控再启动",
    )
    parser.add_argument("--no-replace", action="store_false", dest="replace")
    parser.add_argument("--no-browser", action="store_true", help="不要自动打开本机浏览器")
    parser.add_argument(
        "--service",
        action="store_true",
        help="无界面：不弹浏览器；若站控已在别的进程里跑则失败退出（不要 attach）",
    )
    parser.add_argument("--motion-only", action="store_true", help="只跑运动进程（内部用）")
    parser.add_argument("--web-only", action="store_true", help="只跑网页（运动进程须已在）")
    parser.add_argument("--in-process", action="store_true", help="单进程（调试）；真车不要用")
    parser.add_argument(
        "--arm",
        choices=["mock", "sim", "fafu", "auto"],
        default=None,
        help="覆盖配置里的 arm（sim=进程内假控制器，不开串口）",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="允许真臂下发运动（仍须 STATION_ALLOW_LIVE_ARM=1 才能开串口）",
    )
    parser.add_argument(
        "--camera",
        choices=["mock", "auto", "live", "usb", "realsense"],
        default=None,
        help="覆盖配置里的 camera（auto=识别 RealSense 作业相机，跳过笔记本内置摄像头）",
    )
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    if args.arm:
        cfg.arm = args.arm
    if args.camera:
        cfg.camera = args.camera
    if args.allow_motion:
        cfg.arm_allow_motion = True
    from robot_station.serial_guard import apply_live_arm_policy

    cfg.arm = apply_live_arm_policy(cfg.arm)
    if args.host:
        cfg.listen_host = args.host
    if args.port:
        cfg.http_port = args.port
    if args.service:
        args.no_browser = True
    if args.no_browser:
        cfg.open_browser = False
    if os.environ.get("STATION_NO_BROWSER"):
        cfg.open_browser = False

    stop = threading.Event()

    def _quit(*_args: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, _quit)
    signal.signal(signal.SIGTERM, _quit)

    status = inspect_runtime(cfg)
    if (
        status.lock_held
        and not status.http_up
        and not status.motion_up
        and not args.replace
        and not args.motion_only
    ):
        logger.info("检测到站控可能正在启动，等它起来而不另开一份…")
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            status = inspect_runtime(cfg)
            if status.http_up and status.motion_up:
                break
    plan = plan_startup(
        status,
        replace=args.replace,
        web_only=args.web_only,
        motion_only=args.motion_only,
        in_process=args.in_process,
    )
    logger.info("%s", plan.reason)

    if plan.action == "refuse":
        logger.error("%s", plan.reason)
        local, lan = _announce(cfg, _guess_ip(cfg))
        if status.http_up:
            logger.info("若只是想打开页面：本机 %s", local)
            if lan != local:
                logger.info("局域网 %s", lan)
        return 1

    if plan.action == "attach":
        if args.service:
            logger.error(
                "站控已在别的进程里跑。无界面模式不会 attach。"
                "请先关掉已有窗口，或去掉 --service 直接打开已有页面。"
            )
            return 1
        local, lan = _announce(cfg, _guess_ip(cfg))
        logger.info("站控已经在跑。不会再开一份。")
        logger.info("机械臂： %s  或  %s", local, lan)
        _maybe_open_browser(cfg, local, status)
        return 0

    if plan.action == "replace_then_start":
        _replace_old(cfg)
        status = inspect_runtime(cfg)

    if args.motion_only:
        cfg.open_browser = False
        try:
            run_motion_process(cfg, stop)
        except RuntimeError as exc:
            logger.error("%s", exc)
            return 1
        return 0

    want_web_only = args.web_only or plan.action == "web_only"
    if want_web_only and not args.in_process:
        args.web_only = True

    if not should_open_browser(cfg, status):
        cfg.open_browser = False

    from robot_station.web.server import serve_http

    motion_proc: subprocess.Popen | None = None
    if args.in_process:
        front: Station | WebFront = Station(cfg)
    else:
        if not args.web_only:
            motion_proc = spawn_motion(cfg, args.config)
            if not wait_port(cfg.motion_host, cfg.motion_port, 20.0, motion_proc):
                if motion_proc.poll() is not None:
                    logger.error("运动进程启动失败，退出码 %s。看上面的日志。", motion_proc.returncode)
                else:
                    logger.error("运动进程没有在 %s:%s 听起来", cfg.motion_host, cfg.motion_port)
                _stop_proc(motion_proc)
                return 1
        front = WebFront(cfg)

    try:
        front.start()
    except Exception:
        logger.exception("启动失败")
        _stop_proc(motion_proc)
        return 1

    try:
        serve_http(front, cfg, stop)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            logger.error(
                "端口 %s 已被占用。若站控已在跑，直接打开网页即可；确认没人用再：python run_station.py --replace",
                cfg.http_port,
            )
            front.stop()
            _stop_proc(motion_proc)
            return 1
        raise
    finally:
        front.stop()
        _stop_proc(motion_proc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
