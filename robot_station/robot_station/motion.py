"""Motion process: 100 Hz tick, arm adapter."""

from __future__ import annotations

import json
import functools
import logging
import math
import os
import secrets
import socket
import threading
import time
from typing import Any

from robot_station.adapters.arm import ArmAdapter, DisconnectedArm, build_arm
from robot_station.adapters.camera import CameraBank
from robot_station.config import StationConfig
from robot_station.connection import connection_kind, target_backend, serial_endpoint, needs_session, rejected
from robot_station.safety import Safety, SafetyGate
from robot_station.runtime import freeze_boot_code_rev
from robot_station.telem import pack_telem
from robot_station.world import ArmSnap, World
from robot_station.programs import ProgramRunner, WorkbenchStore, format_script, parse_script, validate_tree, number

logger = logging.getLogger("station.motion")

# Written to cmd_err when one 100 Hz tick raises. Kept as a constant so the
# recovery path can recognise and clear exactly this message.
TICK_ERR_TEXT = "运动拍异常，已跳过并继续"
TICK_ERR_LOG_EVERY_S = 5.0
ENDPOINT_HOLD_TIMEOUT_S = 0.35
ENDPOINT_TOLERANCE_DEG = 0.25


def _is_vendor_enable_noise(text: str) -> bool:
    low = (text or "").lower()
    return "motor_reset" in low or "enable failed" in low or "使能失败" in (text or "")


def _live_joints_ok(arm: Any) -> bool:
    if arm is None:
        return False
    if bool(getattr(arm, "enabled", False)):
        return True
    motors = getattr(arm, "motors", None) or []
    n = 0
    for row in motors:
        if str(row.get("name") or "") == "夹爪":
            continue
        if row.get("online"):
            n += 1
    return n >= 6

_WINMM = None
_WINMM_REFS = 0
_WINMM_LOCK = threading.Lock()


def _win_timer_begin() -> bool:
    """Ask Windows for 1 ms sleep resolution so a 100 Hz tick is actually 100 Hz."""
    global _WINMM, _WINMM_REFS
    if os.name != "nt":
        return False
    try:
        import ctypes

        with _WINMM_LOCK:
            if _WINMM is None:
                _WINMM = ctypes.windll.winmm
            if _WINMM_REFS == 0:
                _WINMM.timeBeginPeriod(1)
            _WINMM_REFS += 1
        return True
    except Exception:
        return False


def _win_timer_end() -> None:
    global _WINMM_REFS
    with _WINMM_LOCK:
        if _WINMM is None or _WINMM_REFS <= 0:
            return
        _WINMM_REFS -= 1
        if _WINMM_REFS == 0:
            try:
                _WINMM.timeEndPeriod(1)
            except Exception:
                pass


def _linux_rt_begin() -> bool:
    """Best-effort SCHED_FIFO for the 100 Hz tick. AppImage cannot reliably setcap."""
    if os.name == "nt":
        return False
    try:
        policy = int(getattr(os, "SCHED_FIFO"))
        param = os.sched_param(10)
        os.sched_setscheduler(0, policy, param)
        logger.info("Linux SCHED_FIFO 已启用（prio=10）")
        return True
    except (AttributeError, PermissionError, OSError) as exc:
        logger.info("Linux SCHED_FIFO 不可用，沿用普通调度: %s", exc)
        return False


def _linux_rt_end() -> None:
    if os.name == "nt":
        return
    try:
        other = int(getattr(os, "SCHED_OTHER"))
        os.sched_setscheduler(0, other, os.sched_param(0))
    except (AttributeError, PermissionError, OSError):
        return


def _sleep_until(deadline: float) -> None:
    """Sleep until perf_counter deadline. Last ~0.3 ms spins so 100 Hz does not slip."""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > 0.001:
            time.sleep(remaining - 0.0007)
        elif remaining > 0.0003:
            time.sleep(0)
        else:
            while time.perf_counter() < deadline:
                pass
            return


def manual_control(fn):
    @functools.wraps(fn)
    def guarded(self, *args, **kwargs):
        if self._arm_switching:
            return "连接操作正在进行，请等待完成"
        with self._arm_lock:
            if self._link_disconnected and fn.__name__ != "on_clear_estop":
                return "设备未连接，请先连接目标设备"
            if self.program.active:
                return "程序占用机械臂，请先停止程序再进行手动操作"
            self._stop_endpoint_hold()
            return fn(self, *args, **kwargs)
    return guarded


class MotionCore:
    def __init__(self, cfg: StationConfig, camera: CameraBank | None = None) -> None:
        self.cfg = cfg
        self.world = World()
        self.world.code_rev = freeze_boot_code_rev()
        self.gate = SafetyGate(cfg.watchdog_s)
        self.arm: ArmAdapter = build_arm(
            cfg.arm,
            cfg.arm_joints,
            cfg.arm_speed_deg_s,
            cfg.arm_has_gripper,
            sdk=cfg.arm_sdk,
            cfg_path=cfg.arm_cfg,
            port=cfg.arm_port,
            allow_motion=cfg.arm_allow_motion,
            gripper_id=cfg.arm_gripper_id,
        )
        # Publish an honest offline snap before the first poll. The dataclass
        # default is a connected mock arm, and telemetry sends seq 0 immediately.
        n = int(cfg.arm_joints)
        self.world.arm = ArmSnap(
            online=False,
            enabled=False,
            backend=str(getattr(self.arm, "backend", "") or cfg.arm or "offline"),
            n=n,
            q_deg=[0.0] * n,
            target_deg=[0.0] * n,
            ok=[False] * n,
        )
        self.camera = camera
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._tick_th: threading.Thread | None = None
        self._cmd_hits = 0
        self._cmd_t = time.monotonic()
        self.clients: set[str] = set()
        self._had_clients = False
        self._cart_dxyz = [0.0, 0.0, 0.0]
        self._cart_drpy = [0.0, 0.0, 0.0]
        self._cart_pending = False
        self._cart_follow_owner: str | None = None
        self._ik_err = ""
        self._cmd_err = ""
        self._tick_err_count = 0
        self._tick_err_last_log = -1e9
        self._pub = threading.Condition()
        self._timer_armed = False
        self._rt_armed = False
        self._arm_lock = threading.RLock()
        self._arm_switching = False
        self._connection_session = secrets.token_hex(12)
        self._session_required = False
        self._link_disconnected = False
        self._enable_required = False
        self._publish_connection("disconnected")
        self._watch: Any = None
        self.program = ProgramRunner()
        self._program_owner = ""
        self._program_touch = 0.0
        self._program_scale = 1.0
        self._endpoint_hold: dict | None = None

    def _note_echo(self, t0: object) -> None:
        if t0 is None or t0 == "":
            return
        try:
            self.world.echo_t0 = float(t0)
        except (TypeError, ValueError):
            return

    def start(self) -> None:
        self._session_required = True
        serial = self.arm_kind() == "fafu"
        if not serial or self.cfg.arm_connect_on_start:
            try:
                self.arm.start()
                snap = self.arm.poll()
                if snap.online:
                    self._prepare_connection()
                    if serial and snap.serial_port:
                        self.cfg.arm_port = snap.serial_port
                    self._link_disconnected = False
                    with self._lock:
                        self.world.arm = snap
                    self._publish_connection("connected")
                else:
                    self._link_disconnected = True
                    self._publish_connection("error", snap.link_err or "设备未连接")
            except Exception as exc:
                self._link_disconnected = True
                self._publish_connection("error", str(exc))
                logger.error("连接失败，保留当前目标（不切换仿真）: %s", exc)
        else:
            self._link_disconnected = True
            self._publish_connection("disconnected")
        try:
            if self.camera is not None:
                self.camera.start()
        except Exception:
            self.arm.stop()
            raise
        self._stop.clear()
        self._timer_armed = _win_timer_begin()
        self._rt_armed = _linux_rt_begin()
        self._tick_th = threading.Thread(target=self._tick_loop, name="station-tick", daemon=True)
        self._tick_th.start()
        self._start_watch()
        logger.info("tick %s Hz 臂=%s（连接与使能独立）", self.cfg.control_hz, self.arm_kind())



    # ---- USB 热插拔监视（默认关闭；cfg.arm_watch / camera_watch 打开） ----
    #
    # 只在设备签名变化时才去认硬件，见 robot_station/usb_watch.py。默认关闭
    # 意味着 MotionCore 的行为与从前完全一致（单测、arm: mock 开发机不受影响）。

    def _start_watch(self) -> None:
        try:
            from robot_station.usb_watch import UsbWatcher, watch_enabled

            arm_watch, camera_watch = watch_enabled(self.cfg)
            local_camera = bool(camera_watch) and self.camera is not None
            if not (arm_watch or local_camera):
                return
            from robot_station.usb_watch import camera_signature

            self._watch = UsbWatcher(
                on_arm=self._watch_arm if arm_watch else None,
                on_camera=self._watch_camera if local_camera else None,
                camera_fn=camera_signature if local_camera else None,
            )
            self._watch.start()
            logger.info(
                "USB 监视已开启（臂=%s 相机=%s）：仅在设备变化时探测",
                arm_watch,
                camera_watch,
            )
        except Exception:
            logger.warning("USB 监视启动失败，站控照常运行", exc_info=True)
            self._watch = None

    def _stop_watch(self) -> None:
        watch = self._watch
        self._watch = None
        if watch is None:
            return
        try:
            watch.stop(timeout=1.0)
        except Exception:
            logger.warning("USB 监视停止失败", exc_info=True)

    def note_device_change(self) -> None:
        """Tell the watcher to re-read now, without waiting for its interval.

        Discovery only records changes; it cannot replace a connection target.
        """
        watch = self._watch
        if watch is None:
            return
        try:
            watch.rescan()
        except Exception:
            logger.debug("USB 监视重读失败", exc_info=True)

    def _watch_arm(self, attached: bool) -> None:
        """Discovery is informational; it never changes the active endpoint."""
        logger.info("USB 设备已%s；保留当前连接目标，等待用户选择串口并连接", "插入" if attached else "拔出")

    def _watch_camera(self, attached: bool) -> None:
        """USB camera set changed: re-run the work-camera pick and swap backend."""
        camera = self.camera
        rescan = getattr(camera, "rescan", None)
        if not callable(rescan):
            return
        try:
            before = getattr(getattr(camera, "_inner", None), "device", None)
            bank = rescan()
            after = getattr(bank, "device", None)
        except Exception:
            logger.warning("相机重新识别失败", exc_info=True)
            return
        if before != after:
            logger.info("作业相机已更新：%s → %s", before or "假画面", after or "假画面")
        elif attached:
            logger.debug("相机变化未产生新后端，沿用 %s", after or "假画面")

    def stop(self) -> None:
        self._stop.set()
        self._stop_watch()
        with self._pub:
            self._pub.notify_all()
        if self._tick_th is not None:
            self._tick_th.join(timeout=1.0)
        if self._timer_armed:
            _win_timer_end()
            self._timer_armed = False
        if self._rt_armed:
            _linux_rt_end()
            self._rt_armed = False
        try:
            self.arm.hold()
        except Exception:
            logger.exception("halt on stop")
        if self.camera is not None:
            self.camera.stop()
        self.arm.stop()

    def arm_kind(self) -> str:
        return str(getattr(self.arm, "backend", "") or self.cfg.arm or "mock")

    def on_arm_src(self, cid: str, kind: str) -> str | None:
        """Legacy entry point; the same disconnect-before-connect rule applies."""
        return self.on_link(cid, "connect", kind, self.cfg.arm_port or None)

    def _seed_sim_pose(self, q_deg: list[float]) -> None:
        if not q_deg:
            return
        teach = getattr(self.arm, "teach_pose", None)
        if callable(teach):
            try:
                teach(q_deg)
            except Exception:
                logger.exception("仿真臂同步姿态失败")

    def add_client(self, cid: str) -> None:
        with self._lock:
            self.clients.add(cid)

    def drop_client(self, cid: str) -> None:
        with self._lock:
            was_registered = cid in self.clients
            self.clients.discard(cid)
            empty = not self.clients
        with self._arm_lock:
            if self._endpoint_hold and self._endpoint_hold["owner"] == cid:
                self._stop_endpoint_hold()
        if self.program.active and cid == self._program_owner:
            self.program.stop("程序控制窗口已断开")
            self.arm.hold(abort_path=True)
        if empty:
            with self._arm_lock:
                self.arm.hold(abort_path=True)
                self._drop_cart()
                if was_registered:
                    self._connection_session = secrets.token_hex(12)
                    connection = self.world.connection
                    self._publish_connection(connection.get("status", "disconnected"), connection.get("error", ""))

    def _note(self) -> None:
        with self._lock:
            self._cmd_hits += 1
        self.gate.note_command()

    def on_touch(self, cid: str) -> None:
        if cid == self._program_owner:
            self._program_touch = time.monotonic()
        self._note()

    def on_park(self, cid: str) -> None:
        if self.program.active or self._endpoint_hold:
            return
        if self._cart_follow_owner == cid:
            self._cart_follow_owner = None
            self.arm.hold(abort_path=True)
            return
        park = getattr(self.arm, "park_stream", None)
        if callable(park):
            park()

    def on_estop(self, reason: str = "operator") -> None:
        self._endpoint_hold = None
        self.arm.emergency_stop()
        self.gate.estop(reason)
        if self.program.active:
            self.program.stop("急停已触发，程序停止")
        with self._lock:
            self.world.safety = self.gate.state.name
            self.world.safety_reason = self.gate.reason

    @manual_control
    def on_clear_estop(self, cid: str) -> str | None:
        self.gate.clear()
        self.arm.resume_motion()
        with self._lock:
            self.world.safety = self.gate.state.name
            self.world.safety_reason = self.gate.reason
        return None

    @manual_control
    def on_arm_targets(
        self, cid: str, q_deg: list[float], speed: float, stream: bool = False, t0: float | None = None
    ) -> str | None:
        self._note_echo(t0)
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if self.arm_mode() in ("Gravity", "Gra+Fri"):
            return "请先切回 Position 再发送位置"
        if self.arm_mode() == "Impedance" and self._sdk_float_active():
            return "阻抗环运行中请先切回 Position"
        self._drop_cart()
        err = self.arm.apply_targets(q_deg, speed, stream=stream)
        if err:
            return err
        self._note()
        return None

    def _sdk_float_active(self) -> bool:
        fn = getattr(self.arm, "_float_writer_active", None)
        if callable(fn):
            return bool(fn())
        th = getattr(self.arm, "_grav_th", None)
        return bool(th is not None and th.is_alive())

    def arm_mode(self) -> str:
        return str(getattr(self.arm, "_ctrl_mode", None) or "Position")

    @manual_control
    def on_mode(self, cid: str, mode: str) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        err = self.arm.set_ctrl_mode(mode)
        if err:
            return err
        self._drop_cart()
        self._note()
        return None

    @manual_control
    def on_grav_urdf(self, cid: str, name: str) -> str | None:
        if self.arm.poll().recording:
            return "录制期间不能切换重力模型，请先停止录制"
        fn = getattr(self.arm, "set_grav_urdf", None)
        if not callable(fn):
            return "此后端不支持选择 URDF"
        err = fn(name)
        if err:
            return err
        self._note()
        return None

    @manual_control
    def on_teach(self, cid: str, q_deg: list[float], t0: float | None = None) -> str | None:
        self._note_echo(t0)
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if self.arm_mode() not in ("Gravity", "Gra+Fri"):
            return "示教仅在 Gravity / Gra+Fri 下可用"
        err = self.arm.teach_pose(q_deg)
        if err:
            return err
        self._note()
        return None

    @manual_control
    def on_cartesian(
        self, cid: str, dxyz: list[float], drpy_deg: list[float], t0: float | None = None
    ) -> str | None:
        self._note_echo(t0)
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if self.arm_mode() not in ("Position", "Impedance"):
            return "请先切回 Position 再用键盘"
        if self.arm_mode() == "Impedance" and self._sdk_float_active():
            return "阻抗环运行中请先切回 Position"
        with self._lock:
            self._cart_follow_owner = None
            for i in range(3):
                self._cart_dxyz[i] += float(dxyz[i]) if i < len(dxyz) else 0.0
                self._cart_drpy[i] += float(drpy_deg[i]) if i < len(drpy_deg) else 0.0
            self._cart_pending = True
        self._note()
        return None

    @manual_control
    def on_cartesian_pose(
        self,
        cid: str,
        xyz_m: list[float],
        rpy_deg: list[float],
        speed: float,
        t0: float | None = None,
        *, stream: bool = False,
    ) -> str | None:
        self._note_echo(t0)
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if self.arm_mode() not in ("Position", "Impedance"):
            return "请先切回 Position 再发送笛卡尔目标"
        if self.arm_mode() == "Impedance" and self._sdk_float_active():
            return "阻抗环运行中请先切回 Position"
        self._drop_cart()
        err = self.arm.apply_cartesian_pose(xyz_m, rpy_deg, speed)
        if err:
            self._ik_err = err
            return err
        self._ik_err = ""
        self._cart_follow_owner = cid if stream else None
        self._note()
        return None

    def _drop_cart(self) -> None:
        with self._lock:
            self._cart_follow_owner = None
            self._cart_pending = False
            self._cart_dxyz = [0.0, 0.0, 0.0]
            self._cart_drpy = [0.0, 0.0, 0.0]

    def _flush_cart(self) -> None:
        with self._lock:
            if not self._cart_pending:
                return
            dxyz = self._cart_dxyz[:]
            drpy = self._cart_drpy[:]
            self._cart_dxyz = [0.0, 0.0, 0.0]
            self._cart_drpy = [0.0, 0.0, 0.0]
            self._cart_pending = False
        if all(abs(v) < 1e-12 for v in dxyz + drpy):
            return
        err = self.arm.apply_cartesian(dxyz, drpy)
        self._ik_err = err or ""

    @manual_control
    def on_path(
        self,
        cid: str,
        waypoints: list[list[float]],
        speed: float,
        durations: list[float] | None = None,
    ) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if self.arm_mode() != "Position":
            return "请先切回 Position 再跑路点"
        self._drop_cart()
        err = self.arm.run_path(waypoints, speed, durations_s=durations)
        if err:
            return err
        self._note()
        return None

    @manual_control
    def on_rec_start(self, cid: str, name: str | None = None, teach: str | None = None) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        fn = getattr(self.arm, "record_start", None)
        if not callable(fn):
            return "此后端不支持连续录制"
        try:
            err = fn(name, teach=teach)
        except TypeError:
            err = fn(name)
        if err:
            return err
        self._drop_cart()
        self._note()
        return None

    def on_rec_delete(self, cid: str, path: str) -> str | None:
        fn = getattr(self.arm, "delete_traj", None)
        if not callable(fn):
            from robot_station.traj import delete_recording

            try:
                delete_recording(path)
            except FileNotFoundError as exc:
                return str(exc)
            except OSError as exc:
                return f"删除失败: {exc}"
            return None
        return fn(path)

    def on_rec_rename(self, cid: str, path: str, dest: str) -> str | None:
        fn = getattr(self.arm, "rename_traj", None)
        if not callable(fn):
            from robot_station.traj import rename_recording

            try:
                rename_recording(path, dest)
            except FileNotFoundError as exc:
                return str(exc)
            except FileExistsError as exc:
                return str(exc)
            except ValueError as exc:
                return str(exc)
            except OSError as exc:
                return f"重命名失败: {exc}"
            return None
        return fn(path, dest)

    def on_rec_stop(self, cid: str) -> str | None:
        fn = getattr(self.arm, "record_stop", None)
        if not callable(fn):
            return None
        return fn()

    @manual_control
    def on_replay(
        self,
        cid: str,
        path: str,
        rate: float = 1.0,
        speed: float | None = None,
    ) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        fn = getattr(self.arm, "replay_start", None)
        if not callable(fn):
            return "此后端不支持轨迹回放"
        spd = float(speed if speed is not None else self.cfg.arm_speed_deg_s)
        self._drop_cart()
        err = fn(path, rate=float(rate), speed_deg_s=spd)
        if err:
            return err
        self._note()
        return None

    @manual_control
    def on_home(self, cid: str, speed: float) -> str | None:
        if self.gate.state == Safety.ESTOP_LATCHED:
            self.gate.clear()
            self.arm.resume_motion()
        elif not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if self.arm_mode() not in ("Position", "Impedance"):
            return "请先切回 Position 再复位"
        if self.arm_mode() == "Impedance" and self._sdk_float_active():
            return "阻抗环运行中请先切回 Position"
        self._note()
        self._drop_cart()
        err = self.arm.home(speed)
        if err:
            return err
        self._note()
        return None

    @manual_control
    def on_gripper(
        self, cid: str, open_: bool | None = None, deg: float | None = None, effort: int | None = None
    ) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if effort is not None:
            setter = getattr(self.arm, "set_gripper_effort", None)
            if callable(setter):
                err = setter(int(effort))
                if err:
                    return err
        if deg is not None:
            setter = getattr(self.arm, "set_gripper_deg", None)
            if callable(setter):
                err = setter(float(deg))
                if err:
                    return err
            else:
                self.arm.set_gripper_open(float(deg) >= 40.0)
        elif open_ is not None:
            err = self.arm.set_gripper_open(bool(open_))
            if err:
                return err
        self._note()
        return None

    def _prepare_connection(self) -> None:
        prepare = getattr(self.arm, "prepare_connection", None)
        self._enable_required = callable(prepare)
        if callable(prepare):
            prepare()

    def _publish_connection(self, status: str, error: str = "") -> None:
        with self._lock:
            self.world.connection = {
                "kind": connection_kind(self.arm_kind()),
                "port": str(self.cfg.arm_port or "") if self.arm_kind() == "fafu" else "",
                "status": status, "error": error, "session": self._connection_session,
                "enable_required": self._enable_required,
            }
            if status != "connected":
                self.world.arm.online = False
                self.world.arm.enabled = False
                self.world.arm.moving = False
                self.world.arm.backend = self.arm_kind()
                self.world.arm.link_err = error

    def on_link(self, cid: str, action: str, kind: str | None = None, port: str | None = None) -> str | None:
        act = str(action or "").strip().lower()
        if act not in ("connect", "disconnect"):
            return "未知连接动作"
        if self._arm_switching:
            return "连接操作正在进行，请等待完成"
        with self._arm_lock:
            if self.program.active:
                return "程序占用机械臂，请先停止程序"
            if act == "connect":
                try:
                    name = target_backend(kind) if kind is not None else self.arm_kind()
                    requested = serial_endpoint(port if port is not None else self.cfg.arm_port) if name == "fafu" else ""
                except ValueError as exc:
                    return str(exc)
                snap = self.arm.poll()
                online = snap.online and not self._link_disconnected
                current_port = str(snap.serial_port or self.cfg.arm_port or "")
                if online:
                    if name == self.arm_kind() and (name != "fafu" or requested == current_port):
                        return None
                    return "请先断开当前连接，再连接其它串口或仿真"
                if name == "fafu":
                    from robot_station.serial_guard import live_serial_allowed
                    if not live_serial_allowed():
                        return "当前进程未允许真机串口，请使用真机启动器后连接；不会改连仿真"
            else:
                snap = self.arm.poll()
                if snap.recording or snap.replay_active:
                    return "请先停止录制或回放，再断开设备"
                if self._link_disconnected:
                    return None

            self._arm_switching = True
            self._connection_session = secrets.token_hex(12)
            self._enable_required = True
            self._session_required = True
            self._stop_endpoint_hold()
            self._drop_cart()
            self._ik_err = ""
            self._cmd_err = ""
            self._link_disconnected = True
            self._publish_connection("connecting" if act == "connect" else "disconnecting")
            try:
                # Hold/stop run on the existing adapter before opening any new port.
                # Disconnect preserves support; it never silently removes motor torque.
                self.arm.hold(abort_path=True)
                self.arm.stop()
                if act == "disconnect":
                    self._publish_connection("disconnected")
                    return None
                self.cfg.arm, self.cfg.arm_port = name, requested
                self.arm = DisconnectedArm(name, self.cfg.arm_joints)
                self.arm = build_arm(name, self.cfg.arm_joints, self.cfg.arm_speed_deg_s,
                    self.cfg.arm_has_gripper, sdk=self.cfg.arm_sdk, cfg_path=self.cfg.arm_cfg,
                    port=requested, allow_motion=name=="sim" or bool(self.cfg.arm_allow_motion),
                    gripper_id=self.cfg.arm_gripper_id)
                self.world.arm = ArmSnap(online=False, enabled=False, backend=name,
                    n=self.cfg.arm_joints, ok=[False]*self.cfg.arm_joints)
                self._publish_connection("connecting")
                self.arm.start()
                snap = self.arm.poll()
                if not snap.online:
                    raise RuntimeError(snap.link_err or "设备未响应")
                self._prepare_connection()
                self._link_disconnected = False
                with self._lock:
                    self.world.arm = snap
                self._publish_connection("connected")
                logger.info("已连接 %s %s，等待显式使能", name, requested)
                return None
            except Exception as exc:
                try:
                    self.arm.stop()
                except Exception:
                    logger.exception("连接失败后的资源释放失败")
                self._publish_connection("error", str(exc))
                return str(exc)
            finally:
                self._arm_switching = False
                self.note_device_change()

    @manual_control
    def on_script(self, cid: str, name: str) -> str | None:
        from robot_station.adapters.fafu_arm import map_script

        op = map_script(name)
        if op == "estop":
            self.on_estop("script")
            return None
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        runner = getattr(self.arm, "run_mapped_script", None)
        if not callable(runner):
            if op == "home":
                self.arm.home(self.cfg.arm_speed_deg_s)
                return None
            return "此后端不运行脚本进程"
        err = runner(name)
        if err:
            return err
        self._note()
        return None

    @manual_control
    def on_power(self, cid: str, on: bool) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        # Explicit enable is allowed through the disabled-motion interlock;
        # connection/read-only refusal still applies to every adapter.
        check = getattr(self.arm, "refuse_power_change", self.arm.refuse_motion)
        blocked = check()
        if blocked:
            return blocked
        err = self.arm.set_powered(bool(on))
        if err and _is_vendor_enable_noise(err):
            try:
                snap = self.arm.poll()
            except Exception:
                snap = None
            if snap is not None and bool(snap.enabled):
                logger.warning("使能报错已忽略（反馈确认电机已使能）: %s", err)
                err = None
        if err:
            return err
        self._enable_required = not bool(on)
        self._publish_connection("connected")
        self._note()
        return None

    def note_rtt_ms(self, ms: float) -> None:
        self.world.last_rtt_ms = float(ms)

    def snapshot(self) -> dict:
        with self._lock:
            data = self.world.as_dict()
            data["cmd_err"] = self._cmd_err
            data["program"] = {**self.program.snapshot(), "speed_scale": self._program_scale}
            hold = self._endpoint_hold
            data["trajectory"] = {"endpoint_hold_id": hold["id"] if hold else ""}
            return data

    def _stop_endpoint_hold(self) -> None:
        if self._endpoint_hold:
            self._endpoint_hold = None
            self.arm.hold(abort_path=True)

    @staticmethod
    def _endpoint_reached(snap, target: list[float]) -> bool:
        actual = getattr(snap, "feedback_deg", None) or snap.q_deg
        return len(actual) == len(target) and max(abs(a-b) for a,b in zip(actual,target)) <= ENDPOINT_TOLERANCE_DEG

    def on_workbench(self, cid: str, msg: dict) -> dict:
        """Asset editing and program control share one request/response channel."""
        reply = {"t": "ack", "op": "workbench", "request_id": msg.get("request_id")}
        try:
            store = WorkbenchStore()
            action = str(msg.get("action") or "")
            if action == "catalog":
                result = store.catalog()
            elif action == "points_save":
                result = {"waypoints": store.save_points(msg.get("points"))}
            elif action == "point_capture":
                snap = self.arm.poll()
                if not snap.online:
                    raise ValueError("请先连接机械臂或仿真臂")
                q = (snap.target_deg if msg.get("source") == "target"
                     else (getattr(snap, "feedback_deg", None) or snap.q_deg))
                result = {"q": list(q)}
            elif action == "parse":
                nodes = parse_script(msg.get("script", "")) if "script" in msg else validate_tree(msg.get("nodes"))
                result = {"nodes": nodes, "script": format_script(nodes)}
            elif action == "save":
                result = store.save(msg.get("name"), msg.get("nodes"))
            elif action == "load":
                result = store.load(msg.get("name"))
            elif action == "delete":
                store.delete(msg.get("name"))
                result = store.catalog()
            elif action.startswith("traj_"):
                result = self._trajectory_action(cid, action, msg)
            else:
                with self._arm_lock:
                    result = self._program_control(cid, action, msg, store)
            return {**reply, "ok": True, "data": result}
        except (ValueError, OSError, TypeError, KeyError) as exc:
            return {**reply, "ok": False, "error": str(exc)}

    def _trajectory_action(self, cid: str, action: str, msg: dict) -> dict:
        from robot_station.trajectory_workspace import process_trajectory, validate_frames
        from robot_station.traj import load_traj, resolve_traj_path

        name = str(msg.get("file") or "")
        frames = None
        if action in ("traj_execute", "traj_endpoint"):
            _, raw = load_traj(resolve_traj_path(name))
            frames = validate_frames(raw)
        with self._arm_lock:
            snap = self.arm.poll()
            if action in ("traj_endpoint_touch", "traj_endpoint_stop"):
                hold = self._endpoint_hold
                matches = bool(hold and hold["owner"] == cid and hold["id"] == msg.get("hold_id"))
                if matches:
                    if action == "traj_endpoint_stop":
                        self._stop_endpoint_hold()
                    elif time.monotonic() - hold["touch"] >= ENDPOINT_HOLD_TIMEOUT_S:
                        self._stop_endpoint_hold()
                    else:
                        hold["touch"] = time.monotonic()
                return {"active": bool(matches and self._endpoint_hold)}
            if action == "traj_record_stop":
                error = self.on_rec_stop(cid)
                if error:
                    raise ValueError(error)
                return {"name": self.arm.poll().traj_file}
            busy = snap.recording or snap.replay_active or self.program.active or self._endpoint_hold
            if busy:
                raise ValueError("请先停止录制、回放或程序，再操作轨迹文件")
            if action == "traj_record":
                if not self.gate.motion_allowed():
                    raise ValueError("急停锁存中")
                if snap.moving or bool(getattr(self.arm, "is_homing", False)):
                    raise ValueError("请先停止当前运动")
                if name:
                    error = self.arm.record_start(msg.get("name"), teach=msg.get("teach"), append_from=name)
                else:
                    error = self.on_rec_start(cid, msg.get("name"), msg.get("teach"))
                if error:
                    raise ValueError(error)
                self._drop_cart()
                return {"name": self.arm.poll().traj_file}
            if action == "traj_execute":
                if snap.moving or bool(getattr(self.arm, "is_homing", False)):
                    raise ValueError("请先停止当前运动")
                error = self.on_replay(cid, name, number(msg.get("rate", 0.5), 0.5, 1, "回放倍率"),
                                       number(msg.get("speed", 10), 5, 30, "起点速度"))
                if error:
                    raise ValueError(error)
                return {"name": name}
            if action == "traj_endpoint":
                hold_id = msg.get("hold_id")
                if not isinstance(hold_id, str) or not 1 <= len(hold_id) <= 96:
                    raise ValueError("请使用新版界面的按住移动按钮")
                if snap.moving or bool(getattr(self.arm, "is_homing", False)):
                    raise ValueError("请先停止当前运动")
                target = [math.degrees(x) for x in frames[-1]["pos"]]
                if self._endpoint_reached(snap, target):
                    return {"name": name, "active": False, "reached": True}
                error = self.on_path(cid, [target], 10)
                if error:
                    raise ValueError(error)
                self._endpoint_hold = {"id": hold_id, "owner": cid, "touch": time.monotonic(), "target": target}
                return {"name": name, "active": True, "reached": False}
            if action == "traj_rename":
                error = self.on_rec_rename(cid, name, str(msg.get("name") or ""))
                if error:
                    raise ValueError(error)
                from robot_station.traj import sanitize_traj_name
                return {"name": sanitize_traj_name(msg.get("name"))}
            if action == "traj_delete":
                error = self.on_rec_delete(cid, name)
                if error:
                    raise ValueError(error)
                return {}
            if action != "traj_process":
                raise ValueError("未知轨迹操作")
        # Editing is CPU/file work: keep it off the 100 Hz motion lock.
        return process_trajectory(name, msg.get("operations"), save=bool(msg.get("save")),
                                  dest=str(msg.get("name") or ""), future=msg.get("future"))

    def _program_control(self, cid: str, action: str, msg: dict, store: WorkbenchStore) -> dict:
        if action == "stop":
            self._stop_endpoint_hold()
            self.program.stop()
            self._program_owner = ""
            if self.arm.poll().recording:
                self.arm.record_stop()
            self.arm.hold(abort_path=True)
            self._drop_cart()
        elif action == "pause":
            self.program.pause()
            if self.program.active:
                self.arm.hold(abort_path=True)
        elif action in ("run", "step", "resume"):
            if self._endpoint_hold:
                raise ValueError("请先松开移动到末点按钮")
            snap = self.arm.poll()
            blocked = self.arm.refuse_motion()
            if not self.gate.motion_allowed() or blocked:
                raise ValueError(blocked or "请先解除急停")
            if not snap.online or not snap.enabled:
                raise ValueError("请先连接并使能机械臂")
            if self.arm_mode() != "Position":
                raise ValueError("程序需要 Position 模式")
            if snap.recording:
                raise ValueError("请先停止示教录制")
            if self.program.status == "paused" and action in ("step", "resume"):
                if cid != self._program_owner:
                    raise ValueError("请在启动程序的窗口中继续")
                self.program.resume(single=action == "step")
            else:
                if self.program.active or snap.moving or snap.replay_active or bool(getattr(self.arm, "is_homing", False)):
                    raise ValueError("请先停止当前动作或程序")
                self._program_scale = number(msg.get("speed_scale", 1), 0.1, 1, "速度倍率")
                prepared = ProgramRunner()
                prepared.start(str(msg.get("name") or "未命名程序"), msg.get("nodes"),
                               store.points(), snap.limits, single=action == "step")
                self.arm.hold(abort_path=True)
                self._drop_cart()
                self.program = prepared
                self._program_owner = cid
            self._program_touch = time.monotonic()
            self._note()
        else:
            raise ValueError(f"未知工作台操作：{action}")
        return self.program.snapshot()

    def _dispatch_program_node(self, node: dict) -> str | None:
        if not self.gate.motion_allowed():
            return "急停已锁存"
        kind = node["type"]
        speed = max(5, float(node.get("speed", 20)) * self._program_scale)
        if kind == "movej":
            return self.arm.run_path([node["q"]], speed)
        if kind == "gripper":
            snap = self.arm.poll()
            if not snap.has_gripper or any(m.get("name") == "夹爪" and not m.get("online") for m in snap.motors):
                return "夹爪未连接，程序已停止"
            setter = getattr(self.arm, "set_gripper_effort", None)
            if callable(setter):
                error = setter(node["effort"])
                if error:
                    return error
            return self.arm.set_gripper_open(node["open"])
        if kind == "replay":
            return self.arm.replay_start(node["file"], node["rate"], speed)
        return None

    def wait_snap(self, last_seq: int, timeout_s: float = 0.25) -> tuple[int, dict]:
        """Block until world.seq moves past last_seq, then return (seq, snapshot)."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        seq = last_seq
        with self._pub:
            while not self._stop.is_set():
                with self._lock:
                    seq = int(self.world.seq)
                if seq != last_seq:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._pub.wait(remaining)
        return seq, self.snapshot()

    def _set_cmd_err(self, err: str | None) -> None:
        text = err or ""
        if text and _is_vendor_enable_noise(text) and _live_joints_ok(self.world.arm):
            text = ""
        with self._lock:
            self._cmd_err = text

    def handle(self, msg: dict[str, Any]) -> dict[str, Any] | None:
        try:
            if needs_session(msg):
                if self._arm_switching:
                    return rejected(msg, "连接操作正在进行，请等待完成")
                with self._arm_lock:
                    if self._session_required and msg.get("session") != self._connection_session:
                        if msg.get("t") in ("touch", "park"):
                            return None
                        return rejected(msg, "连接会话已变化，请等待最新设备状态后重试；旧指令未执行")
                    return self._handle(msg)
            return self._handle(msg)
        except Exception as exc:
            logger.exception("运动指令失败")
            return {"t": "ack", "ok": False, "error": str(exc), "op": str(msg.get("t") or "")}

    def _handle(self, msg: dict[str, Any]) -> dict[str, Any] | None:
        kind = msg.get("t")
        cid = str(msg.get("cid") or "")
        if self._arm_switching and kind not in ("add", "drop", "touch", "rtt", "e", "park", "arm_src"):
            return {"t": "ack", "ok": False, "error": "连接操作正在进行", "op": str(kind or "")}
        if kind == "add":
            if cid:
                self.add_client(cid)
        elif kind == "workbench":
            return self.on_workbench(cid, msg)
        elif kind == "drop":
            if cid:
                self.drop_client(cid)
        elif kind == "touch":
            self.on_touch(cid)
        elif kind == "park":
            self.on_park(cid)
        elif kind == "arm_src":
            err = self.on_arm_src(cid, str(msg.get("mode") or ""))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "arm_src"}
        elif kind == "e":
            self.on_estop(str(msg.get("reason") or "operator"))
        elif kind == "clear":
            err = self.on_clear_estop(cid)
            return {"t": "ack", "ok": err is None, "error": err, "op": "clear"}
        elif kind == "arm":
            q = msg.get("q") or []
            speed = float(msg.get("speed") or self.cfg.arm_speed_deg_s)
            stream = bool(msg.get("stream"))
            err = self.on_arm_targets(
                cid, [float(x) for x in q], speed, stream=stream, t0=msg.get("t0")
            )
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "arm"}
        elif kind == "home":
            speed = float(msg.get("speed") or self.cfg.arm_speed_deg_s)
            err = self.on_home(cid, speed)
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "home"}
        elif kind == "grip":
            deg = msg.get("deg")
            effort = msg.get("effort")
            open_raw = msg.get("open")
            err = self.on_gripper(
                cid,
                None if open_raw is None else bool(open_raw),
                None if deg is None else float(deg),
                None if effort is None else int(effort),
            )
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "grip"}
        elif kind == "mode":
            err = self.on_mode(cid, str(msg.get("mode") or "Position"))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "mode"}
        elif kind == "grav_urdf":
            err = self.on_grav_urdf(cid, str(msg.get("name") or msg.get("urdf") or ""))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "grav_urdf"}
        elif kind == "teach":
            q = msg.get("q") or []
            err = self.on_teach(cid, [float(x) for x in q], t0=msg.get("t0"))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "teach"}
        elif kind == "cart":
            dxyz = [float(x) for x in (msg.get("dxyz") or [0, 0, 0])]
            drpy = [float(x) for x in (msg.get("drpy") or [0, 0, 0])]
            if msg.get("speed") is not None:
                setter = getattr(self.arm, "set_teleop_speed", None)
                if callable(setter):
                    setter(float(msg.get("speed")))
            err = self.on_cartesian(cid, dxyz, drpy, t0=msg.get("t0"))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "cart"}
        elif kind == "cart_go":
            xyz = [float(x) for x in (msg.get("xyz") or [0, 0, 0])]
            rpy = [float(x) for x in (msg.get("rpy") or [0, 0, 0])]
            speed = float(msg.get("speed") or self.cfg.arm_speed_deg_s)
            err = self.on_cartesian_pose(cid, xyz, rpy, speed, t0=msg.get("t0"), stream=bool(msg.get("stream")))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "cart_go"}
        elif kind == "path":
            raw = msg.get("q") or []
            speed = float(msg.get("speed") or self.cfg.arm_speed_deg_s)
            wps = [[float(x) for x in row] for row in raw]
            dt_raw = msg.get("dt")
            durations = [float(x) for x in dt_raw] if dt_raw else None
            err = self.on_path(cid, wps, speed, durations=durations)
            return {"t": "ack", "ok": err is None, "error": err, "op": "path"}
        elif kind == "rec_start":
            teach = msg.get("teach")
            err = self.on_rec_start(cid, msg.get("name"), None if teach is None else str(teach))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "rec_start"}
        elif kind == "rec_stop":
            err = self.on_rec_stop(cid)
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "rec_stop"}
        elif kind == "rec_delete":
            err = self.on_rec_delete(cid, str(msg.get("file") or msg.get("path") or ""))
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "rec_delete"}
        elif kind == "rec_rename":
            err = self.on_rec_rename(
                cid,
                str(msg.get("file") or msg.get("path") or ""),
                str(msg.get("name") or msg.get("to") or msg.get("dest") or ""),
            )
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "rec_rename"}
        elif kind == "replay":
            err = self.on_replay(
                cid,
                str(msg.get("file") or msg.get("path") or ""),
                float(msg.get("rate") or 1.0),
                msg.get("speed"),
            )
            self._set_cmd_err(err)
            return {"t": "ack", "ok": err is None, "error": err, "op": "replay"}
        elif kind == "link":
            err = self.on_link(cid, str(msg.get("action") or ""), msg.get("kind"), msg.get("port"))
            return {"t": "ack", "ok": err is None, "error": err, "op": "link", "request_id":msg.get("request_id")}
        elif kind == "script":
            err = self.on_script(cid, str(msg.get("name") or ""))
            return {"t": "ack", "ok": err is None, "error": err, "op": "script"}
        elif kind == "power":
            err = self.on_power(cid, bool(msg.get("on", True)))
            return {"t": "ack", "ok": err is None, "error": err, "op": "power"}
        elif kind == "rtt":
            self.note_rtt_ms(float(msg.get("ms") or 0))
        return None

    def _tick_loop(self) -> None:
        period = 1.0 / max(1, self.cfg.control_hz)
        next_tick = time.perf_counter()
        last = next_tick
        while not self._stop.is_set():
            now_pc = time.perf_counter()
            dt = now_pc - last
            last = now_pc
            try:
                self._step(time.monotonic(), dt)
            except Exception:
                self._note_tick_error()
            next_tick += period
            if next_tick < now_pc - period:
                next_tick = now_pc + period
            _sleep_until(next_tick)

    def _note_tick_error(self) -> None:
        """Keep the loop alive and make the failure visible to the HUD.

        ``_set_cmd_err`` alone is not enough: telemetry waits for ``world.seq``
        to move, and a persistent failure would freeze the page with no error.
        Logging is rate limited so a 100 Hz failure cannot flood the host log.
        """
        now = time.monotonic()
        with self._arm_lock:
            self._stop_endpoint_hold()
        if self.program.active:
            self.program.stop("运动拍异常，程序已停止")
            try:
                self.arm.hold(abort_path=True)
            except Exception:
                logger.exception("程序异常后的保持失败")
        self._tick_err_count += 1
        if now - self._tick_err_last_log >= TICK_ERR_LOG_EVERY_S:
            self._tick_err_last_log = now
            if self._tick_err_count == 1:
                logger.exception("100 Hz tick 异常，本拍跳过并继续")
            else:
                logger.error(
                    "100 Hz tick 仍在异常（连续 %d 次），本拍跳过并继续",
                    self._tick_err_count,
                )
        with self._lock:
            self._cmd_err = TICK_ERR_TEXT
            self.world.seq += 1
            self.world.t_mono = now
        with self._pub:
            self._pub.notify_all()

    def _step(self, now: float, dt: float) -> None:
        with self._lock:
            ncli = len(self.clients)
            hits = self._cmd_hits
            elapsed = now - self._cmd_t
            if elapsed >= 1.0:
                self.world.cmd_hz = hits / elapsed
                self._cmd_hits = 0
                self._cmd_t = now
            moving = bool(self.world.arm.moving)
        got = self._arm_lock.acquire(blocking=False)
        if not got:
            with self._lock:
                self.world.seq += 1
                self.world.t_mono = now
            with self._pub:
                self._pub.notify_all()
            return
        try:
            self._step_arm(now, dt, ncli, moving)
        finally:
            self._arm_lock.release()

    def _step_arm(self, now: float, dt: float, ncli: int, moving: bool) -> None:
        hold = self._endpoint_hold
        if hold and (now - hold["touch"] >= ENDPOINT_HOLD_TIMEOUT_S or
                     not self.gate.motion_allowed() or
                     self._endpoint_reached(self.arm.poll(), hold["target"])):
            self._stop_endpoint_hold()
        if self.program.active and now - self._program_touch > 2.0:
            self.program.stop("程序控制连接超时")
            self.arm.hold(abort_path=True)
        if self.program.status == "running":
            self.gate.note_command()
        homing = bool(getattr(self.arm, "is_homing", False))
        parked = bool(getattr(self.arm, "_parked", False))
        streaming = bool(getattr(self.arm, "_servoing", False))
        if homing:
            self.gate.note_command()
        # Gravity is a background writer; 0.2s UI watchdog would immediately
        # abort a float mode with no 100 Hz command stream. ESTOP latches
        # brake and must not be turned back into a position hold.
        # Parked hold-stream and homing must not be treated as a runaway
        # teleop: hold() used to servo_end and clear the home path.
        force_hold = not self.gate.motion_allowed()
        if streaming and not parked and not homing:
            force_hold = self.gate.tick_watchdog(now, moving) or force_hold
        if ncli == 0:
            if self.gate.state == Safety.OPERATING:
                self.gate.state = Safety.IDLE
            if self._had_clients:
                self.arm.hold(abort_path=True)
                self._drop_cart()
        self._had_clients = ncli > 0
        if self.gate.state == Safety.ESTOP_LATCHED:
            self.arm.emergency_stop()
            self._drop_cart()
        elif force_hold:
            self.arm.hold()
            self._drop_cart()
        else:
            try:
                self._flush_cart()
            except Exception as exc:
                logger.exception("笛卡尔增量失败")
                self._ik_err = f"笛卡尔失败: {exc}"
        self.arm.servo(dt)
        writer_err = str(getattr(self.arm, "_writer_err", "") or "")
        arm = self.arm.poll()
        if self._link_disconnected:
            arm.online = arm.enabled = arm.moving = False
        elif not arm.online and self.world.connection.get("status") == "connected":
            self._connection_session = secrets.token_hex(12)
            self._enable_required = True
            self._link_disconnected = True
            self._drop_cart()
            self._endpoint_hold = None
            if self.program.active:
                self.program.stop("设备连接已断开，程序已停止")
            self._publish_connection("error", arm.link_err or "设备连接已断开，请检查后重新连接")
        if self.world.connection.get("status") == "connected":
            self._publish_connection("connected")
        if self.program.status == "running":
            if writer_err or not arm.online or not arm.enabled:
                self.program.stop(writer_err or "机械臂连接或使能状态丢失")
            else:
                self.program.tick(now, arm, self._dispatch_program_node, self._program_scale)
            if self.program.status in ("error", "completed"):
                self.arm.hold(abort_path=True)
            if self.program.error:
                self._set_cmd_err(self.program.error)
        if writer_err and _is_vendor_enable_noise(writer_err) and _live_joints_ok(arm):
            try:
                self.arm._writer_err = ""
            except Exception:
                pass
            writer_err = ""
        if writer_err:
            self._set_cmd_err(writer_err)
        else:
            stale = self._cmd_err or ""
            if stale == TICK_ERR_TEXT:
                self._set_cmd_err("")
            elif _live_joints_ok(arm) and _is_vendor_enable_noise(stale):
                self._set_cmd_err("")
        if self._ik_err:
            arm.ik_err = self._ik_err
        cam = self.camera.poll() if self.camera is not None else self.world.camera
        with self._lock:
            self.world.seq += 1
            self.world.t_mono = now
            self.world.safety = self.gate.state.name
            self.world.safety_reason = self.gate.reason
            self.world.clients = ncli
            self.world.arm = arm
            self.world.camera = cam
        with self._pub:
            self._pub.notify_all()
        self._tick_err_count = 0


def encode_line(obj: dict) -> bytes:
    return (json.dumps(obj, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


class _Conn:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.lock = threading.Lock()
        self.cids: set[str] = set()
        self.alive = True

    def send(self, obj: dict) -> None:
        data = encode_line(obj)
        with self.lock:
            self.sock.sendall(data)

    def send_raw(self, data: bytes) -> None:
        with self.lock:
            self.sock.sendall(data)


def serve_motion(
    core: MotionCore,
    host: str,
    port: int,
    stop: threading.Event,
    srv: socket.socket | None = None,
    *,
    already_listening: bool = False,
) -> None:
    if srv is None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, int(port)))
        already_listening = False
    if not already_listening:
        srv.listen(8)
    srv.settimeout(0.3)
    conns: list[_Conn] = []
    conns_lock = threading.Lock()
    logger.info("运动进程听 %s:%s", host, port)

    def _drop(dead: list[_Conn]) -> None:
        if not dead:
            return
        with conns_lock:
            for conn in dead:
                conn.alive = False
                if conn in conns:
                    conns.remove(conn)
                try:
                    conn.sock.close()
                except OSError:
                    pass
                for cid in list(conn.cids):
                    core.drop_client(cid)

    def broadcast(obj: dict) -> None:
        with conns_lock:
            targets = list(conns)
        dead: list[_Conn] = []
        for conn in targets:
            try:
                conn.send(obj)
            except OSError:
                dead.append(conn)
        _drop(dead)

    def broadcast_raw(data: bytes) -> None:
        with conns_lock:
            targets = list(conns)
        dead: list[_Conn] = []
        for conn in targets:
            try:
                conn.send_raw(data)
            except OSError:
                dead.append(conn)
        _drop(dead)

    def telemetry() -> None:
        last_seq = -1
        last_hud = 0.0
        while not stop.is_set():
            seq, data = core.wait_snap(last_seq, 0.25)
            if seq == last_seq:
                continue
            last_seq = seq
            try:
                broadcast_raw(pack_telem(data))
                now = time.monotonic()
                if now - last_hud >= 0.1:
                    broadcast({"t": "snap", "d": data})
                    last_hud = now
            except Exception:
                logger.exception("遥测发送失败")

    tel = threading.Thread(target=telemetry, name="motion-tel", daemon=True)
    tel.start()

    def client_loop(conn: _Conn) -> None:
        buf = b""
        try:
            conn.send({"t": "ready", "arm": getattr(core.arm, "backend", "?")})
            while not stop.is_set() and conn.alive:
                try:
                    chunk = conn.sock.recv(16384)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    if not raw.strip():
                        continue
                    try:
                        msg = json.loads(raw.decode("utf-8"))
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        continue
                    if not isinstance(msg, dict):
                        continue
                    cid = str(msg.get("cid") or "")
                    if msg.get("t") == "add" and cid:
                        conn.cids.add(cid)
                    if msg.get("t") == "drop" and cid:
                        conn.cids.discard(cid)
                    ack = core.handle(msg)
                    if ack is None:
                        continue
                    has_req = "req" in msg
                    if has_req:
                        ack["req"] = msg["req"]
                    if has_req or not ack.get("ok", True):
                        try:
                            conn.send(ack)
                        except OSError:
                            return
        finally:
            conn.alive = False
            with conns_lock:
                if conn in conns:
                    conns.remove(conn)
            for cid in list(conn.cids):
                core.drop_client(cid)
            try:
                conn.sock.close()
            except OSError:
                pass

    try:
        while not stop.is_set():
            try:
                sock, _addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            sock.settimeout(0.5)
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            conn = _Conn(sock)
            with conns_lock:
                conns.append(conn)
            threading.Thread(target=client_loop, args=(conn,), name="motion-cli", daemon=True).start()
    finally:
        stop.set()
        with conns_lock:
            leftover = list(conns)
            conns.clear()
        for conn in leftover:
            try:
                conn.sock.close()
            except OSError:
                pass
        try:
            srv.close()
        except OSError:
            pass
        tel.join(timeout=1.0)


def run_motion_process(cfg: StationConfig, stop: threading.Event) -> None:
    from robot_station.lock import ExclusiveLock, MOTION_LOCK

    lock = ExclusiveLock(MOTION_LOCK)
    lock.acquire()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    core: MotionCore | None = None
    boot: threading.Thread | None = None
    try:
        try:
            srv.bind(("127.0.0.1", int(cfg.motion_port)))
            srv.listen(8)
        except OSError as exc:
            raise RuntimeError(
                f"运动端口 {cfg.motion_port} 绑不上：{exc}。"
                "已有一份站控在跑就不要再开；确认没人用可加 --replace。"
            ) from exc
        core = MotionCore(cfg, camera=None)

        def _boot() -> None:
            try:
                core.start()
            except Exception:
                logger.exception("运动核启动失败，9470 继续听")

        boot = threading.Thread(target=_boot, name="motion-boot", daemon=True)
        boot.start()
        serve_motion(
            core,
            "127.0.0.1",
            cfg.motion_port,
            stop,
            srv=srv,
            already_listening=True,
        )
        srv = None
    finally:
        if boot is not None and boot.is_alive():
            boot.join(timeout=8.0)
        if core is not None:
            core.stop()
        if srv is not None:
            try:
                srv.close()
            except OSError:
                pass
        lock.release()
