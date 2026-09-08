"""Motion process: 100 Hz tick, arm adapter."""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from typing import Any

from robot_station.adapters.arm import ArmAdapter, build_arm
from robot_station.adapters.camera import CameraBank
from robot_station.config import StationConfig
from robot_station.safety import Safety, SafetyGate
from robot_station.telem import pack_telem
from robot_station.world import World

logger = logging.getLogger("station.motion")


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


class MotionCore:
    def __init__(self, cfg: StationConfig, camera: CameraBank | None = None) -> None:
        self.cfg = cfg
        self.world = World()
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
        self._ik_err = ""
        self._cmd_err = ""
        self._pub = threading.Condition()
        self._timer_armed = False
        self._arm_lock = threading.RLock()
        self._arm_switching = False

    def _note_echo(self, t0: object) -> None:
        if t0 is None or t0 == "":
            return
        try:
            self.world.echo_t0 = float(t0)
        except (TypeError, ValueError):
            return

    def start(self) -> None:
        wanted = str(self.cfg.arm or "").strip().lower()
        try:
            self.arm.start()
        except Exception as exc:
            if wanted == "fafu":
                logger.error("真机连接失败，改用仿真以便打开页面: %s", exc)
                self._adopt_sim(str(exc))
            else:
                raise
        else:
            if wanted == "fafu" and getattr(self.arm, "_robot", None) is None:
                err = str(getattr(self.arm, "_link_error", "") or "真机未连接")
                logger.error("真机未连上，改用仿真以便打开页面: %s", err)
                self._adopt_sim(err)
            else:
                self._boot_live_arm()
        try:
            if self.camera is not None:
                self.camera.start()
        except Exception:
            self.arm.stop()
            raise
        self._stop.clear()
        self._timer_armed = _win_timer_begin()
        self._tick_th = threading.Thread(target=self._tick_loop, name="station-tick", daemon=True)
        self._tick_th.start()
        logger.info(
            "tick %s Hz  臂=%s  相机=%s",
            self.cfg.control_hz,
            getattr(self.arm, "backend", "?"),
            "local" if self.camera is not None else "网页进程",
        )

    def _boot_live_arm(self) -> None:
        """After a live USB start, Connect is already done; enable so the page is ready.

        SDK construction stays auto_enable=False. Switching 仿真→真机 in the UI
        still does not auto-enable.
        """
        if str(getattr(self.arm, "backend", "") or "") != "fafu":
            return
        if getattr(self.arm, "_robot", None) is None:
            return
        if not bool(self.cfg.arm_allow_motion):
            return
        from robot_station.serial_guard import live_serial_allowed

        if not live_serial_allowed():
            return
        err = self.arm.set_powered(True)
        snap = None
        try:
            snap = self.arm.poll()
        except Exception:
            snap = None
        motors = list(getattr(snap, "motors", None) or []) if snap is not None else []
        offline = [str(m.get("name") or m.get("id") or "") for m in motors if not m.get("online")]
        if err:
            self._cmd_err = f"已连接真机，使能失败：{err}"
            logger.error("%s", self._cmd_err)
        elif offline:
            logger.warning("已使能在线轴；离线：%s", "、".join(offline))
            self._cmd_err = ""
        else:
            self._cmd_err = ""
            logger.info("真机已连接并使能")

    def _adopt_sim(self, reason: str) -> None:
        try:
            self.arm.stop()
        except Exception:
            logger.exception("关闭失败的真机适配器失败")
        nxt = build_arm(
            "sim",
            self.cfg.arm_joints,
            self.cfg.arm_speed_deg_s,
            self.cfg.arm_has_gripper,
            sdk=self.cfg.arm_sdk,
            cfg_path=self.cfg.arm_cfg,
            port=self.cfg.arm_port,
            allow_motion=True,
            gripper_id=self.cfg.arm_gripper_id,
        )
        nxt.start()
        self.arm = nxt
        self.cfg.arm = "sim"
        self._cmd_err = f"真机未连上，已切仿真：{reason}"

    def stop(self) -> None:
        self._stop.set()
        with self._pub:
            self._pub.notify_all()
        if self._tick_th is not None:
            self._tick_th.join(timeout=1.0)
        if self._timer_armed:
            _win_timer_end()
            self._timer_armed = False
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
        """Switch between live USB (fafu) and in-process sim. Does not auto-enable."""
        name = (kind or "").strip().lower()
        if name not in ("sim", "fafu"):
            return f"未知臂源 {kind!r}，请用 sim 或 fafu"
        current = self.arm_kind()
        if name == current:
            return None
        if name == "fafu":
            from robot_station.serial_guard import live_serial_allowed

            if not live_serial_allowed():
                return "未允许真机串口（请用「启动真机.bat」，需 STATION_ALLOW_LIVE_ARM=1）"
        q_keep = list(self.world.arm.q_deg)
        self._arm_switching = True
        try:
            with self._arm_lock:
                try:
                    self.arm.hold()
                except Exception:
                    logger.exception("切换臂源前 hold 失败")
                try:
                    self.arm.stop()
                except Exception:
                    logger.exception("切换臂源时关闭旧臂失败")
                allow = True if name == "sim" else bool(self.cfg.arm_allow_motion)
                try:
                    nxt = build_arm(
                        name,
                        self.cfg.arm_joints,
                        self.cfg.arm_speed_deg_s,
                        self.cfg.arm_has_gripper,
                        sdk=self.cfg.arm_sdk,
                        cfg_path=self.cfg.arm_cfg,
                        port=self.cfg.arm_port,
                        allow_motion=allow,
                        gripper_id=self.cfg.arm_gripper_id,
                    )
                    nxt.start()
                except Exception as exc:
                    logger.exception("切换到 %s 失败，回退仿真", name)
                    nxt = build_arm(
                        "sim",
                        self.cfg.arm_joints,
                        self.cfg.arm_speed_deg_s,
                        self.cfg.arm_has_gripper,
                        sdk=self.cfg.arm_sdk,
                        cfg_path=self.cfg.arm_cfg,
                        port=self.cfg.arm_port,
                        allow_motion=True,
                        gripper_id=self.cfg.arm_gripper_id,
                    )
                    nxt.start()
                    self.arm = nxt
                    self.cfg.arm = "sim"
                    self._seed_sim_pose(q_keep)
                    return f"真机连接失败，已留在仿真：{exc}"
                if name == "fafu" and getattr(nxt, "_robot", None) is None:
                    err = str(getattr(nxt, "_link_error", "") or "真机未连接")
                    self.arm = nxt
                    self.cfg.arm = name
                    return err
                self.arm = nxt
                self.cfg.arm = name
                if name == "sim":
                    self._seed_sim_pose(q_keep)
                self._drop_cart()
                logger.info("臂源已切换为 %s", name)
                return None
        finally:
            self._arm_switching = False

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
            self.clients.discard(cid)
            empty = not self.clients
        if empty:
            self.arm.hold(abort_path=True)

    def _note(self) -> None:
        with self._lock:
            self._cmd_hits += 1
        self.gate.note_command()

    def on_touch(self, cid: str) -> None:
        self._note()

    def on_park(self, cid: str) -> None:
        park = getattr(self.arm, "park_stream", None)
        if callable(park):
            park()

    def on_estop(self, reason: str = "operator") -> None:
        self.gate.estop(reason)
        self.arm.emergency_stop()
        with self._lock:
            self.world.safety = self.gate.state.name
            self.world.safety_reason = self.gate.reason

    def on_clear_estop(self, cid: str) -> str | None:
        self.gate.clear()
        self.arm.resume_motion()
        with self._lock:
            self.world.safety = self.gate.state.name
            self.world.safety_reason = self.gate.reason
        return None

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

    def on_mode(self, cid: str, mode: str) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        err = self.arm.set_ctrl_mode(mode)
        if err:
            return err
        self._note()
        return None

    def on_teach(self, cid: str, q_deg: list[float], t0: float | None = None) -> str | None:
        self._note_echo(t0)
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        if self.arm_mode() not in ("Gravity", "Gra+Fri"):
            return "示教仅在 Gravity / Gra+Fri 下可用"
        self.arm.teach_pose(q_deg)
        self._note()
        return None

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
            for i in range(3):
                self._cart_dxyz[i] += float(dxyz[i]) if i < len(dxyz) else 0.0
                self._cart_drpy[i] += float(drpy_deg[i]) if i < len(drpy_deg) else 0.0
            self._cart_pending = True
        self._note()
        return None

    def on_cartesian_pose(
        self,
        cid: str,
        xyz_m: list[float],
        rpy_deg: list[float],
        speed: float,
        t0: float | None = None,
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
        self._note()
        return None

    def _drop_cart(self) -> None:
        with self._lock:
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
        err = self.arm.run_path(waypoints, speed, durations_s=durations)
        if err:
            return err
        self._note()
        return None

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

    def on_rec_stop(self, cid: str) -> str | None:
        fn = getattr(self.arm, "record_stop", None)
        if not callable(fn):
            return None
        return fn()

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
        err = fn(path, rate=float(rate), speed_deg_s=spd)
        if err:
            return err
        self._note()
        return None

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
        err = self.arm.home(speed)
        if err:
            return err
        self._note()
        return None

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

    def on_link(self, cid: str, action: str) -> str | None:
        act = (action or "").strip().lower()
        if act == "connect":
            try:
                self.arm.start()
            except Exception as exc:
                return str(exc)
            if getattr(self.arm, "_robot", None) is None:
                return str(getattr(self.arm, "_link_error", "") or "机械臂未连接")
            snap = self.arm.poll()
            motors = list(getattr(snap, "motors", None) or [])
            offline = [str(m.get("name") or m.get("id") or "") for m in motors if not m.get("online")]
            if offline:
                logger.warning("已连接调试板，离线：%s", "、".join(offline))
            self._cmd_err = ""
            return None
        if act == "disconnect":
            try:
                self.arm.hold()
            except Exception:
                logger.exception("断开前 hold 失败")
            self.arm.stop()
            return None
        return f"未知连接动作 {action!r}"

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

    def on_power(self, cid: str, on: bool) -> str | None:
        if not self.gate.motion_allowed():
            return "急停锁存中"
        blocked = self.arm.refuse_motion()
        if blocked:
            return blocked
        err = self.arm.set_powered(bool(on))
        if err and _is_vendor_enable_noise(err):
            try:
                snap = self.arm.poll()
            except Exception:
                snap = None
            if _live_joints_ok(snap):
                logger.warning("使能报错已忽略（J1–J6 在线）: %s", err)
                err = None
        if err:
            return err
        self._note()
        return None

    def note_rtt_ms(self, ms: float) -> None:
        self.world.last_rtt_ms = float(ms)

    def snapshot(self) -> dict:
        with self._lock:
            data = self.world.as_dict()
            data["cmd_err"] = self._cmd_err
            return data

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
            return self._handle(msg)
        except Exception as exc:
            logger.exception("运动指令失败")
            return {"t": "ack", "ok": False, "error": str(exc), "op": str(msg.get("t") or "")}

    def _handle(self, msg: dict[str, Any]) -> dict[str, Any] | None:
        kind = msg.get("t")
        cid = str(msg.get("cid") or "")
        if self._arm_switching and kind not in ("add", "drop", "touch", "rtt", "e", "park", "arm_src"):
            return {"t": "ack", "ok": False, "error": "正在切换仿真/真机", "op": str(kind or "")}
        if kind == "add":
            if cid:
                self.add_client(cid)
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
            err = self.on_cartesian_pose(cid, xyz, rpy, speed, t0=msg.get("t0"))
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
            err = self.on_link(cid, str(msg.get("action") or ""))
            return {"t": "ack", "ok": err is None, "error": err, "op": "link"}
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
            self._step(time.monotonic(), dt)
            next_tick += period
            if next_tick < now_pc - period:
                next_tick = now_pc + period
            _sleep_until(next_tick)

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
        homing = bool(getattr(self.arm, "is_homing", False))
        parked = bool(getattr(self.arm, "_parked", False))
        streaming = bool(getattr(self.arm, "_servoing", False))
        if homing:
            self.gate.note_command()
        # Gravity is a background writer; 0.2s UI watchdog would immediately
        # abort a float mode with no 100 Hz command stream. ESTOP still
        # calls hold() and stops the loop.
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
        if self.gate.state == Safety.ESTOP_LATCHED or force_hold:
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
        if writer_err and _is_vendor_enable_noise(writer_err) and _live_joints_ok(arm):
            try:
                self.arm._writer_err = ""
            except Exception:
                pass
            writer_err = ""
        if writer_err:
            self._set_cmd_err(writer_err)
        elif _live_joints_ok(arm):
            stale = self._cmd_err or ""
            if _is_vendor_enable_noise(stale):
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
) -> None:
    if srv is None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, int(port)))
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
    try:
        try:
            srv.bind(("127.0.0.1", int(cfg.motion_port)))
        except OSError as exc:
            raise RuntimeError(
                f"运动端口 {cfg.motion_port} 绑不上：{exc}。"
                "已有一份站控在跑就不要再开；确认没人用可加 --replace。"
            ) from exc
        core = MotionCore(cfg, camera=None)
        core.start()
        serve_motion(core, "127.0.0.1", cfg.motion_port, stop, srv=srv)
        srv = None
    finally:
        if core is not None:
            core.stop()
        if srv is not None:
            try:
                srv.close()
            except OSError:
                pass
        lock.release()
