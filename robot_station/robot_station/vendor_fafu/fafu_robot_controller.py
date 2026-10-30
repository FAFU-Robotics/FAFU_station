"""Station-specific SDK overrides.

All unchanged behavior is inherited from the actual fafu_arm_sdk submodule.
Only offline-motor, gripper, serial discovery and streaming adaptations live here.
Keep these overrides under the station regression tests when updating the SDK.
"""
from __future__ import annotations

import fafu_robot_controller as _SDK

# Override bodies use the SDK's enums, decorators and helpers so their state and
# protocol definitions cannot drift into another copied SDK implementation.
for _export_name, _export_value in vars(_SDK).items():
    if not _export_name.startswith("__"):
        globals()[_export_name] = _export_value


class FafuRobotController(_SDK.FafuRobotController):
    def close_connection(self, *, joint_release="stop", gripper_release="brake"):
        if joint_release != "preserve":
            return super().close_connection(joint_release=joint_release, gripper_release=gripper_release)
        # A newly connected/read-only controller must not change motor modes
        # just to release its port. In particular, neither STOP (sag) nor
        # POSITION (enable) is an acceptable implicit disconnect action.
        if getattr(self, "_servo_active", False):
            raise RuntimeError("关闭串口前必须先结束本机伺服会话")
        ht = self._ht
        try:
            if ht.is_polling():
                ht.stop_state_polling()
        except Exception:
            pass
        try:
            if ht.is_async_rx():
                ht.disable_async_rx()
        except Exception:
            pass
        ht.close()
        self._set_state(RobotState.DISCONNECTED)

    @functools.wraps(_SDK.FafuRobotController.__init__)
    def __init__(self, *args, **kwargs):
        self._gripper_online = True
        try:
            super().__init__(*args, **kwargs)
        except Exception:
            if getattr(self, "_ht", None) is not None:
                try:
                    self.close_connection(joint_release="preserve")
                except Exception:
                    pass
            raise
        if not self._has_gripper:
            self._gripper_online = False
        # Keep the SDK initialization and future fields; narrow only polling.
        try:
            hz = max(10.0, float(self._cfg.control_rate_hz or 50))
            self._ht.start_state_polling(self._controlled_motor_ids(), hz)
        except Exception:
            pass

    @property
    def gripper_online(self) -> bool:
        """``True`` if a gripper was configured and answered the startup ping."""
        return bool(self._has_gripper and self._gripper_online)

    @property
    def state_verbose(self) -> bool:
        """Whether state transitions are printed to stdout (default off)."""
        return self._state_verbose

    def _require_gripper_ready(self, action: str) -> None:
        """Guard for gripper commands (``gripper_control`` / ``open`` /
        ``close``).

        The gripper (motor M7) is an **independent** actuator from the 6
        arm joints, so — unlike joint motion — it is allowed to run
        *during* a joint-streaming session (``SERVOING``) and does not
        change the joint state.  It is blocked only when the arm is
        disconnected, estopped, not enabled, already doing a
        force-``grasp`` (``GRASPING``), or floating in gravity comp.
        An offline gripper (startup ping missed) is rejected immediately
        so the caller is not blocked on a read timeout.
        """
        if not self._gripper_online:
            raise RuntimeError(
                f"{action} 失败: 夹爪 M{self._gripper_motor_id} 启动时未响应，"
                f"当前仅机械臂关节可用。检查夹爪供电 / 接线后重启。"
            )
        st = self._state
        if st in (RobotState.IDLE, RobotState.MOVING, RobotState.SERVOING):
            return
        if st in (RobotState.DISABLED, RobotState.BRAKED):
            try:
                if self.is_enabled:
                    return   # leave the joint state as-is; gripper is orthogonal
            except Exception:
                pass
            raise RobotStateError(
                f"{action} 需要电机已使能, 当前 state={st.name}。请先调用 enable()。"
            )
        if st is RobotState.DISCONNECTED:
            raise RobotStateError(f"{action} 失败: 连接已关闭 (state=DISCONNECTED)。")
        if st is RobotState.DEAD:
            raise RobotStateError(
                f"{action} 被拒绝: 掉电/通信丢失锁定 (DEAD)。"
                "电源恢复后调用 recover(confirm=True)。"
            )
        if st is RobotState.ESTOP:
            raise RobotStateError(
                f"{action} 被拒绝: 处于急停状态 (ESTOP)。请先调用 resume()。"
            )
        if st is RobotState.GRASPING:
            raise RobotStateError(f"{action} 被拒绝: 正在力控抓取 (grasp) 中。")
        if st is RobotState.GRAVITY_COMP:
            raise RobotStateError(f"{action} 被拒绝: 重力补偿 / 拖动示教中。")
        raise RobotStateError(f"{action} 被拒绝: state={st}。")

    def _stream_link_ok(self) -> bool:
        """Cheap per-tick liveness check for high-rate loops (servo_j /
        move_MIT / gravity comp).

        Uses the driver's ``Stats.last_rx_age_ms``: while async RX is running,
        motors losing power stop replying and the age climbs.  If it exceeds
        :attr:`_dead_rx_timeout_ms` we latch ``DEAD`` and return ``False`` so
        the caller stops streaming.  If async RX is off we cannot cheaply tell,
        so we return ``True`` (use :meth:`check_alive` at boundaries instead).
        """
        try:
            if not self._ht.is_async_rx():
                return True
            age = float(getattr(self._ht.get_stats(), "last_rx_age_ms", 0.0))
        except Exception:
            return True
        # Driver reports -1 until the first CAN RX. That is "bus not ready",
        # not mid-stream silence — do not latch DEAD at open.
        if age < 0.0:
            return True
        if age > self._dead_rx_timeout_ms:
            self._enter_dead(
                f"no CAN RX for {age:.0f} ms (motor power / USB / bus lost mid-stream?)")
            return False
        return True

    def recover(self, *, confirm: bool = False) -> bool:
        """Leave ``DEAD`` after motor power / CAN has been restored.

        ★ SAFETY ★ This never moves the arm.  It verifies the motors respond
        again, restarts background RX / polling if needed, forces every motor
        to the safe non-energised ``STOP`` (``0x00``) mode, and transitions to
        ``DISABLED`` — **not** ``IDLE``.  You must then call :meth:`enable`
        explicitly to re-arm; ``enable`` holds the *current* measured pose (no
        jump), so the arm cannot lurch toward a stale pre-blackout target.

        ``confirm=True`` is required because this re-touches the bus after a
        fault; make sure the workspace is clear first.

        Returns ``True`` if the link is back and we moved to ``DISABLED``;
        ``False`` if the motors are still unresponsive (stays ``DEAD``).
        """
        if self._state is not RobotState.DEAD:
            print(f"[FafuRobot] recover: not in DEAD (state={self._state}); ignored.")
            return self._state is not RobotState.DEAD
        if not confirm:
            raise RuntimeError(
                "recover(confirm=True) required (safety): confirm the arm is "
                "powered and the workspace is clear before re-touching the bus.")
        # Restart background tasks if they dropped.
        try:
            if not self._ht.is_async_rx():
                self._ht.enable_async_rx()
                time.sleep(0.1)
        except Exception as e:
            print(f"[FafuRobot] recover: enable_async_rx failed: {e}")
        try:
            if not self._ht.is_polling():
                hz = float(self._cfg.control_rate_hz) if self._cfg.control_rate_hz else 50.0
                self._ht.start_state_polling(
                    self._controlled_motor_ids(), max(10.0, hz)
                )
        except Exception as e:
            print(f"[FafuRobot] recover: start_state_polling failed: {e}")
        # Probe: read fresh; if still nothing, stay DEAD.
        alive = False
        for mid in self._joint_motor_ids:
            try:
                if self._ht.read_motor_state(mid, 0.2) is not None:
                    alive = True
                    break
            except Exception:
                pass
        if not alive:
            print("[FafuRobot] recover: motors still unresponsive; "
                  "check power / USB / CAN and retry. (state stays DEAD)")
            return False
        # Link is back. Force a known-safe non-energised state; never move.
        for mid in self._controlled_motor_ids():
            try:
                self._ht.stop(mid)
            except Exception:
                pass
        self._last_cmd_turns = None
        self._dead_reason = None
        self._set_state(RobotState.DISABLED)
        print("[FafuRobot] recover: link restored; motors left in STOP (0x00, "
              "free). Call enable() to re-arm (holds current pose, no jump) "
              "before commanding motion.")
        return True

    @property
    def is_enabled(self) -> bool:
        """``True`` iff every controlled motor is in position-control mode.

        An offline gripper is excluded so a missing M7 cannot make the
        6-DoF arm look disabled.
        """
        for mid in self._controlled_motor_ids():
            s = self._ht.get_cached_state(mid)
            if s is None:
                s = self._ht.read_motor_state(mid, 0.05)
            if s is None or s.mode != self.MODE_POSITION:
                return False
        return True

    def _enable_impl(self, *, allow_motor_reset: bool = True) -> None:
        """Switch every motor to position control (mode ``0x0A``).

        Resolution path (cheapest to most aggressive):

        0. **Fast path.** Read every motor's live state; if all are
           already in MODE_POSITION (left over from a previous
           ``servo_end("hold")`` or ``close_connection(release="hold")``)
           return without sending any mode-change frame.  This both
           saves ~30 ms and avoids a known false-failure mode of
           ``set_motor_mode(0x0A)`` whose internal "switch + read-back"
           sequence sometimes reports ``None`` on a freshly-reopened
           port even when the mode change actually succeeded.
        1. **Normal path.** Call :meth:`_switch_mode_all` with up to
           3 retries; it now does a fresh ``read_motor_state`` to
           verify failures, filtering out the false-failure case
           above.
        2. **Soft-reboot path.** If the normal path still fails, print
           a diagnostic for every motor (mode / fault / position read
           live), then issue a firmware-level ``motor_reset`` to every
           failed motor (fire-and-forget reset of the on-motor
           controller — does **not** clear zero / config), wait 1 s
           for them to come back up, and try the normal path one more
           time.

        Most cases of "refused mode 0x0A" after a previous
        ``servo_end`` / ``close_connection`` are resolved at step 2
        without a hardware power-cycle.

        Parameters
        ----------
        allow_motor_reset : bool, optional
            Default ``True``.  Set to ``False`` to keep the legacy
            behaviour (raise immediately if step 1 fails).  Useful
            when something else owns the bus and you do not want a
            stealth ``motor_reset``.

        Raises
        ------
        RuntimeError
            Either step 1 (when ``allow_motor_reset=False``) or both
            stages exhausted.  In the latter case the printed
            diagnostic above tells you whether the motors are
            unreachable (no state read), faulted (non-zero ``fault``
            register), or mechanically jammed (mode = 0x0A but the
            commanded position cannot be tracked).
        """
        # Stage-0: fast path. read once, skip the whole switch if every
        # motor is already in MODE_POSITION. This handles the common
        # case of "previous program left motors holding" cleanly and
        # cheaply, and dodges the sync-read false-failure issue.
        already_active = True
        seen_mit_mode = False   # 0x0B = MIT (set_many_mit / 0x8093) 残留模式
        gripper_not_ready = False
        grip_id = (
            int(self._gripper_motor_id)
            if self._has_gripper and self._gripper_motor_id is not None
            else None
        )
        controlled_ids = self._controlled_motor_ids()
        for mid in controlled_ids:
            s = self._ht.read_motor_state(mid, 0.2)
            if s is None or int(s.mode) != self.MODE_POSITION:
                if grip_id is not None and int(mid) == grip_id:
                    gripper_not_ready = True
                    continue
                already_active = False
                if s is not None and int(s.mode) == self.MODE_MIT:
                    seen_mit_mode = True
                break
        if already_active:
            if gripper_not_ready:
                self._mark_gripper_offline("not in position while joints hold")
            print("[FafuRobot] all motors already in position control hold; "
                  "enable() is a no-op.")
            return

        # MIT 残留 (mode 0x0B): 实测本固件下 set_motor_mode(0x0A) 切不动 (哪怕用
        # per-motor 位置帧隐式切也失败, 见 set_motor_mode 注释), 必须 motor_reset
        # 软重启才能回到位置模式. 常规 _switch_mode_all 的 3 次重试注定失败, 只会
        # 白白拉长"电机掉电 -> 下垂"的窗口. 检测到 0x0B 就直接跳到 motor_reset.
        if seen_mit_mode and allow_motor_reset:
            print("[FafuRobot] enable: MIT residual mode (0x0B) detected; "
                  "motor_reset directly (skip doomed mode-switch retries) ...")
            for mid in controlled_ids:
                try:
                    self._ht.motor_reset(mid)
                except Exception as e:
                    print(f"  motor {mid}: motor_reset failed: {e}")
            # 与 stage-2 一致用 1.0s: 上次日志 J2/J3/J4 在 0.6s 后仍卡在 0x0B.
            time.sleep(1.0)
            if self._switch_mode_all(self.MODE_POSITION,
                                     label="position (post-MIT-reset)", max_retry=3):
                time.sleep(0.05)
                print("[FafuRobot] all motors enabled (recovered via motor_reset).")
                return

            # 部分电机 reset 后仍卡在 0x0B -> 对这些电机做几轮更"用力"的重试:
            # 每轮对每个卡住电机 stop()(踢一下) + 连发 3 次 motor_reset (文本
            # 后门命令 fire-and-forget, 可能丢帧, 多发几次更稳), 再等 1.2s.
            for rnd in range(1, 4):
                still_mit: List[int] = []
                for mid in controlled_ids:
                    try:
                        s = self._ht.read_motor_state(mid, 0.2)
                        if s is not None and int(s.mode) == self.MODE_MIT:
                            still_mit.append(mid)
                    except Exception:
                        still_mit.append(mid)
                if not still_mit:
                    break
                print(f"[FafuRobot] enable: motors {still_mit} still in MIT 0x0B; "
                      f"aggressive motor_reset round {rnd} (stop + reset x3) ...")
                for mid in still_mit:
                    try:
                        self._ht.stop(mid)
                    except Exception:
                        pass
                    for _ in range(3):
                        try:
                            self._ht.motor_reset(mid)
                        except Exception as e:
                            print(f"  motor {mid}: motor_reset failed: {e}")
                        time.sleep(0.05)
                time.sleep(1.2)
                if self._switch_mode_all(self.MODE_POSITION,
                                         label=f"position (post-MIT-reset-{rnd + 1})",
                                         max_retry=3):
                    time.sleep(0.05)
                    print(f"[FafuRobot] all motors enabled "
                          f"(recovered via motor_reset round {rnd + 1}).")
                    return

            # 反复软重启仍有电机卡在 0x0B -> 本固件把 MIT 模式锁死, 纯软件救不回,
            # 直接给出断电指引 (落到 stage-2 也只是再白等一遍).
            stuck = []
            for mid in controlled_ids:
                try:
                    s = self._ht.read_motor_state(mid, 0.2)
                    if s is not None and int(s.mode) == self.MODE_MIT:
                        stuck.append(mid)
                except Exception:
                    pass
            if stuck:
                self._print_motor_diagnostic(
                    prefix="enable: MIT 0x0B stuck after repeated motor_reset; ")
                raise RuntimeError(
                    f"enable failed: motors {stuck} stuck in MIT mode (0x0B) and "
                    f"will not respond to motor_reset. This firmware locks the MIT/"
                    f"运控 mode so tightly that only a HARD POWER-CYCLE of the arm "
                    f"clears it. Please power the arm off and on, then rerun. "
                    f"(To avoid this: after using MIT/持续保持, press [y] 软重启 "
                    f"before quitting so motors are left in position mode 0x0A.)"
                )

            # (理论上到不了这里) 快速路径没完全恢复 -> 落到 stage-2 全量 reset.
            print("[FafuRobot] enable: MIT fast-path reset incomplete; "
                  "falling through to full motor_reset recovery ...")

        if self._switch_mode_all(self.MODE_POSITION, label="position", max_retry=3):
            time.sleep(0.05)
            print("[FafuRobot] all motors enabled (position control hold).")
            return

        # A faulted/offline gripper must not motor_reset J1–J6. That 1 s
        # stall on the 100 Hz writer kills servo_start; the UI still shows
        # joints enabled, so the arm looks armed but never moves.
        if self._joint_motors_position_ready():
            self._mark_gripper_offline("blocked enable; joints already holding")
            print("[FafuRobot] joints are in position control; continuing without gripper")
            return

        # Stage-1 failed. Print a diagnostic so the next message is
        # actionable instead of a generic "refused".
        self._print_motor_diagnostic(prefix="enable: stage-1 failed; ")

        if not allow_motor_reset:
            raise RuntimeError(
                "enable failed: at least one motor refused mode 0x0A "
                "(motor_reset recovery disabled by allow_motor_reset=False)"
            )

        # Stage-2: motor_reset is a firmware-level soft reboot of the
        # on-motor controller. It is the equivalent of pulling power on
        # *only that motor* for a few hundred ms, without losing zero
        # calibration / soft limits / etc. Use as last resort before
        # asking the user to power-cycle.
        print("[FafuRobot] enable: attempting motor_reset on every motor (soft reboot) ...")
        reset_ids = [
            mid for mid in controlled_ids
            if grip_id is None or int(mid) != grip_id
        ]
        for mid in reset_ids:
            try:
                self._ht.motor_reset(mid)
            except Exception as e:
                print(f"  motor {mid}: motor_reset failed: {e}")
        # Firmware needs time to come back. Empirically ~0.6 s is
        # enough on the boards we have; 1.0 s gives margin.
        time.sleep(1.0)

        if not self._switch_mode_all(self.MODE_POSITION, label="position (post-reset)", max_retry=3):
            if self._joint_motors_position_ready():
                self._mark_gripper_offline("still blocking after motor_reset")
                time.sleep(0.05)
                print("[FafuRobot] joints enabled; continuing without gripper")
                return
            self._print_motor_diagnostic(prefix="enable: stage-2 (post-reset) failed; ")
            raise RuntimeError(
                "enable failed even after motor_reset; check the "
                "diagnostic above. Likely causes: (a) motor controller "
                "in latched FAULT state -> hard power-cycle; "
                "(b) USB-CAN bus disconnected / wrong COM port; "
                "(c) mechanical jam holding the joint outside soft limits."
            )

        time.sleep(0.05)
        print("[FafuRobot] all motors enabled (recovered via motor_reset).")

    def servo_start(self, opts: Optional[ServoOpts] = None) -> None:
        """Enter an online-streaming joint-servo session.

        Idempotent on the second call (warns and returns).  After this
        every joint motor has a firmware watchdog armed (1) and the
        wrapper has cached the current joint configuration as the
        initial ``last_target`` (so the first :meth:`servo_j` cannot
        trip the step-clamp warning).  The gripper is **not** touched
        and is not subject to the servo watchdog.

        Parameters
        ----------
        opts : ServoOpts, optional
            Tunables.  Defaults to :class:`ServoOpts` (watchdog 100 ms,
            max_vel 1.0 rad/s, max_step 0.05 rad, max_lag 0.2 rad).

        Raises
        ------
        RuntimeError
            If the driver fails to enable async RX (servo cannot run on
            the blocking sync-RX path) or any joint motor fails to
            report a starting position.
        """
        if self._servo_active:
            print("[FafuRobot] servo_start: already servoing; call servo_end first")
            return

        # State-machine guard. ``allow_disabled`` because servo_start
        # enables the motors itself below; but reject if disconnected /
        # estopped / busy with another operation.
        self._require_ready("servo_start", allow_disabled=True)

        opts = ServoOpts(**vars(opts)) if opts is not None else ServoOpts()
        if opts.watchdog_ms < 0:
            opts.watchdog_ms = 0
        if opts.watchdog_ms == 0:
            print("[FafuRobot] servo_start: WARNING watchdog disabled; "
                  "host crash will not stop motors. Use only for offline tests.")
        elif opts.watchdog_ms < 30:
            print(f"[FafuRobot] servo_start: WARNING watchdog_ms={opts.watchdog_ms} "
                  "< 30 may trip spuriously at 100Hz; recommend >= 50 ms.")
        if opts.max_vel <= 0.0:
            opts.max_vel = 1.0
        if opts.max_step_rad <= 0.0:
            opts.max_step_rad = 0.05
        if opts.rate_hz <= 0.0:
            opts.rate_hz = 100.0
        if opts.lookahead_time < 0.0:
            opts.lookahead_time = 0.0

        if not self.is_enabled:
            print("[FafuRobot] servo_start: motors not enabled, calling enable() ...")
            self.enable()

        # Servo cannot tolerate the 5-15 ms sync RX wait inside
        # send_can_and_recv_, so force async RX on if it is not already.
        if not self._ht.is_async_rx():
            try:
                self._ht.enable_async_rx()
                time.sleep(0.05)
            except Exception as e:
                raise RuntimeError(f"servo_start: enable_async_rx failed: {e}") from e

        # A servo session moves motors outside the move_j S-curve, so any
        # previously-cached commanded pose is no longer valid; force the
        # next move_j to start from the measured pose.
        self._last_cmd_turns = None

        # Capture current pos as last_target so the first servo_j is a
        # zero-step move and never trips the step-clamp warning.
        last_target: List[float] = []
        for mid in self._joint_motor_ids:
            s = self._ht.get_cached_state(mid)
            if s is None:
                s = self._ht.read_motor_state(mid, 0.1)
            if s is None:
                raise RuntimeError(
                    f"servo_start: cannot read motor {mid} starting position")
            last_target.append(s.position)

        # Pre-build the numpy arrays consumed by
        # ``set_many_pos_vel_tqe_partial`` so the per-tick allocation
        # cost is zero (we only memcpy target_pos / target_vel into
        # the pre-allocated buffers below; active_ids / hold_ids never
        # change for a session).
        active_ids_np = np.asarray(self._joint_motor_ids, dtype=np.int32)
        # Non-joint motors are normally "held" at cached pos each tick.
        # The gripper must be excluded: otherwise every servo_j overwrites
        # M7 back to the position captured at servo_start, canceling
        # open_gripper/close_gripper (non-blocking) commands.
        hold_motor_ids = [
            mid for mid in self._cfg.motor_ids if mid not in self._joint_motor_ids
        ]
        if self._has_gripper and self._gripper_motor_id is not None:
            hold_motor_ids = [
                mid for mid in hold_motor_ids if mid != self._gripper_motor_id
            ]
        hold_ids_np = np.asarray(hold_motor_ids, dtype=np.int32)
        # Pre-allocated output buffers reused every tick.
        N = len(self._joint_motor_ids)
        active_pos_np = np.zeros(N, dtype=np.float64)
        active_vel_np = np.zeros(N, dtype=np.float64)

        # Arm the firmware watchdog on every JOINT motor (gripper excluded
        # — gripper is not part of the servo loop, watching it would brake
        # the jaws while the loop is still in start-up).
        if opts.watchdog_ms > 0:
            for mid in self._joint_motor_ids:
                try:
                    self._ht.set_timeout(mid, int(opts.watchdog_ms))
                except Exception as e:
                    print(f"[FafuRobot] servo_start: set_timeout({mid}) failed: {e}")

        # If MIT channel requested, resolve + convert kp/kd to raw ONCE (per
        # vendor kp_float2int, radian_2pi) so the per-tick hot path only does a
        # cheap gravity-ff call + one set_many_mit. Physical gains default to
        # the vendor replay values (6-DoF) or a scalar.
        self._servo_mit_kp_raw: List[int] = []
        self._servo_mit_kd_raw: List[int] = []
        # MIT frame slot count = max JOINT motor id (gripper excluded), so a
        # 6-joint arm builds a 6-slot 62B frame, not a 7-slot 72B (>64) one.
        self._servo_mit_max_motor_id = int(max(self._joint_motor_ids))
        if opts.use_mit:
            if self.num_joints > 6:
                raise RuntimeError(
                    f"servo use_mit needs <=6 joints, but num_joints="
                    f"{self.num_joints}; exclude the gripper.")
            kp = opts.mit_kp
            kd = opts.mit_kd
            nj = self.num_joints
            if kp is None:
                kp = ([30.0, 40.0, 55.0, 15.0, 7.0, 5.0][:nj] if nj == 6 else 20.0)
            if kd is None:
                kd = ([3.0, 4.0, 5.5, 1.5, 0.7, 0.5][:nj] if nj == 6 else 2.0)
            kp_v = ([float(kp)] * nj if np.isscalar(kp)
                    else [float(x) for x in kp])
            kd_v = ([float(kd)] * nj if np.isscalar(kd)
                    else [float(x) for x in kd])
            for i in range(nj):
                model = (self._dyn_motor_models[i]
                         if self._dyn_motor_models is not None else "")
                coeff = TORQUE_COEFF.get(model, 1.0) or 1.0
                self._servo_mit_kp_raw.append(
                    int(np.clip(round((kp_v[i] / coeff) * 10.0 * _TWO_PI),
                                -32768, 32767)))
                self._servo_mit_kd_raw.append(
                    int(np.clip(round((kd_v[i] / coeff) * 10.0 * _TWO_PI),
                                -32768, 32767)))
            if opts.mit_gravity_ff and not self.has_dynamics:
                print("[FafuRobot] servo_start: use_mit gravity_ff requested but "
                      "no dynamics model; MIT will run kp/kd-only.")

        self._servo_opts = opts
        self._servo_last_target_turns = list(last_target)
        self._servo_filtered_target_turns = list(last_target)
        self._servo_active_ids_np = active_ids_np
        self._servo_hold_ids_np = hold_ids_np
        self._servo_active_pos_np = active_pos_np
        self._servo_active_vel_np = active_vel_np
        # Position frames use 6 bytes per motor, so M7 fits: 7*6+2 = 44B.
        # Include its slot but leave it absent from active_ids/hold_ids, which
        # makes the C++ encoder write NAN_INT16 ("no operation"). This is
        # necessary even though the gripper is not controlled by servo_j:
        # six slots plus the query are rounded to a 48B CAN-FD payload, and
        # some M7 firmware interprets the following 0x50 padding bytes as its
        # own position/velocity/torque command. An explicit NAN slot prevents
        # the gripper from driving whenever the arm stream sends 0x8090.
        #
        # MIT remains capped at six joints above because it uses 10B per slot;
        # seven MIT slots plus the query would be 72B and exceed CAN-FD.
        frame_ids = list(self._joint_motor_ids) + [int(m) for m in hold_motor_ids]
        if (
            not opts.use_mit
            and self._has_gripper
            and self._gripper_motor_id is not None
        ):
            frame_ids.append(int(self._gripper_motor_id))
        self._servo_max_motor_id = int(max(frame_ids)) if frame_ids else int(
            max(self._joint_motor_ids)
        )
        self._servo_max_torque = int(self._cfg.max_torque_raw)
        self._servo_active = True
        self._set_state(RobotState.SERVOING)
        self._servo_tick_count = 0
        self._servo_clamp_count = 0
        self._servo_lag_count = 0
        self._servo_lag_streak = 0
        self._servo_lag_warned = False
        self._servo_aborted_reason = None
        self._servo_started_at = time.monotonic()

        ff_tag = "on" if opts.feedforward_vel else "off"
        la_tag = (f"{opts.lookahead_time * 1000:.0f}ms"
                  if opts.lookahead_time > 0 else "off")
        chan = ("MIT 0x8093 kp={} kd={} (raw)".format(
                    self._servo_mit_kp_raw, self._servo_mit_kd_raw)
                if opts.use_mit else "pos+vel+maxtqe 0x8090")
        print(
            f"[FafuRobot] servo_start: watchdog={opts.watchdog_ms}ms, "
            f"max_vel={opts.max_vel}rad/s, max_step={opts.max_step_rad}rad, "
            f"max_lag={opts.max_lag_rad}rad, "
            f"feedforward={ff_tag}, lookahead={la_tag}, rate={opts.rate_hz:.0f}Hz, "
            f"channel={chan}"
        )

    def get_motor_states(self, *, prefer_cache: bool = True) -> Dict[int, "pm.MotorState"]:
        """Return raw :class:`MotorState` objects keyed by motor id."""
        return self._read_states(self._controlled_motor_ids(), prefer_cache=prefer_cache)

    @staticmethod
    def _resolve_cfg_path(cfg_path: str) -> str:
        if not (os.path.isabs(cfg_path) or os.path.exists(cfg_path)):
            candidate = os.path.join(_HERE, cfg_path)
            if os.path.exists(candidate):
                cfg_path = candidate
        try:
            from robot_station.adapters.fafu_arm import native_openable_path

            return native_openable_path(cfg_path)
        except Exception:
            return cfg_path

    def _pick_serial_port(self, preferred: Optional[str]) -> str:
        from robot_station.adapters.fafu_arm import resolve_live_serial_port

        pref = (preferred or "").strip()
        if pref and pref.lower() != "auto":
            return resolve_live_serial_port(pref, [])
        try:
            usb_ports = pm.find_likely_debug_boards()
        except Exception as e:
            raise RuntimeError(f"无法枚举串口，请明确选择端口: {e}") from e
        return resolve_live_serial_port(pref, usb_ports)

    def _mark_gripper_offline(self, reason: str) -> None:
        """Keep joint motion; a bad M7 is a warning, not an arm fault."""
        gid = self._gripper_motor_id
        was = bool(self._gripper_online)
        self._gripper_online = False
        if gid is not None:
            missing = [int(x) for x in (getattr(self, "_missing_motors", None) or [])]
            if int(gid) not in missing:
                missing.append(int(gid))
                self._missing_motors = missing
        if was:
            print(
                f"[FafuRobot] warning: gripper M{gid} {reason}; "
                f"joint 1-6 motion continues"
            )

    def _joint_motors_position_ready(self) -> bool:
        """True when every joint (not the gripper) is in MODE_POSITION."""
        if not self._joint_motor_ids:
            return False
        for mid in self._joint_motor_ids:
            s = self._ht.get_cached_state(mid)
            if s is None:
                s = self._ht.read_motor_state(mid, 0.05)
            if s is None or int(s.mode) != self.MODE_POSITION:
                return False
        return True

    def _controlled_motor_ids(self) -> List[int]:
        """Motors that must be on the bus for enable / mode / polling.

        Joints are always included. The gripper is included only while it
        answered the startup ping and is not latched in fault/offline.
        """
        ids = list(self._joint_motor_ids)
        if not (self._has_gripper and self._gripper_online and self._gripper_motor_id is not None):
            return ids
        gid = int(self._gripper_motor_id)
        try:
            cfg_ids = {int(m) for m in list(self._cfg.motor_ids)}
        except Exception:
            cfg_ids = set()
        # Station may have dropped M7 from cfg so joint servo can run.
        # Do not put it back into enable / is_enabled / 0x8090 slot count.
        if cfg_ids and gid not in cfg_ids:
            return ids
        extra = {int(x) for x in (getattr(self, "_station_offline_ids", None) or [])}
        extra.update(int(x) for x in (getattr(self, "_missing_motors", None) or []))
        s = None
        try:
            s = self._ht.get_cached_state(gid)
        except Exception:
            s = None
        unusable = gid in extra
        if s is not None:
            if hasattr(s, "online") and not bool(s.online):
                unusable = True
            if int(getattr(s, "fault", 0) or 0):
                unusable = True
        if unusable:
            self._mark_gripper_offline("offline or faulted")
            return ids
        ids.append(gid)
        return ids

    def _precheck_communication(self) -> None:
        """Ping the bus. Missing axes are warnings; never fail Connect.

        The gripper is independent. A silent joint is marked offline.
        All joints silent at open usually means the bus is not ready yet,
        not that the debug-board link is down — do not raise.
        """
        gripper_id = self._gripper_motor_id if self._has_gripper else None
        bad_joints: List[int] = []
        joint_ids: List[int] = []
        gripper_missing = False
        for mid in self._cfg.motor_ids:
            s = self._ht.read_motor_state(mid, 0.5)
            if gripper_id is not None and mid == gripper_id:
                if s is None or int(getattr(s, "fault", 0) or 0):
                    gripper_missing = True
                elif hasattr(s, "online") and not bool(s.online):
                    gripper_missing = True
                continue
            joint_ids.append(int(mid))
            if s is None:
                bad_joints.append(mid)
        if gripper_missing:
            self._mark_gripper_offline("did not respond or is faulted")
        if not bad_joints:
            return
        if joint_ids and len(bad_joints) < len(joint_ids):
            self._missing_motors = list(bad_joints)
            print(
                f"[FafuRobot] warning: motors {bad_joints} did not respond "
                f"within 500ms; joints stay offline, link stays up"
            )
            return
        print(
            f"[FafuRobot] warning: motors {bad_joints} did not respond "
            f"within 500ms; keeping debug-board link (bus may still be waking)"
        )

    def _switch_mode_all(
        self,
        mode: int,
        *,
        label: str,
        max_retry: int = 3,
    ) -> bool:
        """Drive every controlled motor to ``mode``, verifying the actual state.

        An offline gripper is skipped so a missing M7 cannot fail enable
        of the 6-DoF arm.  Returns True as soon as every motor's
        *measured* state is at ``mode``.  This intentionally trusts the
        post-switch read rather than ``set_motor_mode``'s return value,
        because that return value can legitimately be ``None`` on a busy
        / freshly-reopened port even when the mode change succeeded
        (the binding's internal "switch -> sleep -> read confirm"
        sequence is timing-sensitive).  We back-stop by issuing a
        fresh read on every motor that the binding flagged as
        failing, and only treat it as a real failure if the read
        confirms a wrong mode.
        """
        for attempt in range(1, max_retry + 1):
            failed: List[int] = []
            for mid in self._controlled_motor_ids():
                s = self._ht.set_motor_mode(mid, mode)
                if s is None or int(s.mode) != mode:
                    failed.append(mid)

            # ★ Post-failure verify: do a fresh read on every "failed"
            # motor to filter out binding false negatives. If the read
            # confirms the target mode is already there, the motor is
            # actually fine — common case after a sync-read timeout
            # inside set_motor_mode(0x0A).
            if failed:
                time.sleep(0.03)
                really_failed: List[int] = []
                for mid in failed:
                    s = self._ht.read_motor_state(mid, 0.3)
                    if s is None or int(s.mode) != mode:
                        really_failed.append(mid)
                failed = really_failed

            gid = (
                int(self._gripper_motor_id)
                if self._has_gripper and self._gripper_motor_id is not None
                else None
            )
            if gid is not None and gid in failed:
                failed = [m for m in failed if int(m) != gid]
                self._mark_gripper_offline("did not enter requested mode")

            if not failed:
                return True
            if attempt < max_retry:
                print(
                    f"[FafuRobot] switch to {label}: retry {attempt + 1}, "
                    f"failed motors = {failed}"
                )
                time.sleep(0.1)
        return False

    def _move_linear_sync(
        self,
        targets_turns: Dict[int, float],
        *,
        speed_pct: int,
        duration: Optional[float] = None,
        tolerance_turns: float = _MOVE_J_TOLERANCE_RAD / _TWO_PI,
        timeout_s: float = 10.0,
        block: bool = True,
        abort_check: Optional[Callable[[], bool]] = None,
    ) -> None:
        """Official-style synchronized *linear* move.

        Synchronized-arrival move: set per-joint velocity for a common
        arrival time, broadcast pos+vel+max-torque, then wait for arrival:

        1. read the current position of every motor,
        2. pick ``v_i = (target_i - current_i) / duration`` so all joints
           finish together,
        3. broadcast the ``set_many_pos_vel_tqe`` frame (the motors'
           on-board loop drives toward the target with that velocity as
           feed-forward / target speed),
        4. when ``block`` is ``True``, **keep re-sending that same frame**
           while polling the measured positions until every targeted joint
           is within ``tolerance_turns`` or ``timeout_s`` elapses.

        IMPORTANT: this uses the same ``set_many_pos_vel_tqe`` (CAN ID
        ``0x8090``, firmware ``MODE_POS_VEL_TQE``) path as the S-curve
        ``move_j``.  On this FDCAN bridge that channel is a **streaming**
        channel: a single one-shot frame does *not* sustain motion (and a
        watchdog left over from a prior MIT/servo session would kill it
        outright), so we must refresh the frame continuously — exactly like
        :meth:`_move_scurve`'s ``run_control_loop`` and :meth:`servo_j`.
        The difference vs S-curve is only the profile: linear holds a
        constant target + velocity cap (harder start/stop, official
        ``jointsSyncArrival`` semantics) instead of a cosine ease-in/out.
        """
        # 1) fresh current positions for manipulator joints only.  The gripper
        #    is an independent actuator and must not be re-commanded by a
        #    move_j frame: doing so would re-enable a J7 that replay safety
        #    deliberately braked after closing, causing sustained force/heat.
        meas_pos: Dict[int, float] = {}
        for mid in self._joint_motor_ids:
            s = self._ht.read_motor_state(mid, 0.1)
            if s is None:
                raise RuntimeError(
                    f"could not read motor {mid} starting position; "
                    f"aborting move_j(style=linear) for safety"
                )
            meas_pos[mid] = s.position

        # 2) duration: explicit, or derived from speed so the joint with
        #    the largest delta travels at <= v_avg_target turns/s.
        v_avg_target = (speed_pct / 100.0) * _VEL_AVG_MAX_TPS
        max_abs_dpos = 0.0
        for mid, tgt in targets_turns.items():
            max_abs_dpos = max(max_abs_dpos, abs(float(tgt) - meas_pos[mid]))
        if duration is None:
            if max_abs_dpos < 1e-5:
                dur = _DT_MIN_S
            else:
                dur = max(_DT_MIN_S, max_abs_dpos / max(v_avg_target, 1e-3))
        else:
            dur = max(_DT_MIN_S, float(duration))

        # 3) v_i = (target - current) / duration.  Only J1..Jn are included;
        #    omitted higher slots are protocol NAN/no-op and leave J7 alone.
        max_torque = int(self._cfg.max_torque_raw)
        cmds: List[pm.ManyMotorCmd] = []
        for mid in self._joint_motor_ids:
            if mid in targets_turns:
                tgt = float(targets_turns[mid])
                vel = (tgt - meas_pos[mid]) / dur
                cmds.append(pm.ManyMotorCmd(mid, tgt, vel, max_torque))
            else:
                cmds.append(pm.ManyMotorCmd(mid, meas_pos[mid], 0.0, max_torque))

        max_mid = max(self._joint_motor_ids)
        # first broadcast == posVelMaxTorque() + motor_send_cmd()
        self._ht.set_many_pos_vel_tqe(cmds, pm.PosUnit.Turns, max_mid, 0.05)

        last_cmd: Dict[int, float] = dict(self._last_cmd_turns or {})
        for mid in self._joint_motor_ids:
            last_cmd[mid] = (
                float(targets_turns[mid]) if mid in targets_turns else meas_pos[mid]
            )

        if not block:
            self._last_cmd_turns = last_cmd
            return

        # 4) waitForPosition: keep re-sending the (constant) frame to sustain
        #    motion on the streaming 0x8090 channel while polling until every
        #    targeted joint settles.  A single one-shot frame does NOT move
        #    the joint here (see docstring), so the resend is mandatory.
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        reached = False
        while time.monotonic() < deadline:
            if abort_check is not None and abort_check():
                self._last_cmd_turns = None
                raise RuntimeError(
                    "move_j(style=linear) aborted (abort_check returned True)"
                )
            # keep-alive resend (fire-and-forget, no reply wait) — refreshes
            # the position loop / watchdog exactly like the S-curve loop.
            self._ht.set_many_pos_vel_tqe(cmds, pm.PosUnit.Turns, max_mid, 0.0)
            ok = True
            for mid, tgt in targets_turns.items():
                s = self._ht.read_motor_state(mid, 0.1)
                if s is None or abs(s.position - float(tgt)) > tolerance_turns:
                    ok = False
                    break
            if ok:
                reached = True
                break
            time.sleep(0.02)

        if not reached:
            print(
                f"[FafuRobot] move_j(style=linear): not settled within "
                f"{timeout_s:.1f}s. If the residual is small (~1 deg) it is "
                f"the pos_vel_MAXtqe gravity steady-state error; if the joint "
                f"barely moved, raise speed/max_torque or check for a leftover "
                f"watchdog from a prior MIT/servo session."
            )
        self._last_cmd_turns = last_cmd
