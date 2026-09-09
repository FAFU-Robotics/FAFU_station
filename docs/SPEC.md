# FAFU 机械臂站控 · 规格

本文是实现合同。改功能先改本文。

## 范围

本站**只负责机械臂与作业相机**。客户 PC 通过 USB 直连机械臂调试板，软件在本机运行。

机械臂协议与控制走官方 [fafu_arm_sdk](https://github.com/FAFU-Robotics/fafu_arm_sdk)。界面布局学 [Panthera-HT Host](https://github.com/HighTorque-Robotics/Panthera-HT_Host)。

## 形态

交付物是**客户 PC 上的本机宿主**：

1. **客户交付（推荐）**：`packaging/windows/build_portable.ps1` 生成 `dist/FAFUArmStation/`。客户解压后双击 `FAFUArmStation.exe` 或 `启动真机.bat`：第一次拷到 `%LOCALAPPDATA%\FAFUArmStation`（不写 PATH），并打开窗口；以后点桌面快捷方式即可。启动真机在 USB 连上后自动 Connect 并使能（SDK 构造仍 `auto_enable=False`）。页面切「仿真臂」再切回真机仍不自动使能。
2. **开发机**：无便携 `runtime\python310` 时，双击 `启动真机.bat`（本机 Python 3.10）。一次动作拉起本机宿主，并打开**独立应用窗口**（不是系统浏览器）。关窗口不结束真机运动进程（9470 继续 100 Hz hold）；窗口只是查看器，再开即复用。无真机时关窗口仍结束本机宿主。页面右侧 **臂源** 可在真机 USB 与仿真臂之间切换，无需第二个启动脚本。
3. 本机进程托管网页：默认 `http://127.0.0.1:9400/`。
4. 真机时 PC 用 USB 串口连接机械臂调试板（`fafu_motor` + `FafuRobotController`）。官方 SDK **没有 TCP/以太网控臂**：链路是 FafuRobotController → HightorqueSerial → USB 串口（robot.cfg 的 port=auto / COMx，波特率 4 Mbps）→ 调试板 CAN-FD。不能把电机环改成网口直连。
5. 无真机时在同一页面切「仿真臂」（`arm_src=sim`），或命令行 `arm: mock` / `arm: sim` 调试界面。站控内部网页进程↔运动进程已经是 `127.0.0.1:9470` TCP（`TCP_NODELAY`）。该链路上 **100 Hz 是 88 字节二进制姿态**，完整 JSON 快照约 **10 Hz**；流式关节/笛卡尔/示教指令 latest-wins，不在 TCP 里排队。浏览器不能裸 TCP：指令走 `/ws/cmd`，姿态走 `/ws` 二进制，顶栏 RTT 走独立 `/ws/rtt`（Worker 线程测，不跟 3D/视频抢主线程）。指令闭环用 `echo_t0` 回显，只认新值。发指令时暂停假视频编码与推送。本机闭环应经常 < 20 ms。

默认进 **机械臂** 页。总览保留，不作默认。作业相机嵌在臂页里，没有独立相机页。

## 第一期「做完」

- 臂页布局学 [Panthera-HT_Host](https://github.com/HighTorque-Robotics/Panthera-HT_Host)：Joints、Connection、CONTROL MODE、夹爪。正中是 **3D 参考臂**（Three.js；关节链取 SDK `fafu_baseV1.urdf`，与旧名 `fafu_follower.urdf` 相同；外观用 SDK SolidWorks STL 的降采样网格 `/static/meshes/*.bin`，按面积保留大面以免抽稀出破洞，加载失败则回退圆柱）。相机按网格包络自动框选，双击视口可再框选。作业相机一路（接本机笔记本 USB），画面在 3D 下方略放大，可用「3D」「相机」开关。3D 姿态跟 100 Hz 二进制 `q_deg` / `gripper_deg` / `gripper_open`（JSON HUD 10 Hz 不得覆盖）。`link6` STL 按张开姿态导出，3D 拆出两侧指尖并按开合平移（指尖内侧约 45 mm，闭合行程约 42 mm）。开合跟 `gripper_deg` 模拟量，3D 按行程插补（约 0.4 s 追上跳变），禁止指尖瞬移；`gripper_open` 不得一拍把指尖打到限位。仿真夹爪约 90°/s 插补到目标，不得一拍打到限位。CAD 上色用第一版分段浅灰（`0x8a9199`…`0xc5cdd4`），不用 URDF 统一 0.75 银灰，也禁止另造黑白分段。导出仍写 `/static/meshes/urdf_visual.json` 备查，3D 不采用。
- 真臂：官方 [fafu_arm_sdk](https://github.com/FAFU-Robotics/fafu_arm_sdk)。`fafu_motor` 须与启动站控的 CPython ABI 一致（仓库预编译为 Windows **cp310**；本机 3.11 不能载入，真机用 `启动真机.bat` / `py -3.10`）。连接必须 `auto_enable=False`。
- **真机串口许可保留**：未设置 `STATION_ALLOW_LIVE_ARM=1` 时，`arm: fafu` / `auto` 被改成 `sim`，真 `FafuRobotController` 不会被构造。默认 yaml 仍是 `arm: mock`，`arm_allow_motion: false`。
- **真机运动路径**（两道闸都要开）：`set STATION_ALLOW_LIVE_ARM=1`（Linux：`export`），且 `arm: fafu`（或 `--arm fafu`）与 `arm_allow_motion: true`（或 `--allow-motion`）。只开第一道闸、第二道仍 false：开串口只读，不 `enable` / 不下发。
- `arm: sim`：FafuArm + FakeFafuController，方法名与真机相同，**不开 USB**。关节插补后的末端用与 [fafu_arm_sdk](https://github.com/FAFU-Robotics/fafu_arm_sdk) `fafu_follower.urdf` 相同的 FK（`tool_link`）。网页拉运动子进程必须带 `--arm`。
- 滑条限位来自 `robot.cfg`（与 SDK 一致），经快照下发给前端。点「发送位置」，或勾选「实时跟随」（`servo_start(ServoOpts)` + `servo_j`）。未勾选时拖关节滑条只改页面目标，不发 servo。禁止因拖滑条而自动勾选实时跟随（真机与仿真同一套交互）。勾选后才 100 Hz 跟随；真机速度受页面 Speed（deg/s）限制，不会一拍甩到目标。
- **空间切换（页面按钮，不是 CONTROL MODE）**：Joints 卡顶栏 `关节空间` / `笛卡尔`。关节空间是 J1–J6 滑条。笛卡尔是末端 **X/Y/Z（米）与滚转/俯仰/偏航（度）滑条**。拖动只改目标，**松手或点「发送位置」** 才下发。仿真与真机都走限速 `servo_j` 路点；**禁止 `move_j(block=False)`**（一拍打向目标并锁存非零速度），高位降肩时先折肘。取消「实时跟随」、松开 WASD 后发 `t: park`：同姿态继续 100 Hz `servo_j`（喂固件看门狗），不要 `servo_end`，也不要在 SERVOING 里调 `enable()`。浏览器不算 IK。切到笛卡尔时关掉关节实时跟随。键盘 WASD 仍是增量遥操作（`t: cart`）。
- Keyboard：W/S=X、A/D=Y、Q/E=Z；1–6 为末端系右乘旋转（与 SDK `test_fafu_keyboard_cartesian` 相同）。夹爪开合统一 **空格切换**（按当前 `gripper_open` 取反）；急停只用 `Esc`，空格不再急停。输入框打字时空格照常输入。真机优先 SDK `setup_dynamics`（pinocchio）。未装 pinocchio 时笛卡尔仍须可解：100 Hz WASD 只用站控 URDF 阻尼最小二乘（不得在写者线程跑 Trac-IK）。已安装 `pytracik` 时，页面笛卡尔「发送位置」（`cart_go`）在 DLS 无解后可再用一次 Trac-IK。不得因缺 pinocchio 直接丢键。重力/阻抗用站控 URDF 质量/质心算 G(q) 经 MIT 下发（Trac-IK 不能代替；有 pinocchio 时仍走 SDK `start_gravity_compensation`）。零位靠近奇异时允许位置优先（放宽姿态）以免 WASD 无解。滑条焦点不得吞掉方向键。键盘与「实时跟随」共用 servo 流：按键期间暂停跟随脉冲，松手后把滑条同步到实测角再恢复跟随。页面笛卡尔滑条走 `t: cart_go`（等 ACK：IK 后限速 servo 路点），拖动过程不下发。`move_j` / 复位 / 路点须等 100 Hz 写者线程结束 servo 会话，失败要回给网页，不得静默丢指令。笛卡尔增量在运动进程按 tick 合并，IK 从**指令位姿**累加（真机从已限速的 `servo` 指令角累加，避免松键后臂继续猛追）。100 Hz 下发最新目标；**真机**每拍 `servo_j` 按界面 Speed（deg/s）限幅，禁止把滑条大跳或 IK 大步一次性打到电机。仿真 Position 仍可本拍落到指令角。真机与仿真「发送位置」都走限速 servo 路点（禁止先 `servo_end` 再 `move_j`，那会多约 120 ms）。网页每 100 ms `touch` 不得单独维持运动意图：跟随/键盘停止后约 0.35 s `t: park`，会话保持，同姿态 `servo_j`。SERVOING 期间禁止 `enable()`（官方 SDK 会 `RobotStateError` 正忙）；`is_enabled()` 不得在 100 Hz 写者上阻塞读。真机 `ServoOpts`：`rate_hz=100`，`lookahead_time=0`，`max_vel` / `max_step_rad` 按 UI 最大约 80°/s 做硬件保险（约 `1.6 rad/s` / `0.017 rad/tick`），禁止再使用 `max_vel=10`、`max_step_rad=0.20`。默认 `arm: mock` 的流式跟随与笛卡尔同样本拍落地。流式指令（跟随 / 键盘 / 示教）fire-and-forget，网页不得为这些指令等 ACK，也不得用第二层抽样或 `sleep` 再挡住关节快照。使能失败 / `servo_j` 拒绝 / IK 无解必须在遥操作期间仍能出现在网页提示（不得因暂停 HUD 而吞掉）。
- CONTROL MODE：
  - `Position`：`move_j` / `servo_j`。从 Gravity / Impedance 切回时先停补偿环，再 `enable` + 零速度保持当前角（避免跳变，禁止 `move_j(block=False)` 锁存速度）。
  - `Gravity` / `Gra+Fri`：真机独立线程 + `abort_check`；`dyn_ready` 时走 SDK `start_gravity_compensation`，否则走站控 `fafu_dyn.G(q)` + `apply_compensation_torque`（Gra+Fri 带摩擦）。仿真仍是滑条示教（`teleport`），不是力矩环。不得因缺 pinocchio 禁用这三颗按钮。真机零力矩要求 **J1–J6 全部在线**（夹爪可离线）；`G(q)` 仍按 6 轴计算，下发 τ 按在线 `joint_motor_ids` 映射且长度等于 SDK `num_joints`。重力环读角走 `_q_from_sdk`。环异常退出须切回位置保持（禁止 PWM 关闭），100 Hz 写者不得在 MIT 上继续 `servo_j`。
  - `Impedance`：真机用补偿环默认 K/B/I、`hold_on_release=False`（围着进入该模式时的姿态柔顺）。仿真为较慢的位置跟随。无力矩下发接口时才禁用。
- 夹爪：开/关 + J7 角度（`gripper_control`）。键盘 **空格** 按当前开合状态切换（页面仍保留打开/闭合按钮）。真机开合必须带 SDK `effort`（`set_pos_vel_tqe` 固件力矩上限；默认 `robot.cfg` 的 `gripper_max_torque_raw=300` ≈ 1.6 Nm）。禁止 `effort=None`（会走 `set_pos_vel_acc`，顶到电机满电流）。页面「夹持力矩」范围 50–800 raw。不要把遥操作闭合映射成 SDK `grasp()`：`grasp` 会切 `GRASPING`，SERVOING 驻留时非法。接触早停 `grasp(force_threshold)` 只适合独立抓取会话。路点可带每点时长。Scripts 只映射到本站已有 API，**不另起进程抢串口**。
- **连续示教轨迹**（与路点面板分开）：Gravity / Gra+Fri 拖臂与 Position 软件操控共用**同一 100 Hz 只读采样器**，由运动进程写盘，不在浏览器里攒。文件 `%LOCALAPPDATA%\FAFUArmStation\recordings\traj_YYYYMMDD_HHMMSS.jsonl`（`STATION_RECORDINGS_DIR` 可覆盖）。首行头 `{"v":1,"kind":"traj","teach":"drag"|"soft","mode":"...","n":6}`，后续帧与 SDK jsonl 兼容（`t`/`pos`/`vel`/`gpos`/`gvel`，弧度）。夹爪记**指令**角，不用浮空反馈。网页分 **拖臂录制**（默认，`teach=drag`：自动切 Gra+Fri/Gravity 后 100 Hz 记实测角，停录回到 Position）与 **软件录制**（`teach=soft`：Position 跟随/键盘）。`t: rec_delete` 删除 recordings 内选中文件。网页 HUD 显示帧数与时长。回放必须 Position：若仍在浮空则先 `_reenter_position`；先按 `_replay_approach_waypoints` **慢速**走到起点（约 35% UI 速度；只在直线会探桌时抬 J3，**禁止**把 J5/J6 清零当复位；禁止 `move_j(block=False)`），到位并短暂停稳后才启动时间戳，再 100 Hz `servo_j`。走近起点期间不得按轨迹时间轴抢写 `servo_j`（那会高频发抖）。预处理：重采样 10 ms、位置窗 7 滑动平均、速度由平滑位置梯度重算。回放倍率 0.5–1.0。夹爪仅在 `|Δgpos| > ~2°` 时 `gripper_control(effort=…)`，夹爪离线则跳过该指令且不得把「夹爪离线」写成运动故障、不得 `servo_j False` 结束回放会话。禁止 `grasp()`。**禁止**把路点面板稀疏关键帧当成连续轨迹回放；**禁止** MIT 回放。路点面板与 `t: path` / `run_path` 逻辑不变。写者互斥：遥操作 / 复位路点 / 回放 / 重力，同时只一个主人。急停停止录制并**保留**已写文件，回放中止后姿态保持。新的跟随 / WASD / 路点 / 复位 / 切模式中止回放。指令 `t: rec_start`（可带 `teach=drag|soft`）/ `rec_stop` / `rec_delete` / `replay`（等 ACK）；列表 `GET /api/traj`，删除 `DELETE /api/traj?name=`。快照：`recording` / `rec_frames` / `rec_s` / `rec_teach` / `replay_active` / `traj_file`。
- Connection：Connect / Disconnect（`start` / `stop`）。启动真机（`arm=fafu` 且允许运动）在串口连上后自动 `enable`（SDK 构造仍 `auto_enable=False`）。**一颗电机（含夹爪）没起来不得把整臂标成未连接**：调试板串口连上且总线仍有 RX 即为 Link 已连接；缺轴只在电机表标离线，状态栏写「已连接 · n/m 轴在线」。顶栏「已连接」表示臂 USB/仿真植物在线，不是网页 WebSocket。拔线或 `last_rx_age_ms` 超过超时后不得用过期 MotorState 缓存标在线（`online=false`、电机表离线，并释放串口以便再次 Connect）。SDK `_precheck_communication` 在站控里改为警告而非整段失败。`get_joint_values` 缺一轴反馈时保持连接并用已有反馈/上次角度。**交付缺轴（夹爪或任意电机未装/未上电）时，使能与控制只针对在线电机，不得因缺轴拒绝整臂。** 官方 `enable()` / `is_enabled()` / `servo_start` 仍按全部 `cfg.motor_ids` 工作；站控用 Python 配置代理摘掉无应答 ID，使能与 `servo_start` 只切换在线关节（禁止再走 SDK 全员 `enable()` / `motor_reset`）。不得把 SDK `_has_gripper=False` 而 M7 仍留在 `motor_ids` 里（否则 `servo_j` 每拍 hold 离线夹爪）。夹爪离线只在电机表/顶栏提示，不得写成红色 `link_err` 盖住运动错误。未勾选「实时跟随」时拖滑条不下发，须点「发送位置」或勾选跟随。**臂源**：`t: arm_src` `mode=sim|fafu` 热切换适配器（先 hold 再关旧臂）。仿真不开 USB；切回真机不自动 `enable`。无 `STATION_ALLOW_LIVE_ARM=1` 时拒绝切真机。`hold()` 冻结当前角，不得把位置保持误做成 `MODE_STOP` 下坠。**急停保持当前姿态**（位置环 `hold` / `MODE_POSITION`），禁止调用 SDK `emergency_stop()`（那是 PWM 关闭、关节自由落体）。急停锁存后其它运动拒绝；**「复位」例外**：可从保持姿态受控回零。`DEAD` 需 `recover` 再 `resume`。
- **复位**：禁止 `go_home(block=False)` 一拍把六轴打向全零。真机用 FK 看「当前角直线插补到零」会不会把末端压到桌面以下（约 0.14 m）。已经安全则沿该直线回零（各姿态轨迹不同）；否则**只把 J3 抬到刚够**，再沿直线回零。100 Hz 写者按关节空间 inf-norm **同步插补**整条折线（快轴不等慢轴），中间拐点飞过、不停顿；禁止把路点密化成 15° 停靠点再对各轴独立限速（那会停-走抽搐）。禁止固定折肘到 95° 再把 J2 降到 0（上臂竖直扫顶，tool z 可到 0.6 m+）。峰值高度大约不超过 0.40 m；已经更高的姿态只下降、不再抬。速度上限约 55°/s，走已限速的 `servo_j`。仿真仍可 `go_home` 插补。到位后 `_parked`，继续同姿态 `servo_j` hold；后续键盘应清掉残留 `_home_path` 并恢复遥操作。
- 相机：启动时自动识别本机 USB 作业相机（优先 Intel RealSense D405），跳过笔记本内置摄像头。未识别则未连接提示 + 假彩条。**运行中插入作业相机 USB 自动开流进作业画面，不必重启进程**；拔线回到假彩条并写明原因。采集掉线则按同一序列号重开。画面走 `/ws/video`。`camera: mock` 不热插拔。
- 无真臂只读探测：`python -m robot_station.adapters.fafu_arm --sim`。不要在其它程序占用同一 USB 时再开真实串口。

未做：H.264、网页直接跑 SDK 子进程。

## 进程

本机两进程：运动进程听 `127.0.0.1:9470`，网页默认听 `127.0.0.1:9400`。需要局域网访问时把 `listen_host` 改成 `0.0.0.0`。

启动纪律：

- 9400 与 9470 都已在听：视为站控已运行，**复用**，不杀进程。桌面启动器只打开本机窗口。9470 还在时，即使 `/api/info` 看起来像 mock / 相机不对，也**不得** `--replace` 整站。
- **例外（代码换代）**：`/api/info` 的 `code_rev` 与磁盘控制路径 hash 不同（含旧进程没有该字段）时，启动真机必须 `--replace` 9470。关窗口不结束运动；只开窗口而源码已改时，不复用旧进程，否则使能/伺服修改永远进不了真机。
- 只有 9470 在听、9400 掉了：只补 `--web-only`（可加 `--replace` **只结束 9400**）。桌面窗口在前台时监视 9400，掉了自动补网页。
- 只有明确整站 `--replace` 且 9470 已不在时，才结束运动进程。
- 运动进程先占锁、再绑 9470。锁文件在 Windows 用 `%TEMP%`，在 POSIX 用 `/tmp`。
- `--service` 表示无界面：不弹浏览器；若站控已在别的进程里跑则失败退出（不要 attach）。
- 网页 HTTP 循环若异常退出，同一进程重绑 9400，不动 9470。

## 100 Hz 口径

仿真与真机走**同一条闭环**：浏览器 100 Hz 下发 → 运动 tick 100 Hz `servo_j` → 关节快照按 tick 立刻回浏览器。本机闭环延迟应在一两拍以内（约 10–20 ms 量级）。仿真 Position 遥操作本拍落到指令角，不要再叠 40°/s 软件爬行。**真机相反**：流式目标可以立刻更新，但送入电机的角必须按界面 Speed 限幅；否则滑条一点或 WASD 一拍就会以数百度每秒甩向目标。`move_j` / 发送位置仍按界面速度走完规划时间。

| 环节 | 口径 |
|---|---|
| 臂串口环 / 仿真插补 | 100 Hz tick。流式 `servo_j`：仿真 Position 本拍落到指令角；真机按界面 Speed 限幅，`ServoOpts` 硬件保险约 `max_vel=1.6`、`max_step_rad=0.017`（对应 ≤80°/s），看门狗约 150 ms，`lookahead_time=0`。`move_j` 仍用界面速度。重力环 200 Hz（pinocchio 走 SDK，否则站控 G(q)+MIT）。连续示教只读采样同 100 Hz tick（不进 200 Hz 力矩环）；回放按时间戳 `servo_j`，不走路点 inf-norm 插补 |
| 浏览器指令 | 实时跟随与笛卡尔 **100 Hz** 最新值，走独立 `/ws/cmd`（只含指令和 ACK，不带快照）。网页进程→运动进程的流式指令 latest-wins，不得在 TCP 里积压旧目标 |
| 关节遥测 | 运动 TCP 与浏览器 `/ws` 都是 **100 Hz 二进制** 88 字节姿态帧；JSON HUD **10 Hz**。3D 只吃二进制帧（rAF latest-wins）；HUD 不得用 10 Hz JSON 改关节。遥操作时不刷 HUD/滑条。`/ws/rtt` 在 Worker 里测链路。闭环 `echo_t0` 只认新值。本机应经常 < 20 ms |
| 视频 | ~30 Hz，独立通道。跟随 / 笛卡尔 / 示教期间 **停编码、停推送**，避免 PNG 堵住 asyncio 和主线程 |
| 时钟 | Windows 运动进程把多媒体定时器收到 1 ms，避免 `sleep` 卡在 15.6 ms |

## 页面

顶栏：机械臂、连接、指令 Hz、RTT、急停。

机械臂页：左 Joints（空间切换：关节 / 笛卡尔；含 J7）+ 路点 + 示教轨迹（连续录制/回放，不并进路点按钮）；中 3D 参考臂 + 作业相机；右 Connection（Connect/Disconnect）/ 模式 / End Effector（位姿+关节力矩）/ 脚本映射。命令按钮按下有按压高亮；Connect/Disconnect、使能/去使能、夹爪开合按快照显示当前态，等待 ACK 时该按钮为「正在…」。

`/lab`：本机状态页（本站 URL、运动进程是否在听）。

## 本机控制

USB 直连本机，无登录密码，无抢控制权。打开窗口即可动臂。网页每 100 ms `touch` 喂伺服看门狗。最后一个网页客户端断开时 `hold()`。伺服流仍用 0.2 s 指令看门狗。

## 安全

软件急停：本站锁存，**电机保持当前姿态**（位置环，零速度帧），不是 SDK `emergency_stop()` 卸力。禁止用 `move_j(..., block=False)` 做保持（会锁存非零速度，电机空转）。解除急停：若 SDK 状态为 `ESTOP` 才 `resume()`；若为 `DEAD` 则先 `recover(confirm=True)`。「复位」在急停锁存时仍可把臂受控送回零位。到位后先以同一目标再发一帧 `servo_j`（前馈速度≈0）再 `servo_end("hold")`，并取消网页「实时跟随」。

看门狗触发时：`hold()` 结束 servo / 重力环并保持当前角。禁止用 `close_connection(joint_release="stop")` 或 `MODE_STOP` 作为常规 hold / 急停。
