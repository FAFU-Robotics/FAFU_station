# FAFU 机械臂站控

客户 **PC 本机** 运行的机械臂宿主：打开后直接进 **机械臂** 页（布局参考 [Panthera-HT Host](https://github.com/HighTorque-Robotics/Panthera-HT_Host)，正中是 3D 参考臂，作业相机在下方）。PC 用 USB 直连机械臂。

当前：**默认 yaml 仍是假臂**。未设置 `STATION_ALLOW_LIVE_ARM=1` 时绝不会构造真 `FafuRobotController`。真机运动还要 `arm_allow_motion: true` 或 `--allow-motion`。作业相机仍是未连接格子。

本站只管机械臂和作业相机。

## 启动

**客户（推荐）：** 使用打包好的私有运行时，不必在电脑上安装 Python。

在开发机执行 `packaging\windows\build_portable.bat`，把 **`dist\FAFUArmStation.exe`**（单个文件）交给 Windows 客户。Ubuntu 客户发给 **`dist/FAFUArmStation-x86_64.AppImage`**（须在 Ubuntu 22.04 / Docker 打包，见 `packaging/linux/README.md`）。客户插上 USB，双击该文件打开控制页。该运行时只给本软件用，**不会**写入系统 PATH。

开发或未打包时，默认地址仍是 `http://127.0.0.1:9400/`。

| 谁 | 怎么开 | 文件 |
|---|---|---|
| 客户 Windows（推荐） | 插 USB 后双击收到的 `FAFUArmStation.exe` | 解到用户目录并打开控制页；后台建桌面快捷方式 |
| 客户 Ubuntu（推荐） | 插 USB 后双击 `FAFUArmStation-x86_64.AppImage` | 独立 GTK 窗口；`chmod +x`；缺 FUSE 时 `--appimage-extract-and-run` |
| 开发机 Windows | 双击 `启动真机.bat` | 优先目录内私有 `runtime\python310`；否则本机 Python 3.10 |
| 开发机 Ubuntu | `bash 启动真机.sh` | 优先私有 `runtime/python310/bin/python3`；否则本机 Python 3.10 |
| 任意平台命令行 | `python run_station.py` | 运动进程 `:9470` + 网页 `:9400` |
| 只看总览、不控臂 | `python tools/status_panel.py` | 需站控已在本机运行 |

```bat
启动真机.bat
```

默认拉起真机 USB（`STATION_ALLOW_LIVE_ARM=1`、`arm=fafu`、允许运动）。USB 连上后自动 Connect 并使能。无臂或不想动真机时，在页面右侧 **臂源** 点「仿真臂」即可，不必另开启动脚本。切回「真机 USB」不会自动使能。

或：

```bash
python run_station.py
```

手工已经在跑时再执行一次 `run_station.py`：只打印地址，不会再开一份。真要重启才加 `--replace`。

无真机先熟悉界面：在 `启动真机.bat` 打开的窗口里切到「仿真臂」，或：

```bash
python run_station.py --arm sim
```

官方 [fafu_arm_sdk](https://github.com/FAFU-Robotics/fafu_arm_sdk) 预编译 `fafu_motor` 是 **cp310**。本机若是 Python 3.11，不要用系统 Python 开真串口。请用：

```bat
启动真机.bat
```

或：

```bat
set STATION_ALLOW_LIVE_ARM=1
set FAFU_ARM_SDK=D:\fafu_arm_sdk
py -3.10 run_station.py --replace --arm fafu --allow-motion
```

只读探测（开串口、读关节、**不使能、不动臂**）：

```bat
set STATION_ALLOW_LIVE_ARM=1
set FAFU_ARM_SDK=D:\fafu_arm_sdk
py -3.10 -m robot_station.adapters.fafu_arm
```

启动真机成功连上 USB 后会 `enable()`；之后页面即可下发 `move_j` / `servo_j`。急停或拔 USB 可立刻停。SDK 目录也会自动查找仓库旁的 `fafu_arm_sdk`。

## 用法

1. 打开后直接在机械臂页。
2. 本机即可动臂。急停切入刹车（电机不停电、无位置保持力矩，抬着可能下沉），不是 PWM 卸力；急停后仍可点「复位」受控回零。其它运动需先解除急停。
3. 关节：左侧可切 **关节空间** / **笛卡尔**。关节空间拖滑条后点「发送位置」，或勾选「实时跟随」（100 Hz servo）。未勾选时拖滑条只改目标，**不会**自动勾上实时跟随。勾选后的跟随速度受页面 Speed（deg/s）限制，不会一拍甩到目标。笛卡尔是末端 X/Y/Z 与姿态滑条：拖动只改目标，松手或「发送位置」后 IK + `move_j`，按 Speed 走到该位姿。键盘 WASD 仍是增量遥操作（无 pinocchio 时走站控 URDF IK；pytracik 不进入 100 Hz 写者）。复位、夹爪角度。`arm: mock` 与 `arm: sim` 的流式控制都是本拍落地；「发送位置」与笛卡尔滑条仍按界面速度。真臂且 `arm_allow_motion=false` 时命令会被拒绝。
4. Gravity / Gra+Fri：仿真拖滑条示教；真机与 SDK `test_fafu_motion_interactive` 的 `[t]` 示教相同（纯 `G(q)` 力矩前馈，Gra+Fri 另加摩擦；有 pinocchio 走 `gravity_compensation_step`，否则站控 `G(q)` + MIT）。Position 位置控制不变。连续示教默认 **拖臂录制**（开始后切入 Gravity，纯 `G(q)`），也可切 **软件录制**；回放先慢速走到起点再按时间戳 `servo_j`。可选中文件删除。路点面板仍是稀疏关键帧，由后端插补，可设每点时长。脚本只映射到本站 API，不另起进程抢串口。
5. 臂页正中是 3D 参考臂（100 Hz 关节；外观为 SDK `fafu_baseV1` 的 SolidWorks STL，自动框选进视口）。作业相机一路：启动时自动识别本机 USB（优先 RealSense D405，跳过笔记本内置摄像头）；没插作业相机时是未连接 + 假画面。可用「3D」「相机」开关。
6. 本机状态页：`http://127.0.0.1:9400/lab`。

规格：[`docs/SPEC.md`](docs/SPEC.md) · 客户说明：[`docs/客户使用说明.md`](docs/客户使用说明.md)（Windows 便携包 `app/README.md`）· [`docs/客户使用说明-linux.md`](docs/客户使用说明-linux.md)（AppImage `app/README.md`） · 交付/开发说明：[`docs/使用说明.md`](docs/使用说明.md) · 打包：[`packaging/windows/README.md`](packaging/windows/README.md) · [`packaging/linux/README.md`](packaging/linux/README.md)

## 验收

```bash
python -m unittest discover -s tests -v
```
