# 单文件 exe 验收清单

客户交付物是 **`dist\FAFUArmStation.exe`**（一个文件）。本清单用于开发机打包后、以及一台**未安装 Python** 的 Windows 10/11 上手测。

自动化测试只覆盖页脚格式、C# 能编译、监视接线；**本文件记录端到端手测**。

## A. 开发机构建（必须先完成）

环境：能找到官方 `fafu_arm_sdk`（仓库旁 `..\fafu_arm_sdk` 或 `FAFU_ARM_SDK`），可访问互联网。

```bat
cd robot_station
powershell -NoProfile -ExecutionPolicy Bypass -File packaging\windows\build_portable.ps1
```

通过标准：

- [ ] 生成 `dist\FAFUArmStation\`（含 `runtime\python310\python.exe`、`app\vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py`）
- [ ] 生成 **`dist\FAFUArmStation.exe`**（不是目录里那个薄启动器）
- [ ] 目录内无 `PUT_SDK_HERE.txt`
- [ ] `app\VERSION` 有版本号
- [ ] 把 exe 大小、SHA-256 记在下方「最近一次构建」

记录 SHA-256：

```bat
certutil -hashfile dist\FAFUArmStation.exe SHA256
```

## B. 干净 Windows 上安装与窗口（不要求真机）

把 **A 产出的那一个 exe** 拷到目标机（不要拷整个文件夹）。

- [ ] 双击 exe。若 SmartScreen 提示「Windows 已保护你的电脑」：更多信息 → **仍要运行**
- [ ] 立刻出现启动窗（第一次可能显示「正在安装…」）
- [ ] 控制页打开（独立窗口，不是系统浏览器）
- [ ] 桌面出现快捷方式 **FAFUArmStation**，目标为 `%LOCALAPPDATA%\FAFUArmStation\FAFUArmStation.exe`
- [ ] 本机浏览器可打开 `http://127.0.0.1:9400/`
- [ ] 右侧可切到 **仿真臂**，拖滑条 +「发送位置」3D 会动
- [ ] 再双击同一个下载的 exe：不再长时间安装，直接进控制页
- [ ] 点桌面快捷方式：同样进控制页
- [ ] 关窗口后真机未连时站控可退出；日志 `%TEMP%\fafu-station-setup.log`、`fafu-station-app.log`

缺 WebView2：应弹出确认框并给出安装链接。无串口：提示安装 CH340 / CP210x / FTDI，仍可进仿真。

## C. 真机 USB（调试板已装驱动、设备管理器有 COM）

启动**前**插上机械臂 USB。

- [ ] 顶栏显示已连接（不是只开着窗口）
- [ ] 自动 Connect 并使能；电机保持当前姿态、不自行甩臂
- [ ] Speed 20–30 deg/s，点「发送位置」小动一轴，方向正确
- [ ] 急停刹车；解除后可继续
- [ ] 启动后再插 USB：臂源切到真机后需再点「使能」（不自动使能）

## D. 作业相机 D405

- [ ] 启动前已插 D405：画面不是假彩条，顶栏可为「真相机」
- [ ] Windows「设置 → 隐私 → 相机」允许桌面应用
- [ ] 启动后才插 D405：数秒内应变真画面（网页进程 USB 监视）

## E. 更新

- [ ] 在 `%LOCALAPPDATA%\FAFUArmStation\recordings\` 放一个假 `traj_keep.jsonl`
- [ ] 用新版本 exe 覆盖安装
- [ ] `recordings\traj_keep.jsonl` 仍在
- [ ] 旧版已删除的 `app\` 文件不应残留；`app\VERSION` 为新版本
- [ ] 控制路径变了会换 9470（`boot_rev`）

## F. 卸载

- [ ] 运行 `%LOCALAPPDATA%\FAFUArmStation\Uninstall.bat`，或删除该目录 + 桌面快捷方式
- [ ] 不改系统 PATH、不卸载系统 Python

## 最近一次构建

| 项 | 值 |
|----|----|
| 日期 | 2026-09-19 18:44:36 |
| 版本 | 1.6.1 |
| SHA-256 | `90E4575E03DF1C43CC771A742EA16AEEA750B8CB0EC45DD50102D0FF0337E17D` |
| 大小 | 54.4 MB（57080757 字节） |
| payload SHA-256 | `9B4E9E86D049A0E663F14B77B263189E896E2E5404448E5000AF1A64AC114F15`（与 exe 哈希不同） |
| SDK | 已打进 `app\vendor\fafu_arm_sdk`（含 `fafu_motor.cp310-win_amd64.pyd`） |
| 页脚 | `FAFUEXE1` 校验通过（zip 54.3 MB，stub 95.5 KB） |
| 本机手测 A | 通过：`build_portable.ps1` 成功；私有 Python 可 `import websockets, yaml, numpy, webview, pyrealsense2`；`import_fafu_sdk` 载入 cp310 `fafu_motor`；无 `PUT_SDK_HERE.txt`；`vcruntime140.dll` 在 `runtime\python310\` |
| 干净机手测 B–F | 见下方「2026-09-19 本轮验证」。 |

未签名的 exe 在客户机上出现 SmartScreen 是预期行为，不是打包失败。

## 2026-09-19 本轮验证

自动化：`python -m unittest tests.test_single_exe tests.test_portable tests.test_usb_watch tests.test_station_launch -v` → **82 tests OK**。

### B. 本机双击外层 exe（开发机，非干净 VM）

通过（进程与运行时均来自 `%LOCALAPPDATA%\FAFUArmStation`，不依赖系统 Python）：

- [x] 只启动 `dist\FAFUArmStation.exe`；解压到 `%LOCALAPPDATA%\FAFUArmStation`
- [x] 启动窗 + 控制页：`fafu-station-app.log` 写 `control page loaded`；`GET http://127.0.0.1:9400/` 200，页面含 `#app`
- [x] 桌面快捷方式 `FAFUArmStation.lnk` 目标为 `%LOCALAPPDATA%\FAFUArmStation\FAFUArmStation.exe`
- [x] `/api/info`：`arm=sim`、`live_serial_allowed=true`、`dyn_ready=true`、`boot_rev=378eb40b1d62`
- [x] 二次启动：`bundle stamp matches, skip unpack`（约 0 秒，不再解压）
- [x] 预检：`preflight webview2=True vcruntime=True com=COM3,COM4`
- [x] PATH 剥离后，安装目录内 `python.exe` 仍可 `import websockets, yaml, numpy, webview, pyrealsense2`

未做：独立「未装 Python 的 Windows VM」、客户机 SmartScreen 点击路径。本机已装 Python / WebView2，不能替代干净机。

### C. 真机 USB

未通过（本机没有机械臂调试板）：

- 设备管理器 COM3/COM4 是 **蓝牙串口**（BTHENUM），不是 CH340/CP210x/FTDI
- 启动以 `--arm fafu --allow-motion`，host 日志：`no USB debug board detected; check the cable and try again`，随后切到 `sim://fake`（`enabled=False`）
- motion 进程带 `--arm-watch --camera-watch`；日志 `USB 监视 … 臂=True 相机=True`，`tick 100 Hz 臂=sim`

有调试板时再勾选 C 节使能/急停项。

### D. 作业相机 D405

未通过（设备在 PnP 中，SDK 打不开）：

- PnP 有 `Intel(R) RealSense(TM) Depth Camera 405`，状态 `Unknown`
- 捆绑 `pyrealsense2` 枚举 **0** 台设备；HUD `backend=mock`，`reason=未找到 USB 作业相机`（假彩条 fps≈24）
- 相机监视在网页进程轮询（约每 9s 一次 PnP 警告），接线正确，缺可用 D405 驱动/权限

### E. 更新

- [x] 安装前在 `%LOCALAPPDATA%\FAFUArmStation\recordings\traj_keep.jsonl` 写入假轨迹
- [x] 新 exe 解压后该文件仍在
- [x] 故意留下的 `leftover_old.txt`、`app\stale_deleted.py` 已被 `ReplaceInstall` 删掉
- [x] `app\VERSION` 为 1.6.1；`boot_rev` 已换成当前包

### F. 卸载

- [x] 关闭站控进程后删除 `%LOCALAPPDATA%\FAFUArmStation` 与桌面 `FAFUArmStation.lnk`（与 `Uninstall.bat` 相同；bat 含 `pause`，本轮非交互执行同等命令）
- [x] 目录与快捷方式均已不存在；9400/9470 不再监听
- [x] 用户/系统 PATH 未写入 `FAFUArmStation`；系统 Python 安装仍在

`Uninstall.bat` 仍在便携树 `dist\FAFUArmStation\Uninstall.bat` 中，随下次双击 exe 再装回。

## 对「还差三件事」的处理

1. **干净机**：意见成立，本环境做不到。没有 Hyper-V / Windows Sandbox。`verify_portable.ps1` 只证明捆绑运行时自洽，**不能**代替未装 Python 的 Win10/11 手测。预检仍只在 `PortableLauncher.cs`，不另写一份 Python `preflight.py`（两套逻辑会漂）。
2. **三个空文件**：已删 `robot_station/preflight.py`、`tests/test_preflight.py`；`verify_portable.ps1` 改为真实的打包后校验。
3. **`dist/BUILD_MANIFEST.json`**：`dist/` 本身不入库。`pack_single_exe.ps1` 现在每次写出清单，且 `exe_sha256` 与 `payload_sha256` 分开；`verify_portable.ps1` 会核对清单与磁盘上的 exe 一致。
