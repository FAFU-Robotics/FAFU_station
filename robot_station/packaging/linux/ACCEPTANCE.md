# Linux AppImage 验收清单

客户交付物是 **`dist/FAFUArmStation-x86_64.AppImage`**（一个文件）。本清单用于 Ubuntu 22.04 打包后，以及一台**未安装 Python** 的 Ubuntu 22.04 / 24.04 上手测。

自动化测试只覆盖脚本与源码断言；**本文件记录端到端手测**。Windows 开发机不能代替干净 Ubuntu。**GHA 绿 ≠ 客户验收过。**

## 谁测什么

| 角色 | 环境 | 允许宣称完成 |
|---|---|---|
| Windows 开发 | 本机 unittest + 可选 Docker | 代码与脚本契约（`tests/test_linux_*.py`） |
| GHA | `linux-appimage`（ubuntu-22.04）打出 AppImage + `BUILD_MANIFEST.json` | 构建产物存在且 `verify_appimage.sh` 退出 0 |
| x86 Ubuntu 22.04 / 24.04 干净机 | 双击 AppImage | 窗口 / 仿真臂 / 24.04 同一份包开窗（本节 B） |
| Jetson 或工控机 | 源码 `启动真机.sh` + USB | 真机 100 Hz（本节 C）。**不要在 Jetson 上打 x86_64 客户包** |

不要在 Windows 或 WSL 上冒充「客户双击验收」（USB 与 WebKit 都不可信）。

## A. 构建（必须先完成）

任选一条路：

- GitHub Actions：`packaging-smoke.yml` → **Run workflow** → 下载 `FAFUArmStation-x86_64-AppImage`
- 已装 Docker 的 Windows：`packaging\linux\build_appimage.bat`（无 Docker 会失败并提示改用 GHA / 实体 Ubuntu）
- 实体 Ubuntu 22.04 x86_64：`bash packaging/linux/build_appimage.sh`

须能找到官方 `fafu_arm_sdk`。无 SDK **不得**当交付成功。

通过标准：

- [ ] 生成 `dist/FAFUArmStation.AppDir/`（含 `runtime/python310/bin/python3`、`app/vendor/fafu_arm_sdk/fafu_robot_python/fafu_robot_controller.py`）
- [ ] 生成 **`dist/FAFUArmStation-x86_64.AppImage`**
- [ ] 生成 **`dist/BUILD_MANIFEST.json`**（`magic=FAFUAPP1`，`image_sha256` 与文件一致，`sdk_present=true`）
- [ ] 目录内无 `PUT_SDK_HERE.txt`
- [ ] 存在 `fafu_motor.cpython-310-*-linux-gnu.so`
- [ ] `app/VERSION` 有版本号
- [ ] `verify_appimage.sh` 退出码 0（含 `import fafu_motor`）

A 通过只说明包能打出来。还不能对客户说「验收过」。

## B. 干净 Ubuntu 上打开窗口（不要求真机）

把 **A 产出的那一个 AppImage** 拷到目标机。

- [ ] `chmod +x FAFUArmStation-x86_64.AppImage` 后双击
- [ ] 若缺 FUSE：安装 `libfuse2`，或 `--appimage-extract-and-run`
- [ ] 立刻出现独立控制窗口（GTK，不是系统浏览器）
- [ ] 本机浏览器可打开 `http://127.0.0.1:9400/`
- [ ] 右侧可切到 **仿真臂**，拖滑条 +「发送位置」3D 会动
- [ ] Ubuntu 24.04 用**同一份** 22.04 打的 AppImage 也能开窗

## C. 真机 USB（需要调试板）

优先在 x86 工控机或 **Jetson 源码模式**（`启动真机.sh`）上做，不要把 Jetson 当 AppImage 打包机。

- [ ] 插上调试板。若无权限：预检提示安装 udev（pkexec）后 `/dev/fafu_debug_board` 出现
- [ ] Connect 并使能；`move_j` 可动
- [ ] 无串口时仍可进仿真
- [ ] 若 ModemManager 抢口：按提示 stop 后能连
- [ ] 关窗口不结束已连接真机 9470；再开窗口复用
- [ ] 100 Hz 保持（无 `SCHED_FIFO` 权限时仍须可运行，只是普通调度）

## D. 轨迹与更新

- [ ] 录制后文件在 `~/.local/share/FAFUArmStation/recordings/`
- [ ] 换新 AppImage 后该目录仍在

未做：在 Windows 开发机上冒充 Linux 真机验收。
