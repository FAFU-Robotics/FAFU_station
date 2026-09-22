<!-- station-customer-readme-linux -->
# FAFU 机械臂站控 · 客户使用说明（Ubuntu）

本文面向**现场操作员**。打开软件目录里的 `README.md` 就是这份说明。

本软件在 **Ubuntu 22.04 / 24.04（64 位）** 上运行，用 USB 直连机械臂调试板。打开后就是机械臂控制页：3D 参考臂、关节/笛卡尔、作业相机、急停、示教与回放。

Windows 客户请看随 Windows 安装包提供的说明。

---

## 1. 您收到的是什么

您收到的是**一个文件**：`FAFUArmStation-x86_64.AppImage`。不必解压压缩包，也不必打开文件夹去找脚本。

```bash
chmod +x FAFUArmStation-x86_64.AppImage
```

插上机械臂 USB 后，双击该文件即可打开控制窗口。

**不必**在客户电脑上安装 Python。

| 位置 | 作用 |
|------|------|
| 您下载的 AppImage | 发给您的启动文件 |
| `~/.local/share/FAFUArmStation/recordings/` | 示教轨迹 |
| `~/.cache/fafu-station-app` | 窗口缓存 |

更新：用新的 AppImage 覆盖旧文件即可（示教轨迹会保留）。

若双击没反应：安装 `sudo apt install libfuse2`，或在终端执行：

```bash
./FAFUArmStation-x86_64.AppImage --appimage-extract-and-run
```

卸载：删除 AppImage。可选删除上面的 recordings 与 cache 目录。

---

## 2. 电脑与接线

| 项目 | 要求 |
|------|------|
| 系统 | Ubuntu 22.04 或 24.04，64 位 |
| 机械臂 | USB 线插到**运行本软件的这台电脑** |
| 作业相机 | USB 插到同一台电脑。交付相机是 Intel RealSense D405。软件不会用笔记本盖上的内置摄像头 |
| 窗口 | 独立应用窗口（不是浏览器） |

建议：USB-C 笔记本用可靠转接；启动前先插好机械臂 USB。

第一次若提示无法访问串口：按窗口说明输入管理员密码安装 udev 规则。装完后调试板会出现 `/dev/fafu_debug_board`。

若一直连不上 USB：可能是 ModemManager 占用串口。在终端执行 `sudo systemctl stop ModemManager` 后再试。

---

## 3. 第一次开机（建议按这个顺序）

1. **清场**：工作空间内无障碍、周围无人。手臂下方不要伸手去托。
2. **接线**：机械臂 USB（以及作业相机，如有）插到本机。
3. 双击 AppImage，等待独立窗口打开。
4. 顶栏应变为已连接。USB 连上后会自动 Connect 并使能（电机保持当前姿态，不会自己甩臂）。
5. 无真机时，右侧 **臂源** 点「仿真臂」即可熟悉界面。

急停：页面按钮或 `Esc`。急停后电机刹车，抬着的臂可能下沉。解除急停后再运动；急停中仍可点「复位」受控回零。

更细的操作（关节/笛卡尔、示教、夹爪）与 Windows 版相同：见开发交付的完整客户说明，或窗口内的提示。

---

## 4. 常见问题

- **窗口打不开**：确认是官方 AppImage；缺 FUSE 时用上面的 `libfuse2` / `--appimage-extract-and-run`。不必安装 Python。若官方 AppImage 仍提示缺少 WebKitGTK，这是包装问题，请向发件方更换新包，不要自行 `apt install python3`。
- **没有串口 / 无法 Connect**：安装 udev、确认 USB、排除 ModemManager。
- **作业相机假画面**：确认 D405 已插上，不要用笔记本内置摄像头。
- 日志：`~/.cache/fafu-station-app.log` 与 `/tmp` 下的 `fafu-station-host.log`（若有）。
