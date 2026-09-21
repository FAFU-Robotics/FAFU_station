# Ubuntu AppImage 打包

给客户**一个 AppImage**：双击后打开独立控制窗口。运行时是私有 Python 3.10，只给本软件用。

**不会：** 安装系统 Python、写 PATH、改客户的 pip。  
**客户不必：** 解压文件夹、寻找 `.sh`。  
**必须在 Ubuntu 22.04 x86_64 上构建**，以便 22.04 / 24.04 都能跑。不要在 Windows Python 上打 AppImage，也不要用 WSL 冒充客户双击验收。

禁止用 PyInstaller onefile 当运行时。默认不打 pinocchio。无 SDK 的 Linux `.so` 时**不得**打出发给客户的 AppImage。

## 三条打包路（Windows 开发约束）

| 路径 | 谁用 | 说明 |
|---|---|---|
| **GitHub Actions `linux-appimage`** | Windows 团队**主路径** | 仓库根 `.github/workflows/packaging-smoke.yml`，`ubuntu-22.04` **真构建**。`workflow_dispatch` 点一下出 artifact。无 `fafu_arm_sdk` 则 job **失败**。 |
| **本机 Docker** | 已装 Docker Desktop 时的可选入口 | 双击 [`build_appimage.bat`](build_appimage.bat)。没有 docker 会打印「请用 GitHub Actions 或实体 Ubuntu 22.04」并非 0 退出。不要在 Windows 上直接跑 `.sh`。 |
| **实体 Ubuntu 22.04 x86_64** | 有 Linux 打包机时 | `bash packaging/linux/build_appimage.sh`。不要用 Jetson（aarch64）打客户包。 |

不要把「CI 打出 artifact」写成客户验收已通过。干净机手测见 [`ACCEPTANCE.md`](ACCEPTANCE.md)。

### GitHub Actions

1. 打开仓库 **Actions → packaging-smoke → Run workflow**（`workflow_dispatch`）。
2. 等 `linux-appimage` 结束。
3. 下载 artifact `FAFUArmStation-x86_64-AppImage`（含 `FAFUArmStation-x86_64.AppImage` 与 `BUILD_MANIFEST.json`）。

`linux-smoke` 只跑 unittest，不是客户包。

### 本机 Docker（可选）

仓库根须能看到 `robot_station/` 与 `fafu_arm_sdk/`。

```bat
packaging\linux\build_appimage.bat
```

或：

```bat
docker build -f robot_station/packaging/linux/Dockerfile -t fafu-appimage .
docker run --rm -e APPIMAGE_EXTRACT_AND_RUN=1 -v %CD%:/src fafu-appimage
```

失败日志：`robot_station\dist\appimage-build.log`。

### 实体 Ubuntu 22.04

```bash
cd robot_station
export FAFU_ARM_SDK=/path/to/fafu_arm_sdk
bash packaging/linux/build_appimage.sh
```

| 路径 | 作用 |
|---|---|
| **`dist/FAFUArmStation-x86_64.AppImage`** | **发给客户的唯一文件** |
| `dist/BUILD_MANIFEST.json` | `FAFUAPP1`：版本、SHA-256、是否含 SDK / `fafu_motor`、glibc、`uname -m` |
| `dist/FAFUArmStation.AppDir/` | 构建中间物 |
| `runtime/python310/` | 私有 CPython 3.10 |
| `app/` | 站控源码 |
| `app/vendor/fafu_arm_sdk/` | 含 Linux `fafu_motor.so` |

只生成 AppDir、不打 squashfs：

```bash
bash packaging/linux/build_appimage.sh --skip-image --allow-no-sdk
```

（`--allow-no-sdk` 只用于调试仿真包，不能发给客户。）

打包脚本固定 `APPIMAGE_EXTRACT_AND_RUN=1`，**不依赖宿主机 FUSE**。`linuxdeploy` 失败则整次构建失败，不会再 wrap 一个缺 GTK 的 squashfs。工具 URL 已 pin，不用 `continuous` 标签。

打包后会跑 [`verify_appimage.sh`](verify_appimage.sh)（有 SDK 时必须 `import fafu_motor`，并核对 manifest 哈希）。这是构建校验，**不能**代替干净 Ubuntu 手测。

## 交给客户

只发送 **`FAFUArmStation-x86_64.AppImage`**。客户：

1. `chmod +x FAFUArmStation-x86_64.AppImage`
2. 插上机械臂 USB
3. 双击该文件

若提示缺 FUSE：`sudo apt install libfuse2`，或：

```bash
./FAFUArmStation-x86_64.AppImage --appimage-extract-and-run
```

第一次可能弹出 udev / 串口说明（需要管理员密码安装规则）。轨迹写在 `~/.local/share/FAFUArmStation/recordings/`。

更新：用新的 AppImage 覆盖旧文件。示教轨迹在用户目录，会保留。

卸载：删除 AppImage。可选删除 `~/.local/share/FAFUArmStation` 与 `~/.cache/fafu-station-app`。

## 谁测什么

| 角色 | 环境 | 允许宣称完成 |
|---|---|---|
| Windows 开发 | 本机 unittest + 可选 Docker | 代码与脚本契约 |
| GHA | ubuntu-22.04 打出 AppImage | 构建产物存在且 verify 通过 |
| x86 Ubuntu 22.04/24.04 干净机 | 双击 AppImage | 窗口 / 仿真臂 / 24.04 兼容 |
| Jetson 或工控机 | 源码 `启动真机.sh` + USB | 真机 100 Hz，**不是**客户包 |

**GHA 绿 ≠ 客户验收过。** ACCEPTANCE B/C/D 仍须手测。不要在 Windows / WSL 上冒充客户双击验收。

## 隔离边界

- **关在 AppImage 里：** Python 3.10、站控、SDK `.so`、尽量完整的 GTK3 + WebKitGTK 4.0（从 22.04 收集）
- **无法隔离：** USB 内核驱动、首次 udev 规则（需 pkexec）、缺 FUSE 时的 libfuse2
