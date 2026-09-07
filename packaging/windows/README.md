# Windows 私有运行时打包

给客户一份**自带 Python 3.10** 的目录，启动时只在本进程使用这份解释器。

**不会：** 安装系统 Python、写 PATH、`setx`、改注册表、往客户的 pip 用户目录装包。

## 在开发机上打包

1. 建议本机已能访问互联网（下载 embeddable Python 与 get-pip）。
2. 将官方 `fafu_arm_sdk` 放到仓库旁、`vendor\fafu_arm_sdk`，或设置 `FAFU_ARM_SDK`。
3. 双击 `build_portable.bat`，或：

```bat
powershell -ExecutionPolicy Bypass -File packaging\windows\build_portable.ps1
```

输出目录：`dist\FAFUArmStation\`

| 路径 | 作用 |
|------|------|
| `FAFUArmStation.exe` | 启动器：第一次拷到用户目录并打开页面 |
| `启动真机.bat` / `StartLive.bat` | 与 exe 相同：第一次安装，之后直接启动 |
| `runtime\python310\` | 私有 CPython 3.10 + 本软件依赖 |
| `app\` | 站控源码 |
| `app\vendor\fafu_arm_sdk\` | 私有 SDK（若打包时找得到） |
| `Install.bat` | 可选；与第一次双击 exe 相同（拷贝并启动） |
| `Uninstall.bat` | 删除上述目录和快捷方式 |

## 交给客户

把整个 `dist\FAFUArmStation` 文件夹拷走（可打 zip）。客户：

1. 插上机械臂 USB
2. 双击 `FAFUArmStation.exe`（或 `启动真机.bat`）

第一次会装到 `%LOCALAPPDATA%\FAFUArmStation`（不写 PATH）并打开页面；USB 连上后自动 Connect 并使能。以后点桌面快捷方式即可。

更新：再发一份新 zip，双击其中的 exe，会覆盖用户目录里的副本。

## 隔离边界

- **关在安装目录里：** Python 3.10、websockets / PyYAML / pywebview / numpy / pyrealsense2、站控代码、SDK
- **无法隔离：** USB 串口驱动、已安装的 Edge/WebView2、（若 `.pyd` 需要且未私有携带）VC++ 运行库

不要把 `runtime\python310` 加到客户的系统 PATH。启动器已经在进程内临时前置该目录。
