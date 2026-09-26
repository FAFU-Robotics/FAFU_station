# Windows 私有运行时打包

给客户**一个 exe**：双击后解到本用户目录并打开控制窗口。运行时是私有 Python 3.10，只给本软件用。

**不会：** 安装系统 Python、写 PATH、`setx`、改注册表、往客户的 pip 用户目录装包。  
**客户不必：** 解压 zip、打开文件夹、寻找 `.bat`。

## 在开发机上打包

1. 建议本机已能访问互联网（下载 embeddable Python 与 get-pip）。
2. 将官方 `fafu_arm_sdk` 放到仓库旁、`vendor\fafu_arm_sdk`，或设置 `FAFU_ARM_SDK`。没有 SDK 时**不会**生成发给客户的单个 exe。
3. 默认**不**打 pinocchio。重力走站控 `G(q)` + SDK `apply_compensation_torque`。若打包机有 conda-forge pinocchio，可加 `-WithPinocchio`。
4. 双击 `build_portable.bat`，或：

```bat
powershell -ExecutionPolicy Bypass -File packaging\windows\build_portable.ps1
```

只打便携目录、不打单个 exe：加 `-SkipSingleExe`。已有目录时只打单个 exe：

```bat
powershell -ExecutionPolicy Bypass -File packaging\windows\pack_single_exe.ps1
```

| 路径 | 作用 |
|---|---|
| **`dist\FAFUArmStation.exe`** | **发给客户的唯一文件** |
| `dist\BUILD_MANIFEST.json` | 打包脚本写出的版本/SHA（`exe_sha256` 与 zip 的 `payload_sha256` 必须不同） |
| `dist\FAFUArmStation\` | 构建中间物（U 盘/调试可就地运行） |
| 目录内薄 `FAFUArmStation.exe` | 就地启动器；也是解压后用户目录里的日常启动器 |
| `runtime\python310\` | 私有 CPython 3.10 + 本软件依赖 |
| `app\` | 站控源码（更新时覆盖用户目录里的这一份） |
| `app\README.md` | 客户使用说明（源文件 `docs/客户使用说明.md`） |
| `app\urdf\` | 重力补偿 URDF |
| `app\vendor\fafu_arm_sdk\` | 私有 SDK |
| `Install.bat` / `Uninstall.bat` | 从已解压目录拷到用户目录 / 卸载 |

单个 exe 的格式：`SingleFileLauncher` 外壳 + zip 载荷 + 48 字节页脚（SHA-256、长度、魔数 `FAFUEXE1`）。第一次双击解压到 `%LOCALAPPDATA%\FAFUArmStation`，以后校验戳相同则直接启动。新版本会**替换**该目录（保留 `recordings\`）。

薄启动器启动前检测 WebView2、VC++ 运行库、串口；缺项会弹窗说明。

打包后会自动跑 [`verify_portable.ps1`](verify_portable.ps1)：检查页脚、SDK、清单哈希，并用**捆绑** Python 在剥离 PATH 后 import 依赖。这是开发机构建校验，**不能**代替干净 Windows 手测。

手测清单：[`ACCEPTANCE.md`](ACCEPTANCE.md)

## 交给客户

只发送 **`dist\FAFUArmStation.exe`**。客户：

1. 插上机械臂 USB
2. 双击该 exe

第一次会看到「正在安装…」，随后打开控制页，并在桌面建立快捷方式。USB 连上后自动 Connect 并使能。操作说明在安装后的 `app\README.md`。

更新：把新的单个 exe 发给对方，再双击一次即可覆盖。控制路径变了会自动换运动进程。

卸载：运行用户目录里的 `Uninstall.bat`，或删除 `%LOCALAPPDATA%\FAFUArmStation` 和桌面快捷方式。

## 隔离边界

- **关在安装目录里：** Python 3.10、websockets / PyYAML / pywebview / numpy / pyrealsense2、站控代码、SDK
- **无法隔离：** USB 串口驱动、已安装的 Edge/WebView2、（若 `.pyd` 需要且未私有携带）VC++ 运行库

不要把 `runtime\python310` 加到客户的系统 PATH。启动器已经在进程内临时前置该目录。
