# FAFU 机械臂站控

当前版本：**v1.7.0**。安装包与更新说明见 [GitHub Release](https://github.com/FAFU-Robotics/FAFU_station/releases/tag/v1.7.0)。

本仓库是 FAFU 自研机械臂的软件集合，包含站控宿主和官方 SDK：

```
FAFU_station/
├── robot_station/    # PC 本机站控：网页、3D 参考臂、USB 真机
└── fafu_arm_sdk/     # Git 子模块：官方 C++ / Python SDK
```

站控会自动查找同一仓库中的 `fafu_arm_sdk` 子模块，一般不必再设 `FAFU_ARM_SDK`。

首次克隆（同时拉取 SDK）：

```bash
git clone --recurse-submodules https://github.com/FAFU-Robotics/FAFU_station.git
```

已有工作目录初始化 SDK，或切换主仓库提交后同步 SDK：

```bash
git submodule update --init --recursive
```

更新 SDK 到上游 `main`，然后在主仓库记录新的 SDK 提交：

```bash
git submodule update --remote fafu_arm_sdk
git add fafu_arm_sdk
```

SDK 的改动须先在子模块中提交并推送，再提交主仓库中的子模块指针。

各自的独立仓库仍在：

- [FAFU-Robotics/robot_station](https://github.com/FAFU-Robotics/robot_station)
- [FAFU-Robotics/fafu_arm_sdk](https://github.com/FAFU-Robotics/fafu_arm_sdk)

## 启动站控

客户 Windows 推荐用打包好的私有运行时，不必安装 Python。开发机在 `robot_station` 目录操作：

```bat
cd robot_station
启动真机.bat
```

或：

```bash
cd robot_station
python run_station.py
```

开发或未打包时，默认地址是 `http://127.0.0.1:9400/`。详细步骤、真机使能、仿真臂与打包见 [`robot_station/README.md`](robot_station/README.md)。

## SDK

Windows x64 支持 Python **3.10 / 3.12**，分别加载 `cp310` / `cp312` 扩展。
开发机安装依赖并为当前 Python 构建 SDK：

```bat
python -m pip install -r robot_station\requirements.txt -r fafu_arm_sdk\fafu_robot_python\requirements.txt
robot_station\scripts\rebuild_fafu_motor.bat
```

构建需要 VS2022 C++ 工具和 CMake。`STATION_PYTHON` 可指定目标解释器。
Python API、C++ 构建与例程见 [`fafu_arm_sdk/README.md`](fafu_arm_sdk/README.md)。

## 程序工作台

独立的连接、手动控制、路点和示教页面，以及 UR 风格程序树和 FAFU 简化脚本，见 [程序与作业工作台](robot_station/docs/PROGRAMMING.md)。

## 验收

```bash
cd robot_station
python -m unittest discover -s tests -v
```
