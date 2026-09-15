# FAFU 机械臂站控

本仓库是 FAFU 自研机械臂的软件集合，包含站控宿主和官方 SDK：

```
FAFU_station/
├── robot_station/    # PC 本机站控：网页、3D 参考臂、USB 真机
└── fafu_arm_sdk/     # 官方 C++ / Python SDK（含预编译 fafu_motor）
```

站控会自动查找**仓库旁**的 `fafu_arm_sdk`。克隆本仓库后，这两个目录已经是兄弟关系，一般不必再设 `FAFU_ARM_SDK`。

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

预编译 `fafu_motor` 是 **cp310**。Python API、C++ 构建与例程见 [`fafu_arm_sdk/README.md`](fafu_arm_sdk/README.md)。

## 验收

```bash
cd robot_station
python -m unittest discover -s tests -v
```
