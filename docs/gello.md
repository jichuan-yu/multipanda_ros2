# GELLO 遥操作使用指南

## 1. 概述

GELLO 遥操作是一种基于舵机臂的双臂遥操作方案，用于在 MuJoCo 仿真中控制 Franka 机械臂。本方案支持两种控制模式：基础关节阻抗控制和完整安全控制（带碰撞检测）。

> **注意**：本指南基于实际测试验证，不依赖 `gello-teleop` 包，使用自定义遥操作脚本。

## 2. 系统要求

- Ubuntu 22.04 + ROS2 Humble
- 已配置好 `my_task_description` 及 `dual_arm_reactive_control` 等仿真包
- Python 3.10+
- 物理 GELLO 舵机臂（通过 USB 连接）

## 3. 安装步骤

### 3.1 安装 gello_software（核心依赖）

```bash
# 创建安装目录
mkdir -p ~/libraries/gello
cd ~/libraries/gello

# 克隆 gello_software 仓库
git clone https://github.com/wuphilipp/gello_software.git
cd gello_software

# 初始化并更新子模块（包含 Dynamixel SDK）
git submodule init && git submodule update

# 安装 gello_software（开发模式）
pip install -e .

# 安装 Dynamixel SDK（使用仓库中的子模块版本）
pip install -e third_party/DynamixelSDK/python
```

### 3.2 确认脚本位置

脚本已包含在 `multipanda_ros2` 仓库中：

```bash
# 脚本路径
~/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts/gello_franka_ros2.py
```

### 3.3 关于 gello-teleop（可选）

> **说明**：官方文档推荐安装 `gello-teleop` 包，但经过测试，本项目的自定义脚本**不依赖**该包。如果安装失败，可以**安全跳过**此步骤，脚本仍能正常运行。

如需尝试安装（可选）：

```bash
# 可选：安装 gello-teleop（如果需要正运动学功能）
pip install git+https://github.com/RLinf/gello-teleop.git
```

## 4. 硬件设置

### 4.1 连接 GELLO 舵机臂

将 GELLO 设备通过 USB 连接到电脑。

### 4.2 查看 USB 设备号

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

通常 GELLO 设备会被识别为 `/dev/ttyUSB0`。

### 4.3 设置权限

```bash
sudo chmod 666 /dev/ttyUSB0
# 或永久添加到 dialout 组（推荐）
sudo usermod -aG dialout $USER
newgrp dialout
```

## 5. 运行指南

### 5.1 方式一：基础模式（关节阻抗控制）

适合快速测试 GELLO 功能：

**终端 1：启动仿真**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 launch franka_bringup dual_franka_sim.launch.py
```

**终端 2：启动关节阻抗控制器**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 run controller_manager spawner dual_joint_impedance_controller
```

**终端 3：启动 GELLO 遥操作**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
cd ~/dual_panda_ws/src/multipanda_ros2/gello_teleop
python3 scripts/gello_franka_ros2.py
```

### 5.2 方式二：完整模式（带安全控制）

适合完整实验，包含碰撞检测和相机：

**终端 1：启动完整仿真**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true
```

**终端 2：Web 视频服务器**
```bash
source /opt/ros/humble/setup.bash
ros2 run web_video_server web_video_server
# 访问 http://localhost:8080 查看相机
```

**终端 3：碰撞体可视化**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 run dual_arm_reactive_control collision_env_visualizer_node --ros-args -p base_frame:=world
```

**终端 4：双臂安全控制器**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node
```

**终端 5：启动 GELLO 遥操作**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
cd ~/dual_panda_ws/src/multipanda_ros2/gello_teleop
python3 scripts/gello_franka_ros2.py
```

## 6. 控制模式配置

编辑 `scripts/gello_franka_ros2.py` 中的 `CONTROL_MODE` 变量：

| 模式 | 值 | 说明 |
|------|-----|------|
| `LEFT_ARM`  | 1 | 只控制左臂，右臂保持默认位置 |
| `RIGHT_ARM` | 2 | 只控制右臂，左臂保持默认位置 |
| `BOTH_ARMS` | 3 | 双臂镜像控制（默认） |

## 7. 标定说明

### 7.1 标定原理

GELLO 舵机臂的关节角度需要与 Franka 机械臂对齐。标定偏移已预先计算并集成到脚本中：

```python
# 固定标定偏移（π/2 倍数）
joint_offsets = (
    1 * np.pi,        # 关节1
    1 * np.pi,        # 关节2
    2 * np.pi,        # 关节3
    1 * np.pi,        # 关节4
    1 * np.pi,        # 关节5
    1 * np.pi,        # 关节6
    1.25 * np.pi,     # 关节7
)
```

### 7.2 重新标定（如需）

运行官方标定脚本：

```bash
cd ~/libraries/gello/gello_software/ros2/src/franka_gello_state_publisher/scripts/
python3 get_offsets.py --port /dev/ttyUSB0
```

## 8. 话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `/dualarm_teleop_cmd` | `std_msgs/Float64MultiArray` | 遥操作命令输出（14 个关节角度） |
| `/dual_joint_impedance/joints_desired` | `std_msgs/Float64MultiArray` | 基础模式关节目标 |

## 9. 文件结构

```
~/dual_panda_ws/src/multipanda_ros2/
├── gello_teleop/
│   └── scripts/
│       └── gello_franka_ros2.py    # 主遥操作脚本
├── teleop/                         # 其他遥操作方式
│   ├── key_teleop_joint.py
│   ├── spacemouse_teleop_cartesian.py
│   └── ...
└── docs/
    ├── gello.md                    # 本说明文档
    ├── RUN_SIM.md
    └── teleop.md
```

## 10. 故障排查

### 10.1 USB 设备无法识别

```bash
# 检查设备
lsusb
# 检查驱动
lsmod | grep ftdi
# 加载驱动
sudo modprobe ftdi_sio
sudo modprobe usbserial
```

### 10.2 权限问题

```bash
sudo chmod 666 /dev/ttyUSB0
```

### 10.3 关节跟踪不顺畅

检查脚本中的关节限幅和控制参数，确保与控制器配置匹配。

### 10.4 仿真启动失败

确保工作空间已正确编译：

```bash
cd ~/dual_panda_ws
colcon build
source install/setup.bash
```

### 10.5 gello-teleop 安装失败

**解决方案**：跳过此步骤，本项目的自定义脚本不依赖 `gello-teleop`，安装失败不影响遥操作功能。

## 11. 与其他遥操作方式对比

| 遥操作方式 | 输入设备 | 控制空间 | 适用场景 |
|-----------|---------|---------|---------|
| GELLO | 舵机臂 | 关节空间 | 直观双臂遥操作 |
| 键盘 | 键盘 | 关节/笛卡尔 | 快速测试 |
| SpaceMouse | 3D鼠标 | 笛卡尔空间 | 精细笛卡尔控制 |

## 12. 注意事项

1. 确保 GELLO 设备已正确连接并上电
2. 首次使用前建议进行标定
3. 根据实验需求选择合适的启动方式
4. 完整模式需要更多系统资源
5. `gello-teleop` 安装失败不影响本项目的遥操作功能

---

**版本**: v1.0  
**最后更新**: May 2026  
**维护者**: GELLO 遥操作开发组  
**验证方式**: 实际测试确认无需 `gello-teleop`