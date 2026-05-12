# GELLO 遥操作使用指南

## 1. 概述

GELLO 遥操作是一种基于舵机臂的双臂遥操作方案，用于在 MuJoCo 仿真中控制 Franka 机械臂。本方案支持两种控制模式：基础关节阻抗控制和完整安全控制（带碰撞检测）。


## 2. 系统要求

- Ubuntu 22.04 + ROS2 Humble
- Python 3.10+
- 物理 GELLO 舵机臂（通过 USB 连接）

## 3. 安装步骤

### 3.1 安装 gello_software（核心依赖）

选择一个您喜欢的安装目录（示例位置）：

```bash
# 创建安装目录（可自定义路径）
mkdir -p ~/gello_software
cd ~/gello_software

# 克隆 gello_software 仓库
git clone https://github.com/wuphilipp/gello_software.git .

# 初始化并更新子模块（包含 Dynamixel SDK）
git submodule init && git submodule update

# 安装 gello_software（开发模式）
pip install -e .

# 安装 Dynamixel SDK（使用仓库中的子模块版本）
pip install -e third_party/DynamixelSDK/python
```

> **说明**：安装目录可自行选择，无需固定位置。建议选择非工作空间目录（如 `~/gello_software`），避免与 ROS2 工作空间冲突。

### 3.2 获取遥操作脚本

脚本已包含在 `multipanda_ros2` 仓库中。**方式一：已有工作空间（推荐）**

如果您已经有 `dual_panda_ws` 工作空间且已克隆 `multipanda_ros2` 仓库，只需更新到最新版本：

```bash
cd ~/dual_panda_ws/src/multipanda_ros2
git checkout humble
git pull origin humble
```

> **说明**：此命令会更新整个 `multipanda_ros2` 仓库，包含 `gello_teleop` 目录及其他文件。

**方式二：仅获取脚本目录**

如需单独获取 `gello_teleop` 目录（不克隆整个仓库）：

```bash
# 创建目录并初始化 git
mkdir -p ~/dual_panda_ws/src/multipanda_ros2/gello_teleop
cd ~/dual_panda_ws/src/multipanda_ros2/gello_teleop
git init
git remote add origin https://github.com/jichuan-yu/multipanda_ros2.git
git config core.sparseCheckout true
echo "gello_teleop/" >> .git/info/sparse-checkout
git pull origin humble
```

脚本位置：
```bash
~/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts/gello_franka_ros2.py
```

## 4. 硬件设置

### 4.1 连接 GELLO 舵机臂

将 GELLO 设备通过 USB 连接到电脑。

### 4.2 查看 USB 设备号

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

通常 GELLO 设备会被识别为 `/dev/ttyUSB0`。

### 4.3 设置权限（已有权限可跳过）

```bash
sudo chmod 666 /dev/ttyUSB0
# 或永久添加到 dialout 组（推荐）
sudo usermod -aG dialout $USER
newgrp dialout
```

## 5. 运行指南

### 5.0 仿真模式选择

根据需求选择启动方式：

| 模式 | 适用场景 | 启动命令 |
|------|---------|---------|
| **基础模式** | 快速测试、关节级控制 | `ros2 launch franka_bringup dual_franka_sim.launch.py` |
| **完整模式** | 完整实验、带安全控制 | `ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true` |

> **默认配置**：当前脚本默认使用完整模式，发布话题为 `/dualarm_teleop_cmd`，配合双臂安全控制器使用。
>
> **切换模式**：如需切换模式，需手动修改脚本中的发布话题。编辑 `scripts/gello_franka_ros2.py`，修改 `self.joint_publisher` 的话题名称：
> - 基础模式：`/dual_joint_impedance/joints_desired`
> - 完整模式：`/dualarm_teleop_cmd`

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

**问题现象**：GELLO 小幅度运动时从臂关节无响应，或跟随幅度小。

**解决方案**：调整关节阻抗控制器的刚度（k_gain）和阻尼（d_gain）参数。

编辑控制器配置文件：
```bash
vim ~/dual_panda_ws/src/multipanda_ros2/franka_bringup/config/sim/dual_sim_controllers.yaml
```

找到 `dual_joint_impedance_example_controller` 部分，增大从臂（arm_2）的关节增益：

```yaml
dual_joint_impedance_example_controller:
  ros__parameters:
    arm_count: 2
    arm_2:  # 从臂
      arm_id: mj_right
      k_gains:  # 刚度增益，增大可提高跟踪力度
        - 24.0
        - 24.0
        - 24.0
        - 24.0
        - 10.0
        - 6.0
        - 2.0    # 关节7，可尝试增大到 5.0 或更高
      d_gains:  # 阻尼增益，增大可提高响应速度
        - 2.0
        - 2.0
        - 2.0
        - 1.0
        - 1.0
        - 1.0
        - 0.5    # 关节7，可尝试增大到 1.0 或更高
```

修改后重新编译工作空间：
```bash
cd ~/dual_panda_ws
colcon build
source install/setup.bash
```

> **提示**：通常关节7的增益需要特别调整，因为其运动范围和负载特性与其他关节不同。

### 10.4 仿真启动失败

确保工作空间已正确编译：

```bash
cd ~/dual_panda_ws
colcon build
source install/setup.bash
```


**版本**: v1.0  
**最后更新**: May 2026 