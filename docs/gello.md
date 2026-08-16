# GELLO 遥操作使用指南

## 1. 概述

GELLO 遥操作是一种基于舵机臂的遥操作方案，用于在 MuJoCo 仿真中控制 Franka 机械臂。支持单臂和双臂两种配置。

## 2. 系统要求

- Ubuntu 22.04 + ROS2 Humble
- Python 3.10+
- 物理 GELLO 舵机臂（通过 USB 连接）

## 3. 安装步骤

### 3.1 安装 gello_software（核心依赖）

选择一个安装目录（示例位置）：

```bash
# 创建安装目录并克隆仓库
mkdir -p ~/gello_software
git clone https://github.com/wuphilipp/gello_software.git ~/gello_software

# 进入仓库目录
cd ~/gello_software

# 初始化并更新子模块（包含 Dynamixel SDK）
git submodule init
git submodule update

# 安装 gello_software（开发模式）
pip install -e .

# 安装 Dynamixel SDK（使用仓库中的子模块版本）
pip install -e third_party/DynamixelSDK/python
```

> **说明**：安装目录可自行选择，无需固定位置。建议选择非工作空间目录（如 `~/gello_software`），避免与 ROS2 工作空间冲突。

### 3.2 获取遥操作脚本

脚本已包含在 `multipanda_ros2` 仓库中：

```
~/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts/
├── gello_franka_ros2.py            # 双臂遥操作脚本（直连低层阻抗，绕过安全层）
├── gello_franka_safety_ros2.py     # 双臂遥操作脚本（经安全控制器 /dualarm_teleop_cmd）
├── gello_franka_singlearm.py       # 单臂遥操作脚本
└── gello_smoother.py               # 平滑器
```

## 4. 硬件设置

### 4.1 连接 GELLO 舵机臂

将 GELLO 设备通过 USB 连接到电脑。

### 4.2 查看 USB 设备号

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

通常 GELLO 设备会被识别为 `/dev/ttyUSB0`。请确认脚本中设备路径是否正确。

### 4.3 设置权限（已有权限可跳过）

```bash
sudo chmod 666 /dev/ttyUSB0
# 或永久添加到 dialout 组（推荐）
sudo usermod -aG dialout $USER
newgrp dialout
```

## 5. 运行指南

### 5.1 双臂遥操作脚本

适用于控制双臂 Franka 机械臂，支持三种控制模式，带平滑功能。

**终端 1：启动双臂仿真**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 launch franka_bringup dual_franka_sim.launch.py
```

**终端 2：启动双臂关节阻抗控制器**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 run controller_manager spawner dual_joint_impedance_controller
```

**终端 3：启动双臂遥操作脚本**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
cd ~/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts
python3 gello_franka_ros2.py
```

**控制模式**：编辑 `gello_franka_ros2.py` 中的 `CONTROL_MODE` 变量：

| 模式 | 值 | 说明 |
|------|-----|------|
| `LEFT_ARM`  | 1 | 只控制左臂，右臂保持默认位置 |
| `RIGHT_ARM` | 2 | 只控制右臂，左臂保持默认位置 |
| `BOTH_ARMS` | 3 | 双臂镜像控制（默认） |

### 5.2 双臂遥操作脚本（接入安全控制器）

**终端 1：启动双臂仿真**

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=false 
```

**终端 2：启动安全控制器 main_sim_node

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node --ros-args \
  -p bypass_qp_safety_for_debug:=false \
  -p relative_pose_constraint_enabled:=false
```


**终端 3：启动经安全控制器的双臂遥操作脚本**

```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
cd ~/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts
python3 gello_franka_safety_ros2.py --control-mode 3
```


`--control-mode` 取值与 5.1 相同：

| 模式 | 值 | 说明 |
|------|-----|------|
| `LEFT_ARM`  | 1 | 只控制左臂，右臂保持默认位置 |
| `RIGHT_ARM` | 2 | 只控制右臂，左臂保持默认位置 |
| `BOTH_ARMS` | 3 | 双臂控制（默认） |

其他可选参数：

- `--hardware-mode 0/1`：0=仿真（默认），1=真机（夹爪话题切换到 `/left_gripper/width_desired` 等）。
- `--publish-hz 50`：发布频率，默认 50 Hz（在安全控制器 0.5 s 超时范围内）。
- `--left-config / --right-config`：覆盖左右臂 GELLO YAML 标定文件路径（默认读取 config/ 目录）。

```

`/dualarm_teleop_cmd` 应为 17 个元素，末尾三项依次是 `[4.0, arm_selector, 0.0]`，
即 `command_type=4`（绝对关节位置）、所选臂、`allow_safety_violation=0`。

> **无 GELLO 硬件时调试**：可先用 `src/multipanda_ros2/teleop/key_teleop_joint_absolute.py` 或
> `src/multipanda_ros2/teleop/replay_teleop_cmd.py` 往 `/dualarm_teleop_cmd` 发 type=4 命令，
> 确认「sim + 安全控制器」链路通了之后再插 GELLO 硬件。

### 5.3 单臂遥操作脚本

适用于控制单臂 Franka 机械臂。

**终端 1：启动单臂仿真**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
ros2 launch franka_bringup franka_control_sim.launch.py controller_name:=joint_impedance_controller
```

**终端2 ：启动单臂遥操作脚本**
```bash
source /opt/ros/humble/setup.bash
source ~/dual_panda_ws/install/setup.bash
cd ~/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts
python3 gello_franka_singlearm.py
```

## 6. 标定说明

### 6.1 标定原理

GELLO 舵机臂的关节读数要经过 `(原始角 − joint_offsets) × joint_signs` 校正后才会与
Franka 机械臂对齐（校正公式位于 gello 库 `gello/robots/dynamixel.py`）。标定偏移
与方向分别存储在左右臂 YAML 中：

```text
gello_teleop/config/gello_arm1.yaml   # 左臂（串口 /dev/ttyUSB0）
gello_teleop/config/gello_arm2.yaml   # 右臂（串口 /dev/ttyUSB1）
```

标定采用**单点标准姿态**：把 GELLO 摆到下述姿态，修正后 7 个关节角应为

```text
[0, 0, 0, -0.5*pi, 0, 0.5*pi, 0.25*pi]
```

`joint_offsets` 的取整规则：

- 前 6 个关节：取**最近 0.5*pi 整数倍**；
- 第 7 个关节（末端旋转）：取**最近 0.25*pi 整数倍**。

> **注意**：`joint_offsets = 原始读数 − sign × 目标角`（再按上述规则取整）。换一次
> 装配或换一台 GELLO 舵机臂后需要重新标定。

### 6.2 重新标定步骤

**Step 1: 计算左臂 joint_offsets 并写回**

将**左臂** GELLO 摆到第 6.1 节的标准姿态，运行：

```bash
cd ~/dual_panda_ws/src/multipanda_ros2/gello_teleop/scripts
python3 gello_calibrate_offsets.py --arm left
```

脚本从 `gello_arm1.yaml` 读取串口与 joint_signs，按回车后读取当前原始角度，
打印逐关节对比表（原始角 / 取整后 offset / 校正后角 vs 目标 / 残差），
确认残差可接受后将结果写回 `gello_arm1.yaml`。

**Step 2: 计算右臂 joint_offsets 并写回**

将**右臂** GELLO 摆到同样姿态，运行：

```bash
python3 gello_calibrate_offsets.py --arm right
```

左、右臂各需一次（`--arm both` 可一次连标两条臂，会依序等待两次回车确认）。

**可选参数**：

- `--left-config / --right-config`：指定其他 YAML 路径（默认 config/ 目录）。
- `--auto-confirm`：跳过回车确认（需确保运行时已摆好标准姿态）。


### 6.3 标定脚本功能说明

`gello_teleop/scripts/gello_calibrate_offsets.py` 自动完成：

1. **读取配置**：从左右臂 YAML 加载串口、关节 id 与 joint_signs；
2. **原始读数**：`DynamixelDriver`（波特率 57600，10 次预热）读取当前 7 个关节电机原始角；
3. **偏移量计算**：`offset = raw − sign × target`，前 6 关节向最近 `0.5*pi`、第 7 关节向最近 `0.25*pi` 整数倍取整；
4. **格式安全写回**：**仅替换 YAML 中 `joint_offsets` 一行的数值**，原文件其余内容（缩进、注释、键顺序、flow 列表风格）保持不变；
5. **左右独立**：结果分别写回 `gello_arm1.yaml`、`gello_arm2.yaml`，互不影响。

## 7. 话题说明

### 双臂脚本话题（直连，`gello_franka_ros2.py`）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/dual_joint_impedance/joints_desired` | `Float64MultiArray` | 双臂关节目标（14个关节角度，绕过安全层） |

### 双臂遥操作脚本（接入安全控制器，`gello_franka_safety_ros2.py`）话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/dualarm_teleop_cmd` | `Float64MultiArray` | 遥操作命令输入（17 元素：14 关节 + 类型 + 臂选择 + 安全标志） |
| `/mj_left/joints_desired` | `JointState` | 安全控制器输出的左臂期望关节位置 |
| `/mj_right/joints_desired` | `JointState` | 安全控制器输出的右臂期望关节位置 |

### 单臂脚本话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_impedance/joints_desired` | `JointState` | 单臂关节目标 |
| `/panda/joint_states` | `JointState` | 单臂关节状态反馈 |

## 8. 文件结构

```
~/dual_panda_ws/src/multipanda_ros2/
├── gello_teleop/
│   ├── config/
│   │   └── yam_auto_generated.yaml  # GELLO 标定配置文件
│   └── scripts/
│       ├── gello_franka_ros2.py            # 双臂遥操作脚本（直连低层，绕过安全层）
│       ├── gello_franka_safety_ros2.py     # 双臂遥操作脚本（经安全控制器 /dualarm_teleop_cmd）
│       ├── gello_franka_singlearm.py       # 单臂遥操作脚本
│       └── gello_smoother.py               # 平滑器
└── docs/
    └── gello.md                     # 本说明文档
```

**版本**: v1.1
**最后更新**: Aug 2026