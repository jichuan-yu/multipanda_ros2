# GELLO 遥操作使用指南（dual_panda_ws_latest）

## 1. 概述

GELLO 是一种基于 Dynamixel 舵机臂的低成本遥操作方案，本项目用它驱动 Franka 机械臂：

```
GELLO 舵机臂 --(USB/Dynamixel)--> 桥接脚本 --(ROS 2)--> 关节阻抗控制器 --> 机械臂
```

`gello_teleop/scripts/` 下提供三个桥接脚本，区别只在“把关节目标发到哪个话题”：

| 脚本 | 目标话题 | 数据路径 | 适用场景 |
|------|---------|---------|---------|
| `gello_franka_ros2.py` | `/<arm_id>/joints_desired` (`sensor_msgs/JointState`) | 直接驱动 `dual_joint_impedance_controller` | 快速验证、调试；**绕过安全层** |
| `gello_franka_safety_ros2.py` | `/dualarm_teleop_cmd` (`Float64MultiArray`, `command_type=4`) | 经 `main_sim_node` 安全控制器（限幅／避障／相对位姿约束／QP）后再下发 | 正式实验（推荐） |
| `gello_franka_singlearm.py` | `/panda/joints_desired` (`sensor_msgs/JointState`) | 单臂 `joint_impedance_controller` | 单臂调试 |

辅助脚本：

| 脚本 | 作用 |
|------|------|
| `gello_smoother.py` | 关节目标平滑器（速度限幅、死区、保持上一拍） |
| `gello_calibrate_offsets.py` | 交互式标定，把新 `joint_offsets` 写回 `config/gello_arm{1,2}.yaml` |
| `gello_franka_singlearm_monitor.py` | 纯 GELLO 读数打印（不依赖 ROS），用于检查硬件/标定 |

> 与旧工作空间（`~/dual_panda_ws`）的主要差异：旧版用一个 14 维 `Float64MultiArray`
> 发到 `/dual_joint_impedance/joints_desired`；新版控制器按臂分别订阅
> `/<arm_id>/joints_desired`，消息类型为 `sensor_msgs/msg/JointState`，**必须携带关节名**
> （`<arm_id>_joint1` … `<arm_id>_joint7`）。
>
> 旧工作空间 `~/dual_panda_ws/src/multipanda_ros2/tools/` 下的
> `gello_joint_teleop_node.py` / `gello_cartesian_teleop_node.py` 依赖已不存在的
> `gello_teleop.gello_teleop_agent` 模块，且仍向示例控制器发 14 维数组，
> **未迁移**到本工作空间；请使用上表中的三个桥接脚本。

## 2. 系统要求

- Ubuntu 22.04 + ROS 2 Humble
- Python 3.10+
- 物理 GELLO 舵机臂（USB 转串口）
- `gello` Python 包（见 3.1）

## 3. 安装

### 3.1 安装 gello_software（核心依赖）

```bash
mkdir -p ~/gello_software
cd ~/gello_software

# 可选：创建独立 conda 环境（Python 3.11）
conda create -n panda python=3.11 -y
conda activate panda

git clone https://github.com/wuphilipp/gello_software.git gello_software
cd gello_software
git submodule init
git submodule update

pip install -r requirements.txt --no-deps
pip install -e .
pip install -e third_party/DynamixelSDK/python
```

宿主机（如本仓库的开发机）也可以直接装进系统 Python，无需 conda：

```bash
python3 -c "import gello; print(gello.__file__)"
```

只要能在**运行桥接脚本的同一个解释器**里 `import gello` 即可。

### 3.2 桥接脚本位置

`gello_teleop` 是脚本目录（不含 `package.xml`），直接用 `python3` 运行，不需要 `colcon build`：

```bash
~/dual_panda_ws_latest/src/multipanda_ros2/gello_teleop/
├── config/
│   ├── gello_arm1.yaml        # 左臂：串口、joint_ids、offsets、signs、gripper_config
│   ├── gello_arm2.yaml        # 右臂
│   └── yam_auto_generated.yaml
├── img/
└── scripts/
```

## 4. 硬件设置

### 4.1 连接 GELLO

把 GELLO 通过 USB 连接到主机，查看设备号：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

通常会被识别为 `/dev/ttyUSB0`（左臂）与 `/dev/ttyUSB1`（右臂）。确认与
`config/gello_arm1.yaml` / `gello_arm2.yaml` 中的 `port` 一致。

### 4.2 权限

```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
# 或永久加入 dialout 组（推荐）
sudo usermod -aG dialout $USER
newgrp dialout
```

## 5. 话题与数据流

### 5.1 仿真（MuJoCo）

`dualarm_task_bringup/tray_transport.launch.py` 默认启动
`dual_joint_impedance_controller`（臂名 `mj_left` / `mj_right`）、两个
`gripper_action_bridge` 和 `main_sim_node`：

```
GELLO ─┬─ 直接模式 ──> /mj_left/joints_desired (JointState)
       │                        └─> dual_joint_impedance_controller
       └─ 安全模式 ──> /dualarm_teleop_cmd (type=4)
                              └─> main_sim_node ──> /mj_left/joints_desired
夹爪   ──> /mj_left_gripper/width_desired (Float64, m)
反馈   <── /mj_left/joint_states (JointState, 100 Hz)
```

### 5.2 真机

`franka_bringup/launch/real/dual_franka_control.launch.py` 默认臂名
`panda_left` / `panda_right`：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/panda_left/joints_desired` | `sensor_msgs/JointState` | 左臂目标关节 |
| `/panda_right/joints_desired` | `sensor_msgs/JointState` | 右臂目标关节 |
| `/panda_left/joint_states` | `sensor_msgs/JointState` | 左臂反馈 |
| `/panda_right/joint_states` | `sensor_msgs/JointState` | 右臂反馈 |
| `/panda_left_gripper/width_desired` | `std_msgs/Float64` | 左夹爪宽度（米，0–0.08） |
| `/panda_right_gripper/width_desired` | `std_msgs/Float64` | 右夹爪宽度 |

> 夹爪宽度 = `0.08 * (1 - gello_gripper_value)`，`gello_gripper_value` 来自舵机 ID 8
> （`gripper_config: [8, 195, 152]`）。

## 6. 运行指南（仿真）

所有终端都先执行：

```bash
export DUAL_PANDA_WS=~/dual_panda_ws_latest
cd "$DUAL_PANDA_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 6.1 终端 1：启动仿真（含阻抗控制器、夹爪桥、安全控制器）

```bash
ros2 launch dualarm_task_bringup tray_transport.launch.py use_rviz:=true
```

等待日志出现 `Joint impedance controller is active`。此时 MuJoCo 处于暂停状态
（`unpause:=false`）：先启动 GELLO 桥接（6.2 或 6.3），再在 MuJoCo 窗口按 Space
启动物理仿真。

常用参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `start_controller` | `true` | 是否启动 `main_sim_node` 安全控制器 |
| `arm_id_1` / `arm_id_2` | `mj_left` / `mj_right` | 仿真臂名（改了就同步改桥接脚本 `--left-arm-id`） |
| `use_rviz` | `false` | 是否启动 RViz |
| `record_data` | `false` | 是否录制控制器 CSV |

### 6.2 终端 2：GELLO（推荐走安全层）

```bash
python3 src/multipanda_ros2/gello_teleop/scripts/gello_franka_safety_ros2.py --control-mode 3
```

`main_sim_node` 会对绝对关节目标做限幅、避障、相对位姿约束和 QP 后再发给
`dual_joint_impedance_controller`。安全层未收到 `/dualarm_teleop_cmd` 时不会动作，
因此干净启动顺序是：仿真就绪 → GELLO 桥接 → 再在 MuJoCo 里按 Space。

### 6.3 直接驱动（绕过安全层，仅调试用）

需要先关掉 `main_sim_node`，否则两个节点会抢同一个话题：

```bash
# 终端 1
ros2 launch dualarm_task_bringup tray_transport.launch.py use_rviz:=true start_controller:=false

# 终端 2
python3 src/multipanda_ros2/gello_teleop/scripts/gello_franka_ros2.py --control-mode 3
```

该脚本自身做关节限幅 + 平滑，并按控制模式只发布被选中臂的话题。

### 6.4 验证数据流

```bash
ros2 topic hz /mj_left/joint_states                    # 反馈 ~100 Hz
ros2 topic echo /mj_left/joints_desired --once         # 直接模式：目标关节
ros2 topic echo /dualarm_teleop_cmd --once             # 安全模式：type=4 命令
ros2 topic echo /mj_left_gripper/width_desired --once  # 夹爪宽度
```

### 6.5 无硬件自检（不需要 GELLO / 机械臂）

`tools/tests/gello_bridge_smoke.py` 用假 GELLO（固定关节读数）替换真实舵机臂，
再喂一份假的 `/joint_states`，检查两个双臂桥接脚本发布的**话题名、关节名和消息长度**：

```bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
  python3 src/multipanda_ros2/tools/tests/gello_bridge_smoke.py
```

期望输出（`mj_*` 与 `panda_*` 两种臂名各验证一次）：

```
PASS direct bridge (mj_left/mj_right): joints_desired + gripper width
PASS direct bridge (panda_left/panda_right): joints_desired + gripper width
PASS safety bridge (arm_id=panda): /dualarm_teleop_cmd type=4 + gripper width
PASS: GELLO bridge smoke test finished
```

> 该脚本会占用 `/joint_states` 和 `/dualarm_teleop_cmd`，必须用独立 `ROS_DOMAIN_ID`
> 运行，避免与真实仿真/机械臂互相干扰。


## 7. 运行指南（真机）

```bash
# 终端 1：双臂 + 关节阻抗控制器 + 夹爪桥
ros2 launch franka_bringup dual_franka_control.launch.py \
  robot_ip_1:=172.16.0.2 \
  robot_ip_2:=172.16.0.3 \
  load_gripper_1:=true load_gripper_2:=true \
  controller_name:=dual_joint_impedance_controller \
  use_rviz:=false

# 终端 2：GELLO（默认臂名 panda_left/panda_right，夹爪 /panda_*_gripper/width_desired）
python3 src/multipanda_ros2/gello_teleop/scripts/gello_franka_ros2.py --hardware-mode 1
```

> 真机上没有安全层时**务必**先确认启动位姿与 GELLO 位姿一致（单臂脚本内置
> 启动对齐检查，双臂脚本启动日志会打印首帧 GELLO 读数，请人工核对）。
> 需要不同臂名时显式传 `--left-arm-id` / `--right-arm-id`。

## 8. 控制模式与命令行参数

### 8.1 控制模式（`--control-mode`）

| 模式 | 值 | 说明 |
|------|-----|------|
| `LEFT_ARM` | 1 | 只控制左臂（右臂不发任何指令） |
| `RIGHT_ARM` | 2 | 只控制右臂（左臂不发任何指令） |
| `BOTH_ARMS` | 3 | 双臂同时控制（默认） |

> 两个双臂脚本在启动时都会初始化左右两条 GELLO（`config/gello_arm1.yaml` / `gello_arm2.yaml`），
> 即使 `--control-mode` 只选了单臂也需要两个串口都能打开；只接一条舵机臂时请用
> `gello_franka_singlearm.py`。

### 8.2 `gello_franka_ros2.py` / `gello_franka_safety_ros2.py`

| 参数 | 默认 | 说明 |
|------|------|------|
| `--left-config` | `config/gello_arm1.yaml` | 左臂 GELLO 配置 |
| `--right-config` | `config/gello_arm2.yaml` | 右臂 GELLO 配置 |
| `--control-mode` | `3` | 见 8.1 |
| `--hardware-mode` | `0` | `0`=仿真（`use_sim_time=true`，臂名 `mj_*`）；`1`=真机（臂名 `panda_*`） |
| `--publish-hz` | `50.0` | 命令发布频率 |
| `--left-arm-id` / `--right-arm-id` | 随 `--hardware-mode` | 仅 `gello_franka_ros2.py`：覆盖默认臂名（`mj_left`/`mj_right` 或 `panda_left`/`panda_right`） |
| `--arm-id` | 随 `--hardware-mode` | 仅 `gello_franka_safety_ros2.py`：关节名前缀（仿真 `mj`，真机 `panda`） |

### 8.3 `gello_franka_singlearm.py`

| 参数 | 默认 | 说明 |
|------|------|------|
| `--config` | `config/gello_arm1.yaml` | GELLO 配置（相对路径按 `gello_teleop/config/` 解析） |
| `--arm-id` | `panda` | 关节名前缀 |
| `--target-topic` | `/panda/joints_desired` | 关节目标话题 |
| `--joint-state-topic` | `/panda/joint_states` | 关节反馈话题 |
| `--gripper-topic` | `/panda_gripper/width_desired` | 夹爪宽度话题 |
| `--publish-hz` | `50.0` | 发布频率 |
| `--no-smoother` | 关闭 | 关闭平滑，直接透传 GELLO 读数 |

单臂仿真示例（`franka_bringup/launch/sim/franka_sim.launch.py`，臂名 `panda`）：

```bash
ros2 launch franka_bringup franka_sim.launch.py
ros2 run controller_manager spawner joint_impedance_controller
python3 src/multipanda_ros2/gello_teleop/scripts/gello_franka_singlearm.py
```

> 单臂仿真 launch 不会启动 `gripper_action_bridge`，所以夹爪话题无人订阅；
> 单臂真机（`franka_control.launch.py` 会启动 `gripper_action_bridge_real`，
> `arm_id=panda` → `/panda_gripper/width_desired`）才有夹爪控制。

## 9. 标定

### 9.1 原理

GELLO 每个舵机的零位与 Franka 关节零位不同，桥接脚本按 YAML 里的
`joint_offsets` / `joint_signs` 做线性换算：

```yaml
agent:
  port: "/dev/ttyUSB0"
  dynamixel_config:
    joint_ids: [1, 2, 3, 4, 5, 6, 7]
    joint_offsets: [...]   # 每个关节的零位偏移（rad）
    joint_signs: [...]     # 每个关节的方向（±1）
    gripper_config: [8, 195, 152]   # [舵机ID, 张开角度, 闭合角度]
```

### 9.2 查看当前读数

```bash
python3 src/multipanda_ros2/gello_teleop/scripts/gello_franka_singlearm_monitor.py \
  --port /dev/ttyUSB0 --rate 5
```

### 9.3 重新标定并写回 YAML

把 GELLO 摆成与机械臂一致的姿态后运行：

```bash
python3 src/multipanda_ros2/gello_teleop/scripts/gello_calibrate_offsets.py \
  --arm both
```

脚本会读取当前舵机角度、换算新 `joint_offsets`，并写回
`config/gello_arm1.yaml` 与 `config/gello_arm2.yaml`（可用 `--left-config` /
`--right-config` 指定其他路径）。

## 10. 文件结构

```
src/multipanda_ros2/
├── gello_teleop/
│   ├── config/
│   │   ├── gello_arm1.yaml            # 左臂标定/串口
│   │   ├── gello_arm2.yaml            # 右臂标定/串口
│   │   └── yam_auto_generated.yaml
│   ├── img/
│   └── scripts/
│       ├── gello_franka_ros2.py            # 双臂直连（/<arm_id>/joints_desired）
│       ├── gello_franka_safety_ros2.py     # 双臂经 /dualarm_teleop_cmd
│       ├── gello_franka_singlearm.py       # 单臂
│       ├── gello_franka_singlearm_monitor.py
│       ├── gello_calibrate_offsets.py
│       └── gello_smoother.py
├── docs/
│   ├── gello.md                       # 本说明
│   └── RUN_REAL.md
└── ...
```

## 11. 故障排查

### 11.1 USB 设备无法识别

```bash
lsusb
lsmod | grep ftdi
sudo modprobe ftdi_sio
sudo modprobe usbserial
```

### 11.2 `Config file not found` / `Failed to initialize Gello Agent`

- 确认 `config/gello_arm*.yaml` 存在，或用 `--left-config/--right-config` 指定绝对路径；
- 确认 `port` 与 `ls -l /dev/ttyUSB*` 一致，并且当前用户有读写权限；
- 确认运行脚本的解释器能 `import gello`（见 3.1）。

### 11.3 日志一直显示 `Waiting for joint states to initialize smoothers...`

桥接脚本只在收到关节反馈后才下发目标（避免从零位跳变）。检查：

```bash
ros2 topic list | grep joint_states
ros2 topic echo /mj_left/joint_states --once
```

- 仿真：确认 `dual_joint_impedance_controller` 已激活（`Joint impedance controller is active`），
  它负责发布 `/<arm_id>/joint_states`；
- 真机：确认控制器已 spawn，`/panda_left/joint_states` 有数据；
- 若臂名不是 `mj_*` / `panda_*`，用 `--left-arm-id` / `--right-arm-id`（双臂）或
  `--joint-state-topic`（单臂）对齐。

### 11.4 机械臂不动 / 只轻微抖动

- **直接模式**下若 `main_sim_node` 同时在跑，会覆盖你的目标：用
  `start_controller:=false` 关掉它，或改用 `gello_franka_safety_ros2.py`；
- **安全模式**下 `main_sim_node` 若判定违反安全约束（碰撞、相对位姿），会拒绝或修正命令，
  查看其终端日志；
- 关节 7 跟随偏软：调大 `franka_bringup/config/*/dual_*controllers.yaml` 中
  `dual_joint_impedance_controller` 对应臂的 `k_gains`/`d_gains` 第 7 项。

### 11.5 启动瞬间大幅跳动

- 平滑器需要 `/joint_states` 才能初始化到实际位姿，启动前保持 GELLO 与机械臂姿态接近；
- 单臂脚本内置 1.0 rad 的启动对齐检查（`startup_joint_threshold`），超限会直接退出并打印两边的关节值。

### 11.6 夹爪不动作

```bash
ros2 topic info /mj_left_gripper/width_desired
```

- 仿真：话题由 `gripper_action_bridge`（`arm_id=left` → `/mj_left_gripper/width_desired`）订阅，
  确认 `tray_transport.launch.py` 已启动；
- 真机：`gripper_action_bridge_real`（`arm_id=panda_left` → `/panda_left_gripper/width_desired`）；
- 单臂仿真 launch 不含夹爪桥，属预期行为。

真机上夹爪完全不动时，先看桥的终端日志，它现在会打印每条命令的结果：

| 日志 | 含义 / 处理 |
| --- | --- |
| `Gripper homing action server '.../homing' did not become available` | `franka_gripper` 节点没起来；起好后发布一次 `/panda_left_gripper/homing_desired`（`std_msgs/Bool`），否则 `max_width = 0`，所有 move/grasp 都会失败 |
| `goal rejected by the action server` | 未 homing，或上一条命令还没结束 |
| `failed: ...` | libfranka 报错（多为未 homing 或夹爪未使能） |
| `stopped short of the target width (object detected)` | 正常：闭合时夹到物体，力达到 `force` 后停止 |
| `no goal response / no result within ... s` | 夹爪节点卡住，检查 `/franka_gripper` 日志；桥会发 cancel 并继续下一条 |

实现上桥已对齐官方 `gello/robots/panda.py`：`command_joint_state()` 每个控制周期发一条
**阻塞**命令（`gripper.goto(width, speed=1, force=1)`，内部 `goto` 会等动作结束），
桥同样只保留最新的 `width_desired` 样本、等上一条动作返回后再发下一条。
50 Hz 的原始样本不会被逐条转发（否则每条 `Grasp` 会在加速阶段被下一条 abort）。
可调参数：`speed`、`force`、`epsilon_inner`/`epsilon_outer`、`min_change`、`max_width`、
`use_grasp`、`auto_home`、`command_timeout`、`home_timeout`。离线验证脚本：

```bash
python3 tools/tests/gripper_bridge_panda_style_test.py            # default Grasp 路径
python3 tools/tests/gripper_bridge_panda_style_test.py --use-move # use_grasp:=false
```

---
**最后更新**: 2026-10（适配 `dual_panda_ws_latest` 的 per-arm `/<arm_id>/joints_desired` 接口）
