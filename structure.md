# dualarm_reactive_control 项目技术文档

该程序包环境要求、避障功能标准接口及遥控集成方案如下：

## 1. 环境依赖要求

根据工程根目录的 `README.md` 及包内的 `CMakeLists.txt` / `package.xml`，所需环境如下：

* **基础环境：**
    * **操作系统：** Ubuntu 22.04 LTS
    * **机器人系统：** ROS2 Humble（配合 `colcon` 构建系统，实机运行需要实时内核 Real-time kernel）
    * **C++ 标准：** C++14 及以上
* **核心依赖库：**
    * **libfranka 0.9.2：** Franka 机器人底层控制接口
    * **MuJoCo 3.2.0：** 物理仿真引擎
    * **DQ Robotics：** 用于运动学或对偶四元数计算
    * **Eigen3 / Boost System**
* **关键相关功能包：**
    * **开源仓库依赖：** 需要配合同一工作空间下的 `multipanda_ros2` 仓库以及 `mujoco_ros_pkgs`。
    * **常规 ROS 依赖包：** `rclcpp`、`geometry_msgs`、`shape_msgs`、`trajectory_msgs`、`kdl_parser`、`tf2`、`tf2_ros`、`franka_gripper`、`franka_msgs` 等。

## 2. 避障功能的标准输入输出接口

系统的避障核心建立在**层次化二次规划 (HQP)** 与碰撞检测机制上。主要节点如 `dual_arm_safe_controller_sim / exp` 会接收外部输入目标及障碍物，融合处理后输出底层的安全关节指令。

### 📥 标准输入接口 (Input)

1.  **动态障碍物信息话题：**
    * **Topic:** `/dynamic_obstacle`
    * **消息类型：** 自定义消息 `dual_arm_hqp_controller/CollisionObject`（参考 `moveit_msgs`）
    * **包含字段：**
        * `id`: 障碍物唯一标识符。
        * `geometry` (`shape_msgs/SolidPrimitive`): 支持 BOX、SPHERE、CYLINDER。
        * `pose` (`geometry_msgs/Pose`): 三维空间位姿。
        * `twist` (`geometry_msgs/Twist`): 速度与旋转趋势（用于前瞻避障）。
        * `operation` (byte): 操作指令，如添加 (ADD=0)、移除 (REMOVE=1) 或仅移动目标 (MOVE=3)。
2.  **静态障碍物配置：**
    * 从参数服务器中直接以 yaml 或配置参数字典的方式传入（由 `CollisionEnv::loadCollisionObjects` 读取并缓存）。
3.  **自身状态监测（隐式避障输入）：**
    * 通过订阅 `/panda_1` 和 `/panda_2` 的 `/joint_states` 和 `/franka_states` 话题获取当前机械臂状态。
4.  **参考目标轨迹：**
    * 订阅话题 `/dualArm_traj` 接收双臂期望跟随的初始参考路线。

### 📤 标准输出接口 (Output)

1.  **安全控制指令（避障修正后）：**
    * 控制器进行硬约束/软约束求解后，输出安全的关节位点：
    * **Topic:** `/panda_1/panda_1_joint_impedance_tracking_controller/joint_command`
    * **Topic:** `/panda_2/panda_2_joint_impedance_tracking_controller/joint_command`
    * **消息类型：** `trajectory_msgs/JointTrajectoryPoint`（包含关节位置、速度等）。
2.  **可视化输出（用于 RViz）：**
    * 通过 `rviz_visualization` 节点将障碍物包装为 Marker：
    * **Topic:** `/dynamic_obstacle_markers`
    * **消息类型：** `visualization_msgs/MarkerArray`

---

## 3. 遥控集成架构方案：SpaceMouse + Reactive Control

旨在解决 `spacemouse_teleop` 脚本与 `dualarm_reactive_control` 在任务空间接口和底层驱动模式上的差异。

### 当前存在的“数据鸿沟”
* **SpaceMouse 遥控端：** 输出笛卡尔位姿（`std_msgs::Float64MultiArray`），针对的是工作空间阻抗控制器。
* **Reactive Control 安全约束端：** 输入为关节空间轨迹（`trajectory_msgs::JointTrajectory`），要求底层运行关节空间阻抗跟踪控制器。

### 架构策略设计

#### 方案一：级联桥接架构（前置 IK）
在保持现有代码逻辑不变的前提下，增加一个中间节点（IK Node）进行信号“翻译”。
* **信号流：** SpaceMouse -> IK Node (计算 $q_{ref}, \dot{q}_{ref}$) -> `/dualArm_traj` -> SafeController。
* **特点：** 实现快，但将遥控约束在特定关节参考上，遇到障碍物时无法充分利用 7自由度的零空间（Null-space）性能。

#### 方案二：HQP 原生融合架构（推荐）
将笛卡尔任务直接写入 `dual_arm_safe_controller` 内部的 HQP 代价函数（Cost Function）。
* **核心思想：** 将代价函数从关节跟踪 $\|\dot{q} - \dot{q}_{ref}\|^2$ 改为笛卡尔空间误差 $\|J(q)\dot{q} - \dot{X}_{cmd}\|^2$。
* **特点：** 遥控顺滑度卓越。控制器自动利用冗余自由度在零空间“闪避”障碍物，仅在必然碰撞时才牺牲操作精度。

---

## 4. 方案二（HQP 原生融合）详细开发路线

### 阶段一：通讯接口改造
1.  **修改 `spacemouse_pub.py`：** 将输出从绝对位姿改为**增量速度指令 (Twist)**。
    * 读取 6 轴摇杆偏移量，乘以缩放系数 $(scale\_pos, scale\_rot)$ 得到期望线速度 $v_{des}$ 和角速度 $\omega_{des}$。
2.  **修改安全控制器订阅：** 移除 `/dualArm_traj` 订阅，新增 `/spacemouse/twist_cmd` 订阅，更新成员变量 `V_cmd_left` 和 `V_cmd_right`。

### 阶段二：核心算法层改造 (HQP 优化目标修改)
在 `constraint_manager.cpp` 中重写代价函数：
1.  **获取末端雅可比矩阵：** 利用运动学库获取 $J_{left}$ 和 $J_{right}$，组合成 $J_{14}$。
2.  **重写代价函数：** 最小化末端实际速度 $J\dot{q}$ 与期望速度 $V_{cmd}$ 的误差：
    $$\min_{\dot{q}} \frac{1}{2} \|J\dot{q} - V_{cmd}\|_W^2$$
    展开为 QP 标准型 $\frac{1}{2}\dot{q}^T H \dot{q} + f^T \dot{q}$ 后：
    * **二次项系数矩阵：** $H = J^T W J + \lambda I$
    * **一次项系数向量：** $f = -J^T W V_{cmd}$
3.  **保留安全约束：** 碰撞避免的 CBF 约束方程 $A\dot{q} \le b$ 无需改动。

### 阶段三：仿真环境验证
1.  **场景搭建：** 在 `my_task_description` 中加载双臂 Panda 及动态测试障碍物。
2.  **底层加载：** MuJoCo ROS2 层挂载 `joint_impedance_controller`。
3.  **预期表现：**
    * **无障碍物：** 机械臂末端完全遵循手部操作。
    * **自碰撞/限位：** 尝试危险动作时，会被 HQP 判定约束阻挡。
    * **冗余避障：** 操控末端不动时，若障碍物飞向手肘，手肘会自动抬起（利用零空间）避障，而末端维持原位。

### 阶段四：进阶优化（冗余空间位姿保持）
在代价函数中加入**引力惩罚项 (Null-space Task)**，引导机械臂向舒适的 "Home 位姿" ($q_{rest}$) 恢复：
* $H_{new} = J^T W J + k_{null} I$
* $f_{new} = -J^T W V_{cmd} + k_{null}(q - q_{rest})$