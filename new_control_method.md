# 联合控制方法与双臂阻抗控制器开发总结

本文档记录了在 ROS 2 + MuJoCo 仿真环境中，单臂控制器的调试过程以及基于 `instruction.md` 需求开发的全新双臂联合阻抗控制器（Multi-Joint Impedance Controller）的技术细节。

## 1. 单臂控制器 Bug 排查与修复

### 1.1 浮点数隐式转换问题 (0.0 被判定为 False)
在单臂调试初期，发现即使成功加载了控制器并发布了目标关节位姿，机器人的状态也没有更新。
- **问题根源**：原 C++ 源码在 `desiredJointCallback` 中使用了 `if (msg.data[0])` 的逻辑来验证数据有效性。当第一个关节的目标角度恰好发布为 `0.0` 时，C++ 隐式将其转换为布尔值 `false`，导致回调函数直接提前退出（过滤掉了指令）。
- **修复方案**：在当时的测试 Python 脚本中临时将 `0.0` 改为 `0.001` 以规避该错误。修改后单臂控制器恢复正常指令跟踪。

### 1.2 仿真时钟 (TF Time Jump) 警告刷屏屏蔽
- **问题根源**：在 `use_sim_time=True` 的状态下，MuJoCo 生成 `/clock` 话题。由于仿真步长的微小精度抖动（纳秒级别的微小回退），ROS 2 的 `tf2_buffer` 会频繁报 `[WARN] [tf2_buffer] [onTimeJump]: Detected jump back in time` 警告。
- **修复方案**：这属于仿真环境正常现象，不影响控制逻辑。通过在启动 launch 文件时添加 `--ros-args --log-level WARN` 或 `ERROR` 参数进行日志层级过滤屏蔽。

---

## 2. 新型双臂联合控制器 (MultiJointImpedanceController)

按照需求，我开发了用于双臂（`mj_left` 和 `mj_right`）协调控制的新控制器，它能够通过单个话题接收双臂协同任务的指令。

### 2.1 C++ 控制器架构
- **新增文件**：
  - `multi_joint_impedance_controller.hpp`
  - `multi_joint_impedance_controller.cpp`
- **动态寻址**：通过读取 `arm_count`（动态臂数）参数，并在初始化阶段通过 `arm_1.arm_id`、`arm_2.arm_id`（分别为 `mj_left`、`mj_right`）动态申请硬件控制接口。
- **数据解包与数组切片 (Array Slicing)**：
  - 双臂控制器订阅 `/dual_joint_impedance/joints_desired` 话题，接收长度为 `14` 的 `Float64MultiArray` 数据。
  - 数据解析逻辑：前 7 个 double 元素作为 `mj_left` 的目标关节位置，后 7 个元素作为 `mj_right` 的目标关节位置。
- **PID 与阻抗控制 (Impedance Control)**：
  - 分别获取两只手臂的当前姿态 (`q`)、速度 (`dq`)，再结合配置文件中设定的刚度 (`k_gains`) 和阻尼 (`d_gains`) 求解目标扭矩，并将扭矩下发至末端执行器。

### 2.2 控制器注册与配置 (Pluginlib & 编译系统)
为了让 ROS 2 的 `controller_manager` 能够识别并加载我们写好的新控制器，进行了以下核心配置：
- **`franka_example_controllers.xml`**：对外暴露出 `franka_example_controllers/MultiJointImpedanceController` 插件。
- **`CMakeLists.txt`**：将新的 `.cpp` 源文件加入到编译链路中。
- **`dual_sim_controllers.yaml`**：将 `dual_joint_impedance_controller` 的 `type` 绑定至新插件，并在 `ros__parameters` 下分配各自机器人的 PID 增益参数 (`k_gains`/`d_gains`) 和各自的 id。

---

## 3. 自动化测试脚本 (`tools/dual_auto_pub.py`)

为了验证上述双臂控制器的执行效果，编写了一个 Python 发型节点：
- **发布维度**：每 0.02 秒以 50Hz 频率发布 `长度为14的张量`。
- **运动规律**：
  - **左臂 (mj_left)**: 关节 4 和 5 施加基于 `math.cos` 的周期正弦运动。
  - **右臂 (mj_right)**: 关节 4 和 5 施加基于 `math.sin` 的周期正弦运动。
  - **相位差 (Phase Difference)**：使得两只手臂在 Y/Z 轴方向呈现相差 90 度的波形交互协作动作。

---

## 4. 运行及验证命令速查

在确认环境编译无误（`colcon build` 之后）时：

1. **启动双臂仿真**：
   ```bash
   ros2 launch franka_bringup dual_franka_sim.launch.py --ros-args --log-level WARN
   ```
2. **激活双臂阻抗控制器**：
   ```bash
   ros2 control load_controller dual_joint_impedance_controller --set-state active
   ```
3. **下发控制指令信号**：
   ```bash
   python3 tools/dual_auto_pub.py
   ```