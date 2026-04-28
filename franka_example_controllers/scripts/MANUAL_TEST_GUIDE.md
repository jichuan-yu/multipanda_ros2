#!/bin/bash
# 手动测试师兄控制器 - 逐步指南

cat << 'EOF'
========================================
手动测试师兄控制器
========================================

此指南会逐步帮你定位问题根源。

前置条件：已启动仿真
  ros2 launch my_task_description my_task_sim.launch.py

========================================
步骤1：验证仿真环境
========================================

运行以下命令检查：

  ros2 node list

期望看到多个节点（spawner, controller等）

  ros2 topic hz /joint_states

期望看到约30-50 Hz的频率

如果以上都正常，进入步骤2。

========================================
步骤2：验证topic_bridge
========================================

新终端启动topic_bridge：

  source install/setup.bash
  ros2 run franka_example_controllers topic_bridge

期望看到：
  [INFO] [topic_bridge]: Topic Bridge started
  [INFO] [topic_bridge]: Subscribing: /joint_states
  [INFO] [topic_bridge]: Remapping: mj_left → panda_1, mj_right → panda_2

验证桥接话题：

  ros2 topic hz /panda_1/panda_1_franka_state_controller/joint_states

期望看到约30 Hz的频率

如果正常，进入步骤3。

========================================
步骤3：测试师兄控制器（无适配器）
========================================

新终端启动师兄控制器：

  source install/setup.bash
  ros2 run dual_arm_reactive_control dual_arm_mprc_node

观察输出：

✓ 成功标志：
  [INFO] Dual arm safe controller initialized
  [INFO] Dual Arm Reactive Control Node Started

❌ 失败标志：
  [ERROR] HQP safe controller failed to find a solution
  HQP Error: ub < lb

如果师兄控制器能正常运行（无HQP错误），
说明问题在适配器的IK求解。

如果师兄控制器立即崩溃（HQP错误），
说明问题在师兄控制器配置或关节限制。

========================================
步骤4：发送安全关节位置
========================================

如果步骤3成功（师兄控制器运行正常），
在另一个终端发送已知安全的关节位置：

  source install/setup.bash
  python3 src/multipanda_ros2/franka_example_controllers/scripts/test_safe_joint_position.py

这将直接发送初始关节位置（绕过IK求解）。

如果师兄控制器能接收并发布关节命令，
说明师兄控制器本身工作正常，
问题确实在适配器的IK求解。

========================================
步骤5：检查师兄控制器发布命令
========================================

  ros2 topic hz /dual_joint_impedance/joints_desired

期望看到约100 Hz的频率（师兄控制器的控制频率）

========================================
诊断结论
========================================

情况A：师兄控制器在步骤3就崩溃（HQP错误）
  → 师兄控制器配置问题
  → 检查：关节限制、碰撞球配置

情况B：师兄控制器能运行，但步骤4后仍崩溃
  → 适配器的数据格式问题
  → 检查：关节名称顺序、数据类型

情况C：师兄控制器完全正常，能发布命令
  → 问题在适配器的IK求解
  → 检查：IK算法、目标笛卡尔位置

情况D：师兄控制器不发布命令
  → 师兄控制器内部状态机问题
  → 检查：ControlState、轨迹缓冲区

========================================
EOF
