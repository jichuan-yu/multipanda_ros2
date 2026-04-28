#!/bin/bash
# 快速诊断命令

echo "=========================================="
echo "快速诊断 - 逐条运行以下命令"
echo "=========================================="
echo ""

cat << 'EOF'
# 1. 检查ROS2节点数量
ros2 node list

# 2. 检查joint_states频率
ros2 topic hz /joint_states --once --timeout 2

# 3. 检查panda_1关节状态（topic_bridge后）
ros2 topic hz /panda_1/panda_1_franka_state_controller/joint_states --once --timeout 2

# 4. 查看实际关节位置（第一条消息）
ros2 topic echo /joint_states --once | grep -A 20 "position:"

# 5. 查看师兄控制器是否在发布命令
ros2 topic hz /dual_joint_impedance/joints_desired --once --timeout 2

# 6. 如果师兄控制器运行中，检查其节点信息
ros2 node info /dual_arm_mprc_node

# 7. 检查师兄控制器的订阅话题
ros2 topic info /dualArm_traj

========================================
EOF

echo "当前系统状态："
echo ""

# 执行快速检查
echo -n "ROS2节点数: "
node_count=$(ros2 node list 2>/dev/null | wc -l)
echo "$node_count"

echo -n "joint_states频率: "
timeout 2 ros2 topic hz /joint_states 2>&1 | grep "average rate" | head -1 || echo "未发布"

echo -n "panda_1关节状态: "
timeout 2 ros2 topic hz /panda_1/panda_1_franka_state_controller/joint_states 2>&1 | grep "average rate" | head -1 || echo "未发布"

echo ""
echo "如果panda_1关节状态'未发布'，请启动topic_bridge："
echo "  ros2 run franka_example_controllers topic_bridge"
echo ""
