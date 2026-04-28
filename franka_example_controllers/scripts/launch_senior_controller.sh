#!/bin/bash
# 正确启动师兄控制器的脚本

echo "=========================================="
echo "启动师兄控制器（带正确配置）"
echo "=========================================="
echo ""

# 检查师兄控制器是否已经在运行
if ros2 node list | grep -q "dual_arm_mprc_node"; then
    echo "❌ 师兄控制器已在运行，请先关闭："
    echo "   ros2 node kill /dual_arm_mprc_node"
    exit 1
fi

echo "步骤1：设置ROS2参数（机器人base位置）"
echo "  /panda_1_xyz: 0.0 0.0 0.0"
echo "  /panda_2_xyz: 0.0 0.0 0.0"
ros2 param set /dual_arm_mprc_node /panda_1_xyz "0.0 0.0 0.0" 2>/dev/null || echo "  (节点未启动，稍后设置)"
ros2 param set /dual_arm_mprc_node /panda_2_xyz "0.0 0.0 0.0" 2>/dev/null || echo "  (节点未启动，稍后设置)"
echo ""

echo "步骤2：检查关节状态是否可用"
if ! timeout 2 ros2 topic hz /panda_1/panda_1_franka_state_controller/joint_states > /dev/null 2>&1; then
    echo "❌ panda_1关节状态不可用"
    echo "   请确保topic_bridge正在运行"
    exit 1
fi
echo "✓ 关节状态可用"
echo ""

echo "步骤3：启动师兄控制器（后台运行）"
echo "  提示：Ctrl-C无法停止后台进程"
echo "  停止方法：ros2 node kill /dual_arm_mprc_node"
echo ""

# 启动师兄控制器
ros2 run dual_arm_reactive_control dual_arm_mprc_node &
SENIOR_PID=$!

# 等待节点启动
sleep 2

# 设置参数
if ros2 node list | grep -q "dual_arm_mprc_node"; then
    echo "✓ 师兄控制器已启动（PID: $SENIOR_PID）"

    # 设置机器人base参数
    echo "设置机器人base位置参数..."
    ros2 param set /dual_arm_mprc_node /panda_1_xyz "0.0 0.0 0.0"
    ros2 param set /dual_arm_mprc_node /panda_2_xyz "0.0 0.0 0.0"
    ros2 param set /dual_arm_mprc_node /panda_1_rpy "0.0 0.0 0.0"
    ros2 param set /dual_arm_mprc_node /panda_2_rpy "0.0 0.0 0.0"

    echo ""
    echo "=========================================="
    echo "师兄控制器运行中"
    echo "=========================================="
    echo ""
    echo "查看日志："
    echo "  ros2 node info /dual_arm_mprc_node"
    echo ""
    echo "停止控制器："
    echo "  ros2 node kill /dual_arm_mprc_node"
    echo ""
else
    echo "❌ 师兄控制器启动失败"
    exit 1
fi
