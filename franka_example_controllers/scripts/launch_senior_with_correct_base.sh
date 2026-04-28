#!/bin/bash
# 正确启动师兄控制器 - 设置MuJoCo匹配的base位置

echo "=========================================="
echo "启动师兄控制器（MuJoCo匹配配置）"
echo "=========================================="
echo ""

# 检查师兄控制器是否已经在运行
if ros2 node list 2>/dev/null | grep -q "dual_arm_mprc_node"; then
    echo "❌ 师兄控制器已在运行，请先关闭："
    echo "   ros2 node kill /dual_arm_mprc_node"
    exit 1
fi

echo "关键配置："
echo "  MuJoCo中 mj_left:  (0, 0.26, 0)"
echo "  MuJoCo中 mj_right: (0, -0.26, 0)"
echo "  师兄控制器必须匹配这些位置！"
echo ""

echo "启动师兄控制器..."
echo "提示：使用Ctrl+C停止"
echo ""

# 在后台启动师兄控制器并设置参数
ros2 run dual_arm_reactive_control dual_arm_mprc_node &
SENIOR_PID=$!

# 等待节点启动
sleep 2

# 设置机器人base位置参数（必须匹配MuJoCo！）
echo "设置机器人base位置参数..."
if ros2 node list 2>/dev/null | grep -q "dual_arm_mprc_node"; then
    ros2 param set /dual_arm_mprc_node /panda_1_xyz "0.0 0.26 0.0"
    ros2 param set /dual_arm_mprc_node /panda_2_xyz "0.0 -0.26 0.0"
    ros2 param set /dual_arm_mprc_node /panda_1_rpy "0.0 0.0 0.0"
    ros2 param set /dual_arm_mprc_node /panda_2_rpy "0.0 0.0 0.0"

    echo ""
    echo "✓ 师兄控制器已启动（PID: $SENIOR_PID）"
    echo "✓ Base位置已设置为MuJoCo匹配值"
    echo ""
    echo "=========================================="
    echo "验证参数"
    echo "=========================================="
    ros2 param get /dual_arm_mprc_node /panda_1_xyz
    ros2 param get /dual_arm_mprc_node /panda_2_xyz
    echo ""
else
    echo "❌ 师兄控制器启动失败"
    kill $SENIOR_PID 2>/dev/null
    exit 1
fi
