#!/bin/bash
# 正确启动师兄控制器 - 使用launch文件加载参数

echo "=========================================="
echo "启动师兄控制器（MuJoCo参数匹配）"
echo "=========================================="
echo ""

# 检查师兄控制器是否已经在运行
if ros2 node list 2>/dev/null | grep -q "dual_arm_mprc_node"; then
    echo "❌ 师兄控制器已在运行，请先关闭："
    echo "   ros2 node kill /dual_arm_mprc_node"
    exit 1
fi

echo "使用参数文件启动..."
echo "  panda_1 base: (0.0, 0.26, 0.0)"
echo "  panda_2 base: (0.0, -0.26, 0.0)"
echo ""

# 使用launch文件启动（会自动加载参数）
ros2 launch franka_example_controllers senior_controller.launch.py

# 如果launch失败，回退到直接启动并手动提示
if [ $? -ne 0 ]; then
    echo ""
    echo "=========================================="
    echo "Launch文件失败，使用回退方法"
    echo "=========================================="
    echo ""
    echo "请手动设置参数后启动："
    echo ""
    echo "  # 在启动前设置参数"
    echo "  export ROS_DOMAIN_ID=0"
    echo "  ros2 param load /dual_arm_mprc_node \\"
    echo "    /home/xiaozy24/dual_panda_ws/src/multipanda_ros2/franka_example_controllers/config/senior_controller_params.yaml"
    echo ""
    echo "  # 然后启动"
    echo "  ros2 run dual_arm_reactive_control dual_arm_mprc_node"
    echo ""
fi
