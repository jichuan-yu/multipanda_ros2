#!/bin/bash
# 诊断数据流：从适配器到机械臂

echo "=========================================="
echo "数据流诊断 - 从适配器到机械臂"
echo "=========================================="
echo ""

echo "1. 检查节点是否运行"
echo "------------------------------------------"
ros2 node list
echo ""

echo "2. 检查话题列表"
echo "------------------------------------------"
echo "关键话题："
ros2 topic list | grep -E "dualArm|dual_joint|pose_desired"
echo ""

echo "3. 检查轨迹话题（适配器 → 师兄控制器）"
echo "------------------------------------------"
echo "发布频率："
timeout 3 ros2 topic hz /dualArm_traj 2>/dev/null || echo "无数据"
echo ""
echo "最新消息："
timeout 2 ros2 topic echo /dualArm_traj --once 2>/dev/null || echo "无数据"
echo ""

echo "4. 检查关节命令话题（师兄控制器 → 底层控制器）"
echo "------------------------------------------"
echo "发布频率："
timeout 3 ros2 topic hz /dual_joint_impedance/joints_desired 2>/dev/null || echo "无数据"
echo ""
echo "最新消息："
timeout 2 ros2 topic echo /dual_joint_impedance/joints_desired --once 2>/dev/null || echo "无数据"
echo ""

echo "5. 检查底层控制器状态"
echo "------------------------------------------"
ros2 control list_controllers
echo ""

echo "6. 检查关节状态（硬件 → 控制器）"
echo "------------------------------------------"
ros2 topic list | grep joint_state
echo ""

echo "=========================================="
echo "诊断完成"
echo "=========================================="
