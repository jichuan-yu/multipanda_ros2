#!/bin/bash
# 师兄控制器连接诊断脚本

echo "=========================================="
echo "师兄控制器连接诊断"
echo "=========================================="
echo ""

echo "1. 检查节点是否运行："
echo "------------------------------------------"
ros2 node list | grep -E "cartesian2traj|dual_arm"
echo ""

echo "2. 检查话题列表："
echo "------------------------------------------"
ros2 topic list | grep -E "dualArm|dualarm|pose_desired"
echo ""

echo "3. 检查 /dualArm_traj 话题详情："
echo "------------------------------------------"
ros2 topic info /dualArm_traj
echo ""

echo "4. 检查适配器发布情况："
echo "------------------------------------------"
ros2 topic hz /dualArm_traj
echo ""

echo "5. 监控 /dualArm_traj 话题（5秒）："
echo "------------------------------------------"
timeout 5 ros2 topic echo /dualArm_traj --once || echo "没有收到消息"
echo ""

echo "6. 检查师兄控制器的订阅："
echo "------------------------------------------"
echo "查找 dual_arm_safe_controller_sim 节点的订阅："
ros2 node info /dual_arm_mprc_node 2>/dev/null | grep -A 20 "Subscribers"
echo ""

echo "7. 检查关节命令发布："
echo "------------------------------------------"
ros2 topic hz /dual_joint_impedance/joints_desired
echo ""

echo "8. 检查底层控制器状态："
echo "------------------------------------------"
ros2 control list_controllers
echo ""

echo "=========================================="
echo "诊断完成"
echo "=========================================="
