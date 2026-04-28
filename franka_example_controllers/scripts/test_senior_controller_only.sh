#!/bin/bash
# 隔离测试师兄控制器 - 不使用适配器

echo "=========================================="
echo "隔离测试师兄控制器"
echo "=========================================="
echo ""

echo "步骤1：检查仿真是否运行"
node_count=$(ros2 node list 2>/dev/null | wc -l)
if [ "$node_count" -lt 3 ]; then
    echo "❌ 错误：ROS2节点数量不足（当前：$node_count），仿真可能未运行！"
    echo "请先启动：ros2 launch my_task_description my_task_sim.launch.py"
    echo ""
    echo "当前运行的节点："
    ros2 node list 2>/dev/null || echo "无节点运行"
    exit 1
fi
echo "✓ 检测到 $node_count 个ROS2节点"
echo ""

echo "步骤2：检查joint_states话题"
ros2 topic hz /joint_states --once --timeout 2 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ /joint_states 正在发布"
else
    echo "❌ /joint_states 未发布"
    exit 1
fi
echo ""

echo "步骤3：检查panda_1关节状态（通过topic_bridge）"
timeout 3 ros2 topic hz /panda_1/panda_1_franka_state_controller/joint_states > /tmp/panda1_hz.txt 2>&1
if grep -q "average rate" /tmp/panda1_hz.txt; then
    rate=$(head -1 /tmp/panda1_hz.txt | grep -oP '\d+\.\d+')
    echo "✓ /panda_1/panda_1_franka_state_controller/joint_states 正在发布"
    echo "  频率：${rate} Hz"
else
    echo "❌ panda_1关节状态未发布！topic_bridge可能未运行"
    echo ""
    echo "当前panda_1话题状态："
    ros2 topic info /panda_1/panda_1_franka_state_controller/joint_states 2>/dev/null || echo "  话题不存在"
    echo ""
    echo "请启动topic_bridge："
    echo "  ros2 run franka_example_controllers topic_bridge"
    exit 1
fi
echo ""

echo "步骤4：启动师兄控制器（10秒测试）"
echo "  如果师兄控制器正常，应该看到："
echo "  - Dual arm safe controller initialized"
echo "  - Control state is set to TRACKING"
echo "  - 不应该有HQP错误"
echo ""

# 启动师兄控制器
timeout 10 ros2 run dual_arm_reactive_control dual_arm_mprc_node 2>&1 | tee /tmp/senior_test.log

echo ""
echo "=========================================="
echo "测试结果分析"
echo "=========================================="
echo ""

if grep -q "Dual arm safe controller initialized" /tmp/senior_test.log; then
    echo "✓ 师兄控制器初始化成功"
else
    echo "❌ 师兄控制器初始化失败"
    exit 1
fi

if grep -q "HQP Error" /tmp/senior_test.log; then
    echo "❌ 师兄控制器出现HQP错误"
    echo ""
    echo "可能的根本原因："
    echo "1. 师兄控制器的关节限制配置与实际关节位置冲突"
    echo "2. PandaRobot模型参数不匹配"
    echo "3. 碰撞球参数错误"
    echo ""
    echo "建议检查："
    echo "- 师兄控制器的config/control_parameters_sim.yaml"
    echo "- 实际关节位置是否超出q_lb/q_ub范围"
    echo "- collision sphere配置"
    exit 1
else
    echo "✓ 师兄控制器无HQP错误"
    echo ""
    echo "结论：师兄控制器本身工作正常"
    echo "问题在适配器的IK求解或轨迹发布"
fi

echo ""
echo "详细日志保存在：/tmp/senior_test.log"
