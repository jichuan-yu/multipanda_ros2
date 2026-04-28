#!/bin/bash
# 话题重映射脚本
# 解决师兄控制器期望的话题名称与实际系统不匹配的问题

echo "=========================================="
echo "创建话题重映射"
echo "=========================================="
echo ""

# 创建重映射配置
cat > /tmp/topic_remapping.yaml <<EOF
topic_remappings:
  # 师兄控制器订阅的话题 → 实际系统的话题
  - from: /panda_1/panda_1_franka_state_controller/joint_states
    to:   /mj_left/panda_1_franka_state_controller/joint_states
  - from: /panda_2/panda_2_franka_state_controller/joint_states
    to:   /mj_right/panda_2_franka_state_controller/joint_states

  # 师兄控制器订阅的Franka状态
  - from: /panda_1/panda_1_franka_state_controller/franka_states
    to:   /mj_left/panda_1_franka_state_controller/franka_states
  - from: /panda_2/panda_2_franka_state_controller/franka_states
    to:   /mj_right/panda_2_franka_state_controller/franka_states
EOF

echo "重映射配置已创建: /tmp/topic_remapping.yaml"
echo ""

echo "启动方法1: 使用ros2 topic relay (推荐)"
echo "------------------------------------------"
echo "# 终端A: 重映射关节状态"
echo "ros2 run ros2_topic_tools relay /panda_1/panda_1_franka_state_controller/joint_states /mj_left/panda_1_franka_state_controller/joint_states"
echo ""
echo "# 终端B: 重映射第二个关节状态"
echo "ros2 run ros2_topic_tools relay /panda_2/panda_2_franka_state_controller/joint_states /mj_right/panda_2_franka_state_controller/joint_states"
echo ""

echo "启动方法2: 使用rules_tool (自动)"
echo "------------------------------------------"
echo "# 安装工具（如果还没有）"
echo "sudo apt install ros-${ROS_DISTRO}-ros1-bridge"
echo ""
echo "# 启动重映射节点"
echo "ros2 run ros1_bridge ros1_bridge.py"
echo ""

echo "=========================================="
echo "使用说明"
echo "=========================================="
echo ""
echo "1. 保持arm_id为mj_left/mj_right启动仿真"
echo "2. 启动上面的重映射命令"
echo "3. 然后启动师兄控制器和适配器"
echo ""
echo "这样师兄控制器以为订阅panda_1，"
echo "实际收到的mj_left的数据！"
