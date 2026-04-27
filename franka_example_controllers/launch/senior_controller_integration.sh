#!/bin/bash
# 师兄HQP控制器集成启动脚本
#
# 此脚本启动完整的控制系统，包括：
# 1. 适配器节点（笛卡尔→轨迹转换）
# 2. 师兄的HQP安全控制器
# 3. 底层控制器
# 4. 键盘控制

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  师兄HQP控制器集成系统${NC}"
echo -e "${BLUE}========================================${NC}"

WORKSPACE="/home/xiaozy24/dual_panda_ws"

# 检查工作空间
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}错误: 工作空间不存在: $WORKSPACE${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 工作空间: $WORKSPACE${NC}"

# ========================================
# 终端1: 仿真环境 (假设已启动)
# ========================================
echo -e "${YELLOW}步骤1: 检查仿真环境...${NC}"
if ! docker ps | grep -q multipanda-container; then
    echo -e "${RED}警告: Docker容器未运行${NC}"
    echo -e "${YELLOW}请先启动仿真环境${NC}"
else
    echo -e "${GREEN}✓ Docker容器正在运行${NC}"
fi

# ========================================
# 终端2: 底层控制器 (假设已启动)
# ========================================
echo -e "${YELLOW}步骤2: 检查底层控制器...${NC}"
echo -e "${YELLOW}请确保 dual_joint_impedance_controller 已启动${NC}"
echo -e "${YELLOW}如果没有，请在Docker容器中运行:${NC}"
echo -e "${BLUE}  ros2 control load_controller dual_joint_impedance_controller${NC}"
echo -e "${BLUE}  ros2 control set_controller_state dual_joint_impedance_controller active${NC}"

# ========================================
# 终端3: 师兄的HQP控制器
# ========================================
echo -e "${YELLOW}步骤3: 启动师兄的HQP控制器...${NC}"
echo -e "${YELLOW}请在Docker容器中运行以下命令:${NC}"
echo -e "${BLUE}  cd $WORKSPACE${NC}"
echo -e "${BLUE}  source install/setup.bash${NC}"
echo -e "${BLUE}  ros2 run dual_arm_reactive_control dualarm_mprc_node${NC}"

# ========================================
# 终端4: 适配器节点
# ========================================
echo -e "${YELLOW}步骤4: 启动适配器节点...${NC}"
echo -e "${YELLOW}在Docker容器中运行:${NC}"
echo -e "${BLUE}  cd $WORKSPACE${NC}"
echo -e "${BLUE}  source install/setup.bash${NC}"
echo -e "${BLUE}  ros2 run franka_example_controllers cartesian2traj_adapter${NC}"

# ========================================
# 终端5: 键盘控制 (宿主机)
# ========================================
echo -e "${YELLOW}步骤5: 启动键盘控制...${NC}"
echo -e "${YELLOW}在宿主机中运行:${NC}"
echo -e "${BLUE}  cd ~/dual_panda_ws${NC}"
echo -e "${BLUE}  source myenv/bin/activate${NC}"
echo -e "${BLUE}  python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py${NC}"

# ========================================
# 提供快速启动选项
# ========================================
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  快速启动选项${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "选择启动方式:"
echo "1) 手动启动 (推荐 - 可以查看每个终端的输出)"
echo "2) 自动启动适配器节点"
echo "3) 仅显示启动命令"
echo ""
read -p "请选择 (1-3): " choice

case $choice in
    1)
        echo -e "${GREEN}已选择手动启动模式${NC}"
        echo -e "${YELLOW}请按照上面的步骤依次在各个终端中启动${NC}"
        ;;
    2)
        echo -e "${YELLOW}正在启动适配器节点...${NC}"
        docker exec -it multipanda-container bash -c "
            cd $WORKSPACE &&
            source install/setup.bash &&
            ros2 run franka_example_controllers cartesian2traj_adapter
        "
        ;;
    3)
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}  完整启动命令${NC}"
        echo -e "${BLUE}========================================${NC}"
        echo ""
        echo -e "${GREEN}终端3 - 师兄HQP控制器 (Docker容器):${NC}"
        echo "docker exec -it multipanda-container bash"
        "cd $WORKSPACE && source install/setup.bash &&"
        "ros2 run dual_arm_reactive_control dualarm_mprc_node"
        echo ""
        echo -e "${GREEN}终端4 - 适配器节点 (Docker容器):${NC}"
        echo "docker exec -it multipanda-container bash"
        "cd $WORKSPACE && source install/setup.bash &&"
        "ros2 run franka_example_controllers cartesian2traj_adapter"
        echo ""
        echo -e "${GREEN}终端5 - 键盘控制 (宿主机):${NC}"
        echo "cd ~/dual_panda_ws && source myenv/bin/activate &&"
        "python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py"
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  系统信息${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "话题流:"
echo -e "  ${BLUE}key_safe_pub.py${NC} → /dualarm_mprc/pose_desired"
echo -e "  ${BLUE}cartesian2traj_adapter${NC} → 转换 → /dualArm_traj"
echo -e "  ${BLUE}dual_arm_safe_controller_sim${NC} → HQP控制 → /dual_joint_impedance/joints_desired"
echo -e "  ${BLUE}dual_joint_impedance_controller${NC} → 机器人执行"
echo ""
echo -e "测试命令:"
echo -e "  # 查看话题列表"
echo -e "  ros2 topic list"
echo ""
echo -e "  # 监控笛卡尔输入"
echo -e "  ros2 topic echo /dualarm_mprc/pose_desired"
echo ""
echo -e "  # 监控轨迹输出"
echo -e "  ros2 topic echo /dualArm_traj"
echo ""
echo -e "  # 监控关节命令"
echo -e "  ros2 topic echo /dual_joint_impedance/joints_desired"
echo ""
echo -e "${GREEN}========================================${NC}"
