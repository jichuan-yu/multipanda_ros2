#!/bin/bash
# DualArmMprcController HQP模式切换脚本
# 用法: ./switch_hqp_mode.sh [true|false]

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查参数
if [ "$#" -ne 1 ]; then
    echo -e "${RED}错误: 需要一个参数${NC}"
    echo "用法: $0 [true|false]"
    echo "  true  - 启用HQP模式"
    echo "  false - 使用零空间模式"
    exit 1
fi

USE_HQP=$1
CONFIG_FILE="/home/xiaozy24/dual_panda_ws/src/multipanda_ros2/franka_example_controllers/config/dualarm_mprc_controller.yaml"

# 验证参数
if [ "$USE_HQP" != "true" ] && [ "$USE_HQP" != "false" ]; then
    echo -e "${RED}错误: 参数必须是 'true' 或 'false'${NC}"
    exit 1
fi

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  DualArmMprcController 模式切换${NC}"
echo -e "${YELLOW}========================================${NC}"

# 1. 检查控制器是否在运行
echo -e "${YELLOW}步骤1: 检查控制器状态...${NC}"
if ros2 control list_controllers | grep -q "dualarm_mprc_controller.*active"; then
    CONTROLLER_RUNNING=true
    echo -e "${GREEN}✓ 控制器正在运行${NC}"
else
    CONTROLLER_RUNNING=false
    echo -e "${YELLOW}⚠ 控制器未运行或未激活${NC}"
fi

# 2. 备份原配置文件
echo -e "${YELLOW}步骤2: 备份配置文件...${NC}"
BACKUP_FILE="${CONFIG_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
cp "$CONFIG_FILE" "$BACKUP_FILE"
echo -e "${GREEN}✓ 配置已备份到: $BACKUP_FILE${NC}"

# 3. 修改配置文件
echo -e "${YELLOW}步骤3: 修改配置文件...${NC}"
if [ "$USE_HQP" = "true" ]; then
    sed -i 's/use_hqp: false/use_hqp: true/' "$CONFIG_FILE"
    echo -e "${GREEN}✓ 已启用HQP模式${NC}"
else
    sed -i 's/use_hqp: true/use_hqp: false/' "$CONFIG_FILE"
    echo -e "${GREEN}✓ 已切换到零空间模式${NC}"
fi

# 4. 如果控制器正在运行，重启它
if [ "$CONTROLLER_RUNNING" = true ]; then
    echo -e "${YELLOW}步骤4: 重启控制器...${NC}"

    # 卸载控制器
    echo -e "${YELLOW}  - 卸载控制器...${NC}"
    ros2 control unload_controller dualarm_mprc_controller

    # 等待一秒
    sleep 1

    # 重新加载控制器
    echo -e "${YELLOW}  - 重新加载控制器...${NC}"
    ros2 control load_controller dualarm_mprc_controller

    # 启动控制器
    echo -e "${YELLOW}  - 启动控制器...${NC}"
    ros2 control set_controller_state dualarm_mprc_controller active

    echo -e "${GREEN}✓ 控制器已重启${NC}"
fi

# 5. 验证新配置
echo -e "${YELLOW}步骤5: 验证新配置...${NC}"
if grep -q "use_hqp: $USE_HQP" "$CONFIG_FILE"; then
    echo -e "${GREEN}✓ 配置文件已正确修改${NC}"
else
    echo -e "${RED}✗ 配置文件修改失败${NC}"
    # 恢复备份
    cp "$BACKUP_FILE" "$CONFIG_FILE"
    echo -e "${YELLOW}⚠ 已从备份恢复配置${NC}"
    exit 1
fi

echo -e "${YELLOW}========================================${NC}"
if [ "$USE_HQP" = "true" ]; then
    echo -e "${GREEN}✓ 成功切换到HQP模式${NC}"
    echo -e "${YELLOW}特点: 层次化约束、高级安全控制${NC}"
else
    echo -e "${GREEN}✓ 成功切换到零空间模式${NC}"
    echo -e "${YELLOW}特点: 计算快速、稳定可靠${NC}"
fi
echo -e "${YELLOW}========================================${NC}"

# 提示用户可以开始测试
echo ""
echo -e "${YELLOW}提示:${NC}"
echo "  - 控制器已准备就绪"
echo "  - 可以开始键盘控制测试"
echo "  - 使用性能监控脚本查看效果："
echo "    python3 scripts/performance_monitor.py"
