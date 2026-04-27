#!/bin/bash
# DualArmMprcController Launch Script
# This script launches the DualArmMprcController with ros2_control

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  DualArmMprcController Launch Script${NC}"
echo -e "${GREEN}========================================${NC}"

# Workspace setup
WORKSPACE="/home/xiaozy24/dual_panda_ws"
echo -e "${YELLOW}Workspace: ${WORKSPACE}${NC}"

# Source workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
cd "$WORKSPACE"
source install/setup.bash

# Set controller parameters
echo -e "${YELLOW}Setting controller parameters...${NC}"

# Default: use null-space control (backwards compatible)
USE_HQP=${USE_HQP:-false}

echo -e "${YELLOW}Control mode: $([ "$USE_HQP" = "true" ] && echo "HQP (Advanced)" || echo "Null-Space (Standard)")${NC}"

# Launch controller with ros2_control
echo -e "${YELLOW}Starting DualArmMprcController...${NC}"

ros2 run franka_example_controllers franka_example_controllers \
  --ros-args \
  --params-file "$WORKSPACE/src/multipanda_ros2/franka_example_controllers/config/dualarm_mprc_controller.yaml" \
  -p use_hqp:=$USE_HQP

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Controller launched successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
