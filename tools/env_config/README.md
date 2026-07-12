# Environment Config Convention

## Overview

This directory contains YAML configuration files for defining collision environments. Each config file defines a set of static obstacles that will be generated into both MuJoCo XML and MPRC YAML formats.

---

## Shape Conventions

### Box (长方体)

```yaml
- id: "box_01"
  type: "Box"
  dimensions: [length, width, height]  # 沿XYZ轴的尺寸 (米)
  pose:
    position: [x, y, z]                # 中心位置 (米)
    orientation: [x, y, z, w]           # 四元数 (可选，默认 [0,0,0,1])
  material:                            # 可选
    friction: [2, 0.005, 0.0001]      # 滑动/扭转/滚动摩擦
    rgba: [1, 0, 0, 1]                 # 颜色 (R,G,B,A)
```

**MuJoCo Note:** `size` = [length/2, width/2, height/2]

---

### Cylinder (圆柱)

```yaml
- id: "cylinder_01"
  type: "Cylinder"
  dimensions: [radius, height]         # 半径和高度 (米)
  pose:
    position: [x, y, z]                # 底面中心位置 (米)
    orientation: [x, y, z, w]           # 四元数 (可选，默认 [0,0,0,1])
  material:                            # 可选
    friction: [2, 0.005, 0.0001]
    rgba: [0, 1, 0, 1]
```

**MuJoCo Note:** `size` = [radius, height/2], 默认Z轴朝上

---

### Sphere (球体)

```yaml
- id: "sphere_01"
  type: "Sphere"
  dimensions: [radius]                  # 半径 (米)
  pose:
    position: [x, y, z]                # 球心位置 (米)
    orientation: [x, y, z, w]           # 球体忽略方向，但保留字段
  material:                            # 可选
    friction: [2, 0.005, 0.0001]
    rgba: [0, 0, 1, 1]
```

**MuJoCo Note:** `size` = [radius, radius, radius]

---

## Full Config Example

```yaml
# my_environment.yaml
# 环境描述（可选）
description: "Simple test environment with a table and some obstacles"

# 碰撞物体列表
collision_objects:
  # 工作台面
  - id: "table_surface"
    type: "Box"
    dimensions: [0.6, 0.8, 0.05]      # 60cm x 80cm x 5cm
    pose:
      position: [0.4, 0.0, 0.0]      # 中心位置
      orientation: [0, 0, 0, 1]

  # 圆柱障碍物
  - id: "pillar_01"
    type: "Cylinder"
    dimensions: [0.05, 0.4]          # r=5cm, h=40cm
    pose:
      position: [0.5, 0.2, 0.2]      # 底面中心
      orientation: [0, 0, 0, 1]
    material:
      rgba: [0.2, 0.4, 0.8, 1]

  # 球体障碍物
  - id: "ball_01"
    type: "Sphere"
    dimensions: [0.08]               # r=8cm
    pose:
      position: [0.6, -0.2, 0.08]    # 球心位置
    material:
      rgba: [1, 0.5, 0, 1]

  # 方块
  - id: "box_01"
    type: "Box"
    dimensions: [0.1, 0.1, 0.1]
    pose:
      position: [0.5, 0.0, 0.05]
      orientation: [0, 0, 0, 1]
```

---

## Usage

```bash
# 从工作空间根目录运行 (推荐)
python3 src/multipanda_ros2/tools/env_generator.py --config env_config/my_environment.yaml

# 从 tools 目录运行
cd src/multipanda_ros2/tools
python3 env_generator.py --config env_config/my_environment.yaml

# 自定义输出路径
python3 src/multipanda_ros2/tools/env_generator.py \
    --config env_config/my_environment.yaml \
    --xml-output /path/to/objects.xml \
    --yaml-output /path/to/collision_env_sim.yaml

# 预览 (不写入文件)
python3 src/multipanda_ros2/tools/env_generator.py --config env_config/my_environment.yaml --dry-run
```

---

## Default Output Paths

- **MuJoCo XML:** `src/multipanda_ros2/franka_description/mujoco/franka/objects.xml`
- **MPRC YAML:** `src/dualarm_mprc/dualarm_reactive_control/config/collision_env_my_task.yaml`

---

## Coordinate System

- **X**: Forward direction (away from robot base)
- **Y**: Left/Right direction
- **Z**: Upward direction
- **Origin**: Robot base position

All positions are in meters (m).
