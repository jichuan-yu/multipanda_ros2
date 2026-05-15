# 双臂机器人安全控制与碰撞环境配置指南

本文档总结了系统中 MuJoCo 仿真、URDF 模型与 MPRC 安全控制器之间碰撞环境的对应关系及配置逻辑。

## 1. 核心架构与文件关系

系统的碰撞检测分为三个独立但必须保持同步的部分：**MuJoCo 物理环境**（真实碰撞）、**MPRC 控制器计算环境**（安全约束）以及 **RViz 可视化标记**（调试参考）。

| 组成部分 | 负责内容 | 核心配置文件 |
| :--- | :--- | :--- |
| **MuJoCo 仿真** | 物理实体、零件 STL、接触力计算 | `objects.xml`, `my_task_scene_dynamic.xml` |
| **机器人本体** | 机械臂 7 关节 + Hand 的包络球定义 | `panda_collision_spheres.yaml` |
| **外部环境** | 静态障碍物的几何定义与位姿 | `collision_env_my_task.yaml` |
| **安全可视化** | 实时碰撞对距离、分级颜色提醒、最近点连线 | `collision_env_visualizer.cpp` |
| **逻辑骨架** | 定义链接前缀、基座偏移、手爪挂载 | `dual_panda_arm_sim.urdf.xacro` |

---

## 2. 机器人本体碰撞模型 (MPRC)

为了实现高频实时避障，系统不使用复杂的网格碰撞，而是使用 **包络球 (Bounding Spheres)** 覆盖机械臂：

- **带手版本 (With Hand)**: 使用 `panda_collision_spheres.yaml`。
  - 总计 **30个碰撞球**。
  - 末端 `panda_hand` 链接额外包含 3 个球来保护手爪区域。
- **不带手版本 (No Hand)**: 使用 `panda_collision_spheres_nohand.yaml`。
  - 总计 **27个碰撞球**。
  - 仅覆盖到 `link7`。

**关键点**：控制器通过 `PandaRobot` 类的运动学模型计算这些球在世界系下的位置。

---

## 3. 自动化生成器

### 3.1 概述

手动配置碰撞环境繁琐且容易出错。系统提供了 **自动化生成器** `generate_collision_env.py`，可自动生成随机非重叠的柱状障碍物配置，并同步更新 MuJoCo XML 和 MPRC YAML 文件。

**位置**: `src/multipanda_ros2/tools/generate_collision_env.py`

### 3.2 使用方法

```bash
# 进入工作目录
cd /home/xiaozy24/dual_panda_ws

# 生成默认配置（12个柱体：10个小柱 + 2个大柱）
python3 src/multipanda_ros2/tools/generate_collision_env.py

# 使用指定种子生成（可复现）
python3 src/multipanda_ros2/tools/generate_collision_env.py --seed 42

# 预览配置而不写入文件
python3 src/multipanda_ros2/tools/generate_collision_env.py --dry-run

# 自定义安全间距
python3 src/multipanda_ros2/tools/generate_collision_env.py --margin 0.02
```

### 3.3 当前区域配置

生成器使用 **6 个预定义区域** 来放置障碍物：

| 区域 | X 范围 (m) | Y 范围 (m) | 柱体数量 |
|------|------------|------------|----------|
| Zone1_center | [0.2, 0.4] | [-0.05, 0.05] | 1 小柱 |
| Zone2_left_rear | [0.2, 0.4] | [-0.45, -0.4] | 1 小柱 |
| Zone3_left_front | [0.2, 0.4] | [0.4, 0.45] | 1 小柱 |
| Zone4_middle | [0.4, 0.6] | [-0.4, 0.4] | 3 小柱 + 1 大柱 |
| Zone5_right_middle | [0.6, 0.75] | [-0.3, 0.3] | 2 小柱 + 1 大柱 |
| Zone6_right_front | [0.75, 0.85] | [-0.2, 0.2] | 2 小柱 |

**总计**: 10 小柱 (0.05×0.05m) + 2 大柱 (0.1×0.1m) = 12 个柱体

- **小柱**: 尺寸 0.05×0.05m，高度随机选择 {0.3, 0.4, 0.5, 0.6, 0.7}m
- **大柱**: 尺寸 0.1×0.1m，固定高度 0.2m
- **坐标精度**: 所有 X、Y 坐标为 0.01 的整数倍
- **截面类型**: 每个柱体随机选择 **Box (矩形)** 或 **Cylinder (圆形)** 截面，概率各 50%
  - 圆形截面直径 = min(原长, 原宽)，即小柱直径 0.05m，大柱直径 0.1m
  - 重叠检测统一按矩形截面进行，安全条件自然满足

### 3.4 自定义生成参数

如需修改生成参数（区域范围、柱体数量、尺寸等），请编辑生成器源码：

**关键修改位置**:

1. **修改区域定义** (约第 87-100 行):
   ```python
   ZONES = [
       # 格式: Zone(x_min, x_max, y_min, y_max, num_small, num_large, name)
       Zone(0.2, 0.4, -0.05, 0.05, 1, 0, "Zone1_center"),
       # 添加或修改区域...
   ]
   ```

2. **修改柱体尺寸** (约第 76-84 行):
   ```python
   SMALL_SIZE = 0.05      # 小柱边长
   LARGE_SIZE = 0.1       # 大柱边长
   SMALL_HEIGHTS = [0.3, 0.4, 0.5, 0.6, 0.7]  # 小柱高度选项
   LARGE_HEIGHT = 0.2     # 大柱固定高度
   ```

3. **修改安全间距** (约第 102 行):
   ```python
   def __init__(self, seed: int = None, margin: float = 0.01, ...):
   ```

### 3.5 生成器输出

运行生成器后会自动更新两个文件：

| 文件 | 路径 |
|------|------|
| MuJoCo XML | `src/multipanda_ros2/franka_description/mujoco/franka/objects.xml` |
| MPRC YAML | `src/dualarm_mprc/dualarm_reactive_control/config/collision_env_my_task.yaml` |

---

## 4. 环境障碍物映射逻辑

### 4.1 静态柱体环境（当前配置）

当前环境使用 **12 个静态柱体** 固定在地面上，每个柱体随机选择 **矩形 (Box)** 或 **圆形 (Cylinder)** 截面。

#### 矩形柱体 (Box)

**MuJoCo (XML)**:
```xml
<geom name="pillar_01" type="box" size="0.025 0.025 0.3" pos="0.2 0.0 0.3"
      friction="2 0.005 0.0001" solimp="0.998 0.998 0.001" solref="0.001 1"
      rgba="0.2 0.4 0.8 1"/>
```
- `size`: 半长宽高，即 `[length/2, width/2, height/2]`

**MPRC Config (YAML)**:
```yaml
- id: "pillar_01"
  type: "Box"
  dimensions: [0.05, 0.05, 0.6]  # 全长、全宽、全高
  pose:
    position: [0.2, 0.0, 0.3]     # X, Y, height/2
    orientation: [0.0, 0.0, 0.0, 1.0]
```

#### 圆形柱体 (Cylinder)

**MuJoCo (XML)**:
```xml
<geom name="pillar_02" type="cylinder" size="0.025 0.15" pos="0.3 0.0 0.15"
      friction="2 0.005 0.0001" solimp="0.998 0.998 0.001" solref="0.001 1"
      rgba="0.2 0.4 0.8 1"/>
```
- `size`: `[radius, height/2]`，即 `[直径/2, 高度/2]`
- 对于 0.05×0.05m 的小柱：`size="0.025 0.15"` (半径 0.025m)
- 对于 0.1×0.1m 的大柱：`size="0.05 0.1"` (半径 0.05m)

**MPRC Config (YAML)**:
```yaml
- id: "pillar_02"
  type: "Cylinder"
  dimensions: [0.05, 0.4]  # 直径、高度
  pose:
    position: [0.3, 0.0, 0.2]     # X, Y, height/2
    orientation: [0.0, 0.0, 0.0, 1.0]
```

**注意**：
- 静态障碍物使用 `<geom>` 直接放置在 worldbody 中，无需 `<body>` 和 `<joint>`
- 圆形柱体的 `radius = min(length, width) / 2`
- 重叠检测统一按矩形截面进行
- 柱体中心 Z 坐标 = 柱体高度 / 2

### 4.2 动态障碍物（已弃用，保留参考）

旧版配置使用动态球体障碍物，现已替换为静态柱体。如需恢复动态障碍物，请参考：

**MuJoCo (XML)**:
```xml
<body name="dyn_sphere_1" pos="0.2 0.0 0.6" gravcomp="1">
    <freejoint name="dyn_sphere_1_joint"/>
    <geom name="dyn_sphere_1" type="sphere" size="0.05" rgba="1 0 0 1" mass="0.1"/>
</body>
```

**MPRC Config (YAML)**:
```yaml
- id: "dyn_sphere_1"
  type: "Sphere"
  dimensions: [0.05]
  pose:
    position: [0.2, 0.0, 0.6]
    orientation: [0.0, 0.0, 0.0, 1.0]
```

---

## 5. 双臂系统的几何对齐

在双臂系统中，基座的平移位置对避障计算至关重要：

- **URDF 逻辑配置**:
  - 左臂 (`mj_left`): `xyz="0 +0.26 0"`
  - 右臂 (`mj_right`): `xyz="0 -0.26 0"`
- **可视化/计算节点注入**:
  在启动 `collision_env_visualizer_node` 时，必须传入 `robot1_xyz` 和 `robot2_xyz` 参数，确保计算碰撞球位置时，起始原点与 URDF 保持一致。

---

## 6. 常见调试问题

- **虚假障碍物 (Phantom Obstacle)**: 如果在 MuJoCo 中删除了物体但未修改 `collision_env_my_task.yaml`，机器人会避让一个看不见的"空气墙"。
- **碰撞球重叠**: 如果日志显示 `Setting Robot Base to: 0, 0, 0`，说明双臂计算基座未正确加载参数，会导致避障逻辑完全错乱。
- **零件渲染但无避障**: 如果在 MuJoCo 里通过 `objects.xml` 添加了物体，但没有在 YAML 里注册，机器人会直接穿模撞向物体。
- **柱体高度不匹配**: 如果 MuJoCo 中柱体看起来被截断或悬浮，检查 `pos` 的 Z 值是否等于 `height/2`。

---

## 7. 维护规范

每次修改仿真场景或机器人构型（如更换手爪）后，请按以下顺序执行：

1. **使用生成器或手动修改** `src/` 下对应的 `.xml`, `.yaml`
2. **编译**:
   ```bash
   docker exec -it multipanda-container bash
   cd /home/xiaozy24/dual_panda_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select franka_description dual_arm_reactive_control
   ```
3. **测试**: 通过 `ros2 launch` 启动仿真，并在 RViz 中确认 `collision_env_markers` 话题的柱体显示情况

---

## 8. 碰撞安全可视化 (RViz)

为了方便调试和监控，系统实现了动态分级可视化逻辑。

### 8.1 通信机制
- **消息类型**: 使用自定义消息 `CollisionPairArray` 传输计算出的碰撞数据。
- **发布节点**: 安全控制器 (`main_sim_node`) 在控制循环中实时发布。
- **订阅节点**: 可视化节点 (`collision_env_visualizer_node`) 订阅该话题并更新 RViz Marker。

### 8.2 分级显示逻辑
当碰撞体（障碍物或机器人球体）之间的距离 $d$ 缩短时，渲染状态会发生变化：

| 状态 | 距离条件 | 可视化效果 (RViz) |
| :--- | :--- | :--- |
| **安全 (Safe)** | $d > d_{active}$ | 默认颜色（障碍物黄色，球体绿/蓝），透明度 $A=0.4$ |
| **预警 (Warning)** | $d_{safe} < d \leq d_{active}$ | 颜色变红，透明度随距离线性增加 ($0.4 \rightarrow 1.0$)，**生成黄色引导连线** |
| **危险 (Danger)** | $d \leq d_{safe}$ | 颜色深红，完全不透明 ($A=1.0$)，**引导连线变红** |

### 8.3 最近点动态连线
- **功能**: 当距离小于 $d_{active}$ 时，系统会自动在障碍物与对应的机器人碰撞球之间绘制一条直线。
- **物理意义**: 直线连接了两者的 **最近点 (Nearest Points)**，代表了当前的碰撞梯度方向。
- **生命周期**: 连线随运动实时刷新，当物体离开活跃范围（或不再计算该对碰撞）后，连线会自动消失。

---

## 9. 配置文件位置

| 文件 | 路径 |
|------|------|
| 自动化生成器 | `src/multipanda_ros2/tools/generate_collision_env.py` |
| MuJoCo 障碍物 | `src/multipanda_ros2/franka_description/mujoco/franka/objects.xml` |
| MPRC 碰撞配置 | `src/dualarm_mprc/dualarm_reactive_control/config/collision_env_my_task.yaml` |
| 机器人碰撞球 | `src/dualarm_mprc/dualarm_reactive_control/config/panda_collision_spheres.yaml` |
