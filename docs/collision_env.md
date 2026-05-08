# 双臂机器人安全控制与碰撞环境配置指南

本文档总结了系统中 MuJoCo 仿真、URDF 模型与 MPRC 安全控制器之间碰撞环境的对应关系及配置逻辑。

## 1. 核心架构与文件关系

系统的碰撞检测分为三个独立但必须保持同步的部分：**MuJoCo 物理环境**（真实碰撞）、**MPRC 控制器计算环境**（安全约束）以及 **RViz 可视化标记**（调试参考）。

| 组成部分 | 负责内容 | 核心配置文件 |
| :--- | :--- | :--- |
| **MuJoCo 仿真** | 物理实体、零件 STL、接触力计算 | `objects.xml`, `my_task_scene_dynamic.xml` |
| **机器人本体** | 机械臂 7 关节 + Hand 的包络球定义 | `panda_collision_spheres.yaml` |
| **外部环境** | 静态障碍物的几何定义与位姿 | `collision_env_my_task.yaml` |
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

## 3. 环境障碍物映射逻辑

当在仿真中添加一个物体（如立方体）时，必须在两个地方同时定义：

1.  **MuJoCo (XML)**:
    ```xml
    <body name="obj_box_01" pos="0.5 -0.13 0.061">
        <geom type="box" size="0.03 0.03 0.03" .../>
    </body>
    ```
2.  **MPRC Config (YAML)**:
    ```yaml
    - id: "obj_box_01"
      type: "Box"
      dimensions: [0.06, 0.06, 0.06] # MuJoCo size 的两倍
      pose: { position: [0.5, -0.13, 0.061] }
    ```
**注意**：MuJoCo 的 `size` 通常是半长宽（Half-extent），而 MPRC 的 `dimensions` 是全长度。

---

## 4. 双臂系统的几何对齐

在双臂系统中，基座的平移位置对避障计算至关重要：

- **URDF 逻辑配置**:
  - 左臂 (`mj_left`): `xyz="0 +0.26 0"`
  - 右臂 (`mj_right`): `xyz="0 -0.26 0"`
- **可视化/计算节点注入**:
  在启动 `collision_env_visualizer_node` 时，必须传入 `robot1_xyz` 和 `robot2_xyz` 参数，确保计算碰撞球位置时，起始原点与 URDF 保持一致。

---

## 5. 常见调试问题

- **虚假障碍物 (Phantom Obstacle)**: 如果在 MuJoCo 中删除了物体但未修改 `collision_env_my_task.yaml`，机器人会避让一个看不见的“空气墙”。
- **碰撞球重叠**: 如果日志显示 `Setting Robot Base to: 0, 0, 0`，说明双臂计算基座未正确加载参数，会导致避障逻辑完全错乱。
- **零件渲染但无避障**: 如果在 MuJoCo 里通过 `objects.xml` 添加了物体，但没有在 YAML 里注册，机器人会直接穿模撞向物体。

---

## 6. 维护规范

每次修改仿真场景或机器人构型（如更换手爪）后，请按以下顺序执行：
1.  修改 `src/` 下对应的 `.xml`, `.yaml` 或 `.xacro`。
2.  执行 `colcon build --packages-select <相关包名>` 同步到 `install` 目录。
3.  通过 `ros2 launch` 启动，并在 RViz 中确认 `collision_env_markers` 话题的球体覆盖情况。
