# MPRC 碰撞模型可视化指南

为了观察 MPRC 控制器在实时避障时具体探测到的碰撞体（27 个包围球），我们已经在代码中增加了将这些球作为 ROS 2 Marker 发布的功能。

## 1. 启动仿真与控制

按照常规步骤启动 MuJoCo 仿真和 MPRC 节点：

```bash
# 终端 1: 启动仿真
./run sim

# 终端 2: 启动 MPRC 节点
ros2 run dual_arm_reactive_control dualarm_mprc_node
```

## 2. 在 RViz2 中查看

1.  在本地或容器内打开 RViz2：
    ```bash
    rviz2
    ```
2.  在 RViz2 左侧面板设置 **Global Options -> Fixed Frame** 为 `world`。
3.  点击 **Add** 按钮，选择 **By topic**。
4.  找到并添加 `/mprc/collision_spheres` 话题下的 **MarkerArray**。
5.  (可选) 如果你想看环境障碍物，添加 `/dynamic_obstacle_markers` 话题。

## 3. 可视化说明

*   **绿色球体**：代表左臂 (Panda 1) 的实时碰撞包围球。
*   **黄色球体**：代表右臂 (Panda 2) 的实时碰撞包围球。
*   **红色球体**：代表检测到的环境动态障碍物。

通过这种方式，你可以直观地看到 MPRC 算法是如何在包围球进入障碍物影响区域时产生排斥力防止碰撞的。
