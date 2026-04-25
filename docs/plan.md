# 组件 B (DualArmMprcController) 升级计划：由势场梯度模型进化为 HQP-CBF 模型

该计划旨在提升组件 B 的安全性与避障能力，使其达到组件 A (dual_arm_reactive_control) 的控制水平，同时保持其作为 controller_interface 插件的易用性。

## 第一阶段：模型描述统一 (P1 - Geometric Modeling)
**目标**：将机器人从“质点模型”升级为“27轴向包围球模型”。
- [ ] **代码迁移**：将 `PandaRobot` 类从组件 A 迁移/重构至组件 B 的包围空间中。
- [ ] **配置加载**：在 `on_configure` 阶段加载 `panda_collision_spheres_nohand.yaml` 配置文件。
- [ ] **运动学关联**：建立包围球球心位置、半径与当前关节状态 `q` 的实时映射。
- [ ] **可视化同步**：更新 `/mprc/collision_spheres` 话题，在 RViz 中显示 27 个包围球而非 16 个点。

## 第二阶段：环境感知接入 (P2 - Environment Awareness)
**目标**：使控制器具备感知动态障碍物与静态环境的能力。
- [ ] **依赖注入**：在 `CMakeLists.txt` 中引入 `fcl` (Flexible Collision Library) 与 `yaml-cpp`。
- [ ] **感知引擎**：在控制器中实例化 `CollisionEnv` 类。
- [ ] **话题对接**：订阅 `/dynamic_obstacle` 话题，将环境障碍物信息同步至 FCL 碰撞场景。

## 第三阶段：控制引擎重构 (P3 - Control Engine Overhaul)
**目标**：将基于零空间的势场法替换为层次化二次规划 (HQP) 优化的控制障碍函数 (CBF)。
- [ ] **求解器集成**：引入 `OsqpEigen` 求解器并配置优化矩阵 (H, f)。
- [ ] **硬约束构建 (Level 0)**：
    - 实现关节限位硬约束。
    - 将碰撞距离转化为 **CBF 约束**：针对 27 个包围球中的每一个，建立 `dq` 的不等式约束，确保距离始终大于安全阈值。
- [ ] **任务层构建 (Level 1)**：
    - 将来自 `key_safe_pub` 的指令转化为末端位姿追踪的目标函数。
- [ ] **求解逻辑**：在 `update()` 循环中调用 QP 求解器，计算最优的 `q_desired`。

## 第四阶段：稳健性与性能优化 (P4 - Robustness & Optimization)
**目标**：确保在控制器高频 (1kHz) 下的实时可见性。
- [ ] **性能分析**：测量单次 `update` 中 QP 求解与 FCL 测距的耗时。
- [ ] **平滑处理**：对求解出的 `q_desired` 进行加速度滤波或限幅，防止输出震荡。

---

**执行注意事项**：
1. 保持 `ros2_control` 接口的兼容性，确保输出话题 `/dual_joint_impedance/joints_desired` 格式不变。
2. 配置文件路径需处理为相对于工作空间或包路径的绝对路径。
