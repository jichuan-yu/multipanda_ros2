# 师兄控制器OSQP "Non Convex"错误修复指南

## 问题诊断总结

**根本原因**：ALLCBF碰撞避免约束在初始位置被激活，导致OSQP求解器认为问题非凸。

**分析依据**：
- OSQP错误：`q: 1.51068e+210`（极端数值）
- 约束类型：`COLLISION_AVOIDANCE_ALLCBF`
- 碰撞激活距离：`d_active_ = 0.3m`
- 碰撞安全距离：`d_safe_ = 0.1m`
- 碰撞增益：`K_collision_avoidance_ = 5`（很高！）

## 解决方案（按优先级）

### 方案1：降低碰撞增益（推荐，不重新编译）

**在Docker容器中执行**：

```bash
# 进入容器
docker exec -it xiaozy24_dev bash

# 进入工作空间
cd /home/xiaozy24/dual_panda_ws
source install/setup.bash

# 运行配置修改脚本
python3 src/multipanda_ros2/franka_example_controllers/scripts/fix_collision_config.py
```

**修改内容**：
- `K_collision_avoidance: 5 → 1`（降低碰撞避免严格性）
- `K_joint_limit: 5.0 → 2.0`（降低关节限制严格性）

**重启测试**：
```bash
# 关闭旧控制器
ros2 node kill /dual_arm_mprc_node

# 重新启动
ros2 run dual_arm_reactive_control dual_arm_mprc_node

# 新终端：发送测试轨迹
python3 src/multipanda_ros2/franka_example_controllers/scripts/test_safe_joint_position.py
```

**预期结果**：
- ✅ 师兄控制器正常运行，无OSQP错误
- ✅ 开始发布关节命令
- ✅ 可以看到机械臂响应

---

### 方案2：禁用碰撞避免约束（需要重新编译）

如果方案1仍有问题，完全禁用碰撞避免约束：

**编辑源码**：
```bash
# 在Docker容器中
cd /home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/src

# 编辑文件
nano dual_arm_safe_controller_sim.cpp
```

**找到约166行**：
```cpp
constraint_manager_.addConstraint(Constraint(3, ConstraintType::COLLISION_AVOIDANCE_ALLCBF));
```

**改为**：
```cpp
// constraint_manager_.addConstraint(Constraint(3, ConstraintType::COLLISION_AVOIDANCE_ALLCBF));
```

**重新编译**：
```bash
cd /home/xiaozy24/dual_panda_ws/src/dualarm_mprc
colcon build --packages-select dual_arm_reactive_control
source install/setup.bash
```

**测试**：同方案1

---

### 方案3：调整初始位置（避免关节4接近极限）

**问题**：关节4 = -2.356，接近下界-3.07

**解决**：使用test_safe_joint_position.py（已修改为安全位置）

```bash
python3 src/multipanda_ros2/franka_example_controllers/scripts/test_safe_joint_position.py
```

新的安全位置：
```
关节4: -1.5（中间位置，远离极限）
其他关节也调整到中心位置
```

---

## 诊断脚本使用

### 查看约束添加位置
```bash
python3 src/multipanda_ros2/franka_example_controllers/scripts/test_senior_no_collision.py
```

这会显示：
- 师兄控制器中约束添加的代码位置
- 如何禁用碰撞避免约束
- 当前碰撞配置检查

---

## 验证步骤

### 1. 检查师兄控制器是否启动
```bash
ros2 node list | grep dual_arm_mprc
```

### 2. 检查是否收到轨迹
```bash
# 师兄控制器终端应该看到：
# New trajectory set. Traj length: 1
# Control state is set to Control State: TRACKING
```

### 3. 检查是否发布关节命令
```bash
ros2 topic hz /dual_joint_impedance/joints_desired
# 应该看到约100 Hz
```

### 4. 检查是否有OSQP错误
```bash
# 师兄控制器终端不应该有：
# [ERROR] HQP safe controller failed to find a solution
# OSQP ERROR: Non Convex
```

---

## 预期最终结果

**成功标志**：
```
[INFO] Dual arm safe controller initialized
[INFO] Dual Arm Reactive Control Node Started
New trajectory set. Traj length: 1
Control state is set to Control State: TRACKING
(无OSQP错误)
```

**师兄控制器开始发布命令**：
```bash
ros2 topic hz /dual_joint_impedance/joints_desired
# average rate: 100.2
```

---

## 如果还不工作

### 检查碰撞球配置
```bash
ls -la /home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/panda_collision_spheres.yaml
```

如果不存在，师兄控制器可能无法初始化碰撞环境。

### 检查PandaRobot初始化
查看师兄控制器日志中是否有关于DH参数或碰撞球的错误信息。

### 完全禁用约束（临时测试）
在dual_arm_safe_controller_sim.cpp中注释掉所有约束添加：
```cpp
// constraint_manager_.addConstraint(Constraint(0, ConstraintType::JOINT_LIMIT_WITH_ACC));
// constraint_manager_.addConstraint(Constraint(0, ConstraintType::SINGULARITY_AVOIDANCE));
// constraint_manager_.addConstraint(Constraint(3, ConstraintType::COLLISION_AVOIDANCE_ALLCBF));
```

如果这样能工作，说明确实是约束问题。

---

## 恢复原配置

```bash
cp /home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/control_parameters_sim.yaml.backup \
   /home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/control_parameters_sim.yaml
```

---

## 关键代码位置

- **约束管理**: `src/dualarm_mprc/dualarm_reactive_control/src/utils/constraint_manager.cpp`
  - `joint_limit_constraint_with_acc()`: 约832行
  - `collision_avoidance_constraint_ALLCBF()`: 约1045行

- **主控制器**: `src/dualarm_mprc/dualarm_reactive_control/src/dual_arm_safe_controller_sim.cpp`
  - 约束添加: 约140行
  - update()函数: 约300行

- **配置文件**: `config/control_parameters_sim.yaml`
