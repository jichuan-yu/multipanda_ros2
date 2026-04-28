# 🔍 紧急诊断：机械臂不动问题

## 🚨 问题现象
- ✅ 适配器正常发送轨迹（看到Published日志）
- ❌ 师兄控制器没有"New trajectory set"输出
- ❌ 机械臂完全不移动

**结论**：师兄控制器**没有接收到**我们发布的轨迹！

---

## 📋 快速诊断步骤

### 第一步：检查师兄控制器是否真的在运行

```bash
# 在Docker容器中
ros2 node list
```

**期望看到**：
```
/cartesian2traj_adapter
/dual_arm_mprc_node
```

**如果只看到cartesian2traj_adapter**：
- ❌ 师兄控制器没有启动或已经崩溃
- ✅ **解决方法**：重新启动师兄控制器

---

### 第二步：检查话题连接

```bash
# 检查 /dualArm_traj 话题信息
ros2 topic info /dualArm_traj
```

**期望看到**：
```
Publisher count: 1
  - /cartesian2traj_adapter
Subscription count: 1
  - /dual_arm_mprc_node
```

**如果Subscription count: 0**：
- ❌ 师兄控制器没有订阅这个话题
- ⚠️ **可能原因**：师兄控制器初始化失败

---

### 第三步：手动测试话题发布

```bash
# 在另一个终端，手动发布一条轨迹
ros2 topic pub /dualArm_traj trajectory_msgs/msg/JointTrajectory \
  "{
    header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
    joint_names: ['panda_1_joint1', 'panda_1_joint2', 'panda_1_joint3', 'panda_1_joint4', 'panda_1_joint5', 'panda_1_joint6', 'panda_1_joint7',
                  'panda_2_joint1', 'panda_2_joint2', 'panda_2_joint3', 'panda_2_joint4', 'panda_2_joint5', 'panda_2_joint6', 'panda_2_joint7'],
    points: [{
      positions: [0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.0,
                   0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.0]
    }]
  }" --once
```

**观察师兄控制器终端**：
- ✅ 如果看到"New trajectory set" → 话题连接正常
- ❌ 如果没有任何输出 → 话题连接有问题

---

### 第四步：检查师兄控制器是否收到关节状态

师兄控制器需要**关节状态反馈**才能工作：

```bash
# 检查关节状态话题
ros2 topic list | grep joint_state
```

**期望看到**：
```
/panda_1/panda_1_franka_state_controller/joint_states
/panda_2/panda_2_franka_state_controller/joint_states
```

**如果没有**：
- ❌ 硬件接口没有提供关节状态
- ❌ 师兄控制器无法工作（需要关节状态进行控制）

---

## 🎯 最可能的原因

根据经验，最可能的原因是：

### 原因1：师兄控制器启动失败 ⭐⭐⭐

**检查方法**：
```bash
# 查看师兄控制器启动时的完整日志
ros2 run dual_arm_reactive_control dual_arm_mprc_node --ros-args --log-level DEBUG
```

**常见错误**：
- 找不到配置文件
- YAML配置文件错误
- 碰撞球文件路径错误

---

### 原因2：硬件接口不匹配 ⭐⭐

师兄控制器期望：
```cpp
// 订阅这些话题
/panda_1/panda_1_franka_state_controller/joint_states
/panda_2/panda_2_franka_state_controller/joint_states
/panda_1/panda_1_franka_state_controller/franka_states
/panda_2/panda_2_franka_state_controller/franka_states
```

**检查方法**：
```bash
# 这些话题是否存在并发布数据？
ros2 topic hz /panda_1/panda_1_franka_state_controller/joint_states
ros2 topic hz /panda_2/panda_2_franka_state_controller/joint_states
```

**如果没有数据**：
- 需要确认硬件接口配置
- 可能需要修改师兄的订阅话题名称

---

### 原因3：参数缺失 ⭐

师兄控制器期望的参数：
```yaml
/panda_1_xyz
/panda_2_xyz
/trajectory_file
```

**检查方法**：
```bash
# 检查参数是否加载
ros2 param list | grep panda
```

---

## ✅ 临时解决方案

如果上述诊断都失败，建议：

### 方案A：使用我们之前的控制器

```bash
# 使用零空间模式的DualArmMprcController
ros2 control load_controller dualarm_mprc_controller
ros2 control set_controller_state dualarm_mprc_controller active
```

### 方案B：修改适配器，直接发布关节命令

绕过师兄控制器，适配器直接发布关节命令：

```cpp
// 在适配器中直接发布到 /dual_joint_impedance/joints_desired
// 而不是通过师兄的控制器
```

---

## 📝 请提供以下信息

为了进一步诊断，请提供：

1. **完整的师兄控制器启动日志**：
   ```bash
   ros2 run dual_arm_reactive_control dual_arm_mprc_node 2>&1 | tee senior_controller.log
   ```

2. **节点列表**：
   ```bash
   ros2 node list
   ```

3. **话题信息**：
   ```bash
   ros2 topic info /dualArm_traj
   ros2 topic info /dual_joint_impedance/joints_desired
   ```

4. **关节状态话题**：
   ```bash
   ros2 topic list | grep joint
   ros2 topic hz /panda_1/panda_1_franka_state_controller/joint_states
   ```

---

**请先执行这些诊断步骤，然后告诉我结果！** 🚀
