# 接入师兄成熟项目dualarm_mprc的接口适配方案

**方案类型**：轻量级接口适配节点
**设计原则**：最小改动，直接利用师兄项目成熟功能
**日期**：2026-04-27

---

## 📋 方案概述

### 核心思路
**不重新实现HQP控制逻辑**，而是创建一个轻量级的适配节点，将我们的笛卡尔输入接口转换为师兄项目期望的关节轨迹输入接口。

### 系统架构
```
key_safe_pub.py
    │
    │ 24维笛卡尔位姿
    ↓
┌─────────────────────────────────────────┐
│   Cartesian2TrajectoryAdapter (新)      │  ← 轻量级适配节点
│   - 订阅: /dualarm_mprc/pose_desired    │
│   - IK求解: 笛卡尔 → 关节               │
│   - 发布: /dualArm_traj                 │
└──────────────┬──────────────────────────┘
               │
               │ JointTrajectory (14维关节轨迹)
               ↓
┌─────────────────────────────────────────┐
│  DualArmSafeControllerSim (师兄项目)    │  ← 成熟HQP控制器
│  - 订阅: /dualArm_traj                  │
│  - HQP安全控制                          │
│  - 发布: /dual_joint_impedance/joints_desired │
└──────────────┬──────────────────────────┘
               │
               │ 14维关节命令
               ↓
         dual_joint_impedance_controller
```

---

## 🔌 接口对比分析

### 输入接口差异

#### 我们的接口（保持不变）
```cpp
话题: /dualarm_mprc/pose_desired
类型: std_msgs::msg::Float64MultiArray
维度: 24
布局: [left_pos(3), left_rot(9), right_pos(3), right_rot(9)]
     [左臂位置(3), 左臂旋转矩阵(9), 右臂位置(3), 右臂旋转矩阵(9)]
频率: 连续流式输入（键盘控制实时更新）
```

#### 师兄项目的输入接口
```cpp
话题: /dualArm_traj
类型: trajectory_msgs::msg::JointTrajectory
维度: 14个关节
布局: [panda_1_joint1...7, panda_2_joint1...7]
频率: 离散轨迹点（每次新目标时发布）
```

**关键差异**：
- ❌ 笛卡尔 vs 关节
- ❌ 连续流 vs 离散点
- ❌ 24维 vs 14维

### 输出接口对比

#### 我们的输出
```cpp
话题: /dual_joint_impedance/joints_desired
类型: std_msgs::msg::Float64MultiArray
维度: 14
布局: [mj_left_q(7), mj_right_q(7)]
```

#### 师兄项目的输出
```cpp
话题: /dual_joint_impedance/joints_desired
类型: std_msgs::msg::Float64MultiArray
维度: 14
布局: [panda_1_q(7), panda_2_q(7)]
```

✅ **输出接口完全兼容！无需转换！**

---

## 🎯 适配方案设计

### 方案1：轻量级适配节点（推荐）⭐

#### 节点功能
```cpp
class Cartesian2TrajectoryAdapter : public rclcpp::Node {
public:
    Cartesian2TrajectoryAdapter() : Node("cartesian2traj_adapter") {
        // 1. 订阅我们的笛卡尔输入
        cartesian_sub_ = this->create_subscription<Float64MultiArray>(
            "/dualarm_mprc/pose_desired", 10,
            std::bind(&Cartesian2TrajectoryAdapter::cartesianCallback, this, _1)
        );

        // 2. 发布师兄期望的关节轨迹
        traj_pub_ = this->create_publisher<JointTrajectory>(
            "/dualArm_traj", 10
        );

        // 3. 初始化IK求解器（使用现有FrankaRobotModel）
        // 或使用师兄项目中的IK方法
    }

private:
    void cartesianCallback(const Float64MultiArray::SharedPtr msg) {
        // 提取24维笛卡尔位姿
        // [left_pos(3), left_rot(9), right_pos(3), right_rot(9)]

        // IK求解：笛卡尔 → 关节
        Vector7d q_left_ik = solveIK(msg->data[0..11]);    // 左臂
        Vector7d q_right_ik = solveIK(msg->data[12..23]);  // 右臂

        // 构造JointTrajectory消息
        auto traj_msg = JointTrajectory();
        traj_msg.header.stamp = this->now();

        auto point = JointTrajectoryPoint();
        point.positions = {q_left_ik(0), ..., q_left_ik(6),
                          q_right_ik(0), ..., q_right_ik(6)};

        traj_msg.points.push_back(point);

        // 发布轨迹
        traj_pub_->publish(traj_msg);
    }
};
```

#### 关键设计决策

**1. IK求解器选择**
- ✅ **推荐**：使用师兄项目中的IK方法（如果有）
- ✅ **备选**：使用我们现有的FrankaRobotModel进行IK
- ✅ **简单方案**：使用雅可比伪逆（当前实现）

**2. 轨迹构造策略**
- **方案A**：每次收到笛卡尔目标，转换为单个关节点
  ```cpp
  // 简单直接
  traj_msg.points.push_back(single_joint_point);
  ```
- **方案B**：缓存多个笛卡尔目标，生成多段轨迹
  ```cpp
  // 更平滑，但需要缓存和时机管理
  traj_buffer.push_back(joint_point);
  if (traj_buffer.size() >= N) {
      publish_trajectory();
  }
  ```

**推荐**：先用方案A（简单），如需要平滑再考虑方案B

**3. 控制模式兼容**
师兄项目有两种控制器类型：
```cpp
enum class ControllerType {
    HQP,         // 层次化QP
    HardSoftQP   // 软硬约束QP
};
```
- **推荐**：使用HQP模式（师兄主要使用）
- **配置**：通过参数文件设置

---

### 方案2：集成到ros2_control（备选）

如果需要将师兄的控制器集成到ros2_control框架：

#### 修改内容
1. 将`DualArmSafeControllerSim`改造为ros2_control控制器插件
2. 保持师兄的控制逻辑不变
3. 仅修改接口部分（订阅话题 → update逻辑）

#### 工作量评估
- ⚠️ **较大**：需要重构师兄项目的ROS节点架构
- ⚠️ **风险**：可能破坏师兄项目的稳定性
- ✅ **优势**：统一的控制器管理

**不推荐**，除非有明确的ros2_control集成需求

---

## 🔧 实现细节

### 文件结构

#### 新增文件
```
src/multipanda_ros2/franka_example_controllers/
├── src/adapters/
│   └── cartesian2traj_adapter.cpp      # 笛卡尔→轨迹适配器
├── include/franka_example_controllers/adapters/
│   └── cartesian2traj_adapter.hpp
└── launch/
    └── senior_controller_integration.launch.py  # 启动脚本
```

#### 保留文件（参考用）
```
src/subscriber/dualarm_mprc_controller.cpp   # 保留但不使用
src/utils/hierarchical_qp.cpp                # 保留（参考）
src/utils/constraint_manager.cpp             # 保留（参考）
```

### IK求解实现

#### 选项1：使用雅可比伪逆（现有实现）
```cpp
Vector7d solveIK(const Vector3d& pos, const Matrix3d& rot,
                 const Vector7d& q_current, PandaRobot& robot) {
    // 当前末端位姿
    Matrix4d T_current = robot.getEndEffectorPose(q_current);
    Vector3d pos_current = T_current.block<3,1>(0,3);
    Matrix3d rot_current = T_current.block<3,3>(0,0);

    // 笛卡尔误差
    Vector3d delta_pos = pos - pos_current;
    Matrix3d delta_rot = rot * rot_current.transpose();
    AngleAxisd aa(delta_rot);

    Vector6d delta_x;
    delta_x << delta_pos, aa.axis() * aa.angle();

    // 雅可比伪逆
    Matrix6d J = robot.getJacobian(q_current);
    Matrix6d J_pinv;
    dampedPseudoInverse(J, J_pinv);

    // 关节增量
    Vector7d delta_q = J_pinv * delta_x;

    return q_current + delta_q;
}
```

#### 选项2：使用师兄的IK方法（如果有）
```bash
# 检查师兄项目中是否有IK实现
grep -r "inverseKinematics\|solveIK" \
  /home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/
```

### 启动流程

#### 完整启动脚本
```bash
#!/bin/bash
# launch_senior_controller_integration.sh

# 1. 启动仿真环境（终端1）
docker exec -it multipanda-container bash
source install/setup.bash
ros2 launch multipanda_gazebo multipanda.launch.py

# 2. 启动底层控制器（终端2）
ros2 control load_controller dual_joint_impedance_controller
ros2 control set_controller_state dual_joint_impedance_controller active

# 3. 启动师兄的HQP控制器（终端3）
ros2 run dual_arm_reactive_control dual_arm_safe_controller_sim

# 4. 启动适配器节点（终端4）
ros2 run franka_example_controllers cartesian2traj_adapter

# 5. 启动键盘控制（终端5，宿主机）
cd ~/dual_panda_ws
source myenv/bin/activate
python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py
```

---

## ⚠️ 潜在问题和解决方案

### 问题1：控制频率不匹配

**现象**：
- 师兄控制器：100Hz内部循环
- 我们的键盘输入：不确定频率

**解决方案**：
- ✅ 适配器节点直接转发，不做频率控制
- ✅ 师兄的控制器内部有轨迹插值器，会平滑输入

### 问题2：笛卡尔目标连续变化

**现象**：
- 键盘控制时，目标连续变化
- 师兄的控制器期望离散轨迹点

**解决方案**：
- ✅ 每次收到新的笛卡尔目标，转换为关节点并发布
- ✅ 师兄的trajectory_interpolator会处理连续输入
- ✅ 或者设置阈值：只有当目标变化足够大时才发布

**实现示例**：
```cpp
void cartesianCallback(const Float64MultiArray::SharedPtr msg) {
    // 提取新目标
    Vector7d q_new = solveIK(msg);

    // 检查与上次目标的差异
    if ((q_new - q_last_.norm() < threshold_) {
        return;  // 变化太小，忽略
    }

    // 发布轨迹
    publishTrajectory(q_new);
    q_last_ = q_new;
}
```

### 问题3：IK求解失败

**现象**：
- 笛卡尔目标不可达
- IK求解无解

**解决方案**：
- ✅ 使用damped least squares雅可比伪逆
- ✅ 检查IK求解结果，失败时保持上次目标
- ✅ 添加警告日志

```cpp
Vector7d q_ik = solveIK(...);
if (q_ik.hasNaN() || !isValidJointConfiguration(q_ik)) {
    RCLCPP_WARN(get_logger(), "IK solve failed, keeping last target");
    return;  // 保持上次目标
}
```

### 问题4：关节限位冲突

**现象**：
- IK求解的关节位置超出限位
- 师兄的约束管理器可能与IK冲突

**解决方案**：
- ✅ IK求解后进行关节限位饱和
- ✅ 或在IK中添加关节限位约束

```cpp
Vector7d q_ik = solveIK(...);
q_ik = q_ik.cwiseMax(q_min).cwiseMin(q_max);  // 饱和处理
```

---

## 📊 方案对比

| 特性 | 方案1：适配节点 | 方案2：ros2_control集成 |
|------|----------------|----------------------|
| **实现难度** | ✅ 简单（~200行代码） | ⚠️ 复杂（重构架构） |
| **开发时间** | ✅ 短（1-2小时） | ⚠️ 长（1-2天） |
| **风险** | ✅ 低（不影响师兄项目） | ⚠️ 中（可能引入bug） |
| **灵活性** | ✅ 高（独立节点） | ⚠️ 低（框架限制） |
| **维护性** | ✅ 好（职责单一） | ⚠️ 中（代码混合） |
| **性能** | ✅ 良好（轻量级） | ⚠️ 可能略差 |

**推荐**：**方案1（适配节点）** ⭐

---

## 🎯 实施步骤

### 第一阶段：基础适配（1-2小时）

1. **创建适配器节点框架**
   - [ ] 创建cartesian2traj_adapter.hpp/cpp
   - [ ] 实现基本订阅/发布逻辑
   - [ ] 编译验证

2. **实现IK求解**
   - [ ] 集成现有雅可比伪逆代码
   - [ ] 测试单臂IK
   - [ ] 测试双臂IK

3. **消息转换**
   - [ ] 24维→14维转换
   - [ ] 构造JointTrajectory消息
   - [ ] 验证消息格式

### 第二阶段：集成测试（1小时）

4. **启动师兄控制器**
   - [ ] 编译师兄项目
   - [ ] 启动仿真环境
   - [ ] 启动师兄的HQP控制器

5. **端到端测试**
   - [ ] 启动适配器节点
   - [ ] 启动键盘控制
   - [ ] 测试基本运动功能
   - [ ] 测试安全约束

### 第三阶段：优化完善（可选）

6. **性能优化**
   - [ ] 添加目标变化阈值
   - [ ] 优化IK求解性能
   - [ ] 减少延迟

7. **错误处理**
   - [ ] IK失败检测
   - [ ] 关节限位检查
   - [ ] 异常日志记录

---

## 📝 关键代码片段

### 消息转换示例

```cpp
void Cartesian2TrajectoryAdapter::cartesianCallback(
    const Float64MultiArray::SharedPtr msg) {

    // 1. 解析24维笛卡尔位姿
    Vector3d left_pos, right_pos;
    Matrix3d left_rot, right_rot;

    left_pos << msg->data[0], msg->data[1], msg->data[2];
    left_rot << msg->data[3],  msg->data[4],  msg->data[5],
               msg->data[6],  msg->data[7],  msg->data[8],
               msg->data[9],  msg->data[10], msg->data[11];

    right_pos << msg->data[12], msg->data[13], msg->data[14];
    right_rot << msg->data[15], msg->data[16], msg->data[17],
               msg->data[18], msg->data[19], msg->data[20],
               msg->data[21], msg->data[22], msg->data[23];

    // 2. IK求解
    Vector7d q_left = solveIK(left_pos, left_rot, q_left_current_);
    Vector7d q_right = solveIK(right_pos, right_rot, q_right_current_);

    // 3. 关节限位
    q_left = q_left.cwiseMax(q_min_).cwiseMin(q_max_);
    q_right = q_right.cwiseMax(q_min_).cwiseMin(q_max_);

    // 4. 构造轨迹消息
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.header.stamp = this->now();
    traj_msg.joint_names = {
        "panda_1_joint1", "panda_1_joint2", ..., "panda_1_joint7",
        "panda_2_joint1", "panda_2_joint2", ..., "panda_2_joint7"
    };

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions.resize(14);
    for (int i = 0; i < 7; i++) {
        point.positions[i] = q_left(i);
        point.positions[i+7] = q_right(i);
    }

    traj_msg.points.push_back(point);

    // 5. 发布
    traj_pub_->publish(traj_msg);

    // 6. 更新当前关节状态
    q_left_current_ = q_left;
    q_right_current_ = q_right;
}
```

---

## ✅ 优势分析

### 为什么这个方案好？

1. **最小改动** ⭐
   - 不修改师兄项目代码
   - 不重新实现复杂HQP逻辑
   - 风险极低

2. **快速实现** ⭐
   - 预计1-2小时完成基础功能
   - 代码量小（~200行）
   - 易于测试和调试

3. **功能完整** ⭐
   - 直接使用师兄成熟的HQP控制
   - 所有安全约束完整保留
   - 性能经过验证

4. **易于维护** ⭐
   - 适配器逻辑简单
   - 问题定位容易
   - 可以参考已迁移的代码

5. **灵活性高** ⭐
   - 可以替换IK方法
   - 可以调整转换策略
   - 不影响师兄项目

---

## 🚦 审核要点

请您审核以下关键设计决策：

### 1. **架构选择**
✅ 适配节点 vs ⚠️ ros2_control集成
- **推荐**：适配节点
- **理由**：简单、低风险、易维护

### 2. **IK求解方法**
- 选项A：雅可比伪逆（现有实现）
- 选项B：使用师兄的IK方法（如果有）
- 选项C：第三方IK库（如TracIK）

**您倾向哪个？**

### 3. **轨迹发布策略**
- 方案A：每次笛卡尔目标→单个关节点（简单）
- 方案B：缓存多个目标→多段轨迹（平滑）

**您倾向哪个？**

### 4. **控制频率处理**
- 直接转发，由师兄控制器内部处理
- 适配器节点做频率控制

**您倾向哪个？**

### 5. **文件组织**
- 新建`adapters/`目录存放适配器代码
- 保留现有`subscriber/dualarm_mprc_controller.cpp`作为参考

**您同意吗？**

---

## 📋 下一步

**等待您的审核和反馈**：
1. ✅ 架构设计是否合理？
2. ✅ 关键决策是否同意？
3. ✅ 还有其他考虑因素？
4. ✅ 可以开始实施吗？

**审核通过后**：
1. 创建适配器节点代码
2. 配置CMakeLists.txt
3. 编写启动脚本
4. 测试验证

---

**方案版本**：v1.0
**设计完成时间**：2026-04-27
**等待审核**：⏳
