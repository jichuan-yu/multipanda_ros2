# HQP实现差异分析

**对比对象**：
- 组件A：师兄成熟项目 `dualarm_mprc/dualarm_reactive_control/src/dual_arm_safe_controller_sim.cpp`
- 组件B：我们的新实现 `franka_example_controllers/src/subscriber/dualarm_mprc_controller.cpp`

**分析日期**：2026-04-27

---

## 🔍 核心实现差异

### 1. **轨迹处理系统** ⚠️ **关键差异**

#### 组件A（成熟项目）
```cpp
// 完整的轨迹缓冲和插值系统
traj_buffer_ = std::make_unique<TrajectoryBuffer14d>(buffer_size_);
traj_interpolator_(control_period_);

// 在TRACKING状态：
if (traj_interpolator_.isEmpty()) {
    exception_type_ = ExceptionType::EMPTY_TRAJECTORY;
}
else if (!traj_buffer_->isFull()) {
    Vector14d q_traj = traj_interpolator_.nextStep();
    traj_buffer_->input(q_traj);
}

// 根据缓冲区状态输出：
if (traj_buffer_->isEmpty()) {
    q_r = q_r_last;  // 保持上一次参考
    exception_type_ = ExceptionType::EMPTY_BUFFER;
}
else if (!traj_buffer_->isFilled()) {
    q_r = traj_buffer_->preview_filter();  // 预览（不弹出）
}
else {
    q_r = traj_buffer_->output_filter();   // 滤波输出
}

// 计算参考速度
q_dot_r = (q_r - q_r_last) / control_period_;
```

**特点**：
- ✅ 完整的轨迹缓冲（循环缓冲区）
- ✅ 移动平均滤波（平滑轨迹）
- ✅ 轨迹插值器（恒定速度）
- ✅ 多种输出策略（empty, preview, filter）
- ✅ 参考速度计算（数值微分）

#### 组件B（我们的实现）
```cpp
// ❌ 轨迹缓冲系统未使用
// 虽然 trajectory_buffer_ 和 trajectory_interpolator_ 已初始化，
// 但在HQP控制逻辑中完全没有使用

// 直接从笛卡尔目标计算期望关节位置：
Vector6d delta_x;
delta_x.head<3>() = target_pos - current_pos;
delta_x.tail<3>() = aa.axis() * aa.angle();

Eigen::MatrixXd J_pinv;
dampedPseudoInverse(J, J_pinv);
Vector7d delta_q_task = k_task * J_pinv * delta_x;
q_desired_arm_list[arm_idx] = q_cur + delta_q_task;

// ❌ 没有轨迹平滑
// ❌ 没有缓冲管理
// ❌ 没有参考速度生成
```

**影响**：
- ❌ 轨迹不平滑，可能有抖动
- ❌ 突变的笛卡尔目标导致突变的关节命令
- ❌ 缺少速度控制，可能超过速度限制

---

### 2. **HQP控制回路** ⚠️ **关键差异**

#### 组件A（成熟项目）
```cpp
// 闭环控制（使用当前关节状态）
constraint_manager_.generateCost(q, q_r, q_dot_r, H, f);
constraint_manager_.generateConstraints2HQP(q, dq_c_last, constraints);

// 求解关节速度
hqp_solver_.setCost(H, f);
hqp_solver_.setConstraints(constraints);
hqp_solver_.solve(hqp_result);
q_dot_c = hqp_result.x;

// 积分得到关节位置（闭环）
q1_c_ = q1_ + q1_dot_c_ * control_period_;  // 使用q1_（当前状态）
q2_c_ = q2_ + q2_dot_c_ * control_period_;
```

**特点**：
- ✅ **闭环控制**：使用实际当前关节状态 q
- ✅ **速度级控制**：求解关节速度然后积分
- ✅ **反馈修正**：每个周期都基于实际状态修正

#### 组件B（我们的实现）
```cpp
// ❌ 混合控制方式（位置级 + 速度级）
// 1. 先用雅可比伪逆计算期望关节位置（位置级控制）
q_desired_arm_list[arm_idx] = q_cur + delta_q_task;

// 2. 然后用HQP求解（速度级控制）
Vector14d q_desired;
q_desired << q_desired_arm_list[0], q_desired_arm_list[1];
constraint_manager_->generateCost(q_current, q_desired, dq_current, H, f);
constraint_manager_->generateConstraints2HQP(q_current, dq_current, priority_constraints);

// 求解
solveHQP(q_current, q_desired, dq_solution);

// 积分
Vector14d q_hqp = q_current + dq_solution * dt;
```

**问题**：
- ❌ **混合架构**：先用雅可比伪逆（位置级），再用HQP（速度级）
- ❌ **缺少参考速度**：dq_current总是Zero()
- ❌ **控制逻辑不一致**：两步控制可能导致性能下降
- ❌ **成本函数目标不当**：以q_desired（雅可比伪逆结果）为目标，而不是轨迹参考

---

### 3. **状态机逻辑** ⚠️ **关键差异**

#### 组件A（成熟项目）
```cpp
// 完整的状态机逻辑
switch (current_control_state_) {
    case ControlState::TRACKING:
        // 正常轨迹跟踪
        // 处理轨迹插值和缓冲
        break;

    case ControlState::REACTING:
        // 大误差响应模式
        q_r = q_r_last;  // 停止轨迹更新
        q_dot_r.setZero();  // 速度归零
        break;

    case ControlState::STOPPING:
        // 停止模式
        q1_dot_r_.setZero();
        q2_dot_r_.setZero();
        break;
}

// 状态转换逻辑
switch (current_control_state_) {
    case ControlState::TRACKING:
        if (exception_type_ != NO_EXCEPTION) {
            next_state = STOPPING;
        }
        else if (tracking_error >= threshold) {
            next_state = REACTING;
        }
        break;

    case ControlState::REACTING:
        if (tracking_error < recovery_threshold) {
            next_state = TRACKING;
        }
        break;
}
```

**特点**：
- ✅ 三态完整逻辑（TRACKING/REACTING/STOPPING）
- ✅ 每个状态都有明确的控制策略
- ✅ 完整的异常检测和状态转换

#### 组件B（我们的实现）
```cpp
// ❌ 状态机逻辑简化
void updateControlState(const Vector14d& q_current, const Vector14d& q_desired) {
    Vector14d tracking_error = q_current - q_desired;
    double max_error = tracking_error.cwiseAbs().maxCoeff();

    switch (current_control_state_) {
        case ControlState::STOPPING:
            if (has_target_) {
                next_control_state_ = ControlState::TRACKING;
            }
            break;

        case ControlState::TRACKING:
            if (max_error > 0.5) {
                next_control_state_ = ControlState::REACTING;
            }
            break;

        case ControlState::REACTING:
            if (max_error < 0.1) {
                next_control_state_ = ControlState::TRACKING;
            }
            break;
    }

    current_control_state_ = next_control_state_;
}

// ❌ 状态机结果没有影响控制逻辑
// 虽然调用了 updateControlState()，但控制逻辑没有根据状态改变
```

**问题**：
- ❌ **状态机未生效**：状态机运行但控制逻辑不响应
- ❌ **缺少REACTING控制策略**：检测到大误差但没有特殊处理
- ❌ **缺少异常处理**：没有对应的状态转换逻辑
- ❌ **STOPPING状态不完整**：没有真正停止控制

---

### 4. **异常检测和处理** ⚠️ **关键差异**

#### 组件A（成熟项目）
```cpp
// 详细的异常检测
if (!hqp_result.success || hqp_result.x.size() != 14) {
    exception_type_ = ExceptionType::QP_SOLVER_ERROR;
    RCLCPP_ERROR(node_->get_logger(),
                 "HQP safe controller failed to find a solution. Do not send joint command.");
}

// 关节命令偏离检测
Vector7d dev1 = q1_ - q1_c_;
Vector7d dev2 = q2_ - q2_c_;
if (dev1.lpNorm<Infinity>() >= joint_deviation_threshold ||
    dev2.lpNorm<Infinity>() >= joint_deviation_threshold ||
    q1_dot_c_.lpNorm<Infinity>() >= velocity_command_threshold ||
    q2_dot_c_.lpNorm<Infinity>() >= velocity_command_threshold) {
    exception_type_ = ExceptionType::LARGE_DEVIATION;
    RCLCPP_ERROR(node_->get_logger(),
                 "Joint command deviation is too large. Do not send joint command.");
}

// 异常响应
switch (current_control_state_) {
    case ControlState::TRACKING:
    case ControlState::REACTING:
        if (exception_type_ == ExceptionType::NO_EXCEPTION) {
            pubJointCommand(q1_c_, q2_c_, q1_dot_c_, q2_dot_c_);  // 正常发布
        }
        // 如果有异常，不发布命令
        break;
    case ControlState::STOPPING:
        // 不发布命令
        break;
}
```

**特点**：
- ✅ **多层次异常检测**：QP求解失败、偏离过大、速度过大
- ✅ **安全响应**：异常时不发布命令
- ✅ **详细的错误日志**
- ✅ **数据记录**：记录异常情况

#### 组件B（我们的实现）
```cpp
// ❌ 异常检测不完整
bool solveHQP(const Vector14d& q_current, const Vector14d& q_desired,
              Vector14d& dq_solution) {
    if (!hqp_solver_ || !constraint_manager_) {
        return false;
    }

    // ... 求解逻辑 ...

    if (result.success) {
        dq_solution = result.x.head<14>();
        return true;
    } else {
        RCLCPP_WARN(get_node()->get_logger(), "HQP solver failed to find solution");
        return false;  // ⚠️ 返回false但没有安全检查
    }
}

// ❌ 没有关节命令偏离检测
// ❌ 没有速度命令检查
// ❌ 求解失败后仍然会发布命令（回退到零空间控制）
```

**问题**：
- ❌ **缺少偏离检测**：不检查关节命令是否合理
- ❌ **缺少速度限制**：不检查求解结果的速度是否过大
- ❌ **不安全回退**：HQP失败后直接回退，可能引入突变

---

### 5. **参考轨迹生成** ⚠️ **关键差异**

#### 组件A（成熟项目）
```cpp
// 生成完整的参考轨迹
Vector14d q_r, q_dot_r;  // 参考位置和参考速度

// 根据状态机处理
switch (current_control_state_) {
    case ControlState::TRACKING:
        // 从轨迹缓冲和插值器获取平滑的参考轨迹
        q_r = traj_buffer_->output_filter();
        q_dot_r = (q_r - q_r_last) / control_period_;  // 数值微分
        break;

    case ControlState::REACTING:
        // 停止轨迹更新
        q_r = q_r_last;
        q_dot_r.setZero();
        break;
}

// 传递给ConstraintManager
constraint_manager_.generateCost(q, q_r, q_dot_r, H, f);
```

**特点**：
- ✅ **平滑参考轨迹**：经过缓冲和滤波
- ✅ **参考速度**：显式计算参考速度
- ✅ **状态相关**：不同状态有不同的参考策略

#### 组件B（我们的实现）
```cpp
// ❌ 直接从笛卡尔目标计算关节位置
for (int arm_idx = 0; arm_idx < 2; ++arm_idx) {
    // 计算笛卡尔误差
    Vector6d delta_x;
    delta_x.head<3>() = target_pos - current_pos;
    delta_x.tail<3>() = aa.axis() * aa.angle();

    // 雅可比伪逆
    Eigen::MatrixXd J_pinv;
    dampedPseudoInverse(J, J_pinv);
    Vector7d delta_q_task = k_task * J_pinv * delta_x;
    q_desired_arm_list[arm_idx] = q_cur + delta_q_task;  // 直接位置控制
}

// ❌ 参考速度为零
Vector14d dq_current = Vector14d::Zero();  // 始终为零！

// 传递给ConstraintManager
constraint_manager_->generateCost(q_current, q_desired, dq_current, H, f);
```

**问题**：
- ❌ **无轨迹平滑**：直接从笛卡尔误差计算，不平滑
- ❌ **零参考速度**：dq_current始终为Zero
- ❌ **位置级控制**：不是真正的速度级HQP控制

---

### 6. **控制架构** ⚠️ **架构差异**

#### 组件A（成熟项目）
```
┌─────────────────────────────────────────────────────┐
│                    轨迹输入                          │
│            /dualArm_traj (JointTrajectory)           │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│              轨迹缓冲和插值系统                       │
│  TrajectoryBuffer → MovingAverageFilter            │
│  LinearInterpolator → 恒定速度轨迹                  │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│               参考轨迹生成                            │
│   q_r (参考位置), q_dot_r (参考速度)                │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│              HQP速度控制器                            │
│   min ||q_dot - q_dot_r||²  (跟踪参考速度)           │
│   s.t. 约束 (关节限位、碰撞避免等)                    │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
              q_dot_c (关节速度)
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│               积分得到关节位置                         │
│   q_c = q + q_dot_c * dt  (闭环积分)                 │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
              发布关节命令
```

**特点**：
- ✅ **纯速度级控制**：HQP求解关节速度
- ✅ **轨迹预处理**：缓冲和插值平滑轨迹
- ✅ **闭环控制**：基于实际状态积分

#### 组件B（我们的实现）
```
┌─────────────────────────────────────────────────────┐
│              笛卡尔目标输入                           │
│   /dualarm_mprc/pose_desired (Float64MultiArray)    │
│   [left_pos(3), left_rot(9), right_pos(3), right...] │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│          雅可比伪逆计算期望关节位置                    │
│   q_desired = q_cur + J_pinv * delta_x              │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│               HQP控制器（混合）                       │
│   min ||q - q_desired||²  (跟踪期望位置)             │
│   s.t. 约束                                          │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
            q_dot (关节速度)
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│               积分得到关节位置                         │
│   q_hqp = q_current + q_dot * dt                    │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
              发布关节命令
```

**问题**：
- ❌ **混合架构**：位置级（雅可比）+ 速度级（HQP）
- ❌ **缺少轨迹预处理**：无缓冲和插值
- ❌ **控制目标不当**：以雅可比伪逆结果为目标

---

## 📊 差异总结表

| 功能模块 | 组件A（成熟项目） | 组件B（我们的实现） | 影响 |
|---------|-----------------|-------------------|------|
| **轨迹处理** | ✅ 完整缓冲+插值+滤波 | ❌ 未使用 | ❌ 轨迹不平滑 |
| **参考轨迹** | ✅ 平滑的q_r和q_dot_r | ❌ 直接计算，q_dot=0 | ❌ 控制性能差 |
| **HQP目标** | ✅ 跟踪参考速度 | ❌ 跟踪期望位置 | ❌ 控制逻辑混乱 |
| **控制架构** | ✅ 纯速度级闭环 | ❌ 混合架构 | ❌ 性能下降 |
| **状态机** | ✅ 完整三态逻辑 | ❌ 简化且未生效 | ❌ 异常处理差 |
| **异常检测** | ✅ 多层次安全检查 | ❌ 基本检测 | ❌ 安全性降低 |
| **参考速度** | ✅ 数值微分计算 | ❌ 始终为零 | ❌ 控制不精确 |

---

## 🎯 主要问题汇总

### 1. **控制架构问题** ⚠️ **最严重**
- **问题**：混合了位置级（雅可比伪逆）和速度级（HQP）控制
- **影响**：控制逻辑不一致，性能下降
- **应该**：统一为纯速度级HQP控制

### 2. **轨迹处理缺失** ⚠️ **严重影响**
- **问题**：轨迹缓冲和插值系统未使用
- **影响**：轨迹不平滑，响应突变
- **应该**：使用完整的轨迹缓冲→插值→滤波流程

### 3. **参考速度为零** ⚠️ **性能关键**
- **问题**：dq_current始终为Zero()
- **影响**：无法进行速度跟踪控制
- **应该**：计算参考速度q_dot_r

### 4. **状态机未生效** ⚠️ **功能缺失**
- **问题**：状态机存在但控制逻辑不响应
- **影响**：异常处理不完整
- **应该**：根据状态改变控制策略

### 5. **缺少闭环反馈** ⚠️ **性能问题**
- **问题**：控制逻辑没有充分利用闭环反馈
- **影响**：控制精度和鲁棒性下降
- **应该**：强化闭环控制逻辑

---

## 💡 关键发现

**最核心的差异**：

1. **组件A是真正的速度级HQP控制器**
   - 输入：平滑的参考轨迹（q_r, q_dot_r）
   - 求解：关节速度q_dot_c
   - 输出：积分得到关节位置q_c

2. **组件B是混合控制器**
   - 第一步：雅可比伪逆（位置级）→ q_desired
   - 第二步：HQP（速度级）→ q_dot
   - 问题：两步控制逻辑冲突

**这解释了为什么效果不达预期**！

---

## 📝 建议修复顺序（按重要性）

1. **最紧急**：统一控制架构为纯速度级HQP
2. **很重要**：实现轨迹缓冲和插值
3. **很重要**：计算和使用参考速度
4. **重要**：让状态机逻辑生效
5. **次要**：完善异常检测和处理

---

**文档版本**：v1.0
**分析完成时间**：2026-04-27
**下一步**：等待您的分析和修改指示
