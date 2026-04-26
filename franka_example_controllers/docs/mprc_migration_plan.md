# 完整MPRC安全控制系统迁移计划

## 目标
将组件A（dual_arm_safe_controller_sim.cpp）的完整安全控制系统迁移到组件B（DualArmMprcController），实现完全等效的安全控制能力。

## 组件A核心架构分析

### 1. 求解器架构
```
HQP求解器 (hierarchical_qp)
├── 层次化约束管理
├── 优先级队列
└── OsqpEigen求解器
```

### 2. 约束管理系统
```
ConstraintManager
├── JOINT_LIMIT_WITH_ACC (关节限位+加速度)
├── COLLISION_AVOIDANCE_ALLCBF (碰撞避免)
├── SINGULARITY_AVOIDANCE (奇异避免)
├── RELATIVE_POSE (相对位姿)
└── TILT_LIMIT (倾角限制)
```

### 3. 状态机系统
```
ControlState
├── TRACKING (正常轨迹跟踪)
├── REACTING (反应模式)
└── STOPPING (停止模式)
```

### 4. 轨迹处理系统
```
TrajectoryBuffer (轨迹缓冲)
├── 滑动滤波
└── 数据管理

TrajectoryInterpolator (轨迹插值)
├── 实时插值
└── 流式处理
```

## 迁移步骤

### 阶段1：核心求解器迁移
**目标**: 移植HQP求解器和约束管理系统

#### 1.1 移植HQP求解器
- [ ] 复制`hierarchical_qp.h`和`hierarchical_qp.cpp`
- [ ] 适配到franka_example_controllers命名空间
- [ ] 集成OsqpEigen依赖

#### 1.2 移植ConstraintManager
- [ ] 复制`constraint_manager.h`和`constraint_manager.cpp`
- [ ] 适配接口到ros2_control架构
- [ ] 移除硬编码路径，改为参数配置

#### 1.3 集成到DualArmMprcController
- [ ] 添加HQP和ConstraintManager成员变量
- [ ] 修改update()函数使用HQP而非零空间控制
- [ ] 保持相同的输入输出接口

### 阶段2：状态机迁移
**目标**: 移植完整的状态机逻辑

#### 2.1 移植状态机定义
- [ ] 添加ControlState枚举
- [ ] 实现状态转换逻辑
- [ ] 添加异常检测

#### 2.2 移植异常处理
- [ ] 移植ExceptionType定义
- [ ] 实现异常检测逻辑
- [ ] 添加状态转换触发条件

### 阶段3：轨迹系统迁移
**目标**: 移植轨迹缓冲和插值系统

#### 3.1 移植轨迹缓冲
- [ ] 移植TrajectoryBuffer类
- [ ] 实现滑动滤波
- [ ] 适配24-dim位姿输入

#### 3.2 移植轨迹插值
- [ ] 移植TrajectoryInterpolator类
- [ ] 实现实时插值逻辑
- [ ] 适配流式输入

### 阶段4：参数系统迁移
**目标**: 建立等效的参数配置系统

#### 4.1 YAML配置文件
- [ ] 创建control_params.yaml配置
- [ ] 迁移所有约束参数
- [ ] 添加安全参数配置

#### 4.2 ROS参数集成
- [ ] 在on_configure()中加载参数
- [ ] 支持运行时参数调整
- [ ] 参数验证

### 阶段5：数据记录迁移
**目标**: 移植完整的数据记录系统

#### 5.1 数据记录类
- [ ] 移植数据记录功能
- [ ] 适配ROS2的日志系统
- [ ] 支持可选的数据记录

#### 5.2 话题发布
- [ ] 安全状态话题
- [ ] 调试信息话题
- [ ] 性能监控话题

## 架构适配策略

### 接口适配
组件A和组件B的接口差异需要处理：

#### 输入接口差异
- **组件A**: `/dualArm_traj` (JointTrajectory, 14维)
- **组件B**: `/dualarm_mprc/pose_desired` (Float64MultiArray, 24维)

**解决方案**: 在DualArmMprcController中添加位姿到关节空间的转换

#### 输出接口差异
- **组件A**: `/dual_joint_impedance/joints_desired` (Float64MultiArray, 14维)
- **组件B**: `/dual_joint_impedance/joints_desired` (Float64MultiArray, 14维)

**解决方案**: 保持一致，无需修改

### 控制循环适配
- **组件A**: 独立循环，通过订阅话题触发
- **组件B**: ros2_control的update()循环，1kHz固定频率

**解决方案**: 将HQP求解集成到update()中，保持1kHz频率

## 文件结构

### 新增文件
```
franka_example_controllers/
├── include/franka_example_controllers/
│   ├── utils/
│   │   ├── hierarchical_qp.h          # HQP求解器
│   │   ├── constraint_manager.h       # 约束管理器
│   │   ├── trajectory_buffer.h        # 轨迹缓冲
│   │   └── trajectory_interpolator.h  # 轨迹插值
│   └── subscriber/
│       └── dualarm_mprc_controller.hpp  # 更新后的控制器
├── src/
│   ├── utils/
│   │   ├── hierarchical_qp.cpp
│   │   ├── constraint_manager.cpp
│   │   ├── trajectory_buffer.cpp
│   │   └── trajectory_interpolator.cpp
│   └── subscriber/
│       └── dualarm_mprc_controller.cpp
└── config/
    └── control_params.yaml             # 控制参数配置
```

### CMakeLists.txt修改
- 添加OsqpEigen依赖
- 添加yaml-cpp依赖
- 添加新源文件

## 兼容性保证

### 向后兼容
1. **输入接口**: 保持24维位姿输入
2. **输出接口**: 保持14维关节输出
3. **话题名称**: 不改变现有话题
4. **默认行为**: 在无配置时使用安全默认值

### 渐进迁移
1. **第一阶段**: 先迁移HQP和约束，保持简单接口
2. **第二阶段**: 添加状态机和轨迹处理
3. **第三阶段**: 完整数据记录和监控

## 验证计划

### 功能验证
1. 约束满足测试
2. 状态机转换测试
3. 轨迹跟踪精度测试
4. 安全场景测试

### 性能验证
1. 实时性测试（<1ms计算时间）
2. 内存使用测试
3. CPU占用率测试

### 对比验证
1. 与组件A行为对比
2. 相同场景下的输出对比
3. 安全指标对比

## 风险和缓解

### 主要风险
1. **复杂度增加**: HQP系统复杂度远高于零空间控制
2. **性能开销**: 约束计算可能增加计算时间
3. **参数配置**: 大量参数需要正确配置

### 缓解措施
1. **分阶段迁移**: 先核心功能，次要功能
2. **充分测试**: 每个阶段都要充分测试
3. **参数模板**: 提供经过验证的参数模板
4. **回退机制**: 保留简化版本作为备选

## 时间估算

- 阶段1 (核心求解器): 2-3天
- 阶段2 (状态机): 1-2天
- 阶段3 (轨迹系统): 1-2天
- 阶段4 (参数系统): 1天
- 阶段5 (数据记录): 1天
- 测试和调试: 2-3天

**总计**: 约8-12天完整迁移
