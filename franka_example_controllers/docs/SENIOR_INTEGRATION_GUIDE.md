# 师兄HQP控制器集成使用指南

**文档版本**: v1.0
**日期**: 2026-04-27
**状态**: ✅ 实现完成

---

## 📋 系统架构

### 数据流
```
key_safe_pub.py (键盘控制)
    ↓ 24维笛卡尔位姿
    /dualarm_mprc/pose_desired
    ↓
┌─────────────────────────────────────┐
│ Cartesian2TrajectoryAdapter         │  ← 适配器节点
│ - IK求解: 笛卡尔→关节                │
│ - 消息转换: Float64MultiArray→      │
│            JointTrajectory          │
└──────────────┬──────────────────────┘
               ↓ 14维关节轨迹
               /dualArm_traj
               ↓
┌─────────────────────────────────────┐
│ DualArmSafeControllerSim            │  ← 师兄HQP控制器
│ - 完整HQP安全控制                   │
│ - 轨迹插值和缓冲                     │
│ - 约束管理                          │
└──────────────┬──────────────────────┘
               ↓ 14维关节命令
               /dual_joint_impedance/joints_desired
               ↓
         dual_joint_impedance_controller
               ↓
              机器人执行
```

### 关键特性

✅ **零改动师兄项目** - 完全保留师兄的成熟HQP控制
✅ **轻量级适配** - 仅200行适配代码
✅ **接口兼容** - 保持原有键盘控制接口
✅ **功能完整** - 所有安全约束完整保留

---

## 🚀 快速启动

### 前提条件

1. ✅ Docker容器已启动
2. ✅ 仿真环境已运行
3. ✅ 底层控制器已加载

### 启动步骤

#### 方式1：使用启动脚本（推荐）

```bash
cd /home/xiaozy24/dual_panda_ws/src/multipanda_ros2/franka_example_controllers/launch
./senior_controller_integration.sh
```

#### 方式2：手动启动

**终端1**: 仿真环境（假设已运行）
```bash
# 已启动 multipanda.launch.py
```

**终端2**: 底层控制器（Docker容器）
```bash
docker exec -it multipanda-container bash
source install/setup.bash

# 加载底层控制器
ros2 control load_controller dual_joint_impedance_controller
ros2 control set_controller_state dual_joint_impedance_controller active
```

**终端3**: 师兄HQP控制器（Docker容器）
```bash
docker exec -it multipanda-container bash
source install/setup.bash

# 启动师兄的控制器
ros2 run dual_arm_reactive_control dual_arm_safe_controller_sim
```

**终端4**: 适配器节点（Docker容器）
```bash
docker exec -it multipanda-container bash
source install/setup.bash

# 启动适配器
ros2 run franka_example_controllers cartesian2traj_adapter
```

**终端5**: 键盘控制（宿主机）
```bash
cd ~/dual_panda_ws
source myenv/bin/activate

# 启动键盘控制
python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py
```

---

## 🔧 编译

### 首次编译

```bash
cd /home/xiaozy24/dual_panda_ws

# 编译我们的包（包含适配器）
colcon build --packages-select franka_example_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release

# 师兄的项目应该已经编译过
# 如果需要，编译师兄的项目：
# colcon build --packages-select dual_arm_reactive_control
```

### 增量编译

```bash
# 只编译修改的包
colcon build --packages-select franka_example_controllers
```

---

## 📊 验证和测试

### 1. 检查节点是否运行

```bash
# 查看ROS节点列表
ros2 node list

# 应该看到：
# /cartesian2traj_adapter
# /dual_arm_safe_controller_sim
# /dual_joint_impedance_controller
```

### 2. 检查话题连接

```bash
# 查看话题列表
ros2 topic list

# 应该看到：
# /dualarm_mprc/pose_desired
# /dualArm_traj
# /dual_joint_impedance/joints_desired
```

### 3. 监控数据流

```bash
# 监控笛卡尔输入
ros2 topic echo /dualarm_mprc/pose_desired --once

# 监控轨迹输出（应该有JointTrajectory消息）
ros2 topic echo /dualArm_traj --once

# 监控关节命令
ros2 topic echo /dual_joint_impedance/joints_desired --once
```

### 4. 功能测试

**测试项目**:
- [ ] 单臂运动控制（左臂/右臂）
- [ ] 双臂协同运动
- [ ] 关节限位保护
- [ ] 碰撞避免
- [ ] 急停响应

---

## 🐛 故障排除

### 问题1：适配器节点无法启动

**症状**:
```
ros2 run franka_example_controllers cartesian2traj_adapter
# 错误: Package not found
```

**解决**:
```bash
# 1. 检查是否编译
colcon build --packages-select franka_example_controllers

# 2. 检查环境变量
source install/setup.bash

# 3. 检查可执行文件
ls install/franka_example_controllers/lib/cartesian2traj_adapter
```

### 问题2：师兄控制器找不到

**症状**:
```
ros2 run dual_arm_reactive_control dual_arm_safe_controller_sim
# 错误: Package not found
```

**解决**:
```bash
# 编译师兄的项目
cd /home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control
colcon build --packages-select dual_arm_reactive_control

# 或者检查是否在正确的工作空间
```

### 问题3：没有轨迹发布

**症状**:
```bash
ros2 topic echo /dualArm_traj
# 没有输出
```

**解决**:
```bash
# 1. 检查适配器是否运行
ros2 node list | grep cartesian2traj

# 2. 检查输入话题
ros2 topic hz /dualarm_mprc/pose_desired

# 3. 检查适配器日志
ros2 run cartesian2traj_adapter --ros-args --log-level DEBUG
```

### 问题4：IK求解失败

**症状**:
```
IK solution contains NaN
```

**原因**:
- 笛卡尔目标不可达
- 目标在奇异构型附近

**解决**:
- 检查键盘控制的目标位置是否合理
- 师兄的控制器有轨迹平滑，可以容忍小的IK误差
- 如果频繁失败，检查碰撞球配置

### 问题5：师兄控制器不响应

**症状**:
```bash
ros2 topic echo /dualArm_traj
# 有消息输出
```

但机器人不动。

**解决**:
```bash
# 1. 检查师兄控制器是否真的在运行
ros2 node list | grep dual_arm_safe_controller

# 2. 检查师兄控制器的订阅
ros2 topic info /dualArm_traj

# 3. 查看师兄控制器的日志
# 应该有 "New trajectory set" 的输出
```

---

## 📈 性能监控

### 监控控制频率

```bash
# 监控关节命令频率
ros2 topic hz /dual_joint_impedance/joints_desired

# 预期: ~100 Hz (师兄控制器的控制频率)
```

### 监控计算性能

师兄的控制器会输出计算时间：
```
computation_time: 0.00234  # 秒
```

正常范围：0.001 - 0.005 秒

---

## 🔍 调试技巧

### 查看详细日志

```bash
# 适配器节点（DEBUG级别）
ros2 run franka_example_controllers cartesian2traj_adapter \
  --ros-args --log-level DEBUG

# 师兄的控制器（已经有很多输出）
ros2 run dual_arm_reactive_control dual_arm_safe_controller_sim
```

### 使用rviz可视化

```bash
# 启动rviz
rviz2

# 添加显示:
# - RobotModel
# - TF
# - MarkerArray (碰撞球)
```

### 记录和分析数据

```bash
# 记录话题
ros2 bag record /dualarm_mprc/pose_desired /dualArm_traj \
  /dual_joint_impedance/joints_desired

# 回放
ros2 bag play rosbag_record

# 分析
ros2 bag info rosbag_record
```

---

## 📝 配置文件

### 适配器参数

默认参数在 `cartesian2traj_adapter.cpp` 中定义：
```cpp
position_threshold_ = 0.001;  // 1mm
rotation_threshold_ = 0.01;   // ~0.57度
max_delta = 0.1;              // 最大关节增量 (rad)
```

### 师兄控制器参数

配置文件路径（师兄项目）：
```bash
/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/
├── control_parameters_sim.yaml    # 控制参数
└── panda_collision_spheres.yaml   # 碰撞球配置
```

---

## 🎯 与原DualArmMprcController的区别

| 特性 | DualArmMprcController | 师兄HQP控制器 |
|------|---------------------|--------------|
| **控制方法** | 雅可比伪逆 + CBF | HQP + 完整约束系统 |
| **轨迹处理** | ❌ 无 | ✅ 缓冲 + 插值 + 滤波 |
| **关节限位** | ✅ 简单限位 | ✅ 加速度约束 |
| **碰撞避免** | ✅ CBF | ✅ ALLCBF (多约束) |
| **奇异避免** | ❌ 无 | ✅ 支持 |
| **状态机** | ⚠️ 简化 | ✅ 完整三态 |
| **异常处理** | ⚠️ 基础 | ✅ 详细 |
| **成熟度** | ⏳ 开发中 | ✅ 生产验证 |

---

## 📞 技术支持

### 遇到问题时

1. **查看日志**：所有节点都有详细日志输出
2. **检查连接**：`ros2 topic list` 和 `ros2 node list`
3. **参考代码**：保留的 `dualarm_mprc_controller.cpp` 可作为参考
4. **师兄项目文档**：查看 `dualarm_mprc/README.md`

### 关键文件位置

```
工作空间根: /home/xiaozy24/dual_panda_ws/

适配器代码:
  src/multipanda_ros2/franka_example_controllers/
  ├── src/adapters/cartesian2traj_adapter.cpp
  └── include/.../cartesian2traj_adapter.hpp

师兄项目:
  src/dualarm_mprc/dualarm_reactive_control/
  ├── src/dual_arm_safe_controller_sim.cpp
  └── config/

启动脚本:
  src/multipanda_ros2/franka_example_controllers/
  └── launch/senior_controller_integration.sh

文档:
  src/multipanda_ros2/franka_example_controllers/docs/
  ├── SENIOR_PROJECT_INTEGRATION_PLAN.md
  ├── HQP_IMPLEMENTATION_GAPS.md
  └── SENIOR_INTEGRATION_GUIDE.md (本文件)
```

---

## ✅ 验收标准

### 功能验收

- [x] 编译成功，无错误
- [ ] 节点正常启动
- [ ] 键盘控制响应正常
- [ ] 单臂/双臂运动控制正常
- [ ] 安全约束生效
- [ ] 控制性能满足要求

### 性能验收

- [ ] 控制频率: ~100 Hz
- [ ] 计算时间: < 5 ms
- [ ] 无明显延迟
- [ ] 轨迹平滑无抖动

---

**更新日期**: 2026-04-27
**文档版本**: v1.0
**维护者**: DualArmMprcController团队
