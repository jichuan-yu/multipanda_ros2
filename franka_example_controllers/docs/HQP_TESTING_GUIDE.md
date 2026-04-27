# HQP模式测试指南

## 📋 测试概述

本文档提供DualArmMprcController中HQP高级安全控制模式的详细测试指南，包括功能验证、性能对比和参数调优建议。

## 🎯 测试目标

### 主要目标
- ✅ 验证HQP模式基本功能正确性
- ✅ 对比HQP模式与零空间模式的性能差异
- ✅ 评估安全约束的有效性
- ✅ 测试模式切换和回退机制
- ✅ 确定最优参数配置

### 测试环境要求
- ✅ Docker容器运行正常
- ✅ 基础仿真环境已启动（终端1）
- ✅ 底层控制器已加载（终端2）
- ✅ 编译版本包含HQP功能

---

## 🚀 快速测试流程

### 第一步：零空间模式基准测试（推荐起点）

#### 终端3：启动零空间模式控制器
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
source install/setup.bash

# 确认使用零空间模式（默认）
ros2 param set /dualarm_mprc_controller use_hqp false
ros2 control list_controllers
```

#### 终端4：启动键盘控制
```bash
# 在宿主机终端
cd ~/dual_panda_ws
source myenv/bin/activate
python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py
```

#### 基准测试项目
1. **基本运动控制**
   - [ ] 单臂运动（左臂/右臂独立测试）
   - [ ] 双臂协同运动
   - [ ] 关节运动平滑性

2. **安全功能验证**
   - [ ] 关节限位保护（尝试超出限位）
   - [ ] 碰撞避免（双臂相互靠近）
   - [ ] 急停响应

3. **性能基准记录**
   ```bash
   # 监控控制频率
   ros2 topic hz /dual_joint_impedance/joints_desired

   # 查看关节命令
   ros2 topic echo /dual_joint_impedance/joints_desired --once
   ```

**记录基准数据**：
- 平均控制频率：_______ Hz
- 最大跟踪误差：_______ rad
- 碰撞球数量：_______ 个/臂

---

### 第二步：HQP模式功能测试

#### 启用HQP模式
```bash
# 在终端3（控制器运行中）
ros2 param set /dualarm_mprc_controller use_hqp true

# 确认模式切换成功
ros2 param get /dualarm_mprc_controller use_hqp

# 查看日志确认HQP激活
# 应该看到：HQP mode enabled
```

#### HQP功能验证
1. **基本控制功能**
   - [ ] 重复零空间模式的所有测试项目
   - [ ] 验证运动控制响应性
   - [ ] 检查关节命令平滑性

2. **HQP特有功能**
   - [ ] 层次化约束生效（查看日志）
   - [ ] 关节限位加速度约束
   - [ ] 多约束同时满足能力

3. **状态机测试**
   ```
   测试场景：
   - [ ] TRACKING → REACTING 转换（大误差时）
   - [ ] REACTING → TRACKING 恢复（误差减小时）
   - [ ] 异常检测和处理
   ```

#### 监控HQP性能
```bash
# 实时监控控制命令
ros2 topic echo /dual_joint_impedance/joints_desired

# 查看碰撞球（HQP模式下应该更密集）
ros2 topic echo /mprc/collision_spheres --once

# 监控控制器状态（如果实现）
ros2 topic echo /dualarm_mprc_controller/status
```

---

### 第三步：模式切换和回退测试

#### 动态模式切换
```bash
# 测试1：零空间 → HQP
ros2 param set /dualarm_mprc_controller use_hqp false
# 等待5秒，观察控制稳定性
ros2 param set /dualarm_mprc_controller use_hqp true
# 观察切换过程是否平滑

# 测试2：HQP → 零空间
ros2 param set /dualarm_mprc_controller use_hqp true
ros2 param set /dualarm_mprc_controller use_hqp false
```

**预期结果**：
- ✅ 切换过程平滑，无突变
- ✅ 机器人保持稳定
- ✅ 控制命令连续

#### 自动回退机制测试
```bash
# 触发HQP求解失败场景
# 方法1：设置极端约束参数
ros2 param set /dualarm_mprc_controller collision_d_min 100.0

# 观察日志，应该看到：
# "HQP solve failed, falling back to null-space control"

# 恢复正常参数
ros2 param set /dualarm_mprc_controller collision_d_min 0.05
```

---

## 📊 性能对比测试

### 测试方案

#### 方案1：轨迹跟踪精度对比
**测试任务**：执行相同的笛卡尔空间轨迹

**步骤**：
1. 零空间模式下记录轨迹：`traj_nullspace.csv`
2. HQP模式下记录轨迹：`traj_hqp.csv`
3. 对比跟踪误差

**数据记录脚本**：
```bash
# 记录关节命令数据
ros2 topic echo /dual_joint_impedance/joints_desired > traj_nullspace.csv
ros2 topic echo /dual_joint_impedance/joints_desired > traj_hqp.csv
```

**分析指标**：
- 平均跟踪误差
- 最大跟踪误差
- 轨迹平滑性（速度变化率）

#### 方案2：安全约束响应对比
**测试场景**：双臂相互靠近碰撞边界

**对比点**：
| 指标 | 零空间模式 | HQP模式 |
|------|-----------|---------|
| 碰撞避免距离 | | |
| 响应速度 | | |
| 运动平滑性 | | |
| 关节限位保护 | | |

#### 方案3：计算性能对比
**监控指标**：
```bash
# 监控控制周期（如果实现）
ros2 topic hz /dual_joint_impedance/joints_desired

# 查看CPU使用率
docker stats multipanda-container
```

**性能表格**：
| 模式 | 控制频率 | CPU使用率 | 内存使用 |
|------|---------|----------|---------|
| 零空间 | | | |
| HQP | | | |

---

## 🔧 参数调优指南

### 关键参数说明

#### 控制器参数
```yaml
# config/dualarm_mprc_controller.yaml
use_hqp: false                    # HQP模式开关
cbf_gamma: 0.1                    # CBF修正增益
collision_d_min: 0.05             # 最小安全距离（米）
```

#### 约束参数
```yaml
# config/control_params.yaml
K_joint_tracking: 2.0             # 关节跟踪增益
K_joint_limit: 100.0              # 关节限位增益
K_collision_avoidance: 10.0       # 碰撞避免增益
d_safe: 0.1                       # 安全距离裕度（米）
d_active: 0.3                     # 约束激活距离（米）
```

### 调优建议

#### 1. 跟踪性能优化
**问题**：跟踪误差过大
**解决方案**：
- 增大 `K_joint_tracking` (建议范围: 1.0 ~ 5.0)
- 减小 `d_safe` (建议范围: 0.05 ~ 0.15)

#### 2. 安全性优化
**问题**：碰撞避免响应不足
**解决方案**：
- 增大 `K_collision_avoidance` (建议范围: 5.0 ~ 20.0)
- 增大 `d_active` (建议范围: 0.2 ~ 0.5)
- 减小 `collision_d_min` (建议范围: 0.03 ~ 0.08)

#### 3. 实时性优化
**问题**：控制频率下降
**解决方案**：
- 减少碰撞球数量（修改PandaRobot配置）
- 增大 `kAvoidanceInterval` (建议范围: 50 ~ 100)
- 降低约束更新频率

#### 4. 运动平滑性优化
**问题**：关节运动抖动
**解决方案**：
- 减小 `K_joint_tracking`
- 增大 `cbf_gamma` (建议范围: 0.05 ~ 0.2)
- 启用轨迹插值滤波

---

## 🐛 故障排除

### 问题1：HQP模式无法启用
**症状**：`ros2 param set use_hqp true` 无效

**诊断步骤**：
```bash
# 检查控制器状态
ros2 control list_controllers

# 查看参数是否设置成功
ros2 param get /dualarm_mprc_controller use_hqp

# 查看日志错误
ros2 run franka_example_controllers franka_example_controllers --ros-args --log-level DEBUG
```

**可能原因**：
- HQP求解器未正确初始化
- 约束管理器加载失败
- 配置文件路径错误

### 问题2：HQP求解频繁失败
**症状**：日志频繁显示"HQP solve failed"

**诊断步骤**：
```bash
# 检查约束参数是否合理
ros2 param list /dualarm_mprc_controller

# 查看具体约束违反情况
ros2 topic echo /dualarm_mprc_controller/status
```

**解决方案**：
1. 放宽约束参数（增大`d_safe`，减小`K_*`增益）
2. 检查碰撞球配置是否过多
3. 验证关节限位设置是否正确

### 问题3：模式切换后机器人不稳定
**症状**：切换模式时机器人抖动或停止

**诊断步骤**：
```bash
# 监控控制命令连续性
ros2 topic echo /dual_joint_impedance/joints_desired
```

**解决方案**：
1. 确保切换时机器人处于静止状态
2. 检查两种模式的控制命令差异
3. 调整HQP积分时间步长`dt`

### 问题4：性能下降明显
**症状**：启用HQP后控制频率显著降低

**诊断步骤**：
```bash
# 监控计算时间
# 在代码中添加时间测量日志
```

**优化方案**：
1. 减少约束层次数量
2. 降低碰撞球更新频率
3. 使用更快的QP求解器配置

---

## 📈 测试报告模板

### 测试环境
- 测试日期：___________
- 测试人员：___________
- 编译版本：___________
- ROS 2版本：___________

### 功能测试结果

| 测试项目 | 零空间模式 | HQP模式 | 备注 |
|---------|-----------|---------|------|
| 基本运动控制 | ✅/❌ | ✅/❌ | |
| 关节限位保护 | ✅/❌ | ✅/❌ | |
| 碰撞避免 | ✅/❌ | ✅/❌ | |
| 模式切换 | ✅/❌ | ✅/❌ | |
| 回退机制 | ✅/❌ | ✅/❌ | |

### 性能测试结果

| 性能指标 | 零空间模式 | HQP模式 | 差异 |
|---------|-----------|---------|------|
| 控制频率 (Hz) | | | |
| 平均跟踪误差 (rad) | | | |
| 最大跟踪误差 (rad) | | | |
| CPU使用率 (%) | | | |
| 内存使用 (MB) | | | |

### 参数配置
**最终使用的参数**：
```yaml
# 控制器参数
use_hqp: _______
cbf_gamma: _______
collision_d_min: _______

# 约束参数
K_joint_tracking: _______
K_joint_limit: _______
K_collision_avoidance: _______
d_safe: _______
d_active: _______
```

### 结论和建议
1. **总体评估**：
   - [ ] HQP模式功能完整，可投入使用
   - [ ] HQP模式需要进一步优化
   - [ ] 建议使用零空间模式

2. **主要优势**：
   - _______________________
   - _______________________

3. **需要改进**：
   - _______________________
   - _______________________

4. **后续工作建议**：
   - _______________________
   - _______________________

---

## 🎓 进阶测试

### 压力测试
```bash
# 测试极端关节位置
# 测试快速运动响应
# 测试长时间运行稳定性
```

### 边界条件测试
```bash
# 测试奇异构型附近的控制
# 测试工作空间边界的控制
# 测试多约束同时激活的情况
```

### 安全验证测试
```bash
# 测试急停功能
# 测试异常恢复能力
# 测试约束违反后的保护机制
```

---

## 📞 技术支持

### 遇到问题时的检查清单
- [ ] 查看控制器日志：`ros2 log list`
- [ ] 检查参数配置：`ros2 param dump`
- [ ] 验证话题连接：`ros2 topic list`
- [ ] 监控系统资源：`docker stats`

### 调试技巧
1. **分步测试**：先测试单臂，再测试双臂
2. **参数对比**：记录不同参数下的控制效果
3. **日志分析**：使用DEBUG级别日志获取详细信息
4. **可视化辅助**：使用RViz观察碰撞球和机器人状态

---

## 附录：常用命令

### 控制器管理
```bash
# 加载控制器
ros2 control load_controller dualarm_mprc_controller

# 启动控制器
ros2 control set_controller_state dualarm_mprc_controller active

# 停止控制器
ros2 control set_controller_state dualarm_mprc_controller inactive

# 卸载控制器
ros2 control unload_controller dualarm_mprc_controller
```

### 参数管理
```bash
# 查看所有参数
ros2 param list /dualarm_mprc_controller

# 获取单个参数
ros2 param get /dualarm_mprc_controller use_hqp

# 设置参数
ros2 param set /dualarm_mprc_controller use_hqp true

# 导出参数配置
ros2 param dump /dualarm_mprc_controller > params.yaml
```

### 监控和调试
```bash
# 监控话题频率
ros2 topic hz /dual_joint_impedance/joints_desired

# 查看话题内容
ros2 topic echo /dual_joint_impedance/joints_desired

# 记录话题数据
ros2 bag record /dual_joint_impedance/joints_desired

# 查看节点图
ros2 run rqt_graph rqt_graph
```

---

**更新日期**：2026-04-27
**文档版本**：v1.0
**维护者**：DualArmMprcController开发团队
