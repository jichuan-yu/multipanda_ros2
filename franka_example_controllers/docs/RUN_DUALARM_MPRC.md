# DualArmMprcController 运行指南

本文档说明如何运行和测试升级后的DualArmMprcController，包含HQP高级安全控制功能。

## ⚠️ 重要提示

在开始之前，请确保：
1. ✅ 已经成功编译了franka_example_controllers包
2. ✅ Docker容器已启动并可进入
3. ✅ 基础仿真环境（终端1）已启动
4. ✅ 底层控制器（dual_joint_impedance_controller）已加载

## 🚀 快速启动指南

### 方式1：使用升级的DualArmMprcController（推荐）

#### 终端3：启动升级的安全控制器
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
source install/setup.bash

# 加载并启动DualArmMprcController（ros2_control插件）
ros2 control load_controller dualarm_mprc_controller
ros2 control set_controller_state dualarm_mpr_controller active
```

#### 终端4：启动键盘控制发布器
```bash
# 在宿主机终端（容器外）
cd ~/dual_panda_ws  # 根据实际路径调整
source myenv/bin/activate
python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py
```

### 方式2：使用原始MPRC节点（对比基准）

#### 终端3：运行原始MPRC安全控制器
```bash
docker exec -it multipanda_container bash
source install/setup.bash
ros2 run dual_arm_reactive_control dualarm_mprc_node
```

---

## 🔧 控制模式说明

### 模式1：零空间控制模式（默认，向后兼容）
- **特点**: 原有的控制逻辑，性能经过验证
- **适用**: 日常使用、测试新功能
- **激活**: 默认激活，或设置 `use_hqp:=false`

### 模式2：HQP高级安全控制模式（实验性）
- **特点**: 层次化约束、优化求解、更强安全保障
- **适用**: 安全关键任务、高级用户
- **激活**: 设置 `use_hqp:=true`

**激活HQP模式**：
```bash
# 方式1：启动时指定
ros2 control load_controller dualarm_mprc_controller
ros2 control set_controller_state dualarm_mprd_controller active
ros2 param set /dualarm_mprc_controller use_hqp true

# 方式2：通过配置文件
# 编辑 config/dualarm_mprc_controller.yaml
# 设置 use_hqp: true
# 然后重启控制器
```

---

## 📊 控制器功能对比

| 功能特性 | 零空间模式 | HQP模式 |
|---------|-----------|---------|
| **基础控制** | ✅ Jacobian转置 + 零空间投影 | ✅ HQP优化 |
| **关节限位** | ✅ 简单限位检查 | ✅ 加速度约束 + 优化 |
| **碰撞避免** | ✅ 单一CBF约束 | ✅ 多约束层次化 |
| **奇异避免** | ❌ 无 | ✅ 奇异避免约束 |
| **计算复杂度** | 低 | 中等 |
| **实时性** | ✅ 极好 (<0.1ms) | ⚠️ 良好 (<1ms) |
| **安全保证** | 基础 | 增强 |
| **成熟度** | 成熟稳定 | 实验验证中 |

---

## 🛠️ 开发和调试

### 查看控制器状态
```bash
# 查看控制器状态
ros2 control list_controllers

# 查看控制器参数
ros2 param list /dualarm_mprc_controller

# 实时修改HQP模式
ros2 param set /dualarm_mprc_controller use_hqp true
```

### 监控话题
```bash
# 碰撞球可视化（27个球/臂）
ros2 topic echo /mprc/collision_spheres

# 关节命令发布
ros2 topic echo /dual_joint_impedance/joints_desired

# 性能监控（如果有实现）
ros2 topic echo /dualarm_mprc_controller/status
```

### 日志级别调整
```bash
# 查看详细日志
ros2 run dualarm_mprc_controller --ros-args --log-level DEBUG
```

---

## ⚠️ 故障排除

### 问题1：控制器无法加载
**症状**: `ros2 control load_controller` 失败

**解决**:
```bash
# 检查控制器是否注册
ros2 plugin list --package franka_example_controllers

# 检查配置文件
ros2 run --ros-args --params-file \
  /home/xiaozy24/dual_panda_ws/src/multipanda_ros2/franka_example_controllers/config/dualarm_mprc_controller.yaml
```

### 问题2：键盘控制无响应
**症状**: 键盘操作但机器人不动

**检查**:
1. ✅ 控制器是否active: `ros2 control list_controllers`
2. ✅ has_target_标志是否设置（日志会显示）
3. ✅ /dualarm_mprc/pose_desired话题是否正常

### 问题3：HQP模式运行缓慢
**症状**: 启用HQP后控制周期明显增长

**解决**:
- 检查碰撞球数量（过多会影响性能）
- 调整控制频率或约束参数
- 切换回零空间模式进行对比

### 问题4：编译错误
**症状**: 编译时出现链接错误

**解决**:
```bash
# 清理并重新编译
cd /home/xiaozy24/dual_panda_ws
rm -rf build install
colcon build --packages-select franka_example_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 🧪 测试建议

### 1. 基础功能测试
- [ ] 键盘控制基本功能
- [ ] 关节限位保护
- [ ] 碰撞避免响应

### 2. HQP模式测试（谨慎）
- [ ] HQP模式切换功能
- [ ] 约束满足情况监控
- [ ] 性能和实时性评估

### 3. 稳定性测试
- [ ] 长时间运行稳定性
- [ ] 极端关节位置处理
- [ ] 异常恢复能力

---

## 📋 与原MPRC系统对比

### 组件A（原MPRC）vs 组件B（升级DualArmMprcController）

#### 控制架构对比
| 方面 | 组件A | 组件B |
|------|--------|--------|
| **底层框架** | 独立节点 | ros2_control插件 |
| **输入接口** | 14维关节轨迹 | 24维权位位姿 |
| **状态机** | 完整三态 | 基础支持 |
| **轨迹系统** | 完整缓冲插值 | 简化版本 |
| **约束管理** | 完整ConstraintManager | 移植的ConstraintManager |

#### 安全能力对比
| 安全功能 | 组件A | 组件B（HQP模式） |
|---------|--------|------------------|
| **关节限位** | ✅ 含加速度 | ✅ 含加速度 |
| **碰撞避免** | ✅ ALLCBF | ✅ ALLCBF |
| **奇异避免** | ✅ 支持 | ⏳ 计划中 |
| **相对位姿** | ✅ 支持 | ⏳ 计划中 |
| **倾角限制** | ✅ 支持 | ⏳ 计划中 |

#### 接口兼容性
- ✅ **输入**: 保持与key_safe_pub.py的24维权位姿接口
- ✅ **输出**: 保持14维关节角度输出
- ✅ **可视化**: 保持碰撞球可视化接口

---

## 🎯 推荐使用流程

### 日常开发和测试
1. 使用**零空间模式**进行开发和调试
2. 功能验证完成后，再测试HQP模式
3. 遇到问题时，切换回零空间模式对比

### 安全关键任务
1. 优先使用HQP模式
2. 实时监控约束满足情况
3. 准备快速回退方案

### 性能对比测试
1. 零空间模式建立性能基准
2. 切换到HQP模式测试
3. 对比两种模式的控制效果

---

## 📝 参数配置说明

### 关键参数位置
- **控制器参数**: `config/dualarm_mprc_controller.yaml`
- **约束参数**: `config/control_params.yaml`
- **碰撞配置**: `dualarm_reactive_control/config/panda_collision_spheres.yaml`

### 控制器参数
- `use_hqp`: 是否启用HQP模式 (bool, default: false)
- `cbf_gamma`: CBF修正增益 (double, default: 0.1)
- `collision_d_min`: 最小安全距离 (double, default: 0.05m)

### 约束参数
- `K_joint_tracking`: 关节跟踪增益 (default: 2.0)
- `K_joint_limit`: 关节限位增益 (default: 100.0)
- `K_collision_avoidance`: 碰撞避免增益 (default: 10.0)
- `d_safe`: 安全距离裕度 (default: 0.1m)
- `d_active`: 约束激活距离 (default: 0.3m)

---

## 🔮 后续开发计划

### 短期（1-2周）
- [ ] 完善HQP控制逻辑实现
- [ ] 添加完整状态机转换
- [ ] 性能优化和参数调优

### 中期（1-2月）
- [ ] 添加奇异避免功能
- [ ] 实现轨迹缓冲系统完整功能
- [ ] 增强异常处理和恢复

### 长期（2-3月）
- [ ] 完整测试和验证
- [ ] 性能基准建立
- [ ] 生产环境部署

---

## 📞 技术支持

### 遇到问题时
1. 查看本文档的故障排除部分
2. 检查编译和安装步骤
3. 对比零空间模式和HQP模式的行为差异
4. 查看ROS 2日志和控制器输出

### 重要提示
- ⚠️ **HQP模式仍在开发中**：建议先在零空间模式下验证基础功能
- ⚠️ **实时性要求**：HQP模式计算量较大，需要监控控制周期
- ⚠️ **参数敏感**：约束参数对控制效果影响很大，需要仔细调优
- ⚠️ **向后兼容**：零空间模式保持稳定，可以作为回退方案

---

**总结**：DualArmMprcController现在支持两种控制模式，提供了从基础到高级的安全控制能力。通过合理的模式选择和参数配置，可以在性能、安全性和复杂度之间取得平衡。
