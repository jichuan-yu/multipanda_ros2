# DualArmMprcController 快速参考卡

## 🚀 快速启动（5分钟）

### 终端1：启动仿真环境
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
source install/setup.bash
ros2 launch multipanda_gazebo multipanda.launch.py
```

### 终端2：启动控制器
```bash
# 加载控制器
ros2 control load_controller dualarm_mprc_controller

# 启动控制器
ros2 control set_controller_state dualarm_mprc_controller active

# 检查状态
ros2 control list_controllers
```

### 终端3：启动键盘控制（宿主机）
```bash
cd ~/dual_panda_ws
source myenv/bin/activate
python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py
```

---

## 🎛️ 控制模式切换

### 零空间模式（默认，稳定）
```bash
ros2 param set /dualarm_mprc_controller use_hqp false
```
**特点**：计算快速，实时性好，经过验证

### HQP模式（高级安全）
```bash
ros2 param set /dualarm_mprc_controller use_hqp true
```
**特点**：层次化约束，安全性强，计算量适中

### 使用辅助脚本
```bash
python3 scripts/mode_switcher.py status      # 查看当前模式
python3 scripts/mode_switcher.py nullspace   # 切换到零空间模式
python3 scripts/mode_switcher.py hqp         # 切换到HQP模式
```

---

## 📊 性能监控

### 启动性能监控
```bash
python3 scripts/performance_monitor.py
```

**输出**：
- 实时统计信息（每100条消息）
- CSV数据文件：`performance_data_YYYYMMDD_HHMMSS.csv`
- 摘要报告：`performance_summary_YYYYMMDD_HHMMSS.txt`

### 快速检查
```bash
# 监控控制频率
ros2 topic hz /dual_joint_impedance/joints_desired

# 查看关节命令
ros2 topic echo /dual_joint_impedance/joints_desired --once

# 查看碰撞球
ros2 topic echo /mprc/collision_spheres --once
```

---

## 🔧 常用参数配置

### 控制器参数（dualarm_mprc_controller.yaml）
```yaml
use_hqp: false              # HQP模式开关
cbf_gamma: 0.1              # CBF修正增益
collision_d_min: 0.05       # 最小安全距离（米）
```

### 约束参数（control_params.yaml）
```yaml
K_joint_tracking: 2.0       # 关节跟踪增益
K_joint_limit: 100.0        # 关节限位增益
K_collision_avoidance: 10.0 # 碰撞避免增益
d_safe: 0.1                 # 安全距离裕度（米）
d_active: 0.3               # 约束激活距离（米）
```

### 动态修改参数
```bash
# 查看所有参数
ros2 param list /dualarm_mprc_controller

# 修改参数
ros2 param set /dualarm_mprc_controller use_hqp true
ros2 param set /dualarm_mprc_controller cbf_gamma 0.15
```

---

## 🐛 快速故障排除

### 问题1：控制器无法加载
```bash
# 检查控制器注册
ros2 plugin list --package franka_example_controllers

# 查看详细日志
ros2 run franka_example_controllers franka_example_controllers --ros-args --log-level DEBUG
```

### 问题2：键盘控制无响应
```bash
# 检查控制器状态
ros2 control list_controllers

# 检查话题连接
ros2 topic list | grep dualarm

# 确认has_target标志（查看日志）
```

### 问题3：HQP模式性能下降
```bash
# 检查控制频率
ros2 topic hz /dual_joint_impedance/joints_desired

# 切换回零空间模式对比
ros2 param set /dualarm_mprc_controller use_hqp false

# 查看系统资源
docker stats multipanda-container
```

---

## 📋 模式对比速查表

| 特性 | 零空间模式 | HQP模式 |
|------|-----------|---------|
| **控制方法** | 雅可比转置 + 零空间投影 | HQP优化求解 |
| **安全功能** | CBF碰撞避免 + 关节限位 | 层次化约束系统 |
| **计算时间** | <0.1 ms | <1 ms |
| **控制频率** | ~1000 Hz | ~500-1000 Hz |
| **稳定性** | ✅ 成熟稳定 | ⚠️ 实验验证中 |
| **适用场景** | 日常使用 | 安全关键任务 |

---

## 📁 关键文件位置

### 配置文件
```
config/
├── dualarm_mprc_controller.yaml   # 控制器配置
└── control_params.yaml            # 约束参数配置
```

### 核心代码
```
include/franka_example_controllers/subscriber/
└── dualarm_mprc_controller.hpp    # 控制器头文件

src/subscriber/
└── dualarm_mprc_controller.cpp    # 控制器实现

src/utils/
├── hierarchical_qp.cpp            # HQP求解器
├── constraint_manager.cpp         # 约束管理器
├── trajectory_buffer.cpp          # 轨迹缓冲
└── trajectory_interpolator.cpp    # 轨迹插值
```

### 文档
```
docs/
├── RUN_DUALARM_MPRC.md            # 运行指南
├── HQP_TESTING_GUIDE.md           # HQP测试指南
├── SCRIPTS_USAGE.md               # 脚本使用说明
└── MIGRATION_COMPLETE.md          # 项目总结
```

### 辅助脚本
```
scripts/
├── performance_monitor.py         # 性能监控
└── mode_switcher.py               # 模式切换
```

---

## 🎯 典型使用场景

### 场景1：日常开发
```bash
# 使用零空间模式（稳定性高）
ros2 param set /dualarm_mprc_controller use_hqp false
# 进行控制开发和调试
```

### 场景2：安全测试
```bash
# 使用HQP模式（安全性强）
ros2 param set /dualarm_mprc_controller use_hqp true
# 启动性能监控
python3 scripts/performance_monitor.py
# 进行安全约束测试
```

### 场景3：性能对比
```bash
# 测试零空间模式
python3 scripts/mode_switcher.py nullspace
python3 scripts/performance_monitor.py &
# ... 进行控制操作 ...

# 切换到HQP模式对比
python3 scripts/mode_switcher.py hqp
python3 scripts/performance_monitor.py &
# ... 进行相同控制操作 ...

# 对比两次测试的CSV数据
```

---

## ⚠️ 重要提醒

### ⭐ 推荐做法
1. ✅ **开发阶段**：使用零空间模式
2. ✅ **功能验证**：对比测试两种模式
3. ✅ **安全关键**：优先使用HQP模式
4. ✅ **性能敏感**：根据测试结果选择

### ⚠️ 注意事项
1. ⚠️ **HQP模式**仍在开发中，建议先在零空间模式验证基础功能
2. ⚠️ **实时性要求**：HQP模式计算量较大，需要监控控制频率
3. ⚠️ **参数敏感**：约束参数对控制效果影响很大，需要仔细调优
4. ⚠️ **向后兼容**：零空间模式保持稳定，可以作为回退方案

---

## 📞 获取帮助

### 文档资源
- 📖 [运行指南](RUN_DUALARM_MPRC.md) - 详细启动说明
- 📖 [HQP测试指南](HQP_TESTING_GUIDE.md) - 完整测试流程
- 📖 [脚本使用说明](SCRIPTS_USAGE.md) - 辅助工具使用
- 📖 [项目总结](MIGRATION_COMPLETE.md) - 技术细节

### 调试技巧
```bash
# 查看ROS日志
ros2 log list

# 监控节点图
ros2 run rqt_graph rqt_graph

# 查看话题信息
ros2 topic info /dual_joint_impedance/joints_desired
ros2 topic echo /dual_joint_impedance/joints_desired
```

---

## 🔍 快速自检

### 系统就绪检查
- [ ] 编译成功：`colcon build --packages-select franka_example_controllers`
- [ ] 容器运行：`docker ps | grep multipanda`
- [ ] 控制器加载：`ros2 control list_controllers`
- [ ] 键盘控制：能控制机器人运动
- [ ] 模式切换：能成功切换use_hqp参数
- [ ] 性能监控：脚本能正常运行

---

**更新日期**：2026-04-27
**版本**：v1.0
**状态**：✅ 编译通过，功能正常
