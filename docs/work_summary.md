# 本周总结


## 1. gello接入安全控制器

### 1.1 新增经安全控制器的桥接脚本

原桥接 `gello_franka_ros2.py` 直连低层阻抗，新增
`gello_franka_safety_ros2.py` 使命令先进安全控制器：

```
GELLO -> smoother -> /dualarm_teleop_cmd (type=4, arm_selector)
       -> main_sim_node（限幅/避障/相对位姿/QP）
       -> /mj_{left,right}/joints_desired -> dual_joint_impedance_controller
```

消息格式（`Float64MultiArray`，17 元素）：

`[left_q(7), right_q(7), command_type=4, arm_selector, allow_safety_violation]`


### 1.2 标定脚本 gello_calibrate_offsets.py

单点标准姿态标定：GELLO 摆到 `[0, 0, 0, -0.5π, 0, 0.5π, 0.25π]`，读原始角解 offset：

```
corrected_i = sign_i * (raw_i − offset_i)
offset_i = raw_i − sign_i × TARGET_i   （前6关节按 0.5π 取整，第7关节按 0.25π）
```

读原始角 → 求 offset → 左右独立写回 `gello_arm1.yaml` / `gello_arm2.yaml`。


## 2. 双臂协同误差修复


### 2.1 实验复现

| # | 关节构型 | 速度 | Safe | teleop_err | safe_err |
|---|---------|------|------|-----------|----------|
| 1 | 一致 | 缓速 | on | ≈0 | ≈0 |
| 2 | 一致 | 快速 | on | ≈0 | <1 mm（亚毫米） |
| 3 | **不一致** | 快速 | on | ≈0 | **显著误差** |
| 4 | 一致 | 缓速 | bypass | ≈0 | ≈0 |
| 5 | 一致 | 快速 | bypass | ≈0 | ≈0 |
| 6 | **不一致** | 快速 | bypass | ≈0 | ≈0 |

### 2.2 相对位姿约束定义在左臂末端系

`RELATIVE_POSE` 约束（`constraint_manager.cpp relative_pose_constraint`）的位置分量定义在
左臂末端坐标系 R1：
e_rp = R1ᵀ·(p2 − p1) − p_rel
而 monitor / teleop 参考都是世界系。双臂协调平移必然伴随左臂旋转（R1 最多转
~1.6°），此时 `R1ᵀ(p2−p1)=const` 会把世界系相对向量"带着转"，产生伪误差：


### 2.2HardSoftQP>HQP

残余误差8mm~
HardsoftQP priority≥1是软约束
尝试增大HardsoftQP位姿约束权重，误差减小但会影响CBF
尝试HQP，priority=2，误差1.72 mm


### 2.3 跟踪缓慢

cpp
double alpha = 1.0;
q1_c_ = (1.0-alpha)*q1_c_ + alpha*q1_ + q1_dot_c_*control_period_;
// = q + q_dot_c*T（alpha=1：命令永远只领先实际 ~0.0025 rad）

内环产生力矩 `k·0.0025 ≈ 1.5 N·m` 

1.尝试命令积分：`q_c += q_dot_c · T`+ clamp限幅
2.尝试模仿透传的梯形速度曲线……
