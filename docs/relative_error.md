# 双臂协同误差分析实验

## 实验目的

研究协同误差的产生源自当前控制链条的哪一层。

## 控制流程

当前系统的控制流程：

```
teleop (遥操作命令) → safe (安全控制器) → impedance (阻抗控制器)
```

## 实验方法

使用 `relative_error_monitor` 节点监控双臂相对位置误差：
- **初始基准**：从 `/joint_states` 获取初始关节状态，计算初始相对位置
- **Teleop 误差**：teleop command 目标相对位置与初始基准的偏差
- **Safe 误差**：safe command 目标相对位置与初始基准的偏差

### 监控话题
- `/joint_states` - 当前关节状态（设置初始基准）
- `/dualarm_teleop_cmd` - 遥操作命令
- `/mj_left/joints_desired` - 左臂期望关节命令（safe output）
- `/mj_right/joints_desired` - 右臂期望关节命令（safe output）

### 实验条件定义
- **双臂关节角度一致**：左右臂关节角度理想值完全相同
- **双臂关节角度不一致**：双臂末端对称虚拟夹取物体的任务配置

---

## 实验结果

### 实验 1：缓速移动 + Safe + 关节角度一致

![实验1](../data/mprc/20260806_090244_relative_error_plot.png)

- `teleop_err_norm ≈ 0 mm`
- `safe_err_norm ≈ 0 mm`

---

### 实验 2：快速移动 + Safe + 关节角度一致

![实验2](../data/mprc/20260806_091128_relative_error_plot.png)

- `teleop_err_norm ≈ 0 mm`
- `safe_err_norm < 1 mm`（亚毫米级误差）

---

### 实验 3：快速移动 + Safe + 关节角度不一致

![实验3](../data/mprc/20260806_091731_relative_error_plot.png)

- `teleop_err_norm ≈ 0 mm`
- `safe_err_norm` 出现显著误差

---

### 实验 4：缓速移动 + 绕过 Safe + 关节角度一致

![实验4](../data/mprc/20260806_093148_relative_error_plot.png)

- `teleop_err_norm ≈ 0 mm`
- `safe_err_norm ≈ 0 mm`（无 safe 修改）

---

### 实验 5：快速移动 + 绕过 Safe + 关节角度一致

![实验5](../data/mprc/20260806_093916_relative_error_plot.png)

- `teleop_err_norm ≈ 0 mm`
- `safe_err_norm ≈ 0 mm`（无 safe 修改）

---

### 实验 6：快速移动 + 绕过 Safe + 关节角度不一致

![实验6](../data/mprc/20260806_094210_relative_error_plot.png)

- `teleop_err_norm ≈ 0 mm`
- `safe_err_norm ≈ 0 mm`（无 safe 修改）

---

## 实验结论

| 实验条件 | Teleop 误差 | Safe 误差 | 协同误差来源 |
|---------|------------|----------|------------|
| 缓速 + Safe + 一致 | ≈ 0 | ≈ 0 | - |
| 快速 + Safe + 一致 | ≈ 0 | < 1 mm | Safe 层 |
| 快速 + Safe + 不一致 | ≈ 0 | 显著误差 | Safe 层 |
| 缓速 + 绕过 Safe + 一致 | ≈ 0 | ≈ 0 | - |
| 快速 + 绕过 Safe + 一致 | ≈ 0 | ≈ 0 | - |
| 快速 + 绕过 Safe + 不一致 | ≈ 0 | ≈ 0 | - |

### 关键发现

1. **遥操作层精度**：teleop command 的相对误差极小（接近 0），遥操作脚本发布的控制指令精度可靠，可以忽略不计。

2. **Safe 层引入误差**：
   - 在快速移动时，Safe 修改会引入亚毫米级误差（关节角度一致时）
   - 在双臂关节角度不一致的任务配置下，Safe 修改会产生显著的协同误差

3. **误差根源确认**：当绕过 Safe 层（`bypass_qp_safety_for_debug:=true`）时，无论何种实验条件，协同误差都很小，表现良好。这证明**协同误差出自 Safe 层**。

### 原因分析

Safe 层产生误差的可能原因：
- QP 求解器在处理不对称任务约束时的数值误差
- 碰撞避免约束与协同约束之间的冲突
- 约束权重在不对称配置下的不当分配

---

## 2026-08-15 补充：方案 B —— 积分参考（有界速度追踪），替代"速度置零"补丁

### 背景

绕过 Safe 层的调试路径（`bypass_qp_safety_for_debug:=true`）中，原来的稳定性方案是**把阻抗指令速度直接置零**
（`q_dot_c.setZero()`），只发布位置参考。这是必要的，因为原参考生成器是死区/再锚定式的：

- `q_r = q_actual + J⁺·Δx`（每帧/每条消息重新锚定到实际位形，位置误差永远不累积）
- `q_dot_r = J⁺·Δx / control_period_`（100 s⁻¹ 的合成增益，消息间隔内保持恒定）

若把 `q_dot_r` 原样发给阻抗内环，`d·dq_d` 会持续推动内环 → 超调 → 下一条消息残差反号 → 振荡发散
（实测 1471–2378 mm）。置零方案虽稳定，但留下了 ~3 mm 的相对误差。

### 方案 B 实现（`dual_arm_safe_controller_sim.cpp`）

新增内部积分参考，**不再锚定实际**：

```
v = sat(Kp · delta(x_target, x_ref), v_max)     // Kp=20 1/s, v_max_lin=0.05 m/s, v_max_rot=0.15 rad/s
x_ref += v · dt                                  // 任务空间参考位姿积分
q_dot_ref = J(q_ref)⁺ · v                        // 在参考位形处做微分 IK（自洽）
q_ref += q_dot_ref · dt                          // 关节参考积分
```

- 发布 `position = q_ref`，`velocity = q_dot_ref`（位置/速度自洽，速度有界）
- 第一条消息时把参考锚定在实际位形（`initIntegratedReference`），之后不再重锚
- 仅在 `TASK_SPACE_POSE`（类型 5）指令下生效，其它指令类型保持原语义
- 新参数 `use_integrated_reference`（默认 false，可运行时 `ros2 param set` 切换）

### 验证结果（决定性对称扫频：双臂同步 +10mm 再返回，2mm/步）

同一锚点构型、独立 Python DH FK 交叉验证：

| 指标（实际级最大相对误差） | 速度置零基线 | 方案 B |
|----------------------------|-------------|--------|
| 慢扫 2 s/步                | 3.005 mm    | **0.538 mm** |
| 快扫 0.1 s/步（原最恶劣工况）| 6.755 mm    | **1.032 mm** |
| monitor safe max（快扫）   | 10.35 mm    | 3.875 mm |

- 方案 B 全程稳定无发散；快扫下误差仅为基线的 1/6.5，慢扫下为 1/5.6
- 时间轨迹平滑、收尾归零（实际误差终值 0.02 mm）
- 快扫命令隐含关节速度：典型 0.01–0.02 rad/s，峰值（采样差分噪声）< 0.5 rad/s —— 有界且与参考真实速度一致

### 使用方法

```bash
# 启用方案 B（需同时绕过 Safe 调试路径）
ros2 run dual_arm_reactive_control main_sim_node \
  --ros-args -p bypass_qp_safety_for_debug:=true -p use_integrated_reference:=true
```

### 实现注意（本次修复的两个坑）

1. `use_integrated_reference` 在无遥操消息时被运行时打开，会让 inactive 分支读到**未初始化**的 `q_ref_`
   （Eigen 默认 = 垃圾值）并把垃圾位置发布出去 → 机械臂飞离健康构型。已修复：inactive 分支只在
   `ref_initialized_` 为真时才使用 `q_ref_`，否则保持实际位置；`stepIntegratedReference` 也加了守卫。
2. 方案 B 只应接管绝对任务空间位姿指令（类型 5），若对 `JOINT_POSITION`（类型 4）等其它指令也生效，
   复位/保持类指令会被错误的参考覆盖。已加按指令类型生效的开关 `use_integrated_ref_now_`。

---

## 使用方法

### 启动监控节点

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control relative_error_monitor_node --ros-args \
  -p output_path:=/home/xiaozy24/dual_panda_ws/data/mprc/ \
  -p output_prefix:=$(date +%Y%m%d_%H%M%S)_relative_error
```

### 启动控制器（绕过 Safe）

```bash
ros2 run dual_arm_reactive_control main_sim_node \
  --ros-args \
  -p bypass_qp_safety_for_debug:=true
```

### 绘图脚本

```bash
source ~/myenv/bin/activate
python3 src/multipanda_ros2/teleop/plot_relative_error.py <csv_file> -o output.png

# 从指定时间点开始绘制（例如 100s）
python3 src/multipanda_ros2/teleop/plot_relative_error.py <csv_file> -o output.png -t 100
```

---

## 2026-08-16 补充：数据分代说明 + 底层（阻抗）控制器实验入口

### 数据分代

`relative_error.md` 里 exp01–06 是原作者 8/06 的记录；当前代码下**应以 8/15 验证表为基线**
（见上文 2026-08-15 一节：快扫 monitor safe max 10.35 mm / 实际级 6.755 mm / 慢扫 3.005 mm）。
因此 exp05/06 的 `safe_err ≈ 0 mm` 属于旧数据，已不符合当前清单的语义，**不要再用它当作
“绕过 Safe 层就无误差”的依据**。当前真正测到的是“命令 vs 实际”的 FK 相对几何误差。

### 底层阻抗控制器的语法

即便不跑 Safe（`main_sim_node`），系统仍会自动拉起 `dual_joint_impedance_controller`
（`franka_example_controllers/MultiJointImpedanceController`），它直接订阅

- `/mj_left/joints_desired`、`/mj_right/joints_desired`（`sensor_msgs/JointState`）

并按关节名匹配，取 `position→q_d`、`velocity→dq_d`，内环：
`tau = K·clamp(q_d−q, ±e_q_max) + D·(dq_d−dq) + coriolis`（K=600…, D=30…, e_q_max=0.4, alpha_filt=0.01）。
注意 `dq_d` 是**阻尼参考**而非力矩前馈：
- `velocity=0`（`zero`）＝纯调节 `−D·dq`，即当前 `setZero` 型 bypass 的表现；
- `velocity=内部自洽参考速度`（`integrated`）＝ `use_integrated_reference:=true` 型；
- `velocity=J†·Δx/0.01` 且每条消息重锚（`deadbeat`）＝ 原旧版 bypass，速度误差不被 clamp，
  会复现 1471–2378 mm 的振荡发散。

### 直接打底层控制器的脚本

新增 `src/multipanda_ros2/teleop/direct_impedance_sweep.py`：不启 `main_sim_node`，
把“前移 0.01 m → 停 → 回程 → 停”直接发到 `/mj_*/joints_desired`，三种 `--vel-mode`
（`zero` / `integrated` / `deadbeat`）可逐个隔离底层跟踪效果。用法：

```bash
# 只起 sim（自动带 impedance），monitor 照常开；不要起 main_sim_node
python3 src/multipanda_ros2/teleop/direct_impedance_sweep.py \
  --step 0.002 --steps 5 --interval 0.1 --hold 6.0 --vel-mode zero
```

因为扫频是左右对称的，monitor 的 `safe_err*` 恒≈0（命令相对几何不变）；底层跟踪误差要看
`curr_rel − safe_rel`（或轴向分量），即“实际 vs 命令”的 FK 相对几何差。

---

*实验日期：2026-08-06*
