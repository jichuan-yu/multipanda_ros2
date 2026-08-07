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

*实验日期：2026-08-06*
