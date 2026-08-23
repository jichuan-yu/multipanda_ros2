# 相对误差复现实验 · 分步操作手册（手动模式）

> 复现 `docs/relative_error.md` 的 6 组实验：双臂绝对式任务空间遥操作，向前 0.01 m
> 再返回，监控 teleop / safe 相对位姿误差并绘图。
>
> 驱动方式**只用自动绝对任务空间遥操作** `auto_absolute_teleop.py`（六组 exp01–06 通用）。
> 它是键盘绝对版 `key_teleop_cartesian_absolute.py` 的自动化孪生：发相同的
> `command_type=5`（TASK_SPACE_POSE）指令到 `/dualarm_teleop_cmd`、`arm_selector=BOTH`，
> monitor 只对 type=5 有意义地算 `teleop_err`；命令里左右手相对几何不变 → `teleop_err ≈ 0`，
> 与 error.md 语义一致。脚本启动时把命令基**锚定到真实 EE**（首帧 `/joint_states` 的 FK，
> 与 monitor 基线同源），因此一致/不一致（exp03/06）构型都能直接驱动、不跳变、保持相对几何。
>
> 脚本一次跑完「前移 → 停 → 回程 → 停」序列：`--step 0.002 --steps 5` ⇒ 左右 EE 同步
> +0.01 m，`--back` 自动返回原点；静止期间以 1 Hz 重发当前目标，控制器 `teleop_timeout=999 s`
> 不会退回 STOPPING。每组的快慢只由步骤 D 的 `--interval` 决定（缓速组 2.0 / 快速组 0.1）。
>
> 注意（bypass 组）：绝对位姿命令在 `bypass_qp_safety_for_debug:=true` 下
> 速度 = `Jᶜⁱ·(T_target−T_current)/0.01`、**没有任何限幅**，锚点与真实 EE 稍有偏差就会
> 完全失控。务必等脚本打印 `[AUTO_TELEOP] anchor` 后再移动；换组前先按第 0 节清理、等
> 机器人稳定到目标初始构型再重锚。
>
> 这批结果输出到 **`data/mprc/`** 下，前缀统一用 `exp01_…` ~ `exp06_…`。
> 注意：`data/mprc/20260813_*.csv`（约 24MB / 17MB）是失败/NaN 的旧数据，请忽略。

> **可选用（底层隔离）**：如果只想看“底层阻抗控制器本身”的跟踪误差（不想经过
> `dual_joint_impedance` 上游的 Safe 层），**不起 `main_sim_node`**，改跑
> `src/multipanda_ros2/teleop/direct_impedance_sweep.py`（把参考直接发到
> `/mj_{left,right}/joints_desired`，三种 `dq_d` 策略 `--vel-mode zero|integrated|deadbeat`）。
> 用法与数据解读见 `docs/relative_error.md` 末尾“2026-08-16 补充”。

---

## 0. 前置（每个终端都执行一次）

在**桌面终端**里打开 4 个标签页（sim 需要图形显示，不要在纯 SSH 无头会话里起 sim）：

```bash
cd ~/dual_panda_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
# 如果你平时还要额外 source 别的东西（例如导出 DISPLAY），保持你平时的环境即可
```

**先清理可能残留的旧进程**（sim / 控制器 / 监视器 / 遥操作），避免端口与话题冲突：

```bash
pkill -f my_task_sim.launch.py; pkill -f main_sim_node; pkill -f relative_error_monitor_node
pkill -f auto_absolute_teleop; pkill -f key_teleop_cartesian_absolute.py; pkill -f key_teleop_cartesian.py
pkill -f replay_teleop_cmd; pkill -f record_teleop_cmd; pkill -f record_ee_pose
pkill -f csv_teleop_cartesian_absolute; pkill -f gen_anchored_sweep; sleep 2
```

（没有残留进程时提示 `process not found` 属正常，直接忽略；若 sim 曾非正常退出，可再加 `pkill -9 -f` 强制清理。）

确认话题能通（起过一次后可用）：

```bash
ros2 topic list | grep -E 'joint_states|joints_desired'
```

---





### 步骤 A（终端 1）启动仿真器 sim

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=false
```

- **构型（关节角度）由 initial_positions_1/2 决定**，必须整条用**单引号**包住 `name:=value`，
  值内部还要保留一对**双引号**（这是 `my_task_sim.launch.py` 的 Command 拼接要求，
  否则 ros2 launch 会把空格拆开，xacro 报 `error: no such option: -0`）。
- 每个构型换参数即可：

| 构型 | initial_positions_1 | initial_positions_2 |
|------|--------------------|--------------------|
| 一致（symmetric） | `"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"` | `"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"` |
| 不一致（asymmetric，右臂） | `"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"` | `"0.0 -1.100 0.0 -2.800 0.0 1.900 0.900"` |

等待 `joint_states` 有频率（约 30~60 s；等不到就再等一会儿）：

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic hz /joint_states
```

出现形如 `average rate: 1000.XXX` 即可 Ctrl-C 退出 hz。

### 步骤 B（终端 2）启动安全控制器 main_sim_node

Safe 组（开 QP + 协同约束，默认也是这组值）：

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node --ros-args \
  -p bypass_qp_safety_for_debug:=false \
  -p relative_pose_constraint_enabled:=false
```

cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node --ros-args \
  -p bypass_qp_safety_for_debug:=false \
  -p relative_pose_constraint_enabled:=true \
  -p cmd_alpha:=1.0


Bypass 组：

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node --ros-args \
  -p bypass_qp_safety_for_debug:=true \
  -p relative_pose_constraint_enabled:=false
```

等待期望关节命令话题就绪：

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic hz /mj_left/joints_desired
ros2 topic hz /mj_right/joints_desired
```

### 步骤 C（终端 3）启动 relative_error_monitor（写 CSV）

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dual_arm_reactive_control relative_error_monitor_node --ros-args \
  -p output_path:=/home/botao/dual_panda_ws/data/mprc/ \
  -p output_prefix:=exp01_relative_error
```

- 输出文件：`data/mprc/exp01_relative_error.csv`（`output_path + output_prefix + ".csv"`）。
- 启动日志里应看到 **"Initial relative position baseline set"** 以及左右 EE 坐标、
  相对距离（一致构型 ≈ 0.52）。这行出现后再做步骤 D。

### 步骤 D（终端 4）自动绝对任务空间遥操作（唯一驱动方式，六组 exp01–06 通用）


```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PWD}/src/multipanda_ros2:${PYTHONPATH}
python3 src/multipanda_ros2/teleop/auto_absolute_teleop_ik.py \
  --target safety --arm both --step 0.01 --steps 10 --interval 0.1 --hold 5.0
```

 cd /home/botao/dual_panda_ws
source /opt/ros/humble/setup.bash
python3 src/multipanda_ros2/teleop/key_teleop_cartesian_absolute_ik.py --arm left \
    --step-position 0.01

### 步骤 E（收尾）检查 CSV，准备下一组

```bash
cd ~/dual_panda_ws
head -5 data/mprc/exp01_relative_error.csv
wc -l data/mprc/exp01_relative_error.csv
```



### 步骤 F（后处理）绘图

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/multipanda_ros2/teleop/plot_relative_error.py \
  data/mprc/exp01_relative_error.csv -o data/mprc/exp01_relative_error_plot.png
```

---

## 2. 六组实验对照表

| # | 构型 | interval(s) | bypass | RELATIVE_POSE | 输出前缀 |
|---|------|------------|--------|--------------|----------|
| 1 | 一致 | 2.0 | false | true | exp01 |
| 2 | 一致 | 0.1 | false | true | exp02 |
| 3 | 不一致 | 0.1 | false | true | exp03 |
| 4 | 一致 | 2.0 | true | false | exp04 |
| 5 | 一致 | 0.1 | true | false | exp05 |
| 6 | 不一致 | 0.1 | true | false | exp06 |

> `interval(s)` 是步骤 D 自动脚本的步进间隔（缓速 2.0 / 快速 0.1），直接作为
> `--interval` 参数；六组之间只改这个值和 A/B 里的开关参数。

每组按第 1 节的 A→B→C→D→E→F 走一遍，把步骤 C 里的 `exp0X` 前缀、A/B 里的参数、
步骤 D 里的 `--interval` 都换成表内值。
预计每组 2~3 分钟，6 组共 15~20 分钟。

## 3. 汇总（跑完第 2 节全部实验之后）

一键统计 6 组 CSV 的峰值误差，复制到终端：

```bash
cd ~/dual_panda_ws && python3 - <<'PY'
import pandas as pd, glob, os, numpy as np
files = sorted(glob.glob('data/mprc/exp0*_relative_error.csv'))
print(f"{'file':38s} {'rows':>7s} {'NaN%':>6s} {'teleop_max_mm':>13s} {'safe_max_mm':>12s}")
for f in files:
    df = pd.read_csv(f)
    n_nan = int(df[['teleop_err_norm','safe_err_norm']].isna().sum().sum())
    tot = df[['teleop_err_norm','safe_err_norm']].size
    print(f"{os.path.basename(f):38s} {len(df):7d} {100*n_nan/tot:5.1f}% "
          f"{np.nanmax(df['teleop_err_norm'])*1e3:10.3f} {np.nanmax(df['safe_err_norm'])*1e3:10.3f}")
PY
```

画六合一对比图（可选）：

```bash
cd ~/dual_panda_ws && python3 - <<'PY'
import pandas as pd, matplotlib.pyplot as plt, glob, os
fig, axes = plt.subplots(2,3, figsize=(18,8), sharex=True)
for ax, f in zip(axes.ravel(), sorted(glob.glob('data/mprc/exp0*_relative_error.csv'))):
    df = pd.read_csv(f)
    df['time'] = df['timestamp'] - df['timestamp'].iloc[0]
    ax.plot(df['time'], df['teleop_err_norm']*1e3, label='teleop', lw=1.2)
    ax.plot(df['time'], df['safe_err_norm']*1e3, label='safe', lw=1.2)
    ax.set_title(os.path.basename(f)[:8]); ax.legend(); ax.grid(alpha=.3)
    ax.set_xlabel('t(s)'); ax.set_ylabel('mm')
plt.tight_layout(); plt.savefig('data/mprc/exp01-06_compare.png', dpi=130); print('saved exp01-06_compare.png')
PY
```

---

## 4. 常见问题

| 现象 | 原因 / 处理 |
|------|------------|
| 在 `~` 或其他目录复制命令，报找不到包 / `YAML::BadFile` / 找不到 yaml | 控制器按相对路径读 ./src/... 配置，必须先 `cd ~/dual_panda_ws`。本手册每段命令已内置 cd，整段复制即可 |
| teleop 报 `ModuleNotFoundError: No module named 'rclpy'` | 漏了 ROS 底层环境。先 `source /opt/ros/humble/setup.bash` 再 `source install/setup.bash`。本手册窗口型命令块已统一内置两步 source，整段复制即可 |
| 已带双 source 仍报 `No module named 'rclpy'`（终端不加载环境变量） | 你的终端没有真正载入 humble。用步骤 D 命令块里的两行 `export` 显式注入 `/opt/ros/humble/*` 路径（保底写法，已验证），随后手动注入后可先跑 `python3 -c "import rclpy"` 自检 |
| launch 报 `xacro: error: no such option: -0` | initial_positions 传参丢引号。按本手册写：外层单引号 + 值内双引号 |
| mujoco 报 `Failed to initialize GLFW` | 在无图形终端（纯 SSH）起 sim。改用桌面终端，或 `xvfb-run -a ros2 launch ...` |
| teleop 打印 `FAIL: no /joint_states within 20s` | sim 还没就绪，先确认 `ros2 topic hz /joint_states` 有输出再跑 |
| teleop 一直收不到目标/手臂不动 | 步骤 A/B 的话题都就绪后再跑 C/D；顺序别反 |
| CSV 前 1-2 行 NaN | 正常（monitor 等第一条 teleop/safe 命令），建议从第 3 行起统计 |
| 旧掉线后 monitor 不再写 | Ctrl-C 该组 monitor 进程，下组用新前缀重新起 |
| 遥操作脚本报 `ModuleNotFoundError: No module named 'rclpy'` | 用步骤 D 命令块里两行 `export` 显式注入 `/opt/ros/humble/*` 路径（已验证） |
| 自动脚本开跑时手臂先「跳」一段 / 猛冲 | 锚点 ≠ 真实 EE（多半 sim 尚未稳定到初始构型就起了脚本，或旧进程残留）。按第 0 节清理重启 sim，等 `joint_states` 稳定、`[AUTO_TELEOP] anchor` 的值与真实 EE 一致后再跑 |
| bypass 组一开就失控/猛冲 | 绝对命令 bypass 下速度=`Jᶜⁱ·(T_target−T_current)/0.01`、无任何限幅，锚点必须等于真实 EE。务必等 `[AUTO_TELEOP] anchor` 打印（且锚定值 == 真实 EE）后再移动；锚定要在机器人稳定到初始构型后 |
| exp03/06（非一致构型）驱动不正常 | 自动绝对脚本启动时锚定真实 EE，非一致构型同样适用，直接跑即可；不要用增量版 `key_teleop_cartesian.py` / `auto_increment_teleop.py`（command_type=2，monitor 会误当关节增量，`teleop_err` 无意义） |
---

## 5. 2026-08-17 · Fix A：safe 模式慢速遥操作修复（命令积分）

### 现象（复现确认）

safe 模式（QP 开）比 bypass 明显拖沓：3 s×0.03 m 前移在数据里只有 ~0.02 rad/s，
回程尾段只剩 ~0.0002 rad/s 蠕动；而 bypass 同命令 ~0.1+ rad/s 跟手。

### 根因（诊断已排除 QP/clipping）

- QP 输出一直是**有效且未裁剪**的：`x=-f`、`|x|=|f|`，`f = -(dq_r + K·(q_r−q))`
  每帧存在（复现目标下 `|f|≈0.44`）。
- 但命令生成处旧实现是**每帧锚定实际**：

  ```cpp
  double alpha = 1.0;
  q1_c_ = (1.0-alpha)*q1_c_ + alpha*q1_ + q1_dot_c_*control_period_;
  // = q + q_dot_c*T（alpha=1 ⇒ 无记忆，命令永远只领先实际 ~0.0025 rad）
  ```

  内环位置跟踪项只看到 `k·0.0025 ≈ 1.5 N·m` 的微推力 → 整个 safe 链路变成蠕动。
  与内环 `alpha_filt=0.01`、悬停时 sim 被暂停等**无关**（bypass/safe 对比已排除）。

### 修复（Fix A：命令真正积分）

文件与内容（备份见文末）：

1. `src/dualarm_mprc/dualarm_reactive_control/src/dual_arm_safe_controller_sim.cpp:727-736`

   ```cpp
   // 命令位置 = QP 输出的有界速度的积分，不再每帧锚定到实际状态 q_1/q_2。
   q1_c_ += q1_dot_c_ * control_period_;
   q2_c_ += q2_dot_c_ * control_period_;
   // 位置级 clamp 到关节限位，防止命令积分越限。
   q1_c_ = q1_c_.cwiseMax(robot1_.q_lb).cwiseMin(robot1_.q_ub);
   q2_c_ = q2_c_.cwiseMax(robot2_.q_lb).cwiseMin(robot2_.q_ub);
   ```

2. `config/control_parameters_sim.yaml`：`K_joint_tracking: 4.0 → 8.0`
   （命令推进速度 ≈ `K·(q_r−q)`；K=8 时 0.03–0.09 rad 误差 → 0.25–0.7 rad/s）

语义不变：`q_dot_c` 仍是受 CBF/限位/相对姿态约束裁剪后的 QP 输出，
积分后约束**直接作用于命令位置**；STOPPING/TRACKING 已有 `q_c = q` 重新锚定，
`dev = q−q_c` 稳态 ≈ −0.03…−0.08 rad，远低于 0.3 阈值，`LARGE_DEVIATION` 兜底保留。

### 预期效果（以本次实测内环带宽 ~0.1–0.18 rad/s 估算）

| 指标 | 修复前（实测） | 修复后（预期） |
|---|---|---|
| 前向命令推进 | 0.013–0.022 rad/s | ≈0.25–0.7 rad/s |
| 0.03 m（≈0.09 rad）到位 | 4–7 s 仍爬不到 | ≈0.5–1.5 s |
| 回程尾段 | 0.0002 rad/s、>10 s 蠕动 | <0.001 rad@1.5–2 s |
| 与 bypass 差距 | 5–10× 慢 | ≈1.2–2× |

### 复测步骤

沿用本手册第 0–2 节（safe 组 exp01–03；bypass 组 exp04–06 应无回退），
重点看 `act` 命令/实际速度是否提升到 0.1+ rad/s、到位后是否停在 <0.001 rad，
`<dir>/ctrl.log` 无 `LARGE_DEVIATION` / `QP_SOLVER_ERROR`。

### Fix A.2（2026-08-17）：命令速度限幅 —— 修复“只过去不回来”回归

首次实跑 `--step 0.01 --steps 10 --interval 0.1 --hold 5.0` 发现问题：**前移正常、回程只动一点就停**。

**根因**：Fix A 后命令以 QP 速度 `K·(q_r−q)` 积分，上限可达 `dq_ub=2.5 rad/s`。回程掉头瞬间
命令大幅往回推进，而实际（内环带宽实测 ~0.1–0.18 rad/s）跟不上 → `dev = q−q_c` 冲破 0.3 阈值
→ `LARGE_DEVIATION`（cpp:762）→ 命令停发、走 `TELEOPERATING → REACTING → STOPPING`，机械臂停在半途。

**修复**：命令积分前对 `q_dot_c` 每关节限幅。新增可调参数 `cmd_velocity_limit`（默认 **0.2 rad/s**，sim/exp yaml 已同步）：

```cpp
q1_dot_c_ = q1_dot_c_.cwiseMax(-cmd_velocity_limit_).cwiseMin(cmd_velocity_limit_);
q2_dot_c_ = q2_dot_c_.cwiseMax(-cmd_velocity_limit_).cwiseMin(cmd_velocity_limit_);
```

- 声明/读取：`cmd_velocity_limit_ = 0.2`（header）、`declare_parameter`、每个控制循环 `get_parameter`（可在线调：`ros2 param set <node> cmd_velocity_limit 0.25`）。
- 限幅后命令始终处于内环可跟随速度范围，掉头偏差保持 ~0.05–0.1 rad；0.1 m（≈0.3 rad）回程约 1.5–2 s 完成，仍比修复前快 5–10 倍。
- **注意**：`cmd_velocity_limit` 是 Fix A.2 新增量，之前的 `.bak` 备份仍是 Fix A 修复前原样；Fix A + A.2 的完整差异见 `git diff`。

### Fix A.3（2026-08-17）：QP 代价/约束改 open-loop —— 修复“摇头晃脑幅度越来越大”（发散振荡）

A.2 后复现实验出现更严重的振荡：命令来回抽搐、幅度递增直到发散。

**根因**：Fix A 后命令以 QP 速度持续积分，但 QP 代价用的误差是 **`K*(q_r − q)`（实际位形闭环）**：
命令速度由“实际”驱动，实际又去“跟随命令”——二者经内环跟踪滞后 τ 形成**二阶耦合反馈环**。
特征方程 `s² + (1/τ)·s + K/τ`，带 K=8、τ≈0.5s → 复根 **−1 ± j3.9**（强欠阻尼），加上
速度 clamp 的非线性 → 能量逐周期注入 → “摇头晃脑幅度越来越大”。A.2 只压了振幅、没切断反馈。

**修复**：QP 代价和约束的误差/位形基点从**实际 q 改为命令位形 q_c（open-loop）**：

```cpp
constraint_manager_.generateCost(q_c, q_r, q_dot_r, H, f);              // open loop (Fix A.3)
constraint_manager_.generateConstraints2HQP(q_c, dq_c_last, constraints); // open loop (Fix A.3)
```

- 命令动力学变为 `dq_c/dt = K·(q_r − q_c)`（限幅 0.2）→ **一阶稳定**（根 −K），命令自主单调收敛到目标，
  实际只被动跟随 —— 与 bypass/集成参考已验证稳定的“命令自洽”架构同原理；
- 约束（关节限位、CBF 碰撞）同样基于命令位形 q_c（命令防越界；CBF 用“超前命令位形”计算 → 保守安全）；
- `use_task_space_cost_`（TASK_SPACE_POSE 专用）分支保持不变；
- **两种求解器都已同步**（HQP 与 HardSoftQP）。

**影响与注意**：
- 命令收敛到目标的固有耗时 = 距离/0.2（clamp），0.1 rad 步进约 0.5–0.7 s，内环随动到位共 ~1.5 s；
- **验证时请确认控制器跑的是 QP 路径**：`ros2 param set <main_sim_node> bypass_qp_safety_for_debug false`
  后再发复现命令（bypass=true 时不走 QP、A.3 不生效）；
- 若目标穿过安全约束（CBF），命令会停在约束前，实际随之停稳（保守安全，不会振荡）。

### 备份（修改前原样）

- `dualarm_reactive_control/src/dual_arm_safe_controller_sim.cpp.bak`
- `dualarm_reactive_control/config/control_parameters_sim.yaml.bak`
- `dualarm_reactive_control/config/control_parameters_exp.yaml.bak`
| 自动脚本打印 `[AUTO_TELEOP] done` 后手臂停在终点 | 正常，脚本已发完前移+回程全部序列，机器人保持末位姿；换下一组前按第 0 节清理 |