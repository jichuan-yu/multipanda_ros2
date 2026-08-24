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
>（全链 CSV `exp0X.csv` 是 2026-08 之后新增的记录方式；早期只录了
>  `*_relative_error.csv`，请以本手册步骤 C/F 的新方式为准。）

> **记录方式速览（布局：一次实验的所有 CSV 集中在 `data/mprc/<exp>/` 一个文件夹，所有图集中在 `data/mprc/<exp>_figs/`）**
> - `data/mprc/exp0X/exp0X.csv` —— **全链 CSV**（步骤 C1 `record_experiment.py`，100 Hz 一张表，
>   按列名前缀分 teleop / safe / low-level / actual 四层：`tel_raw/tel_type`（teleop 原始命令）、
>   `cmd_q/cmd_dq`（safe 命令）、`fql/fdq`（low-level 滤波）、`act_q/act_dq/act_tau/acc`（实际执行），
>   另有 `eeL/R` 位姿与距离列）。⚠️ 必须与步骤 D 遥操作**并发运行**，否则 teleop 列全 NaN。
> - `data/mprc/exp0X/exp0X_relative_error.csv` —— 相对误差（步骤 C2 `relative_error_monitor`）。
> - `data/mprc/exp0X/robot{1,2}_reference_exp0X.csv` —— controller 的 q_r 参考轨迹（步骤 B 开
>   `data_record_ON:=true` 生成，绘制 FK 图时叠加 ref 层用）。controller 的所有中间结果
>   （`HQP_result_*.csv`、`computation_time_*.csv` 等）也写进同一目录 `data/mprc/exp0X/`。
> - **所有图**统一出到 `data/mprc/exp0X_figs/`：步骤 F1/F2/F3 三个绘图脚本的 `-o` 都可省
>   （默认按新布局自动落位；`--refL/--refR/--rel` 也在同目录自动发现，无需手填）。
> - **一键出图（无需 source、不另开终端）**：`./mkfigs exp0X` 一条命令跑完
>   F1(fk 10图)+F2(经典7图)+F3(monitor快照)，全部落 `data/mprc/exp0X_figs/`；
>   recorder 停止时也默认自动出图（见步骤 C），只有手动补图时才用 mkfigs。
> - 绘图首选 **步骤 F1 `plot_experiment_fk.py`**（对全链 CSV 出 10 张 FK 分层图）。

> **可选用（底层隔离）**：如果只想看“底层阻抗控制器本身”的跟踪误差（不想经过
> `dual_joint_impedance` 上游的 Safe 层），**不起 `main_sim_node`**，改跑
> `src/multipanda_ros2/teleop/direct_impedance_sweep.py`（把参考直接发到
> `/mj_{left,right}/joints_desired`，三种 `dq_d` 策略 `--vel-mode zero|integrated|deadbeat`）。
> 用法与数据解读见 `docs/relative_error.md` 末尾“2026-08-16 补充”。

---

## 0. 前置（每个终端都执行一次）

在**桌面终端**里打开 **4 个标签页**（A:1 sim、B:2 控制器、C1:3 全链记录器（**自动拉起相对误差 monitor，Ctrl+C 停止后自动出全套图**）、D:4 遥操作；sim 需要图形显示，不要在纯 SSH 无头会话里起 sim）：

```bash
cd ~/dual_panda_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
# 如果你平时还要额外 source 别的东西（例如导出 DISPLAY），保持你平时的环境即可
```

**先清理可能残留的旧进程**（sim / 控制器 / 监视器 / 遥操作），避免端口与话题冲突：

```bash
pkill -f my_task_sim.launch.py; pkill -f main_sim_node; pkill -f relative_error_monitor_node
pkill -f record_experiment
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

> 下面每组命令都追加三行 `data_record_*` 参数：记录 q_r 参考轨迹（供步骤 F1 的 FK 图叠加
> ref 层），输出 **`data/mprc/exp0X/robot1_reference_exp0X.csv`** / `robot2_reference_exp0X.csv`
> （controller 在 `data_record_path` 下再开 `data_record_prefix` 同名子目录，中间结果
> `HQP_result_*.csv`、`computation_time_*.csv` 等也都在里面；`data_record_path` 末尾必须带 `/`）。
> `data_record_prefix:=exp0X`，每组换成当前组前缀（exp01…exp06）。

Safe 组（开 QP + 协同约束，默认也是这组值）：

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node --ros-args \
  -p bypass_qp_safety_for_debug:=false \
  -p relative_pose_constraint_enabled:=false \
  -p data_record_ON:=true \
  -p data_record_path:=/home/botao/dual_panda_ws/data/mprc/ \
  -p data_record_prefix:=exp0X
```

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node --ros-args \
  -p bypass_qp_safety_for_debug:=false \
  -p relative_pose_constraint_enabled:=true \
  -p cmd_alpha:=0.0 \
  -p data_record_ON:=true \
  -p data_record_path:=/home/botao/dual_panda_ws/data/mprc/ \
  -p data_record_prefix:=exp0X
```


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

### 步骤 C（终端 3）启动记录器（全链 CSV + 自动 monitor + 停止后自动出图）

**C1 — 全链记录器 `record_experiment.py`（终端 3，新记录方式）**

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/multipanda_ros2/teleop/record_experiment.py -o data/mprc/exp0X/exp0X
```

- 输出文件：`data/mprc/exp0X/exp0X.csv`（一表四层：`tel_raw/tel_type`（teleop 原始命令）、
  `cmd_q/cmd_dq`（safe 命令）、`fql/fdq`（low-level 滤波）、`act_q/act_dq/act_tau/acc`（实际执行），
  另有 `eeL/R` 位姿与距离列）。注意 `-o` 用 `data/mprc/<exp>/<exp>`（自动建目录），
  保证 recorder / monitor / controller 三类 CSV 都在 `data/mprc/<exp>/` 同一个文件夹。
- ⚠️ **必须与步骤 D 的遥操作并发运行**：先把这个 recorder 跑起来，保持到步骤 D 的
  「前移 → 回程」序列全部发完、最后 Ctrl+C 才停。先跑 teleop 或提前停 recorder，
  `tel_*` 列会全 NaN，FK 图就没有 teleop 层。
- recorder 是纯 python（依赖 rclpy）；若报 `No module named 'rclpy'`，用步骤 D 里的两行
  `export` 显式注入 `/opt/ros/humble/*` 路径。

**C2 — 相对误差 monitor `relative_error_monitor_node`（自动并入终端 3，不再单独开终端）**

上面的 recorder 启动后会自动在**同一终端**后台拉起 monitor，参数自动推导
（`output_path` = `data/mprc/exp0X/`、`output_prefix` = `exp0X_relative_error`），
启动日志里同样应看到 **"Initial relative position baseline set"** 以及左右 EE 坐标、
相对距离（一致构型 ≈ 0.52），这行出现后再做步骤 D。
- 输出文件：`data/mprc/exp0X/exp0X_relative_error.csv`（与 recorder / controller 的 CSV 同目录）。
- Ctrl+C 停 recorder 时 monitor 随之一并结束（文件已写完整），随后 recorder **自动跑 `plot_all`
  出全套图**进 `data/mprc/exp0X_figs/`（等价于步骤 F 的 `./mkfigs exp0X`；整链无 teleop 数据时会
  跳过并打印补图命令）。不想自动拉 monitor / 自动出图时，分别加 `--no-monitor` / `--no-plot`。
- 个别需要单独跑 monitor 的场景，手动命令仍可用：
  `ros2 run dual_arm_reactive_control relative_error_monitor_node --ros-args -p output_path:=/home/botao/dual_panda_ws/data/mprc/exp0X/ -p output_prefix:=exp0X_relative_error`。

### 步骤 D（终端 5）自动绝对任务空间遥操作（唯一驱动方式，六组 exp01–06 通用）


```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PWD}/src/multipanda_ros2:${PYTHONPATH}
python3 src/multipanda_ros2/teleop/auto_absolute_teleop_ik.py \
  --target safety --arm both --step 0.01 --steps 10 --interval 0.1 --hold 5.0
```

> 组前预置（可选）：每次实验开始前让双臂初始构型不同——一键把左臂向后 15cm、
> 向下 15cm、向左（远离右臂）15cm，跑完自动退出，无需键盘精调：

```bash
cd /home/botao/dual_panda_ws
source /opt/ros/humble/setup.bash
python3 src/multipanda_ros2/teleop/auto_absolute_teleop_preset_ik.py \
  --seq -x:15,-z:15,+y:15 --step 0.01 --interval 0.1 --hold 2.0
```

> 方向按 world 系：`-x` 向后、`-z` 向下、`+y` 向左（远离右臂）；若某方向实际
> 运动反了，改 `--seq` 里对应符号（如 `+y`→`-y`）。结束时打印 left-EE 净位移，
> 与期望 `[x:-0.150, y:+0.150, z:-0.150]` 对照即可确认。

> 备选（键盘手动驱动，一次只动一臂；需要双臂同步时仍用上面的自动脚本）：

```bash
 cd /home/botao/dual_panda_ws
source /opt/ros/humble/setup.bash
python3 src/multipanda_ros2/teleop/key_teleop_cartesian_absolute_ik.py --arm left \
    --step-position 0.01
```

### 步骤 E（收尾）检查 CSV，准备下一组

```bash
cd ~/dual_panda_ws
head -5 data/mprc/exp0X/exp0X_relative_error.csv
wc -l data/mprc/exp0X/exp0X_relative_error.csv
# 全链 CSV：确认有行、且 teleop 命令确实被录到了（tel_type 非空比例 > 0）
head -3 data/mprc/exp0X/exp0X.csv
wc -l data/mprc/exp0X/exp0X.csv
python3 -c "import pandas as pd; d=pd.read_csv('data/mprc/exp0X/exp0X.csv'); print('tel_type non-NaN:', round(100*d['tel_type'].notna().mean(),1), '%')"
```

`tel_type non-NaN` 应 > 0（录制期间步骤 D 确实在发 `/dualarm_teleop_cmd`）；若为 0，
说明 recorder 与遥操作没并发，删掉本组 `data/mprc/exp0X/exp0X.csv` 重录。



### 步骤 F（后处理）绘图 —— 一键全套（无需 source、不另开终端）

**默认方案：一条命令跑完 F1+F2+F3**（`./mkfigs` 自动 `cd` 到 workspace，无需环境变量）：

```bash
cd ~/dual_panda_ws
./mkfigs exp0X
```

- 等价于分开跑 **F1**（`plot_experiment_fk.py` → 10 张 FK 图，`01–03_fk_pos_{x,y,z}.png`、
  `04–06_fk_rel_{x,y,z}.png`、`07–08_fk_vel_{L,R}.png`、`09–10_fk_acc_{L,R}.png`）、
  **F2**（`plot_experiment.py` → 经典 7 张）、**F3**（`plot_relative_error.py` →
  `exp0X_relative_error_plot.png`）。
- `--refL/--refR/--rel` 与 `-o` 全部自动发现，图统一落 `data/mprc/exp0X_figs/`。
- recorder 停止时也会自动执行这一步（见步骤 C），这里用于补图/重画。
- 自定义图目录用 `-o`，只想跑某一步用 `--skip-fk/--skip-classic/--skip-rel`。

**底层拆分（需要单独重跑某一步时）**：

```bash
cd ~/dual_panda_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/multipanda_ros2/teleop/plot_experiment_fk.py data/mprc/exp0X/exp0X.csv
python3 src/multipanda_ros2/teleop/plot_experiment.py data/mprc/exp0X/exp0X.csv
python3 src/multipanda_ros2/teleop/plot_relative_error.py data/mprc/exp0X/exp0X_relative_error.csv
```


说明：本手册用 `type=5`（TASK_SPACE_POSE）驱动，FK 图的 teleop 层只对 `type=4` 绝对关节目标做 FK，
**图上不出现 teleop target 层属正常**；链路完整度用 safe cmd → actual + ref 层判断。
若步骤 B 没开 `data_record_ON`，ref 层自动跳过（不报错）。

---

## 附录：removefix（`096ba14`）与原版 fine（`4cadedd`）的具体区别

> **版本指认**
> - **fine（原版）** = good 控制器状态 = git `4cadedd`
>   （`fix(control): 恢复 good 控制器状态（8-17 baseline + cmd_alpha 参数化 + Fix A.3 开环代价
>   + 位置级钳位，移除 A.2 速度钳位 / A.4 加速度斜坡）`），对应备份
>   `backups/restore_good_20260823/`。
> - **removefix** = 在 fine 基础上移除开环代价（Fix A.3）与位置级钳位 = git `096ba14`
>   （`fix(control): 移除 Fix A.3 开环代价与位置级钳位（8.17 closed-loop 行为）`），对应备份
>   `backups/openloop_removed_20260824/dual_arm_safe_controller_sim.cpp`。
> - 两者**只差 `dual_arm_safe_controller_sim.cpp` 一个文件**，共 3 类差异（14+/21-）。

### 1) Fix A.3 开环代价（HQP / HardSoftQP 两个分支）
| | HQP 分支 | HardSoftQP 分支 |
|---|---|---|
| fine | `generateCost(q_c, q_r, q_dot_r, H, f)`<br>`generateConstraints2HQP(q_c, dq_c_last, constraints)` | `generateCost(q_c, ...)`<br>`generateConstraints2HardSoftQP(q_c, ...)` |
| removefix | `generateCost(q, q_r, q_dot_r, H, f)`（closed loop）<br>`generateConstraints2HQP(q, ...)` | `generateCost(q, ...)`<br>`generateConstraints2HardSoftQP(q, ...)` |

- 语义：fine 在 **INTEGRATED COMMAND `q_c`**（open loop）上评估跟踪误差与约束；removefix 改用实际关节 `q`（closed loop，8.17 行为）。
- 背景：Fix A.3 的动机是闭环误差 `(q_r - q_actual)` 驱动命令积分器后经内环滞后（τ≈0.5 s）回馈，形成失稳共轭对 `s² + (1/τ)s + K/τ`（K=4, τ≈0.5 → roots -0.5, -2），配合限幅非线性出现逐步放大的 head-shake 极限环；open loop 使命令自身一阶收敛（根 s=-K），内环只跟随。移除后回到 8.17 的闭环振荡表现。

### 2) 命令位置级钳位
- fine（有）：
  ```cpp
  q1_c_ = q1_c_.cwiseMax(robot1_.q_lb).cwiseMin(robot1_.q_ub);
  q2_c_ = q2_c_.cwiseMax(robot2_.q_lb).cwiseMin(robot2_.q_ub);
  ```
- removefix（删掉这两行）：命令积分后不再做位置级限位保护。

### 3) 记录时间戳基准（⚠️ 额外差异，非用户要求的两项改动）
| | 时间戳 |
|---|---|
| fine（及 8.17 / 017078d 等全部历史版本） | **相对** `node_->now().seconds() - start_time_`（从数据记录开始计时） |
| removefix | **绝对** `node_->now().seconds()`（绝对 sim 时钟秒） |

- 涉及 6 处：`recordData`、`recordHQPResult`、`recordHardSoftQPResult`、`recordComputationTime`、`recordTeleopTrackingData`、`recordTaskSpaceError`。
- 影响：绝对时间戳与 `record_experiment.py` 的 `t_sim` / teleop / monitor 同基准，`robot*_reference_*.csv` 可按绝对时间与其它 CSV/bag 精确对齐。
- ⚠️ **恢复到 fine 后时间戳回到相对**。若实验需要绝对时间对齐，需把上述 6 处重新改回 `node_->now().seconds()`（无 `- start_time_`）。

### 不变项（fine 与 removefix 相同）
- constraint_manager 的 Fix #1（世界系相对位置约束：两参数 `setRelativePose(T, p_rel_world)` + `e_rp_world` 分支）—— 两者都含，与 8.17 相比是后来补齐的实现端。
- `cmd_alpha` 参数化（online-tunable command mixing coefficient）。
- 恢复/保留的 `relative_error_monitor.cpp`（工作区未提交改动仅为文件头注释，与本次无关）。

