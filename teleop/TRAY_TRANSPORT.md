# 托盘搬运参考轨迹输入

`tray_transport_input.py` 与 `key_teleop_cartesian_absolute.py` 使用相同的输入层：

- 双臂目标只发布到 `/dualarm_teleop_cmd`，格式为 `[左xyz/xyzw, 右xyz/xyzw, 5, 3, 0]`。最后的 `0` 保持安全控制器避障开启。
- 夹爪使用 `/mj_{left,right}_gripper/grasp_desired`，通过原有夹爪桥接执行。
- 通过控制器的 `relative_pose_constraint_enabled` 参数切换协同约束。
- 不向机械臂关节控制器、MuJoCo 关节状态或 MoveIt 执行接口发命令。

这是场景几何驱动的**任务参考轨迹生成器**。它对完整路径做逆解预检，可选做离散网格碰撞预检；不是带全局搜索和动态重规划的 OMPL/cuRobo 规划器。在线实际轨迹仍由安全控制器决定。跟踪偏差再大也不会冻结尚未播完的参考轨迹，而是保留最低播放倍率，让前移的参考点给安全控制器提供新的目标；这并不保证一定能找到绕行路径。参考播完后仍需要实际满足阶段到位条件，机械臂运动始终由安全控制器决定。

## Docker / myenv 运行

实际环境为 `multipanda-container`，ROS Humble，`/home/developer/myenv`（Python 3.10）。工作区挂载在 `/home/xiaozy24/dual_panda_ws`；容器中的 `/home/developer/dual_panda_ws` 不是此代码目录。

进入容器后：

```bash
docker exec -it multipanda-container bash
source /opt/ros/humble/setup.bash
source /home/developer/myenv/bin/activate
cd /home/xiaozy24/dual_panda_ws
source install/setup.bash
```

先进行离线预览，**不启动 ROS 节点、不发运动指令**：

```bash
python src/multipanda_ros2/teleop/tray_transport_input.py \
  --check-collisions --output /tmp/tray_transport_plan.json
```

去掉 `--check-collisions` 可以只做逆解预检，不需要 Python MuJoCo。默认初始位姿由 Panda DH 模型的 home 关节角计算；在线执行时改用实际反馈。优先读取 `/ee_pose`；待机时该话题可能仅在控制器启动时发布一次，因此在其缺失或过期时，用新鲜 `/joint_states` 和一致的 Panda DH 模型计算 TCP 位姿。两种来源都过期才会中止。

也可以只读检查运行中的反馈和实际起点预检（不发运动、夹爪或参数命令）：

```bash
python src/multipanda_ros2/teleop/tray_transport_input.py --check-feedback --check-collisions
```

启动使用 `tray_transport.yaml` 生成的托盘环境、安全控制器及更新后的夹爪桥接，然后停止键盘/CSV 等其他输入源，运行：

```bash
python src/multipanda_ros2/teleop/tray_transport_input.py --execute --check-collisions
```

节点自动发现具有协同参数的控制器；多个候选时需用 `--controller-node /dual_arm_mprc_node` 指定。它监听 `/dualarm_teleop_cmd` 的实际消息，而不是按发布者数量判断：空闲键盘节点、残留的 DDS 发布端只提示，不阻塞启动；真正收到其他输入命令时才停止本任务。自身消息在 `layout.dim[0].label` 中附带随机实例标识，17 个控制数值不变，安全控制器无需修改；这也兼容不向 Python 回调提供发布者 GID 的 ROS Humble。报告中的其他发布端名称/GID 是候选来源列表，未知名称也不直接等同于活跃输入。

这是一项输入冲突检测，不是控制器端的独占仲裁；它不能撤回另一输入已经发给控制器的第一条命令。因此搬运时仍不要同时操作键盘或播放 CSV。宿主机 Jazzy 与容器 Humble 共用 ROS 域时，可能同时看到未知节点及 DDS 反序列化警告；建议所有任务输入统一在容器内运行，并关闭宿主机遗留的键盘输入。

挡板尺寸修改后必须重启仿真以重新加载 `objects.xml`；输入节点不会热修改正在运行的物理环境。源目录和当前工作区 `install/franka_description/share/franka_description/mujoco/franka/objects.xml` 已同步生成。

场景应从 YAML 所描述的初始位置开始：当前版本不从视觉或物体位姿话题估计已经移动的托盘。中断后不要直接重跑一个已经搬到中途的托盘。

## 点位与阶段

配置位于 `tools/env_config/tray_transport.yaml` 的 `tray_transport_input` 段。坐标单位为米，全部目标使用 `/ee_pose` 对应的世界坐标和 `panda_ee_site` TCP。

按世界 ±Y 选挡板。当前托盘旋转 90°，因此左臂夹 `back_wall`，右臂夹 `front_wall`。四块挡板均高 4 cm（底面仍为 z=0.21，上沿为 z=0.25）。指垫中心对准挡板长边与高度的中央。TCP 高度根据机器人 XML 中的手指根部、指垫中心和 TCP 偏移计算，夹爪闭合轴对准挡板法向。夹爪的两个相差 180° 的等价朝向按每臂实际初始姿态选择最近者，避免绕工具轴半转。中央抓取下，竖直姿态在较远臂的高处不可达；默认 `grasp_tilt_degrees: 10` 沿挡板向前倾斜 10°，使手腕更靠近基座。该角度已通过离线接触和可达性测试；不是绕工具轴自转。

默认生成的点位：

| 阶段 | 左臂 xyz | 右臂 xyz | 协同 |
|---|---|---|---|
| 准备位置 | (0.6001, 0.495, 0.3094) | (0.6001, 0.205, 0.3094) | 关 |
| 张开后下移至夹取位置 | (0.6001, 0.495, 0.2294) | (0.6001, 0.205, 0.2294) | 关 |
| 闭合并确认夹取 | 同上 | 同上 | 确认完成后开 |
| 抬升至搬运准备位置 | (0.6001, 0.495, 0.4394) | (0.6001, 0.205, 0.4394) | 开 |
| 大板凳上方准备位置 | (0.6001, -0.205, 0.4394) | (0.6001, -0.495, 0.4394) | 开 |
| 下移放置 | (0.6001, -0.205, 0.3794) | (0.6001, -0.495, 0.3794) | 开 |
| 张开夹爪 | 同上 | 同上 | 张开完成后关 |
| 上移结束 | (0.6001, -0.205, 0.4394) | (0.6001, -0.495, 0.4394) | 关 |

初始点由实际反馈确定。搬运高度取起始、目标支撑面中较高者再加 6 cm，所以从低凳名义抬升 21 cm，为横移保留净空。6 cm 的搬运/撤离高度同时给挡板上偏最多约 18 mm 的有效夹取保留可达余量；原 8 cm 在中央抓取、实际夹点偏高时会再次超过可达边界。抓取完成后，用当时实际双臂位姿重新锚定搬运参考，并给两臂施加相同平移，避免名义位姿误差与控制器捕获的相对位姿约束互相冲突。表格是名义值；在线点位会包含这项毫米级修正。

目前支持水平托盘/座面及绕竖直轴的旋转；不支持翻转托盘或倾斜座面。修改环境后预检不通过时，应修改任务几何或环境，不能忽略失败继续执行。

## 播放速度和到位判据

每段采用短时平滑加速、匀速、短时平滑减速，两臂共用虚拟时间；起止速度和加速度为零。加速和减速各默认 0.5 秒虚拟时间，短距离段降低峰值速度。名义速度上限：

- 空载接近/撤离：4 cm/s。
- 托盘横移：2.5 cm/s。
- 夹取下降、抬升、放置下降：0.8 cm/s。
- 姿态变化上限：0.25 rad/s。

根据**两臂中较大的实际跟踪误差**缩放时间：位置误差 ≤5 mm 时正常播放，5–25 mm 区间计算减速；旋转误差对应 0.03–0.15 rad。两种限制取更严格者，但倍率不会低于 `tray_transport_input.min_playback_scale`，默认 **0.2**。因此即使实际位置落后几十厘米，参考点也会以至少名义时间推进速率的 20% 继续前移，给安全控制器寻找绕行路径的机会。

最低倍率必须在 `(0, 1]` 内；设为 `1.0` 即不因跟踪误差减慢虚拟时间。起步使用短时速度渐变，跟踪恢复保留倍率渐变，段首/段尾仍保留零速度；“20%”指虚拟时间倍率，不是每时刻笛卡尔速度都恒定。日志分别报告当前参考点的 `tracking_error` 和阶段终点的 `endpoint_error`，避免把两种误差混淆。

切换相关参数位于 YAML 的 `tray_transport_input`，均须为正数：

| 参数 | 默认值（秒） | 用途 |
| --- | --- | --- |
| `motion_ramp_time` | 0.5 | 每段加速、减速各自时长 |
| `arrival_settle_time` | 0.1 | 普通到位确认 |
| `grasp_settle_time` | 0.25 | 可夹取区域内稳定确认 |
| `gripper_settle_time` | 0.15 | 夹爪反馈稳定确认 |
| `cooperation_settle_time` | 0.05 | 协同参数切换后的等待 |

以 23 cm 抬升、速度上限 0.8 cm/s 为例，无跟踪减速时名义播放时长由旧版约 54 秒缩短至 29.25 秒，前 3 秒参考位移由约 0.36 mm 增至 22 mm。实际跟踪误差仍可减慢播放或延长到位等待；阶段边界仍停车并确认反馈，不跨越夹爪动作连续插值。

接近、最终放置和撤离要求轨迹播放完毕、两臂实际位置误差 <3 mm、姿态误差 <0.025 rad，并连续满足 0.1 s（`arrival_settle_time`）。

`lift` 与 `transport` 是中间路标，不再要求精确到达 XYZ，也不必等该段参考播到末端。实际位姿与当前参考都满足以下条件 0.1 s 即接续下一段参考（不会跳变参考位置）：

- `lift → transport`：从两臂相对放置目标的位置估计托盘底部高度，扣除姿态偏差引起的边角下沉后，高于大板凳至少 `transit_min_clearance`（默认 2.5 cm）；姿态误差不超过 `transit_orientation_tolerance`（默认 0.05 rad）。
- `transport → place`：满足上述净空和姿态条件，且两臂各自推算的托盘中心与凳子中心 XY 距离不超过 `transport_xy_tolerance`（默认 3 cm），位于座面边界内至少 3 cm。剩余横向误差在放置段继续修正。

这些判据使用实际抓取后的锚定目标，保留高位夹取偏移；仅允许继续移动，不允许提前释放。未达到净空或放置区域时仍等待，并输出 `transit_gate` 的具体误差。超时信息也包含未满足的几何条件。

夹取下降使用几何可夹取区域，不要求精确到达名义 Z，也不要求虚拟轨迹已播完。两臂均须满足：

- 主指垫与挡板的保守竖直重叠高度 ≥ `grasp_min_overlap`，默认 8 mm。
- XY 偏差 ≤3 mm，姿态偏差 ≤0.025 rad；不接受低于原目标超过 3 mm 的下冲。
- 连续 0.25 s（`grasp_settle_time`）内实际位姿保持稳定（相对观察起点的位置变化 ≤0.8 mm、姿态变化 ≤0.008 rad）；经过区域但仍明显运动时不算到位。

当前挡板高 40 mm、主指垫高 17 mm，理想竖直姿态下允许高约 20.5 mm；默认 10° 倾斜时，保守重叠计算将范围收紧到约 18.8 mm。若安全控制器因板凳避障停在此范围内，可直接进入闭合，不会一直等待原目标点。接受时将发布目标冻结在实际位姿，随后仍检查夹爪动作结果、宽度和夹持力。搬运参考从实际夹取位姿接续，因此较高的接触点偏移也会保留至放置阶段，不会丢失该偏移而把托盘压入凳面。

可夹取区域判据仅用于夹取下降；中间搬运阶段另用上述净空判据。放置不会因高于凳面而提前释放；没有足够接触重叠、只有一臂到位或横向/姿态不合格时仍不进入闭合。日志会输出 `grasp_region` 和每侧 `overlap`，可区分有效夹取范围与真正不可达。计时使用仿真时钟。参考仍在播放时，不因大跟踪误差或耗时较长而触发跟踪超时；参考播完后，才开始 `--stage-timeout`（默认 120 秒仿真时间）的实际到位等待。到位前不跨越夹取/放置等动作边界。大幅时钟跳变、反馈过期、外部命令冲突或持续失去夹持力仍会停止任务。

## 夹爪完成反馈

`gripper_action_bridge.cpp` 新增 `/mj_{left,right}_gripper/grasp_status`，格式为 `[request_id, state]`：`1` 已接受，`2` 成功，`-1` 未接受/服务器不存在，`-2` 执行失败。原有五元素夹爪输入保持兼容；自动任务追加第六元素请求编号，排除旧动作结果。

输入节点同时检查 `/mj_{left,right}_gripper_sim_node/joint_states` 中的宽度与力。闭合成功必须有夹持宽度和至少 3 N 的力，持续稳定 0.15 s（`gripper_settle_time`）；空夹即使动作返回成功也不会抬升。释放必须实际张开到目标宽度附近。异常或 Ctrl-C 不会自动松开托盘或关闭已开启的协同约束；正在执行的夹爪动作会收到停止请求。

修改桥接后需重建，已启动的桥接进程需在下一次启动仿真时加载新版。容器内可只构建和安装这个可执行文件：

```bash
colcon build --base-paths src/multipanda_ros2/franka_example_controllers \
  --packages-select franka_example_controllers --cmake-target gripper_action_bridge \
  --cmake-args \
  -DFranka_DIR=/home/developer/Libraries/libfranka/lib/cmake/Franka \
  -Dmujoco_DIR=/home/developer/Libraries/mujoco/lib/cmake/mujoco \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
install -m 755 build/franka_example_controllers/gripper_action_bridge \
  install/franka_example_controllers/lib/franka_example_controllers/gripper_action_bridge
```

## 验证范围

离线预检每约 5 mm 或 0.04 rad 采样逆解。`--check-collisions` 在独立 MuJoCo 模型中检测名义逆解分支的网格接触，包含搬运时平移的托盘与 `cube_` 命名负载；允许夹爪与托盘、负载内部的预期接触，拒绝超过 1 mm 的其他穿透。它不模拟负载滑移，也不是连续碰撞证明，不能保证实际安全控制器选择的关节轨迹与该逆解分支一致。

测试：

```bash
python -m unittest discover -s src/multipanda_ros2/teleop/tests -v
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python src/multipanda_ros2/teleop/tests/ros_tray_smoke.py
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python src/multipanda_ros2/teleop/tests/ros_tray_smoke.py --reject-close
```

此外可用 `ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1 python src/multipanda_ros2/teleop/tests/ros_gripper_bridge_smoke.py` 检查实际编译的夹爪桥接对成功、失败、拒绝和旧五元素消息的处理。

待机末端反馈回归可在 `ros_tray_smoke.py` 命令后添加 `--idle-no-ee --check-feedback`，验证没有 `/ee_pose` 时仍可用关节反馈完成只读预检；`--idle-no-ee --reject-close` 验证从待机进入执行流程。

ROS 测试在专用域中使用合成反馈，验证消息格式、任务顺序和约束开关，不是物理搬运实验。真实安全控制器中的碰撞距离阈值仍可能使末端停在有效夹取区域之外，或停在放置位置之前；此时程序会报告跟踪超时，需要结合实际日志核对几何模型与阈值。

夹取避障到位回归：在隔离域运行 `ros_tray_smoke.py --grasp-stop-offset 0.010`，模拟双臂下降时被挡在名义目标上方 10 mm，验证仍完成夹取、搬运和释放顺序。

输入冲突回归：在隔离域分别运行 `ros_tray_smoke.py --competitor idle --check-feedback`、`--competitor active`、`--competitor late`，验证空闲发布端不阻塞、实际外部命令在启动前/运行中被检出；`--competitor idle --grasp-stop-offset 0.010` 验证自身消息回送不会误判并仍可完成任务。

大误差继续播放回归：在隔离域运行 `ros_tray_smoke.py --detour-release-distance 0.080`，模拟实际机械臂停在起点，只有参考领先至少 80 mm 后才恢复跟随，验证大误差不再冻结参考并可完成任务。
