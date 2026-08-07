# Franka ROS2 仿真运行指南 (双臂 + 安全控制器 + 遥操作)

本文档说明了如何从进入 Docker 容器开始，编译、运行仿真，并使用遥操作脚本通过 ROS 2 Topic 控制双臂 Panda 机器人运动。

## 前置准备

启动 Docker 容器时，为了确保能看到仿真界面并与容器外进行 ROS 2 通信，请运行以下完整的 Docker 启动命令（如果已有容器在运行请先停止并删除旧的，然后重启）：

```bash
docker run -it -d --name multipanda-container \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/xiaozy24/dual_panda_ws:/home/xiaozy24/dual_panda_ws \
  -w /home/xiaozy24/dual_panda_ws \
  build-env:multipanda_ros2-amd64 bash
```

run 指令后容器自动处于已启动状态。

此后再次使用该容器时，按如下指令启动该容器：

```bash
docker start multipanda-container
```

---

## 终端 1：启动带相机的自定义仿真

在第一个终端中，进入容器并启动双臂 MuJoCo 仿真。

```bash
docker exec -it multipanda-container bash
# 跳过复杂的碰撞库，如有需求单独编译
colcon build --packages-skip nvblox coal
source install/setup.bash
# 启动带有自定义环境 and 多相机渲染的任务仿真
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true
```

启动后，你应该能看到包含两个相机零件(STL)的 MuJoCo 环境弹开，并且机器人加载在里面。同时 MuJoCo 会通过离屏渲染持续向 ROS 2 发布 `fixed_cam`、`left_arm_cam`、`right_arm_cam` 三路图像。

---

## 终端 2：流畅查看相机视频流 (Web 服务方案)

由于跨 Docker 容器向宿主机进行 X11 图形界面转发（如 OpenCV、rqt）会死锁带宽，导致高帧率图片产生极其严重的卡顿，因此我们采用将本网络变为网页视频流的大方向硬件解耦方案：

```bash
docker exec -it multipanda-container bash
source install/setup.bash
# 如果尚未安装，请运行: sudo apt-get update && sudo apt-get install -y ros-humble-web-video-server
ros2 run web_video_server web_video_server
```

启动服务后，请打开宿主机上的浏览器（Chrome/Edge 等），在地址栏输入：**http://localhost:8080**

网页中会自动汇总出当前 MuJoCo 发布的所有图像话题。您只需点击 `/mujoco_server/cameras/left_arm_cam/rgb/image_raw` 等名称链接，即可在浏览器页卡中获得极度丝滑无阻滞的相机推流画面！

---

## 终端 3：碰撞体可视化发布节点

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control collision_env_visualizer_node --ros-args -p base_frame:=world
```

启动发布节点后，在 rviz2 的图形化界面中点击 Add，在 By topic 面板选择 `/collision_env_markers` 的 MarkerArray 添加到可视化区域。

---

## 终端 4：运行双臂安全控制器 (dualarm_mprc)

### 碰撞检测后端选择

`dualarm_reactive_control` 支持两种碰撞检测后端，可根据场景需求选择：

| 后端 | 描述 | 性能 | 适用场景 |
|------|------|------|----------|
| **FCL** | Flexible Collision Library (默认) | 基准 (~2ms/查询) | 简单环境，兼容性优先 |
| **Coal** | FCL 的现代继任者 (HPP-FCL) | 快 5-15x (~0.5ms/查询) | **推荐**: 复杂障碍物，CPU 优化 |

### 编译不同后端

**重要提示**: 所有编译命令必须在容器内执行，并确保 ROS2 环境已正确 source：

```bash
# 进入容器
docker exec -it multipanda-container bash

# 首次编译需要先构建依赖包
source /opt/ros/humble/setup.bash
colcon build --packages-select franka_msgs

# 然后选择以下任一方式编译 dual_arm_reactive_control
```

**使用 FCL (默认):**
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_arm_reactive_control
source install/setup.bash
```

**使用 Coal:**
```bash
source /opt/ros/humble/setup.bash

# 首次使用 Coal 需要先构建 Coal 库
cd /home/xiaozy24/dual_panda_ws/src/dualarm_mprc/coal
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF
make -j$(nproc)

# 然后编译控制器
cd /home/xiaozy24/dual_panda_ws
colcon build --packages-select dual_arm_reactive_control --cmake-args -DUSE_COAL=ON
source install/setup.bash
```

**注意**: 如需使用 GPU 加速的 nvblox 后端（实验性），请参考 [nvblox GPU 加速文档](../../dualarm_mprc/docs/nvblox.md)。

### 运行控制器

**使用 MPRC :**

编译后运行安全控制器节点：

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node
或
ros2 run dual_arm_reactive_control main_sim_node \
  --ros-args \
  -p bypass_qp_safety_for_debug:=true \
  -p relative_pose_constraint_enabled:=false \
  -p data_record_ON:=true \
  -p data_record_path:=/home/xiaozy24/dual_panda_ws/data/ \
  -p data_record_prefix:=$(date +%Y%m%d_%H%M%S)
```

控制器启动后会直接进入待命状态，监听以下话题：
- `/dualarm_teleop_cmd` - 遥操作命令（键盘/SpaceMouse/CSV回放）
- `/dualArm_traj` - 轨迹命令（可选，用于预定义轨迹）

发送遥操作命令后会自动切换到 `TELEOPERATING` 模式。

更多遥操作实现细节请参考：[dualarm_mprc 遥操作文档](../../dualarm_mprc/docs/tele_operation.md)

**使用 MPC :**

MPC控制器作为baseline时：

```bash
docker exec -it multipanda-container bash
source install/setup.bash
python -m curobo.examples.getting_started.reactive_control_ros2
```

---

## 终端 5：数据监控器

### `safe_index`监控器

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control safe_index_monitor_node --ros-args \
  -p output_path:=/home/xiaozy24/dual_panda_ws/data/mpc/ \
  -p output_prefix:=$(date +%Y%m%d_%H%M%S) \
  -p d_safe:=0.05
```

**监听话题：**
- `/dualarm_teleop_cmd` - 遥操作命令（键盘/SpaceMouse/CSV回放）
- `/joint_states` - 当前关节状态（用于遥操作的初始位置）
- `/mj_left/joints_desired` - 左臂期望关节命令
- `/mj_right/joints_desired` - 右臂期望关节命令

**输出 CSV 格式：**
| 列名 | 说明 |
|------|------|
| timestamp | 时间戳 |
| source | 数据源 ("left_desired" / "right_desired" / "left_teleop" / "right_teleop") |
| arm_id | 机械臂 ID (1=左, 2=右) |
| q1-q7 | 关节位置 (rad) |
| dq1-dq7 | 关节速度 (rad/s) |
| distance | 碰撞距离 (m) |
| safe_index | 安全指数 (distance - d_safe) |

### `pose_error`监控器

监控**遥操作命令目标位姿**与**实际关节状态**之间的任务空间误差（位置 + 朝向）。

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control pose_error_monitor_node --ros-args \
  -p output_path:=/home/xiaozy24/dual_panda_ws/data/mpc/ \
  -p output_prefix:=$(date +%Y%m%d_%H%M%S)_pose_error
```

**监听话题：**
- `/dualarm_teleop_cmd` - 遥操作命令（键盘/SpaceMouse/CSV回放）
- `/joint_states` - 当前关节状态（mj_left_joint1-7, mj_right_joint1-7）

**输出 CSV 格式：**
| 列名 | 说明 |
|------|------|
| timestamp | 时间戳 |
| arm | 机械臂标识 ("left" / "right") |
| pos_err_x/y/z | 位置误差分量 (m) |
| pos_err_norm | 位置误差范数 (m) |
| orient_err_x/y/z | 朝向误差旋转向量分量 (rad) |
| orient_err_norm | 朝向误差旋转角度 (rad) |
| curr_x/y/z | 当前末端位置 (m) |
| target_x/y/z | 目标末端位置 (m) |

### `relative_error`监控器

监控**双臂相对位置**，计算 teleop command 和 safe command 相对于初始基准的误差。

首次从 `/joint_states` 获取初始位置并设置相对位置基准，然后持续计算 teleop 和 safe 命令相对于该基准的偏差。

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control relative_error_monitor_node --ros-args \
  -p output_path:=/home/xiaozy24/dual_panda_ws/data/mprc/ \
  -p output_prefix:=$(date +%Y%m%d_%H%M%S)_relative_error
```

**监听话题：**
- `/joint_states` - 当前关节状态（用于设置初始基准）
- `/dualarm_teleop_cmd` - 遥操作命令（键盘/SpaceMouse/CSV回放）
- `/mj_left/joints_desired` - 左臂期望关节命令（safe command）
- `/mj_right/joints_desired` - 右臂期望关节命令（safe command）

**输出 CSV 格式：**
| 列名 | 说明 |
|------|------|
| timestamp | 时间戳 |
| init_rel_x/y/z | 初始相对位置基准 (right_ee - left_ee, m) |
| init_rel_dist | 初始相对距离 (m) |
| curr_rel_x/y/z | 当前实际相对位置 (m) |
| curr_rel_dist | 当前实际相对距离 (m) |
| teleop_rel_x/y/z | Teleop目标相对位置 (m) |
| teleop_rel_dist | Teleop目标相对距离 (m) |
| teleop_err_x/y/z | Teleop相对基准的误差 (m) |
| teleop_err_norm | Teleop误差范数 (m) |
| safe_rel_x/y/z | Safe command目标相对位置 (m) |
| safe_rel_dist | Safe command目标相对距离 (m) |
| safe_err_x/y/z | Safe command相对基准的误差 (m) |
| safe_err_norm | Safe command误差范数 (m) |

注：当 teleop 或 safe 数据不可用时，对应字段输出为 `NaN`。

### `rosbag`录制

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 bag record -o teleop_safe_controller \
    /dualarm_teleop_cmd \
    /mj_left_gripper/grasp_desired \
    /mj_right_gripper/grasp_desired \
    /mj_left/joints_desired \
    /mj_right/joints_desired \
    /ee_pose \
    /min_distance \
    /collision_pairs \
    /joint_states
```

---

## 终端 6：运行遥操作脚本

### 方式 1：键盘关节空间控制

```bash
cd dual_panda_ws
source ~/myenv/bin/activate
python3 src/multipanda_ros2/teleop/key_teleop_joint.py --step-size 0.002
```

### 方式 2：键盘任务空间控制

```bash
cd dual_panda_ws
source ~/myenv/bin/activate
python3 src/multipanda_ros2/teleop/key_teleop_cartesian_absolute.py 
```

### 方式 3：SpaceMouse 任务空间控制

```bash
cd dual_panda_ws
source ~/myenv/bin/activate
python3 src/multipanda_ros2/teleop/spacemouse_teleop_cartesian.py --scale-translation 0.0000007
```

### 方式 4：CSV 轨迹回放(实时模式)

关节空间回放：
```bash
cd dual_panda_ws
source ~/myenv/bin/activate
python3 src/multipanda_ros2/teleop/csv_teleop_joint.py --csv-file trajectory.csv --loop
```

任务空间回放：
```bash
cd dual_panda_ws
source ~/myenv/bin/activate
python3 src/multipanda_ros2/teleop/csv_teleop_cartesian.py --csv-file cartesian_trajectory.csv --loop
```

更多遥操作脚本详情请参考：[遥操作脚本文档](teleop.md)

---

## 终端 7：实时查看 `/ee_pose` 六元组（白色窗口）

```bash
docker exec -it multipanda-container bash
source install/setup.bash
# 输出左右臂六元组: (x,y,z,roll,pitch,yaw)，角度单位为弧度
python3 /home/xiaozy24/dual_panda_ws/src/multipanda_ros2/teleop/show_ee_pose_six_tuple.py
```

注：脚本会弹出白底黑字窗口，实时显示左右臂六元组。

---

## Real Robot

```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2

ros2 control load_controller cartesian_impedance_controller
ros2 control set_controller_state cartesian_impedance_controller active
```

---

## 相关文档

- [dualarm_mprc 遥操作文档](../../dualarm_mprc/docs/tele_operation.md) - 控制器端遥操作实现详情
- [遥操作脚本文档](teleop.md) - 遥操作脚本使用说明
- [主文档](main.md) - 项目整体说明
