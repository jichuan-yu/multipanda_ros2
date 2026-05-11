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
colcon build
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

`dualarm_reactive_control` 支持三种碰撞检测后端，可根据场景需求选择：

| 后端 | 描述 | 性能 | 适用场景 |
|------|------|------|----------|
| **FCL** | Flexible Collision Library (默认) | 基准 (~2ms/查询) | 简单环境，兼容性优先 |
| **Coal** | FCL 的现代继任者 (HPP-FCL) | 快 5-15x (~0.5ms/查询) | 复杂障碍物，CPU 优化 |
| **nvblox GPU** | NVIDIA GPU 加速 TSDF/ESDF | 最快 (~0.1ms/查询) | 高频控制，大量动态障碍物 |

### 编译不同后端

**使用 FCL (默认):**
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
colcon build --packages-select dual_arm_reactive_control
source install/setup.bash
```

**使用 Coal:**
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
colcon build --packages-select dual_arm_reactive_control --cmake-args -DUSE_COAL=ON
source install/setup.bash
```

**使用 nvblox GPU:**
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
colcon build --packages-select dual_arm_reactive_control --cmake-args -DUSE_NVBLOX=ON
source install/setup.bash
```

### 运行控制器

编译后运行安全控制器节点：

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node
```

控制器启动后会直接进入待命状态，监听以下话题：
- `/dualarm_teleop_cmd` - 遥操作命令（键盘/SpaceMouse/CSV回放）
- `/dualArm_traj` - 轨迹命令（可选，用于预定义轨迹）

发送遥操作命令后会自动切换到 `TELEOPERATING` 模式。

### nvblox GPU 配置

使用 nvblox 后端时，可在配置文件中调整参数：

```yaml
# config/nvblox_collision.yaml
voxel_size: 0.02          # 体素大小 (米)
max_distance: 2.0         # 最大 ESDF 距离
gpu_device_id: 0          # GPU 设备 ID
enable_async_transfer: true  # 异步 CUDA 操作
```

更多遥操作实现细节请参考：[dualarm_mprc 遥操作文档](../../dualarm_mprc/docs/tele_operation.md)

---

## 终端 5：运行遥操作脚本

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
python3 src/multipanda_ros2/teleop/key_teleop_cartesian.py --step-position 0.001 --step-rotation 0.01
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
