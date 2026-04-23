# Franka ROS2 仿真运行指南 (单臂/双臂 + 控制器)

本文档说明了如何从进入 Docker 容器开始，编译、运行仿真，并使用相关脚本通过 ROS 2 Topic 持续发布指令来控制机械臂运动。

## 调整参数时的工作流

当需要修改相机位置、模型位置、控制器参数或其他仿真配置时，请按以下流程操作：

### 1. 启动容器
先启动容器，然后按如下指令运行：
```bash
docker start multipanda-container
```
通过 `docker exec -it multipanda-container bash` 进入容器，确保每个终端都处于项目工作空间目录下（multipanda_ws）

### 2. 修改源码中的参数
- 优先修改工作区源码目录中的文件，而不是 `install/` 目录。
- 常见修改位置包括：
  - `my_task_description/launch/my_task_sim.launch.py`
  - 控制器配置文件
  - 机器人模型 XML / 生成 XML 的逻辑

### 3. 确认修改的是当前实际运行的工作区
- 当前仿真使用的是：
  - `/home/developer/multipanda_ws/install/my_task_description`
- 如果源码不在该工作区内，需要先同步到对应 workspace。

### 4. 重新编译工作区
```bash
cd /home/developer/multipanda_ws
colcon build --packages-select my_task_description --symlink-install
```（注意选择对应的packages,不需要全部编译，这里一my_task_description为例子）

### 5. 重新加载环境
```bash
source install/setup.bash
```

### 6. 重新启动仿真
- 先停止旧的 `ros2 launch` 进程
- 再执行新的启动命令
- 修改后的 XML、相机位置、物体位置等才会生效

### 7. 检查是否真正生效
```bash
ros2 pkg prefix my_task_description
ros2 launch my_task_description my_task_sim.launch.py
```
如果输出仍然是 `/home/developer/multipanda_ws/install/...`，说明当前运行的就是这个工作区。

### 8. 如果改动后没有变化
- 检查是否改到了错误的源码目录
- 检查是否忘记重新 `colcon build`
- 检查是否忘记重新 `source install/setup.bash`
- 检查是否还在运行旧的 launch 进程## 调整参数时的工作流



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
  -v /home/xiaozy24/python_code/multipanda_ros2:/home/xiaozy24/python_code/multipanda_ros2 \
  -w /home/xiaozy24/python_code/multipanda_ros2 \
  build-env:multipanda_ros2-amd64 bash
```

先启动容器，然后按如下指令运行：
```bash
docker start multipanda-container
```
请打开 **4个独立的终端** 均通过 `docker exec -it multipanda-container bash` 进入容器，确保每个终端都处于项目工作空间目录下。

---

## 终端 1：启动带相机的自定义仿真

在第一个终端中，进入容器并启动双臂 MuJoCo 仿真。

```bash
docker exec -it multipanda-container bash
source install/setup.bash
# 启动带有自定义环境和多相机渲染的任务仿真
ros2 launch my_task_description my_task_sim.launch.py
```
*启动后，你应该能看到包含两个相机零件(STL)的 MuJoCo 环境弹开，并且机器人加载在里面。同时 MuJoCo 会通过离屏渲染持续向 ROS 2 发布 `fixed_cam`, `left_arm_cam`, `right_arm_cam` 三路图像。*

---

## 终端 2：流畅查看相机视频流 (Web 服务方案)

由于跨 Docker 容器向宿主机进行 X11 图形界面转发（如 OpenCV、rqt）会死锁带宽，导致高帧率图片产生极其严重的卡顿，因此我们采用将本网络变为网页视频流的大方向硬件解耦方案：

```bash
docker exec -it multipanda-container bash
source install/setup.bash
# 如果尚未安装，请运行: sudo apt-get update && sudo apt-get install -y ros-humble-web-video-server
ros2 run web_video_server web_video_server
```
*启动服务后，请打开宿主机上的浏览器（Chrome/Edge 等），在地址栏输入：**http://localhost:8080***
*网页中会自动汇总出当前 MuJoCo 发布的所有图像话题。您只需点击 `/mujoco_server/cameras/left_arm_cam/rgb/image_raw` 等名称链接，即可在浏览器页卡中获得极度丝滑无阻滞的相机推流画面！*

---

## 终端 3：切换控制器

在第三个终端中，加载控制器并使其激活。

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 control load_controller multi_cartesian_impedance_controller 
ros2 control set_controller_state multi_cartesian_impedance_controller inactive 
ros2 control set_controller_state multi_cartesian_impedance_controller active 
```
*激活成功后，你可以通过 `ros2 topic list` 看到相关对话。这说明机器人现在正在等待外部发送目标笛卡尔空间指令。*

---

## 终端 4：运行控制脚本

在第四个终端中，运行自动/交互式控制脚本。

```bash
cd python_code/multipanda_ros2
source ~/myenv/bin/activate #进入你的虚拟环境
python3 tools/spacemouse_pub.py
```

执行后，切回 MuJoCo 仿真界面，你将可以通过外部设备或者代码定义的轨迹直接驱动 Panda 机械臂阵列！
