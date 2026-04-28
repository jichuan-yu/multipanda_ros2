# Franka ROS2 仿真运行指南 (单臂/双臂 + 控制器)

本文档说明了如何从进入 Docker 容器开始，编译、运行仿真，并使用相关脚本通过 ROS 2 Topic 持续发布指令来控制机械臂运动。

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
run指令后容器自动处于已启动状态

此后再次使用该容器时，按如下指令启动该容器：
```bash
docker start multipanda-container
```
请打开 **3个独立的终端** 均通过 `docker exec -it multipanda-container bash` 进入容器，确保每个终端都处于项目工作空间目录下。

---

## 终端 1：启动带相机的自定义仿真

在第一个终端中，进入容器并启动双臂 MuJoCo 仿真。

```bash
docker exec -it multipanda-container bash
colcon build
source install/setup.bash
# 启动带有自定义环境和多相机渲染的任务仿真
ros2 launch my_task_description my_task_sim.launch.py
```
*启动后，你应该能看到包含两个相机零件(STL)的 MuJoCo 环境弹开，并且机器人加载在里面。同时 MuJoCo 会通过离屏渲染持续向 ROS 2 发布 `fixed_cam`, `left_arm_cam`, `right_arm_cam` 三路图像。此外，底层控制器 `dual_joint_impedance_controller` 也会被自动加载和激活，这说明机器人现在已经准备好接收关节目标指令。*

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

## 终端 3：运行安全控制脚本

在第三个终端中，运行安全控制 ROS2 节点

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run dual_arm_reactive_control dualarm_mprc_node
ros2 param set /dual_arm_mprc_node /panda_1_xyz "0.0 0.26 0.0"
ros2 param set /dual_arm_mprc_node /panda_2_xyz "0.0 -0.26 0.0"
ros2 param set /dual_arm_mprc_node /panda_1_rpy "0.0 0.0 0.0"
ros2 param set /dual_arm_mprc_node /panda_2_rpy "0.0 0.0 0.0"
```

如果 `ros2 run` 找不到可执行文件，可以直接运行安装路径：

```bash
docker exec -it multipanda-container bash -c "cd /home/xiaozy24/dual_panda_ws && source install/setup.bash && ./install/dual_arm_reactive_control/lib/dual_arm_reactive_control/dualarm_mprc_node"
```

## 终端 4：运行话题重发布节点

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run franka_example_controllers topic_bridge
```

## 终端 5：运行安全键盘发布器

```bash
cd dual_panda_ws
source ~/myenv/bin/activate
python3 src/multipanda_ros2/spacemouse_teleop/key_safe_pub.py
```

---

## Real Robot

``` bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2

ros2 control load_controller cartesian_impedance_controller 
ros2 control set_controller_state cartesian_impedance_controller active 

```