# Franka ROS2 仿真运行指南 (单臂/双臂 + 控制器)

本文档说明了如何从进入 Docker 容器开始，编译、运行仿真，并使用相关脚本通过 ROS 2 Topic 持续发布指令来控制机械臂运动。

## 前置准备

启动 Docker 容器时，为了确保能看到仿真界面并与容器外进行 ROS 2 通信，请运行以下完整的 Docker 启动命令（如果已有容器在运行请先停止并删除旧的，然后重启）：
```bash
docker run -it -d --name multipanda-container \
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

## 终端 2：渲染查看图像 (新功能)

在第二个终端中，为了不因为 OpenCV 与 Python 绑定导致同步卡顿，你可以直接调用 ROS 2 的高性能图像展示工具查看特定视角：

```bash
docker exec -it multipanda-container bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*启动后，在弹出的窗口左上角下拉菜单选择 `/mujoco_server/cameras/left_arm_cam/rgb/image_raw` 等话题即可观察夹爪视角的视频流。*

---

## 终端 3：切换控制器

在第二个终端中，加载控制器并使其激活。

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
