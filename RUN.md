# Franka ROS2 仿真运行指南 (单臂/双臂 + 控制器)

本文档说明了如何从进入 Docker 容器开始，编译、运行仿真，并使用相关脚本通过 ROS 2 Topic 持续发布指令来控制机械臂运动。

## 前置准备 (重要：对于 SpaceMouse 使用者)
如果您要使用 SpaceMouse 控制机械臂，请在启动 Docker 容器时确认已经挂载了 `spacenavd` 的 socket 文件，否则容器内的 Python 包无法读取外部鼠标硬件的信号。

完整的 Docker 启动挂载命令（如果已有容器在运行请先停止并删除旧的，然后重启）：
```bash
docker run -it -d --name multipanda-container --net=host --ipc=host --pid=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$XAUTHORITY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.Xauthority:$XAUTHORITY \
  -v /var/run/spnav.sock:/var/run/spnav.sock \
  -v /home/xiaozy24/python_code/multipanda_ros2:/home/xiaozy24/python_code/multipanda_ros2 \
  -w /home/xiaozy24/python_code/multipanda_ros2 \
  build-env:multipanda_ros2-amd64 bash
```
请打开 **3个独立的终端** 均通过 `docker exec -it multipanda-container bash` 进入容器，确保每个终端都处于项目工作空间目录下（例如 `/home/xiaozy24/python_code/multipanda_ros2`）。

---

## 终端 1：编译并启动仿真

**注意**: 如果你重建了容器且映射路径发生变化，你必须先删除旧的编译缓存：
```bash
rm -rf build install log
```

在第一个终端中，编译工作空间并启动单臂 MuJoCo 仿真。

```bash
# 1. 编译工作空间下的包
colcon build
或者 colcon build --packages-select franka_bringup

# 2. Source 环境变量
source install/setup.bash

# 3. 运行单臂 / 双臂仿真 Launch 文件
## 如果跑单机：
ros2 launch franka_bringup franka_sim.launch.py

## 如果跑双机 (配合 SpaceMouse / 键盘脚本)：
ros2 launch franka_bringup dual_franka_sim.launch.py
```
*启动后，你应该能看到 MuJoCo 仿真器界面弹开，并且机器人加载在里面。此时默认启动的是内置的、无通信的例程控制器。*

---

## 终端 2：切换控制器

```bash
# 1. Source 环境变量
source install/setup.bash

# 2. 根据你想用的控制方式加载带有 Topic 监听能力的控制器

# 【选择A: 如果要使用单机关节控制】
ros2 control load_controller joint_impedance_controller --set-state active

# 【选择B: 如果要使用基于笛卡尔空间双臂控制 (也就是键盘/SpaceMouse需要使用)】
ros2 control load_controller multi_cartesian_impedance_controller --set-state active
```
*激活成功后，你可以通过 `ros2 topic list` 看到名为 `/multi_cartesian_impedance/pose_desired` (若选择B) 的话题。这说明机器人现在正在等待外部发送目标笛卡尔空间指令。*

---

## 终端 3：运行自动化/交互式控制脚本

这里提供多种发布策略（键盘、SpaceMouse鼠标、代码预设），需要您在虚拟环境 (venv) 内执行，它们都以特定的频率（例如50Hz 或 100Hz）向控制器话题发送控制指令信号：

```bash
# 1. 挂载环境
source install/setup.bash
source ~/myenv/bin/activate

# 2. 修改对应脚本的执行权限（只需一次）
chmod +x tools/spacemouse_pub.py

# 3. 运行你的控制脚本 (根据你之前加载的控制器，选择下方其中一个！)


# 【选择A: SpaceMouse 笛卡尔空间双臂控制】
# (需要确保容器在建立时绑定 -v /var/run/spnav.sock:/var/run/spnav.sock)
python3 tools/spacemouse_pub.py

# 【选择B: 键盘 笛卡尔空间交互式控制】
# (使用 Z/X 切换左右手， W/S/A/D/Q/E 及 J/L/I/K/U/O 平移或旋转)
python3 tools/key_pub.py

# 【选择C: 无聊双臂正弦波自动舞动测试】
python3 tools/dual_auto_pub.py 
```

执行后，切回 MuJoCo 仿真界面，你将可以通过外部设备或者代码定义的轨迹直接驱动 Panda 机械臂阵列！
