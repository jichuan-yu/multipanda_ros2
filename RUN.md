# Franka ROS2 仿真运行指南 (单臂 + Subscriber 控制器)

本文档说明了如何从进入 Docker 容器开始，编译、运行仿真，并使用 Python 脚本通过 ROS 2 Topic 持续发布指令来控制机械臂运动。

## 前置准备
请先进入您的 Docker 容器，并打开 **3个独立的终端**，确保每个终端都处于项目工作空间目录下（例如 `~/python_code/multipanda_ros2` 或对应的挂载路径）。

---

## 终端 1：编译并启动仿真

在第一个终端中，编译工作空间并启动单臂 MuJoCo 仿真。

```bash
# 1. 编译工作空间下的包
colcon build
或者 colcon build --packages-select franka_bringup

# 2. Source 环境变量
source install/setup.bash

# 3. 运行单机仿真 Launch 文件
ros2 launch franka_bringup franka_sim.launch.py
```
*启动后，你应该能看到 MuJoCo 仿真器界面弹开，并且机器人加载在里面。此时默认启动的是内置的、无通信的例程控制器。*

---

## 终端 2：切换控制器

```bash
# 1. Source 环境变量
source install/setup.bash

# 2. 加载带有 Topic 监听能力的控制器
ros2 control load_controller joint_impedance_controller

# 3. 激活新控制器
ros2 control set_controller_state joint_impedance_controller inactive
ros2 control set_controller_state joint_impedance_controller active
```
*激活成功后，你可以通过 `ros2 topic list` 看到名为 `/joint_impedance/joints_desired` 的话题。这说明机器人现在正在等待外部发送目标关节指令。*

---

## 终端 3：运行自动化控制脚本

使用我们编写的 Python 脚本，以 100Hz 的频率向 `/joint_impedance/joints_desired` 话题持续发布正弦波关节运动指令。

```bash
# 1. Source 环境变量
source install/setup.bash

# 2. 修改脚本执行权限（如果之前未修改）
chmod +x src/tools/auto_pub.py

# 3. 运行自动化发布脚本
python3 src/tools/auto_pub.py
```

执行后，切回 MuJoCo 仿真界面，你将看到 Panda 机械臂的第 4 和 第 5 关节按照指定的正弦波轨迹进行平滑的周期运动。
