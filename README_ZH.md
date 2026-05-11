# multipanda_ros2
## 基于 `ros2_control` 框架的 Panda 机器人仿真与真机集成
<img src="docs/images/single_sim.png" alt="" height="250">
<img src="docs/images/dual_sim.png" alt=""   height="250">
<img src="docs/images/garmi_sim.png" alt=""  height="250">

该项目在 ROS2 Humble 中实现了原始 `franka_ros` 仓库的大部分功能，专为 Franka Emika 机器人（Panda）设计。
本项目在官方不再支持 Panda 的原版 `franka_ros2` 基础上进行了大幅扩展。

此外，还集成了多臂 MuJoCo 仿真，这意味着您现在可以在仿真和真实机器人上运行相同的控制器。
该仿真设计为 [`mujoco_ros_pkg`](https://github.com/ubi-agni/mujoco_ros_pkgs/) 的插件。

**当前版本依赖于该仓库的[一个分支（fork）](https://github.com/tenfoldpaper/mujoco_ros_pkgs)**，该分支实现了 `ros2_control` 插件以及用于简单机器人设置（例如安装在移动底座上的 Panda 机械臂）的通用 `SystemInterface`。这意味着您应该安装这个 fork，而不是原始仓库，直到两者合并。

正在进行将 FR3 集成到此架构中的工作。

## 文档

该项目的文档可[在此处](./docs/main.md)获取。

## 可用功能
关于更多详细信息，请参阅文档。

* 真实机器人（Real robot）:
    * FrankaState 广播器（broadcaster）
    * 所有控制接口（扭矩、位置、速度、笛卡尔空间）。
    * 所有接口的示例控制器
    * 可以使用 rqt_controller_manager 切换控制器
    * 通过 `~/service_server/error_recovery` 进行运行时 franka::ControlException 错误恢复
        * 恢复后，将再次执行先前执行的控制循环，因此无需重新加载。
    * 提供运行时内部参数设置服务，类似于更新后的 `franka_ros2` 所提供的功能
* 仿真机器人（Sim robot）:
    * 与真实机器人相同，除了此时不可用的笛卡尔控制指令接口，且目前没有计划实现此接口。
    * 夹爪服务器，其接口与真实夹爪相同（即 action server）
    * 上文列出的所有真实单臂示例控制器都适用于仿真机器人的相应接口，开箱即用。
    * FrankaState 实现了基础属性：扭矩、关节位置/速度、`O_T_EE` 和 `O_F_ext_hat`。
    * 模型提供了所有现有函数：`pose`，`zeroJacobian`，`bodyJacobian`，`mass`，`gravity`，`coriolis`。
        * 由于当前的限制，gravity 仅返回来自 MuJoCo 的对应 `qfrc_gravcomp` 力。
        * Coriolis = `qfrc_bias - qfrc_gravcomp`
    * 相机功能可用，作为 `mujoco_ros_pkg` 功能的一部分。您只需在您的 mujoco XML 文件中添加一个 `<camera>` 对象，该包就会处理它。
    * 通过分支仓库的 `mujoco_ros2_control_system` 包，您可以轻松地向机器人添加具有额外自由度的组件。请查看 `garmi_packages/garmi_description/robots/*.ros2_control.xacro` 以获取有关如何执行此操作的示例。

## 已知问题
* 关节位置控制器可能会导致电机行为异常。建议目前使用扭矩或速度控制器。
* 默认的 `franka_moveit_config` 依赖于已弃用的 `warehouse_ros_mongo`。现已更改为 `warehouse_ros_sqlite` 以确保 `rosdep install` 能正常工作。目前请参考[此处](https://discourse.ros.org/t/fixing-moveit2-humble-moveit-ros-benchmarks-package/32048)的讨论获取可能的解决方案；一旦确认了合适的解决方案，它将被添加进项目中。

## 使用一键安装脚本进行安装（推荐）
1. 递归克隆包含 **mujoco_ros_pkgs** 的仓库:
```
git clone --recursive https://github.com/tenfoldpaper/multipanda_ros2.git
```
2. 然后进入克隆下来的目录:
```
cd multipanda_ros2
```
3. 运行一条命令以构建 docker 镜像（需要一些时间）:
```
./tools/setup_env
```
4. 镜像构建完成后，启动开发容器:
```
./run
```
默认配置允许网络通信、GPU 访问、GUI 应用程序的显示转发（display forwarding）、硬件设备等。默认情况下，该脚本会在容器内的 `~/multipanda_ws` 目录下打开一个 bash shell（作为开发者用户，密码可在 Dockerfile 中修改）。

5. 构建 ROS2 包:
```
colcon build
```
* 如果遇到一些缺少包的问题，请在运行 `colcon build` 之前在容器内运行以下命令：
    ```
    sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r
    ```
* 如果遇到SSL证书验证失败的错误（如 `SSL: CERTIFICATE_VERIFY_FAILED`），可以尝试以下解决方案：
    ```bash
    # 方案1：更新CA证书
    sudo apt update && sudo apt install -y ca-certificates
    
    # 方案2：如果方案1无效，可以临时禁用SSL验证（不推荐用于生产环境）
    export PYTHONHTTPSVERIFY=0
    rosdep update
    rosdep install --from-paths src --ignore-src -y -r
    ```
* 如果仍然无法解决，可以尝试手动安装缺失的依赖包，而不是使用rosdep。
* 如果遇到"Build step for lodepng failed"错误，说明MuJoCo无法下载lodepng依赖。我们提供了一个修复脚本：
```bash
chmod +x fix_mujoco_lodepng.sh
./fix_mujoco_lodepng.sh
```

或者，您可以手动安装lodepng：
```bash
# 安装lodepng依赖
sudo apt install -y libpng-dev

# 创建临时目录
mkdir -p ~/tmp
cd ~/tmp

# 下载lodepng
git clone https://github.com/lvandeve/lodepng.git
cd lodepng
git checkout 17d08dd26cac4d63f43af217ebd70318bfb8189c

# 编译lodepng
gcc -c lodepng.cpp -o lodepng.o
ar rcs liblodepng.a lodepng.o

# 安装lodepng
sudo mkdir -p /usr/local/include/lodepng
sudo cp lodepng.h /usr/local/include/lodepng/
sudo cp liblodepng.a /usr/local/lib/

# 更新动态链接库缓存
sudo ldconfig

# 清理临时目录
cd ~
rm -rf ~/tmp
```
6. 要验证安装是否成功，运行:
```
source ~/multipanda_ws/install/setup.bash && \
ros2 launch franka_bringup franka_sim.launch.py
```
* 这将调出具有一个 Franka Panda 机械臂的 MuJoCo 仿真。
* 如果要在另一个终端中打开 docker 容器，需使用 *docker exec* 命令:
    ```
    docker exec -it --user developer multipanda-container bash
    ```
7. (可选) 检查机器人连接、实时 (RT) 内核和屏幕是否都工作正常（确保 FCI 功能已激活）。
    - 您可以通过运行以下命令检查 docker 是否已正确连至屏幕:
        - `ros2 run rviz2 rviz2` 或者
        - `~/Libraries/mujoco/bin/simulate`
    - 要测试实时内核和机器人连接，请运行:
        - `~/Libraries/libfranka/bin/communication_test <robot-ip>`

## 手动安装指南（不推荐）
（系统应在 Ubuntu 22.04，ROS2 Humble，Panda 4.2.2 & 4.2.1系统包，使用`libfranka` 0.9.2 以及 MuJoCo 3.2.0）测试过
在运行带有实时内核的 Ubuntu 22.04（如果您希望在真实机器人上运行）上执行以下操作：
1. 按照[说明][humble-instructions]安装 ROS2 Humble，然后创建一个工作空间（workspace）。
5. 安装库依赖:
    - 安装 [Eigen **3.3.9**](https://gitlab.com/libeigen/eigen/-/releases/3.3.9). 如果按照 `libfranka` 步骤已安装了 Eigen 3.4.0 请卸载。
        - 如果使用 Eigen 3.4.0 某些函数将发生问题并导致编译失败。
    - 安装 [`dq-robotics`](https://dqrobotics.github.io/) C++ 版本
2. 从源码编译 `libfranka` 0.9.2: 
    - 安装依赖: `sudo apt-get install -y build-essential cmake git libpoco-dev  libfmt-dev`
        - (理想情况下，您应已安装 `build-essential`，`cmake` 以及 `git`)
    - 克隆 `libfranka` 仓库: `git clone https://github.com/frankaemika/libfranka.git`
    && mkdir /home/user/Libraries/libfranka \
    && cd libfranka \
    - 签出 0.9.2 并更新子模块（submodules）:
        ``` bash
        cd libfranka
        git checkout 0.9.2 
        git submodule init 
        git submodule update 
        ```
    - 编译该库，并将其安装在您选择的目录（此处为 `~/Libraries/libfranka`）：
        ``` bash
        mkdir build && cd build
        cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/home/user/Libraries/libfranka ..
        cmake --build .
        cmake --install .
        ```

3. 根据 [指南][mujoco-instructions] 从源码编译 MuJoCo **3.2.0**（受到 `mujoco_ros_pkg` 约束）。
4. 安装 `mujoco_ros_pkg`，更明确地说应当安装[这个 fork 分支](https://github.com/tenfoldpaper/mujoco_ros_pkgs).
6. 克隆此仓库（即 multipanda）到您工作空间的 `src` 文件夹中。
7. 在工作空间的 root 目录运行以下 rosdep 命令安装依赖: 
    
    `rosdep install --from-paths src -y --ignore-src`
7. 将 `{mujoco/libfranka}/lib/cmake` 目录导出为环境的 `CMAKE_PREFIX_PATH`，即
    - 在 `~/.bashrc` 中, `export CMAKE_PREFIX_PATH={path to mujoco installation}/lib/cmake:{path to libfranka installation}/lib/cmake`
8. 将编译路径添加到 `LD_LIBRARY_PATH` 将其加在 `~/.bashrc` 如下行: 
    
    `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{path to libfranka install}/lib:{path to mujoco's install}/lib`
9. source 此工作空间，在工作空间根目录中调用以下指令: 
    
    `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
    - 此处因为 MuJoCo 与 libfranka 的路径已加入至 `CMAKE_PREFIX_PATH` 中，因此不需要额外的编译 args。
    - MuJoCo 路径即为您安装目录所在的路径。此 `lib` 中仅存在两个 `.so` 文件，和一个 `cmake` 目录。
    - 同样地， `libfranka` 路径也应该包含一个 `cmake` 目录和对应的 `.so` 文件。
10. 如何运行:
    - 单臂（Single arm）:
        1. 使用真实机器人的情况下，source 当前工作空间，并运行：
            - 默认参数下: `ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>`.
            - 混合模式下: `ros2 launch franka_bringup multimode_franka.launch.py robot_ip:=<fci-ip>`.
            - `arm_id` 被锚定为 `panda`。
        2. 使用仿真机器人的情况下，source 当前工作空间，并运行：
            - 默认参数下: `ros2 launch franka_bringup franka_sim.launch.py`.
            - `arm_id` 被固定为 `panda`。
            - 暂行仅支持挂接了夹爪的机器人运行。
    - 双臂（Dual arm）:
        1. 使用真实机器人的情况下，source 当前工作空间，并运行：
            - 默认参数下: `ros2 launch franka_bringup dual_franka.launch.py robot_ip_1:=<robot-1-fci-ip> robot_ip_2:=<robot-2-fci-ip> arm_id_1=<robot-1-name> arm_id_2=<robot-2-name>`.
            - 混合模式下: `ros2 launch franka_bringup dual_multimode_franka.launch.py robot_ip_1:=<robot-1-fci-ip> robot_ip_2:=<robot-2-fci-ip> arm_id_1=<robot-1-name> arm_id_2=<robot-2-name>`.
            - 没有设置默认的 `arm_id`。
            - grippers (夹持器) 默认被设为 true 运行开启; 加入 `hand_n=false` 关闭它们。
        2. 使用仿真机器人的情况下，source 当前工作空间，并运行：
            - 默认参数下: `ros2 launch franka_bringup dual_franka_sim.launch.py`.
            - 默认使用  `arm_id_1=mj_left` 以及  `arm_id_2=mj_right` 。
    - Garmi（带双 Panda 臂的移动底盘机器人）:
        1. 使用仿真机器人的情况下，source 当前工作空间，并运行：
            - `ros2 launch garmi_bringup sim_garmi.launch.py`

## 引用 
如果 multipanda_ros2 框架对您的学术研究提供了帮助，请您引用我们的论文：

**[Bridging the Sim-to-Real Gap with multipanda_ros2: A Real-Time ROS2 Framework for Multimanual Systems](https://arxiv.org/abs/2602.02269)** 

```bibtex
@misc{škerlj2026multipanda_ros2,
      title={Bridging the Sim-to-Real Gap with multipanda_ros2: A Real-Time ROS2 Framework for Multimanual Systems}, 
      author={Jon Škerlj and Seongjin Bien and Abdeldjallil Naceri and Sami Haddadin},
      year={2026},
      eprint={2602.02269},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={[https://arxiv.org/abs/2602.02269](https://arxiv.org/abs/2602.02269)}, 
}
```

## 鸣谢
本原有的初构版是由 mcbed 基于 [humble][mcbed-humble] `franka_ros2` 拓展的端口而来。

## 许可证（License）

包含在 `multipanda_ros2` 内所有的软件包均按照跟原版 `franka_ros2` 一致在 [Apache 2.0 license][apache-2.0] 下发放。

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble
[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
[humble-instructions]: https://docs.ros.org/en/humble/Installation.html