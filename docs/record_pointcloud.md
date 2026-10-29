**1. 环境准备**

1.1 安装 ROS2 (Humble)

bash

\#
参考官方文档：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

sudo apt update && sudo apt install ros-humble-desktop

1.2 安装 RealSense ROS2 驱动

bash

sudo apt install ros-humble-librealsense2\* ros-humble-realsense2-\*

1.3 验证相机

bash

realsense-viewer

确认相机能正常输出彩色/深度图。

1.4 (可选) Python 环境 -- 用于读取 bag 文件

使用 conda 创建独立环境：

bash

conda create -n ros2_bag python=3.10 -y

conda activate ros2_bag

pip install numpy open3d rosbags sensor-msgs

**2. 启动相机并录制点云**

2.1 启动相机节点（同时开启点云和对齐）

bash

ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
align_depth.enable:=true

注意：D405 没有红外发射器，无需添加 depth_module.emitter_enabled:=true。

2.2 录制 ROS2 bag 文件（包含点云和 TF）

在新终端执行：

bash

ros2 bag record -o my_pointcloud_data /camera/camera/depth/color/points
/tf /tf_static

按 Ctrl+C 停止录制。

录制后生成文件夹 my_pointcloud_data/，内含 metadata.yaml 和 \*.db3
数据文件。

3\. 可视化点云 (RViz2)

3.1 播放 bag 文件

bash

ros2 bag play my_pointcloud_data \--loop

3.2 启动 RViz2

另开终端：

bash

rviz2

3.3 配置 RViz2

Fixed Frame：设置为 camera_link（或 camera_depth_optical_frame）。

添加点云：Add → By topic → 选择 /camera/camera/depth/color/points → OK。

（可选）调整点云颜色：Color → Transformer → RGB8。
