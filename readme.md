# ICP 动态追踪：从 ROS2 Bag 中提取移动支架点云

## 功能简介

本程序从 ROS2 bag 文件（Realsense D405 录制）中逐帧读取点云，并利用 ****ICP（迭代最近点）**** 算法动态追踪并提取****支架****（或任意手动选定的物体）的点云。核心特点：

- ****第一帧手动框选****：通过预设的包围盒提取支架初始点云。

- ****后续帧自动追踪****：采用 ICP 将上一帧支架点云配准到当前帧，保证点数稳定、形状完整。

- ****全局 ROI 加速****：根据初始支架位置自动生成感兴趣区域，大幅减少每帧需处理的点数，提升运行效率。

- ****输出每帧支架点云****：保存为 PLY 文件，便于后续分析或可视化。

## 系统要求

- Ubuntu 22.04 / ROS2 Humble

- Conda（推荐）或系统 Python 3.10

- 已录制的 ROS2 bag 文件（包含 `/camera/camera/depth/color/points` 话题）

## 安装与依赖

### 1. 创建 Conda 环境（推荐）

```bash

conda create -n pointcloud_tracking python=3.10 -y

conda activate pointcloud_tracking

**2. 安装依赖**

bash

pip install -r requirements.txt

**注意**：rosbag2_py 和 rclpy 需要 ROS2 Humble 环境。如果你在 conda 中无法导入，请**退出 conda** 并使用系统 Python（确保已安装 ROS2）。但本程序已通过 rosbag2_py 直接读取 bag 文件，不依赖 rclpy，因此在纯 Python 环境下也可运行（需安装 rosbags 替代？实际代码用了 rosbag2_py，它依赖 ROS2 的 C++ 库。推荐在系统 Python 中运行，或使用 rosbags 库的版本。为简化，我们建议直接在系统终端（已 source ROS2）运行，并使用 pip3 install --user open3d sensor-msgs。用户可根据实际情况调整。

**使用方法**

**1. 修改配置参数**

编辑 icp_stable_track.py 开头的配置部分：

- BAG_PATH：你的 bag 文件夹路径。
- INIT_BBOX：**第一帧中支架的精确包围盒**（需提前通过观察确定）。
- GLOBAL_ROI_EXPAND：全局 ROI 扩大倍数（默认 3.0）。
- OUTPUT_DIR：输出 PLY 文件的目录。

**2. 运行程序**

bash

conda activate pointcloud_tracking   *# 如果使用 conda*

python icp_stable_track.py

**3. 结果**

- 每帧支架点云保存为 frame_XXXX_stand.ply。
- 终端输出每帧提取的点数。
- 每隔 VISUALIZE_INTERVAL 帧会弹出 Open3D 窗口可视化当前结果（灰色为场景，红色为支架）。

**参数调优指南**

参数

作用

调优建议

INIT_BBOX

第一帧支架的精确区域

通过 RViz2 或 Open3D 观察支架的 X,Y,Z 范围，适当缩小以减少背景噪点。

GLOBAL_ROI_EXPAND

全局 ROI 扩大倍数

支架运动范围大则增大（如 4.0），范围小则减小（如 2.0）。

VOXEL_SIZE

体素下采样大小

点云密集时可增大（0.01），稀疏时减小（0.003）。

ICP_MAX_CORRESPONDENCE_DISTANCE

ICP 最大对应点距离

运动快/点云稀疏时增大（0.08~0.1）。

SEARCH_EXPAND_FACTOR

候选区域扩大倍数

保证候选区域包含支架新位置，一般 2.0~3.0。

**常见问题**

**Q: 第一帧提取失败，提示“请调整初始包围盒”？**
A: 检查 INIT_BBOX 范围是否准确包含了支架所有点，可先用其他工具（如 CloudCompare）测量支架在 bag 第一帧中的坐标范围。

**Q: 后续帧追踪偏移或点数大幅下降？**
A: 增大 SEARCH_EXPAND_FACTOR 或 ICP_MAX_CORRESPONDENCE_DISTANCE，同时确认全局 ROI 没有切掉支架。

**Q: 提示 **ModuleNotFoundError: No module named 'rosbag2_py'**？**
A: 需要已安装 ROS2 Humble 并 source 环境（source /opt/ros/humble/setup.bash）。或者在系统终端直接运行（不进入 conda），并安装其他依赖。