# 固定相机点云运行说明

`my_task_sim.launch.py` 会为固定环境相机启动 RGB-D 渲染和
`depth_image_proc::PointCloudXyzrgbNode`，并把点云发布到：

```text
/mujoco_server/cameras/fixed_cam/points
```

左右腕部相机继续提供 RGB 图像，但不生成点云。

## 构建

```bash
cd /home/csc/csc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select depth_image_proc my_task_description
source install/setup.bash
```

## 启动

```bash
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true
```

点云转换默认开启。不需要点云时使用：

```bash
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=true enable_pointcloud:=false
```

## RViz 2 设置

点云配置包含在 `visualize_dual_franka.rviz` 中，启动后应检查：

- `Global Options -> Fixed Frame` 为 `base_link`。
- `Fixed Camera PointCloud` 已启用。
- 颜色转换器为 `RGB8`。

## 检查话题和 TF

```bash
ros2 topic hz /mujoco_server/cameras/fixed_cam/points
ros2 run tf2_ros tf2_echo base_link fixed_cam_optical_frame
```

深度图来自：

```text
/mujoco_server/cameras/fixed_cam/depth/image_raw
```

只有点云转换节点或 RViz 订阅相关图像时，MuJoCo 才会进行对应的离屏渲染。

`robot_state_publisher`、`joint_state_publisher`、点云转换节点和 RViz 必须与
MuJoCo 一样启用 `use_sim_time`。否则点云使用仿真时间，而机器人 TF 使用系统时间，
表现为 RViz 只有直接选择相机帧时才能显示点云。
