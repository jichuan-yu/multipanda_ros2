# 双臂轨迹跟踪演示

## 系统架构

```
traj1.csv → main_sim_node (MPRC安全控制器)
                 ↓
    /dual_joint_impedance/joints_desired
                 ↓
    multi_joint_impedance_controller → 机械臂
```

## 运行步骤

### 终端1: 启动仿真环境
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
colcon build
source install/setup.bash
ros2 launch my_task_description my_task_sim.launch.py use_rviz:=false
```

### 终端2: 启动MPRC安全控制器
```bash
docker exec -it multipanda-container bash
cd /home/xiaozy24/dual_panda_ws
colcon build --packages-select dual_arm_reactive_control
source install/setup.bash
ros2 run dual_arm_reactive_control main_sim_node --ros-args -p trajectory_file:=/home/xiaozy24/dual_panda_ws/src/multipanda_ros2/tools/traj1.csv
```

## 文件说明

### 工具脚本 (src/multipanda_ros2/tools/)
- `traj1.csv` - 轨迹数据文件 (14列: 左臂7关节 + 右臂7关节)
- `record_trajectory.py` - 轨迹录制工具

### 控制器说明
- `main_sim_node` (dual_arm_reactive_control包)
  - 订阅: `/dualArm_traj` (trajectory_msgs/JointTrajectory)
  - 发布: `/dual_joint_impedance/joints_desired` (std_msgs/Float64MultiArray)
  - 功能: 安全控制器，处理碰撞避免、关节限制等

- `multi_joint_impedance_controller` (franka_example_controllers包)
  - 订阅: `/dual_joint_impedance/joints_desired`
  - 输出: 关节力矩命令
  - 功能: 底层关节阻抗控制

## 轨迹格式

CSV文件格式 (14列):
```
q1_left,q2_left,q3_left,q4_left,q5_left,q6_left,q7_left,q1_right,q2_right,q3_right,q4_right,q5_right,q6_right,q7_right
-0.0,-0.785,0.0,-2.356,0.0,1.571,0.785,-0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
...
```
