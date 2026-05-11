

## Prerequisites

Ubuntu 22.04 LTS + ROS 2 Humble

For real robot experiments, a real-time kernal is required. Please refer to [Franka Control Inferface](https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html).




## Single arm controller:

0. Ruturn to home position (0, -PI/4, 0, -3PI/4, 0, PI/2, PI/4):
```bash
ros2 launch franka_bringup move_to_start.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true
```

---

1. Launch cartesian controller:
``` bash
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true \
  controller_name:=cartesian_impedance_controller \
  use_rviz:=false
```

(Optional) Record rosbag data in a new terminal:
```bash
# Record all topics (recommended for full data capture)
ros2 bag record -o ./data/$(date +%Y%m%d_%H%M%S) --all

# Or record specific topics only
ros2 bag record -o ./data/$(date +%Y%m%d_%H%M%S) \
  /panda/joint_states \
  /panda_gripper/joint_states \
  /panda_gripper/width_desired \
  /panda_gripper/grasp_desired \
  /cartesian_impedance/ee_pose \
  /cartesian_impedance/pose_desired \
  /panda/franka_states
```

Press `Ctrl+C` to stop recording when done.


2. Start spacemouse teleoperation:

```bash
cd src/multipanda_ros2
python3 spacemouse_teleop/spacemouse_pub_singlearm_real.py
```

---
1. Launch joint controller:


