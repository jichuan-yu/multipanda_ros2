

## Prerequisites

Ubuntu 22.04 LTS + ROS 2 Humble

For real robot experiments, a real-time kernal is required. Please refer to [Franka Control Inferface](https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html).




---
## SpaceMouse Teleoperation With Cartesian Impedance Controller (Not Recommended):

0. Ruturn to home position (0, -PI/4, 0, -3PI/4, 0, PI/2, PI/4):
```bash
ros2 launch franka_bringup move_to_start.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true
```

1. Launch cartesian impedance controller:
``` bash
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.3 \
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
  /panda/ee_pose \
  /cartesian_impedance/pose_desired \
  /panda/external_wrench
```

Press `Ctrl+C` to stop recording when done.


2. Start spacemouse teleoperation:

```bash
cd src/multipanda_ros2
python3 spacemouse_teleop/spacemouse_pub_singlearm_real.py
```

---
## SpaceMouse Teleoperation With IK + Joint Impedance Controller (Recommended):

0. Ruturn to home position (0, -PI/4, 0, -3PI/4, 0, PI/2, PI/4):
```bash
ros2 launch franka_bringup move_to_start.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true
```

1. Launch joint controller:
``` bash
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true \
  controller_name:=joint_impedance_controller \
  use_rviz:=false
```

Set Collision Behavior in a new terminal
```bash
bash src/multipanda_ros2/franka_hardware/param_setter_scripts.sh panda
```

(Optional) Record rosbag data in a new terminal:
```bash
# Record all topics (recommended for full data capture)
ros2 bag record -o ./data/$(date +%Y%m%d_%H%M%S) --all

# Or record specific topics only
ros2 bag record -o ./data/$(date +%Y%m%d_%H%M%S) \
  /panda/joint_states \
  /panda/filtered_joint_states \
  /panda/external_wrench \
  /panda_gripper/joint_states \
  /panda_gripper/width_desired \
  /panda_gripper/grasp_desired \
  /panda/joints_desired 
```
Press `Ctrl+C` to stop recording when done.

2. SpaceMouse Teleop:
```bash
conda activate panda
cd src/multipanda_ros2
python3 spacemouse_teleop/spacemouse_pub_singlearm_joint.py
```



## Dual-Arm Joint Impedance Controller
0. Ruturn to home position (0, -PI/4, 0, -3PI/4, 0, PI/2, PI/4):
```bash
ros2 launch franka_bringup move_to_start.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true
```
```bash
ros2 launch franka_bringup move_to_start.launch.py \
  robot_ip:=172.16.0.3 \
  load_gripper:=true
```
1. Launch joint controller:
```bash
ros2 launch franka_bringup dual_franka_control.launch.py \
  robot_ip_1:=172.16.0.3 \
  robot_ip_2:=172.16.0.2 \
  arm_id_1:=panda_left \
  arm_id_2:=panda_right \
  load_gripper_1:=true \
  load_gripper_2:=true \
  controller_name:=dual_joint_impedance_controller \
  use_rviz:=false
```

```bash
ros2 launch franka_bringup dual_franka_sim_control.launch.py \
  controller_name:=dual_joint_impedance_controller \
  use_rviz:=false
```

2. SpaceMouse Teleop:
First check connected spacemouse devices:
```bash
conda activate panda
cd src/multipanda_ros2
python3 spacemouse_teleop/list_spacemouse.py
```
It should output:
```bash
Found 2 SpaceMouse device(s) (16 HID interfaces total):

  [0] path=/dev/hidraw6
       vendor_id=0x256F  product_id=0xC635
       manufacturer='3Dconnexion'  product='SpaceMouse Compact'

  [1] path=/dev/hidraw5
       vendor_id=0x256F  product_id=0xC635
       manufacturer='3Dconnexion'  product='SpaceMouse Compact'
```

Add the paths to the `DualArmTeleopConfig` in `spacemouse_pub_dualarm_joint.py`, then run:
```bash
python3 spacemouse_teleop/spacemouse_pub_dualarm_joint.py
```



## Dual-Arm Joint Impedance Controller With RealSense Cameras

Launch Controller
```bash
ros2 launch franka_bringup dual_franka_control_with_realsense.launch.py \
  robot_ip_1:=172.16.0.3 \
  robot_ip_2:=172.16.0.2 \
  arm_id_1:=panda_left \
  arm_id_2:=panda_right \
  load_gripper_1:=true \
  load_gripper_2:=true \
  controller_name:=dual_joint_impedance_controller \
  use_rviz:=false
```

Visualize camera topics:
```bash
ros2 run rqt_image_view rqt_image_view
```


Topics:
```bash
```




