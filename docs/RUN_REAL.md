

## Prerequisites

Ubuntu 22.04 LTS + ROS 2 Humble

For real robot experiments, a real-time kernal is required. Please refer to [Franka Control Inferface](https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html).




## Single arm controller:

Ruturn to home position:




Launch robot controller:
``` bash
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.3 \
  load_gripper:=true \
  controller_name:=cartesian_impedance_controller \
  use_rviz:=false
```










