# ROS 2 Controller for Franka Panda

This repository is a fork of [multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2.git). For the original documentation and installation guide, see [README_ORIGIN.md](./README_ORIGIN.md).




## Controllers and Communication Interfaces



- `cartesian_impedance_controller`
``` bash
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true \
  controller_name:=cartesian_impedance_controller \
  use_rviz:=false
```

| Topic Name | Message Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/panda/joint_states` (from joint_state_broadcaster) | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Real-time joint states. |
| `/panda_gripper/joint_states` (from panda_gripper) | `sensor_msgs/msg/JointState` | Output (Pub) | ~17 Hz (state_publish_rate should be 50Hz, see franka_gripper/config)| Gripper joint states. |
| `/cartesian_impedance/target_pose` | `geometry_msgs/msg/PoseStamped` | Input (Sub) | 100Hz recommended | Target pose for the end-effector (in robot base frame) |
| `/cartesian_impedance/ee_pose` | `geometry_msgs/msg/PoseStamped` | Output (Pub) | 1000Hz | Current  pose for the end-effector (in robot base frame) |
| `/cartesian_impedance/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 1000Hz | Current external wrench of the robot in the base frame. `wrench.force.xyz` is external force and `wrench.torque.xyz` is external torque. |




## Spacemouse Teleoperation

#### Installation:
1. Install hidapi:
```bash
sudo apt-get install libhidapi-dev
```

2. Add Linux permissions:
```bash
echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-hidraw-permissions.rules
sudo usermod -aG plugdev $USER
newgrp plugdev
```

3. Install pyspacemouse:
```bash
pip install pyspacemouse
```


