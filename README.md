# ROS 2 Controller for Franka Panda

This repository is a fork of [multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2.git). For the original documentation and installation guide, see [README_ORIGIN.md](./README_ORIGIN.md).




## Controllers and Communication Interfaces

### Gripper Control
We developed a [gripper action bridge](src/multipanda_ros2/franka_example_controllers/src/subscriber/gripper_action_bridge.cpp) to provide a more convenient control interface.

**Controller Interfaces:**
| Topic Name | Message Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/<arm_id>_gripper/width_desired` | `std_msgs/msg/Float64` | Input (Sub) | Event-driven | External move command for target gripper width. |
| `/<arm_id>_gripper/grasp_desired` | `std_msgs/msg/Float64MultiArray` | Input (Sub) | Event-driven | External grasp command for width, speed, force, and epsilon values. |
| `/<arm_id>_gripper/homing_desired` | `std_msgs/msg/Bool` | Input (Sub) | Event-driven | External homing command for gripper calibration. |
| `/<arm_id>_gripper/stop_desired` | `std_msgs/msg/Bool` | Input (Sub) | Event-driven | External stop command for gripper motion. |
| `/<arm_id>_gripper/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | ~17 Hz (state_publish_rate should be 50Hz, see franka_gripper/config)| Gripper joint states. |

For more examples, please refer to [gripper_control](./docs/gripper_control.md)

### Cartesian Impedance Controller (Single Arm)

``` bash
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true \
  controller_name:=cartesian_impedance_controller \
  use_rviz:=false
```

**Controller Interfaces:**
| Topic Name | Message Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/<arm_id>/joint_states` (from joint_state_broadcaster) | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Real-time joint states. |
| `/cartesian_impedance/pose_desired` | `geometry_msgs/msg/PoseStamped` | Input (Sub) | 100Hz recommended | Desired pose for the end-effector (in robot base frame) |
| `/cartesian_impedance/ee_pose` | `geometry_msgs/msg/PoseStamped` | Output (Pub) | 1000Hz | Current  pose for the end-effector (in robot base frame) |
| `/cartesian_impedance/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 1000Hz | Current external wrench of the robot in the base frame. `wrench.force.xyz` is external force and `wrench.torque.xyz` is external torque. |

**Controller Parameters:**
| Parameter Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_id` | `string` | `panda` | Arm namespace used to resolve interfaces and robot model state. |
| `pos_stiff` | `vector<double>` | see config | Cartesian stiffness gains as a 6-element vector: translational (X,Y,Z) then rotational (Rx,Ry,Rz). |
| `n_stiffness` | `vector<double>` | see_config | Null-space stiffness per joint (7 values) used for posture regulation toward `desired_qn`. |

The control law is implemented as:

$$
τ = τ_{task} + τ_{coriolis} + τ_{null}
$$

$$
τ_{task} = J^T\left(-K e - D(J\dot q)\right)
$$

$$
τ_{null} = \left(I - J^T J^{\dagger}\right)
\left(n_{stiffness}(q_d^n - q) - 2\sqrt{n_{stiffness}}\,\dot q\right)
$$


> With `pos_stiff: [2000.0, 2000.0, 2000.0, 50.0, 50.0, 50.0], n_stiffness: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]`,the positioning error is approximately 5 mm / 0.03 rad

### Joint Impedance Controller (Single Arm)

**Controller Interfaces:**
| Topic / Interface Name | Message Type / Interface Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/<arm_id>/joint_states` (from joint_state_broadcaster) | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Real-time joint states used by the controller. |
| `/joint_impedance/joints_desired` | `sensor_msgs/msg/JointState` | Input (Sub) | 100Hz recommended | Desired joint positions and velocities. `position[0..6]` are the target joint positions, and `velocity[0..6]` are the target joint velocities. |


**Controller Parameters:**
| Parameter Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_id` | `string` | `panda` | Arm namespace used to resolve interfaces and robot model state. |
| `k_gains` | `vector<double>` | see config | Joint position stiffness gains. Must contain 7 values. |
| `d_gains` | `vector<double>` | see config | Joint damping gains. Must contain 7 values. |
| `alpha`  (internal constant)| `double` | `0.24` | Low-pass filter coefficient (~50 Hz) for measured joint velocity. |
| `pos_saturation`  (internal constant)| `double` | `0.2 rad` | Saturation bound for position error `q_d - q`. |
| `vel_saturation`  (internal constant)| `double` | `0.5 rad/s` | Saturation bound for velocity error `dq_d - dq`. |

The control law is implemented as:

$$
τ = K_p\,\mathrm{sat}(q_d - q) + K_d\,\mathrm{sat}(\dot q_d - \dot q) + τ_{coriolis}
$$




## Spacemouse Teleoperation

### Installation:
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

3. Install Python Dependencies:
```bash
# Create conda environment (Python 3.10)
conda create -n panda python=3.10 -y

# Activate the environment
conda activate panda

# Install Pinocchio and (preferably) pink from conda-forge
conda install -c conda-forge pinocchio pink -y

# Install pyspacemouse via pip
pip install pyspacemouse
```

### Examples:




## GELLO Teleoperation
### Installation:



### Examples:
