# ROS 2 Controller for Franka Panda

This repository is a fork of [multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2.git). For the original documentation and installation guide, see [README_ORIGIN.md](./README_ORIGIN.md).


## Installation

Follow the guidance in [README_ORIGIN.md](./README_ORIGIN.md).

Note:

libfranka may crash when used with Eigen >= 3.4.0. To avoid this, install Eigen 3.3.9 as follows:

```bash
cd ~/libraries
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -xzf eigen-3.3.9.tar.gz
cd eigen-3.3.9

mkdir build && cd build
cmake ..
sudo make install
```

The Eigen headers will be installed to /usr/local/include/eigen3/Eigen

When building libfranka, use the following commands:

```bash
cd ~/libraries/libfranka
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTS=OFF \
  -DEigen3_DIR=/usr/local/share/eigen3/cmake \
  -DCMAKE_INSTALL_PREFIX=$HOME/libraries/libfranka

cmake --build . -j"$(nproc)"

cmake --install .
```



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

```bash
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
| `/<arm_id>/ee_pose` | `geometry_msgs/msg/PoseStamped` | Output (Pub) | 1000Hz | Current pose for the end-effector (in robot base frame). |
| `/<arm_id>/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 1000Hz | Current external wrench of the robot in the base frame. `wrench.force.xyz` is external force and `wrench.torque.xyz` is external torque. |

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

```bash
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.3 \
  arm_id:=panda \
  load_gripper:=true \
  controller_name:=joint_impedance_controller \
  use_rviz:=false
```

**Controller Interfaces:**
| Topic / Interface Name | Message Type / Interface Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/<arm_id>/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 100Hz by YAML default | Real-time joint states used by the controller. |
| `/<arm_id>/joints_desired` | `sensor_msgs/msg/JointState` | Input (Sub) | 100Hz recommended | Desired joint positions and velocities. `position[0..6]` are the target joint positions, and `velocity[0..6]` are the target joint velocities. |
| `/<arm_id>/filtered_joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 100Hz by YAML default | Filtered desired commands. `position[0..6]` are filtered `q_d_target`, and `velocity[0..6]` are filtered `dq_d_target`. |
| `/<arm_id>/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 100Hz by YAML default | External wrench estimated from the Franka robot state. `wrench.force.xyz` is force and `wrench.torque.xyz` is torque. |


**Controller Parameters:**
| Parameter Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_id` | `string` | `panda` | Arm namespace used to resolve interfaces and robot model state. |
| `k_gains` | `vector<double>` | see config | Joint position stiffness gains. Must contain 7 values. |
| `d_gains` | `vector<double>` | see config | Joint damping gains. Must contain 7 values. |
| `e_q_max` | `vector<double>` | see config | Maximum absolute position tracking error used by the impedance law. Must contain 7 values. |
| `alpha_filt` | `double` | `0.1` | First-order low-pass coefficient for desired position and velocity targets. The controller clamps it to `[0, 1]`; `1.0` disables filtering. |
| `publish_rate` | `double` | see config | Publish rate for `joint_states`, `filtered_joint_states`, and `external_wrench`. |
| `control_frequency` | `double` | `1000.0` | Nominal controller update frequency used to quantize `publish_rate` into an integer cycle interval. |

The controller receives raw desired commands as `q_{d,raw}` and `\dot q_{d,raw}`. Each control cycle, it filters both desired position and desired velocity with a first-order IIR filter:

$$
q_d[k] = (1-\alpha_{filt}) q_d[k-1] + \alpha_{filt} q_{d,raw}[k]
$$

$$
\dot q_d[k] = (1-\alpha_{filt}) \dot q_d[k-1] + \alpha_{filt} \dot q_{d,raw}[k]
$$

For controller frequency `f_c` and cutoff frequency `f_{cut}`, a useful small-frequency approximation is:

$$
\alpha_{filt} \approx \frac{2\pi f_{cut}}{f_c}
$$

The position error is clipped before applying stiffness; the velocity error is not clipped:

$$
e_q = \mathrm{clip}(q_d - q, -e_{q,max}, e_{q,max})
$$

$$
e_{\dot q} = \dot q_d - \dot q
$$

The torque command is:

$$
τ = K_p e_q + K_d e_{\dot q} + τ_{coriolis}
$$

> `/<arm_id>/filtered_joint_states` is published by default and contains the filtered desired target, not the measured robot state.


### Dual Arm Joint Impedance Controller
Run in MuJoCo Simulation:
```bash
ros2 launch franka_bringup dual_franka_sim_control.launch.py \
  arm_id_1:=mj_left \
  arm_id_2:=mj_right \
  controller_name:=dual_joint_impedance_controller \
  use_rviz:=false
```


Run on real robot:
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

**Controller Interfaces:**
| Topic / Interface Name | Message Type / Interface Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/dual_arm/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Real-time joint states for both arms.|
| `/<arm_id>/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 100Hz by YAML default | Real-time joint states for each arm, published separately per namespace. |
| `/<arm_id>/joints_desired` | `sensor_msgs/msg/JointState` | Input (Sub) | Event-driven / 100Hz recommended | Desired joint positions and velocities per arm. Names must match `arm_id_joint1..7`. |
| `/<arm_id>/filtered_joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 100Hz by YAML default | Filtered desired commands for each arm. `position[0..6]` are filtered `q_d_target`, and `velocity[0..6]` are filtered `dq_d_target`. |
| `/<arm_id>/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 100Hz by YAML default | External wrench estimated from each arm's robot state. `wrench.force.xyz` is force and `wrench.torque.xyz` is torque. |

**Controller Parameters:**
| Parameter Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_count` | `int` | see config | Number of arms managed by the controller. |
| `control_frequency` | `double` | `1000.0` | Nominal controller update frequency used to quantize `publish_rate`. |
| `publish_rate` | `double` | see config | Publish rate for per-arm `joint_states`, `filtered_joint_states`, and `external_wrench`. |
| `arm_i.arm_id` | `string` | see config | Arm namespace for arm `i`. |
| `arm_i.k_gains` | `vector<double>` | see config | Joint position stiffness gains for arm `i`. Must contain 7 values. |
| `arm_i.d_gains` | `vector<double>` | see config | Joint damping gains for arm `i`. Must contain 7 values. |
| `arm_i.e_q_max` | `vector<double>` | see config | Maximum absolute position tracking error used by the impedance law for arm `i`. Must contain 7 values. |
| `arm_i.alpha_filt` | `double` | `0.1` | First-order low-pass coefficient for desired position and velocity targets for arm `i`. The controller clamps it to `[0, 1]`. |

The dual-arm controller applies the same filtering and torque law independently for each arm:

$$
q_d[k] = (1-\alpha_{filt}) q_d[k-1] + \alpha_{filt} q_{d,raw}[k]
$$

$$
\dot q_d[k] = (1-\alpha_{filt}) \dot q_d[k-1] + \alpha_{filt} \dot q_{d,raw}[k]
$$

$$
e_q = \mathrm{clip}(q_d - q, -e_{q,max}, e_{q,max}), \quad
e_{\dot q} = \dot q_d - \dot q
$$

$$
τ = K_p e_q + K_d e_{\dot q} + τ_{coriolis}
$$

> Potential Bug: In `dual_franka_sim.launch.py`, the controller's output `/joint_states` cannot be remapped to `/dual_arm/joint_states`, causing both `/joint_state_publisher` and `/joint_state_broadcaster` to publish to `/joint_states` simultaneously, leading to confused messages.

#### Dual Arm Joint Impedance Controller With 3 RealSense Cameras

```bash
ros2 launch franka_bringup dual_franka_control_with_realsense.launch.py \
  robot_ip_1:=172.16.0.3 \
  robot_ip_2:=172.16.0.2 \
  arm_id_1:=panda_left \
  arm_id_2:=panda_right \
  load_gripper_1:=true \
  load_gripper_2:=true \
  controller_name:=dual_joint_impedance_controller \
  launch_rqt_image_view:=true \
  apply_collision_params:=true \
  use_rviz:=false
```

**Camera Topics:**

| Camera | Compressed topic | Message Type |
| :--- | :--- | :--- |
| `<arm_id_1>_wrist` | `/cameras/<arm_id_1>_wrist/color/image_raw` with `image_transport:=compressed` | `sensor_msgs/msg/CompressedImage` |
| `<arm_id_2>_wrist` | `/cameras/<arm_id_2>_wrist/color/image_raw` with `image_transport:=compressed` | `sensor_msgs/msg/CompressedImage` |
| `fixed` | `/cameras/fixed/color/image_raw` with `image_transport:=compressed` | `sensor_msgs/msg/CompressedImage` |

- **Default launch resolution**: color and depth are configured to `640x480 @ 30Hz` by default (see launch profiles in [franka_bringup/launch/real/dual_franka_control_with_realsense.launch.py](franka_bringup/launch/real/dual_franka_control_with_realsense.launch.py#L81-L86)).

**Gripper Topics:**

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/panda_left_gripper/grasp_desired` | `std_msgs/msg/Float64MultiArray` | Grasp command (width, speed, force, epsilon) |
| `/panda_left_gripper/joint_states` | `sensor_msgs/msg/JointState` | Left gripper joint states |
| `/panda_left_gripper/width_desired` | `std_msgs/msg/Float64` | Left gripper width command |
| `/panda_right_gripper/grasp_desired` | `std_msgs/msg/Float64MultiArray` | Grasp command for right gripper |
| `/panda_right_gripper/joint_states` | `sensor_msgs/msg/JointState` | Right gripper joint states |
| `/panda_right_gripper/width_desired` | `std_msgs/msg/Float64` | Right gripper width command |

#### Single Arm Joint Impedance Controller With 2 RealSense Cameras

```bash
ros2 launch franka_bringup franka_control_with_realsense.launch.py \
  robot_ip:=172.16.0.2 \
  arm_id:=panda_right \
  load_gripper:=true \
  controller_name:=joint_impedance_controller \
  wrist_camera:=right \
  launch_rqt_image_view:=true \
  apply_collision_params:=true \
  use_rviz:=false
```

**Launch Arguments (additional to `franka_control.launch.py`):**

| Argument | Default | Description |
| :--- | :--- | :--- |
| `arm_id` | `panda` | Arm namespace used for URDF, controller interfaces, gripper bridge, and wrist camera topic. |
| `launch_realsense` | `true` | Whether to launch RealSense camera nodes together with robot bringup. |
| `wrist_camera` | `left` | Wrist camera selector. Must be `left` or `right`. The launch file maps it to built-in D405 serial numbers, otherwise it raises an error. |

**Camera Topics:**

| Camera | Compressed topic | Message Type |
| :--- | :--- | :--- |
| `<arm_id>_wrist` (from `wrist_camera=left/right`) | `/cameras/<arm_id>_wrist/color/image_raw` with `image_transport:=compressed` | `sensor_msgs/msg/CompressedImage` |
| `fixed` | `/cameras/fixed/color/image_raw` with `image_transport:=compressed` | `sensor_msgs/msg/CompressedImage` |

- **Default launch resolution**: color and depth are configured to `640x480 @ 30Hz` by default (see launch profiles in [franka_bringup/launch/real/franka_control_with_realsense.launch.py](franka_bringup/launch/real/franka_control_with_realsense.launch.py)).


## Spacemouse Teleoperation

[PySpaceMouse](https://github.com/JakubAndrysek/PySpaceMouse.git)
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
[GELLO: General, Low-Cost, and Intuitive Teleoperation Framework](https://github.com/wuphilipp/gello_software.git)
### Installation:



### Examples:
