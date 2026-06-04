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
  load_gripper:=true \
  controller_name:=joint_impedance_controller \
  use_rviz:=false
```

**Controller Interfaces:**
| Topic / Interface Name | Message Type / Interface Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/<arm_id>/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Real-time joint states used by the controller. |
| `/<arm_id>/joints_desired` | `sensor_msgs/msg/JointState` | Input (Sub) | 100Hz recommended | Desired joint positions and velocities. `position[0..6]` are the target joint positions, and `velocity[0..6]` are the target joint velocities. |
| `/<arm_id>/filtered_joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Internal state-space filtered commands. Set `pub_filt_state=true` in the controller config to enable this topic. |
| `/<arm_id>/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 1000Hz | External wrench estimated from the Franka robot state. `wrench.force.xyz` is force and `wrench.torque.xyz` is torque. |


**Controller Parameters:**
| Parameter Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_id` | `string` | `panda` | Arm namespace used to resolve interfaces and robot model state. |
| `k_gains` | `vector<double>` | see config | Joint position stiffness gains. Must contain 7 values. |
| `d_gains` | `vector<double>` | see config | Joint damping gains. Must contain 7 values. |
| `k_filt` | `vector<double>` | see config | State-space filter stiffness gains for the desired joint trajectory. Must contain 7 values. |
| `d_filt` | `vector<double>` | see config | State-space filter damping gains for the desired joint trajectory. Must contain 7 values. |
| `dq_max` | `vector<double>` | see config | Maximum absolute filtered joint velocity. Must contain 7 values. |
| `ddq_max` | `vector<double>` | see config | Maximum absolute filtered joint acceleration. Must contain 7 values. |
| `pub_filt_state` | `bool` | `false` | Publish `/<arm_id>/filtered_joint_states` when enabled. |

The controller first filters the desired joint commands with a second-order state-space model:

$$
\ddot q_{filt} = \mathrm{clip}\left(k_{filt}(q_d - q_{filt}) + d_{filt}(\dot q_d - \dot q_{filt}), -\ddot q_{max}, \ddot q_{max}\right)
$$

$$
\dot q_{filt} \leftarrow \mathrm{clip}(\dot q_{filt} + \ddot q_{filt} \Delta t, -\dot q_{max}, \dot q_{max})
$$

$$
q_{filt} \leftarrow q_{filt} + \dot q_{filt} \Delta t
$$

The torque command is then computed from the filtered states:

$$
τ = K_p(q_{filt} - q) + K_d(\dot q_{filt} - \dot q) + τ_{coriolis}
$$

For low-frequency references, the filter delay can be estimated empirically from `k_filt` and `d_filt`:

Without velocity feedforward:
$$
t_{delay} \approx \frac{d_{filt}}{k_{filt}}
$$

With velocity feedforward, $\omega \ll \sqrt{k_{filt}}$
$$
t_{delay} \approx \frac{d_{filt}}{k_{filt}^2} \omega^2 \quad 
$$


> The state-space filter allows the upper-level Policy Controller to send commands directly to the 1000 Hz controller at a low frequency (for example, 10 Hz).


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
| `/dual_arm/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Real-time joint states for both arms|
| `/<arm_id>/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Real-time joint states for each arm, published separately per namespace. |
| `/<arm_id>/joints_desired` | `sensor_msgs/msg/JointState` | Input (Sub) | Event-driven / 100Hz recommended | Desired joint positions and velocities per arm. Names must match `arm_id_joint1..7`. |
| `/<arm_id>/filtered_joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 1000Hz | Internal state-space filtered commands for each arm (enabled with `arm_i.pub_filt_state`). |
| `/<arm_id>/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 1000Hz | External wrench estimated from each arm's robot state. `wrench.force.xyz` is force and `wrench.torque.xyz` is torque. |


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
  use_rviz:=false
```

**Camera Topics:**

| Camera | Compressed topic | Message Type |
| :--- | :--- | :--- |
| `d405_left` | `/cameras/d405_left/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `d405_right` | `/cameras/d405_right/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `d435f` | `/cameras/d435f/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |

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
