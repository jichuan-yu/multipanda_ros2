# ROS 2 Controller for Franka Panda

This repository is a fork of [multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2.git). For the original documentation and installation guide, see [README_ORIGIN.md](./README_ORIGIN.md).

This README describes the merged `dev-jichuan` branch. Real-robot control follows
`dev-jichuan`; simulation configuration and the MuJoCo submodule come from the
simulation branch. Task scenes and experiment orchestration live in the adjacent
`dualarm_mprc` repository. The `brickcraft-duo` release pins the earlier real-control
baseline and does not include the subsequent simulation merge.


## Installation

Follow the guidance in [README_ORIGIN.md](./README_ORIGIN.md).

For this fork, clone the documented branch into a ROS 2 Humble workspace:

```bash
mkdir -p ~/dual_panda_ws/src
cd ~/dual_panda_ws/src
git clone --branch dev-jichuan https://github.com/jichuan-yu/multipanda_ros2.git
git -C multipanda_ros2 submodule update --init --recursive
```

`mujoco_ros_pkgs` is the only configured submodule. Use the commit recorded by
the parent repository; its configured remote branch is `srt_xzy`. libfranka and
MuJoCo are supplied through external installation prefixes.

The local libfranka build uses Eigen 3.3.9. To install that header version:

```bash
mkdir -p ~/libraries
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

Real-robot control launch files start
[`gripper_action_bridge_real`](franka_example_controllers/src/subscriber/gripper_action_bridge_real.cpp)
when `load_gripper` (or the corresponding per-arm flag) is true. Pass the full
arm ID, such as `panda`, `panda_left`, or `panda_right`. Command topics below are
handled by the bridge; `joint_states` comes from the gripper action server.

**Controller Interfaces:**

| Topic Name | Message Type | Direction | Freq. | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/<arm_id>_gripper/width_desired` | `std_msgs/msg/Float64` | Input (Sub) | Event-driven | External move command for target gripper width. |
| `/<arm_id>_gripper/grasp_desired` | `std_msgs/msg/Float64MultiArray` | Input (Sub) | Event-driven | External grasp command for width, speed, force, and epsilon values. |
| `/<arm_id>_gripper/homing_desired` | `std_msgs/msg/Bool` | Input (Sub) | Event-driven | External homing command for gripper calibration. |
| `/<arm_id>_gripper/stop_desired` | `std_msgs/msg/Bool` | Input (Sub) | Event-driven | External stop command for gripper motion. |
| `/<arm_id>_gripper/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | Configured timer rate; see below | Gripper joint states. |

`width_desired` uses a fixed move speed of `0.1 m/s`. `grasp_desired` accepts
`[width, speed, force, epsilon_inner, epsilon_outer]`; omitted trailing values
default to `[0.0, 0.1, 10.0, 0.005, 0.005]`. Homing and stop forward a command
only when `Bool.data` is true. Real actions are `/<arm_id>_gripper/move`,
`grasp`, and `homing`; stop is a `std_srvs/srv/Trigger` service at
`/<arm_id>_gripper/stop`.

The gripper server declares state/feedback timer rates of `30/10 Hz`.
[franka_gripper_node.yaml](franka_gripper/config/franka_gripper_node.yaml)
overrides them to `50/30 Hz` for the node named `panda_gripper` only. Renamed
nodes such as `panda_left_gripper` retain the declared defaults unless given
matching overrides. These are configured rates, not measured throughput.

Simulation keeps
[`gripper_action_bridge`](franka_example_controllers/src/subscriber/gripper_action_bridge.cpp).
For dual-arm simulation, pass `arm_id:=left` or `arm_id:=right`: command topics
are `/mj_<arm_id>_gripper/*_desired`, actions/services are under
`/mj_<arm_id>_gripper_sim_node/`, and joint states are published at
`/mj_<arm_id>_gripper_sim_node/joint_states`. Grasp commands optionally append a
sixth value, `request_id`; `/mj_<arm_id>_gripper/grasp_status` publishes
`[request_id, state]`, where `1=accepted`, `2=success`, `-1=not accepted/server
unavailable`, and `-2=execution failed`. Five-element commands use request ID `0`.

See [gripper_control](./docs/gripper_control.md) for simulation topic examples.

### Launch and Parameter Conventions

The parameter tables below show the **real-robot YAML configuration**, unless
explicitly labeled as C++ declaration defaults or simulation overrides. The
controller manager's `update_rate` is `1000 Hz`; `control_frequency` is a separate
parameter used to calculate publication intervals and does not set the loop rate.

| Launch file | Default controller | Default arm IDs | Gripper loaded by default |
| :--- | :--- | :--- | :--- |
| [franka_control.launch.py](franka_bringup/launch/real/franka_control.launch.py) | `cartesian_impedance_controller` | `panda` | `false` |
| [dual_franka_control.launch.py](franka_bringup/launch/real/dual_franka_control.launch.py) | `dual_joint_impedance_controller` | `panda_left`, `panda_right` | Both `true` |
| [franka_control_with_realsense.launch.py](franka_bringup/launch/real/franka_control_with_realsense.launch.py) | `joint_impedance_controller` | `panda` | `false` |
| [dual_franka_control_with_realsense.launch.py](franka_bringup/launch/real/dual_franka_control_with_realsense.launch.py) | `dual_joint_impedance_controller` | `panda_left`, `panda_right` | Both `true` |

Real-robot IP arguments are required. `use_rviz`, `use_fake_hardware`, and
`fake_sensor_commands` default to `false`. The examples explicitly enable the
gripper and select a controller. Single-arm launch propagates `arm_id` through
the URDF and YAML substitutions. The dual-arm controller YAML fixes IDs to
`panda_left`/`panda_right`; changing only the dual-arm launch arguments does not
rewrite these controller parameters.



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
| `/joint_states` (from joint_state_broadcaster) | `sensor_msgs/msg/JointState` | Output (Pub) | Nominal 1000 Hz | Measured joint states; the Cartesian controller does not publish a separate `/<arm_id>/joint_states`. |
| `/cartesian_impedance/pose_desired` | `geometry_msgs/msg/PoseStamped` | Input (Sub) | 100Hz recommended | Desired pose for the end-effector (in robot base frame) |
| `/<arm_id>/ee_pose` | `geometry_msgs/msg/PoseStamped` | Output (Pub) | 1000Hz | Current pose for the end-effector (in robot base frame). |
| `/<arm_id>/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 1000Hz | Current external wrench of the robot in the base frame. `wrench.force.xyz` is external force and `wrench.torque.xyz` is external torque. |

**Controller Parameters:**

| Parameter Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_id` | `string` | `panda` | Arm namespace used to resolve interfaces and robot model state. |
| `pos_stiff` | `vector<double>` | `[2000.0, 2000.0, 2000.0, 50.0, 50.0, 50.0]` | Cartesian stiffness: translational (X,Y,Z), then rotational (Rx,Ry,Rz). |
| `n_stiffness` | `vector<double>` | `[2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]` | Per-joint null-space stiffness toward the joint configuration captured on activation. |

These values come from
[single_controllers.yaml](franka_bringup/config/real/single_controllers.yaml).
The C++ declaration defaults are `pos_stiff=[100.0, 100.0, 100.0, 10.0, 10.0, 10.0]`
and seven `10.0` values for `n_stiffness`. The single-arm simulation YAML uses
those lower stiffness values. The controller captures the current end-effector
pose on activation; target messages are interpreted in the robot base frame.

The control law is implemented as:

$$
τ = τ_{task} + τ_{coriolis} + τ_{null}
$$

$$
τ_{task} = J^T\left(-K e - D(J\dot q)\right)
$$

$$
τ_{null} = \left(I - J^T (J^T)^{\dagger}\right)
\left(K_n(q_d^n - q) - D_n\dot q\right)
$$


Here `K_n=diag(n_stiffness)` and `D_n=diag(2*sqrt(n_stiffness))`.
Cartesian damping is `2*sqrt(k)` on translational axes and `0.5*2*sqrt(k)`
on rotational axes. The implementation uses a damped pseudoinverse with
`lambda=0.1` and clips each commanded joint torque to within `1.0 Nm` of the
previous robot command `tau_J_d` per update. Stiffness settings alone do not
specify a guaranteed tracking error.

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
| `k_gains` | `vector<double>` | `[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 100.0]` | Joint position stiffness gains. Must contain 7 values. |
| `d_gains` | `vector<double>` | `[32.0, 32.0, 32.0, 28.0, 16.0, 12.0, 10.0]` | Joint damping gains. Must contain 7 values. |
| `e_q_max` | `vector<double>` | `[0.02, 0.02, 0.02, 0.02, 0.03, 0.03, 0.03]` | Maximum absolute position error applied by the impedance law, in rad. Must contain 7 values. |
| `alpha_filt` | `double` | `0.05` | First-order low-pass coefficient for desired position and velocity targets. Clamped to `[0, 1]`; `1.0` disables filtering. |
| `publish_rate` | `double` | `100.0` | Requested publish rate for `joint_states`, `filtered_joint_states`, and `external_wrench`. |
| `control_frequency` | `double` | `1000.0` | Nominal controller update frequency used to quantize `publish_rate` into an integer cycle interval. |

Configuration source:
[single_controllers.yaml](franka_bringup/config/real/single_controllers.yaml).
Without YAML overrides, C++ declares `alpha_filt=1.0` and `publish_rate=1000.0`;
the three gain/error arrays are empty and must be supplied. Publication is
quantized to `max(1, round(control_frequency / publish_rate))` update cycles;
`publish_rate <= 0` publishes each update in this single-arm implementation.
Realtime publishers may skip samples when their lock is unavailable.

Input `JointState.name` must include `<arm_id>_joint1` through
`<arm_id>_joint7`, with one position per name. Entries are matched by name, so
their order may differ. Velocity may be empty (treated as zero) or contain one
value per name. `header.stamp` is not used to schedule the target.
Targets remain latched between messages. Activation initializes desired
positions from the measured joints and desired velocities to zero. The per-arm
`joint_states` and `filtered_joint_states` publishers fill `effort` with zeros;
use the hardware broadcaster for measured joint torque.

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

The single-arm controller applies the same `1.0 Nm` per-update torque clipping
relative to `tau_J_d` as the Cartesian controller.


### Dual Arm Joint Impedance Controller
Run in MuJoCo Simulation. The base launch starts the simulator and
`joint_state_broadcaster`; spawn the impedance controller in a second terminal
after `/controller_manager` is available:

```bash
ros2 launch franka_bringup dual_franka_sim.launch.py \
  arm_id_1:=mj_left \
  arm_id_2:=mj_right \
  use_rviz:=false
```

```bash
ros2 run controller_manager spawner dual_joint_impedance_controller \
  --controller-manager /controller_manager
```

The removed `dual_franka_sim_control.launch.py` and
`franka_control_sim.launch.py` wrappers are not available on this branch.
The base simulation launch does not start gripper bridges or task controllers.
The included server defaults to `unpause=false`; resume simulation in the viewer
to advance the dynamics.
For task scenes, use the adjacent project's
[task bringup package](https://github.com/jichuan-yu/dualarm_mprc/tree/dev-Jerry/dualarm_task_bringup).

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
| `/dual_arm/joint_states` (real) / `/joint_states` (simulation) | `sensor_msgs/msg/JointState` | Output (Pub) | Nominal 1000 Hz | Both arms from joint_state_broadcaster; routing depends on the base launch. |
| `/<arm_id>/joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 100Hz by YAML default | Real-time joint states for each arm, published separately per namespace. |
| `/<arm_id>/joints_desired` | `sensor_msgs/msg/JointState` | Input (Sub) | Event-driven / 100Hz recommended | Desired joint positions and velocities per arm. Names must match `arm_id_joint1..7`. |
| `/<arm_id>/filtered_joint_states` | `sensor_msgs/msg/JointState` | Output (Pub) | 100Hz by YAML default | Filtered desired commands for each arm. `position[0..6]` are filtered `q_d_target`, and `velocity[0..6]` are filtered `dq_d_target`. |
| `/<arm_id>/external_wrench` | `geometry_msgs/msg/WrenchStamped` | Output (Pub) | 100Hz by YAML default | External wrench estimated from each arm's robot state. `wrench.force.xyz` is force and `wrench.torque.xyz` is torque. |

**Controller Parameters:**

| Parameter Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_count` | `int` | `2` | Number of arms managed by the controller. |
| `control_frequency` | `double` | `1000.0` | Nominal controller update frequency used to quantize `publish_rate`. |
| `publish_rate` | `double` | `100.0` | Requested publish rate for per-arm `joint_states`, `filtered_joint_states`, and `external_wrench`. |
| `arm_i.arm_id` | `string` | `panda_left` / `panda_right` | Arm namespace for `i=1` / `i=2`. |
| `arm_i.k_gains` | `vector<double>` | `[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 100.0]` | Per-arm joint position stiffness; 7 values. |
| `arm_i.d_gains` | `vector<double>` | `[32.0, 32.0, 32.0, 28.0, 16.0, 12.0, 10.0]` | Per-arm joint damping; 7 values. |
| `arm_i.e_q_max` | `vector<double>` | `[0.02, 0.02, 0.02, 0.02, 0.03, 0.03, 0.03]` | Per-arm position error limits in rad; 7 values. |
| `arm_i.alpha_filt` | `double` | `0.05` | First-order target filter coefficient for arm `i`, clamped to `[0, 1]`. |

Real configuration:
[dual_controllers.yaml](franka_bringup/config/real/dual_controllers.yaml).
C++ declares `arm_count=0`, `publish_rate=100.0`, `control_frequency=1000.0`,
and per-arm `alpha_filt=1.0` when not supplied. Per-arm IDs and 7-element
gain/error arrays must be configured. Joint commands use the same name matching
and optional-velocity rules as the single-arm controller. Publication intervals
are quantized as above; the dual-arm controller also publishes each update when
`publish_rate <= 0`. Its current update loop writes the calculated torque directly
and does not apply the single-arm controller's torque-step clipping.

Simulation overrides for both arms:

| Parameter | Dual-arm simulation | Single-arm simulation |
| :--- | :--- | :--- |
| Arm IDs | `mj_left`, `mj_right` | `panda` |
| `k_gains` | `[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 100.0]` | `[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]` |
| `d_gains` | `[30.0, 30.0, 30.0, 15.0, 12.0, 12.0, 9.0]` | `[30.0, 30.0, 30.0, 12.0, 12.0, 12.0, 6.0]` |
| `e_q_max` | `[0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4]` | `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]` |
| `alpha_filt` | `0.01` | `0.01` |
| `publish_rate` | `100.0` | `1000.0` (C++ default; absent from YAML) |
| `control_frequency` | `1000.0` | `1000.0` (C++ default; absent from YAML) |

Sources: [dual_sim_controllers.yaml](franka_bringup/config/sim/dual_sim_controllers.yaml)
and [single_sim_controllers.yaml](franka_bringup/config/sim/single_sim_controllers.yaml).
For single-arm simulation, launch `franka_sim.launch.py`, then spawn
`joint_impedance_controller` on `/controller_manager`. Changing simulation
launch arm IDs alone does not rewrite the IDs in these YAML files or MuJoCo models.

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

The preserved base simulation launch configures both `joint_state_publisher`
and `joint_state_broadcaster` on `/joint_states`. The dual-arm launch also lists
gripper sources with a duplicated `/joint_states/joint_states` suffix, while
the gripper servers publish a single `/joint_states` suffix. Inspect publisher
ownership with `ros2 topic info /joint_states -v` when diagnosing mixed or missing
joint-state updates; these are current launch limitations, not a ROS remapping
restriction.

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

| Camera | Compressed topic (when the transport is available) | Message Type |
| :--- | :--- | :--- |
| `<arm_id_1>_wrist` | `/cameras/<arm_id_1>_wrist/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `<arm_id_2>_wrist` | `/cameras/<arm_id_2>_wrist/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `fixed` | `/cameras/fixed/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |

Raw topics omit `/compressed` and use `sensor_msgs/msg/Image`. The launch passes
the base `.../color/image_raw` topic to `rqt_image_view` with
`image_transport:=compressed`.

The two D405 wrist cameras request color at `640x480 @ 30 Hz`; the fixed D435f
requests `848x480 @ 30 Hz`. All three set `enable_depth=false` and
`enable_color=true`; depth profile strings do not enable a depth stream.
The camera serial numbers are fixed in
[dual_franka_control_with_realsense.launch.py](franka_bringup/launch/real/dual_franka_control_with_realsense.launch.py)
and must match the connected devices.

`launch_realsense`, `launch_rqt_image_view`, and `apply_collision_params` all
default to `true`. The last flag schedules the collision-parameter service
call after three seconds; it does not describe the initial hardware thresholds.

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

**Launch Arguments (relative to `franka_control.launch.py`):**

| Argument | Default | Description |
| :--- | :--- | :--- |
| `arm_id` | `panda` | Arm namespace used for URDF, controller interfaces, gripper bridge, and wrist camera topic. |
| `launch_realsense` | `true` | Whether to launch RealSense camera nodes together with robot bringup. |
| `wrist_camera` | `left` | Wrist camera selector. Must be `left` or `right`. The launch file maps it to built-in D405 serial numbers, otherwise it raises an error. |
| `controller_name` | `joint_impedance_controller` | Overrides the Cartesian-controller default of the camera-free single-arm wrapper. |
| `launch_rqt_image_view` | `true` | Launch wrist and fixed camera viewer processes. |
| `apply_collision_params` | `true` | Schedule the collision-parameter service call after three seconds. |

**Camera Topics:**

| Camera | Compressed topic (when the transport is available) | Message Type |
| :--- | :--- | :--- |
| `<arm_id>_wrist` (from `wrist_camera=left/right`) | `/cameras/<arm_id>_wrist/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `fixed` | `/cameras/fixed/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |

The selected D405 wrist camera requests `640x480 @ 30 Hz`; the fixed D435f
requests `848x480 @ 30 Hz`. Both disable depth and enable color. Raw topic
names omit `/compressed` and carry `sensor_msgs/msg/Image`. Camera serial
numbers and profiles are fixed in
[franka_control_with_realsense.launch.py](franka_bringup/launch/real/franka_control_with_realsense.launch.py).


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

From the workspace root, after sourcing ROS and the workspace overlay and
starting the matching joint impedance controller:

```bash
# Single arm: panda, /panda/joints_desired
python3 src/multipanda_ros2/spacemouse_teleop/spacemouse_pub_singlearm_joint.py

# Dual arm: panda_left and panda_right, separate joints_desired topics
python3 src/multipanda_ros2/spacemouse_teleop/spacemouse_pub_dualarm_joint.py
```

See the scripts' configuration classes for device and motion settings.

## GELLO Teleoperation
[GELLO: General, Low-Cost, and Intuitive Teleoperation Framework](https://github.com/wuphilipp/gello_software.git)
### Installation:

Install the linked GELLO software and make its `gello` Python package available
to the ROS Python environment. The local adapter is
[gello_franka_ros2.py](gello_teleop/scripts/gello_franka_ros2.py).


### Examples:

The retained adapter defaults to `/dev/ttyUSB0`, a `0.02 s` timer, and both-arm
mode. It publishes a 14-position `std_msgs/msg/Float64MultiArray` on the legacy
`/dual_joint_impedance/joints_desired` topic. The current dual-arm impedance
controller instead subscribes to per-arm `sensor_msgs/msg/JointState` messages
on `/<arm_id>/joints_desired`. This adapter requires conversion of both the
message type and joint names before it can drive that controller; it is not a
direct launch example for the current interface.
