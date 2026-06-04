#!/usr/bin/env python3

from dataclasses import dataclass, field
from pathlib import Path
import os
import subprocess
import threading
import time

import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask
import pyspacemouse
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64, Float64MultiArray


def _resolve_workspace_root() -> Path:
    env_root = os.environ.get("DUAL_PANDA_WS") or os.environ.get("WORKSPACE_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()

    this_file = Path(__file__).resolve()
    # spacemouse_teleop -> multipanda_ros2 -> src -> dual_panda_ws
    return this_file.parents[3]


def _get_urdf_from_xacro(xacro_path: str, arm_id: str, hand: bool = True) -> str:
    """Generate URDF from the real Panda xacro used by the workspace."""
    workspace_root = _resolve_workspace_root()
    xacro_path = (workspace_root / xacro_path).resolve()

    full_command = (
        f"source {workspace_root}/install/setup.bash && "
        f"xacro {xacro_path} arm_id:={arm_id} hand:={'true' if hand else 'false'}"
    )

    result = subprocess.run(
        full_command,
        capture_output=True,
        text=True,
        shell=True,
        executable="/bin/bash",
        check=True,
    )
    return result.stdout


@dataclass
class SpacemouseConfig:
    arm_id: str = 'panda'
    joint_state_topic: str = '/panda/joint_states'
    target_joint_topic: str = '/panda/joints_desired'
    gripper_width_topic: str = '/panda_gripper/width_desired'
    gripper_grasp_topic: str = '/panda_gripper/grasp_desired'
    ee_frame: str = 'panda_hand_tcp'
    base_frame: str = 'panda_link0'
    xacro_path: str = 'src/multipanda_ros2/franka_description/robots/real/panda_arm.urdf.xacro'
    xacro_hand: bool = True
    publish_hz: float = 100.0
    scale_pos: float = 0.1
    scale_rot: float = 0.2
    deadzone: float = 0.05
    gripper_open_width: float = 0.08
    gripper_close_width: float = 0.03
    gripper_speed: float = 0.1
    gripper_force: float = 20.0
    motion_mapping: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, -1],
            ],
            dtype=float,
        )
    )


class SpaceMouseJointTeleopNode(Node):
    INIT_TIMEOUT_SEC = 2.0

    def __init__(self, config: SpacemouseConfig = None):
        super().__init__('spacemouse_joint_teleop')

        self.config = config if config is not None else SpacemouseConfig()

        self.arm_id = self.config.arm_id
        self.joint_state_topic = self.config.joint_state_topic
        self.target_joint_topic = self.config.target_joint_topic
        self.gripper_width_topic = self.config.gripper_width_topic
        self.gripper_grasp_topic = self.config.gripper_grasp_topic
        self.ee_frame = self.config.ee_frame
        self.frame_id = self.config.base_frame
        self.publish_hz = float(self.config.publish_hz)
        self.scale_pos = float(self.config.scale_pos)
        self.scale_rot = float(self.config.scale_rot)
        self.deadzone = float(self.config.deadzone)
        self.motion_mapping = np.asarray(self.config.motion_mapping, dtype=float)

        if self.motion_mapping.shape != (6, 6):
            raise ValueError('SpacemouseConfig.motion_mapping must be a 6x6 matrix')

        self.publisher = self.create_publisher(JointState, self.target_joint_topic, 10)
        self.width_pub = self.create_publisher(
            Float64,
            self.gripper_width_topic,
            10,
        )
        self.grasp_pub = self.create_publisher(
            Float64MultiArray,
            self.gripper_grasp_topic,
            10,
        )

        self.pose_cmd_lock = threading.Lock()
        self.pose_fb_lock = threading.Lock()
        self.mouse_state_lock = threading.Lock()
        self.state_lock = threading.Lock()

        self.pose_cmd = np.eye(4)
        self.configuration = None
        self.pose_fb = None
        self.pose_cmd_initialized_from_fb = False
        self.mouse_state = None
        self.prev_buttons = None

        self.gripper_open_width = float(self.config.gripper_open_width)
        self.gripper_close_width = float(self.config.gripper_close_width)
        self.gripper_speed = float(self.config.gripper_speed)
        self.gripper_force = float(self.config.gripper_force)

        self.arm_joint_names = [f'{self.arm_id}_joint{i+1}' for i in range(7)]
        self.q_current = None
        self.q_full_current = None
        self.joint_states_received = False
        self._missing_joint_state_logged = False
        self._last_pose_log_time = 0.0

        self.urdf_xml = _get_urdf_from_xacro(
            self.config.xacro_path,
            self.arm_id,
            hand=self.config.xacro_hand,
        )
        self.model = pin.buildModelFromXML(self.urdf_xml)
        self.data = self.model.createData()

        self.ee_frame_id = self.model.getFrameId(self.ee_frame)
        if self.ee_frame_id < 0:
            raise RuntimeError(f'Frame {self.ee_frame} not found in Pinocchio model')

        self.arm_joint_q_indices = self._resolve_arm_joint_q_indices()
        self.ee_task = FrameTask(self.ee_frame, position_cost=1.0, orientation_cost=1.0)
        self.posture_task = PostureTask(cost=1e-3)
        self.posture_q_arm = np.array(
            [
                0.0,
                -0.785398,
                0.0,
                -2.35619,
                0.0,
                1.5708,
                0.785398,
            ],
            dtype=float,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        init_start = time.monotonic()
        while (not self.joint_states_received and (time.monotonic() - init_start) < self.INIT_TIMEOUT_SEC):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.joint_states_received:
            raise RuntimeError(
                f'Timeout waiting for {self.joint_state_topic} (>{self.INIT_TIMEOUT_SEC:.1f}s). '
                'Cannot initialize joint-space command from current robot state.'
            )

        success = pyspacemouse.open()
        if not success:
            self.get_logger().error('Could not connect to spacemouse. Ensure HID access!')
            raise RuntimeError('Failed to open SpaceMouse device')

        self.get_logger().info('SpaceMouse joint teleop (single arm) initialized!')

    def _resolve_arm_joint_q_indices(self):
        indices = []
        for joint_name in self.arm_joint_names:
            joint_id = self.model.getJointId(joint_name)
            if joint_id == 0:
                raise RuntimeError(f'Joint {joint_name} not found in Pinocchio model')
            indices.append(self.model.idx_qs[joint_id])
        return indices

    def _build_full_configuration(self, q_arm: np.ndarray) -> np.ndarray:
        q_full = pin.neutral(self.model)
        for i, q_index in enumerate(self.arm_joint_q_indices):
            q_full[q_index] = float(q_arm[i])
        return q_full

    def _extract_arm_from_full(self, q_full: np.ndarray) -> np.ndarray:
        return np.array([float(q_full[idx]) for idx in self.arm_joint_q_indices], dtype=float)

    def _send_gripper_width(self, width: float):
        msg = Float64()
        msg.data = float(width)
        self.width_pub.publish(msg)

    def _send_gripper_grasp(self, width: float, speed: float = None, force: float = None, epsilon: float = 0.01):
        msg = Float64MultiArray()
        spd = self.gripper_speed if speed is None else float(speed)
        frc = self.gripper_force if force is None else float(force)
        msg.data = [float(width), float(spd), float(frc), float(epsilon)]
        self.grasp_pub.publish(msg)

    def joint_state_callback(self, msg: JointState):
        try:
            indices = []
            for name in self.arm_joint_names:
                if name in msg.name:
                    indices.append(msg.name.index(name))
                else:
                    indices.append(None)

            if not all(idx is not None for idx in indices):
                if not self._missing_joint_state_logged:
                    missing = [self.arm_joint_names[i] for i, idx in enumerate(indices) if idx is None]
                    self.get_logger().warn(
                        'JointState names do not fully match expected Panda joints. '
                        f'Missing: {missing}. Received names: {list(msg.name)}'
                    )
                    self._missing_joint_state_logged = True
                return

            q_arm = np.array([msg.position[idx] for idx in indices], dtype=float)
            with self.state_lock:
                self.q_current = q_arm.copy()
                if not self.joint_states_received:
                    self.joint_states_received = True
            
            if not self.pose_cmd_initialized_from_fb:
                self._initialize_pose_from_current_state(q_arm)
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse joint_states: {exc}')

    def _initialize_pose_from_current_state(self, q_arm: np.ndarray):
        q_full = self._build_full_configuration(q_arm)
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        frame_pose = self.data.oMf[self.ee_frame_id]
        pose_cmd = np.eye(4)
        pose_cmd[0:3, 0:3] = frame_pose.rotation
        pose_cmd[0:3, 3] = frame_pose.translation
        with self.pose_cmd_lock:
            self.pose_cmd = pose_cmd
        with self.pose_fb_lock:
            self.pose_fb = pose_cmd.copy()
        self.pose_cmd_initialized_from_fb = True
        self.get_logger().info(f'Initialized target pose from current FK of {self.ee_frame}.')

    def step(self, state):
        if not state:
            return

        raw_input = np.array(
            [state.x, state.y, state.z, state.roll, state.pitch, state.yaw],
            dtype=float,
        )
        raw_input[np.abs(raw_input) < self.deadzone] = 0.0

        mapped_input = self.motion_mapping @ raw_input

        dt = 1.0 / max(self.publish_hz, 1.0)
        gain = np.array(
            [
                self.scale_pos * dt,
                self.scale_pos * dt,
                self.scale_pos * dt,
                self.scale_rot * dt,
                self.scale_rot * dt,
                self.scale_rot * dt,
            ],
            dtype=float,
        )
        delta = mapped_input * gain
        dx, dy, dz, drx, dry, drz = delta.tolist()

        if any([dx, dy, dz, drx, dry, drz]):
            rot_x = Rotation.from_rotvec([drx, 0.0, 0.0])
            rot_y = Rotation.from_rotvec([0.0, dry, 0.0])
            rot_z = Rotation.from_rotvec([0.0, 0.0, drz])
            r_delta = (rot_z * rot_y * rot_x).as_matrix()

            with self.pose_cmd_lock:
                self.pose_cmd[0, 3] += dx
                self.pose_cmd[1, 3] += dy
                self.pose_cmd[2, 3] += dz
                self.pose_cmd[0:3, 0:3] = r_delta @ self.pose_cmd[0:3, 0:3]

        try:
            buttons = getattr(state, 'buttons', None)
            if buttons:
                if self.prev_buttons is None:
                    self.prev_buttons = [False] * len(buttons)

                if len(buttons) > 0 and buttons[0] and not self.prev_buttons[0]:
                    self._send_gripper_width(self.gripper_open_width)

                if len(buttons) > 1 and buttons[1] and not self.prev_buttons[1]:
                    self._send_gripper_grasp(self.gripper_close_width)

                self.prev_buttons = list(buttons)
        except Exception:
            pass

    def update_mouse_state(self, state):
        with self.mouse_state_lock:
            self.mouse_state = state

    def consume_mouse_state(self):
        with self.mouse_state_lock:
            state = self.mouse_state
            self.mouse_state = None
        return state

    def get_target_pose_matrix(self):
        with self.pose_cmd_lock:
            return self.pose_cmd.copy()

    def _solve_ik(self, target_pose: np.ndarray, q_arm_feedback: np.ndarray):
        if self.configuration is None:
            q_full = self._build_full_configuration(q_arm_feedback)
            self.configuration = pink.Configuration(self.model, self.data, q_full)
        
        self.ee_task.set_target(pin.SE3(target_pose[0:3, 0:3], target_pose[0:3, 3]))
        posture_target = self.configuration.q.copy()
        # Set posture task targets for the arm joints to maintain current posture
        # for i, q_index in enumerate(self.arm_joint_q_indices):
        #     posture_target[q_index] = self.posture_q_arm[i]
        self.posture_task.set_target(posture_target)

        velocity = pink.solve_ik(
            self.configuration,
            [self.ee_task, self.posture_task],
            dt=1.0 / self.publish_hz,
            solver='quadprog',
        )
        self.configuration.integrate_inplace(velocity, 1.0 / max(self.publish_hz, 1.0))
        q_desired = self._extract_arm_from_full(self.configuration.q)
               
        return q_desired

    def _publish_joint_command(self, q_desired: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.arm_joint_names
        msg.position = q_desired.tolist()
        msg.velocity = [0.0] * 7
        self.publisher.publish(msg)

    def timer_callback(self):
        state = self.consume_mouse_state()
        if state is not None:
            self.step(state)

        if not self.joint_states_received:
            return

        with self.state_lock:
            q_arm_feedback = None if self.q_current is None else self.q_current.copy()

        if q_arm_feedback is None:
            return

        target_pose = self.get_target_pose_matrix()
        q_desired = self._solve_ik(target_pose, q_arm_feedback)
        self._publish_joint_command(q_desired)

        joint_error = q_desired - q_arm_feedback
        joint_error_norm = float(np.linalg.norm(joint_error))

        now = time.monotonic()
        if now - self._last_pose_log_time >= 2.0:
            q_full_feedback = self._build_full_configuration(q_arm_feedback)
            pin.forwardKinematics(self.model, self.data, q_full_feedback)
            pin.updateFramePlacements(self.model, self.data)
            measured_frame = self.data.oMf[self.ee_frame_id]
            measured_pose = np.eye(4)
            measured_pose[0:3, 0:3] = measured_frame.rotation
            measured_pose[0:3, 3] = measured_frame.translation

            pos_err = target_pose[0:3, 3] - measured_pose[0:3, 3]
            pos_err_norm = float(np.linalg.norm(pos_err))
            rot_err = target_pose[0:3, 0:3] @ measured_pose[0:3, 0:3].T
            rotvec = Rotation.from_matrix(rot_err).as_rotvec()
            ori_err_norm = float(np.linalg.norm(rotvec))

            self.get_logger().info(
                f'Current target pose:\n{np.array2string(target_pose, precision=4, suppress_small=True)}\n'
                f'Current FK pose:\n{np.array2string(measured_pose, precision=4, suppress_small=True)}\n'
                f'Joint tracking error: {joint_error.tolist()} (norm={joint_error_norm:.6f} rad)\n'
                f'Position error: norm={pos_err_norm:.6f} m '
                f'Orientation error: norm={ori_err_norm:.6f} rad'
            )
            self._last_pose_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseJointTeleopNode()

    node.create_timer(1.0 / max(node.publish_hz, 1.0), node.timer_callback)

    stop_event = threading.Event()

    def spacemouse_loop():
        while not stop_event.is_set():
            try:
                state = pyspacemouse.read()
                if not state:
                    node.get_logger().warning('Empty state received from SpaceMouse')
                else:
                    node.update_mouse_state(state)
            except Exception as exc:
                node.get_logger().warning(f'SpaceMouse read loop exception: {exc}')

    sm_thread = threading.Thread(target=spacemouse_loop, daemon=True)
    sm_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        sm_thread.join(timeout=1.0)
        pyspacemouse.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()