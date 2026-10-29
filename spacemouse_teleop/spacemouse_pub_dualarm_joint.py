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
class ArmTeleopConfig:
    """单个手臂的遥操作配置"""

    arm_id: str = 'panda_left'
    target_joint_topic: str = '/panda_left/joints_desired'
    gripper_width_topic: str = '/panda_left_gripper/width_desired'
    gripper_grasp_topic: str = '/panda_left_gripper/grasp_desired'
    spacemouse_path: str = ''
    ee_frame: str = 'panda_left_hand_tcp'
    base_frame: str = 'panda_left_link0'
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

@dataclass
class DualArmTeleopConfig:
    """双臂遥操作总配置"""

    joint_state_topic: str = '/dual_arm/joint_states'
    left: ArmTeleopConfig = field(
        default_factory=lambda: ArmTeleopConfig(
            arm_id='panda_left',
            target_joint_topic='/panda_left/joints_desired',
            spacemouse_path='/dev/hidraw5',
            ee_frame='panda_left_hand_tcp',
            base_frame='panda_left_link0',
        )
    )
    right: ArmTeleopConfig = field(
        default_factory=lambda: ArmTeleopConfig(
            arm_id='panda_right',
            target_joint_topic='/panda_right/joints_desired',
            gripper_width_topic='/panda_right_gripper/width_desired',
            gripper_grasp_topic='/panda_right_gripper/grasp_desired',
            spacemouse_path='/dev/hidraw6',
            ee_frame='panda_right_hand_tcp',
            base_frame='panda_right_link0',
        )
    )
    publish_hz: float = 100.0


class _ArmState:
    """单个手臂的内部状态"""

    INIT_TIMEOUT_SEC = 2.0

    def __init__(self, config: ArmTeleopConfig, node: Node):
        self.config = config
        self.node = node
        self.side = config.arm_id

        self.arm_id = config.arm_id
        self.ee_frame = config.ee_frame
        self.base_frame = config.base_frame
        self.target_joint_topic = config.target_joint_topic
        self.gripper_width_topic = config.gripper_width_topic
        self.gripper_grasp_topic = config.gripper_grasp_topic
        self.publish_hz = float(config.publish_hz)
        self.scale_pos = float(config.scale_pos)
        self.scale_rot = float(config.scale_rot)
        self.deadzone = float(config.deadzone)
        self.motion_mapping = np.asarray(config.motion_mapping, dtype=float)

        if self.motion_mapping.shape != (6, 6):
            raise ValueError(f"[{self.side}] motion_mapping must be a 6x6 matrix")

        self.arm_joint_names = [f'{self.arm_id}_joint{i + 1}' for i in range(7)]

        self.urdf_xml = _get_urdf_from_xacro(
            config.xacro_path,
            self.arm_id,
            hand=config.xacro_hand,
        )
        self.model = pin.buildModelFromXML(self.urdf_xml)
        self.data = self.model.createData()

        self.ee_frame_id = self.model.getFrameId(self.ee_frame)
        if self.ee_frame_id < 0:
            raise RuntimeError(f"[{self.side}] Frame {self.ee_frame} not found in Pinocchio model")

        self.arm_joint_q_indices = self._resolve_arm_joint_q_indices()
        self.ee_task = FrameTask(self.ee_frame, position_cost=1.0, orientation_cost=1.0)
        self.posture_task = PostureTask(cost=1e-3)
        self.configuration = None

        self.pose_cmd_lock = threading.Lock()
        self.pose_fb_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.mouse_state_lock = threading.Lock()

        self.pose_cmd = np.eye(4)
        self.pose_fb = None
        self.pose_cmd_initialized_from_fb = False
        self.mouse_state = None
        self.prev_buttons = None

        self.q_current = None
        self.q_full_current = None
        self.joint_states_received = False
        self._missing_joint_state_logged = False
        self._last_pose_log_time = 0.0

        # gripper publishers
        self.gripper_open_width = float(config.gripper_open_width)
        self.gripper_close_width = float(config.gripper_close_width)
        self.gripper_speed = float(config.gripper_speed)
        self.gripper_force = float(config.gripper_force)

        self.width_pub = node.create_publisher(
            Float64,
            self.gripper_width_topic,
            10,
        )
        self.grasp_pub = node.create_publisher(
            Float64MultiArray,
            self.gripper_grasp_topic,
            10,
        )

    def _resolve_arm_joint_q_indices(self):
        indices = []
        for joint_name in self.arm_joint_names:
            joint_id = self.model.getJointId(joint_name)
            if joint_id == 0:
                raise RuntimeError(f'[{self.side}] Joint {joint_name} not found in model')
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

    def update_mouse_state(self, state):
        with self.mouse_state_lock:
            self.mouse_state = state

    def consume_mouse_state(self):
        with self.mouse_state_lock:
            state = self.mouse_state
            self.mouse_state = None
        return state

    def parse_joint_state(self, msg: JointState):
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
                    self.node.get_logger().warn(
                        f'[{self.side}] JointState missing: {missing}. '
                        f'Received: {list(msg.name)}'
                    )
                    self._missing_joint_state_logged = True
                return False

            q_arm = np.array([msg.position[idx] for idx in indices], dtype=float)
            with self.state_lock:
                self.q_current = q_arm.copy()
                if not self.joint_states_received:
                    self.joint_states_received = True

            if not self.pose_cmd_initialized_from_fb:
                self._initialize_pose_from_current_state(q_arm)
            return True
        except Exception as exc:
            self.node.get_logger().warn(f'[{self.side}] Failed to parse joint_states: {exc}')
            return False

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
        self.node.get_logger().info(f'[{self.side}] Initialized target pose from FK of {self.ee_frame}.')

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
            [self.scale_pos * dt] * 3 + [self.scale_rot * dt] * 3,
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

        # gripper buttons
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

    def get_target_pose_matrix(self):
        with self.pose_cmd_lock:
            return self.pose_cmd.copy()

    def solve_ik(self, target_pose: np.ndarray, q_arm_feedback: np.ndarray):
        if self.configuration is None:
            q_full = self._build_full_configuration(q_arm_feedback)
            self.configuration = pink.Configuration(self.model, self.data, q_full)

        self.ee_task.set_target(pin.SE3(target_pose[0:3, 0:3], target_pose[0:3, 3]))
        posture_target = self.configuration.q.copy()
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

    def maybe_log(self, q_desired: np.ndarray, q_feedback: np.ndarray, target_pose: np.ndarray):
        now = time.monotonic()
        if now - self._last_pose_log_time < 2.0:
            return

        q_full_feedback = self._build_full_configuration(q_feedback)
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
        joint_err = q_desired - q_feedback
        joint_err_norm = float(np.linalg.norm(joint_err))

        self.node.get_logger().info(
            f'[{self.side}] pos_err={pos_err_norm:.4f}m '
            f'ori_err={ori_err_norm:.4f}rad '
            f'joint_err={joint_err_norm:.4f}rad'
        )
        self._last_pose_log_time = now


class DualArmSpaceMouseTeleopNode(Node):
    INIT_TIMEOUT_SEC = 2.0

    def __init__(self, config: DualArmTeleopConfig = None):
        super().__init__('dual_arm_spacemouse_teleop')

        self.config = config if config is not None else DualArmTeleopConfig()
        self.publish_hz = float(self.config.publish_hz)

        self.left = _ArmState(self.config.left, self)
        self.right = _ArmState(self.config.right, self)

        self.left_joint_pub = self.create_publisher(
            JointState, self.config.left.target_joint_topic, 10
        )
        self.right_joint_pub = self.create_publisher(
            JointState, self.config.right.target_joint_topic, 10
        )
        self.get_logger().info(
            f'Publishing left joint commands to {self.config.left.target_joint_topic}'
        )
        self.get_logger().info(
            f'Publishing right joint commands to {self.config.right.target_joint_topic}'
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        # Wait for joint states before opening spacemouse
        init_start = time.monotonic()
        while (
            not (self.left.joint_states_received and self.right.joint_states_received)
            and (time.monotonic() - init_start) < self.INIT_TIMEOUT_SEC
        ):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.left.joint_states_received:
            raise RuntimeError(
                f'Left arm timeout waiting for {self.config.joint_state_topic}'
            )
        if not self.right.joint_states_received:
            raise RuntimeError(
                f'Right arm timeout waiting for {self.config.joint_state_topic}'
            )

        # Open spacemouse devices
        # If spacemouse_path is set, use it for stable device binding;
        # otherwise fall back to DeviceNumber enumeration.
        left_path = self.config.left.spacemouse_path
        if left_path:
            self.left_device = pyspacemouse.open(path=left_path)

        if not self.left_device:
            raise RuntimeError(
                f'Failed to open left spacemouse (path={left_path!r})'
            )
        self.get_logger().info(
            f'Left spacemouse opened (path={left_path!r})'
        )

        right_path = self.config.right.spacemouse_path
        if right_path:
            self.right_device = pyspacemouse.open(path=right_path)

        if not self.right_device:
            raise RuntimeError(
                f'Failed to open right spacemouse (path={right_path!r})'
            )
        self.get_logger().info(
            f'Right spacemouse opened (path={right_path!r})'
        )

        self._stop_event = threading.Event()

        self.create_timer(1.0 / max(self.publish_hz, 1.0), self.timer_callback)

        self.get_logger().info('Dual-arm SpaceMouse teleop initialized!')

    def joint_state_callback(self, msg: JointState):
        self.left.parse_joint_state(msg)
        self.right.parse_joint_state(msg)

    def _publish_arm_joint_state(self, arm_state: _ArmState, publisher, q_arm: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = arm_state.arm_joint_names
        msg.position = q_arm.tolist()
        msg.velocity = [0.0] * arm_state.arm_joint_names.__len__()
        publisher.publish(msg)

    def timer_callback(self):
        # Consume spacemouse states
        lstate = self.left.consume_mouse_state()
        rstate = self.right.consume_mouse_state()

        if lstate is not None:
            self.left.step(lstate)
        if rstate is not None:
            self.right.step(rstate)

        if not (self.left.joint_states_received and self.right.joint_states_received):
            return

        with self.left.state_lock:
            l_q_fb = None if self.left.q_current is None else self.left.q_current.copy()
        with self.right.state_lock:
            r_q_fb = None if self.right.q_current is None else self.right.q_current.copy()

        if l_q_fb is None or r_q_fb is None:
            return

        l_target = self.left.get_target_pose_matrix()
        r_target = self.right.get_target_pose_matrix()

        l_q_des = self.left.solve_ik(l_target, l_q_fb)
        r_q_des = self.right.solve_ik(r_target, r_q_fb)

        self._publish_arm_joint_state(self.left, self.left_joint_pub, l_q_des)
        self._publish_arm_joint_state(self.right, self.right_joint_pub, r_q_des)

        self.left.maybe_log(l_q_des, l_q_fb, l_target)
        self.right.maybe_log(r_q_des, r_q_fb, r_target)

    def _spacemouse_loop(self, device, arm_state: _ArmState, side: str):
        while not self._stop_event.is_set():
            try:
                state = device.read()
                if state:
                    arm_state.update_mouse_state(state)
            except Exception as exc:
                self.get_logger().warning(f'{side} spacemouse read error: {exc}')

    def run(self):
        l_thread = threading.Thread(
            target=self._spacemouse_loop,
            args=(self.left_device, self.left, 'Left'),
            daemon=True,
        )
        r_thread = threading.Thread(
            target=self._spacemouse_loop,
            args=(self.right_device, self.right, 'Right'),
            daemon=True,
        )
        l_thread.start()
        r_thread.start()

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop_event.set()
            l_thread.join(timeout=1.0)
            r_thread.join(timeout=1.0)
            self.left_device.close()
            self.right_device.close()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Example: customize config here if needed
    config = DualArmTeleopConfig()

    node = DualArmSpaceMouseTeleopNode(config)
    node.run()


if __name__ == '__main__':
    main()
