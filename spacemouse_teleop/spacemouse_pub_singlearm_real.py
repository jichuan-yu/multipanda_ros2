#!/usr/bin/env python3

from dataclasses import dataclass, field
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import math
import time
import threading
from scipy.spatial.transform import Rotation
import pyspacemouse



@dataclass
class SpacemouseConfig:
    target_pose_topic: str = '/cartesian_impedance/pose_desired' # desired pose topic to publish to
    ee_pose_topic: str = '/cartesian_impedance/ee_pose' # robot end-effector pose feedback
    base_frame: str = 'panda_link0'
    publish_hz: float = 100.0
    '''
        Essentially the scales are the maximum translation/rotation velocity in m/s or rad/s
        delta_pos = scale_pos / publish_hz * mouse_state ([0,1]) in each control cycle.
    '''
    scale_pos: float = 0.1 #  0.1 m/s 
    scale_rot: float = 0.2 # 0.2 rad/s

    deadzone: float = 0.05 # 5% deadzone by default
    # Gripper control defaults (m, m, m/s, N)
    # Button 0 - open, Button 1 - close
    arm_id: str = 'panda'
    gripper_open_width: float = 0.08
    gripper_close_width: float = 0.0
    gripper_speed: float = 0.1
    gripper_force: float = 10.0
    # 6x6 mapping from normalized SpaceMouse input [tx, ty, tz, rx, ry, rz]
    # to robot command axes [dx, dy, dz, drx, dry, drz].
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




class SpaceMouseTeleopNode(Node):
    INIT_TIMEOUT_SEC = 1.0

    def __init__(self, config: SpacemouseConfig = None):
        super().__init__('spacemouse_teleop')

        self.config = config if config is not None else SpacemouseConfig()

        self.target_pose_topic = self.config.target_pose_topic
        self.ee_pose_topic = self.config.ee_pose_topic
        self.frame_id = self.config.base_frame
        self.publish_hz = float(self.config.publish_hz)
        self.scale_pos = float(self.config.scale_pos)
        self.scale_rot = float(self.config.scale_rot)
        self.deadzone = float(self.config.deadzone)
        self.motion_mapping = np.asarray(self.config.motion_mapping, dtype=float)

        if self.motion_mapping.shape != (6, 6):
            raise ValueError("SpacemouseConfig.motion_mapping must be a 6x6 matrix")

        self.publisher = self.create_publisher(
            PoseStamped,
            self.target_pose_topic,
            10
        )
        # Gripper publishers (use gripper action bridge topics)
        # read gripper defaults from config rather than ROS parameters
        self.arm_id = self.config.arm_id
        self.gripper_open_width = float(self.config.gripper_open_width)
        self.gripper_close_width = float(self.config.gripper_close_width)
        self.gripper_speed = float(self.config.gripper_speed)
        self.gripper_force = float(self.config.gripper_force)

        self.width_pub = self.create_publisher(
            Float64,
            f"/{self.arm_id}_gripper/width_desired",
            10,
        )
        self.grasp_pub = self.create_publisher(
            Float64MultiArray,
            f"/{self.arm_id}_gripper/grasp_desired",
            10,
        )

        self.pose_lock = threading.Lock()
        self.mouse_state_lock = threading.Lock()
        self.mouse_state = None
        self.pose = np.eye(4)
        self.pose_initialized_from_ee_pose = False
        self.prev_buttons = None  # For button edge detection
        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            self.ee_pose_topic,
            self.ee_pose_callback,
            10,
        )

        # Initialize target pose from current robot EE pose if available.
        init_start = time.monotonic()
        while (not self.pose_initialized_from_ee_pose and
             (time.monotonic() - init_start) < self.INIT_TIMEOUT_SEC):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.pose_initialized_from_ee_pose:
            raise RuntimeError(
                f"Timeout waiting for {self.ee_pose_topic} (>{self.INIT_TIMEOUT_SEC:.1f}s). "
                f"Cannot initialize self.pose from current end-effector pose."
            )
        
        success = pyspacemouse.open()
        if not success:
            self.get_logger().error("Could not connect to spacemouse. Ensure HID access!")
            exit(1)

        self._last_pose_log_time = 0.0

        self.get_logger().info('SpaceMouse publisher (single arm) initialized!')

    def _send_gripper_width(self, width: float):
        msg = Float64()
        msg.data = float(width)
        self.width_pub.publish(msg)

    def _send_gripper_grasp(self, width: float, speed: float = None, force: float = None, epsilon: float = 0.01):
        msg = Float64MultiArray()
        spd = self.gripper_speed if speed is None else float(speed)
        frc = self.gripper_force if force is None else float(force)
        # expected order: width, speed, force, epsilon
        msg.data = [float(width), float(spd), float(frc), float(epsilon)]
        self.grasp_pub.publish(msg)

    def ee_pose_callback(self, msg: PoseStamped):
        if self.pose_initialized_from_ee_pose:
            return
        rotation = Rotation.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]).as_matrix()
        with self.pose_lock:
            self.pose = np.eye(4)
            self.pose[0:3, 0:3] = rotation
            self.pose[0:3, 3] = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ], dtype=float)
        self.pose_initialized_from_ee_pose = True
        self.get_logger().info(f'Initialized target pose from {self.ee_pose_topic}.')

    def step(self, state):
        if not state:
            return

        # Normalized SpaceMouse axes in [-1, 1].
        raw_input = np.array(
            [
                state.x,
                state.y,
                state.z,
                state.roll,
                state.pitch,
                state.yaw,
            ],
            dtype=float,
        )

        # Apply deadzone per axis.
        raw_input[np.abs(raw_input) < self.deadzone] = 0.0

        # Map device axes to robot command axes.
        mapped_input = self.motion_mapping @ raw_input

        # Convert velocity limits (m/s, rad/s) to per-cycle increments.
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
            R_delta = (rot_z * rot_y * rot_x).as_matrix()
            
            with self.pose_lock:
                self.pose[0, 3] += dx
                self.pose[1, 3] += dy
                self.pose[2, 3] += dz
                self.pose[0:3, 0:3] = R_delta @ self.pose[0:3, 0:3]

        # If SpaceMouse state provides buttons, map button presses to gripper commands
        try:
            buttons = getattr(state, 'buttons', None)
            if buttons:
                # Detect button edge (False -> True) to send command only once per press
                if self.prev_buttons is None:
                    self.prev_buttons = [False] * len(buttons)
                
                # Button 0 -> open gripper
                if len(buttons) > 0 and buttons[0] and not self.prev_buttons[0]:
                    self._send_gripper_width(self.gripper_open_width)
                
                # Button 1 -> grasp/close gripper
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

    def get_pose_msg(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id

        with self.pose_lock:
            arm_T = self.pose.copy()
        pose_msg.pose.position.x = float(arm_T[0, 3])
        pose_msg.pose.position.y = float(arm_T[1, 3])
        pose_msg.pose.position.z = float(arm_T[2, 3])

        quat = Rotation.from_matrix(arm_T[0:3, 0:3]).as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        return pose_msg

    def timer_callback(self):
        state = self.consume_mouse_state()
        if state is not None:
            self.step(state)
        self.publisher.publish(self.get_pose_msg())
        now = time.monotonic()
        if now - self._last_pose_log_time >= 2.0:
            with self.pose_lock:
                pose_snapshot = self.pose.copy()
            self.get_logger().info(
                f'Current cartesian pose:\n{np.array2string(pose_snapshot, precision=4, suppress_small=True)}'
            )
            self._last_pose_log_time = now



def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseTeleopNode()

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
