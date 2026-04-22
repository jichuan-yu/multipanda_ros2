#!/usr/bin/env python3

from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import time
import threading
from scipy.spatial.transform import Rotation
import pyspacemouse



@dataclass
class SpacemouseConfig:
    target_pose_topic: str = '/cartesian_impedance/target_pose' # target pose topic to publish to
    ee_pose_topic: str = '/cartesian_impedance/ee_pose' # robot end-effector pose feedback
    base_frame: str = 'panda_link0'
    publish_hz: float = 100.0
    scale_pos: float = 0.00001
    scale_rot: float = 0.00001
    deadzone: float = 0.05 # 5% deadzone by default
    motion_mapping: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
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

        if self.motion_mapping.shape != (4, 4):
            raise ValueError("SpacemouseConfig.motion_mapping must be a 4x4 matrix")

        self.publisher = self.create_publisher(
            PoseStamped,
            self.target_pose_topic,
            10
        )

        self.pose = np.eye(4)
        
        self.pose_initialized_from_ee_pose = False

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

        self.get_logger().info('SpaceMouse publisher (single arm) initialized!')

    def ee_pose_callback(self, msg: PoseStamped):
        if self.pose_initialized_from_ee_pose:
            return
        rotation = Rotation.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]).as_matrix()
        self.pose = np.eye(4)
        self.pose[0:3, 0:3] = rotation
        self.pose[0:3, 3] = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        self.pose_initialized_from_ee_pose = True
        self.get_logger().info(f'Initialized target pose from {self.ee_pose_topic}.')

    def process_state(self, state):
        if not state:
            return

        is_idle = (state.x == 0.0 and state.y == 0.0 and state.z == 0.0 and 
                   state.roll == 0.0 and state.pitch == 0.0 and state.yaw == 0.0 and 
                   not any(state.buttons))
        
        if not is_idle:
            print(f"[Pyspacemouse State] x: {state.x:.3f}, y: {state.y:.3f}, z: {state.z:.3f}, roll: {state.roll:.3f}, pitch: {state.pitch:.3f}, yaw: {state.yaw:.3f}, buttons: {state.buttons}")

        def apply_dz(val):
            return val if abs(val) > self.deadzone else 0.0

        # Mapping to robot cartesian offsets 
        dx = apply_dz(state.y) * self.scale_pos
        dy = apply_dz(state.x) * self.scale_pos
        dz = apply_dz(state.z) * self.scale_pos
        
        drx = apply_dz(state.pitch) * self.scale_rot
        dry = apply_dz(-state.roll) * self.scale_rot
        drz = apply_dz(-state.yaw) * self.scale_rot

        raw_motion = np.array([dx, dy, dz, 1.0], dtype=float)
        mapped_motion = self.motion_mapping @ raw_motion
        dx, dy, dz = mapped_motion[0], mapped_motion[1], mapped_motion[2]
        
        if any([dx, dy, dz, drx, dry, drz]):
            rot_x = Rotation.from_rotvec([drx, 0.0, 0.0])
            rot_y = Rotation.from_rotvec([0.0, dry, 0.0])
            rot_z = Rotation.from_rotvec([0.0, 0.0, drz])
            R_delta = (rot_z * rot_y * rot_x).as_matrix()
            
            # 基于基座坐标系（全局坐标系）进行平移
            self.pose[0, 3] += dx
            self.pose[1, 3] += dy
            self.pose[2, 3] += dz
            
            # 基于基座坐标系（以当前末端位置为旋转中心）进行旋转
            self.pose[0:3, 0:3] = R_delta @ self.pose[0:3, 0:3]

    def get_pose_msg(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id

        arm_T = self.pose
        pose_msg.pose.position.x = float(arm_T[0, 3])
        pose_msg.pose.position.y = float(arm_T[1, 3])
        pose_msg.pose.position.z = float(arm_T[2, 3])

        rotation = arm_T[0:3, 0:3]
        trace = float(np.trace(rotation))
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            pose_msg.pose.orientation.w = 0.25 * s
            pose_msg.pose.orientation.x = float((rotation[2, 1] - rotation[1, 2]) / s)
            pose_msg.pose.orientation.y = float((rotation[0, 2] - rotation[2, 0]) / s)
            pose_msg.pose.orientation.z = float((rotation[1, 0] - rotation[0, 1]) / s)
        else:
            if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
                s = math.sqrt(1.0 + float(rotation[0, 0]) - float(rotation[1, 1]) - float(rotation[2, 2])) * 2.0
                pose_msg.pose.orientation.w = float((rotation[2, 1] - rotation[1, 2]) / s)
                pose_msg.pose.orientation.x = 0.25 * s
                pose_msg.pose.orientation.y = float((rotation[0, 1] + rotation[1, 0]) / s)
                pose_msg.pose.orientation.z = float((rotation[0, 2] + rotation[2, 0]) / s)
            elif rotation[1, 1] > rotation[2, 2]:
                s = math.sqrt(1.0 + float(rotation[1, 1]) - float(rotation[0, 0]) - float(rotation[2, 2])) * 2.0
                pose_msg.pose.orientation.w = float((rotation[0, 2] - rotation[2, 0]) / s)
                pose_msg.pose.orientation.x = float((rotation[0, 1] + rotation[1, 0]) / s)
                pose_msg.pose.orientation.y = 0.25 * s
                pose_msg.pose.orientation.z = float((rotation[1, 2] + rotation[2, 1]) / s)
            else:
                s = math.sqrt(1.0 + float(rotation[2, 2]) - float(rotation[0, 0]) - float(rotation[1, 1])) * 2.0
                pose_msg.pose.orientation.w = float((rotation[1, 0] - rotation[0, 1]) / s)
                pose_msg.pose.orientation.x = float((rotation[0, 2] + rotation[2, 0]) / s)
                pose_msg.pose.orientation.y = float((rotation[1, 2] + rotation[2, 1]) / s)
                pose_msg.pose.orientation.z = 0.25 * s

        return pose_msg

    def timer_callback(self):
        self.publisher.publish(self.get_pose_msg())

def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseTeleopNode()

    node.create_timer(1.0 / max(node.publish_hz, 1.0), node.timer_callback)
    
    stop_event = threading.Event()
    def spacemouse_loop():
        while not stop_event.is_set():
            try:
                state = pyspacemouse.read()
                if state:
                    node.process_state(state)
                else:
                    time.sleep(0.001)
            except Exception:
                time.sleep(0.001)
            
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
