#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import time
import threading
from scipy.spatial.transform import Rotation
import pyspacemouse








class SpaceMouseTeleopNode(Node):
    def __init__(self):
        super().__init__('spacemouse_teleop')

        self.publisher = self.create_publisher(
            PoseStamped,
            '/cartesian_impedance/target_pose', 
            10
        )

        self.pose = np.eye(4)
        
        self.pose_initialized_from_ee_pose = False

        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            '/cartesian_impedance/ee_pose',
            self.ee_pose_callback,
            10,
        )

        # Initialize target pose from current robot EE pose if available.
        init_timeout_sec = 1.0
        init_start = time.monotonic()
        while (not self.pose_initialized_from_ee_pose and
               (time.monotonic() - init_start) < init_timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.pose_initialized_from_ee_pose:
            raise RuntimeError(
                f"Timeout waiting for /cartesian_impedance/ee_pose (>{init_timeout_sec:.1f}s). "
                f"Cannot initialize self.pose from current end-effector pose."
            )
        
        # Scale pos correctly (using pyspacemouse processed scaling, similar range -1 to 1) 
        self.scale_pos = 0.0000007
        self.scale_rot = 0.0000035
        self.deadzone = 0.05
        
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
        self.get_logger().info('Initialized target pose from /cartesian_impedance/ee_pose.')

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
        pose_msg.header.frame_id = 'panda_link0'

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
    
    node.create_timer(0.01, node.timer_callback)
    
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
