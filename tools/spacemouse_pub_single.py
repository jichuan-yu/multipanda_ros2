#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState
import numpy as np
import math
import time
import threading

import pyspacemouse

def euler_to_matrix(rx, ry, rz):
    cz, sz = math.cos(rz), math.sin(rz)
    cy, sy = math.cos(ry), math.sin(ry)
    cx, sx = math.cos(rx), math.sin(rx)
    Rx = np.array([[1,  0,   0],
                   [0, cx, -sx],
                   [0, sx,  cx]])
    Ry = np.array([[cy,  0, sy],
                   [ 0,  1,  0],
                   [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx

class SpaceMouseTeleopNode(Node):
    def __init__(self):
        super().__init__('spacemouse_teleop')
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/cartesian_impedance/pose_desired', 
            10
        )

        self.pose = np.eye(4)
        
        self.pose_initialized_from_robot_state = False

        self.robot_state_sub = self.create_subscription(
            FrankaState,
            '/franka_robot_state_broadcaster/robot_state',
            self.robot_state_callback,
            10,
        )

        # Initialize target pose from current robot EE pose if available.
        init_timeout_sec = 1.0
        init_start = time.monotonic()
        while (not self.pose_initialized_from_robot_state and
               (time.monotonic() - init_start) < init_timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.pose_initialized_from_robot_state:
            raise RuntimeError(
                f"Timeout waiting for /franka_robot_state_broadcaster/robot_state "
                f"(>{init_timeout_sec:.1f}s). Cannot initialize self.pose from o_t_ee."
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

    def robot_state_callback(self, msg: FrankaState):
        if self.pose_initialized_from_robot_state:
            return
        if len(msg.o_t_ee) != 16:
            self.get_logger().warn('Received robot_state with invalid o_t_ee size, ignoring message.')
            return
        self.pose = np.array(msg.o_t_ee, dtype=float).reshape(4, 4)
        self.pose_initialized_from_robot_state = True
        self.get_logger().info('Initialized target pose from /franka_robot_state_broadcaster/robot_state o_t_ee.')

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
            R_delta = euler_to_matrix(drx, dry, drz)
            
            # 基于基座坐标系（全局坐标系）进行平移
            self.pose[0, 3] += dx
            self.pose[1, 3] += dy
            self.pose[2, 3] += dz
            
            # 基于基座坐标系（以当前末端位置为旋转中心）进行旋转
            self.pose[0:3, 0:3] = R_delta @ self.pose[0:3, 0:3]

    def get_pose_array(self):
        arr = []
        arm_T = self.pose
        arr.extend(arm_T[0:3, 3].tolist())
        arr.extend(arm_T[0:3, 0:3].flatten().tolist())
        return arr

    def timer_callback(self):
        msg = Float64MultiArray()
        data = self.get_pose_array()
        if data[0] == 0.0: data[0] = 0.001
        msg.data = data
        self.publisher.publish(msg)

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
