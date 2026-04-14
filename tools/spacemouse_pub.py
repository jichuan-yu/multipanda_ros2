#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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
            '/multi_cartesian_impedance/pose_desired', 
            10
        )
        
        T_left = np.eye(4)
        T_left[0:3, 0:3] = euler_to_matrix(math.pi, 0.0, 0.0)
        T_left[0:3, 3] = [0.3, 0.2, 0.5]
        
        T_right = np.eye(4)
        T_right[0:3, 0:3] = euler_to_matrix(math.pi, 0.0, 0.0)
        T_right[0:3, 3] = [0.3, -0.2, 0.5]
        
        self.poses = {
            'left': T_left,
            'right': T_right
        }
        self.selected_arm = 'left'
        
        # Scale pos correctly (using pyspacemouse processed scaling, similar range -1 to 1) 
        self.scale_pos = 0.0000007
        self.scale_rot = 0.0000035
        self.deadzone = 0.05
        
        success = pyspacemouse.open()
        if not success:
            self.get_logger().error("Could not connect to spacemouse. Ensure HID access!")
            exit(1)

        self.get_logger().info('SpaceMouse publisher (via native pyspacemouse) initialized!')
        self.get_logger().info('Button 0 (Left) -> Select Left Arm')
        self.get_logger().info('Button 1 (Right) -> Select Right Arm')
        
        self.prev_left_pressed = False
        self.prev_right_pressed = False

    def process_state(self, state):
        if not state:
            return

        is_idle = (state.x == 0.0 and state.y == 0.0 and state.z == 0.0 and 
                   state.roll == 0.0 and state.pitch == 0.0 and state.yaw == 0.0 and 
                   not any(state.buttons))
        
        if not is_idle:
            print(f"[Pyspacemouse State] x: {state.x:.3f}, y: {state.y:.3f}, z: {state.z:.3f}, roll: {state.roll:.3f}, pitch: {state.pitch:.3f}, yaw: {state.yaw:.3f}, buttons: {state.buttons}")

        try:
            is_left_pressed = bool(state.buttons[0])
        except IndexError:
            is_left_pressed = False
            
        try:
            is_right_pressed = bool(state.buttons[1])
        except IndexError:
            is_right_pressed = False
        
        if is_left_pressed and not self.prev_left_pressed:
            self.selected_arm = 'left'
            self.get_logger().info('Switched to LEFT arm')
        elif is_right_pressed and not self.prev_right_pressed:
            self.selected_arm = 'right'
            self.get_logger().info('Switched to RIGHT arm')
            
        self.prev_left_pressed = is_left_pressed
        self.prev_right_pressed = is_right_pressed

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
            self.poses[self.selected_arm][0, 3] += dx
            self.poses[self.selected_arm][1, 3] += dy
            self.poses[self.selected_arm][2, 3] += dz
            
            # 基于基座坐标系（以当前末端位置为旋转中心）进行旋转
            self.poses[self.selected_arm][0:3, 0:3] = R_delta @ self.poses[self.selected_arm][0:3, 0:3]

    def get_pose_array(self, arm_name):
        arr = []
        arm_T = self.poses[arm_name]
        arr.extend(arm_T[0:3, 3].tolist())
        arr.extend(arm_T[0:3, 0:3].flatten().tolist())
        return arr

    def timer_callback(self):
        msg = Float64MultiArray()
        data = []
        data.extend(self.get_pose_array('left'))
        data.extend(self.get_pose_array('right'))
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
