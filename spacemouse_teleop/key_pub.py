#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty
import threading
import numpy as np
import math

# Using scipy for rotation calculations if available, or just implement basic rotations
def euler_to_matrix(rx, ry, rz):
    # Rz * Ry * Rx
    cz = math.cos(rz)
    sz = math.sin(rz)
    cy = math.cos(ry)
    sy = math.sin(ry)
    cx = math.cos(rx)
    sx = math.sin(rx)
    
    Rx = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx, cx]])
    Ry = np.array([[cy, 0, sy],
                   [0, 1, 0],
                   [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0],
                   [sz, cz, 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

class KeyTeleopNode(Node):
    def __init__(self):
        super().__init__('key_teleop')
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/multi_cartesian_impedance/pose_desired', 
            10
        )
        
        # Initial positions and rotations (Euler angles for simplicity)
        # Assuming initial standard pose for panda 
        # (O_T_EE is roughly: x=0.3, y=0.0, z=0.5, and orientation is rotated 180 around X)
        self.poses = {
            'left': {
                'pos': np.array([0.3, 0.2, 0.5]),
                'rot': np.array([math.pi, 0.0, 0.0]) # rx, ry, rz
            },
            'right': {
                'pos': np.array([0.3, -0.2, 0.5]),
                'rot': np.array([math.pi, 0.0, 0.0])
            }
        }
        
        self.selected_arm = 'left'
        
        # Delta steps
        self.step_pos = 0.01
        self.step_rot = 0.05
        
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        self.get_logger().info('Key publisher initialized.')
        self.print_usage()

    def print_usage(self):
        print("""
----------------------------------------
Keyboard Teleop for Dual Cartesian Arm
----------------------------------------
Z/X : Select Left/Right arm (Current: {})
W/S : Forward/Backward (X)
A/D : Left/Right (Y)
Q/E : Up/Down (Z)
J/L : Rotate around X (-/+)
I/K : Rotate around Y (-/+)
U/O : Rotate around Z (-/+)
Ctrl-C to quit
----------------------------------------
""".format(self.selected_arm.upper()))

    def update_pose(self, key):
        redraw = False
        if key == 'z':
            self.selected_arm = 'left'
            redraw = True
        elif key == 'x':
            self.selected_arm = 'right'
            redraw = True
            
        arm = self.poses[self.selected_arm]
        
        # Position
        if key == 'w':
            arm['pos'][0] += self.step_pos
        elif key == 's':
            arm['pos'][0] -= self.step_pos
        elif key == 'a':
            arm['pos'][1] += self.step_pos
        elif key == 'd':
            arm['pos'][1] -= self.step_pos
        elif key == 'q':
            arm['pos'][2] += self.step_pos
        elif key == 'e':
            arm['pos'][2] -= self.step_pos
            
        # Rotation
        elif key == 'j':
            arm['rot'][0] -= self.step_rot
        elif key == 'l':
            arm['rot'][0] += self.step_rot
        elif key == 'i':
            arm['rot'][1] -= self.step_rot
        elif key == 'k':
            arm['rot'][1] += self.step_rot
        elif key == 'u':
            arm['rot'][2] -= self.step_rot
        elif key == 'o':
            arm['rot'][2] += self.step_rot
            
        if redraw:
            self.print_usage()

    def get_pose_array(self, arm_name):
        arr = []
        arm = self.poses[arm_name]
        arr.extend(arm['pos'].tolist())
        
        R = euler_to_matrix(*arm['rot'])
        # Add 9 elements for rotation matrix (row-major)
        arr.extend(R.flatten().tolist())
        return arr

    def timer_callback(self):
        msg = Float64MultiArray()
        # Left first, then right. Total 24 elements
        msg.data = [1.0] # 1.0 at index 0 to ensure it's evaluated as float truthy initially for the cpp code wait, the c++ code checks `if (msg.data[0])` for left arm, wait!
        # The C++ code says:
        # if (msg.data[0]) {
        #   for (int i=0; i<3; ++i) arm.desired_position[i] = msg.data[offset+i]
        # So msg.data[0] is actally the X position of the first arm!! 
        # If x=0, it will ignore. We should be careful. In our case X is around 0.3, so it's > 0.
        
        data = []
        data.extend(self.get_pose_array('left'))
        data.extend(self.get_pose_array('right'))
        
        # Make sure data[0] is never exactly 0.0 to prevent the bug
        if data[0] == 0.0:
            data[0] = 0.001
            
        msg.data = data
        self.publisher.publish(msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyTeleopNode()
    
    settings = termios.tcgetattr(sys.stdin)
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03': # ctrl-c
                break
            if key:
                node.update_pose(key.lower())
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
