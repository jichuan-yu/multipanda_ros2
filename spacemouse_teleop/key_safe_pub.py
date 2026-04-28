#!/usr/bin/env python3
"""
key_safe_pub.py
Keyboard teleop publisher for the DualArmMprcController.

Publishes a 24-element Float64MultiArray to /dualarm_mprc/pose_desired:
  [left_pos(3), left_rotmat(9), right_pos(3), right_rotmat(9)]

No oscbf / multi_cartesian_impedance logic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import sys

import select
import termios
import tty
import threading
import numpy as np
import math


# ── Rotation helper ──────────────────────────────────────────────────────────
def euler_to_matrix(rx, ry, rz):
    """Build rotation matrix Rz * Ry * Rx from Euler angles (rad)."""
    cz, sz = math.cos(rz), math.sin(rz)
    cy, sy = math.cos(ry), math.sin(ry)
    cx, sx = math.cos(rx), math.sin(rx)

    Rx = np.array([[1, 0,   0  ],
                   [0, cx, -sx ],
                   [0, sx,  cx ]])
    Ry = np.array([[ cy, 0, sy],
                   [  0, 1,  0],
                   [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class KeySafePubNode(Node):
    def __init__(self):
        super().__init__('key_safe_pub')
        self.set_parameters([rclpy.Parameter('use_sim_time',
                                             rclpy.Parameter.Type.BOOL, True)])

        # Publisher → DualArmMprcController
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/dualarm_mprc/pose_desired',
            10
        )

        # Initial poses (matching my_task_sim.launch.py startup configuration)
        self.poses = {
            'left': {
                'pos': np.array([0.307,  0.0, 0.487]),
                'rot': np.array([math.pi, 0.0, 0.0])  # rx, ry, rz
            },
            'right': {
                'pos': np.array([0.307,  0.0, 0.487]),
                'rot': np.array([math.pi, 0.0, 0.0])
            }
        }

        self.selected_arm = 'left'

        # Step sizes
        self.step_pos = 0.001    # m (total step per keypress)
        self.step_rot = 0.005    # rad (total step per keypress)

        # Incremental motion settings
        self.num_steps = 10      # number of incremental steps per keypress
        self.publish_interval = 2  # publish every N timer callbacks (50Hz / 2 = 25Hz motion)
        self.callback_count = 0

        # Queue for incremental motions: list of (arm_name, axis, value) tuples
        # axis: 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z'
        self.motion_queue = []
        self.current_motion_step = 0
        self.current_motion_target = None

        # Gripper publishers
        self.left_gripper_pub  = self.create_publisher(
            Float64, '/mj_left_gripper/width_desired', 10)
        self.right_gripper_pub = self.create_publisher(
            Float64, '/mj_right_gripper/width_desired', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info('key_safe_pub initialized → /dualarm_mprc/pose_desired')
        self.print_usage()

    # ── Console helpers ───────────────────────────────────────────────────────
    def print_usage(self):
        msg = """
----------------------------------------
Keyboard Teleop  →  DualArmMprcController
(Incremental Motion: 10 steps per keypress)
----------------------------------------
Z/X : Select Left/Right arm (Current: {})
W/S : Forward/Backward (X)
A/D : Left/Right (Y)
Q/E : Up/Down (Z)
J/L : Rotate around X (-/+)
I/K : Rotate around Y (-/+)
U/O : Rotate around Z (-/+)
C/V : Close/Open Gripper
Ctrl-C to quit
----------------------------------------
Note: Each keypress executes 10 incremental steps
for smooth, safe motion with small joint changes.
----------------------------------------
""".format(self.selected_arm.upper()).replace('\n', '\r\n')
        print(msg, flush=True)

    # ── Gripper ───────────────────────────────────────────────────────────────
    def move_gripper(self, arm_name, width):
        msg = Float64()
        msg.data = float(width)
        if arm_name == 'left':
            self.left_gripper_pub.publish(msg)
        else:
            self.right_gripper_pub.publish(msg)

    # ── Pose update from key ──────────────────────────────────────────────────
    def update_pose(self, key):
        if key == 'z':
            self.selected_arm = 'left'
            return
        if key == 'x':
            self.selected_arm = 'right'
            return

        arm_name = self.selected_arm

        # Calculate incremental step (1/10 of total step)
        inc_pos = self.step_pos / self.num_steps
        inc_rot = self.step_rot / self.num_steps

        # Position: add N incremental steps to queue
        if   key == 'w':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'pos_x', inc_pos))
        elif key == 's':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'pos_x', -inc_pos))
        elif key == 'a':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'pos_y', inc_pos))
        elif key == 'd':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'pos_y', -inc_pos))
        elif key == 'q':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'pos_z', inc_pos))
        elif key == 'e':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'pos_z', -inc_pos))
        # Rotation
        elif key == 'j':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'rot_x', -inc_rot))
        elif key == 'l':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'rot_x', inc_rot))
        elif key == 'i':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'rot_y', -inc_rot))
        elif key == 'k':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'rot_y', inc_rot))
        elif key == 'u':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'rot_z', -inc_rot))
        elif key == 'o':
            for _ in range(self.num_steps):
                self.motion_queue.append((arm_name, 'rot_z', inc_rot))
        # Gripper (immediate, not incremental)
        elif key == 'c': self.move_gripper(arm_name, 0.0)
        elif key == 'v': self.move_gripper(arm_name, 0.08)

    # ── Build 12-element arm payload: pos(3) + rotmat(9) ─────────────────────
    def get_pose_array(self, arm_name):
        arm = self.poses[arm_name]
        R = euler_to_matrix(*arm['rot'])
        arr = arm['pos'].tolist() + R.flatten().tolist()   # 3 + 9 = 12
        return arr

    # ── Timer: publish 24-element message ────────────────────────────────────
    def timer_callback(self):
        self.callback_count += 1

        # Process motion queue at reduced rate (25Hz instead of 50Hz)
        if self.callback_count % self.publish_interval == 0 and self.motion_queue:
            # Get next motion from queue
            arm_name, axis, value = self.motion_queue.pop(0)

            # Apply the incremental change
            arm = self.poses[arm_name]
            if axis == 'pos_x':
                arm['pos'][0] += value
            elif axis == 'pos_y':
                arm['pos'][1] += value
            elif axis == 'pos_z':
                arm['pos'][2] += value
            elif axis == 'rot_x':
                arm['rot'][0] += value
            elif axis == 'rot_y':
                arm['rot'][1] += value
            elif axis == 'rot_z':
                arm['rot'][2] += value

        # Publish current poses
        data = self.get_pose_array('left') + self.get_pose_array('right')

        # Safety: prevent x == 0.0 (guard against downstream C++ check `if data[0]`)
        if data[0] == 0.0:
            data[0] = 0.001

        msg = Float64MultiArray()
        msg.data = data
        self.publisher.publish(msg)


# ── Raw-key reader ────────────────────────────────────────────────────────────
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = KeySafePubNode()

    settings = termios.tcgetattr(sys.stdin)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':   # Ctrl-C
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
