#!/usr/bin/env python3
"""
Keyboard teleoperation test script to simulate GELLO input.
Controls 7 joints and gripper with incremental movements.
Publishes to ROS2 topics for robot control.
"""

import argparse
import sys
import select
import tty
import termios
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rclpy.action import ActionClient
from franka_msgs.action import Move as MoveAction

from gello_smoother import Smoother

# Default joint angles (from gello_franka_singlearm.py)
DEFAULT_JOINTS = np.array([
    0.5 * np.pi,
    -0.785398,
    0.0,
    -2.35619,
    0.0,
    1.5708,
    0.785398,
], dtype=float)

JOINT_LIMITS = np.array([
    [-2.8973, 2.8973],
    [-1.7628, 1.7628],
    [-2.8973, 2.8973],
    [-3.0718, -0.0698],
    [-2.8973, 2.8973],
    [-0.0175, 3.7525],
    [-2.8973, 2.8973],
], dtype=float)

# Increment step size
JOINT_STEP = 0.05
GRIPPER_STEP = 0.05

# Key bindings
KEY_BINDINGS = {
    'q': (0, 1),   # Joint 1 +
    'a': (0, -1),  # Joint 1 -
    'w': (1, 1),   # Joint 2 +
    's': (1, -1),  # Joint 2 -
    'e': (2, 1),   # Joint 3 +
    'd': (2, -1),  # Joint 3 -
    'r': (3, 1),   # Joint 4 +
    'f': (3, -1),  # Joint 4 -
    't': (4, 1),   # Joint 5 +
    'g': (4, -1),  # Joint 5 -
    'y': (5, 1),   # Joint 6 +
    'h': (5, -1),  # Joint 6 -
    'u': (6, 1),   # Joint 7 +
    'j': (6, -1),  # Joint 7 -
    'o': (7, 1),   # Gripper open
    'l': (7, -1),  # Gripper close
}


class KeyboardTeleopNode(Node):
    def __init__(self, joint_state_topic='/joint_states'):
        super().__init__('keyboard_teleop_test')
        
        # Initialize joint angles
        self.current_joints = DEFAULT_JOINTS.copy()
        self.gripper_value = 0.0  # 0 = open, 1 = closed
        self.max_gripper_open = 0.09
        self.joint_state_topic = joint_state_topic
        self.actual_joints_received = False
        
        # Publishers
        self.joint_publisher = self.create_publisher(
            JointState, 
            '/joint_impedance/joints_desired', 
            10
        )
        
        # Action client for gripper (direct to sim controller)
        self.gripper_action_client = ActionClient(self, MoveAction, '/panda_gripper_sim_node/move')

        # Subscriber for real robot feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )
        
        # Smoother for joint command output (same params as gello_franka_singlearm.py)
        self.smoother = Smoother(
            publish_hz=50.0,
            max_velocity=0.15,
            position_tolerance=0.05,
            joint_limits=JOINT_LIMITS,
            logger=self.get_logger()
        )
        self.smoother.set_initial_position(self.current_joints.copy())
        self.smoother.set_target(self.current_joints)
        
        # Timer for publishing
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('Keyboard teleop test node started')
        self.get_logger().info('Gripper publisher topic: /panda_gripper/width_desired (via action bridge)')
        
    def timer_callback(self):
        if not self.actual_joints_received:
            return

        # Publish smoothed joint commands
        self.smoother.set_target(self.current_joints)
        smoothed_joints = self.smoother.update()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'panda_joint{i+1}' for i in range(7)]
        msg.position = smoothed_joints.tolist()
        msg.velocity = [0.0] * 7
        self.joint_publisher.publish(msg)
    
    def send_gripper_command(self, width):
        """Send gripper move command via action client."""
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server not available')
            return
        
        goal_msg = MoveAction.Goal()
        goal_msg.width = width
        goal_msg.speed = 0.1
        
        self.gripper_action_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Sending gripper command: width={width:.3f}m')
    
    def update_joint(self, joint_idx, delta):
        if joint_idx < 7:
            self.current_joints[joint_idx] += delta
            self.current_joints[joint_idx] = np.clip(
                self.current_joints[joint_idx],
                JOINT_LIMITS[joint_idx, 0],
                JOINT_LIMITS[joint_idx, 1]
            )
            self.smoother.set_target(self.current_joints)
    
    def update_gripper(self, delta):
        self.gripper_value += delta
        self.gripper_value = np.clip(self.gripper_value, 0.0, 1.0)
        # Calculate width: 0.09m = open, 0.0m = closed
        gripper_width = self.max_gripper_open * (1 - self.gripper_value)
        self.send_gripper_command(gripper_width)
    
    def reset(self):
        self.current_joints = DEFAULT_JOINTS.copy()
        self.gripper_value = 0.0
        self.smoother.set_target(self.current_joints)
        self.send_gripper_command(self.max_gripper_open)

    def joint_state_callback(self, msg):
        try:
            indices = [msg.name.index(f'panda_joint{i+1}') for i in range(7) if f'panda_joint{i+1}' in msg.name]
            if len(indices) == 7:
                actual_joints = np.array([msg.position[i] for i in indices], dtype=float)
                self.smoother.set_actual_position(actual_joints)

                if not self.actual_joints_received:
                    self.actual_joints_received = True
                    self.get_logger().info('Received first actual joint feedback, initializing smoother from real robot position.')
                    self.smoother.set_initial_position(actual_joints)
                    self.smoother.set_target(self.current_joints)
                    self.smoother.set_follower_enabled(True)
        except Exception:
            pass


def parse_args():
    parser = argparse.ArgumentParser(description='Keyboard teleoperation test with real robot feedback.')
    parser.add_argument('--joint-state-topic', default='/joint_states', help='ROS2 topic for real robot joint state feedback.')
    return parser.parse_args()


def get_key():
    """Read a single key from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_help():
    """Print help message with key bindings."""
    print("\n" + "="*70)
    print("Keyboard Teleoperation Test (GELLO Simulator)")
    print("="*70)
    print("Joint Controls (increment: {:.3f} rad):".format(JOINT_STEP))
    print("  q/a: Joint 1")
    print("  w/s: Joint 2")
    print("  e/d: Joint 3")
    print("  r/f: Joint 4")
    print("  t/g: Joint 5")
    print("  y/h: Joint 6")
    print("  u/j: Joint 7")
    print("\nGripper Controls (increment: {:.0f}%):".format(GRIPPER_STEP * 100))
    print("  o: Open gripper")
    print("  l: Close gripper")
    print("\nOther:")
    print("  x: Reset to default position")
    print("  ?: Show this help")
    print("  Ctrl+C: Exit")
    print("="*70 + "\n")


def print_joint_state(joints, gripper_value, mode: str = None):
    """Print current joint angles, gripper state, and smoother mode."""
    print("\r" + " " * 180 + "\r", end="")
    print("Joints: [", end="")
    for i, j in enumerate(joints):
        print("{:.4f}".format(j), end="" if i == 6 else ", ")
    mode_text = f"  Mode: {mode}" if mode is not None else ""
    print("]  Gripper: {:.0f}% closed (width: {:.3f}m){}".format(
        gripper_value * 100, 0.09 * (1 - gripper_value), mode_text), end="", flush=True)


def keyboard_thread(node):
    """Keyboard input handling thread."""
    print_help()
    print_joint_state(node.current_joints, node.gripper_value, node.smoother.current_mode())
    
    try:
        while rclpy.ok():
            key = get_key()
            
            if key is None:
                continue
            
            if key == '\x03':  # Ctrl+C
                print("\n\nExiting...")
                break
            
            if key == 'x':  # Reset
                node.reset()
                print("\nReset to default position")
            
            elif key == '?':  # Help
                print_help()
            
            elif key in KEY_BINDINGS:
                joint_idx, direction = KEY_BINDINGS[key]
                
                if joint_idx < 7:  # Joint control
                    delta = direction * JOINT_STEP
                    node.update_joint(joint_idx, delta)
                
                else:  # Gripper control
                    delta = direction * GRIPPER_STEP
                    node.update_gripper(delta)
            
            print_joint_state(node.current_joints, node.gripper_value, node.smoother.current_mode())
    
    except KeyboardInterrupt:
        print("\n\nExiting...")


def main(args=None):
    parsed_args = parse_args()
    rclpy.init(args=args)
    node = KeyboardTeleopNode(joint_state_topic=parsed_args.joint_state_topic)
    
    # Start keyboard thread
    kb_thread = threading.Thread(target=keyboard_thread, args=(node,))
    kb_thread.daemon = True
    kb_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
