#!/usr/bin/env python3
"""
Keyboard teleoperation for dual Panda robot in Cartesian/task space.

Key Mapping:
Position:
  W/S: X axis (forward/backward)
  A/D: Y axis (left/right)
  Q/E: Z axis (up/down)
Rotation (small angle approx):
  J/L: Roll (X rotation)
  I/K: Pitch (Y rotation)
  U/O: Yaw (Z rotation)
Arm Selection:
  Z: Left arm
  X: Right arm
  B: Both arms
Gripper Control:
  N: Open gripper (0.08m)
  M: Close gripper (0.0m)
Step Size:
  [: Decrease step
  ]: Increase step

Publishes:
  - Float64MultiArray to /dualarm_teleop_cmd (arm control)
  - Float64 to /mj_{left,right}_gripper/width_desired (gripper control)

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/key_teleop_cartesian.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
import threading
import numpy as np
import argparse
import time
import os

# Add parent directory to path for imports
if __name__ == '__main__':
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Import base teleop node
try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    # Fallback for direct execution
    from base_teleop import BaseTeleopNode


class KeyCartesianTeleop(BaseTeleopNode):
    """Keyboard teleoperation for Cartesian space control."""

    # Command type constants
    TASK_SPACE_INCREMENT = 2

    def __init__(self, step_position: float = 0.001, step_rotation: float = 0.01):
        """Initialize key Cartesian teleop node."""
        super().__init__('key_cartesian_teleop', publish_rate=100.0)

        self.step_position = step_position
        self.step_rotation = step_rotation
        self.joint_states_received = False

        # Gripper control parameters
        self.gripper_max_width = 0.08  # Franka gripper max opening (m)
        self.gripper_min_width = 0.0   # Fully closed

        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Gripper control publishers (for gripper_action_bridge)
        self.left_gripper_pub = self.create_publisher(
            Float64,
            '/mj_left_gripper/width_desired',
            10
        )
        self.right_gripper_pub = self.create_publisher(
            Float64,
            '/mj_right_gripper/width_desired',
            10
        )

        self.print_usage()

    def print_usage(self):
        msg = """
========================================
Task Space Teleop for Dual Panda
========================================
Position Control:
  W/S: X axis (forward/back)
  A/D: Y axis (left/right)
  Q/E: Z axis (up/down)

Rotation Control:
  J/L: Roll (X rotation)
  I/K: Pitch (Y rotation)
  U/O: Yaw (Z rotation)

Arm Selection:
  Z : Left arm
  X : Right arm
  B : Both arms

Gripper Control:
  N : Open gripper (0.08m)
  M : Close gripper (0.0m)

Step Size:
  [ : Decrease step
  ] : Increase step

Ctrl-C to quit
========================================
"""
        print(msg, flush=True)

    def status_callback(self):
        """Print status at 1Hz."""
        print(f"\r[{self.selected_arm.upper()}] Pos: {self.step_position:.4f}m | Rot: {self.step_rotation:.4f}rad   ",
              end='', flush=True)

    def joint_state_callback(self, msg):
        """Update current joint states from /joint_states."""
        try:
            left_indices = []
            for i in range(1, 8):
                joint_name = f'mj_left_joint{i}'
                if joint_name in msg.name:
                    left_indices.append(msg.name.index(joint_name))

            right_indices = []
            for i in range(1, 8):
                joint_name = f'mj_right_joint{i}'
                if joint_name in msg.name:
                    right_indices.append(msg.name.index(joint_name))

            if len(left_indices) == 7 and len(right_indices) == 7:
                if not self.joint_states_received:
                    self.joint_states_received = True
                    print(f"Joint states received. Ready for control!\n", flush=True)

        except Exception as e:
            pass

    def update_from_key(self, key: str) -> bool:
        """Update pose based on key input."""
        # Arm selection
        if key == 'z':
            if self.selected_arm != 'left':
                self.set_selected_arm('left')
                print(f"\n[Switched to LEFT arm]", flush=True)
            return False
        elif key == 'x':
            if self.selected_arm != 'right':
                self.set_selected_arm('right')
                print(f"\n[Switched to RIGHT arm]", flush=True)
            return False
        elif key == 'b':
            if self.selected_arm != 'both':
                self.set_selected_arm('both')
                print(f"\n[Switched to BOTH arms]", flush=True)
            return False

        # Step size adjustment
        elif key == '[':
            self.step_position = max(0.0001, self.step_position * 0.8)
            self.step_rotation = max(0.001, self.step_rotation * 0.8)
            print(f"\n[Step size: pos={self.step_position:.4f}m, rot={self.step_rotation:.4f}rad]", flush=True)
            return False
        elif key == ']':
            self.step_position = min(0.01, self.step_position * 1.25)
            self.step_rotation = min(0.1, self.step_rotation * 1.25)
            print(f"\n[Step size: pos={self.step_position:.4f}m, rot={self.step_rotation:.4f}rad]", flush=True)
            return False

        # Position/rotation deltas
        pos_delta = np.zeros(3)
        rot_delta = np.zeros(3)
        updated = False

        # Position: W/S (X), A/D (Y), Q/E (Z)
        if key == 'w':
            pos_delta[0] = self.step_position
            updated = True
        elif key == 's':
            pos_delta[0] = -self.step_position
            updated = True
        elif key == 'a':
            pos_delta[1] = self.step_position
            updated = True
        elif key == 'd':
            pos_delta[1] = -self.step_position
            updated = True
        elif key == 'q':
            pos_delta[2] = self.step_position
            updated = True
        elif key == 'e':
            pos_delta[2] = -self.step_position
            updated = True

        # Rotation: J/L (roll), I/K (pitch), U/O (yaw)
        elif key == 'j':
            rot_delta[0] = self.step_rotation  # Roll
            updated = True
        elif key == 'l':
            rot_delta[0] = -self.step_rotation
            updated = True
        elif key == 'i':
            rot_delta[1] = self.step_rotation  # Pitch
            updated = True
        elif key == 'k':
            rot_delta[1] = -self.step_rotation
            updated = True
        elif key == 'u':
            rot_delta[2] = self.step_rotation  # Yaw
            updated = True
        elif key == 'o':
            rot_delta[2] = -self.step_rotation
            updated = True

        # Gripper control: N (open), M (close)
        elif key == 'n':
            self.control_gripper(self.gripper_max_width)  # Open gripper
            return True
        elif key == 'm':
            self.control_gripper(self.gripper_min_width)  # Close gripper
            return True

        if updated:
            self.publish_teleop_command(pos_delta, rot_delta)
            return True

        return False

    def publish_teleop_command(self, pos_delta: np.ndarray, rot_delta: np.ndarray):
        """Publish Cartesian increment as teleop command."""
        # Prepare end-effector deltas for both arms
        # Format: [x, y, z, qx, qy, qz] (6 elements per arm)
        left_ee_delta = np.zeros(6)
        right_ee_delta = np.zeros(6)

        if self.selected_arm in ['left', 'both']:
            left_ee_delta[0:3] = pos_delta
            left_ee_delta[3:6] = rot_delta  # Small angle approximation

        if self.selected_arm in ['right', 'both']:
            right_ee_delta[0:3] = pos_delta
            right_ee_delta[3:6] = rot_delta

        # Clamp to safety limits
        left_ee_delta[0:3] = np.clip(left_ee_delta[0:3], -self.max_position_increment, self.max_position_increment)
        left_ee_delta[3:6] = np.clip(left_ee_delta[3:6], -self.max_rotation_increment, self.max_rotation_increment)
        right_ee_delta[0:3] = np.clip(right_ee_delta[0:3], -self.max_position_increment, self.max_position_increment)
        right_ee_delta[3:6] = np.clip(right_ee_delta[3:6], -self.max_rotation_increment, self.max_rotation_increment)

        # Create and publish command message
        msg = self.create_teleop_command(
            arm_selector=self.get_arm_selector(),
            command_type=self.TASK_SPACE_INCREMENT,
            left_ee_delta=left_ee_delta,
            right_ee_delta=right_ee_delta,
            allow_safety_violation=False
        )

        self.teleop_pub.publish(msg)

    def control_gripper(self, width: float):
        """
        Control gripper opening width.

        Publishes to gripper_action_bridge topics which handle the actual
        gripper control in the simulation.

        Args:
            width: Desired gripper width in meters (0.0 = closed, 0.08 = fully open)
        """
        msg = Float64()
        msg.data = width

        gripper_name = ""
        if self.selected_arm == 'left':
            self.left_gripper_pub.publish(msg)
            gripper_name = "LEFT"
        elif self.selected_arm == 'right':
            self.right_gripper_pub.publish(msg)
            gripper_name = "RIGHT"
        elif self.selected_arm == 'both':
            # Control both grippers simultaneously
            self.left_gripper_pub.publish(msg)
            self.right_gripper_pub.publish(msg)
            gripper_name = "BOTH"

        action = "OPEN" if width > 0.04 else "CLOSE"
        print(f"\n[{gripper_name} GRIPPER {action}] Width: {width:.3f}m", flush=True)


def get_key(settings):
    """Non-blocking key input."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    parser = argparse.ArgumentParser(description='Cartesian space keyboard teleop')
    parser.add_argument('--step-position', type=float, default=0.001,
                        help='Position increment per keypress (m)')
    parser.add_argument('--step-rotation', type=float, default=0.01,
                        help='Rotation increment per keypress (rad)')
    parser.add_argument('--arm', type=str, choices=['left', 'right', 'both'],
                        default='left', help='Initial arm selection')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)
    node = KeyCartesianTeleop(
        step_position=parser_args.step_position,
        step_rotation=parser_args.step_rotation
    )
    node.set_selected_arm(parser_args.arm)

    # Create status timer
    status_timer = node.create_timer(1.0, node.status_callback)

    settings = termios.tcgetattr(sys.stdin)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            key = get_key(settings).lower()

            if key == '\x03':  # ctrl-c
                break

            if key:
                node.update_from_key(key)

            time.sleep(0.01)

    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nCartesian teleop stopped.")


if __name__ == '__main__':
    main()
