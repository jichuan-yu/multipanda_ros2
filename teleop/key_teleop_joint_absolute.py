#!/usr/bin/env python3
"""
Keyboard teleoperation for dual Panda robot in joint space using absolute position commands.

This script integrates key commands internally and sends absolute joint positions.
Key mapping is identical to key_teleop_joint.py for consistency.

Key Mapping:
  Joint 1-7 Increase: 1/2/3/4/5/6/7 (top row)
  Joint 1-7 Decrease: Q/W/E/R/T/Y/U (second row)
  Arm Selection: Z (left), X (right), B (both)
  Step Size: [ = decrease, ] = increase
  R: Reset to current joint states

Publishes: Float64MultiArray to /dualarm_teleop_cmd (command_type=4 JOINT_POSITION)

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/key_teleop_joint_absolute.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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

# Add parent directory to path for imports (for direct execution)
if __name__ == '__main__':
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Import base teleop node
try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    # Fallback for direct execution
    from base_teleop import BaseTeleopNode


class KeyJointAbsoluteTeleop(BaseTeleopNode):
    """Keyboard teleoperation for joint space control with absolute position commands."""

    # Command type constants
    JOINT_POSITION = 4

    def __init__(self, step_size: float = 0.002):
        """Initialize key joint absolute teleop node."""
        super().__init__('key_joint_absolute_teleop', publish_rate=100.0)

        self.step_joint = step_size
        self.joint_states_received = False

        # Target joint positions (integrated from keyboard commands)
        self.target_joint_states = {
            'left': self.HOME_POSITION.copy(),
            'right': self.HOME_POSITION.copy()
        }

        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.print_usage()

    def print_usage(self):
        msg = """
========================================
Joint Space Absolute Teleop for Dual Panda
========================================
Joint Increment (+):
  1/2/3/4/5/6/7 : Joint 1-7

Joint Decrement (-):
  Q/W/E/R/T/Y/U : Joint 1-7

Arm Selection:
  Z : Left arm
  X : Right arm
  B : Both arms

Step Size:
  [ : Decrease step
  ] : Increase step

Reset:
  R : Reset target to current joint states

Ctrl-C to quit
========================================
Waiting for joint states...
"""
        print(msg, flush=True)

    def status_callback(self):
        """Print status at 1Hz."""
        if self.joint_states_received:
            left_target = self.target_joint_states['left']
            right_target = self.target_joint_states['right']
            print(f"\r[{self.selected_arm.upper()}] Step: {self.step_joint:.4f} | "
                  f"L: [{left_target[0]:7.3f}, {left_target[1]:7.3f}, {left_target[2]:7.3f}, {left_target[3]:7.3f}, {left_target[4]:7.3f}, {left_target[5]:7.3f}, {left_target[6]:7.3f}] | "
                  f"R: [{right_target[0]:7.3f}, {right_target[1]:7.3f}, {right_target[2]:7.3f}, {right_target[3]:7.3f}, {right_target[4]:7.3f}, {right_target[5]:7.3f}, {right_target[6]:7.3f}]   ",
                  end='', flush=True)

    def joint_state_callback(self, msg):
        """Update current joint states from /joint_states."""
        try:
            # Extract left arm joints
            left_indices = []
            for i in range(1, 8):
                joint_name = f'mj_left_joint{i}'
                if joint_name in msg.name:
                    left_indices.append(msg.name.index(joint_name))

            # Extract right arm joints
            right_indices = []
            for i in range(1, 8):
                joint_name = f'mj_right_joint{i}'
                if joint_name in msg.name:
                    right_indices.append(msg.name.index(joint_name))

            if len(left_indices) == 7:
                self.current_joint_states['left'] = np.array([msg.position[i] for i in left_indices])

            if len(right_indices) == 7:
                self.current_joint_states['right'] = np.array([msg.position[i] for i in right_indices])

            if not self.joint_states_received and len(left_indices) == 7 and len(right_indices) == 7:
                self.joint_states_received = True
                # Initialize target positions to current positions
                self.target_joint_states['left'] = self.current_joint_states['left'].copy()
                self.target_joint_states['right'] = self.current_joint_states['right'].copy()
                print(f"\nJoint states received. Ready for control!", flush=True)

        except Exception as e:
            pass

    def update_joint(self, key: str) -> bool:
        """Update joint angle based on key input."""
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
            self.step_joint = max(0.0005, self.step_joint * 0.8)
            print(f"\n[Step size: {self.step_joint:.4f} rad]", flush=True)
            return False
        elif key == ']':
            self.step_joint = min(0.05, self.step_joint * 1.25)
            print(f"\n[Step size: {self.step_joint:.4f} rad]", flush=True)
            return False

        # Joint commands - integrate from ACTUAL current position
        delta = np.zeros(7)
        updated = False

        # Increase: 1/2/3/4/5/6/7
        increase_keys = ['1', '2', '3', '4', '5', '6', '7']
        if key in increase_keys:
            joint_idx = increase_keys.index(key)
            delta[joint_idx] = self.step_joint
            updated = True

        # Decrease: Q/W/E/R/T/Y/U
        decrease_keys = ['q', 'w', 'e', 'r', 't', 'y', 'u']
        if key in decrease_keys:
            joint_idx = decrease_keys.index(key)
            delta[joint_idx] = -self.step_joint
            updated = True

        if updated:
            # Use ACTUAL current joint states as base, not internal target
            if self.selected_arm in ['left', 'both']:
                # Base = actual position + delta
                target = self.current_joint_states['left'] + delta
                # Clamp to joint limits
                target = self.clamp_joint_position(target)
                self.target_joint_states['left'] = target

            if self.selected_arm in ['right', 'both']:
                # Base = actual position + delta
                target = self.current_joint_states['right'] + delta
                # Clamp to joint limits
                target = self.clamp_joint_position(target)
                self.target_joint_states['right'] = target

            self.publish_teleop_command()
            return True

        return False

    def publish_teleop_command(self):
        """Publish absolute joint position as teleop command."""
        # Create and publish command message
        msg = self.create_teleop_command(
            arm_selector=self.get_arm_selector(),
            command_type=self.JOINT_POSITION,
            left_joint_delta=self.target_joint_states['left'],
            right_joint_delta=self.target_joint_states['right'],
            allow_safety_violation=False
        )

        self.teleop_pub.publish(msg)


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
    parser = argparse.ArgumentParser(description='Joint space keyboard teleop with absolute position commands')
    parser.add_argument('--step-size', type=float, default=0.002,
                        help='Joint increment per keypress (rad)')
    parser.add_argument('--arm', type=str, choices=['left', 'right', 'both'],
                        default='left', help='Initial arm selection')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)
    node = KeyJointAbsoluteTeleop(step_size=parser_args.step_size)
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
                node.update_joint(key)

            time.sleep(0.01)

    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nJoint absolute teleop stopped.")


if __name__ == '__main__':
    main()
