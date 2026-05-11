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
Gripper:
  C: Close
  V: Open
Step Size:
  [: Decrease step
  ]: Increase step

Usage:
    source ~/myenv/bin/activate
    source /path/to/dual_panda_ws/install/setup.bash
    python src/multipanda_ros2/teleop/key_teleop_cartesian.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
import threading
import numpy as np
import argparse
import time

from .base_teleop import BaseTeleopNode
from .utils.transformations import small_angle_to_delta_rotation, pose_to_msg


class KeyCartesianTeleop(BaseTeleopNode):
    """Keyboard teleoperation for Cartesian space control."""

    def __init__(self, step_position: float = 0.001, step_rotation: float = 0.01):
        """
        Initialize key Cartesian teleop node.

        Args:
            step_position: Position increment per keypress in meters
            step_rotation: Rotation increment per keypress in radians
        """
        super().__init__('key_cartesian_teleop', publish_rate=100.0)

        self.step_position = step_position
        self.step_rotation = step_rotation
        self.joint_states_received = False

        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Status timer
        self.status_timer = self.create_timer(1.0, self.status_callback)

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

Step Size:
  [ : Decrease step
  ] : Increase step

Ctrl-C to quit
========================================
Waiting for joint states...
"""
        print(msg, flush=True)

    def status_callback(self):
        """Print status at 1Hz."""
        if self.joint_states_received:
            left_q = self.current_joint_states['left']
            right_q = self.current_joint_states['right']
            print(f"\r[{self.selected_arm.upper()}] Pos: {self.step_position:.4f}m | Rot: {self.step_rotation:.4f}rad | "
                  f"L: [{left_q[0]:6.2f}, {left_q[1]:6.2f}, ...] | "
                  f"R: [{right_q[0]:6.2f}, {right_q[1]:6.2f}, ...]   ",
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
                print(f"\nJoint states received. Ready for control!", flush=True)

        except Exception as e:
            pass

    def update_from_key(self, key: str) -> bool:
        """
        Update pose based on key input.

        Args:
            key: Pressed key character

        Returns:
            True if command was published
        """
        # Arm selection
        if key == 'z':
            if self.selected_arm != 'left':
                self.selected_arm = 'left'
                print(f"\n[Switched to LEFT arm]", flush=True)
            return False
        elif key == 'x':
            if self.selected_arm != 'right':
                self.selected_arm = 'right'
                print(f"\n[Switched to RIGHT arm]", flush=True)
            return False
        elif key == 'b':
            if self.selected_arm != 'both':
                self.selected_arm = 'both'
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

        if updated:
            self.publish_command(pos_delta, rot_delta)
            return True

        return False

    def publish_command(self, pos_delta: np.ndarray, rot_delta: np.ndarray):
        """
        Publish Cartesian increment command.

        Args:
            pos_delta: 3-element position increment [x, y, z]
            rot_delta: 3-element rotation increment [roll, pitch, yaw]
        """
        # Clamp to safety limits
        pos_delta = np.clip(pos_delta, -self.max_position_increment, self.max_position_increment)
        rot_delta = np.clip(rot_delta, -self.max_rotation_increment, self.max_rotation_increment)

        # Convert rotation to quaternion (small angle approximation)
        rot_quat = small_angle_to_delta_rotation(rot_delta)

        # Create Pose message
        pose_msg = pose_to_msg(pos_delta, rot_quat)

        # Create command message
        left_pose = pose_msg if self.selected_arm in ['left', 'both'] else None
        right_pose = pose_msg if self.selected_arm in ['right', 'both'] else None

        msg = self.create_teleop_command(
            arm_selector=self.get_arm_selector(),
            command_type=self.TASK_SPACE_INCREMENT,
            left_ee_delta=left_pose,
            right_ee_delta=right_pose
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
