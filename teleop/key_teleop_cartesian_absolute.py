#!/usr/bin/env python3
"""
Keyboard teleoperation for dual Panda robot in Cartesian/task space using absolute pose commands.

This script uses LAST COMMAND position as integration base (not actual EE pose).
Key mapping is identical to key_teleop_cartesian.py for consistency.

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
  - Float64MultiArray to /dualarm_teleop_cmd (command_type=5 TASK_SPACE_POSE)
  - Float64MultiArray to /mj_{left,right}_gripper/grasp_desired (gripper force control)
Subscribes: /ee_pose for actual end-effector pose feedback (for display only)

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/key_teleop_cartesian_absolute.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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


def quaternion_multiply(q1, q2):
    """Multiply two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quaternion_from_axis_angle(axis, angle):
    """Create quaternion from axis-angle representation."""
    axis = axis / np.linalg.norm(axis)
    half_angle = angle / 2.0
    sin_half = np.sin(half_angle)
    return np.array([np.cos(half_angle), axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half])


def quaternion_to_array(q):
    """Convert quaternion [w, x, y, z] to [x, y, z, w] format for message."""
    return np.array([q[1], q[2], q[3], q[0]])


def quaternion_from_array(q_array):
    """Convert [x, y, z, w] to quaternion [w, x, y, z]."""
    return np.array([q_array[3], q_array[0], q_array[1], q_array[2]])


class KeyCartesianAbsoluteTeleop(BaseTeleopNode):
    """Keyboard teleoperation for Cartesian space control with absolute pose commands.

    Uses LAST COMMAND position as integration base (accumulates deltas from previous command).
    First command uses default home position as base.
    """

    # Command type constants
    TASK_SPACE_POSE = 5

    def __init__(self, step_position: float = 0.001, step_rotation: float = 0.01):
        """Initialize key Cartesian absolute teleop node."""
        super().__init__('key_cartesian_absolute_teleop', publish_rate=100.0)

        self.step_position = step_position
        self.step_rotation = step_rotation
        self.ee_pose_received = False

        # Gripper control parameters
        self.gripper_max_width = 0.08  # Franka gripper max opening (m)
        self.gripper_min_width = 0.0   # Fully closed

        # ACTUAL end-effector poses from /ee_pose topic (position + quaternion [w, x, y, z])
        # Default home position (gripper tip position, not flange)
        self.actual_ee_poses = {
            'left': {
                'position': np.array([0.307, 0.26, 0.487]),
                'quaternion': np.array([0.0, 1.0, 0.0, 0.0])  # [w, x, y, z] - 180° X rotation
            },
            'right': {
                'position': np.array([0.307, -0.26, 0.487]),
                'quaternion': np.array([0.0, 1.0, 0.0, 0.0])  # [w, x, y, z] - 180° X rotation
            }
        }

        # Subscriber for actual end-effector poses
        self.ee_pose_sub = self.create_subscription(
            Float64MultiArray,
            '/ee_pose',
            self.ee_pose_callback,
            10
        )

        # Gripper control publishers (for gripper_action_bridge - force control)
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/mj_left_gripper/grasp_desired',
            10
        )
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/mj_right_gripper/grasp_desired',
            10
        )

        self.print_usage()
        self.target_ee_poses = {
            'left': {
                'position': np.array([0.307, 0.26, 0.487]),
                'quaternion': np.array([0.0, 1.0, 0.0, 0.0])  # [w, x, y, z] - 180° X rotation
            },
            'right': {
                'position': np.array([0.307, -0.26, 0.487]),
                'quaternion': np.array([0.0, 1.0, 0.0, 0.0])  # [w, x, y, z] - 180° X rotation
            }
        }

    def print_usage(self):
        msg = """
========================================
Task Space Absolute Teleop for Dual Panda
(Using LAST COMMAND position as integration base)
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

Note: Each keypress uses LAST COMMAND position as base
      (deltas accumulate from previous command)
      First command uses default home position

Ctrl-C to quit
========================================
"""
        print(msg, flush=True)

    def ee_pose_callback(self, msg):
        """Update actual end-effector poses from /ee_pose topic."""
        try:
            if len(msg.data) >= 14:
                # Left arm: [x, y, z, qx, qy, qz, qw]
                left_pos = np.array([msg.data[0], msg.data[1], msg.data[2]])
                left_quat = quaternion_from_array([msg.data[3], msg.data[4], msg.data[5], msg.data[6]])

                # Right arm: [x, y, z, qx, qy, qz, qw]
                right_pos = np.array([msg.data[7], msg.data[8], msg.data[9]])
                right_quat = quaternion_from_array([msg.data[10], msg.data[11], msg.data[12], msg.data[13]])

                self.actual_ee_poses['left']['position'] = left_pos
                self.actual_ee_poses['left']['quaternion'] = left_quat
                self.actual_ee_poses['right']['position'] = right_pos
                self.actual_ee_poses['right']['quaternion'] = right_quat

                if not self.ee_pose_received:
                    self.ee_pose_received = True
                    print(f"\nEnd-effector pose feedback received. Ready for control!\n", flush=True)

        except Exception as e:
            pass

    def status_callback(self):
        """Print status at 1Hz."""
        if self.ee_pose_received:
            left_target_pos = self.target_ee_poses['left']['position']
            right_target_pos = self.target_ee_poses['right']['position']
            left_actual_pos = self.actual_ee_poses['left']['position']
            right_actual_pos = self.actual_ee_poses['right']['position']
            print(f"\r[{self.selected_arm.upper()}] Pos: {self.step_position:.4f}m | Rot: {self.step_rotation:.4f}rad | "
                  f"L_target: [{left_target_pos[0]:7.3f}, {left_target_pos[1]:7.3f}, {left_target_pos[2]:7.3f}] | "
                  f"R_target: [{right_target_pos[0]:7.3f}, {right_target_pos[1]:7.3f}, {right_target_pos[2]:7.3f}]   ",
                  end='', flush=True)

    def update_from_key(self, key: str) -> bool:
        """Update pose based on key input, using LAST COMMAND position as base."""
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
            # Use LAST COMMAND position as base (accumulates deltas from previous command)
            if self.selected_arm in ['left', 'both']:
                # Base = last command position + delta
                target_pos = self.target_ee_poses['left']['position'] + pos_delta

                # Base = last command orientation + rotation delta
                if np.linalg.norm(rot_delta) > 0:
                    rot_angle = np.linalg.norm(rot_delta)
                    rot_axis = rot_delta / rot_angle
                    delta_quat = quaternion_from_axis_angle(rot_axis, rot_angle)
                    target_quat = quaternion_multiply(
                        delta_quat, self.target_ee_poses['left']['quaternion']
                    )
                    # Normalize quaternion
                    target_quat /= np.linalg.norm(target_quat)
                else:
                    target_quat = self.target_ee_poses['left']['quaternion'].copy()

                # Update last command position
                self.target_ee_poses['left']['position'] = target_pos
                self.target_ee_poses['left']['quaternion'] = target_quat

            if self.selected_arm in ['right', 'both']:
                # Base = last command position + delta
                target_pos = self.target_ee_poses['right']['position'] + pos_delta

                # Base = last command orientation + rotation delta
                if np.linalg.norm(rot_delta) > 0:
                    rot_angle = np.linalg.norm(rot_delta)
                    rot_axis = rot_delta / rot_angle
                    delta_quat = quaternion_from_axis_angle(rot_axis, rot_angle)
                    target_quat = quaternion_multiply(
                        delta_quat, self.target_ee_poses['right']['quaternion']
                    )
                    # Normalize quaternion
                    target_quat /= np.linalg.norm(target_quat)
                else:
                    target_quat = self.target_ee_poses['right']['quaternion'].copy()

                # Update last command position
                self.target_ee_poses['right']['position'] = target_pos
                self.target_ee_poses['right']['quaternion'] = target_quat

            self.publish_teleop_command()
            return True

        return False

    def publish_teleop_command(self):
        """Publish absolute end-effector pose as teleop command."""
        # Format: [left_x,y,z,qx,qy,qz,qw, right_x,y,z,qx,qy,qz,qw] (14 elements)
        left_pose_data = np.concatenate([
            self.target_ee_poses['left']['position'],
            quaternion_to_array(self.target_ee_poses['left']['quaternion'])  # [x, y, z, w]
        ])

        right_pose_data = np.concatenate([
            self.target_ee_poses['right']['position'],
            quaternion_to_array(self.target_ee_poses['right']['quaternion'])  # [x, y, z, w]
        ])

        # Create and publish command message
        msg = self.create_teleop_command(
            arm_selector=self.get_arm_selector(),
            command_type=self.TASK_SPACE_POSE,
            left_ee_delta=left_pose_data,
            right_ee_delta=right_pose_data,
            allow_safety_violation=False
        )

        self.teleop_pub.publish(msg)

        # Print published data
        print(f"\n[Publishing] data: {msg.data}")

    def control_gripper(self, width: float):
        """
        Control gripper with force-based grasp.

        Publishes to gripper_action_bridge topics which handle the actual
        gripper control in the simulation using force control parameters.

        Args:
            width: Desired gripper width in meters (0.0 = closed, 0.08 = fully open)

        Grasp data format: [width, speed, force, epsilon_inner, epsilon_outer]
        - width: Target width (m)
        - speed: Closing speed (m/s)
        - force: Grasping force (N)
        - epsilon_inner: Inner tolerance (m)
        - epsilon_outer: Outer tolerance (m)
        """
        msg = Float64MultiArray()
        # Hardcoded force control parameters
        msg.data = [
            width,          # Target width (m)
            0.05,           # Speed (m/s)
            30.0,           # Force (N)
            0.005,          # epsilon_inner (m)
            0.005           # epsilon_outer (m)
        ]

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
        print(f"\n[{gripper_name} GRIPPER {action}] Width: {width:.3f}m | Speed: 0.05m/s | Force: 30.0N", flush=True)


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
    parser = argparse.ArgumentParser(description='Cartesian space keyboard teleop with absolute pose commands')
    parser.add_argument('--step-position', type=float, default=0.001,
                        help='Position increment per keypress (m)')
    parser.add_argument('--step-rotation', type=float, default=0.01,
                        help='Rotation increment per keypress (rad)')
    parser.add_argument('--arm', type=str, choices=['left', 'right', 'both'],
                        default='left', help='Initial arm selection')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)
    node = KeyCartesianAbsoluteTeleop(
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
        print("\nCartesian absolute teleop stopped.")


if __name__ == '__main__':
    main()
