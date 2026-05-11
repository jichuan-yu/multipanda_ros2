#!/usr/bin/env python3
"""
SpaceMouse teleoperation for dual Panda robot in Cartesian/task space.

SpaceMouse Mapping:
  Translation: Direct mapping (scaled)
  Rotation: Direct mapping (scaled)
  Buttons:
    Button 1: Select left arm
    Button 2: Select right arm

Usage:
    source ~/myenv/bin/activate
    source /path/to/dual_panda_ws/install/setup.bash
    python src/multipanda_ros2/teleop/spacemouse_teleop_cartesian.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import numpy as np
import argparse
import time

from .base_teleop import BaseTeleopNode
from .utils.transformations import small_angle_to_delta_rotation, pose_to_msg

# Add tools directory to path for pyspacemouse import
import sys
import os
_tools_path = os.path.join(os.path.dirname(__file__), '..', 'tools')
if _tools_path not in sys.path:
    sys.path.insert(0, _tools_path)
import pyspacemouse


class SpaceMouseTeleop(BaseTeleopNode):
    """SpaceMouse teleoperation for Cartesian space control."""

    def __init__(
        self,
        scale_translation: float = 0.0001,
        scale_rotation: float = 0.0005,
        deadzone: float = 0.05
    ):
        """
        Initialize SpaceMouse teleop node.

        Args:
            scale_translation: Translation scale factor (m per unit)
            scale_rotation: Rotation scale factor (rad per unit)
            deadzone: Deadzone for axis values (normalized 0-1)
        """
        super().__init__('spacemouse_teleop', publish_rate=100.0)

        self.scale_translation = scale_translation
        self.scale_rotation = scale_rotation
        self.deadzone = deadzone
        self.joint_states_received = False

        # Initialize SpaceMouse
        try:
            self.spacemouse_device = pyspacemouse.open()
            print(f"Connected to SpaceMouse", flush=True)
        except Exception as e:
            self.get_logger().error(f'Failed to open SpaceMouse: {e}')
            self.spacemouse_device = None

        # Previous button state for edge detection
        self.prev_buttons = [0, 0]

        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Status and control timers
        self.status_timer = self.create_timer(1.0, self.status_callback)
        self.control_timer = self.create_timer(1.0 / 100.0, self.control_callback)

        self.get_logger().info('SpaceMouse Teleop initialized')
        self.print_usage()

    def print_usage(self):
        msg = """
========================================
SpaceMouse Teleop for Dual Panda
========================================
Translation: Push/pull for XYZ
Rotation: Twist for roll/pitch/yaw

Buttons:
  Button 1: Left arm
  Button 2: Right arm

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
            print(f"\r[{self.selected_arm.upper()}] | "
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

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to a value."""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def control_callback(self):
        """Main control loop - reads SpaceMouse and publishes commands."""
        if self.spacemouse_device is None:
            return

        state = pyspacemouse.read()
        if state is None:
            return

        # Check button presses for arm selection
        self.handle_buttons(list(state.buttons))

        # Read translation and rotation (apply deadzone)
        x = self.apply_deadzone(state.x) * self.scale_translation
        y = self.apply_deadzone(state.y) * self.scale_translation
        z = self.apply_deadzone(state.z) * self.scale_translation

        roll = self.apply_deadzone(state.roll) * self.scale_rotation
        pitch = self.apply_deadzone(state.pitch) * self.scale_rotation
        yaw = self.apply_deadzone(state.yaw) * self.scale_rotation

        # Check if there's any meaningful input
        pos_delta = np.array([x, y, z])
        rot_delta = np.array([roll, pitch, yaw])

        if np.linalg.norm(pos_delta) > 1e-9 or np.linalg.norm(rot_delta) > 1e-9:
            self.publish_command(pos_delta, rot_delta)

    def handle_buttons(self, buttons: list):
        """
        Handle button presses for arm selection.

        Args:
            buttons: Current button state list
        """
        # Ensure we have at least 2 buttons
        while len(buttons) < 2:
            buttons.append(0)

        # Check for button press edges
        for i in range(min(2, len(self.prev_buttons))):
            prev = self.prev_buttons[i]
            curr = buttons[i] if i < len(buttons) else 0
            if prev == 0 and curr == 1:  # Rising edge
                if i == 0:
                    self.selected_arm = 'left'
                    print(f"\n[Switched to LEFT arm]", flush=True)
                elif i == 1:
                    self.selected_arm = 'right'
                    print(f"\n[Switched to RIGHT arm]", flush=True)

        self.prev_buttons = buttons.copy()

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


def main(args=None):
    parser = argparse.ArgumentParser(description='SpaceMouse Cartesian teleop')
    parser.add_argument('--scale-translation', type=float, default=0.0001,
                        help='Translation scale factor (m per unit)')
    parser.add_argument('--scale-rotation', type=float, default=0.0005,
                        help='Rotation scale factor (rad per unit)')
    parser.add_argument('--deadzone', type=float, default=0.05,
                        help='Deadzone for axis values (normalized 0-1)')
    parser.add_argument('--arm', type=str, choices=['left', 'right', 'both'],
                        default='left', help='Initial arm selection')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        node = SpaceMouseTeleop(
            scale_translation=parser_args.scale_translation,
            scale_rotation=parser_args.scale_rotation,
            deadzone=parser_args.deadzone
        )
        node.set_selected_arm(parser_args.arm)

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()

        # Main thread just waits
        while rclpy.ok():
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        pyspacemouse.close()
        rclpy.shutdown()
        print("SpaceMouse teleop stopped.")


if __name__ == '__main__':
    main()
