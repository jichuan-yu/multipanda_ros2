#!/usr/bin/env python3
"""
SpaceMouse teleoperation for dual Panda robot in Cartesian/task space.

SpaceMouse Mapping:
  Translation: Direct mapping (scaled)
  Rotation: Direct mapping (scaled)
  Buttons:
    Button 0: Select left arm
    Button 1: Select right arm

Publishes: Float64MultiArray to /dualarm_teleop_cmd (standard ROS2 message)

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/spacemouse_teleop_cartesian.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import threading
import numpy as np
import argparse
import time
import os

# Add tools directory to path for pyspacemouse import
_tools_path = os.path.join(os.path.dirname(__file__), '..', 'tools')
if _tools_path not in os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', '..'):
    import sys
    sys.path.insert(0, _tools_path)
import pyspacemouse

# Add parent directory to path for base_teleop import
if __name__ == '__main__':
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Import base teleop node
try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    # Fallback for direct execution
    from base_teleop import BaseTeleopNode


class SpaceMouseTeleop(BaseTeleopNode):
    """SpaceMouse teleoperation for Cartesian space control."""

    # Command type constants
    TASK_SPACE_INCREMENT = 2

    def __init__(
        self,
        scale_translation: float = 0.0000007,
        scale_rotation: float = 0.0000035,
        deadzone: float = 0.05
    ):
        """Initialize SpaceMouse teleop node."""
        super().__init__('spacemouse_teleop', publish_rate=100.0)

        self.scale_translation = scale_translation
        self.scale_rotation = scale_rotation
        self.deadzone = deadzone
        self.joint_states_received = False

        # Initialize SpaceMouse
        try:
            success = pyspacemouse.open()
            if not success:
                self.get_logger().error("Could not connect to SpaceMouse")
                raise RuntimeError("SpaceMouse not found")
            print("Connected to SpaceMouse", flush=True)
        except Exception as e:
            self.get_logger().error(f'Failed to open SpaceMouse: {e}')
            raise

        self.selected_arm = 'left'
        self.prev_buttons = [0, 0]

        # Status timer
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # Control timer (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_callback)

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
"""
        print(msg, flush=True)

    def status_callback(self):
        """Print status at 1Hz."""
        print(f"\r[{self.selected_arm.upper()}] | Scale: T={self.scale_translation:.2e} R={self.scale_rotation:.2e}   ",
              end='', flush=True)

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to a value."""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def control_callback(self):
        """Main control loop - reads SpaceMouse and publishes commands."""
        state = pyspacemouse.read()
        if state is None:
            return

        # Handle buttons for arm selection
        try:
            curr_buttons = [bool(state.buttons[0]), bool(state.buttons[1])]
        except (IndexError, TypeError):
            curr_buttons = [False, False]

        # Button press detection
        for i, (prev, curr) in enumerate(zip(self.prev_buttons, curr_buttons)):
            if not prev and curr:  # Rising edge
                if i == 0:
                    self.set_selected_arm('left')
                    print(f"\n[Switched to LEFT arm]", flush=True)
                elif i == 1:
                    self.set_selected_arm('right')
                    print(f"\n[Switched to RIGHT arm]", flush=True)

        self.prev_buttons = curr_buttons

        # Apply deadzone and scale
        dx = self.apply_deadzone(state.x) * self.scale_translation
        dy = self.apply_deadzone(state.y) * self.scale_translation
        dz = self.apply_deadzone(state.z) * self.scale_translation

        droll = self.apply_deadzone(state.roll) * self.scale_rotation
        dpitch = self.apply_deadzone(state.pitch) * self.scale_rotation
        dyaw = self.apply_deadzone(state.yaw) * self.scale_rotation

        # Update pose if there's meaningful input
        if abs(dx) > 1e-12 or abs(dy) > 1e-12 or abs(dz) > 1e-12 or \
           abs(droll) > 1e-12 or abs(dpitch) > 1e-12 or abs(dyaw) > 1e-12:
            self.publish_teleop_command(np.array([dx, dy, dz]), np.array([droll, dpitch, dyaw]))

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


def main(args=None):
    parser = argparse.ArgumentParser(description='SpaceMouse Cartesian teleop')
    parser.add_argument('--scale-translation', type=float, default=0.0000007,
                        help='Translation scale factor')
    parser.add_argument('--scale-rotation', type=float, default=0.0000035,
                        help='Rotation scale factor')
    parser.add_argument('--deadzone', type=float, default=0.05,
                        help='Deadzone for axis values')
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
