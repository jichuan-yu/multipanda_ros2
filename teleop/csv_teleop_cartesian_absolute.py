#!/usr/bin/env python3
"""
CSV trajectory playback for dual Panda robot in Cartesian/task space using ABSOLUTE pose commands.
Sends absolute end-effector poses instead of increments.

CSV Format:
    timestamp,x_left,y_left,z_left,qx_left,qy_left,qz_left,qw_left,x_right,...,qw_right

Publishes: Float64MultiArray to /dualarm_teleop_cmd (command_type=5 TASK_SPACE_POSE)

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/csv_teleop_cartesian_absolute.py --csv-file trajectory.csv
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import threading
import numpy as np
import argparse
import time
import csv
import os

# Add parent directory to path for imports
if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Import base teleop node
try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    # Fallback for direct execution
    from base_teleop import BaseTeleopNode


class CSVCartesianAbsoluteTeleop(BaseTeleopNode):
    """CSV playback for Cartesian space control using absolute pose commands."""

    # Command type constants
    TASK_SPACE_POSE = 5

    def __init__(self, csv_file: str, loop: bool = False, publish_rate: float = 100.0, allow_safety_violation: bool = False):
        """Initialize CSV Cartesian absolute teleop node."""
        super().__init__('csv_cartesian_absolute_teleop', publish_rate=publish_rate)

        self.csv_file = csv_file
        self.loop = loop
        self.allow_safety_violation = allow_safety_violation
        self.joint_states_received = False
        self.current_index = 0
        self.start_time = None

        # Load trajectory
        self.trajectory = self.load_csv(csv_file)

        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Control timer
        self.control_timer = self.create_timer(1.0 / publish_rate, self.control_callback)

        self.get_logger().info(f'Loaded {len(self.trajectory)} points from {csv_file}')
        self.print_usage()

    def print_usage(self):
        msg = f"""
========================================
CSV Task Space ABSOLUTE Teleop Playback
========================================
File: {self.csv_file}
Points: {len(self.trajectory)}
Loop: {self.loop}
Mode: Sending absolute pose commands
========================================
Playing trajectory...
"""
        print(msg, flush=True)

    def load_csv(self, csv_file: str) -> list:
        """Load pose trajectory from CSV file."""
        trajectory = []
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)  # Skip header

                for row in reader:
                    if len(row) >= 15:  # timestamp + 7 left (pos+quat) + 7 right
                        timestamp = float(row[0])
                        # Left arm: position + quaternion [x,y,z,qx,qy,qz,qw]
                        left_pos = np.array([float(x) for x in row[1:4]])
                        left_quat = np.array([float(x) for x in row[4:8]])
                        left_ee = np.concatenate([left_pos, left_quat])  # 7 elements

                        # Right arm: position + quaternion [x,y,z,qx,qy,qz,qw]
                        right_pos = np.array([float(x) for x in row[8:11]])
                        right_quat = np.array([float(x) for x in row[11:15]])
                        right_ee = np.concatenate([right_pos, right_quat])  # 7 elements

                        trajectory.append((timestamp, left_ee, right_ee))

            return trajectory

        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {e}')
            raise

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
                    print(f"Joint states received. Starting playback...", flush=True)

        except Exception as e:
            pass

    def control_callback(self):
        """Main control loop - publishes absolute pose teleop commands."""
        if not self.trajectory or not self.joint_states_received:
            return

        # Initialize start time
        if self.start_time is None:
            self.start_time = time.time()
            return

        # Get current trajectory time
        current_time = time.time() - self.start_time

        # Find corresponding trajectory point
        while (self.current_index < len(self.trajectory) and
               self.trajectory[self.current_index][0] < current_time):
            self.current_index += 1

        # Check if trajectory is finished
        if self.current_index >= len(self.trajectory):
            if self.loop:
                # Restart trajectory
                self.start_time = time.time()
                self.current_index = 0
                return
            else:
                # Trajectory finished
                print(f"\nTrajectory playback complete.", flush=True)
                self.control_timer.cancel()
                return

        # Get target pose (absolute)
        timestamp, left_ee, right_ee = self.trajectory[self.current_index]

        # Publish as absolute teleop command
        self.publish_teleop_absolute_pose(left_ee, right_ee)

        # Print progress
        progress = (self.current_index / len(self.trajectory)) * 100
        print(f"\rProgress: {progress:.1f}% | "
              f"L: [{left_ee[0]:6.3f}, {left_ee[1]:6.3f}, {left_ee[2]:6.3f}] | "
              f"R: [{right_ee[0]:6.3f}, {right_ee[1]:6.3f}, {right_ee[2]:6.3f}]   ",
              end='', flush=True)

    def publish_teleop_absolute_pose(self, left_ee: np.ndarray, right_ee: np.ndarray):
        """Publish absolute Cartesian pose as teleop command."""
        # Create and publish teleop command
        msg = self.create_teleop_command(
            arm_selector=self.BOTH_ARMS,
            command_type=self.TASK_SPACE_POSE,
            left_ee_delta=left_ee,   # Contains [x,y,z,qx,qy,qz,qw]
            right_ee_delta=right_ee,  # Contains [x,y,z,qx,qy,qz,qw]
            allow_safety_violation=self.allow_safety_violation
        )

        self.teleop_pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='CSV Cartesian space absolute teleop')
    parser.add_argument('--csv-file', type=str, required=True,
                        help='Path to CSV file with pose trajectory')
    parser.add_argument('--loop', action='store_true',
                        help='Loop the trajectory')
    parser.add_argument('--rate', type=float, default=100.0,
                        help='Publishing rate (Hz)')
    parser.add_argument('--disable-safety', action='store_true',
                        help='Disable collision avoidance (allow_safety_violation=1)')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        node = CSVCartesianAbsoluteTeleop(
            csv_file=parser_args.csv_file,
            loop=parser_args.loop,
            publish_rate=parser_args.rate,
            allow_safety_violation=parser_args.disable_safety
        )

        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()

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
        rclpy.shutdown()
        print("CSV absolute teleop stopped.")


if __name__ == '__main__':
    main()
