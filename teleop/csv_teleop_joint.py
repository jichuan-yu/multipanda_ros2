#!/usr/bin/env python3
"""
CSV trajectory playback for dual Panda robot in joint space.
Simulates teleoperation commands by playing back CSV data.

CSV Format:
    timestamp,q1_left,q2_left,q3_left,q4_left,q5_left,q6_left,q7_left,q1_right,...,q7_right

Publishes: Float64MultiArray to /dualarm_teleop_cmd (teleop interface)

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/csv_teleop_joint.py --csv-file trajectory.csv
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


class CSVJointTeleop(BaseTeleopNode):
    """CSV playback for joint space control via teleop interface."""

    # Command type constants
    JOINT_POSITION_INCREMENT = 0

    def __init__(self, csv_file: str, loop: bool = False, publish_rate: float = 100.0):
        """Initialize CSV joint teleop node."""
        super().__init__('csv_joint_teleop', publish_rate=publish_rate)

        self.csv_file = csv_file
        self.loop = loop
        self.joint_states_received = False
        self.current_index = 0
        self.start_time = None
        self.prev_q = None

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
CSV Joint Space Teleop Playback
========================================
File: {self.csv_file}
Points: {len(self.trajectory)}
Loop: {self.loop}
Mode: Sending as teleop commands
========================================
Playing trajectory...
"""
        print(msg, flush=True)

    def load_csv(self, csv_file: str) -> list:
        """Load joint trajectory from CSV file."""
        trajectory = []
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)  # Skip header

                for row in reader:
                    if len(row) >= 15:  # timestamp + 7 left + 7 right
                        timestamp = float(row[0])
                        left_joints = np.array([float(x) for x in row[1:8]])
                        right_joints = np.array([float(x) for x in row[8:15]])
                        # Clamp to limits
                        left_joints = np.clip(left_joints, self.JOINT_LIMITS_LOWER, self.JOINT_LIMITS_UPPER)
                        right_joints = np.clip(right_joints, self.JOINT_LIMITS_LOWER, self.JOINT_LIMITS_UPPER)
                        trajectory.append((timestamp, left_joints, right_joints))

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
                    # Initialize prev_q with current state
                    self.current_joint_states['left'] = np.array([msg.position[i] for i in left_indices])
                    self.current_joint_states['right'] = np.array([msg.position[i] for i in right_indices])
                    print(f"Joint states received. Starting playback...", flush=True)

        except Exception as e:
            pass

    def control_callback(self):
        """Main control loop - publishes trajectory increments as teleop commands."""
        if not self.trajectory or not self.joint_states_received:
            return

        # Initialize start time
        if self.start_time is None:
            self.start_time = time.time()
            _, left_joints, right_joints = self.trajectory[0]
            self.prev_q = np.concatenate([left_joints, right_joints])
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
                _, left_joints, right_joints = self.trajectory[0]
                self.prev_q = np.concatenate([left_joints, right_joints])
                return
            else:
                # Trajectory finished
                print(f"\nTrajectory playback complete.", flush=True)
                self.control_timer.cancel()
                return

        # Get target position
        timestamp, left_joints, right_joints = self.trajectory[self.current_index]
        q_target = np.concatenate([left_joints, right_joints])

        # Calculate increment from previous position
        delta = q_target - self.prev_q

        # Clamp increment to safety limits
        max_delta = 0.002  # Same as controller limit
        delta = np.clip(delta, -max_delta, max_delta)

        # Publish as teleop command
        self.publish_teleop_increment(delta)

        self.prev_q = q_target

        # Print progress
        progress = (self.current_index / len(self.trajectory)) * 100
        print(f"\rProgress: {progress:.1f}% | "
              f"L: [{left_joints[0]:6.2f}, {left_joints[1]:6.2f}, ...] | "
              f"R: [{right_joints[0]:6.2f}, {right_joints[1]:6.2f}, ...]   ",
              end='', flush=True)

    def publish_teleop_increment(self, delta: np.ndarray):
        """Publish joint increment as teleop command."""
        # Split delta into left and right arms
        left_delta = delta[0:7]
        right_delta = delta[7:14]

        # Create and publish teleop command
        msg = self.create_teleop_command(
            arm_selector=self.BOTH_ARMS,
            command_type=self.JOINT_POSITION_INCREMENT,
            left_joint_delta=left_delta,
            right_joint_delta=right_delta,
            allow_safety_violation=False
        )

        self.teleop_pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='CSV joint space teleop')
    parser.add_argument('--csv-file', type=str, required=True,
                        help='Path to CSV file with joint trajectory')
    parser.add_argument('--loop', action='store_true',
                        help='Loop the trajectory')
    parser.add_argument('--rate', type=float, default=100.0,
                        help='Publishing rate (Hz)')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        node = CSVJointTeleop(
            csv_file=parser_args.csv_file,
            loop=parser_args.loop,
            publish_rate=parser_args.rate
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
        print("CSV teleop stopped.")


if __name__ == '__main__':
    main()
