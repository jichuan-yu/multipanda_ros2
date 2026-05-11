#!/usr/bin/env python3
"""
CSV trajectory playback for dual Panda robot in joint space.

CSV Format:
    timestamp,q1_left,q2_left,q3_left,q4_left,q5_left,q6_left,q7_left,q1_right,...,q7_right

Usage:
    source ~/myenv/bin/activate
    source /path/to/dual_panda_ws/install/setup.bash
    python src/multipanda_ros2/teleop/csv_teleop_joint.py --csv-file trajectory.csv
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import numpy as np
import argparse
import time
import csv

from .base_teleop import BaseTeleopNode


class CSVJointTeleop(BaseTeleopNode):
    """CSV playback for joint space control."""

    def __init__(self, csv_file: str, loop: bool = False, publish_rate: float = 100.0):
        """
        Initialize CSV joint teleop node.

        Args:
            csv_file: Path to CSV file with joint trajectories
            loop: Whether to loop the trajectory
            publish_rate: Publishing rate in Hz
        """
        super().__init__('csv_joint_teleop', publish_rate=publish_rate)

        self.csv_file = csv_file
        self.loop = loop
        self.joint_states_received = False

        # Load trajectory
        self.trajectory = self.load_csv(csv_file)
        self.current_index = 0
        self.start_time = None

        # Previous position for delta calculation
        self.prev_left = None
        self.prev_right = None

        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / publish_rate,
            self.control_callback
        )

        self.get_logger().info(f'Loaded {len(self.trajectory)} points from {csv_file}')
        self.print_usage()

    def print_usage(self):
        msg = f"""
========================================
CSV Joint Space Playback
========================================
File: {self.csv_file}
Points: {len(self.trajectory)}
Loop: {self.loop}
========================================
Playing trajectory...
"""
        print(msg, flush=True)

    def load_csv(self, csv_file: str) -> list:
        """
        Load joint trajectory from CSV file.

        Args:
            csv_file: Path to CSV file

        Returns:
            List of (timestamp, left_joints, right_joints) tuples
        """
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
                        trajectory.append((timestamp, left_joints, right_joints))

            return trajectory

        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {e}')
            raise

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
                print(f"Joint states received. Starting playback...", flush=True)

        except Exception as e:
            pass

    def control_callback(self):
        """Main control loop - publishes trajectory increments."""
        if not self.trajectory or not self.joint_states_received:
            return

        # Initialize start time
        if self.start_time is None:
            self.start_time = time.time()
            # Initialize previous positions
            _, self.prev_left, self.prev_right = self.trajectory[0]
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
                _, self.prev_left, self.prev_right = self.trajectory[0]
                return
            else:
                # Trajectory finished
                print(f"\nTrajectory playback complete.", flush=True)
                self.control_timer.cancel()
                return

        # Get target position
        timestamp, target_left, target_right = self.trajectory[self.current_index]

        # Calculate deltas
        left_delta = target_left - self.prev_left
        right_delta = target_right - self.prev_right

        # Update previous positions
        self.prev_left = target_left.copy()
        self.prev_right = target_right.copy()

        # Publish command
        self.publish_command(left_delta, right_delta)

        # Print progress
        progress = (self.current_index / len(self.trajectory)) * 100
        print(f"\rProgress: {progress:.1f}% | "
              f"L: [{target_left[0]:6.2f}, {target_left[1]:6.2f}, ...] | "
              f"R: [{target_right[0]:6.2f}, {target_right[1]:6.2f}, ...]   ",
              end='', flush=True)

    def publish_command(self, left_delta: np.ndarray, right_delta: np.ndarray):
        """
        Publish joint increment command.

        Args:
            left_delta: 7-element left arm joint increment
            right_delta: 7-element right arm joint increment
        """
        # Clamp to safety limits
        left_delta = self.clamp_joint_delta(left_delta)
        right_delta = self.clamp_joint_delta(right_delta)

        msg = self.create_teleop_command(
            arm_selector=self.BOTH_ARMS,
            command_type=self.JOINT_POSITION_INCREMENT,
            left_joint_delta=left_delta,
            right_joint_delta=right_delta
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
        rclpy.shutdown()
        print("CSV teleop stopped.")


if __name__ == '__main__':
    main()
