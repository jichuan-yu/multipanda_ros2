#!/usr/bin/env python3
"""
CSV trajectory playback for dual Panda robot in Cartesian/task space.

CSV Format:
    timestamp,x_left,y_left,z_left,qx_left,qy_left,qz_left,qw_left,x_right,...,qw_right

Usage:
    source ~/myenv/bin/activate
    source /path/to/dual_panda_ws/install/setup.bash
    python src/multipanda_ros2/teleop/csv_teleop_cartesian.py --csv-file trajectory.csv
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
from .utils.transformations import pose_from_msg, quaternion_multiply, normalize_quaternion


class CSVCartesianTeleop(BaseTeleopNode):
    """CSV playback for Cartesian space control."""

    def __init__(self, csv_file: str, loop: bool = False, publish_rate: float = 100.0):
        """
        Initialize CSV Cartesian teleop node.

        Args:
            csv_file: Path to CSV file with pose trajectories
            loop: Whether to loop the trajectory
            publish_rate: Publishing rate in Hz
        """
        super().__init__('csv_cartesian_teleop', publish_rate=publish_rate)

        self.csv_file = csv_file
        self.loop = loop
        self.joint_states_received = False

        # Load trajectory
        self.trajectory = self.load_csv(csv_file)
        self.current_index = 0
        self.start_time = None

        # Previous poses for delta calculation
        self.prev_left_pose = None
        self.prev_right_pose = None

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
CSV Task Space Playback
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
        Load pose trajectory from CSV file.

        Args:
            csv_file: Path to CSV file

        Returns:
            List of (timestamp, left_pos, left_quat, right_pos, right_quat) tuples
        """
        trajectory = []
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)  # Skip header

                for row in reader:
                    if len(row) >= 16:  # timestamp + 7 left (pos+quat) + 8 right
                        timestamp = float(row[0])
                        left_pos = np.array([float(x) for x in row[1:4]])
                        left_quat = normalize_quaternion(np.array([float(x) for x in row[4:8]]))
                        right_pos = np.array([float(x) for x in row[8:11]])
                        right_quat = normalize_quaternion(np.array([float(x) for x in row[11:15]]))
                        trajectory.append((timestamp, left_pos, left_quat, right_pos, right_quat))

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
            # Initialize previous poses
            _, left_pos, left_quat, right_pos, right_quat = self.trajectory[0]
            self.prev_left_pose = (left_pos, left_quat)
            self.prev_right_pose = (right_pos, right_quat)
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
                _, left_pos, left_quat, right_pos, right_quat = self.trajectory[0]
                self.prev_left_pose = (left_pos, left_quat)
                self.prev_right_pose = (right_pos, right_quat)
                return
            else:
                # Trajectory finished
                print(f"\nTrajectory playback complete.", flush=True)
                self.control_timer.cancel()
                return

        # Get target pose
        timestamp, left_pos, left_quat, right_pos, right_quat = self.trajectory[self.current_index]

        # Calculate deltas
        left_pos_delta, left_quat_delta = self.calculate_pose_delta(
            self.prev_left_pose, (left_pos, left_quat)
        )
        right_pos_delta, right_quat_delta = self.calculate_pose_delta(
            self.prev_right_pose, (right_pos, right_quat)
        )

        # Update previous poses
        self.prev_left_pose = (left_pos.copy(), left_quat.copy())
        self.prev_right_pose = (right_pos.copy(), right_quat.copy())

        # Publish command
        self.publish_command(left_pos_delta, left_quat_delta, right_pos_delta, right_quat_delta)

        # Print progress
        progress = (self.current_index / len(self.trajectory)) * 100
        print(f"\rProgress: {progress:.1f}% | "
              f"L: [{left_pos[0]:6.3f}, {left_pos[1]:6.3f}, {left_pos[2]:6.3f}] | "
              f"R: [{right_pos[0]:6.3f}, {right_pos[1]:6.3f}, {right_pos[2]:6.3f}]   ",
              end='', flush=True)

    def calculate_pose_delta(self, prev_pose, curr_pose):
        """
        Calculate incremental pose change.

        Args:
            prev_pose: (position, quaternion) of previous pose
            curr_pose: (position, quaternion) of current pose

        Returns:
            (position_delta, quaternion_delta)
        """
        prev_pos, prev_quat = prev_pose
        curr_pos, curr_quat = curr_pose

        # Position delta
        pos_delta = curr_pos - prev_pos

        # Rotation delta: delta_quat = curr * prev^(-1)
        # For quaternions: inverse = [x, y, z, -w] if we want [x, y, z, w] format
        # Actually quaternion inverse is [-x, -y, -z, w] for unit quaternions
        prev_quat_inv = np.array([-prev_quat[0], -prev_quat[1], -prev_quat[2], prev_quat[3]])
        quat_delta = quaternion_multiply(curr_quat, prev_quat_inv)

        # For small rotations, quaternion is approximately [rx/2, ry/2, rz/2, 1]
        # Extract rotation from this
        return pos_delta, quat_delta

    def publish_command(
        self,
        left_pos_delta: np.ndarray,
        left_quat_delta: np.ndarray,
        right_pos_delta: np.ndarray,
        right_quat_delta: np.ndarray
    ):
        """
        Publish Cartesian increment command.

        Args:
            left_pos_delta: 3-element left arm position increment
            left_quat_delta: 4-element left arm quaternion increment
            right_pos_delta: 3-element right arm position increment
            right_quat_delta: 4-element right arm quaternion increment
        """
        # Clamp to safety limits
        left_pos_delta = np.clip(left_pos_delta, -self.max_position_increment, self.max_position_increment)
        right_pos_delta = np.clip(right_pos_delta, -self.max_position_increment, self.max_position_increment)

        # Create Pose messages
        from .utils.transformations import pose_to_msg
        left_pose_msg = pose_to_msg(left_pos_delta, left_quat_delta)
        right_pose_msg = pose_to_msg(right_pos_delta, right_quat_delta)

        msg = self.create_teleop_command(
            arm_selector=self.BOTH_ARMS,
            command_type=self.TASK_SPACE_INCREMENT,
            left_ee_delta=left_pose_msg,
            right_ee_delta=right_pose_msg
        )

        self.teleop_pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='CSV Cartesian space teleop')
    parser.add_argument('--csv-file', type=str, required=True,
                        help='Path to CSV file with pose trajectory')
    parser.add_argument('--loop', action='store_true',
                        help='Loop the trajectory')
    parser.add_argument('--rate', type=float, default=100.0,
                        help='Publishing rate (Hz)')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        node = CSVCartesianTeleop(
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
