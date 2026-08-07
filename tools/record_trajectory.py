#!/usr/bin/env python3
"""
record_trajectory.py

Dual-arm trajectory recording script.
Records joint positions every 1 second when the position has changed.

Output format: CSV with 14 columns
    [q1_left, q2_left, q3_left, q4_left, q5_left, q6_left, q7_left,
     q1_right, q2_right, q3_right, q4_right, q5_right, q6_right, q7_right]

Usage:
    python3 record_trajectory.py output.csv
    Press Ctrl-C to stop recording and save the file.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import sys
import time
import threading
import numpy as np


class TrajectoryRecorder(Node):
    def __init__(self, output_file):
        super().__init__('trajectory_recorder')
        self.set_parameters([rclpy.Parameter('use_sim_time',
                                             rclpy.Parameter.Type.BOOL, True)])

        self.output_file = output_file
        self.trajectory = []  # List of 14-element arrays
        self.last_position = None
        self.position_threshold = 0.001  # rad, minimum change to record

        # Joint names for left and right arms
        self.left_joint_names = [f'mj_left_joint{i}' for i in range(1, 8)]
        self.right_joint_names = [f'mj_right_joint{i}' for i in range(1, 8)]

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Current joint positions
        self.current_positions = None
        self.positions_received = False

        # Recording state
        self.recording = True
        self.start_time = time.time()

        # Timer for recording every 0.1 second
        self.record_timer = self.create_timer(0.1, self.record_callback)

        self.get_logger().info(f'Trajectory Recorder initialized')
        self.get_logger().info(f'Output file: {output_file}')
        self.get_logger().info(f'Waiting for joint states...')
        self.get_logger().info(f'Press Ctrl-C to stop and save')

    def joint_state_callback(self, msg):
        """Store current joint positions."""
        try:
            left_joints = []
            right_joints = []

            for name in self.left_joint_names:
                idx = msg.name.index(name)
                left_joints.append(msg.position[idx])

            for name in self.right_joint_names:
                idx = msg.name.index(name)
                right_joints.append(msg.position[idx])

            self.current_positions = np.array(left_joints + right_joints)
            self.positions_received = True

        except ValueError as e:
            # Joint not found in message yet
            pass

    def record_callback(self):
        """Record position every 1 second if changed."""
        if not self.positions_received:
            return

        if self.current_positions is None:
            return

        # Check if position changed significantly
        if self.last_position is None:
            changed = True
        else:
            diff = np.abs(self.current_positions - self.last_position)
            changed = np.any(diff > self.position_threshold)

        if changed:
            self.trajectory.append(self.current_positions.copy())
            self.last_position = self.current_positions.copy()
            elapsed = time.time() - self.start_time
            print(f'[+{elapsed:.1f}s] Recorded waypoint {len(self.trajectory)}: '
                  f'[{self.current_positions[0]:.3f}, ..., {self.current_positions[13]:.3f}]')

    def save_trajectory(self):
        """Save trajectory to CSV file."""
        if not self.trajectory:
            print('No trajectory points recorded.')
            return

        try:
            with open(self.output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                # Header
                header = (['q1_left', 'q2_left', 'q3_left', 'q4_left', 'q5_left', 'q6_left', 'q7_left'] +
                          ['q1_right', 'q2_right', 'q3_right', 'q4_right', 'q5_right', 'q6_right', 'q7_right'])
                writer.writerow(header)

                # Data (each waypoint is a row), format to 3 decimal places
                for point in self.trajectory:
                    writer.writerow([round(x, 3) for x in point.tolist()])

            print(f'\nTrajectory saved to {self.output_file}')
            print(f'Total waypoints: {len(self.trajectory)}')

        except Exception as e:
            print(f'Error saving trajectory: {e}')

    def stop_recording(self):
        """Stop recording and save."""
        self.recording = False
        self.save_trajectory()


def main(args=None):
    if len(sys.argv) < 2:
        print('Usage: python3 record_trajectory.py <output_file.csv>')
        print('Example: python3 record_trajectory.py trajectory.csv')
        sys.exit(1)

    output_file = sys.argv[1]

    rclpy.init(args=args)
    recorder = TrajectoryRecorder(output_file)

    # Spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok() and recorder.recording:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nStopping recording...')
    finally:
        recorder.stop_recording()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
