#!/usr/bin/env python3
"""
record_mprc_trajectory.py

Records MPRC controller desired joint positions from /dual_joint_impedance/joints_desired.
Records joint positions every 0.1 second when the position has changed.

Output format: CSV with 14 columns
    [q1_left, q2_left, q3_left, q4_left, q5_left, q6_left, q7_left,
     q1_right, q2_right, q3_right, q4_right, q5_right, q6_right, q7_right]

Usage:
    python3 record_mprc_trajectory.py output.csv
    Press Ctrl-C to stop recording and save the file.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import sys
import time
import threading
import numpy as np


class MprcTrajectoryRecorder(Node):
    def __init__(self, output_file):
        super().__init__('mprc_trajectory_recorder')
        self.set_parameters([rclpy.Parameter('use_sim_time',
                                             rclpy.Parameter.Type.BOOL, True)])

        self.output_file = output_file
        self.trajectory = []  # List of 14-element arrays
        self.last_position = None
        self.position_threshold = 0.001  # rad, minimum change to record

        # Subscribe to MPRC desired joint states
        self.joint_state_sub = self.create_subscription(
            Float64MultiArray,
            '/dual_joint_impedance/joints_desired',
            self.mprc_callback,
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

        self.get_logger().info(f'MPRC Trajectory Recorder initialized')
        self.get_logger().info(f'Output file: {output_file}')
        self.get_logger().info(f'Waiting for /dual_joint_impedance/joints_desired...')
        self.get_logger().info(f'Press Ctrl-C to stop and save')

    def mprc_callback(self, msg):
        """Store current joint positions from MPRC output."""
        if len(msg.data) == 14:
            self.current_positions = np.array(msg.data)
            self.positions_received = True

    def record_callback(self):
        """Record position every 0.1 second if changed."""
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
                # Header (same as record_trajectory.py)
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
        print('Usage: python3 record_mprc_trajectory.py <output_file.csv>')
        print('Example: python3 record_mprc_trajectory.py mprc_traj.csv')
        sys.exit(1)

    output_file = sys.argv[1]

    rclpy.init(args=args)
    recorder = MprcTrajectoryRecorder(output_file)

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
