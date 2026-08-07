#!/usr/bin/env python3
"""
Record actual end-effector poses of dual Panda robot to CSV.

Subscribes to /ee_pose topic and records to CSV file.
Output format matches test_simple_safe.csv:
    timestamp,x_left,y_left,z_left,qx_left,qy_left,qz_left,qw_left,
    x_right,y_right,z_right,qx_right,qy_right,qz_right,qw_right

Usage:
    python3 record_ee_pose.py [output_file.csv]
    Press Ctrl-C to stop recording and save the file.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import sys
import time
import threading


class EEPoseRecorder(Node):
    def __init__(self, output_file):
        super().__init__('ee_pose_recorder')
        self.set_parameters([rclpy.Parameter('use_sim_time',
                                             rclpy.Parameter.Type.BOOL, True)])

        self.output_file = output_file
        self.rows = []
        self.start_time = time.time()
        self.recording = True
        self.pose_received = False

        # Subscribe to end-effector pose topic
        # Format: [left_x,y,z,qx,qy,qz,qw, right_x,y,z,qx,qy,qz,qw] (14 elements)
        self.ee_pose_sub = self.create_subscription(
            Float64MultiArray,
            '/ee_pose',
            self.ee_pose_callback,
            10
        )

        # Recording timer - record every 0.01s (100Hz)
        self.record_timer = self.create_timer(0.01, self.record_callback)

        self.get_logger().info(f'EE Pose Recorder initialized')
        self.get_logger().info(f'Output file: {output_file}')
        self.get_logger().info(f'Waiting for /ee_pose topic...')
        self.get_logger().info(f'Press Ctrl-C to stop and save')

        # Store latest pose data
        self.latest_pose = None

    def ee_pose_callback(self, msg):
        """Store latest end-effector pose."""
        if len(msg.data) >= 14:
            self.latest_pose = msg.data
            if not self.pose_received:
                self.pose_received = True
                self.get_logger().info('Receiving /ee_pose data!')

    def record_callback(self):
        """Record pose at 100Hz."""
        if not self.pose_received or self.latest_pose is None:
            return

        elapsed = time.time() - self.start_time

        # Extract left and right poses
        # Format: [left_x,y,z,qx,qy,qz,qw, right_x,y,z,qx,qy,qz,qw]
        row = {
            'timestamp': f'{elapsed:.3f}',
            'x_left': f'{self.latest_pose[0]:.3f}',
            'y_left': f'{self.latest_pose[1]:.3f}',
            'z_left': f'{self.latest_pose[2]:.3f}',
            'qx_left': f'{self.latest_pose[3]:.3f}',
            'qy_left': f'{self.latest_pose[4]:.3f}',
            'qz_left': f'{self.latest_pose[5]:.3f}',
            'qw_left': f'{self.latest_pose[6]:.3f}',
            'x_right': f'{self.latest_pose[7]:.3f}',
            'y_right': f'{self.latest_pose[8]:.3f}',
            'z_right': f'{self.latest_pose[9]:.3f}',
            'qx_right': f'{self.latest_pose[10]:.3f}',
            'qy_right': f'{self.latest_pose[11]:.3f}',
            'qz_right': f'{self.latest_pose[12]:.3f}',
            'qw_right': f'{self.latest_pose[13]:.3f}',
        }

        self.rows.append(row)

        # Print progress every 100 rows (approx 1 second)
        if len(self.rows) % 100 == 0:
            print(f'[{elapsed:.1f}s] Recorded {len(self.rows)} rows', flush=True)

    def save_to_csv(self):
        """Save recorded poses to CSV file."""
        if not self.rows:
            print('No data recorded.')
            return

        fieldnames = [
            'timestamp', 'x_left', 'y_left', 'z_left', 'qx_left', 'qy_left', 'qz_left', 'qw_left',
            'x_right', 'y_right', 'z_right', 'qx_right', 'qy_right', 'qz_right', 'qw_right'
        ]

        try:
            with open(self.output_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.rows)

            elapsed = time.time() - self.start_time
            print(f'\nSaved {len(self.rows)} rows to {self.output_file}')
            print(f'Duration: {elapsed:.1f}s')

        except Exception as e:
            print(f'Error saving CSV: {e}')

    def stop_recording(self):
        """Stop recording and save."""
        self.recording = False
        self.save_to_csv()


def main(args=None):
    # Parse output filename
    if len(sys.argv) >= 2:
        output_file = sys.argv[1]
    else:
        output_file = f'ee_pose_{int(time.time())}.csv'

    rclpy.init(args=args)
    recorder = EEPoseRecorder(output_file)

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
