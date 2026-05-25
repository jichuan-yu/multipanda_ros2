#!/usr/bin/env python3
"""
Log minimum distance from MPRC controller to CSV file.
Uses pre-computed distance data from the controller (no additional calculation).

Subscribes to /min_distance topic: [d_left, d_right, safe_idx_left, safe_idx_right]

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/distance_logger.py --safety on
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import argparse
import os
from pathlib import Path


class DistanceLogger(Node):
    """Log minimum distance to CSV file using controller's pre-computed data."""

    def __init__(self, safety: str, output_dir: str = '/home/xiaozy24/dual_panda_ws/test_results'):
        super().__init__('distance_logger')

        self.safety = safety
        self.output_dir = Path(output_dir)
        self.start_time = None
        self.row_count = 0

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Create CSV files for left and right arms
        self.files = {}
        self.writers = {}
        for arm in ['left', 'right']:
            filename = f"{arm}_arm_safety_{self.safety}.csv"
            filepath = self.output_dir / filename
            f = open(filepath, 'w', newline='')
            writer = csv.writer(f)
            writer.writerow(['time', 'min_distance'])
            self.files[arm] = f
            self.writers[arm] = writer

        # Subscribe to minimum distance topic from controller
        # Format: [d_left, d_right, safe_idx_left, safe_idx_right]
        self.distance_sub = self.create_subscription(
            Float64MultiArray,
            '/min_distance',
            self.distance_callback,
            10
        )

        self.get_logger().info(f"Logging to {self.output_dir}")
        self.get_logger().info(f"Safety: {safety}")

    def distance_callback(self, msg):
        """Process minimum distance data from controller."""
        # Get start time
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Calculate current time
        current_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        current_time = round(current_time, 3)

        # Extract data: [d_left, d_right, safe_idx_left, safe_idx_right]
        if len(msg.data) >= 2:
            d_left = msg.data[0]
            d_right = msg.data[1]

            # Log to CSV (3 decimal places)
            self.writers['left'].writerow([current_time, round(d_left, 3)])
            self.writers['right'].writerow([current_time, round(d_right, 3)])
            self.row_count += 2

    def close(self):
        """Close all files."""
        for f in self.files.values():
            f.close()
        self.get_logger().info(f"Logged {self.row_count} rows total")


def main(args=None):
    parser = argparse.ArgumentParser(description='Log minimum distance from MPRC controller')
    parser.add_argument('--safety', type=str, choices=['on', 'off'], required=True,
                        help='Safety status (on/off)')
    parser.add_argument('--output-dir', type=str, default='/home/xiaozy24/dual_panda_ws/test_results',
                        help='Output directory for CSV files')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        node = DistanceLogger(
            safety=parser_args.safety,
            output_dir=parser_args.output_dir
        )

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if 'node' in locals():
            node.close()
            node.destroy_node()
        rclpy.shutdown()
        print("Distance logger stopped.")


if __name__ == '__main__':
    main()
