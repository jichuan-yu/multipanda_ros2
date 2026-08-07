#!/usr/bin/env python3
"""
Log minimum distance from MPRC controller to CSV file.
Uses pre-computed distance data from the controller (no additional calculation).

Subscribes to /min_distance topic: [d_left, d_right, safe_idx_left, safe_idx_right]

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/distance_logger.py --output test_results/test1.csv
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

    def __init__(self, output_file: str):
        super().__init__('distance_logger')

        self.output_file = Path(output_file)
        self.start_time = None
        self.row_count = 0

        # Create output directory
        self.output_file.parent.mkdir(parents=True, exist_ok=True)

        # Create single CSV file with both arms data
        self.f = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['time', 'd_left', 'd_right', 'safe_idx_left', 'safe_idx_right'])

        # Subscribe to minimum distance topic from controller
        # Format: [d_left, d_right, safe_idx_left, safe_idx_right]
        self.distance_sub = self.create_subscription(
            Float64MultiArray,
            '/min_distance',
            self.distance_callback,
            10
        )

        self.get_logger().info(f"Logging to {self.output_file}")

    def distance_callback(self, msg):
        """Process minimum distance data from controller."""
        # Get start time
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Calculate current time
        current_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        current_time = round(current_time, 3)

        # Extract data: [d_left, d_right, safe_idx_left, safe_idx_right]
        if len(msg.data) >= 4:
            d_left = msg.data[0]
            d_right = msg.data[1]
            safe_idx_left = msg.data[2]
            safe_idx_right = msg.data[3]

            # Log to CSV (3 decimal places)
            self.writer.writerow([
                current_time,
                round(d_left, 3),
                round(d_right, 3),
                round(safe_idx_left, 3),
                round(safe_idx_right, 3)
            ])
            self.row_count += 1

    def close(self):
        """Close file."""
        self.f.close()
        self.get_logger().info(f"Logged {self.row_count} rows total")


def main(args=None):
    parser = argparse.ArgumentParser(description='Log minimum distance from MPRC controller')
    parser.add_argument('--output', type=str, default='/home/xiaozy24/dual_panda_ws/test_results/distance.csv',
                        help='Output CSV file path')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)

    try:
        node = DistanceLogger(output_file=parser_args.output)
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
