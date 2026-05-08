#!/usr/bin/env python3
"""
csv_pub_direct.py
Publishes trajectory data from a CSV file DIRECTLY to the Joint Impedance Controller.
This bypasses the MPRC safety controller.

Input CSV format: 14 columns of joint positions
    [q1_left, ..., q7_left, q1_right, ..., q7_right]

Output: 14-element Float64MultiArray to /dual_joint_impedance/joints_desired
    [q1_left, ..., q7_left, q1_right, ..., q7_right]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pandas as pd
import sys

class CsvPubDirectNode(Node):
    def __init__(self, csv_file):
        super().__init__('csv_pub_direct')
        self.set_parameters([rclpy.Parameter('use_sim_time',
                                             rclpy.Parameter.Type.BOOL, True)])

        # Load CSV
        try:
            self.df = pd.read_csv(csv_file)
            self.get_logger().info(f'Loaded {len(self.df)} points from {csv_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {e}')
            sys.exit(1)

        self.index = 0

        # Publisher → Directly to Joint Impedance Controller
        # This bypasses DualArmMprcController
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/dual_joint_impedance/joints_desired',
            10
        )

        # 50 Hz timer (20ms interval)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('csv_pub_direct initialized @ 50Hz (Bypassing MPRC)')

    def timer_callback(self):
        if self.index >= len(self.df):
            self.get_logger().info('Trajectory finished')
            self.timer.cancel()
            return

        # Get joint positions for current step
        row = self.df.iloc[self.index].values
        
        # Build 14-element message
        # Format: [q1_l, q2_l, q3_l, q4_l, q5_l, q6_l, q7_l, q1_r, q2_r, q3_r, q4_r, q5_r, q6_r, q7_r]
        msg = Float64MultiArray()
        msg.data = row.tolist()

        self.publisher.publish(msg)
        
        if self.index % 50 == 0:
            self.get_logger().info(f'Step {self.index}/{len(self.df)}')
            
        self.index += 1

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 csv_pub_direct.py <input_file.csv>')
        return

    csv_file = sys.argv[1]
    rclpy.init()
    node = CsvPubDirectNode(csv_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
