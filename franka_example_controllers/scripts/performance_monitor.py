#!/usr/bin/env python3
"""
DualArmMprcController性能监控脚本

功能：
- 监控控制频率
- 记录关节命令数据
- 计算跟踪误差
- 对比不同模式性能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import time
from datetime import datetime
import numpy as np
import sys


class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # 订阅关节命令话题
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/dual_joint_impedance/joints_desired',
            self.joint_callback,
            10
        )

        # 数据存储
        self.joint_data = []
        self.timestamps = []
        self.start_time = None
        self.message_count = 0

        # CSV文件设置
        self.csv_filename = f"performance_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        # 性能统计
        self.frequency = 0.0
        self.last_message_time = None

        self.get_logger().info(f'Performance Monitor Started')
        self.get_logger().info(f'Data will be saved to: {self.csv_filename}')

        # 创建CSV文件并写入表头
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'freq_hz'] +
                           [f'joint_{i}' for i in range(14)])

    def joint_callback(self, msg):
        current_time = time.time()

        # 初始化开始时间
        if self.start_time is None:
            self.start_time = current_time
            self.last_message_time = current_time

        # 计算频率
        time_diff = current_time - self.last_message_time
        if time_diff > 0:
            instant_freq = 1.0 / time_diff
            # 移动平均滤波
            self.frequency = 0.9 * self.frequency + 0.1 * instant_freq

        self.last_message_time = current_time

        # 存储数据
        self.message_count += 1
        elapsed_time = current_time - self.start_time

        timestamp = f"{elapsed_time:.6f}"

        # 写入CSV
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            row = [timestamp, f"{self.frequency:.2f}"] + \
                  [f"{val:.6f}" for val in msg.data]
            writer.writerow(row)

        # 存储到内存（用于实时统计）
        self.timestamps.append(elapsed_time)
        self.joint_data.append(list(msg.data))

        # 每100条消息输出统计信息
        if self.message_count % 100 == 0:
            self.print_statistics()

    def print_statistics(self):
        if len(self.joint_data) < 2:
            return

        # 计算统计信息
        joint_array = np.array(self.joint_data)

        # 关节位置范围
        joint_min = joint_array.min(axis=0)
        joint_max = joint_array.max(axis=0)
        joint_range = joint_max - joint_min

        # 关节速度（简单差分）
        if len(joint_array) > 1:
            joint_diff = np.diff(joint_array, axis=0)
            joint_vel_avg = np.mean(np.abs(joint_diff), axis=0)
            joint_vel_max = np.max(np.abs(joint_diff), axis=0)
        else:
            joint_vel_avg = np.zeros(14)
            joint_vel_max = np.zeros(14)

        self.get_logger().info(f'═══════════════════════════════════════')
        self.get_logger().info(f'Performance Statistics (Messages: {self.message_count})')
        self.get_logger().info(f'Control Frequency: {self.frequency:.2f} Hz')
        self.get_logger().info(f'Elapsed Time: {self.timestamps[-1]:.2f} s')
        self.get_logger().info(f'─── Joint Motion Range (rad) ───')
        for i in range(14):
            arm = "Left" if i < 7 else "Right"
            joint = i % 7
            self.get_logger().info(
                f'  {arm} J{joint}: {joint_range[i]:.4f} '
                f'[{joint_min[i]:.4f}, {joint_max[i]:.4f}]'
            )
        self.get_logger().info(f'─── Joint Velocity (rad/step) ───')
        for i in range(14):
            arm = "Left" if i < 7 else "Right"
            joint = i % 7
            self.get_logger().info(
                f'  {arm} J{joint}: avg={joint_vel_avg[i]:.6f}, '
                f'max={joint_vel_max[i]:.6f}'
            )
        self.get_logger().info(f'═══════════════════════════════════════')

    def save_summary(self):
        """保存最终统计摘要"""
        if len(self.joint_data) == 0:
            self.get_logger().warn('No data collected')
            return

        summary_filename = f"performance_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

        with open(summary_filename, 'w') as f:
            f.write(f"Performance Summary\n")
            f.write(f"{'='*50}\n")
            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total Messages: {self.message_count}\n")
            f.write(f"Control Frequency: {self.frequency:.2f} Hz\n")
            f.write(f"Duration: {self.timestamps[-1]:.2f} s\n")
            f.write(f"CSV File: {self.csv_filename}\n")
            f.write(f"\n")

            if len(self.joint_data) > 1:
                joint_array = np.array(self.joint_data)

                # 关节统计
                f.write(f"Joint Statistics:\n")
                f.write(f"{'-'*50}\n")
                for i in range(14):
                    arm = "Left" if i < 7 else "Right"
                    joint = i % 7
                    joint_data = joint_array[:, i]

                    f.write(f"{arm} Arm Joint {joint}:\n")
                    f.write(f"  Min: {np.min(joint_data):.6f} rad\n")
                    f.write(f"  Max: {np.max(joint_data):.6f} rad\n")
                    f.write(f"  Mean: {np.mean(joint_data):.6f} rad\n")
                    f.write(f"  Std: {np.std(joint_data):.6f} rad\n")
                    f.write(f"  Range: {np.ptp(joint_data):.6f} rad\n")
                    f.write(f"\n")

        self.get_logger().info(f'Summary saved to: {summary_filename}')
        return summary_filename


def main(args=None):
    rclpy.init(args=args)

    performance_monitor = PerformanceMonitor()

    try:
        rclpy.spin(performance_monitor)
    except KeyboardInterrupt:
        performance_monitor.get_logger().info('Interrupted by user')

        # 保存最终统计
        performance_monitor.save_summary()
    finally:
        performance_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
