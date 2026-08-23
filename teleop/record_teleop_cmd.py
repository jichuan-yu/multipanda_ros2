#!/usr/bin/env python3
"""
极简遥操作指令录制脚本
监听 /dualarm_teleop_cmd 话题，将所有指令保存到 CSV 文件
无任何处理，直接保存原始数据
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import time
from datetime import datetime


class TeleopRecorder(Node):
    def __init__(self, output_file=None):
        super().__init__('teleop_recorder')

        # 设置使用仿真时间
        self.set_parameters([
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        # 生成文件名：teleop_record_YYYYMMDD_HHMMSS.csv
        if output_file is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_file = f'teleop_record_{timestamp}.csv'

        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # 写入表头（仅用于可读性，回放时不使用）
        self.csv_writer.writerow([
            'data0', 'data1', 'data2', 'data3', 'data4', 'data5', 'data6',
            'data7', 'data8', 'data9', 'data10', 'data11', 'data12', 'data13',
            'command_type', 'arm_selector', 'allow_safety_violation'
        ])
        self.csv_file.flush()

        # 订阅遥操作指令话题
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/dualarm_teleop_cmd',
            self.callback,
            10
        )

        self.count = 0
        self.get_logger().info(f'录制开始，保存到: {output_file}')
        print(f'录音中... (按 Ctrl+C 停止录制)')

    def callback(self, msg):
        """直接保存原始数据到 CSV"""
        self.csv_writer.writerow(msg.data)
        self.csv_file.flush()
        self.count += 1

        if self.count % 10 == 0:
            print(f'\r已录制 {self.count} 条指令', end='', flush=True)

    def stop(self):
        self.csv_file.close()
        print(f'\n录制完成，共保存 {self.count} 条指令')


def main():
    rclpy.init()

    import argparse
    parser = argparse.ArgumentParser(description='录制遥操作指令到CSV')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='输出CSV文件名 (默认: teleop_record_YYYYMMDD_HHMMSS.csv)')
    args = parser.parse_args()

    recorder = TeleopRecorder(args.output)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
