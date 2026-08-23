#!/usr/bin/env python3
"""
极简遥操作指令回放脚本
从 CSV 文件读取指令，发布到 /dualarm_teleop_cmd 话题
无任何处理，直接回放原始数据
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import argparse
import time


class TeleopReplay(Node):
    def __init__(self, csv_file, rate_hz=100):
        super().__init__('teleop_replay')

        # 设置使用仿真时间
        self.set_parameters([
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        # 发布遥操作指令
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/dualarm_teleop_cmd',
            10
        )

        # 读取 CSV 文件
        self.commands = []
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)  # 跳过表头
            for row in reader:
                # 转换为 float
                self.commands.append([float(x) for x in row])

        self.total_commands = len(self.commands)
        self.get_logger().info(f'加载了 {self.total_commands} 条指令')

        # 回放参数
        self.rate_hz = rate_hz
        self.current_index = 0
        self.loop = False
        self.is_running = False

        print(f'回放准备完成: {self.total_commands} 条指令 @ {rate_hz} Hz')
        print('按 Enter 开始回放...')

    def start_replay(self, loop=False):
        """开始回放"""
        self.loop = loop
        self.current_index = 0
        self.is_running = True

    def stop_replay(self):
        """停止回放"""
        self.is_running = False

    def replay_step(self):
        """单步回放，在主循环中调用"""
        if not self.is_running:
            return False

        if self.current_index < self.total_commands:
            msg = Float64MultiArray()
            msg.data = self.commands[self.current_index]
            self.publisher.publish(msg)

            self.current_index += 1

            if self.current_index % 10 == 0:
                print(f'\r回放进度: {self.current_index}/{self.total_commands}',
                      end='', flush=True)
            return True
        else:
            # 回放结束
            if self.loop:
                self.current_index = 0
                print('\n循环回放...')
                return True
            else:
                self.is_running = False
                print(f'\n回放完成！共发送 {self.current_index} 条指令')
                return False


def main():
    parser = argparse.ArgumentParser(description='回放遥操作指令CSV')
    parser.add_argument('csv_file', type=str, help='CSV文件路径')
    parser.add_argument('--rate', type=float, default=100,
                        help='回放频率 (Hz, 默认: 100)')
    parser.add_argument('--loop', action='store_true',
                        help='循环回放')
    args = parser.parse_args()

    rclpy.init()
    replay = TeleopReplay(args.csv_file, args.rate)

    # 等待用户按 Enter 开始
    input()

    try:
        replay.start_replay(loop=args.loop)

        # 主回放循环
        period = 1.0 / args.rate
        while replay.is_running:
            start_time = time.time()

            # 发布一条指令
            replay.replay_step()

            # 精确控制频率
            elapsed = time.time() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print('\n回放中断')
    finally:
        replay.stop_replay()
        replay.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
