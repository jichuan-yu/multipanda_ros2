#!/usr/bin/env python3
"""
测试师兄控制器 - 发送安全的关节位置

直接发送已知安全的关节位置，绕过IK求解
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time


class SafeJointPositionTester(Node):
    def __init__(self):
        super().__init__('safe_joint_position_tester')

        # 发布到师兄控制器
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/dualArm_traj',
            10
        )

        # 安全的关节位置（远离关节限制的中心位置）
        # 关节4限制：[-3.07, -0.07]，当前-2.356太接近下界
        # 改为-1.5（中间位置）
        self.safe_position = [
            0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5,  # panda_1 (远离极限)
            0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5   # panda_2 (远离极限)
        ]

        self.joint_names = [
            "panda_1_joint1", "panda_1_joint2", "panda_1_joint3",
            "panda_1_joint4", "panda_1_joint5", "panda_1_joint6", "panda_1_joint7",
            "panda_2_joint1", "panda_2_joint2", "panda_2_joint3",
            "panda_2_joint4", "panda_2_joint5", "panda_2_joint6", "panda_2_joint7"
        ]

        self.timer = self.create_timer(1.0, self.publish_callback)
        self.count = 0
        self.max_count = 5  # 发送5次

        self.get_logger().info('Safe Joint Position Tester Started')
        self.get_logger().info(f'Publishing safe position: {self.safe_position}')

    def publish_callback(self):
        if self.count >= self.max_count:
            self.get_logger().info('Test completed')
            return

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = "world"
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.safe_position
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0

        traj.points.append(point)
        self.traj_pub.publish(traj)

        self.count += 1
        self.get_logger().info(f'Published safe position [{self.count}/{self.max_count}]')


def main(args=None):
    rclpy.init(args=args)
    node = SafeJointPositionTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
