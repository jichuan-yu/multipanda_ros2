#!/usr/bin/env python3
"""Subscribe to depth/camera_info — triggers depth rendering in mujoco_ros."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class DepthTrigger(Node):
    def __init__(self):
        super().__init__('depth_trigger')
        self.sub = self.create_subscription(CameraInfo, 'depth/camera_info', lambda m: None, 1)


def main():
    rclpy.init()
    rclpy.spin(DepthTrigger())


if __name__ == '__main__':
    main()