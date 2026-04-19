#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rqt_gui.main import Main
import sys

def main():
    rclpy.init()
    # rqt_image_view doesn't parse standard ROS2 args through the CLI wrapper well in Humble.
    # So we bypass it.
    pass
