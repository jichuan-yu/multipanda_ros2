#!/usr/bin/env python3
"""
Realtime white-window display of dual-arm EE pose as 6-tuples:
  L: (x, y, z, roll, pitch, yaw)
  R: (x, y, z, roll, pitch, yaw)

Data source: /ee_pose (std_msgs/Float64MultiArray)
Format: [Lx, Ly, Lz, Lqx, Lqy, Lqz, Lqw, Rx, Ry, Rz, Rqx, Rqy, Rqz, Rqw]
"""

import math
import threading
import tkinter as tk
from typing import Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def quaternion_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class EEPoseWindow(Node):
    def __init__(self, root: tk.Tk) -> None:
        super().__init__('ee_pose_six_tuple_viewer')
        self.root = root
        self.lock = threading.Lock()
        self.latest_left = 'L: waiting for /ee_pose ...'
        self.latest_right = 'R: waiting for /ee_pose ...'

        self.root.title('EE Pose Viewer')
        self.root.configure(bg='white')
        self.root.geometry('1040x220')
        self.root.resizable(False, False)

        title = tk.Label(
            root,
            text='Dual-arm EE Pose',
            bg='white',
            fg='black',
            font=('DejaVu Sans Mono', 18, 'bold')
        )
        title.pack(anchor='w', padx=18, pady=(14, 6))

        self.left_label = tk.Label(
            root,
            text=self.latest_left,
            bg='white',
            fg='black',
            font=('DejaVu Sans Mono', 16),
            justify='left',
            anchor='w'
        )
        self.left_label.pack(anchor='w', padx=18, pady=6)

        self.right_label = tk.Label(
            root,
            text=self.latest_right,
            bg='white',
            fg='black',
            font=('DejaVu Sans Mono', 16),
            justify='left',
            anchor='w'
        )
        self.right_label.pack(anchor='w', padx=18, pady=6)

        hint = tk.Label(
            root,
            text='Format: (x, y, z, roll, pitch, yaw)  |  angle unit: rad',
            bg='white',
            fg='black',
            font=('DejaVu Sans Mono', 11)
        )
        hint.pack(anchor='w', padx=18, pady=(10, 0))

        self.create_subscription(Float64MultiArray, '/ee_pose', self.callback, 10)

    def callback(self, msg: Float64MultiArray) -> None:
        d = msg.data
        if len(d) < 14:
            return

        lroll, lpitch, lyaw = quaternion_to_rpy(d[3], d[4], d[5], d[6])
        rroll, rpitch, ryaw = quaternion_to_rpy(d[10], d[11], d[12], d[13])

        left = f"L: ({d[0]:+.4f}, {d[1]:+.4f}, {d[2]:+.4f}, {lroll:+.4f}, {lpitch:+.4f}, {lyaw:+.4f})"
        right = f"R: ({d[7]:+.4f}, {d[8]:+.4f}, {d[9]:+.4f}, {rroll:+.4f}, {rpitch:+.4f}, {ryaw:+.4f})"

        with self.lock:
            self.latest_left = left
            self.latest_right = right

    def update_ui(self) -> None:
        with self.lock:
            self.left_label.config(text=self.latest_left)
            self.right_label.config(text=self.latest_right)

        self.root.after(50, self.update_ui)


def spin_ros(node: Node) -> None:
    rclpy.spin(node)


def main() -> None:
    rclpy.init()
    root = tk.Tk()
    node = EEPoseWindow(root)

    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    def on_close() -> None:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    node.update_ui()
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
