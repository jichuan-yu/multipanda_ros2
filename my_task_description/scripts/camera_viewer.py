#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class IndependentCameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.bridge = CvBridge()
        
        # 使用字典保存最新的图像帧，将“ROS接收”和“OpenCV刷新”解耦
        self.latest_frames = {
            "Left Arm Cam": None,
            "Fixed Global Cam": None,
            "Right Arm Cam": None
        }
        
        # 独立订阅，强制使用传感器级别的 Best Effort (尽力而为) QoS策略
        self.sub_left = self.create_subscription(
            Image, 
            '/mujoco_server/cameras/left_arm_cam/rgb/image_raw',
            lambda msg: self.store_frame("Left Arm Cam", msg),
            qos_profile_sensor_data
        )
        self.sub_fixed = self.create_subscription(
            Image, 
            '/mujoco_server/cameras/fixed_cam/rgb/image_raw',
            lambda msg: self.store_frame("Fixed Global Cam", msg),
            qos_profile_sensor_data
        )
        self.sub_right = self.create_subscription(
            Image, 
            '/mujoco_server/cameras/right_arm_cam/rgb/image_raw',
            lambda msg: self.store_frame("Right Arm Cam", msg),
            qos_profile_sensor_data
        )

        # 创建一个定时器，统一以指定的帧率（如 30 FPS）刷新 UI
        # 这样无论传感器发送多快，图形界面都不会被阻塞或拖垮系统
        self.render_timer = self.create_timer(1.0 / 30.0, self.render_windows)
        self.get_logger().info('Camera Viewer started. UI rendering is decoupled from ROS network at 30 FPS limit...')

    def store_frame(self, window_name, msg):
        self.latest_frames[window_name] = msg

    def render_windows(self):
        # 每秒执行 30 次，统一将已收到的最新帧绘制出来
        for window_name, msg in self.latest_frames.items():
            if msg is not None:
                try:
                    cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    cv2.imshow(window_name, cv_img)
                except Exception as e:
                    self.get_logger().error(f"Error processing image for {window_name}: {e}")
        
        # 整个程序循环只需要集中调用一次 waitKey，消除回调函数中疯狂 waitKey 带来的严重积压
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = IndependentCameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
