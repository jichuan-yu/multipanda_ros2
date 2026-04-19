#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class IndependentCameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.bridge = CvBridge()
        
        # 独立订阅，放弃时间同步（消除等待带来的卡顿）
        self.sub_left = self.create_subscription(
            Image, 
            '/mujoco_server/cameras/left_arm_cam/rgb/image_raw',
            self.left_callback,
            10
        )
        self.sub_fixed = self.create_subscription(
            Image, 
            '/mujoco_server/cameras/fixed_cam/rgb/image_raw',
            self.fixed_callback,
            10
        )
        self.sub_right = self.create_subscription(
            Image, 
            '/mujoco_server/cameras/right_arm_cam/rgb/image_raw',
            self.right_callback,
            10
        )

        self.get_logger().info('Independent Camera Viewer started. Displaying separate windows...')

    def left_callback(self, msg):
        self.show_image("Left Arm Cam", msg)

    def fixed_callback(self, msg):
        self.show_image("Fixed Global Cam", msg)

    def right_callback(self, msg):
        self.show_image("Right Arm Cam", msg)

    def show_image(self, window_name, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow(window_name, cv_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image for {window_name}: {e}")

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
