#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.bridge = CvBridge()
        
        # Subscribe to the three camera topics
        self.sub_left = message_filters.Subscriber(self, Image, '/mujoco_server/cameras/left_arm_cam/rgb/image_raw')
        self.sub_fixed = message_filters.Subscriber(self, Image, '/mujoco_server/cameras/fixed_cam/rgb/image_raw')
        self.sub_right = message_filters.Subscriber(self, Image, '/mujoco_server/cameras/right_arm_cam/rgb/image_raw')

        # Use an ApproximateTimeSynchronizer to sync messages from all three cameras
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_fixed, self.sub_right], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info('Camera Viewer Node has been started, waiting for images...')

    def sync_callback(self, msg_left, msg_fixed, msg_right):
        try:
            # Convert ROS Image messages to OpenCV images
            cv_img_left = self.bridge.imgmsg_to_cv2(msg_left, "bgr8")
            cv_img_fixed = self.bridge.imgmsg_to_cv2(msg_fixed, "bgr8")
            cv_img_right = self.bridge.imgmsg_to_cv2(msg_right, "bgr8")
            
            # Make sure they have the same height for hstack
            # Assuming they are the same size, but let's resize to fixed height if needed
            target_height = 480
            def resize_img(img):
                h, w = img.shape[:2]
                new_w = int((target_height / h) * w)
                return cv2.resize(img, (new_w, target_height))
            
            img_l = resize_img(cv_img_left)
            img_f = resize_img(cv_img_fixed)
            img_r = resize_img(cv_img_right)
            
            # Add text labels
            cv2.putText(img_l, "Left Arm Cam", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(img_f, "Fixed Global Cam", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(img_r, "Right Arm Cam", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Concatenate images horizontally
            combined_img = np.hstack((img_l, img_f, img_r))
            
            # Display the image
            cv2.imshow("MuJoCo Multi-Camera Monitor", combined_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing images: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
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
