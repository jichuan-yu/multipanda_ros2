#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        # Setup using simulation time
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/joint_impedance/joints_desired', 
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback) # 100Hz
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Panda Initial Home Position (Change 0.0 to 0.001 to bypass C++ condition bug)
        self.initial_q = [0.001, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        self.get_logger().info('Joint commander initialized, waiting to send commands...')

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = list(self.initial_q)
        
        # Create a tiny sine wave motion on joint 4 & 5 to see it moving
        current_time = self.get_clock().now().nanoseconds / 1e9
        t = current_time - self.start_time
        delta_angle = (math.pi / 8.0) * (1 - math.cos(math.pi / 2.5 * t))
        
        # Add delta to joint 4 and 5
        msg.data[3] += delta_angle
        msg.data[4] += delta_angle

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    commander = JointCommander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
