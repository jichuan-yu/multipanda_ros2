#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class DualJointCommander(Node):
    def __init__(self):
        super().__init__('dual_joint_commander')
        # Setup using simulation time
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/dual_joint_impedance/joints_desired', 
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback) # 100Hz
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Panda Initial Home Position for both arms
        # Changed 0.0 to 0.001 to bypass any empty checks
        self.initial_q_left = [0.001, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        self.initial_q_right = [0.001, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        
        self.get_logger().info('Dual joint commander initialized, waiting to send commands...')

    def timer_callback(self):
        msg = Float64MultiArray()
        # Combine both arms into a 14-element array
        msg.data = list(self.initial_q_left) + list(self.initial_q_right)
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        t = current_time - self.start_time
        
        # Left arm motion (sine wave)
        delta_angle_left = (math.pi / 8.0) * (1 - math.cos(math.pi / 2.5 * t))
        
        # Right arm motion (cosine wave, different phase)
        delta_angle_right = (math.pi / 8.0) * (1 - math.sin(math.pi / 2.5 * t))
        
        # Add delta to joint 4 and 5 of Left Arm (indices 3 and 4)
        msg.data[3] += delta_angle_left
        msg.data[4] += delta_angle_left
        
        # Add delta to joint 4 and 5 of Right Arm (indices 10 and 11)
        msg.data[7 + 3] += delta_angle_right
        msg.data[7 + 4] += delta_angle_right

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    commander = DualJointCommander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
