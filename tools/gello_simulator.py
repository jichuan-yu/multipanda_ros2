#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty
import threading
import numpy as np

class GelloSimulator(Node):
    def __init__(self):
        super().__init__('gello_simulator')
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 发布模拟的 GELLO 关节角度
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/gello/joint_states',
            10
        )
        
        # 初始关节角度（默认姿态）
        self.joint_angles = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        
        # 关节调整步长
        self.step = 0.05
        
        # 控制频率
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        self.get_logger().info('Gello simulator initialized')
        self.print_usage()

    def print_usage(self):
        print("""
----------------------------------------
Gello Simulator (Keyboard Control)
----------------------------------------
Use the following keys to control the virtual Gello arm:
1/2 : Joint 1 +/-
3/4 : Joint 2 +/-
5/6 : Joint 3 +/-
7/8 : Joint 4 +/-
9/0 : Joint 5 +/-
Q/W : Joint 6 +/-
E/R : Joint 7 +/-
Space : Reset to default position
Ctrl-C : Quit
----------------------------------------
""")

    def update_joints(self, key):
        # 关节控制
        if key == '1':
            self.joint_angles[0] += self.step
        elif key == '2':
            self.joint_angles[0] -= self.step
        elif key == '3':
            self.joint_angles[1] += self.step
        elif key == '4':
            self.joint_angles[1] -= self.step
        elif key == '5':
            self.joint_angles[2] += self.step
        elif key == '6':
            self.joint_angles[2] -= self.step
        elif key == '7':
            self.joint_angles[3] += self.step
        elif key == '8':
            self.joint_angles[3] -= self.step
        elif key == '9':
            self.joint_angles[4] += self.step
        elif key == '0':
            self.joint_angles[4] -= self.step
        elif key == 'q':
            self.joint_angles[5] += self.step
        elif key == 'w':
            self.joint_angles[5] -= self.step
        elif key == 'e':
            self.joint_angles[6] += self.step
        elif key == 'r':
            self.joint_angles[6] -= self.step
        elif key == ' ':
            # 重置到默认位置
            self.joint_angles = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
            print("Reset to default position")

    def timer_callback(self):
        # 发布模拟的关节角度
        msg = Float64MultiArray()
        msg.data = self.joint_angles.tolist()
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published joint angles: {self.joint_angles}')

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = GelloSimulator()
    
    settings = termios.tcgetattr(sys.stdin)
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03': # ctrl-c
                break
            if key:
                node.update_joints(key.lower())
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
