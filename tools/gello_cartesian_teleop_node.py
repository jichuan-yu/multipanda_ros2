#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from gello_teleop.franka_fk import FrankaFK
from scipy.spatial.transform import Rotation
import numpy as np
import os

class GelloCartesianTeleopNode(Node):
    def __init__(self):
        super().__init__('gello_cartesian_teleop_node')
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 检查是否使用模拟模式
        self.use_mock = os.environ.get('MOCK_GELLO') == '1'
        
        if not self.use_mock:
            # 使用真实 GELLO 主臂
            try:
                from gello_teleop.gello_teleop_agent import GelloTeleopAgent
                self.gello_agent = GelloTeleopAgent()
                self.get_logger().info('Real Gello agent initialized successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize Gello agent: {e}')
                self.use_mock = True  # 失败时切换到模拟模式
        else:
            # 使用模拟 GELLO 主臂
            self.subscription = self.create_subscription(
                Float64MultiArray,
                '/gello/joint_states',
                self.joint_states_callback,
                10
            )
            self.get_logger().info('Using mock Gello (subscribing to /gello/joint_states)')
        
        # 初始化 FrankaFK
        try:
            self.franka_fk = FrankaFK()
            self.get_logger().info('FrankaFK initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize FrankaFK: {e}')
            return
        
        # 发布到多臂笛卡尔阻抗控制器
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/multi_cartesian_impedance/pose_desired',
            10
        )
        
        # 左臂固定位姿
        self.left_pose = [0.3, 0.25, 0.45, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # 从臂偏移量
        self.offset = [0.0, -0.45, 0.0]
        
        # 右臂关节角度（初始为默认姿态）
        self.right_joints = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        
        # 控制频率
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        self.get_logger().info('Gello cartesian teleop node initialized')

    def joint_states_callback(self, msg):
        try:
            # 获取模拟的 GELLO 主臂关节角度
            joint_angles = np.array(msg.data)
            if len(joint_angles) != 7:
                self.get_logger().warn(f'Expected 7 joint angles, got {len(joint_angles)}')
                return
            
            self.right_joints = joint_angles
            self.get_logger().info(f'Received mock joint angles: {joint_angles}')
            
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

    def timer_callback(self):
        try:
            if not self.use_mock:
                # 从真实 GELLO 主臂获取关节角度
                joint_angles, gripper_state = self.gello_agent.get_action()
                self.get_logger().info(f'Received real joint angles: {joint_angles}')
                self.right_joints = joint_angles
            
            # 计算末端位姿
            pos, quat = self.franka_fk.get_fk(self.right_joints)
            self.get_logger().info(f'Calculated end effector position: {pos}')
            
            # 将四元数转换为旋转矩阵
            rot_mat = Rotation.from_quat(quat).as_matrix()
            
            # 计算右臂目标位置（加上偏移）
            right_pos = [
                pos[0] + self.offset[0],
                pos[1] + self.offset[1],
                pos[2] + self.offset[2]
            ]
            
            # 构建控制命令
            command = self.left_pose.copy()
            command.extend(right_pos)
            command.extend(rot_mat.flatten().tolist())
            
            # 确保第一个值非零
            if command[0] == 0.0:
                command[0] = 0.001
                
            # 发布命令
            msg = Float64MultiArray()
            msg.data = command
            self.publisher.publish(msg)
            self.get_logger().info(f'Published cartesian command')
            
        except Exception as e:
            self.get_logger().error(f'Error in teleop callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GelloCartesianTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
