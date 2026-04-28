#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteCommand
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    启动师兄控制器，加载正确的MuJoCo匹配参数
    """

    # 参数文件路径
    config_file = os.path.join(
        get_package_share_directory('franka_example_controllers'),
        'config',
        'senior_controller_params.yaml'
    )

    return LaunchDescription([
        # 师兄控制器节点
        Node(
            package='dual_arm_reactive_control',
            executable='dual_arm_mprc_node',
            name='dual_arm_mprc_node',
            output='screen',
            parameters=[config_file],  # 加载参数文件
            emulate_tty=True,  # 保持终端输出
        )
    ])
