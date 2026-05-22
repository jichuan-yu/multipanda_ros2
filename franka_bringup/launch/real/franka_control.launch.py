# Copyright (c) 2021 Franka Emika GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    controller_name_parameter_name = 'controller_name'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'real', 'franka.launch.py']
                )
            ]
        ),
        launch_arguments={
            robot_ip_parameter_name: robot_ip,
            load_gripper_parameter_name: load_gripper,
            use_fake_hardware_parameter_name: use_fake_hardware,
            fake_sensor_commands_parameter_name: fake_sensor_commands,
            use_rviz_parameter_name: use_rviz,
        }.items(),
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_name],
        output='screen',
    )

    gripper_bridge = Node(
        package='franka_example_controllers',
        executable='gripper_action_bridge',
        name='gripper_action_bridge',
        output='screen',
        parameters=[{'arm_id': 'panda', 'use_sim': False}],
        condition=IfCondition(load_gripper),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                description='Hostname or IP address of the robot.',
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value='false',
                description='Visualize the robot in Rviz',
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value='false',
                description='Use fake hardware',
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value='false',
                description="Fake sensor commands. Only valid when '{}' is true".format(
                    use_fake_hardware_parameter_name
                ),
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value='false',
                description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                'without an end-effector.',
            ),
            DeclareLaunchArgument(
                controller_name_parameter_name,
                default_value='cartesian_impedance_controller',
                description='Controller name to spawn after base bringup.',
            ),
            base_launch,
            gripper_bridge,
            controller_spawner,
        ]
    )
