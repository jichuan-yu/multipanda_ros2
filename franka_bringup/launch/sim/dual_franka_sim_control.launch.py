#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_id_1_parameter_name = 'arm_id_1'
    arm_id_2_parameter_name = 'arm_id_2'
    initial_positions_1_parameter_name = 'initial_positions_1'
    initial_positions_2_parameter_name = 'initial_positions_2'
    use_rviz_parameter_name = 'use_rviz'
    controller_name_parameter_name = 'controller_name'
    # Fixed variables, modify this in dual_franka_sim.launch.py
    load_gripper = True # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.

    arm_id_1 = LaunchConfiguration(arm_id_1_parameter_name)
    arm_id_2 = LaunchConfiguration(arm_id_2_parameter_name)
    initial_positions_1 = LaunchConfiguration(initial_positions_1_parameter_name)
    initial_positions_2 = LaunchConfiguration(initial_positions_2_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'sim', 'dual_franka_sim.launch.py']
                )
            ]
        ),
        launch_arguments={
            arm_id_1_parameter_name: arm_id_1,
            arm_id_2_parameter_name: arm_id_2,
            initial_positions_1_parameter_name: initial_positions_1,
            initial_positions_2_parameter_name: initial_positions_2,
            use_rviz_parameter_name: use_rviz,
        }.items(),
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_name],
        output='screen',
    )

    gripper_bridge_1 = None
    gripper_bridge_2 = None
    if load_gripper:
        gripper_bridge_1 = Node(
            package='franka_example_controllers',
            executable='gripper_action_bridge',
            name=[arm_id_1, '_gripper_action_bridge'],
            output='screen',
            parameters=[{'arm_id': arm_id_1}],
        )

        gripper_bridge_2 = Node(
            package='franka_example_controllers',
            executable='gripper_action_bridge',
            name=[arm_id_2, '_gripper_action_bridge'],
            output='screen',
            parameters=[{'arm_id': arm_id_2}],
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            arm_id_1_parameter_name,
            default_value='mj_left',
            description='Unique arm ID of robot 1.'
        ),
        DeclareLaunchArgument(
            arm_id_2_parameter_name,
            default_value='mj_right',
            description='Unique arm ID of robot 2.'
        ),
        DeclareLaunchArgument(
            initial_positions_1_parameter_name,
            default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of robot 1.'
        ),
        DeclareLaunchArgument(
            initial_positions_2_parameter_name,
            default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of robot 2.'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'
        ),
        DeclareLaunchArgument(
            controller_name_parameter_name,
            default_value='dual_joint_impedance_controller',
            description='Controller name to spawn after base bringup.'
        ),
        base_launch,
        controller_spawner,
    ] + ([gripper_bridge_1] if gripper_bridge_1 is not None else []) + ([gripper_bridge_2] if gripper_bridge_2 is not None else [])
    )
