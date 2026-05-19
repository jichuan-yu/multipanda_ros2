from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_id_parameter_name = 'arm_id'
    initial_positions_parameter_name = 'initial_positions'
    use_rviz_parameter_name = 'use_rviz'
    controller_name_parameter_name = 'controller_name'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    initial_positions = LaunchConfiguration(initial_positions_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'sim', 'franka_sim.launch.py']
                )
            ]
        ),
        launch_arguments={
            arm_id_parameter_name: arm_id,
            initial_positions_parameter_name: initial_positions,
            use_rviz_parameter_name: use_rviz,
        }.items(),
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_name, '-c', '/controller_manager'],
        output='screen',
    )

    gripper_bridge = Node(
        package='franka_example_controllers',
        executable='gripper_action_bridge',
        name='gripper_action_bridge',
        output='screen',
        parameters=[{'arm_id': arm_id}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value='panda',
                description='The name of the robot. Defaults to panda.',
            ),
            DeclareLaunchArgument(
                initial_positions_parameter_name,
                default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
                description='Initial joint positions of the robot in simulation.',
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value='false',
                description='Visualize the robot in Rviz',
            ),
            DeclareLaunchArgument(
                controller_name_parameter_name,
                default_value='cartesian_impedance_controller',
                description='Controller name to spawn after base bringup.',
            ),
            sim_launch,
            controller_spawner,
            gripper_bridge,
        ]
    )