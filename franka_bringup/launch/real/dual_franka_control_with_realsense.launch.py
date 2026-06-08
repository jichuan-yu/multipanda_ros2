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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _realsense_launch(
    camera_name,
    serial_no,
    depth_profile,
    depth_color_profile=None,
    rgb_color_profile=None,
    depth_exposure=None,
    rgb_exposure=None,
    condition=None,
):
    launch_arguments = {
        'camera_namespace': 'cameras',
        'camera_name': camera_name,
        'serial_no': serial_no,
        'enable_depth': 'false',
        'enable_color': 'true',
        'enable_infra1': 'false',
        'enable_infra2': 'false',
        'depth_module.depth_profile': depth_profile,
    }

    if depth_color_profile is not None:
        launch_arguments['depth_module.color_profile'] = depth_color_profile

    if rgb_color_profile is not None:
        launch_arguments['rgb_camera.color_profile'] = rgb_color_profile

    if depth_exposure is not None:
        launch_arguments['depth_module.enable_auto_exposure'] = 'false'
        launch_arguments['depth_module.exposure'] = depth_exposure

    if rgb_exposure is not None:
        launch_arguments['rgb_camera.enable_auto_exposure'] = 'false'
        launch_arguments['rgb_camera.exposure'] = rgb_exposure

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]
        ),
        launch_arguments=launch_arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    robot_ip_1_parameter_name = 'robot_ip_1'
    robot_ip_2_parameter_name = 'robot_ip_2'
    load_gripper_1_parameter_name = 'load_gripper_1'
    load_gripper_2_parameter_name = 'load_gripper_2'
    arm_id_1_parameter_name = 'arm_id_1'
    arm_id_2_parameter_name = 'arm_id_2'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    controller_name_parameter_name = 'controller_name'

    launch_realsense_parameter_name = 'launch_realsense'

    # Camera topic names.
    d435f_camera_name = 'fixed'

    d405_left_serial = "'409122274276'"
    d405_right_serial = "'352122272335'"
    d435f_serial = "'244222074410'"

    d405_left_depth_profile = '640,480,30'
    d405_left_color_profile = '640,480,30'
    d405_right_depth_profile = '640,480,30'
    d405_right_color_profile = '640,480,30'
    d435f_depth_profile = '848,480,30'
    d435f_color_profile = '848,480,30'

    robot_ip_1 = LaunchConfiguration(robot_ip_1_parameter_name)
    robot_ip_2 = LaunchConfiguration(robot_ip_2_parameter_name)
    load_gripper_1 = LaunchConfiguration(load_gripper_1_parameter_name)
    load_gripper_2 = LaunchConfiguration(load_gripper_2_parameter_name)
    arm_id_1 = LaunchConfiguration(arm_id_1_parameter_name)
    arm_id_2 = LaunchConfiguration(arm_id_2_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)

    launch_realsense = LaunchConfiguration(launch_realsense_parameter_name)
    launch_rqt_image_view = LaunchConfiguration('launch_rqt_image_view')
    apply_collision_params = LaunchConfiguration('apply_collision_params')

    # Collision behavior payload (for cartesian assembly tasks)
    collision_payload = (
        "{ "
        "lower_torque_thresholds_acceleration: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], "
        "upper_torque_thresholds_acceleration: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], "
        "lower_torque_thresholds_nominal: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], "
        "upper_torque_thresholds_nominal: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], "
        "lower_force_thresholds_acceleration: [20.0,20.0,20.0,25.0,25.0,25.0], "
        "upper_force_thresholds_acceleration: [20.0,20.0,20.0,25.0,25.0,25.0], "
        "lower_force_thresholds_nominal: [50.0,50.0,60.0,50.0,50.0,50.0], "
        "upper_force_thresholds_nominal: [50.0,50.0,60.0,50.0,50.0,50.0] "
        "}"
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'real', 'dual_franka.launch.py']
                )
            ]
        ),
        launch_arguments={
            robot_ip_1_parameter_name: robot_ip_1,
            robot_ip_2_parameter_name: robot_ip_2,
            load_gripper_1_parameter_name: load_gripper_1,
            load_gripper_2_parameter_name: load_gripper_2,
            arm_id_1_parameter_name: arm_id_1,
            arm_id_2_parameter_name: arm_id_2,
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

    gripper_bridge_1 = Node(
        package='franka_example_controllers',
        executable='gripper_action_bridge',
        name=[arm_id_1, '_gripper_action_bridge'],
        output='screen',
        parameters=[{'arm_id': arm_id_1}],
        condition=IfCondition(load_gripper_1),
    )

    gripper_bridge_2 = Node(
        package='franka_example_controllers',
        executable='gripper_action_bridge',
        name=[arm_id_2, '_gripper_action_bridge'],
        output='screen',
        parameters=[{'arm_id': arm_id_2}],
        condition=IfCondition(load_gripper_2),
    )
    def _apply_collision_params(context):
        arm1 = arm_id_1.perform(context)
        arm2 = arm_id_2.perform(context)
        actions = []
        for arm in (arm1, arm2):
            service_name = f"/{arm}_param_service_server/set_full_collision_behavior"
            actions.append(
                ExecuteProcess(
                    cmd=[
                        'bash', '-lc',
                        f"ros2 service call {service_name} franka_msgs/srv/SetFullCollisionBehavior '{collision_payload}' || true",
                    ],
                    output='screen',
                )
            )
        return actions

    collision_timer = TimerAction(
        period=3.0,
        actions=[OpaqueFunction(function=_apply_collision_params)],
        condition=IfCondition(apply_collision_params),
    )

    # Launch rqt_image_view windows for both wrists and fixed camera when requested
    def _launch_image_views(context):
        launch_value = LaunchConfiguration('launch_rqt_image_view').perform(context).lower() == 'true'
        if not launch_value:
            return []
        arm1 = arm_id_1.perform(context)
        arm2 = arm_id_2.perform(context)
        return [
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                    f'/cameras/{arm1}_wrist/color/image_raw',
                    '--ros-args', '-p', 'image_transport:=compressed',
                ],
                output='screen',
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                    f'/cameras/{arm2}_wrist/color/image_raw',
                    '--ros-args', '-p', 'image_transport:=compressed',
                ],
                output='screen',
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                    '/cameras/fixed/color/image_raw',
                    '--ros-args', '-p', 'image_transport:=compressed',
                ],
                output='screen',
            ),
        ]

    d405_left_launch = _realsense_launch(
        [arm_id_1, '_wrist'],
        d405_left_serial,
        d405_left_depth_profile,
        depth_color_profile=d405_left_color_profile,
        depth_exposure='21000',
        condition=IfCondition(launch_realsense),
    )
    d405_right_launch = _realsense_launch(
        [arm_id_2, '_wrist'],
        d405_right_serial,
        d405_right_depth_profile,
        depth_color_profile=d405_right_color_profile,
        depth_exposure='21000',
        condition=IfCondition(launch_realsense),
    )
    d435f_launch = _realsense_launch(
        d435f_camera_name,
        d435f_serial,
        d435f_depth_profile,
        rgb_color_profile=d435f_color_profile,
        rgb_exposure='175',
        condition=IfCondition(launch_realsense),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_1_parameter_name,
            description='Hostname or IP address of robot 1.'
        ),
        DeclareLaunchArgument(
            robot_ip_2_parameter_name,
            description='Hostname or IP address of robot 2.'
        ),
        DeclareLaunchArgument(
            arm_id_1_parameter_name,
            default_value='panda_left',
            description='Unique arm ID of robot 1.'
        ),
        DeclareLaunchArgument(
            arm_id_2_parameter_name,
            default_value='panda_right',
            description='Unique arm ID of robot 2.'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name
            )
        ),
        DeclareLaunchArgument(
            load_gripper_1_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector for robot 1.'
        ),
        DeclareLaunchArgument(
            load_gripper_2_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector for robot 2.'
        ),
        DeclareLaunchArgument(
            controller_name_parameter_name,
            default_value='dual_joint_impedance_controller',
            description='Controller name to spawn after base bringup.'
        ),
        DeclareLaunchArgument(
            launch_realsense_parameter_name,
            default_value='true',
            description='Launch RealSense cameras together with dual-arm bringup.'
        ),
        DeclareLaunchArgument(
            'launch_rqt_image_view',
            default_value='true',
            description='Launch rqt_image_view windows for wrist and fixed cameras.'
        ),
        DeclareLaunchArgument(
            'apply_collision_params',
            default_value='true',
            description='Call param service to set collision thresholds at startup.'
        ),
        base_launch,
        controller_spawner,
        gripper_bridge_1,
        gripper_bridge_2,
        collision_timer,
        OpaqueFunction(function=_launch_image_views),
        d405_left_launch,
        d405_right_launch,
        d435f_launch,
    ])
