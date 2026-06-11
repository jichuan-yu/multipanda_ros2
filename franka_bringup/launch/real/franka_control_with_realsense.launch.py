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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
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
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    controller_name_parameter_name = 'controller_name'

    launch_realsense_parameter_name = 'launch_realsense'
    launch_rqt_image_view_parameter_name = 'launch_rqt_image_view'

    wrist_camera_parameter_name = 'wrist_camera'

    # Fixed RealSense identities for this setup.
    d405_wrist_topic_name = 'panda_wrist'
    d435f_fixed_topic_name = 'fixed'

    d405_left_serial = "'409122274276'"
    d405_right_serial = "'352122272335'"
    d435f_serial = "'244222074410'"

    d405_depth_profile = '640,480,30'
    d405_color_profile = '640,480,30'
    d435f_depth_profile = '848,480,30'
    d435f_color_profile = '848,480,30'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)

    launch_realsense = LaunchConfiguration(launch_realsense_parameter_name)
    launch_rqt_image_view = LaunchConfiguration(launch_rqt_image_view_parameter_name)
    apply_collision_params = LaunchConfiguration('apply_collision_params')

    wrist_camera = LaunchConfiguration(wrist_camera_parameter_name)

    # Collision behavior payload (for cartesian assembly tasks)
    collision_payload = (
        '{ '
        'lower_torque_thresholds_acceleration: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], '
        'upper_torque_thresholds_acceleration: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], '
        'lower_torque_thresholds_nominal: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], '
        'upper_torque_thresholds_nominal: [20.0,20.0,18.0,18.0,16.0,14.0,12.0], '
        'lower_force_thresholds_acceleration: [20.0,20.0,20.0,25.0,25.0,25.0], '
        'upper_force_thresholds_acceleration: [20.0,20.0,20.0,25.0,25.0,25.0], '
        'lower_force_thresholds_nominal: [50.0,50.0,60.0,50.0,50.0,50.0], '
        'upper_force_thresholds_nominal: [50.0,50.0,60.0,50.0,50.0,50.0] '
        '}'
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'real', 'franka.launch.py'])]
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
        name=['', '_gripper_action_bridge'],
        output='screen',
        parameters=[{'arm_id': 'panda'}],
        condition=IfCondition(load_gripper),
    )

    # TimerAction to call set_full_collision_behavior on common service prefixes
    collision_timer = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-lc',
                    f"ros2 service call /panda_param_service_server/set_full_collision_behavior "
                    f"franka_msgs/srv/SetFullCollisionBehavior '{collision_payload}' || true"
                ],
                output='screen',
            ),
        ],
        condition=IfCondition(apply_collision_params),
    )

    def _launch_cameras(context):
        wrist_camera_value = wrist_camera.perform(context)
        launch_rqt_image_view_value = launch_rqt_image_view.perform(context).lower() == 'true'
        wrist_camera_map = {
            'left': d405_left_serial,
            'right': d405_right_serial,
        }

        if wrist_camera_value not in wrist_camera_map:
            raise RuntimeError(
                "Invalid wrist_camera='{}'. Supported values are 'left' or 'right'.".format(
                    wrist_camera_value,
                )
            )

        wrist_launch = _realsense_launch(
            d405_wrist_topic_name,
            wrist_camera_map[wrist_camera_value],
            d405_depth_profile,
            depth_color_profile=d405_color_profile,
            depth_exposure='24000',
            condition=IfCondition(launch_realsense),
        )

        fixed_launch = _realsense_launch(
            d435f_fixed_topic_name,
            d435f_serial,
            d435f_depth_profile,
            rgb_color_profile=d435f_color_profile,
            rgb_exposure='185',
            condition=IfCondition(launch_realsense),
        )

        image_view_launches = []
        if launch_rqt_image_view_value:
            image_view_launches = [
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                        '/cameras/panda_wrist/color/image_raw',
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

        return [wrist_launch, fixed_launch, *image_view_launches]

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'
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
            load_gripper_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector.'
        ),
        DeclareLaunchArgument(
            controller_name_parameter_name,
            default_value='joint_impedance_controller',
            description='Controller name to spawn after base bringup.'
        ),
        DeclareLaunchArgument(
            launch_realsense_parameter_name,
            default_value='true',
            description='Launch RealSense cameras together with bringup.'
        ),
        DeclareLaunchArgument(
            launch_rqt_image_view_parameter_name,
            default_value='true',
            description='Launch two rqt_image_view windows for panda_wrist and fixed.'
        ),
        DeclareLaunchArgument(
            wrist_camera_parameter_name,
            default_value='left',
            description="Wrist camera selector. Supported: 'left' or 'right'."
        ),
        DeclareLaunchArgument(
            'apply_collision_params',
            default_value='true',
            description='Call param service to set collision thresholds at startup.'
        ),
        base_launch,
        controller_spawner,
        gripper_bridge,
        collision_timer,
        OpaqueFunction(function=_launch_cameras),
    ])
