import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def concatenate_ns(ns1, ns2, absolute=False):
    if len(ns1) == 0: return ns2
    if len(ns2) == 0: return ns1
    if ns1[0] == '/': ns1 = ns1[1:]
    if ns1[-1] == '/': ns1 = ns1[:-1]
    if ns2[0] == '/': ns2 = ns2[1:]
    if ns2[-1] == '/': ns2 = ns2[:-1]
    if absolute: ns1 = '/' + ns1
    return ns1 + '/' + ns2

def generate_launch_description():
    arm_id_1_param = "arm_id_1"
    arm_id_2_param = "arm_id_2"
    initial_positions_1_param = 'initial_positions_1'
    initial_positions_2_param = 'initial_positions_2'
    use_rviz_param = 'use_rviz'

    arm_id_1 = LaunchConfiguration(arm_id_1_param)
    arm_id_2 = LaunchConfiguration(arm_id_2_param)
    initial_positions_1 = LaunchConfiguration(initial_positions_1_param)
    initial_positions_2 = LaunchConfiguration(initial_positions_2_param)
    use_rviz = LaunchConfiguration(use_rviz_param)

    load_gripper = True
    
    franka_desc_dir = get_package_share_directory('franka_description')
    my_task_desc_dir = get_package_share_directory('my_task_description')
    franka_bringup_path = get_package_share_directory('franka_bringup')
    
    mj_dual_file = 'mj_dual.xml' if load_gripper else 'mj_dual_ng.xml'
    mj_dual_path = os.path.join(franka_desc_dir, 'mujoco', 'franka', mj_dual_file)
    objects_path = os.path.join(franka_desc_dir, 'mujoco', 'franka', 'objects.xml')
    
    stl_cam1 = os.path.join(my_task_desc_dir, 'mujoco', 'assets', 'camera_part1.stl')
    stl_cam2 = os.path.join(my_task_desc_dir, 'mujoco', 'assets', 'camera_part2.stl')

    task_run_dir = os.path.join(tempfile.gettempdir(), 'mujoco_test_env')
    os.makedirs(task_run_dir, exist_ok=True)
    
    franka_assets = os.path.join(franka_desc_dir, 'mujoco', 'franka', 'assets')
    symlink_assets = os.path.join(task_run_dir, 'assets')
    if os.path.exists(symlink_assets):
        try:
            os.remove(symlink_assets)
        except:
            pass
    try:
        os.symlink(franka_assets, symlink_assets)
    except:
        pass

    import re
    with open(mj_dual_path, 'r') as f:
        mj_dual_text = f.read()
    


    mj_dual_text = re.sub(
        r'(<body name="mj_left_hand"[^>]*>)',
        r'\1\n                        <body name="left_real_camera" pos="0 0 0" quat="1 0 0 0">\n                          <camera name="left_arm_cam" pos="0.067283 0.080304 -0.053574" xyaxes="0.542260 -0.345347 -0.766044 0.184060 0.939191 -0.292410" fovy="58"/>\n                          <geom type="mesh" mesh="cam1" material="cam_mat" mass="0.25" contype="1" conaffinity="1"/>\n                          <geom type="mesh" mesh="cam2" material="cam_mat" mass="0.25" contype="1" conaffinity="1"/>\n                        </body>',
        mj_dual_text
    )
  
    mj_dual_text = re.sub(
        r'(<body name="mj_right_hand"[^>]*>)',
        r'\1\n                        <body name="right_real_camera" pos="0 0 0" quat="1 0 0 0">\n                          <camera name="right_arm_cam" pos="0.067283 0.080304 -0.053574" xyaxes="0.542260 -0.345347 -0.766044 0.184060 0.939191 -0.292410" fovy="58"/>\n                          <geom type="mesh" mesh="cam1" material="cam_mat" mass="0.25" contype="1" conaffinity="1"/>\n                          <geom type="mesh" mesh="cam2" material="cam_mat" mass="0.25" contype="1" conaffinity="1"/>\n                        </body>',
        mj_dual_text
    )
    
    dynamic_mj_dual_path = os.path.join(task_run_dir, 'mj_dual_dynamic.xml')
    with open(dynamic_mj_dual_path, 'w') as f:
        f.write(mj_dual_text)

    # Generate custom mujoco xml
    xml_content = f"""<mujoco model="my_task_scene">

  <!-- Load the dynamically modified mj_dual model with tracking cameras -->
  <include file="{dynamic_mj_dual_path}"/>
  <include file="{objects_path}"/>

  <!-- Visual/Environment Settings -->
  <statistic center="0.3 0 0.4" extent="1"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20" offwidth="640" offheight="480"/>
  </visual>

  <!-- Ground plane & Skybox -->
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

 <!-- My Custom Objects -->
  <asset>
    <!-- Use absolute path so MuJoCo finds them from anywhere -->
    <mesh name="cam1" file="{stl_cam1}" scale="0.001 0.001 0.001"/>
    <mesh name="cam2" file="{stl_cam2}" scale="0.001 0.001 0.001"/>
    <material name="cam_mat" rgba="0.7 0.7 0.7 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" pos="0 0 0" type="plane" material="groundplane"/>
    
    <!-- World cameras -->
    <camera name="fixed_cam" pos="1.3 0.0 0.8" xyaxes="0 1 0 -0.5 0 1" fovy="60"/>
    
  </worldbody> 
</mujoco>
"""

    temp_xml_path = os.path.join(task_run_dir, 'my_task_scene_dynamic.xml')
    with open(temp_xml_path, 'w') as f:
        f.write(xml_content)

    xml_file = temp_xml_path

    franka_xacro_file = os.path.join(franka_desc_dir, 'robots', 'sim', "dual_panda_arm_sim.urdf.xacro")

    mjros_config_file = os.path.join(franka_bringup_path, 'config', 'sim', 'dual_sim_controllers.yaml')


    import yaml
    with open(mjros_config_file, 'r') as f:
        merged_config = yaml.safe_load(f)
    
    if 'mujoco_server' not in merged_config:
        merged_config['mujoco_server'] = {'ros__parameters': {}}
    if 'ros__parameters' not in merged_config['mujoco_server']:
        merged_config['mujoco_server']['ros__parameters'] = {}
        
    merged_config['mujoco_server']['ros__parameters']['cam_config'] = {
        'fixed_cam': {'stream_type': 1, 'frequency': 30.0, 'width': 320, 'height': 240},
        'left_arm_cam': {'stream_type': 1, 'frequency': 30.0, 'width': 320, 'height': 240},
        'right_arm_cam': {'stream_type': 1, 'frequency': 30.0, 'width': 320, 'height': 240}
    }
    
    merged_mjros_config_file = os.path.join(task_run_dir, 'merged_sim_controllers.yaml')
    with open(merged_mjros_config_file, 'w') as f:
        yaml.dump(merged_config, f)


    ns=""




    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' arm_id_1:=', arm_id_1, 
            ' arm_id_2:=', arm_id_2,
            ' hand_1:=', str(load_gripper).lower(),
            ' hand_2:=', str(load_gripper).lower(),
            ' initial_positions_1:=', initial_positions_1,
            ' initial_positions_2:=', initial_positions_2])

    params = {'robot_description': robot_description}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace= ns,
        parameters=[params]
    )

    jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
    if load_gripper:
        jsp_source_list.append(concatenate_ns(ns, 'mj_left_gripper_sim_node/joint_states/joint_states', True))
        jsp_source_list.append(concatenate_ns(ns, 'mj_right_gripper_sim_node/joint_states/joint_states', True))

    node_joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace= ns,
            parameters=[{'source_list': jsp_source_list, 'rate': 30}],
    )

    node_left_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0.93969262', '0.34202014', '0', '0', 'mj_left_link8', 'left_real_camera'],
        output='screen',
    )

    node_right_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0.93969262', '0.34202014', '0', '0', 'mj_right_link8', 'right_real_camera'],
        output='screen',
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz', 'visualize_dual_franka.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            use_rviz_param, default_value='false', description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            arm_id_1_param, default_value='mj_left', description='Unique name of robot 1.'),
        DeclareLaunchArgument(
            arm_id_2_param, default_value='mj_right', description='Unique name of robot 2.'),
        DeclareLaunchArgument(
            initial_positions_1_param, default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"', description='Init pos robot 1.'),
        DeclareLaunchArgument(
            initial_positions_2_param, default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"', description='Init pos robot 2.'),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(franka_bringup_path + '/launch/sim/launch_mujoco_ros_server.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_file,
                'verbose': "false",
                'ns': ns,
                'mujoco_plugin_config': merged_mjros_config_file
            }.items()
        ),

        node_robot_state_publisher,
        node_joint_state_publisher,
        node_left_camera_tf,
        node_right_camera_tf,

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['multi_cartesian_impedance_controller', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        Node(
            package='franka_example_controllers',
            executable='gripper_action_bridge',
            name='left_gripper_action_bridge',
            output='screen',
            parameters=[{'arm_id': 'left'}]
        ),
        Node(
            package='franka_example_controllers',
            executable='gripper_action_bridge',
            name='right_gripper_action_bridge',
            output='screen',
            parameters=[{'arm_id': 'right'}]
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )
    ])
