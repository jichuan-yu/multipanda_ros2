#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
import sys
import select
import termios
import tty
import threading
import numpy as np
import pinocchio as pin
import pink
from pink import tasks
from pink.tasks import FrameTask
import math
import os
import subprocess
import time

def get_urdf_from_xacro():
    """Generate URDF from xacro using current workspace paths and settings"""
    # Define paths
    ws_base = "/home/xiaozy24/dual_panda_ws"
    xacro_path = os.path.join(ws_base, "src/multipanda_ros2/franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro")
    
    # We need to source ROS and workspace setup files to let xacro find packages
    # Using a shell approach to source and run xacro in one go
    full_command = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {ws_base}/install/setup.bash && "
        f"xacro {xacro_path} arm_id_1:=left arm_id_2:=right hand_1:=true hand_2:=true"
    )
    
    try:
        # Run via shell to support 'source' and environment variables
        result = subprocess.run(full_command, capture_output=True, text=True, check=True, shell=True, executable="/bin/bash")
        return result.stdout
    except Exception as e:
        print(f"Error generating URDF: {e}")
        if hasattr(e, 'stderr'):
            print(f"Xacro stderr: {e.stderr}")
        return None

def euler_to_matrix(rx, ry, rz):
    # Rz * Ry * Rx
    cz = math.cos(rz)
    sz = math.sin(rz)
    cy = math.cos(ry)
    sy = math.sin(ry)
    cx = math.cos(rx)
    sx = math.sin(rx)
    
    Rx = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx, cx]])
    Ry = np.array([[cy, 0, sy],
                   [0, 1, 0],
                   [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0],
                   [sz, cz, 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

class KeyTeleopIKNode(Node):
    def __init__(self):
        super().__init__('key_teleop_ik')
        
        # IK setup
        # Generate URDF dynamically to match the simulation environment
        urdf_xml = get_urdf_from_xacro()
        if urdf_xml is None:
            self.get_logger().error('Failed to generate URDF from xacro.')
            sys.exit(1)
            
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        
        # Define a valid initial configuration (q0) that respects joint limits
        # Using the home position provided by the user
        q_home = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        
        # We need to map these 7 joints to the full robot configuration (18 joints total)
        # From previous investigation:
        # Joints 1-7: left arm (idx_q 0-6)
        # Joints 8-9: left finger (idx_q 7-8)
        # Joints 10-16: right arm (idx_q 9-15)
        # Joints 17-18: right finger (idx_q 16-17)
        q0 = pin.neutral(self.model)
        q0[0:7] = q_home
        q0[9:16] = q_home
            
        self.configuration = pink.Configuration(self.model, self.data, q0)
        
        # Target frames
        self.left_ee_frame = "left_hand_tcp"
        self.right_ee_frame = "right_hand_tcp"
        
        # Initial positions and rotations
        # User requested: [0.307, 0.0, 0.487] for both arms relative to their base? 
        # Actually base_link is at (0,0,0). 
        # Left panda base is at (0, 0.26, 0). Right panda base is at (0, -0.26, 0).
        # We should set targets in world frame (base_link).
        self.poses = {
            'left': {
                'pos': np.array([0.307, 0.26, 0.487]),
                'rot': np.array([math.pi, 0.0, 0.0]) # rx, ry, rz
            },
            'right': {
                'pos': np.array([0.307, -0.26, 0.487]),
                'rot': np.array([math.pi, 0.0, 0.0])
            }
        }
        
        self.selected_arm = 'left'
        self.step_pos = 0.01
        self.step_rot = 0.05
        
        # Publishers
        # Primary: publish joint angles to dual_joint_impedance_controller
        self.joint_desired_pub = self.create_publisher(Float64MultiArray, '/dual_joint_impedance/joints_desired', 10)
        # Secondary: publish full joint state for monitoring
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.left_gripper_pub = self.create_publisher(Float64, '/mj_left_gripper/width_desired', 10)
        self.right_gripper_pub = self.create_publisher(Float64, '/mj_right_gripper/width_desired', 10)

        # Pink tasks
        self.tasks = {
            'left': FrameTask(self.left_ee_frame, position_cost=1.0, orientation_cost=1.0),
            'right': FrameTask(self.right_ee_frame, position_cost=1.0, orientation_cost=1.0)
        }
        
        # Initialize tasks with current identity or current EE pose
        for arm_name in ['left', 'right']:
            pos = self.poses[arm_name]['pos']
            R = euler_to_matrix(*self.poses[arm_name]['rot'])
            target_pose = pin.SE3(R, pos)
            self.tasks[arm_name].set_target(target_pose)

        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz is enough for teleop steps
        self.get_logger().info('Key IK publisher initialized.')
        self.print_usage()

    def print_usage(self):
        msg = """
----------------------------------------
Keyboard Teleop for Dual Cartesian Arm (IK)
----------------------------------------
Z/X : Select Left/Right arm (Current: {})
W/S : Forward/Backward (X)
A/D : Left/Right (Y)
Q/E : Up/Down (Z)
J/L : Rotate around X (-/+)
I/K : Rotate around Y (-/+)
U/O : Rotate around Z (-/+)
C/V : Close/Open Gripper
Ctrl-C to quit
----------------------------------------
""".format(self.selected_arm.upper()).replace('\n', '\r\n')
        print(msg, flush=True)

    def move_gripper(self, arm_name, width):
        msg = Float64()
        msg.data = float(width)
        if arm_name == 'left':
            self.left_gripper_pub.publish(msg)
        else:
            self.right_gripper_pub.publish(msg)

    def update_pose(self, key):
        if key == 'z':
            self.selected_arm = 'left'
            self.get_logger().info('Selected LEFT arm')
        elif key == 'x':
            self.selected_arm = 'right'
            self.get_logger().info('Selected RIGHT arm')
            
        arm = self.poses[self.selected_arm]
        
        # Position
        if key == 'w':
            arm['pos'][0] += self.step_pos
        elif key == 's':
            arm['pos'][0] -= self.step_pos
        elif key == 'a':
            arm['pos'][1] += self.step_pos
        elif key == 'd':
            arm['pos'][1] -= self.step_pos
        elif key == 'q':
            arm['pos'][2] += self.step_pos
        elif key == 'e':
            arm['pos'][2] -= self.step_pos
            
        # Rotation
        elif key == 'j':
            arm['rot'][0] -= self.step_rot
        elif key == 'l':
            arm['rot'][0] += self.step_rot
        elif key == 'i':
            arm['rot'][1] -= self.step_rot
        elif key == 'k':
            arm['rot'][1] += self.step_rot
        elif key == 'u':
            arm['rot'][2] -= self.step_rot
        elif key == 'o':
            arm['rot'][2] += self.step_rot
            
        elif key == 'c':
            self.move_gripper(self.selected_arm, 0.0)
        elif key == 'v':
            self.move_gripper(self.selected_arm, 0.08)
            
        # Update target for Pink
        pos = arm['pos']
        R = euler_to_matrix(*arm['rot'])
        target_pose = pin.SE3(R, pos)
        self.tasks[self.selected_arm].set_target(target_pose)

    def solve_ik(self):
        # Solve IK using Pink
        # We want to maintain both arms' targets
        # Using 'quadprog' as the default solver which is common for pink
        velocity = pink.solve_ik(self.configuration, self.tasks.values(), dt=0.05, solver="quadprog")
        self.configuration.integrate_inplace(velocity, 0.05)
        return self.configuration.q

    def timer_callback(self):
        start_time = time.perf_counter()
        q = self.solve_ik()
        
        # Publish joint angles to dual_joint_impedance_controller
        # Expected format: [left_j1, left_j2, ..., left_j7, right_j1, right_j2, ..., right_j7]
        joint_desired_msg = Float64MultiArray()
        # Extract 7 joints for left arm (indices 0-6)
        # Extract 7 joints for right arm (indices 9-15, skipping finger joints 7-8)
        joint_desired_msg.data = []
        joint_desired_msg.data.extend(q[0:7].tolist())  # Left arm: joint1-7
        joint_desired_msg.data.extend(q[9:16].tolist()) # Right arm: joint1-7
        
        self.joint_desired_pub.publish(joint_desired_msg)
        
        # Also publish full joint states for monitoring
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        # Pinocchio joint names include 'universe' at index 0, which we skip for JointState
        joint_names = []
        for i in range(1, self.model.njoints):
            joint_names.append(self.model.names[i])
            
        joint_state.name = joint_names
        joint_state.position = q.tolist()
        
        self.joint_pub.publish(joint_state)
        
        # Verification: FK
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        left_id = self.model.getFrameId(self.left_ee_frame)
        right_id = self.model.getFrameId(self.right_ee_frame)
        
        left_fk = self.data.oMf[left_id]
        right_fk = self.data.oMf[right_id]
        
        # Log FK results only when position changes significantly to verify IK
        current_left_pos = left_fk.translation
        current_right_pos = right_fk.translation
        
        if not hasattr(self, 'last_printed_pos'):
            self.last_printed_pos = {'left': np.zeros(3), 'right': np.zeros(3)}

        diff_left = np.linalg.norm(current_left_pos - self.last_printed_pos['left'])
        diff_right = np.linalg.norm(current_right_pos - self.last_printed_pos['right'])

        if diff_left > 1e-4 or diff_right > 1e-4:
            self.get_logger().info(f'\n--- IK Verification (FK result) ---')
            self.get_logger().info(f'Selected Arm: {self.selected_arm.upper()}')
            self.get_logger().info(f'Target Left:  {self.poses["left"]["pos"]}')
            self.get_logger().info(f'Actual Left:  {current_left_pos.tolist()}')
            self.get_logger().info(f'Target Right: {self.poses["right"]["pos"]}')
            self.get_logger().info(f'Actual Right: {current_right_pos.tolist()}')
            self.get_logger().info(f'-----------------------------------')
            
            self.last_printed_pos['left'] = current_left_pos.copy()
            self.last_printed_pos['right'] = current_right_pos.copy()
            
        # Log computational time
        end_time = time.perf_counter()
        comp_time_ms = (end_time - start_time) * 1000
        if not hasattr(self, 'comp_times'):
            self.comp_times = []
        self.comp_times.append(comp_time_ms)
        
        # Periodically show average computational time
        if len(self.comp_times) >= 20: # Every 1 second at 20Hz
            avg_time = sum(self.comp_times) / len(self.comp_times)
            max_time = max(self.comp_times)
            self.get_logger().info(f'[Comp Time] Avg: {avg_time:.2f}ms | Max: {max_time:.2f}ms')
            self.comp_times = []

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyTeleopIKNode()
    
    settings = termios.tcgetattr(sys.stdin)
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key:
                if key == '\x03': # Ctrl-C
                    break
                node.update_pose(key)
            
            # Spin once to run timer/callbacks
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
