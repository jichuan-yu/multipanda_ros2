#!/usr/bin/env python3
"""
Key-controlled teleop with IK for dual Panda robot.
Uses PyKDL for stable inverse kinematics.
Publishes joint angles to /dualArm_traj for mprc controller.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
import threading
import numpy as np
import math
import os
import PyKDL as kdl

# Load URDF for KDL chain
from urdf_parser_py.urdf import URDF


class KDLPandaIK:
    """KDL-based IK solver for Panda robot"""

    def __init__(self, base_link="panda_link0", tip_link="panda_link8", base_offset=None):
        """
        Args:
            base_link: Base link name
            tip_link: Tip link name
            base_offset: [x, y, z] offset of the robot base from world origin
        """
        self.chain = None
        self.ik_solver = None
        self.fk_solver = None
        self.base_link = base_link
        self.tip_link = tip_link
        self.base_offset = np.array(base_offset) if base_offset is not None else np.zeros(3)
        self._init_from_robot_description()

    def _init_from_robot_description(self):
        """Initialize KDL chain from robot description"""
        try:
            # Try to get robot description from parameter server
            import rclpy
            if not rclpy.ok():
                rclpy.init()

            node = rclpy.create_node('kdl_panda_ik_temp')
            robot_desc = node.get_parameter('robot_description').value

            if robot_desc:
                urdf_model = URDF.from_xml_string(robot_desc)
                self.chain = kdl.Chain()
                success = self._build_chain(urdf_model)
                if success:
                    self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)  # Levenberg-Marquardt
                    self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
                    print("KDL IK solver initialized successfully")
                    return True
        except Exception as e:
            print(f"Failed to init KDL from robot_description: {e}")

        # Fallback: build manual KDL chain for Panda
        print("Using manual KDL chain for Panda")
        self._build_manual_chain()
        return True

    def _build_chain(self, urdf_model):
        """Build KDL chain from URDF"""
        # This would parse URDF properly, but using fallback instead
        return False

    def _build_manual_chain(self):
        """Build manual KDL chain for Panda robot (7 DOF + end effector)"""
        self.chain = kdl.Chain()

        # Panda DH parameters - using KDL rotation methods
        # Joint 1: RotZ, d=0.333
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0, 0, 0.333))
        ))
        # Joint 2: RotZ, alpha=-90°
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RPY(-math.pi/2, 0, 0), kdl.Vector(0, 0, 0))
        ))
        # Joint 3: RotZ, d=0.316, alpha=90°
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RPY(math.pi/2, 0, 0), kdl.Vector(0, 0, 0.316))
        ))
        # Joint 4: RotZ, a=0.0825, alpha=90°
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RPY(math.pi/2, 0, 0), kdl.Vector(0, 0, 0.0825))
        ))
        # Joint 5: RotZ, a=-0.0825, alpha=-90°
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RPY(-math.pi/2, 0, 0), kdl.Vector(0, 0, -0.0825))
        ))
        # Joint 6: RotZ, d=0.384, alpha=90°
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RPY(math.pi/2, 0, 0), kdl.Vector(0, 0, 0.384))
        ))
        # Joint 7: RotZ, alpha=90°
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RPY(math.pi/2, 0, 0), kdl.Vector(0, 0, 0))
        ))
        # End effector offset (link8): d=0.088, fixed joint
        # In PyKDL, fixed joint is created with Joint(name) without type
        self.chain.addSegment(kdl.Segment(
            kdl.Joint("link8_fixed"),
            kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0, 0, 0.088))
        ))

        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        return True

    def solve_ik(self, q_init, target_pos, target_rot_matrix, max_iter=100):
        """Solve IK for target pose

        Args:
            q_init: Initial joint angles (7-element array)
            target_pos: Target position [x, y, z] in world frame
            target_rot_matrix: Target rotation (3x3 matrix)
            max_iter: Maximum iterations

        Returns:
            q_sol: Solution joint angles
            success: Whether IK succeeded
        """
        if self.ik_solver is None:
            return q_init, False

        # Convert world frame target to robot base frame
        target_pos_base = target_pos - self.base_offset

        # Debug output (only once per instance)
        if not hasattr(self, '_debug_printed'):
            print(f"[KDL IK] Base offset: {self.base_offset}")
            self._debug_printed = True

        # Create KDL arrays
        q_init_kdl = kdl.JntArray(7)
        for i in range(7):
            q_init_kdl[i] = q_init[i]

        # Create target frame (in base frame)
        target_frame = kdl.Frame()
        target_frame.p = kdl.Vector(target_pos_base[0], target_pos_base[1], target_pos_base[2])

        # Convert rotation matrix to KDL rotation
        target_frame.M = kdl.Rotation(
            target_rot_matrix[0, 0], target_rot_matrix[0, 1], target_rot_matrix[0, 2],
            target_rot_matrix[1, 0], target_rot_matrix[1, 1], target_rot_matrix[1, 2],
            target_rot_matrix[2, 0], target_rot_matrix[2, 1], target_rot_matrix[2, 2]
        )

        # Solve IK
        q_sol_kdl = kdl.JntArray(7)
        ret = self.ik_solver.CartToJnt(q_init_kdl, target_frame, q_sol_kdl)

        # Extract solution
        q_sol = np.array([q_sol_kdl[i] for i in range(7)])

        # Joint limits
        q_lb = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        q_ub = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        q_sol = np.clip(q_sol, q_lb, q_ub)

        # Verify solution
        if ret >= 0:
            return q_sol, True
        else:
            # Check if close enough
            fk_frame = kdl.Frame()
            self.fk_solver.JntToCart(q_sol_kdl, fk_frame)
            pos_error = np.linalg.norm([
                fk_frame.p.x() - target_pos_base[0],
                fk_frame.p.y() - target_pos_base[1],
                fk_frame.p.z() - target_pos_base[2]
            ])
            if pos_error < 0.05:  # 5cm tolerance
                return q_sol, True

        return q_init, False

    def forward_kinematics(self, q):
        """Compute forward kinematics

        Returns:
            pos: End-effector position [x, y, z] in world frame
            rot: Rotation matrix (3x3)
        """
        if self.fk_solver is None:
            return np.zeros(3), np.eye(3)

        q_kdl = kdl.JntArray(7)
        for i in range(7):
            q_kdl[i] = q[i]

        frame = kdl.Frame()
        self.fk_solver.JntToCart(q_kdl, frame)

        # Position in base frame, convert to world frame
        pos = np.array([
            frame.p.x() + self.base_offset[0],
            frame.p.y() + self.base_offset[1],
            frame.p.z() + self.base_offset[2]
        ])

        # Extract rotation matrix
        rot = np.eye(3)
        for i in range(3):
            for j in range(3):
                rot[i, j] = frame.M[i, j]

        return pos, rot


class KeyIKTeleop(Node):
    def __init__(self):
        super().__init__('key_ik_teleop')
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Publisher for joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/dualArm_traj',
            10
        )

        # Subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Initialize IK solvers with base offsets
        # Left arm: base offset [0, +0.26, 0] (from dual_panda_arm_sim.urdf.xacro)
        # Right arm: base offset [0, -0.26, 0]
        self.ik_left = KDLPandaIK(base_offset=[0, 0.26, 0])
        self.ik_right = KDLPandaIK(base_offset=[0, -0.26, 0])

        # Current joint angles (initial guess for IK) - home position
        # Use copy() to ensure independent arrays
        self.q_init = {
            'left': np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]).copy(),
            'right': np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]).copy()
        }

        # Compute initial end-effector poses from home joint positions using FK
        # This ensures self.poses matches the actual FK of q_init
        from scipy.spatial.transform import Rotation as R

        pos_l, rot_l = self.ik_left.forward_kinematics(self.q_init['left'])
        pos_r, rot_r = self.ik_right.forward_kinematics(self.q_init['right'])

        # Convert rotation matrices to Euler angles (XYZ convention)
        euler_l = R.from_matrix(rot_l).as_euler('xyz')
        euler_r = R.from_matrix(rot_r).as_euler('xyz')

        # Initial poses (position + Euler angles) - computed from FK of home position
        self.poses = {
            'left': {
                'pos': pos_l.copy(),
                'rot': euler_l.copy()
            },
            'right': {
                'pos': pos_r.copy(),
                'rot': euler_r.copy()
            }
        }

        self.selected_arm = 'left'
        self.step_pos = 0.01
        self.step_rot = 0.05
        self.ik_success = {'left': True, 'right': True}

        # Movement threshold for publishing (must match interpolator's vel * dt = 2.0 * 0.01 = 0.02 rad)
        self.min_movement_threshold = 0.02  # rad

        # Accumulated target for each arm (for small movement accumulation)
        self.accumulated_delta = {
            'left': np.zeros(3),  # [dx, dy, dz]
            'right': np.zeros(3)
        }

        # Flag to track if target has been updated
        self.target_updated = False
        self.last_published_q = None
        self.joint_states_received = False

        # Low-rate timer for status check (1Hz)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('Key IK Teleop initialized')
        self.print_usage()

    def print_usage(self):
        msg = """
========================================
Keyboard IK Teleop for Dual Panda Arm
========================================
Controls:
  Z/X : Select Left/Right arm
  W/S : Forward/Backward (X axis)
  A/D : Left/Right (Y axis)
  Q/E : Up/Down (Z axis)
  J/L : Rotate around X (-/+)
  I/K : Rotate around Y (-/+)
  U/O : Rotate around Z (-/+)
  Ctrl-C to quit
========================================
Waiting for joint states...
"""
        print(msg, flush=True)

    def status_callback(self):
        """Print status at 1Hz"""
        if self.joint_states_received:
            left_pos = self.poses['left']['pos']
            right_pos = self.poses['right']['pos']
            print(f"\r[{self.selected_arm.upper()}] L: [{left_pos[0]:.3f}, {left_pos[1]:.3f}, {left_pos[2]:.3f}]  "
                  f"R: [{right_pos[0]:.3f}, {right_pos[1]:.3f}, {right_pos[2]:.3f}]  "
                  f"IK: L={'OK' if self.ik_success['left'] else 'FAIL'} "
                  f"R={'OK' if self.ik_success['right'] else 'FAIL'}   ", end='', flush=True)

    def joint_state_callback(self, msg):
        """Update current joint states from /joint_states"""
        try:
            # Extract left arm joints
            left_indices = []
            for i in range(1, 8):
                if f'mj_left_joint{i}' in msg.name:
                    left_indices.append(msg.name.index(f'mj_left_joint{i}'))

            # Extract right arm joints
            right_indices = []
            for i in range(1, 8):
                if f'mj_right_joint{i}' in msg.name:
                    right_indices.append(msg.name.index(f'mj_right_joint{i}'))

            if len(left_indices) == 7:
                self.q_init['left'] = np.array([msg.position[i] for i in left_indices])

            if len(right_indices) == 7:
                self.q_init['right'] = np.array([msg.position[i] for i in right_indices])

            if not self.joint_states_received and len(left_indices) == 7 and len(right_indices) == 7:
                self.joint_states_received = True
                print(f"\nJoint states received. Ready for control!", flush=True)
                # Initialize target poses to current end-effector positions
                pos_l, rot_l = self.ik_left.forward_kinematics(self.q_init['left'])
                pos_r, rot_r = self.ik_right.forward_kinematics(self.q_init['right'])
                self.poses['left']['pos'] = pos_l
                self.poses['right']['pos'] = pos_r

        except Exception as e:
            pass

    def euler_to_rot_matrix(self, euler):
        """Convert Euler angles (XYZ) to rotation matrix"""
        from scipy.spatial.transform import Rotation
        return Rotation.from_euler('xyz', euler).as_matrix()

    def update_pose(self, key):
        """Update target pose based on key input - accumulates small movements"""
        if key == 'z':
            if self.selected_arm != 'left':
                self.selected_arm = 'left'
                return True
        elif key == 'x':
            if self.selected_arm != 'right':
                self.selected_arm = 'right'
                return True

        # Get accumulated delta for selected arm
        delta = self.accumulated_delta[self.selected_arm]
        updated = False

        # Position control - accumulate delta instead of applying directly
        if key == 'w':
            delta[0] += self.step_pos
            updated = True
        elif key == 's':
            delta[0] -= self.step_pos
            updated = True
        elif key == 'a':
            delta[1] += self.step_pos
            updated = True
        elif key == 'd':
            delta[1] -= self.step_pos
            updated = True
        elif key == 'q':
            delta[2] += self.step_pos
            updated = True
        elif key == 'e':
            delta[2] -= self.step_pos
            updated = True

        # Rotation control - apply directly (rotation changes are larger)
        elif key == 'j':
            self.poses[self.selected_arm]['rot'][0] -= self.step_rot
            self.target_updated = True
            return True
        elif key == 'l':
            self.poses[self.selected_arm]['rot'][0] += self.step_rot
            self.target_updated = True
            return True
        elif key == 'i':
            self.poses[self.selected_arm]['rot'][1] -= self.step_rot
            self.target_updated = True
            return True
        elif key == 'k':
            self.poses[self.selected_arm]['rot'][1] += self.step_rot
            self.target_updated = True
            return True
        elif key == 'u':
            self.poses[self.selected_arm]['rot'][2] -= self.step_rot
            self.target_updated = True
            return True
        elif key == 'o':
            self.poses[self.selected_arm]['rot'][2] += self.step_rot
            self.target_updated = True
            return True

        if updated:
            # Apply the movement immediately without accumulation
            self.poses[self.selected_arm]['pos'] += delta
            delta[:] = 0  # Reset temporary delta
            self.target_updated = True

        return updated

    def compute_ik(self):
        """Compute IK for both arms using KDL"""
        # Left arm IK
        target_pos_left = self.poses['left']['pos']
        target_rot_left = self.euler_to_rot_matrix(self.poses['left']['rot'])

        # Check if target pose matches FK of current q_init (within tolerance)
        pos_fk_left, rot_fk_left = self.ik_left.forward_kinematics(self.q_init['left'])
        pos_error_left = np.linalg.norm(target_pos_left - pos_fk_left)
        rot_error_left = np.linalg.norm(target_rot_left - rot_fk_left, 'fro')

        if pos_error_left < 1e-6 and rot_error_left < 1e-6:
            # Target matches current q_init, use q_init directly to avoid IK drift
            q_left = self.q_init['left']
            success_left = True
        else:
            q_left, success_left = self.ik_left.solve_ik(
                self.q_init['left'],
                target_pos_left,
                target_rot_left
            )
            if success_left:
                self.q_init['left'] = q_left

        self.ik_success['left'] = success_left

        # Right arm IK
        target_pos_right = self.poses['right']['pos']
        target_rot_right = self.euler_to_rot_matrix(self.poses['right']['rot'])

        # Check if target pose matches FK of current q_init (within tolerance)
        pos_fk_right, rot_fk_right = self.ik_right.forward_kinematics(self.q_init['right'])
        pos_error_right = np.linalg.norm(target_pos_right - pos_fk_right)
        rot_error_right = np.linalg.norm(target_rot_right - rot_fk_right, 'fro')

        if pos_error_right < 1e-6 and rot_error_right < 1e-6:
            # Target matches current q_init, use q_init directly to avoid IK drift
            q_right = self.q_init['right']
            success_right = True
        else:
            q_right, success_right = self.ik_right.solve_ik(
                self.q_init['right'],
                target_pos_right,
                target_rot_right
            )
            if success_right:
                self.q_init['right'] = q_right

        self.ik_success['right'] = success_right

        return q_left, q_right

    def publish_trajectory(self):
        """Publish trajectory only when target is updated and movement is significant"""
        q_left, q_right = self.compute_ik()
        
        # Determine current state for BOTH arms
        # Use the actual current posture of the non-selected arm from q_init
        # to ensure it stays static while the other moves
        if self.selected_arm == 'left':
            q_current_active = q_left
            q_current_static = self.q_init['right']
            q_target = np.concatenate([q_left, self.q_init['right']])
        else:
            q_current_active = q_right
            q_current_static = self.q_init['left']
            q_target = np.concatenate([self.q_init['left'], q_right])

        # Only publish if there's significant movement or first user input
        should_publish = False

        if self.last_published_q is None:
            # First time - initialize but don't publish yet
            self.last_published_q = q_target.copy()
            self.target_updated = False
            return

        # Check if there's significant movement
        if self.target_updated:
            diff = np.abs(q_target - self.last_published_q)
            max_diff = np.max(diff)
            if max_diff > 0.005:  
                should_publish = True
            else:
                self.target_updated = False
                return

        if should_publish:
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = [f'mj_left_joint{i+1}' for i in range(7)] + \
                               [f'mj_right_joint{i+1}' for i in range(7)]

            total_duration = 2.0  
            num_points = 10

            for i in range(num_points + 1):
                alpha = i / num_points
                point = JointTrajectoryPoint()
                # Interpolate only the target arm, keep the other fixed at its LAST position
                interpolated_q = (1 - alpha) * self.last_published_q + alpha * q_target
                point.positions = interpolated_q.tolist()
                point.velocities = [0.0] * 14
                
                t = alpha * total_duration
                point.time_from_start.sec = int(t)
                point.time_from_start.nanosec = int((t - int(t)) * 1e9)
                
                traj.points.append(point)

            self.traj_pub.publish(traj)
            self.save_to_csv(traj)
            self.log_published_data(q_left, q_right)

            self.last_published_q = q_target.copy()
            self.target_updated = False

    def log_published_data(self, q_left, q_right):
        """Log the published joint angles to console"""
        print(f"\n[Publishing to /dualArm_traj]")
        print(f"  Left arm (q1-q7):  {q_left[0]:8.4f}, {q_left[1]:8.4f}, {q_left[2]:8.4f}, {q_left[3]:8.4f}, {q_left[4]:8.4f}, {q_left[5]:8.4f}, {q_left[6]:8.4f}")
        print(f"  Right arm (q1-q7): {q_right[0]:8.4f}, {q_right[1]:8.4f}, {q_right[2]:8.4f}, {q_right[3]:8.4f}, {q_right[4]:8.4f}, {q_right[5]:8.4f}, {q_right[6]:8.4f}")
        print(f"  IK Status: Left={'OK' if self.ik_success['left'] else 'FAIL'}, Right={'OK' if self.ik_success['right'] else 'FAIL'}")
        print(f"  Target: [{self.selected_arm.upper()}] pos={self.poses[self.selected_arm]['pos']}")
        sys.stdout.flush()

    def save_to_csv(self, traj_msg):
        """Save JointTrajectory to temp.csv in src/multipanda_ros2/tools/"""
        try:
            # Get the path to src/multipanda_ros2/tools/
            # This script is in src/multipanda_ros2/spacemouse_teleop/
            current_dir = os.path.dirname(os.path.abspath(__file__))
            tools_dir = os.path.join(os.path.dirname(current_dir), 'tools')
            
            # Ensure the directory exists
            if not os.path.exists(tools_dir):
                os.makedirs(tools_dir, exist_ok=True)
                
            csv_path = os.path.join(tools_dir, 'temp.csv')
            with open(csv_path, 'w') as f:
                # Header
                f.write("q1_left,q2_left,q3_left,q4_left,q5_left,q6_left,q7_left,q1_right,q2_right,q3_right,q4_right,q5_right,q6_right,q7_right\n")
                # Data points
                for point in traj_msg.points:
                    line = ",".join([f"{p:.6f}" for p in point.positions])
                    f.write(line + "\n")
            print(f"\n[CSV Monitor] Saved to: {csv_path}")
        except Exception as e:
            print(f"\n[CSV Monitor] Failed to save CSV: {e}")


def get_key(settings):
    """Non-blocking key input"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyIKTeleop()

    settings = termios.tcgetattr(sys.stdin)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            key = get_key(settings).lower()

            if key == '\x03':  # ctrl-c
                break

            if key:
                updated = node.update_pose(key)
                if updated:
                    node.publish_trajectory()

            # Also check and publish periodically if target was updated
            node.publish_trajectory()

            # Small sleep
            import time
            time.sleep(0.01)

    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop stopped.")


if __name__ == '__main__':
    main()
