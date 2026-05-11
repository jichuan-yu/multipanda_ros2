#!/usr/bin/env python3
"""
Direct joint control teleop for dual Panda robot.
Q/W/E/R/T/Y/U: Increase joints 1-7 by 0.03 rad
A/S/D/F/G/H/J: Decrease joints 1-7 by 0.03 rad
Z/X: Select Left/Right arm
Publishes 11-point interpolated trajectories to temp.csv
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
import os


class KeyJointTeleop(Node):
    def __init__(self):
        super().__init__('key_joint_teleop')
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

        # Current joint angles - home position
        self.q_current = {
            'left': np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),
            'right': np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        }

        self.selected_arm = 'left'
        self.step_joint = 0.3  # rad
        self.joint_states_received = False

        # Joint limits for Panda
        self.q_limits = {
            'lower': np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]),
            'upper': np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        }

        self.last_published_q = None

        # Status timer
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('Key Joint Teleop initialized')
        self.print_usage()

    def print_usage(self):
        msg = """
========================================
Direct Joint Control for Dual Panda Arm
========================================
Joint Increment (+ 0.03 rad):
  Q/W/E/R/T/Y/U : Joint 1-7

Joint Decrement (- 0.03 rad):
  A/S/D/F/G/H/J : Joint 1-7

Arm Selection:
  Z : Select Left arm
  X : Select Right arm

Ctrl-C to quit
========================================
Waiting for joint states...
"""
        print(msg, flush=True)

    def status_callback(self):
        """Print status at 1Hz"""
        if self.joint_states_received:
            left_q = self.q_current['left']
            right_q = self.q_current['right']
            print(f"\r[{self.selected_arm.upper()}] "
                  f"L: [{left_q[0]:7.4f}, {left_q[1]:7.4f}, {left_q[2]:7.4f}, {left_q[3]:7.4f}, {left_q[4]:7.4f}, {left_q[5]:7.4f}, {left_q[6]:7.4f}] "
                  f"R: [{right_q[0]:7.4f}, {right_q[1]:7.4f}, {right_q[2]:7.4f}, {right_q[3]:7.4f}, {right_q[4]:7.4f}, {right_q[5]:7.4f}, {right_q[6]:7.4f}]   ", 
                  end='', flush=True)

    def joint_state_callback(self, msg):
        """Update current joint states from /joint_states"""
        try:
            # Extract left arm joints
            left_indices = []
            for i in range(1, 8):
                joint_name = f'mj_left_joint{i}'
                if joint_name in msg.name:
                    left_indices.append(msg.name.index(joint_name))

            # Extract right arm joints
            right_indices = []
            for i in range(1, 8):
                joint_name = f'mj_right_joint{i}'
                if joint_name in msg.name:
                    right_indices.append(msg.name.index(joint_name))

            if len(left_indices) == 7:
                self.q_current['left'] = np.array([msg.position[i] for i in left_indices])

            if len(right_indices) == 7:
                self.q_current['right'] = np.array([msg.position[i] for i in right_indices])

            if not self.joint_states_received and len(left_indices) == 7 and len(right_indices) == 7:
                self.joint_states_received = True
                print(f"\nJoint states received. Ready for control!", flush=True)

        except Exception as e:
            pass

    def update_joint(self, key):
        """Update joint angle based on key input"""
        if key == 'z':
            if self.selected_arm != 'left':
                self.selected_arm = 'left'
                print(f"\n[Switched to LEFT arm]", flush=True)
            return False
        elif key == 'x':
            if self.selected_arm != 'right':
                self.selected_arm = 'right'
                print(f"\n[Switched to RIGHT arm]", flush=True)
            return False

        # Capture the START configuration BEFORE updating any joints
        # This ensures the first point in the CSV matches the current physical state
        q_start = np.concatenate([self.q_current['left'].copy(), self.q_current['right'].copy()])

        # Joint increase commands (Q/W/E/R/T/Y/U -> joints 0-6)
        updated = False
        increase_keys = ['q', 'w', 'e', 'r', 't', 'y', 'u']
        if key in increase_keys:
            joint_idx = increase_keys.index(key)
            self.q_current[self.selected_arm][joint_idx] += self.step_joint
            # Clamp to limits
            self.q_current[self.selected_arm][joint_idx] = np.clip(
                self.q_current[self.selected_arm][joint_idx],
                self.q_limits['lower'][joint_idx],
                self.q_limits['upper'][joint_idx]
            )
            updated = True

        # Joint decrease commands (A/S/D/F/G/H/J -> joints 0-6)
        decrease_keys = ['a', 's', 'd', 'f', 'g', 'h', 'j']
        if key in decrease_keys:
            joint_idx = decrease_keys.index(key)
            self.q_current[self.selected_arm][joint_idx] -= self.step_joint
            # Clamp to limits
            self.q_current[self.selected_arm][joint_idx] = np.clip(
                self.q_current[self.selected_arm][joint_idx],
                self.q_limits['lower'][joint_idx],
                self.q_limits['upper'][joint_idx]
            )
            updated = True

        if updated:
            # q_target is where we WANT to go (now that q_current is updated)
            q_target = np.concatenate([self.q_current['left'], self.q_current['right']])
            self.publish_trajectory(q_start, q_target)
            return True

        return False

    def publish_trajectory(self, q_start, q_target):
        """Publish 11-point interpolated trajectory from q_start to q_target"""
        # Always publish
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [f'mj_left_joint{i+1}' for i in range(7)] + \
                           [f'mj_right_joint{i+1}' for i in range(7)]

        total_duration = 0.1 # 缩短为 0.1 秒
        num_points = 10

        for i in range(num_points + 1):
            alpha = i / num_points
            point = JointTrajectoryPoint()
            # Linear interpolation: from q_start to q_target
            interpolated_q = (1 - alpha) * q_start + alpha * q_target
            point.positions = interpolated_q.tolist()
            point.velocities = [0.0] * 14
            
            t = alpha * total_duration
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            
            traj.points.append(point)

        self.traj_pub.publish(traj)
        self.save_to_csv(traj)
        self.log_published_data(q_start, q_target)

        # last_published_q is no longer strictly needed for interpolation 
        # but we keep it updated for completeness
        self.last_published_q = q_target.copy()

    def log_published_data(self, q_start, q_target):
        """Log the published joint angles"""
        print(f"\n[Published to /dualArm_traj]")
        print(f"  Start q[0]: {q_start[0]:.4f} -> Target q[0]: {q_target[0]:.4f}")
        sys.stdout.flush()

    def save_to_csv(self, traj_msg):
        """Save JointTrajectory to temp.csv"""
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            tools_dir = os.path.join(os.path.dirname(current_dir), 'tools')
            
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
            print(f"[CSV] Saved to: {csv_path}")
        except Exception as e:
            print(f"[CSV] Failed to save: {e}")


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
    node = KeyJointTeleop()

    settings = termios.tcgetattr(sys.stdin)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            key = get_key(settings).lower()

            if key == '\x03':  # ctrl-c
                break

            if key:
                # Execution happens inside update_joint now
                node.update_joint(key)

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
        print("\nJoint teleop stopped.")


if __name__ == '__main__':
    main()
