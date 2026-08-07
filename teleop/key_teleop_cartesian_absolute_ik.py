#!/usr/bin/env python3
"""
Keyboard teleoperation for dual Panda robot: Cartesian target with LOCAL IK,
sending joint-space ABSOLUTE position commands.

This is a sibling of key_teleop_cartesian_absolute.py. The two share identical
key mapping and the same "last command pose as integration base" semantics.
The difference:

  - key_teleop_cartesian_absolute.py   -> sends TASK_SPACE_POSE (type=5); the
                                          safety controller runs its own IK.
  - key_teleop_cartesian_absolute_ik.py-> runs IK LOCALLY (same DH model the
                                          controller uses) and sends
                                          JOINT_POSITION (type=4), i.e. absolute
                                          joint angles.

The local IK uses panda_dh_kinematics.PandaDHKinematics, which reproduces the
controller's DH forward kinematics / Jacobian exactly (verified by an IK->FK
round-trip self-test), so the solved joint angles track the real robot.

Key Mapping:
Position:
  W/S: X axis (forward/backward)
  A/D: Y axis (left/right)
  Q/E: Z axis (up/down)
Rotation (small angle approx):
  J/L: Roll (X rotation)
  I/K: Pitch (Y rotation)
  U/O: Yaw (Z rotation)
Arm Selection:
  Z: Left arm
  X: Right arm
  B: Both arms
Gripper Control:
  N: Open gripper (0.08m)
  M: Close gripper (0.0m)
Step Size:
  [: Decrease step
  ]: Increase step

Publishes (selected via --target):
  --target safety (default):
    - Float64MultiArray to /dualarm_teleop_cmd (command_type=4 JOINT_POSITION),
      routed THROUGH the safety controller (clamping, collision avoidance, QP).
  --target direct:
    - sensor_msgs/JointState to /mj_left/joints_desired and /mj_right/joints_desired
      (the low-level joint impedance controller inputs), BYPASSING the safety
      controller so the IK output can be inspected unmodified. Republished at
      --direct-rate so the streaming impedance controller always has a fresh
      setpoint.
  - Float64MultiArray to /mj_{left,right}_gripper/grasp_desired (gripper force control)
Subscribes:
  - /joint_states for actual joint angles (IK seed + initial targets)
  - /ee_pose for actual end-effector pose feedback (display only)

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/key_teleop_cartesian_absolute_ik.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
import threading
import numpy as np
import argparse
import time
import os

# Add parent directory to path for imports (for direct execution)
if __name__ == '__main__':
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Import base teleop node
try:
    from teleop.base_teleop import BaseTeleopNode
    from teleop.utils.panda_dh_kinematics import (
        PandaDHKinematics, HOME_POSITION, matrix_to_quaternion_wxyz,
    )
except ImportError:
    # Fallback for direct execution
    from base_teleop import BaseTeleopNode
    from utils.panda_dh_kinematics import (
        PandaDHKinematics, HOME_POSITION, matrix_to_quaternion_wxyz,
    )


def quaternion_multiply(q1, q2):
    """Multiply two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quaternion_from_axis_angle(axis, angle):
    """Create quaternion [w, x, y, z] from axis-angle representation."""
    axis = axis / np.linalg.norm(axis)
    half_angle = angle / 2.0
    sin_half = np.sin(half_angle)
    return np.array([np.cos(half_angle), axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half])


def quaternion_to_array(q):
    """Convert quaternion [w, x, y, z] to [x, y, z, w] format for message."""
    return np.array([q[1], q[2], q[3], q[0]])


def quaternion_from_array(q_array):
    """Convert [x, y, z, w] to quaternion [w, x, y, z]."""
    return np.array([q_array[3], q_array[0], q_array[1], q_array[2]])


class KeyCartesianAbsoluteIKTeleop(BaseTeleopNode):
    """Keyboard teleop: Cartesian target integrated locally, solved via IK,
    published as absolute joint positions (JOINT_POSITION, type=4).

    Uses LAST COMMAND pose as integration base (accumulates deltas from previous
    command). On startup the integration base and joint targets are synced to the
    actual robot state so the first command never causes a jump.
    """

    # Command type constant for absolute joint position
    JOINT_POSITION = 4

    def __init__(self, step_position: float = 0.001, step_rotation: float = 0.01,
                 publish_target: str = 'safety', direct_rate: float = 50.0):
        """Initialize key Cartesian absolute (IK) teleop node.

        Args:
            step_position: Translation increment per keypress (m).
            step_rotation: Rotation increment per keypress (rad).
            publish_target: 'safety' -> send JOINT_POSITION (type=4) to
                /dualarm_teleop_cmd (routed through the safety controller).
                'direct' -> send JointState straight to the low-level
                /mj_{left,right}/joints_desired inputs of the joint impedance
                controller, BYPASSING the safety controller (for inspection).
            direct_rate: Republish rate (Hz) of the direct low-level command
                stream (only used when publish_target == 'direct').
        """
        super().__init__('key_cartesian_absolute_ik_teleop', publish_rate=100.0)

        assert publish_target in ('safety', 'direct'), \
            f"publish_target must be 'safety' or 'direct', got {publish_target!r}"
        self.publish_target = publish_target
        self.direct_rate = direct_rate

        self.step_position = step_position
        self.step_rotation = step_rotation
        self.joint_states_received = False
        self.ee_pose_received = False

        # Gripper control parameters
        self.gripper_max_width = 0.08  # Franka gripper max opening (m)
        self.gripper_min_width = 0.0   # Fully closed

        # Local DH kinematics (one model per arm, in the world frame).
        self.kinematics = {
            'left': PandaDHKinematics('left'),
            'right': PandaDHKinematics('right'),
        }

        # ACTUAL joint states from /joint_states (used as IK seed).
        self.current_joint_states = {
            'left': HOME_POSITION.copy(),
            'right': HOME_POSITION.copy(),
        }

        # Target joint positions produced by IK (what we publish).
        self.target_joint_states = {
            'left': HOME_POSITION.copy(),
            'right': HOME_POSITION.copy(),
        }

        # Target end-effector poses (integration base = last command pose).
        # Stored as position[3] + quaternion[w, x, y, z].
        # Initialized to the FK of the home configuration (matches the DH model).
        p0, q0 = self.kinematics['left'].forward_position_quat(HOME_POSITION)
        p1, q1 = self.kinematics['right'].forward_position_quat(HOME_POSITION)
        self.target_ee_poses = {
            'left': {'position': p0, 'quaternion': q0},
            'right': {'position': p1, 'quaternion': q1},
        }

        # ACTUAL end-effector poses from /ee_pose (display only).
        self.actual_ee_poses = {
            'left': {'position': p0.copy(), 'quaternion': q0.copy()},
            'right': {'position': p1.copy(), 'quaternion': q1.copy()},
        }

        # Last IK solve quality per arm (position/orientation error), for status.
        self.last_ik_error = {
            'left': (0.0, 0.0, True),
            'right': (0.0, 0.0, True),
        }

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.ee_pose_sub = self.create_subscription(
            Float64MultiArray, '/ee_pose', self.ee_pose_callback, 10
        )

        # Gripper control publishers (force control via gripper_action_bridge)
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/mj_left_gripper/grasp_desired', 10
        )
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/mj_right_gripper/grasp_desired', 10
        )

        # Direct low-level publishers: /mj_{left,right}/joints_desired are the
        # JointState inputs consumed by the joint impedance controller (the same
        # topics the safety controller publishes downstream). In 'direct' mode we
        # publish here ourselves, bypassing the safety controller entirely.
        self.left_joints_desired_pub = self.create_publisher(
            JointState, '/mj_left/joints_desired', 10
        )
        self.right_joints_desired_pub = self.create_publisher(
            JointState, '/mj_right/joints_desired', 10
        )

        # In direct mode, keep the low-level fed with a steady setpoint stream
        # (the impedance controller tracks the latest received position). Keys
        # update target_joint_states; this timer republishes them continuously.
        self._direct_timer = None
        if self.publish_target == 'direct':
            self._direct_timer = self.create_timer(
                1.0 / max(self.direct_rate, 1.0), self._direct_republish_cb)

        self.print_usage()
        mode = 'DIRECT -> /mj_{left,right}/joints_desired (bypasses safety controller)' \
            if self.publish_target == 'direct' else \
            'SAFETY  -> /dualarm_teleop_cmd (routed through safety controller)'
        print(f"[Publish mode: {mode}]\n", flush=True)

    def print_usage(self):
        msg = """
========================================
Task Space Absolute Teleop (LOCAL IK) for Dual Panda
  Cartesian target -> IK -> JOINT_POSITION (type=4)
(Using LAST COMMAND pose as integration base)
========================================
Position Control:
  W/S: X axis (forward/back)
  A/D: Y axis (left/right)
  Q/E: Z axis (up/down)

Rotation Control:
  J/L: Roll (X rotation)
  I/K: Pitch (Y rotation)
  U/O: Yaw (Z rotation)

Arm Selection:
  Z : Left arm
  X : Right arm
  B : Both arms

Gripper Control:
  N : Open gripper (0.08m)
  M : Close gripper (0.0m)

Step Size:
  [ : Decrease step
  ] : Increase step

Note: Each keypress integrates the Cartesian target from the last command,
      solves IK locally (same DH model as the controller), and publishes
      absolute joint angles. IK is seeded from the current joint state, so
      solutions stay smooth and nearby.

Ctrl-C to quit
========================================
"""
        print(msg, flush=True)

    # ----------------------- Subscribers -----------------------
    def joint_state_callback(self, msg):
        """Update actual joint states from /joint_states (IK seed)."""
        try:
            left_indices, right_indices = [], []
            for i in range(1, 8):
                ln, rn = f'mj_left_joint{i}', f'mj_right_joint{i}'
                if ln in msg.name:
                    left_indices.append(msg.name.index(ln))
                if rn in msg.name:
                    right_indices.append(msg.name.index(rn))

            if len(left_indices) == 7:
                self.current_joint_states['left'] = np.array([msg.position[i] for i in left_indices])
            if len(right_indices) == 7:
                self.current_joint_states['right'] = np.array([msg.position[i] for i in right_indices])

            if not self.joint_states_received and len(left_indices) == 7 and len(right_indices) == 7:
                self.joint_states_received = True
                # Sync integration base + joint targets to reality so the first
                # command does not cause a jump.
                for arm in ('left', 'right'):
                    self.target_joint_states[arm] = self.current_joint_states[arm].copy()
                    p, q = self.kinematics[arm].forward_position_quat(self.current_joint_states[arm])
                    self.target_ee_poses[arm]['position'] = p
                    self.target_ee_poses[arm]['quaternion'] = q
                print(f"\nJoint states received. IK targets synced to current pose. Ready!\n", flush=True)
                # In direct mode, immediately seed the low-level with the
                # current (synced) setpoint so the impedance controller holds the
                # present pose instead of its internal zero desired.
                if self.publish_target == 'direct':
                    self.publish_direct_joint_states(quiet=True)
        except Exception:
            pass

    def ee_pose_callback(self, msg):
        """Update actual end-effector poses from /ee_pose (display only)."""
        try:
            if len(msg.data) >= 14:
                self.actual_ee_poses['left']['position'] = np.array(msg.data[0:3])
                self.actual_ee_poses['left']['quaternion'] = quaternion_from_array(msg.data[3:7])
                self.actual_ee_poses['right']['position'] = np.array(msg.data[7:10])
                self.actual_ee_poses['right']['quaternion'] = quaternion_from_array(msg.data[10:14])
                if not self.ee_pose_received:
                    self.ee_pose_received = True
        except Exception:
            pass

    # ----------------------- Status -----------------------
    def status_callback(self):
        """Print status at 1Hz."""
        if not self.joint_states_received:
            print(f"\r[Waiting for /joint_states ...]{'':40}", end='', flush=True)
            return
        lp = self.target_ee_poses['left']['position']
        rp = self.target_ee_poses['right']['position']
        print(f"\r[{self.selected_arm.upper()}|{self.publish_target}] "
              f"pos={self.step_position:.4f}m rot={self.step_rotation:.4f}rad | "
              f"L_target:[{lp[0]:7.3f},{lp[1]:7.3f},{lp[2]:7.3f}] "
              f"R_target:[{rp[0]:7.3f},{rp[1]:7.3f},{rp[2]:7.3f}]   ",
              end='', flush=True)

    # ----------------------- Key handling -----------------------
    def update_from_key(self, key: str) -> bool:
        """Integrate Cartesian target from the key, solve IK, publish joints."""
        # Arm selection
        if key == 'z':
            self._switch_arm('left')
            return False
        elif key == 'x':
            self._switch_arm('right')
            return False
        elif key == 'b':
            self._switch_arm('both')
            return False

        # Step size adjustment
        elif key == '[':
            self.step_position = max(0.0001, self.step_position * 0.8)
            self.step_rotation = max(0.001, self.step_rotation * 0.8)
            print(f"\n[Step size: pos={self.step_position:.4f}m, rot={self.step_rotation:.4f}rad]", flush=True)
            return False
        elif key == ']':
            self.step_position = min(0.01, self.step_position * 1.25)
            self.step_rotation = min(0.1, self.step_rotation * 1.25)
            print(f"\n[Step size: pos={self.step_position:.4f}m, rot={self.step_rotation:.4f}rad]", flush=True)
            return False

        # Gripper control (does not affect arm targets)
        elif key == 'n':
            self.control_gripper(self.gripper_max_width)
            return True
        elif key == 'm':
            self.control_gripper(self.gripper_min_width)
            return True

        # Position/rotation deltas
        pos_delta = np.zeros(3)
        rot_delta = np.zeros(3)
        updated = False

        # Position: W/S (X), A/D (Y), Q/E (Z)
        pos_map = {'w': (0, 1), 's': (0, -1), 'a': (1, 1), 'd': (1, -1), 'q': (2, 1), 'e': (2, -1)}
        if key in pos_map:
            axis, sign = pos_map[key]
            pos_delta[axis] = sign * self.step_position
            updated = True

        # Rotation: J/L (roll), I/K (pitch), U/O (yaw)
        rot_map = {'j': (0, 1), 'l': (0, -1), 'i': (1, 1), 'k': (1, -1), 'u': (2, 1), 'o': (2, -1)}
        if key in rot_map:
            axis, sign = rot_map[key]
            rot_delta[axis] = sign * self.step_rotation
            updated = True

        if not updated:
            return False

        # Integrate Cartesian target (last command as base) for selected arm(s).
        if self.selected_arm in ('left', 'both'):
            self._integrate_target('left', pos_delta, rot_delta)
            self._solve_and_store_ik('left')
        if self.selected_arm in ('right', 'both'):
            self._integrate_target('right', pos_delta, rot_delta)
            self._solve_and_store_ik('right')

        self.publish_teleop_command()
        return True

    def _switch_arm(self, arm: str):
        if self.selected_arm != arm:
            self.set_selected_arm(arm)
            print(f"\n[Switched to {arm.upper()} arm]", flush=True)

    def _integrate_target(self, arm: str, pos_delta: np.ndarray, rot_delta: np.ndarray):
        """Apply position + rotation deltas to the target pose (last command base)."""
        cur = self.target_ee_poses[arm]
        cur['position'] = cur['position'] + pos_delta
        if np.linalg.norm(rot_delta) > 0:
            angle = np.linalg.norm(rot_delta)
            axis = rot_delta / angle
            delta_q = quaternion_from_axis_angle(axis, angle)
            new_q = quaternion_multiply(delta_q, cur['quaternion'])
            cur['quaternion'] = new_q / np.linalg.norm(new_q)

    def _solve_and_store_ik(self, arm: str):
        """Solve IK for the arm's target pose and store the resulting joints.

        Seeds from the actual current joint state so the solution is the nearest
        smooth one. Records solve quality for status display.
        """
        target = self.target_ee_poses[arm]
        seed = self.current_joint_states[arm]
        q_sol, perr, rerr, ok = self.kinematics[arm].inverse_kinematics(
            target['position'], target['quaternion'], q_seed=seed,
            tol=1e-6, max_iter=100)
        self.target_joint_states[arm] = q_sol
        self.last_ik_error[arm] = (perr, rerr, ok)
        if not ok:
            print(f"\n[{arm.upper()} IK WARNING] did not fully converge: "
                  f"pos_err={perr:.2e} m, rot_err={rerr:.2e} rad", flush=True)

    # ----------------------- Publish -----------------------
    def publish_teleop_command(self):
        """Dispatch the solved joint targets to the configured publish target."""
        if self.publish_target == 'direct':
            self.publish_direct_joint_states(quiet=False)
        else:
            self.publish_safety_teleop_command()

    def publish_safety_teleop_command(self):
        """(safety path) Publish absolute joints as /dualarm_teleop_cmd (type=4)."""
        msg = self.create_teleop_command(
            arm_selector=self.get_arm_selector(),
            command_type=self.JOINT_POSITION,
            left_joint_delta=self.target_joint_states['left'],
            right_joint_delta=self.target_joint_states['right'],
            allow_safety_violation=False
        )
        self.teleop_pub.publish(msg)

        l = self.target_joint_states['left'].round(3)
        r = self.target_joint_states['right'].round(3)
        print(f"\n[Publish JOINT_POSITION {self.selected_arm}] "
              f"L:[{l[0]:.2f},{l[1]:.2f},{l[2]:.2f},{l[3]:.2f},{l[4]:.2f},{l[5]:.2f},{l[6]:.2f}] "
              f"R:[{r[0]:.2f},{r[1]:.2f},{r[2]:.2f},{r[3]:.2f},{r[4]:.2f},{r[5]:.2f},{r[6]:.2f}]")

    def publish_direct_joint_states(self, quiet: bool = False):
        """(direct path) Publish JointState to the low-level impedance controller
        inputs /mj_{left,right}/joints_desired, bypassing the safety controller.

        Both arms' current target_joint_states are sent every call so the
        streaming impedance controller always has a valid, fresh setpoint
        (mirrors what the safety controller publishes downstream). Velocities are
        sent as zero (pure position setpoint, matches the single-arm bypass).

        Args:
            quiet: when True (timer republish), suppress the per-call log line.
        """
        stamp = self.get_clock().now().to_msg()
        for side, pub in (('left', self.left_joints_desired_pub),
                          ('right', self.right_joints_desired_pub)):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [f'mj_{side}_joint{i + 1}' for i in range(7)]
            msg.position = self.target_joint_states[side].tolist()
            msg.velocity = [0.0] * 7
            pub.publish(msg)

        if not quiet:
            l = self.target_joint_states['left'].round(3)
            r = self.target_joint_states['right'].round(3)
            print(f"\n[Publish DIRECT JointState {self.selected_arm}] "
                  f"L:[{l[0]:.2f},{l[1]:.2f},{l[2]:.2f},{l[3]:.2f},{l[4]:.2f},{l[5]:.2f},{l[6]:.2f}] "
                  f"R:[{r[0]:.2f},{r[1]:.2f},{r[2]:.2f},{r[3]:.2f},{r[4]:.2f},{r[5]:.2f},{r[6]:.2f}]")

    def _direct_republish_cb(self):
        """Timer callback that keeps the low-level setpoint stream alive in
        direct mode (silent republish of the latest solved joints)."""
        if self.joint_states_received:
            self.publish_direct_joint_states(quiet=True)

    # ----------------------- Gripper -----------------------
    def control_gripper(self, width: float):
        """Control gripper with force-based grasp (gripper_action_bridge)."""
        msg = Float64MultiArray()
        msg.data = [width, 0.05, 30.0, 0.005, 0.005]  # width, speed, force, eps_in, eps_out

        gripper_name = ""
        if self.selected_arm == 'left':
            self.left_gripper_pub.publish(msg)
            gripper_name = "LEFT"
        elif self.selected_arm == 'right':
            self.right_gripper_pub.publish(msg)
            gripper_name = "RIGHT"
        elif self.selected_arm == 'both':
            self.left_gripper_pub.publish(msg)
            self.right_gripper_pub.publish(msg)
            gripper_name = "BOTH"

        action = "OPEN" if width > 0.04 else "CLOSE"
        print(f"\n[{gripper_name} GRIPPER {action}] Width: {width:.3f}m | Speed: 0.05m/s | Force: 30.0N", flush=True)


def get_key(settings):
    """Non-blocking key input."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Cartesian target keyboard teleop with local IK -> absolute joint positions')
    parser.add_argument('--step-position', type=float, default=0.001,
                        help='Position increment per keypress (m)')
    parser.add_argument('--step-rotation', type=float, default=0.01,
                        help='Rotation increment per keypress (rad)')
    parser.add_argument('--arm', type=str, choices=['left', 'right', 'both'],
                        default='left', help='Initial arm selection')
    parser.add_argument('--target', type=str, choices=['safety', 'direct'],
                        default='safety',
                        help="Publish target: 'safety' -> /dualarm_teleop_cmd via the "
                             "safety controller (default); 'direct' -> JointState straight "
                             "to /mj_{left,right}/joints_desired (low-level impedance "
                             "controller), bypassing the safety controller for inspection.")
    parser.add_argument('--direct-rate', type=float, default=50.0,
                        help='Republish rate (Hz) of the direct low-level command stream '
                             '(only used with --target direct). Default: 50.')
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)
    node = KeyCartesianAbsoluteIKTeleop(
        step_position=parser_args.step_position,
        step_rotation=parser_args.step_rotation,
        publish_target=parser_args.target,
        direct_rate=parser_args.direct_rate,
    )
    node.set_selected_arm(parser_args.arm)

    # Create status timer
    status_timer = node.create_timer(1.0, node.status_callback)

    settings = termios.tcgetattr(sys.stdin)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            key = get_key(settings).lower()

            if key == '\x03':  # ctrl-c
                break

            if key:
                node.update_from_key(key)

            time.sleep(0.01)

    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nCartesian absolute (IK) teleop stopped.")


if __name__ == '__main__':
    main()
