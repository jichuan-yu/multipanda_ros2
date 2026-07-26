#!/usr/bin/env python3
"""
Keyboard teleoperation for dual Panda robot — Cartesian integration with on-script IK.

This script is a variant of key_teleop_cartesian_absolute.py. The original sends
absolute task-space (Cartesian) poses to /dualarm_teleop_cmd (command_type=5
TASK_SPACE_POSE) and lets the safety controller perform the IK internally (via a
damped-least-squares Jacobian pseudo-inverse).

This variant instead solves the IK IN THE SCRIPT and publishes absolute JOINT-SPACE
position commands (command_type=4 JOINT_POSITION). The key mapping, Cartesian
integration logic, and feel are identical to the original; only the output is joint
positions instead of a Cartesian pose.

The IK method is identical to the controller's computeJointFromTaskSpace():
    J# = J^T (J J^T + lambda^2 I)^-1,   lambda = 0.01
iterated to convergence against the pose error
    [Delta_p ; rotvec(R_target R_current^T)].
See panda_kinematics.py (a faithful port of PandaRobot in robot_kinematics.cpp).

Key Mapping (same as key_teleop_cartesian_absolute.py):
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

Publishes:
  - Float64MultiArray to /dualarm_teleop_cmd (command_type=4 JOINT_POSITION)
  - Float64MultiArray to /mj_{left,right}_gripper/grasp_desired (gripper force control)
Subscribes:
  - /joint_states for the actual joint configuration used as the IK linearization point
    (the controller does the same: it builds T_current and the Jacobian at the actual q)

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
if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

# Import base teleop node
try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    # Fallback for direct execution
    from base_teleop import BaseTeleopNode

# Import Panda kinematics + IK (faithful port of the controller's kinematics/IK)
try:
    from teleop.panda_kinematics import (
        PandaKinematics,
        solve_ik_dls,
        normalize_joint_angles,
    )
except ImportError:
    from panda_kinematics import PandaKinematics, solve_ik_dls, normalize_joint_angles


def quaternion_multiply(q1, q2):
    """Multiply two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quaternion_from_axis_angle(axis, angle):
    """Create quaternion from axis-angle representation."""
    axis = axis / np.linalg.norm(axis)
    half_angle = angle / 2.0
    sin_half = np.sin(half_angle)
    return np.array(
        [np.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half]
    )


class KeyCartesianAbsoluteIKTeleop(BaseTeleopNode):
    """Keyboard teleop: Cartesian integration + on-script IK -> joint position commands.

    Cartesian targets are integrated exactly like key_teleop_cartesian_absolute
    (LAST COMMAND position as base, deltas accumulate from the previous command).
    Before publishing, each selected arm's target Cartesian pose is converted to an
    absolute joint configuration with the DLS IK solver, then a JOINT_POSITION command
    is sent. Unselected arms hold their current actual joint configuration.
    """

    # Absolute joint position command (see teleop.md / base_teleop.py)
    JOINT_POSITION = 4

    # Robot base poses (defaults of dual_arm_safe_controller_sim.cpp)
    LEFT_BASE_XYZ = (0.0, 0.26, 0.0)
    RIGHT_BASE_XYZ = (0.0, -0.26, 0.0)

    def __init__(self, step_position: float = 0.001, step_rotation: float = 0.01):
        """Initialize key Cartesian absolute IK teleop node."""
        super().__init__("key_cartesian_absolute_ik_teleop", publish_rate=100.0)

        self.step_position = step_position
        self.step_rotation = step_rotation
        self.joint_state_received = False

        # Gripper control parameters
        self.gripper_max_width = 0.08  # Franka gripper max opening (m)
        self.gripper_min_width = 0.0  # Fully closed

        # Kinematics models (base configured to match the controller's robot bases)
        self.kin = {
            "left": PandaKinematics(),
            "right": PandaKinematics(),
        }
        self.kin["left"].set_base(self.LEFT_BASE_XYZ)
        self.kin["right"].set_base(self.RIGHT_BASE_XYZ)

        # DLS IK parameters (lambda matches the controller's computeJointFromTaskSpace)
        self.ik_damping = 0.01
        self.ik_max_iter = 50
        self.ik_tol = 1e-4

        # Actual joint configurations from /joint_states (used as the IK seed / base,
        # exactly as the controller builds T_current and the Jacobian at the actual q)
        self.current_q = {
            "left": self.HOME_POSITION.copy(),
            "right": self.HOME_POSITION.copy(),
        }

        # Cartesian integration targets (LAST COMMAND pose as base).
        # Default home position matches key_teleop_cartesian_absolute.py.
        home_quat = np.array([0.0, 1.0, 0.0, 0.0])  # [w, x, y, z] - 180 deg X rotation
        self.target_ee_poses = {
            "left": {"position": np.array([0.307, 0.26, 0.487]), "quaternion": home_quat.copy()},
            "right": {"position": np.array([0.307, -0.26, 0.487]), "quaternion": home_quat.copy()},
        }

        # Subscriber for actual joint states (IK linearization point)
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Gripper control publishers (for gripper_action_bridge - force control)
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, "/mj_left_gripper/grasp_desired", 10
        )
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, "/mj_right_gripper/grasp_desired", 10
        )

        self.print_usage()

    def print_usage(self):
        msg = """
========================================
Task Space Absolute Teleop for Dual Panda
(Cartesian integration + on-script IK -> JOINT_POSITION)
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

Note: Each keypress integrates Cartesian deltas from the LAST COMMAND
      pose, then solves IK to obtain absolute joint positions.
      First command uses the default home position.

Ctrl-C to quit
========================================
"""
        print(msg, flush=True)

    def joint_state_callback(self, msg):
        """Update actual joint configurations from /joint_states (IK seed)."""
        try:
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            updated = True
            for arm, prefix in (("left", "mj_left_joint"), ("right", "mj_right_joint")):
                q = np.zeros(7)
                ok = True
                for j in range(7):
                    jname = f"{prefix}{j + 1}"
                    idx = name_to_idx.get(jname, -1)
                    if idx < 0 or idx >= len(msg.position):
                        ok = False
                        break
                    q[j] = msg.position[idx]
                if ok:
                    self.current_q[arm] = normalize_joint_angles(q)
                else:
                    updated = False

            if updated and not self.joint_state_received:
                self.joint_state_received = True
                print(
                    "\nJoint states received. Ready for control (IK enabled)!\n",
                    flush=True,
                )
        except Exception:
            pass

    def status_callback(self):
        """Print status at 1Hz."""
        if self.joint_state_received:
            left_target_pos = self.target_ee_poses["left"]["position"]
            right_target_pos = self.target_ee_poses["right"]["position"]
            print(
                f"\r[{self.selected_arm.upper()}] Pos: {self.step_position:.4f}m | "
                f"Rot: {self.step_rotation:.4f}rad | "
                f"L_target: [{left_target_pos[0]:7.3f}, {left_target_pos[1]:7.3f}, "
                f"{left_target_pos[2]:7.3f}] | "
                f"R_target: [{right_target_pos[0]:7.3f}, {right_target_pos[1]:7.3f}, "
                f"{right_target_pos[2]:7.3f}]   ",
                end="",
                flush=True,
            )

    def update_from_key(self, key: str) -> bool:
        """Update pose based on key input, using LAST COMMAND position as base."""
        # Arm selection
        if key == "z":
            if self.selected_arm != "left":
                self.set_selected_arm("left")
                print("\n[Switched to LEFT arm]", flush=True)
            return False
        elif key == "x":
            if self.selected_arm != "right":
                self.set_selected_arm("right")
                print("\n[Switched to RIGHT arm]", flush=True)
            return False
        elif key == "b":
            if self.selected_arm != "both":
                self.set_selected_arm("both")
                print("\n[Switched to BOTH arms]", flush=True)
            return False

        # Step size adjustment
        elif key == "[":
            self.step_position = max(0.0001, self.step_position * 0.8)
            self.step_rotation = max(0.001, self.step_rotation * 0.8)
            print(
                f"\n[Step size: pos={self.step_position:.4f}m, "
                f"rot={self.step_rotation:.4f}rad]",
                flush=True,
            )
            return False
        elif key == "]":
            self.step_position = min(0.01, self.step_position * 1.25)
            self.step_rotation = min(0.1, self.step_rotation * 1.25)
            print(
                f"\n[Step size: pos={self.step_position:.4f}m, "
                f"rot={self.step_rotation:.4f}rad]",
                flush=True,
            )
            return False

        # Position/rotation deltas
        pos_delta = np.zeros(3)
        rot_delta = np.zeros(3)
        updated = False

        # Position: W/S (X), A/D (Y), Q/E (Z)
        if key == "w":
            pos_delta[0] = self.step_position
            updated = True
        elif key == "s":
            pos_delta[0] = -self.step_position
            updated = True
        elif key == "a":
            pos_delta[1] = self.step_position
            updated = True
        elif key == "d":
            pos_delta[1] = -self.step_position
            updated = True
        elif key == "q":
            pos_delta[2] = self.step_position
            updated = True
        elif key == "e":
            pos_delta[2] = -self.step_position
            updated = True

        # Rotation: J/L (roll), I/K (pitch), U/O (yaw)
        elif key == "j":
            rot_delta[0] = self.step_rotation  # Roll
            updated = True
        elif key == "l":
            rot_delta[0] = -self.step_rotation
            updated = True
        elif key == "i":
            rot_delta[1] = self.step_rotation  # Pitch
            updated = True
        elif key == "k":
            rot_delta[1] = -self.step_rotation
            updated = True
        elif key == "u":
            rot_delta[2] = self.step_rotation  # Yaw
            updated = True
        elif key == "o":
            rot_delta[2] = -self.step_rotation
            updated = True

        # Gripper control: N (open), M (close)
        elif key == "n":
            self.control_gripper(self.gripper_max_width)  # Open gripper
            return True
        elif key == "m":
            self.control_gripper(self.gripper_min_width)  # Close gripper
            return True

        if updated:
            # Integrate Cartesian deltas from the LAST COMMAND pose (same as original)
            for arm in ("left", "right"):
                if (arm == "left" and self.selected_arm in ("left", "both")) or (
                    arm == "right" and self.selected_arm in ("right", "both")
                ):
                    target_pos = self.target_ee_poses[arm]["position"] + pos_delta

                    if np.linalg.norm(rot_delta) > 0:
                        rot_angle = np.linalg.norm(rot_delta)
                        rot_axis = rot_delta / rot_angle
                        delta_quat = quaternion_from_axis_angle(rot_axis, rot_angle)
                        target_quat = quaternion_multiply(
                            delta_quat, self.target_ee_poses[arm]["quaternion"]
                        )
                        target_quat /= np.linalg.norm(target_quat)
                    else:
                        target_quat = self.target_ee_poses[arm]["quaternion"].copy()

                    self.target_ee_poses[arm]["position"] = target_pos
                    self.target_ee_poses[arm]["quaternion"] = target_quat

            self.publish_teleop_command()
            return True

        return False

    def solve_arm_ik(self, arm: str) -> np.ndarray:
        """Solve IK for one arm's target Cartesian pose from the actual configuration.

        Returns the absolute joint configuration (clamped to joint limits). On failure
        (e.g. target unreachable), returns the best solution found.
        """
        target = self.target_ee_poses[arm]
        q_seed = self.current_q[arm]
        q_sol, err = solve_ik_dls(
            self.kin[arm],
            target["position"],
            target["quaternion"],
            q_seed,
            damping=self.ik_damping,
            max_iter=self.ik_max_iter,
            tol=self.ik_tol,
        )
        if err > 1e-2:
            print(
                f"\n[IK {arm}] large residual err={err:.4f} (target may be unreachable); "
                f"sending best-effort solution",
                flush=True,
            )
        return q_sol

    def publish_teleop_command(self):
        """Solve IK for selected arms and publish a JOINT_POSITION command.

        Selected arms receive their IK-solved absolute joint targets; unselected arms
        hold their current actual configuration (the controller only applies the
        selected arm's slots anyway). Message format: [left7, right7, cmd_type, ...].
        """
        left_q = self.current_q["left"]
        right_q = self.current_q["right"]

        if self.selected_arm in ("left", "both"):
            left_q = self.solve_arm_ik("left")
        if self.selected_arm in ("right", "both"):
            right_q = self.solve_arm_ik("right")

        msg = self.create_teleop_command(
            arm_selector=self.get_arm_selector(),
            command_type=self.JOINT_POSITION,
            left_joint_delta=left_q,
            right_joint_delta=right_q,
            allow_safety_violation=False,
        )

        self.teleop_pub.publish(msg)

        # Print published data
        print(f"\n[Publishing JOINT_POSITION ({self.selected_arm})] data: {msg.data}")

    def control_gripper(self, width: float):
        """
        Control gripper with force-based grasp.

        Publishes to gripper_action_bridge topics which handle the actual
        gripper control in the simulation using force control parameters.

        Args:
            width: Desired gripper width in meters (0.0 = closed, 0.08 = fully open)
        """
        msg = Float64MultiArray()
        msg.data = [
            width,  # Target width (m)
            0.05,  # Speed (m/s)
            30.0,  # Force (N)
            0.005,  # epsilon_inner (m)
            0.005,  # epsilon_outer (m)
        ]

        gripper_name = ""
        if self.selected_arm == "left":
            self.left_gripper_pub.publish(msg)
            gripper_name = "LEFT"
        elif self.selected_arm == "right":
            self.right_gripper_pub.publish(msg)
            gripper_name = "RIGHT"
        elif self.selected_arm == "both":
            self.left_gripper_pub.publish(msg)
            self.right_gripper_pub.publish(msg)
            gripper_name = "BOTH"

        action = "OPEN" if width > 0.04 else "CLOSE"
        print(
            f"\n[{gripper_name} GRIPPER {action}] Width: {width:.3f}m | "
            f"Speed: 0.05m/s | Force: 30.0N",
            flush=True,
        )


def get_key(settings):
    """Non-blocking key input."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Cartesian keyboard teleop with on-script IK -> joint position commands"
    )
    parser.add_argument(
        "--step-position", type=float, default=0.001, help="Position increment per keypress (m)"
    )
    parser.add_argument(
        "--step-rotation", type=float, default=0.01, help="Rotation increment per keypress (rad)"
    )
    parser.add_argument(
        "--arm",
        type=str,
        choices=["left", "right", "both"],
        default="left",
        help="Initial arm selection",
    )
    parser_args = parser.parse_args(args)

    rclpy.init(args=args)
    node = KeyCartesianAbsoluteIKTeleop(
        step_position=parser_args.step_position, step_rotation=parser_args.step_rotation
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

            if key == "\x03":  # ctrl-c
                break

            if key:
                node.update_from_key(key)

            time.sleep(0.01)

    except Exception as e:
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nCartesian absolute IK teleop stopped.")


if __name__ == "__main__":
    main()
