#!/usr/bin/env python3
"""
Automated absolute task-space teleop with LOCAL IK for the dual Panda.

IK-enabled sibling of auto_absolute_teleop.py (which publishes TASK_SPACE_POSE
type=5 and lets the safety controller run its own IK). This one runs IK LOCALLY
(same DH model as the controller, panda_dh_kinematics.py) and publishes absolute
joint angles, conceptually identical to key_teleop_cartesian_absolute_ik.py but
with a scripted trajectory instead of keyboard input.

Profile (emulates the manual relative-error experiment "双臂同时向前 0.01m"):
  1. wait for /joint_states
  2. anchor the absolute Cartesian target to the CURRENT true EE pose via FK
  3. one no-op anchor command (delta ~ 0) to sync the pipelines
  4. N steps forward (step m every interval s) -> hold -> N steps back -> hold

Each step: Cartesian delta on the LAST COMMAND pose -> local IK (seeded from the
current joint state) -> absolute joints, published as:
  --target safety (default): Float64MultiArray to /dualarm_teleop_cmd
                             (JOINT_POSITION type=4) via the safety controller.
  --target direct          : JointState to /mj_{left,right}/joints_desired,
                             bypassing the safety controller, kept alive at
                             --direct-rate Hz.

Re-publishing the same ABSOLUTE target is idempotent (JOINT_POSITION keeps q_r
at the same goal) and is NOT the harmful TASK_SPACE_POSE re-arm case discussed
in auto_absolute_teleop.py. The keep-alive only guards a lowered
teleop_cmd_timeout_ (default 999 s, dual_arm_safe_controller_sim.h:143).

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/auto_absolute_teleop_ik.py --step 0.001 --steps 10 --interval 2.0 --hold 5.0
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
import sensor_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from teleop.base_teleop import BaseTeleopNode
    from teleop.utils.panda_dh_kinematics import PandaDHKinematics, HOME_POSITION
except ImportError:
    from base_teleop import BaseTeleopNode
    from utils.panda_dh_kinematics import PandaDHKinematics, HOME_POSITION


def quaternion_multiply(q1, q2):
    """Multiply two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quaternion_from_axis_angle(axis, angle):
    """Create quaternion [w, x, y, z] from axis-angle representation."""
    axis = axis / np.linalg.norm(axis)
    half_angle = angle / 2.0
    sin_half = np.sin(half_angle)
    return np.array([np.cos(half_angle), axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half])


class AutoAbsoluteTeleopIK(BaseTeleopNode):
    """Scripted twin-arm absolute task-space teleop with LOCAL IK.

    Publishes absolute joint targets (JOINT_POSITION type=4 -> safety controller,
    or direct JointState -> low-level impedance controllers). The Cartesian sweep
    is anchored on the real initial FK and accumulated on the LAST COMMAND pose,
    exactly like the keyboard IK sibling.
    """

    # Absolute joint position command type
    JOINT_POSITION = 4

    def __init__(self, step=0.002, steps=5, interval=1.0, hold=6.0,
                 axis='x', sign=1.0, back=True, arm='both',
                 publish_target='safety', direct_rate=50.0,
                 keepalive_period=1.0, allow_safety_violation=False):
        super().__init__('auto_absolute_teleop_ik', publish_rate=100.0)

        assert publish_target in ('safety', 'direct'), \
            f"publish_target must be 'safety' or 'direct', got {publish_target!r}"
        assert arm in ('left', 'right', 'both'), f"bad arm {arm!r}"

        self.step = step
        self.steps = steps
        self.interval = interval
        self.hold = hold
        self.axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        self.direction = sign          # +1 forward, -1 backward
        self.back = back
        self.publish_target = publish_target
        self.direct_rate = direct_rate
        self.keepalive_period = keepalive_period
        self.allow_safety_violation = allow_safety_violation  # data[16] -> safety controller

        self.joint_states_received = False
        self.ee_pose_received = False

        # Local DH kinematics (one model per arm, world frame) — EXACTLY the
        # controller's FK/IK model, so IK solutions track the real robot.
        self.kinematics = {
            'left': PandaDHKinematics('left'),
            'right': PandaDHKinematics('right'),
        }

        # ACTUAL joint states (IK seed + initial sync base).
        self.current_joint_states = {
            'left': HOME_POSITION.copy(),
            'right': HOME_POSITION.copy(),
        }

        # Absolute joints solved by IK (what we publish).
        self.target_joint_states = {
            'left': HOME_POSITION.copy(),
            'right': HOME_POSITION.copy(),
        }

        # Cartesian integration base (LAST COMMAND pose), same semantics as the
        # keyboard IK sibling. position[3] + quaternion[w, x, y, z].
        p0, q0 = self.kinematics['left'].forward_position_quat(HOME_POSITION)
        p1, q1 = self.kinematics['right'].forward_position_quat(HOME_POSITION)
        self.target_ee_poses = {
            'left': {'position': p0, 'quaternion': q0},
            'right': {'position': p1, 'quaternion': q1},
        }

        # ACTUAL EE poses from /ee_pose (display only).
        self.actual_ee_poses = {
            'left': {'position': p0.copy(), 'quaternion': q0.copy()},
            'right': {'position': p1.copy(), 'quaternion': q1.copy()},
        }

        # Last IK solve quality per arm (pos_err, rot_err, converged).
        self.last_ik_error = {
            'left': (0.0, 0.0, True),
            'right': (0.0, 0.0, True),
        }

        # Subscribers. /joint_states is best-effort depth 1 (same as
        # auto_absolute_teleop.py); /ee_pose is display only.
        js_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.js_sub = self.create_subscription(
            sensor_msgs.msg.JointState, '/joint_states', self.joint_state_callback, js_qos)
        self.ee_pose_sub = self.create_subscription(
            Float64MultiArray, '/ee_pose', self.ee_pose_callback, 10)

        # Direct low-level publishers (only used with --target direct): JointState
        # streams to the joint impedance controller inputs, bypassing mprc.
        self.left_joints_desired_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/mj_left/joints_desired', 10)
        self.right_joints_desired_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/mj_right/joints_desired', 10)

        # In direct mode, keep the streaming setpoint alive at --direct-rate Hz.
        self._direct_timer = None
        if self.publish_target == 'direct':
            self._direct_timer = self.create_timer(
                1.0 / max(self.direct_rate, 1.0), self._direct_republish_cb)

        self.set_selected_arm(arm)
        mode = ('DIRECT -> /mj_{left,right}/joints_desired (bypasses safety controller)'
                if self.publish_target == 'direct' else
                'SAFETY  -> /dualarm_teleop_cmd (JOINT_POSITION type=4 via safety controller)')
        print(f'\n[Publish mode: {mode}]', flush=True)

    # ----------------------- Subscribers -----------------------
    def joint_state_callback(self, msg):
        """/joint_states -> actual joints (IK seed); sync targets on first frame."""
        try:
            left_indices = []
            right_indices = []
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
                for arm in ('left', 'right'):
                    self.target_joint_states[arm] = self.current_joint_states[arm].copy()
                    p, q = self.kinematics[arm].forward_position_quat(self.current_joint_states[arm])
                    self.target_ee_poses[arm]['position'] = p
                    self.target_ee_poses[arm]['quaternion'] = q
                print('\nJoint states received. IK targets synced to current pose. Ready!\n',
                      flush=True)
                # In direct mode, immediately seed the low-level with the current
                # (synced) setpoint so the impedance controller holds the pose.
                if self.publish_target == 'direct':
                    self.publish_direct_joint_states(quiet=True)
        except Exception:
            pass

    def ee_pose_callback(self, msg):
        """/ee_pose -> actual EE poses (display only)."""
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

    # ----------------------- Cartesian (de)integration -----------------------
    def _integrate_target(self, arm, pos_delta, rot_delta):
        """Apply position + rotation deltas to the target pose (LAST COMMAND base)."""
        cur = self.target_ee_poses[arm]
        cur['position'] = cur['position'] + pos_delta
        if np.linalg.norm(rot_delta) > 0:
            angle = np.linalg.norm(rot_delta)
            axis = rot_delta / angle
            delta_q = quaternion_from_axis_angle(axis, angle)
            cur['quaternion'] = quaternion_multiply(delta_q, cur['quaternion'])
            cur['quaternion'] = cur['quaternion'] / np.linalg.norm(cur['quaternion'])

    def _solve_and_store_ik(self, arm):
        """Solve IK for the arm's target pose, seeded from the actual joints.

        Seeding from the current joint state keeps the solution smooth and
        nearby (same approach as the keyboard IK sibling). Records quality.
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
    def _publish_current(self, quiet=False):
        """Dispatch current absolute joint targets to the configured target."""
        if self.publish_target == 'direct':
            self.publish_direct_joint_states(quiet=quiet)
        else:
            self.publish_safety_teleop_command(quiet=quiet)

    def publish_safety_teleop_command(self, quiet=False):
        """(safety path) Publish absolute joints as /dualarm_teleop_cmd (type=4)."""
        msg = self.create_teleop_command(
            arm_selector=self.get_arm_selector(),
            command_type=self.JOINT_POSITION,
            left_joint_delta=self.target_joint_states['left'],
            right_joint_delta=self.target_joint_states['right'],
            allow_safety_violation=self.allow_safety_violation,
        )
        self.teleop_pub.publish(msg)
        if not quiet:
            l = self.target_joint_states['left'].round(3)
            r = self.target_joint_states['right'].round(3)
            print(f"\n[Publish JOINT_POSITION {self.selected_arm}] "
                  f"L:[{l[0]:.2f},{l[1]:.2f},{l[2]:.2f},{l[3]:.2f},{l[4]:.2f},{l[5]:.2f},{l[6]:.2f}] "
                  f"R:[{r[0]:.2f},{r[1]:.2f},{r[2]:.2f},{r[3]:.2f},{r[4]:.2f},{r[5]:.2f},{r[6]:.2f}]",
                  flush=True)

    def publish_direct_joint_states(self, quiet=False):
        """(direct path) Publish JointState to /mj_{left,right}/joints_desired.

        Velocities are zero (pure position setpoint, same profile the safety
        controller publishes downstream).
        """
        stamp = self.get_clock().now().to_msg()
        for side, pub in (('left', self.left_joints_desired_pub),
                          ('right', self.right_joints_desired_pub)):
            msg = sensor_msgs.msg.JointState()
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
                  f"R:[{r[0]:.2f},{r[1]:.2f},{r[2]:.2f},{r[3]:.2f},{r[4]:.2f},{r[5]:.2f},{r[6]:.2f}]",
                  flush=True)

    def _direct_republish_cb(self):
        """Timer keep-alive of the direct low-level setpoint stream (silent)."""
        if self.joint_states_received:
            self.publish_direct_joint_states(quiet=True)

    # --------------------------- helpers ---------------------------
    def _spin(self, sec):
        """Spin for wall-time sec (keeps subscriptions alive)."""
        deadline = time.time() + sec
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def _spin_keepalive(self, sec):
        """Spin for `sec` s while silently re-publishing the current absolute
        target so the safety controller never sees a teleop gap.

        Idempotent for type=4 (absolute joints), so it never re-arms the
        controller's task-space loop; it only guards a lowered teleop timeout
        (default 999 s).
        """
        next_ka = time.time() + self.keepalive_period
        deadline = time.time() + sec
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.time() >= next_ka:
                self._publish_current(quiet=True)
                next_ka = time.time() + self.keepalive_period
            time.sleep(0.02)
    def _move_once(self, direction):
        """Apply one Cartesian delta on the LAST COMMAND pose for the selected
        arm(s), solve local IK, and publish the resulting absolute joints."""
        pos_delta = np.zeros(3)
        pos_delta[self.axis_idx] = direction * self.step
        if self.selected_arm == 'both':
            arms = ('left', 'right')
        else:
            arms = (self.selected_arm,)
        for arm in arms:
            self._integrate_target(arm, pos_delta, np.zeros(3))
            self._solve_and_store_ik(arm)
        self._publish_current(quiet=False)

    def _sweep(self, direction):
        """Move the absolute target by step*steps along the chosen axis."""
        for i in range(1, self.steps + 1):
            delta = direction * self.step
            self._move_once(direction)
            print('[AUTO_IK] step {:d}/{:d} d={:+.4f} | l_x={:+.4f} r_x={:+.4f} '
                  '| L_ik_err={:.1e} R_ik_err={:.1e}'.format(
                      i, self.steps, delta,
                      self.target_ee_poses['left']['position'][0],
                      self.target_ee_poses['right']['position'][0],
                      self.last_ik_error['left'][0],
                      self.last_ik_error['right'][0]),
                  flush=True)
            # live gap between steps (gives the robot time to move)
            self._spin_keepalive(self.interval)

    # --------------------------- run --------------------------------
    def run(self):
        """Anchor to the real state, then execute the scripted sweep/hold."""
        # 1. Wait for real joint states (anchor to the true initial config)
        t0 = time.time()
        while not self.joint_states_received and time.time() - t0 < 20.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.joint_states_received:
            print('[AUTO_IK] FAIL: no /joint_states within 20s; aborting', flush=True)
            return False
        print('[AUTO_IK] joint states locked; using real FK as absolute anchor',
              flush=True)

        # 2. Anchor the integration base to the true current EE pose (already
        #    synced by the callback; re-derive for verification + print).
        p_l, q_l = self.kinematics['left'].forward_position_quat(self.current_joint_states['left'])
        p_r, q_r = self.kinematics['right'].forward_position_quat(self.current_joint_states['right'])
        self.target_ee_poses['left'] = {'position': p_l, 'quaternion': q_l}
        self.target_ee_poses['right'] = {'position': p_r, 'quaternion': q_r}
        print('[AUTO_IK] anchor  L={} / R={}'.format(np.round(p_l, 4), np.round(p_r, 4)),
              flush=True)

        # 3. One no-op anchor target (absolute joints == current) to sync pipelines
        self._publish_current(quiet=False)
        self._spin(1.0)

        # 4. Forward sweep + hold
        self._sweep(self.direction)
        print('[AUTO_IK] arrived; holding {:.1f}s'.format(self.hold), flush=True)
        self._spin_keepalive(self.hold)

        # 5. Back sweep + settle
        if self.back:
            print('[AUTO_IK] returning', flush=True)
            self._sweep(-self.direction)
            self._spin_keepalive(self.hold)

        print('[AUTO_IK] done', flush=True)
        return True


def quaternion_from_array(q_array):
    """Convert [x, y, z, w] to quaternion [w, x, y, z]."""
    return np.array([q_array[3], q_array[0], q_array[1], q_array[2]])


def main():
    parser = argparse.ArgumentParser(
        description='Automated absolute task-space teleop with LOCAL IK -> absolute joints')
    # Sweep profile (same knobs as auto_absolute_teleop.py)
    parser.add_argument('--step', type=float, default=0.002, help='step size per move (m)')
    parser.add_argument('--steps', type=int, default=5,
                        help='number of steps (total move = step*steps, e.g. 0.01 m)')
    parser.add_argument('--interval', type=float, default=1.0, help='seconds between steps')
    parser.add_argument('--hold', type=float, default=6.0, help='hold after sweep/return (s)')
    parser.add_argument('--axis', choices=['x', 'y', 'z'], default='x', help='translation axis')
    parser.add_argument('--sign', type=float, default=1.0, help='+1 forward, -1 backward')
    parser.add_argument('--no-back', action='store_true', help='do not sweep back to origin')
    # IK publishing knobs (same knobs as key_teleop_cartesian_absolute_ik.py)
    parser.add_argument('--arm', type=str, choices=['left', 'right', 'both'], default='both',
                        help='which arm(s) move (default both, mirrors the manual experiment)')
    parser.add_argument('--target', type=str, choices=['safety', 'direct'], default='safety',
                        help="'safety' -> /dualarm_teleop_cmd type=4 via safety controller "
                             "(default); 'direct' -> JointState to "
                             "/mj_{left,right}/joints_desired bypassing the safety controller.")
    parser.add_argument('--direct-rate', type=float, default=50.0,
                        help='keep-alive republish rate (Hz) for --target direct')
    parser.add_argument('--disable-cbf', action='store_true',
                        help='set teleop message allow_safety_violation=1 (data[16]) to '
                             'disable the priority-1 collision-avoidance CBF layer')
    args = parser.parse_args()

    rclpy.init()
    node = AutoAbsoluteTeleopIK(
        step=args.step,
        steps=args.steps,
        interval=args.interval,
        hold=args.hold,
        axis=args.axis,
        sign=args.sign,
        back=not args.no_back,
        arm=args.arm,
        publish_target=args.target,
        direct_rate=args.direct_rate,
        allow_safety_violation=args.disable_cbf,
    )
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
