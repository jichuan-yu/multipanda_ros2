#!/usr/bin/env python3
"""
Automated absolute task-space teleop for the relative-error reproduction.

Emulates the manual experiment in docs/relative_error.md ("双臂同时向前 0.01"):
  * publishes TASK_SPACE_POSE absolute targets (command_type=5, arm_selector=BOTH)
  * anchors the absolute base to the CURRENT true EE pose by reading /joint_states
    and running the exact same FK as the controller
    (teleop/utils/panda_dh_kinematics.py), so the sweep is a clean pure translation
    "forward 0.01 m" from whatever initial configuration is running
    (symmetric / asymmetric joint angles).
  * sequence: N steps forward (step m every interval s) -> hold -> N steps back -> hold
  * re-publishes the current target during waits so the safe controller never
    times out back to STOPPING.

Usage:
  python3 auto_absolute_teleop.py --step 0.002 --steps 5 --interval 0.1 --hold 6.0 --back
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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from teleop.base_teleop import BaseTeleopNode
    from teleop.utils.panda_dh_kinematics import PandaDHKinematics, HOME_POSITION
except ImportError:
    from base_teleop import BaseTeleopNode
    from utils.panda_dh_kinematics import PandaDHKinematics, HOME_POSITION


class AutoAbsoluteTeleop(BaseTeleopNode):
    """Non-interactive twin-arm absolute task-space teleop node."""

    def __init__(self, step, steps, interval, hold, axis='x', sign=1.0, back=True):
        super().__init__('auto_absolute_teleop', publish_rate=100.0)

        self.step = step
        self.steps = steps
        self.interval = interval
        self.hold = hold
        self.back = back
        self.axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        self.direction = sign  # +1 forward, -1 backward

        # FK models (bases match monitor/controller: left y=+0.26, right y=-0.26)
        self.kin_left = PandaDHKinematics('left')
        self.kin_right = PandaDHKinematics('right')

        # Current joint states (fall back to HOME until /joint_states arrives)
        self.q_left = HOME_POSITION.copy()
        self.q_right = HOME_POSITION.copy()
        self.joint_states_received = False

        # Subscriber for current joint states (best-effort, same as monitor)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.js_sub = self.create_subscription(
            sensor_msgs.msg.JointState, '/joint_states', self.joint_state_cb, qos)

        # Absolute targets (anchored to real FK when joint states arrive)
        self.target_left_pos = None
        self.target_right_pos = None
        self.target_left_quat_xyzw = None    # [x, y, z, w]
        self.target_right_quat_xyzw = None
# --------------------------- callbacks --------------------------
    def joint_state_cb(self, msg):
        try:
            for prefix, arr in (('mj_left', self.q_left), ('mj_right', self.q_right)):
                for i in range(7):
                    name = '{}_joint{}'.format(prefix, i + 1)
                    arr[i] = msg.position[msg.name.index(name)]
            self.joint_states_received = True
        except Exception:
            pass

    # --------------------------- helpers ----------------------------
    def _spin(self, sec):
        """Keep spinning while sleeping wall-time sec."""
        deadline = time.time() + sec
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def _publish_target(self):
        """Publish the current absolute target as TASK_SPACE_POSE (both arms)."""
        left = np.concatenate([self.target_left_pos, self.target_left_quat_xyzw])
        right = np.concatenate([self.target_right_pos, self.target_right_quat_xyzw])
        msg = self.create_teleop_command(
            arm_selector=self.BOTH_ARMS,
            command_type=self.TASK_SPACE_POSE,
            left_ee_delta=left,
            right_ee_delta=right,
            allow_safety_violation=False,
        )
        self.teleop_pub.publish(msg)

    def _publish_while(self, sec):
        """Keep the current target alive for `sec` s (1 Hz keep-alive).

        The controller's teleop timeout is 999 s (dual_arm_safe_controller_sim.h),
        so a single publish at hold start is enough; the 1 Hz keep-alive below only
        guards against a lowered timeout. High-rate re-publish is actively harmful:
        every TASK_SPACE_POSE message makes the controller re-arm its task-space
        error-feedback loop from the CURRENT robot state, which is what destabilized
        bypass mode (unconstrained 100x deadbeat) in the original reproduction.
        """
        self._publish_target()
        t0 = time.time()
        while time.time() - t0 < sec and rclpy.ok():
            self._spin(min(1.0, max(0.2, sec)))
            self._publish_target()

    def _sweep(self, direction):
        """Move the absolute target by step*steps along the chosen axis."""
        for i in range(1, self.steps + 1):
            delta = direction * self.step
            self.target_left_pos[self.axis_idx] += delta
            self.target_right_pos[self.axis_idx] += delta
            self._publish_target()
            print('[AUTO_TELEOP] step {:d}/{:d} d={:+.4f} | l_x={:+.4f} r_x={:+.4f}'.format(
                i, self.steps, delta, self.target_left_pos[0], self.target_right_pos[0]),
                flush=True)
            # live gap between steps (also gives the robot time to move)
            self._spin(self.interval)

    # --------------------------- run --------------------------------
    def run(self):
        # 1. Wait for real joint states (to anchor to the true initial config)
        t0 = time.time()
        while not self.joint_states_received and time.time() - t0 < 20.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.joint_states_received:
            print('[AUTO_TELEOP] FAIL: no /joint_states within 20s; aborting', flush=True)
            return False
        print('[AUTO_TELEOP] joint states locked; using real FK as absolute anchor', flush=True)

        # 2. Anchor the absolute target to the true current EE pose
        p_l, q_l_wxyz = self.kin_left.forward_position_quat(self.q_left)
        p_r, q_r_wxyz = self.kin_right.forward_position_quat(self.q_right)
        self.target_left_pos = p_l.copy()
        self.target_right_pos = p_r.copy()
        self.target_left_quat_xyzw = q_l_wxyz[[1, 2, 3, 0]].copy()   # wxyz -> xyzw
        self.target_right_quat_xyzw = q_r_wxyz[[1, 2, 3, 0]].copy()
        print('[AUTO_TELEOP] anchor  L={} / R={}'.format(p_l, p_r), flush=True)

        # 3. One no-op anchor target (delta ~= 0) to sync pipelines
        self._publish_target()
        self._spin(1.0)

        # 4. Forward sweep + hold
        self._sweep(self.direction)
        print('[AUTO_TELEOP] arrived; holding {:.1f}s'.format(self.hold), flush=True)
        self._publish_while(self.hold)

        # 5. Back sweep + settle
        if self.back:
            print('[AUTO_TELEOP] returning', flush=True)
            self._sweep(-self.direction)
            self._publish_while(self.hold)

        print('[AUTO_TELEOP] done', flush=True)
        return True
def main():
    parser = argparse.ArgumentParser(description='Automated absolute task-space teleop')
    parser.add_argument('--step', type=float, default=0.002, help='step size per move (m)')
    parser.add_argument('--steps', type=int, default=5,
                        help='number of steps (total move = step*steps, e.g. 0.01 m)')
    parser.add_argument('--interval', type=float, default=1.0, help='seconds between steps')
    parser.add_argument('--hold', type=float, default=6.0, help='hold after sweep/return (s)')
    parser.add_argument('--axis', choices=['x', 'y', 'z'], default='x', help='translation axis')
    parser.add_argument('--sign', type=float, default=1.0, help='+1 forward, -1 backward')
    parser.add_argument('--no-back', action='store_true', help='do not sweep back to origin')
    args = parser.parse_args()

    rclpy.init()
    node = AutoAbsoluteTeleop(
        step=args.step,
        steps=args.steps,
        interval=args.interval,
        hold=args.hold,
        axis=args.axis,
        sign=args.sign,
        back=not args.no_back,
    )
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()