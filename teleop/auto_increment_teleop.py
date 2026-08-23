#!/usr/bin/env python3
"""
Automated INCREMENTAL task-space teleop for the relative-error reproduction.

Mirror of auto_absolute_teleop.py but using TASK_SPACE_INCREMENT (command_type=2,
arm_selector=BOTH): each step publishes a *relative* end-effector delta
[+step, 0, 0] for BOTH arms simultaneously. This is exactly what the manual
keyboard path does (key_teleop_cartesian.py), just scripted with a fixed
step size / interval so the experiment is reproducible.

Sequence: N steps forward (step m every interval s) -> hold -> N steps back -> hold.
A zero-delta keep-alive message is re-published during holds so the safe
controller never times out back to STOPPING.

Usage:
  python3 auto_increment_teleop.py --step 0.002 --steps 5 --interval 1.0 --hold 6.0 --back
"""

import argparse
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

try:
    from teleop.base_teleop import BaseTeleopNode
except ImportError:
    from base_teleop import BaseTeleopNode


class AutoIncrementTeleop(BaseTeleopNode):
    """Non-interactive twin-arm incremental task-space teleop node."""

    def __init__(self, step, steps, interval, hold, axis='x', sign=1.0, back=True):
        super().__init__('auto_increment_teleop', publish_rate=100.0)

        self.step = step
        self.steps = steps
        self.interval = interval
        self.hold = hold
        self.back = back
        self.axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        self.direction = sign  # +1 forward, -1 backward

        self.selected_arm = 'both'   # command BOTH arms

    # --------------------------- publish helpers --------------------------
    def _publish_delta(self, delta):
        """Publish one task-space increment message for both arms."""
        left_ee_delta = np.zeros(6)
        right_ee_delta = np.zeros(6)
        left_ee_delta[self.axis_idx] = delta
        right_ee_delta[self.axis_idx] = delta
        msg = self.create_teleop_command(
            arm_selector=self.BOTH_ARMS,
            command_type=self.TASK_SPACE_INCREMENT,
            left_ee_delta=left_ee_delta,
            right_ee_delta=right_ee_delta,
            allow_safety_violation=False)
        self.teleop_pub.publish(msg)
        return left_ee_delta[0:3].copy(), right_ee_delta[0:3].copy()

    def _spin(self, duration):
        """Sleep while the daemon rclpy.spin thread handles callbacks."""
        time.sleep(duration)

    def _publish_while(self, duration):
        """Keep pipeline alive with zero-delta (hold current position)."""
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self._publish_delta(0.0)
            time.sleep(0.05)

    def _sweep(self, direction):
        for i in range(1, self.steps + 1):
            delta = direction * self.step
            l, r = self._publish_delta(delta)
            print('[AUTO_TELEOP] step {:d}/{:d} d={:+.4f} | l_x={:+.4f} r_x={:+.4f}'.format(
                i, self.steps, delta, l[0], r[0]), flush=True)
            self._spin(self.interval)

    # --------------------------- run --------------------------------
    def run(self):
        # 0. Daemon thread keeps callbacks alive (same pattern as key_teleop)
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        # 1. Give the controller time to reach TELEOPERATING before moving
        print('[AUTO_TELEOP] priming pipeline with zero-delta for 2.0s', flush=True)
        self._publish_while(2.0)

        # 2. Forward sweep + hold
        self._sweep(self.direction)
        print('[AUTO_TELEOP] arrived; holding {:.1f}s'.format(self.hold), flush=True)
        self._publish_while(self.hold)

        # 3. Back sweep + settle
        if self.back:
            print('[AUTO_TELEOP] returning', flush=True)
            self._sweep(-self.direction)
            self._publish_while(self.hold)

        print('[AUTO_TELEOP] done', flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description='Automated incremental task-space teleop')
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
    node = AutoIncrementTeleop(
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
