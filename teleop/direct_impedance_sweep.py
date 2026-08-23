#!/usr/bin/env python3
"""
Direct low-level impedance experiment driver.

Bypasses the safety (safe) layer entirely: this script publishes the joint
reference straight to the bottom impedance controller's command topics

    /mj_left/joints_desired   (sensor_msgs/JointState)
    /mj_right/joints_desired  (sensor_msgs/JointState)

which are consumed by `dual_joint_impedance_controller`
(franka_example_controllers/MultiJointImpedanceController).  The controller
does, per cycle:

    tau = K . clamp(q_d - q, +/-e_q_max)  +  D . (dq_d - dq)  +  coriolis

with K=[600,600,600,600,250,150,100], D=[30,30,30,15,12,12,9],
e_q_max=0.4 rad, alpha_filt=0.01 in dual_sim_controllers.yaml.

So this isolates the tracking behaviour of the BOTTOM controller alone:  same
"symmetric / asymmetric configs, forward 0.01 m + back" experiment as
docs/relative_error.md, but with `main_sim_node` (safe) never running.

The JointState it sends has `position = q_d` (7 joints) and `velocity = dq_d`.
Three `dq_d` policies are provided (see --vel-mode) -- precisely the velocity
semantics discussed for the original-vs-bypass divergence:

  zero      : dq_d = 0  ->  pure regulation (D pulls against -D.dq).
              Mirrors the current "setZero" bypass.  Expect modest position
              tracking lag at the holding points.

  integrated: internal q_ref;  dq_d = mean joint velocity of the actual step
              (self-consistent, bounded).  Cleanest tracking, mirrors the
              `use_integrated_reference:=true` path.

  deadbeat  : each message  dq_d = (J(q)+)*(T_target - T_fk(q))/0.01  AND
              q_d = q + J+*(T_target - T_fk(q)) re-anchored from the CURRENT
              measured state (original 100 s^-1 deadbeat).  Reproduces, at the
              impedance layer only, the divergence of the original feedforward.

To measure the result: run relative_error_monitor (reads /joint_states and
/mj_{left,right}/joints_desired).  Because the sweep is symmetric, `safe_err*`
stays ~0 by construction; the real impedance tracking error is the difference
between CURRENT and COMMANDED relative geometry:

    err_mm = 1e3 * norm(curr_rel - safe_rel)     (x-axis component: sweep axis)

Usage (sim already launched, do NOT start main_sim_node):

    python3 direct_impedance_sweep.py --step 0.002 --steps 5 --interval 0.1 --hold 6.0 --vel-mode zero
    python3 direct_impedance_sweep.py --step 0.002 --steps 5 --interval 2.0 --hold 6.0 --vel-mode integrated
    python3 direct_impedance_sweep.py --step 0.002 --steps 5 --interval 0.1 --hold 6.0 --vel-mode deadbeat --fb-hz 100
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
import sensor_msgs.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
try:
    from utils.panda_dh_kinematics import PandaDHKinematics, HOME_POSITION
except ImportError:
    from teleop.utils.panda_dh_kinematics import PandaDHKinematics, HOME_POSITION

N_JOINTS = 7
_PROBE_DT = 0.01  # deadbeat gain denominator (s), mirrors old control_period_
_JOINT_NAMES = {side: ['mj_{}_joint{}'.format(side, i + 1) for i in range(N_JOINTS)]
                for side in ('left', 'right')}

class DirectImpedanceSweep(Node):
    def __init__(self, step, steps, interval, hold, axis, direction, back,
                 vel_mode, fb_hz):
        super().__init__('direct_impedance_sweep')
        self.step = step
        self.steps = steps
        self.interval = interval
        self.hold = hold
        self.direction = direction
        self.back = back
        self.vel_mode = vel_mode
        self.fb_period = 1.0 / max(1.0, fb_hz)
        self.axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        self.kin = {side: PandaDHKinematics(side) for side in ('left', 'right')}
        self.q = {side: np.array(HOME_POSITION, dtype=float) for side in ('left', 'right')}
        self.js_received = False
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.js_sub = self.create_subscription(
            sensor_msgs.msg.JointState, '/joint_states', self.js_cb, qos)
        self.left_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/mj_left/joints_desired', 10)
        self.right_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/mj_right/joints_desired', 10)
        self.q_d = {side: None for side in ('left', 'right')}
        self.p_target = {side: None for side in ('left', 'right')}
        self.q_quat = {side: None for side in ('left', 'right')}
        self.q_last_sent = None
        self.t_last_sent = None

    def js_cb(self, msg):
        try:
            names = list(msg.name)
            for side in ('left', 'right'):
                q = self.q[side]
                for i in range(N_JOINTS):
                    q[i] = msg.position[names.index(_JOINT_NAMES[side][i])]
            self.js_received = True
        except Exception:
            pass

    def _spin(self, sec):
        deadline = time.time() + sec
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def _send(self, qL, qR, vL, vR, quiet=False):
        stamp = self.get_clock().now().to_msg()
        for side, q, v, pub in (('left', qL, vL, self.left_pub),
                                ('right', qR, vR, self.right_pub)):
            msg = sensor_msgs.msg.JointState()
            msg.header.stamp = stamp
            msg.name = _JOINT_NAMES[side]
            msg.position = np.asarray(q, dtype=float).tolist()
            msg.velocity = np.asarray(v, dtype=float).tolist()
            pub.publish(msg)
        if not quiet:
            print('[DIRECT] L q={} v={}  R q={} v={}'.format(
                np.round(qL, 4), np.round(vL, 3),
                np.round(qR, 4), np.round(vR, 3)), flush=True)

    def _velocity_for(self, mode, side):
        """dq_d policy: deadbeat = original 1/0.01 s re-anchored gain.

        zero / integrated want dq_d computed synchronously with the step commit
        (handled in `sweep` for integrated); here non-deadbeat -> 0 (regulation).
        """
        if mode != 'deadbeat':
            return np.zeros(N_JOINTS)
        q_cur = self.q[side]
        J = self.kin[side].jacobian_world(q_cur)[:3, :]
        p_cur = self.kin[side].forward_kinematics(q_cur)[:3, 3]
        dx = self.p_target[side] - p_cur
        dq = np.linalg.pinv(J) @ dx
        return dq / _PROBE_DT

    def _q_d_deadbeat(self, side):
        if self.vel_mode != 'deadbeat':
            return self.q_d[side]
        q_cur = self.q[side]
        J = self.kin[side].jacobian_world(q_cur)[:3, :]
        p_cur = self.kin[side].forward_kinematics(q_cur)[:3, 3]
        dx = self.p_target[side] - p_cur
        dq = np.linalg.pinv(J) @ dx
        return q_cur + dq

    def _publish_setpoint(self, quiet=False):
        qL = self._q_d_deadbeat('left')
        qR = self._q_d_deadbeat('right')
        vL = self._velocity_for(self.vel_mode, 'left')
        vR = self._velocity_for(self.vel_mode, 'right')
        self._send(qL, qR, vL, vR, quiet=quiet)

    def _publish_while(self, sec):
        """Republish the current target for `sec` s."""
        t0 = time.time()
        while time.time() - t0 < sec and rclpy.ok():
            self._publish_setpoint(quiet=True)
            self._spin(min(self.fb_period, 0.2))

    def run(self):
        t0 = time.time()
        while not self.js_received and time.time() - t0 < 20.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.js_received:
            print('[DIRECT] FAIL: no /joint_states within 20 s; aborting', flush=True)
            return False
        print('[DIRECT] joint states locked; anchoring to true EE', flush=True)

        for side in ('left', 'right'):
            p, q_wxyz = self.kin[side].forward_position_quat(self.q[side])
            self.p_target[side] = p.copy()
            self.q_quat[side] = q_wxyz.copy()
            self.q_d[side] = self.q[side].copy()
        print('[DIRECT] anchor  L={} R={}'.format(
            self.p_target['left'], self.p_target['right']), flush=True)

        self._publish_setpoint(quiet=True)   # no-op anchor sync
        self._spin(1.0)
        # reference bookkeeping for the integrated dq_d (previous commit time)
        self.q_last_sent = {s: self.q_d[s].copy() for s in ('left', 'right')}
        self.t_last_sent = self.get_clock().now().seconds()

        def sweep(out_to_apex):
            # offsets monotonic per phase: outward 1..steps; return steps-1..0
            offsets = list(range(1, self.steps + 1)) if out_to_apex \
                else list(range(self.steps - 1, -1, -1))
            for j, offset in enumerate(offsets):
                for side in ('left', 'right'):
                    target_pos = self.p_target[side].copy()
                    target_pos[self.axis_idx] += self.direction * offset * self.step
                    qn, perr, rerr, ok = self.kin[side].inverse_kinematics(
                        target_pos, self.q_quat[side],
                        q_seed=self.q_d[side], tol=1e-9, max_iter=300, damping=0.02)
                    if not ok:
                        print('[DIRECT] WARN IK not converged side={} '
                              'perr={:.2e} rerr={:.2e}'.format(side, perr, rerr), flush=True)
                    self.q_d[side] = qn
                print('[DIRECT] step {}/{} d={:+.4f}'.format(
                    j + 1, len(offsets), self.direction * self.step), flush=True)

                if self.vel_mode == 'integrated':
                    # dq_d = mean joint velocity of the reference between commits
                    dt = max(1e-3, self.get_clock().now().seconds() - self.t_last_sent)
                    vL = (self.q_d['left'] - self.q_last_sent['left']) / dt
                    vR = (self.q_d['right'] - self.q_last_sent['right']) / dt
                    self._send(self.q_d['left'], self.q_d['right'], vL, vR, quiet=True)
                else:
                    self._publish_setpoint(quiet=True)

                self.q_last_sent = {s: self.q_d[s].copy() for s in ('left', 'right')}
                self.t_last_sent = self.get_clock().now().seconds()
                self._spin(max(0.01, self.interval))
            print('[DIRECT] arrived; holding {:.1f} s'.format(self.hold), flush=True)
            self._publish_while(self.hold)

        sweep(True)
        if self.back:
            print('[DIRECT] returning', flush=True)
            sweep(False)
            self._publish_while(self.hold)

        print('[DIRECT] done', flush=True)
        return True

def main():
    ap = argparse.ArgumentParser(description='Direct bottom-layer impedance sweep')
    ap.add_argument('--step', type=float, default=0.002, help='per-step translation (m)')
    ap.add_argument('--steps', type=int, default=5, help='steps (total = step*steps, e.g. 0.01 m)')
    ap.add_argument('--interval', type=float, default=1.0, help='seconds between steps')
    ap.add_argument('--hold', type=float, default=6.0, help='hold after sweep/return (s)')
    ap.add_argument('--axis', choices=['x', 'y', 'z'], default='x', help='translation axis')
    ap.add_argument('--sign', type=float, default=1.0, help='+1 forward, -1 backward')
    ap.add_argument('--no-back', action='store_true', help='do not sweep back to origin')
    ap.add_argument('--vel-mode', choices=['zero', 'integrated', 'deadbeat'],
                    default='zero', help='dq_d policy for the impedance controller')
    ap.add_argument('--fb-hz', type=float, default=10.0,
                    help='republish rate (Hz); use 100 for deadbeat')
    args = ap.parse_args()

    rclpy.init()
    node = DirectImpedanceSweep(
        step=args.step, steps=args.steps, interval=args.interval, hold=args.hold,
        axis=args.axis, direction=args.sign, back=not args.no_back,
        vel_mode=args.vel_mode, fb_hz=args.fb_hz)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
