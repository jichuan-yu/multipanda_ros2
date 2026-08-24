#!/usr/bin/env python3
"""
Preset-move absolute task-space teleop with LOCAL IK (multi-segment 3-axis sweep).

A thin subclass of AutoAbsoluteTeleopIK (auto_absolute_teleop_ik.py) that
executes a scripted multi-segment Cartesian translation on ONE arm and then
exits. Purpose: before each relative-error experiment, jog the LEFT arm away
from the RIGHT arm so the two arms start from different initial configurations,
without manual keyboard driving.

Default profile (matches the manual key-jog habit):
    left arm: back -15 cm, then down -15 cm, then left (+Y, away from the right
    arm) 15 cm, at 0.01 m/step. The arm STAYS at the final offset (no return).

Direction convention (world frame, same as key_teleop_cartesian_absolute_ik.py):
    -x : backward (away from the operator)
    -z : down
    +y : left  (away from the right arm; LEFT base sits at Y=+0.26,
         RIGHT at Y=-0.26, so +Y moves the left arm off to the side)
If a direction actually moves the opposite way on your rig, flip the sign of
that token in --seq (e.g. +y -> -y).

Local IK is identical to the parent (panda_dh_kinematics.py, same DH model as
the safety controller); publishing is identical too:

    --target safety (default): Float64MultiArray /dualarm_teleop_cmd
                              (JOINT_POSITION type=4) via the safety controller.
    --target direct          : JointState to /mj_{left,right}/joints_desired,
                              bypassing the safety controller.

Usage:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/auto_absolute_teleop_preset_ik.py \
        --seq -x:15,-z:15,+y:15 --step 0.01 --interval 1.0 --hold 5.0
"""

import argparse
import os
import re
import sys
import time

import numpy as np
import rclpy

if __name__ == '__main__':
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from teleop.auto_absolute_teleop_ik import AutoAbsoluteTeleopIK
except ImportError:
    from auto_absolute_teleop_ik import AutoAbsoluteTeleopIK


AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}


def parse_move(token, default_steps):
    """Parse one move token '<sign><axis>[:<steps>]' -> (axis, sign, steps).

    Examples: '-x:15' -> ('x', -1, 15); '+y' -> ('y', +1, default_steps).
    """
    m = re.match(r'^([+-])([xyz])(?::(\d+))?$', token.strip().lower())
    if not m:
        raise ValueError(f"bad move {token!r}; expected '<+-><x|y|z>[:<steps>]', "
                         f"e.g. '-x:15' or '+y'")
    sign = -1 if m.group(1) == '-' else 1
    steps = int(m.group(3)) if m.group(3) else default_steps
    if steps <= 0:
        raise ValueError(f"move {token!r} must have steps > 0")
    return m.group(2), sign, steps

def _join_seq_value(argv):
    """argparse 会把 `--seq -x:15,...` 中那个以 '-' 开头的值误判成新选项
    （报 'argument --seq: expected one argument'）。这里把 `--seq <value>`
    预先拼成 `--seq=<value>` 再交给 argparse，等号形式不受影响。"""
    out = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == '--seq' and i + 1 < len(argv) and argv[i + 1].startswith('-'):
            out.append('--seq=' + argv[i + 1])
            i += 2
            continue
        out.append(a)
        i += 1
    return out

class AutoPresetTeleopIK(AutoAbsoluteTeleopIK):
    """One-arm multi-segment Cartesian preset sweep with local IK."""

    def __init__(self, moves, step=0.01, interval=1.0, hold=5.0,
                 arm='left', publish_target='safety', direct_rate=50.0,
                 allow_safety_violation=False):
        super().__init__(step=step, steps=0, interval=interval, hold=hold,
                         axis='x', sign=1.0, back=False, arm=arm,
                         publish_target=publish_target, direct_rate=direct_rate,
                         allow_safety_violation=allow_safety_violation)
        self.moves = moves  # list of (axis, sign, steps)

        # Expected net displacement per axis (for the final sanity print).
        self.net_expected = {}
        for axis, sign, steps in moves:
            self.net_expected[axis] = self.net_expected.get(axis, 0.0) + sign * steps * step

        print('\n[PRESET_IK] planned moves (step = {:.4f} m):'.format(step), flush=True)
        for i, (axis, sign, steps) in enumerate(self.moves, start=1):
            print(f'  [{i}] {sign:+d}{axis} x {steps} ticks'
                  f'  ->  {sign * steps * step:+.3f} m along {axis}', flush=True)
        print('  net: ' + '  '.join(f'{a}={v:+.3f}' for a, v in self.net_expected.items()),
              flush=True)

    def _sweep_axis(self, axis, sign, steps):
        """Move the selected arm(s) by sign*step along `axis` for `steps` ticks."""
        self.axis_idx = AXIS_INDEX[axis]
        arms = ('left', 'right') if self.selected_arm == 'both' else (self.selected_arm,)
        for i in range(1, steps + 1):
            pos_delta = np.zeros(3)
            pos_delta[self.axis_idx] = sign * self.step
            for arm in arms:
                self._integrate_target(arm, pos_delta, np.zeros(3))
                self._solve_and_store_ik(arm)
            self._publish_current(quiet=True)
            pos = self.target_ee_poses['left']['position']
            print(f'[PRESET_IK] step {i:2d}/{steps:2d} d_{axis}={sign * self.step:+.4f} | '
                  f'l_{axis}={pos[self.axis_idx]:+.4f} | '
                  f'L_ik_pos_err={self.last_ik_error["left"][0]:.1e}',
                  flush=True)
            self._spin_keepalive(self.interval)

    def run(self):
        """Anchor to the real state, execute each preset segment, hold, exit."""
        # 1. Wait for real joint states (anchor to the true initial config).
        t0 = time.time()
        while not self.joint_states_received and time.time() - t0 < 20.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.joint_states_received:
            print('[PRESET_IK] FAIL: no /joint_states within 20s; aborting', flush=True)
            return False
        print('[PRESET_IK] joint states locked; using real FK as absolute anchor',
              flush=True)

        # 2. Anchor the integration base to the true current EE pose.
        p_l, q_l = self.kinematics['left'].forward_position_quat(self.current_joint_states['left'])
        p_r, q_r = self.kinematics['right'].forward_position_quat(self.current_joint_states['right'])
        self.target_ee_poses['left'] = {'position': p_l, 'quaternion': q_l}
        self.target_ee_poses['right'] = {'position': p_r, 'quaternion': q_r}
        print('[PRESET_IK] anchor  L={} / R={}'.format(np.round(p_l, 4), np.round(p_r, 4)),
              flush=True)

        # 3. One no-op anchor target (absolute joints == current) to sync pipelines.
        self._publish_current(quiet=False)
        self._spin(1.0)

        # 4. Execute each preset segment (arm stays offset at the end; no return).
        for i, (axis, sign, steps) in enumerate(self.moves, start=1):
            print(f'[PRESET_IK] segment {i}/{len(self.moves)}: '
                  f'{sign:+d}{axis} x {steps} ticks '
                  f'-> {sign * steps * self.step:+.3f} m along {axis}', flush=True)
            self._sweep_axis(axis, sign, steps)

        # 5. Final hold, then report the achieved net displacement.
        print(f'[PRESET_IK] arrived; holding {self.hold:.1f}s', flush=True)
        self._spin_keepalive(self.hold)

        dP = self.target_ee_poses['left']['position'] - p_l
        exp = self.net_expected
        print(f'\n[PRESET_IK] done. left-EE net delta = '
              f'[{dP[0]:+.3f}, {dP[1]:+.3f}, {dP[2]:+.3f}] m | '
              f'expected [x:{exp.get("x", 0.0):+.3f}, '
              f'y:{exp.get("y", 0.0):+.3f}, z:{exp.get("z", 0.0):+.3f}] m',
              flush=True)
        if abs(dP[0]) < 0.01 and abs(dP[1]) < 0.01 and abs(dP[2]) < 0.01:
            print('[PRESET_IK] WARNING: net displacement ~ 0 — did the arm move? '
                  'Check segment logs above.', flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Preset-move absolute task-space teleop with LOCAL IK -> '
                    'absolute joints (single multi-segment Cartesian sweep, exits '
                    'afterwards)')
    parser.add_argument('--seq', type=str, default='-x:15,-z:15,+y:15',
                        help="comma-separated moves, each '<+-><x|y|z>[:<steps>]'; "
                             "default '-x:15,-z:15,+y:15' = left arm back 15 cm, "
                             "down 15 cm, left(+Y) 15 cm at --step 0.01")
    parser.add_argument('--step', type=float, default=0.01,
                        help='step size per tick (m); total per move = step*steps')
    parser.add_argument('--steps', type=int, default=15,
                        help='steps for moves given without an explicit :N')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='seconds between ticks')
    parser.add_argument('--hold', type=float, default=6.0,
                        help='hold at the final offset before exiting (s)')
    parser.add_argument('--arm', type=str, choices=['left', 'right', 'both'],
                        default='left', help='which arm(s) move (default left)')
    parser.add_argument('--target', type=str, choices=['safety', 'direct'],
                        default='safety',
                        help="'safety' -> /dualarm_teleop_cmd type=4 via safety "
                             "controller (default); 'direct' -> JointState to "
                             "/mj_{left,right}/joints_desired bypassing the safety "
                             "controller.")
    parser.add_argument('--direct-rate', type=float, default=50.0,
                        help='keep-alive republish rate (Hz) for --target direct')
    parser.add_argument('--disable-cbf', action='store_true',
                        help='set teleop message allow_safety_violation=1 (data[16]) '
                             'to disable the priority-1 collision-avoidance CBF layer')
    args = parser.parse_args(_join_seq_value(sys.argv[1:]))

    tokens = [t for t in args.seq.split(',') if t.strip()]
    if not tokens:
        parser.error("--seq must contain at least one move (e.g. '-x:15,-z:15,+y:15')")
    try:
        moves = [parse_move(t, args.steps) for t in tokens]
    except ValueError as e:
        parser.error(str(e))

    rclpy.init()
    node = AutoPresetTeleopIK(
        moves=moves,
        step=args.step,
        interval=args.interval,
        hold=args.hold,
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