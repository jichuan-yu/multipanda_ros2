#!/usr/bin/env python3
"""
Plot reference / command / state trajectories from a rosbag to localize spikes.

The three values (per the MPRC dual-arm controller):
  reference : teleop expectation, rebuilt from /dualarm_teleop_cmd
              (Float64MultiArray; same decoding as DualArmSafeControllerSim::
              teleopCmdCB, dual_arm_safe_controller_sim.cpp:1173-1516)
  command   : safe command, /mj_left/joints_desired + /mj_right/joints_desired
              (sensor_msgs/JointState, 7 dims each)
  state     : actual joints, /joint_states (sensor_msgs/JointState, 14 dims)

Diagnostic logic: spikes in reference -> teleop/IK side; spikes in command
while reference is smooth -> QP/controller; spikes in state while command is
smooth -> execution/impedance layer.

Usage:
  python3 plot_rcd_trajectories.py <rosbag_dir_or_db3> [-o out_prefix]
        [--spike-joint 0.02] [--spike-ee 0.005] [--spike-rpy 0.05]
        [--csv out.csv]

Reuses utils/panda_dh_kinematics.py (FK/Jacobian identical to the controller's
internal model) and the plotting style of plot_relative_error.py.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

sys.path.insert(0, str(Path(__file__).resolve().parent / 'utils'))
from panda_dh_kinematics import PandaDHKinematics, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER

# ---------------------------------------------------------------------------
# Constants (must match the controller: dual_arm_safe_controller_sim.cpp)
# ---------------------------------------------------------------------------
CONTROL_PERIOD = 0.01      # 100 Hz
MAX_JOINT_STEP = 0.002     # joint increment clamp (rad)
MAX_JOINT_VEL = 0.5        # joint velocity clamp (rad/s)
IK_DAMPING = 0.01          # DLS damping in computeJointFromTaskSpace

# Command types (header dual_arm_safe_controller_sim.h:234-239)
CMD_JOINT_POS_INC = 0
CMD_JOINT_VEL = 1
CMD_TASK_SPACE_INC = 2
CMD_TASK_SPACE_VEL = 3
CMD_JOINT_POS = 4
CMD_TASK_SPACE_POSE = 5

# Arm selector (header:241-244)
ARM_LEFT = 1
ARM_RIGHT = 2
ARM_BOTH = 3

JOINT_NAMES = ['mj_left_joint1', 'mj_left_joint2', 'mj_left_joint3', 'mj_left_joint4',
               'mj_left_joint5', 'mj_left_joint6', 'mj_left_joint7',
               'mj_right_joint1', 'mj_right_joint2', 'mj_right_joint3', 'mj_right_joint4',
               'mj_right_joint5', 'mj_right_joint6', 'mj_right_joint7']

COLORS = {'reference': '#d62728', 'command': '#1f77b4', 'state': '#2ca02c'}
LABELS = {'reference': 'reference (teleop)', 'command': 'command (QP out)', 'state': 'state (actual)'}

LEFT = np.arange(0, 7)
RIGHT = np.arange(7, 14)

# ---------------------------------------------------------------------------
# rosbag loading
# ---------------------------------------------------------------------------
def load_bag(bag_path: str):
    """Extract the three value streams from a rosbag.

    Returns (teleop_df, state_df, cmd_df): DataFrames with a 't' column (s).
      teleop_df: data (list/array), type, arm, flag
      state_df : q[14] columns q0..q13 (+ qdot0..qdot13)
      cmd_df   : same q columns, from merged left+right joints_desired
    """
    path = Path(bag_path)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([path], default_typestore=typestore) as reader:
        conns = {c.topic: c for c in reader.connections}

        def read_topic(topic):
            if topic not in conns:
                return []
            out = []
            for _, ts, raw in reader.messages(connections=[conns[topic]]):
                out.append((ts / 1e9, reader.deserialize(raw, conns[topic].msgtype)))
            return out

        teleop_raw = read_topic('/dualarm_teleop_cmd')
        state_raw = read_topic('/joint_states')
        cmd_l = read_topic('/mj_left/joints_desired')
        cmd_r = read_topic('/mj_right/joints_desired')

    # --- state ---
    idx = {n: i for i, n in enumerate(state_raw[0][1].name)} if state_raw else {}
    q_idx = [idx[n] for n in JOINT_NAMES if n in idx]
    qd_idx = [idx[n] for n in JOINT_NAMES if n in idx]  # velocity same names
    rows = []
    for t, m in state_raw:
        pos = np.asarray(m.position)
        vel = np.asarray(m.velocity) if len(m.velocity) else np.zeros(len(pos))
        rows.append([t] + list(pos[q_idx]) + list(vel[q_idx]))
    state_df = pd.DataFrame(rows, columns=['t'] + [f'q{i}' for i in range(14)] + [f'qd{i}' for i in range(14)])

    # --- command (left+right merged, assume both use the same joint order) ---
    rows_l, rows_r = [], []
    for t, m in cmd_l:
        pos = np.asarray(m.position)
        vel = np.asarray(m.velocity) if len(m.velocity) else np.zeros(len(pos))
        rows_l.append([t] + list(pos) + list(vel))
    for t, m in cmd_r:
        pos = np.asarray(m.position)
        vel = np.asarray(m.velocity) if len(m.velocity) else np.zeros(len(pos))
        rows_r.append([t] + list(pos) + list(vel))
    cmd_df = None
    if rows_l and rows_r:
        # right arm columns -> q7..q13, then nearest-neighbor merge onto left grid
        l = pd.DataFrame(rows_l, columns=['t'] + [f'q{i}' for i in range(7)] + [f'qd{i}' for i in range(7)])
        r = pd.DataFrame(rows_r, columns=['t'] + [f'q{i}' for i in range(7)] + [f'qd{i}' for i in range(7)])
        r.rename(columns={f'q{i}': f'q{i + 7}' for i in range(7)} |
                          {f'qd{i}': f'qd{i + 7}' for i in range(7)}, inplace=True)
        cmd_df = pd.merge_asof(l, r, on='t', direction='nearest')

    # --- teleop ---
    rows = []
    for t, m in teleop_raw:
        d = np.asarray(m.data)
        rows.append([t, d[14], d[15] if len(d) > 15 else ARM_BOTH,
                     d[16] if len(d) > 16 else 0.0, d[:14]])
    teleop_df = pd.DataFrame(rows, columns=['t', 'type', 'arm', 'flag', 'data'])

    return teleop_df, state_df, cmd_df


# ---------------------------------------------------------------------------
# reference rebuild (mirror of teleopCmdCB / process* in the controller)
# ---------------------------------------------------------------------------
def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _quat_rotvec(q):
    """[qx,qy,qz,qw] -> rotation vector (angle*axis), like AngleAxisd."""
    qx, qy, qz, qw = q
    n = np.sqrt(qx * qx + qy * qy + qz * qz)
    if n < 1e-12:
        return np.zeros(3)
    ang = 2.0 * np.arctan2(n, qw)
    return ang * np.array([qx, qy, qz]) / n


def build_reference(teleop_df, state_df, kin_left, kin_right):
    """Rebuild the absolute reference target for every teleop message.

    The controller accumulates each increment onto the *latest actual state*
    (q_teleop_target_ << q1_, q2_), so we align each teleop msg to the most
    recent /joint_states via merge_asof (direction='backward').
    Returns DataFrame(t, q0..q13) with NaN rows where no state was available.
    """
    if teleop_df.empty or state_df.empty:
        return pd.DataFrame(columns=['t'] + [f'q{i}' for i in range(14)])

    tele = teleop_df.copy()
    st = state_df[['t'] + [f'q{i}' for i in range(14)]].copy()
    tele = pd.merge_asof(tele, st, on='t', direction='backward', suffixes=('', '_st'))
    # merge_asof with suffix renames q0.. to q0_st for overlapping columns
    for i in range(14):
        tele.rename(columns={f'q{i}': f'q{i}_st'}, inplace=True)

    rows = []
    for _, row in tele.iterrows():
        base = row[[f'q{i}_st' for i in range(14)]].to_numpy(dtype=float)
        if np.isnan(base).any():
            continue
        qt = base.copy()
        typ = int(row['type'])
        arm = int(row['arm'])
        d = row['data']

        sel_left = arm in (ARM_LEFT, ARM_BOTH)
        sel_right = arm in (ARM_RIGHT, ARM_BOTH)

        if typ == CMD_JOINT_POS_INC:
            if sel_left:
                for i in LEFT:
                    qt[i] = base[i] + _clamp(float(d[i]), MAX_JOINT_STEP)
            if sel_right:
                for i in RIGHT:
                    qt[i] = base[i] + _clamp(float(d[i]), MAX_JOINT_STEP)
        elif typ == CMD_JOINT_VEL:
            if sel_left:
                for i in LEFT:
                    qt[i] = base[i] + _clamp(float(d[i]), MAX_JOINT_VEL) * CONTROL_PERIOD
            if sel_right:
                for i in RIGHT:
                    qt[i] = base[i] + _clamp(float(d[i]), MAX_JOINT_VEL) * CONTROL_PERIOD
        elif typ in (CMD_TASK_SPACE_INC, CMD_TASK_SPACE_VEL):
            # task data: [Lx,Ly,Lz,Lqx,Lqy,Lqz, Rx,Ry,Rz,Rqx,Rqy,Rqz] (indices 0-11)
            for side, kin, sel, off in (('left', kin_left, sel_left, 0), ('right', kin_right, sel_right, 6)):
                if not sel:
                    continue
                q_side = qt[LEFT if side == 'left' else RIGHT]
                J = kin.jacobian_world(q_side)
                delta_x = np.concatenate([d[off:off + 3], d[off + 3:off + 6]])
                JJt = J @ J.T + IK_DAMPING ** 2 * np.eye(6)
                dq = J.T @ np.linalg.solve(JJt, delta_x)
                q_side = q_side + dq * CONTROL_PERIOD if typ == CMD_TASK_SPACE_VEL else q_side + dq
                if side == 'left':
                    qt[LEFT] = q_side
                else:
                    qt[RIGHT] = q_side
        elif typ == CMD_JOINT_POS:
            if sel_left:
                qt[LEFT] = np.clip(np.asarray(d[0:7], dtype=float), JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
            if sel_right:
                qt[RIGHT] = np.clip(np.asarray(d[7:14], dtype=float), JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
        elif typ == CMD_TASK_SPACE_POSE:
            # [Lx,Ly,Lz,Lqx,Lqy,Lqz,Lqw, Rx,Ry,Rz,Rqx,Rqy,Rqz,Rqw]
            for side, kin, sel, off in (('left', kin_left, sel_left, 0), ('right', kin_right, sel_right, 7)):
                if not sel:
                    continue
                q_side = qt[LEFT if side == 'left' else RIGHT]
                T_cur = kin.forward_kinematics(q_side)
                p_tgt = np.asarray(d[off:off + 3], dtype=float)
                R_tgt = _quat_to_R(np.asarray(d[off + 3:off + 7], dtype=float))  # qx,qy,qz,qw
                delta_p = p_tgt - T_cur[:3, 3]
                delta_o = _quat_rotvec(_R_to_quat(R_tgt @ T_cur[:3, :3].T))
                J = kin.jacobian_world(q_side)
                JJt = J @ J.T + IK_DAMPING ** 2 * np.eye(6)
                dq = J.T @ np.linalg.solve(JJt, np.concatenate([delta_p, delta_o]))
                if side == 'left':
                    qt[LEFT] = q_side + dq
                else:
                    qt[RIGHT] = q_side + dq
        else:
            continue
        rows.append([row['t']] + list(qt))

    return pd.DataFrame(rows, columns=['t'] + [f'q{i}' for i in range(14)])


# ---------------------------------------------------------------------------
# rotation helpers
# ---------------------------------------------------------------------------
def _quat_to_R(q):
    qx, qy, qz, qw = q
    x2, y2, z2 = qx + qx, qy + qy, qz + qz
    xx, xy, xz = qx * x2, qx * y2, qx * z2
    yy, yz, zz = qy * y2, qy * z2, qz * z2
    wx, wy, wz = qw * x2, qw * y2, qw * z2
    return np.array([[1 - (yy + zz), xy - wz, xz + wy],
                     [xy + wz, 1 - (xx + zz), yz - wx],
                     [xz - wy, yz + wx, 1 - (xx + yy)]])


def _R_to_quat(R):
    """R -> [qx,qy,qz,qw] (Shepperd)."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S,
                         (R[1, 0] - R[0, 1]) / S, 0.25 * S])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S,
                         (R[2, 1] - R[1, 2]) / S])
    if R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S,
                         (R[0, 2] - R[2, 0]) / S])
    S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S,
                     (R[1, 0] - R[0, 1]) / S])


def matrix_to_rpy(R):
    """R -> [roll, pitch, yaw] (ZYX intrinsic, matching 'world' Euler)."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return np.array([x, y, z])


# ---------------------------------------------------------------------------
# FK trajectories + spike detection
# ---------------------------------------------------------------------------
def fk_ee(times, q_mat, kin_left, kin_right):
    """q_mat: (N,14) -> positions (N,2,3), rpy (N,2,3)."""
    n = q_mat.shape[0]
    pos = np.full((n, 2, 3), np.nan)
    rpy = np.full((n, 2, 3), np.nan)
    for i in range(n):
        q = q_mat[i]
        if np.isnan(q).any():
            continue
        for s, kin in ((0, kin_left), (1, kin_right)):
            T = kin.forward_kinematics(q[LEFT if s == 0 else RIGHT])
            pos[i, s] = T[:3, 3]
            rpy[i, s] = matrix_to_rpy(T[:3, :3])
    return pos, rpy


def detect_spikes(times, values, threshold):
    """Return a mask of the SAME shape as values, True where |diff| > threshold
    (spike = the sample AFTER the jump)."""
    vals = np.asarray(values, dtype=float)
    diff = np.abs(np.diff(vals, axis=0))
    mask = np.zeros_like(vals, dtype=bool)
    mask[1:] = diff > threshold
    return mask


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------
def _plot_series(ax, t, data, spike_mask, label):
    ok = ~np.isnan(data)
    ax.plot(t[ok], data[ok], color=COLORS[label], label=LABELS[label], linewidth=1.2)
    if spike_mask.any():
        ax.scatter(t[spike_mask], data[spike_mask], color='red', s=18, zorder=5,
                   label='spike' if label == 'reference' else None)


def plot_trajectories(ref, cmd, state, pos, rpy, pos_cmd, pos_state, rpy_cmd, rpy_state,
                      spikes, out_prefix):
    """pos: (N,2,3) for reference; rpy likewise. pos_cmd/pos_state: command/state."""
    t_ref = ref['t'].to_numpy()
    t_cmd = cmd['t'].to_numpy()
    t_st = state['t'].to_numpy()
    t0 = min(t_ref[0], t_cmd[0], t_st[0])

    arm_names = ['left arm', 'right arm']
    dim_names = ['x', 'y', 'z']
    rpy_names = ['roll', 'pitch', 'yaw']

    # --- Fig 1: EE position ---
    fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
    for a in range(2):
        for d in range(3):
            ax = axes[a, d]
            _plot_series(ax, t_ref - t0, pos[:, a, d], spikes['ref_pos'][:, a, d], 'reference')
            _plot_series(ax, t_cmd - t0, pos_cmd[:, a, d], spikes['cmd_pos'][:, a, d], 'command')
            _plot_series(ax, t_st - t0, pos_state[:, a, d], spikes['state_pos'][:, a, d], 'state')
            ax.set_ylabel(f'{dim_names[d]} (m)')
            ax.set_title(f'{arm_names[a]} EE position - {dim_names[d]}')
            ax.grid(True, alpha=0.3, linestyle='--')
    axes[0, 0].legend(fontsize=9)
    fig.suptitle('EE position: reference / command / state (spikes in red)', fontsize=14, fontweight='bold')
    fig.tight_layout()
    fig.savefig(f'{out_prefix}_ee_position.png', dpi=150)
    plt.close(fig)

    # --- Fig 2: EE orientation RPY ---
    fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
    for a in range(2):
        for d in range(3):
            ax = axes[a, d]
            _plot_series(ax, t_ref - t0, rpy[:, a, d], spikes['ref_rpy'][:, a, d], 'reference')
            _plot_series(ax, t_cmd - t0, rpy_cmd[:, a, d], spikes['cmd_rpy'][:, a, d], 'command')
            _plot_series(ax, t_st - t0, rpy_state[:, a, d], spikes['state_rpy'][:, a, d], 'state')
            ax.set_ylabel(f'{rpy_names[d]} (rad)')
            ax.set_title(f'{arm_names[a]} EE orientation - {rpy_names[d]}')
            ax.grid(True, alpha=0.3, linestyle='--')
    axes[0, 0].legend(fontsize=9)
    fig.suptitle('EE orientation (RPY): reference / command / state (spikes in red)', fontsize=14, fontweight='bold')
    fig.tight_layout()
    fig.savefig(f'{out_prefix}_ee_orientation.png', dpi=150)
    plt.close(fig)

    # --- Fig 3: joint angles ---
    fig, axes = plt.subplots(2, 7, figsize=(22, 8), sharex=True)
    for a in range(2):
        for j in range(7):
            ax = axes[a, j]
            col = j if a == 0 else j + 7
            _plot_series(ax, t_ref - t0, ref[f'q{col}'].to_numpy(), spikes['ref_joint'][:, col], 'reference')
            _plot_series(ax, t_cmd - t0, cmd[f'q{col}'].to_numpy(), spikes['cmd_joint'][:, col], 'command')
            _plot_series(ax, t_st - t0, state[f'q{col}'].to_numpy(), spikes['state_joint'][:, col], 'state')
            ax.set_ylabel(f'q{col + 1} (rad)')
            ax.set_title(f'{arm_names[a]} joint {j + 1}')
            ax.grid(True, alpha=0.3, linestyle='--')
    axes[0, 0].legend(fontsize=9)
    fig.suptitle('Joint angles: reference / command / state (spikes in red)', fontsize=14, fontweight='bold')
    fig.tight_layout()
    fig.savefig(f'{out_prefix}_joints.png', dpi=150)
    plt.close(fig)


def report_spikes(name, times, mask, t0, values, dim_desc):
    """mask may be 1D (N,) or ND; dim_desc(idx_tuple) -> label of the dimension."""
    coords = np.argwhere(mask)
    if not len(coords):
        return 0
    print(f"  {name}: {len(coords)} spike(s)")
    for c in coords[:25]:
        i = c[0]
        dims = c[1:]
        print(f"    t={times[i] - t0:7.3f}s  {dim_desc(tuple(dims))}  value={values[tuple(c)]:.5f}")
    return len(coords)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='Plot reference/command/state trajectories from a rosbag')
    ap.add_argument('bag', help='rosbag directory or .db3 file (zstd ok)')
    ap.add_argument('-o', '--out', default='rcd', help='output prefix for PNGs (default: rcd)')
    ap.add_argument('--spike-joint', type=float, default=0.02, help='joint spike threshold (rad)')
    ap.add_argument('--spike-ee', type=float, default=0.005, help='EE position spike threshold (m)')
    ap.add_argument('--spike-rpy', type=float, default=0.05, help='EE orientation spike threshold (rad)')
    ap.add_argument('--csv', help='optional merged CSV output')
    args = ap.parse_args()

    print(f"Loading bag: {args.bag}")
    teleop_df, state_df, cmd_df = load_bag(args.bag)
    print(f"  teleop: {len(teleop_df)} msgs, state: {len(state_df)} msgs, command: {len(cmd_df)} msgs")
    if teleop_df.empty:
        print("ERROR: no /dualarm_teleop_cmd messages in bag"); sys.exit(1)
    if cmd_df is None or state_df.empty:
        print("ERROR: missing joints_desired or joint_states"); sys.exit(1)

    kin_left = PandaDHKinematics('left')
    kin_right = PandaDHKinematics('right')

    print("Rebuilding reference from teleop commands...")
    ref = build_reference(teleop_df, state_df, kin_left, kin_right)
    print(f"  reference points: {len(ref)}")

    # FK for all three values
    q_ref = ref[[f'q{i}' for i in range(14)]].to_numpy()
    q_cmd = cmd_df[[f'q{i}' for i in range(14)]].to_numpy()
    q_st = state_df[[f'q{i}' for i in range(14)]].to_numpy()
    pos, rpy = fk_ee(ref['t'].to_numpy(), q_ref, kin_left, kin_right)
    pos_cmd, rpy_cmd = fk_ee(cmd_df['t'].to_numpy(), q_cmd, kin_left, kin_right)
    pos_state, rpy_state = fk_ee(state_df['t'].to_numpy(), q_st, kin_left, kin_right)

    # spike detection
    spikes = {
        'ref_joint': detect_spikes(ref['t'].to_numpy(), q_ref, args.spike_joint),
        'cmd_joint': detect_spikes(cmd_df['t'].to_numpy(), q_cmd, args.spike_joint),
        'state_joint': detect_spikes(state_df['t'].to_numpy(), q_st, args.spike_joint),
        'ref_pos': detect_spikes(ref['t'].to_numpy(), pos, args.spike_ee),
        'cmd_pos': detect_spikes(cmd_df['t'].to_numpy(), pos_cmd, args.spike_ee),
        'state_pos': detect_spikes(state_df['t'].to_numpy(), pos_state, args.spike_ee),
        'ref_rpy': detect_spikes(ref['t'].to_numpy(), rpy, args.spike_rpy),
        'cmd_rpy': detect_spikes(cmd_df['t'].to_numpy(), rpy_cmd, args.spike_rpy),
        'state_rpy': detect_spikes(state_df['t'].to_numpy(), rpy_state, args.spike_rpy),
    }

    print("\n=== Spike report ===")
    total = 0
    for name, t, mask, vals, ddesc in [
        ('reference joints', ref['t'].to_numpy(), spikes['ref_joint'], q_ref,
         lambda i: f"q{i % 14 + 1}"),
        ('command joints', cmd_df['t'].to_numpy(), spikes['cmd_joint'], q_cmd,
         lambda i: f"q{i % 14 + 1}"),
        ('state joints', state_df['t'].to_numpy(), spikes['state_joint'], q_st,
         lambda i: f"q{i % 14 + 1}"),
        ('reference EE pos', ref['t'].to_numpy(), spikes['ref_pos'], pos.reshape(-1, 6),
         lambda i: f"arm{i // 6 % 2 + 1}/axis{i % 3 + 1}"),
    ]:
        total += report_spikes(name, t, mask, t0 := min(ref['t'][0], cmd_df['t'][0], state_df['t'][0]),
                               vals.ravel(), ddesc)
    print(f"Total spikes above thresholds: {total}")

    print(f"\nPlotting -> {args.out}_*.png")
    plot_trajectories(ref, cmd_df, state_df, pos, rpy, pos_cmd, pos_state, rpy_cmd, rpy_state,
                      spikes, args.out)

    if args.csv:
        # aligned to command grid
        grid = cmd_df[['t']].copy()
        for df, pre in ((ref, 'ref'), (state_df, 'state')):
            m = pd.merge_asof(grid, df[['t'] + [f'q{i}' for i in range(14)]],
                              on='t', direction='nearest', suffixes=('', '_x'))
            for i in range(14):
                grid[f'{pre}_q{i}'] = m[f'q{i}']
        for i in range(14):
            grid[f'cmd_q{i}'] = cmd_df[f'q{i}'].to_numpy()
        grid.to_csv(args.csv, index=False)
        print(f"CSV -> {args.csv}")


if __name__ == '__main__':
    main()
