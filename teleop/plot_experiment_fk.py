#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plot_experiment_fk.py — 一张全链 CSV 出 10 张 FK 分层对比图。

输入: record_experiment.py 生成的全链 CSV。
  层:
    teleop : tel_raw0..13 (绝对关节目标, 仅 command_type==4 且该臂被选中)
    safe   : cmd_qL/R0..6 + cmd_dqL/R0..6 (q_c / q_dot_c, controller /mj_*/joints_desired)
    filt   : fqlL/R0..6 + fdqL/R0..6 (low-level filtered_joint_states, 整层没数据则跳过)
    actual : act_qL/R0..6 + act_dqL/R0..6 + accL/R0..6 (/joint_states)
    ref    : --refL/--refR 叠加 controller data_record 的 q_r (仅位姿/相对位姿图)
输出 10 张 PNG:
  01_fk_pos_x.png / 02_fk_pos_y.png / 03_fk_pos_z.png   左右臂 EE 位姿, L/R 两个子图, 各层同图
  04_fk_rel_x.png / 05_fk_rel_y.png / 06_fk_rel_z.png   相对位姿 rel=(T_L^-1 T_R) 平移量
  07_fk_vel_L.png / 08_fk_vel_R.png                     关节速度 safe/filt/actual 同图
  09_fk_acc_L.png / 10_fk_acc_R.png                     差分加速度 safe/filt/actual 同图

FK 口径与 controller 完全一致 (复用 PandaRobot::getT 同款 DH 表,
teleop/utils/panda_dh_kinematics.py), 因此本脚本 FK 位姿 == /ee_pose 口径。

用法:
  source ~/myenv/bin/activate
  python3 src/multipanda_ros2/teleop/plot_experiment_fk.py data/mprc/exp02.csv \
      -o data/mprc/exp02_fk [--refL data/mprc/robot1_reference_*.csv \
      --refR data/mprc/robot2_reference_*.csv] [--start 0 --end 20] [--show]

说明:
- 加速度均为 velocity 一阶差分 (100Hz), 第一拍为空。
- teleop 层只对 JOINT_POSITION (type=4) 的绝对关节目标做 FK, 避免增量/速度/任务空间
  指令被误画成目标位姿。
"""

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from utils.panda_dh_kinematics import PandaDHKinematics  # noqa: E402

JOINT = list(range(7))
KIN = {"L": PandaDHKinematics("left"), "R": PandaDHKinematics("right")}

# 层样式 (位姿图用颜色区分层; 速度/加速度图用关节色, 层用线型区分)
STYLE = {
    "teleop": dict(ls="-",  lw=1.6, alpha=0.90, label="teleop target (q_t)"),
    "ref":    dict(ls="-",  lw=0.8, alpha=0.55, label="ref q_r (data_record)"),
    "safe":   dict(ls="--", lw=1.1, alpha=1.00, label="safe cmd (q_c/dq_c)"),
    "filt":   dict(ls=":",  lw=1.1, alpha=0.90, label="filtered (low-level)"),
    "actual": dict(ls="-",  lw=1.0, alpha=1.00, label="actual (joint_states)"),
}
COLOR = {"teleop": "#1f77b4", "ref": "#7f7f7f", "safe": "#ff7f0e",
         "filt": "#2ca02c", "actual": "#d62728"}


def t_of(df):
    """优先 t_rel, 其次 t_sim 对齐到 0, 最后按行号*100Hz。"""
    if "t_rel" in df.columns and df["t_rel"].notna().any():
        return df["t_rel"].to_numpy()
    if "t_sim" in df.columns and df["t_sim"].notna().any():
        t = df["t_sim"].to_numpy()
        m = np.isfinite(t)
        return t - np.nanmin(t[m])
    return np.arange(len(df)) * 0.01


def load_q(df, base):
    """取 base∈{cmd_q, act_q, fql, cmd_dq, act_dq, fdq, acc} 的 L/R 矩阵。列缺失或全 NaN -> None。"""
    if not all(f"{base}L{j}" in df.columns for j in JOINT) or \
            not all(f"{base}R{j}" in df.columns for j in JOINT):
        return None, None
    qL = np.column_stack([df[f"{base}L{j}"] for j in JOINT]).astype(float)
    qR = np.column_stack([df[f"{base}R{j}"] for j in JOINT]).astype(float)
    if np.isnan(qL).all() and np.isnan(qR).all():
        return None, None
    return qL, qR


def load_teleop(df):
    """teleop 绝对关节目标: 仅 command_type==4 且该臂被选中 (arm: 1=L, 2=R, 3=both)。"""
    if not all(f"tel_raw{j}" in df.columns for j in range(14)) or \
            "tel_type" not in df.columns or "tel_arm" not in df.columns:
        return None, None
    q = df[[f"tel_raw{j}" for j in range(14)]].to_numpy(dtype=float)
    typ = df["tel_type"].to_numpy(dtype=float)
    arm = df["tel_arm"].to_numpy(dtype=float)
    qL = q[:, :7].copy()
    qR = q[:, 7:].copy()
    mL = (typ == 4) & np.isin(arm, [1, 3])
    mR = (typ == 4) & np.isin(arm, [2, 3])
    qL[~mL] = np.nan
    qR[~mR] = np.nan
    if np.isnan(qL).all() and np.isnan(qR).all():
        return None, None
    return qL, qR


def load_ref(path):
    """controller data_record reference CSV (列0=时间, 列1..7=q_r), 可含 NaN 行首。"""
    if not path:
        return None
    arr = pd.read_csv(path, header=None).to_numpy()
    return arr[:, 1:8].astype(float)


def fk_layer(qL, qR):
    """对 (qL, qR) 逐行 FK, 返回 (pL, pR, rel) 各 (N,3)。
    rel = (T_L^-1 T_R) 平移量 — 与 constraint_manager.relative_pose_error 口径一致。"""
    n = min(len(qL), len(qR))
    pL = np.full((n, 3), np.nan)
    pR = np.full((n, 3), np.nan)
    rel = np.full((n, 3), np.nan)
    for i in range(n):
        ql, qr = qL[i], qR[i]
        if not (np.isfinite(ql).all() and np.isfinite(qr).all()):
            continue
        TL = KIN["L"].forward_kinematics(ql)
        TR = KIN["R"].forward_kinematics(qr)
        pL[i] = TL[:3, 3]
        pR[i] = TR[:3, 3]
        rel[i] = (np.linalg.inv(TL) @ TR)[:3, 3]
    return pL, pR, rel


def finite_diff_acc(dq, t):
    """velocity 一阶差分 (第一拍 NaN)。"""
    acc = np.full_like(dq, np.nan)
    if dq is None or len(dq) < 2:
        return acc
    dt = np.diff(t)
    ok = dt > 1e-6
    if not ok.any():
        return acc
    acc[1:] = np.diff(dq, axis=0) / np.maximum(dt, 1e-6)[:, None]
    acc[1:][~ok] = np.nan
    return acc


def _valid(series, name):
    v = series.get(name)
    return v is not None and np.isfinite(v).any()


def series_all_blank(series):
    return not (_valid(series, "L") or _valid(series, "R") or _valid(series, "rel"))


def build_pose_series(df, refL, refR):
    """位姿层列表: 每层 dict(key, t, L, R, rel)。ref 层用自身等距时间轴。"""
    items = []
    qL, qR = load_teleop(df)
    if qL is not None:
        items.append(("teleop", qL, qR, t_of(df)))
    for base, key in (("cmd_q", "safe"), ("fql", "filt"), ("act_q", "actual")):
        qL, qR = load_q(df, base)
        if qL is not None:
            items.append((key, qL, qR, t_of(df)))
    if refL is not None or refR is not None:
        n = min(refL.shape[0] if refL is not None else 1 << 30,
                refR.shape[0] if refR is not None else 1 << 30)
        qL = refL[:n] if refL is not None else None
        qR = refR[:n] if refR is not None else None
        items.append(("ref", qL, qR, np.arange(n) * 0.01))
    out = []
    for key, qL, qR, t in items:
        if qL is None or qR is None:
            continue
        pL, pR, rel = fk_layer(qL, qR)
        s = dict(key=key, t=t, L=pL, R=pR, rel=rel)
        if series_all_blank(s):
            continue
        out.append(s)
    return out


def build_vel_series(df):
    t = t_of(df)
    out = []
    for base, key in (("cmd_dq", "safe"), ("fdq", "filt"), ("act_dq", "actual")):
        dqL, dqR = load_q(df, base)
        if dqL is None:
            continue
        s = dict(key=key, t=t, L=dqL, R=dqR, rel=None)
        if series_all_blank(s):
            continue
        out.append(s)
    return out


def build_acc_series(df):
    t = t_of(df)
    out = []
    for base, key in (("cmd_dq", "safe"), ("fdq", "filt"), ("act_dq", "actual")):
        dqL, dqR = load_q(df, base)
        if dqL is None:
            continue
        aL, aR = finite_diff_acc(dqL, t), finite_diff_acc(dqR, t)
        if key == "actual":
            accL, accR = load_q(df, "acc")
            if accL is not None:
                aL, aR = accL, accR  # 优先用录制时算好的实际加速度列
        s = dict(key=key, t=t, L=aL, R=aR, rel=None)
        if series_all_blank(s):
            continue
        out.append(s)
    return out


def _plot_pose(ax, s, side, axis):
    col = {"L": s["L"], "R": s["R"]}[side]
    st = STYLE[s["key"]]
    ax.plot(s["t"], col[:, axis], color=COLOR[s["key"]], ls=st["ls"],
            lw=st["lw"], alpha=st["alpha"], label=st["label"])


def fig_pos_axis(series, axis, out):
    fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    for side, ax in zip("LR", axs):
        for s in series:
            if s["L"] is None or s["R"] is None:
                continue
            _plot_pose(ax, s, side, axis)
        ax.set_ylabel("pos %s [m]" % "xyz"[axis], fontsize=9)
        ax.set_title("%s arm EE position %s — all layers" % (side, "xyz"[axis]))
        ax.legend(fontsize=8, ncol=3)
        ax.grid(alpha=0.3)
    axs[-1].set_xlabel("t [s]")
    fig.suptitle("EE position %s — teleop target / safe cmd / filtered / actual" % "xyz"[axis])
    fig.tight_layout()
    fig.savefig(os.path.join(out, "0%d_fk_pos_%s.png" % (axis + 1, "xyz"[axis])), dpi=150)
    plt.close(fig)


def fig_rel_axis(series, axis, out):
    fig, ax = plt.subplots(figsize=(13, 5))
    drew = False
    for s in series:
        if not _valid(s, "rel"):
            continue
        st = STYLE[s["key"]]
        ax.plot(s["t"], s["rel"][:, axis], color=COLOR[s["key"]], ls=st["ls"],
                lw=st["lw"], alpha=st["alpha"], label=st["label"])
        drew = True
    ax.set_xlabel("t [s]")
    ax.set_ylabel("rel %s [m]" % "xyz"[axis])
    ax.set_title("Relative pose %s — right EE w.r.t. left (T_L^-1 T_R)" % "xyz"[axis])
    if drew:
        ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "0%d_fk_rel_%s.png" % (axis + 4, "xyz"[axis])), dpi=150)
    plt.close(fig)
def _fig_mseries(series, side, kind, out, fname):
    """速度/加速度图: 每关节一颜色, 每层一线型。"""
    cmap = plt.get_cmap("tab10")
    fig, ax = plt.subplots(figsize=(13, 6))
    for s in series:
        y = {"L": s["L"], "R": s["R"]}[side]
        if y is None:
            continue
        st = STYLE[s["key"]]
        z = 2 if s["key"] == "actual" else 1
        for j in JOINT:
            ax.plot(s["t"], y[:, j], color=cmap(j), ls=st["ls"],
                    lw=st["lw"], alpha=st["alpha"], zorder=z)
    ax.set_xlabel("t [s]")
    ax.set_ylabel({"vel": "joint velocity [rad/s]",
                   "acc": "joint acceleration [rad/s^2]"}[kind])
    ax.set_title("%s arm joint %s — safe cmd / filtered / actual" % (side, kind))
    ax.grid(alpha=0.3)
    layers = [Line2D([0], [0], color="#555555", ls=STYLE[s["key"]]["ls"], lw=1.2,
                     label=STYLE[s["key"]]["label"]) for s in series]
    joints_ = [Line2D([0], [0], color=cmap(j), lw=1.2, label="j%d" % (j + 1))
               for j in JOINT]
    ax.legend(handles=layers + joints_, ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, fname), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="FK 分层对比: 位姿/相对位姿/速度/加速度 10 张图")
    ap.add_argument("csv", help="record_experiment.py 的全链 CSV")
    ap.add_argument("-o", "--out", default=None, help="输出目录 (默认 <csv名>_fk)")
    ap.add_argument("--refL", default=None, help="robot1_reference_*.csv (q_r)")
    ap.add_argument("--refR", default=None, help="robot2_reference_*.csv (q_r)")
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    if "cmd_qL0" not in df.columns or "act_qL0" not in df.columns:
        print("[plot_experiment_fk] 需要 record_experiment.py 的全链 CSV (缺 cmd_qL0/act_qL0)")
        return 1

    t = t_of(df)
    m0 = np.ones(len(df), dtype=bool)
    if args.start is not None:
        m0 &= t >= args.start
    if args.end is not None:
        m0 &= t <= args.end
    df = df[m0].reset_index(drop=True)

    out = args.out or (os.path.splitext(args.csv)[0] + "_fk")
    os.makedirs(out, exist_ok=True)

    refL, refR = load_ref(args.refL), load_ref(args.refR)
    pos_s = build_pose_series(df, refL, refR)
    vel_s = build_vel_series(df)
    acc_s = build_acc_series(df)

    if not pos_s:
        print("[plot_experiment_fk] 无任何有效 FK 位姿层 (数据全空?)")
        return 1

    for axis in range(3):
        fig_pos_axis(pos_s, axis, out)
        fig_rel_axis(pos_s, axis, out)
    if vel_s:
        _fig_mseries(vel_s, "L", "vel", out, "07_fk_vel_L.png")
        _fig_mseries(vel_s, "R", "vel", out, "08_fk_vel_R.png")
    if acc_s:
        _fig_mseries(acc_s, "L", "acc", out, "09_fk_acc_L.png")
        _fig_mseries(acc_s, "R", "acc", out, "10_fk_acc_R.png")

    print("[plot_experiment_fk] layers  pose=%s" % [s["key"] for s in pos_s])
    print("[plot_experiment_fk] layers  vel =%s" % [s["key"] for s in vel_s])
    print("[plot_experiment_fk] layers  acc =%s" % [s["key"] for s in acc_s])
    print("[plot_experiment_fk] done -> %s" % out)
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())