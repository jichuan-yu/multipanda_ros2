#!/usr/bin/env python3
"""
plot_experiment.py — 一次实验一张图的看图工具：从一张 CSV 一次生成整组 PNG（多子图、分文件）

输入自动识别两种格式：
  A) record_experiment.py 生成的全链 CSV（含 cmd_qL0..6 / act_qL0..6 / dev / acc / ee / min_distance 列）
  B) relative_error_monitor 的旧版相对误差 CSV（含 teleop_err_norm / safe_err_norm 列，如 exp01_relative_error.csv）

用法 (建议新布局: 一次实验的 CSV 全在 data/mprc/<exp>/, 图全进 data/mprc/<exp>_figs/):
    source ~/myenv/bin/activate
    # 全链模式：一次生成 01_joint_pos.png ... 07_ee_pose.png ... 等一组图
    python3 src/multipanda_ros2/teleop/plot_experiment.py data/mprc/exp02/exp02.csv
    #   --refL/--refR/--rel 与 -o 均可省: 新布局下同目录自动找
    #   robot{1,2}_reference_exp02.csv 与 exp02_relative_error.csv,
    #   图默认进 data/mprc/exp02_figs/
    # 相对误差模式（旧数据，单文件出图）
    python3 src/multipanda_ros2/teleop/plot_experiment.py data/mprc/exp01_relative_error.csv
    # 可选叠加: --refL/--refR 是 controller data_record 里 robot1/robot2_reference_*.csv（q_r）
    #           --rel CSV 是 relative_error_monitor 输出，用于相对误差面板对齐
    #           --start/--end 时间窗(相对 CSV 起点)
输出: out 目录下 markdown 化的多个 PNG（一次出多个图）。

说明:
- 时间轴优先取 t_rel 列（相对 recorder 首帧）；reference CSV (q_r) 用其绝对 sim 秒减 recorder
  首帧绝对 sim 秒 (t_sim) 对齐到同一 t_rel 轴；recorder 无 t_sim 列时退回首行对齐。
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

JOINT = list(range(7))


def t_of(df):
    """返回 (t_sec_array, t_colname) ：优先 t_rel，其次 t_sim 对齐到 0，最后用行号*采样率。"""
    if "t_rel" in df.columns and df["t_rel"].notna().any():
        return df["t_rel"].to_numpy(), "t_rel"
    if "t_sim" in df.columns and df["t_sim"].notna().any():
        t = df["t_sim"].to_numpy()
        m = np.isfinite(t)
        return t - np.nanmin(t[m]), "t_sim"
    return np.arange(len(df)) * 0.01, "row"


def _plot_joint_ax(ax, t, df, arm, kind="q", lims=None, ref=None):
    """一个臂一个子图，按 kind 选择列（颜色按关节索引统一）。"""
    cmap = plt.get_cmap("tab10")
    def P(col, **kw):
        if col in df.columns and df[col].notna().any():
            ax.plot(t, df[col], **kw)

    for j in JOINT:
        c = cmap(j)
        if kind == "q":
            P(f"cmd_q{arm}{j}", color=c, lw=1.0, ls="--", alpha=0.9)
            P(f"act_q{arm}{j}", color=c, lw=1.2)
        elif kind == "dq":
            P(f"cmd_dq{arm}{j}", color=c, lw=1.0)
            P(f"act_dq{arm}{j}", color=c, lw=0.6, alpha=0.5)
        elif kind == "acc":
            P(f"acc{arm}{j}", color=c, lw=1.0)
        elif kind == "dev":
            P(f"dev{arm}{j}", color=c, lw=1.0)
        elif kind == "dqerr":
            P(f"dq_err{arm}{j}", color=c, lw=1.0)
    if ref is not None:
        ax.axhline(ref, color="k", lw=0.6, ls=":")
        ax.axhline(-ref, color="k", lw=0.6, ls=":")
    ax.set_title(f"{arm} arm — {kind}")
    ax.grid(alpha=0.3)
    if lims:
        ax.set_ylim(*lims)


def mode_of(df):
    if "cmd_qL0" in df.columns:
        return "full"
    if "teleop_err_norm" in df.columns:
        return "relonly"
    return None
def plot_full(df, t, out_dir, refL, refR, rel_df):
    """全链模式：一批 PNG。"""
    figs = {}

    # 01 关节位置（每臂一图，q_r 可选叠加；ref 时间轴绝对对齐到 recorder t_rel）
    f, axs = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    for arm, ax in zip("LR", axs):
        _plot_joint_ax(ax, t, df, arm, kind="q")
        ref = refL if arm == "L" else refR
        if ref is not None and len(ref[1]):
            for j in JOINT:
                ax.plot(ref[0], ref[1][:, j], color="gray", lw=0.5, alpha=0.5)
    axs[0].legend([f"cmd{q}" for q in range(7)] + [f"act{q}" for q in range(7)],
                  ncol=7, fontsize=8)
    f.suptitle("Joint position: dashed=cmd(safe) solid=actual gray=q_r")
    figs["01_joint_pos.png"] = f

    def gen(kind, label, ref_v, fname, ylim=None):
        f, a = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        for arm, ax in zip("LR", a):
            _plot_joint_ax(ax, t, df, arm, kind=kind, ref=ref_v)
        f.suptitle(label)
        figs[fname] = f

    gen("dq", "Joint velocity: cmd_dq(solid-ish)/ act_dq +-0.15(A.2)",
        0.15, "02_joint_vel.png")
    gen("acc", "Joint acc (one-step diff of act_dq) +-0.25(A.4)",
        0.25, "03_joint_acc.png")
    gen("dev", "dev = act_q - cmd_q,  +-0.3 = LARGE_DEVIATION", 0.3, "04_deviation.png")
    gen("dqerr", "dq_err = act_dq - cmd_dq (inner-loop lag)", None, "05_dq_err.png")

    # 06 EE 位姿
    if {"eeLx", "eeRx", "eeLz"}.issubset(df.columns):
        f, ax = plt.subplots(figsize=(13, 5))
        cmap = plt.get_cmap("tab10")
        for arm, off in (("L", 0), ("R", 3)):
            for k, ax_l in enumerate(("x", "y", "z")):
                c = f"ee{arm}{ax_l}"
                if c in df.columns:
                    ax.plot(t, df[c], lw=1.0, label=c, color=cmap(off + k))
        ax.legend(ncol=6, fontsize=8); ax.grid(alpha=0.3)
        ax.set_title("EE position (from /ee_pose)")
        figs["06_ee_pose.png"] = f

    # 07 相对误差（优先叠加 --rel 的 monitor CSV）
    f, ax = plt.subplots(figsize=(13, 6))
    n_lab = 0
    if rel_df is not None and not rel_df.empty:
        rt = t_of(rel_df)[0]
        if "teleop_err_norm" in rel_df.columns:
            s = rel_df["teleop_err_norm"] * 1e3
            m = s.notna()
            if m.any():
                ax.plot(rt[m], s[m], label="teleop_err(mm)")
                n_lab += 1
            s = rel_df["safe_err_norm"] * 1e3
            m = s.notna()
            if m.any():
                ax.plot(rt[m], s[m], label="safe_err(mm)")
                n_lab += 1
    if n_lab:
        ax.legend()
    ax.grid(alpha=0.3)
    ax.set_title("Relative pose error (teleop/safe vs baseline)")
    figs["07_relative_error.png"] = f

    return figs


def plot_relonly(df, t, out_dir):
    """旧 monitor CSV：相对误差 norm + 相对三轴。"""
    figs = {}
    f, ax = plt.subplots(figsize=(13, 6))
    any_p = False
    for col, lab in (("teleop_err_norm", "teleop_err(mm)"),
                     ("safe_err_norm", "safe_err(mm)")):
        if col not in df.columns:
            continue
        s = df[col] * 1e3
        m = s.notna()
        if m.any():
            ax.plot(t[m], s[m], lw=1.2, label=lab)
            any_p = True
    if not any_p:
        print("[plot_experiment] relonly mode: 无有效 err_norm 数据")
    ax.legend(); ax.grid(alpha=0.3)
    ax.set_title("Relative pose error norm (from monitor)")
    figs["01_rel_err_norm.png"] = f

    f, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for ax, c in zip(axs, ("x", "y", "z")):
        drew = False
        for pre, lab in (("teleop_rel_", "teleop"), ("safe_rel_", "safe"),
                         ("curr_rel_", "actual")):
            col = f"{pre}{c}"
            if col not in df.columns:
                continue
            s = df[col]
            m = s.notna()
            if m.any():
                ax.plot(t[m], s[m], lw=1.0, label=lab)
                drew = True
        ax.set_title(f"rel {c} (right-left EE)")
        if drew:
            ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    figs["02_rel_xyz.png"] = f
    return figs


def _layout_out_dir(csv_path):
    """布局感知的默认图目录:
    新布局 data/mprc/<exp>/<exp>.csv                -> data/mprc/<exp>_figs
    新布局 data/mprc/<exp>/<exp>_relative_error.csv -> data/mprc/<exp>_figs
    旧布局 data/mprc/<exp>.csv 等                    -> <csv 去扩展名>_figs
    """
    apath = os.path.abspath(csv_path)
    d = os.path.dirname(apath)
    stub = os.path.splitext(os.path.basename(apath))[0]
    if stub.endswith("_relative_error"):
        exp = stub[:-len("_relative_error")]
        if os.path.basename(d) == exp:
            return os.path.join(os.path.dirname(d), exp + "_figs")
    if os.path.basename(d) == stub:
        return os.path.join(os.path.dirname(d), stub + "_figs")
    return os.path.join(d, stub + "_figs")


def main():
    ap = argparse.ArgumentParser(description="一次CSV出整组图")
    ap.add_argument("csv", help="record_experiment 或 relative_error_monitor 的 CSV")
    ap.add_argument("-o", "--out", default=None, help="输出目录（默认 <csv名>_plots）")
    ap.add_argument("--rel", default=None, help="relative_error_monitor CSV（叠加相对误差面板）")
    ap.add_argument("--refL", default=None, help="robot1_reference_*.csv (q_r)")
    ap.add_argument("--refR", default=None, help="robot2_reference_*.csv (q_r)")
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    mode = mode_of(df)
    if mode is None:
        print("[plot_experiment] 无法识别 CSV 格式（缺 cmd_qL0 / teleop_err_norm 列）")
        return 1

    # recorder 首帧绝对 sim 秒 —— 必须在 start/end 过滤前取, 作为 ref 层绝对时间->t_rel 的锚点
    sim0 = None
    if "t_sim" in df.columns and pd.to_numeric(df["t_sim"], errors="coerce").notna().any():
        sim0 = float(pd.to_numeric(df["t_sim"], errors="coerce").iloc[0])

    t, _ = t_of(df)
    m0 = np.ones(len(t), dtype=bool)
    if args.start is not None:
        m0 &= t >= args.start
    if args.end is not None:
        m0 &= t <= args.end
    t = t[m0]
    df = df[m0].reset_index(drop=True)

    # 布局自动发现: 新布局 data/mprc/<exp>/<exp>.csv 时, 同目录自动找
    # <exp>_relative_error.csv (--rel) 与 robot{1,2}_reference_<exp>.csv (--refL/R)
    if mode == "full":
        exp_stub = os.path.splitext(os.path.basename(args.csv))[0]
        cdir = os.path.dirname(os.path.abspath(args.csv))
        if args.rel is None:
            cand = os.path.join(cdir, exp_stub + "_relative_error.csv")
            if os.path.isfile(cand):
                args.rel = cand
                print("[plot_experiment] 自动发现 --rel: %s" % cand)
        if args.refL is None:
            cand = os.path.join(cdir, "robot1_reference_%s.csv" % exp_stub)
            if os.path.isfile(cand):
                args.refL = cand
                print("[plot_experiment] 自动发现 --refL: %s" % cand)
        if args.refR is None:
            cand = os.path.join(cdir, "robot2_reference_%s.csv" % exp_stub)
            if os.path.isfile(cand):
                args.refR = cand
                print("[plot_experiment] 自动发现 --refR: %s" % cand)

    out_dir = args.out or _layout_out_dir(args.csv)
    os.makedirs(out_dir, exist_ok=True)

    def load_ref_with_time(path):
        """robot{1,2}_reference_*.csv: 列0=绝对 sim 秒, 列1..7=q_r。
        返回 (t_rel_aligned, q7) —— 减 recorder 首帧绝对 sim 秒; 无 t_sim 时退回首行对齐。"""
        arr = pd.read_csv(path, header=None).to_numpy(dtype=float)
        t_ref = arr[:, 0].copy()
        t_al = t_ref - sim0 if sim0 is not None else t_ref - t_ref[0]
        return t_al, arr[:, 1:8]

    refL = refR = None
    if args.refL:
        refL = load_ref_with_time(args.refL)
    if args.refR:
        refR = load_ref_with_time(args.refR)

    rel_df = pd.read_csv(args.rel) if args.rel else None

    figs = plot_full(df, t, out_dir, refL, refR, rel_df) if mode == "full" \
        else plot_relonly(df, t, out_dir)

    for name, fig in figs.items():
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, name), dpi=150)
        print(f"  saved {out_dir}/{name}")
    print(f"[plot_experiment] done: {len(figs)} figures -> {out_dir}")
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())