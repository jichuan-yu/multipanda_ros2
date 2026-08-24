#!/usr/bin/env python3
"""
plot_all.py — 一键出某次实验的整套图（不必再分别跑 F1/F2/F3 三个终端命令）。

用法:
    python3 src/multipanda_ros2/teleop/plot_all.py exp0X
    python3 src/multipanda_ros2/teleop/plot_all.py data/mprc/exp0X/exp0X.csv
    # 或 workspace 根目录下的 mkfigs:  ./mkfigs exp0X

自动按顺序执行:
    F1 plot_experiment_fk.py   -> 10 张 FK 分层图（ref 层自动叠加 robot{1,2}_reference_<exp>.csv）
    F2 plot_experiment.py      -> 经典 7 张图（自动叠加 monitor 相对误差面板）
    F3 plot_relative_error.py  -> monitor 快照（若 <exp>/<exp>_relative_error.csv 存在）
输出全部进 data/mprc/<exp>_figs/（三个绘图脚本都不需要 ROS，纯 pandas/matplotlib）。

可选开关: --skip-fk / --skip-classic / --skip-rel, -o 自定义图目录。
"""
import argparse
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))


def _workspace_root():
    # _HERE = <ws>/src/multipanda_ros2/teleop  -> 返回 <ws>
    return os.path.dirname(os.path.dirname(os.path.dirname(_HERE)))


def main():
    ap = argparse.ArgumentParser(description="一键出某次实验整套图 (F1 fk + F2 经典 + F3 monitor)")
    ap.add_argument("exp", help="实验前缀 (如 exp0X) 或整链 CSV 路径")
    ap.add_argument("-o", "--out", default=None, help="图目录 (默认 <data>/<exp>_figs)")
    ap.add_argument("--skip-fk", action="store_true", help="跳过 F1 FK 10 图")
    ap.add_argument("--skip-classic", action="store_true", help="跳过 F2 经典 7 图")
    ap.add_argument("--skip-rel", action="store_true", help="跳过 F3 monitor 快照")
    args = ap.parse_args()

    # --- 定位整链 CSV ---
    arg = args.exp
    if arg.endswith(".csv"):
        csv = arg if os.path.isabs(arg) else os.path.join(_workspace_root(), arg)
        exp = os.path.splitext(os.path.basename(csv))[0]
    else:
        exp = arg
        csv = os.path.join(_workspace_root(), "data", "mprc", exp, exp + ".csv")
    if not os.path.isfile(csv):
        print("[plot_all] 找不到整链 CSV: %s" % csv)
        print("[plot_all]   用法: plot_all.py exp0X   (expects data/mprc/exp0X/exp0X.csv)")
        return 1
    csv = os.path.abspath(csv)

    # --- 图目录 ---（与 plot_experiment_fk 的布局感知默认值一致）
    sys.path.insert(0, _HERE)
    from plot_experiment_fk import _layout_out_dir  # noqa: E402
    figs = os.path.abspath(args.out) if args.out else _layout_out_dir(csv)
    os.makedirs(figs, exist_ok=True)

    # --- 拼步骤 ---
    rel_csv = os.path.join(os.path.dirname(csv), exp + "_relative_error.csv")
    rel_exists = os.path.isfile(rel_csv)
    steps = []
    if not args.skip_fk:
        steps.append(("F1 FK 10 图", [sys.executable, os.path.join(_HERE, "plot_experiment_fk.py"),
                                      csv, "-o", figs]))
    if not args.skip_classic:
        steps.append(("F2 经典 7 图", [sys.executable, os.path.join(_HERE, "plot_experiment.py"),
                                       csv, "-o", figs]))
    if not args.skip_rel:
        if rel_exists:
            steps.append(("F3 monitor 快照", [sys.executable, os.path.join(_HERE, "plot_relative_error.py"),
                                              rel_csv, "-o", os.path.join(figs, exp + "_relative_error_plot.png")]))
        else:
            print("[plot_all] 未找到 %s，跳过 F3" % rel_csv)
    if not steps:
        print("[plot_all] 无事可做 (全被 --skip 跳过)")
        return 0

    print("[plot_all] 实验 %s" % exp)
    print("[plot_all] 整链 CSV : %s" % csv)
    print("[plot_all] 图目录   : %s" % figs)
    for name, cmd in steps:
        print("\n===== %s =====" % name)
        r = subprocess.call(cmd)
        if r != 0:
            print("[plot_all] %s 失败 (exit=%d)" % (name, r))
            return r
    print("\n[plot_all] 完成 %d 步 -> %s" % (len(steps), figs))
    return 0


if __name__ == "__main__":
    sys.exit(main())