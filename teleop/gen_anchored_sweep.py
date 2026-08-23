#!/usr/bin/env python3
"""生成「锚定当前真实 EE」的 ±0.01 m 双臂同步前移轨迹 CSV。

本地新增的辅助脚本（不在远程仓库），只把内联 Python 固化成一行命令；
原版可执行链路仍是 record_ee_pose.py + csv_teleop_cartesian_absolute.py，
本脚本只负责在中间造一张时间表，不参与机器人控制。

用法（三个参数，抄文档照填即可）:
    python3 gen_anchored_sweep.py <base_ee.csv> <out_sweep.csv> <interval>

  base_ee.csv : D-1 用 record_ee_pose.py 录到的「当前真实末端位姿」（15 列表头）
  out_sweep.csv : 输出给 csv_teleop_cartesian_absolute.py 播放的轨迹表
  interval    : 相邻两行目标之间的秒数；缓速组填 2.0，快速组填 0.1

它做的事（也就是原文档里那段内联脚本的算术）:
  把录到的真实 EE 当原点，双臂 x 每步 +0.002 m、共 5 步到 +0.01 m（顶点停留
  HOLD 秒），再每步 -0.002 m 回到原点（停留 HOLD 秒）。左右手相对位置全程不变，
  这正是 relative_error_monitor 在这 6 组里得到 teleop_err ≈ 0 的原因。
"""
import sys
import csv
import numpy as np

STEP = 0.002      # 每步前移量 (m)
N_STEPS = 5       # 步数：总前移 = STEP*N_STEPS = 0.01 m
HOLD = 6.0        # 顶点 / 回程原点停留时间 (s)

# record_ee_pose.py 输出的 15 列表头顺序（播放器按列序读取，勿改顺序）
HEADER_TAIL = ['x_left', 'y_left', 'z_left', 'qx_left', 'qy_left', 'qz_left', 'qw_left',
               'x_right', 'y_right', 'z_right', 'qx_right', 'qy_right', 'qz_right', 'qw_right']


def mean(rows, names, n=20):
    return np.mean([[float(r[x]) for x in names] for r in rows[:n]], axis=0)


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(2)
    base_file, out_file, interval = sys.argv[1], sys.argv[2], float(sys.argv[3])

    rows = list(csv.DictReader(open(base_file)))
    if len(rows) < 5:
        print(f'ERROR: {base_file} 有效行太少（{len(rows)}），请至少录 3~5 s')
        sys.exit(1)

    # 把录到的若干帧取平均，作为锚点（真实 EE 的位置和姿态）
    base = np.concatenate([mean(rows, ['x_left', 'y_left', 'z_left']),
                           mean(rows, ['qx_left', 'qy_left', 'qz_left', 'qw_left']),
                           mean(rows, ['x_right', 'y_right', 'z_right']),
                           mean(rows, ['qx_right', 'qy_right', 'qz_right', 'qw_right'])])

    def pose(k):
        p = base.copy()
        p[0] += k * STEP   # 左臂 x 前移
        p[7] += k * STEP   # 右臂 x 前移
        return p

    # 去程：原点(0) -> +0.002 ... -> +0.01 m（每一步停留 interval 秒）
    pts = [(k * interval, pose(k)) for k in range(N_STEPS + 1)]
    # 回程：+0.008 -> ... -> 原点；顶点与回程原点各停留 HOLD 秒
    for k in range(N_STEPS - 1, -1, -1):
        pts.append((N_STEPS * interval + HOLD + (N_STEPS - k) * interval, pose(k)))
    pts.append((2 * N_STEPS * interval + HOLD, pose(0)))

    with open(out_file, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['timestamp'] + HEADER_TAIL)
        for ts, p in pts:
            w.writerow([f'{ts:.3f}'] + [f'{v:.6f}' for v in p])
    print(f'wrote {out_file}: {len(pts)} pts, apex l_x={base[0] + N_STEPS * STEP:.4f}')


if __name__ == '__main__':
    main()