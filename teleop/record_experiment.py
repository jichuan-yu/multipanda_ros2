#!/usr/bin/env python3
"""
record_experiment.py — 一次实验一条 CSV 的全链数据录制器（零编译，复用现有话题接口）

数据分层（同一张表内，按列名前缀区分，全部打 sim 时钟的统一时间戳）：

  teleop 层   : /dualarm_teleop_cmd  (std_msgs/Float64MultiArray)
                tel_raw0..13(原始14个数), tel_type, tel_arm, tel_allow
  safe 层输出 : /mj_left/joints_desired + /mj_right/joints_desired
                cmd_qL0..6, cmd_qR0..6      (q_c, 最终命令位置)
                cmd_dqL0..6, cmd_dqR0..6    (q_dot_c, controller 同消息发布)
  low-level   : /mj_left/filtered_joint_states + /mj_right/filtered_joint_states
                fqlL/R0..6, fdqL/R0..6      (滤波后 desired, 真正进入阻抗控制器的输入)
                —— 该话题是 ROS 世界话题, 话题没数据时这些列全为 NaN, 不影响其余列
  safe 层内部 : controller 的 data_record_ON 目录另有 q_r / HQP 解 / 计算时间
  底层执行    : /joint_states
                act_qL/R, act_dqL/R, act_tauL/R, accL/R(一阶差分估计)
  EE / 距离   : /ee_pose(14) + /min_distance(前9列)
  derived     : devL/R   = act_q - cmd_q（对照 LARGE_DEVIATION 阈值 0.3）
                dq_errL/R = act_dq - cmd_dq（内环跟踪滞后）

用法（仿真启动后、实验开始前运行，Ctrl+C 停止）:
    source ~/myenv/bin/activate
    python3 src/multipanda_ros2/teleop/record_experiment.py -o data/mprc/exp02
    # -> data/mprc/exp02.csv
建议与 controller data_record_ON 和 relative_error_monitor 同时跑，事后按 sim 时间对齐。

说明:
- 时间统一取 recorder 的 sim 时钟(use_sim_time=True)，各话题按"最新缓存"归并，
  与 controller 100Hz 发布节奏匹配。
- joint4/joint6 做与 controller/monitor 相同的 normalizeJointAngle。
- 加速度为 velocity 一阶差分估计；如需真值叠加 bag 记录 franka_states。
"""

import argparse
import csv
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

LEFT_PREFIX = "mj_left_joint"
RIGHT_PREFIX = "mj_right_joint"


def normalize_joint(q):
    """与 dual_arm_safe_controller_sim.cpp normalizeJointAngle 一致。"""
    q = q.copy()
    if q[3] > 0:
        q[3] -= 2 * np.pi
    elif q[3] < -2 * np.pi:
        q[3] += 2 * np.pi
    if q[5] < -0.0175:
        q[5] += 2 * np.pi
    elif q[5] > 3.7525 + np.pi:
        q[5] -= 2 * np.pi
    return q


def parse_js_pos_vel_eff(msg):
    """把 /joint_states 按 mj_left_joint1..7 / mj_right_joint1..7 拆成两臂。
    返回 (qL7, qR7, dqL7, dqR7, tauL7, tauR7)；缺失分量用 NaN。"""
    idx = {}
    for i in range(len(msg.name)):
        nm = msg.name[i]
        if nm.startswith(LEFT_PREFIX):
            k = (0, int(nm[len(LEFT_PREFIX):]) - 1)
        elif nm.startswith(RIGHT_PREFIX):
            k = (1, int(nm[len(RIGHT_PREFIX):]) - 1)
        else:
            continue
        if 0 <= k[1] < 7 and k not in idx:
            idx[k] = i
    qL = np.full(7, np.nan); qR = np.full(7, np.nan)
    dqL = np.full(7, np.nan); dqR = np.full(7, np.nan)
    tL = np.full(7, np.nan); tR = np.full(7, np.nan)
    for (arm, j), i in idx.items():
        q = msg.position[i] if i < len(msg.position) else np.nan
        dq = msg.velocity[i] if i < len(msg.velocity) else np.nan
        tau = msg.effort[i] if i < len(msg.effort) else np.nan
        if arm == 0:
            qL[j], dqL[j], tL[j] = q, dq, tau
        else:
            qR[j], dqR[j], tR[j] = q, dq, tau
    return normalize_joint(qL), normalize_joint(qR), dqL, dqR, tL, tR
class ExperimentRecorder(Node):
    def __init__(self, out_path, rate=100, verbose=False):
        super().__init__("experiment_recorder")
        self.set_parameters([rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self.out_path = os.path.abspath(out_path)
        if not self.out_path.endswith(".csv"):
            self.out_path += ".csv"
        os.makedirs(os.path.dirname(self.out_path), exist_ok=True)
        self.verbose = verbose
        self.period = 1.0 / rate

        # 最新消息缓存（按话题）
        self.teleop = None          # Float64MultiArray.data
        self.qL = self.qR = self.dqL = self.dqR = None
        self.tauL = self.tauR = None
        self.cqL = self.cqR = None  # 命令 (safe) 关节
        self.cdqL = self.cdqR = None
        self.fqlL = self.fqlR = None  # low-level 滤波输入 (filtered_joint_states)
        self.fdqL = self.fdqR = None
        self.ee = None              # /ee_pose 14 元素
        self.mdist = None           # /min_distance 前 9 元素

        self.accL = self.accR = np.full(7, np.nan)
        self._prev_dq = None
        self._prev_t = None
        self._start_t = None
        self._rows = 0

        prof = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Float64MultiArray, "/dualarm_teleop_cmd",
                                 self.cb_teleop, prof)
        self.create_subscription(JointState, "/joint_states", self.cb_joint, prof)
        self.create_subscription(JointState, "/mj_left/joints_desired",
                                 self.cb_cmd_left, prof)
        self.create_subscription(JointState, "/mj_right/joints_desired",
                                 self.cb_cmd_right, prof)
        self.create_subscription(Float64MultiArray, "/ee_pose", self.cb_ee, prof)
        self.create_subscription(Float64MultiArray, "/min_distance", self.cb_mdist, prof)
        self.create_subscription(JointState, "/mj_left/filtered_joint_states",
                                 self.cb_filt_left, prof)
        self.create_subscription(JointState, "/mj_right/filtered_joint_states",
                                 self.cb_filt_right, prof)

        self.f = open(self.out_path, "w", newline="", buffering=65536)
        self.w = csv.writer(self.f)
        self.w.writerow(self.columns())
        self.f.flush()

        self.create_timer(self.period, self.tick)
        self.get_logger().info(f"record_experiment -> {self.out_path} @ {rate:.0f}Hz")

    # ---------- 回调 ----------
    def cb_teleop(self, msg):
        self.teleop = list(msg.data)

    def cb_joint(self, msg):
        self.qL, self.qR, self.dqL, self.dqR, self.tauL, self.tauR = parse_js_pos_vel_eff(msg)

    def cb_cmd_left(self, msg):
        p = np.array(msg.position[:7], dtype=float) if len(msg.position) >= 7 else np.full(7, np.nan)
        v = np.array(msg.velocity[:7], dtype=float) if len(msg.velocity) >= 7 else np.full(7, np.nan)
        self.cqL, self.cdqL = normalize_joint(p), v

    def cb_cmd_right(self, msg):
        p = np.array(msg.position[:7], dtype=float) if len(msg.position) >= 7 else np.full(7, np.nan)
        v = np.array(msg.velocity[:7], dtype=float) if len(msg.velocity) >= 7 else np.full(7, np.nan)
        self.cqR, self.cdqR = normalize_joint(p), v

    def cb_filt_left(self, msg):
        p = np.array(msg.position[:7], dtype=float) if len(msg.position) >= 7 else np.full(7, np.nan)
        v = np.array(msg.velocity[:7], dtype=float) if len(msg.velocity) >= 7 else np.full(7, np.nan)
        self.fqlL, self.fdqL = normalize_joint(p), v

    def cb_filt_right(self, msg):
        p = np.array(msg.position[:7], dtype=float) if len(msg.position) >= 7 else np.full(7, np.nan)
        v = np.array(msg.velocity[:7], dtype=float) if len(msg.velocity) >= 7 else np.full(7, np.nan)
        self.fqlR, self.fdqR = normalize_joint(p), v

    def cb_ee(self, msg):
        if len(msg.data) >= 14:
            self.ee = list(msg.data[:14])

    def cb_mdist(self, msg):
        self.mdist = list(msg.data[:9])

    # ---------- 列定义 ----------
    @staticmethod
    def columns():
        cols = ["t_sim", "t_rel"]
        cols += ["tel_raw%d" % i for i in range(14)]
        cols += ["tel_type", "tel_arm", "tel_allow"]
        cols += ["cmd_qL%d" % i for i in range(7)] + ["cmd_qR%d" % i for i in range(7)]
        cols += ["cmd_dqL%d" % i for i in range(7)] + ["cmd_dqR%d" % i for i in range(7)]
        cols += ["act_qL%d" % i for i in range(7)] + ["act_qR%d" % i for i in range(7)]
        cols += ["act_dqL%d" % i for i in range(7)] + ["act_dqR%d" % i for i in range(7)]
        cols += ["act_tauL%d" % i for i in range(7)] + ["act_tauR%d" % i for i in range(7)]
        cols += ["accL%d" % i for i in range(7)] + ["accR%d" % i for i in range(7)]
        cols += ["devL%d" % i for i in range(7)] + ["devR%d" % i for i in range(7)]
        cols += ["dq_errL%d" % i for i in range(7)] + ["dq_errR%d" % i for i in range(7)]
        cols += ["fqlL%d" % i for i in range(7)] + ["fqlR%d" % i for i in range(7)]
        cols += ["fdqL%d" % i for i in range(7)] + ["fdqR%d" % i for i in range(7)]
        cols += ["eeLx", "eeLy", "eeLz", "eeRx", "eeRy", "eeRz"]
        cols += ["dL", "dR", "siL", "siR", "d_mutual", "rel_active", "viol", "d_active", "d_safe"]
        return cols

    # ---------- 100Hz 归并一行 ----------
    def tick(self):
        if self.qL is None:
            return  # 等第一条 /joint_states

        t = self.get_clock().now().nanoseconds * 1e-9
        if self._start_t is None:
            self._start_t = t
            self._prev_dq = np.concatenate([self.dqL, self.dqR])
            self._prev_t = t

        # 加速度估计 (实际速度一阶差分)
        cur = np.concatenate([self.dqL, self.dqR])
        dt = max(t - self._prev_t, 1e-4)
        acc = (cur - self._prev_dq) / dt
        self.accL, self.accR = acc[:7], acc[7:]
        self._prev_dq, self._prev_t = cur, t

        nan7 = np.full(7, np.nan)
        cmdq = np.concatenate([self.cqL if self.cqL is not None else nan7,
                               self.cqR if self.cqR is not None else nan7])
        cmddq = np.concatenate([self.cdqL if self.cdqL is not None else nan7,
                                self.cdqR if self.cdqR is not None else nan7])
        dev = np.concatenate([self.qL, self.qR]) - cmdq
        dq_err = np.concatenate([self.dqL, self.dqR]) - cmddq

        row = ["%.6f" % t, "%.6f" % (t - self._start_t)]
        tel = self.teleop if self.teleop is not None else [""] * 17
        row += ["%.6f" % v if isinstance(v, float) else str(v) for v in tel[:17]]

        vals = []
        for arr in (self.cqL, self.cqR, self.cdqL, self.cdqR,
                    self.qL, self.qR, self.dqL, self.dqR,
                    self.tauL, self.tauR, self.accL, self.accR,
                    dev[:7], dev[7:], dq_err[:7], dq_err[7:],
                    self.fqlL, self.fqlR, self.fdqL, self.fdqR):
            a = arr if arr is not None else nan7
            vals += ["%.6f" % v for v in a]
        row += vals

        if self.ee is not None:
            row += ["%.6f" % self.ee[i] for i in (0, 1, 2, 7, 8, 9)]
        else:
            row += ["nan"] * 6
        if self.mdist is not None:
            row += ["%.6f" % v for v in self.mdist]
        else:
            row += ["nan"] * 9

        self.w.writerow(row)
        self._rows += 1
        if self._rows % 500 == 0:
            self.f.flush()
            if self.verbose:
                self.get_logger().info(f"rows={self._rows}")

    def stop(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info(f"record_experiment done: {self._rows} rows -> {self.out_path}")


def main():
    ap = argparse.ArgumentParser(description="全链实验录制: teleop/safe命令/底层执行 一次一张CSV")
    ap.add_argument("-o", "--out", default="data/mprc/experiment",
                    help="输出前缀 (默认 data/mprc/experiment，会自动补 .csv)")
    ap.add_argument("--rate", type=float, default=100.0, help="归并采样率 Hz (默认 100)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    rclpy.init(args=None)
    node = ExperimentRecorder(args.out, args.rate, args.verbose)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()