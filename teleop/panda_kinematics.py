"""
Franka Panda forward kinematics, geometric Jacobian, and damped-least-squares IK.

This module is a faithful Python port of the C++ ``PandaRobot`` kinematics used by
the dual-arm safe controller, so that an IK solve performed in a teleop script
matches what the controller does internally.

Source of truth (port these exactly):
  - src/dualarm_mprc/dualarm_reactive_control/src/utils/robot_kinematics.cpp
      * DH_ table (constructor)
      * Ti Craig modified-DH transform (used by getR / getT / getJacobian)
      * setBase / eul2Rotm
      * getT(q, T)        -> world-frame FK (starts from T_base_)
      * getJacobian(q, J) -> 6x7 geometric Jacobian in the BASE frame
  - src/dualarm_mprc/dualarm_reactive_control/src/dual_arm_safe_controller_sim.cpp
      * DualArmSafeControllerSim::computeJointFromTaskSpace()  -> DLS IK step

The IK method (``solve_ik_dls``) applies the same damped-least-squares pseudo-inverse
of the Jacobian as ``computeJointFromTaskSpace``:

    J# = J^T (J J^T + lambda^2 I)^-1,   lambda = 0.01
    task_err = [Delta_p ; rotvec(R_target R_current^T)]
    q <- q + J# task_err

and iterates it to convergence.  For the dual-arm setup the robot bases have zero
rotation (RPY = 0), so the base-frame Jacobian coincides with the world-frame one;
the world-frame pose error and the base-frame Jacobian are therefore consistent,
exactly as in the controller.
"""

import numpy as np

N_JOINTS = 7

# Craig's modified DH table for the Franka Panda (9 rows).
# Column order: [theta, d, a, alpha]  (matches the C++ trailing comment).
# Rows 0-6 are the 7 revolute joints; row 7 is the flange (fixed -pi/4 yaw to
# align with the hand frame); row 8 is the end-effector site offset (0.1035 m).
# The theta of rows 0-6 is overwritten by the joint angles q at evaluation time.
PANDA_DH = np.array(
    [
        [0.0, 0.333, 0.0, 0.0],  # joint 1
        [0.0, 0.0, 0.0, -np.pi * 0.5],  # joint 2
        [0.0, 0.316, 0.0, np.pi * 0.5],  # joint 3
        [0.0, 0.0, 0.0825, np.pi * 0.5],  # joint 4
        [0.0, 0.384, -0.0825, -np.pi * 0.5],  # joint 5
        [0.0, 0.0, 0.0, np.pi * 0.5],  # joint 6
        [0.0, 0.0, 0.088, np.pi * 0.5],  # joint 7
        [-0.7854, 0.107, 0.0, 0.0],  # flange (-pi/4)
        [0.0, 0.1035, 0.0, 0.0],  # EE site offset (panda_ee_site)
    ],
    dtype=float,
)

# Joint limits (rad), matching robot_kinematics.cpp / base_teleop.py.
Q_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])


def dh_transform(theta, d, a, alpha):
    """Craig modified-DH single-joint transform: Rx(alpha) Tx(a) Rz(theta) Tz(d)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def euler_xyz_to_rotation(rpy):
    """Fixed-axis XYZ rotation matrix Rx(r) Ry(p) Ry(y) (mirrors eul2Rotm)."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def quaternion_to_rotation(q):
    """Convert quaternion [w, x, y, z] to a 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_to_quaternion(R):
    """Convert a 3x3 rotation matrix to a unit quaternion [w, x, y, z] (Shepperd)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0  # s = 4 w
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0  # s = 4 x
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0  # s = 4 y
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0  # s = 4 z
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rotation_to_rotvec(R):
    """Rotation vector (angle * axis, angle in [0, pi]) of a rotation matrix.

    Goes through the quaternion to stay robust near angle = 0 and angle = pi.
    Mirrors Eigen's ``AngleAxisd(R)`` convention used by the controller.
    """
    q = rotation_to_quaternion(R)  # [w, x, y, z]
    w, v = q[0], q[1:]
    if w < 0.0:  # flip sign so the represented angle lies in [0, pi]
        w, v = -w, -v
    w = float(np.clip(w, -1.0, 1.0))
    angle = 2.0 * np.arccos(w)
    norm_v = np.linalg.norm(v)
    if norm_v < 1e-12 or angle < 1e-12:
        return np.zeros(3)
    return angle * (v / norm_v)


class PandaKinematics:
    """Franka Panda kinematics: world-frame FK and base-frame geometric Jacobian.

    A faithful port of ``PandaRobot`` (robot_kinematics.cpp). The robot base is
    configured via ``set_base``; defaults to identity (base at origin, no rotation).
    """

    def __init__(self):
        self.T_base = np.eye(4)

    def set_base(self, xyz, rpy=(0.0, 0.0, 0.0)):
        """Set the world pose of the robot base (mirrors PandaRobot::setBase)."""
        self.T_base = np.eye(4)
        self.T_base[:3, :3] = euler_xyz_to_rotation(rpy)
        self.T_base[:3, 3] = np.asarray(xyz, dtype=float)

    def _dh_table(self, q):
        DH = PANDA_DH.copy()
        DH[:N_JOINTS, 0] = q  # overwrite the first 7 thetas with the joint angles
        return DH

    def getT(self, q):
        """World-frame forward kinematics 4x4 homogeneous transform (starts at T_base)."""
        DH = self._dh_table(q)
        T = self.T_base.copy()
        for i in range(DH.shape[0]):
            T = T @ dh_transform(DH[i, 0], DH[i, 1], DH[i, 2], DH[i, 3])
        return T

    def getJacobian(self, q):
        """6x7 geometric Jacobian [Jp; Jo] in the BASE frame (starts at identity).

        For each revolute joint i: Jp_i = z_i x (p_e - p_i), Jo_i = z_i, where the
        end-effector point p_e is taken after all 9 DH rows (flange + EE site), so it
        is consistent with ``getT``. With zero base rotation this equals the
        world-frame Jacobian.
        """
        DH = self._dh_table(q)
        T = np.eye(4)
        T_all = []
        for i in range(DH.shape[0]):
            T = T @ dh_transform(DH[i, 0], DH[i, 1], DH[i, 2], DH[i, 3])
            T_all.append(T.copy())

        J = np.zeros((6, N_JOINTS))
        pe = T_all[-1][:3, 3]  # EE position (after all 9 rows)
        for i in range(N_JOINTS):
            T_prev = T_all[i]
            z = T_prev[:3, 2]
            pi = T_prev[:3, 3]
            J[:3, i] = np.cross(z, pe - pi)
            J[3:, i] = z
        return J


def solve_ik_dls(
    kin,
    target_pos,
    target_quat,
    q_seed,
    damping=0.01,
    max_iter=50,
    tol=1e-4,
):
    """Damped-least-squares IK to reach an absolute EE pose.

    Uses exactly the per-step update of the controller's ``computeJointFromTaskSpace``
    (``J# = J^T (J J^T + lambda^2 I)^-1`` with ``lambda = damping``), iterated to
    convergence. The pose error follows the controller convention:

        Delta_p   = p_target - R_target R_current^T p_current
        Delta_ori = rotvec(R_target R_current^T)
        q <- q + J# [Delta_p ; Delta_ori]

    Args:
        kin: PandaKinematics instance (base already configured).
        target_pos: 3-vector target EE position in the world frame.
        target_quat: target EE orientation as a quaternion [w, x, y, z].
        q_seed: 7-vector initial joint configuration (linearization point).
        damping: DLS damping factor (default 0.01, matches the controller).
        max_iter: maximum number of DLS iterations.
        tol: convergence tolerance on the 6-vector pose-error norm.

    Returns:
        (q_best, err_norm): the best joint solution found (lowest error, clamped to
        joint limits) and the norm of its residual pose error.
    """
    q = np.clip(np.asarray(q_seed, dtype=float).copy(), Q_LOWER, Q_UPPER)
    p_target = np.asarray(target_pos, dtype=float)
    R_target = quaternion_to_rotation(target_quat)
    I6 = np.eye(6)
    lam2 = damping * damping

    q_best = q.copy()
    err_best = np.inf

    for _ in range(max_iter):
        T = kin.getT(q)
        R_cur = T[:3, :3]
        p_cur = T[:3, 3]

        R_delta = R_target @ R_cur.T
        pos_err = p_target - R_target @ R_cur.T @ p_cur
        ori_err = rotation_to_rotvec(R_delta)
        err = np.concatenate([pos_err, ori_err])
        err_norm = float(np.linalg.norm(err))

        if err_norm < err_best:
            err_best = err_norm
            q_best = q.copy()
        if err_norm < tol:
            break

        J = kin.getJacobian(q)
        # DLS right pseudo-inverse step: dq = J^T (J J^T + lambda^2 I)^-1 err
        dq = J.T @ np.linalg.solve(J @ J.T + lam2 * I6, err)
        q = np.clip(q + dq, Q_LOWER, Q_UPPER)

    return q_best, err_best


def normalize_joint_angles(q):
    """Match DualArmSafeControllerSim::normalizeJointAngle for joint 4 and joint 6.

    MuJoCo reports joint angles normalized to [-pi, pi], but joint 4's range is
    entirely negative ([-3.0718, -0.0698]) and joint 6 wraps near pi. This brings
    received joint states into the same convention the controller uses.
    """
    q = np.asarray(q, dtype=float).copy()
    # Joint 4 (index 3): limit [-3.0718, -0.0698]
    if q[3] > 0.0:
        q[3] -= 2.0 * np.pi
    elif q[3] < -2.0 * np.pi:
        q[3] += 2.0 * np.pi
    # Joint 6 (index 5): limit [-0.0175, 3.7525]
    if q[5] < -0.0175:
        q[5] += 2.0 * np.pi
    elif q[5] > 3.7525 + np.pi:
        q[5] -= 2.0 * np.pi
    return q


if __name__ == "__main__":
    # Self-test: FK of the Panda home configuration should land near the
    # gripper-tip home pose used by the teleop scripts.
    HOME_Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

    kin_left = PandaKinematics()
    kin_left.set_base((0.0, 0.26, 0.0))
    kin_right = PandaKinematics()
    kin_right.set_base((0.0, -0.26, 0.0))

    for name, kin, expect in [
        ("left", kin_left, np.array([0.307, 0.26, 0.487])),
        ("right", kin_right, np.array([0.307, -0.26, 0.487])),
    ]:
        T = kin.getT(HOME_Q)
        p = T[:3, 3]
        quat = rotation_to_quaternion(T[:3, :3])  # [w, x, y, z]
        print(f"[{name}] FK(home) pos = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]")
        print(f"[{name}]          quat[w,x,y,z] = [{quat[0]:.4f}, {quat[1]:.4f}, "
              f"{quat[2]:.4f}, {quat[3]:.4f}]")
        print(f"[{name}]          expected pos = [{expect[0]:.4f}, {expect[1]:.4f}, "
              f"{expect[2]:.4f}]  (err = {np.linalg.norm(p - expect):.4e})")

    # Round-trip IK: solve for the home pose and check we recover HOME_Q.
    target_pos = kin_left.getT(HOME_Q)[:3, 3]
    target_quat = rotation_to_quaternion(kin_left.getT(HOME_Q)[:3, :3])
    q_seed = HOME_Q + np.array([0.05, -0.05, 0.05, -0.05, 0.05, -0.05, 0.05])
    q_sol, err = solve_ik_dls(kin_left, target_pos, target_quat, q_seed)
    print(f"\n[IK round-trip] err = {err:.4e}, max|q_sol - HOME| = "
          f"{np.max(np.abs(q_sol - HOME_Q)):.4e}")
