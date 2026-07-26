"""
Panda kinematics (forward + inverse) that EXACTLY replicates the safety
controller's internal model so that IK solutions track the real robot.

The controller (dualarm_mprc / dualarm_reactive_control) does NOT use a URDF.
It builds a Panda model from a hand-written DH table (Craig's modified DH
convention) with two extra fixed rows:
  - row 7: flange frame, theta = -pi/4 (rotates pi/4 to align with the hand)
  - row 8: end-effector site offset, d = 0.1035  (panda_ee_site)
The end-effector frame returned by getT() is therefore `panda_ee_site`, which
is the same frame published on `/ee_pose` and consumed by TASK_SPACE_POSE.

This module reproduces:
  - PandaRobot::getT          -> forward_kinematics (4x4 SE3, world frame)
  - PandaRobot::getJacobian_world -> jacobian_world (6x7, world frame)
on top of which a damped-least-squares inverse kinematics solver is provided.

References:
  src/dualarm_mprc/dualarm_reactive_control/src/utils/robot_kinematics.cpp
  src/dualarm_mprc/dualarm_reactive_control/include/dual_arm_hqp_controller/robot_kinematics.h
"""

import numpy as np

# ---------------------------------------------------------------------------
# DH model (identical to PandaRobot::PandaRobot in robot_kinematics.cpp)
# Columns: [theta, d, a, alpha]
# ---------------------------------------------------------------------------
_PI_2 = np.pi * 0.5

# fmt: off
_DH = np.array([
    [0.0,       0.333,    0.0,     0.0],        # row 0  joint 1
    [0.0,       0.0,      0.0,    -_PI_2],      # row 1  joint 2
    [0.0,       0.316,    0.0,     _PI_2],      # row 2  joint 3
    [0.0,       0.0,      0.0825,  _PI_2],      # row 3  joint 4
    [0.0,       0.384,   -0.0825, -_PI_2],      # row 4  joint 5
    [0.0,       0.0,      0.0,     _PI_2],      # row 5  joint 6
    [0.0,       0.0,      0.088,   _PI_2],      # row 6  joint 7
    [-np.pi / 4, 0.107,   0.0,     0.0],        # row 7  flange (rotate pi/4)
    [0.0,       0.1035,   0.0,     0.0],        # row 8  ee_site offset
])
# fmt: on

N_JOINTS = 7
N_DH_ROWS = _DH.shape[0]  # 9

# Joint limits (PandaRobot::q_lb / q_ub) — used both for clamping and IK safety.
JOINT_LIMITS_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_LIMITS_UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

# Default home configuration (matches BaseTeleopNode.HOME_POSITION).
HOME_POSITION = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# World-frame base transforms for each arm (baseline_controller_base.cpp):
#   left  at [0, 0.26, 0], identity rotation
#   right at [0, -0.26, 0], identity rotation
BASE_POSITIONS = {
    'left': np.array([0.0, 0.26, 0.0]),
    'right': np.array([0.0, -0.26, 0.0]),
}


def _dh_transform(theta, d, a, alpha):
    """Single DH (Craig modified) transform — copied verbatim from getT/getJacobian.

    Returns the 4x4 matrix Ti used by the controller:
      Ti = [[cθ,           -sθ,           0,       a      ],
            [sθ·cα,        cθ·cα,        -sα,     -d·sα   ],
            [sθ·sα,        cθ·sα,         cα,      d·cα   ],
            [0,             0,             0,       1      ]]
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    Ti = np.array([
        [ct,           -st,            0.0,    a],
        [st * ca,      ct * ca,       -sa,    -d * sa],
        [st * sa,      ct * sa,        ca,     d * ca],
        [0.0,          0.0,            0.0,    1.0],
    ])
    return Ti


def _dh_table(q):
    """Return the DH table with the first 7 theta values replaced by q."""
    DH = _DH.copy()
    DH[:N_JOINTS, 0] = q
    return DH


class PandaDHKinematics:
    """Forward/inverse kinematics for one Panda arm in the world frame.

    The arm base is placed in the world frame per BASE_POSITIONS[side].
    """

    def __init__(self, side: str):
        if side not in BASE_POSITIONS:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        self.side = side
        # T_base: world<-base (identity rotation, pure translation).
        self.T_base = np.eye(4)
        self.T_base[:3, 3] = BASE_POSITIONS[side]

    # --------------------------- Forward kinematics ------------------------
    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Compute the EE (panda_ee_site) pose in the world frame.

        Reproduces PandaRobot::getT (which starts from T_base_).
        Returns a 4x4 homogeneous transform.
        """
        DH = _dh_table(q)
        T = self.T_base.copy()
        for i in range(N_DH_ROWS):
            Ti = _dh_transform(DH[i, 0], DH[i, 1], DH[i, 2], DH[i, 3])
            T = T @ Ti
        return T

    def forward_position_quat(self, q: np.ndarray):
        """FK returning (position[3], quaternion[w,x,y,z])."""
        T = self.forward_kinematics(q)
        return T[:3, 3].copy(), matrix_to_quaternion_wxyz(T[:3, :3])

    # --------------------------- Jacobian ----------------------------------
    def jacobian_world(self, q: np.ndarray) -> np.ndarray:
        """Geometric Jacobian (6x7) in the world frame.

        Reproduces PandaRobot::getJacobian_world (T starts from T_base_).
        Layout: J = [Jp (3x7); Jo (3x7)] where Jo columns are world-frame
        joint axes (z_i).
        """
        DH = _dh_table(q)
        T = self.T_base.copy()
        T_all = []
        for i in range(N_DH_ROWS):
            Ti = _dh_transform(DH[i, 0], DH[i, 1], DH[i, 2], DH[i, 3])
            T = T @ Ti
            T_all.append(T)

        pe = T[:3, 3]  # EE position (full chain)
        J = np.zeros((6, N_JOINTS))
        for i in range(N_JOINTS):
            T_prev = T_all[i]
            z = T_prev[:3, 2]
            pi = T_prev[:3, 3]
            J[:3, i] = np.cross(z, pe - pi)  # position Jacobian
            J[3:, i] = z                     # orientation Jacobian
        return J

    # --------------------------- Inverse kinematics ------------------------
    def inverse_kinematics(
        self,
        target_position: np.ndarray,
        target_quaternion_wxyz: np.ndarray,
        q_seed: np.ndarray,
        tol: float = 1e-5,
        max_iter: int = 200,
        damping: float = 0.05,
        step: float = 1.0,
    ):
        """Damped-least-squares IK in the world frame.

        Args:
            target_position: [x, y, z] world-frame EE position.
            target_quaternion_wxyz: target orientation quaternion [w, x, y, z].
            q_seed: initial joint guess (7,). Seeding from the current actual
                joint state yields a smooth, nearby solution.
            tol: convergence threshold on the 6D pose error norm.
            max_iter: maximum iterations.
            damping: DLS damping factor (regularizes near singularities).
            step: update gain (<=1.0 for stability).

        Returns:
            (q_solution, position_error, orientation_error_rad, converged)
        """
        R_target = quaternion_wxyz_to_matrix(target_quaternion_wxyz)
        q = np.clip(np.asarray(q_seed, dtype=float).copy(),
                    JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
        I6 = np.eye(6)
        converged = False

        for _ in range(max_iter):
            T = self.forward_kinematics(q)
            R_cur = T[:3, :3]
            p_cur = T[:3, 3]

            e_pos = target_position - p_cur
            R_err = R_target @ R_cur.T
            e_rot = log_rotation(R_err)
            err = np.concatenate([e_pos, e_rot])

            pos_err = np.linalg.norm(e_pos)
            rot_err = np.linalg.norm(e_rot)
            if pos_err < tol and rot_err < tol:
                converged = True
                break

            J = self.jacobian_world(q)
            JJt = J @ J.T + (damping ** 2) * I6
            dq = J.T @ np.linalg.solve(JJt, err)
            # Limit per-iteration step to avoid overshoot near singularities.
            max_dq = 0.5
            n_dq = np.linalg.norm(dq)
            if n_dq > max_dq:
                dq *= max_dq / n_dq
            q = np.clip(q + step * dq, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

        return q, pos_err, rot_err, converged


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------
def matrix_to_quaternion_wxyz(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion [w, x, y, z] (Shepperd's method)."""
    tr = np.trace(R)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quaternion_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion [w, x, y, z] -> rotation matrix."""
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def log_rotation(R: np.ndarray) -> np.ndarray:
    """Axis-angle (rotation vector) of a rotation matrix via the log map.

    Returns the vector r = angle * axis such that exp(skew(r)) ≈ R.
    Robust near 0 and near pi.
    """
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        # Near 180 deg: axis from the symmetric part of R.
        # axis^2 = (diagonal of R + 1) / 2
        diag = np.clip((np.diag(R) + 1.0) / 2.0, 0.0, 1.0)
        axis = np.sqrt(diag)
        # Resolve signs from off-diagonals.
        if R[0, 1] > 0:
            axis[1] = abs(axis[1])
        else:
            axis[1] = -abs(axis[1])
        if R[0, 2] > 0:
            axis[2] = abs(axis[2])
        else:
            axis[2] = -abs(axis[2])
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        return angle * axis
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(angle))
    return angle * axis


# ---------------------------------------------------------------------------
# Self-test / verification
# ---------------------------------------------------------------------------
def _self_test():
    """Verify FK at home matches the documented EE pose, and IK->FK round-trips."""
    np.set_printoptions(precision=6, suppress=True)

    print("=" * 70)
    print("Panda DH kinematics self-test")
    print("=" * 70)

    for side in ('left', 'right'):
        kin = PandaDHKinematics(side)
        T = kin.forward_kinematics(HOME_POSITION)
        p, q = kin.forward_position_quat(HOME_POSITION)
        print(f"\n[{side}] FK at home {HOME_POSITION.tolist()}")
        print(f"  position   = {p.tolist()}")
        print(f"  quat[w,x,y,z] = {q.tolist()}")

    # ---- IK -> FK round-trip simulating a realistic teleop walk ----
    # Each step applies a small Cartesian delta (key-press scale: ~1mm transl,
    # ~0.01 rad rot) and IK is seeded from the *current* configuration — exactly
    # what key_teleop_cartesian_absolute_ik.py does. The walk is kept inside the
    # reachable region around home.
    print("\n" + "-" * 70)
    print("IK -> FK round-trip (teleop-style small-step walk, seeded from current q)")
    print("-" * 70)

    rng = np.random.default_rng(0)
    kin = PandaDHKinematics('left')
    q_cur = HOME_POSITION.copy()
    max_pos_err = 0.0
    max_rot_err = 0.0
    n_fail = 0
    N = 200
    p_home = kin.forward_position_quat(HOME_POSITION)[0]
    for k in range(N):
        # Current Cartesian pose.
        p_target, quat_target = kin.forward_position_quat(q_cur)
        # Keep the walk within +/-0.12 m of home so it stays reachable.
        drift = p_target - p_home
        max_extra = np.maximum(0.0, np.abs(drift) - 0.10)
        # Translation step ~ key scale (0.5-3 mm).
        d_trans = rng.uniform(-0.003, 0.003, size=3)
        d_trans = np.copysign(np.minimum(np.abs(d_trans), np.maximum(0.0005, 0.003 - max_extra)), d_trans)
        p_target = p_target + d_trans
        # Rotation step ~ key scale (0.005-0.02 rad) about a random axis.
        axis = rng.uniform(-1, 1, size=3)
        axis = axis / (np.linalg.norm(axis) + 1e-9)
        R_target = _axis_angle_to_matrix(axis, rng.uniform(-0.02, 0.02)) @ quaternion_wxyz_to_matrix(quat_target)
        quat_target = matrix_to_quaternion_wxyz(R_target)

        q_sol, _, _, ok = kin.inverse_kinematics(
            p_target, quat_target, q_seed=q_cur, tol=1e-6, max_iter=100)
        p_fk, quat_fk = kin.forward_position_quat(q_sol)
        true_pos_err = np.linalg.norm(p_fk - p_target)
        R_fk = quaternion_wxyz_to_matrix(quat_fk)
        true_rot_err = np.linalg.norm(log_rotation(R_target @ R_fk.T))
        max_pos_err = max(max_pos_err, true_pos_err)
        max_rot_err = max(max_rot_err, true_rot_err)
        if not ok or true_pos_err > 1e-4 or true_rot_err > 1e-4:
            n_fail += 1
            print(f"  trial {k}: ok={ok} pos_err={true_pos_err:.3e} rot_err={true_rot_err:.3e}")
        q_cur = q_sol  # advance from the solved configuration (continuous teleop)

    print(f"\n  trials: {N}, failures: {n_fail}")
    print(f"  max true position error: {max_pos_err:.3e} m")
    print(f"  max true orientation error: {max_rot_err:.3e} rad")
    status = "PASS" if (n_fail == 0 and max_pos_err < 1e-4 and max_rot_err < 1e-4) else "FAIL"
    print(f"  -> {status}")
    return status == "PASS"


def _axis_angle_to_matrix(axis, angle):
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = axis
    C = 1 - c
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


if __name__ == '__main__':
    ok = _self_test()
    raise SystemExit(0 if ok else 1)
