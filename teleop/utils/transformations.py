"""
Transformation utilities for teleoperation.
Handles rotation representations and pose conversions.
"""

import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from typing import Tuple, Optional


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to rotation matrix.

    Args:
        q: Quaternion [x, y, z, w] or [w, x, y, z]

    Returns:
        3x3 rotation matrix
    """
    # Normalize quaternion
    q = q / np.linalg.norm(q)

    # Check if input is [w, x, y, z] format
    if len(q) == 4:
        # Assume [x, y, z, w] format (geometry_msgs standard)
        x, y, z, w = q[0], q[1], q[2], q[3]
    else:
        raise ValueError("Quaternion must have 4 elements")

    # Compute rotation matrix
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
    ])

    return R


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert rotation matrix to quaternion.

    Args:
        R: 3x3 rotation matrix

    Returns:
        Quaternion [x, y, z, w]
    """
    # Trace of the matrix
    tr = np.trace(R)

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    return np.array([x, y, z, w])


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions.

    Args:
        q1: First quaternion [x, y, z, w]
        q2: Second quaternion [x, y, z, w]

    Returns:
        Quaternion product q1 * q2
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def small_angle_to_delta_rotation(
    delta_rot: np.ndarray
) -> np.ndarray:
    """
    Convert small angle increments to a quaternion delta.

    For small angles, the quaternion is approximately [rx/2, ry/2, rz/2, 1].

    Args:
        delta_rot: [rx, ry, rz] small rotation increments in radians

    Returns:
        Quaternion [x, y, z, w] representing the incremental rotation
    """
    rx, ry, rz = delta_rot

    # For small angles, quaternion ≈ [rx/2, ry/2, rz/2, 1]
    # Normalize to get proper quaternion
    q = np.array([rx/2, ry/2, rz/2, 1.0])
    q = q / np.linalg.norm(q)

    return q


def pose_to_msg(
    position: np.ndarray,
    orientation: Optional[np.ndarray] = None
) -> Pose:
    """
    Convert position and orientation to Pose message.

    Args:
        position: [x, y, z] position
        orientation: [x, y, z, w] quaternion (optional, defaults to identity)

    Returns:
        Pose message
    """
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])

    if orientation is not None:
        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])
    else:
        # Identity quaternion
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

    return pose


def pose_from_msg(pose: Pose) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract position and orientation from Pose message.

    Args:
        pose: Pose message

    Returns:
        Tuple of (position [x,y,z], quaternion [x,y,z,w])
    """
    position = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z
    ])

    quaternion = np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])

    return position, quaternion


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion.

    Args:
        q: Quaternion [x, y, z, w]

    Returns:
        Normalized quaternion
    """
    norm = np.linalg.norm(q)
    if norm < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / norm
