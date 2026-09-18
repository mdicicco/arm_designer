from __future__ import annotations

import math

import numpy as np


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """ZYX Euler angles (roll, pitch, yaw) consistent with rpy_to_matrix."""
    R = np.asarray(R, dtype=float)
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-8
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def link_to_port_T(xyz: np.ndarray, rpy: tuple[float, float, float]) -> np.ndarray:
    R = rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    return make_T(R, np.asarray(xyz, dtype=float))


def mate_rotation_parent_to_child(clocking_rad: float) -> np.ndarray:
    """Rotation R_pc such that child port axes align with parent port when mated.

    Parent port P and child port C both have +Z outward. When mated: origins
    coincide, child Z equals -parent Z, and at zero clocking X axes match.
    Positive clocking rotates the child port about the parent port +Z axis.
    """
    align = np.diag([1.0, -1.0, -1.0])
    return rot_z(clocking_rad) @ align


def fixed_joint_T_parent_link_to_child_link(
    T_parentlink_parentport: np.ndarray,
    T_childlink_childport: np.ndarray,
    clocking_rad: float,
) -> np.ndarray:
    """Transform from parent link frame to child link frame (joint origin = child link origin convention)."""
    R_pc = mate_rotation_parent_to_child(clocking_rad)
    T_parentport_childport = make_T(R_pc, np.zeros(3))
    return T_parentlink_parentport @ T_parentport_childport @ inv_T(T_childlink_childport)
