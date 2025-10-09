"""Frame conversion helpers between Assetto Corsa and ROS."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


ASSETTO_ROT_X_RAD = math.radians(90.0)
ASSETTO_ROT_Z_RAD = math.radians(-0.78229)
ASSETTO_TO_MAP_TRANSLATION = np.array([-30.849, 50.074, 0.0], dtype=float)
ASSETTO_WORLD_UP_VECTOR = np.array([0.0, 1.0, 0.0], dtype=float)
ASSETTO_LOCAL_TO_BASE = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=float
)
IDENTITY_QUATERNION = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
EPSILON = 1e-9


def assetto_to_map_transform() -> Tuple[np.ndarray, np.ndarray]:
    rx = ASSETTO_ROT_X_RAD
    rz = ASSETTO_ROT_Z_RAD

    cx, sx = math.cos(rx), math.sin(rx)
    cz, sz = math.cos(rz), math.sin(rz)

    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=float)
    rot_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=float)

    rotation = rot_z @ rot_x
    translation = ASSETTO_TO_MAP_TRANSLATION.copy()
    return rotation, translation


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    quaternion = np.array([qx, qy, qz, qw], dtype=float)
    return quaternion / np.linalg.norm(quaternion)


class FrameTransformer:
    """Utility class that caches common Assetto-to-ROS frame conversions."""

    def __init__(self) -> None:
        self.rotation_assetto_to_map, self.translation_assetto_to_map = assetto_to_map_transform()
        up_map = self.assetto_world_to_map_vector(ASSETTO_WORLD_UP_VECTOR)
        self.map_up_axis = up_map / np.linalg.norm(up_map)

    def world_to_map(self, world_position: np.ndarray) -> np.ndarray:
        return self.rotation_assetto_to_map @ world_position + self.translation_assetto_to_map

    def assetto_world_to_map_vector(self, vector: np.ndarray) -> np.ndarray:
        return self.rotation_assetto_to_map @ vector

    def assetto_local_to_base(self, vector: np.ndarray) -> np.ndarray:
        return ASSETTO_LOCAL_TO_BASE @ vector

    def orientation_from_contact_points(self, contact_points: np.ndarray) -> np.ndarray:
        points = np.asarray(contact_points, dtype=float).reshape(-1, 3)

        if points.shape[0] < 3:
            return IDENTITY_QUATERNION.copy()

        centroid = points.mean(axis=0)
        _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
        up_map = self.assetto_world_to_map_vector(vh[-1])

        up_norm = np.linalg.norm(up_map)
        if up_norm < EPSILON:
            return IDENTITY_QUATERNION.copy()
        up_map /= up_norm

        if np.dot(up_map, self.map_up_axis) < 0.0:
            up_map = -up_map

        forward_map = self.assetto_world_to_map_vector(
            points[:2].mean(axis=0) - points[2:].mean(axis=0)
        )
        forward_map -= np.dot(forward_map, up_map) * up_map

        forward_norm = np.linalg.norm(forward_map)
        if forward_norm < EPSILON:
            return IDENTITY_QUATERNION.copy()
        forward_map /= forward_norm

        left_map = np.cross(up_map, forward_map)
        left_norm = np.linalg.norm(left_map)
        if left_norm < EPSILON:
            return IDENTITY_QUATERNION.copy()
        left_map /= left_norm

        # Recompute the forward axis to ensure orthonormality of the basis.
        forward_map = np.cross(left_map, up_map)
        forward_norm = np.linalg.norm(forward_map)
        if forward_norm < EPSILON:
            return IDENTITY_QUATERNION.copy()
        forward_map /= forward_norm

        rotation_matrix = np.column_stack((forward_map, left_map, up_map))
        return rotation_matrix_to_quaternion(rotation_matrix)


__all__ = ["FrameTransformer", "assetto_to_map_transform", "rotation_matrix_to_quaternion"]
