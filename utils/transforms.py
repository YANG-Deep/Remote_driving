"""CARLA rigid transforms built from basis vectors, avoiding matrix binding inconsistencies."""

from __future__ import annotations
import numpy as np


def transform_matrix(pose) -> np.ndarray:
    """Actor/sensor local to world; columns are forward, right, up."""
    basis = [pose.get_forward_vector(), pose.get_right_vector(), pose.get_up_vector()]
    matrix = np.eye(4)
    matrix[:3, :3] = np.array([[v.x, v.y, v.z] for v in basis]).T
    matrix[:3, 3] = [pose.location.x, pose.location.y, pose.location.z]
    return matrix


def inverse_transform_matrix(pose) -> np.ndarray:
    matrix = transform_matrix(pose)
    rotation = matrix[:3, :3].T.copy()
    matrix[:3, 3] = -rotation @ matrix[:3, 3]
    matrix[:3, :3] = rotation
    return matrix
