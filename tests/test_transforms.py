"""Regressions observed against local CARLA build 3b938d7."""

import unittest
import carla
import numpy as np
from utils.transforms import transform_matrix, inverse_transform_matrix


class TransformTests(unittest.TestCase):
    def test_positive_yaw_maps_forward_to_right_not_left(self):
        pose = carla.Transform(carla.Location(10, 20, 1), carla.Rotation(yaw=90))
        np.testing.assert_allclose(
            transform_matrix(pose) @ [1, 0, 0, 1], [10, 21, 1, 1], atol=1e-6
        )

    def test_inverse_recovers_local_coordinates_at_rotated_translated_pose(self):
        pose = carla.Transform(carla.Location(10, 20, 1), carla.Rotation(yaw=90))
        np.testing.assert_allclose(
            inverse_transform_matrix(pose) @ [10, 21, 1, 1], [1, 0, 0, 1], atol=1e-6
        )

    def test_pitch_and_roll_match_carla_transform_action(self):
        pose = carla.Transform(
            carla.Location(10, 20, 1), carla.Rotation(pitch=15, yaw=37, roll=9)
        )
        point = pose.transform(carla.Location(2, 3, 4))
        np.testing.assert_allclose(
            transform_matrix(pose) @ [2, 3, 4, 1],
            [point.x, point.y, point.z, 1],
            atol=2e-6,
        )

    def test_radar_markers_are_aligned_to_camera_capture_pose(self):
        from core.models import RangeObservation, RangeState
        from perception.radar_sensor import align_observation_points

        current = carla.Transform(carla.Location(x=1))
        capture = carla.Transform()
        original = RangeObservation(
            "front", 1.0, RangeState.VALID, 1.0, 20, (5.0, 0.0, 1.0)
        )
        result = align_observation_points([original], current, capture)[0]
        np.testing.assert_allclose(result.point, [6.0, 0.0, 1.0])
        self.assertEqual(result.distance_m, 1.0)
        self.assertEqual(original.point, (5.0, 0.0, 1.0))
