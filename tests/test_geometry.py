"""Physical geometry regressions: wrong axes/bumper anchors must fail."""

import unittest
import numpy as np
from core.models import VehicleGeometry
from utils.trajectory import predict_footprints


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.geometry = VehicleGeometry(
            4.0,
            1.0,
            40.0,
            -2.0,
            body_center_x_m=0.2,
            half_length_m=4.0,
            half_width_m=1.5,
            ground_z_m=-0.1,
        )

    def test_initial_body_corners_are_actor_local_not_rear_axle_local(self):
        footprint = predict_footprints(self.geometry, 20, False, 3, 0.2)[0]
        np.testing.assert_allclose(
            footprint, [(4.2, -1.5), (4.2, 1.5), (-3.8, 1.5), (-3.8, -1.5)]
        )

    def test_straight_reverse_translates_whole_body_backwards(self):
        footprints = predict_footprints(self.geometry, 0, True, 3, 0.2)
        np.testing.assert_allclose(
            footprints[-1], [(1.2, -1.5), (1.2, 1.5), (-6.8, 1.5), (-6.8, -1.5)]
        )

    def test_positive_steering_moves_front_right_and_swings_tail_left(self):
        footprint = predict_footprints(self.geometry, 20, False, 1, 0.1)[-1]
        self.assertGreater(footprint[0][1], -1.5)
        self.assertLess(footprint[3][1], -1.5)

    def test_opposite_turns_are_mirrored_and_keep_width(self):
        right = predict_footprints(self.geometry, 25, False, 3, 0.2)[-1]
        left = predict_footprints(self.geometry, -25, False, 3, 0.2)[-1]
        np.testing.assert_allclose(right[0], [left[1][0], -left[1][1]])
        self.assertAlmostEqual(np.linalg.norm(np.array(right[0]) - right[1]), 3)

    def test_zero_step_is_rejected(self):
        with self.assertRaises(ValueError):
            predict_footprints(self.geometry, 0, False, 3, 0)

    def test_gear_switch_rejects_opposite_direction_frame_overlay(self):
        from visualization.trajectory_overlay import build_frame_overlays

        paths, sweep = build_frame_overlays(self.geometry, 20.0, False, True, 0.05)
        self.assertEqual(paths, [])
        self.assertEqual(sweep, [])


class SpeedTests(unittest.TestCase):
    def test_vertical_settling_is_not_reported_as_driving_speed(self):
        from types import SimpleNamespace
        from core.vehicle_manager import VehicleManager

        manager = VehicleManager.__new__(VehicleManager)
        manager._reverse = False
        manager.vehicle = SimpleNamespace(
            get_velocity=lambda: SimpleNamespace(x=3.0, y=4.0, z=10.0)
        )
        self.assertAlmostEqual(manager.get_speed_kmh(), 18.0)


class CollisionEnvelopeTests(unittest.TestCase):
    def test_rear_collision_extension_affects_clearance_and_swept_body(self):
        g = VehicleGeometry(
            4,
            1,
            40,
            -2,
            half_length_m=4,
            half_width_m=1.5,
            rear_collision_extension_m=0.55,
        )
        self.assertAlmostEqual(g.clearance(-5.05, 0), 0.5)
        initial = predict_footprints(g, 0, False, 1, 0.1)[0]
        self.assertAlmostEqual(initial[2][0], -4.55)


class CurvatureCalibrationTests(unittest.TestCase):
    def test_measured_curvature_profile_changes_heading_and_preserves_mirroring(self):
        import math

        g = VehicleGeometry(
            4, 1, 70, -2, curvature_samples=((0.0, 0.0), (30.0, 0.1), (60.0, 0.3))
        )
        right = predict_footprints(g, 30, False, 1, 0.1)[-1]
        left = predict_footprints(g, -30, False, 1, 0.1)[-1]
        heading = math.atan2(right[0][1] - right[3][1], right[0][0] - right[3][0])
        self.assertAlmostEqual(heading, 0.1)
        self.assertAlmostEqual(right[0][1], -left[1][1])


class RearSlipTests(unittest.TestCase):
    def test_calibrated_rear_motion_can_include_lateral_scrub(self):
        import math

        g = VehicleGeometry(
            4,
            1,
            70,
            -2,
            curvature_samples=((0.0, 0.0), (30.0, 0.1)),
            rear_slip_samples=((0.0, 0.0), (30.0, 0.1)),
        )
        polygon = predict_footprints(g, 30, False, 0.001, 0.001)[-1]
        cy = sum(y for x, y in polygon) / 4
        rear_y = cy - 2 * math.sin(0.0001)
        self.assertGreater(rear_y / 0.001, 0.09)
        self.assertLess(rear_y / 0.001, 0.11)
