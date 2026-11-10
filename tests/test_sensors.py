"""Regression tests for frame integrity and radar freshness, without a server."""

import unittest
from types import SimpleNamespace
import numpy as np
from core.models import VehicleGeometry, RangeState
from core.frame_sync import FrameSynchronizer
from perception.radar_sensor import RadarTracker


class SensorTests(unittest.TestCase):
    def test_camera_frames_are_never_mixed_or_lost_on_partial_arrival(self):
        sync = FrameSynchronizer(("front", "rear"), capacity=3)
        sync.push("front", SimpleNamespace(frame=10))
        sync.push("rear", SimpleNamespace(frame=11))
        self.assertIsNone(sync.pop_common())
        sync.push("front", SimpleNamespace(frame=11))
        group = sync.pop_common()
        self.assertEqual([item.frame for item in group.values()], [11, 11])

    def test_out_of_order_does_not_displace_newer_common_frames(self):
        sync = FrameSynchronizer(("a", "b"), capacity=2)
        for frame in [3, 4, 1]:
            sync.push("a", SimpleNamespace(frame=frame))
        sync.push("b", SimpleNamespace(frame=4))
        self.assertEqual(sync.pop_common()["a"].frame, 4)

    def tracker(self):
        g = VehicleGeometry(4, 1, 40, -2, half_length_m=4, half_width_m=1.5)
        return RadarTracker(g, ("front",))

    def test_slant_range_is_converted_to_horizontal_body_clearance(self):
        t = self.tracker()
        t.update("front", np.array([[5.0, 0.0, 1.0]]), frame=2, timestamp=1.0)
        obs = t.observations(1.0)["front"]
        self.assertAlmostEqual(obs.distance_m, 1.0)
        self.assertEqual(obs.state, RangeState.VALID)

    def test_empty_packet_clears_old_distance_without_marking_safe(self):
        t = self.tracker()
        t.update("front", np.array([[5.0, 0.0, 1.0]]), 1, 1.0)
        t.update("front", np.empty((0, 3)), 2, 1.05)
        obs = t.observations(1.05)["front"]
        self.assertIsNone(obs.distance_m)
        self.assertEqual(obs.state, RangeState.NO_RETURN)

    def test_stale_mark_and_expiry(self):
        t = self.tracker()
        t.update("front", np.array([[5.0, 0.0, 1.0]]), 1, 1.0)
        self.assertEqual(t.observations(1.25)["front"].state, RangeState.STALE)
        self.assertIsNotNone(t.observations(1.25)["front"].distance_m)
        self.assertIsNone(t.observations(1.6)["front"].distance_m)

    def test_new_closer_target_is_not_smoothed_away(self):
        t = self.tracker()
        t.update("front", np.array([[6.0, 0.0, 1.0]]), 1, 1.0)
        t.update("front", np.array([[4.2, 0.0, 1.0]]), 2, 1.05)
        self.assertAlmostEqual(t.observations(1.05)["front"].distance_m, 0.2)

    def test_ground_self_and_high_returns_are_rejected(self):
        t = self.tracker()
        t.update(
            "front",
            np.array([[5.0, 0.0, 0.0], [0.0, 0.0, 1.0], [5.0, 0.0, 5.0]]),
            1,
            1.0,
        )
        self.assertEqual(t.observations(1.0)["front"].state, RangeState.NO_RETURN)

    def test_left_and_right_corners_are_distinct(self):
        t = self.tracker()
        t.update("front", np.array([[5.0, -2.0, 1.0], [5.0, 3.0, 1.0]]), 1, 1.0)
        self.assertAlmostEqual(
            t.observations(1.0)["front_left"].distance_m, 1.1180339887
        )
        self.assertAlmostEqual(
            t.observations(1.0)["front_right"].distance_m, 1.8027756377
        )


class RadarTransformTests(unittest.TestCase):
    def test_capture_pose_not_current_mount_is_used_for_moving_sensor(self):
        from perception.radar_sensor import radar_points_to_vehicle

        detection = SimpleNamespace(depth=5.0, azimuth=0.0, altitude=0.0, velocity=0.0)
        capture = np.array(
            [
                [0.0, -1.0, 0.0, 10.0],
                [1.0, 0.0, 0.0, 20.0],
                [0.0, 0.0, 1.0, 1.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        inverse = np.array(
            [
                [1.0, 0.0, 0.0, -11.0],
                [0.0, 1.0, 0.0, -20.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        np.testing.assert_allclose(
            radar_points_to_vehicle([detection], capture, inverse), [[-1.0, 5.0, 1.0]]
        )


class RadarRangeLimitTests(unittest.TestCase):
    def test_returns_outside_configured_sensor_range_are_not_usable(self):
        from perception.radar_sensor import radar_points_to_vehicle

        detection = SimpleNamespace(
            depth=100.0, azimuth=0.0, altitude=0.0, velocity=0.0
        )
        self.assertEqual(
            len(radar_points_to_vehicle([detection], np.eye(4), np.eye(4))), 0
        )
