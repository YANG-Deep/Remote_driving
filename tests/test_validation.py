import unittest
from core.models import RangeObservation, RangeState, VehicleGeometry
from perception.validation import compare_ranges, edge_rays


class ValidationTests(unittest.TestCase):
    def test_reference_misses_cannot_become_radar_values(self):
        radar = [RangeObservation("front", None, RangeState.NO_RETURN, 1, 1)]
        reference = {"front": 1.0}
        report = compare_ranges(radar, reference)
        self.assertEqual(report["front"]["status"], "miss")
        self.assertIsNone(radar[0].distance_m)

    def test_error_is_signed_and_stale_measurements_not_scored(self):
        radar = [
            RangeObservation("left", 1.2, RangeState.VALID, 1, 1),
            RangeObservation("rear", 2.0, RangeState.STALE, 0.5, 0),
        ]
        report = compare_ranges(radar, {"left": 1.0, "rear": 1.0})
        self.assertAlmostEqual(report["left"]["error_m"], 0.2)
        self.assertEqual(report["rear"]["status"], "unavailable")

    def test_rays_cover_long_body_sides_at_multiple_heights(self):
        g = VehicleGeometry(4, 1, 40, -2, half_length_m=4, half_width_m=1.5)
        rays = edge_rays(g, spacing_m=1.0, heights=(0.3, 1.2), range_m=4.0)
        self.assertTrue(
            any(abs(a[0] - 3) < 0.01 and a[1] < -1.5 and b[1] < a[1] for a, b in rays)
        )
        self.assertEqual(set(round(a[2], 1) for a, b in rays), {0.3, 1.2})

    def test_diagonal_reference_rays_do_not_exceed_requested_range(self):
        import math

        g = VehicleGeometry(4, 1, 40, -2)
        for start, end in edge_rays(g, range_m=8.0):
            self.assertLessEqual(math.dist(start, end), 8.0)

    def test_reference_discards_hits_at_ray_origin(self):
        from types import SimpleNamespace
        import carla
        from perception.validation import ReferenceValidator

        validator = ReferenceValidator.__new__(ReferenceValidator)
        validator.geometry = VehicleGeometry(4, 1, 40, -2, half_width_m=1.5)
        validator.rays = [((0, -1.52, 1), (0, -5, 1))]
        validator.excluded = set()
        validator.world = SimpleNamespace(
            cast_ray=lambda origin, target: [
                SimpleNamespace(location=origin, label=carla.CityObjectLabel.Truck)
            ]
        )
        vehicle = SimpleNamespace(get_transform=lambda: carla.Transform())
        self.assertEqual(validator.sample(vehicle), {})
