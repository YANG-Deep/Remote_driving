import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
import unittest
import numpy as np
from core.models import VehicleGeometry, RangeObservation, RangeState
from perception.bev_stitcher import BevStitcher, resize_contain
from utils import camera_math


class DisplayTests(unittest.TestCase):
    def test_rectangular_bev_preserves_meter_scale_in_both_axes(self):
        s = BevStitcher(width_px=300, height_px=500, px_per_m=25.0)
        self.assertEqual(s.ground_to_canvas(0, 0), (150, 250))
        self.assertEqual(s.ground_to_canvas(1, 1), (175, 225))
        self.assertEqual(s.canvas_to_ground(175, 225), (1.0, 1.0))
        self.assertEqual(s.stitch({}).shape, (500, 300, 3))
        self.assertFalse(s.last_coverage.any())

    def test_camera_contains_all_original_edges(self):
        image = np.full((100, 100, 3), 255, np.uint8)
        image[0] = (0, 0, 255)
        image[-1] = (0, 255, 0)
        result = resize_contain(image, 200, 100)
        np.testing.assert_array_equal(result[0, 50], [0, 0, 255])
        np.testing.assert_array_equal(result[-1, 149], [0, 255, 0])

    def test_ground_projection_can_keep_out_of_frame_endpoints(self):
        k = camera_math.make_intrinsics(100, 100, 90)
        point = camera_math.ground_to_pixel(1, 10, k, (0, 0, 1), (0, 0, 0), clip=False)
        self.assertAlmostEqual(point[0], 550.0)
        self.assertIsNone(camera_math.ground_to_pixel(1, 10, k, (0, 0, 1), (0, 0, 0)))

    def test_hud_handles_unknown_and_expired_distances(self):
        from visualization.hud import Hud
        from visualization.distance_overlay import format_sector_summary

        s = BevStitcher(width_px=300, height_px=500)
        hud = Hud(width=960, height=600)
        try:
            observations = [
                RangeObservation("front", 0.5, RangeState.VALID, 1, 1),
                RangeObservation("left", None, RangeState.NO_RETURN, 1, 1),
                RangeObservation("right", None, RangeState.STALE, 0, 0),
            ]
            summary = format_sector_summary(observations)
            self.assertIn("0.5", summary)
            self.assertNotIn("无障碍", summary)
            hud.render(s.stitch({}), {}, [], observations, {"相机": "过期"}, s, False)
        finally:
            hud.close()

    def test_stale_image_does_not_receive_fresh_radar_geometry(self):
        from visualization.hud import Hud

        s = BevStitcher(width_px=300, height_px=500)
        hud = Hud(width=960, height=600)
        image = np.zeros(s.canvas_shape, np.uint8)
        hit = RangeObservation("front", 0.5, RangeState.VALID, 1, 1, (4.5, 0.0, 1.0))
        try:
            hud.render(image, {}, [], [hit], {}, s, False, image_age_s=0.3)
            self.assertFalse(image.any())
        finally:
            hud.close()
