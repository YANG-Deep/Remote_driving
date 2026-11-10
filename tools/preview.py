"""Offline geometry preview. Synthetic ground grid and ranges, never live acceptance evidence."""

from __future__ import annotations
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
import pygame
import config
from core.models import VehicleGeometry, RangeObservation, RangeState, SECTOR_BEARINGS
from perception.bev_stitcher import BevStitcher, camera_mounts
from utils import camera_math
from main import build_bev_base, overlay_bev
from visualization.hud import Hud
from visualization.trajectory_overlay import build_trajectory, build_swept_footprints


def ground_grid_frames(mounts, geometry):
    """Render only a flat known grid by ray/plane intersections, independently of BEV remap."""
    k = camera_math.make_intrinsics(
        config.CAMERA_WIDTH_PX, config.CAMERA_HEIGHT_PX, config.CAMERA_FOV_DEG
    )
    v, u = np.mgrid[: k.height, : k.width]
    frames = {}
    for key, (location, rotation) in mounts.items():
        forward, right, up = camera_math.carla_rotation_to_basis(*rotation)
        rays = (
            forward
            + (u[..., None] - k.cx) / k.fx * right
            - (v[..., None] - k.cy) / k.fy * up
        )
        ground = rays[:, :, 2] < -1e-6
        distance = (geometry.ground_z_m - location[2]) / np.minimum(
            rays[:, :, 2], -1e-6
        )
        x = location[0] + rays[:, :, 0] * distance
        y = location[1] + rays[:, :, 1] * distance
        frame = np.full((k.height, k.width, 3), (95, 80, 65), np.uint8)
        tile = (np.floor(x) + np.floor(y)).astype(np.int64) % 2
        floor = np.where(tile[:, :, None] == 0, [65, 70, 75], [80, 85, 90]).astype(
            np.uint8
        )
        marking = (np.abs(x - np.round(x)) < 0.025) | (np.abs(y - np.round(y)) < 0.025)
        floor[marking] = (175, 180, 185)
        # Known orange parking guides, not obstacle/distance evidence.
        floor[(np.abs(y - 2.6) < 0.04) | (np.abs(y + 2.6) < 0.04)] = (40, 175, 235)
        frame[ground] = floor[ground]
        frames[key] = frame
    return frames


def main():
    output = Path("artifacts/preview")
    output.mkdir(parents=True, exist_ok=True)
    config.WINDOW_TITLE = "离线几何预览 · 合成地面 / 模拟测距（非 CARLA 实测）"
    g = VehicleGeometry(4.5, 1.1, 40.0, -2.3, half_length_m=3.97, half_width_m=1.45)
    mounts = camera_mounts(g)
    hud = Hud()
    try:
        s = BevStitcher(
            width_px=hud._bev_rect.width,
            height_px=hud._bev_rect.height,
            geometry=g,
            mounts=mounts,
        )
        frames = ground_grid_frames(mounts, g)
        observations = [
            RangeObservation(sector, None, RangeState.NO_RETURN, 0, 0)
            for sector in SECTOR_BEARINGS
        ]
        observations[0] = RangeObservation("front", 0.8, RangeState.VALID, 0, 0)
        observations[6] = RangeObservation("left", 1.2, RangeState.VALID, 0, 0)
        for name, steer, reverse in [
            ("straight", 0.0, False),
            ("right", 20.0, False),
            ("left", -20.0, False),
            ("reverse", 20.0, True),
        ]:
            trajectory = build_trajectory(g, steer, reverse)
            bev = overlay_bev(
                s.stitch(frames),
                build_bev_base(s),
                trajectory,
                s,
                build_swept_footprints(g, steer, reverse),
            )
            hud.render(
                bev,
                frames,
                trajectory,
                observations,
                {"数据": "离线合成", "转角": f"{steer:+.0f}°", "测距": "模拟值"},
                s,
                reverse,
            )
            pygame.image.save(hud.screen, str(output / f"{name}.png"))
        print(s.coverage_report())
    finally:
        hud.close()


if __name__ == "__main__":
    main()
