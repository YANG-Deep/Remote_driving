"""Four-camera ground-plane remapping with explicit coverage and isotropic scale."""

from __future__ import annotations
import cv2
import numpy as np
import config
from core.models import VehicleGeometry
from utils import camera_math


def camera_mounts(geometry: VehicleGeometry) -> dict:
    """Four mounts in actor coordinates, set outside actual body bounds."""
    xmin, xmax, ymin, ymax = geometry.bounds
    cx, cy = geometry.body_center_x_m, geometry.body_center_y_m
    z = geometry.ground_z_m + config.CAMERA_MOUNT_HEIGHT_M
    pitch = config.CAMERA_PITCH_DEG
    return {
        "front": ((xmax + 0.25, cy, z), (pitch, 0.0, 0.0)),
        "rear": ((xmin - 0.25, cy, z), (pitch, 180.0, 0.0)),
        "left": ((cx, ymin - 0.1, z), (pitch, -90.0, 0.0)),
        "right": ((cx, ymax + 0.1, z), (pitch, 90.0, 0.0)),
    }


class BevStitcher:
    def __init__(
        self,
        canvas_size_px: int = config.BEV_SIZE_PX,
        px_per_m: float | None = None,
        *,
        width_px: int | None = None,
        height_px: int | None = None,
        geometry: VehicleGeometry | None = None,
        mounts: dict | None = None,
    ):
        self.width = width_px or canvas_size_px
        self.height = height_px or canvas_size_px
        self.geometry = geometry or VehicleGeometry(4.0, 1.0, 40.0, -2.0)
        self.mounts = mounts if mounts is not None else config.CAMERA_MOUNTS
        self._px_per_m = (
            px_per_m if px_per_m is not None else self.height / (2 * config.BEV_RANGE_M)
        )
        if min(self.width, self.height, self._px_per_m) <= 0:
            raise ValueError("BEV dimensions and scale must be positive")
        rows, cols = np.mgrid[: self.height, : self.width].astype(np.float32)
        x, y = (self.height / 2 - rows) / self._px_per_m, (
            cols - self.width / 2
        ) / self._px_per_m
        z = np.full_like(x, self.geometry.ground_z_m)
        self._maps = {}
        k = camera_math.make_intrinsics(
            config.CAMERA_WIDTH_PX, config.CAMERA_HEIGHT_PX, config.CAMERA_FOV_DEG
        )
        xmin, xmax, ymin, ymax = self.geometry.bounds
        visible_side = {
            "front": x >= xmax,
            "rear": x <= xmin,
            "left": y <= ymin,
            "right": y >= ymax,
        }
        for key, (location, rotation) in self.mounts.items():
            delta = np.stack(
                (x - location[0], y - location[1], z - location[2]), axis=-1
            )
            forward, right, up = camera_math.carla_rotation_to_basis(*rotation)
            depth = delta @ forward
            safe_depth = np.maximum(depth, 0.001)
            u = (k.cx + k.fx * (delta @ right) / safe_depth).astype(np.float32)
            v = (k.cy - k.fy * (delta @ up) / safe_depth).astype(np.float32)
            valid = (
                (depth > 0.02)
                & (u >= 0)
                & (u < k.width - 1)
                & (v >= 0)
                & (v < k.height - 1)
                & visible_side[key]
            )
            # Lower seam weights near edges; invalid samples have exactly zero weight.
            edge = np.minimum.reduce(
                [
                    u / (k.width / 2),
                    (k.width - 1 - u) / (k.width / 2),
                    v / (k.height / 2),
                    (k.height - 1 - v) / (k.height / 2),
                ]
            )
            weight = np.where(valid, np.clip(edge, 0.05, 1.0), 0.0).astype(np.float32)
            self._maps[key] = (u, v, weight)
        self.last_coverage = np.zeros((self.height, self.width), bool)
        hatch = ((rows.astype(int) + cols.astype(int)) // 9 % 2) * 8
        self._empty = np.zeros((self.height, self.width, 3), np.uint8)
        self._empty[:] = config.BEV_EMPTY_COLOR
        self._empty += hatch[:, :, None].astype(np.uint8)

    @property
    def canvas_size_px(self) -> int:
        """Compatibility: width only; use canvas_shape for rectangular output."""
        return self.width

    @property
    def canvas_shape(self) -> tuple[int, int, int]:
        return self.height, self.width, 3

    @property
    def px_per_m(self) -> float:
        return self._px_per_m

    def ground_to_canvas(self, x: float, y: float) -> tuple[int, int]:
        return round(self.width / 2 + y * self._px_per_m), round(
            self.height / 2 - x * self._px_per_m
        )

    def canvas_to_ground(self, u: float, v: float) -> tuple[float, float]:
        return (self.height / 2 - v) / self._px_per_m, (
            u - self.width / 2
        ) / self._px_per_m

    def meters_to_px(self, meters: float) -> float:
        return meters * self._px_per_m

    def stitch(self, frames: dict[str, np.ndarray]) -> np.ndarray:
        color = np.zeros(self.canvas_shape, np.float32)
        weights = np.zeros((self.height, self.width), np.float32)
        for key, frame in frames.items():
            if key not in self._maps:
                continue
            if frame.shape != (config.CAMERA_HEIGHT_PX, config.CAMERA_WIDTH_PX, 3):
                raise ValueError(
                    f"camera {key}: unexpected image dimensions {frame.shape}"
                )
            u, v, weight = self._maps[key]
            warped = cv2.remap(
                frame, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
            )
            color += warped.astype(np.float32) * weight[:, :, None]
            weights += weight
        self.last_coverage = weights > 0
        result = self._empty.copy()
        blended = color / np.maximum(weights[:, :, None], 1e-6)
        result[self.last_coverage] = np.clip(
            blended[self.last_coverage], 0, 255
        ).astype(np.uint8)
        return result

    def coverage_report(self) -> dict[str, float]:
        """Measured ground-plane FOV coverage, not radar/3D occlusion coverage."""
        rows, cols = np.mgrid[: self.height, : self.width]
        x = (self.height / 2 - rows) / self._px_per_m
        y = (cols - self.width / 2) / self._px_per_m
        xmin, xmax, ymin, ymax = self.geometry.bounds
        clearance = np.hypot(
            np.maximum.reduce([xmin - x, np.zeros_like(x), x - xmax]),
            np.maximum.reduce([ymin - y, np.zeros_like(y), y - ymax]),
        )
        ring = (clearance >= 0.15) & (clearance <= 4.0)
        denominator = int(ring.sum())
        return {
            "sampled_ground_pixels": denominator,
            "covered_fraction": (
                float(self.last_coverage[ring].mean()) if denominator else 0.0
            ),
        }


def resize_contain(image: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Preserve full camera FOV, with aspect-preserving padding."""
    h, w = image.shape[:2]
    if min(h, w, target_w, target_h) <= 0:
        raise ValueError("image and panel dimensions must be positive")
    scale = min(target_w / w, target_h / h)
    new_w, new_h = max(1, round(w * scale)), max(1, round(h * scale))
    if (new_w, new_h) == (target_w, target_h):
        return cv2.resize(image, (new_w, new_h))
    result = np.empty((target_h, target_w, 3), np.uint8)
    cv2.rectangle(
        result, (0, 0), (target_w - 1, target_h - 1), config.HUD_PANEL_COLOR, -1
    )
    left, top = (target_w - new_w) // 2, (target_h - new_h) // 2
    result[top : top + new_h, left : left + new_w] = cv2.resize(image, (new_w, new_h))
    return result


def resize_cover(image: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Legacy utility; camera panels use resize_contain to preserve blind-zone context."""
    h, w = image.shape[:2]
    scale = max(target_w / w, target_h / h)
    resized = cv2.resize(
        image, (max(target_w, round(w * scale)), max(target_h, round(h * scale)))
    )
    top, left = (resized.shape[0] - target_h) // 2, (resized.shape[1] - target_w) // 2
    return resized[top : top + target_h, left : left + target_w]


def crop_center_square(image: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    side = min(h, w)
    return image[(h - side) // 2 : (h + side) // 2, (w - side) // 2 : (w + side) // 2]


def is_finite_homography(homography: np.ndarray) -> bool:
    return bool(np.isfinite(homography).all() and abs(homography[2, 2]) > 1e-9)
