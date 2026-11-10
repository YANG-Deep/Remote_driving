"""Shared actor-local geometry and timestamped observations (metres, seconds)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


@dataclass(frozen=True)
class VehicleGeometry:
    wheelbase_m: float
    half_track_m: float
    max_steer_deg: float
    rear_axle_x_m: float
    body_center_x_m: float = 0.0
    half_length_m: float = 3.97
    half_width_m: float = 1.45
    ground_z_m: float = 0.0
    body_center_y_m: float = 0.0
    height_m: float = 3.46
    rear_collision_extension_m: float = 0.0
    curvature_samples: tuple[tuple[float, float], ...] = ()
    rear_slip_samples: tuple[tuple[float, float], ...] = ()

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (
            self.body_center_x_m - self.half_length_m - self.rear_collision_extension_m,
            self.body_center_x_m + self.half_length_m,
            self.body_center_y_m - self.half_width_m,
            self.body_center_y_m + self.half_width_m,
        )

    def clearance(self, x: float, y: float) -> float:
        """Shortest horizontal distance to the body rectangle, zero inside."""
        xmin, xmax, ymin, ymax = self.bounds
        return math.hypot(max(xmin - x, 0.0, x - xmax), max(ymin - y, 0.0, y - ymax))

    def sector(self, x: float, y: float) -> str:
        """Assign by nearest body side/corner, +Y is right."""
        xmin, xmax, ymin, ymax = self.bounds
        longitudinal = "front" if x > xmax else "rear" if x < xmin else ""
        lateral = "right" if y > ymax else "left" if y < ymin else ""
        if longitudinal and lateral:
            return f"{longitudinal}_{lateral}"
        return longitudinal or lateral or "front"


SECTOR_BEARINGS = {
    "front": 0.0,
    "front_right": 45.0,
    "right": 90.0,
    "rear_right": 135.0,
    "rear": 180.0,
    "rear_left": 225.0,
    "left": 270.0,
    "front_left": 315.0,
}


class RangeState(str, Enum):
    VALID = "valid"
    NO_RETURN = "no_return"
    STALE = "stale"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True)
class RangeObservation:
    sector: str
    distance_m: float | None
    state: RangeState
    timestamp: float
    frame: int
    point: tuple[float, float, float] | None = None
    source: str = "radar"
    age_s: float = 0.0

    @property
    def bearing_deg(self) -> float:
        return SECTOR_BEARINGS[self.sector]
