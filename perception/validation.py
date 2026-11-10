"""Sampled scene-query reference. Never used as a production range source."""

from __future__ import annotations
import numpy as np
from utils.transforms import transform_matrix, inverse_transform_matrix
from core.models import VehicleGeometry, RangeObservation, RangeState


def edge_rays(
    geometry: VehicleGeometry,
    spacing_m: float = 0.5,
    heights: tuple[float, ...] = (0.1, 0.3, 1.2),
    range_m: float = 8.0,
):
    if spacing_m <= 0 or range_m <= 0:
        raise ValueError("ray spacing and range must be positive")
    xmin, xmax, ymin, ymax = geometry.bounds
    xs = np.linspace(xmin, xmax, max(2, int(np.ceil((xmax - xmin) / spacing_m)) + 1))
    ys = np.linspace(ymin, ymax, max(2, int(np.ceil((ymax - ymin) / spacing_m)) + 1))
    rays = []
    for height in heights:
        z = geometry.ground_z_m + height
        for x in xs:
            for y, sign in [(ymin, -1), (ymax, 1)]:
                rays.append(
                    ((float(x), y + sign * 0.02, z), (float(x), y + sign * range_m, z))
                )
        for y in ys:
            for x, sign in [(xmin, -1), (xmax, 1)]:
                rays.append(
                    ((x + sign * 0.02, float(y), z), (x + sign * range_m, float(y), z))
                )
        # Corner diagonal rays catch obstacles outside both extents.
        for x, sx in [(xmin, -1), (xmax, 1)]:
            for y, sy in [(ymin, -1), (ymax, 1)]:
                rays.append(
                    (
                        (x + sx * 0.02, y + sy * 0.02, z),
                        (x + sx * range_m / 2**0.5, y + sy * range_m / 2**0.5, z),
                    )
                )
    return rays


def compare_ranges(
    observations: list[RangeObservation], reference: dict[str, float]
) -> dict:
    """Signed positive error means radar reports more clearance than reference."""
    report = {}
    for obs in observations:
        ref = reference.get(obs.sector)
        if obs.state in (RangeState.STALE, RangeState.UNAVAILABLE):
            status = "unavailable"
        elif obs.distance_m is None and ref is not None:
            status = "miss"
        elif obs.distance_m is not None and ref is not None:
            status = "compared"
        elif obs.distance_m is not None:
            status = "reference_miss"
        else:
            status = "no_samples"
        report[obs.sector] = {
            "status": status,
            "radar_m": obs.distance_m,
            "reference_m": ref,
            "error_m": obs.distance_m - ref if status == "compared" else None,
        }
    return report


class ReferenceValidator:
    def __init__(self, world, geometry: VehicleGeometry):
        import carla

        self.world, self.geometry = world, geometry
        self.rays = edge_rays(geometry)
        self.excluded = {
            getattr(carla.CityObjectLabel, name)
            for name in ("Roads", "RoadLines", "Terrain", "Sky", "None")
            if hasattr(carla.CityObjectLabel, name)
        }

    def sample(self, vehicle) -> dict[str, float]:
        import carla

        transform = vehicle.get_transform()
        inverse = inverse_transform_matrix(transform)
        result = {}
        for start, end in self.rays:
            origin = transform.transform(carla.Location(*start))
            target = transform.transform(carla.Location(*end))
            # Only nearest physical intersection: never query through a foreground object.
            hits = sorted(
                self.world.cast_ray(origin, target),
                key=lambda hit: origin.distance(hit.location),
            )
            if (
                not hits
                or hits[0].label in self.excluded
                or origin.distance(hits[0].location) <= 0.001
            ):
                continue
            point = hits[0].location
            x, y, _z, _w = inverse @ [point.x, point.y, point.z, 1.0]
            distance = self.geometry.clearance(x, y)
            if distance <= 0.001:
                continue
            sector = self.geometry.sector(x, y)
            result[sector] = min(result.get(sector, float("inf")), distance)
        return result
